// CUDA backend for TraceBackend (NVIDIA GPUs).
//
// MVP scope (scrum-cuda-backend-mvp subtask 4, M2–M5):
//   - BeginSession / EndSession / Reset: device lifecycle + geometry H2D.
//   - TraceLayer: host-side InitRayFirstMs root-gen + H2D + megakernel
//     trace_single_ms_kernel + 4B exit-count readback.
//   - DrainExits: bulk D2H copy of ExitRayRecord buffer.
//   - Recombine: single-MS stub (returns empty DeviceRayBatch).
//   - Per-ray crystal orientation: InitRayFirstMs samples an independent
//     orientation per root ray (the halo-ring distribution comes from this);
//     d_rot_c2w_ is sized n_roots × 9 and indexed by tid in the kernel.
//     Mirrors metal_trace_backend.mm's per-ray rot upload path.
//   - HostRayBatch ingest only; RootRaySource::FromDevice never reaches this
//     backend (single-MS).
//   - WlPoolSize()=M (296.6 DR-3): one session covers the whole spectrum via a
//     device wl_pool; each ray draws a per-ray wl_idx (host first-layer sample;
//     transit carries it through) and the kernel reads d_wl_pool_[wl_idx].n_idx.
//   - Device root-gen (gen_root_kernel) is the remaining 296.6 piece; the host
//     InitRayFirstMs path above is the per-ray-wl fallback.
//
// Build gate: this entire translation unit is added to lumice_obj only when
// LUMICE_CUDA_ENABLED is ON (see CMakeLists.txt). Other backends are
// uninvolved.
//
// Crystal/host-side ray generation re-uses the same `MakeCrystal` /
// `InitRayFirstMs` chain as `CpuTraceBackend::TraceLayer` (cpu_trace_backend.cpp
// 312-325 / 135-140). The kernel is a per-thread single-orientation trace using
// the polygon-slab half-space method (mirrors legacy `PropagateSlab` in
// optics.cpp:111-125): per polygon face, compute the ray/plane intersection
// `t = -(p·n + d) / (dir·n)`, accept rays with `dir·n > kSlabEps` (leaving the
// half-space), pick the minimum `t` as the exit face. Convex-crystal invariant
// guarantees one face is always found, eliminating the Möller-Trumbore
// absolute-ε face-miss bug (task-cuda-traversal-robustness; G2 fix).
//
// D3 三问：
//   ① 推进了"内部遍历无漏面"的缝不变量（凸晶体内必命中出射面）。
//   ② 删除了 `ray_triangle()` 辅助 + 三角级设备缓冲（d_verts_/d_norms_/
//      d_tri_poly_id_）+ O(tri_cnt) 入射面法向量扫描。
//   ③ 对标 legacy `PropagateSlab` 既有架构 + Metal polygon-slab；无 CPU batch
//      适配器、无镜像 CPU 工作单元。GPU 线程=一光线，几何在 session 启动时
//      上传一次（poly 级 AoS：n 三连续、d 单值）。

#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/wl_pool.hpp"  // WlEntry / ComputeWlPool / ResolveWlPoolSize (296.6 per-ray wl, DR-3)

#if defined(LUMICE_CUDA_ENABLED)

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/device_filter_desc.hpp"
#include "core/exit_seam.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/shared/filter_shared.h"
#include "core/shared/optics_shared.h"
#include "core/shared/pcg_shared.h"
#include "core/shared/traversal_shared.h"
#include "core/trace_ops.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

// One-shot probe of cudaGetDeviceCount. Cached after the first call. Errors
// (driver not installed, no devices, runtime mismatch) all collapse to false;
// the simulator interprets that as "no CUDA backend available, fall back to
// legacy CPU".
bool ProbeCudaDevice() {
  int n = 0;
  cudaError_t err = cudaGetDeviceCount(&n);
  if (err != cudaSuccess) {
    (void)cudaGetLastError();
    return false;
  }
  return n > 0;
}

// Throw BackendUnavailableError if err != cudaSuccess. Clears the sticky CUDA
// error state so subsequent calls see a clean slate.
void CheckCuda(cudaError_t err, const char* ctx) {
  if (err != cudaSuccess) {
    std::string msg = std::string{"CudaTraceBackend: CUDA error at "} + ctx + ": " + cudaGetErrorString(err);
    (void)cudaGetLastError();
    throw BackendUnavailableError(std::move(msg));
  }
}

// Exit buffer capacity. Coefficient 2 = reflect+refract conservative upper
// bound per hit; +4 = safety margin matching the Metal EnsureExitBuffers
// constant. Capped at 64 MiB / sizeof(ExitRayRecord) ≈ 760k records.
size_t ComputeExitCap(size_t n_roots, size_t max_hits) {
  size_t cap = n_roots * (max_hits * 2u + 4u);
  size_t hard_cap = (size_t{64u} * 1024u * 1024u) / sizeof(ExitRayRecord);
  return std::min(cap, hard_cap);
}

// Continuation buffer capacity (296.4). Each root may emit up to
// (max_hits * 2 + 4) candidate continuations (matches ComputeExitCap upper
// bound — every outward exit is a continuation candidate). Same 64 MiB hard
// cap scaled by float[4] per slot (dir3 + weight) so the cont pool never
// exceeds the exit-record pool's memory footprint by more than 1.
size_t ComputeContCap(size_t n_roots, size_t max_hits) {
  size_t cap = n_roots * (max_hits * 2u + 4u);
  // Each cont slot = 3 float dir + 1 float weight + 1 uint32 wl_idx = 20B,
  // safely below ExitRayRecord (~36B). Reuse the exit-pool hard cap row.
  size_t hard_cap = (size_t{64u} * 1024u * 1024u) / sizeof(ExitRayRecord);
  return std::min(cap, hard_cap);
}

// File-local mirror of Metal `kFaceCoplanarFloor` (metal_trace_backend.mm:1270)
// — kept numerically in sync with `simulator.cpp::detail::PolygonFaceOfTri`
// and `crystal.cpp::BuildPolygonFaceData`. Used by tri-to-poly mapping below.
constexpr float kCudaFaceCoplanarFloor = 1e-2f;

// PCG-stream isolation nonces. Independent values keep the three CUDA-side PCG
// streams (transit orientation sampler / emit-gate prob draw / future device
// root-gen) statistically separated even when spec.seed degenerates to 0.
// Same role as Metal's `kTransitNonce` (metal_trace_backend.mm:1604) and the
// host-side gate-seed derivation.
constexpr uint32_t kCudaTransitNonce = 0xA5A5A5A5u;
constexpr uint32_t kCudaGateNonce    = 0x5A5A5A5Au;
// DrainExits host-side prob draw nonce (296.5). The final-layer drain runs its
// own RNG stream independent of the device-side gate/transit streams so a
// session that re-seeds (BeginSession) cleanly resets the drain sequence
// (mirrors Metal `impl_->rng` re-seeding semantics — see scrum-258 lesson on
// BeginSession RNG resets being filter parity's hidden third rail).
constexpr uint32_t kCudaDrainNonce   = 0xD5A1B3C7u;

// File-local mirror of Metal `ComputeJacobianEnvelopeForDeviceGen`
// (metal_trace_backend.mm:259). The four branches MUST stay numerically
// identical to `ComputeJacobianEnvelope` in `src/core/math.cpp` (consumed by
// `RandomSampler::SampleSphericalPointsSph` generic-rejection path); if the
// host envelope changes, mirror the change here in the same PR.
float CudaJacobianEnvelopeForDeviceGen(const Distribution& dist) {
  switch (dist.type) {
    case DistributionType::kGaussian:
      return std::cos(std::max(std::abs(dist.mean) - 3.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kZigzag:
      return std::cos(std::max(std::abs(dist.mean) - dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kLaplacian:
      return std::cos(std::max(std::abs(dist.mean) - 5.0f * dist.std, 0.0f) * math::kDegreeToRad);
    case DistributionType::kUniform:
    case DistributionType::kNoRandom:
    case DistributionType::kGaussianLegacy:
      return 1.0f;
  }
  return 1.0f;
}

// Build the transit-form GenRootKernelParams. Mirrors Metal's
// `BuildTransitRootParams` (metal_trace_backend.mm:1593): orientation +
// geometry fields are derived from `axis_dist` exactly as Metal's
// `BuildGenRootParams` would; sun_* / wl_pool_size are populated (defaults)
// but unused by `transit_multi_ms_kernel`. Caller fills `gen_seed` /
// `gen_ray_base` from the Impl-level transit_seed_ / transit_ray_count_.
lm_pcg::GenRootKernelParams BuildTransitGpParams(const AxisDistribution& axis_dist,
                                                  uint32_t tri_count, uint32_t num_rays) {
  lm_pcg::GenRootKernelParams gp{};
  gp.tri_count = tri_count;
  gp.num_rays  = num_rays;
  // Sun / wl_pool fields unused by transit; leave zero (BuildGenRootParams
  // populates them, but the transit kernel reads only orientation + geometry).
  gp.sun_lon = 0.0f;
  gp.sun_lat = 0.0f;
  gp.sun_half_angle = 0.0f;
  gp.wl_pool_size = 1u;  // wl_idx pass-through; 1 keeps "% pool_size" defined

  float lat_mean_rad = axis_dist.latitude_dist.mean * math::kDegreeToRad;
  float lat_std_rad  = axis_dist.latitude_dist.std  * math::kDegreeToRad;
  float rejection_m  = 1.0f;
  uint32_t lat_path  = lm_pcg::kLatPathGenericReject;

  if (axis_dist.IsFullSphereUniform()) {
    lat_path = lm_pcg::kLatPathFullSphere;
  } else if (axis_dist.latitude_dist.type == DistributionType::kNoRandom) {
    lat_path = lm_pcg::kLatPathNoRandom;
  } else if (axis_dist.latitude_dist.type == DistributionType::kGaussianLegacy) {
    lat_path = lm_pcg::kLatPathGaussLegacy;
  } else if (axis_dist.latitude_dist.type == DistributionType::kGaussian) {
    // Same Rayleigh threshold as math.cpp:469 / metal_trace_backend.mm:1513:
    // colatitude_center + 3σ < 0.5°.
    constexpr float kPolarThresholdRad = 0.5f * math::kDegreeToRad;
    float colatitude_center = math::kPi_2 - std::abs(lat_mean_rad);
    bool use_rayleigh = (colatitude_center + 3.0f * lat_std_rad) < kPolarThresholdRad;
    if (use_rayleigh) {
      lat_path = lm_pcg::kLatPathRayleigh;
    } else {
      lat_path = lm_pcg::kLatPathGenericReject;
      rejection_m = CudaJacobianEnvelopeForDeviceGen(axis_dist.latitude_dist);
    }
  } else {
    rejection_m = CudaJacobianEnvelopeForDeviceGen(axis_dist.latitude_dist);
  }

  gp.lat_path        = lat_path;
  gp.lat_dist_type   = static_cast<uint32_t>(axis_dist.latitude_dist.type);
  gp.lat_mean_rad    = lat_mean_rad;
  gp.lat_std_rad     = lat_std_rad;
  gp.lat_rejection_m = rejection_m;

  gp.az_type     = static_cast<uint32_t>(axis_dist.azimuth_dist.type);
  gp.az_mean_rad = axis_dist.azimuth_dist.mean * math::kDegreeToRad;
  gp.az_std_rad  = axis_dist.azimuth_dist.std  * math::kDegreeToRad;
  gp.az_pad      = 0.0f;

  gp.roll_type     = static_cast<uint32_t>(axis_dist.roll_dist.type);
  gp.roll_mean_rad = axis_dist.roll_dist.mean * math::kDegreeToRad;
  gp.roll_std_rad  = axis_dist.roll_dist.std  * math::kDegreeToRad;
  gp.roll_pad      = 0.0f;
  return gp;
}

// Compute tri-to-poly mapping (argmax over polygon-face normals + coplanar
// floor). Mirrors Metal `UploadCrystal` (metal_trace_backend.mm:1271-1286)
// byte-for-byte; the absolute-ε first-match anti-pattern guard in
// doc/numerical-robustness.md约定 2 applies — DO NOT replace with a
// `dot > 1-ε` early-exit.
void FillTriToPoly(const Crystal& crystal, std::vector<uint16_t>& out) {
  size_t tri_cnt  = crystal.TotalTriangles();
  size_t poly_cnt = crystal.PolygonFaceCount();
  out.resize(tri_cnt);
  const float* tri_norms = crystal.GetTriangleNormal();
  const float* poly_norms = crystal.GetPolygonFaceNormal();
  constexpr uint16_t kInvalidIdU16 = 0xffffu;
  for (size_t t = 0; t < tri_cnt; ++t) {
    const float* tn = tri_norms + t * 3u;
    int best_p = -1;
    float best_dot = -1.0f;
    for (size_t p = 0; p < poly_cnt; ++p) {
      const float* pn = poly_norms + p * 3u;
      float dot = tn[0] * pn[0] + tn[1] * pn[1] + tn[2] * pn[2];
      if (dot > best_dot) {
        best_dot = dot;
        best_p = static_cast<int>(p);
      }
    }
    out[t] = (best_p >= 0 && best_dot >= 1.0f - kCudaFaceCoplanarFloor)
                 ? static_cast<uint16_t>(best_p)
                 : kInvalidIdU16;
  }
}

// --- Device-side geometry --------------------------------------------------

__device__ inline float dot3(const float* a, const float* b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Build ExitFaceSeq from a thread-local face-id record. `len` is bounded by
// the caller (rec_len) via the `if (rec_len < ExitFaceSeq::kCap)` guard on
// every append, so `len <= kCap` is invariant here. uint32_t loop counter
// (not uint8_t) keeps the comparison correct should kCap ever grow > 255.
__device__ inline ExitFaceSeq BuildExitPath(const uint8_t* path_rec, uint32_t len) {
  ExitFaceSeq seq{};  // zero-init data_[kCap]; trailing bytes stay 0
  seq.size_ = static_cast<uint8_t>(len);
  for (uint32_t k = 0u; k < len; ++k) {
    seq.data_[k] = path_rec[k];
  }
  return seq;
}

// --- trace_single_ms_kernel -----------------------------------------------
//
// One thread per root ray. Per-bounce refraction splitting (mirrors legacy
// optics.cpp HitSurface + simulator.cpp:914 fan-out, plus Metal exit-seam):
// at each hit, emit the refracted exit contribution (weight = w*(1-r)) and
// continue along the reflected branch with weight w*r. Convex-crystal
// assumption ⇒ non-TIR refraction direction is always outward (cos_exit > 0);
// the kernel falls through that emit safely if the assumption breaks.
// frame invariant 6: exit_dir is rotated crystal->world using the per-ray
// d_rot_c2w[tid*9..tid*9+8] row-major matrix before being written.

// scrum-cuda-backend-complete 296.4 (task-cuda-multi-ms) Step 3: kernel gains
// ms_mode-aware emit gate.
//
// ms_mode == 0 (single MS or final-layer multi-MS): every refracted exit /
//   entry-face external reflect writes an `ExitRayRecord` (existing path).
// ms_mode == 1 (non-final multi-MS layer): each emit candidate draws an
//   independent PCG prob (`pcg_uniform(gate_stream) < ms_prob`):
//     pass → write continuation buffer (d_cont_*); the next-layer trace
//            kernel reads these via transit_multi_ms_kernel.
//     fail → mid-exit: write to the exit buffer tagged with this layer's
//            `ms_layer_idx`. Mirrors Metal lumice_trace.metal:633-660 and
//            simulator.cpp:472 (filter_pass + prob_fail → outgoing) so the
//            cumulative XYZ image preserves energy across MS layers. (296.4
//            ships without filter; 296.5 adds the filter gate that wraps
//            this prob gate.)
//   The per-ray gate_stream is seeded once at kernel entry as
//     {gate_seed, gate_ray_base + tid, 0}. Successive `pcg_uniform` calls
//   inside the ray advance `slot` per draw, so multiple emit sites per ray
//   (entry external-reflect + per-bounce refracted exit) consume disjoint
//   prob streams — matches CPU's per-emit `rng.GetUniform()` cadence.
__global__ void trace_single_ms_kernel(const float* __restrict__ d_dirs,        // 3 × n_roots (crystal-local)
                                       const float* __restrict__ d_pos,         // 3 × n_roots (crystal-local)
                                       const float* __restrict__ d_ws,          // n_roots
                                       const uint32_t* __restrict__ d_from_poly,  // n_roots (entry-face polygon id)
                                       uint32_t n_roots,
                                       const float* __restrict__ d_poly_n,      // 3 × poly_cnt (outward polygon normals)
                                       const float* __restrict__ d_poly_d,      // poly_cnt (plane constant, p·n + d = 0)
                                       uint32_t poly_cnt,
                                       const float* __restrict__ d_rot_c2w,     // 9 × n_roots, row-major per ray
                                       const WlEntry* __restrict__ d_wl_pool,   // wl_pool_size entries (per-ray optics)
                                       const uint32_t* __restrict__ d_root_wl_idx,  // n_roots (per-ray wl index, 296.6 DR-3)
                                       uint32_t max_hits,
                                       ExitRayRecord* __restrict__ d_exit,
                                       uint32_t exit_cap,
                                       uint32_t* __restrict__ d_exit_count,
                                       uint16_t crystal_id,
                                       uint8_t ms_layer_idx,
                                       // ms_mode==1 emit-gate inputs (296.4). Pass 0 / nullptr / 0
                                       // for single-MS or final-layer dispatches.
                                       uint8_t ms_mode,
                                       float ms_prob,
                                       float* __restrict__ d_cont_d,
                                       float* __restrict__ d_cont_w,
                                       uint32_t* __restrict__ d_cont_wl_idx,
                                       uint32_t* __restrict__ d_cont_count,
                                       uint32_t cont_cap,
                                       uint32_t gate_seed,
                                       uint32_t gate_ray_base,
                                       // 296.5 filter gate inputs. ms_mode==0 dispatches pass
                                       // nullptr/0 — DeviceFilterCheck branches are never executed.
                                       const DeviceFilterDesc* __restrict__ d_filter_desc,
                                       const uint32_t* __restrict__ d_getfn_offsets,
                                       const uint8_t* __restrict__ d_getfn_bytes,
                                       const DeviceFilterDesc* __restrict__ d_complex_sub_desc,
                                       uint32_t filter_desc_max_ci,
                                       uint32_t crystal_config_id) {
  const uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= n_roots) {
    return;
  }

  float dir[3] = {d_dirs[tid * 3u + 0u], d_dirs[tid * 3u + 1u], d_dirs[tid * 3u + 2u]};
  float org[3] = {d_pos[tid * 3u + 0u], d_pos[tid * 3u + 1u], d_pos[tid * 3u + 2u]};
  float w = d_ws[tid];

  // Per-ray wavelength (296.6 DR-3): the ray's lifetime wl_idx selects its
  // refractive index from the pool, and tags every exit / continuation record
  // it emits so the host reconstructs the wavelength (simulator.cpp) and the
  // next MS layer keeps the same wl. Replaces the former per-dispatch scalar
  // n_idx (single wl per session).
  const uint32_t wl_idx = d_root_wl_idx[tid];
  const float n_idx = d_wl_pool[wl_idx].n_idx;

  // ms_mode==1 emit-gate PCG stream. Disjoint per (gate_seed, layer-batch,
  // tid); each pcg_uniform draw inside the ray bumps `slot` so multiple emit
  // sites consume independent prob draws (mirrors CPU's per-emit
  // rng.GetUniform()). The stream is constructed even in ms_mode==0 (cost is
  // 3 register writes, ~zero) so the branches below stay symmetric and the
  // compiler can hoist the dead variable away on ms_mode==0.
  lm_pcg::PcgStream gate_stream;
  gate_stream.seed       = gate_seed;
  gate_stream.global_idx = gate_ray_base + tid;
  gate_stream.slot       = 0u;

  // Initial from_poly = entry-face polygon id from InitRay_p_fid output.
  // p sits exactly on this polygon (no epsilon push); without suppressing
  // self-hit the first slab sweep would return t≈0 against the entry face.
  // Mirrors the CPU/Metal Propagate which uses `to_face_` as the from_face
  // guard for the first iteration.
  uint32_t from_poly = d_from_poly[tid];

  // Per-ray crystal->world rotation (row-major). frame invariant 6.
  const float* rot = d_rot_c2w + static_cast<size_t>(tid) * 9u;

  // ── Entry-face air→glass first interaction ───────────────────────────────
  // Mirrors legacy `TraceRayBasicInfo` order (simulator.cpp:383):
  //   HitSurface(to_face_in = entry face) → Propagate(from_face_ = entry face)
  // The pre-295.6 kernel skipped this and let the main loop march to an
  // *interior* far face using the un-refracted sun direction (treated as if
  // the ray were already inside the crystal). Fix: process the entry face
  // explicitly here, then enter the main loop with the refracted inward dir.
  //
  // `from_poly` already holds the entry-face polygon id (InitRay_p_fid →
  // d_from_poly). `org` already sits on this polygon (area-sampled) so no
  // march is needed. We only need its outward normal: read directly from the
  // polygon-level normal buffer (O(1), no warp divergence).
  //
  // Equivalence to the prior O(tri_cnt) tri-scan path: BuildPolygonFaceData
  // guarantees every valid polygon owns ≥1 triangle, so the old
  // `entry_nrm_found` predicate was equivalent to `from_poly < poly_cnt`
  // for all non-sentinel from_poly. The kInvalidId widened sentinel
  // (0xFFFFFFFFu) is filtered by the same `from_poly < poly_cnt` guard.
  // Thread-local rich-exit path recording (task-cuda-rich-exit / 296.3).
  // Mirrors Metal shader (lumice_trace.metal:500-505): the entry face is
  // path[0], each subsequent hit is appended *before* Fresnel at that face.
  // When an exit record is emitted, `path` contains [entry, f_1, ..., f_K]
  // where f_K is the face the ray exits through. Bounded by ExitFaceSeq::kCap
  // (64); every append is guarded so worst-case rec_len = 1 + max_hits stays
  // within the static array. `face_id` widens to uint8_t (poly_cnt ≤ ~40 on
  // all ice crystals; the BeginSession log records poly_cnt for diagnosis).
  uint8_t path_rec[ExitFaceSeq::kCap];
  uint32_t rec_len = 0u;

  if ((from_poly < poly_cnt) && (w > 0.0f)) {
    // Record entry face as path[0]. Matches Metal's to_face=root_tf[tid] which
    // is captured at hit=0 before Fresnel (lumice_trace.metal:505). `from_poly
    // < poly_cnt` is guaranteed by the outer guard and poly_cnt ≤ ~40 << 255
    // for every supported crystal type, so the uint8_t cast is safe.
    path_rec[rec_len++] = static_cast<uint8_t>(from_poly);

    float entry_nrm[3] = {d_poly_n[from_poly * 3u + 0u], d_poly_n[from_poly * 3u + 1u],
                          d_poly_n[from_poly * 3u + 2u]};
    // cos_theta_e < 0: sun ray points into the crystal (guaranteed by
    // InitRay_p_fid's area-weighted projection filter). rr_e = 1/n_idx
    // (air→glass) ⇒ 1 - rr_e^2 > 0 ⇒ dd_e > 0 ⇒ never TIR at entry.
    float cos_theta_e = dot3(dir, entry_nrm);
    float rr_e = 1.0f / n_idx;
    float dd_e = (1.0f - rr_e * rr_e) / (cos_theta_e * cos_theta_e) + rr_e * rr_e;
    float refl_e = lm_optics::GetReflectRatio(dd_e, rr_e);
    float w_refl_e = refl_e * w;
    float w_refr_e = w - w_refl_e;

    // Reflected branch → outward exit contribution. cos(refl_dir, entry_nrm)
    // = -cos_theta_e > 0, so the reflected ray leaves the crystal; emit as
    // exit record (mirrors legacy HitSurface external-reflect path).
    if (w_refl_e > 0.0f) {
      float rd0 = dir[0] - 2.0f * cos_theta_e * entry_nrm[0];
      float rd1 = dir[1] - 2.0f * cos_theta_e * entry_nrm[1];
      float rd2 = dir[2] - 2.0f * cos_theta_e * entry_nrm[2];
      float exit_world[3];
      for (int i = 0; i < 3; ++i) {
        exit_world[i] = rot[i * 3 + 0] * rd0 + rot[i * 3 + 1] * rd1 + rot[i * 3 + 2] * rd2;
      }
      if (ms_mode == 1u) {
        // ms_mode==1 emit gate: filter THEN prob (mirrors Metal
        // lumice_trace.metal:633-660 + simulator.cpp:472 outgoing semantics).
        // filter-fail → implicit drop (no buffer write), enforcing AC1
        // "filter-fail光线不跨MS层传播". exit_world is the WORLD-space ray
        // direction Metal/CPU FilterMatchDirection expects.
        const uint32_t gate_slot = static_cast<uint32_t>(ms_layer_idx) * filter_desc_max_ci;
        const bool filter_pass = lm_filter::DeviceFilterCheck(
            d_filter_desc[gate_slot], d_complex_sub_desc, path_rec, rec_len, d_getfn_bytes, d_getfn_offsets, gate_slot,
            exit_world, crystal_config_id);
        if (filter_pass) {
          // Prob gate (prob-pass → continuation; prob-fail → mid-exit). The
          // gate_stream advances on each pcg_uniform draw so the per-bounce
          // refracted exit gate below consumes a disjoint draw slot.
          bool do_continue = (lm_pcg::pcg_uniform(gate_stream) < ms_prob);
          if (do_continue) {
            uint32_t cslot = atomicAdd(d_cont_count, 1u);
            if (cslot < cont_cap) {
              d_cont_d[cslot * 3u + 0u] = exit_world[0];
              d_cont_d[cslot * 3u + 1u] = exit_world[1];
              d_cont_d[cslot * 3u + 2u] = exit_world[2];
              d_cont_w[cslot]           = w_refl_e;
              d_cont_wl_idx[cslot]      = wl_idx;  // 296.6 DR-3: carry ray's wl to its continuation
            }
          } else {
            uint32_t slot = atomicAdd(d_exit_count, 1u);
            if (slot < exit_cap) {
              ExitRayRecord& rec = d_exit[slot];
              rec.dir[0] = exit_world[0];
              rec.dir[1] = exit_world[1];
              rec.dir[2] = exit_world[2];
              rec.weight = w_refl_e;
              rec.path = BuildExitPath(path_rec, rec_len);
              rec.crystal_id = crystal_id;
              rec.ms_layer_idx = ms_layer_idx;
              rec.wl_idx = static_cast<uint8_t>(wl_idx);
            }
          }
        }
        // filter_pass == false: implicit drop (Design A termination).
      } else {
        uint32_t slot = atomicAdd(d_exit_count, 1u);
        if (slot < exit_cap) {
          ExitRayRecord& rec = d_exit[slot];
          rec.dir[0] = exit_world[0];
          rec.dir[1] = exit_world[1];
          rec.dir[2] = exit_world[2];
          rec.weight = w_refl_e;
          // Entry external reflect: path = [entry_face]. Matches Metal's
          // hit=0 mid-exit emission with rec_len=1 (lumice_trace.metal:658-662).
          rec.path = BuildExitPath(path_rec, rec_len);
          rec.crystal_id = crystal_id;
          rec.ms_layer_idx = ms_layer_idx;
          rec.wl_idx = static_cast<uint8_t>(wl_idx);
        }
      }
    }

    // Refracted branch → bend into crystal. After this dir points inward
    // (fd·n = cos_theta_e * sd_e < 0 since cos_theta_e < 0, sd_e > 0).
    // `from_poly` is left unchanged so the main loop's first march excludes
    // the entry face's triangles (mirrors legacy Propagate's
    // `from_face_ = to_face_` first-iter guard).
    float sd_e = sqrtf(dd_e);
    dir[0] = rr_e * dir[0] - (rr_e - sd_e) * cos_theta_e * entry_nrm[0];
    dir[1] = rr_e * dir[1] - (rr_e - sd_e) * cos_theta_e * entry_nrm[1];
    dir[2] = rr_e * dir[2] - (rr_e - sd_e) * cos_theta_e * entry_nrm[2];
    w = w_refr_e;
  }
  // ── End entry-face interaction ───────────────────────────────────────────

  for (uint32_t hit = 0u; hit < max_hits; ++hit) {
    if (w <= 0.0f) {
      break;
    }
    // Polygon-slab traversal: mirrors legacy `PropagateSlab`
    // (optics.cpp:111-125). For each polygon face whose outward normal makes
    // `dir·n > kSlabEps` (the ray is leaving that half-space), compute the
    // ray/plane intersection `t = -(org·n + d) / denom`. Pick the minimum t
    // as the exit face. Convex-crystal invariant guarantees one face is
    // always found — no absolute-ε face-miss bug (the Möller-Trumbore route
    // dropped TIR-reflected near-parallel rays via `det ∈ [-1e-8, 1e-8]`
    // and `t > 1e-6f` thresholds; see doc/numerical-robustness.md约定 2).
    // Single-source per-face intersect lives in lm_traversal::SlabFaceT;
    // see src/core/shared/traversal_shared.h for the denom-gate /
    // from-face exclusion contract shared with Metal + CPU.
    float t_best = 1e30f;
    uint32_t hit_poly = 0xFFFFFFFFu;
    for (uint32_t fi = 0u; fi < poly_cnt; ++fi) {
      if (fi == from_poly) {
        continue;
      }
      float nx = d_poly_n[fi * 3u + 0u];
      float ny = d_poly_n[fi * 3u + 1u];
      float nz = d_poly_n[fi * 3u + 2u];
      float fd = d_poly_d[fi];
      float t = lm_traversal::SlabFaceT(dir[0], dir[1], dir[2], org[0], org[1], org[2], nx, ny, nz, fd);
      if (t < t_best) {
        t_best = t;
        hit_poly = fi;
      }
    }
    // Accept t slightly negative (-kSlabEps) for TIR-edge cases where the
    // hit-point sits just inside the source face within float32 rounding.
    // Mirrors CPU's `eps_thr = -math::kFloatEps` relaxed threshold
    // (optics.cpp:138). On a convex crystal this should never trigger the
    // hard exit — if it does, geometry data is anomalous.
    if (hit_poly == 0xFFFFFFFFu || t_best <= -lm_traversal::kSlabEps) {
      break;
    }

    // Advance origin to hit point.
    org[0] += t_best * dir[0];
    org[1] += t_best * dir[1];
    org[2] += t_best * dir[2];

    // Append this face to the rich-exit path before Fresnel (matches Metal
    // shader order at lumice_trace.metal:505). Guard against overflow: worst
    // case rec_len = 1 (entry) + max_hits; when max_hits == kCap a tail face
    // would overflow, so silently drop the tail (degrades to the legacy
    // truncation Metal also performs via `if (rec_len < kRecCap)`). Overflow
    // is only reachable at max_hits == kCap == 64 — typical configs use
    // max_hits ≤ 8 so this guard is defensive, not active in practice.
    if (rec_len < static_cast<uint32_t>(ExitFaceSeq::kCap)) {
      path_rec[rec_len++] = static_cast<uint8_t>(hit_poly);
    }

    float nrm[3] = {d_poly_n[hit_poly * 3u + 0u], d_poly_n[hit_poly * 3u + 1u], d_poly_n[hit_poly * 3u + 2u]};

    // Fresnel: same formula as cpu/metal HitSurface. cos_theta is signed; rr
    // flips between n (going outward) and 1/n (going inward). dd ≤ 0 ⇒ TIR.
    float cos_theta = dot3(dir, nrm);
    float rr = (cos_theta > 0.0f) ? n_idx : (1.0f / n_idx);
    float dd = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
    bool is_tir = (dd <= 0.0f);
    float refl_ratio = lm_optics::GetReflectRatio(fmaxf(dd, 0.0f), rr);
    if (is_tir) {
      refl_ratio = 1.0f;
    }

    float w_refl = refl_ratio * w;
    float w_refr = is_tir ? 0.0f : (w - w_refl);

    // Emit the refracted exit contribution. Convex-crystal assumption: for
    // non-TIR, the refracted direction is outward (cos_exit > 0). The
    // cos_exit > 0 guard below is defensive — non-convex crystals would
    // simply emit zero refracted exits at grazing-angle edge cases.
    // TODO: non-convex crystal continuation (refracted ray re-enters) is
    // out of MVP scope.
    if (!is_tir && w_refr > 0.0f) {
      float sd = sqrtf(fmaxf(dd, 0.0f));
      float fdx = rr * dir[0] - (rr - sd) * cos_theta * nrm[0];
      float fdy = rr * dir[1] - (rr - sd) * cos_theta * nrm[1];
      float fdz = rr * dir[2] - (rr - sd) * cos_theta * nrm[2];
      float cos_exit = fdx * nrm[0] + fdy * nrm[1] + fdz * nrm[2];
      if (cos_exit > 0.0f) {
        float exit_world[3];
        for (int i = 0; i < 3; ++i) {
          exit_world[i] = rot[i * 3 + 0] * fdx + rot[i * 3 + 1] * fdy + rot[i * 3 + 2] * fdz;
        }
        if (ms_mode == 1u) {
          // Per-bounce refracted exit emit gate (296.5): same filter-then-prob
          // semantics as the entry-face emit above. path_rec already includes
          // hit_poly (appended at the top of this hit, before Fresnel).
          const uint32_t gate_slot = static_cast<uint32_t>(ms_layer_idx) * filter_desc_max_ci;
          const bool filter_pass = lm_filter::DeviceFilterCheck(
              d_filter_desc[gate_slot], d_complex_sub_desc, path_rec, rec_len, d_getfn_bytes, d_getfn_offsets,
              gate_slot, exit_world, crystal_config_id);
          if (filter_pass) {
            bool do_continue = (lm_pcg::pcg_uniform(gate_stream) < ms_prob);
            if (do_continue) {
              uint32_t cslot = atomicAdd(d_cont_count, 1u);
              if (cslot < cont_cap) {
                d_cont_d[cslot * 3u + 0u] = exit_world[0];
                d_cont_d[cslot * 3u + 1u] = exit_world[1];
                d_cont_d[cslot * 3u + 2u] = exit_world[2];
                d_cont_w[cslot]           = w_refr;
                d_cont_wl_idx[cslot]      = wl_idx;  // 296.6 DR-3: carry ray's wl to its continuation
              }
            } else {
              uint32_t slot = atomicAdd(d_exit_count, 1u);
              if (slot < exit_cap) {
                ExitRayRecord& rec = d_exit[slot];
                rec.dir[0] = exit_world[0];
                rec.dir[1] = exit_world[1];
                rec.dir[2] = exit_world[2];
                rec.weight = w_refr;
                rec.path = BuildExitPath(path_rec, rec_len);
                rec.crystal_id = crystal_id;
                rec.ms_layer_idx = ms_layer_idx;
                rec.wl_idx = static_cast<uint8_t>(wl_idx);
              }
            }
          }
          // filter_pass == false: implicit drop (Design A termination).
        } else {
          uint32_t slot = atomicAdd(d_exit_count, 1u);
          if (slot < exit_cap) {
            ExitRayRecord& rec = d_exit[slot];
            rec.dir[0] = exit_world[0];
            rec.dir[1] = exit_world[1];
            rec.dir[2] = exit_world[2];
            rec.weight = w_refr;
            // Rich exit path = [entry_face, f_1, ..., f_K] where f_K = hit_poly
            // (the face the ray refracted through). Mirrors Metal mid-exit
            // emission at lumice_trace.metal:658-662.
            rec.path = BuildExitPath(path_rec, rec_len);
            rec.crystal_id = crystal_id;
            rec.ms_layer_idx = ms_layer_idx;
            rec.wl_idx = static_cast<uint8_t>(wl_idx);
          }
        }
      }
    }

    // Reflect along the same bounce; continue with reduced weight.
    dir[0] = dir[0] - 2.0f * cos_theta * nrm[0];
    dir[1] = dir[1] - 2.0f * cos_theta * nrm[1];
    dir[2] = dir[2] - 2.0f * cos_theta * nrm[2];
    w = w_refl;
    from_poly = hit_poly;
  }
}

// --- transit_multi_ms_kernel ----------------------------------------------
//
// Device-resident MS-layer transit (296.4). Consumes a continuation batch
// produced by `trace_single_ms_kernel` in ms_mode==1 and emits root rays for
// the next MS layer. The whole continuation hop stays on device — only the
// 4-byte continuation count crosses the seam during Recombine. Mirrors Metal
// `transit_root_kernel` (lumice_trace.metal:1213-1294) form-for-form so the
// two backends share the orientation + entry-point sampling math via
// `lm_pcg::*` and cannot drift on either the PCG hash or the sample logic.
//
// PCG stream identity: caller pre-derives `gp.gen_seed = transit_seed_` and
// `gp.gen_ray_base = transit_ray_count_` so (gen_seed, gen_ray_base + tid,
// slot) is unique across (layer, batch, tid, draw). transit_ray_count_ is
// advanced by the host AFTER the dispatch returns (Recombine tail). Re-using
// the same global_idx silently kills cross-seed independence — the AC2b
// guard catches it (scrum-267.3 lesson).
__global__ void transit_multi_ms_kernel(
    const float* __restrict__ d_cont_d_in,        // 3 × n_rays (world-space)
    const float* __restrict__ d_cont_w_in,        // n_rays (carried continuation weight)
    const uint32_t* d_cont_wl_idx_in,               // n_rays (pass-through wl_idx); no __restrict__: Recombine may pass same buffer as d_root_wl_idx_out
    float* __restrict__ d_root_d_out,             // 3 × n_rays (crystal-local)
    float* __restrict__ d_root_p_out,             // 3 × n_rays (crystal-local entry point)
    float* __restrict__ d_root_w_out,             // n_rays (post-transit weight)
    uint32_t* __restrict__ d_root_from_poly_out,  // n_rays (entry-face polygon id; kInvalidId widened)
    float* __restrict__ d_root_rot_out,           // 9 × n_rays (row-major crystal→world)
    uint32_t* d_root_wl_idx_out,                   // n_rays (pass-through); no __restrict__: may alias d_cont_wl_idx_in
    const float* __restrict__ d_tri_vtx,          // 9 × tri_cnt (3 verts × 3 coords)
    const float* __restrict__ d_tri_norm,         // 3 × tri_cnt (outward triangle normal)
    const float* __restrict__ d_tri_area,         // tri_cnt
    const uint16_t* __restrict__ d_tri_to_poly,   // tri_cnt (kInvalidId on coplanar miss)
    lm_pcg::GenRootKernelParams gp,
    uint32_t n_rays) {
  const uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= n_rays || gp.tri_count == 0u) {
    return;
  }

  // 1. Orientation sample (shares sample_lat_lon_roll with trace via the
  //    shared pcg_shared.h). gp.gen_ray_base + tid yields a disjoint
  //    global_idx per (layer, batch, tid).
  lm_pcg::PcgStream stream;
  stream.seed       = gp.gen_seed;
  stream.global_idx = gp.gen_ray_base + tid;
  stream.slot       = 0u;
  float lon = 0.0f;
  float lat = 0.0f;
  float roll = 0.0f;
  lm_pcg::sample_lat_lon_roll(stream, gp, lon, lat, roll);

  float mat9[9];
  lm_pcg::build_crystal_rotation_9(lon, lat, roll, mat9);

  // 2. World-space continuation direction → crystal-local frame.
  float d_world[3] = {d_cont_d_in[tid * 3u + 0u],
                      d_cont_d_in[tid * 3u + 1u],
                      d_cont_d_in[tid * 3u + 2u]};
  float d_crystal[3];
  lm_pcg::apply_inverse_mat9(mat9, d_world, d_crystal);

  // 3. Triangle area × facing weighted pick. kMaxTriPerKernel-sized stack
  //    array; tri_cnt is BeginSession-validated to fit (otherwise it would
  //    throw BackendUnavailableError before any kernel dispatch).
  float proj_prob[lm_pcg::kMaxTriPerKernel];
  uint32_t n_tri = gp.tri_count;
  if (n_tri > lm_pcg::kMaxTriPerKernel) {
    n_tri = lm_pcg::kMaxTriPerKernel;  // defensive; host should have already throw'd
  }
  for (uint32_t t = 0u; t < n_tri; ++t) {
    float dot = d_crystal[0] * d_tri_norm[t * 3u + 0u]
              + d_crystal[1] * d_tri_norm[t * 3u + 1u]
              + d_crystal[2] * d_tri_norm[t * 3u + 2u];
    // -dot * area: negate so triangles whose outward normal opposes the ray
    // (i.e. the ray enters that face) get positive weight; clamp the rest to
    // zero (mirrors Metal lumice_trace.metal:1265).
    proj_prob[t] = fmaxf(-dot * d_tri_area[t], 0.0f);
  }
  float u_cat = lm_pcg::pcg_uniform(stream);
  uint32_t tri_id = lm_pcg::categorical_sample(proj_prob, n_tri, u_cat);

  // 4. Uniform sample inside the chosen triangle → entry point p.
  float p[3];
  lm_pcg::sample_triangle(stream, d_tri_vtx + tri_id * 9u, p);

  // 5. tri_to_poly. kInvalidId (uint16_t 0xffff) signals "triangle has no
  //    polygon backing under the coplanar-floor predicate" — mirror Metal's
  //    InitRay_p_fid zero-weight fallback so downstream trace dispatches see
  //    a benign drop (w=0 short-circuits the entry-face emit and main loop).
  uint16_t to_face_u16 = d_tri_to_poly[tri_id];
  float w = d_cont_w_in[tid];
  constexpr uint16_t kInvalidIdU16 = 0xffffu;
  uint32_t to_face_u32;
  if (to_face_u16 == kInvalidIdU16) {
    w = 0.0f;
    to_face_u32 = 0xFFFFFFFFu;  // widened sentinel; trace kernel's `from_poly < poly_cnt`
                                // guard filters this naturally.
  } else {
    to_face_u32 = static_cast<uint32_t>(to_face_u16);
  }

  // 6. Emit root buffers. d_root_d_out / d_root_p_out are crystal-local (the
  //    trace kernel consumes them in that frame), d_root_rot_out is the
  //    crystal→world matrix for invariant-6 conversion on emit, and
  //    d_root_wl_idx_out is the lifetime wavelength tag (MVP: 0).
  d_root_d_out[tid * 3u + 0u] = d_crystal[0];
  d_root_d_out[tid * 3u + 1u] = d_crystal[1];
  d_root_d_out[tid * 3u + 2u] = d_crystal[2];
  d_root_p_out[tid * 3u + 0u] = p[0];
  d_root_p_out[tid * 3u + 1u] = p[1];
  d_root_p_out[tid * 3u + 2u] = p[2];
  d_root_w_out[tid] = w;
  d_root_from_poly_out[tid] = to_face_u32;
  for (uint32_t k = 0u; k < 9u; ++k) {
    d_root_rot_out[tid * 9u + k] = mat9[k];
  }
  d_root_wl_idx_out[tid] = d_cont_wl_idx_in[tid];
}

}  // namespace

bool CudaDeviceAvailable() {
  static std::once_flag flag;
  static bool cached = false;
  std::call_once(flag, []() { cached = ProbeCudaDevice(); });
  return cached;
}

// --- Impl ----------------------------------------------------------------

struct CudaTraceBackend::Impl {
  Logger* logger = nullptr;
  bool in_session_ = false;
  bool buffers_allocated_ = false;

  // Crystal geometry — uploaded once per session at polygon-face granularity.
  // No triangle-level data: the kernel does polygon-slab traversal, not
  // Möller-Trumbore.
  float* d_poly_n_ = nullptr;  // 3 × poly_cnt (outward polygon normals, AoS)
  float* d_poly_d_ = nullptr;  // poly_cnt (plane constant: p·n + d = 0)
  uint32_t poly_cnt_ = 0;

  // --- Multi-MS continuation (296.4) ---------------------------------------
  // Per-session triangle pool (transit_multi_ms_kernel needs vtx/norm/area +
  // tri-to-poly for area×facing-weighted entry-point sampling). Single-CI MVP
  // — multi-CI 在 296.6 落地。Layout mirrors Metal `tri_*_buf_`.
  float*    d_tri_vtx_     = nullptr;  // 9 × tri_cnt (3 verts × 3 coords)
  float*    d_tri_norm_    = nullptr;  // 3 × tri_cnt (outward triangle normal)
  float*    d_tri_area_    = nullptr;  // tri_cnt
  uint16_t* d_tri_to_poly_ = nullptr;  // tri_cnt (kInvalidId on coplanar miss)
  uint32_t  tri_cnt_       = 0;

  // Per-ray crystal->world rotation buffer (9 floats/ray, row-major
  // Rotation::mat_). Allocated in EnsureSessionBuffers; filled per TraceLayer
  // from InitRayFirstMs all_data[i].crystal_rot_ output (ms_mode==0 path) or
  // by transit_multi_ms_kernel (ms_mode==1 path). Sized to cont_cap_ slots
  // so the transit-kernel write path stays in-bounds. Mirrors
  // metal_trace_backend.mm's per-ray rot upload (frame invariant 6).
  float* d_rot_c2w_ = nullptr;

  // Per-batch root-ray buffers. Sized to cont_cap_ slots so the transit
  // kernel can fill up to cont_cap_ root entries (continuation rays from one
  // layer may exceed the next layer's n_roots after fan-out).
  float* d_dirs_ = nullptr;
  float* d_pos_ = nullptr;
  float* d_ws_ = nullptr;
  uint32_t* d_from_poly_ = nullptr;  // per-ray entry-face polygon id
  // Per-ray wavelength index (296.6 DR-3). Sized like the other root buffers
  // (cont_cap_); the trace kernel reads d_wl_pool_[d_root_wl_idx_[tid]].n_idx for
  // per-ray refraction and tags exit/continuation records with it. Filled by the
  // host first-layer path (per-ray wl sample) or written by transit_multi_ms_kernel
  // (continuation pass-through) on subsequent layers.
  uint32_t* d_root_wl_idx_ = nullptr;

  // --- Continuation ring (296.4) -------------------------------------------
  // Single ping-pong slot (no multi-generation tracking required by seam
  // lifetime contract: DeviceRayBatch::backend_ptr valid only until the next
  // Recombine; see trace_backend.hpp). Written by trace_single_ms_kernel when
  // ms_mode==1, consumed by transit_multi_ms_kernel.
  float*    d_cont_d_       = nullptr;   // 3 × cont_cap (world-space dir)
  float*    d_cont_w_       = nullptr;   // cont_cap (continuation weight)
  uint32_t* d_cont_wl_idx_  = nullptr;   // cont_cap (wl pass-through; MVP always 0)
  uint32_t* d_cont_count_   = nullptr;   // 1 uint32 (atomic, device)
  size_t    cont_cap_       = 0;
  uint32_t  h_cont_count_   = 0;         // host snapshot after 4B D2H readback
  uint32_t* pinned_cont_count_ = nullptr;  // 1 uint32 pinned host (D2H staging)

  // Session-level exit pool.
  ExitRayRecord* d_exit_ = nullptr;
  uint32_t* d_exit_count_ = nullptr;
  size_t exit_cap_ = 0;
  size_t alloc_exit_cap_ = 0;  // physically-allocated d_exit_ capacity (grow-only across MS layers)
  uint32_t h_exit_count_ = 0;

  // Pinned staging.
  float* pinned_dirs_ = nullptr;
  float* pinned_pos_ = nullptr;
  float* pinned_ws_ = nullptr;
  uint32_t* pinned_from_poly_ = nullptr;
  uint32_t* pinned_root_wl_idx_ = nullptr;  // n_roots (296.6 per-ray wl, H2D staging)
  float* pinned_rot_c2w_ = nullptr;  // n_roots × 9 (mirrors d_rot_c2w_)
  ExitRayRecord* pinned_exit_ = nullptr;

  RandomNumberGenerator rng_{0u};  // re-seeded in BeginSession from spec.seed

  size_t n_roots_ = 0;

  const SceneConfig* scene_ = nullptr;
  const RenderConfig* render_ = nullptr;
  WlParam wl_{};

  // --- Per-ray wavelength pool (296.6 DR-3) --------------------------------
  // Single session covers the whole spectrum: each ray samples a wl_idx in
  // [0, wl_pool_size_) and is tagged with it for life (device root-gen / host
  // fallback draw it; transit passes it through). The device trace kernel reads
  // d_wl_pool_[wl_idx].n_idx (per-ray refraction) and emits the wl_idx on exit
  // records so the host reconstructs the per-ray wavelength (simulator.cpp).
  // Mirrors Metal's wl_pool (metal_trace_backend.mm). wl_pool_size_==0 means the
  // pool is unbuilt (WlPoolSize() returns it → caller stays on per-batch wl).
  uint32_t wl_pool_size_ = 0u;
  bool illuminant_mode_ = false;
  IlluminantType illuminant_{};
  std::vector<WlEntry> wl_pool_host_;
  WlEntry* d_wl_pool_ = nullptr;  // wl_pool_size_ × WlEntry, uploaded once per BeginSession

  cudaEvent_t ev_start_h2d_{};
  cudaEvent_t ev_end_h2d_{};
  cudaEvent_t ev_end_kernel_{};
  cudaEvent_t ev_end_d2h_{};
  bool events_created_ = false;

  // --- Transit-kernel PCG state (296.4) ------------------------------------
  // Independent PCG stream for the device-resident continuation engine. The
  // counter is monotone across the whole session (NOT reset across batches /
  // EndSession idle gaps) so the (transit_seed_, transit_ray_count_ + tid)
  // tuple stays disjoint per (layer, batch, tid) — re-using the same
  // global_idx silently collapses cross-seed independence (scrum-267.3 lesson;
  // see [[project_scrum267_continuation_engine_done]]). transit_seeded_ guards
  // the first-time reset so Reset() can free buffers without zeroing the
  // counter mid-session.
  uint32_t transit_seed_      = 0u;
  size_t   transit_ray_count_ = 0;
  bool     transit_seeded_    = false;

  // --- Emit-gate PCG state (296.4) -----------------------------------------
  // Independent PCG stream for the ms_mode==1 emit-gate prob draw inside
  // `trace_single_ms_kernel`. Same monotone-counter discipline as transit_*.
  uint32_t gate_seed_      = 0u;
  size_t   gate_ray_count_ = 0;
  bool     gate_seeded_    = false;

  // --- MS layer tracking (296.4) -------------------------------------------
  // Populated by BeginSession from spec.scene->ms_.size() and advanced by
  // TraceLayer / Recombine. ms_mode is derived as
  //   ms_mode = (ms_layer_idx_ + 1 < n_ms_layers_) ? 1 : 0
  // so the final layer writes exit records (ms_mode==0) and earlier layers
  // write continuation records (ms_mode==1).
  uint8_t ms_layer_idx_ = 0u;
  uint8_t n_ms_layers_  = 0u;

  // --- Filter buffers (296.5) ----------------------------------------------
  // Mirrors Metal's filter_desc_buf_ / getfn_offsets_buf_ / getfn_bytes_buf_ /
  // complex_sub_desc_buf_ quartet (metal_trace_backend.mm:724-734). The CUDA
  // kernel emit gate (ms_mode==1) consumes these via DeviceFilterCheck; the
  // 1-byte dummy fallback covers no-filter sessions so the kernel pointers are
  // always non-null even on the ms_mode==0 path (which doesn't read them).
  DeviceFilterDesc* d_filter_desc_       = nullptr;  // n_slot DeviceFilterDescs (or 1B dummy)
  uint32_t*         d_getfn_offsets_     = nullptr;  // (n_slot + 1) uint32 prefix-sum (or 1B dummy)
  uint8_t*          d_getfn_bytes_       = nullptr;  // flat GetFn byte stream (or 1B dummy)
  DeviceFilterDesc* d_complex_sub_desc_  = nullptr;  // flat Complex sub-descs (or 1B dummy)
  uint32_t          filter_desc_max_ci_  = 0u;       // per-layer ci stride (MVP=1)
  uint32_t          filter_n_slot_       = 0u;       // total descriptor slots
  uint32_t          crystal_config_id_   = 0xFFFFu;  // for DeviceFilterMatchCrystal

  // --- Final-layer host filter (296.5) -------------------------------------
  // DrainExits applies FilterSpec::Check + prob to records tagged with the
  // final ms_layer_idx (mirrors Metal ReadbackExitRays:2447-2578). Captured in
  // BeginSession from the last ms_setting; single-CI MVP, multi-CI is 296.6.
  FilterConfig     final_layer_filter_config_{};
  Crystal          final_layer_crystal_{};
  AxisDistribution final_layer_axis_dist_{};
  uint8_t          final_ms_layer_idx_ = 0u;
  float            final_ms_prob_      = 0.0f;
  // `drain_rng_` is the host-side prob-draw RNG used by DrainExits. Seeded
  // ONCE per Run() (first BeginSession, guarded by drain_seeded_) with
  // `spec.seed XOR kCudaDrainNonce` and advanced monotonically across all
  // per-wavelength-batch drains — mirroring legacy's single Simulator rng_ and
  // the transit_/gate_ "seed once, advance" discipline. Re-seeding it on every
  // BeginSession (as it did before 296.5 Bug B) replays the same mt19937 prefix
  // each batch, biasing the final-layer prob keep-fraction (measured 0.53 vs
  // 0.50 → +7.4% energy); corr stayed 0.996 because the bias is spatially
  // uniform (the "corr masks energy bug" trap).
  std::mt19937 drain_rng_{0u};
  bool         drain_seeded_ = false;

  void Reset();
  void EnsureSessionBuffers(size_t n);
  void EnsureFilterBuffers(const SessionSpec& spec);
  // Grow ONLY the exit buffer for a device-roots continuation layer, leaving the
  // root buffers (d_dirs_/d_pos_/d_ws_/d_from_poly_/d_rot_c2w_) untouched so the
  // transit_multi_ms_kernel's device-resident continuation rays survive into the
  // next TraceLayer. EnsureSessionBuffers must NOT run on the device-roots path:
  // its n_roots_!=n realloc would free those buffers mid-flight (296.4 root-cause).
  void EnsureExitCapacity(size_t n);
};

void CudaTraceBackend::Impl::Reset() {
  // cudaFree / cudaFreeHost on nullptr are no-ops per CUDA spec, so we don't
  // need explicit allocated-flag guards. Errors are intentionally ignored —
  // teardown must not throw.
  cudaFree(d_poly_n_);
  d_poly_n_ = nullptr;
  cudaFree(d_poly_d_);
  d_poly_d_ = nullptr;
  cudaFree(d_tri_vtx_);
  d_tri_vtx_ = nullptr;
  cudaFree(d_tri_norm_);
  d_tri_norm_ = nullptr;
  cudaFree(d_tri_area_);
  d_tri_area_ = nullptr;
  cudaFree(d_tri_to_poly_);
  d_tri_to_poly_ = nullptr;
  cudaFree(d_rot_c2w_);
  d_rot_c2w_ = nullptr;
  cudaFree(d_dirs_);
  d_dirs_ = nullptr;
  cudaFree(d_pos_);
  d_pos_ = nullptr;
  cudaFree(d_ws_);
  d_ws_ = nullptr;
  cudaFree(d_from_poly_);
  d_from_poly_ = nullptr;
  cudaFree(d_root_wl_idx_);
  d_root_wl_idx_ = nullptr;
  cudaFree(d_wl_pool_);
  d_wl_pool_ = nullptr;
  cudaFree(d_cont_d_);
  d_cont_d_ = nullptr;
  cudaFree(d_cont_w_);
  d_cont_w_ = nullptr;
  cudaFree(d_cont_wl_idx_);
  d_cont_wl_idx_ = nullptr;
  cudaFree(d_cont_count_);
  d_cont_count_ = nullptr;
  cudaFree(d_exit_);
  d_exit_ = nullptr;
  cudaFree(d_exit_count_);
  d_exit_count_ = nullptr;
  cudaFree(d_filter_desc_);
  d_filter_desc_ = nullptr;
  cudaFree(d_getfn_offsets_);
  d_getfn_offsets_ = nullptr;
  cudaFree(d_getfn_bytes_);
  d_getfn_bytes_ = nullptr;
  cudaFree(d_complex_sub_desc_);
  d_complex_sub_desc_ = nullptr;

  cudaFreeHost(pinned_dirs_);
  pinned_dirs_ = nullptr;
  cudaFreeHost(pinned_pos_);
  pinned_pos_ = nullptr;
  cudaFreeHost(pinned_ws_);
  pinned_ws_ = nullptr;
  cudaFreeHost(pinned_from_poly_);
  pinned_from_poly_ = nullptr;
  cudaFreeHost(pinned_root_wl_idx_);
  pinned_root_wl_idx_ = nullptr;
  cudaFreeHost(pinned_rot_c2w_);
  pinned_rot_c2w_ = nullptr;
  cudaFreeHost(pinned_cont_count_);
  pinned_cont_count_ = nullptr;
  cudaFreeHost(pinned_exit_);
  pinned_exit_ = nullptr;

  if (events_created_) {
    cudaEventDestroy(ev_start_h2d_);
    cudaEventDestroy(ev_end_h2d_);
    cudaEventDestroy(ev_end_kernel_);
    cudaEventDestroy(ev_end_d2h_);
    events_created_ = false;
  }

  poly_cnt_ = 0;
  tri_cnt_ = 0;
  exit_cap_ = 0;
  alloc_exit_cap_ = 0;
  cont_cap_ = 0;
  h_exit_count_ = 0;
  h_cont_count_ = 0;
  n_roots_ = 0;
  ms_layer_idx_ = 0u;
  n_ms_layers_ = 0u;
  filter_desc_max_ci_ = 0u;
  filter_n_slot_ = 0u;
  crystal_config_id_ = 0xFFFFu;
  final_layer_filter_config_ = FilterConfig{};
  final_layer_crystal_ = Crystal{};
  final_layer_axis_dist_ = AxisDistribution{};
  final_ms_layer_idx_ = 0u;
  final_ms_prob_ = 0.0f;
  buffers_allocated_ = false;
  in_session_ = false;
  scene_ = nullptr;
  render_ = nullptr;
  // transit_seed_ / transit_ray_count_ / transit_seeded_ and the gate_*
  // counterparts are deliberately NOT cleared — the first-seeding gate in
  // BeginSession resets the monotone counter exactly once per spec.seed,
  // matching the Metal `transit_ray_count_` discipline (scrum-267).
}

void CudaTraceBackend::Impl::EnsureSessionBuffers(size_t n) {
  if (buffers_allocated_ && n_roots_ == n) {
    return;  // MVP: n_roots is fixed within a session; idempotent fast-path.
  }
  // Guard against UAF: if a prior kernel is still writing to d_exit_, freeing
  // now would corrupt device memory. cudaDeviceSynchronize is a no-op on the
  // first call (buffers_allocated_ == false) and safe before any re-alloc.
  if (buffers_allocated_) {
    cudaDeviceSynchronize();
  }

  // Release old per-batch buffers (cudaFree(nullptr) is a no-op per CUDA spec).
  cudaFree(d_dirs_);     d_dirs_ = nullptr;
  cudaFree(d_pos_);      d_pos_ = nullptr;
  cudaFree(d_ws_);       d_ws_ = nullptr;
  cudaFree(d_from_poly_); d_from_poly_ = nullptr;
  cudaFree(d_root_wl_idx_); d_root_wl_idx_ = nullptr;
  cudaFree(d_rot_c2w_);  d_rot_c2w_ = nullptr;
  cudaFree(d_exit_);     d_exit_ = nullptr;
  cudaFree(d_exit_count_); d_exit_count_ = nullptr;
  cudaFree(d_cont_d_);      d_cont_d_ = nullptr;
  cudaFree(d_cont_w_);      d_cont_w_ = nullptr;
  cudaFree(d_cont_wl_idx_); d_cont_wl_idx_ = nullptr;
  cudaFree(d_cont_count_);  d_cont_count_ = nullptr;
  cudaFreeHost(pinned_dirs_);     pinned_dirs_ = nullptr;
  cudaFreeHost(pinned_pos_);      pinned_pos_ = nullptr;
  cudaFreeHost(pinned_ws_);       pinned_ws_ = nullptr;
  cudaFreeHost(pinned_from_poly_); pinned_from_poly_ = nullptr;
  cudaFreeHost(pinned_root_wl_idx_); pinned_root_wl_idx_ = nullptr;
  cudaFreeHost(pinned_rot_c2w_);  pinned_rot_c2w_ = nullptr;
  cudaFreeHost(pinned_exit_);     pinned_exit_ = nullptr;
  cudaFreeHost(pinned_cont_count_); pinned_cont_count_ = nullptr;

  size_t max_hits = scene_ != nullptr ? scene_->max_hits_ : kMaxHits;
  exit_cap_ = ComputeExitCap(n, max_hits);
  cont_cap_ = ComputeContCap(n, max_hits);

  // Root / rotation / continuation buffers MUST be sized to cont_cap_, not n:
  // the transit_multi_ms_kernel writes up to cont_cap_ roots in a single
  // dispatch (the next-layer fan-out from this layer's continuation count).
  // Sizing by n alone would overrun these buffers on layers where the
  // continuation set exceeds the original root_ray_count. Pinned-staging
  // counterparts (filled only on the ms_mode==0 first-layer ingest path)
  // stay sized to n_roots since they back the host root-gen output only.
  size_t buf_cap = std::max<size_t>(n, cont_cap_);

  // Check every CUDA allocation. On failure Reset() tears down all buffers
  // (including session-level geometry) and BackendUnavailableError propagates
  // to TraceLayer's caller, ending the session cleanly.
  auto ck = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::EnsureSessionBuffers: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };

  ck(cudaMalloc(&d_dirs_,        3 * buf_cap * sizeof(float)),             "cudaMalloc d_dirs");
  ck(cudaMalloc(&d_pos_,         3 * buf_cap * sizeof(float)),             "cudaMalloc d_pos");
  ck(cudaMalloc(&d_ws_,          buf_cap * sizeof(float)),                 "cudaMalloc d_ws");
  ck(cudaMalloc(&d_from_poly_,   buf_cap * sizeof(uint32_t)),              "cudaMalloc d_from_poly");
  ck(cudaMalloc(&d_root_wl_idx_, buf_cap * sizeof(uint32_t)),              "cudaMalloc d_root_wl_idx");
  ck(cudaMalloc(&d_rot_c2w_,     9 * buf_cap * sizeof(float)),             "cudaMalloc d_rot_c2w");
  ck(cudaMalloc(&d_exit_,        exit_cap_ * sizeof(ExitRayRecord)),       "cudaMalloc d_exit");
  ck(cudaMalloc(&d_exit_count_,  sizeof(uint32_t)),                        "cudaMalloc d_exit_count");
  ck(cudaMalloc(&d_cont_d_,      3 * cont_cap_ * sizeof(float)),           "cudaMalloc d_cont_d");
  ck(cudaMalloc(&d_cont_w_,      cont_cap_ * sizeof(float)),               "cudaMalloc d_cont_w");
  ck(cudaMalloc(&d_cont_wl_idx_, cont_cap_ * sizeof(uint32_t)),            "cudaMalloc d_cont_wl_idx");
  ck(cudaMalloc(&d_cont_count_,  sizeof(uint32_t)),                        "cudaMalloc d_cont_count");

  ck(cudaHostAlloc(&pinned_dirs_,     3 * n * sizeof(float),               cudaHostAllocDefault), "cudaHostAlloc pinned_dirs");
  ck(cudaHostAlloc(&pinned_pos_,      3 * n * sizeof(float),               cudaHostAllocDefault), "cudaHostAlloc pinned_pos");
  ck(cudaHostAlloc(&pinned_ws_,       n * sizeof(float),                   cudaHostAllocDefault), "cudaHostAlloc pinned_ws");
  ck(cudaHostAlloc(&pinned_from_poly_, n * sizeof(uint32_t),               cudaHostAllocDefault), "cudaHostAlloc pinned_from_poly");
  ck(cudaHostAlloc(&pinned_root_wl_idx_, n * sizeof(uint32_t),             cudaHostAllocDefault), "cudaHostAlloc pinned_root_wl_idx");
  ck(cudaHostAlloc(&pinned_rot_c2w_,  9 * n * sizeof(float),               cudaHostAllocDefault), "cudaHostAlloc pinned_rot_c2w");
  ck(cudaHostAlloc(&pinned_exit_,     exit_cap_ * sizeof(ExitRayRecord),   cudaHostAllocDefault), "cudaHostAlloc pinned_exit");
  ck(cudaHostAlloc(&pinned_cont_count_, sizeof(uint32_t),                  cudaHostAllocDefault), "cudaHostAlloc pinned_cont_count");

  // First-use cont-counter zeroing. cudaMalloc carries no zero-init guarantee
  // (the CUDA spec is explicit), so a freshly allocated d_cont_count_ may
  // hold non-zero garbage. Without this memset, the first ms_mode==1 trace
  // dispatch would atomicAdd into a non-zero starting offset and silently
  // overrun the cont_d_/cont_w_/cont_wl_idx_ pool — caught only by the
  // parity-battery cross-seed test. Per-layer recycling (after Recombine
  // drains the count) is owned by Recombine itself; here we only handle the
  // initial state.
  ck(cudaMemset(d_cont_count_, 0, sizeof(uint32_t)), "cudaMemset d_cont_count_ (init)");

  n_roots_ = n;
  alloc_exit_cap_ = exit_cap_;  // track physical d_exit_ size for grow-only EnsureExitCapacity
  buffers_allocated_ = true;
}

void CudaTraceBackend::Impl::EnsureExitCapacity(size_t n) {
  // Device-roots continuation layer: root buffers already hold the transit
  // output and are sized by the layer that wrote them — do NOT touch them. Only
  // the exit pool may need to grow for this layer's fan-out. exit_cap_ tracks the
  // logical cap used by the kernel/overflow-clamp; alloc_exit_cap_ tracks the
  // physical d_exit_ allocation so we realloc only when genuinely insufficient.
  const size_t need = ComputeExitCap(n, scene_ != nullptr ? scene_->max_hits_ : kMaxHits);
  exit_cap_ = need;
  if (!buffers_allocated_ || need <= alloc_exit_cap_) {
    return;  // existing d_exit_ already holds `need` records.
  }
  cudaDeviceSynchronize();  // a prior layer's kernel may still be writing d_exit_.
  auto ck = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::EnsureExitCapacity: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };
  cudaFree(d_exit_);            d_exit_ = nullptr;
  cudaFreeHost(pinned_exit_);   pinned_exit_ = nullptr;
  ck(cudaMalloc(&d_exit_, need * sizeof(ExitRayRecord)), "cudaMalloc d_exit (grow)");
  ck(cudaHostAlloc(&pinned_exit_, need * sizeof(ExitRayRecord), cudaHostAllocDefault),
     "cudaHostAlloc pinned_exit (grow)");
  alloc_exit_cap_ = need;
}

// EnsureFilterBuffers — mirror of Metal `EnsureFilterBuffers`
// (metal_trace_backend.mm:1090-1226). Builds the per-(layer,ci) DeviceFilterDesc
// array + the flat GetFn byte stream + Complex sub-desc buffer; uploads them
// to device memory. For sessions with no Scene/ms_/setting_ entries the four
// device pointers fall back to 1-byte dummies so the kernel's ms_mode==1 emit
// gate can always bind valid pointers; a `type=kDeviceFilterTypeNone` dummy
// desc makes DeviceFilterCheck return true (pass-through).
//
// Single-CI MVP: max_ci is structurally 1, so `gate_slot = ms_layer_idx`.
// Multi-CI per-layer crystal pool is 296.6.
void CudaTraceBackend::Impl::EnsureFilterBuffers(const SessionSpec& spec) {
  // Free any prior session's filter buffers (cudaFree on nullptr is a no-op).
  cudaFree(d_filter_desc_);       d_filter_desc_       = nullptr;
  cudaFree(d_getfn_offsets_);     d_getfn_offsets_     = nullptr;
  cudaFree(d_getfn_bytes_);       d_getfn_bytes_       = nullptr;
  cudaFree(d_complex_sub_desc_);  d_complex_sub_desc_  = nullptr;
  filter_desc_max_ci_ = 0u;
  filter_n_slot_      = 0u;

  auto ck = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::EnsureFilterBuffers: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };

  auto alloc_dummies = [&]() {
    ck(cudaMalloc(&d_filter_desc_, sizeof(DeviceFilterDesc)), "cudaMalloc d_filter_desc (dummy)");
    DeviceFilterDesc dummy{};
    dummy.type = kDeviceFilterTypeNone;
    ck(cudaMemcpy(d_filter_desc_, &dummy, sizeof(DeviceFilterDesc), cudaMemcpyHostToDevice),
       "cudaMemcpy d_filter_desc (dummy)");
    uint32_t dummy_offsets[2] = {0u, 0u};
    ck(cudaMalloc(&d_getfn_offsets_, sizeof(dummy_offsets)), "cudaMalloc d_getfn_offsets (dummy)");
    ck(cudaMemcpy(d_getfn_offsets_, dummy_offsets, sizeof(dummy_offsets), cudaMemcpyHostToDevice),
       "cudaMemcpy d_getfn_offsets (dummy)");
    ck(cudaMalloc(&d_getfn_bytes_, 1u), "cudaMalloc d_getfn_bytes (dummy)");
    ck(cudaMalloc(&d_complex_sub_desc_, sizeof(DeviceFilterDesc)), "cudaMalloc d_complex_sub_desc (dummy)");
    filter_desc_max_ci_ = 1u;
    filter_n_slot_      = 1u;
  };

  if (spec.scene == nullptr || spec.scene->ms_.empty()) {
    alloc_dummies();
    return;
  }

  size_t n_layers = spec.scene->ms_.size();
  size_t max_ci   = 0;
  for (const auto& ms : spec.scene->ms_) {
    if (ms.setting_.size() > max_ci) {
      max_ci = ms.setting_.size();
    }
  }
  if (max_ci == 0) {
    alloc_dummies();
    return;
  }

  filter_desc_max_ci_ = static_cast<uint32_t>(max_ci);
  size_t n_slot = n_layers * max_ci;
  filter_n_slot_ = static_cast<uint32_t>(n_slot);

  // Build per-slot descriptors + per-slot GetFn byte stripes via a private
  // proto RNG (private to keep the session's main rng_ pristine for the first
  // TraceLayer's per-batch MakeCrystal). Hex prism GetFn is shape-invariant
  // across orientation/aspect, so any sampled instance suffices for the
  // canonical bytes + GetFn table (mirrors Metal proto_rng pattern).
  RandomNumberGenerator proto_rng(0xC0FEFEEDu);
  std::vector<DeviceFilterDesc> descs(n_slot);
  std::vector<std::vector<uint8_t>> per_slot_bytes(n_slot);
  std::vector<DeviceFilterDesc> all_sub_descs;

  for (size_t mi = 0; mi < n_layers; ++mi) {
    const auto& ms = spec.scene->ms_[mi];
    for (size_t ci = 0; ci < ms.setting_.size(); ++ci) {
      const auto& setting = ms.setting_[ci];
      Crystal proto = MakeCrystal(proto_rng, setting.crystal_.param_);
      size_t slot = mi * max_ci + ci;
      descs[slot] = detail::BuildDeviceFilterDesc(setting.filter_, proto, setting.crystal_.axis_);
      per_slot_bytes[slot] = detail::BuildDeviceGetFnBytes(proto);
      if (descs[slot].type == kDeviceFilterTypeComplex) {
        const auto* complex_p = std::get_if<ComplexFilterParam>(&setting.filter_.param_);
        // BuildDeviceFilterDesc produces Complex only when the variant carries
        // ComplexFilterParam; absence here is a programming error.
        assert(complex_p != nullptr && "Complex desc type without ComplexFilterParam variant");
        descs[slot].sub_desc_start = static_cast<uint32_t>(all_sub_descs.size());
        detail::BuildComplexSubDescs(*complex_p, proto, descs[slot].symmetry, descs[slot].sigma_a,
                                     descs[slot].d_applicable != 0u, all_sub_descs);
      }
    }
    // Trailing slots (ms.setting_.size() < max_ci) keep zero-init
    // DeviceFilterDesc{type=kDeviceFilterTypeNone} + empty GetFn stripe, so an
    // out-of-range gate_slot surfaces as a pass-through true.
  }

  // Upload descriptors.
  size_t descs_bytes = n_slot * sizeof(DeviceFilterDesc);
  ck(cudaMalloc(&d_filter_desc_, descs_bytes), "cudaMalloc d_filter_desc");
  ck(cudaMemcpy(d_filter_desc_, descs.data(), descs_bytes, cudaMemcpyHostToDevice), "cudaMemcpy d_filter_desc");

  // Upload GetFn prefix-sum offsets + flat byte stream.
  std::vector<uint32_t> offsets(n_slot + 1u, 0u);
  for (size_t i = 0; i < n_slot; ++i) {
    offsets[i + 1] = offsets[i] + static_cast<uint32_t>(per_slot_bytes[i].size());
  }
  size_t offsets_bytes = offsets.size() * sizeof(uint32_t);
  ck(cudaMalloc(&d_getfn_offsets_, offsets_bytes), "cudaMalloc d_getfn_offsets");
  ck(cudaMemcpy(d_getfn_offsets_, offsets.data(), offsets_bytes, cudaMemcpyHostToDevice),
     "cudaMemcpy d_getfn_offsets");

  size_t total_bytes = offsets.back();
  size_t alloc_bytes = std::max<size_t>(total_bytes, 1u);  // 1-byte floor so the buffer is always bindable
  ck(cudaMalloc(&d_getfn_bytes_, alloc_bytes), "cudaMalloc d_getfn_bytes");
  if (total_bytes > 0) {
    std::vector<uint8_t> flat(total_bytes, 0u);
    for (size_t i = 0; i < n_slot; ++i) {
      if (!per_slot_bytes[i].empty()) {
        std::memcpy(flat.data() + offsets[i], per_slot_bytes[i].data(), per_slot_bytes[i].size());
      }
    }
    ck(cudaMemcpy(d_getfn_bytes_, flat.data(), total_bytes, cudaMemcpyHostToDevice), "cudaMemcpy d_getfn_bytes");
  }

  // Upload Complex sub-descs (1-byte dummy fallback when none present, so the
  // kernel pointer is always bindable; DeviceFilterCheck only reads through it
  // on Complex slots).
  size_t sub_desc_bytes = all_sub_descs.size() * sizeof(DeviceFilterDesc);
  size_t sub_alloc_bytes = std::max<size_t>(sub_desc_bytes, sizeof(DeviceFilterDesc));
  ck(cudaMalloc(&d_complex_sub_desc_, sub_alloc_bytes), "cudaMalloc d_complex_sub_desc");
  if (sub_desc_bytes > 0) {
    ck(cudaMemcpy(d_complex_sub_desc_, all_sub_descs.data(), sub_desc_bytes, cudaMemcpyHostToDevice),
       "cudaMemcpy d_complex_sub_desc");
  }

  // Capture final-layer host filter state — DrainExits applies FilterSpec on
  // records tagged with the final ms_layer_idx (mirrors Metal last_layer_*).
  // Single-CI MVP: take setting_[0] of the last MS layer.
  const auto& last_ms_layer = spec.scene->ms_.back();
  if (!last_ms_layer.setting_.empty()) {
    const auto& last_setting = last_ms_layer.setting_[0];
    final_layer_filter_config_ = last_setting.filter_;
    RandomNumberGenerator drain_proto_rng(0xC0FEFEEDu);
    final_layer_crystal_ = MakeCrystal(drain_proto_rng, last_setting.crystal_.param_);
    final_layer_axis_dist_ = last_setting.crystal_.axis_;
  }
  final_ms_layer_idx_ = static_cast<uint8_t>(n_layers - 1u);
  final_ms_prob_ = last_ms_layer.prob_;

  // crystal_config_id_ for DeviceFilterMatchCrystal. Mirrors Metal line 1705-
  // 1707: when the source crystal has no config_id (kInvalidId), surface
  // 0xFFFFu so a Crystal-type filter never matches by accident.
  crystal_config_id_ = (final_layer_crystal_.config_id_ == kInvalidId)
                          ? 0xFFFFu
                          : static_cast<uint32_t>(final_layer_crystal_.config_id_);
}

CudaTraceBackend::CudaTraceBackend(Logger* logger) : impl_(std::make_unique<Impl>()) {
  impl_->logger = logger;
}

CudaTraceBackend::~CudaTraceBackend() {
  if (impl_) {
    impl_->Reset();
  }
}

void CudaTraceBackend::BeginSession(const SessionSpec& spec) {
  if (impl_->in_session_) {
    throw BackendUnavailableError("CudaTraceBackend::BeginSession called on already-open session");
  }

  cudaError_t err = cudaSetDevice(0);
  if (err != cudaSuccess) {
    (void)cudaGetLastError();
    throw BackendUnavailableError(std::string{"CudaTraceBackend::BeginSession: cudaSetDevice failed: "} +
                                  cudaGetErrorString(err));
  }

  try {
    impl_->scene_ = spec.scene;
    impl_->render_ = spec.render;
    impl_->wl_ = spec.wl;
    impl_->rng_.SetSeed(spec.seed);

    // MVP: single MS, single crystal config — ms_[0].setting_[0].
    if (spec.scene == nullptr || spec.scene->ms_.empty() || spec.scene->ms_[0].setting_.empty()) {
      throw BackendUnavailableError("CudaTraceBackend::BeginSession: empty SceneConfig.ms_/setting_");
    }
    const auto& ms_setting = spec.scene->ms_[0].setting_[0];

    // Build a deterministic Crystal sample for geometry upload. dummy_rng
    // does NOT touch impl_->rng_ so the persistent batch RNG stays pristine
    // for the first TraceLayer's MakeCrystal + InitRayFirstMs.
    //
    // Determinism premise: Crystal geometry (vertices, normals, polygon
    // topology) is fully determined by CrystalParam. MakeCrystal's internal
    // CrystalMaker visits the CrystalParam variant; the RNG is consulted
    // only when the variant carries a stochastic shape parameter (random
    // wedge angle / aspect ratio), and even then the *topology* (vertex
    // count, face indexing) is identical across draws — only the vertex
    // coordinates change. CPU backend builds a fresh Crystal per batch for
    // the same CrystalParam (cpu_trace_backend.cpp:325) and consumes it as
    // the trace oracle; we mirror that pattern here. If CrystalParam carries
    // a stochastic *shape*, the per-batch geometry the kernel traces against
    // will diverge from the BeginSession upload — that's a follow-up
    // (multi-shape pool) and not in MVP scope.
    RandomNumberGenerator dummy_rng{0u};
    Crystal crystal_for_geom = MakeCrystal(dummy_rng, ms_setting.crystal_.param_);

    size_t poly_cnt = crystal_for_geom.PolygonFaceCount();
    if (poly_cnt == 0) {
      throw BackendUnavailableError("CudaTraceBackend::BeginSession: degenerate crystal geometry (0 polygons)");
    }
    impl_->poly_cnt_ = static_cast<uint32_t>(poly_cnt);

    // Detect stochastic CrystalParam: if a Crystal built with the session
    // seed differs in polygon count from dummy_rng's crystal, the uploaded
    // geometry will diverge from per-batch TraceLayer crystals and G1 parity
    // may fail. MVP dual_fisheye_ref uses deterministic prism/pyramid shapes.
    if (impl_->logger != nullptr) {
      RandomNumberGenerator probe_rng{spec.seed};
      Crystal probe_crystal = MakeCrystal(probe_rng, ms_setting.crystal_.param_);
      if (probe_crystal.PolygonFaceCount() != poly_cnt) {
        ILOG_WARN(*impl_->logger,
                  "CudaTraceBackend::BeginSession: stochastic CrystalParam detected — "
                  "uploaded geometry may diverge from per-batch TraceLayer crystals; "
                  "G1 parity may fail for non-deterministic crystal shapes");
      }
    }

    // Polygon-face geometry H2D: outward normals (3 × poly_cnt, AoS) +
    // plane constants (poly_cnt). Layout matches Crystal::GetPolygonFace*
    // contracts (crystal.cpp:191/574/578); the kernel reads them via
    // `d_poly_n[fi*3 + {0,1,2}]` and `d_poly_d[fi]`, mirroring legacy
    // `PropagateSlab` (optics.cpp:111-125).
    CheckCuda(cudaMalloc(&impl_->d_poly_n_, 3 * poly_cnt * sizeof(float)), "BeginSession cudaMalloc d_poly_n");
    CheckCuda(cudaMemcpy(impl_->d_poly_n_, crystal_for_geom.GetPolygonFaceNormal(),
                         3 * poly_cnt * sizeof(float), cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_poly_n");
    CheckCuda(cudaMalloc(&impl_->d_poly_d_, poly_cnt * sizeof(float)), "BeginSession cudaMalloc d_poly_d");
    CheckCuda(cudaMemcpy(impl_->d_poly_d_, crystal_for_geom.GetPolygonFaceDist(),
                         poly_cnt * sizeof(float), cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_poly_d");

    // Triangle pool H2D for transit_multi_ms_kernel (296.4). vtx/norm/area
    // come straight from Crystal; tri_to_poly is computed host-side via the
    // argmax + coplanar-floor predicate (mirrors Metal UploadCrystal). The
    // upload is session-scoped (single CI in MVP); multi-CI per-layer crystal
    // pool is 296.6.
    size_t tri_cnt = crystal_for_geom.TotalTriangles();
    if (tri_cnt == 0) {
      throw BackendUnavailableError("CudaTraceBackend::BeginSession: degenerate crystal geometry (0 triangles)");
    }
    if (tri_cnt > static_cast<size_t>(lm_pcg::kMaxTriPerKernel)) {
      throw BackendUnavailableError(
          std::string{"CudaTraceBackend::BeginSession: tri_count="} + std::to_string(tri_cnt) +
          " exceeds kMaxTriPerKernel=" + std::to_string(lm_pcg::kMaxTriPerKernel) +
          " (transit_multi_ms_kernel proj_prob[] stack bound)");
    }
    impl_->tri_cnt_ = static_cast<uint32_t>(tri_cnt);
    CheckCuda(cudaMalloc(&impl_->d_tri_vtx_, 9 * tri_cnt * sizeof(float)),
              "BeginSession cudaMalloc d_tri_vtx");
    CheckCuda(cudaMemcpy(impl_->d_tri_vtx_, crystal_for_geom.GetTriangleVtx(),
                         9 * tri_cnt * sizeof(float), cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_tri_vtx");
    CheckCuda(cudaMalloc(&impl_->d_tri_norm_, 3 * tri_cnt * sizeof(float)),
              "BeginSession cudaMalloc d_tri_norm");
    CheckCuda(cudaMemcpy(impl_->d_tri_norm_, crystal_for_geom.GetTriangleNormal(),
                         3 * tri_cnt * sizeof(float), cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_tri_norm");
    CheckCuda(cudaMalloc(&impl_->d_tri_area_, tri_cnt * sizeof(float)),
              "BeginSession cudaMalloc d_tri_area");
    CheckCuda(cudaMemcpy(impl_->d_tri_area_, crystal_for_geom.GetTirangleArea(),
                         tri_cnt * sizeof(float), cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_tri_area");
    std::vector<uint16_t> tri_to_poly_host;
    FillTriToPoly(crystal_for_geom, tri_to_poly_host);
    CheckCuda(cudaMalloc(&impl_->d_tri_to_poly_, tri_cnt * sizeof(uint16_t)),
              "BeginSession cudaMalloc d_tri_to_poly");
    CheckCuda(cudaMemcpy(impl_->d_tri_to_poly_, tri_to_poly_host.data(),
                         tri_cnt * sizeof(uint16_t), cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_tri_to_poly");

    // Per-ray wavelength pool (296.6 DR-3). One session covers the whole
    // spectrum; build the WlEntry table (n_idx + spd_weight per sampled wl) from
    // the scene's light source + this crystal and upload it once. The host
    // first-layer path / device gen-root draws a per-ray wl_idx in [0, M); the
    // trace kernel reads d_wl_pool_[wl_idx].n_idx (per-ray refraction) and tags
    // exit records with wl_idx for host-side wavelength reconstruction
    // (simulator.cpp). Mirrors Metal ComputeWlPool + EnsureWlPoolBuffer. The
    // buffer is allocated once (size = env-resolved M, stable per process) and
    // re-uploaded each BeginSession since the crystal n_idx is scene-dependent.
    Logger& wl_logger = impl_->logger != nullptr ? *impl_->logger : GetGlobalLogger();
    impl_->wl_pool_size_ = ResolveWlPoolSize(wl_logger);
    const auto& spectrum = spec.scene->light_source_.spectrum_;
    impl_->illuminant_mode_ = std::holds_alternative<IlluminantType>(spectrum);
    impl_->illuminant_ = impl_->illuminant_mode_ ? std::get<IlluminantType>(spectrum) : IlluminantType{};
    ComputeWlPool(crystal_for_geom, impl_->illuminant_mode_, impl_->illuminant_, spec.wl.wl_, spec.wl.weight_,
                  impl_->wl_pool_size_, impl_->wl_pool_host_);
    if (impl_->d_wl_pool_ == nullptr) {
      CheckCuda(cudaMalloc(&impl_->d_wl_pool_, impl_->wl_pool_size_ * sizeof(WlEntry)),
                "BeginSession cudaMalloc d_wl_pool");
    }
    CheckCuda(cudaMemcpy(impl_->d_wl_pool_, impl_->wl_pool_host_.data(), impl_->wl_pool_size_ * sizeof(WlEntry),
                         cudaMemcpyHostToDevice),
              "BeginSession cudaMemcpy d_wl_pool");

    // MS layer count + initial layer index. Used by TraceLayer to derive
    // ms_mode (continuation vs final exit) when Step 5 lands. n_ms_layers_
    // is captured once per session (multi-CI is 296.6 work and does not
    // change ms_.size()).
    size_t n_ms = spec.scene->ms_.size();
    if (n_ms > 255u) {
      throw BackendUnavailableError("CudaTraceBackend::BeginSession: ms_.size() > 255 (uint8_t ms_layer_idx_)");
    }
    impl_->n_ms_layers_ = static_cast<uint8_t>(n_ms);
    impl_->ms_layer_idx_ = 0u;

    // PCG-stream seeding (296.4). transit / gate / future device-gen each get
    // an independent stream; nonce-XOR keeps them disjoint when spec.seed=0
    // (driver-non-deterministic) collapses gen_seed_ to zero. transit_seeded_
    // / gate_seeded_ guard the first-time reset of the monotone counter so a
    // BeginSession in the middle of a Run() does not overwrite the running
    // (transit_seed_, transit_ray_count_) PCG position (mirrors the Metal
    // first-seeding gate, see metal_trace_backend.mm:2037).
    impl_->transit_seed_ = spec.seed ^ kCudaTransitNonce;
    impl_->gate_seed_    = spec.seed ^ kCudaGateNonce;
    if (!impl_->transit_seeded_) {
      impl_->transit_ray_count_ = 0;
      impl_->transit_seeded_ = true;
    }
    if (!impl_->gate_seeded_) {
      impl_->gate_ray_count_ = 0;
      impl_->gate_seeded_ = true;
    }
    // Drain RNG: seed ONCE per Run() and advance across all per-wavelength-batch
    // BeginSession calls, exactly like the transit_/gate_ counters above. The
    // earlier "re-seed every BeginSession" assumed DrainExits is called
    // per-session-not-per-frame — false: BeginSession runs per wavelength batch
    // (same spec.seed each time), so reseeding replayed one mt19937 prefix every
    // batch and biased the final-layer prob keep-fraction (296.5 Bug B). The
    // drain_seeded_ guard mirrors transit_seeded_/gate_seeded_; cross-seed
    // independence comes from spec.seed varying between runs (fresh Impl → guard
    // re-armed). See [[project_scrum296_progress_filter_blocked]].
    if (!impl_->drain_seeded_) {
      impl_->drain_rng_.seed(spec.seed ^ kCudaDrainNonce);
      impl_->drain_seeded_ = true;
    }

    // Upload per-(layer,ci) filter descriptors so the kernel's emit gate
    // (ms_mode==1) can call DeviceFilterCheck, and the final-layer host filter
    // info is captured for DrainExits. Idempotent across BeginSession calls.
    impl_->EnsureFilterBuffers(spec);

    // Per-ray rotation matrix device buffer (cont_cap_ × 9) is allocated
    // lazily in EnsureSessionBuffers, not here — n_roots is unknown until the
    // first TraceLayer call. rng_ stays pristine (no sampling here).
    // d_cont_count_ is zeroed in the first TraceLayer of each session
    // (ms_layer_idx_==0 path) after EnsureSessionBuffers returns, even when
    // the buffer is reused from a prior session (EnsureSessionBuffers fast-path
    // skips zeroing). Recombine owns per-layer zeroing for subsequent layers.

    if (!impl_->events_created_) {
      CheckCuda(cudaEventCreate(&impl_->ev_start_h2d_),   "BeginSession cudaEventCreate ev_start_h2d");
      CheckCuda(cudaEventCreate(&impl_->ev_end_h2d_),     "BeginSession cudaEventCreate ev_end_h2d");
      CheckCuda(cudaEventCreate(&impl_->ev_end_kernel_),  "BeginSession cudaEventCreate ev_end_kernel");
      CheckCuda(cudaEventCreate(&impl_->ev_end_d2h_),     "BeginSession cudaEventCreate ev_end_d2h");
      impl_->events_created_ = true;
    }

    impl_->in_session_ = true;
    if (impl_->logger != nullptr) {
      ILOG_INFO(*impl_->logger,
                "CudaTraceBackend::BeginSession: device=0 poly_cnt={} tri_cnt={} n_ms={} seed={} n_idx={:.4f}",
                poly_cnt, tri_cnt, n_ms, spec.seed,
                crystal_for_geom.GetRefractiveIndex(spec.wl.wl_));
    }
  } catch (...) {
    impl_->Reset();
    throw;
  }
}

LayerHandlePtr CudaTraceBackend::TraceLayer(const RootRaySource& roots) {
  if (!impl_->in_session_) {
    throw BackendUnavailableError("CudaTraceBackend::TraceLayer called outside session");
  }
  // Determine `n` from the active source: host-mode uses the caller's count,
  // device-mode uses the count stamped by the previous Recombine on the
  // continuation ring. The device branch trusts `DeviceRayBatch::backend_ptr`
  // is the d_dirs_ tag handed back by Recombine (see seam contract +
  // Recombine implementation below).
  const size_t n = roots.is_device ? roots.device.count : roots.host.count;
  if (n == 0) {
    // No-op layer: keep ms_layer_idx_ as-is; the caller may still invoke
    // Recombine to bump it, but with zero rays nothing transits.
    // Zero h_cont_count_ so a subsequent Recombine sees zero continuations
    // even if a prior ms_mode==1 layer left a stale value.
    impl_->h_cont_count_ = 0u;
    return std::make_unique<CudaLayerHandle>(0u, LayerStats{0u, 0.0f});
  }

  if (roots.is_device) {
    // Device-roots continuation layer: the root buffers were filled in-place by
    // the previous Recombine's transit_multi_ms_kernel and are already sized by
    // that layer. Running EnsureSessionBuffers here (n = cont_n != n_roots_)
    // would free + realloc them mid-flight, destroying the continuation rays
    // (296.4 root-cause: d_ws_/d_from_poly_ then read uninitialised → w=0 →
    // every layer-1 ray skips entry → ~80% of multi-MS energy lost). Only the
    // exit pool may need to grow for this layer's exits.
    impl_->EnsureExitCapacity(n);
  } else {
    impl_->EnsureSessionBuffers(n);
  }

  // Local helper: Reset() + BackendUnavailableError for CUDA call failures
  // inside TraceLayer. EnsureSessionBuffers has its own internal Reset path;
  // this lambda is only for the CUDA calls after it returns.
  auto ck_reset = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      impl_->Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::TraceLayer: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };

  // Per-session zero of d_cont_count_ at first layer: EnsureSessionBuffers
  // zeroes on first alloc but skips zeroing when reusing buffers (fast-path).
  // A prior session that ended without Recombine may leave a non-zero count;
  // ms_layer_idx_==0 is the session-start sentinel (BeginSession resets it).
  if (impl_->ms_layer_idx_ == 0) {
    ck_reset(cudaMemset(impl_->d_cont_count_, 0, sizeof(uint32_t)),
             "cudaMemset d_cont_count_ (session start)");
  }

  // Resolve current MS layer's setting + refractive index. MVP uses the first
  // crystal_ of `ms_[ms_layer_idx_].setting_[0]` (single-CI assumption); multi-
  // CI per-layer crystal pool is 296.6.
  if (impl_->ms_layer_idx_ >= impl_->n_ms_layers_) {
    throw BackendUnavailableError(
        std::string{"CudaTraceBackend::TraceLayer: ms_layer_idx_="} +
        std::to_string(impl_->ms_layer_idx_) + " >= n_ms_layers_=" +
        std::to_string(impl_->n_ms_layers_) + " (over-traced past final layer)");
  }
  const auto& ms_layer   = impl_->scene_->ms_[impl_->ms_layer_idx_];
  const auto& ms_setting = ms_layer.setting_[0];

  // ms_mode = continuation iff this is NOT the final MS layer. Final layer
  // (ms_mode==0) writes every exit candidate to d_exit_; non-final layer
  // (ms_mode==1) routes each candidate through the per-emit PCG gate (see
  // trace_single_ms_kernel header comment).
  const bool is_final_layer = (impl_->ms_layer_idx_ + 1u >= impl_->n_ms_layers_);
  const uint8_t ms_mode = is_final_layer ? 0u : 1u;
  const float ms_prob = is_final_layer ? 0.0f : ms_layer.prob_;

  cudaEventRecord(impl_->ev_start_h2d_);

  if (!roots.is_device) {
    // ── Host-roots path (first MS layer or single-MS session) ──────────────
    // Host-side root-ray generation, mirroring cpu_trace_backend.cpp:135-140.
    // Build a fresh Crystal per batch via MakeCrystal(rng_, param_) so the
    // crystal-orientation RNG draw stays consistent with the CPU oracle's
    // batch-by-batch ordering. Wrapped in try/catch: std::bad_alloc from
    // RayBuffer growth or other C++ exceptions reset device state before
    // propagating (mirrors BeginSession's exception-safety contract).
    try {
      Crystal crystal = MakeCrystal(impl_->rng_, ms_setting.crystal_.param_);
      const auto& axis_dist = ms_setting.crystal_.axis_;

      // Mirror the CPU pattern (cpu_trace_backend.cpp:107-108): workspace[0]/[1]
      // capacities sized for the InitRay/HitSurface fan-out so EmplaceBack
      // doesn't grow them mid-trace. We only consume workspace[0] (first-hit
      // snapshot of d/p/w/crystal_rot_), but InitRayFirstMs writes both slots.
      RayBuffer workspace[2]{};
      workspace[0].Reset(n * 2);
      workspace[1].Reset(n * 4);
      // all_data is the per-batch RaySeg bookkeeping buffer. Use the same
      // AllocateAllData sizing as the simulator (simulator.cpp:811) — overshoots
      // for our single-MS use but keeps the capacity contract obviously safe.
      RayBuffer all_data = AllocateAllData(*impl_->scene_, n);
      InitRayFirstMs(impl_->rng_, impl_->scene_->light_source_.param_, impl_->wl_, n, crystal,
                     impl_->ms_layer_idx_, axis_dist, workspace, all_data);

      // Copy crystal-local d/p/w + per-ray crystal->world rotation into pinned
      // staging. to_face_ is the InitRay_p_fid entry-face polygon id; the
      // kernel needs it as the initial from_poly to suppress self-intersect on
      // the first polygon-slab sweep (p sits exactly on this polygon).
      // IdType (uint16_t per raypath.hpp) widens cleanly to uint32_t;
      // kInvalidId (0xFFFFFFFFu after widening) maps to the kernel's "no
      // previous face" sentinel. InitRayFirstMs samples an independent crystal
      // orientation per root ray (halo-ring distribution comes from this); each
      // RaySeg.crystal_rot_ is row-major 3x3. Aligns d_rot_c2w_ with d_dirs_'s
      // crystal-local frame — frame invariant 6. Mirrors
      // metal_trace_backend.mm:1375.
      for (size_t i = 0; i < n; ++i) {
        const auto& r = all_data[i];
        impl_->pinned_dirs_[i * 3 + 0] = r.d_[0];
        impl_->pinned_dirs_[i * 3 + 1] = r.d_[1];
        impl_->pinned_dirs_[i * 3 + 2] = r.d_[2];
        impl_->pinned_pos_[i * 3 + 0] = r.p_[0];
        impl_->pinned_pos_[i * 3 + 1] = r.p_[1];
        impl_->pinned_pos_[i * 3 + 2] = r.p_[2];
        // Per-ray wavelength (296.6 DR-3): draw a wl_idx and take the weight from
        // the pool's spd_weight. In per-ray (illuminant) mode InitRayFirstMs ran
        // with the session's zero-wl WlParam, so r.w_ is NOT the spectral weight;
        // the pool is the single source. In discrete mode the pool is degenerate
        // (all entries == spec_wl) so this is identical to r.w_. Mirrors Metal's
        // host-gen fallback (metal_trace_backend.mm:1377).
        uint32_t wl_idx = static_cast<uint32_t>(impl_->rng_.GetUniform() * static_cast<float>(impl_->wl_pool_size_));
        if (wl_idx >= impl_->wl_pool_size_) {
          wl_idx = impl_->wl_pool_size_ - 1u;  // guard GetUniform()→1.0 rounding
        }
        impl_->pinned_root_wl_idx_[i] = wl_idx;
        impl_->pinned_ws_[i] = impl_->wl_pool_host_[wl_idx].spd_weight;
        impl_->pinned_from_poly_[i] =
            (r.to_face_ == kInvalidId) ? 0xFFFFFFFFu : static_cast<uint32_t>(r.to_face_);
        std::memcpy(impl_->pinned_rot_c2w_ + i * 9, r.crystal_rot_.GetMat(), 9 * sizeof(float));
      }
    } catch (...) {
      impl_->Reset();
      throw;
    }

    // H2D upload — all copies issue on the default stream; the synchronous
    // cudaMemcpy(&h_exit_count_, ...) below is also default-stream and joins
    // these copies before the host reads the count, so per-ray rot/dir/p/w/
    // from_poly are guaranteed visible to the kernel launch.
    cudaMemcpyAsync(impl_->d_rot_c2w_, impl_->pinned_rot_c2w_, 9 * n * sizeof(float),
                    cudaMemcpyHostToDevice);
    cudaMemcpyAsync(impl_->d_dirs_, impl_->pinned_dirs_, 3 * n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpyAsync(impl_->d_pos_, impl_->pinned_pos_, 3 * n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpyAsync(impl_->d_ws_, impl_->pinned_ws_, n * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpyAsync(impl_->d_from_poly_, impl_->pinned_from_poly_, n * sizeof(uint32_t),
                    cudaMemcpyHostToDevice);
    cudaMemcpyAsync(impl_->d_root_wl_idx_, impl_->pinned_root_wl_idx_, n * sizeof(uint32_t),
                    cudaMemcpyHostToDevice);
    // Harvest any sticky async error from the H2D block.
    ck_reset(cudaGetLastError(), "H2D batch");
  } else {
    // ── Device-roots path (continuation from previous Recombine) ───────────
    // Root buffers (d_dirs_/d_pos_/d_ws_/d_from_poly_/d_rot_c2w_) were filled
    // in-place by `transit_multi_ms_kernel` inside the previous Recombine
    // dispatch; no H2D / pinned staging needed (host pointers do not cross
    // the seam in this path — §5 铁律). Per-ray n_idx now comes from the device
    // wl_pool via each ray's wl_idx (296.6 DR-3); the continuation rays carry
    // their wl_idx in d_root_wl_idx_ (written by transit_multi_ms_kernel), so the
    // device-roots path needs no host crystal / refractive-index resolution.
    // backend_ptr sanity: not strictly required for correctness (the rays live
    // in our own Impl buffers), but a NULL here means the caller forgot to
    // pass back the Recombine result — flag it loudly.
    if (roots.device.backend_ptr == nullptr) {
      throw BackendUnavailableError(
          "CudaTraceBackend::TraceLayer: device-roots path got NULL backend_ptr "
          "(caller did not propagate the previous Recombine result)");
    }
  }

  cudaEventRecord(impl_->ev_end_h2d_);

  ck_reset(cudaMemset(impl_->d_exit_count_, 0, sizeof(uint32_t)), "cudaMemset d_exit_count");

  uint32_t max_hits = static_cast<uint32_t>(impl_->scene_->max_hits_);
  uint32_t grid = (static_cast<uint32_t>(n) + 255u) / 256u;

  // Splitting (deterministic per-bounce fan-out) — no per-thread RNG needed.
  // The ms_mode/ms_prob/gate_* parameters route the kernel's emit gate (see
  // trace_single_ms_kernel header for the per-emit gate semantics). Final-
  // layer dispatches pass ms_mode=0 + nullptr cont buffers; non-final layers
  // pass ms_mode=1 with the d_cont_* ring + gate PCG seed.
  trace_single_ms_kernel<<<grid, 256>>>(
      impl_->d_dirs_, impl_->d_pos_, impl_->d_ws_, impl_->d_from_poly_,
      static_cast<uint32_t>(n), impl_->d_poly_n_, impl_->d_poly_d_,
      impl_->poly_cnt_, impl_->d_rot_c2w_, impl_->d_wl_pool_, impl_->d_root_wl_idx_,
      max_hits, impl_->d_exit_,
      static_cast<uint32_t>(impl_->exit_cap_), impl_->d_exit_count_,
      /*crystal_id=*/0u, /*ms_layer_idx=*/impl_->ms_layer_idx_,
      ms_mode, ms_prob,
      ms_mode == 1u ? impl_->d_cont_d_       : nullptr,
      ms_mode == 1u ? impl_->d_cont_w_       : nullptr,
      ms_mode == 1u ? impl_->d_cont_wl_idx_  : nullptr,
      ms_mode == 1u ? impl_->d_cont_count_   : nullptr,
      ms_mode == 1u ? static_cast<uint32_t>(impl_->cont_cap_) : 0u,
      impl_->gate_seed_,
      NarrowPcgRayBase(impl_->gate_ray_count_, n, "cuda-gate"),
      // 296.5 filter gate params. ms_mode==0 dispatches can pass these as
      // nullptr/0 because the gate code is unreachable; we always pass real
      // pointers to avoid undefined behaviour from null-pointer-arith warnings
      // on stricter compilers — EnsureFilterBuffers guarantees they are
      // non-null (1-byte dummies in the no-filter session case).
      ms_mode == 1u ? impl_->d_filter_desc_       : nullptr,
      ms_mode == 1u ? impl_->d_getfn_offsets_     : nullptr,
      ms_mode == 1u ? impl_->d_getfn_bytes_       : nullptr,
      ms_mode == 1u ? impl_->d_complex_sub_desc_  : nullptr,
      ms_mode == 1u ? impl_->filter_desc_max_ci_  : 0u,
      impl_->crystal_config_id_);
  // Peek immediately after launch: illegal address / invalid config errors
  // surface here before the synchronizing 4B readback, giving a precise
  // error origin instead of a deferred cudaErrorLaunchFailure on Memcpy.
  ck_reset(cudaPeekAtLastError(), "kernel launch");
  cudaEventRecord(impl_->ev_end_kernel_);

  // 4B readback (synchronous on the default stream — also the first sync
  // point that surfaces async kernel errors: check the return value).
  ck_reset(cudaMemcpy(&impl_->h_exit_count_, impl_->d_exit_count_, sizeof(uint32_t),
                      cudaMemcpyDeviceToHost), "4B readback");
  cudaEventRecord(impl_->ev_end_d2h_);
  cudaEventSynchronize(impl_->ev_end_d2h_);

  float h2d_ms = 0.0f;
  float kernel_ms = 0.0f;
  float d2h_ms = 0.0f;
  cudaEventElapsedTime(&h2d_ms, impl_->ev_start_h2d_, impl_->ev_end_h2d_);
  cudaEventElapsedTime(&kernel_ms, impl_->ev_end_h2d_, impl_->ev_end_kernel_);
  cudaEventElapsedTime(&d2h_ms, impl_->ev_end_kernel_, impl_->ev_end_d2h_);

  if (impl_->logger != nullptr) {
    ILOG_DEBUG(*impl_->logger,
               "CudaTraceBackend::TraceLayer: n={} ms_layer={} ms_mode={} ms_prob={:.3f} "
               "exit_count={} H2D={:.2f}ms kernel={:.2f}ms D2H={:.2f}ms",
               n, impl_->ms_layer_idx_, ms_mode, ms_prob,
               impl_->h_exit_count_, h2d_ms, kernel_ms, d2h_ms);
  }

  if (impl_->h_exit_count_ >= impl_->exit_cap_) {
    if (impl_->logger != nullptr) {
      ILOG_WARN(*impl_->logger,
                "CudaTraceBackend::TraceLayer: exit buffer overflow (count={} cap={}); tail dropped",
                impl_->h_exit_count_, impl_->exit_cap_);
    }
    impl_->h_exit_count_ = static_cast<uint32_t>(impl_->exit_cap_);
  }

  // Advance gate PCG counter ONLY in ms_mode==1 (final-layer dispatches do
  // not draw the gate PCG so they leave the counter undisturbed). Bumping
  // by `n` (root-ray count) — each ray's emit draws share a single
  // global_idx + advancing `slot`, so two batches with disjoint
  // global_idx ranges have fully disjoint streams.
  if (ms_mode == 1u) {
    impl_->gate_ray_count_ += n;
  }

  // Continuation-count readback (only meaningful for ms_mode==1; the kernel
  // does not touch d_cont_count_ on the ms_mode==0 path so it stays at the
  // value Recombine / EnsureSessionBuffers last zeroed it to).
  size_t cont_count_for_handle = 0u;
  if (ms_mode == 1u) {
    ck_reset(cudaMemcpy(&impl_->h_cont_count_, impl_->d_cont_count_, sizeof(uint32_t),
                        cudaMemcpyDeviceToHost), "4B cont_count readback");
    if (impl_->h_cont_count_ >= impl_->cont_cap_) {
      if (impl_->logger != nullptr) {
        ILOG_WARN(*impl_->logger,
                  "CudaTraceBackend::TraceLayer: cont buffer overflow (count={} cap={}); tail dropped",
                  impl_->h_cont_count_, impl_->cont_cap_);
      }
      impl_->h_cont_count_ = static_cast<uint32_t>(impl_->cont_cap_);
    }
    cont_count_for_handle = impl_->h_cont_count_;
  } else {
    impl_->h_cont_count_ = 0u;
  }

  return std::make_unique<CudaLayerHandle>(cont_count_for_handle,
                                           LayerStats{impl_->h_exit_count_, 0.0f});
}

RootRaySource CudaTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  (void)spec;
  if (!impl_->in_session_) {
    throw BackendUnavailableError("CudaTraceBackend::Recombine called outside session");
  }
  // Consume the layer handle (frees per-layer state at scope exit). The
  // continuation count is the authoritative value from impl_->h_cont_count_
  // populated by TraceLayer's tail readback.
  handle.reset();

  // Local error helper.
  auto ck_reset = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      impl_->Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::Recombine: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };

  const uint32_t cont_n = impl_->h_cont_count_;

  // No continuations → nothing to transit. Bump ms_layer_idx_ so the next
  // TraceLayer (if any) advances; return an empty DeviceRayBatch so the
  // caller can short-circuit. Also zero d_cont_count_ for the next layer.
  //
  // Null guard: an n=0 first-layer TraceLayer early-returns before
  // EnsureSessionBuffers runs, leaving d_cont_count_ == nullptr while the
  // documented contract still allows a follow-up Recombine. cudaMemset(nullptr)
  // returns cudaErrorInvalidValue → ck_reset would Reset + throw. When no
  // buffers exist there is nothing to zero, so skip the memset. (296.4 review)
  if (cont_n == 0u) {
    if (impl_->d_cont_count_ != nullptr) {
      ck_reset(cudaMemset(impl_->d_cont_count_, 0, sizeof(uint32_t)), "cudaMemset d_cont_count");
    }
    impl_->ms_layer_idx_++;
    return RootRaySource::FromDevice(DeviceRayBatch{});
  }

  // Dispatch transit_multi_ms_kernel: read cont_d/cont_w/cont_wl_idx, write
  // root_d/root_p/root_w/root_from_poly/root_rot/root_wl_idx into the
  // Impl-owned root buffers (d_dirs_/d_pos_/d_ws_/d_from_poly_/d_rot_c2w_).
  // wl_idx pass-through: d_cont_wl_idx_ is passed as both input and output
  // (aliases). __restrict__ has been removed from those two kernel params to
  // avoid C/CUDA UB (no-alias contract violation).
  const auto& ms_setting = impl_->scene_->ms_[impl_->ms_layer_idx_].setting_[0];
  lm_pcg::GenRootKernelParams gp =
      BuildTransitGpParams(ms_setting.crystal_.axis_, impl_->tri_cnt_, cont_n);
  gp.gen_seed     = impl_->transit_seed_;
  gp.gen_ray_base = NarrowPcgRayBase(impl_->transit_ray_count_, cont_n, "cuda-transit");

  uint32_t grid = (cont_n + 255u) / 256u;
  transit_multi_ms_kernel<<<grid, 256>>>(
      impl_->d_cont_d_, impl_->d_cont_w_, impl_->d_cont_wl_idx_,
      impl_->d_dirs_, impl_->d_pos_, impl_->d_ws_, impl_->d_from_poly_,
      impl_->d_rot_c2w_, impl_->d_root_wl_idx_,  // 296.6 DR-3: pass per-ray wl_idx from continuation into the
                                                 // next layer's root buffer (read by trace_single_ms_kernel)
      impl_->d_tri_vtx_, impl_->d_tri_norm_, impl_->d_tri_area_, impl_->d_tri_to_poly_,
      gp, cont_n);
  ck_reset(cudaPeekAtLastError(), "transit kernel launch");
  // Synchronize the default stream only — cudaDeviceSynchronize would block
  // all streams (including potential future overlapping streams) unnecessarily.
  ck_reset(cudaStreamSynchronize(cudaStreamDefault), "transit kernel sync");

  // Advance PCG counter + MS layer index. transit_ray_count_ uses the
  // continuation count (= number of kernel threads = number of unique PCG
  // global_idx values consumed).
  impl_->transit_ray_count_ += cont_n;
  impl_->ms_layer_idx_++;

  // Zero d_cont_count_ for the next layer's emit gate (the buffer is reused).
  ck_reset(cudaMemset(impl_->d_cont_count_, 0, sizeof(uint32_t)), "cudaMemset d_cont_count");

  if (impl_->logger != nullptr) {
    ILOG_DEBUG(*impl_->logger,
               "CudaTraceBackend::Recombine: cont_n={} new_layer={} transit_ray_count={}",
               cont_n, impl_->ms_layer_idx_, impl_->transit_ray_count_);
  }

  // backend_ptr cookie identifies "this is the Impl's root ring" — opaque to
  // the caller, used only as a non-null sanity tag by the next TraceLayer's
  // device-roots branch (the actual buffer addressing lives in Impl).
  DeviceRayBatch batch{};
  batch.backend_ptr = static_cast<void*>(impl_->d_dirs_);
  batch.count = cont_n;
  return RootRaySource::FromDevice(batch);
}

size_t CudaTraceBackend::ReadbackExitRays(std::vector<ExitRayRecord>& out) {
  return DrainExits(out);
}

size_t CudaTraceBackend::DrainExits(std::vector<ExitRayRecord>& out) {
  if (!impl_->in_session_) {
    throw BackendUnavailableError("CudaTraceBackend::DrainExits called outside session");
  }
  out.clear();
  if (impl_->h_exit_count_ == 0u) {
    return 0u;
  }
  auto t0 = std::chrono::steady_clock::now();
  size_t count = impl_->h_exit_count_;
  cudaError_t err_d2h =
      cudaMemcpy(impl_->pinned_exit_, impl_->d_exit_, count * sizeof(ExitRayRecord), cudaMemcpyDeviceToHost);
  if (err_d2h != cudaSuccess) {
    out.clear();
    impl_->Reset();
    throw BackendUnavailableError(std::string{"CudaTraceBackend::DrainExits: D2H cudaMemcpy: "} +
                                  cudaGetErrorString(err_d2h));
  }

  // Apply final-layer host filter + prob gate (296.5). Mirrors Metal
  // ReadbackExitRays:2447-2578: mid-layer exits (ms_layer_idx_idx != final)
  // already passed kernel-side filter+prob and are emitted verbatim; final-
  // layer records run FilterSpec::Check + a host RNG prob draw to enforce the
  // legacy CollectData per-layer semantics (simulator.cpp:425).
  const uint8_t final_layer = impl_->final_ms_layer_idx_;
  std::unique_ptr<FilterSpec> spec = FilterSpec::Create(
      impl_->final_layer_filter_config_, impl_->final_layer_crystal_, impl_->final_layer_axis_dist_);
  std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);

  out.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    const ExitRayRecord& src = impl_->pinned_exit_[i];
    if (src.ms_layer_idx != final_layer) {
      // Mid-layer exits already filtered+prob-gated in the kernel; emit verbatim.
      out.push_back(src);
      continue;
    }

    // Reconstruct {RaySeg, RaypathRecorder} for FilterSpec::Check.
    RaySeg r{};
    r.d_[0] = src.dir[0];
    r.d_[1] = src.dir[1];
    r.d_[2] = src.dir[2];
    r.w_ = src.weight;
    r.to_face_ = kInvalidId;
    r.is_continue_ = false;
    r.crystal_config_id_ = impl_->final_layer_crystal_.config_id_;

    // GetFn remap: raw kernel path[] carries poly-face indices; FilterSpec
    // expects post-GetFn ids (mirrors Metal ReadbackExitRays:2539-2545). For
    // paths longer than RaypathRecorder::kInlineCap a stack arena holds the
    // overflow tail so FilterSpec::Check can read past data_ safely
    // (task-284 fix).
    RaypathRecorder rec{};
    rec.Clear();
    uint8_t seq_len = static_cast<uint8_t>(std::min<size_t>(src.path.size_, kMaxHits));
    rec.size_ = seq_len;
    uint8_t local_arena[kMaxHits]{};
    const bool has_overflow = (seq_len > RaypathRecorder::kInlineCap);
    rec.overflow_idx_ = has_overflow ? 0u : RaypathRecorder::kNoOverflow;
    for (uint8_t k = 0; k < seq_len; ++k) {
      const uint8_t v = static_cast<uint8_t>(impl_->final_layer_crystal_.GetFn(static_cast<IdType>(src.path.data_[k])));
      if (k < RaypathRecorder::kInlineCap) {
        rec.data_[k] = v;
      }
      local_arena[k] = v;
    }
    const uint8_t* arena_for_check = has_overflow ? local_arena : nullptr;
    if (spec && !spec->Check(r, rec, arena_for_check)) {
      continue;  // filter-fail → drop (Design A termination on final layer)
    }
    // Legacy mirror: rng<prob means "would have continued"; since there is no
    // next layer, drop. final_ms_prob_=0 → never drops (drain_rng_ value
    // always >= 0). Mirrors Metal ReadbackExitRays:2556 / simulator.cpp:444.
    if (prob_dist(impl_->drain_rng_) < impl_->final_ms_prob_) {
      continue;
    }

    ExitRayRecord emitted = src;
    // Replace the raw-poly path with the GetFn-remapped face-number path so
    // downstream raw-XYZ / raypath consumers see the same canonical face ids
    // the legacy + Metal paths produce. Mirrors Metal `path_src` selection
    // (ReadbackExitRays:2570-2573).
    emitted.path.size_ = seq_len;
    const uint8_t* path_src = has_overflow ? local_arena : rec.data_;
    std::memcpy(emitted.path.data_, path_src, seq_len);
    out.push_back(emitted);
  }

  auto t1 = std::chrono::steady_clock::now();
  double drain_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  if (impl_->logger != nullptr) {
    ILOG_DEBUG(*impl_->logger,
               "CudaTraceBackend::DrainExits: read={} kept={} D2H+filter={:.2f}ms",
               count, out.size(), drain_ms);
  }

  // Reset counter for the next TraceLayer call.
  cudaMemset(impl_->d_exit_count_, 0, sizeof(uint32_t));
  impl_->h_exit_count_ = 0u;
  return out.size();
}

void CudaTraceBackend::EndSession() {
  if (!impl_->in_session_) {
    return;
  }
  cudaDeviceSynchronize();
  impl_->Reset();
}

uint32_t CudaTraceBackend::WlPoolSize() const {
  // Resolve from env on demand (mirrors MetalTraceBackend::WlPoolSize): the
  // driving loop queries this BEFORE BeginSession to pick the per-ray-wl path,
  // so it must answer M even when wl_pool_size_ has not been latched yet.
  if (impl_->wl_pool_size_ != 0u) {
    return impl_->wl_pool_size_;
  }
  Logger& logger = impl_->logger != nullptr ? *impl_->logger : GetGlobalLogger();
  return ResolveWlPoolSize(logger);
}

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
