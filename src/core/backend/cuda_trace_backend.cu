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
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <random>
#include <sstream>
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
#include "core/geo3d.hpp"  // Rotation (rectangular az0 derivation, mirrors Metal ComputeAz0)
#include "core/math.hpp"  // math::kDegreeToRad
#include "core/projection.hpp"  // ComputeEARScale (dual-fisheye r_scale derivation)
#include "core/shared/accum_shared.h"  // AccumXyzToPixel (S2 device-fused emit gate)
#include "core/shared/filter_shared.h"
#include "core/shared/optics_shared.h"
#include "core/shared/pcg_shared.h"
#include "core/shared/projection_shared.h"  // RectangularForward / FisheyeEqualAreaForward
#include "core/shared/traversal_shared.h"
#include "core/trace_ops.hpp"
#include "core/simulator.hpp"  // PartitionCrystalRayNum (multi-CI per-layer partition)
#include "util/logger.hpp"

namespace lumice {

namespace {

// sm_61 (Pascal, 2016) — the PTX floor emitted in the CUDA fatbin (see
// CMAKE_CUDA_ARCHITECTURES in CMakeLists.txt). A device below this cannot run
// our code: compute_61 PTX will not JIT *down* to an older SM. Such a device
// must be rejected at probe time and routed to the legacy CPU fallback, rather
// than discovered via a deep kernel-launch failure (cudaErrorNoKernelImage).
// Encoded as major*10 + minor to match cudaDeviceProp.
constexpr int kCudaCapabilityFloor = 61;

// Result of the one-shot device probe. Selects the first CUDA device whose
// compute capability meets the PTX floor, and captures a human-readable
// diagnostic line (device count, per-device name + capability, driver/runtime
// versions, verdict) so an unavailable/degraded path is debuggable from a
// single run — no second release cycle just to gather logs (task-282 lesson).
struct CudaProbe {
  bool eligible = false;
  int device = 0;
  std::string diagnostics;
};

// Probe cudaGetDeviceCount + per-device capability. All errors (driver not
// installed, no devices, runtime mismatch, all-devices-too-old) collapse to
// eligible=false; the simulator interprets that as "no CUDA backend available,
// fall back to legacy CPU".
CudaProbe RunCudaProbe() {
  CudaProbe probe;
  std::ostringstream os;

  int driver_ver = 0;
  int runtime_ver = 0;
  cudaDriverGetVersion(&driver_ver);
  cudaRuntimeGetVersion(&runtime_ver);
  (void)cudaGetLastError();

  int n = 0;
  const cudaError_t count_err = cudaGetDeviceCount(&n);
  if (count_err != cudaSuccess) {
    (void)cudaGetLastError();
    os << "no usable CUDA device (cudaGetDeviceCount: " << cudaGetErrorString(count_err)
       << "), driver=" << driver_ver << " runtime=" << runtime_ver << " -> legacy CPU fallback";
    probe.diagnostics = os.str();
    return probe;
  }

  os << n << " CUDA device(s), driver=" << driver_ver << " runtime=" << runtime_ver
     << ", floor=sm_" << kCudaCapabilityFloor << ": ";
  int selected = -1;
  for (int i = 0; i < n; ++i) {
    cudaDeviceProp prop{};
    if (cudaGetDeviceProperties(&prop, i) != cudaSuccess) {
      (void)cudaGetLastError();
      os << "[dev" << i << " properties-query-failed] ";
      continue;
    }
    const int cap = prop.major * 10 + prop.minor;
    const bool meets = cap >= kCudaCapabilityFloor;
    os << "[dev" << i << " '" << prop.name << "' sm_" << cap << (meets ? " ok" : " too-old") << "] ";
    if (meets && selected < 0) {
      selected = i;
    }
  }
  if (selected >= 0) {
    probe.eligible = true;
    probe.device = selected;
    os << "-> CUDA on dev" << selected;
  } else {
    os << "-> no device >= sm_" << kCudaCapabilityFloor << ", legacy CPU fallback";
  }
  probe.diagnostics = os.str();
  return probe;
}

// Cached once per process (function-local static init is thread-safe).
const CudaProbe& CudaProbeOnce() {
  static const CudaProbe probe = RunCudaProbe();
  return probe;
}

// Device index BeginSession must bind (cudaSetDevice) — the first eligible
// device chosen by the probe, not a hardcoded 0 (a user machine may have an
// old display GPU at index 0 and the compute card at index 1).
int CudaSelectedDevice() { return CudaProbeOnce().device; }

// Throw BackendUnavailableError if err != cudaSuccess. Clears the sticky CUDA
// error state so subsequent calls see a clean slate.
void CheckCuda(cudaError_t err, const char* ctx) {
  if (err != cudaSuccess) {
    std::string msg = std::string{"CudaTraceBackend: CUDA error at "} + ctx + ": " + cudaGetErrorString(err);
    (void)cudaGetLastError();
    throw BackendUnavailableError(std::move(msg));
  }
}

// scrum-306.2: CUDA's HasDeviceXyzAccum() is unconditionally true, so every exit
// is accumulated into d_xyz_buf_ via EmitToDeviceXyz — the trace kernel writes
// NOTHING to d_exit_ / d_exit_count_ (DrainExits always reads 0 records). The
// d_exit_/pinned_exit_ pool is therefore dead weight; sizing it to the analytic
// bound (n_roots × (2·max_hits+4), formerly computed by a since-removed
// ComputeExitCap helper) ballooned to GBs at large LUMICE_DISPATCH_RAY_NUM,
// causing the big-dispatch throughput collapse + parity OOM. Allocate a token
// slot so the kernel's (unused) d_exit pointer stays bindable. If
// HasDeviceXyzAccum ever goes false, restore that analytic-bound sizing (see
// ComputeContCap below for the still-live formula) AND the kernel's d_exit
// write path together.
constexpr size_t kCudaDeadExitCap = 256u;

// Continuation buffer capacity (296.4). Coefficient 2 = reflect+refract
// conservative upper bound per hit; +4 = safety margin matching the Metal
// EnsureExitBuffers constant. Each root may emit up to (max_hits * 2 + 4)
// candidate continuations — every outward exit is a continuation candidate.
//
// `n_roots * (max_hits*2 + 4)` is the ANALYTIC HARD upper bound on records a
// layer can emit (every root emits at most that many), so sizing to it means
// the kernel's `slot < cont_cap` append guard can never trip and no
// continuation is ever silently dropped. A former `std::min` with a 64 MiB cap
// (~762600 records, then shared with the now-removed exit-cap sizing) was a
// memory optimization, but it CORRUPTED output for high-continuation configs
// that exceed it even at the DEFAULT dispatch: scrum-296 Step D dig-out found
// ms_multi_crystal's layer-1 emitted ~954k > 762k → tail dropped → ds_corr vs
// legacy fell to 0.913, while total energy (1.02) and cross-seed self-corr
// (0.9998) both stayed clean and MASKED the bug (only corr-vs-legacy caught
// it). Correctness over memory: size to the true bound — a dropped
// continuation loses a downstream ray's entire sub-tree. A dispatch so large
// the buffer cannot be allocated fails via cudaMalloc → BackendUnavailableError
// → graceful legacy fallback — never silently-wrong output. Cont slots (~20B)
// are smaller than ExitRayRecord, so this pool's footprint stays below the
// (now-dead) exit pool's.
size_t ComputeContCap(size_t n_roots, size_t max_hits) {
  return n_roots * (max_hits * 2u + 4u);
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
// Device root-gen PCG nonce (296.6). gen_root_kernel seeds each ray on
// (gen_seed = spec.seed ^ kCudaGenNonce, gen_ray_base + tid); the nonce keeps the
// gen stream disjoint from transit/gate/drain so the same spec.seed produces
// independent draws across the four streams.
constexpr uint32_t kCudaGenNonce     = 0x3C9A7F11u;
// DrainExits host-side prob draw nonce (296.5). The final-layer drain runs its
// own RNG stream independent of the device-side gate/transit streams so a
// session that re-seeds (BeginSession) cleanly resets the drain sequence
// (mirrors Metal `impl_->rng` re-seeding semantics — see scrum-258 lesson on
// BeginSession RNG resets being filter parity's hidden third rail).
constexpr uint32_t kCudaDrainNonce   = 0xD5A1B3C7u;
// Continuation-pool shuffle PCG nonce (task-gpu-backend-recombine-shuffle).
// shuffle_cont_kernel decorrelates the per-parent-CI grouping the per-CI trace
// kernels leave in cont[written_slot] before the next layer's per-CI slicing
// consumes them (mirrors legacy host Fisher-Yates in simulator.cpp:946-950).
// Layer-level seed = shuffle_seed_ ^ ms_layer_idx_ keeps each layer's Feistel
// independent (Metal-side kMetalShuffleNonce takes the same value, symmetric).
constexpr uint32_t kCudaShuffleNonce = 0xB17CA3D9u;

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

// Build the gen-form GenRootKernelParams for the device root sampler
// (gen_root_kernel, 296.6). Reuses BuildTransitGpParams for the orientation +
// geometry fields (identical), then fills the sun cone + wl_pool_size that the
// gen kernel additionally needs. Mirrors Metal BuildGenRootParams
// (metal_trace_backend.mm:1413-1415 sun conversion). Caller fills gen_seed /
// gen_ray_base from the Impl gen_seed_ / gen_ray_count_ stream.
lm_pcg::GenRootKernelParams BuildGenGpParams(const AxisDistribution& axis_dist, uint32_t tri_count,
                                             uint32_t num_rays, const SunParam& sun, uint32_t wl_pool_size) {
  lm_pcg::GenRootKernelParams gp = BuildTransitGpParams(axis_dist, tri_count, num_rays);
  gp.sun_lon        = (sun.azimuth_ + 180.0f) * math::kDegreeToRad;
  gp.sun_lat        = -sun.altitude_ * math::kDegreeToRad;
  gp.sun_half_angle = (sun.diameter_ * 0.5f) * math::kDegreeToRad;
  gp.wl_pool_size   = wl_pool_size;
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

// S2 device-fused projection + XYZ accumulation for the ms_mode==0 emit gate.
// Mirrors the Metal final-layer `proj_type==0 / proj_type==1` blocks
// (lumice_trace.metal:743-811). `exit_world` is the world-space exit direction
// (sky direction is -exit_world); writes to `d_xyz_buf` go through the shared
// `AccumXyzToPixel` helper so single-source semantics with Metal stay tight.
// `d_landed_weight` tallies only in-bounds primary writes; overlap dual-write
// (dual-fisheye opposite-hemisphere) does NOT bump landed_weight (matches
// ScatterOutgoingToXyz Pass 2 / Metal lumice_trace.metal:788-810).
__device__ inline void EmitToDeviceXyz(float* __restrict__ d_xyz_buf,
                                       float* __restrict__ d_landed_weight,
                                       const float exit_world[3],
                                       float cmf_x,
                                       float cmf_y,
                                       float cmf_z,
                                       float w_emit,
                                       uint32_t proj_type,
                                       float az0,
                                       float r_scale,
                                       float max_abs_dz,
                                       uint32_t img_w,
                                       uint32_t img_h) {
  const float sx = -exit_world[0];
  const float sy = -exit_world[1];
  const float sz = -exit_world[2];
  const int iw_i = static_cast<int>(img_w);
  const int ih_i = static_cast<int>(img_h);
  const float img_w_f = static_cast<float>(img_w);
  const float img_h_f = static_cast<float>(img_h);
  if (proj_type == 0u) {
    auto proj_r = lm_proj::RectangularForward(sx, sy, sz);
    float lon = proj_r.x - az0;
    const float two_pi = 2.0f * LM_PI_F;
    lon = lon - two_pi * floorf((lon + LM_PI_F) / two_pi);
    const float lat = proj_r.y;
    const float short_res = static_cast<float>(min(img_w / 2u, img_h));
    const float proj_scl = short_res / LM_PI_F;
    int raw_x = static_cast<int>(floorf(lon * proj_scl + img_w_f * 0.5f + 0.5f));
    const int ix = ((raw_x % iw_i) + iw_i) % iw_i;
    const int iy = static_cast<int>(floorf(-lat * proj_scl + img_h_f * 0.5f + 0.5f));
    if (iy >= 0 && iy < ih_i) {
      const uint32_t pix_flat = static_cast<uint32_t>(iy) * img_w + static_cast<uint32_t>(ix);
      AccumXyzToPixel(d_xyz_buf, pix_flat, cmf_x, cmf_y, cmf_z, w_emit);
      atomicAdd(d_landed_weight, w_emit);
    }
  } else {
    // Dual-fisheye equal-area: primary + opposite-hemisphere overlap.
    const int   dual_short = min(static_cast<int>(img_w) / 2, static_cast<int>(img_h));
    const float r          = static_cast<float>(dual_short) * 0.5f;
    const float cy_pix     = img_h_f * 0.5f;
    const float cx_left    = img_w_f * 0.5f - r;
    const float cx_right   = img_w_f * 0.5f + r;
    const bool  is_upper = (sz >= 0.0f);
    const float z_hemi   = is_upper ? sz : -sz;
    auto ea_r = lm_proj::FisheyeEqualAreaForward(sx, sy, z_hemi, r_scale);
    const float xn = ea_r.x;
    const float yn = ea_r.y;
    const float cx_primary = is_upper ? cx_left : cx_right;
    const float sx_primary = is_upper ? -1.0f : 1.0f;
    const float fx = sx_primary * yn * r + cx_primary;
    const float fy = xn * r + cy_pix;
    const int   ix = static_cast<int>(floorf(fx + 0.5f));
    const int   iy = static_cast<int>(floorf(fy + 0.5f));
    if (ix >= 0 && ix < iw_i && iy >= 0 && iy < ih_i) {
      const uint32_t pix_flat = static_cast<uint32_t>(iy) * img_w + static_cast<uint32_t>(ix);
      AccumXyzToPixel(d_xyz_buf, pix_flat, cmf_x, cmf_y, cmf_z, w_emit);
      atomicAdd(d_landed_weight, w_emit);
    }
    // Overlap dual-write: opposite hemisphere (dz<0). landed_weight NOT bumped
    // (parity with ScatterOutgoingToXyz Pass 2 / Metal:788-810).
    if (max_abs_dz > 0.0f && z_hemi < max_abs_dz) {
      const float z_opp = -z_hemi;
      auto ea_r2 = lm_proj::FisheyeEqualAreaForward(sx, sy, z_opp, r_scale);
      const float xn2 = ea_r2.x;
      const float yn2 = ea_r2.y;
      const bool  is_upper_opp = !is_upper;
      const float cx_opp       = is_upper_opp ? cx_left : cx_right;
      const float sx_opp       = is_upper_opp ? -1.0f : 1.0f;
      const float fx2 = sx_opp * yn2 * r + cx_opp;
      const float fy2 = xn2 * r + cy_pix;
      const int   ix2 = static_cast<int>(floorf(fx2 + 0.5f));
      const int   iy2 = static_cast<int>(floorf(fy2 + 0.5f));
      if (ix2 >= 0 && ix2 < iw_i && iy2 >= 0 && iy2 < ih_i) {
        const uint32_t pix_flat2 =
            static_cast<uint32_t>(iy2) * img_w + static_cast<uint32_t>(ix2);
        AccumXyzToPixel(d_xyz_buf, pix_flat2, cmf_x, cmf_y, cmf_z, w_emit);
      }
    }
  }
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
                                       uint32_t crystal_config_id,
                                       // S2 device-fused accumulation (ms_mode==0 final-layer emit).
                                       // d_xyz_buf may be nullptr only when ms_mode==1 (the ms_mode==0
                                       // branches below dereference it; ms_mode==1 branches don't).
                                       float* __restrict__ d_xyz_buf,
                                       float* __restrict__ d_landed_weight,
                                       uint32_t proj_type,
                                       float az0,
                                       float r_scale,
                                       float max_abs_dz,
                                       uint32_t img_w,
                                       uint32_t img_h,
                                       float last_ms_prob,
                                       uint32_t gate_seed_final,
                                       uint32_t gate_ray_base_final) {
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
        const uint32_t gate_slot = static_cast<uint32_t>(ms_layer_idx) * filter_desc_max_ci +
                                   static_cast<uint32_t>(crystal_id);
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
            // scrum-302 S2 device-fused MID-EXIT (filter-pass && !do_continue).
            // Project + accumulate into the device XYZ buffer, mirroring Metal
            // mid-exit (lumice_trace.metal:635-702). NO further prob draw —
            // !do_continue already IS the mid-exit outcome. Previously this wrote
            // an ExitRayRecord into d_exit, which HasDeviceXyzAccum()==true makes
            // the simulator discard (exit_records dropped) → every mid-layer's
            // exits were silently lost → energy = 1/(layers) deficit.
            const float cmf_x = d_wl_pool[wl_idx].cmf_x;
            const float cmf_y = d_wl_pool[wl_idx].cmf_y;
            const float cmf_z = d_wl_pool[wl_idx].cmf_z;
            EmitToDeviceXyz(d_xyz_buf, d_landed_weight, exit_world,
                            cmf_x, cmf_y, cmf_z, w_refl_e,
                            proj_type, az0, r_scale, max_abs_dz, img_w, img_h);
          }
        }
        // filter_pass == false: implicit drop (Design A termination).
      } else {
        // ms_mode==0 (S2 device-fused): filter → final-layer prob draw →
        // project → AccumXyzToPixel. Entry-face external reflect path. PCG
        // slot=1 keeps this draw disjoint from the per-bounce refract emit's
        // slot=0 below (same ray, same gate stream — multiple draws need
        // distinct slot ids).
        const uint32_t gate_slot_e = static_cast<uint32_t>(ms_layer_idx) * filter_desc_max_ci +
                                     static_cast<uint32_t>(crystal_id);
        const bool filter_pass_e = lm_filter::DeviceFilterCheck(
            d_filter_desc[gate_slot_e], d_complex_sub_desc, path_rec, rec_len, d_getfn_bytes, d_getfn_offsets,
            gate_slot_e, exit_world, crystal_config_id);
        if (filter_pass_e) {
          lm_pcg::PcgStream gate_f;
          gate_f.seed       = gate_seed_final;
          gate_f.global_idx = gate_ray_base_final + tid;
          gate_f.slot       = 1u;  // entry external reflect emit
          // Mirrors S1 Bug 3 fix: rng < last_ms_prob ⇒ "would continue" ⇒
          // there is no next layer ⇒ drop. Only the (1 - last_ms_prob)
          // remainder reaches the image.
          const bool prob_drop_e = (lm_pcg::pcg_uniform(gate_f) < last_ms_prob);
          if (!prob_drop_e) {
            const float cmf_x = d_wl_pool[wl_idx].cmf_x;
            const float cmf_y = d_wl_pool[wl_idx].cmf_y;
            const float cmf_z = d_wl_pool[wl_idx].cmf_z;
            EmitToDeviceXyz(d_xyz_buf, d_landed_weight, exit_world,
                            cmf_x, cmf_y, cmf_z, w_refl_e,
                            proj_type, az0, r_scale, max_abs_dz, img_w, img_h);
          }
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
          const uint32_t gate_slot = static_cast<uint32_t>(ms_layer_idx) * filter_desc_max_ci +
                                   static_cast<uint32_t>(crystal_id);
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
              // scrum-302 S2 device-fused MID-EXIT (per-bounce refracted,
              // filter-pass && !do_continue). Project + accumulate, mirror Metal
              // mid-exit (lumice_trace.metal:635-702). No further prob draw.
              // Was: ExitRayRecord write → discarded under HasDeviceXyzAccum →
              // mid-layer energy loss (see entry-face branch above).
              const float cmf_x = d_wl_pool[wl_idx].cmf_x;
              const float cmf_y = d_wl_pool[wl_idx].cmf_y;
              const float cmf_z = d_wl_pool[wl_idx].cmf_z;
              EmitToDeviceXyz(d_xyz_buf, d_landed_weight, exit_world,
                              cmf_x, cmf_y, cmf_z, w_refr,
                              proj_type, az0, r_scale, max_abs_dz, img_w, img_h);
            }
          }
          // filter_pass == false: implicit drop (Design A termination).
        } else {
          // ms_mode==0 (S2 device-fused) per-bounce refracted exit emit.
          // Mirrors Metal lumice_trace.metal:714-815 (the ms_mode==0 emit
          // gate). PCG slot=0 is reserved for this path; the entry-face
          // external reflect uses slot=1 above.
          const uint32_t gate_slot_r = static_cast<uint32_t>(ms_layer_idx) * filter_desc_max_ci +
                                       static_cast<uint32_t>(crystal_id);
          const bool filter_pass_r = lm_filter::DeviceFilterCheck(
              d_filter_desc[gate_slot_r], d_complex_sub_desc, path_rec, rec_len, d_getfn_bytes, d_getfn_offsets,
              gate_slot_r, exit_world, crystal_config_id);
          if (filter_pass_r) {
            lm_pcg::PcgStream gate_f;
            gate_f.seed       = gate_seed_final;
            gate_f.global_idx = gate_ray_base_final + tid;
            gate_f.slot       = 0u;  // per-bounce refract emit
            const bool prob_drop_r = (lm_pcg::pcg_uniform(gate_f) < last_ms_prob);
            if (!prob_drop_r) {
              const float cmf_x = d_wl_pool[wl_idx].cmf_x;
              const float cmf_y = d_wl_pool[wl_idx].cmf_y;
              const float cmf_z = d_wl_pool[wl_idx].cmf_z;
              EmitToDeviceXyz(d_xyz_buf, d_landed_weight, exit_world,
                              cmf_x, cmf_y, cmf_z, w_refr,
                              proj_type, az0, r_scale, max_abs_dz, img_w, img_h);
            }
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

// --- gen_root_kernel -------------------------------------------------------
//
// Device-side first-layer root generation (296.6). Replaces the host
// InitRayFirstMs + 5-way H2D upload: each thread samples a fresh crystal
// orientation, an incident direction inside the sun cone, an entry triangle +
// point, and a per-ray wl_idx — writing the same root buffers the trace kernel
// reads, all device-resident (no host pointers cross the seam, §5 铁律).
// Mirrors Metal `gen_root_kernel` (lumice_trace.metal:844-942) form-for-form and
// shares the orientation / cap / triangle samplers via `lm_pcg::*`, so the two
// GPU backends cannot drift on the PCG hash or the sample logic. The transit
// kernel above is the per-layer sibling (incident dir from the continuation
// buffer instead of the sun; weight carried instead of pool-sampled).
__global__ void gen_root_kernel(float* __restrict__ d_root_d,           // 3 × n_rays (crystal-local)
                                float* __restrict__ d_root_p,           // 3 × n_rays (crystal-local entry point)
                                float* __restrict__ d_root_w,           // n_rays (spd weight)
                                uint32_t* __restrict__ d_root_from_poly,  // n_rays (entry-face poly id)
                                float* __restrict__ d_root_rot,         // 9 × n_rays (crystal→world)
                                uint32_t* __restrict__ d_root_wl_idx,   // n_rays (per-ray wl index)
                                const float* __restrict__ d_tri_vtx,    // 9 × tri_cnt
                                const float* __restrict__ d_tri_norm,   // 3 × tri_cnt
                                const float* __restrict__ d_tri_area,   // tri_cnt
                                const uint16_t* __restrict__ d_tri_to_poly,  // tri_cnt
                                const WlEntry* __restrict__ d_wl_pool,  // wl_pool_size entries
                                lm_pcg::GenRootKernelParams gp,
                                uint32_t n_rays) {
  const uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= n_rays || gp.tri_count == 0u) {
    return;
  }
  const uint32_t global_idx = gp.gen_ray_base + tid;

  // Per-ray wavelength (slot 20, isolated from the orientation/triangle draws
  // below — mirrors MSL gen_root_kernel:881-895). The wl_idx is the photon's
  // lifetime tag: it selects spd_weight here and n_idx in the trace kernel.
  lm_pcg::PcgStream wl_stream;
  wl_stream.seed       = gp.gen_seed;
  wl_stream.global_idx = global_idx;
  wl_stream.slot       = 20u;
  uint32_t wl_idx = static_cast<uint32_t>(lm_pcg::pcg_uniform(wl_stream) * static_cast<float>(gp.wl_pool_size));
  if (wl_idx >= gp.wl_pool_size) {
    wl_idx = gp.wl_pool_size - 1u;  // guard pcg_uniform → 1.0 rounding
  }
  d_root_wl_idx[tid] = wl_idx;

  // 1. Crystal orientation → rotation matrix.
  lm_pcg::PcgStream stream;
  stream.seed       = gp.gen_seed;
  stream.global_idx = global_idx;
  stream.slot       = 0u;
  float lon = 0.0f;
  float lat = 0.0f;
  float roll = 0.0f;
  lm_pcg::sample_lat_lon_roll(stream, gp, lon, lat, roll);
  float mat9[9];
  lm_pcg::build_crystal_rotation_9(lon, lat, roll, mat9);

  // 2. Incident direction in the sun cone (WORLD space) → crystal-local.
  float d_world[3];
  lm_pcg::sample_sph_cap(stream, gp.sun_lon, gp.sun_lat, gp.sun_half_angle, d_world);
  float d_crystal[3];
  lm_pcg::apply_inverse_mat9(mat9, d_world, d_crystal);

  // 3. Triangle area × facing weighted pick → uniform point on the chosen tri.
  float proj_prob[lm_pcg::kMaxTriPerKernel];
  uint32_t n_tri = gp.tri_count;
  if (n_tri > lm_pcg::kMaxTriPerKernel) {
    n_tri = lm_pcg::kMaxTriPerKernel;  // defensive; host already throw'd otherwise
  }
  for (uint32_t t = 0u; t < n_tri; ++t) {
    float dot = d_crystal[0] * d_tri_norm[t * 3u + 0u] + d_crystal[1] * d_tri_norm[t * 3u + 1u] +
                d_crystal[2] * d_tri_norm[t * 3u + 2u];
    proj_prob[t] = fmaxf(-dot * d_tri_area[t], 0.0f);
  }
  float u_cat = lm_pcg::pcg_uniform(stream);
  uint32_t tri_id = lm_pcg::categorical_sample(proj_prob, n_tri, u_cat);
  float p[3];
  lm_pcg::sample_triangle(stream, d_tri_vtx + tri_id * 9u, p);

  // 4. tri_to_poly. kInvalidId → zero-weight drop (InitRay_p_fid fallback).
  uint16_t to_face_u16 = d_tri_to_poly[tri_id];
  float weight = d_wl_pool[wl_idx].spd_weight;
  constexpr uint16_t kInvalidIdU16 = 0xffffu;
  uint32_t to_face_u32;
  if (to_face_u16 == kInvalidIdU16) {
    weight = 0.0f;
    to_face_u32 = 0xFFFFFFFFu;
  } else {
    to_face_u32 = static_cast<uint32_t>(to_face_u16);
  }

  // 5. Emit root buffers (crystal-local d/p, crystal→world rot for invariant-6).
  d_root_d[tid * 3u + 0u] = d_crystal[0];
  d_root_d[tid * 3u + 1u] = d_crystal[1];
  d_root_d[tid * 3u + 2u] = d_crystal[2];
  d_root_p[tid * 3u + 0u] = p[0];
  d_root_p[tid * 3u + 1u] = p[1];
  d_root_p[tid * 3u + 2u] = p[2];
  d_root_w[tid] = weight;
  d_root_from_poly[tid] = to_face_u32;
  for (uint32_t k = 0u; k < 9u; ++k) {
    d_root_rot[tid * 9u + k] = mat9[k];
  }
}

// --- shuffle_cont_kernel ---------------------------------------------------
//
// Continuation-pool decorrelation shuffle (task-gpu-backend-recombine-shuffle).
// Gather-form Feistel permutation: each thread tid in [0, n) computes
// `src = feistel_bijection(tid, n, seed)` (lm_pcg, shared with MSL) and copies
// (d, w, wl_idx) from in[src] to out[tid]. The dest must NOT alias the source,
// so the caller passes cont[other_slot] as the dest and swaps the slot pointers
// afterwards (see Recombine).
//
// Why: when a multi-CI layer's per-CI trace_single_ms_kernel dispatches run on
// the default stream, cont[written_slot] ends up grouped by parent CI. The next
// layer's per-CI slicing then hands "parent-correlated" subsets to each child
// CI, biasing the ray→crystal pairing. Legacy host Fisher-Yates breaks this
// (simulator.cpp:946-950); explore-300 confirmed the GPU backends were missing
// the same step.
__global__ void shuffle_cont_kernel(const float* __restrict__    in_d,
                                    const float* __restrict__    in_w,
                                    const uint32_t* __restrict__ in_wl,
                                    float* __restrict__          out_d,
                                    float* __restrict__          out_w,
                                    uint32_t* __restrict__       out_wl,
                                    uint32_t n_rays, uint32_t seed) {
  const uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= n_rays) {
    return;
  }
  const uint32_t src = lm_pcg::feistel_bijection(tid, n_rays, seed);
  out_d[tid * 3u + 0u] = in_d[src * 3u + 0u];
  out_d[tid * 3u + 1u] = in_d[src * 3u + 1u];
  out_d[tid * 3u + 2u] = in_d[src * 3u + 2u];
  out_w[tid]  = in_w[src];
  out_wl[tid] = in_wl[src];
}

}  // namespace

bool CudaDeviceAvailable() { return CudaProbeOnce().eligible; }

std::string CudaDeviceDiagnostics() { return CudaProbeOnce().diagnostics; }

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

  // Grow-only physical capacities for the per-CI geometry re-upload path
  // (UploadCrystalGeometry). Multi-CI mirrors Metal UploadCrystal: each ci in a
  // layer re-uploads its own crystal's geometry into the SAME device buffers
  // before its trace/transit dispatch (ci dispatches are serialized, so reuse is
  // safe). poly_cnt_/tri_cnt_ track the currently-uploaded crystal's sizes;
  // alloc_*_cap_ track the physically-allocated buffer sizes so a smaller ci
  // does not realloc. Mirrors Metal EnsurePolyBuffers/EnsureTriBuffers.
  uint32_t  alloc_poly_cap_ = 0;
  uint32_t  alloc_tri_cap_  = 0;

  // --- scrum-306.2 geometry pool (device-gen path) -------------------------
  // Per-(layer,ci) crystal geometry uploaded ONCE per scene and PERSISTED
  // across the per-batch BeginSession/EndSession cycle (geom_pool_built_ guard).
  // Replaces the per-batch MakeCrystal + 6×blocking-H2D re-upload of IDENTICAL
  // geometry: rng_ is reset to the constant effective_seed_ every BeginSession
  // (worker_count=1 GPU route) → MakeCrystal yields the same shapes every batch,
  // so the prior path re-uploaded the same bytes per batch — the dominant
  // per-dispatch host-sync memcpy (explore-async E2). K=1 shape per (layer,ci):
  // PARITY-EXACT with the prior per-batch path (current path already traces one
  // shape/(layer,ci)/session); the §5 per-ray K-shape pool is a later
  // statistical refinement, not needed for parity. Buffers concatenate all
  // slots; pool_*_off_/pool_*_cnt_ index each slot (slot = layer_slot_base_[mi]+ci).
  float*    d_pool_poly_n_      = nullptr;  // 3 × Σ poly_cnt
  float*    d_pool_poly_d_      = nullptr;  //     Σ poly_cnt
  float*    d_pool_tri_vtx_     = nullptr;  // 9 × Σ tri_cnt
  float*    d_pool_tri_norm_    = nullptr;  // 3 × Σ tri_cnt
  float*    d_pool_tri_area_    = nullptr;  //     Σ tri_cnt
  uint16_t* d_pool_tri_to_poly_ = nullptr;  //     Σ tri_cnt
  std::vector<uint32_t> pool_poly_off_;
  std::vector<uint32_t> pool_poly_cnt_;
  std::vector<uint32_t> pool_tri_off_;
  std::vector<uint32_t> pool_tri_cnt_;
  std::vector<uint32_t> layer_slot_base_;   // ms layer mi → first slot index
  std::vector<Crystal>  pool_crystals_;     // host slot crystals (config_id; wl-pool repr)
  bool        geom_pool_built_ = false;
  const void* pool_scene_      = nullptr;   // scene the pool was built for (rebuild guard)
  // scrum-306.2 increment 4: filter descriptors + wl pool are per-session-CONSTANT
  // (config + crystal n_idx fixed across batches) — persist them across the
  // per-batch cycle instead of free+malloc+H2D every BeginSession (post-pool the
  // dominant host-API cost: cudaFree 40% + cudaMemcpy 43%, nsys 4M-run). Rebuilt
  // only on scene change (pool_scene_ sentinel) or full teardown.
  bool        filter_built_      = false;
  bool        wl_pool_uploaded_  = false;

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

  // --- Continuation ring (296.4 + multi-CI 全量) ---------------------------
  // PING-PONG (2 slots), mirroring Metal cont_d/cont_w[slot]. Multi-CI moved
  // transit INTO TraceLayer (lazy, per-CI), so a continuation layer READS the
  // previous layer's continuations from slot in=(ms_layer_idx_-1)&1 (slice per
  // ci) while WRITING this layer's continuations to slot out=ms_layer_idx_&1.
  // A single buffer would be a read-after-write hazard across the ci loop. Each
  // slot is written by trace_single_ms_kernel (ms_mode==1) and read, per-ci-
  // slice, by transit_multi_ms_kernel in the NEXT layer's TraceLayer.
  float*    d_cont_d_[2]      = {nullptr, nullptr};  // 3 × cont_cap (world-space dir)
  float*    d_cont_w_[2]      = {nullptr, nullptr};  // cont_cap (continuation weight)
  uint32_t* d_cont_wl_idx_[2] = {nullptr, nullptr};  // cont_cap (per-ray wl pass-through)
  uint32_t* d_cont_count_[2]  = {nullptr, nullptr};  // 1 uint32 each (atomic, device)
  // Per-slot continuation capacity (multi-CI 3+ layer): a continuation layer
  // grows ONLY its out_slot (the in_slot holds rays being read this layer and
  // must not realloc). Each slot grows lazily when it becomes the out_slot, so
  // the two slots may differ in size. root_cap_ tracks the root/rotation buffers
  // (d_dirs_/d_pos_/d_ws_/d_from_poly_/d_root_wl_idx_/d_rot_c2w_) which hold this
  // layer's per-ci gen/transit output (≤ n_in roots).
  size_t    cont_cap_[2]    = {0, 0};
  size_t    root_cap_       = 0;
  uint32_t  h_cont_count_   = 0;         // host snapshot after 4B D2H readback
  uint32_t* pinned_cont_count_ = nullptr;  // 1 uint32 pinned host (D2H staging)

  // Per-(layer) per-ci ray-count partition carry (largest-remainder fractional
  // remainder), persistent across batches within a session. Indexed by MS layer;
  // each entry sized to that layer's crystal_cnt. Mirrors simulator.cpp:830
  // ray_alloc_carry[mi]. Cleared per BeginSession.
  std::vector<std::vector<double>> ray_alloc_carry_;

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

  // scrum-306.2 async-stream port (increment 1): all per-dispatch GPU work
  // (kernels / memcpy / memset / events) runs on this dedicated non-default
  // stream instead of the legacy default stream. Increment 1 keeps host-blocking
  // at the SAME logical points (every former cudaDeviceSynchronize / blocking
  // cudaMemcpy → cudaStreamSynchronize(stream_)), so semantics are preserved and
  // parity must stay 10/10; it only establishes the stream infrastructure.
  // Increment 2 then defers the count-readback waits to overlap D2H with compute.
  // Created once (gated by stream_created_), destroyed in full Reset() / dtor —
  // persists across the per-batch keep_persistent_buffers path like the events.
  cudaStream_t stream_{};
  bool stream_created_ = false;

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

  // --- Device root-gen PCG state (296.6) -----------------------------------
  // gen_root_kernel's stream. Same monotone-counter discipline as transit_/gate_:
  // gen_ray_count_ advances by n per first-layer dispatch and is NOT reset across
  // batches (gen_seeded_ guards the one-time reset), so (gen_seed_, gen_ray_count_
  // + tid) stays disjoint per (batch, tid) — re-using global_idx collapses crystal
  // orientation sampling across batches (scrum-267.3 lesson). disable_device_gen_
  // (LUMICE_DISABLE_DEVICE_GEN) forces the host InitRayFirstMs fallback.
  uint32_t gen_seed_      = 0u;
  size_t   gen_ray_count_ = 0;
  bool     gen_seeded_    = false;
  bool     disable_device_gen_ = false;

  // --- Continuation-pool shuffle PCG state (task-gpu-backend-recombine-shuffle)
  // shuffle_cont_kernel uses (shuffle_seed_ ^ ms_layer_idx_) as the per-layer
  // Feistel seed; no monotone counter (the Feistel is keyed once per layer, not
  // per ray, and each layer's seed is naturally disjoint).
  uint32_t shuffle_seed_ = 0u;

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

  // --- S2 device-fused XYZ accumulation -----------------------------------
  // ms_mode==0 emit gate accumulates per-ray (cmf_x/y/z * weight) directly into
  // a device-resident W*H*3 float buffer via atomicAdd, replacing the per-exit
  // PCIe round-trip (`DrainExits` + host projection). scrum-312 third clock: the
  // buffer PERSISTS across per-batch sessions (allocated once, zeroed on alloc);
  // BeginSession no longer zeroes it. `ReadbackXyzAccum` D2H copies it to the host
  // and zeros it, but the simulator now drains on display cadence (a whole window
  // of batches), not per batch.
  float*   d_xyz_buf_       = nullptr;  // alloc_xyz_w_ * alloc_xyz_h_ * 3 floats, atomicAdd target
  float*   d_landed_weight_ = nullptr;  // 1 float, atomicAdd target (running total)
  // scrum-312: dims the persistent d_xyz_buf_ was actually allocated for. Unlike
  // img_w_/img_h_ (per-session, cleared by Reset), these survive across sessions
  // so ReadbackXyzAccum — which drains BETWEEN sessions — can release-safe-verify
  // the caller's dims against the real buffer capacity (guards the Bug-1 class:
  // dims decoupled from the buffer). Zeroed only on full teardown (buffer freed).
  uint32_t alloc_xyz_w_     = 0u;
  uint32_t alloc_xyz_h_     = 0u;
  uint32_t proj_type_       = 0u;       // 0=rectangular, 1=dual_fisheye_equal_area
  float    az0_             = 0.0f;     // rectangular: view azimuth offset (radians)
  float    r_scale_         = 1.0f;     // dual_fisheye: equal-area r scale
  float    max_abs_dz_      = 0.0f;     // dual_fisheye: overlap zone |sky.z| threshold
  uint32_t img_w_           = 0u;
  uint32_t img_h_           = 0u;

  // --- Final-layer host filter (296.5) -------------------------------------
  // DrainExits applies FilterSpec::Check + prob to records tagged with the
  // final ms_layer_idx (mirrors Metal ReadbackExitRays:2447-2578). Captured in
  // BeginSession from the last ms_setting; single-CI MVP, multi-CI is 296.6.
  // Multi-CI 全量: the final layer may hold several crystals; DrainExits indexes
  // these per-exit by ExitRayRecord.crystal_id. Populated in BeginSession from
  // the last MS layer's settings (proto crystals — filter only needs topology).
  // Mirrors Metal last_layer_crystals_[ci] (metal_trace_backend.mm:2403-2409).
  std::vector<FilterConfig>     final_layer_filter_configs_;
  std::vector<Crystal>          final_layer_crystals_;
  std::vector<AxisDistribution> final_layer_axis_dists_;
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

  // keep_persistent_buffers=true (per-batch EndSession) preserves the large
  // device + pinned buffers across sessions to avoid per-batch alloc churn;
  // false (error paths + destructor) is full teardown. See definition.
  void Reset(bool keep_persistent_buffers = false);
  void EnsureSessionBuffers(size_t n);
  void EnsureFilterBuffers(const SessionSpec& spec);
  // Grow ONLY the exit buffer for a device-roots continuation layer, leaving the
  // root buffers (d_dirs_/d_pos_/d_ws_/d_from_poly_/d_rot_c2w_) untouched so the
  // transit_multi_ms_kernel's device-resident continuation rays survive into the
  // next TraceLayer. EnsureSessionBuffers must NOT run on the device-roots path:
  // its n_roots_!=n realloc would free those buffers mid-flight (296.4 root-cause).
  void EnsureExitCapacity(size_t n);

  // Continuation-layer (3+ MS) capacity growth (grow-only). Grows the exit pool,
  // the root/rotation buffers (to hold n_in transit outputs), and ONLY the given
  // out_slot continuation buffer (to hold this layer's fan-out). The in_slot is
  // left intact — it holds the previous layer's continuations being read this
  // layer. Mirrors Metal EnsureRootBuffers(total)+EnsureContBuffer(out_slot).
  void EnsureContCapacity(size_t n_in, int out_slot);

  // Per-CI geometry (multi-CI, mirrors Metal UploadCrystal/Ensure*Buffers).
  // EnsureGeomCapacity grows d_poly_*/d_tri_* (grow-only) to hold a crystal of
  // the given size; UploadCrystalGeometry resizes + H2D-copies one crystal's
  // polygon/triangle pool (incl. the tri_to_poly argmax map) and updates
  // poly_cnt_/tri_cnt_. Call once per (layer,ci) before that ci's trace/transit.
  void EnsureGeomCapacity(size_t poly_cnt, size_t tri_cnt);
  void UploadCrystalGeometry(const Crystal& crystal);

  // scrum-306.2: build the per-(layer,ci) geometry pool once (device-gen path).
  // Consumes rng_ via MakeCrystal in the EXACT ci-loop order (layers outer, ci
  // inner) so pooled shapes are byte-identical to the prior per-batch upload.
  void BuildGeomPool(const SceneConfig& scene);
};

void CudaTraceBackend::Impl::Reset(bool keep_persistent_buffers) {
  // cudaFree / cudaFreeHost on nullptr are no-ops per CUDA spec, so we don't
  // need explicit allocated-flag guards. Errors are intentionally ignored —
  // teardown must not throw.
  //
  // scrum-cuda-async-engine-port (304.2): per-batch EndSession passes
  // keep_persistent_buffers=true so the large device + pinned buffers
  // (ray/cont/exit/geometry/xyz/wl-pool) and their capacity trackers PERSIST
  // across sessions, reused via the idempotent EnsureSessionBuffers fast-path
  // (buffers_allocated_ && n_roots_ == n) + grow-only EnsureGeomCapacity. This
  // mirrors Metal's Reset() which never frees MTLBuffers between sessions
  // (metal_trace_backend.mm:1807) and eliminates the per-batch cudaMalloc +
  // cudaHostAlloc churn that pinned CUDA throughput to GPU ~1-2% utilization
  // (root cause: explore-303). Full teardown (keep_persistent_buffers=false)
  // runs on error paths + the destructor.

  // scrum-306.2 increment 4: filter descriptors are now PERSISTED across the
  // per-batch keep path (EnsureFilterBuffers is idempotent on filter_built_), so
  // they are freed only on full teardown below — not every session end.

  if (!keep_persistent_buffers) {
    cudaFree(d_poly_n_);     d_poly_n_ = nullptr;
    cudaFree(d_poly_d_);     d_poly_d_ = nullptr;
    cudaFree(d_tri_vtx_);    d_tri_vtx_ = nullptr;
    cudaFree(d_tri_norm_);   d_tri_norm_ = nullptr;
    cudaFree(d_tri_area_);   d_tri_area_ = nullptr;
    cudaFree(d_tri_to_poly_); d_tri_to_poly_ = nullptr;
    // scrum-306.2 geometry pool — freed only on full teardown (persists across
    // the per-batch keep_persistent path so it is uploaded once per scene).
    cudaFree(d_pool_poly_n_);      d_pool_poly_n_ = nullptr;
    cudaFree(d_pool_poly_d_);      d_pool_poly_d_ = nullptr;
    cudaFree(d_pool_tri_vtx_);     d_pool_tri_vtx_ = nullptr;
    cudaFree(d_pool_tri_norm_);    d_pool_tri_norm_ = nullptr;
    cudaFree(d_pool_tri_area_);    d_pool_tri_area_ = nullptr;
    cudaFree(d_pool_tri_to_poly_); d_pool_tri_to_poly_ = nullptr;
    pool_poly_off_.clear(); pool_poly_cnt_.clear();
    pool_tri_off_.clear();  pool_tri_cnt_.clear();
    layer_slot_base_.clear(); pool_crystals_.clear();
    geom_pool_built_ = false;
    pool_scene_ = nullptr;
    // increment 4: filter descriptors + wl pool persist across the keep path;
    // free + invalidate them only here on full teardown (d_wl_pool_ freed below).
    cudaFree(d_filter_desc_);      d_filter_desc_ = nullptr;
    cudaFree(d_getfn_offsets_);    d_getfn_offsets_ = nullptr;
    cudaFree(d_getfn_bytes_);      d_getfn_bytes_ = nullptr;
    cudaFree(d_complex_sub_desc_); d_complex_sub_desc_ = nullptr;
    filter_built_ = false;
    wl_pool_uploaded_ = false;
    filter_desc_max_ci_ = 0u;
    filter_n_slot_ = 0u;
    crystal_config_id_ = 0xFFFFu;
    final_layer_filter_configs_.clear();
    final_layer_crystals_.clear();
    final_layer_axis_dists_.clear();
    final_ms_layer_idx_ = 0u;
    final_ms_prob_ = 0.0f;
    // Grow-only geometry capacities track the freed buffers above; zero them so
    // a fresh session re-allocates rather than reusing a dangling capacity.
    alloc_poly_cap_ = 0;
    alloc_tri_cap_ = 0;
    cudaFree(d_rot_c2w_);      d_rot_c2w_ = nullptr;
    cudaFree(d_dirs_);         d_dirs_ = nullptr;
    cudaFree(d_pos_);          d_pos_ = nullptr;
    cudaFree(d_ws_);           d_ws_ = nullptr;
    cudaFree(d_from_poly_);    d_from_poly_ = nullptr;
    cudaFree(d_root_wl_idx_);  d_root_wl_idx_ = nullptr;
    cudaFree(d_wl_pool_);      d_wl_pool_ = nullptr;
    for (int s = 0; s < 2; ++s) {
      cudaFree(d_cont_d_[s]);      d_cont_d_[s] = nullptr;
      cudaFree(d_cont_w_[s]);      d_cont_w_[s] = nullptr;
      cudaFree(d_cont_wl_idx_[s]); d_cont_wl_idx_[s] = nullptr;
      cudaFree(d_cont_count_[s]);  d_cont_count_[s] = nullptr;
    }
    cudaFree(d_exit_);         d_exit_ = nullptr;
    cudaFree(d_exit_count_);   d_exit_count_ = nullptr;
    // S2 device-fused XYZ accumulation buffers.
    cudaFree(d_xyz_buf_);      d_xyz_buf_ = nullptr;
    cudaFree(d_landed_weight_); d_landed_weight_ = nullptr;
    alloc_xyz_w_ = 0u;  // scrum-312: buffer freed → clear its remembered dims
    alloc_xyz_h_ = 0u;

    cudaFreeHost(pinned_dirs_);        pinned_dirs_ = nullptr;
    cudaFreeHost(pinned_pos_);         pinned_pos_ = nullptr;
    cudaFreeHost(pinned_ws_);          pinned_ws_ = nullptr;
    cudaFreeHost(pinned_from_poly_);   pinned_from_poly_ = nullptr;
    cudaFreeHost(pinned_root_wl_idx_); pinned_root_wl_idx_ = nullptr;
    cudaFreeHost(pinned_rot_c2w_);     pinned_rot_c2w_ = nullptr;
    cudaFreeHost(pinned_cont_count_);  pinned_cont_count_ = nullptr;
    cudaFreeHost(pinned_exit_);        pinned_exit_ = nullptr;

    if (events_created_) {
      cudaEventDestroy(ev_start_h2d_);
      cudaEventDestroy(ev_end_h2d_);
      cudaEventDestroy(ev_end_kernel_);
      cudaEventDestroy(ev_end_d2h_);
      events_created_ = false;
    }
    if (stream_created_) {  // scrum-306.2 async-stream
      cudaStreamDestroy(stream_);
      stream_created_ = false;
    }

    // Capacity trackers tied to the freed persistent buffers + the
    // EnsureSessionBuffers idempotent-fastpath key (buffers_allocated_/n_roots_)
    // are zeroed ONLY here — leaving them intact on the persistent path is what
    // lets the next BeginSession skip re-allocation.
    exit_cap_ = 0;
    alloc_exit_cap_ = 0;
    cont_cap_[0] = 0;
    cont_cap_[1] = 0;
    root_cap_ = 0;
    n_roots_ = 0;
    buffers_allocated_ = false;
  }

  // Session state — always reset. The next BeginSession re-initialises these;
  // clearing here matches the prior Reset() contract and keeps accumulators /
  // RAII vectors clean. (poly_cnt_/tri_cnt_ describe the current crystal and are
  // re-set by BeginSession's UploadCrystalGeometry before any read.)
  ray_alloc_carry_.clear();
  poly_cnt_ = 0;
  tri_cnt_ = 0;
  h_exit_count_ = 0;
  h_cont_count_ = 0;
  ms_layer_idx_ = 0u;
  n_ms_layers_ = 0u;
  // increment 4: filter-descriptor state (filter_desc_max_ci_/filter_n_slot_/
  // crystal_config_id_/final_layer_*/final_ms_*) is produced by EnsureFilterBuffers
  // and now PERSISTS across the per-batch keep path — reset only on full teardown
  // (keep=false block above), not here, or the idempotent-skip path reads zeros.
  proj_type_ = 0u;
  az0_ = 0.0f;
  r_scale_ = 1.0f;
  max_abs_dz_ = 0.0f;
  img_w_ = 0u;
  img_h_ = 0u;
  in_session_ = false;
  scene_ = nullptr;
  render_ = nullptr;
  // transit_seed_ / transit_ray_count_ / transit_seeded_ and the gate_*
  // counterparts are deliberately NOT cleared — the first-seeding gate in
  // BeginSession resets the monotone counter exactly once per spec.seed,
  // matching the Metal `transit_ray_count_` discipline (scrum-267).
}

void CudaTraceBackend::Impl::EnsureGeomCapacity(size_t poly_cnt, size_t tri_cnt) {
  // Grow-only: a ci whose crystal is smaller than a previously-uploaded one
  // reuses the existing (larger) buffers. cudaFree(nullptr) is a no-op.
  if (poly_cnt > alloc_poly_cap_) {
    cudaFree(d_poly_n_); d_poly_n_ = nullptr;
    cudaFree(d_poly_d_); d_poly_d_ = nullptr;
    CheckCuda(cudaMalloc(&d_poly_n_, 3 * poly_cnt * sizeof(float)), "EnsureGeomCapacity cudaMalloc d_poly_n");
    CheckCuda(cudaMalloc(&d_poly_d_, poly_cnt * sizeof(float)), "EnsureGeomCapacity cudaMalloc d_poly_d");
    alloc_poly_cap_ = static_cast<uint32_t>(poly_cnt);
  }
  if (tri_cnt > alloc_tri_cap_) {
    cudaFree(d_tri_vtx_);     d_tri_vtx_ = nullptr;
    cudaFree(d_tri_norm_);    d_tri_norm_ = nullptr;
    cudaFree(d_tri_area_);    d_tri_area_ = nullptr;
    cudaFree(d_tri_to_poly_); d_tri_to_poly_ = nullptr;
    CheckCuda(cudaMalloc(&d_tri_vtx_,     9 * tri_cnt * sizeof(float)),    "EnsureGeomCapacity cudaMalloc d_tri_vtx");
    CheckCuda(cudaMalloc(&d_tri_norm_,    3 * tri_cnt * sizeof(float)),    "EnsureGeomCapacity cudaMalloc d_tri_norm");
    CheckCuda(cudaMalloc(&d_tri_area_,    tri_cnt * sizeof(float)),        "EnsureGeomCapacity cudaMalloc d_tri_area");
    CheckCuda(cudaMalloc(&d_tri_to_poly_, tri_cnt * sizeof(uint16_t)),     "EnsureGeomCapacity cudaMalloc d_tri_to_poly");
    alloc_tri_cap_ = static_cast<uint32_t>(tri_cnt);
  }
}

void CudaTraceBackend::Impl::UploadCrystalGeometry(const Crystal& crystal) {
  // Per-CI geometry H2D, mirrors Metal UploadCrystal (metal_trace_backend.mm:1158).
  // Polygon-face slab geometry + triangle pool (for transit entry-point sampling)
  // + the tri_to_poly argmax map. Updates poly_cnt_/tri_cnt_ to this crystal.
  size_t poly_cnt = crystal.PolygonFaceCount();
  size_t tri_cnt  = crystal.TotalTriangles();
  if (poly_cnt == 0) {
    throw BackendUnavailableError("CudaTraceBackend::UploadCrystalGeometry: degenerate crystal geometry (0 polygons)");
  }
  if (tri_cnt == 0) {
    throw BackendUnavailableError("CudaTraceBackend::UploadCrystalGeometry: degenerate crystal geometry (0 triangles)");
  }
  if (tri_cnt > static_cast<size_t>(lm_pcg::kMaxTriPerKernel)) {
    throw BackendUnavailableError(
        std::string{"CudaTraceBackend::UploadCrystalGeometry: tri_count="} + std::to_string(tri_cnt) +
        " exceeds kMaxTriPerKernel=" + std::to_string(lm_pcg::kMaxTriPerKernel) +
        " (gen/transit kernel proj_prob[] stack bound)");
  }
  EnsureGeomCapacity(poly_cnt, tri_cnt);

  CheckCuda(cudaMemcpy(d_poly_n_, crystal.GetPolygonFaceNormal(), 3 * poly_cnt * sizeof(float),
                       cudaMemcpyHostToDevice), "UploadCrystalGeometry cudaMemcpy d_poly_n");
  CheckCuda(cudaMemcpy(d_poly_d_, crystal.GetPolygonFaceDist(), poly_cnt * sizeof(float),
                       cudaMemcpyHostToDevice), "UploadCrystalGeometry cudaMemcpy d_poly_d");
  CheckCuda(cudaMemcpy(d_tri_vtx_, crystal.GetTriangleVtx(), 9 * tri_cnt * sizeof(float),
                       cudaMemcpyHostToDevice), "UploadCrystalGeometry cudaMemcpy d_tri_vtx");
  CheckCuda(cudaMemcpy(d_tri_norm_, crystal.GetTriangleNormal(), 3 * tri_cnt * sizeof(float),
                       cudaMemcpyHostToDevice), "UploadCrystalGeometry cudaMemcpy d_tri_norm");
  CheckCuda(cudaMemcpy(d_tri_area_, crystal.GetTirangleArea(), tri_cnt * sizeof(float),
                       cudaMemcpyHostToDevice), "UploadCrystalGeometry cudaMemcpy d_tri_area");
  std::vector<uint16_t> tri_to_poly_host;
  FillTriToPoly(crystal, tri_to_poly_host);
  CheckCuda(cudaMemcpy(d_tri_to_poly_, tri_to_poly_host.data(), tri_cnt * sizeof(uint16_t),
                       cudaMemcpyHostToDevice), "UploadCrystalGeometry cudaMemcpy d_tri_to_poly");

  poly_cnt_ = static_cast<uint32_t>(poly_cnt);
  tri_cnt_  = static_cast<uint32_t>(tri_cnt);
}

void CudaTraceBackend::Impl::BuildGeomPool(const SceneConfig& scene) {
  // Build the per-(layer,ci) geometry pool ONCE. MakeCrystal is consumed from
  // rng_ in the EXACT order the per-batch ci-loop used it (layers outer, ci
  // inner; rng_ freshly re-seeded to spec.seed in BeginSession), so the pooled
  // shapes are byte-identical to the prior per-batch upload. The pool then
  // persists across batches (geom_pool_built_) — eliminating the per-batch 6×
  // H2D re-upload of identical geometry.
  pool_crystals_.clear();
  pool_poly_off_.clear(); pool_poly_cnt_.clear();
  pool_tri_off_.clear();  pool_tri_cnt_.clear();
  layer_slot_base_.clear();

  std::vector<float>    h_poly_n, h_poly_d, h_tri_vtx, h_tri_norm, h_tri_area;
  std::vector<uint16_t> h_tri2poly;
  uint32_t poly_acc = 0u, tri_acc = 0u;

  for (size_t mi = 0; mi < scene.ms_.size(); ++mi) {
    layer_slot_base_.push_back(static_cast<uint32_t>(pool_crystals_.size()));
    const auto& settings = scene.ms_[mi].setting_;
    for (size_t ci = 0; ci < settings.size(); ++ci) {
      Crystal crystal = MakeCrystal(rng_, settings[ci].crystal_.param_);
      size_t poly_cnt = crystal.PolygonFaceCount();
      size_t tri_cnt  = crystal.TotalTriangles();
      if (poly_cnt == 0 || tri_cnt == 0) {
        throw BackendUnavailableError("CudaTraceBackend::BuildGeomPool: degenerate crystal geometry");
      }
      if (tri_cnt > static_cast<size_t>(lm_pcg::kMaxTriPerKernel)) {
        throw BackendUnavailableError(
            std::string{"CudaTraceBackend::BuildGeomPool: tri_count="} + std::to_string(tri_cnt) +
            " exceeds kMaxTriPerKernel=" + std::to_string(lm_pcg::kMaxTriPerKernel));
      }
      const float* pn = crystal.GetPolygonFaceNormal();
      const float* pd = crystal.GetPolygonFaceDist();
      h_poly_n.insert(h_poly_n.end(), pn, pn + 3 * poly_cnt);
      h_poly_d.insert(h_poly_d.end(), pd, pd + poly_cnt);
      const float* tv = crystal.GetTriangleVtx();
      const float* tn = crystal.GetTriangleNormal();
      const float* ta = crystal.GetTirangleArea();
      h_tri_vtx.insert(h_tri_vtx.end(), tv, tv + 9 * tri_cnt);
      h_tri_norm.insert(h_tri_norm.end(), tn, tn + 3 * tri_cnt);
      h_tri_area.insert(h_tri_area.end(), ta, ta + tri_cnt);
      std::vector<uint16_t> t2p;
      FillTriToPoly(crystal, t2p);
      h_tri2poly.insert(h_tri2poly.end(), t2p.begin(), t2p.end());

      pool_poly_off_.push_back(poly_acc);
      pool_poly_cnt_.push_back(static_cast<uint32_t>(poly_cnt));
      pool_tri_off_.push_back(tri_acc);
      pool_tri_cnt_.push_back(static_cast<uint32_t>(tri_cnt));
      poly_acc += static_cast<uint32_t>(poly_cnt);
      tri_acc  += static_cast<uint32_t>(tri_cnt);
      pool_crystals_.push_back(std::move(crystal));
    }
  }

  // Free any prior pool (scene change), then allocate + H2D-upload once.
  cudaFree(d_pool_poly_n_);      d_pool_poly_n_ = nullptr;
  cudaFree(d_pool_poly_d_);      d_pool_poly_d_ = nullptr;
  cudaFree(d_pool_tri_vtx_);     d_pool_tri_vtx_ = nullptr;
  cudaFree(d_pool_tri_norm_);    d_pool_tri_norm_ = nullptr;
  cudaFree(d_pool_tri_area_);    d_pool_tri_area_ = nullptr;
  cudaFree(d_pool_tri_to_poly_); d_pool_tri_to_poly_ = nullptr;

  CheckCuda(cudaMalloc(&d_pool_poly_n_,      3 * poly_acc * sizeof(float)),    "BuildGeomPool malloc pool_poly_n");
  CheckCuda(cudaMalloc(&d_pool_poly_d_,          poly_acc * sizeof(float)),    "BuildGeomPool malloc pool_poly_d");
  CheckCuda(cudaMalloc(&d_pool_tri_vtx_,     9 * tri_acc * sizeof(float)),     "BuildGeomPool malloc pool_tri_vtx");
  CheckCuda(cudaMalloc(&d_pool_tri_norm_,    3 * tri_acc * sizeof(float)),     "BuildGeomPool malloc pool_tri_norm");
  CheckCuda(cudaMalloc(&d_pool_tri_area_,        tri_acc * sizeof(float)),     "BuildGeomPool malloc pool_tri_area");
  CheckCuda(cudaMalloc(&d_pool_tri_to_poly_,     tri_acc * sizeof(uint16_t)),  "BuildGeomPool malloc pool_tri_to_poly");

  CheckCuda(cudaMemcpy(d_pool_poly_n_, h_poly_n.data(), h_poly_n.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "BuildGeomPool H2D pool_poly_n");
  CheckCuda(cudaMemcpy(d_pool_poly_d_, h_poly_d.data(), h_poly_d.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "BuildGeomPool H2D pool_poly_d");
  CheckCuda(cudaMemcpy(d_pool_tri_vtx_, h_tri_vtx.data(), h_tri_vtx.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "BuildGeomPool H2D pool_tri_vtx");
  CheckCuda(cudaMemcpy(d_pool_tri_norm_, h_tri_norm.data(), h_tri_norm.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "BuildGeomPool H2D pool_tri_norm");
  CheckCuda(cudaMemcpy(d_pool_tri_area_, h_tri_area.data(), h_tri_area.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "BuildGeomPool H2D pool_tri_area");
  CheckCuda(cudaMemcpy(d_pool_tri_to_poly_, h_tri2poly.data(), h_tri2poly.size() * sizeof(uint16_t),
                       cudaMemcpyHostToDevice), "BuildGeomPool H2D pool_tri_to_poly");

  geom_pool_built_ = true;
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
  for (int s = 0; s < 2; ++s) {
    cudaFree(d_cont_d_[s]);      d_cont_d_[s] = nullptr;
    cudaFree(d_cont_w_[s]);      d_cont_w_[s] = nullptr;
    cudaFree(d_cont_wl_idx_[s]); d_cont_wl_idx_[s] = nullptr;
    cudaFree(d_cont_count_[s]);  d_cont_count_[s] = nullptr;
  }
  cudaFreeHost(pinned_dirs_);     pinned_dirs_ = nullptr;
  cudaFreeHost(pinned_pos_);      pinned_pos_ = nullptr;
  cudaFreeHost(pinned_ws_);       pinned_ws_ = nullptr;
  cudaFreeHost(pinned_from_poly_); pinned_from_poly_ = nullptr;
  cudaFreeHost(pinned_root_wl_idx_); pinned_root_wl_idx_ = nullptr;
  cudaFreeHost(pinned_rot_c2w_);  pinned_rot_c2w_ = nullptr;
  cudaFreeHost(pinned_exit_);     pinned_exit_ = nullptr;
  cudaFreeHost(pinned_cont_count_); pinned_cont_count_ = nullptr;

  size_t max_hits = scene_ != nullptr ? scene_->max_hits_ : kMaxHits;
  exit_cap_ = kCudaDeadExitCap;  // d_exit_ is dead (device-fused XYZ); see kCudaDeadExitCap
  const size_t cont_cap0 = ComputeContCap(n, max_hits);
  cont_cap_[0] = cont_cap0;
  cont_cap_[1] = cont_cap0;

  // Root / rotation / continuation buffers MUST be sized to cont_cap, not n:
  // the per-ci transit writes up to n roots and the trace fan-out fills cont up
  // to ComputeContCap. EnsureContCapacity grows these per continuation layer.
  size_t buf_cap = std::max<size_t>(n, cont_cap0);
  root_cap_ = buf_cap;

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
  for (int s = 0; s < 2; ++s) {
    ck(cudaMalloc(&d_cont_d_[s],      3 * cont_cap0 * sizeof(float)),       "cudaMalloc d_cont_d");
    ck(cudaMalloc(&d_cont_w_[s],      cont_cap0 * sizeof(float)),           "cudaMalloc d_cont_w");
    ck(cudaMalloc(&d_cont_wl_idx_[s], cont_cap0 * sizeof(uint32_t)),        "cudaMalloc d_cont_wl_idx");
    ck(cudaMalloc(&d_cont_count_[s],  sizeof(uint32_t)),                    "cudaMalloc d_cont_count");
  }

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
  ck(cudaMemset(d_cont_count_[0], 0, sizeof(uint32_t)), "cudaMemset d_cont_count_[0] (init)");
  ck(cudaMemset(d_cont_count_[1], 0, sizeof(uint32_t)), "cudaMemset d_cont_count_[1] (init)");

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
  // scrum-306.2: d_exit_ is dead (device-fused XYZ; HasDeviceXyzAccum() always
  // true → the kernel never writes d_exit_/d_exit_count_). It never needs to grow;
  // keep the token capacity. See kCudaDeadExitCap.
  (void)n;
  exit_cap_ = kCudaDeadExitCap;
}

void CudaTraceBackend::Impl::EnsureContCapacity(size_t n_in, int out_slot) {
  // Exit pool grows for this layer's fan-out (also updates exit_cap_).
  EnsureExitCapacity(n_in);
  if (!buffers_allocated_) {
    return;  // first layer hasn't allocated yet (shouldn't happen on the device-roots path)
  }
  auto ck = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::EnsureContCapacity: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };

  // Grow the root/rotation buffers if this continuation layer's incoming count
  // exceeds them (per-ci transit/trace reuse root buf [0, ci_n), ci_n ≤ n_in).
  if (n_in > root_cap_) {
    cudaDeviceSynchronize();  // a prior layer's transit/trace may still read roots.
    cudaFree(d_dirs_);        d_dirs_ = nullptr;
    cudaFree(d_pos_);         d_pos_ = nullptr;
    cudaFree(d_ws_);          d_ws_ = nullptr;
    cudaFree(d_from_poly_);   d_from_poly_ = nullptr;
    cudaFree(d_root_wl_idx_); d_root_wl_idx_ = nullptr;
    cudaFree(d_rot_c2w_);     d_rot_c2w_ = nullptr;
    ck(cudaMalloc(&d_dirs_,        3 * n_in * sizeof(float)),   "cudaMalloc d_dirs (grow)");
    ck(cudaMalloc(&d_pos_,         3 * n_in * sizeof(float)),   "cudaMalloc d_pos (grow)");
    ck(cudaMalloc(&d_ws_,          n_in * sizeof(float)),       "cudaMalloc d_ws (grow)");
    ck(cudaMalloc(&d_from_poly_,   n_in * sizeof(uint32_t)),    "cudaMalloc d_from_poly (grow)");
    ck(cudaMalloc(&d_root_wl_idx_, n_in * sizeof(uint32_t)),    "cudaMalloc d_root_wl_idx (grow)");
    ck(cudaMalloc(&d_rot_c2w_,     9 * n_in * sizeof(float)),   "cudaMalloc d_rot_c2w (grow)");
    root_cap_ = n_in;
  }

  // Grow ONLY the out_slot continuation buffer to hold this layer's fan-out. The
  // in_slot buffer holds the previous layer's continuations being READ this
  // layer (per-ci slices) and must NOT be reallocated.
  const size_t need = ComputeContCap(n_in, scene_ != nullptr ? scene_->max_hits_ : kMaxHits);
  if (need > cont_cap_[out_slot]) {
    cudaDeviceSynchronize();
    cudaFree(d_cont_d_[out_slot]);      d_cont_d_[out_slot] = nullptr;
    cudaFree(d_cont_w_[out_slot]);      d_cont_w_[out_slot] = nullptr;
    cudaFree(d_cont_wl_idx_[out_slot]); d_cont_wl_idx_[out_slot] = nullptr;
    ck(cudaMalloc(&d_cont_d_[out_slot],      3 * need * sizeof(float)),    "cudaMalloc d_cont_d (grow)");
    ck(cudaMalloc(&d_cont_w_[out_slot],      need * sizeof(float)),        "cudaMalloc d_cont_w (grow)");
    ck(cudaMalloc(&d_cont_wl_idx_[out_slot], need * sizeof(uint32_t)),     "cudaMalloc d_cont_wl_idx (grow)");
    cont_cap_[out_slot] = need;
  }
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
  // scrum-306.2 increment 4: filter descriptors are per-session-constant (config
  // fixed across batches) — built once per scene and persisted (filter_built_,
  // reset on scene change / full teardown). Skips the free+malloc+H2D churn that
  // ran every BeginSession (post-pool the dominant host-API cost).
  if (filter_built_) {
    return;
  }
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
    filter_built_       = true;
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

  // Capture final-layer host filter state PER-CI — DrainExits applies FilterSpec
  // on records tagged with the final ms_layer_idx, indexed by ExitRayRecord
  // .crystal_id (mirrors Metal last_layer_crystals_[ci], metal:2403-2409). Proto
  // crystals (deterministic rng) suffice: the host filter only needs the face
  // topology (GetFn), which is param-determined, not the exact traced instance.
  const auto& last_ms_layer = spec.scene->ms_.back();
  const size_t last_ci_cnt = last_ms_layer.setting_.size();
  RandomNumberGenerator drain_proto_rng(0xC0FEFEEDu);
  final_layer_filter_configs_.clear();
  final_layer_crystals_.clear();
  final_layer_axis_dists_.clear();
  final_layer_filter_configs_.reserve(last_ci_cnt);
  final_layer_crystals_.reserve(last_ci_cnt);
  final_layer_axis_dists_.reserve(last_ci_cnt);
  for (size_t ci = 0; ci < last_ci_cnt; ++ci) {
    const auto& s = last_ms_layer.setting_[ci];
    final_layer_filter_configs_.push_back(s.filter_);
    final_layer_crystals_.push_back(MakeCrystal(drain_proto_rng, s.crystal_.param_));
    final_layer_axis_dists_.push_back(s.crystal_.axis_);
  }
  final_ms_layer_idx_ = static_cast<uint8_t>(n_layers - 1u);
  final_ms_prob_ = last_ms_layer.prob_;

  // crystal_config_id_ for DeviceFilterMatchCrystal. Per-ci config ids are passed
  // inline to the trace kernel (ci_cfg_id); this session-level value mirrors the
  // first final-layer crystal for any legacy single-value consumer.
  crystal_config_id_ =
      (!final_layer_crystals_.empty() && final_layer_crystals_[0].config_id_ != kInvalidId)
          ? static_cast<uint32_t>(final_layer_crystals_[0].config_id_)
          : 0xFFFFu;
  filter_built_ = true;
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

  const int cuda_dev = CudaSelectedDevice();
  cudaError_t err = cudaSetDevice(cuda_dev);
  if (err != cudaSuccess) {
    (void)cudaGetLastError();
    throw BackendUnavailableError(std::string{"CudaTraceBackend::BeginSession: cudaSetDevice("} +
                                  std::to_string(cuda_dev) + ") failed: " + cudaGetErrorString(err));
  }

  try {
    impl_->scene_ = spec.scene;
    impl_->render_ = spec.render;
    impl_->wl_ = spec.wl;
    impl_->rng_.SetSeed(spec.seed);
    impl_->ray_alloc_carry_.clear();  // per-(layer,ci) partition carry — fresh per session

    // scrum-306.2 increment 4: a scene change invalidates every per-session-constant
    // cache (geometry pool, filter descriptors, wl pool). Within one scene these are
    // built/uploaded ONCE and reused across the per-batch BeginSession cycle.
    if (impl_->pool_scene_ != static_cast<const void*>(spec.scene)) {
      impl_->geom_pool_built_   = false;
      impl_->filter_built_      = false;
      impl_->wl_pool_uploaded_  = false;
      impl_->pool_scene_        = static_cast<const void*>(spec.scene);
    }

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

    // scrum-306.2: resolve the device-gen escape hatch (idempotent env read,
    // once per Run via the gen_seeded_ guard) BEFORE geometry setup so the
    // pool-vs-legacy upload path can branch on it. Resolved here instead of the
    // later seed block; the duplicate read there is removed.
    Logger& geom_logger = impl_->logger != nullptr ? *impl_->logger : GetGlobalLogger();
    if (!impl_->gen_seeded_) {
      impl_->disable_device_gen_ = env::DisableDeviceGen(geom_logger);
    }

    // Geometry to device. Device-gen path: build the per-(layer,ci) pool ONCE
    // and persist it across the per-batch cycle (parity-exact — same shapes in
    // the same rng_ order; eliminates the per-batch 6×blocking-H2D re-upload of
    // identical geometry). Host-roots fallback keeps the legacy per-batch
    // single-crystal upload (byte-exact rng_ interleaving with InitRayFirstMs).
    // crystal_for_geom (dummy_rng; shape-independent refractive index) still
    // feeds the wl pool + refractive-index log below in both paths.
    if (!impl_->disable_device_gen_) {
      if (!impl_->geom_pool_built_) {
        impl_->BuildGeomPool(*spec.scene);  // sets geom_pool_built_
      }
      impl_->poly_cnt_ = impl_->pool_poly_cnt_[0];
      impl_->tri_cnt_  = impl_->pool_tri_cnt_[0];
    } else {
      impl_->UploadCrystalGeometry(crystal_for_geom);
    }
    size_t tri_cnt = impl_->tri_cnt_;

    // Per-ray wavelength pool (296.6 DR-3). One session covers the whole
    // spectrum; build the WlEntry table (n_idx + spd_weight per sampled wl) from
    // the scene's light source + this crystal and upload it once. The host
    // first-layer path / device gen-root draws a per-ray wl_idx in [0, M); the
    // trace kernel reads d_wl_pool_[wl_idx].n_idx (per-ray refraction) and tags
    // exit records with wl_idx for host-side wavelength reconstruction
    // (simulator.cpp). Mirrors Metal ComputeWlPool + EnsureWlPoolBuffer. The
    // buffer is allocated once (size = env-resolved M, stable per process) and
    // re-uploaded each BeginSession since the crystal n_idx is scene-dependent.
    // scrum-306.2 increment 4: the wl pool (n_idx + spd_weight per sampled wl) is
    // per-session-constant (crystal n_idx + spectrum fixed across batches), so
    // compute + upload it ONCE per scene rather than every BeginSession.
    Logger& wl_logger = impl_->logger != nullptr ? *impl_->logger : GetGlobalLogger();
    if (!impl_->wl_pool_uploaded_) {
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
      impl_->wl_pool_uploaded_ = true;
    }

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
    impl_->gen_seed_     = spec.seed ^ kCudaGenNonce;
    impl_->shuffle_seed_ = spec.seed ^ kCudaShuffleNonce;
    if (!impl_->transit_seeded_) {
      impl_->transit_ray_count_ = 0;
      impl_->transit_seeded_ = true;
    }
    if (!impl_->gate_seeded_) {
      impl_->gate_ray_count_ = 0;
      impl_->gate_seeded_ = true;
    }
    if (!impl_->gen_seeded_) {
      impl_->gen_ray_count_ = 0;
      impl_->gen_seeded_ = true;
      // disable_device_gen_ is resolved earlier (geometry-setup block) so the
      // pool-vs-legacy upload path can branch on it before BuildGeomPool.
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

    // S2 device-fused XYZ accumulation: allocate the W*H*3 device buffer +
    // landed-weight scalar and zero them. Sized to render.resolution_; the
    // ms_mode==0 kernel emit gate atomicAdds (cmf * w) into d_xyz_buf_ and
    // adds the in-bounds weight into d_landed_weight_. ReadbackXyzAccum D2H
    // copies them and zeros for the next batch.
    if (spec.render == nullptr) {
      throw BackendUnavailableError("CudaTraceBackend::BeginSession: spec.render is null");
    }
    impl_->img_w_ = static_cast<uint32_t>(spec.render->resolution_[0]);
    impl_->img_h_ = static_cast<uint32_t>(spec.render->resolution_[1]);
    if (impl_->img_w_ == 0u || impl_->img_h_ == 0u) {
      throw BackendUnavailableError(
          "CudaTraceBackend::BeginSession: render.resolution_ has a zero dimension");
    }
    // Projection routing — mirrors Metal BeginSession lines 1947-1955.
    if (spec.render->lens_.type_ == LensParam::kDualFisheyeEqualArea) {
      impl_->proj_type_   = 1u;
      impl_->max_abs_dz_  = spec.render->overlap_;
      impl_->r_scale_     = projection::ComputeEARScale(spec.render->overlap_);
      impl_->az0_         = 0.0f;  // unused on dual-fisheye path
    } else {
      impl_->proj_type_   = 0u;
      impl_->max_abs_dz_  = 0.0f;
      impl_->r_scale_     = 1.0f;
      // az0 = atan2(camera-rot applied to +Z). Same derivation as Metal
      // ComputeAz0 (metal_trace_backend.mm:163, MakeCameraRotation at
      // scatter_accum.hpp:75). Only meaningful at zenith view (el ≈ 90°);
      // IsCompatible enforces that constraint so non-zenith rectangular
      // configs fall back to legacy CPU. Inlined here (instead of including
      // scatter_accum.hpp) to keep the .cu translation unit's host-include
      // surface narrow.
      Rotation camera_rot;
      float ax_z_chain[3]{ 0.0f, 0.0f, 1.0f };
      float ax_y_chain[3]{ 0.0f, 1.0f, 0.0f };
      camera_rot
          .Chain({ ax_z_chain, (-90.0f + spec.render->view_.ro_) * math::kDegreeToRad })
          .Chain({ ax_y_chain, (90.0f - spec.render->view_.el_) * math::kDegreeToRad })
          .Chain({ ax_z_chain, spec.render->view_.az_ * math::kDegreeToRad });
      float ax_z[3]{ 0.0f, 0.0f, 1.0f };
      camera_rot.Apply(ax_z);
      impl_->az0_ = std::atan2(ax_z[1], ax_z[0]);
    }
    const size_t xyz_floats = static_cast<size_t>(impl_->img_w_) *
                              static_cast<size_t>(impl_->img_h_) * 3u;
    // scrum-312 (third-clock drain): d_xyz_buf_ / d_landed_weight_ are PERSISTENT
    // accumulators across batches. Zero ONLY on fresh allocation — the former
    // per-call memset moved to ReadbackXyzAccum's post-drain reset, so device
    // accumulation survives across BeginSession/EndSession within a drain window
    // (the simulator drains on display cadence, not per batch). The buffer is
    // always zero at a window start: first window via this alloc-zero, later
    // windows via the previous drain's memset.
    if (impl_->d_xyz_buf_ == nullptr) {
      CheckCuda(cudaMalloc(&impl_->d_xyz_buf_, xyz_floats * sizeof(float)),
                "BeginSession cudaMalloc d_xyz_buf");
      CheckCuda(cudaMemset(impl_->d_xyz_buf_, 0, xyz_floats * sizeof(float)),
                "BeginSession cudaMemset d_xyz_buf");
      // scrum-312: remember the dims this persistent buffer was sized for so the
      // between-session drain can verify caller dims against real capacity.
      impl_->alloc_xyz_w_ = impl_->img_w_;
      impl_->alloc_xyz_h_ = impl_->img_h_;
    }
    if (impl_->d_landed_weight_ == nullptr) {
      CheckCuda(cudaMalloc(&impl_->d_landed_weight_, sizeof(float)),
                "BeginSession cudaMalloc d_landed_weight");
      CheckCuda(cudaMemset(impl_->d_landed_weight_, 0, sizeof(float)),
                "BeginSession cudaMemset d_landed_weight");
    }

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
    if (!impl_->stream_created_) {  // scrum-306.2 async-stream
      CheckCuda(cudaStreamCreate(&impl_->stream_), "BeginSession cudaStreamCreate");
      impl_->stream_created_ = true;
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

  // Local helper: Reset() + BackendUnavailableError for CUDA call failures.
  auto ck_reset = [this](cudaError_t e, const char* ctx) {
    if (e != cudaSuccess) {
      impl_->Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::TraceLayer: "} + ctx + ": " +
                                    cudaGetErrorString(e));
    }
  };

  // Resolve current MS layer. Multi-CI (全量, owner 2026-06-27): a layer may hold
  // several crystals (ScatteringSetting) each with a proportion; PartitionCrystal-
  // RayNum splits this layer's rays into contiguous per-ci slices, and each ci is
  // gen/transit'd + traced through ITS OWN crystal geometry. Mirrors Metal
  // TraceLayer's ci-loop (metal_trace_backend.mm:2086-2288) and legacy
  // simulator.cpp:826-940.
  if (impl_->ms_layer_idx_ >= impl_->n_ms_layers_) {
    throw BackendUnavailableError(
        std::string{"CudaTraceBackend::TraceLayer: ms_layer_idx_="} +
        std::to_string(impl_->ms_layer_idx_) + " >= n_ms_layers_=" +
        std::to_string(impl_->n_ms_layers_) + " (over-traced past final layer)");
  }
  const auto& ms_layer = impl_->scene_->ms_[impl_->ms_layer_idx_];
  const size_t crystal_cnt = ms_layer.setting_.size();
  const bool first_ms = !roots.is_device;

  // ms_mode = continuation iff this is NOT the final MS layer. Final layer
  // (ms_mode==0) writes every exit candidate to d_exit_; non-final layer
  // (ms_mode==1) routes each candidate through the per-emit PCG gate.
  const bool is_final_layer = (impl_->ms_layer_idx_ + 1u >= impl_->n_ms_layers_);
  const uint8_t ms_mode = is_final_layer ? 0u : 1u;
  const float ms_prob = is_final_layer ? 0.0f : ms_layer.prob_;

  // Continuation ping-pong slots: this layer READS the previous layer's
  // continuations from cont[in_slot] (per-ci slice) and WRITES its own to
  // cont[out_slot]. Mirrors Metal slot = ms_idx & 1.
  const int out_slot = static_cast<int>(impl_->ms_layer_idx_ & 1u);
  const int in_slot = (impl_->ms_layer_idx_ == 0u)
                          ? 0
                          : static_cast<int>((impl_->ms_layer_idx_ - 1u) & 1u);

  // Buffer sizing. First layer allocates root + both cont slots (EnsureSession-
  // Buffers). Continuation layers must NOT realloc the root/in_slot buffers
  // (in_slot holds the device-resident continuations being read; 296.4 root-
  // cause). Only the exit pool may grow. (3+ layer cont-out growth: see
  // EnsureContCapacity follow-up; canonical 2-layer multi-CI fits cont_cap_.)
  if (first_ms) {
    impl_->EnsureSessionBuffers(n);
  } else {
    impl_->EnsureContCapacity(n, out_slot);
  }

  // Partition this layer's `n` rays across its crystals by proportion. carry is
  // per-(layer) persistent across batches (largest-remainder), mirroring
  // simulator.cpp:830 ray_alloc_carry[mi].
  std::vector<float> proportions;
  proportions.reserve(crystal_cnt);
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    proportions.push_back(ms_layer.setting_[ci].crystal_proportion_);
  }
  if (impl_->ray_alloc_carry_.size() < impl_->n_ms_layers_) {
    impl_->ray_alloc_carry_.resize(impl_->n_ms_layers_);
  }
  auto& carry = impl_->ray_alloc_carry_[impl_->ms_layer_idx_];
  if (carry.size() != crystal_cnt) {
    carry.assign(crystal_cnt, 0.0);
  }
  auto crystal_ray_num = PartitionCrystalRayNum(proportions, n, carry);

  // Zero the layer's accumulators ONCE before the ci-loop: each ci's trace
  // dispatch APPENDS to d_exit_ / cont[out_slot] via atomic counters.
  cudaEventRecord(impl_->ev_start_h2d_, impl_->stream_);
  ck_reset(cudaMemset(impl_->d_exit_count_, 0, sizeof(uint32_t)), "cudaMemset d_exit_count");
  ck_reset(cudaMemset(impl_->d_cont_count_[out_slot], 0, sizeof(uint32_t)),
           "cudaMemset d_cont_count_[out_slot]");

  const uint32_t max_hits = static_cast<uint32_t>(impl_->scene_->max_hits_);
  size_t ci_start = 0;  // running offset into cont[in_slot] for continuation slices
  for (size_t ci = 0; ci < crystal_cnt; ci++) {
    const size_t ci_n = crystal_ray_num[ci];
    if (ci_n == 0u) {
      continue;
    }
    const auto& ms_setting = ms_layer.setting_[ci];
    const uint32_t cin = static_cast<uint32_t>(ci_n);
    const uint32_t grid = (cin + 255u) / 256u;

    // Per-ci geometry. Device-gen path (default): point at this (layer,ci)'s
    // pre-uploaded pool slot — built ONCE in BeginSession, persisted across
    // batches, no per-batch MakeCrystal / 6×H2D (scrum-306.2; parity-exact since
    // rng_ is reset to the constant effective_seed_ every batch → same shapes).
    // Host-roots fallback (disable_device_gen_): per-batch MakeCrystal + legacy
    // single-crystal upload (byte-exact rng_ interleaving with InitRayFirstMs).
    // gen/transit read the slot's triangle pool, trace its polygon-slab — all via
    // geom_* below, so the kernels themselves are unchanged.
    std::unique_ptr<Crystal> ci_crystal_fb;  // fallback path only (avoids Crystal default-ctor)
    uint32_t ci_cfg_id;
    const float* geom_poly_n; const float* geom_poly_d; uint32_t geom_poly_cnt;
    const float* geom_tri_vtx; const float* geom_tri_norm; const float* geom_tri_area;
    const uint16_t* geom_tri_to_poly; uint32_t geom_tri_cnt;
    if (impl_->disable_device_gen_) {
      ci_crystal_fb = std::make_unique<Crystal>(MakeCrystal(impl_->rng_, ms_setting.crystal_.param_));
      impl_->UploadCrystalGeometry(*ci_crystal_fb);
      ci_cfg_id = (ci_crystal_fb->config_id_ == kInvalidId)
                      ? 0xFFFFu : static_cast<uint32_t>(ci_crystal_fb->config_id_);
      geom_poly_n = impl_->d_poly_n_; geom_poly_d = impl_->d_poly_d_; geom_poly_cnt = impl_->poly_cnt_;
      geom_tri_vtx = impl_->d_tri_vtx_; geom_tri_norm = impl_->d_tri_norm_;
      geom_tri_area = impl_->d_tri_area_; geom_tri_to_poly = impl_->d_tri_to_poly_;
      geom_tri_cnt = impl_->tri_cnt_;
    } else {
      const uint32_t slot = impl_->layer_slot_base_[impl_->ms_layer_idx_] + static_cast<uint32_t>(ci);
      const Crystal& pc = impl_->pool_crystals_[slot];
      ci_cfg_id = (pc.config_id_ == kInvalidId) ? 0xFFFFu : static_cast<uint32_t>(pc.config_id_);
      const uint32_t po = impl_->pool_poly_off_[slot];
      const uint32_t to = impl_->pool_tri_off_[slot];
      geom_poly_n = impl_->d_pool_poly_n_ + 3u * po; geom_poly_d = impl_->d_pool_poly_d_ + po;
      geom_poly_cnt = impl_->pool_poly_cnt_[slot];
      geom_tri_vtx = impl_->d_pool_tri_vtx_ + 9u * to; geom_tri_norm = impl_->d_pool_tri_norm_ + 3u * to;
      geom_tri_area = impl_->d_pool_tri_area_ + to; geom_tri_to_poly = impl_->d_pool_tri_to_poly_ + to;
      geom_tri_cnt = impl_->pool_tri_cnt_[slot];
    }

    // ── Generate / transit this ci's root rays into root buf [0, ci_n) ──────
    if (first_ms && !impl_->disable_device_gen_) {
      // Device root-gen (296.6): samples orientation + sun-cone dir + entry point
      // + per-ray wl on device for THIS ci's crystal geometry. gen_ray_count_ is
      // monotone so each (layer,ci,batch) consumes a disjoint PCG range.
      lm_pcg::GenRootKernelParams gp =
          BuildGenGpParams(ms_setting.crystal_.axis_, geom_tri_cnt, cin,
                           impl_->scene_->light_source_.param_, impl_->wl_pool_size_);
      gp.gen_seed     = impl_->gen_seed_;
      gp.gen_ray_base = NarrowPcgRayBase(impl_->gen_ray_count_, ci_n, "cuda-gen");
      gen_root_kernel<<<grid, 256, 0, impl_->stream_>>>(impl_->d_dirs_, impl_->d_pos_, impl_->d_ws_, impl_->d_from_poly_,
                                     impl_->d_rot_c2w_, impl_->d_root_wl_idx_, geom_tri_vtx,
                                     geom_tri_norm, geom_tri_area, geom_tri_to_poly,
                                     impl_->d_wl_pool_, gp, cin);
      ck_reset(cudaPeekAtLastError(), "gen_root_kernel launch");
      impl_->gen_ray_count_ += ci_n;
    } else if (first_ms) {
      // Host-roots fallback (LUMICE_DISABLE_DEVICE_GEN): InitRayFirstMs for ci_n
      // rays through ci_crystal, then H2D the staged root buffers [0, ci_n).
      try {
        const auto& axis_dist = ms_setting.crystal_.axis_;
        RayBuffer workspace[2]{};
        workspace[0].Reset(ci_n * 2);
        workspace[1].Reset(ci_n * 4);
        RayBuffer all_data = AllocateAllData(*impl_->scene_, ci_n);
        InitRayFirstMs(impl_->rng_, impl_->scene_->light_source_.param_, impl_->wl_, ci_n, *ci_crystal_fb,
                       impl_->ms_layer_idx_, axis_dist, workspace, all_data);
        for (size_t i = 0; i < ci_n; ++i) {
          const auto& r = all_data[i];
          impl_->pinned_dirs_[i * 3 + 0] = r.d_[0];
          impl_->pinned_dirs_[i * 3 + 1] = r.d_[1];
          impl_->pinned_dirs_[i * 3 + 2] = r.d_[2];
          impl_->pinned_pos_[i * 3 + 0] = r.p_[0];
          impl_->pinned_pos_[i * 3 + 1] = r.p_[1];
          impl_->pinned_pos_[i * 3 + 2] = r.p_[2];
          uint32_t wl_idx = static_cast<uint32_t>(impl_->rng_.GetUniform() *
                                                  static_cast<float>(impl_->wl_pool_size_));
          if (wl_idx >= impl_->wl_pool_size_) {
            wl_idx = impl_->wl_pool_size_ - 1u;
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
      cudaMemcpyAsync(impl_->d_rot_c2w_, impl_->pinned_rot_c2w_, 9 * ci_n * sizeof(float),
                      cudaMemcpyHostToDevice);
      cudaMemcpyAsync(impl_->d_dirs_, impl_->pinned_dirs_, 3 * ci_n * sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpyAsync(impl_->d_pos_, impl_->pinned_pos_, 3 * ci_n * sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpyAsync(impl_->d_ws_, impl_->pinned_ws_, ci_n * sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpyAsync(impl_->d_from_poly_, impl_->pinned_from_poly_, ci_n * sizeof(uint32_t),
                      cudaMemcpyHostToDevice);
      cudaMemcpyAsync(impl_->d_root_wl_idx_, impl_->pinned_root_wl_idx_, ci_n * sizeof(uint32_t),
                      cudaMemcpyHostToDevice);
      ck_reset(cudaGetLastError(), "H2D batch");
    } else {
      // ── Continuation: transit cont[in_slot] slice [ci_start, ci_start+ci_n) ──
      // into root buf [0, ci_n) using THIS ci's crystal (orientation resample +
      // entry-point sample). Mirrors Metal EncodeTransitRoot's per-ci slice
      // (metal_trace_backend.mm:2236-2248). Offset the in_slot cont pointers by
      // ci_start; the kernel indexes tid in [0, ci_n). transit_ray_count_ is
      // monotone → disjoint PCG range per (layer,ci,batch).
      lm_pcg::GenRootKernelParams gp =
          BuildTransitGpParams(ms_setting.crystal_.axis_, geom_tri_cnt, cin);
      gp.gen_seed     = impl_->transit_seed_;
      gp.gen_ray_base = NarrowPcgRayBase(impl_->transit_ray_count_, ci_n, "cuda-transit");
      transit_multi_ms_kernel<<<grid, 256, 0, impl_->stream_>>>(
          impl_->d_cont_d_[in_slot] + ci_start * 3u, impl_->d_cont_w_[in_slot] + ci_start,
          impl_->d_cont_wl_idx_[in_slot] + ci_start,
          impl_->d_dirs_, impl_->d_pos_, impl_->d_ws_, impl_->d_from_poly_,
          impl_->d_rot_c2w_, impl_->d_root_wl_idx_,
          geom_tri_vtx, geom_tri_norm, geom_tri_area, geom_tri_to_poly,
          gp, cin);
      ck_reset(cudaPeekAtLastError(), "transit kernel launch");
      impl_->transit_ray_count_ += ci_n;
      ci_start += ci_n;
    }

    // ── Trace this ci's ci_n root rays through ci_crystal geometry ──────────
    // Exits APPEND to d_exit_ (crystal_id=ci tag); continuations APPEND to
    // cont[out_slot] (atomic counters, zeroed once before the loop). The kernel
    // computes its filter-desc slot as ms_layer_idx*max_ci + crystal_id, so
    // passing crystal_id=ci selects this ci's per-(layer,ci) filter descriptor.
    // S2: record ev_end_h2d_ right BEFORE the kernel launch and ev_end_kernel_
    // right AFTER, both inside the loop. The LAST iteration's pair wins.
    // Stream order is strict (default stream) so end_h2d → kernel → end_kernel,
    // and kernel_ms = elapsed(end_h2d, end_kernel) is positive and meaningful.
    cudaEventRecord(impl_->ev_end_h2d_, impl_->stream_);
    // Filter / gate buffers are passed unconditionally (S2): the ms_mode==0
    // device-fused emit gate also calls DeviceFilterCheck. Previously these
    // were short-circuited to nullptr on ms_mode==0 dispatches — keeping that
    // shortcut would make the new ms_mode==0 emit gate deref nullptr.
    trace_single_ms_kernel<<<grid, 256, 0, impl_->stream_>>>(
        impl_->d_dirs_, impl_->d_pos_, impl_->d_ws_, impl_->d_from_poly_,
        cin, geom_poly_n, geom_poly_d,
        geom_poly_cnt, impl_->d_rot_c2w_, impl_->d_wl_pool_, impl_->d_root_wl_idx_,
        max_hits, impl_->d_exit_,
        static_cast<uint32_t>(impl_->exit_cap_), impl_->d_exit_count_,
        /*crystal_id=*/static_cast<uint32_t>(ci), /*ms_layer_idx=*/impl_->ms_layer_idx_,
        ms_mode, ms_prob,
        ms_mode == 1u ? impl_->d_cont_d_[out_slot]      : nullptr,
        ms_mode == 1u ? impl_->d_cont_w_[out_slot]      : nullptr,
        ms_mode == 1u ? impl_->d_cont_wl_idx_[out_slot] : nullptr,
        ms_mode == 1u ? impl_->d_cont_count_[out_slot]  : nullptr,
        ms_mode == 1u ? static_cast<uint32_t>(impl_->cont_cap_[out_slot]) : 0u,
        impl_->gate_seed_,
        NarrowPcgRayBase(impl_->gate_ray_count_, ci_n, "cuda-gate"),
        impl_->d_filter_desc_,
        impl_->d_getfn_offsets_,
        impl_->d_getfn_bytes_,
        impl_->d_complex_sub_desc_,
        impl_->filter_desc_max_ci_,
        ci_cfg_id,
        // S2 device-fused accumulation params.
        impl_->d_xyz_buf_, impl_->d_landed_weight_,
        impl_->proj_type_, impl_->az0_, impl_->r_scale_, impl_->max_abs_dz_,
        impl_->img_w_, impl_->img_h_,
        impl_->final_ms_prob_,
        impl_->gate_seed_,
        NarrowPcgRayBase(impl_->gate_ray_count_, ci_n, "cuda-gate-final"));
    ck_reset(cudaPeekAtLastError(), "kernel launch");
    // S2: ev_end_kernel_ recorded inside the loop captures real kernel time.
    // The original outside-loop placement (right next to ev_end_h2d_) collapsed
    // kernel_ms to ~0ms after the multi-CI rewrite — they recorded back-to-back
    // before any compute observed by the timer.
    cudaEventRecord(impl_->ev_end_kernel_, impl_->stream_);
    // S2: gate_ray_count_ is advanced for BOTH ms_modes now — the ms_mode==0
    // path also draws prob via the gate stream so every dispatch (final or
    // not) must consume a disjoint global_idx range to avoid stream collision
    // across batches. Mirrors the monotone-counter discipline of transit/gen.
    impl_->gate_ray_count_ += ci_n;
  }  // ci-loop

  // 4B readback (synchronous on the default stream — also the first sync
  // point that surfaces async kernel errors: check the return value).
  ck_reset(cudaMemcpy(&impl_->h_exit_count_, impl_->d_exit_count_, sizeof(uint32_t),
                      cudaMemcpyDeviceToHost), "4B readback");
  cudaEventRecord(impl_->ev_end_d2h_, impl_->stream_);
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

  // gate PCG counter was advanced per-ci inside the loop (monotone), so no
  // layer-level advance here.

  // Continuation-count readback (only meaningful for ms_mode==1). Reads the OUT
  // slot counter this layer's ci-loop appended into; the kernel leaves it
  // untouched on the ms_mode==0 (final) path.
  size_t cont_count_for_handle = 0u;
  if (ms_mode == 1u) {
    ck_reset(cudaMemcpy(&impl_->h_cont_count_, impl_->d_cont_count_[out_slot], sizeof(uint32_t),
                        cudaMemcpyDeviceToHost), "4B cont_count readback");
    if (impl_->h_cont_count_ >= impl_->cont_cap_[out_slot]) {
      if (impl_->logger != nullptr) {
        ILOG_WARN(*impl_->logger,
                  "CudaTraceBackend::TraceLayer: cont buffer overflow (count={} cap={}); tail dropped",
                  impl_->h_cont_count_, impl_->cont_cap_[out_slot]);
      }
      impl_->h_cont_count_ = static_cast<uint32_t>(impl_->cont_cap_[out_slot]);
    }
    cont_count_for_handle = impl_->h_cont_count_;
  } else {
    impl_->h_cont_count_ = 0u;
  }

  return std::make_unique<CudaLayerHandle>(cont_count_for_handle,
                                           LayerStats{impl_->h_exit_count_, 0.0f});
}

RootRaySource CudaTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  if (!impl_->in_session_) {
    throw BackendUnavailableError("CudaTraceBackend::Recombine called outside session");
  }
  // Consume the layer handle (frees per-layer state at scope exit). The
  // continuation count is the authoritative value from impl_->h_cont_count_
  // populated by TraceLayer's tail readback.
  handle.reset();

  // Transit is now LAZY (multi-CI 全量): it is performed per-ci inside the NEXT
  // TraceLayer's continuation branch (mirrors Metal — Recombine just advances
  // the layer index and returns the continuation count). The continuations this
  // just-traced layer produced live in cont[written_slot]; the next TraceLayer
  // reads them per-ci-slice as in_slot. No transit kernel / cont_count zeroing
  // here: the next TraceLayer zeroes its own out_slot before its ci-loop.
  const uint32_t cont_n = impl_->h_cont_count_;
  const int written_slot = static_cast<int>(impl_->ms_layer_idx_ & 1u);

  // Continuation-pool decorrelation shuffle (task-gpu-backend-recombine-shuffle).
  // Mirrors legacy host Fisher-Yates (simulator.cpp:946-950): per-CI trace
  // dispatches leave cont[written_slot] grouped by parent CI; without this
  // shuffle, the next layer's per-CI slicing would hand parent-correlated
  // subsets to each child CI (explore-300 root cause).
  //
  // Slot accounting (cuda_trace_backend.cu:1930-1933 F-verify, plan §8):
  //   - This layer wrote cont[out_slot = ms_layer_idx_ & 1u]; we are BEFORE
  //     the ++ so written_slot above is exactly that out_slot.
  //   - The next layer N+1 reads in_slot = ((N+1)-1) & 1 = N & 1 = written_slot.
  //   - Gather READ cont[written_slot] (source), WRITE cont[other_slot] (dest),
  //     then SWAP the slot pointers so cont[written_slot] holds the shuffled
  //     data the next layer will consume.
  //   - cont[other_slot] post-swap holds the stale source; the next layer will
  //     overwrite it (out_slot = (N+1) & 1 = other_slot) — no aliasing.
  if (spec.shuffle && cont_n > 1u) {
    const int other_slot = 1 - written_slot;
    // Ensure cont[other_slot] has capacity for cont_n entries. Mirrors the
    // grow-on-overflow path in EnsureContCapacity (alloc d/w/wl together).
    if (impl_->cont_cap_[other_slot] < cont_n) {
      auto ck = [this](cudaError_t e, const char* ctx) {
        if (e != cudaSuccess) {
          impl_->Reset();
          throw BackendUnavailableError(std::string{"CudaTraceBackend::Recombine: "} + ctx + ": " +
                                        cudaGetErrorString(e));
        }
      };
      cudaDeviceSynchronize();
      cudaFree(impl_->d_cont_d_[other_slot]);      impl_->d_cont_d_[other_slot] = nullptr;
      cudaFree(impl_->d_cont_w_[other_slot]);      impl_->d_cont_w_[other_slot] = nullptr;
      cudaFree(impl_->d_cont_wl_idx_[other_slot]); impl_->d_cont_wl_idx_[other_slot] = nullptr;
      ck(cudaMalloc(&impl_->d_cont_d_[other_slot],      3 * cont_n * sizeof(float)),
         "cudaMalloc d_cont_d (shuffle grow)");
      ck(cudaMalloc(&impl_->d_cont_w_[other_slot],      cont_n * sizeof(float)),
         "cudaMalloc d_cont_w (shuffle grow)");
      ck(cudaMalloc(&impl_->d_cont_wl_idx_[other_slot], cont_n * sizeof(uint32_t)),
         "cudaMalloc d_cont_wl_idx (shuffle grow)");
      impl_->cont_cap_[other_slot] = cont_n;
    }
    const uint32_t shuf_seed = impl_->shuffle_seed_ ^ static_cast<uint32_t>(impl_->ms_layer_idx_);
    const uint32_t grid = (cont_n + 255u) / 256u;
    shuffle_cont_kernel<<<grid, 256, 0, impl_->stream_>>>(impl_->d_cont_d_[written_slot],
                                       impl_->d_cont_w_[written_slot],
                                       impl_->d_cont_wl_idx_[written_slot],
                                       impl_->d_cont_d_[other_slot],
                                       impl_->d_cont_w_[other_slot],
                                       impl_->d_cont_wl_idx_[other_slot],
                                       cont_n, shuf_seed);
    cudaError_t launch_err = cudaPeekAtLastError();
    if (launch_err != cudaSuccess) {
      impl_->Reset();
      throw BackendUnavailableError(std::string{"CudaTraceBackend::Recombine: shuffle_cont_kernel launch: "} +
                                    cudaGetErrorString(launch_err));
    }
    // Swap ALL four parallel slot arrays so cont[written_slot] holds the
    // shuffled data (next layer's in_slot = written_slot). Omitting wl_idx
    // here would silently desynchronize per-ray wavelength tagging — Feistel /
    // energy parity could not detect it.
    std::swap(impl_->d_cont_d_[written_slot],      impl_->d_cont_d_[other_slot]);
    std::swap(impl_->d_cont_w_[written_slot],      impl_->d_cont_w_[other_slot]);
    std::swap(impl_->d_cont_wl_idx_[written_slot], impl_->d_cont_wl_idx_[other_slot]);
    std::swap(impl_->cont_cap_[written_slot],      impl_->cont_cap_[other_slot]);
  }

  impl_->ms_layer_idx_++;

  if (impl_->logger != nullptr) {
    ILOG_DEBUG(*impl_->logger, "CudaTraceBackend::Recombine: cont_n={} new_layer={}",
               cont_n, impl_->ms_layer_idx_);
  }

  // backend_ptr is an opaque non-null sanity cookie (continuation rays live in
  // Impl's cont[written_slot]); is_device=true routes the next TraceLayer into
  // the continuation (per-ci transit) branch. When cont_n==0 the next
  // TraceLayer's n==0 guard short-circuits regardless of the cookie.
  DeviceRayBatch batch{};
  batch.backend_ptr = static_cast<void*>(impl_->d_cont_d_[written_slot]);
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
  // Per-ci FilterSpec cache for the final layer (mirrors Metal spec_per_ci,
  // metal:2403-2409). The same crystal_id repeats across many exits; build once.
  const size_t final_ci_cnt = impl_->final_layer_crystals_.size();
  std::vector<std::unique_ptr<FilterSpec>> spec_per_ci(final_ci_cnt);
  for (size_t k = 0; k < final_ci_cnt; ++k) {
    spec_per_ci[k] = FilterSpec::Create(impl_->final_layer_filter_configs_[k],
                                        impl_->final_layer_crystals_[k],
                                        impl_->final_layer_axis_dists_[k]);
  }
  std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);

  out.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    const ExitRayRecord& src = impl_->pinned_exit_[i];
    if (src.ms_layer_idx != final_layer) {
      // Mid-layer exits already filtered+prob-gated in the kernel; emit verbatim.
      out.push_back(src);
      continue;
    }

    // Resolve this exit's final-layer crystal by its per-ci tag (multi-CI).
    const uint16_t cid = src.crystal_id;
    if (cid >= final_ci_cnt) {
      continue;  // safety: stray record from a malformed kernel run
    }
    const Crystal& final_crystal = impl_->final_layer_crystals_[cid];

    // Reconstruct {RaySeg, RaypathRecorder} for FilterSpec::Check.
    RaySeg r{};
    r.d_[0] = src.dir[0];
    r.d_[1] = src.dir[1];
    r.d_[2] = src.dir[2];
    r.w_ = src.weight;
    r.to_face_ = kInvalidId;
    r.is_continue_ = false;
    r.crystal_config_id_ = final_crystal.config_id_;

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
      const uint8_t v = static_cast<uint8_t>(final_crystal.GetFn(static_cast<IdType>(src.path.data_[k])));
      if (k < RaypathRecorder::kInlineCap) {
        rec.data_[k] = v;
      }
      local_arena[k] = v;
    }
    const uint8_t* arena_for_check = has_overflow ? local_arena : nullptr;
    if (spec_per_ci[cid] && !spec_per_ci[cid]->Check(r, rec, arena_for_check)) {
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
  // scrum-cuda-async-engine-port (304.2): keep the large device + pinned
  // buffers allocated across the per-batch session boundary (mirrors Metal's
  // Reset(), metal_trace_backend.mm:1807). The next BeginSession reuses them via
  // the idempotent EnsureSessionBuffers fast-path, eliminating per-batch
  // cudaMalloc + cudaHostAlloc churn. Full teardown happens on the destructor /
  // error paths (Reset() default arg).
  impl_->Reset(/*keep_persistent_buffers=*/true);
}

// S2 device-fused XYZ accumulation: mirrors MetalTraceBackend::HasDeviceXyzAccum.
// Reports `true` so the simulator routes egress through ReadbackXyzAccum (W*H*3
// D2H) instead of the per-exit DrainExits PCIe round-trip that flattened CUDA
// throughput to 0.10–0.12× legacy CPU.
bool CudaTraceBackend::HasDeviceXyzAccum() const { return true; }

// scrum-312 third-clock drain: copies the PERSISTENT cross-batch d_xyz_buf_ +
// d_landed_weight_ accumulator to host, accumulates landed_weight into the running
// scalar, and zeros the device buffers to start the next drain window clean. The
// simulator calls this on display cadence (a whole window of batches), not per
// batch, and possibly BETWEEN sessions. Draining twice with no accumulation in
// between returns zeros on the second call (buffers cleared after the first read).
void CudaTraceBackend::ReadbackXyzAccum(XyzImageData& xyz, float& landed_weight) {
  // scrum-312 (third-clock drain): the XYZ accumulator is persistent and drained
  // on display cadence, which the simulator triggers BETWEEN per-batch sessions
  // (generation-change / producer-pause / run-exit flush). Gate on the buffer
  // being allocated, not on in_session_ — the persisted buffer is valid to read
  // after EndSession(keep_persistent_buffers=true).
  if (impl_->d_xyz_buf_ == nullptr) {
    throw BackendUnavailableError("CudaTraceBackend::ReadbackXyzAccum called before any session allocated the buffer");
  }
  assert(impl_->d_landed_weight_ != nullptr);
  assert(xyz.data != nullptr && "ReadbackXyzAccum: caller must pre-allocate xyz.data");
  // scrum-312 (release-safe, code-review Major): pixel count comes from the dims
  // the persistent buffer was ACTUALLY allocated for (alloc_xyz_w_/h_), NOT from
  // impl_->img_w_/img_h_ (cleared to 0 by EndSession/Reset — reading them here was
  // Bug 1: 0-byte copy → black image). Cross-check the caller's declared dims
  // against the real buffer capacity with a RELEASE-safe throw (not an assert —
  // asserts are no-ops under NDEBUG, which is exactly how Bug 1 stayed silent):
  // a mismatch means the buffer size and the caller's expectation have decoupled
  // (e.g. a resolution change on the persistent buffer), which would corrupt the
  // D2H copy.
  if (static_cast<uint32_t>(xyz.width) != impl_->alloc_xyz_w_ ||
      static_cast<uint32_t>(xyz.height) != impl_->alloc_xyz_h_) {
    throw BackendUnavailableError(
        "CudaTraceBackend::ReadbackXyzAccum: caller dims (" + std::to_string(xyz.width) + "x" +
        std::to_string(xyz.height) + ") != allocated buffer dims (" + std::to_string(impl_->alloc_xyz_w_) +
        "x" + std::to_string(impl_->alloc_xyz_h_) + ")");
  }

  // waitUntilCompleted-equivalent — all preceding TraceLayer kernel work must
  // finalize before the D2H copy. Mirrors Metal's cmd-buffer wait.
  cudaDeviceSynchronize();
  const size_t pix = static_cast<size_t>(impl_->alloc_xyz_w_) * static_cast<size_t>(impl_->alloc_xyz_h_);
  CheckCuda(cudaMemcpy(xyz.data, impl_->d_xyz_buf_, pix * 3u * sizeof(float),
                       cudaMemcpyDeviceToHost),
            "ReadbackXyzAccum D2H d_xyz_buf");
  float lw = 0.0f;
  CheckCuda(cudaMemcpy(&lw, impl_->d_landed_weight_, sizeof(float), cudaMemcpyDeviceToHost),
            "ReadbackXyzAccum D2H d_landed_weight");
  landed_weight += lw;
  // Reset the accumulator so the NEXT drain window starts from zero (scrum-312:
  // this is now the per-window reset — BeginSession no longer zeroes; a second
  // drain with no intervening accumulation returns zeros).
  CheckCuda(cudaMemset(impl_->d_xyz_buf_, 0, pix * 3u * sizeof(float)),
            "ReadbackXyzAccum cudaMemset d_xyz_buf");
  CheckCuda(cudaMemset(impl_->d_landed_weight_, 0, sizeof(float)),
            "ReadbackXyzAccum cudaMemset d_landed_weight");
}

// Mirror MetalTraceBackend::IsCompatible (metal_trace_backend.mm:2426-2437).
// The device-fused emit gate only implements two projections: rectangular @
// zenith view (where ComputeAz0 is meaningful) and dual_fisheye_equal_area @
// the full-globe fov=180 layout. Any other lens config returns false → the
// simulator transparently falls back to legacy CPU. The el≈90° constraint
// reflects that the rectangular projection assumes the camera is looking
// straight up; non-zenith el would require a full rotation pre-multiply in the
// kernel that S2 does not implement.
bool CudaTraceBackend::IsCompatible(const RenderConfig& render) const {
  if (render.lens_.type_ == LensParam::kRectangular) {
    return std::abs(render.view_.el_ - 90.0f) <= 0.01f;
  }
  if (render.lens_.type_ == LensParam::kDualFisheyeEqualArea) {
    // kernel implements fov180 full-globe only; reject custom fov.
    return std::abs(render.lens_.fov_ - 180.0f) <= 0.5f;
  }
  return false;
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

size_t CudaTraceBackend::GetLastBatchCrystalCount() const {
  // task-exit-seam-crystal-count: Impl::final_layer_crystals_ is populated
  // during BeginSession (cuda_trace_backend.cu:2078-2093), so it can be read
  // safely anytime the session is open.
  return impl_->final_layer_crystals_.size();
}

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
