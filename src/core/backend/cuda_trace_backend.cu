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
//   - WlPoolSize()=0; single n_idx per session via SessionSpec::wl.
//   - Multi-MS / filter / device root-gen / WlPool / prob 分流 live in
//     follow-up subtasks.
//
// Build gate: this entire translation unit is added to lumice_obj only when
// LUMICE_CUDA_ENABLED is ON (see CMakeLists.txt). Other backends are
// uninvolved.
//
// Crystal/host-side ray generation re-uses the same `MakeCrystal` /
// `InitRayFirstMs` chain as `CpuTraceBackend::TraceLayer` (cpu_trace_backend.cpp
// 312-325 / 135-140). The kernel is a per-thread single-orientation trace using
// Möller-Trumbore triangle intersection + tri->poly mapping for O(1) from_face
// guard (Metal slab traversal is polygon-level; the two routes converge at the
// exit_dir_world record via frame invariant 6).

#include "core/backend/cuda_trace_backend.hpp"

#if defined(LUMICE_CUDA_ENABLED)

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/exit_seam.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/shared/optics_shared.h"
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

// --- Device-side geometry --------------------------------------------------

__device__ inline float dot3(const float* a, const float* b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

__device__ inline void cross3(const float* a, const float* b, float* out) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// Möller-Trumbore: intersect ray (origin O, dir D) with triangle (v0, v1, v2).
// Returns t > 0 on hit (front + back), -1.0f on miss. No back-face culling —
// the ray sits inside the crystal and may hit faces from either side.
__device__ inline float ray_triangle(const float* O, const float* D, const float* v0, const float* v1,
                                     const float* v2) {
  float e1[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
  float e2[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
  float pvec[3];
  cross3(D, e2, pvec);
  float det = dot3(e1, pvec);
  // Reject near-parallel rays. 1e-8f matches the Metal poly-slab `kFloatEps`
  // sentinel scale; sub-eps determinants produce u/v overflow.
  if (det > -1e-8f && det < 1e-8f) {
    return -1.0f;
  }
  float inv_det = 1.0f / det;
  float tvec[3] = {O[0] - v0[0], O[1] - v0[1], O[2] - v0[2]};
  float u = dot3(tvec, pvec) * inv_det;
  if (u < 0.0f || u > 1.0f) {
    return -1.0f;
  }
  float qvec[3];
  cross3(tvec, e1, qvec);
  float v = dot3(D, qvec) * inv_det;
  if (v < 0.0f || u + v > 1.0f) {
    return -1.0f;
  }
  return dot3(e2, qvec) * inv_det;
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

__global__ void trace_single_ms_kernel(const float* __restrict__ d_dirs,        // 3 × n_roots (crystal-local)
                                       const float* __restrict__ d_pos,         // 3 × n_roots (crystal-local)
                                       const float* __restrict__ d_ws,          // n_roots
                                       const uint32_t* __restrict__ d_from_poly,  // n_roots (entry-face polygon id)
                                       uint32_t n_roots,
                                       const float* __restrict__ d_verts,       // 9 × tri_cnt
                                       const float* __restrict__ d_norms,       // 3 × tri_cnt (outward face normals)
                                       const uint32_t* __restrict__ d_tri_poly_id,  // tri_cnt (tri -> polygon id)
                                       uint32_t tri_cnt,
                                       const float* __restrict__ d_rot_c2w,     // 9 × n_roots, row-major per ray
                                       float n_idx,
                                       uint32_t max_hits,
                                       ExitRayRecord* __restrict__ d_exit,
                                       uint32_t exit_cap,
                                       uint32_t* __restrict__ d_exit_count,
                                       uint16_t crystal_id,
                                       uint8_t ms_layer_idx) {
  const uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= n_roots) {
    return;
  }

  float dir[3] = {d_dirs[tid * 3u + 0u], d_dirs[tid * 3u + 1u], d_dirs[tid * 3u + 2u]};
  float org[3] = {d_pos[tid * 3u + 0u], d_pos[tid * 3u + 1u], d_pos[tid * 3u + 2u]};
  float w = d_ws[tid];

  // Initial from_poly = entry-face polygon id from InitRay_p_fid output.
  // p sits exactly on this polygon's triangle (no epsilon push); without
  // suppressing self-hit the first Möller-Trumbore would return t≈0 against
  // the entry face's triangles. Mirrors the CPU/Metal Propagate which uses
  // `to_face_` as the from_face guard for the first iteration.
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
  // march is needed. We only need its outward normal: scan d_tri_poly_id for
  // the first triangle owned by from_poly (all triangles of a polygon are
  // coplanar → same normal).
  //
  // Cost note: O(tri_cnt) per ray with warp divergence at the break point.
  // MVP crystals have tri_cnt ≈ 8 (prism) so this is negligible; for larger
  // meshes a precomputed poly→first_tri map could eliminate the divergence.
  float entry_nrm[3] = {0.0f, 0.0f, 0.0f};
  bool entry_nrm_found = false;
  for (uint32_t tri = 0u; tri < tri_cnt; ++tri) {
    if (d_tri_poly_id[tri] == from_poly) {
      entry_nrm[0] = d_norms[tri * 3u + 0u];
      entry_nrm[1] = d_norms[tri * 3u + 1u];
      entry_nrm[2] = d_norms[tri * 3u + 2u];
      entry_nrm_found = true;
      break;
    }
  }
  if (entry_nrm_found && w > 0.0f) {
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
      uint32_t slot = atomicAdd(d_exit_count, 1u);
      if (slot < exit_cap) {
        ExitRayRecord& rec = d_exit[slot];
        rec.dir[0] = exit_world[0];
        rec.dir[1] = exit_world[1];
        rec.dir[2] = exit_world[2];
        rec.weight = w_refl_e;
        rec.path = ExitFaceSeq{};
        rec.crystal_id = crystal_id;
        rec.ms_layer_idx = ms_layer_idx;
        rec.wl_idx = 0u;
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
    // Find nearest triangle hit (t > eps) excluding from_poly's triangles.
    float t_min = 1e30f;
    uint32_t hit_tri = 0xFFFFFFFFu;
    for (uint32_t tri = 0u; tri < tri_cnt; ++tri) {
      if (d_tri_poly_id[tri] == from_poly) {
        continue;
      }
      const float* v0 = d_verts + tri * 9u + 0u;
      const float* v1 = d_verts + tri * 9u + 3u;
      const float* v2 = d_verts + tri * 9u + 6u;
      float t = ray_triangle(org, dir, v0, v1, v2);
      if (t > 1e-6f && t < t_min) {
        t_min = t;
        hit_tri = tri;
      }
    }
    if (hit_tri == 0xFFFFFFFFu) {
      // Safety bound — ray escaped without hitting any face. Drop silently
      // (matches the legacy "outgoing with kInvalidId to_face_" path which
      // also zero-emits).
      break;
    }

    uint32_t hit_poly = d_tri_poly_id[hit_tri];

    // Advance origin to hit point.
    org[0] += t_min * dir[0];
    org[1] += t_min * dir[1];
    org[2] += t_min * dir[2];

    float nrm[3] = {d_norms[hit_tri * 3u + 0u], d_norms[hit_tri * 3u + 1u], d_norms[hit_tri * 3u + 2u]};

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
        uint32_t slot = atomicAdd(d_exit_count, 1u);
        if (slot < exit_cap) {
          ExitRayRecord& rec = d_exit[slot];
          rec.dir[0] = exit_world[0];
          rec.dir[1] = exit_world[1];
          rec.dir[2] = exit_world[2];
          rec.weight = w_refr;
          rec.path = ExitFaceSeq{};  // MVP zero-init; path content does not feed G1-G6
          rec.crystal_id = crystal_id;
          rec.ms_layer_idx = ms_layer_idx;
          rec.wl_idx = 0u;
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

  // Crystal geometry — uploaded once per session.
  float* d_verts_ = nullptr;           // 9 × tri_cnt
  float* d_norms_ = nullptr;           // 3 × tri_cnt (outward face normals)
  uint32_t* d_tri_poly_id_ = nullptr;  // tri_cnt
  uint32_t tri_cnt_ = 0;
  uint32_t poly_cnt_ = 0;

  // Per-ray crystal->world rotation buffer (9 floats/ray, row-major
  // Rotation::mat_). Allocated in EnsureSessionBuffers; filled per TraceLayer
  // from InitRayFirstMs all_data[i].crystal_rot_ output. Mirrors
  // metal_trace_backend.mm's per-ray rot upload (frame invariant 6).
  float* d_rot_c2w_ = nullptr;

  // Per-batch root-ray buffers (allocated lazily on first TraceLayer once
  // n_roots is known).
  float* d_dirs_ = nullptr;
  float* d_pos_ = nullptr;
  float* d_ws_ = nullptr;
  uint32_t* d_from_poly_ = nullptr;  // n_roots; per-ray entry-face polygon id

  // Session-level exit pool.
  ExitRayRecord* d_exit_ = nullptr;
  uint32_t* d_exit_count_ = nullptr;
  size_t exit_cap_ = 0;
  uint32_t h_exit_count_ = 0;

  // Pinned staging.
  float* pinned_dirs_ = nullptr;
  float* pinned_pos_ = nullptr;
  float* pinned_ws_ = nullptr;
  uint32_t* pinned_from_poly_ = nullptr;
  float* pinned_rot_c2w_ = nullptr;  // n_roots × 9 (mirrors d_rot_c2w_)
  ExitRayRecord* pinned_exit_ = nullptr;

  RandomNumberGenerator rng_{0u};  // re-seeded in BeginSession from spec.seed

  size_t n_roots_ = 0;

  const SceneConfig* scene_ = nullptr;
  const RenderConfig* render_ = nullptr;
  WlParam wl_{};

  cudaEvent_t ev_start_h2d_{};
  cudaEvent_t ev_end_h2d_{};
  cudaEvent_t ev_end_kernel_{};
  cudaEvent_t ev_end_d2h_{};
  bool events_created_ = false;

  void Reset();
  void EnsureSessionBuffers(size_t n);
};

void CudaTraceBackend::Impl::Reset() {
  // cudaFree / cudaFreeHost on nullptr are no-ops per CUDA spec, so we don't
  // need explicit allocated-flag guards. Errors are intentionally ignored —
  // teardown must not throw.
  cudaFree(d_verts_);
  d_verts_ = nullptr;
  cudaFree(d_norms_);
  d_norms_ = nullptr;
  cudaFree(d_tri_poly_id_);
  d_tri_poly_id_ = nullptr;
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
  cudaFree(d_exit_);
  d_exit_ = nullptr;
  cudaFree(d_exit_count_);
  d_exit_count_ = nullptr;

  cudaFreeHost(pinned_dirs_);
  pinned_dirs_ = nullptr;
  cudaFreeHost(pinned_pos_);
  pinned_pos_ = nullptr;
  cudaFreeHost(pinned_ws_);
  pinned_ws_ = nullptr;
  cudaFreeHost(pinned_from_poly_);
  pinned_from_poly_ = nullptr;
  cudaFreeHost(pinned_rot_c2w_);
  pinned_rot_c2w_ = nullptr;
  cudaFreeHost(pinned_exit_);
  pinned_exit_ = nullptr;

  if (events_created_) {
    cudaEventDestroy(ev_start_h2d_);
    cudaEventDestroy(ev_end_h2d_);
    cudaEventDestroy(ev_end_kernel_);
    cudaEventDestroy(ev_end_d2h_);
    events_created_ = false;
  }

  tri_cnt_ = 0;
  poly_cnt_ = 0;
  exit_cap_ = 0;
  h_exit_count_ = 0;
  n_roots_ = 0;
  buffers_allocated_ = false;
  in_session_ = false;
  scene_ = nullptr;
  render_ = nullptr;
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
  cudaFree(d_rot_c2w_);  d_rot_c2w_ = nullptr;
  cudaFree(d_exit_);     d_exit_ = nullptr;
  cudaFree(d_exit_count_); d_exit_count_ = nullptr;
  cudaFreeHost(pinned_dirs_);     pinned_dirs_ = nullptr;
  cudaFreeHost(pinned_pos_);      pinned_pos_ = nullptr;
  cudaFreeHost(pinned_ws_);       pinned_ws_ = nullptr;
  cudaFreeHost(pinned_from_poly_); pinned_from_poly_ = nullptr;
  cudaFreeHost(pinned_rot_c2w_);  pinned_rot_c2w_ = nullptr;
  cudaFreeHost(pinned_exit_);     pinned_exit_ = nullptr;

  size_t max_hits = scene_ != nullptr ? scene_->max_hits_ : kMaxHits;
  exit_cap_ = ComputeExitCap(n, max_hits);

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

  ck(cudaMalloc(&d_dirs_,      3 * n * sizeof(float)),               "cudaMalloc d_dirs");
  ck(cudaMalloc(&d_pos_,       3 * n * sizeof(float)),               "cudaMalloc d_pos");
  ck(cudaMalloc(&d_ws_,        n * sizeof(float)),                   "cudaMalloc d_ws");
  ck(cudaMalloc(&d_from_poly_, n * sizeof(uint32_t)),                "cudaMalloc d_from_poly");
  ck(cudaMalloc(&d_rot_c2w_,   9 * n * sizeof(float)),               "cudaMalloc d_rot_c2w");
  ck(cudaMalloc(&d_exit_,      exit_cap_ * sizeof(ExitRayRecord)),   "cudaMalloc d_exit");
  ck(cudaMalloc(&d_exit_count_, sizeof(uint32_t)),                   "cudaMalloc d_exit_count");

  ck(cudaHostAlloc(&pinned_dirs_,     3 * n * sizeof(float),               cudaHostAllocDefault), "cudaHostAlloc pinned_dirs");
  ck(cudaHostAlloc(&pinned_pos_,      3 * n * sizeof(float),               cudaHostAllocDefault), "cudaHostAlloc pinned_pos");
  ck(cudaHostAlloc(&pinned_ws_,       n * sizeof(float),                   cudaHostAllocDefault), "cudaHostAlloc pinned_ws");
  ck(cudaHostAlloc(&pinned_from_poly_, n * sizeof(uint32_t),               cudaHostAllocDefault), "cudaHostAlloc pinned_from_poly");
  ck(cudaHostAlloc(&pinned_rot_c2w_,  9 * n * sizeof(float),               cudaHostAllocDefault), "cudaHostAlloc pinned_rot_c2w");
  ck(cudaHostAlloc(&pinned_exit_,     exit_cap_ * sizeof(ExitRayRecord),   cudaHostAllocDefault), "cudaHostAlloc pinned_exit");

  n_roots_ = n;
  buffers_allocated_ = true;
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

    size_t tri_cnt = crystal_for_geom.TotalTriangles();
    size_t poly_cnt = crystal_for_geom.PolygonFaceCount();
    if (tri_cnt == 0 || poly_cnt == 0) {
      throw BackendUnavailableError("CudaTraceBackend::BeginSession: degenerate crystal geometry (0 triangles)");
    }
    impl_->tri_cnt_ = static_cast<uint32_t>(tri_cnt);
    impl_->poly_cnt_ = static_cast<uint32_t>(poly_cnt);

    // Detect stochastic CrystalParam: if a Crystal built with the session
    // seed differs in triangle count from dummy_rng's crystal, the uploaded
    // geometry will diverge from per-batch TraceLayer crystals and G1 parity
    // may fail. MVP dual_fisheye_ref uses deterministic prism/pyramid shapes.
    if (impl_->logger != nullptr) {
      RandomNumberGenerator probe_rng{spec.seed};
      Crystal probe_crystal = MakeCrystal(probe_rng, ms_setting.crystal_.param_);
      if (probe_crystal.TotalTriangles() != tri_cnt) {
        ILOG_WARN(*impl_->logger,
                  "CudaTraceBackend::BeginSession: stochastic CrystalParam detected — "
                  "uploaded geometry may diverge from per-batch TraceLayer crystals; "
                  "G1 parity may fail for non-deterministic crystal shapes");
      }
    }

    // Triangle verts (9 × tri_cnt) + normals (3 × tri_cnt) H2D.
    CheckCuda(cudaMalloc(&impl_->d_verts_, 9 * tri_cnt * sizeof(float)), "BeginSession cudaMalloc d_verts");
    CheckCuda(cudaMemcpy(impl_->d_verts_, crystal_for_geom.GetTriangleVtx(), 9 * tri_cnt * sizeof(float),
                         cudaMemcpyHostToDevice), "BeginSession cudaMemcpy d_verts");
    CheckCuda(cudaMalloc(&impl_->d_norms_, 3 * tri_cnt * sizeof(float)), "BeginSession cudaMalloc d_norms");
    CheckCuda(cudaMemcpy(impl_->d_norms_, crystal_for_geom.GetTriangleNormal(), 3 * tri_cnt * sizeof(float),
                         cudaMemcpyHostToDevice), "BeginSession cudaMemcpy d_norms");

    // tri -> polygon-id reverse map. GetPolygonFaceTriId() returns a per-
    // polygon start-of-triangle-range array (length poly_cnt); each polygon
    // p owns triangles [start_p, start_{p+1}).
    const int* poly_tri_starts = crystal_for_geom.GetPolygonFaceTriId();
    std::vector<uint32_t> h_tri_poly_id(tri_cnt);
    for (uint32_t p = 0; p < impl_->poly_cnt_; ++p) {
      uint32_t start = static_cast<uint32_t>(poly_tri_starts[p]);
      uint32_t end = (p + 1u < impl_->poly_cnt_) ? static_cast<uint32_t>(poly_tri_starts[p + 1u])
                                                 : static_cast<uint32_t>(tri_cnt);
      for (uint32_t t = start; t < end && t < tri_cnt; ++t) {
        h_tri_poly_id[t] = p;
      }
    }
    CheckCuda(cudaMalloc(&impl_->d_tri_poly_id_, tri_cnt * sizeof(uint32_t)), "BeginSession cudaMalloc d_tri_poly_id");
    CheckCuda(cudaMemcpy(impl_->d_tri_poly_id_, h_tri_poly_id.data(), tri_cnt * sizeof(uint32_t),
                         cudaMemcpyHostToDevice), "BeginSession cudaMemcpy d_tri_poly_id");

    // Per-ray rotation matrix device buffer (n_roots × 9) is allocated lazily
    // in EnsureSessionBuffers, not here — n_roots is unknown until the first
    // TraceLayer call. rng_ stays pristine (no sampling here, see plan §D5).

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
                "CudaTraceBackend::BeginSession: device=0 tri_cnt={} poly_cnt={} seed={} n_idx={:.4f}",
                tri_cnt, poly_cnt, spec.seed, crystal_for_geom.GetRefractiveIndex(spec.wl.wl_));
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
  if (roots.is_device) {
    throw BackendUnavailableError("CudaTraceBackend::TraceLayer: device root source unsupported in MVP");
  }
  size_t n = roots.host.count;
  if (n == 0) {
    return std::make_unique<CudaLayerHandle>(0u, LayerStats{0u, 0.0f});
  }

  impl_->EnsureSessionBuffers(n);

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

  // Host-side root-ray generation, mirroring cpu_trace_backend.cpp:135-140.
  // Build a fresh Crystal per batch via MakeCrystal(rng_, param_) so the
  // crystal-orientation RNG draw stays consistent with the CPU oracle's
  // batch-by-batch ordering. Wrapped in try/catch: std::bad_alloc from
  // RayBuffer growth or other C++ exceptions reset device state before
  // propagating (mirrors BeginSession's exception-safety contract).
  const auto& ms_setting = impl_->scene_->ms_[0].setting_[0];
  float n_idx = 0.0f;
  try {
    Crystal crystal = MakeCrystal(impl_->rng_, ms_setting.crystal_.param_);
    const auto& axis_dist = ms_setting.crystal_.axis_;
    n_idx = crystal.GetRefractiveIndex(impl_->wl_.wl_);

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
    InitRayFirstMs(impl_->rng_, impl_->scene_->light_source_.param_, impl_->wl_, n, crystal, 0, axis_dist,
                   workspace, all_data);

    // Copy crystal-local d/p/w + per-ray crystal->world rotation into pinned
    // staging. to_face_ is the InitRay_p_fid entry-face polygon id; the
    // kernel needs it as the initial from_poly to suppress self-intersect on
    // the first Möller-Trumbore sweep (p sits exactly on this polygon's
    // triangle). IdType (uint16_t per raypath.hpp) widens cleanly to uint32_t;
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
      impl_->pinned_ws_[i] = r.w_;
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
  cudaEventRecord(impl_->ev_start_h2d_);
  cudaMemcpyAsync(impl_->d_rot_c2w_, impl_->pinned_rot_c2w_, 9 * n * sizeof(float),
                  cudaMemcpyHostToDevice);
  cudaMemcpyAsync(impl_->d_dirs_, impl_->pinned_dirs_, 3 * n * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpyAsync(impl_->d_pos_, impl_->pinned_pos_, 3 * n * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpyAsync(impl_->d_ws_, impl_->pinned_ws_, n * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpyAsync(impl_->d_from_poly_, impl_->pinned_from_poly_, n * sizeof(uint32_t),
                  cudaMemcpyHostToDevice);
  cudaEventRecord(impl_->ev_end_h2d_);
  // Harvest any sticky async error from the H2D block (cudaMemcpyAsync /
  // cudaEventRecord failures propagate as sticky errors; one GetLastError
  // picks all of them up at lower call overhead than per-call checking).
  ck_reset(cudaGetLastError(), "H2D batch");

  ck_reset(cudaMemset(impl_->d_exit_count_, 0, sizeof(uint32_t)), "cudaMemset d_exit_count");

  uint32_t max_hits = static_cast<uint32_t>(impl_->scene_->max_hits_);
  uint32_t grid = (static_cast<uint32_t>(n) + 255u) / 256u;

  // Splitting (deterministic per-bounce fan-out) — no per-thread RNG needed.
  trace_single_ms_kernel<<<grid, 256>>>(impl_->d_dirs_, impl_->d_pos_, impl_->d_ws_, impl_->d_from_poly_,
                                        static_cast<uint32_t>(n), impl_->d_verts_, impl_->d_norms_,
                                        impl_->d_tri_poly_id_, impl_->tri_cnt_, impl_->d_rot_c2w_, n_idx,
                                        max_hits, impl_->d_exit_,
                                        static_cast<uint32_t>(impl_->exit_cap_), impl_->d_exit_count_,
                                        /*crystal_id=*/0u, /*ms_layer_idx=*/0u);
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
               "CudaTraceBackend::TraceLayer: n={} exit_count={} H2D={:.2f}ms kernel={:.2f}ms D2H={:.2f}ms",
               n, impl_->h_exit_count_, h2d_ms, kernel_ms, d2h_ms);
  }

  if (impl_->h_exit_count_ >= impl_->exit_cap_) {
    if (impl_->logger != nullptr) {
      ILOG_WARN(*impl_->logger,
                "CudaTraceBackend::TraceLayer: exit buffer overflow (count={} cap={}); tail dropped",
                impl_->h_exit_count_, impl_->exit_cap_);
    }
    impl_->h_exit_count_ = static_cast<uint32_t>(impl_->exit_cap_);
  }

  return std::make_unique<CudaLayerHandle>(0u, LayerStats{impl_->h_exit_count_, 0.0f});
}

RootRaySource CudaTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  (void)handle;
  (void)spec;
  // MVP single-MS: Recombine never feeds back into TraceLayer (the simulator
  // single-MS path drains exits and stops). Return an empty DeviceRayBatch.
  return RootRaySource::FromDevice(DeviceRayBatch{});
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
  out.assign(impl_->pinned_exit_, impl_->pinned_exit_ + count);
  auto t1 = std::chrono::steady_clock::now();
  double drain_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  if (impl_->logger != nullptr) {
    ILOG_DEBUG(*impl_->logger,
               "CudaTraceBackend::DrainExits: count={} bytes={} D2H={:.2f}ms",
               count, count * sizeof(ExitRayRecord), drain_ms);
  }

  // Reset counter for the next TraceLayer call.
  cudaMemset(impl_->d_exit_count_, 0, sizeof(uint32_t));
  impl_->h_exit_count_ = 0u;
  return count;
}

void CudaTraceBackend::EndSession() {
  if (!impl_->in_session_) {
    return;
  }
  cudaDeviceSynchronize();
  impl_->Reset();
}

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
