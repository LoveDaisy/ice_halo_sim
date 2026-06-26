// Single-source pure-math polygon-slab face traversal.
// Shared by host C++ (src/core/optics.cpp PropagateSlab), MSL kernel
// (src/core/metal/lumice_trace.metal trace_layer_kernel) and CUDA kernel
// (src/core/backend/cuda_trace_backend.cu trace_single_ms_kernel).
//
// Background: scrum-#295.7 fixed a Möller-Trumbore + absolute-ε face-miss bug
// in the CUDA path by porting to legacy `PropagateSlab` (optics.cpp). The
// fix lived as three independent inline copies across CUDA/Metal/CPU; this
// header collapses them to a single source so the absolute-ε anti-pattern
// (doc/numerical-robustness.md约定 2) cannot silently regress in any backend.
//
// Surface contract: scalar value semantics only. No pointer or address-space
// parameters — caller reads face data (normal + plane constant) and passes
// scalars. See lm_shims.h for the cross-language qualifier shims.
//
// Epsilon coupling (kept in sync by convention, not by mechanism):
//   - This header's `kSlabEps` (1e-5f) is the denominator gate inside SlabFaceT.
//   - Metal `kFloatEps` (lumice_trace.metal:315) and CPU `math::kFloatEps`
//     (core/math.hpp) are numerically equal (1e-5f) and continue to drive each
//     backend's post-loop accept threshold (`eps_thr`). If this value is ever
//     re-tuned, **all three constants must move together**.
//
// From-face exclusion strategy (caller responsibility):
//   - CUDA uses explicit `fi == from_poly` skip inside the slab loop.
//   - Metal/CPU use the post-loop `eps_thr` relaxed threshold for the source
//     face. Both strategies are equivalent **only on convex crystals** (where
//     a ray leaving its source face cannot legitimately re-hit any face). If
//     a non-convex crystal type is ever wired through, this equivalence
//     breaks; the caller must then re-verify its exclusion strategy.
//
// Convex-crystal invariant: the face with the minimum valid t (denom > eps)
// is guaranteed to be the exit face. There is always at least one such face
// for a ray strictly inside the convex hull.
#ifndef LM_TRAVERSAL_SHARED_H_
#define LM_TRAVERSAL_SHARED_H_

#include "lm_shims.h"

namespace lm_traversal {

// Denominator/face gate epsilon. Float32-loose value chosen to absorb
// near-parallel rounding without rejecting any legitimate exit face on the
// configured crystal scales. See header comment for cross-backend coupling.
LM_CONSTANT float kSlabEps = 1e-5f;

// Per-face polygon-slab intersection t value.
//   Returns t if the ray is leaving this face's half-space
//     (dir·normal > kSlabEps), where t = -(org·n + fd) / (dir·n).
//   Returns 1e30f otherwise (face is parallel to the ray or the ray is
//     entering its half-space — not a candidate exit face).
// The 1e30f sentinel makes `if (t < t_best)` naturally skip non-candidates
// without requiring a separate `denom > eps` check at the call site.
//
// Caller responsibilities:
//   - from-face exclusion (see header note on convex-crystal equivalence)
//   - min-t tracking across faces
//   - post-loop accept threshold (e.g. eps_thr relaxation for TIR-edge cases)
// 10 scalar params (3 dir + 3 origin + 3 normal + plane const) — scalar-only
// contract precludes a struct wrapper, so silence the size linter explicitly.
// NOLINTNEXTLINE(readability-function-size)
LM_FN float SlabFaceT(float dx, float dy, float dz,  // ray direction
                      float px, float py, float pz,  // ray origin
                      float nx, float ny, float nz,  // face outward normal
                      float fd) {                    // plane constant (p·n + fd = 0)
  float denom = dx * nx + dy * ny + dz * nz;
  if (denom <= kSlabEps) {
    return 1.0e30f;
  }
  return -(px * nx + py * ny + pz * nz + fd) / denom;
}

}  // namespace lm_traversal

#endif  // LM_TRAVERSAL_SHARED_H_
