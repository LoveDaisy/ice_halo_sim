// Single-source pure-math Fresnel reflection ratio.
// Shared by host C++ (src/core/optics.cpp), MSL kernel
// (src/core/metal/lumice_trace.metal), and (future) CUDA kernel.
//
// Surface contract: scalar value semantics only.
#ifndef LM_OPTICS_SHARED_H_
#define LM_OPTICS_SHARED_H_

#include "lm_shims.h"

namespace lm_optics {

// Unpolarized Fresnel reflection ratio.
//   delta = (1 - rr^2)/cos^2(theta) + rr^2  (positive => transmitted; <=0 => TIR)
//   rr    = relative refractive index along the ray direction
// Caller is responsible for clamping delta >= 0 before invoking.
LM_FN float GetReflectRatio(float delta, float rr) {
  float d_sqrt = LM_SQRT(delta);
  float Rs = (rr - d_sqrt) / (rr + d_sqrt);  // NOLINT(readability-identifier-naming) Fresnel notation
  Rs *= Rs;
  float Rp = (1.0f - rr * d_sqrt) / (1.0f + rr * d_sqrt);  // NOLINT(readability-identifier-naming) Fresnel notation
  Rp *= Rp;
  return (Rs + Rp) * 0.5f;
}

}  // namespace lm_optics

#endif  // LM_OPTICS_SHARED_H_
