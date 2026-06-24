// Single-source pure-math projection forward functions.
// Shared by host C++ (src/core/projection.cpp), MSL kernel
// (src/core/metal/lumice_trace.metal), and (future) CUDA kernel.
//
// Surface contract: scalar value semantics only. Caller owns
// pixel-layout and visibility-rejection concerns.
//
// `lm_proj::ProjXY` is a transition-stage struct that intentionally mirrors
// `lumice::projection::ProjXY` (same fields, same order, trivially copyable).
// CPU wrappers copy the result field-by-field at the boundary. A future
// consolidation step (post CUDA backend MVP) may collapse the two into one.
#ifndef LM_PROJ_SHARED_H_
#define LM_PROJ_SHARED_H_

#include "lm_shims.h"

namespace lm_proj {

struct ProjXY {
  float x;
  float y;
  bool valid;
};

// Equal-area fisheye forward: k = r_scale / sqrt(1 + dz).
// Mirrors projection.cpp:FisheyeEqualAreaForward and lumice_trace.metal :736.
LM_FN ProjXY FisheyeEqualAreaForward(float dx, float dy, float dz, float r_scale) {
  float k = r_scale / LM_SQRT(1.0f + LM_CLAMP(dz, -1.0f + 1e-6f, 1.0f));
  return { k * dx, k * dy, true };
}

// Equidistant fisheye forward: scale = r_scale * theta / (pi/2 * rho).
LM_FN ProjXY FisheyeEquidistantForward(float dx, float dy, float dz, float r_scale) {
  float rho = LM_SQRT(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0.0f, 0.0f, true };
  }
  float theta = LM_ACOS(LM_CLAMP(dz, -1.0f, 1.0f));
  float scale = r_scale * theta / (LM_PI_2F * rho);
  return { scale * dx, scale * dy, true };
}

// Stereographic fisheye forward: scale = r_scale * tan(theta/2) / rho.
LM_FN ProjXY FisheyeStereographicForward(float dx, float dy, float dz, float r_scale) {
  float rho = LM_SQRT(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0.0f, 0.0f, true };
  }
  float theta = LM_ACOS(LM_CLAMP(dz, -1.0f, 1.0f));
  float scale = r_scale * LM_TAN(theta / 2.0f) / rho;
  return { scale * dx, scale * dy, true };
}

// Orthographic fisheye forward: r = sin(theta). Rejects dz < 0 (aliasing).
LM_FN ProjXY FisheyeOrthographicForward(float dx, float dy, float dz, float r_scale) {
  if (dz < 0.0f) {
    return { 0.0f, 0.0f, false };
  }
  return { r_scale * dx, r_scale * dy, true };
}

// Equirectangular forward: x = atan2(dy, dx), y = asin(dz).
LM_FN ProjXY RectangularForward(float dx, float dy, float dz) {
  float lon = LM_ATAN2(dy, dx);
  float lat = LM_ASIN(LM_CLAMP(dz, -1.0f, 1.0f));
  return { lon, lat, true };
}

}  // namespace lm_proj

#endif  // LM_PROJ_SHARED_H_
