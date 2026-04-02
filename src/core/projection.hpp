#ifndef CORE_PROJECTION_H_
#define CORE_PROJECTION_H_

namespace lumice {
namespace projection {

// =============== Return types ===============

// Forward projection result for single-hemisphere projections (Type A: Linear, Fisheye)
// and equirectangular (Type C: Rectangular).
struct ProjXY {
  float x, y;
  bool valid;  // false if direction is behind camera (Type A) or out of range
};

// Forward projection result for dual fisheye projections (Type B).
// Coordinates are normalized: r = 1 at the equator (theta = pi/2).
struct DualProjXY {
  float x, y;     // normalized disc coordinates
  bool is_upper;  // true = upper hemisphere (sky direction dz >= 0)
};

// Inverse projection result: a 3D unit direction.
struct Dir3 {
  float x, y, z;
  bool valid;  // false if input is outside the projection domain (e.g., beyond circle edge)
};


// =============== Type A: Single-hemisphere forward projections ===============
// Input: camera-space unit direction (dz > 0 = in front of camera).
// Output: raw (x, y) in projection-specific units (NOT pixel coordinates).
// The caller (render.cpp glue) handles: visible_range check, world-to-camera rotation,
// FOV-dependent scaling, pixel center offset, and lens_shift.
//
// Precondition: |d| = 1. Internal clamp(-1, 1) on trig inputs for NaN defense.

// Linear (perspective): x = dx/dz, y = dy/dz
ProjXY LinearForward(float dx, float dy, float dz);

// Equal-area fisheye: r = sin(theta/2), theta = pi/2 - asin(dz)
ProjXY FisheyeEqualAreaForward(float dx, float dy, float dz);

// Equidistant fisheye: r = theta, theta = pi/2 - asin(dz)
ProjXY FisheyeEquidistantForward(float dx, float dy, float dz);

// Stereographic fisheye: r = tan(theta/2), theta = pi/2 - asin(dz)
ProjXY FisheyeStereographicForward(float dx, float dy, float dz);


// =============== Type A: Single-hemisphere inverse projections ===============
// Input: raw (x, y) in the same space as forward output.
// Output: camera-space unit direction (dz > 0).

Dir3 LinearInverse(float x, float y);
Dir3 FisheyeEqualAreaInverse(float x, float y);
Dir3 FisheyeEquidistantInverse(float x, float y);
Dir3 FisheyeStereographicInverse(float x, float y);


// =============== Type B: Dual fisheye forward projections ===============
// Input: sky-space unit direction (NO camera rotation applied).
//   is_upper is determined by dz: dz >= 0 → upper hemisphere, dz < 0 → lower.
// Output: normalized disc coordinates where r = 1 at the equator (theta = pi/2).
//   Always valid (full-globe coverage).
//
// GLSL equivalent for DualFisheyeEqualArea (for shader porting):
//   vec2 dualFisheyeEAForward(vec3 d) {
//     float z_abs = abs(d.z);
//     float k = 1.0 / sqrt(1.0 + z_abs);   // normalized: r=1 at equator
//     return vec2(k * d.x, k * d.y);
//     // is_upper = (d.z >= 0.0)
//   }

DualProjXY DualFisheyeEqualAreaForward(float dx, float dy, float dz);
DualProjXY DualFisheyeEquidistantForward(float dx, float dy, float dz);
DualProjXY DualFisheyeStereographicForward(float dx, float dy, float dz);


// =============== Type B: Dual fisheye inverse projections ===============
// Input: normalized disc coordinates + hemisphere flag.
// Output: sky-space unit direction.
//   valid = false if (x, y) is outside the projection domain.
//
// Domain constraints (normalized):
//   EA: r^2 <= 1   (equator at r=1)
//   ED: r <= 1     (equator at r=1)
//   ST: r <= 1     (equator at r=1, theta in [0, pi/2])
//
// GLSL equivalent for DualFisheyeEqualArea inverse:
//   vec3 dualFisheyeEAInverse(vec2 p, bool isUpper) {
//     float r2 = dot(p, p);
//     if (r2 > 1.0) return vec3(0.0);  // invalid
//     float z_abs = 1.0 - r2;
//     float factor = sqrt(1.0 + z_abs);
//     return vec3(factor * p.x, factor * p.y, isUpper ? z_abs : -z_abs);
//   }

Dir3 DualFisheyeEqualAreaInverse(float x, float y, bool is_upper);
Dir3 DualFisheyeEquidistantInverse(float x, float y, bool is_upper);
Dir3 DualFisheyeStereographicInverse(float x, float y, bool is_upper);


// =============== Type C: Rectangular (equirectangular) projection ===============
// Input: sky-space unit direction (NO camera rotation).
// Output: (lon, lat) where lon in [-pi, pi], lat in [-pi/2, pi/2].
//   Always valid (full-globe coverage).
//   The caller handles: az0 subtraction, scaling, lon wrapping, pixel offset.

ProjXY RectangularForward(float dx, float dy, float dz);

// Input: (lon, lat). Output: sky-space unit direction.
Dir3 RectangularInverse(float lon, float lat);


// =============== Dual fisheye texture layout utilities ===============
// These convert between normalized disc coordinates and continuous pixel coordinates
// for the left-right dual fisheye layout (upper=left circle, lower=right circle).
//
// The layout applies a 90-degree rotation and hemisphere-dependent mirroring
// to match the existing render.cpp pixel convention:
//   Upper (left):  px = -y_norm * R + cx_left,  py = x_norm * R + cy
//   Lower (right): px =  y_norm * R + cx_right, py = x_norm * R + cy
// where R = short_res / 2, short_res = min(width/2, height).
//
// NOTE: These are NOT pure projection math — they encode pixel layout conventions.

// Normalized disc coords → continuous pixel coordinates.
// Caller uses floor(fx + 0.5), floor(fy + 0.5) for integer scatter indices.
void DualFisheyeToPixel(float x_norm, float y_norm, bool is_upper, int width, int height, float* fx, float* fy);

// Continuous pixel coordinates → normalized disc coords + hemisphere.
// Returns false if the pixel is outside both circles.
// For gather: pass (px + 0.5f, py + 0.5f) to convert integer pixel to continuous center.
bool PixelToDualFisheye(float fx, float fy, int width, int height, float* x_norm, float* y_norm, bool* is_upper);

}  // namespace projection
}  // namespace lumice

#endif  // CORE_PROJECTION_H_
