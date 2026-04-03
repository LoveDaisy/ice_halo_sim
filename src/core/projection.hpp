#ifndef CORE_PROJECTION_H_
#define CORE_PROJECTION_H_

namespace lumice {
namespace projection {

// =============== Return types ===============

// Forward projection result: (x, y) coordinates + validity flag.
// For fisheye projections, valid is always true (pure math, no rejection).
// For LinearForward, valid = false when direction is behind camera (dz <= 0).
struct ProjXY {
  float x, y;
  bool valid;
};

// Inverse projection result: a 3D unit direction.
struct Dir3 {
  float x, y, z;
  bool valid;  // false if input is outside the projection domain
};


// =============== Linear (perspective) projection ===============
// The only projection with built-in validity rejection (dz <= 0 = behind camera).

ProjXY LinearForward(float dx, float dy, float dz);
Dir3 LinearInverse(float x, float y);


// =============== Fisheye projections (pure math) ===============
// Input: unit direction vector (dx, dy, dz) where dz is the component along the pole axis.
//   dz > 0: same hemisphere as the pole (normal range)
//   dz < 0: past the equator (used for overlap extension)
//   dz = -1: antipodal singularity (clamped internally for EA; acos-safe for ED/ST)
//   Precondition: |(dx, dy, dz)| = 1
//
// r_scale: projection scaling factor (default 1.0).
//   r = 1 in output space represents the "coverage boundary", not a fixed physical angle.
//   r_scale = 1.0: r=1 at equator (theta=pi/2), standard hemisphere coverage.
//   r_scale < 1.0: r=1 beyond equator, extended coverage (e.g. overlap).
//
// The function does NOT know what the pole axis represents physically:
//   - For dual fisheye: pole = zenith/nadir, caller flips z per hemisphere
//   - For single fisheye lens: pole = camera optical axis, caller does world-to-camera rotation
//
// Output: normalized projection coordinates. r=1 at the coverage boundary defined by r_scale.
//   Always valid (no dz <= 0 rejection — caller handles validity).

ProjXY FisheyeEqualAreaForward(float dx, float dy, float dz, float r_scale = 1.0f);
ProjXY FisheyeEquidistantForward(float dx, float dy, float dz, float r_scale = 1.0f);
ProjXY FisheyeStereographicForward(float dx, float dy, float dz, float r_scale = 1.0f);


// =============== Fisheye inverse projections (pure math) ===============
// Input: normalized projection coordinates (x, y) + r_scale.
//   Domain: r = sqrt(x^2 + y^2) <= 1 (in r_scale-normalized space).
//   r > 1 returns valid = false (beyond coverage boundary).
//
// Output: unit direction vector (dx, dy, dz).
//   z may be negative when r_scale < 1 (past-equator direction) — this is correct behavior.
//   Caller flips z for lower hemisphere if needed.

Dir3 FisheyeEqualAreaInverse(float x, float y, float r_scale = 1.0f);
Dir3 FisheyeEquidistantInverse(float x, float y, float r_scale = 1.0f);
Dir3 FisheyeStereographicInverse(float x, float y, float r_scale = 1.0f);


// =============== Rectangular (equirectangular) projection ===============
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
