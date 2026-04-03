#include "core/projection.hpp"

#include <algorithm>
#include <cmath>

#include "core/math.hpp"

namespace lumice {
namespace projection {

// =============== Linear (perspective) ===============

ProjXY LinearForward(float dx, float dy, float dz) {
  if (dz <= 0) {
    return { 0, 0, false };
  }
  return { dx / dz, dy / dz, true };
}

Dir3 LinearInverse(float x, float y) {
  float inv_len = 1.0f / std::sqrt(x * x + y * y + 1.0f);
  return { x * inv_len, y * inv_len, inv_len, true };
}


// =============== Fisheye forward projections (pure math) ===============
// All three use Cartesian form: output = (scale * dx, scale * dy).
// No atan2/cos(az)/sin(az) polar decomposition — Cartesian is equivalent and faster.
// r_scale multiplies the projection core to control coverage boundary (r=1).

// Equal-area: k = r_scale / sqrt(1 + dz). No trig, 1 sqrt.
ProjXY FisheyeEqualAreaForward(float dx, float dy, float dz, float r_scale) {
  float k = r_scale / std::sqrt(1.0f + std::clamp(dz, -1.0f + 1e-6f, 1.0f));
  return { k * dx, k * dy, true };
}

// Equidistant: scale = r_scale * theta / (pi/2 * rho). theta = acos(dz).
ProjXY FisheyeEquidistantForward(float dx, float dy, float dz, float r_scale) {
  float rho = std::sqrt(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0, 0, true };  // pole
  }
  float theta = std::acos(std::clamp(dz, -1.0f, 1.0f));
  float scale = r_scale * theta / (math::kPi_2 * rho);
  return { scale * dx, scale * dy, true };
}

// Stereographic: scale = r_scale * tan(theta/2) / rho. theta = acos(dz).
ProjXY FisheyeStereographicForward(float dx, float dy, float dz, float r_scale) {
  float rho = std::sqrt(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0, 0, true };  // pole
  }
  float theta = std::acos(std::clamp(dz, -1.0f, 1.0f));
  float scale = r_scale * std::tan(theta / 2.0f) / rho;
  return { scale * dx, scale * dy, true };
}


// =============== Fisheye inverse projections (pure math) ===============
// Domain check: r > 1 in the r_scale-normalized space = beyond coverage boundary.
// When r_scale < 1, inverse may return z < 0 (past-equator direction) — this is correct.

// Equal-area inverse: direct Cartesian formula (1 sqrt, no trig).
Dir3 FisheyeEqualAreaInverse(float x, float y, float r_scale) {
  float r2 = x * x + y * y;
  if (r2 > 1.0f) {
    return { 0, 0, 0, false };
  }
  float inv_s = 1.0f / r_scale;
  float xr = x * inv_s;
  float yr = y * inv_s;
  float z = 1.0f - xr * xr - yr * yr;
  float factor = std::sqrt(1.0f + z);
  return { factor * xr, factor * yr, z, true };
}

// Equidistant inverse: theta = r / r_scale * pi/2.
Dir3 FisheyeEquidistantInverse(float x, float y, float r_scale) {
  float r = std::sqrt(x * x + y * y);
  if (r > 1.0f) {
    return { 0, 0, 0, false };
  }
  if (r < 1e-10f) {
    return { 0, 0, 1, true };  // pole
  }
  float theta = r / r_scale * math::kPi_2;
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float inv_r = 1.0f / r;
  return { st * x * inv_r, st * y * inv_r, ct, true };
}

// Stereographic inverse: theta = 2 * atan(r / r_scale).
Dir3 FisheyeStereographicInverse(float x, float y, float r_scale) {
  float r = std::sqrt(x * x + y * y);
  if (r > 1.0f) {
    return { 0, 0, 0, false };
  }
  if (r < 1e-10f) {
    return { 0, 0, 1, true };  // pole
  }
  float theta = 2.0f * std::atan(r / r_scale);
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float inv_r = 1.0f / r;
  return { st * x * inv_r, st * y * inv_r, ct, true };
}


// =============== Rectangular ===============

ProjXY RectangularForward(float dx, float dy, float dz) {
  float lon = std::atan2(dy, dx);
  float lat = std::asin(std::clamp(dz, -1.0f, 1.0f));
  return { lon, lat, true };
}

Dir3 RectangularInverse(float lon, float lat) {
  float cl = std::cos(lat);
  return { cl * std::cos(lon), cl * std::sin(lon), std::sin(lat), true };
}


// =============== Dual fisheye layout utilities ===============

void DualFisheyeToPixel(float x_norm, float y_norm, bool is_upper, int width, int height, float* fx, float* fy) {
  int short_res = std::min(width / 2, height);
  float r = short_res / 2.0f;
  float cy = height / 2.0f;

  if (is_upper) {
    // Upper hemisphere = left circle. 90 deg CW rotation: (x,y) -> (-y, x)
    float cx = width / 2.0f - r;
    *fx = -y_norm * r + cx;
    *fy = x_norm * r + cy;
  } else {
    // Lower hemisphere = right circle. 90 deg CCW + X mirror: (x,y) -> (y, x)
    float cx = width / 2.0f + r;
    *fx = y_norm * r + cx;
    *fy = x_norm * r + cy;
  }
}

bool PixelToDualFisheye(float fx, float fy, int width, int height, float* x_norm, float* y_norm, bool* is_upper) {
  int short_res = std::min(width / 2, height);
  float r = short_res / 2.0f;
  float cy = height / 2.0f;
  float cx_left = width / 2.0f - r;
  float cx_right = width / 2.0f + r;

  float dx_left = fx - cx_left;
  float dy = fy - cy;
  float dist_left_sq = dx_left * dx_left + dy * dy;

  if (dist_left_sq <= r * r) {
    // In left circle (upper hemisphere). Inverse of 90 deg CW: (-y, x) -> (x, y)
    *is_upper = true;
    *x_norm = dy / r;
    *y_norm = -dx_left / r;
    return true;
  }

  float dx_right = fx - cx_right;
  float dist_right_sq = dx_right * dx_right + dy * dy;

  if (dist_right_sq <= r * r) {
    // In right circle (lower hemisphere). Inverse of (y, x): (x, y) -> (y, x)
    *is_upper = false;
    *x_norm = dy / r;
    *y_norm = dx_right / r;
    return true;
  }

  return false;  // outside both circles
}

}  // namespace projection
}  // namespace lumice
