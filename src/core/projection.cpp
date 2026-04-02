#include "core/projection.hpp"

#include <algorithm>
#include <cmath>

#include "core/math.hpp"

namespace lumice {
namespace projection {

// =============== Type A: Single-hemisphere forward ===============

ProjXY LinearForward(float dx, float dy, float dz) {
  if (dz <= 0) {
    return { 0, 0, false };
  }
  return { dx / dz, dy / dz, true };
}

ProjXY FisheyeEqualAreaForward(float dx, float dy, float dz) {
  if (dz <= 0) {
    return { 0, 0, false };
  }
  float az = std::atan2(dy, dx);
  float theta = math::kPi_2 - std::asin(std::clamp(dz, -1.0f, 1.0f));
  float r = std::sin(theta / 2.0f);
  return { r * std::cos(az), r * std::sin(az), true };
}

ProjXY FisheyeEquidistantForward(float dx, float dy, float dz) {
  if (dz <= 0) {
    return { 0, 0, false };
  }
  float az = std::atan2(dy, dx);
  float theta = math::kPi_2 - std::asin(std::clamp(dz, -1.0f, 1.0f));
  float r = theta;
  return { r * std::cos(az), r * std::sin(az), true };
}

ProjXY FisheyeStereographicForward(float dx, float dy, float dz) {
  if (dz <= 0) {
    return { 0, 0, false };
  }
  float az = std::atan2(dy, dx);
  float theta = math::kPi_2 - std::asin(std::clamp(dz, -1.0f, 1.0f));
  float r = std::tan(theta / 2.0f);
  return { r * std::cos(az), r * std::sin(az), true };
}


// =============== Type A: Single-hemisphere inverse ===============

Dir3 LinearInverse(float x, float y) {
  float inv_len = 1.0f / std::sqrt(x * x + y * y + 1.0f);
  return { x * inv_len, y * inv_len, inv_len, true };
}

Dir3 FisheyeEqualAreaInverse(float x, float y) {
  float r = std::sqrt(x * x + y * y);
  if (r > 1.0f) {
    return { 0, 0, 0, false };
  }
  if (r < 1e-10f) {
    return { 0, 0, 1, true };  // on-axis
  }
  float theta = 2.0f * std::asin(std::clamp(r, 0.0f, 1.0f));
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float inv_r = 1.0f / r;
  return { st * x * inv_r, st * y * inv_r, ct, true };
}

Dir3 FisheyeEquidistantInverse(float x, float y) {
  float r = std::sqrt(x * x + y * y);
  if (r > math::kPi_2) {
    return { 0, 0, 0, false };
  }
  if (r < 1e-10f) {
    return { 0, 0, 1, true };
  }
  float theta = r;
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float inv_r = 1.0f / r;
  return { st * x * inv_r, st * y * inv_r, ct, true };
}

Dir3 FisheyeStereographicInverse(float x, float y) {
  float r = std::sqrt(x * x + y * y);
  if (r < 1e-10f) {
    return { 0, 0, 1, true };
  }
  float theta = 2.0f * std::atan(r);
  if (theta > math::kPi_2) {
    return { 0, 0, 0, false };
  }
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float inv_r = 1.0f / r;
  return { st * x * inv_r, st * y * inv_r, ct, true };
}


// =============== Type B: Dual fisheye forward ===============

// Equal-area: direct Cartesian formula (1 sqrt, no trig).
// Normalized so r = 1 at the equator (theta = pi/2).
DualProjXY DualFisheyeEqualAreaForward(float dx, float dy, float dz) {
  float z_abs = std::abs(dz);
  // k = 1/sqrt(1 + |z|): normalized Lambert azimuthal equal-area
  float k = 1.0f / std::sqrt(1.0f + z_abs);
  return { k * dx, k * dy, dz >= 0 };
}

// Equidistant: r = theta / (pi/2), normalized to r = 1 at equator.
DualProjXY DualFisheyeEquidistantForward(float dx, float dy, float dz) {
  float z_abs = std::abs(dz);
  float theta = math::kPi_2 - std::asin(std::clamp(z_abs, -1.0f, 1.0f));
  float r_norm = theta / math::kPi_2;  // normalized: r=1 at equator (theta=pi/2)
  float rho = std::sqrt(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0, 0, dz >= 0 };  // at pole
  }
  float scale = r_norm / rho;
  return { scale * dx, scale * dy, dz >= 0 };
}

// Stereographic: r = tan(theta/2), already r=1 at equator (tan(pi/4)=1).
DualProjXY DualFisheyeStereographicForward(float dx, float dy, float dz) {
  float z_abs = std::abs(dz);
  float theta = math::kPi_2 - std::asin(std::clamp(z_abs, -1.0f, 1.0f));
  float r_norm = std::tan(theta / 2.0f);  // already normalized: tan(pi/4) = 1
  float rho = std::sqrt(dx * dx + dy * dy);
  if (rho < 1e-10f) {
    return { 0, 0, dz >= 0 };  // at pole
  }
  float scale = r_norm / rho;
  return { scale * dx, scale * dy, dz >= 0 };
}


// =============== Type B: Dual fisheye inverse ===============

// Equal-area: direct Cartesian formula (1 sqrt, no trig).
Dir3 DualFisheyeEqualAreaInverse(float x, float y, bool is_upper) {
  float r2 = x * x + y * y;
  if (r2 > 1.0f) {
    return { 0, 0, 0, false };
  }
  float z_abs = 1.0f - r2;
  float factor = std::sqrt(1.0f + z_abs);  // = sqrt(2 - r2)
  return { factor * x, factor * y, is_upper ? z_abs : -z_abs, true };
}

// Equidistant: theta = r * (pi/2)
Dir3 DualFisheyeEquidistantInverse(float x, float y, bool is_upper) {
  float r = std::sqrt(x * x + y * y);
  if (r > 1.0f) {
    return { 0, 0, 0, false };
  }
  if (r < 1e-10f) {
    return { 0, 0, is_upper ? 1.0f : -1.0f, true };  // at pole
  }
  float theta = r * math::kPi_2;
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float scale = st / r;
  return { scale * x, scale * y, is_upper ? ct : -ct, true };
}

// Stereographic: theta = 2 * atan(r)
Dir3 DualFisheyeStereographicInverse(float x, float y, bool is_upper) {
  float r = std::sqrt(x * x + y * y);
  if (r > 1.0f) {
    return { 0, 0, 0, false };
  }
  if (r < 1e-10f) {
    return { 0, 0, is_upper ? 1.0f : -1.0f, true };
  }
  float theta = 2.0f * std::atan(r);
  float st = std::sin(theta);
  float ct = std::cos(theta);
  float scale = st / r;
  return { scale * x, scale * y, is_upper ? ct : -ct, true };
}


// =============== Type C: Rectangular ===============

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
