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

// =============== Overlap r_scale (equal-area) ===============

float ComputeEARScale(float max_abs_dz) {
  return (max_abs_dz <= 0) ? 1.0f : 1.0f / std::sqrt(1.0f + max_abs_dz);
}

// =============== CPU reprojection helpers ===============

void ReprojectEquirectangular(const unsigned char* src, int src_w, int src_h, float r_scale, unsigned char* dst,
                              int dst_w, int dst_h) {
  if (src == nullptr || dst == nullptr || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
    return;
  }

  // Zero-fill dst (overlap-ring / out-of-disc pixels remain black).
  std::fill_n(dst, static_cast<size_t>(dst_w) * dst_h * 3, static_cast<unsigned char>(0));

  float r_scale_sq = r_scale * r_scale;

  for (int py = 0; py < dst_h; ++py) {
    float lat = math::kPi * 0.5f - (static_cast<float>(py) + 0.5f) / dst_h * math::kPi;
    float cos_lat = std::cos(lat);
    float sin_lat = std::sin(lat);

    for (int px = 0; px < dst_w; ++px) {
      float lon = (static_cast<float>(px) + 0.5f) / dst_w * 2.0f * math::kPi - math::kPi;
      // Matches shader rectangularInverse (src/gui/preview_renderer.cpp:240-249).
      float dx = -cos_lat * std::cos(lon);
      float dy = -cos_lat * std::sin(lon);
      float dz = -sin_lat;

      bool is_upper = (dz < 0.0f);
      float z_hemi = std::fabs(dz);

      ProjXY proj = FisheyeEqualAreaForward(dx, dy, z_hemi, r_scale);
      float r_sq = proj.x * proj.x + proj.y * proj.y;
      // Pixels in the overlap ring (r_proj > r_scale, i.e. r_sq > r_scale^2) or
      // outside the disc are left black. When r_scale = 1, only out-of-disc is masked.
      if (r_sq > r_scale_sq) {
        continue;
      }

      float fx = 0.0f;
      float fy = 0.0f;
      DualFisheyeToPixel(proj.x, proj.y, is_upper, src_w, src_h, &fx, &fy);
      int sx = std::clamp(static_cast<int>(std::floor(fx)), 0, src_w - 1);
      int sy = std::clamp(static_cast<int>(std::floor(fy)), 0, src_h - 1);

      size_t src_idx = (static_cast<size_t>(sy) * src_w + sx) * 3;
      size_t dst_idx = (static_cast<size_t>(py) * dst_w + px) * 3;
      dst[dst_idx + 0] = src[src_idx + 0];
      dst[dst_idx + 1] = src[src_idx + 1];
      dst[dst_idx + 2] = src[src_idx + 2];
    }
  }
}

void MaskDualFisheyeOverlap(unsigned char* buf, int w, int h, float r_scale) {
  if (buf == nullptr || w <= 0 || h <= 0 || r_scale >= 1.0f) {
    return;
  }
  float r_scale_sq = r_scale * r_scale;

  for (int py = 0; py < h; ++py) {
    for (int px = 0; px < w; ++px) {
      float x_norm = 0.0f;
      float y_norm = 0.0f;
      bool is_upper = false;
      if (!PixelToDualFisheye(static_cast<float>(px) + 0.5f, static_cast<float>(py) + 0.5f, w, h, &x_norm, &y_norm,
                              &is_upper)) {
        continue;  // outside both circles — leave untouched
      }
      float r_sq = x_norm * x_norm + y_norm * y_norm;
      if (r_sq > r_scale_sq) {
        size_t idx = (static_cast<size_t>(py) * w + px) * 3;
        buf[idx + 0] = 0;
        buf[idx + 1] = 0;
        buf[idx + 2] = 0;
      }
    }
  }
}

}  // namespace projection
}  // namespace lumice
