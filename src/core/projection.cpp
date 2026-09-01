#include "core/projection.hpp"

#include <algorithm>
#include <cmath>

#include "core/math.hpp"
#include "core/shared/projection_shared.h"

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
  auto r = lm_proj::FisheyeEqualAreaForward(dx, dy, dz, r_scale);
  return { r.x, r.y, r.valid };
}

// Equidistant: scale = r_scale * theta / (pi/2 * rho). theta = acos(dz).
ProjXY FisheyeEquidistantForward(float dx, float dy, float dz, float r_scale) {
  auto r = lm_proj::FisheyeEquidistantForward(dx, dy, dz, r_scale);
  return { r.x, r.y, r.valid };
}

// Stereographic: scale = r_scale * tan(theta/2) / rho. theta = acos(dz).
ProjXY FisheyeStereographicForward(float dx, float dy, float dz, float r_scale) {
  auto r = lm_proj::FisheyeStereographicForward(dx, dy, dz, r_scale);
  return { r.x, r.y, r.valid };
}

// Orthographic: r = sin(theta). For |d|=1, sin(theta) = sqrt(dx^2 + dy^2), so
// the forward projection collapses to (dx, dy) directly.
// dz < 0 is rejected: sin(theta) is not injective past theta=pi/2 (sin(120) == sin(60)),
// two distinct rays would otherwise project to the same pixel.
ProjXY FisheyeOrthographicForward(float dx, float dy, float dz, float r_scale) {
  auto r = lm_proj::FisheyeOrthographicForward(dx, dy, dz, r_scale);
  return { r.x, r.y, r.valid };
}


// =============== Fisheye inverse projections (pure math) ===============
// Domain check: r beyond the type's coverage boundary is rejected. The boundary is that type's
// radius at the largest theta the RENDER path will produce, so "the inverse accepts this pixel"
// and "a ray could have landed on this pixel" stay the same statement — which is what makes
// lens_proj_build.hpp's render-domain mask exact rather than approximate. See the per-type cull
// in projection_shared.h::ProjectExitToPixel and its three kFisheye*MinCz notes; the values below
// are the mirror image of those floors.
//
// The dual-fisheye path shares these same functions and is NOT affected by the wider bounds:
// lens_proj_build.hpp::DualFisheyePixelToWorld feeds them coordinates PixelToDualFisheye has
// already clipped to r <= 1, so it never reaches the region that opened up.
//
// When r_scale < 1, inverse may return z < 0 (past-equator direction) — this is correct.

// Largest r each type's inverse accepts at r_scale = 1, i.e. that type's radius at its rim:
//   equal-area     sqrt(2) sin(theta/2)  at theta = 180 deg -> sqrt(2)   (compared as r^2 <= 2)
//   equidistant    theta / (pi/2)        at theta = 180 deg -> 2
//   stereographic  tan(theta/2)          at acos(kFisheyeStereographicMinCz) = 179.5 deg
// The stereographic bound is derived from the shared cull floor rather than restating 179.5 deg,
// so a change to that floor moves both sides at once. Orthographic keeps r <= r_scale
// (theta <= 90 deg) — it is the one type whose forward is not injective past the equator.
//
// kRimSlack is the allowance for the fact that a rim is not a crisp number in float. The forward's
// radius is a function of cz, cz reaches it with a few ulps of error, and the largest radius the
// render path can ACTUALLY produce therefore sits a little above the analytic rim. Without the
// slack the inverse rejects a pixel a ray has just landed on, and the render-domain mask built
// from this inverse (lens_proj_build.hpp) paints background over it — a defect that only appears
// in the outermost ring, which is precisely the region 474.1 opened up. It can be this small only
// because the cull floors bound that error rather than letting it grow; kFisheyeEqualAreaMinCz is
// the one chosen for this reason specifically. 1e-5 of a rim is under a thousandth of a pixel at
// any resolution these lenses render at.
constexpr float kRimSlack = 1.0f + 1e-5f;
constexpr float kFisheyeEqualAreaMaxR2 = 2.0f * kRimSlack * kRimSlack;
constexpr float kFisheyeEquidistantMaxR = 2.0f * kRimSlack;
const float kFisheyeStereographicMaxR = std::tan(0.5f * std::acos(lm_proj::kFisheyeStereographicMinCz)) * kRimSlack;

// Equal-area inverse: direct Cartesian formula (1 sqrt, no trig).
Dir3 FisheyeEqualAreaInverse(float x, float y, float r_scale) {
  float r2 = x * x + y * y;
  if (r2 > kFisheyeEqualAreaMaxR2) {
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
  if (r > kFisheyeEquidistantMaxR) {
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
  if (r > kFisheyeStereographicMaxR) {
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

// Orthographic inverse: sin(theta) = r / r_scale, so (dx, dy) = (x, y) / r_scale
// and dz = cos(theta) = sqrt(1 - (r/r_scale)^2).
Dir3 FisheyeOrthographicInverse(float x, float y, float r_scale) {
  float r2 = x * x + y * y;
  // kRimSlack applies here too, and for the same reason, even though this type's rim did not move
  // in 474.1: r = sin(theta) reaches r_scale at the equator, which is exactly where its cull sits,
  // so a ray culled at cz > 0 can still land a few ulps outside r_scale.
  float rs2 = r_scale * r_scale * kRimSlack * kRimSlack;
  if (r2 > rs2) {
    return { 0, 0, 0, false };
  }
  float inv_rs = 1.0f / r_scale;
  float xr = x * inv_rs;
  float yr = y * inv_rs;
  float dz = std::sqrt(std::max(0.0f, 1.0f - xr * xr - yr * yr));
  return { xr, yr, dz, true };
}


// =============== Rectangular ===============

ProjXY RectangularForward(float dx, float dy, float dz) {
  auto r = lm_proj::RectangularForward(dx, dy, dz);
  return { r.x, r.y, r.valid };
}

Dir3 RectangularInverse(float lon, float lat) {
  float cl = std::cos(lat);
  return { cl * std::cos(lon), cl * std::sin(lon), std::sin(lat), true };
}


// =============== Globe (outside-in unit sphere) ===============

// Solved directly from lm_proj::ProjectExitToPixel's globe forward rather than transcribed
// from the GUI shader, so the pair is a provable mutual inverse in one frame instead of two
// conventions that have to be argued equal. With s = kGlobeCameraD + c.z the forward reads
// u = -c.x/s, v = c.y/s, hence c = (-u*s, v*s, s - D); substituting into |c| = 1 gives
//   (u^2 + v^2 + 1) s^2 - 2 D s + (D^2 - 1) = 0
// whose near root (the visible surface of the sphere, smallest c.z) is the one taken below.
//
// This is the same discriminant the shader's `globeInverse` computes, up to the positive
// factor 1/(u^2+v^2+1): the shader normalizes its eye-space ray direction first, which
// divides both b^2 and c by that factor. So the two agree on validity exactly, and — being
// a function of u^2+v^2 only — that agreement does not depend on the horizontal sign
// convention the two sides use for u.
//
// kGlobeCameraD is shared verbatim with lm_proj (and, by the must-match anchor there, with
// the GUI); no second literal is introduced here.
Dir3 GlobeInverse(float x, float y, float focal) {
  if (!(focal > 0.0f)) {
    return { 0, 0, 0, false };  // degenerate lens (fov -> 180 deg); nothing is imaged
  }
  const float d = lm_proj::kGlobeCameraD;
  float u = x / focal;
  float v = y / focal;
  float a = u * u + v * v + 1.0f;
  float disc = d * d - a * (d * d - 1.0f);
  if (disc < 0.0f) {
    return { 0, 0, 0, false };  // ray misses the sphere
  }
  float s = (d - std::sqrt(disc)) / a;
  if (s <= 0.0f) {
    return { 0, 0, 0, false };  // intersection behind the camera
  }
  // Mathematically |c| == 1 already; normalize to absorb float error near the silhouette
  // (disc ~ 0), matching the same guard in the GUI's CPU mirror.
  float cx = -u * s;
  float cy = v * s;
  float cz = s - d;
  float len = std::sqrt(cx * cx + cy * cy + cz * cz);
  if (len <= 0.0f) {
    return { 0, 0, 0, false };
  }
  return { cx / len, cy / len, cz / len, true };
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

float ComputeEDRScale(float max_abs_dz) {
  return (max_abs_dz <= 0) ? 1.0f : math::kPi_2 / (math::kPi_2 + std::asin(max_abs_dz));
}

float ComputeSTRScale(float max_abs_dz) {
  return (max_abs_dz <= 0) ? 1.0f : 1.0f / std::tan((math::kPi_2 + std::asin(max_abs_dz)) / 2.0f);
}

}  // namespace projection
}  // namespace lumice
