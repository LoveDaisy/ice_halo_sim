#pragma once

#include <algorithm>
#include <cmath>

namespace lumice {

// The one convention every direction in the C API uses (lumice.h, the marker family and the
// raypath-analysis cone centre): a unit vector is the direction light TRAVELS, so the sky point
// it comes FROM sits at altitude = asin(-z) (the zenith is z = -1) and its azimuth is measured as
// the sun's is — the sun at azimuth 0 sits at lon 180, i.e. az = atan2(y, x) - 180.
//
// Both halves of the conversion live here, in src/util/, because they are the ONE place the
// GUI's Point-mode centre (analysis_panel.cpp) and the CLI's `--center <alt>,<az>` agree on
// what "altitude 43, azimuth 0" means. Pure and stateless, no core or config type in sight —
// the shape AGENTS.md admits under src/util/ — so both sides read it without a second copy,
// and `inline` so the shared build's hidden visibility has no symbol to hide.

namespace sky_direction_detail {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;
}  // namespace sky_direction_detail

// Direction light travels -> (altitude, azimuth) in degrees of the sky point it comes from.
// Altitude in [-90, 90]; azimuth wrapped into (-180, 180]. `dir` need not be normalized beyond
// |z| <= 1 (z is clamped for the asin).
inline void DirToAltAz(const float dir[3], float* alt_deg, float* az_deg) {
  using sky_direction_detail::kRad2Deg;
  const float z = std::max(-1.0f, std::min(1.0f, dir[2]));
  *alt_deg = std::asin(-z) * kRad2Deg;
  float az = std::atan2(dir[1], dir[0]) * kRad2Deg - 180.0f;
  while (az > 180.0f) {
    az -= 360.0f;
  }
  while (az < -180.0f) {
    az += 360.0f;
  }
  *az_deg = az;
}

// The inverse: (altitude, azimuth) in degrees of a sky point -> the unit direction light from it
// travels. Read DirToAltAz backwards:
//   altitude = asin(-z)          -> z = -sin(alt)
//   azimuth  = atan2(y, x) - 180 -> x = -cos(alt)cos(az), y = -cos(alt)sin(az)
// The same formula the annotation overlay's altitude curves use (core/annotation_overlay.cpp).
inline void AltAzToDir(float alt_deg, float az_deg, float dir[3]) {
  using sky_direction_detail::kDeg2Rad;
  const float alt = alt_deg * kDeg2Rad;
  const float az = az_deg * kDeg2Rad;
  const float cos_alt = std::cos(alt);
  dir[0] = -cos_alt * std::cos(az);
  dir[1] = -cos_alt * std::sin(az);
  dir[2] = -std::sin(alt);
}

}  // namespace lumice
