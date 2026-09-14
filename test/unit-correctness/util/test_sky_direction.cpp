#include <gtest/gtest.h>

#include <cmath>

#include "util/sky_direction.hpp"

// The one (altitude, azimuth) <-> travel-direction convention the GUI's Point-mode centre and the
// CLI's `--center` share (src/util/sky_direction.hpp). Two kinds of proposition, on purpose: a
// round trip alone would pass for a pair of functions that were each other's inverse under a
// WRONG convention, so the absolute cases below pin the convention against hand-computed
// directions, and only then does the round trip say the inverse is exact.

namespace {

constexpr float kTol = 1e-5f;

void ExpectDir(float alt_deg, float az_deg, float x, float y, float z) {
  float dir[3] = { 0.0f, 0.0f, 0.0f };
  lumice::AltAzToDir(alt_deg, az_deg, dir);
  EXPECT_NEAR(dir[0], x, kTol) << "alt " << alt_deg << " az " << az_deg;
  EXPECT_NEAR(dir[1], y, kTol) << "alt " << alt_deg << " az " << az_deg;
  EXPECT_NEAR(dir[2], z, kTol) << "alt " << alt_deg << " az " << az_deg;
}

}  // namespace

// Absolute values, by hand from the convention stated in lumice.h: light from the horizon point at
// azimuth 0 travels along -x; from the zenith, straight down (z = -1); from azimuth 90, along -y.
TEST(SkyDirection, AltAzToDirPinsTheConvention) {
  ExpectDir(0.0f, 0.0f, -1.0f, 0.0f, 0.0f);
  ExpectDir(90.0f, 0.0f, 0.0f, 0.0f, -1.0f);
  ExpectDir(-90.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  ExpectDir(0.0f, 90.0f, 0.0f, -1.0f, 0.0f);
  ExpectDir(0.0f, 180.0f, 1.0f, 0.0f, 0.0f);
  ExpectDir(0.0f, -90.0f, 0.0f, 1.0f, 0.0f);
  // alt 30, az 0: x = -cos30, z = -sin30.
  ExpectDir(30.0f, 0.0f, -std::sqrt(3.0f) / 2.0f, 0.0f, -0.5f);
}

// The direction the existing GUI test already reads back: +x is altitude 0 from azimuth 180 (the
// value test_analysis_panel_logic.cpp's cone CSV expectation carries as "-0.00" / "-180.00" —
// -180 and 180 are the same azimuth, and the wrap chooses the (-180, 180] side).
TEST(SkyDirection, DirToAltAzPinsTheConvention) {
  float alt = 0.0f;
  float az = 0.0f;
  const float plus_x[3] = { 1.0f, 0.0f, 0.0f };
  lumice::DirToAltAz(plus_x, &alt, &az);
  EXPECT_NEAR(alt, 0.0f, kTol);
  EXPECT_NEAR(std::fabs(az), 180.0f, kTol);

  const float minus_x[3] = { -1.0f, 0.0f, 0.0f };
  lumice::DirToAltAz(minus_x, &alt, &az);
  EXPECT_NEAR(alt, 0.0f, kTol);
  EXPECT_NEAR(az, 0.0f, kTol);

  const float down[3] = { 0.0f, 0.0f, -1.0f };
  lumice::DirToAltAz(down, &alt, &az);
  EXPECT_NEAR(alt, 90.0f, kTol);

  // z past the unit range is clamped, not NaN.
  const float over[3] = { 0.0f, 0.0f, -1.5f };
  lumice::DirToAltAz(over, &alt, &az);
  EXPECT_NEAR(alt, 90.0f, kTol);
}

// The round trip over all four azimuth quadrants and both hemispheres, including points near the
// poles where the azimuth is ill-conditioned (the altitude must still come back; the azimuth is
// only compared where it is defined). Azimuths are chosen inside (-180, 180) so the wrap is exact.
TEST(SkyDirection, AltAzRoundTripsThroughDir) {
  const float alts[] = { -89.5f, -60.0f, -10.0f, 0.0f, 15.0f, 43.0f, 85.0f, 89.5f };
  const float azs[] = { -170.0f, -135.0f, -90.0f, -45.0f, 0.0f, 30.0f, 90.0f, 120.0f, 179.0f };
  for (const float alt : alts) {
    for (const float az : azs) {
      float dir[3] = { 0.0f, 0.0f, 0.0f };
      lumice::AltAzToDir(alt, az, dir);
      const float norm = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
      EXPECT_NEAR(norm, 1.0f, kTol) << "alt " << alt << " az " << az;
      float alt_back = 0.0f;
      float az_back = 0.0f;
      lumice::DirToAltAz(dir, &alt_back, &az_back);
      EXPECT_NEAR(alt_back, alt, 1e-3f) << "alt " << alt << " az " << az;
      if (std::fabs(alt) < 89.0f) {
        EXPECT_NEAR(az_back, az, 1e-3f) << "alt " << alt << " az " << az;
      }
    }
  }
}
