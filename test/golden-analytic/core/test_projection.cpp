#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "core/shared/projection_shared.h"

namespace lumice {
namespace projection {
namespace {

constexpr float kEps = 1e-6f;
constexpr float kEpsPolar = 5e-6f;  // Relaxed for polar-path projections (atan2/asin/sin/cos chain)

// Helper: check unit vector
void ExpectUnitVector(const Dir3& d) {
  float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  EXPECT_NEAR(len, 1.0f, kEps);
}

// =============== Type A: Linear ===============

TEST(Projection, LinearForwardOnAxis) {
  auto r = LinearForward(0, 0, 1);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 0, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, LinearForwardOffAxis) {
  auto r = LinearForward(1, 0, 1);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 1.0f, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, LinearForwardBehind) {
  auto r = LinearForward(0, 0, -1);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, LinearRoundTrip) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 1000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = std::abs(dist(rng)) + 0.01f;
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= len;
    dy /= len;
    dz /= len;
    auto fwd = LinearForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = LinearInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
}

// =============== Fisheye Equal Area ===============

TEST(Projection, FisheyeEqualAreaForwardPole) {
  // Pole: (0, 0, 1) -> center (0, 0)
  auto r = FisheyeEqualAreaForward(0, 0, 1);
  EXPECT_NEAR(r.x, 0, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, FisheyeEqualAreaForwardEquator) {
  // Equator: (1, 0, 0) -> r=1 (with default r_scale=1.0)
  auto r = FisheyeEqualAreaForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

TEST(Projection, FisheyeEqualAreaRoundTrip) {
  // Full-sphere round-trip: caller flips z for lower hemisphere
  std::mt19937 rng(45);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;

    bool is_upper = (dz >= 0);
    float z_hemi = is_upper ? dz : -dz;
    auto fwd = FisheyeEqualAreaForward(dx, dy, z_hemi);
    auto inv = FisheyeEqualAreaInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    // inv.z is in z_hemi space; flip back for lower hemisphere
    float recovered_z = is_upper ? inv.z : -inv.z;
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(recovered_z, dz, kEps);
  }
}

TEST(Projection, FisheyeEqualAreaEqualAreaProperty) {
  // Verify equal-area property: uniform sphere sampling -> uniform disc distribution.
  // Sample many uniform directions, project, bin by radius^2 (should be uniform for equal-area).
  constexpr int kSamples = 100000;
  constexpr int kBins = 10;
  int bins[kBins] = {};

  std::mt19937 rng(46);
  std::uniform_real_distribution<float> u01(0.0f, 1.0f);
  for (int i = 0; i < kSamples; i++) {
    // Uniform sphere sampling (upper hemisphere only for simplicity)
    float z = u01(rng);  // z in [0, 1] for upper hemisphere
    float phi = 2.0f * math::kPi * u01(rng);
    float rho = std::sqrt(1.0f - z * z);
    float dx = rho * std::cos(phi);
    float dy = rho * std::sin(phi);
    auto fwd = FisheyeEqualAreaForward(dx, dy, z);
    float r2 = fwd.x * fwd.x + fwd.y * fwd.y;
    // r^2 should be uniformly distributed in [0, 1] for equal-area projection
    int bin = std::min(static_cast<int>(r2 * kBins), kBins - 1);
    bins[bin]++;
  }

  // Chi-squared test: each bin should have ~kSamples/kBins
  float expected = static_cast<float>(kSamples) / kBins;
  float chi2 = 0;
  for (int i = 0; i < kBins; i++) {
    float diff = bins[i] - expected;
    chi2 += diff * diff / expected;
  }
  // Chi-squared critical value for 9 dof at p=0.01 is 21.67
  EXPECT_LT(chi2, 21.67f) << "Equal-area property violated: chi2=" << chi2;
}

TEST(Projection, FisheyeEqualAreaInverseBeyondDomain) {
  auto r = FisheyeEqualAreaInverse(1.1f, 0);
  EXPECT_FALSE(r.valid);
}

// =============== Fisheye Equidistant ===============

TEST(Projection, FisheyeEquidistantEquatorNorm) {
  // At equator, r should be 1
  auto r = FisheyeEquidistantForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

TEST(Projection, FisheyeEquidistantRoundTrip) {
  std::mt19937 rng(47);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;

    bool is_upper = (dz >= 0);
    float z_hemi = is_upper ? dz : -dz;
    auto fwd = FisheyeEquidistantForward(dx, dy, z_hemi);
    auto inv = FisheyeEquidistantInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    float recovered_z = is_upper ? inv.z : -inv.z;
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(recovered_z, dz, kEps);
  }
}

// =============== Fisheye Stereographic ===============

TEST(Projection, FisheyeStereographicEquatorNorm) {
  auto r = FisheyeStereographicForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

TEST(Projection, FisheyeStereographicRoundTrip) {
  std::mt19937 rng(48);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;

    bool is_upper = (dz >= 0);
    float z_hemi = is_upper ? dz : -dz;
    auto fwd = FisheyeStereographicForward(dx, dy, z_hemi);
    auto inv = FisheyeStereographicInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    float recovered_z = is_upper ? inv.z : -inv.z;
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(recovered_z, dz, kEps);
  }
}

// =============== Rectangular ===============

TEST(Projection, RectangularForwardBasic) {
  // Direction along +x: lon=0, lat=0
  auto r = RectangularForward(1, 0, 0);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 0, kEps);  // lon=0
  EXPECT_NEAR(r.y, 0, kEps);  // lat=0
}

TEST(Projection, RectangularForwardZenith) {
  // Direction up: dz=1 -> lat=pi/2
  auto r = RectangularForward(0, 0, 1);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.y, math::kPi_2, kEps);
}

TEST(Projection, RectangularRoundTrip) {
  std::mt19937 rng(49);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;
    auto fwd = RectangularForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = RectangularInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEpsPolar);
    EXPECT_NEAR(inv.y, dy, kEpsPolar);
    EXPECT_NEAR(inv.z, dz, kEpsPolar);
  }
}

// =============== Layout functions ===============

TEST(Projection, DualFisheyeLayoutRoundTrip) {
  constexpr int kWidth = 800;
  constexpr int kHeight = 400;

  std::mt19937 rng(50);
  std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * math::kPi);
  std::uniform_real_distribution<float> r_dist(0.0f, 0.9f);  // within unit disc
  for (int i = 0; i < 500; i++) {
    float r = r_dist(rng);
    float angle = angle_dist(rng);
    float x_in = r * std::cos(angle);
    float y_in = r * std::sin(angle);
    bool upper = (i % 2 == 0);

    float fx = 0;
    float fy = 0;
    DualFisheyeToPixel(x_in, y_in, upper, kWidth, kHeight, &fx, &fy);

    float x_out = 0;
    float y_out = 0;
    bool upper_out = false;
    bool valid = PixelToDualFisheye(fx, fy, kWidth, kHeight, &x_out, &y_out, &upper_out);
    ASSERT_TRUE(valid);
    EXPECT_EQ(upper_out, upper);
    EXPECT_NEAR(x_out, x_in, kEps);
    EXPECT_NEAR(y_out, y_in, kEps);
  }
}

TEST(Projection, DualFisheyeLayoutOutsideCircles) {
  constexpr int kWidth = 800;
  constexpr int kHeight = 400;

  float x, y;
  bool upper;
  // Pixel at far corner should be outside both circles
  EXPECT_FALSE(PixelToDualFisheye(0, 0, kWidth, kHeight, &x, &y, &upper));
  EXPECT_FALSE(PixelToDualFisheye(static_cast<float>(kWidth), 0, kWidth, kHeight, &x, &y, &upper));
}

// =============== Boundary cases ===============

TEST(Projection, InverseBeyondDomainEA) {
  auto r = FisheyeEqualAreaInverse(1.5f, 0);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, InverseBeyondDomainED) {
  auto r = FisheyeEquidistantInverse(1.5f, 0);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, InverseBeyondDomainST) {
  auto r = FisheyeStereographicInverse(1.5f, 0);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, FisheyeForwardAtPoles) {
  // All three: pole -> center (0,0)
  auto ea = FisheyeEqualAreaForward(0, 0, 1);
  EXPECT_NEAR(ea.x, 0, kEps);
  EXPECT_NEAR(ea.y, 0, kEps);

  auto ed = FisheyeEquidistantForward(0, 0, 1);
  EXPECT_NEAR(ed.x, 0, kEps);
  EXPECT_NEAR(ed.y, 0, kEps);

  auto st = FisheyeStereographicForward(0, 0, 1);
  EXPECT_NEAR(st.x, 0, kEps);
  EXPECT_NEAR(st.y, 0, kEps);
}

// =============== r_scale round-trip (overlap support) ===============

TEST(Projection, FisheyeEARScaleRoundTrip) {
  // With r_scale < 1, projection covers past the equator (z_hemi < 0).
  // Verify forward→inverse round-trip for the overlap zone.
  constexpr float kOverlap = 0.0872f;  // sin(5°)
  float r_scale = 1.0f / std::sqrt(1.0f + kOverlap);

  std::mt19937 rng(60);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;

    bool is_upper = (dz >= 0);
    float z_hemi = is_upper ? dz : -dz;
    auto fwd = FisheyeEqualAreaForward(dx, dy, z_hemi, r_scale);

    // With r_scale < 1, some directions near the equator will have r > 1 (beyond coverage)
    float r2 = fwd.x * fwd.x + fwd.y * fwd.y;
    if (r2 > 1.0f)
      continue;  // skip out-of-coverage directions

    auto inv = FisheyeEqualAreaInverse(fwd.x, fwd.y, r_scale);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    float recovered_z = is_upper ? inv.z : -inv.z;
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(recovered_z, dz, kEps);
  }
}

TEST(Projection, FisheyeEARScaleEquatorInside) {
  // With r_scale < 1, the equator (z_hemi=0) maps to r = r_scale < 1 (inside disc).
  constexpr float kOverlap = 0.0872f;
  float r_scale = 1.0f / std::sqrt(1.0f + kOverlap);
  auto fwd = FisheyeEqualAreaForward(1, 0, 0, r_scale);
  float r = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
  EXPECT_NEAR(r, r_scale, kEps);
}

TEST(Projection, FisheyeEARScaleOverlapBoundary) {
  // At the overlap boundary (z_hemi = -max_abs_dz), r should be ~1.0.
  constexpr float kOverlap = 0.0872f;
  float r_scale = 1.0f / std::sqrt(1.0f + kOverlap);
  // Direction at overlap boundary: z = -kOverlap, rho = sqrt(1 - kOverlap^2)
  float rho = std::sqrt(1.0f - kOverlap * kOverlap);
  auto fwd = FisheyeEqualAreaForward(rho, 0, -kOverlap, r_scale);
  float r = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
  EXPECT_NEAR(r, 1.0f, 1e-4f);
}

// =============== MaxFov Verification ===============

TEST(MaxFov, ReturnsCorrectLimits) {
  EXPECT_FLOAT_EQ(lumice::MaxFov(lumice::LensParam::kLinear), 179.0f);
  EXPECT_FLOAT_EQ(lumice::MaxFov(lumice::LensParam::kFisheyeEqualArea), 360.0f);
  EXPECT_FLOAT_EQ(lumice::MaxFov(lumice::LensParam::kFisheyeEquidistant), 360.0f);
  EXPECT_FLOAT_EQ(lumice::MaxFov(lumice::LensParam::kFisheyeStereographic), 359.0f);
  EXPECT_FLOAT_EQ(lumice::MaxFov(lumice::LensParam::kDualFisheyeEqualArea), 360.0f);
  EXPECT_FLOAT_EQ(lumice::MaxFov(lumice::LensParam::kRectangular), 360.0f);
}

// =============== FOV Scale Verification ===============
// Verify that the short-edge-based FOV semantics produce correct pixel coordinates.
// These tests compute pixel positions using the same scale formulas as render.cpp,
// serving as mathematical anchors independent of reference images.

TEST(FovScale, LinearScaleShortEdge) {
  // For linear projection: scale = short_pix / 2 / tan(fov/2)
  // A ray at angle fov/2 from center should project to exactly short_pix/2 from center.
  constexpr int kWidth = 400;
  constexpr int kHeight = 300;
  constexpr float kFov = 90.0f;
  float short_pix = static_cast<float>(std::min(kWidth, kHeight));  // 300
  float scale = short_pix / 2.0f / std::tan(kFov / 2.0f * math::kDegreeToRad);

  // Direction at 45° from optical axis (fov/2 = 45°): (1, 0, 1) normalized
  float d_norm = 1.0f / std::sqrt(2.0f);
  auto proj = LinearForward(d_norm, 0, d_norm);
  ASSERT_TRUE(proj.valid);

  // proj.x = dx/dz = 1, proj.y = 0
  float px = proj.x * scale;
  // px should equal short_pix/2 = 150
  EXPECT_NEAR(px, short_pix / 2.0f, 1e-4f);
}

TEST(FovScale, EqualAreaScaleShortEdge) {
  // For equal area: scale = short_pix / 2 / sqrt(2) / sin(fov/4)
  // Unified normalization: r = sqrt(1-dz) = sqrt(2)*sin(theta/2).
  // At theta = fov/2 = 90°: r = sqrt(2)*sin(45°) = 1.0.
  // scale * r = short_pix/2 → scale = short_pix / 2 / 1.0... but we need to match
  // the actual formula: scale = short_pix / 2 / sqrt(2) / sin(fov/4).
  constexpr int kWidth = 400;
  constexpr int kHeight = 300;
  constexpr float kFov = 180.0f;  // full hemisphere
  float short_pix = static_cast<float>(std::min(kWidth, kHeight));
  float scale = short_pix / 2.0f / std::sqrt(2.0f) / std::sin(kFov / 4.0f * math::kDegreeToRad);

  // Direction at horizon (theta = 90° from optical axis): (1, 0, 0+epsilon) normalized
  // Unified EA: r = sqrt(1 - dz). At dz ≈ 0: r ≈ 1.0
  auto proj = FisheyeEqualAreaForward(1.0f, 0, 1e-6f);

  float r_pix = std::sqrt(proj.x * proj.x + proj.y * proj.y) * scale;
  // r_pix should equal short_pix/2 = 150 (horizon maps to edge of circle)
  EXPECT_NEAR(r_pix, short_pix / 2.0f, 0.1f);
}

TEST(FovScale, EquidistantScaleShortEdge) {
  // For equidistant: scale = short_pix * pi/2 / (fov * deg2rad)
  // Unified normalization: r = theta / (pi/2).
  // At theta = fov/2 = pi/2: r = 1.0.
  // scale * r = short_pix/2 → scale = short_pix * pi/2 / (fov * deg2rad).
  constexpr float kFov = 180.0f;
  constexpr int kShort = 300;
  float scale = static_cast<float>(kShort) * math::kPi_2 / (kFov * math::kDegreeToRad);

  // Direction at horizon (theta = pi/2): r = theta/(pi/2) = 1.0
  auto proj = FisheyeEquidistantForward(1.0f, 0, 1e-6f);

  float r_pix = std::sqrt(proj.x * proj.x + proj.y * proj.y) * scale;
  EXPECT_NEAR(r_pix, static_cast<float>(kShort) / 2.0f, 0.1f);
}

// =============== Orthographic ===============

TEST(Projection, FisheyeOrthographicForwardPole) {
  auto r = FisheyeOrthographicForward(0, 0, 1);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 0, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, FisheyeOrthographicForwardEquator) {
  // theta = pi/2, dz = 0, sin(theta) = 1; (dx, dy) = (1, 0) lies on unit circle.
  auto r = FisheyeOrthographicForward(1.0f, 0, 0);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 1.0f, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, FisheyeOrthographicForwardOffAxis) {
  // r = sin(theta) in Cartesian form == (dx, dy) for unit direction.
  for (float theta_deg : { 15.0f, 45.0f, 80.0f }) {
    for (float phi_deg : { 0.0f, 90.0f, 180.0f, 270.0f }) {
      float theta = theta_deg * math::kDegreeToRad;
      float phi = phi_deg * math::kDegreeToRad;
      float dx = std::sin(theta) * std::cos(phi);
      float dy = std::sin(theta) * std::sin(phi);
      float dz = std::cos(theta);
      auto r = FisheyeOrthographicForward(dx, dy, dz);
      EXPECT_TRUE(r.valid) << "theta=" << theta_deg << " phi=" << phi_deg;
      EXPECT_NEAR(r.x, dx, kEps);
      EXPECT_NEAR(r.y, dy, kEps);
    }
  }
}

TEST(Projection, FisheyeOrthographicForwardBackHemisphere) {
  // theta > 90 deg -> dz < 0 -> guard trips, returns valid=false.
  for (float theta_deg : { 91.0f, 120.0f, 179.0f }) {
    float theta = theta_deg * math::kDegreeToRad;
    float dx = std::sin(theta);
    float dz = std::cos(theta);
    auto r = FisheyeOrthographicForward(dx, 0.0f, dz);
    EXPECT_FALSE(r.valid) << "theta=" << theta_deg;
  }
}

TEST(Projection, FisheyeOrthographicInverseBoundary) {
  auto on_circle = FisheyeOrthographicInverse(1.0f, 0.0f);
  EXPECT_TRUE(on_circle.valid);
  EXPECT_NEAR(on_circle.x, 1.0f, kEps);
  EXPECT_NEAR(on_circle.y, 0.0f, kEps);
  EXPECT_NEAR(on_circle.z, 0.0f, kEps);

  auto outside = FisheyeOrthographicInverse(1.01f, 0.0f);
  EXPECT_FALSE(outside.valid);
}

TEST(Projection, FisheyeOrthographicRoundTrip) {
  std::mt19937 rng(20260424);
  std::uniform_real_distribution<float> theta_dist(0.0f, math::kPi_2 - 1e-3f);
  std::uniform_real_distribution<float> phi_dist(-math::kPi, math::kPi);
  for (int i = 0; i < 32; ++i) {
    float theta = theta_dist(rng);
    float phi = phi_dist(rng);
    float dx = std::sin(theta) * std::cos(phi);
    float dy = std::sin(theta) * std::sin(phi);
    float dz = std::cos(theta);
    auto fwd = FisheyeOrthographicForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = FisheyeOrthographicInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    EXPECT_NEAR(inv.x, dx, kEpsPolar);
    EXPECT_NEAR(inv.y, dy, kEpsPolar);
    EXPECT_NEAR(inv.z, dz, kEpsPolar);
    ExpectUnitVector(inv);
  }
}

// =============== ComputeEARScale ===============

TEST(ComputeEARScale, NoOverlap) {
  EXPECT_FLOAT_EQ(ComputeEARScale(0.0f), 1.0f);
  EXPECT_FLOAT_EQ(ComputeEARScale(-0.1f), 1.0f);
}

TEST(ComputeEARScale, WithOverlap) {
  float s = ComputeEARScale(0.0872f);  // sin 5°
  EXPECT_LT(s, 1.0f);
  EXPECT_GT(s, 0.9f);
  // Forward at equator (dz=0) maps to r=s (coverage boundary == s in r_scale=1 convention).
  auto proj = FisheyeEqualAreaForward(1.0f, 0.0f, 0.0f, s);
  float r = std::sqrt(proj.x * proj.x + proj.y * proj.y);
  EXPECT_NEAR(r, s, 1e-4f);
}

// =============== lm_proj:: direct forward equivalence ===============
// Guards against future divergence between projection::LinearForward wrapper
// and the shared lm_proj::LinearForward it delegates to (task-unify-shared-projection).

TEST(LmProj, LinearForwardMatchesHost) {
  const float dxs[] = { 0.0f, 0.3f, -0.5f, 0.7f };
  const float dys[] = { 0.0f, -0.4f, 0.6f, 0.2f };
  const float dzs[] = { 1.0f, 0.5f, 0.8f, 2.0f };
  for (int i = 0; i < 4; ++i) {
    auto a = lm_proj::LinearForward(dxs[i], dys[i], dzs[i]);
    auto b = projection::LinearForward(dxs[i], dys[i], dzs[i]);
    EXPECT_EQ(a.valid, b.valid);
    EXPECT_FLOAT_EQ(a.x, b.x);
    EXPECT_FLOAT_EQ(a.y, b.y);
  }
  // dz<=0 rejection.
  auto rej = lm_proj::LinearForward(0.5f, 0.5f, -0.1f);
  EXPECT_FALSE(rej.valid);
}

// =============== ProjectExitToPixel per-type parity vs GetProjFunc ===============
// Regression anchor for task-unify-shared-projection Step 3-5: pins the shared
// ProjectExitToPixel to the current per-type *Project functions. Run BEFORE
// lens_proj.hpp is rewritten to thin wrappers; runs identically AFTER (since
// the wrappers delegate to the same function).

// Build a RenderConfig for a lens type + camera view. Kept minimal — only the
// fields BuildProjParams reads.
RenderConfig MakeRC(LensParam::LensType t, float fov_deg, int w, int h, float az_deg, float el_deg, float ro_deg,
                    float overlap = 0.0f, RenderConfig::VisibleRange vis = RenderConfig::kFull) {
  RenderConfig cfg;
  cfg.lens_.type_ = t;
  cfg.lens_.fov_ = fov_deg;
  cfg.resolution_[0] = w;
  cfg.resolution_[1] = h;
  cfg.lens_shift_[0] = 0;
  cfg.lens_shift_[1] = 0;
  cfg.visible_ = vis;
  cfg.overlap_ = overlap;
  cfg.view_.az_ = az_deg;
  cfg.view_.el_ = el_deg;
  cfg.view_.ro_ = ro_deg;
  return cfg;
}

// Return (px, py) via ProjectExitToPixel + legacy GetProjFunc for the same
// (cfg, dir) pair. Both invoke the same math paths post Step 3, but this
// harness ran on the ORIGINAL *Project bodies first and pinned them.
void ExpectMainHitEqualsLegacy(LensParam::LensType t, const RenderConfig& cfg, float dx, float dy, float dz,
                               const char* label) {
  Rotation rot = lumice::MakeCameraRotation(cfg);
  float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));

  // Shared.
  auto pp = lumice::BuildProjParams(cfg, rot, short_pix);
  auto res = lm_proj::ProjectExitToPixel(pp, dx, dy, dz);

  // Legacy.
  LensProjParam lp{ cfg.lens_.fov_,
                    short_pix,
                    rot,
                    cfg.visible_,
                    { cfg.resolution_[0], cfg.resolution_[1] },
                    { cfg.lens_shift_[0], cfg.lens_shift_[1] },
                    0.0f,
                    1.0f };
  if (cfg.overlap_ > 0) {
    switch (t) {
      case LensParam::kDualFisheyeEqualArea:
        lp.max_abs_dz_ = cfg.overlap_;
        lp.r_scale_ = projection::ComputeEARScale(cfg.overlap_);
        break;
      case LensParam::kDualFisheyeEquidistant:
        lp.max_abs_dz_ = cfg.overlap_;
        lp.r_scale_ = projection::ComputeEDRScale(cfg.overlap_);
        break;
      case LensParam::kDualFisheyeStereographic:
        lp.max_abs_dz_ = cfg.overlap_;
        lp.r_scale_ = projection::ComputeSTRScale(cfg.overlap_);
        break;
      default:
        break;
    }
  }
  auto fn = GetProjFunc(t);
  float d[3] = { dx, dy, dz };
  int xy[2] = { -999, -999 };
  fn(lp, d, xy, 1);

  const int expect_px = xy[0];
  const int expect_py = xy[1];

  if (expect_px == -1 && expect_py == -1) {
    // Legacy miss (visible_range / cz<=0 cull) → shared count must be 0.
    EXPECT_EQ(res.count, 0) << label;
  } else {
    EXPECT_GE(res.count, 1) << label;
    if (res.count >= 1) {
      EXPECT_EQ(res.hits[0].px, expect_px) << label;
      EXPECT_EQ(res.hits[0].py, expect_py) << label;
      EXPECT_TRUE(res.hits[0].bump_landed) << label;
    }
  }
}

TEST(LmProj, ProjectExitPerTypeDefaultView) {
  // Default view: az=0, el=90, ro=0 → identity-ish (Rotation chain still non-trivial
  // due to the (-90+ro)/(90-el) offsets, but nothing weird).
  const struct {
    LensParam::LensType t;
    float fov;
  } kCases[] = {
    { LensParam::kLinear, 90.0f },
    { LensParam::kFisheyeEqualArea, 180.0f },
    { LensParam::kFisheyeEquidistant, 180.0f },
    { LensParam::kFisheyeStereographic, 180.0f },
    { LensParam::kFisheyeOrthographic, 180.0f },
    { LensParam::kRectangular, 90.0f },
    { LensParam::kDualFisheyeEqualArea, 180.0f },
    { LensParam::kDualFisheyeEquidistant, 180.0f },
    { LensParam::kDualFisheyeStereographic, 180.0f },
    { LensParam::kDualFisheyeOrthographic, 180.0f },
    { LensParam::kGlobe, 60.0f },  // globe fov ≤ 90 (MaxFov)
  };
  // Sample a handful of world dirs (must be unit or near-unit; hemispheres matter
  // for cull / dual-fisheye split).
  const float dirs[][3] = {
    { 0.0f, 0.0f, -1.0f },  // straight down (typical incoming ray)
    { 0.1f, 0.1f, -0.99f },  { 0.3f, -0.4f, -0.87f },
    { 0.5f, 0.5f, -0.707f }, { 0.0f, 0.0f, 1.0f },  // straight up (culled for kUpper, kept for kFull/kLower)
    { 0.7f, 0.2f, 0.68f },
  };
  for (const auto& c : kCases) {
    auto cfg = MakeRC(c.t, c.fov, 1024, 512, 0.0f, 90.0f, 0.0f);
    for (const auto& d : dirs) {
      char lbl[64];
      std::snprintf(lbl, sizeof(lbl), "type=%d dir=(%g,%g,%g)", static_cast<int>(c.t), d[0], d[1], d[2]);
      ExpectMainHitEqualsLegacy(c.t, cfg, d[0], d[1], d[2], lbl);
    }
  }
}

TEST(LmProj, ProjectExitPerTypeRotatedView) {
  // Non-default camera (all 3 Euler angles non-zero) — critical: rot[9] contract
  // must be applied on single-lens types but NOT on dual-fisheye/rectangular.
  // (task-unify-shared-projection risk 3.)
  const struct {
    LensParam::LensType t;
    float fov;
  } kCases[] = {
    { LensParam::kLinear, 60.0f },
    { LensParam::kFisheyeEqualArea, 180.0f },
    { LensParam::kFisheyeEquidistant, 180.0f },
    { LensParam::kFisheyeStereographic, 180.0f },
    { LensParam::kFisheyeOrthographic, 180.0f },
    { LensParam::kRectangular, 90.0f },
    { LensParam::kDualFisheyeEqualArea, 180.0f },
    { LensParam::kDualFisheyeEquidistant, 180.0f },
    { LensParam::kDualFisheyeStereographic, 180.0f },
    { LensParam::kDualFisheyeOrthographic, 180.0f },
    { LensParam::kGlobe, 60.0f },  // globe fov ≤ 90 (MaxFov)
  };
  const float dirs[][3] = {
    { 0.2f, -0.3f, -0.933f },
    { 0.6f, 0.1f, -0.79f },
    { 0.0f, 0.0f, -1.0f },
    { -0.5f, 0.4f, -0.768f },
  };
  for (const auto& c : kCases) {
    // az=42, el=60, ro=15 → arbitrary non-trivial rotation.
    auto cfg = MakeRC(c.t, c.fov, 1024, 512, 42.0f, 60.0f, 15.0f, /*overlap=*/0.0f, RenderConfig::kFull);
    for (const auto& d : dirs) {
      char lbl[80];
      std::snprintf(lbl, sizeof(lbl), "type=%d rot dir=(%g,%g,%g)", static_cast<int>(c.t), d[0], d[1], d[2]);
      ExpectMainHitEqualsLegacy(c.t, cfg, d[0], d[1], d[2], lbl);
    }
  }
}

TEST(LmProj, ProjectExitVisibleRangeCull) {
  // kLower should cull d[2]<0 (down-going) for single-lens types.
  auto cfg = MakeRC(LensParam::kLinear, 90.0f, 512, 512, 0.0f, 90.0f, 0.0f,
                    /*overlap=*/0.0f, RenderConfig::kLower);
  Rotation rot = lumice::MakeCameraRotation(cfg);
  auto pp = lumice::BuildProjParams(cfg, rot, 512.0f);
  auto r = lm_proj::ProjectExitToPixel(pp, 0.1f, 0.1f, -0.99f);
  EXPECT_EQ(r.count, 0) << "kLower should reject dz<0";
}

TEST(LmProj, ProjectExitDualFisheyeOverlapDualWrite) {
  // Overlap band → count=2, hit[1].bump_landed=false.
  const float overlap = 0.0872f;  // sin 5°
  auto cfg = MakeRC(LensParam::kDualFisheyeEqualArea, 180.0f, 1024, 512, 0.0f, 90.0f, 0.0f, overlap);
  Rotation rot = lumice::MakeCameraRotation(cfg);
  auto pp = lumice::BuildProjParams(cfg, rot, 512.0f);
  // Ray with |sky_z| < overlap → sky_z=-wz. Pick wz such that |sky_z|=|-wz|<overlap.
  auto r_in = lm_proj::ProjectExitToPixel(pp, 0.9f, 0.1f, 0.04f);
  EXPECT_EQ(r_in.count, 2);
  EXPECT_TRUE(r_in.hits[0].bump_landed);
  EXPECT_FALSE(r_in.hits[1].bump_landed);

  // Ray outside band → count=1.
  auto r_out = lm_proj::ProjectExitToPixel(pp, 0.5f, 0.5f, -0.707f);
  EXPECT_EQ(r_out.count, 1);
  EXPECT_TRUE(r_out.hits[0].bump_landed);
}

// =============== Rectangular wrap equivalence (risk 1 evidence) ===============
// Verifies that CPU's while-loop wrap and Metal's floor-based wrap output the
// same pixel index across a dense lon sweep including ±π boundaries. If any
// mismatch, plan.md risk 1 escalates to a real bug; if all match, both writings
// are provably interchangeable and Metal side (315.3) can adopt either.

TEST(LmProj, RectangularWrapWhileVsFloorEquivalence) {
  const int img_w = 2048;
  const float scale = static_cast<float>(img_w) / 2.0f / math::kPi;  // ~short_res/pi
  auto wrap_while = [](float lon) {
    while (lon < -math::kPi) {
      lon += 2 * math::kPi;
    }
    while (lon > math::kPi) {
      lon -= 2 * math::kPi;
    }
    return lon;
  };
  auto wrap_floor = [](float lon) {
    return lon - 2.0f * math::kPi * std::floor((lon + math::kPi) / (2.0f * math::kPi));
  };
  // Dense sweep + boundary values.
  int mismatches = 0;
  for (int i = -1000; i <= 1000; ++i) {
    float lon = static_cast<float>(i) * (math::kPi / 500.0f);  // covers ~[-2π, 2π]
    float a = wrap_while(lon);
    float b = wrap_floor(lon);
    int px_a = static_cast<int>(std::floor(a * scale + img_w / 2.0f + 0.5f));
    int px_b = static_cast<int>(std::floor(b * scale + img_w / 2.0f + 0.5f));
    px_a = ((px_a % img_w) + img_w) % img_w;
    px_b = ((px_b % img_w) + img_w) % img_w;
    if (px_a != px_b) {
      ++mismatches;
    }
  }
  // Evidence for plan.md risk 1: expect 0 mismatches at pixel granularity.
  // If this ever fires, Step 2 chose the safer path (CPU while-loop) — the
  // shared function stays parity-neutral. Metal/CUDA side is 315.3's concern.
  EXPECT_EQ(mismatches, 0);
}

// =============== Globe projection (315.4) ===============

// AC4 consistency anchor: the device-side kGlobeCameraD (projection_shared.h)
// MUST equal the GUI constant (src/gui/gui_constants.hpp:173, = shader
// globeInverse kGlobeCameraDist). The constant cannot cross the C-API boundary,
// so this guards the two copies from silently drifting.
TEST(LmProj, GlobeCameraDMatchesGuiConstant) {
  EXPECT_FLOAT_EQ(lm_proj::kGlobeCameraD, 4.0f)
      << "kGlobeCameraD must match src/gui/gui_constants.hpp kGlobeCameraD (=4.0)";
}

// The GUI globe forward (preview_renderer.cpp:919 ProjectGlobe) is the numerical
// inverse of the shader `globeInverse` (preview_renderer.cpp:288). This is the
// exact eye-space math the shared render-path globe branch transcribes. Verify
// the transcribed math IS the inverse of the sphere ray-cast over sample pixel
// offsets, so a CLI globe render matches the GUI globe preview (AC2 math check).
TEST(LmProj, GlobeForwardInvertsShaderSphereCast) {
  constexpr float kD = 4.0f;  // kGlobeCameraD
  const float img_radius = 256.0f;
  const float half_fov = 30.0f * math::kDegreeToRad;  // fov=60
  const float focal = img_radius / std::tan(half_fov);

  // Shader globeInverse: pixel offset (y-up) → world/eye dir on the sphere.
  auto globe_inverse = [&](float px, float py, float out[3]) -> bool {
    float dx = px, dy = py, dz = -focal;
    float inv_len = 1.0f / std::sqrt(dx * dx + dy * dy + dz * dz);
    dx *= inv_len;
    dy *= inv_len;
    dz *= inv_len;
    float b = kD * dz;
    float c = kD * kD - 1.0f;
    float disc = b * b - c;
    if (disc < 0.0f) {
      return false;
    }
    float t = -b - std::sqrt(disc);
    if (t <= 0.0f) {
      return false;
    }
    out[0] = kD * 0.0f + t * dx;
    out[1] = t * dy;
    out[2] = kD + t * dz;  // hit_eye = (0,0,D) + t*d ; unit length on sphere
    return true;
  };
  // GUI forward ProjectGlobe: eye_dir → pixel offset (the math transcribed into
  // the shared globe branch, expressed in the GUI's eye frame).
  auto globe_forward = [&](const float e[3], float out[2]) -> bool {
    if (e[2] <= 1.0f / kD) {
      return false;
    }
    float denom = kD - e[2];
    out[0] = e[0] / denom * focal;
    out[1] = e[1] / denom * focal;
    return true;
  };

  // Offsets kept inside the sphere silhouette (radius ~focal*tan(asin(1/D))
  // ≈ 114 px at fov=60): rays outside it miss the sphere by design.
  const float samples[][2] = { { 0.0f, 0.0f },   { 40.0f, 0.0f },   { 0.0f, -55.0f },
                               { 70.0f, 30.0f }, { -60.0f, 80.0f }, { 90.0f, -40.0f } };
  for (const auto& s : samples) {
    float e[3];
    ASSERT_TRUE(globe_inverse(s[0], s[1], e)) << "sample (" << s[0] << "," << s[1] << ") should hit sphere";
    float back[2];
    ASSERT_TRUE(globe_forward(e, back)) << "eye dir should be front-hemisphere";
    EXPECT_NEAR(back[0], s[0], 1e-2f) << "round-trip px";
    EXPECT_NEAR(back[1], s[1], 1e-2f) << "round-trip py";
  }
}

// Pin the shared render-path globe branch to its documented formula: reusing the
// single-lens eye vector c = R^T·(-w), cull when c.z >= -1/D, denom = D + c.z,
// offset = (c.x,c.y)/denom * scale, scale = short_pix/2/tan(fov/2). This keeps
// globe's numerator/pixel-mapping identical to linear (so globe inherits linear's
// verified GUI orientation by transitivity) while swapping only the denom+cull.
TEST(LmProj, GlobeRenderPathMatchesDocumentedFormula) {
  const int w = 1024, h = 512;
  const float fov = 50.0f;
  auto cfg = MakeRC(LensParam::kGlobe, fov, w, h, 20.0f, 35.0f, 10.0f);
  Rotation rot = lumice::MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(w, h));
  auto pp = lumice::BuildProjParams(cfg, rot, short_pix);

  const float* mat = rot.GetMat();  // row-major
  const float kD = lm_proj::kGlobeCameraD;
  const float scale = short_pix / 2.0f / std::tan(fov * math::kDegreeToRad / 2.0f);

  const float dirs[][3] = { { 0.0f, 0.0f, -1.0f },  { 0.0f, 0.0f, 1.0f },   { 0.3f, -0.2f, -0.93f },
                            { -0.4f, 0.5f, 0.77f }, { 0.1f, 0.1f, -0.99f }, { 0.6f, -0.3f, 0.74f } };
  int visible_seen = 0;
  for (const auto& d : dirs) {
    // Reference: c = R^T·(-w) (ApplyRotTranspose semantics).
    float cx = mat[0] * -d[0] + mat[3] * -d[1] + mat[6] * -d[2];
    float cy = mat[1] * -d[0] + mat[4] * -d[1] + mat[7] * -d[2];
    float cz = mat[2] * -d[0] + mat[5] * -d[1] + mat[8] * -d[2];
    auto res = lm_proj::ProjectExitToPixel(pp, d[0], d[1], d[2]);
    if (cz >= -1.0f / kD) {
      EXPECT_EQ(res.count, 0) << "expected cull for cz=" << cz;
    } else {
      ++visible_seen;
      float denom = kD + cz;
      int expect_px = static_cast<int>(std::floor(cx / denom * scale + w / 2.0f + 0.5f));
      int expect_py = static_cast<int>(std::floor(cy / denom * scale + h / 2.0f + 0.5f));
      ASSERT_EQ(res.count, 1);
      EXPECT_EQ(res.hits[0].px, expect_px);
      EXPECT_EQ(res.hits[0].py, expect_py);
      EXPECT_TRUE(res.hits[0].bump_landed);
    }
  }
  EXPECT_GT(visible_seen, 0) << "at least one sample must be globe-visible";
}

}  // namespace
}  // namespace projection
}  // namespace lumice
