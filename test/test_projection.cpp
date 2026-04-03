#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "config/render_config.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"

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

}  // namespace
}  // namespace projection
}  // namespace lumice
