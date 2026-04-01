#include <gtest/gtest.h>

#include <cmath>
#include <random>

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
    dx /= len; dy /= len; dz /= len;
    auto fwd = LinearForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = LinearInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
}

// =============== Type A: Fisheye Equal Area ===============

TEST(Projection, FisheyeEqualAreaForwardOnAxis) {
  auto r = FisheyeEqualAreaForward(0, 0, 1);
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.x, 0, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, FisheyeEqualAreaForwardHorizon) {
  // At horizon: dz=0, theta=pi/2, r=sin(pi/4)=sqrt(2)/2
  float dx = 1, dy = 0, dz = 0.001f;  // slightly above horizon
  auto r = FisheyeEqualAreaForward(dx, dy, dz);
  EXPECT_TRUE(r.valid);
  // r should be close to sin(pi/4)
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, std::sin(math::kPi_4), 0.01f);
}

TEST(Projection, FisheyeEqualAreaForwardBehind) {
  auto r = FisheyeEqualAreaForward(0, 0, -1);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, FisheyeEqualAreaRoundTrip) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 1000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = std::abs(dist(rng)) + 0.01f;
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= len; dy /= len; dz /= len;
    auto fwd = FisheyeEqualAreaForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = FisheyeEqualAreaInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEpsPolar);
    EXPECT_NEAR(inv.y, dy, kEpsPolar);
    EXPECT_NEAR(inv.z, dz, kEpsPolar);
  }
}

// =============== Type A: Fisheye Equidistant ===============

TEST(Projection, FisheyeEquidistantRoundTrip) {
  std::mt19937 rng(43);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 1000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = std::abs(dist(rng)) + 0.01f;
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= len; dy /= len; dz /= len;
    auto fwd = FisheyeEquidistantForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = FisheyeEquidistantInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEpsPolar);
    EXPECT_NEAR(inv.y, dy, kEpsPolar);
    EXPECT_NEAR(inv.z, dz, kEpsPolar);
  }
}

// =============== Type A: Fisheye Stereographic ===============

TEST(Projection, FisheyeStereographicRoundTrip) {
  std::mt19937 rng(44);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 1000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = std::abs(dist(rng)) + 0.01f;
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= len; dy /= len; dz /= len;
    auto fwd = FisheyeStereographicForward(dx, dy, dz);
    ASSERT_TRUE(fwd.valid);
    auto inv = FisheyeStereographicInverse(fwd.x, fwd.y);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEpsPolar);
    EXPECT_NEAR(inv.y, dy, kEpsPolar);
    EXPECT_NEAR(inv.z, dz, kEpsPolar);
  }
}

// =============== Type B: Dual Fisheye Equal Area ===============

TEST(Projection, DualFisheyeEAForwardPole) {
  // Upper pole: (0, 0, 1) -> center of upper disc
  auto r = DualFisheyeEqualAreaForward(0, 0, 1);
  EXPECT_TRUE(r.is_upper);
  EXPECT_NEAR(r.x, 0, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, DualFisheyeEAForwardEquator) {
  // Equator: (1, 0, 0) -> r=1
  auto r = DualFisheyeEqualAreaForward(1, 0, 0);
  EXPECT_TRUE(r.is_upper);  // dz=0 -> upper
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

TEST(Projection, DualFisheyeEAForwardLowerPole) {
  auto r = DualFisheyeEqualAreaForward(0, 0, -1);
  EXPECT_FALSE(r.is_upper);
  EXPECT_NEAR(r.x, 0, kEps);
  EXPECT_NEAR(r.y, 0, kEps);
}

TEST(Projection, DualFisheyeEARoundTrip) {
  std::mt19937 rng(45);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f) continue;
    dx /= len; dy /= len; dz /= len;
    auto fwd = DualFisheyeEqualAreaForward(dx, dy, dz);
    auto inv = DualFisheyeEqualAreaInverse(fwd.x, fwd.y, fwd.is_upper);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
}

TEST(Projection, DualFisheyeEAEqualArea) {
  // Verify equal-area property: uniform sphere sampling -> uniform disc distribution.
  // Sample many uniform directions, project, bin by radius^2 (which should be uniform for equal-area).
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
    auto fwd = DualFisheyeEqualAreaForward(dx, dy, z);
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

TEST(Projection, DualFisheyeEAInverseBeyondDomain) {
  auto r = DualFisheyeEqualAreaInverse(1.1f, 0, true);
  EXPECT_FALSE(r.valid);
}

// =============== Type B: Dual Fisheye Equidistant ===============

TEST(Projection, DualFisheyeEDRoundTrip) {
  std::mt19937 rng(47);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f) continue;
    dx /= len; dy /= len; dz /= len;
    auto fwd = DualFisheyeEquidistantForward(dx, dy, dz);
    auto inv = DualFisheyeEquidistantInverse(fwd.x, fwd.y, fwd.is_upper);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
}

TEST(Projection, DualFisheyeEDEquatorNorm) {
  // At equator, r should be 1
  auto r = DualFisheyeEquidistantForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

// =============== Type B: Dual Fisheye Stereographic ===============

TEST(Projection, DualFisheyeSTRoundTrip) {
  std::mt19937 rng(48);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f) continue;
    dx /= len; dy /= len; dz /= len;
    auto fwd = DualFisheyeStereographicForward(dx, dy, dz);
    auto inv = DualFisheyeStereographicInverse(fwd.x, fwd.y, fwd.is_upper);
    ASSERT_TRUE(inv.valid);
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
}

TEST(Projection, DualFisheyeSTEquatorNorm) {
  auto r = DualFisheyeStereographicForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

// =============== Type C: Rectangular ===============

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
    if (len < 1e-6f) continue;
    dx /= len; dy /= len; dz /= len;
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

TEST(Projection, InverseBeyondDomainFisheyeEA) {
  // r > 1 -> invalid
  auto r = FisheyeEqualAreaInverse(1.5f, 0);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, InverseBeyondDomainFisheyeED) {
  // r > pi/2 -> invalid
  auto r = FisheyeEquidistantInverse(2.0f, 0);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, InverseBeyondDomainFisheyeST) {
  // theta > pi/2 -> invalid
  // tan(pi/4) = 1, so r > 1 means theta > pi/2
  auto r = FisheyeStereographicInverse(1.5f, 0);
  EXPECT_FALSE(r.valid);
}

TEST(Projection, DualFisheyeForwardAtPoles) {
  // All three dual variants: pole -> center (0,0)
  auto ea = DualFisheyeEqualAreaForward(0, 0, 1);
  EXPECT_NEAR(ea.x, 0, kEps);
  EXPECT_NEAR(ea.y, 0, kEps);

  auto ed = DualFisheyeEquidistantForward(0, 0, 1);
  EXPECT_NEAR(ed.x, 0, kEps);
  EXPECT_NEAR(ed.y, 0, kEps);

  auto st = DualFisheyeStereographicForward(0, 0, 1);
  EXPECT_NEAR(st.x, 0, kEps);
  EXPECT_NEAR(st.y, 0, kEps);
}

}  // namespace
}  // namespace projection
}  // namespace lumice
