#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
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

// The single-lens fisheye render domain runs to theta = 180 deg (474.1), but a round trip through
// it is not uniformly conditioned: every type recovers the AZIMUTH by dividing by rho = sin(theta),
// which collapses at the antipode. Equal-area shows it most directly — the inverse recovers
// z = 1 - r^2 with r^2 -> 2, so the subtraction keeps an absolute error of ~2e-7 while the factor
// sqrt(1 + z) it feeds shrinks to 1e-3, and the relative error in the recovered x/y grows as
// 1 / sqrt(1 + z). The round-trip tests below therefore run the well-conditioned bulk of the
// domain at kEps and hand the last 26 deg to SingleFisheyeRimRoundTrip, which states the looser
// tolerance the float arithmetic actually allows rather than pretending the whole domain is equal.
constexpr float kRoundTripMinDz = -0.9f;  // theta <= 154 deg

// Equal-area's past-equator tolerance. Its inverse recovers z as 1 - r^2 with r^2 running to 2, so
// the absolute error in z stays flat while the factor sqrt(1 + z) it feeds shrinks — a few units in
// the last place at the equator become a few 1e-6 by theta = 154 deg. The equator-and-up half of
// the domain, which this test already covered before 474.1, keeps kEps: the half that was added is
// the half that is less well conditioned, and saying so per-sample is more useful than relaxing the
// whole test. (Same spirit as kEpsPolar above.) Equidistant and stereographic need no equivalent —
// they recover z through cos() of a well-conditioned angle.
constexpr float kEpsPastEquator = 5e-6f;

// Angle between two unit vectors, in radians — the tolerance that means the same thing for every
// projection type, unlike a per-component epsilon.
float AngleBetween(const Dir3& a, float bx, float by, float bz) {
  const float dot = std::max(-1.0f, std::min(1.0f, a.x * bx + a.y * by + a.z * bz));
  return std::acos(dot);
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
  // Direct round trip over the whole widened domain — no hemisphere flip. Before 474.1 this test
  // projected |dz| and flipped the sign afterwards, which only ever exercised theta <= 90 deg; the
  // past-equator half of the domain the render path now writes to went untested here.
  std::mt19937 rng(45);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  size_t past_equator = 0;
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;
    if (dz < kRoundTripMinDz) {
      continue;  // see kRoundTripMinDz — the rim is SingleFisheyeRimRoundTrip's job
    }
    if (dz < 0.0f) {
      ++past_equator;
    }

    auto fwd = FisheyeEqualAreaForward(dx, dy, dz);
    auto inv = FisheyeEqualAreaInverse(fwd.x, fwd.y);
    if (!inv.valid) {
      ADD_FAILURE() << "the inverse rejected a direction the render path projects; dz = " << dz;
      continue;
    }
    ExpectUnitVector(inv);
    const float tol = (dz >= 0.0f) ? kEps : kEpsPastEquator;
    EXPECT_NEAR(inv.x, dx, tol) << "dz = " << dz;
    EXPECT_NEAR(inv.y, dy, tol) << "dz = " << dz;
    EXPECT_NEAR(inv.z, dz, tol) << "dz = " << dz;
  }
  EXPECT_GT(past_equator, 100u) << "the sample must actually reach past the equator, or this test "
                                   "still only covers the pre-474.1 domain";
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

TEST(Projection, FisheyeEqualAreaInverseInsideTheWidenedDomain) {
  // r = 1.1 used to be rejected: the domain stopped at r = 1, the equator. It is now inside, and
  // what it recovers is the point of 474.1 — a direction BELOW the horizon of the lens axis.
  // r = sqrt(2) sin(theta/2)  =>  theta = 2 asin(1.1 / sqrt(2)) = 101.1 deg.
  auto r = FisheyeEqualAreaInverse(1.1f, 0);
  ASSERT_TRUE(r.valid);
  ExpectUnitVector(r);
  EXPECT_LT(r.z, 0.0f) << "r > 1 is the past-equator region; a positive z would mean the widening "
                          "moved the radius mapping instead of extending it";
  EXPECT_NEAR(std::acos(r.z) * math::kRadToDegree, 2.0f * std::asin(1.1f / std::sqrt(2.0f)) * math::kRadToDegree,
              1e-3f);
}

// =============== Fisheye Equidistant ===============

TEST(Projection, FisheyeEquidistantEquatorNorm) {
  // At equator, r should be 1
  auto r = FisheyeEquidistantForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

TEST(Projection, FisheyeEquidistantRoundTrip) {
  // Direct round trip over the widened domain — see FisheyeEqualAreaRoundTrip for why the
  // hemisphere flip is gone.
  std::mt19937 rng(47);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  size_t past_equator = 0;
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;
    if (dz < kRoundTripMinDz) {
      continue;
    }
    if (dz < 0.0f) {
      ++past_equator;
    }

    auto fwd = FisheyeEquidistantForward(dx, dy, dz);
    auto inv = FisheyeEquidistantInverse(fwd.x, fwd.y);
    if (!inv.valid) {
      ADD_FAILURE() << "the inverse rejected a direction the render path projects; dz = " << dz;
      continue;
    }
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
  EXPECT_GT(past_equator, 100u) << "the sample must actually reach past the equator";
}

// =============== Fisheye Stereographic ===============

TEST(Projection, FisheyeStereographicEquatorNorm) {
  auto r = FisheyeStereographicForward(1, 0, 0);
  float radius = std::sqrt(r.x * r.x + r.y * r.y);
  EXPECT_NEAR(radius, 1.0f, kEps);
}

TEST(Projection, FisheyeStereographicRoundTrip) {
  // Direct round trip over the widened domain — see FisheyeEqualAreaRoundTrip. r = tan(theta/2)
  // is the one type whose radius diverges rather than converging to a rim, so its far end belongs
  // to SingleFisheyeRimRoundTrip for a second reason on top of the shared azimuth conditioning.
  std::mt19937 rng(48);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  size_t past_equator = 0;
  for (int i = 0; i < 2000; i++) {
    float dx = dist(rng), dy = dist(rng), dz = dist(rng);
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f)
      continue;
    dx /= len;
    dy /= len;
    dz /= len;
    if (dz < kRoundTripMinDz) {
      continue;
    }
    if (dz < 0.0f) {
      ++past_equator;
    }

    auto fwd = FisheyeStereographicForward(dx, dy, dz);
    auto inv = FisheyeStereographicInverse(fwd.x, fwd.y);
    if (!inv.valid) {
      ADD_FAILURE() << "the inverse rejected a direction the render path projects; dz = " << dz;
      continue;
    }
    ExpectUnitVector(inv);
    EXPECT_NEAR(inv.x, dx, kEps);
    EXPECT_NEAR(inv.y, dy, kEps);
    EXPECT_NEAR(inv.z, dz, kEps);
  }
  EXPECT_GT(past_equator, 100u) << "the sample must actually reach past the equator";
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

// The rim radius of each single-fisheye inverse at r_scale = 1, i.e. the value of that type's
// radius formula at the largest theta the render path produces. Restated here from the formulas
// rather than read out of projection.cpp, so a change to either side has to be argued for twice.
float RimRadius(LensParam::LensType t) {
  switch (t) {
    case LensParam::kFisheyeEqualArea:
      return std::sqrt(2.0f);  // sqrt(2) sin(theta/2) at theta = 180 deg
    case LensParam::kFisheyeEquidistant:
      return 2.0f;  // theta / (pi/2) at theta = 180 deg
    case LensParam::kFisheyeStereographic:
      // tan(theta/2) at the numerical cull floor, which is the half angle of this lens's fov
      // ceiling — pinned against MaxFov by StereographicCullFloorIsTheFovCeilingHalfAngle below.
      return std::tan(0.5f * std::acos(lm_proj::kFisheyeStereographicMinCz));
    default:  // kFisheyeOrthographic — sin(theta) is not injective past the equator
      return 1.0f;
  }
}

Dir3 InverseFor(LensParam::LensType t, float x, float y) {
  switch (t) {
    case LensParam::kFisheyeEqualArea:
      return FisheyeEqualAreaInverse(x, y, 1.0f);
    case LensParam::kFisheyeEquidistant:
      return FisheyeEquidistantInverse(x, y, 1.0f);
    case LensParam::kFisheyeStereographic:
      return FisheyeStereographicInverse(x, y, 1.0f);
    default:
      return FisheyeOrthographicInverse(x, y, 1.0f);
  }
}

ProjXY ForwardFor(LensParam::LensType t, float dx, float dy, float dz) {
  switch (t) {
    case LensParam::kFisheyeEqualArea:
      return FisheyeEqualAreaForward(dx, dy, dz, 1.0f);
    case LensParam::kFisheyeEquidistant:
      return FisheyeEquidistantForward(dx, dy, dz, 1.0f);
    case LensParam::kFisheyeStereographic:
      return FisheyeStereographicForward(dx, dy, dz, 1.0f);
    default:
      return FisheyeOrthographicForward(dx, dy, dz, 1.0f);
  }
}

// The cull floor ProjectExitToPixel applies to each single-lens fisheye. Restated from the
// constants in projection_shared.h; SingleFisheyeCullAndInverseDomainAgree checks this restatement
// against the real ProjectExitToPixel rather than trusting it.
float CullFloorCz(LensParam::LensType t) {
  switch (t) {
    case LensParam::kFisheyeEqualArea:
      return lm_proj::kFisheyeEqualAreaMinCz;
    case LensParam::kFisheyeEquidistant:
      return lm_proj::kFisheyeEquidistantMinCz;
    case LensParam::kFisheyeStereographic:
      return lm_proj::kFisheyeStereographicMinCz;
    default:
      return 0.0f;  // orthographic — strict: its branch rejects cz <= 0
  }
}

const LensParam::LensType kSingleFisheyeTypes[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                                    LensParam::kFisheyeStereographic, LensParam::kFisheyeOrthographic };

TEST(Projection, InverseRejectsBeyondTheRim) {
  // r = 1.5 used to be the "beyond domain" probe for all three widened types. It is now INSIDE two
  // of the three domains, so a fixed radius cannot serve them any more — each type is probed
  // against its own rim.
  for (LensParam::LensType t : kSingleFisheyeTypes) {
    const float rim = RimRadius(t);
    EXPECT_TRUE(InverseFor(t, rim * 0.999f, 0.0f).valid) << "type " << static_cast<int>(t) << ": just inside the rim";
    EXPECT_FALSE(InverseFor(t, rim * 1.001f, 0.0f).valid) << "type " << static_cast<int>(t) << ": just outside the rim";
    // And far outside, so a rim that silently grew would still be caught.
    EXPECT_FALSE(InverseFor(t, rim * 4.0f, 0.0f).valid) << "type " << static_cast<int>(t) << ": far outside the rim";
  }
}

TEST(Projection, InverseAtTheRimRecoversTheAntipode) {
  // The rim is not an arbitrary cut-off: it is where theta reaches its maximum. For the two types
  // that run to 180 deg the recovered direction there must be the antipode of the lens axis.
  const Dir3 ea = FisheyeEqualAreaInverse(std::sqrt(2.0f), 0.0f, 1.0f);
  ASSERT_TRUE(ea.valid);
  EXPECT_NEAR(ea.z, -1.0f, 1e-5f);
  const Dir3 ed = FisheyeEquidistantInverse(2.0f, 0.0f, 1.0f);
  ASSERT_TRUE(ed.valid);
  EXPECT_NEAR(ed.z, -1.0f, 1e-5f);
}

TEST(Projection, StereographicCullFloorIsTheFovCeilingHalfAngle) {
  // AC3's closed-form reason, as a mechanical assertion rather than a comment: the per-ray cull
  // floor is not an independently chosen number, it is the half angle of the fov ceiling
  // render_config.cpp already imposes on this lens. If MaxFov changes and the constant does not
  // (or the other way round), this is what says so.
  const float theta_max_deg = std::acos(lm_proj::kFisheyeStereographicMinCz) * math::kRadToDegree;
  EXPECT_NEAR(theta_max_deg, lumice::MaxFov(LensParam::kFisheyeStereographic) / 2.0f, 1e-3f);
  // ... and the radius that half angle implies is far outside any frame, which is why widening
  // stereographic needs no product decision about where to stop.
  EXPECT_GT(RimRadius(LensParam::kFisheyeStereographic), 200.0f);
}

TEST(Projection, SingleFisheyeCullAndInverseDomainAgree) {
  // AC2, mechanically: "a ray could have landed on this pixel" and "the inverse accepts this
  // pixel" must be the same statement, because lens_proj_build.hpp's render-domain mask is built
  // from the second and describes the first. Driven through the real ProjectExitToPixel (identity
  // rotation, visible = full, so the only cull left is the per-type cz floor under test) rather
  // than through a restatement of its branch structure.
  lm_proj::ProjParams p{};
  p.img_w = 1024;
  p.img_h = 1024;
  p.scale = 1.0f;
  p.r_scale = 1.0f;
  const float kIdentity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  std::memcpy(p.rot, kIdentity, sizeof(kIdentity));

  for (LensParam::LensType t : kSingleFisheyeTypes) {
    p.proj_type = static_cast<int>(t);
    const float floor_cz = CullFloorCz(t);
    const float rim = RimRadius(t);
    size_t rendered = 0;
    size_t culled = 0;
    // 20001 steps of theta, plus four azimuths so the check is not confined to one meridian.
    for (int i = 0; i <= 20000; ++i) {
      const float theta = static_cast<float>(i) / 20000.0f * math::kPi;
      for (float az : { 0.0f, 0.7f, 2.4f, 5.1f }) {
        const float cz = std::cos(theta);
        const float cx = std::sin(theta) * std::cos(az);
        const float cy = std::sin(theta) * std::sin(az);
        // ProjectExitToPixel computes c = R^T * (-w); with R = I that is w = -c.
        const auto res = lm_proj::ProjectExitToPixel(p, -cx, -cy, -cz);
        const bool inside_floor = (t == LensParam::kFisheyeOrthographic) ? (cz > floor_cz) : (cz >= floor_cz);
        if (!inside_floor) {
          if (res.count != 0) {
            ADD_FAILURE() << "type " << static_cast<int>(t) << ": theta = " << theta
                          << " is past the cull floor but produced a hit";
          }
          ++culled;
          continue;
        }
        if (res.count != 1) {
          ADD_FAILURE() << "type " << static_cast<int>(t) << ": theta = " << theta
                        << " is inside the cull floor but was rejected";
          continue;
        }
        ++rendered;
        const ProjXY fwd = ForwardFor(t, cx, cy, cz);
        const float r = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
        EXPECT_TRUE(InverseFor(t, fwd.x, fwd.y).valid)
            << "type " << static_cast<int>(t) << ": theta = " << theta << " renders at r = " << r
            << " but the inverse (rim " << rim << ") rejects it — the mask would call this pixel empty";
      }
    }
    EXPECT_GT(rendered, 0u) << "type " << static_cast<int>(t);
    EXPECT_GT(culled, 0u) << "type " << static_cast<int>(t)
                          << ": every type must still cull SOMETHING, or the "
                             "floor under test is not being exercised";
    // The other direction: the inverse must not accept a meaningfully larger disc than the forward
    // can reach, or the mask promises sky no ray can paint. Evaluated AT the floor rather than off
    // the sweep above, whose theta step is coarser than the last decade each floor sits in.
    // 1e-3 of the rim is well under a pixel at any resolution these lenses are used at.
    const float rho_floor = std::sqrt(std::max(0.0f, 1.0f - floor_cz * floor_cz));
    const ProjXY at_floor = ForwardFor(t, rho_floor, 0.0f, floor_cz);
    const float r_floor = std::sqrt(at_floor.x * at_floor.x + at_floor.y * at_floor.y);
    EXPECT_NEAR(r_floor, rim, rim * 1e-3f) << "type " << static_cast<int>(t);
  }
}

TEST(Projection, SingleFisheyeRimRoundTrip) {
  // The last 14 deg the *RoundTrip tests hand over (see kRoundTripMinDz). Two separate claims, and
  // they degrade differently: the RADIUS stays accurate all the way to the floor, while the
  // AZIMUTH does not, because every inverse recovers it by dividing by rho = sin(theta).
  for (LensParam::LensType t :
       { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant, LensParam::kFisheyeStereographic }) {
    const float floor_cz = CullFloorCz(t);
    for (int i = 0; i <= 200; ++i) {
      const float cz = kRoundTripMinDz + (floor_cz - kRoundTripMinDz) * static_cast<float>(i) / 200.0f;
      const float rho = std::sqrt(std::max(0.0f, 1.0f - cz * cz));
      const float az = 0.9f;
      const float cx = rho * std::cos(az);
      const float cy = rho * std::sin(az);
      const ProjXY fwd = ForwardFor(t, cx, cy, cz);
      const Dir3 inv = InverseFor(t, fwd.x, fwd.y);
      if (!inv.valid) {
        ADD_FAILURE() << "type " << static_cast<int>(t) << ": the inverse rejected the rim; cz = " << cz;
        continue;
      }
      ExpectUnitVector(inv);
      // Radius: well conditioned, and the half of the round trip the render domain depends on.
      EXPECT_NEAR(inv.z, cz, 1e-4f) << "type " << static_cast<int>(t) << " cz = " << cz;
      // Direction: 1e-3 rad = 0.06 deg. At this distance from the antipode a whole degree of
      // azimuth spans well under a pixel of arc, so this is tighter than anything observable.
      EXPECT_LT(AngleBetween(inv, cx, cy, cz), 1e-3f) << "type " << static_cast<int>(t) << " cz = " << cz;
    }
  }
}

TEST(Projection, FisheyeForwardAtPoles) {
  // North pole -> centre (0,0) for all three: theta = 0, and every azimuth agrees there.
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

TEST(Projection, ForwardAtTheAntipodeIsCulledRatherThanCollapsedToTheCentre) {
  // The south pole is the one direction where these *Forward functions still answer wrongly, and
  // deliberately so: rho = 0 there, every azimuth is equally correct, and both equal-area (via its
  // dz clamp) and equidistant (via its rho guard) return (0,0) — the CENTRE, which is where
  // theta = 0 belongs, not theta = 180 deg. Widening the domain is what made that reachable, and
  // the fix is the cull floor rather than a change to the shared functions, because those same
  // functions serve the dual-fisheye path where rho = 0 really is the pole and (0,0) really is the
  // right answer. This test pins both halves: the functions still collapse, and the render path
  // never lets a ray get there.
  const auto ea = FisheyeEqualAreaForward(0, 0, -1);
  EXPECT_NEAR(std::sqrt(ea.x * ea.x + ea.y * ea.y), 0.0f, kEps) << "the collapse is still there";
  const auto ed = FisheyeEquidistantForward(0, 0, -1);
  EXPECT_NEAR(std::sqrt(ed.x * ed.x + ed.y * ed.y), 0.0f, kEps) << "the collapse is still there";

  lm_proj::ProjParams p{};
  p.img_w = 256;
  p.img_h = 256;
  p.scale = 1.0f;
  p.r_scale = 1.0f;
  const float kIdentity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  std::memcpy(p.rot, kIdentity, sizeof(kIdentity));
  for (LensParam::LensType t : kSingleFisheyeTypes) {
    p.proj_type = static_cast<int>(t);
    // c = (0,0,-1) is the antipode of the lens axis; w = -c.
    EXPECT_EQ(lm_proj::ProjectExitToPixel(p, 0.0f, 0.0f, 1.0f).count, 0)
        << "type " << static_cast<int>(t) << ": the antipode must be culled, not painted at the centre";
  }
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
    // Legacy miss (cz<=0 / per-type MinCz cull) → shared count must be 0.
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

TEST(LmProj, ProjectExitIgnoresVisibleRangeEntirely) {
  // The inverse of what this used to assert. Until 478.2 the single-lens branch opened with a
  // `visible_range` cull, and this test pinned it: kLower rejected an up-facing camera's
  // down-going ray, count == 0. `visible` is a DISPLAY clip — it decides which pixels reach the
  // screen, never which rays reach the buffer — and it was applied by this one branch and by no
  // other lens family, which is the asymmetry 478.2 removed. So the statement is now that the
  // projection is INDIFFERENT to the setting: the same ray must land on the same pixel under all
  // three values.
  //
  // Asserted as an equality across the three settings rather than as `count == 1` alone: a
  // reinstated cull that happened to keep this particular ray would still pass the weaker form.
  const float wx = 0.1f;
  const float wy = 0.1f;
  const float wz = -0.99f;
  lm_proj::ProjResult results[3];
  const RenderConfig::VisibleRange ranges[3]{ RenderConfig::kUpper, RenderConfig::kLower, RenderConfig::kFull };
  for (int i = 0; i < 3; i++) {
    auto cfg = MakeRC(LensParam::kLinear, 90.0f, 512, 512, 0.0f, 90.0f, 0.0f, /*overlap=*/0.0f, ranges[i]);
    Rotation rot = lumice::MakeCameraRotation(cfg);
    auto pp = lumice::BuildProjParams(cfg, rot, 512.0f);
    results[i] = lm_proj::ProjectExitToPixel(pp, wx, wy, wz);
  }
  ASSERT_EQ(results[2].count, 1) << "the ray must land under kFull, or the comparisons below are vacuous";
  for (int i = 0; i < 2; i++) {
    EXPECT_EQ(results[i].count, results[2].count)
        << "visible=" << static_cast<int>(ranges[i]) << ": the projection culled a ray the display clip owns";
    EXPECT_EQ(results[i].hits[0].px, results[2].hits[0].px) << "visible=" << static_cast<int>(ranges[i]);
    EXPECT_EQ(results[i].hits[0].py, results[2].hits[0].py) << "visible=" << static_cast<int>(ranges[i]);
  }
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

// =============== Rectangular follows the full camera pose ===============
// The equirectangular map is oriented by the camera, and by the WHOLE camera: azimuth, elevation
// and roll all move it. It used to consume only the azimuth (the host reduced the camera
// rotation to a single `az0` scalar, since removed, and the forward subtracted it from the
// longitude), so a
// config could tilt or roll the camera and get a bit-identical frame back. Owner decision
// 2026-09-02: core keeps maximum flexibility and follows the full pose; the GUI deliberately does
// NOT (it wants a fixed all-sky texture and does every pose transform on the front end) — see
// doc/coordinate-convention.md and test_visible_mask_gui_parity.cpp for that half.

// The frame a rectangular ray lands in, for one camera pose.
std::array<int, 2> RectangularPixel(float az, float el, float ro, const float d[3]) {
  const int w = 1024;
  const int h = 512;
  const auto cfg = MakeRC(LensParam::kRectangular, 90.0f, w, h, az, el, ro);
  const Rotation rot = lumice::MakeCameraRotation(cfg);
  const auto pp = lumice::BuildProjParams(cfg, rot, static_cast<float>(std::min(w, h)));
  const auto r = lm_proj::ProjectExitToPixel(pp, d[0], d[1], d[2]);
  if (r.count == 0) {
    return { -1, -1 };
  }
  return { r.hits[0].px, r.hits[0].py };
}

TEST(LmProj, RectangularFollowsCameraElevationAndRoll) {
  // Probe directions chosen off every symmetry plane of the fixture, so that a pose change moving
  // the map cannot leave a probe on its own image by coincidence.
  const float dirs[][3] = {
    { 0.371391f, -0.557086f, -0.742781f },
    { -0.640184f, 0.256074f, -0.724209f },
    { 0.267261f, 0.534522f, 0.801784f },
    { -0.301511f, -0.904534f, 0.301511f },
  };
  const float kAz = 30.0f;
  for (const auto& d : dirs) {
    const auto base = RectangularPixel(kAz, 0.0f, 0.0f, d);
    char lbl[80];
    std::snprintf(lbl, sizeof(lbl), "dir=(%g,%g,%g)", d[0], d[1], d[2]);
    if (base[0] == -1) {
      // Non-fatal: rectangular culls nothing, so this cannot happen — but one unimaged probe
      // must not swallow the remaining rows.
      ADD_FAILURE() << lbl << ": the baseline pose does not image the probe direction";
      continue;
    }

    for (float el : { 25.0f, -40.0f }) {
      const auto moved = RectangularPixel(kAz, el, 0.0f, d);
      EXPECT_NE(base, moved) << lbl << " el=" << el << ": elevation must move the equirectangular map";
    }
    for (float ro : { 35.0f, -70.0f }) {
      const auto moved = RectangularPixel(kAz, 0.0f, ro, d);
      EXPECT_NE(base, moved) << lbl << " ro=" << ro << ": roll must move the equirectangular map";
    }
  }
}

TEST(LmProj, RectangularAtZeroElevationAndRollReproducesTheAzimuthOnlyForm) {
  // The regression anchor for the change above, and the reason the GUI needs no counterpart: at
  // el = ro = 0 — the pose every full-sky lens is pinned to on the GUI side, and the pose every
  // shipped rectangular config uses — the pose-following form must be POINTWISE identical to the
  // azimuth-only form it replaced (`RectangularForward(-w)` with `az` subtracted from the
  // longitude). Written out here rather than referenced, so the equality survives the deletion of
  // ProjParams::az0.
  const int w = 1024;
  const int h = 512;
  const float scale = static_cast<float>(std::min(w / 2, h)) / math::kPi;
  const float dirs[][3] = {
    { 0.371391f, -0.557086f, -0.742781f },
    { -0.640184f, 0.256074f, -0.724209f },
    { 0.267261f, 0.534522f, 0.801784f },
    { -0.301511f, -0.904534f, 0.301511f },
    { 0.0f, 0.0f, -1.0f },
    { 0.0f, 0.0f, 1.0f },
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, -1.0f, 0.0f },
  };
  for (float az : { 0.0f, 30.0f, -75.0f, 179.0f, 200.0f }) {
    for (const auto& d : dirs) {
      // The pre-2026-09-02 formula, verbatim.
      const auto proj = lm_proj::RectangularForward(-d[0], -d[1], -d[2]);
      float lon = proj.x - az * math::kDegreeToRad;
      while (lon < -math::kPi) {
        lon += 2.0f * math::kPi;
      }
      while (lon > math::kPi) {
        lon -= 2.0f * math::kPi;
      }
      const int raw_x = static_cast<int>(std::floor(lon * scale + static_cast<float>(w) / 2.0f + 0.5f));
      const std::array<int, 2> legacy{
        ((raw_x % w) + w) % w,
        static_cast<int>(std::floor(-proj.y * scale + static_cast<float>(h) / 2.0f + 0.5f)),
      };

      const auto got = RectangularPixel(az, 0.0f, 0.0f, d);
      char lbl[96];
      std::snprintf(lbl, sizeof(lbl), "az=%g dir=(%g,%g,%g)", az, d[0], d[1], d[2]);
      // One pixel of slack: the two forms reach the same longitude through different float
      // arithmetic (atan2 of rotated components vs a subtraction), so a sample sitting on a bin
      // edge may round either way. Anything larger is a real divergence. The column distance is
      // circular — the map wraps, so column 0 and column w-1 are neighbours.
      EXPECT_LE(std::abs(got[1] - legacy[1]), 1) << lbl;
      if (std::abs(d[2]) > 0.999f) {
        // The two poles are the one place the equality does NOT hold, and it is a property of
        // the equirectangular map rather than of either formula: longitude is atan2 of two
        // components that both vanish there, so the column a pole lands in is arbitrary. The old
        // form put it at `-az`, the new one at the boresight; both are one degenerate ray in one
        // arbitrary column of the polar row. Asserted as a boundary rather than hidden by a
        // widened tolerance — the row, which is the part that carries meaning, still matches
        // above.
        continue;
      }
      int col_dist = std::abs(got[0] - legacy[0]);
      col_dist = std::min(col_dist, w - col_dist);
      EXPECT_LE(col_dist, 1) << lbl;
    }
  }
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
// scale = short_pix/2/tan(fov/2). Pixel = (-c.x, c.y)/denom * scale.
//
// The HORIZONTAL term is NEGATED (-c.x) vs linear/fisheye: globe is an OUTSIDE-IN
// view (camera looks at a sphere from outside), horizontally mirrored relative to
// the INSIDE-OUT single-lens family. The earlier version reused linear's +c.x "by
// transitivity" — that was the bug (scrum gui-lens-math-cli-alignment): it made the
// CLI globe render a left-right mirror of the GUI globe preview (which correctly
// carries the outside-in handedness via ray-sphere globeInverse). Captured-sample
// renders confirm: -c.x makes the CLI sun/halo land on the same side as the GUI.
// Vertical (c.y) is unchanged. Owner: globe follows the GUI (outside-in per tooltip).
// =============== Handedness pin (right = +az; scrum-321.4) ===============
//
// After 321.2 (commit 9a45fc99) render's single-lens family (Linear + 4 single
// fisheye + single orthographic) was pinned to screen handedness right = +az via
// `xy.x = -xy.x;` in projection_shared.h:237. Globe is intentionally OPPOSITE
// (LEFT), because globe is outside-in view (see GlobeRenderPathMatchesDocumented-
// Formula above and the `-cx` in the globe branch). Prior to 321.4 this owner
// decision was not pinned by an ABSOLUTE screen-side assertion in golden — the
// pre-existing forward/inverse round-trip tests are closed by construction for
// any self-consistent convention and cannot detect a sign flip. These two tests
// close that blind spot (issue.md 321.4 / plan.md Step 1 — right = +az on single
// lens family, opposite side on globe).
TEST(LmProj, ProjectExitSingleLensHandednessRightIsPositiveAz) {
  constexpr int kW = 640;
  constexpr int kH = 320;
  constexpr float kElOff = 3.0f * math::kDegreeToRad;
  constexpr float kAzOff = 15.0f * math::kDegreeToRad;
  const float ce = std::cos(kElOff);
  const float se = std::sin(kElOff);
  const float wz = se;
  const struct {
    LensParam::LensType t;
    float fov;
  } kCases[] = {
    { LensParam::kLinear, 90.0f },
    { LensParam::kFisheyeEqualArea, 180.0f },
    { LensParam::kFisheyeEquidistant, 180.0f },
    { LensParam::kFisheyeStereographic, 180.0f },
    { LensParam::kFisheyeOrthographic, 180.0f },
  };
  auto probe = [&](LensParam::LensType t, float fov, float az_rad, float* px_out) -> bool {
    auto cfg = MakeRC(t, fov, kW, kH, /*az=*/180.0f, /*el=*/0.0f, /*ro=*/0.0f);
    Rotation rot = lumice::MakeCameraRotation(cfg);
    auto pp = lumice::BuildProjParams(cfg, rot, static_cast<float>(std::min(kW, kH)));
    float wx = ce * std::cos(az_rad);
    float wy = ce * std::sin(az_rad);
    auto res = lm_proj::ProjectExitToPixel(pp, wx, wy, wz);
    if (res.count < 1) {
      return false;
    }
    *px_out = static_cast<float>(res.hits[0].px);
    return true;
  };
  const float mid = static_cast<float>(kW) / 2.0f;
  for (const auto& c : kCases) {
    float px_pos = 0.0f;
    ASSERT_TRUE(probe(c.t, c.fov, kAzOff, &px_pos)) << "type=" << static_cast<int>(c.t) << " +az must project";
    EXPECT_GT(px_pos, mid) << "type=" << static_cast<int>(c.t) << " +az must land RIGHT (px > W/2)";

    float px_neg = 0.0f;
    ASSERT_TRUE(probe(c.t, c.fov, -kAzOff, &px_neg)) << "type=" << static_cast<int>(c.t) << " -az must project";
    EXPECT_LT(px_neg, mid) << "type=" << static_cast<int>(c.t) << " -az must land LEFT (px < W/2)";
  }
}

// Globe is intentionally horizontally mirrored vs. single-lens (outside-in view).
// After 321.2 (which flipped single-lens to RIGHT=+az) globe remains LEFT=+az.
// This test pins that "opposite side" so a future refactor cannot silently equalize
// globe with the single-lens family (that would reintroduce the scrum-320 bug of
// CLI-vs-GUI mirror).
TEST(LmProj, ProjectExitGlobeHandednessOppositeOfSingleLens) {
  constexpr int kW = 640;
  constexpr int kH = 320;
  constexpr float kElOff = 3.0f * math::kDegreeToRad;
  constexpr float kAzOff = 15.0f * math::kDegreeToRad;
  const float ce = std::cos(kElOff);
  const float wz = std::sin(kElOff);
  auto cfg = MakeRC(LensParam::kGlobe, 60.0f, kW, kH, /*az=*/0.0f, /*el=*/0.0f, /*ro=*/0.0f);
  Rotation rot = lumice::MakeCameraRotation(cfg);
  auto pp = lumice::BuildProjParams(cfg, rot, static_cast<float>(std::min(kW, kH)));
  const float mid = static_cast<float>(kW) / 2.0f;

  float wx_p = ce * std::cos(kAzOff);
  float wy_p = ce * std::sin(kAzOff);
  auto res_p = lm_proj::ProjectExitToPixel(pp, wx_p, wy_p, wz);
  ASSERT_GE(res_p.count, 1) << "globe +az must be visible";
  EXPECT_LT(static_cast<float>(res_p.hits[0].px), mid)
      << "globe: +az must land LEFT (opposite of single-lens RIGHT; see gui-lens-math-cli-alignment)";

  float wx_n = ce * std::cos(-kAzOff);
  float wy_n = ce * std::sin(-kAzOff);
  auto res_n = lm_proj::ProjectExitToPixel(pp, wx_n, wy_n, wz);
  ASSERT_GE(res_n.count, 1) << "globe -az must be visible";
  EXPECT_GT(static_cast<float>(res_n.hits[0].px), mid) << "globe: -az must land RIGHT (opposite of single-lens LEFT)";
}

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
      // -cx: globe is outside-in, horizontally mirrored vs the single-lens family.
      int expect_px = static_cast<int>(std::floor(-cx / denom * scale + w / 2.0f + 0.5f));
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
