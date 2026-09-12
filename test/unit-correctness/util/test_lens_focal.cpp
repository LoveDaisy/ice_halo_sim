#include <gtest/gtest.h>

#include <optional>

#include "util/lens_focal.hpp"

// The six formula families, each pinned at an input whose answer is a bare literal on purpose:
// this is the one authority both config readers call, so its test must not be written in terms of
// the function's own constants (d = 12mm, the radian factor) or it would agree with any formula
// that happened to be there. Every expected value below is a closed-form angle a reader can check
// by hand from `d = 12`.

namespace {

constexpr float kTol = 1e-3f;

}  // namespace

// linear: fov = 2 · atan(d / f). f = d = 12 → 2 · 45° = 90°.
TEST(LensFocal, LinearAtTwelveMmIsNinetyDegrees) {
  const std::optional<float> fov = lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kLinear, 12.0f);
  ASSERT_TRUE(fov.has_value());
  EXPECT_NEAR(*fov, 90.0f, kTol);
}

// equal-area: fov = 4 · asin(d / 2f). f = 12 → 4 · asin(0.5) = 4 · 30° = 120°.
TEST(LensFocal, EqualAreaAtTwelveMmIsOneTwentyDegrees) {
  const std::optional<float> fov =
      lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeEqualArea, 12.0f);
  ASSERT_TRUE(fov.has_value());
  EXPECT_NEAR(*fov, 120.0f, kTol);
}

// equal-area's domain edge: d / 2f > 1 ⇔ f < 6. Just below has no solution; exactly at it is 360°.
TEST(LensFocal, EqualAreaBelowSixMmHasNoSolution) {
  EXPECT_FALSE(lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeEqualArea, 5.9f).has_value());
  const std::optional<float> edge =
      lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeEqualArea, 6.0f);
  ASSERT_TRUE(edge.has_value());
  EXPECT_NEAR(*edge, 360.0f, kTol);
}

// equidistant: fov = d / f radians. f = 12 → 1 rad = 57.2958°.
TEST(LensFocal, EquidistantAtTwelveMmIsOneRadian) {
  const std::optional<float> fov =
      lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeEquidistant, 12.0f);
  ASSERT_TRUE(fov.has_value());
  EXPECT_NEAR(*fov, 57.2958f, kTol);
}

// stereographic: fov = 4 · atan(d / 2f). f = 6 → 4 · atan(1) = 4 · 45° = 180°.
TEST(LensFocal, StereographicAtSixMmIsOneEightyDegrees) {
  const std::optional<float> fov =
      lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeStereographic, 6.0f);
  ASSERT_TRUE(fov.has_value());
  EXPECT_NEAR(*fov, 180.0f, kTol);
}

// orthographic: fov = 2 · asin(d / f). f = 24 → 2 · asin(0.5) = 60°.
TEST(LensFocal, OrthographicAtTwentyFourMmIsSixtyDegrees) {
  const std::optional<float> fov =
      lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeOrthographic, 24.0f);
  ASSERT_TRUE(fov.has_value());
  EXPECT_NEAR(*fov, 60.0f, kTol);
}

// orthographic's domain edge: d / f > 1 ⇔ f < 12. Just below has no solution; the exact edge is not
// pinned here (asin(1) can round to 180 + 1 ULP, see LensConfigOrthographic.FCalcFovNearBoundary).
TEST(LensFocal, OrthographicBelowTwelveMmHasNoSolution) {
  EXPECT_FALSE(lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeOrthographic, 11.9f).has_value());
  EXPECT_TRUE(lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kFisheyeOrthographic, 12.5f).has_value());
}

// rectangular: full-sky by convention, `f` is ignored, and there is no domain edge to fall off.
TEST(LensFocal, RectangularIgnoresFAndIsZero) {
  for (float f : { 0.5f, 12.0f, 1000.0f }) {
    const std::optional<float> fov = lumice::LensFocalLengthToFovDegrees(lumice::LensFocalFormula::kRectangular, f);
    EXPECT_TRUE(fov.has_value()) << "f=" << f;
    EXPECT_FLOAT_EQ(fov.value_or(-1.0f), 0.0f) << "f=" << f;
  }
}
