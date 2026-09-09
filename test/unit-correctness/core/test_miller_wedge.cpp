#include <gtest/gtest.h>

#include <cmath>

#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/miller_wedge.hpp"

using lumice::ConvertMillerIndexToWedgeAngle;
using lumice::MillerConversionState;

namespace {

// Independent oracle: the textbook formula written out here rather than delegating to
// MillerIndexToWedgeAngleDeg, so that a wrong constant or a dropped factor in the owner is a
// visible disagreement instead of two sides of the same mistake.
float ExpectedAngleDeg(int h, int l) {
  return std::atan(lumice::math::kSqrt3_2 * static_cast<float>(l) / static_cast<float>(h) / lumice::kIceCrystalC) *
         lumice::math::kRadToDegree;
}

}  // namespace

TEST(MillerWedge, ValidIndicesCarryTheAngle) {
  auto r = ConvertMillerIndexToWedgeAngle(1, 0, 1, 3);
  EXPECT_EQ(r.state, MillerConversionState::kValid);
  EXPECT_EQ(r.invalid_index, -1);
  EXPECT_NEAR(r.wedge_angle_deg, 28.00f, 0.01f);
  EXPECT_FLOAT_EQ(r.wedge_angle_deg, ExpectedAngleDeg(1, 1));

  r = ConvertMillerIndexToWedgeAngle(2, 0, 3, 3);
  EXPECT_EQ(r.state, MillerConversionState::kValid);
  EXPECT_NEAR(r.wedge_angle_deg, 38.57f, 0.01f);
  EXPECT_FLOAT_EQ(r.wedge_angle_deg, ExpectedAngleDeg(2, 3));
}

TEST(MillerWedge, ReducibleIndicesAgreeWithTheirReducedForm) {
  // The angle depends only on the ratio h:l, so {2,0,2} must land exactly where {1,0,1} does.
  auto reduced = ConvertMillerIndexToWedgeAngle(1, 0, 1, 3);
  auto scaled = ConvertMillerIndexToWedgeAngle(2, 0, 2, 3);
  EXPECT_EQ(scaled.state, MillerConversionState::kValid);
  EXPECT_FLOAT_EQ(scaled.wedge_angle_deg, reduced.wedge_angle_deg);
}

TEST(MillerWedge, ZeroHIsNoConeNotAnError) {
  auto r = ConvertMillerIndexToWedgeAngle(0, 0, 1, 3);
  EXPECT_EQ(r.state, MillerConversionState::kNoCone);
  EXPECT_EQ(r.invalid_index, -1);
  EXPECT_FLOAT_EQ(r.wedge_angle_deg, 0.0f);
}

TEST(MillerWedge, FewerThanThreeIndicesIsIncomplete) {
  for (int count : { 0, 1, 2 }) {
    auto r = ConvertMillerIndexToWedgeAngle(1, 0, 1, count);
    EXPECT_EQ(r.state, MillerConversionState::kIncomplete) << "provided_count=" << count;
    EXPECT_EQ(r.invalid_index, -1) << "provided_count=" << count;
  }
}

TEST(MillerWedge, IncompleteWinsOverAnythingInTheUnfilledSlots) {
  // A GUI row is typed left to right, so h can be legal while k/l still hold whatever the widget
  // was initialised with. Those slots must not be judged before the user has reached them.
  auto r = ConvertMillerIndexToWedgeAngle(1, 7, -3, 1);
  EXPECT_EQ(r.state, MillerConversionState::kIncomplete);
  EXPECT_EQ(r.invalid_index, -1);
}

TEST(MillerWedge, MoreThanThreeIndicesIsInvalidAndBlamesNoSlot) {
  auto r = ConvertMillerIndexToWedgeAngle(1, 0, 1, 4);
  EXPECT_EQ(r.state, MillerConversionState::kInvalid);
  EXPECT_EQ(r.invalid_index, -1);
}

TEST(MillerWedge, NonZeroKIsUnrepresentable) {
  // {1,1,2} is a second-order pyramidal face rotated 30 degrees off the prism edges. Ignoring k
  // (the old behaviour) silently rendered it as {1,0,2}'s 46.756 degrees; its real angle is
  // 31.545 degrees and the shape cannot be built by this crystal model at all.
  auto r = ConvertMillerIndexToWedgeAngle(1, 1, 2, 3);
  EXPECT_EQ(r.state, MillerConversionState::kInvalid);
  EXPECT_EQ(r.invalid_index, 1);
}

TEST(MillerWedge, NegativeIndexIsBlamedOnTheSlotThatCarriesTheSign) {
  auto r = ConvertMillerIndexToWedgeAngle(1, 0, -1, 3);
  EXPECT_EQ(r.state, MillerConversionState::kInvalid);
  EXPECT_EQ(r.invalid_index, 2);

  r = ConvertMillerIndexToWedgeAngle(-1, 0, 1, 3);
  EXPECT_EQ(r.state, MillerConversionState::kInvalid);
  EXPECT_EQ(r.invalid_index, 0);

  // Both negative: the ratio is positive again, so an angle-only check would wave this through.
  r = ConvertMillerIndexToWedgeAngle(-1, 0, -1, 3);
  EXPECT_EQ(r.state, MillerConversionState::kInvalid);
  EXPECT_EQ(r.invalid_index, 0);
}

TEST(MillerWedge, RatioOutsideTheBuildableRangeIsInvalidAndBlamesNoSlot) {
  // Both indices are individually well-formed; it is the ratio they make that no mesh can carry.
  // l == 0 means a vertical face (a prism face, not a wedge) and a huge l means a face lying flat
  // against the basal plane -- neither survives FillHexCrystalCoef's own alpha range check.
  auto flat = ConvertMillerIndexToWedgeAngle(1, 0, 0, 3);
  EXPECT_EQ(flat.state, MillerConversionState::kInvalid);
  EXPECT_EQ(flat.invalid_index, -1);

  auto steep = ConvertMillerIndexToWedgeAngle(1, 0, 100000, 3);
  EXPECT_GT(ExpectedAngleDeg(1, 100000), 89.9f);  // the input really is past the bound
  EXPECT_EQ(steep.state, MillerConversionState::kInvalid);
  EXPECT_EQ(steep.invalid_index, -1);
}

TEST(MillerWedge, PrimitiveAndAdjudicatorAgreeWhereBothHaveAnOpinion) {
  // The two layers must not drift: the adjudicator is the primitive plus judgement, never a second
  // formula. Sweeping the legal quadrant pins that relationship rather than trusting the call site.
  for (int h = 1; h <= 6; h++) {
    for (int l = 1; l <= 6; l++) {
      auto r = ConvertMillerIndexToWedgeAngle(h, 0, l, 3);
      EXPECT_EQ(r.state, MillerConversionState::kValid) << "h=" << h << " l=" << l;
      EXPECT_FLOAT_EQ(r.wedge_angle_deg, lumice::MillerIndexToWedgeAngleDeg(h, l)) << "h=" << h << " l=" << l;
    }
  }
}
