// The contrast-headroom predicate, asserted on the function itself.
//
// Same division of labour as test_ink_transfer.cpp beside it: the two call sites are expensive to
// stand up (the server's CommitConfig needs a committable document; the GUI's panel needs a window),
// and what is left when both are stripped away is three floats in and one float out. The
// server-side and GUI-side files assert that each path USES this predicate and what it does with
// the answer; this file asserts what the predicate IS.
//
// Numbers here are written as /255 fractions on purpose rather than as decimals. The threshold is
// stated in 8-bit levels (doc/print-mode-subtractive-ink.md §8 + kContrastHeadroomWarnLevels), so a
// case that spells its input the same way can be checked by eye against that argument — a reviewer
// can read "sky at 250/255" and recompute the expected 5/255 headroom without running anything.

#include <gtest/gtest.h>

#include <cmath>

#include "util/contrast_headroom.hpp"

namespace lumice {
namespace {

constexpr float kLevel = 1.0f / 255.0f;

// A grey, so that the per-channel minimum and every individual channel agree: the cases below that
// are about the across-channel rule build their colours by hand instead.
struct Grey {
  float c[3];
};

Grey GreyAtLevel(float levels) {
  const float v = levels * kLevel;
  return Grey{ { v, v, v } };
}

}  // namespace

// The two laws' far ends are exactly representable in this domain, and that exactness is what makes
// the margin an 8-bit statement rather than an approximation of one. Asserted with EXPECT_EQ rather
// than a tolerance for the same reason test_ink_transfer.cpp's ZeroExposureIsTransparent is: a
// function returning 0.999 here would be wrong by an invisible amount and pass any near-check.
TEST(ContrastHeadroom, TheLawsFarEndsAreExactlyZeroHeadroom) {
  const float kWhite[3] = { 1.0f, 1.0f, 1.0f };
  const float kBlack[3] = { 0.0f, 0.0f, 0.0f };
  EXPECT_EQ(ContrastHeadroomMargin(kWhite, ToneLawLimit::kWhite), 0.0f);
  EXPECT_EQ(ContrastHeadroomMargin(kBlack, ToneLawLimit::kBlack), 0.0f);

  // And the opposite ends are full headroom, which is the control: a predicate that returned 0 for
  // everything would pass the two lines above.
  EXPECT_EQ(ContrastHeadroomMargin(kBlack, ToneLawLimit::kWhite), 1.0f);
  EXPECT_EQ(ContrastHeadroomMargin(kWhite, ToneLawLimit::kBlack), 1.0f);
}

// The numeric anchor: a specific input with a headroom that can be recomputed by hand. This is what
// pins the FORMULA rather than just its sign — a predicate measuring the distance to the wrong end,
// or measuring it in the wrong space, would still get the boolean cases below right on extreme
// inputs while being wrong everywhere in between.
TEST(ContrastHeadroom, MarginIsTheDistanceToTheLawsFarEnd) {
  const Grey sky = GreyAtLevel(250.0f);
  EXPECT_NEAR(ContrastHeadroomMargin(sky.c, ToneLawLimit::kWhite), 5.0f * kLevel, 1e-6f);

  const Grey paper = GreyAtLevel(7.0f);
  EXPECT_NEAR(ContrastHeadroomMargin(paper.c, ToneLawLimit::kBlack), 7.0f * kLevel, 1e-6f);

  // The same colour under the other law has the complementary headroom — the two directions are one
  // function with one parameter, not two functions, and this says so.
  EXPECT_NEAR(ContrastHeadroomMargin(sky.c, ToneLawLimit::kBlack), 250.0f * kLevel, 1e-6f);
  EXPECT_NEAR(ContrastHeadroomMargin(paper.c, ToneLawLimit::kWhite), 248.0f * kLevel, 1e-6f);
}

// doc/print-mode-subtractive-ink.md §8: per channel, then the MINIMUM across channels. The case is
// built so that every other plausible reduction gives a different answer — the mean and the CIE-Y
// luminance of this colour are both far above the threshold while the red channel is already on the
// floor, so a luminance-based predicate would stay silent on a paper whose red content is gone.
TEST(ContrastHeadroom, HeadroomIsThePerChannelMinimumNotALuminance) {
  // Deep blue paper: red has 3 levels left, green 40, blue 255.
  const float blue_paper[3] = { 3.0f * kLevel, 40.0f * kLevel, 1.0f };
  EXPECT_NEAR(ContrastHeadroomMargin(blue_paper, ToneLawLimit::kBlack), 3.0f * kLevel, 1e-6f);
  EXPECT_TRUE(ContrastHeadroomIsLow(blue_paper, ToneLawLimit::kBlack));

  // The screen twin: a sky that is saturated in red only.
  const float red_sky[3] = { 1.0f, 100.0f * kLevel, 100.0f * kLevel };
  EXPECT_EQ(ContrastHeadroomMargin(red_sky, ToneLawLimit::kWhite), 0.0f);
  EXPECT_TRUE(ContrastHeadroomIsLow(red_sky, ToneLawLimit::kWhite));
}

// The threshold's own boundary, fixed here rather than left to whichever caller asks first. `<`
// means a configuration sitting exactly on the threshold is not called out.
//
// The exactly-on-the-boundary arm is built out of kContrastHeadroomWarnMargin itself, and the
// just-inside arm out of nextafter of it, because that is the only way to land ON the boundary in
// float: a grey reconstructed as `245 * (1/255)` is a rounding or two away from 1 - 10/255 and would
// make this case a test of float arithmetic rather than of the comparison. The arm is written on the
// kBlack side for the same reason — there the headroom IS the component, with no subtraction to
// round. One arm is enough to pin the operator: ContrastHeadroomIsLow evaluates the margin once and
// compares once, so `limit` reaches only the margin, never the comparison.
TEST(ContrastHeadroom, TheThresholdBoundaryIsExclusive) {
  EXPECT_EQ(kContrastHeadroomWarnLevels, 10) << "the warning level is part of the predicate's contract; see the "
                                                "derivation above its definition before changing it";

  const Grey paper_at_threshold{ { kContrastHeadroomWarnMargin, kContrastHeadroomWarnMargin,
                                   kContrastHeadroomWarnMargin } };
  EXPECT_EQ(ContrastHeadroomMargin(paper_at_threshold.c, ToneLawLimit::kBlack), kContrastHeadroomWarnMargin);
  EXPECT_FALSE(ContrastHeadroomIsLow(paper_at_threshold.c, ToneLawLimit::kBlack));

  const float just_inside = std::nextafter(kContrastHeadroomWarnMargin, 0.0f);
  const Grey paper_inside{ { just_inside, just_inside, just_inside } };
  EXPECT_TRUE(ContrastHeadroomIsLow(paper_inside.c, ToneLawLimit::kBlack));

  // The kWhite side, one whole 8-bit level to either side of the threshold rather than on it — which
  // is what the warning is actually about, and is far enough out that no rounding question arises.
  const Grey sky_below = GreyAtLevel(255.0f - kContrastHeadroomWarnLevels - 1.0f);
  EXPECT_FALSE(ContrastHeadroomIsLow(sky_below.c, ToneLawLimit::kWhite));
  const Grey sky_above = GreyAtLevel(255.0f - kContrastHeadroomWarnLevels + 1.0f);
  EXPECT_TRUE(ContrastHeadroomIsLow(sky_above.c, ToneLawLimit::kWhite));
}

// The four states of the acceptance criteria, read off the predicate that both call sites share.
// The defaults are the interesting half: the shipped default sky (black) and the shipped default
// paper (white) must each be quiet under their OWN law, which is what makes the warning a signal
// rather than a permanent fixture.
TEST(ContrastHeadroom, TheDefaultGroundsOfBothLawsAreQuiet) {
  const float kDefaultSky[3] = { 0.0f, 0.0f, 0.0f };
  const float kDefaultPaper[3] = { 1.0f, 1.0f, 1.0f };
  EXPECT_FALSE(ContrastHeadroomIsLow(kDefaultSky, ToneLawLimit::kWhite));
  EXPECT_FALSE(ContrastHeadroomIsLow(kDefaultPaper, ToneLawLimit::kBlack));

  // And each default is the DEGENERATE ground of the other law, which is why `tone` selects the
  // field to measure as well as the end to measure to. A predicate wired to one field would go off
  // here on a perfectly good configuration.
  EXPECT_TRUE(ContrastHeadroomIsLow(kDefaultPaper, ToneLawLimit::kWhite));
  EXPECT_TRUE(ContrastHeadroomIsLow(kDefaultSky, ToneLawLimit::kBlack));
}

// Out-of-range components are clamped, not passed through. Without the clamp a sky component of 1.2
// would report a NEGATIVE headroom, which compares as "low" and happens to be the right answer —
// while a paper component of -0.2 would report a negative headroom too, also "low", and that one is
// wrong in the quiet direction once it is the minimum of a colour whose other channels are fine.
// Either way the function would be returning a number outside its documented [0, 1] range.
TEST(ContrastHeadroom, OutOfRangeComponentsAreClamped) {
  const float over[3] = { 1.2f, 0.5f, 0.5f };
  EXPECT_EQ(ContrastHeadroomMargin(over, ToneLawLimit::kWhite), 0.0f);
  EXPECT_FALSE(std::isnan(ContrastHeadroomMargin(over, ToneLawLimit::kBlack)));
  EXPECT_NEAR(ContrastHeadroomMargin(over, ToneLawLimit::kBlack), 0.5f, 1e-6f);

  const float under[3] = { -0.2f, 0.5f, 0.5f };
  EXPECT_EQ(ContrastHeadroomMargin(under, ToneLawLimit::kBlack), 0.0f);
  EXPECT_NEAR(ContrastHeadroomMargin(under, ToneLawLimit::kWhite), 0.5f, 1e-6f);
}

}  // namespace lumice
