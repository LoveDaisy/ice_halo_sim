// The mono preview's exposure arithmetic (src/gui/mono_exposure_scale.hpp).
//
// Two propositions carry this file. The first is a REGRESSION one: relative mode is the default,
// so every existing document and every committed reference image is exposed by the formula that
// used to sit inlined at three call sites, and extracting it must not have moved a single bit.
// The assertions below therefore recompute that formula independently rather than calling the
// function twice — a test that says "the function equals itself" would pass through any rewrite.
//
// The second is the NEW one: absolute mode divides by emitted energy and drops the auto anchor.
// Dropping the anchor is the whole point (an EV that sits on a per-frame moving baseline cannot
// be compared across documents), so it gets a direct assertion rather than being implied by an
// expected number that happens to match.
//
// Header-only and ImGui-free, hence unit_correctness_test rather than gui_unit_test.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "core/ev_anchor.hpp"
#include "gui/gui_constants.hpp"
#include "gui/mono_exposure_scale.hpp"

namespace {

using lumice::ComputeEvAuto;
using lumice::TargetWhiteToLinear;
using lumice::gui::ComputeMonoExposure;
using lumice::gui::FormatMonoEvReadout;
using lumice::gui::MonoEvMode;
using lumice::gui::MonoExposure;
using lumice::gui::MonoExposureInput;

MonoExposureInput MakeInput() {
  MonoExposureInput in;
  in.exposure_offset = 1.5f;
  in.ev_auto = -2.25f;
  in.snapshot_intensity = 0.35f;
  in.snapshot_emitted_energy = 1000.0f;
  in.total_pixels = 100;
  return in;
}

// --- relative: the pre-extraction formula, bit for bit ------------------------------------------

TEST(MonoExposureScale, RelativeReproducesTheInlinedFormula) {
  const MonoExposureInput in = MakeInput();
  const MonoExposure got = ComputeMonoExposure(MonoEvMode::kRelative, in);

  // Written out the way app.cpp / app_panels.cpp used to carry it, independently of the header.
  const float ev_total = in.exposure_offset + in.ev_auto;
  const float want_factor = std::pow(2.0f, ev_total);
  const float want_scale = want_factor / in.snapshot_intensity;

  EXPECT_FLOAT_EQ(got.intensity_factor, want_factor);
  EXPECT_FLOAT_EQ(got.intensity_scale, want_scale);
}

TEST(MonoExposureScale, RelativeIgnoresEmittedEnergyEntirely) {
  MonoExposureInput a = MakeInput();
  MonoExposureInput b = MakeInput();
  b.snapshot_emitted_energy = 7.0f;  // two orders of magnitude away from a's
  b.total_pixels = 4096 * 4096;

  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kRelative, a).intensity_scale,
                  ComputeMonoExposure(MonoEvMode::kRelative, b).intensity_scale);
}

// ComputeEvAuto's snapshot_intensity argument must NOT be hardcoded (e.g. to 1.0f) regardless of
// what p99_raw_y is anchored to: ComputeMonoExposure's kRelative branch (above) divides by the
// SAME real snapshot_intensity again, and the two only cancel when both hops see the identical
// value. This pins that two-hop cancellation across three distinct snapshot_intensity values, so
// a change that breaks the identity (e.g. hardcoding either hop's argument) fails here instead of
// silently reintroducing a spurious 1/snapshot_intensity factor into displayed brightness. See
// doc/ev-pipeline-architecture.md §2.5.
TEST(MonoExposureScale, ChainedWithComputeEvAutoTheSnapshotIntensityCancels) {
  // Chosen close to TargetWhiteToLinear(135) so ev_auto stays well inside ComputeEvAuto's
  // [-6, 6] clamp across all three snapshot_intensity values below -- a clamped ev_auto would
  // make the cancellation untestable (the clamp, not the algebra, would decide the outcome).
  const float anchor = 0.25f;  // stand-in for axis_solid_angle * anchor_l99_sky
  const float target_white = 135.0f;
  const float exposure_offset = 1.5f;
  const float target_linear = TargetWhiteToLinear(target_white);
  const float want_scale = std::pow(2.0f, exposure_offset) * target_linear / anchor;

  for (float snapshot_intensity : { 0.2f, 1.0f, 3.5f }) {
    MonoExposureInput in;
    in.exposure_offset = exposure_offset;
    in.ev_auto = ComputeEvAuto(anchor, snapshot_intensity, target_white);
    in.snapshot_intensity = snapshot_intensity;

    const MonoExposure got = ComputeMonoExposure(MonoEvMode::kRelative, in);
    EXPECT_NEAR(got.intensity_scale, want_scale, want_scale * 1e-4f) << "snapshot_intensity=" << snapshot_intensity;
  }
}

TEST(MonoExposureScale, RelativeYieldsZeroScaleWhenNothingHasLanded) {
  MonoExposureInput in = MakeInput();
  in.snapshot_intensity = 0.0f;
  // The guard the inline call sites carried: a zero denominator must produce a black frame, not
  // an infinity that paints the preview white.
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kRelative, in).intensity_scale, 0.0f);
  in.snapshot_intensity = -1.0f;
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kRelative, in).intensity_scale, 0.0f);
}

// --- absolute: emitted-energy anchor, no auto term ----------------------------------------------

TEST(MonoExposureScale, AbsoluteUsesEmittedEnergyWithHandComputedValue) {
  MonoExposureInput in = MakeInput();
  in.exposure_offset = 0.0f;  // intensity_factor == 1, so the anchor term stands alone
  in.snapshot_emitted_energy = 1000.0f;
  in.total_pixels = 100;

  const MonoExposure got = ComputeMonoExposure(MonoEvMode::kAbsolute, in);
  EXPECT_FLOAT_EQ(got.intensity_factor, 1.0f);
  // 1 * 0.08 * 100 / 1000
  EXPECT_FLOAT_EQ(got.intensity_scale, 0.008f);
}

TEST(MonoExposureScale, AbsoluteDropsTheAutoAnchor) {
  MonoExposureInput with_auto = MakeInput();
  MonoExposureInput without_auto = MakeInput();
  without_auto.ev_auto = 0.0f;
  ASSERT_NE(with_auto.ev_auto, 0.0f) << "the fixture must actually carry an auto term to drop";

  // The proposition, stated directly: in absolute mode the auto anchor is not part of the answer.
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kAbsolute, with_auto).intensity_scale,
                  ComputeMonoExposure(MonoEvMode::kAbsolute, without_auto).intensity_scale);

  // And the negative control, so the case above cannot pass by the fixture being degenerate:
  // relative mode DOES move when the same term changes.
  EXPECT_NE(ComputeMonoExposure(MonoEvMode::kRelative, with_auto).intensity_scale,
            ComputeMonoExposure(MonoEvMode::kRelative, without_auto).intensity_scale);
}

TEST(MonoExposureScale, AbsoluteScalesWithManualEvOnly) {
  MonoExposureInput base = MakeInput();
  base.exposure_offset = 0.0f;
  MonoExposureInput plus_one = base;
  plus_one.exposure_offset = 1.0f;

  // One stop is one doubling — the property that lets "EV +2 vs physical" mean anything.
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kAbsolute, plus_one).intensity_scale,
                  2.0f * ComputeMonoExposure(MonoEvMode::kAbsolute, base).intensity_scale);
}

TEST(MonoExposureScale, AbsoluteIsComparableAcrossTwoDifferentlyBrightScenes) {
  // What absolute mode is FOR, at the arithmetic level: a scene whose pixels carry half the
  // energy of another's, at the same emitted energy and the same EV, displays at half the
  // brightness. Under relative mode the two would both be re-anchored to their own P99 and the
  // ratio would be destroyed — asserted as the negative control below.
  MonoExposureInput bright = MakeInput();
  bright.exposure_offset = 0.0f;
  bright.snapshot_intensity = 0.40f;

  MonoExposureInput dim = bright;
  dim.snapshot_intensity = 0.20f;  // half as much landed, same emission

  const float s_bright = ComputeMonoExposure(MonoEvMode::kAbsolute, bright).intensity_scale;
  const float s_dim = ComputeMonoExposure(MonoEvMode::kAbsolute, dim).intensity_scale;
  // Same multiplier for both: the displayed ratio is then whatever the pixel values' ratio is,
  // which is the physical one.
  EXPECT_FLOAT_EQ(s_bright, s_dim);

  // Negative control: relative mode divides each by its own landed intensity, so the multipliers
  // differ by exactly the factor that cancels the physical difference out.
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kRelative, dim).intensity_scale /
                      ComputeMonoExposure(MonoEvMode::kRelative, bright).intensity_scale,
                  2.0f);
}

TEST(MonoExposureScale, AbsoluteYieldsZeroScaleWhenNothingHasBeenEmitted) {
  MonoExposureInput in = MakeInput();
  in.snapshot_emitted_energy = 0.0f;
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kAbsolute, in).intensity_scale, 0.0f);
  in.snapshot_emitted_energy = -1.0f;
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kAbsolute, in).intensity_scale, 0.0f);

  MonoExposureInput no_pixels = MakeInput();
  no_pixels.total_pixels = 0;
  EXPECT_FLOAT_EQ(ComputeMonoExposure(MonoEvMode::kAbsolute, no_pixels).intensity_scale, 0.0f);
}

// --- the int overload, which is what GuiState hands in -------------------------------------------

TEST(MonoExposureScale, IntOverloadMapsTheRenderConfigEncodingAndFallsBackToRelative) {
  const MonoExposureInput in = MakeInput();
  EXPECT_FLOAT_EQ(ComputeMonoExposure(0, in).intensity_scale,
                  ComputeMonoExposure(MonoEvMode::kRelative, in).intensity_scale);
  EXPECT_FLOAT_EQ(ComputeMonoExposure(1, in).intensity_scale,
                  ComputeMonoExposure(MonoEvMode::kAbsolute, in).intensity_scale);
  // Anything else reads as relative, matching core's JSON enum fallback (render_config.hpp lists
  // kRelative first for that reason). A document carrying a value from a future build must render
  // the way it always has, not go dark.
  EXPECT_FLOAT_EQ(ComputeMonoExposure(2, in).intensity_scale,
                  ComputeMonoExposure(MonoEvMode::kRelative, in).intensity_scale);
  EXPECT_FLOAT_EQ(ComputeMonoExposure(-1, in).intensity_scale,
                  ComputeMonoExposure(MonoEvMode::kRelative, in).intensity_scale);
}

// --- the on-screen readout ------------------------------------------------------------------------

TEST(MonoExposureScale, AbsoluteReadoutNamesTheModeAndTheHeightWithoutClaimingAutoApplies) {
  const std::string s = FormatMonoEvReadout(MonoEvMode::kAbsolute, 2.0f, -5.66f);
  EXPECT_NE(s.find("Absolute"), std::string::npos);
  EXPECT_NE(s.find("+2.00"), std::string::npos);
  EXPECT_NE(s.find("physical"), std::string::npos);
  // The auto value must not appear as if it were in effect. In a sparse scene it drifts by many
  // stops while the picture does not move, which is exactly the misreading this readout replaces.
  EXPECT_EQ(s.find("-5.66"), std::string::npos);
  EXPECT_EQ(s.find("auto"), std::string::npos);
}

TEST(MonoExposureScale, RelativeReadoutShowsAllThreeNumbers) {
  const std::string s = FormatMonoEvReadout(MonoEvMode::kRelative, 2.0f, -5.66f);
  EXPECT_NE(s.find("Relative"), std::string::npos);
  EXPECT_NE(s.find("+2.00"), std::string::npos);  // what the user set
  EXPECT_NE(s.find("-5.66"), std::string::npos);  // what the anchor contributed
  EXPECT_NE(s.find("-3.66"), std::string::npos);  // what the picture is actually exposed at
}

TEST(MonoExposureScale, ReadoutIntOverloadMatchesTheEnumOne) {
  EXPECT_EQ(FormatMonoEvReadout(1, 0.5f, 1.0f), FormatMonoEvReadout(MonoEvMode::kAbsolute, 0.5f, 1.0f));
  EXPECT_EQ(FormatMonoEvReadout(0, 0.5f, 1.0f), FormatMonoEvReadout(MonoEvMode::kRelative, 0.5f, 1.0f));
  EXPECT_EQ(FormatMonoEvReadout(9, 0.5f, 1.0f), FormatMonoEvReadout(MonoEvMode::kRelative, 0.5f, 1.0f));
}

}  // namespace
