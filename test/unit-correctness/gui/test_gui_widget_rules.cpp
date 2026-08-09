#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gui/aspect_ratio_rules.hpp"
#include "gui/edit_modal_rules.hpp"
#include "gui/sim_state_rules.hpp"
#include "gui/sun_circle_rules.hpp"

namespace lumice::gui {
namespace {

// The enable/disable rules the GUI's draw functions used to spell inline. Each of these was
// previously reachable only by driving the widget that reads it in a live ImGui frame, which is
// why the units that carried the most of them (app_panels.cpp / edit_modals.cpp) had no
// unit-level coverage at all. Now that they are functions of their arguments, the interesting
// thing to assert is the WHOLE domain — every SimState, every AspectPreset, both sides of every
// cap — rather than the one branch a UI script happened to walk into.

// ---- SimState gates (app_panels.cpp: Run/Stop, ⚠+Revert, New/Open, Save menu, Save-Modified) ----

// Every SimState value, so the tables below are total rather than a sample.
constexpr GuiState::SimState kAllSimStates[] = {
  GuiState::SimState::kIdle, GuiState::SimState::kSimulating, GuiState::SimState::kStopping,
  GuiState::SimState::kDone, GuiState::SimState::kModified,
};

TEST(SimStateRules, BusyIsExactlySimulatingOrStopping) {
  for (GuiState::SimState s : kAllSimStates) {
    const bool expect_busy = (s == GuiState::SimState::kSimulating || s == GuiState::SimState::kStopping);
    EXPECT_EQ(IsBusy(s), expect_busy) << "SimState=" << static_cast<int>(s);
    // The two halves partition busy — no state is both, and nothing else is busy.
    EXPECT_EQ(IsSimulating(s) || IsStopping(s), IsBusy(s));
    EXPECT_FALSE(IsSimulating(s) && IsStopping(s));
  }
}

TEST(SimStateRules, ModifiedIsExactlyOneState) {
  int modified_count = 0;
  for (GuiState::SimState s : kAllSimStates) {
    if (IsModified(s)) {
      ++modified_count;
      EXPECT_EQ(s, GuiState::SimState::kModified);
    }
  }
  EXPECT_EQ(modified_count, 1);
}

TEST(SimStateRules, RunFirstNeedsAServerAndAnIdleBackend) {
  for (GuiState::SimState s : kAllSimStates) {
    // No server ⇒ never runnable, whatever the lifecycle state says.
    EXPECT_FALSE(CanRunFromModal(/*has_server=*/false, s)) << "SimState=" << static_cast<int>(s);
    // With a server, the gate is exactly the top bar's busy notion — this is the shared-owner
    // claim in sim_state_rules.hpp's header, asserted rather than asserted-in-prose.
    EXPECT_EQ(CanRunFromModal(/*has_server=*/true, s), !IsBusy(s)) << "SimState=" << static_cast<int>(s);
  }
}

// ---- Aspect-ratio rules (app_panels.cpp Display group) ----

TEST(AspectRatioRules, OnlyMatchBackgroundNeedsABackground) {
  for (int i = 0; i < kAspectPresetCount; ++i) {
    const auto preset = static_cast<AspectPreset>(i);
    // With a background loaded, every option is selectable.
    EXPECT_FALSE(AspectPresetOptionDisabled(preset, /*has_background=*/true)) << kAspectPresetNames[i];
    // Without one, exactly Match Background is greyed.
    EXPECT_EQ(AspectPresetOptionDisabled(preset, /*has_background=*/false), preset == AspectPreset::kMatchBg)
        << kAspectPresetNames[i];
  }
}

TEST(AspectRatioRules, FlipIsOfferedOnlyByPresetsWithTwoOrientations) {
  // Free has no fixed ratio, 1:1 is its own transpose, Match Background follows the image.
  EXPECT_TRUE(AspectFlipDisabled(AspectPreset::kFree));
  EXPECT_TRUE(AspectFlipDisabled(AspectPreset::k1x1));
  EXPECT_TRUE(AspectFlipDisabled(AspectPreset::kMatchBg));
  EXPECT_FALSE(AspectFlipDisabled(AspectPreset::k16x9));
  EXPECT_FALSE(AspectFlipDisabled(AspectPreset::k3x2));
  EXPECT_FALSE(AspectFlipDisabled(AspectPreset::k4x3));
  EXPECT_FALSE(AspectFlipDisabled(AspectPreset::k2x1));
  // Total over the enum: the three above are the only ones, so a preset added later shows up here
  // as a failure rather than silently inheriting "flippable".
  int disabled_count = 0;
  for (int i = 0; i < kAspectPresetCount; ++i) {
    if (AspectFlipDisabled(static_cast<AspectPreset>(i))) {
      ++disabled_count;
    }
  }
  EXPECT_EQ(disabled_count, 3);
}

// ---- Sun-circle (angular distance) overlay rules ----

TEST(SunCircleRules, DuplicateTestUsesTheHundredthDegreeBand) {
  const std::vector<float> angles = { 22.0f, 46.0f };
  EXPECT_TRUE(SunCircleAlreadyPresent(angles, 22.0f));
  // Just inside the band reads as the same circle; just outside is a new one.
  EXPECT_TRUE(SunCircleAlreadyPresent(angles, 22.0f + kSunCircleDuplicateEpsilonDeg * 0.5f));
  EXPECT_FALSE(SunCircleAlreadyPresent(angles, 22.0f + kSunCircleDuplicateEpsilonDeg * 2.0f));
  EXPECT_FALSE(SunCircleAlreadyPresent(angles, 9.0f));
  EXPECT_FALSE(SunCircleAlreadyPresent({}, 22.0f));
}

TEST(SunCircleRules, LimitBitesExactlyAtTheBufferSize) {
  EXPECT_FALSE(SunCirclesAtLimit(0));
  EXPECT_FALSE(SunCirclesAtLimit(static_cast<std::size_t>(kMaxSunCircles) - 1));
  EXPECT_TRUE(SunCirclesAtLimit(static_cast<std::size_t>(kMaxSunCircles)));
  EXPECT_TRUE(SunCirclesAtLimit(static_cast<std::size_t>(kMaxSunCircles) + 1));
}

TEST(SunCircleRules, CustomAngleIsClampedToADrawableBand) {
  EXPECT_FLOAT_EQ(ClampSunCircleAngle(0.0f), 0.1f);
  EXPECT_FLOAT_EQ(ClampSunCircleAngle(-30.0f), 0.1f);
  EXPECT_FLOAT_EQ(ClampSunCircleAngle(0.1f), 0.1f);
  EXPECT_FLOAT_EQ(ClampSunCircleAngle(22.0f), 22.0f);
  EXPECT_FLOAT_EQ(ClampSunCircleAngle(180.0f), 180.0f);
  EXPECT_FLOAT_EQ(ClampSunCircleAngle(400.0f), 180.0f);
}

// ---- Edit-modal list rules ----

TEST(EditModalRules, SummandRowCapBitesAtTheCapAndNotBefore) {
  EXPECT_FALSE(AtSummandRowCap(0));
  EXPECT_FALSE(AtSummandRowCap(kMaxSummandRows - 1));
  EXPECT_TRUE(AtSummandRowCap(kMaxSummandRows));
  EXPECT_TRUE(AtSummandRowCap(kMaxSummandRows + 1));
}

TEST(EditModalRules, TheLastSummandRowCannotBeDeleted) {
  EXPECT_FALSE(CanDeleteSummandRow(0));
  EXPECT_FALSE(CanDeleteSummandRow(1));
  EXPECT_TRUE(CanDeleteSummandRow(2));
  EXPECT_TRUE(CanDeleteSummandRow(kMaxSummandRows));
}

TEST(EditModalRules, OnlyValidRowsPassTheCommitGate) {
  EXPECT_FALSE(SummandRowBlocksCommit(LUMICE_RAYPATH_VALID));
  EXPECT_TRUE(SummandRowBlocksCommit(LUMICE_RAYPATH_INCOMPLETE));
  EXPECT_TRUE(SummandRowBlocksCommit(LUMICE_RAYPATH_INVALID));
}

TEST(EditModalRules, OkTooltipNamesTheOffendingRowAndItsReason) {
  // A passing row produces no tooltip at all — the caller keeps walking.
  EXPECT_TRUE(SummandRowOkTooltip(0, GuiValidationResult{ LUMICE_RAYPATH_VALID, "" }).empty());
  // Rows are numbered from 1 for the user.
  EXPECT_EQ(SummandRowOkTooltip(0, GuiValidationResult{ LUMICE_RAYPATH_INCOMPLETE, "" }),
            "Row 1: finish typing (incomplete)");
  EXPECT_EQ(SummandRowOkTooltip(2, GuiValidationResult{ LUMICE_RAYPATH_INCOMPLETE, "ignored" }),
            "Row 3: finish typing (incomplete)");
  // Invalid rows surface the validator's own message when it has one, and a generic word when not.
  EXPECT_EQ(SummandRowOkTooltip(1, GuiValidationResult{ LUMICE_RAYPATH_INVALID, "face 9 out of range" }),
            "Row 2: face 9 out of range");
  EXPECT_EQ(SummandRowOkTooltip(1, GuiValidationResult{ LUMICE_RAYPATH_INVALID, "" }), "Row 2: invalid");
}

TEST(EditModalRules, SpectrumCapsBiteAtTheirBoundaries) {
  EXPECT_FALSE(AtSpectrumRowCap(0));
  EXPECT_FALSE(AtSpectrumRowCap(kSpectrumHardMax - 1));
  EXPECT_TRUE(AtSpectrumRowCap(kSpectrumHardMax));
  EXPECT_TRUE(AtSpectrumRowCap(kSpectrumHardMax + 1));

  EXPECT_TRUE(SpectrumCommitBlocked(0));
  EXPECT_FALSE(SpectrumCommitBlocked(1));
}

TEST(EditModalRules, SpectrumValuesAreHeldToTheVisibleBand) {
  // The seed for "Add row" and the sanitize pass on OK read the same band, which is the property
  // that keeps "Add row" from proposing a value the commit then silently moves.
  EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(300.0f), kSpectrumWavelengthMinNm);
  EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(kSpectrumWavelengthMinNm), kSpectrumWavelengthMinNm);
  EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(550.0f), 550.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(kSpectrumWavelengthMaxNm), kSpectrumWavelengthMaxNm);
  EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(900.0f), kSpectrumWavelengthMaxNm);
  // A negative weight would subtract light rather than dim the row.
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(-1.0f), 0.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(2.5f), 2.5f);
}

}  // namespace
}  // namespace lumice::gui
