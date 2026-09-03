// The rules a GUI control follows before anyone renders it: what it shows, what it lets you do,
// what range it maps, and how big the window it sits in is allowed to be.
//
// All of these were once spelled inline inside a draw function, reachable only by driving the
// widget in a live frame — which is why the units carrying the most of them (app_panels.cpp,
// edit_modals.cpp) had no unit-level coverage at all. Now that each is a function of its arguments,
// the interesting assertion is over its WHOLE domain: every SimState, every AspectPreset, every
// shape-scalar slot, both sides of every cap — rather than the one branch a UI script walked into.
//
// One file rather than six because they are one subject and each of the six carried more preamble
// than propositions. Nothing here links against file_io.cpp, so this is the lumice_obj target
// rather than gui_unit_test.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "gui/aspect_ratio_rules.hpp"
#include "gui/composite_exposure_push.hpp"
#include "gui/edit_modal_rules.hpp"
#include "gui/gui_constants.hpp"
#include "gui/shape_scalar_domain.hpp"
#include "gui/sim_state_rules.hpp"
#include "gui/slider_mapping.hpp"
#include "gui/sun_circle_rules.hpp"
#include "gui/window_sizing.hpp"

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
  // Idempotence is what makes "seed then sanitize" a no-op for an untouched row. Without it a row
  // the user never edited would move on every OK.
  for (float nm : { -1e6f, 0.0f, 379.0f, kSpectrumWavelengthMinNm, 550.0f, kSpectrumWavelengthMaxNm, 781.0f, 1e6f }) {
    const float once = ClampSpectrumWavelengthNm(nm);
    EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(once), once) << "input " << nm;
  }
  // A negative weight would subtract light rather than dim the row.
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(-1.0f), 0.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(2.5f), 2.5f);
}

// The shape-scalar domain table. Before it was data, these numbers were written out at each of the
// ten RenderShapeDistTableRow call sites and described in a comment nothing could check, so the
// only way to ask "what does Prism H allow" was to open the crystal modal and drag its slider.
//
// The two assertions worth making about a table are total ones: every slot has a row, and the row
// each slot has is the one it had before the move.

// The values as they stood at the call sites in edit_modals.cpp before extraction. This is the
// "truth table is identical either side of the refactor" check, spelled out independently rather
// than by re-reading the table under test.
struct ExpectedRow {
  int slot;
  const char* name;
  float min_value;
  float max_value;
  const char* fmt;
  SliderScale scale;
};

constexpr ExpectedRow kExpected[] = {
  { LUMICE_SHAPE_SCALAR_HEIGHT, "Height", 1e-4f, 100.0f, "%.4g", SliderScale::kLogLinear },
  { LUMICE_SHAPE_SCALAR_UPPER_H, "Upper H", 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_PRISM_H, "Prism H", 0.0f, 100.0f, "%.4g", SliderScale::kLogLinear },
  { LUMICE_SHAPE_SCALAR_LOWER_H, "Lower H", 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 0, "Face 0", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 1, "Face 1", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 2, "Face 2", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 3, "Face 3", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 4, "Face 4", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 5, "Face 5", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
};
static_assert(sizeof(kExpected) / sizeof(kExpected[0]) == LUMICE_SHAPE_SCALAR_COUNT,
              "the expectation table must cover every shape scalar slot");

TEST(ShapeScalarDomain, EverySlotKeepsTheDomainItsCallSiteUsedToSpell) {
  for (const ExpectedRow& row : kExpected) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(row.slot);
    EXPECT_FLOAT_EQ(d.min_value, row.min_value) << row.name;
    EXPECT_FLOAT_EQ(d.max_value, row.max_value) << row.name;
    EXPECT_EQ(std::string(d.fmt), std::string(row.fmt)) << row.name;
    EXPECT_EQ(d.scale, row.scale) << row.name;
  }
}

TEST(ShapeScalarDomain, EverySlotHasANonEmptyDomain) {
  // Total over the enum rather than over the rows the crystal modal happens to draw: a slot added
  // to lumice.h without a domain here would otherwise stay invisible until someone opened the tab
  // that renders it.
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(slot);
    EXPECT_LT(d.min_value, d.max_value) << "slot " << slot;
    EXPECT_GE(d.min_value, 0.0f) << "slot " << slot;
    if (d.fmt == nullptr) {
      ADD_FAILURE() << "slot " << slot << ": fmt is null";
      continue;  // no format string to inspect for this slot; the rest still get checked
    }
    EXPECT_EQ(d.fmt[0], '%') << "slot " << slot;
  }
}

TEST(ShapeScalarDomain, TheSixFaceSlotsShareOneDomain) {
  // Shared on purpose today (one kFaceSpreadMax for all six). Stored per-slot rather than as a
  // range test, so giving one face its own band later is a data edit — this assertion is what
  // would then have to be updated deliberately instead of the behavior changing silently.
  const ShapeScalarDomain& first = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_FACE_0);
  for (int i = 1; i < 6; ++i) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_FACE_0 + i);
    EXPECT_FLOAT_EQ(d.min_value, first.min_value) << "face " << i;
    EXPECT_FLOAT_EQ(d.max_value, first.max_value) << "face " << i;
    EXPECT_EQ(std::string(d.fmt), std::string(first.fmt)) << "face " << i;
    EXPECT_EQ(d.scale, first.scale) << "face " << i;
  }
}

TEST(ShapeScalarDomain, LogScaledSlotsHaveAStrictlyPositiveLowerBound) {
  // A log mapping divides by min_value (slider_mapping.hpp::LogValueToNorm), so a kLog slot with
  // min 0 is not a tuning choice — it is a division by zero waiting for a user to reach it.
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(slot);
    if (d.scale == SliderScale::kLog) {
      EXPECT_GT(d.min_value, 0.0f) << "slot " << slot;
    }
  }
}

constexpr float kEpsilon = 1e-4f;
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

// The display-time composite-exposure push guard, extracted out of app_panels.cpp so all four of
// its branches are reachable without a full ImGui frame. Pushing when nothing changed re-bakes the
// composite every frame; not pushing when it did leaves the picture at the previous exposure.

TEST(CompositeExposurePush, TheGuardPushesOnTheEdgeOnAChangeAndOnTheFirstBakeAndNeverOtherwise) {
  struct Case {
    const char* name;
    bool active;
    bool was_active;
    float ev_total;
    float last_pushed_ev;
    bool expect_push;
  };
  const Case kCases[] = {
    { "composite off, nothing to bake", false, false, 1.0f, 1.0f, false },
    { "composite off and never pushed", false, false, 2.0f, kNan, false },
    // plan-review Minor #2: composite just went live and last_pushed_ev happens to equal ev_total
    // (a stale value from a prior composite-off period) — the edge alone must still force a push.
    { "the edge into composite, same ev", true, false, 1.0f, 1.0f, true },
    { "already on, ev moved", true, true, 1.5f, 1.0f, true },
    { "already on, ev unchanged", true, true, 1.0f, 1.0f, false },
    { "already on, ev within epsilon", true, true, 1.0f + kEpsilon * 0.5f, 1.0f, false },
    // NaN models "no push has happened yet" — the very first bake must not be skipped just because
    // the edge was already consumed on a prior frame.
    { "already on, first ever push", true, true, 0.0f, kNan, true },
  };
  for (const Case& c : kCases) {
    EXPECT_EQ(ShouldPushCompositeExposure(c.active, c.was_active, c.ev_total, c.last_pushed_ev, kEpsilon),
              c.expect_push)
        << c.name;
  }
}

using lumice::gui::slider_mapping::kLogLinearTSwitch;
using lumice::gui::slider_mapping::kLogLinearX0;
using lumice::gui::slider_mapping::LogLinearNormToValue;
using lumice::gui::slider_mapping::LogLinearValueToNorm;
using lumice::gui::slider_mapping::LogNormToValue;
using lumice::gui::slider_mapping::LogValueToNorm;

// ============================================================
// Convention 1 — the two nonlinear laws: kLog and the kLogLinear hybrid
// ============================================================

// ---- Slider value <-> normalized position, one convention per shape scalar ----
//
// A slider hands ImGui a position in [0, 1] and reads one back; these are the laws that turn that
// into the number the user typed. Getting one wrong does not crash anything -- it makes a slider
// whose left half covers 99% of the range, or whose readout does not come back to what was dragged
// to. Three properties matter and are asserted for each law: the ends land exactly on the ends, the
// map is invertible over its whole range, and it never goes backwards.

TEST(SliderMapping, EachLawPinsItsEndsAndItsCharacteristicPoint) {
  struct AnchorCase {
    const char* name;
    float value;
    float norm;
    float value_tol;
  };
  // The pure log law, over the [0.01, 100] band the background-scale field uses: the geometric
  // midpoint -- not the arithmetic one -- sits at the middle of the slider. That is the whole point
  // of the law: 0.1 and 10 are equally far from 1. (No shape scalar rides this law today; the two
  // that span decades take the hybrid below, which can also reach its own floor exactly.)
  const AnchorCase kLogCases[] = {
    { "log, low end", 0.01f, 0.0f, 1e-6f },
    { "log, geometric midpoint", 1.0f, 0.5f, 1e-4f },
    { "log, high end", 100.0f, 1.0f, 1e-4f },
  };
  for (const AnchorCase& c : kLogCases) {
    EXPECT_NEAR(LogValueToNorm(c.value, 0.01f, 100.0f), c.norm, 1e-6f) << c.name;
    EXPECT_NEAR(LogNormToValue(c.norm, 0.01f, 100.0f), c.value, c.value_tol) << c.name;
  }

  // The pyramid prism height must reach exactly zero, which a pure log law cannot do, so it is
  // linear below a switch point and logarithmic above it. The switch is the characteristic point
  // here, and it must be continuous: the two halves have to agree on the value there, or the slider
  // jumps under the cursor as it crosses. These cases pin the min_val = 0 instance; the
  // positive-floor instance is pinned by TheHybridLawHonoursWhateverFloorItIsGiven below.
  const AnchorCase kLogLinearCases[] = {
    { "log-linear, exact zero", 0.0f, 0.0f, 0.0f },
    { "log-linear, the switch point", kLogLinearX0, kLogLinearTSwitch, 1e-6f },
    { "log-linear, high end", 100.0f, 1.0f, 1e-3f },
  };
  for (const AnchorCase& c : kLogLinearCases) {
    EXPECT_NEAR(LogLinearValueToNorm(c.value, 0.0f, 100.0f), c.norm, 1e-6f) << c.name;
    EXPECT_NEAR(LogLinearNormToValue(c.norm, 0.0f, 100.0f), c.value, c.value_tol) << c.name;
  }
}

TEST(SliderMapping, EachLawIsInvertibleAndNeverGoesBackwards) {
  // Log: geometrically spaced samples, since that is where its resolution is uniform.
  constexpr int kLogSamples = 17;
  float prev_log = LogNormToValue(0.0f, 0.01f, 100.0f);
  for (int i = 0; i <= kLogSamples; ++i) {
    const float t = static_cast<float>(i) / kLogSamples;
    const float value = 0.01f * std::exp(t * std::log(100.0f / 0.01f));
    const float round_trip = LogNormToValue(LogValueToNorm(value, 0.01f, 100.0f), 0.01f, 100.0f);
    EXPECT_NEAR(round_trip, value, std::max(value, 1e-4f) * 1e-4f) << "log value=" << value;
    const float stepped = LogNormToValue(t, 0.01f, 100.0f);
    if (i > 0) {
      EXPECT_GT(stepped, prev_log) << "log step " << i;
    }
    prev_log = stepped;
  }

  // Log-linear: samples chosen to straddle the switch -- below it, on it, and above it -- because a
  // law stitched from two pieces can be invertible on each piece and still lose the seam.
  constexpr float kLogLinearSamples[] = { 0.0f, 0.005f, 0.01f, 0.05f, 0.2f, 1.0f, 10.0f, 50.0f, 100.0f };
  for (float value : kLogLinearSamples) {
    const float round_trip = LogLinearNormToValue(LogLinearValueToNorm(value, 0.0f, 100.0f), 0.0f, 100.0f);
    EXPECT_NEAR(round_trip, value, std::max(value, 1e-3f) * 1e-3f) << "log-linear value=" << value;
  }
  float prev_log_linear = LogLinearNormToValue(0.0f, 0.0f, 100.0f);
  for (int i = 1; i <= 40; ++i) {
    const float value = LogLinearNormToValue(static_cast<float>(i) / 40.0f, 0.0f, 100.0f);
    // Non-decreasing rather than increasing: the two pieces may tie exactly at the seam.
    EXPECT_GE(value, prev_log_linear) << "log-linear step " << i;
    prev_log_linear = value;
  }
}

// The third convention, [0, 1] linear, has no helper at all: SliderWithInput hands the raw value to
// ImGui and the range is enforced by std::clamp at the call site. There is nothing of ours to
// assert, and five cases asserting std::clamp were exactly that -- so the coverage that matters
// lives where the call site is driven, in gui_test's pyramid_h_* modal cases.

using lumice::gui::slider_mapping::LogLinearNormToValueSnapped;
using lumice::gui::slider_mapping::LogNormToValueSnapped;
using lumice::gui::slider_mapping::SqrtNormToValueSnapped;

// At a slider stop the value written back must be the bound itself, bit for bit -- not the bound
// recovered through sqrt or exp/log. EXPECT_EQ, not EXPECT_NEAR, is the point of this case: the
// residue these snaps remove is 3.05e-5 on a 360 deg range, which every tolerance above would
// happily accept, and which core's FloatEqual (threshold 1e-5) reads as a different value. That is
// how an azimuth range dragged to its stop stopped counting as a full turn, and with it a crystal
// axis stopped counting as full-sphere uniform. The interior of each law is unchanged and is pinned
// by the EXPECT_NEAR cases above; these three only bind the ends.
TEST(SliderMapping, EndpointSnappingWritesBackTheBoundExactly) {
  // The uniform-distribution Range slider: sqrt scale over [0, 360].
  const float sqrt_max_360 = std::sqrt(360.0f);
  EXPECT_NE(sqrt_max_360 * sqrt_max_360, 360.0f) << "premise of this case: the plain round trip is not exact here";
  EXPECT_EQ(SqrtNormToValueSnapped(sqrt_max_360, sqrt_max_360, 360.0f), 360.0f);
  EXPECT_EQ(SqrtNormToValueSnapped(0.0f, sqrt_max_360, 360.0f), 0.0f);
  // Past the stop (ImGui clamps, but the guard must not depend on that) and inside it.
  EXPECT_EQ(SqrtNormToValueSnapped(sqrt_max_360 * 1.5f, sqrt_max_360, 360.0f), 360.0f);
  EXPECT_EQ(SqrtNormToValueSnapped(-1.0f, sqrt_max_360, 360.0f), 0.0f);
  EXPECT_FLOAT_EQ(SqrtNormToValueSnapped(3.0f, sqrt_max_360, 360.0f), 9.0f);

  // The pure log law over [0.01, 100]. No shape scalar rides it today — the prism height moved to
  // the hybrid below when it gained a floor of its own — but the law is still reachable from the
  // domain table, so its snapping stays pinned here.
  EXPECT_EQ(LogNormToValueSnapped(1.0f, 0.01f, 100.0f), 100.0f);
  EXPECT_EQ(LogNormToValueSnapped(0.0f, 0.01f, 100.0f), 0.01f);
  EXPECT_EQ(LogNormToValueSnapped(2.0f, 0.01f, 100.0f), 100.0f);
  EXPECT_EQ(LogNormToValueSnapped(-0.5f, 0.01f, 100.0f), 0.01f);
  EXPECT_FLOAT_EQ(LogNormToValueSnapped(0.5f, 0.01f, 100.0f), LogNormToValue(0.5f, 0.01f, 100.0f));

  // Pyramid prism height: log-linear hybrid over [0, 100].
  EXPECT_EQ(LogLinearNormToValueSnapped(1.0f, 0.0f, 100.0f), 100.0f);
  EXPECT_EQ(LogLinearNormToValueSnapped(0.0f, 0.0f, 100.0f), 0.0f);
  EXPECT_EQ(LogLinearNormToValueSnapped(1.25f, 0.0f, 100.0f), 100.0f);
  EXPECT_EQ(LogLinearNormToValueSnapped(-0.25f, 0.0f, 100.0f), 0.0f);
  EXPECT_FLOAT_EQ(LogLinearNormToValueSnapped(0.5f, 0.0f, 100.0f), LogLinearNormToValue(0.5f, 0.0f, 100.0f));
}

// ---- The hybrid law under a floor other than zero ----
//
// The linear segment of the log-linear law used to start at a hardcoded 0. It now starts at the
// floor the domain declares, which is what lets one law serve both a scalar that must reach exactly
// zero and one whose control refuses to go below a small positive number. Two propositions have to
// hold for that generalization to be safe, and they pull in opposite directions: the zero-floor
// instance must be untouched, and the positive-floor instance must actually honour its floor.

TEST(SliderMapping, TheHybridLawIsUnchangedWhereItsFloorIsZero) {
  // The expectations here are the PRE-generalization formulas, written out independently rather
  // than obtained by calling the functions under test with a second set of arguments -- calling the
  // new code twice would agree with itself no matter what the edit did. EXPECT_EQ, not
  // EXPECT_FLOAT_EQ: with min_val = 0 the new expressions reduce to the old ones term for term
  // (x + 0, y - 0), so the result is bit-identical, and anything less than bit-identical here would
  // mean the reduction is not what it is claimed to be.
  constexpr float kSamples[] = { 0.0f, 1e-5f, 0.001f, 0.005f, kLogLinearX0, 0.05f, 1.0f, 25.0f, 100.0f };
  for (float value : kSamples) {
    const float clamped = std::clamp(value, 0.0f, 100.0f);
    float expect_norm = 0.0f;
    if (clamped <= kLogLinearX0) {
      expect_norm = kLogLinearTSwitch * clamped / kLogLinearX0;  // the old linear segment
    } else {
      expect_norm = kLogLinearTSwitch +
                    (1.0f - kLogLinearTSwitch) * std::log(clamped / kLogLinearX0) / std::log(100.0f / kLogLinearX0);
    }
    EXPECT_EQ(LogLinearValueToNorm(value, 0.0f, 100.0f), std::clamp(expect_norm, 0.0f, 1.0f)) << "value=" << value;
  }
  for (int i = 0; i <= 40; ++i) {
    const float norm = static_cast<float>(i) / 40.0f;
    float expect_value = 0.0f;
    if (norm <= kLogLinearTSwitch) {
      expect_value = kLogLinearX0 * norm / kLogLinearTSwitch;  // the old linear segment's inverse
    } else {
      const float t_log = (norm - kLogLinearTSwitch) / (1.0f - kLogLinearTSwitch);
      expect_value = kLogLinearX0 * std::exp(t_log * std::log(100.0f / kLogLinearX0));
    }
    EXPECT_EQ(LogLinearNormToValue(norm, 0.0f, 100.0f), expect_value) << "norm=" << norm;
  }
  // And the snapped inverse still writes back a literal zero at the left stop, which is the whole
  // reason the pyramid's prism height can be switched off from the slider at all.
  EXPECT_EQ(LogLinearNormToValueSnapped(0.0f, 0.0f, 100.0f), 0.0f);
  EXPECT_EQ(LogLinearNormToValueSnapped(-1.0f, 0.0f, 100.0f), 0.0f);
}

TEST(SliderMapping, TheHybridLawHonoursWhateverFloorItIsGiven) {
  // The prism height's band. Its floor exists to stay clear of the gate core applies to the same
  // quantity (a height must be strictly greater than 1e-5), so "the left stop is the floor" is not
  // cosmetic: a stop that landed a few ULP below would hand core a value it rejects.
  const ShapeScalarDomain& height = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_HEIGHT);
  const float floor_v = height.min_value;
  const float max_v = height.max_value;
  EXPECT_GT(floor_v, 1e-5f) << "the floor must clear core's strictly-greater-than gate";
  EXPECT_LT(floor_v, kLogLinearX0) << "the linear segment [floor, x0] must be non-degenerate";

  // Ends. EXPECT_EQ rather than EXPECT_NEAR: the endpoint snap exists precisely so the value at a
  // stop is the bound itself and not the bound recovered through a round trip.
  EXPECT_EQ(LogLinearValueToNorm(floor_v, floor_v, max_v), 0.0f);
  EXPECT_EQ(LogLinearNormToValue(0.0f, floor_v, max_v), floor_v);
  EXPECT_EQ(LogLinearNormToValueSnapped(0.0f, floor_v, max_v), floor_v);
  EXPECT_EQ(LogLinearNormToValueSnapped(-0.25f, floor_v, max_v), floor_v);
  EXPECT_EQ(LogLinearNormToValueSnapped(1.0f, floor_v, max_v), max_v);

  // Below the floor is clamped up to it, not down to zero -- the failure this replaces would have
  // let the left third of the travel report a value the control does not offer.
  EXPECT_EQ(LogLinearValueToNorm(0.0f, floor_v, max_v), 0.0f);
  EXPECT_GE(LogLinearNormToValue(0.0f, floor_v, max_v), floor_v);

  // The seam. The log segment is anchored at kLogLinearX0 whatever the floor is, so the two halves
  // must still meet there; a discontinuity is a slider that jumps under the cursor.
  EXPECT_NEAR(LogLinearValueToNorm(kLogLinearX0, floor_v, max_v), kLogLinearTSwitch, 1e-6f);
  EXPECT_NEAR(LogLinearNormToValue(kLogLinearTSwitch, floor_v, max_v), kLogLinearX0, 1e-6f);

  // Invertible and never backwards over the whole travel, seam included.
  float prev = LogLinearNormToValue(0.0f, floor_v, max_v);
  for (int i = 1; i <= 200; ++i) {
    const float norm = static_cast<float>(i) / 200.0f;
    const float value = LogLinearNormToValue(norm, floor_v, max_v);
    EXPECT_GE(value, prev) << "step " << i;
    EXPECT_NEAR(LogLinearValueToNorm(value, floor_v, max_v), norm, 1e-4f) << "step " << i;
    prev = value;
  }
}

// ---- Display quantization must match the slider's own quantization ----
//
// A slider and the number beside it quantize independently, and the two kinds have to agree.
// "%.Nf" quantizes ABSOLUTELY: it can tell values apart when they differ by more than a fixed
// step. "%.Ng" quantizes RELATIVELY: by more than a fixed fraction of the value. A kLinear slider
// moves the value by a constant amount per pixel of travel; kLog and kLogLinear (above its switch
// point) move it by a constant fraction per pixel. Pair a relative slider with an absolute format
// and the low decades render as a run of identical strings: the handle moves and the number does
// not, which is indistinguishable from a frozen control.
//
// The assertion below is deliberately about MARGIN rather than about today's state. A row can be
// one pixel of extra slider width away from breaking and still pass a "no collisions right now"
// check, looking exactly like a row with twenty times the headroom.

// The widest a Value-column slider gets: measured with the crystal edit modal's own layout, from
// the item rectangle ImGui gave the slider -- 179 px in the horizontal (820 px) modal and 127 px in
// the vertical (420 px) one. The slider does NOT span the modal: it lives in the single stretch
// column of a five-column property table, alongside four fixed-width cells.
constexpr int kMeasuredMaxSliderWidthPx = 179;

// The travel model is one sample per pixel. ImGui's actual position resolution is 1/(w - grab_w),
// i.e. COARSER than this, so a row that resolves every pixel here resolves every position ImGui can
// actually produce. Keyboard nudges are outside the model.
struct NonlinearRow {
  const char* name;
  float min_value;
  float max_value;
  const char* fmt;
  SliderScale scale;
  bool excluded;
};

float NormToValueForRow(const NonlinearRow& row, float norm) {
  switch (row.scale) {
    case SliderScale::kLog:
      return LogNormToValue(norm, row.min_value, row.max_value);
    case SliderScale::kLogLinear:
      return LogLinearNormToValue(norm, row.min_value, row.max_value);
    case SliderScale::kSqrt: {
      const float sqrt_val = norm * std::sqrt(row.max_value);
      return sqrt_val * sqrt_val;
    }
    case SliderScale::kLinear:
      break;
  }
  return row.min_value + (row.max_value - row.min_value) * norm;
}

// The number of adjacent pixel pairs that format to the same string over the whole travel.
int CountAdjacentPixelCollisions(const NonlinearRow& row, int width_px) {
  char prev[64] = { 0 };
  char cur[64] = { 0 };
  int collisions = 0;
  for (int i = 0; i <= width_px; ++i) {
    const float norm = static_cast<float>(i) / static_cast<float>(width_px);
    std::snprintf(cur, sizeof(cur), row.fmt, NormToValueForRow(row, norm));
    if (i > 0 && std::strcmp(cur, prev) == 0) {
      ++collisions;
    }
    std::memcpy(prev, cur, sizeof(prev));
  }
  return collisions;
}

TEST(ShapeScalarDomain, EveryNonlinearRowResolvesEveryPixelOfItsSliderWithMargin) {
  const ShapeScalarDomain& height = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_HEIGHT);
  const ShapeScalarDomain& prism_h = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_PRISM_H);
  const NonlinearRow kRows[] = {
    { "Height", height.min_value, height.max_value, height.fmt, height.scale, false },
    { "Prism H", prism_h.min_value, prism_h.max_value, prism_h.fmt, prism_h.scale, false },
    // The crystal axis orientation sliders (app_panels.cpp), listed rather than omitted so the
    // boundary of this assertion is visible where the assertion is. They are excluded on a
    // mechanism, not on a name: a kSqrt slider's step per pixel is proportional to the square root
    // of the value, which is neither a constant amount nor a constant fraction, so neither family
    // of format matches it and "pick the format that matches the law" has no answer to give here
    // yet. Their domains are spelled at their call sites.
    { "Std", 0.0f, 180.0f, "%.1f", SliderScale::kSqrt, true },
    { "Range", 0.0f, 360.0f, "%.1f", SliderScale::kSqrt, true },
    { "Amplitude", 0.0f, 90.0f, "%.1f", SliderScale::kSqrt, true },
    { "Scale", 0.0f, 90.0f, "%.1f", SliderScale::kSqrt, true },
  };

  for (const NonlinearRow& row : kRows) {
    // The exclusion is derived from the row's law, not from its name: this is what stops a row
    // from being quietly parked outside the assertion by adding a flag to it.
    EXPECT_EQ(row.excluded, row.scale == SliderScale::kSqrt) << row.name;
    if (row.excluded) {
      continue;
    }
    // Today's width, then two and three times it. The multiples are the point: collisions appear
    // as the slider gets WIDER (more pixels over the same range means a smaller step per pixel),
    // never disappear, so zero collisions at 3x the measured width means the width at which this
    // row would first break is beyond 3x -- a margin, not a coin flip. A row that only clears the
    // 1x check is one layout change away from a frozen readout and would be indistinguishable, in
    // a green run, from a row with room to spare.
    for (int multiple : { 1, 2, 3 }) {
      const int width = kMeasuredMaxSliderWidthPx * multiple;
      EXPECT_EQ(CountAdjacentPixelCollisions(row, width), 0)
          << row.name << " at " << multiple << "x the measured slider width (" << width << " px)";
    }
  }
}

using lumice::gui::AspectFitResult;
using lumice::gui::ClampWindowSizeToWorkarea;
using lumice::gui::kAspectClampTolerance;
using lumice::gui::kLeftPanelWidth;
using lumice::gui::kMinWindowHeight;
using lumice::gui::kMinWindowWidth;
using lumice::gui::kRightPanelWidth;
using lumice::gui::kStatusBarHeight;
using lumice::gui::kTopBarHeight;
using lumice::gui::MonitorRect;
using lumice::gui::ResolveAspectFit;
using lumice::gui::SelectMonitorIndexByCenter;

// The startup window size, over the workareas that decide it. A 50px margin comes off each
// dimension before the desired size is honoured, and kMinWindow{Width,Height} is a floor beneath
// that — a workarea small enough to push through it must not shrink the window below the documented
// minimum.
TEST(WindowSizingTest, TheDesiredSizeIsClampedToTheWorkareaAndFlooredAtTheMinimum) {
  struct Case {
    const char* name;
    int work_w;
    int work_h;
    int expect_w;
    int expect_h;
  };
  const Case kCases[] = {
    // 1080p laptop workarea ≈ 1920×900 after menubar/Dock: only the height clamps (900 - 50).
    { "constrained 1080p", 1920, 900, 1600, 850 },
    { "high-DPI dev monitor", 2880, 1800, 1600, 980 },
    { "1366x768 laptop, both dimensions", 1366, 768, 1316, 718 },
    { "pathologically small workarea", 800, 600, kMinWindowWidth, kMinWindowHeight },
    // workarea - margin == desired, so nothing moves.
    { "exact boundary", 1650, 1030, 1600, 980 },
  };
  for (const Case& c : kCases) {
    auto [w, h] = ClampWindowSizeToWorkarea(1600, 980, c.work_w, c.work_h);
    EXPECT_EQ(w, c.expect_w) << c.name;
    EXPECT_EQ(h, c.expect_h) << c.name;
  }
}

// ========== Monitor selection (multi-monitor aspect ratio fix) ==========

// Which monitor a window belongs to is decided by its CENTER point, so a window straddling the seam
// lands on exactly one. The rectangle's left/top edge is inclusive and its right/bottom edge
// exclusive, which is what stops two adjacent monitors from both claiming a point on the seam.
TEST(MonitorSelectionTest, TheCenterPointPicksExactlyOneMonitorWithHalfOpenBounds) {
  constexpr MonitorRect kSingle[] = { { 0, 0, 1920, 1080 } };
  constexpr MonitorRect kDual[] = { { 0, 0, 2560, 1440 }, { 2560, 0, 1920, 1080 } };
  constexpr MonitorRect kOffset[] = { { 100, 200, 300, 400 } };
  struct Case {
    const char* name;
    int cx;
    int cy;
    const MonitorRect* rects;
    int count;
    int expect_index;
  };
  const Case kCases[] = {
    { "no monitors at all", 100, 100, nullptr, 0, -1 },
    { "inside the only monitor", 960, 540, kSingle, 1, 0 },
    { "outside the only monitor", 2000, 500, kSingle, 1, -1 },
    { "inside the primary", 1200, 700, kDual, 2, 0 },
    { "inside the secondary", 3500, 500, kDual, 2, 1 },
    // A window at x=2400 w=400 has its center at 2600, in the secondary.
    { "straddling the seam, center wins", 2600, 200, kDual, 2, 1 },
    { "left/top edge is inclusive", 100, 200, kOffset, 1, 0 },
    { "right/bottom edge is exclusive", 400, 200, kOffset, 1, -1 },
  };
  for (const Case& c : kCases) {
    EXPECT_EQ(SelectMonitorIndexByCenter(c.cx, c.cy, c.rects, c.count), c.expect_index) << c.name;
  }
}

// ========== Aspect-fit clamp detection (screen-too-small feedback) ==========
//
// Tests use the production layout constants (kLeftPanelWidth=400, kRightPanelWidth=300,
// kTopBarHeight=40, kStatusBarHeight=28) so they exercise the same arithmetic
// path as ApplyAspectRatio.

namespace {
constexpr float kCollapsedStripWidth = 20.0f;  // Mirror app.cpp's local constant.
}

// The reported ratio and the clamp flag must agree, on every workarea. Only the wide-monitor row
// pins a polarity outright (2:1 fits, so nothing may be clamped) and only the small-screen 2:1 row
// pins that it must NOT fit; the rest assert the invariant that matters — was_clamped mirrors the
// actual deviation, so the helper can never silently misreport — because the precise numerics of
// the recalc_w gate matter less than never lying about them.
TEST(AspectFitTest, TheClampFlagAlwaysMirrorsTheDeviationItReports) {
  struct Case {
    const char* name;
    int win_w;
    float ratio;
    int work_w;
    int work_h;
    int expect_clamped;  // -1 = derive it from the deviation
  };
  const Case kCases[] = {
    { "wide monitor easily fits 2:1", 1600, 2.0f, 2880, 1800, 0 },
    // 1280x720 picking 2:1: the preview region cannot reach it, because the chrome (panels + top
    // bar + status bar) eats too much.
    { "small screen cannot fit 2:1", 1280, 2.0f, 1280, 720, 1 },
    { "small screen, 16:9", 1280, 16.0f / 9.0f, 1280, 720, -1 },
    // Portrait: the ratio < 1 branch is height-driven, so preview_h grows past the screen if
    // anything rather than preview_w.
    { "portrait on a wide monitor", 1600, 0.5f, 2880, 1800, -1 },
  };
  for (const Case& c : kCases) {
    AspectFitResult fit = ResolveAspectFit(c.win_w, c.ratio, c.work_w, c.work_h, kLeftPanelWidth, kRightPanelWidth,
                                           kTopBarHeight, kStatusBarHeight);
    EXPECT_FLOAT_EQ(fit.requested_preview_ratio, c.ratio) << c.name;
    const float deviation = std::abs(fit.achieved_preview_ratio - c.ratio) / c.ratio;
    EXPECT_EQ(fit.was_clamped, deviation >= kAspectClampTolerance) << c.name;
    if (c.expect_clamped >= 0) {
      EXPECT_EQ(fit.was_clamped, c.expect_clamped != 0) << c.name;
    }
    if (c.expect_clamped == 1) {
      // Sanity band rather than an exact value: what matters is that it stayed plausible.
      EXPECT_LT(fit.achieved_preview_ratio, c.ratio) << c.name;
      EXPECT_GT(fit.achieved_preview_ratio, 0.5f) << c.name;
    }
  }
}

// Collapsed panels → less chrome → small monitor can now fit 2:1 closer.
// We don't assert clamp polarity strictly (depends on numerics), but assert
// the helper consumes the reduced overhead by producing a *less* clamped
// achieved ratio than the equivalent expanded-panels case.
TEST(AspectFitTest, PanelsCollapsedReducesOverhead) {
  AspectFitResult expanded = ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                                              /*work_w=*/1280, /*work_h=*/720, kLeftPanelWidth, kRightPanelWidth,
                                              kTopBarHeight, kStatusBarHeight);
  AspectFitResult collapsed = ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                                               /*work_w=*/1280, /*work_h=*/720, kCollapsedStripWidth,
                                               kCollapsedStripWidth, kTopBarHeight, kStatusBarHeight);
  // Collapsing panels shouldn't make the achieved ratio worse on a small
  // screen — preview region grows toward the full window width.
  EXPECT_GE(collapsed.achieved_preview_ratio, expanded.achieved_preview_ratio);
}

// Pathological case: chrome eats 100% of available height → helper must not
// divide by zero / NaN; was_clamped should stay false (we have no signal).
TEST(AspectFitTest, ChromeExceedsHeightYieldsBenignDefault) {
  AspectFitResult fit =
      ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                       /*work_w=*/1280, /*work_h=*/kMinWindowHeight, kLeftPanelWidth, kRightPanelWidth,
                       /*topbar_h=*/kMinWindowHeight,
                       /*statusbar_h=*/0.0f);
  // Whatever the achieved ratio is, was_clamped must be deterministic (not
  // NaN-driven). The function falls back to "achieved == requested" on the
  // pathological branch.
  EXPECT_FALSE(fit.was_clamped);
  EXPECT_FLOAT_EQ(fit.achieved_preview_ratio, fit.requested_preview_ratio);
}

}  // namespace
}  // namespace lumice::gui
