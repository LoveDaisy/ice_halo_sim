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
#include <limits>
#include <string>
#include <vector>

#include "gui/aspect_ratio_rules.hpp"
#include "gui/composite_exposure_push.hpp"
#include "gui/edit_modal_rules.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_ev_auto.hpp"
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
  { LUMICE_SHAPE_SCALAR_HEIGHT, "Height", 0.01f, 100.0f, "%.2f", SliderScale::kLog },
  { LUMICE_SHAPE_SCALAR_UPPER_H, "Upper H", 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_PRISM_H, "Prism H", 0.0f, 100.0f, "%.4f", SliderScale::kLogLinear },
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

// Helper: build a packed XYZ buffer (stride 3) of size w*h, with Y filled from a row-major
// vector of length w*h and X/Z left at 0 (ComputeP99Y looks only at channel 1).
std::vector<float> MakeXyz(int w, int h, const std::vector<float>& y_values) {
  std::vector<float> data(static_cast<size_t>(w) * static_cast<size_t>(h) * 3, 0.0f);
  for (size_t i = 0; i < y_values.size(); ++i) {
    data[i * 3 + 1] = y_values[i];
  }
  return data;
}

// (The former T1 pinned the f=1 fine path against a no-sizes overload of ComputeP99Y. That
// overload is gone: the function now takes a borrowed `const float*`, which carries no length to
// fall back on, so img_width/img_height became required. With only one call shape left the case
// reduced to asserting a value equals itself, so it was deleted rather than kept as a tautology.
// T2-T4 below still cover the box-sum exact values, the all-zero fallback and the wc=0 guard.)

// T2 — f=2 correctness on a 4x4 image: hand-verify the box-sum + P99 + /f^2.
//
// Layout (Y channel, row-major 4x4):
//   1  2 |  3  4
//   5  6 |  7  8
//   ----+-----
//   9 10 | 11 12
//  13 14 | 15 16
//
// Four 2x2 coarse bins (f=2), each sum:
//   top-left:     1+2+5+6   = 14
//   top-right:    3+4+7+8   = 22
//   bottom-left:  9+10+13+14= 46
//   bottom-right: 11+12+15+16=54
//
// P99 over the 4 nonzero coarse Y values:
//   sorted = {14, 22, 46, 54}; idx = floor(4 * 0.99) = 3 -> y_vals[3] = 54.
// Returned value = 54 / (f^2 = 4) = 13.5.
TEST(EvAuto, ComputeP99YBoxSumsCoarselyAndFallsBackWhenItCannot) {
  std::vector<float> y = { 1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,
                           9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f };
  std::vector<float> xyz = MakeXyz(4, 4, y);
  EXPECT_FLOAT_EQ(ComputeP99Y(xyz.data(), 4, 4, 2), 13.5f);

  // Sanity: the helper itself returns the raw coarse sums.
  std::vector<float> coarse = DownsampleBoxSumY(xyz.data(), 4, 4, 2);
  ASSERT_EQ(coarse.size(), 4u);
  EXPECT_FLOAT_EQ(coarse[0], 14.0f);  // top-left
  EXPECT_FLOAT_EQ(coarse[1], 22.0f);  // top-right
  EXPECT_FLOAT_EQ(coarse[2], 46.0f);  // bottom-left
  EXPECT_FLOAT_EQ(coarse[3], 54.0f);  // bottom-right

  // T3 — all-zero input on the coarse path returns 0.0f (matches the fine-path empty case).
  std::vector<float> zeros = MakeXyz(8, 8, std::vector<float>(8 * 8, 0.0f));
  EXPECT_FLOAT_EQ(ComputeP99Y(zeros.data(), 8, 8, 8), 0.0f);

  // T4 — wc=0 guard: f=8 on a 1x1 image gives wc = hc = 0, so DownsampleBoxSumY collapses to {} and
  // ComputeP99Y falls back to the fine path (which on a single positive Y returns that value).
  std::vector<float> tiny = MakeXyz(1, 1, { 7.5f });
  EXPECT_FLOAT_EQ(ComputeP99Y(tiny.data(), 1, 1, 8), 7.5f);
  EXPECT_TRUE(DownsampleBoxSumY(tiny.data(), 1, 1, 8).empty());
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
// Convention 1 — Prism Height: [0.01, 100] kLog
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
  // Prism height is logarithmic over [0.01, 100], so the geometric midpoint -- not the arithmetic
  // one -- sits at the middle of the slider. That is the whole point of the law: 0.1 and 10 are
  // equally far from 1.
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
  // jumps under the cursor as it crosses.
  const AnchorCase kLogLinearCases[] = {
    { "log-linear, exact zero", 0.0f, 0.0f, 0.0f },
    { "log-linear, the switch point", kLogLinearX0, kLogLinearTSwitch, 1e-6f },
    { "log-linear, high end", 100.0f, 1.0f, 1e-3f },
  };
  for (const AnchorCase& c : kLogLinearCases) {
    EXPECT_NEAR(LogLinearValueToNorm(c.value, 100.0f), c.norm, 1e-6f) << c.name;
    EXPECT_NEAR(LogLinearNormToValue(c.norm, 100.0f), c.value, c.value_tol) << c.name;
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
    const float round_trip = LogLinearNormToValue(LogLinearValueToNorm(value, 100.0f), 100.0f);
    EXPECT_NEAR(round_trip, value, std::max(value, 1e-3f) * 1e-3f) << "log-linear value=" << value;
  }
  float prev_log_linear = LogLinearNormToValue(0.0f, 100.0f);
  for (int i = 1; i <= 40; ++i) {
    const float value = LogLinearNormToValue(static_cast<float>(i) / 40.0f, 100.0f);
    // Non-decreasing rather than increasing: the two pieces may tie exactly at the seam.
    EXPECT_GE(value, prev_log_linear) << "log-linear step " << i;
    prev_log_linear = value;
  }
}

// The third convention, [0, 1] linear, has no helper at all: SliderWithInput hands the raw value to
// ImGui and the range is enforced by std::clamp at the call site. There is nothing of ours to
// assert, and five cases asserting std::clamp were exactly that -- so the coverage that matters
// lives where the call site is driven, in gui_test's pyramid_h_* modal cases.

using lumice::gui::AspectFitResult;
using lumice::gui::ClampWindowSizeToWorkarea;
using lumice::gui::kAspectClampTolerance;
using lumice::gui::kDisplayStripHeight;
using lumice::gui::kLeftPanelWidth;
using lumice::gui::kMinWindowHeight;
using lumice::gui::kMinWindowWidth;
using lumice::gui::kRightPanelWidth;
using lumice::gui::kStatusBarHeight;
using lumice::gui::kTopBarHeight;
using lumice::gui::MonitorRect;
using lumice::gui::ResolveAspectFit;
using lumice::gui::SelectMonitorIndexByCenter;
using lumice::gui::SplitViewportForDisplayStrip;
using lumice::gui::ViewportStripSplit;

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

// ========== Viewport / display-strip split ==========
//
// The preview and the display strip stack inside one band, and the only property that cannot be
// seen by looking at either window alone is that they are COMPLEMENTARY: a one-pixel gap shows the
// framebuffer behind them, a one-pixel overlap paints ImGui chrome over the GL preview, and both
// look plausible in a screenshot of either half. That is why the split has a single owner and why
// this asserts the relation rather than the two rects' individual values.
TEST(ViewportStripSplitTest, ThePreviewAndTheStripExactlyPartitionTheBand) {
  struct Case {
    const char* name;
    float band_y;
    float band_h;
    float strip_h;
  };
  const Case kCases[] = {
    { "default 1600x980 window", kTopBarHeight, 980.0f - kTopBarHeight - kStatusBarHeight, kDisplayStripHeight },
    { "minimum-height window", kTopBarHeight, kMinWindowHeight - kTopBarHeight - kStatusBarHeight,
      kDisplayStripHeight },
    { "band offset by a dragged splitter", 137.0f, 400.0f, kDisplayStripHeight },
    // Degenerate bands: the strip may not push the preview to a negative height, and it may not
    // start outside the band it was given.
    { "band shorter than the strip", kTopBarHeight, 40.0f, kDisplayStripHeight },
    { "band of zero height", kTopBarHeight, 0.0f, kDisplayStripHeight },
  };
  for (const Case& c : kCases) {
    const ViewportStripSplit split = SplitViewportForDisplayStrip(c.band_y, c.band_h, c.strip_h);
    EXPECT_GE(split.preview_h, 0.0f) << c.name;
    EXPECT_GE(split.strip_h, 0.0f) << c.name;
    // The strip starts exactly where the preview stops...
    EXPECT_FLOAT_EQ(split.strip_y, c.band_y + split.preview_h) << c.name;
    // ...and together they cover the band, no more and no less.
    EXPECT_FLOAT_EQ(split.preview_h + split.strip_h, std::max(0.0f, c.band_h)) << c.name;
    EXPECT_LE(split.strip_h, c.strip_h) << c.name;
  }
}

// A band with room to spare gives the strip its full requested height — the strip is FIXED, so a
// taller window must grow the preview, never the strip. (The partition test above is satisfied by a
// split that hands the strip everything, which is why this is asserted separately.)
TEST(ViewportStripSplitTest, TheStripKeepsItsFixedHeightWhileTheBandCanHoldIt) {
  const ViewportStripSplit small = SplitViewportForDisplayStrip(kTopBarHeight, 400.0f, kDisplayStripHeight);
  const ViewportStripSplit large = SplitViewportForDisplayStrip(kTopBarHeight, 1200.0f, kDisplayStripHeight);
  EXPECT_FLOAT_EQ(small.strip_h, kDisplayStripHeight);
  EXPECT_FLOAT_EQ(large.strip_h, kDisplayStripHeight);
  EXPECT_GT(large.preview_h, small.preview_h);
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
