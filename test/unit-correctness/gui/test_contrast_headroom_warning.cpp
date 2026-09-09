// The GUI half of the contrast-headroom warning (doc/print-mode-subtractive-ink.md §8, owner
// decision D6), asked the questions that need no window.
//
// Two propositions live here, and neither is reachable from the predicate's own test
// (test/unit-correctness/util/test_contrast_headroom.cpp, which asserts what the margin IS):
//
//   - the GUI asks about the right FIELD in the right DIRECTION. `tone` selects both, and the two
//     fields' defaults are each other's degenerate value (sky black, paper white), so a wiring
//     mistake here does not produce a near-miss — it produces a warning on every default document
//     that happens to be in the other tone.
//   - the one-click fix changes what it says it changes, and nothing else. This is AC3's substance:
//     an offer to fix something is worse than no offer if the button moves the wrong field.
//
// Deliberately NOT here: whether the notice is non-blocking, and whether it appears on screen at
// all. Those are properties of the call site in app_panels.cpp — it renders a TextColored plus a
// SmallButton inline, opening no popup — and the structural reading of that code is the evidence,
// the same standing panels.cpp's LayerProducesNoRays notice has. Whether the CLI honours the same
// predicate is test/unit-correctness/server/test_contrast_headroom_warning.cpp.

#include <gtest/gtest.h>

#include <algorithm>

#include "gui/gui_state.hpp"
#include "include/lumice.h"
#include "util/contrast_headroom.hpp"

namespace gui = lumice::gui;

namespace {

constexpr float kLevel = 1.0f / 255.0f;
// Four 8-bit levels inside / outside the threshold, so nothing here turns on a rounding question —
// the exact boundary belongs to the predicate's own test.
constexpr float kNoHeadroomBright = 1.0f - (lumice::kContrastHeadroomWarnLevels - 4) * kLevel;
constexpr float kNoHeadroomDark = (lumice::kContrastHeadroomWarnLevels - 4) * kLevel;

gui::RenderConfig Renderer(int tone, float sky, float paper) {
  gui::RenderConfig r;
  r.tone = tone;
  std::fill(r.background, r.background + 3, sky);
  std::fill(r.paper, r.paper + 3, paper);
  return r;
}

}  // namespace

// The four states of AC2, read off the GUI's own wrapper. The two quiet arms carry the shipped
// defaults rather than arbitrary safe values: if the default document warns, the notice is furniture
// and the user learns to look past it.
TEST(ContrastHeadroomWarningGui, TheFourStatesOfTheTwoLaws) {
  const gui::RenderConfig kDefaults{};
  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(Renderer(LUMICE_TONE_SCREEN, kDefaults.background[0], 1.0f)))
      << "the default black sky under screen has the whole range to itself";
  EXPECT_TRUE(gui::ContrastHeadroomIsLowFor(Renderer(LUMICE_TONE_SCREEN, kNoHeadroomBright, 1.0f)));

  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(Renderer(LUMICE_TONE_PRINT, 0.0f, kDefaults.paper[0])))
      << "the default white paper under print has the whole range to itself";
  EXPECT_TRUE(gui::ContrastHeadroomIsLowFor(Renderer(LUMICE_TONE_PRINT, 0.0f, kNoHeadroomDark)));
}

// `tone` selects the field, not only the direction. Each document here is degenerate in the ground
// the live operator does not read, and is therefore fine — and because the two fields' defaults ARE
// each other's degenerate value, this is the arm a one-field wiring mistake fails, loudly.
TEST(ContrastHeadroomWarningGui, OnlyTheGroundTheLiveOperatorReadsIsJudged) {
  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(Renderer(LUMICE_TONE_SCREEN, 0.0f, kNoHeadroomDark)))
      << "under screen the paper is not the ground";
  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(Renderer(LUMICE_TONE_PRINT, kNoHeadroomBright, 1.0f)))
      << "under print the background is not the ground";

  // And a default-constructed renderer, which is what a fresh document holds, is quiet — asserted
  // separately from the arms above because it is the state the user is in before touching anything.
  const gui::RenderConfig fresh;
  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(fresh));
}

// The margin wrapper reports the same number the shared predicate does, for the field `tone` names.
// Worth an assertion of its own because it is what the tooltip shows: a readout taken from the other
// field would be a plausible-looking number that contradicts the warning sitting next to it.
TEST(ContrastHeadroomWarningGui, TheMarginReadoutAgreesWithTheSharedPredicate) {
  const gui::RenderConfig screen = Renderer(LUMICE_TONE_SCREEN, 250.0f * kLevel, 0.0f);
  EXPECT_NEAR(gui::ContrastHeadroomMarginFor(screen), 5.0f * kLevel, 1e-6f);
  EXPECT_EQ(gui::ContrastHeadroomMarginFor(screen),
            lumice::ContrastHeadroomMargin(screen.background, lumice::ToneLawLimit::kWhite));

  const gui::RenderConfig print = Renderer(LUMICE_TONE_PRINT, 1.0f, 7.0f * kLevel);
  EXPECT_NEAR(gui::ContrastHeadroomMarginFor(print), 7.0f * kLevel, 1e-6f);
  EXPECT_EQ(gui::ContrastHeadroomMarginFor(print),
            lumice::ContrastHeadroomMargin(print.paper, lumice::ToneLawLimit::kBlack));
}

// AC3, screen arm: the fix switches the operator and KEEPS the sky the user picked. Keeping it is the
// decision being asserted, not an omission — a pale sky is a perfectly good paper, and it is the
// additive law that cannot use it. A fix that also reset the colour would discard a choice the user
// made on purpose in order to repair a consequence they did not.
TEST(ContrastHeadroomWarningGui, TheScreenFixSwitchesToneAndKeepsTheSky) {
  gui::RenderConfig r = Renderer(LUMICE_TONE_SCREEN, kNoHeadroomBright, 0.5f);
  const gui::RenderConfig before = r;
  gui::ApplyHeadroomFix(r);

  EXPECT_EQ(r.tone, LUMICE_TONE_PRINT);
  EXPECT_TRUE(std::equal(r.background, r.background + 3, before.background)) << "the chosen sky colour is kept";
  EXPECT_TRUE(std::equal(r.paper, r.paper + 3, before.paper)) << "and the paper is not touched either";

  // The fix actually fixes it: the state it produces no longer warns. Asserted rather than assumed,
  // because "switch to print" only helps while the paper on the other side has headroom of its own —
  // this renderer's paper is 0.5, which does.
  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(r));
}

// AC3, print arm: the fix resets the paper and STAYS in print. Staying is the decision — the user is
// repairing print's ground, not asking to leave print, and a fix that switched tone back would undo
// the choice they just made.
TEST(ContrastHeadroomWarningGui, ThePrintFixResetsThePaperAndStaysInPrint) {
  gui::RenderConfig r = Renderer(LUMICE_TONE_PRINT, 0.25f, kNoHeadroomDark);
  const gui::RenderConfig before = r;
  gui::ApplyHeadroomFix(r);

  EXPECT_EQ(r.tone, LUMICE_TONE_PRINT) << "the user asked for print; the fix is to its ground, not to the mode";
  const gui::RenderConfig kDefaults{};
  EXPECT_TRUE(std::equal(r.paper, r.paper + 3, kDefaults.paper)) << "the paper goes back to its shipped default";
  EXPECT_TRUE(std::equal(r.background, r.background + 3, before.background)) << "the sky is not touched";

  EXPECT_FALSE(gui::ContrastHeadroomIsLowFor(r));
}

// The fix is only ever offered where the warning fires, but it must also be HARMLESS if reached
// otherwise — a future caller (a menu item, a keyboard shortcut) should not be able to turn a good
// document into a print one by accident. Applied to a healthy screen document it does switch tone,
// which is the honest answer: there is exactly one thing this fix does on that side. Asserted so
// that the coupling is recorded rather than discovered.
TEST(ContrastHeadroomWarningGui, TheFixHasOneBehaviourPerToneRegardlessOfHeadroom) {
  gui::RenderConfig healthy_screen;
  gui::ApplyHeadroomFix(healthy_screen);
  EXPECT_EQ(healthy_screen.tone, LUMICE_TONE_PRINT)
      << "unconditional on this side: the call site is what gates it, and that gate is "
         "ContrastHeadroomIsLowFor";

  gui::RenderConfig healthy_print;
  healthy_print.tone = LUMICE_TONE_PRINT;
  const gui::RenderConfig before = healthy_print;
  gui::ApplyHeadroomFix(healthy_print);
  EXPECT_EQ(healthy_print, before) << "the paper was already at its default, so the fix is a no-op";
}
