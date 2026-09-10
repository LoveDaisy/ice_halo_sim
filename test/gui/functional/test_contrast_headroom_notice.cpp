// The contrast-headroom notice, driven on a real window.
//
// doc/print-mode-subtractive-ink.md §8 (owner decision D6) gives each tone operator one degenerate
// ground, and the predicate that decides it is shared with the CLI
// (util/contrast_headroom.hpp; the unit tests over it and over gui_state.hpp's wrappers own what it
// COMPUTES and what the repair CHANGES). Three things are left over that no test without a window
// can ask, and they are this file's whole subject:
//
//   - the notice is actually SUBMITTED for the degenerate state, and not for the healthy one. The
//     wrappers being correct says nothing about whether app_panels.cpp consults them.
//   - it is NON-BLOCKING. That is asserted as two facts rather than argued from the source: no modal
//     is up while the notice shows, and a sibling control in the same panel is still reachable. A
//     structural reading ("the code calls no popup API") is the right argument for a reviewer and the
//     wrong one for a regression guard — a future edit that wraps this in a BeginPopupModal would
//     leave every other test in this tree green.
//   - the one-click repair reaches the field THROUGH A CLICK, and the notice then goes away. The
//     mutation itself is unit-tested; that the button is wired to it, and that the result clears the
//     condition, is only visible from here.
//
// Items are addressed by their literal path under ##RightPanel rather than by a "**/" wildcard: the
// wildcard form does not resolve these (measured — it reports the items as absent while the literal
// path finds them), which is the same addressing constraint the suite's combo helpers already carry.

#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "imgui.h"
#include "imgui_te_context.h"
#include "imgui_te_engine.h"
#include "test_gui_shared.hpp"

namespace gui = lumice::gui;

namespace {

// The buttons are the notice's addressable half. The warning line itself is an ImGui::TextColored,
// which is submitted with id == 0 and so never enters the test engine's item registry — the same
// property the aspect-clamp warning's comment in test_view_display_controls.cpp records. Here that
// costs nothing, because the button beside it appears under exactly the same condition and is the
// part a user acts on; the two labels also name which arm fired, so they are not interchangeable.
constexpr const char* kSwitchToPrint = "//##RightPanel/Switch to Print##display_headroom_fix";
constexpr const char* kResetPaper = "//##RightPanel/Reset paper to white##display_headroom_fix";
// A control inside the same Display group, used as the "the panel still works" probe.
constexpr const char* kDisplaySibling = "//##RightPanel/##EV##display_input";

void SetAllThree(float (&dst)[3], float v) {
  for (int j = 0; j < 3; j++) {
    dst[j] = v;
  }
}

}  // namespace

void RegisterContrastHeadroomNoticeTests(ImGuiTestEngine* engine) {
  // The screen arm, which is the state the user feedback behind this work came from: a nearly white
  // sky, overlay lines still crisp, halo gone.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "contrast_headroom_notice", "a_near_white_sky_offers_the_print_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.renderer.tone = LUMICE_TONE_SCREEN;
      SetAllThree(gui::g_state.renderer.background, 0.99f);
      ctx->Yield(3);

      IM_CHECK(ctx->ItemExists(kSwitchToPrint));
      IM_CHECK(!ctx->ItemExists(kResetPaper));

      // Non-blocking, both halves.
      IM_CHECK(ImGui::GetTopMostPopupModal() == nullptr);
      IM_CHECK(ctx->ItemExists(kDisplaySibling));

      ctx->ItemClick(kSwitchToPrint);
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.tone, LUMICE_TONE_PRINT);
      // The sky the user picked survives: it is a perfectly good paper, and keeping it is the
      // decision the repair makes (see ApplyHeadroomFix).
      IM_CHECK_EQ(gui::g_state.renderer.background[0], 0.99f);
      // And the notice is gone, which is the half that says the repair actually repaired something:
      // the default paper it lands on has the whole range to itself.
      IM_CHECK(!ctx->ItemExists(kSwitchToPrint));
      IM_CHECK(!ctx->ItemExists(kResetPaper));
    };
  }

  // The print arm. Same shape on the other law, and the asymmetry of the repair is asserted here
  // rather than inferred: the tone stays print, because the user was fixing print's ground.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "contrast_headroom_notice", "a_near_black_paper_offers_the_paper_reset");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.renderer.tone = LUMICE_TONE_PRINT;
      SetAllThree(gui::g_state.renderer.paper, 0.01f);
      // A sky that is neither default nor degenerate, so "the repair left it alone" is a real
      // observation rather than a match against the value it would have had anyway.
      SetAllThree(gui::g_state.renderer.background, 0.3f);
      ctx->Yield(3);

      IM_CHECK(ctx->ItemExists(kResetPaper));
      IM_CHECK(!ctx->ItemExists(kSwitchToPrint));
      IM_CHECK(ImGui::GetTopMostPopupModal() == nullptr);
      IM_CHECK(ctx->ItemExists(kDisplaySibling));

      ctx->ItemClick(kResetPaper);
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.tone, LUMICE_TONE_PRINT);
      IM_CHECK_EQ(gui::g_state.renderer.paper[0], 1.0f);
      IM_CHECK_EQ(gui::g_state.renderer.background[0], 0.3f);
      IM_CHECK(!ctx->ItemExists(kResetPaper));
    };
  }

  // The negative arm, and the reason the two above are not vacuous: a notice that were always drawn
  // would pass both of them. A fresh document is also the state every other case in this binary
  // starts from, so a notice that fired here would be furniture in every screenshot this suite takes.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "contrast_headroom_notice", "a_fresh_document_says_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists(kSwitchToPrint));
      IM_CHECK(!ctx->ItemExists(kResetPaper));
    };
  }
}
