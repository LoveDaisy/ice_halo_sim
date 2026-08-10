// The right panel's View and Display groups — the controls that decide what the preview shows and
// at what shape, as opposed to what the simulation computes.
//
// What this suite is for. These are the panel's own controls (`##RightPanel`), and what can only
// be answered by a real frame is whether a piece of state the panel derives actually reaches the
// screen. The aspect-clamp warning below is the sharpest instance: it is the only feedback a user
// gets when the window they asked for did not fit the monitor, and it is drawn from a signal
// written on a completely different code path (the GLFW resize in ApplyAspectRatio), so "the flag
// was set" and "the user was told" are two separate claims.
//
// NOT YET here, and belonging here. This file currently holds only the aspect-clamp propositions.
// The rest of `##RightPanel`'s View and Display controls — the lens-type combo and the pose
// sliders it re-gates, the FOV and visible-hemisphere gates, Reset, the aspect preset combo and its
// portrait flip, the background row, and SliderWithInput's three value mappings — are still
// asserted from test_gui_interaction.cpp, whose cases are grouped by an old `p1_slider` /
// `p1_layout` category rather than by the window they drive. When those move, they belong in THIS
// file: a second file on the same window is the shape this suite is being rewritten to remove.
//
// Deliberately NOT here. Whether ResolveAspectFit computes was_clamped correctly is arithmetic
// over integers and is asserted over its whole domain in
// unit-correctness/gui/test_gui_widget_rules.cpp; which preset options are disabled with and
// without a background image is asserted over its whole domain in the same file; whether the
// aspect fields survive a document round trip is
// composition-correctness/gui/test_document_roundtrip_chain.cpp. Nothing below restates them.
//
// What a user sees when these break: they pick 2:1 on a laptop screen that cannot hold it, get a
// window that is not 2:1, and nothing on screen explains why — or the opposite, a permanent
// "Screen too small" banner that no preset makes go away.

#include <string>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// The warning's addressable half. app_panels.cpp draws it as a DISABLED Selectable rather than a
// TextColored precisely so it has a real ImGui id: a Text* widget is emitted with id==0 and the
// test engine's item registry never sees it, so this string is the only handle a test has on
// "the user was told". The ratio detail printed on the following line is a plain Text and is
// therefore NOT assertable here — that is a property of ImGui's item registry, not an omission.
constexpr const char* kClampWarning = "**/Screen too small for this aspect";

// Install a clamp signal without going through ApplyAspectRatio.
//
// ApplyAspectRatio is the only producer of this signal, and it produces it by asking GLFW for a
// window size against the real monitor work area and recording what it got (src/gui/app.cpp). A
// test cannot make the monitor smaller, and calling it would resize the harness window for every
// case that runs after this one — the suite's committed pixel references are captured at the
// harness's default framebuffer size, so a stray resize here would be charged to them. The signal
// is therefore written directly and the panel is asked what it does with it, which is the half of
// the contract this layer owns.
void InstallClampSignal(bool was_clamped) {
  gui::g_state.aspect_clamp.was_clamped = was_clamped;
  gui::g_state.aspect_clamp.requested_preview_ratio = 2.0f;
  gui::g_state.aspect_clamp.achieved_preview_ratio = was_clamped ? 1.01f : 2.0f;
}

}  // namespace

void RegisterViewDisplayControlTests(ImGuiTestEngine* engine) {
  // The positive branch: a clamped window on a real preset says so.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display", "a_clamped_aspect_tells_the_user_the_screen_is_too_small");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.aspect_preset = gui::AspectPreset::k2x1;
      InstallClampSignal(true);
      ctx->Yield(2);

      IM_CHECK(ctx->ItemExists(kClampWarning));
    };
  }

  // The Free preset re-check. app_panels.cpp tests the preset a SECOND time here, after
  // ApplyAspectRatio has already cleared the signal on the Free path — a deliberate belt-and-braces
  // guard against a stale signal arriving from a callback path that missed the clear. That makes it
  // a branch with no other guard: delete the re-check and every green test still passes, because
  // the only way to reach it is a state the producer is not supposed to leave behind.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display", "a_stale_clamp_signal_stays_silent_on_the_free_preset");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.aspect_preset = gui::AspectPreset::kFree;
      InstallClampSignal(true);
      ctx->Yield(2);

      IM_CHECK(!ctx->ItemExists(kClampWarning));
    };
  }

  // The negative branch: same preset as the positive case, signal off. Paired with it deliberately
  // — a warning that is always drawn would pass the positive case on its own.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "view_display", "an_aspect_that_fit_says_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.aspect_preset = gui::AspectPreset::k2x1;
      InstallClampSignal(false);
      ctx->Yield(2);

      IM_CHECK(!ctx->ItemExists(kClampWarning));
    };
  }
}
