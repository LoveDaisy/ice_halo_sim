// The log panel: whether it exists, and the three controls in its header row.
//
// What this suite is for. `RenderLogPanel` (src/gui/app_panels.cpp) is the only place a user can
// see what the GUI and the core are saying, and it is gated twice — the panel is drawn only when
// the user has opened it AND a sink exists to read from. Both gates matter for the same reason:
// this window is the one place a diagnostic can be reported, so a panel that silently fails to
// appear takes the diagnostic with it. None of it is visible without a frame; the panel is not a
// piece of state that can be inspected, it is a window that is either submitted or is not.
//
// Deliberately NOT here, with where each lives instead. Where the panel sits in the window stack —
// the one thing about it that is not local to this file — is functional/test_shell_chrome.cpp, next
// to the other z-order propositions. What the sink does with a message is the logger's, not the
// panel's.
//
// The harness's two gates are CLI globals that ResetTestState does not touch, so every case below
// installs them through an RAII guard rather than clearing them at the end: an assertion failure
// part-way through a case returns immediately, and a case that leaked the log panel would change
// the layout for every case that ran after it in this process.
//
// What a user sees when these break: a Log button that does nothing, or a level they set that the
// next message ignores.

#include <memory>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/gui_logger.hpp"
#include "gui/log_sink.hpp"
#include "test_gui_shared.hpp"

namespace {

// Installs the harness's two log-panel gates and restores them on the way out.
//
// The sink is ATTACHED to the GUI logger, not merely published: g_imgui_log_sink is the pointer the
// panel reads, and in production the same object is also in the logger's sink list — a sink that is
// only published receives nothing, and every assertion about what the panel is showing would then
// be an assertion about an empty buffer. The logger's sink list and level are restored too, since
// they are process-wide and gui_test is one process.
struct ScopedLogPanel {
  bool prev_enable;
  std::shared_ptr<gui::ImGuiLogSink> prev_sink;
  std::vector<spdlog::sink_ptr> prev_sinks;
  spdlog::level::level_enum prev_level;
  int prev_gui_log_level;

  ScopedLogPanel()
      : prev_enable(g_enable_log_panel), prev_sink(gui::g_imgui_log_sink), prev_sinks(gui::GetGuiLogger().sinks()),
        prev_level(gui::GetGuiLogger().level()), prev_gui_log_level(gui::g_state.gui_log_level) {
    g_enable_log_panel = true;
    gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
    gui::GetGuiLogger().sinks().push_back(gui::g_imgui_log_sink);
  }
  ~ScopedLogPanel() {
    gui::GetGuiLogger().sinks() = prev_sinks;
    gui::GetGuiLogger().set_level(prev_level);
    gui::g_state.gui_log_level = prev_gui_log_level;
    gui::g_imgui_log_sink = prev_sink;
    g_enable_log_panel = prev_enable;
    gui::g_state.log_panel_open = false;
  }
};

bool PanelIsUp(ImGuiTestContext* ctx) {
  ImGuiWindow* w = ctx->GetWindowByRef("##LogPanel");
  return w != nullptr && w->WasActive;
}

}  // namespace

void RegisterLogPanelTests(ImGuiTestEngine* engine) {
  // P53. Two independent gates, and the panel needs both. Each is driven on its own so that a
  // regression collapsing them into one — an `||` where there should be an `&&` — fails here rather
  // than shipping a panel that renders against a null sink.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "log_panel", "the_panel_needs_both_the_open_flag_and_a_sink");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedLogPanel scoped;
      ctx->Yield(3);

      // Gate 1: not opened.
      gui::g_state.log_panel_open = false;
      ctx->Yield(3);
      IM_CHECK(!PanelIsUp(ctx));

      // Both gates satisfied.
      gui::g_state.log_panel_open = true;
      ctx->Yield(3);
      IM_CHECK(PanelIsUp(ctx));

      // Gate 2: opened, but no sink to read from. RenderLogPanel returns before Begin.
      std::shared_ptr<gui::ImGuiLogSink> sink = gui::g_imgui_log_sink;
      gui::g_imgui_log_sink.reset();
      ctx->Yield(3);
      IM_CHECK(!PanelIsUp(ctx));
      gui::g_imgui_log_sink = sink;
      ctx->Yield(3);
      IM_CHECK(PanelIsUp(ctx));
    };
  }

  // P46. The status bar's Log button is the panel's only affordance, and its chevron is the only
  // indication of which way it will go. Both directions are driven, because a button that opened
  // but never closed would satisfy either half alone.
  //
  // The chevron is part of the button's LABEL rather than a separate glyph, which is what makes it
  // assertable at all: the two labels are two different item paths, so "the chevron flipped" and
  // "the button is still there" are the same observation.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "log_panel", "the_log_button_flips_its_chevron_and_the_panel_with_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedLogPanel scoped;
      gui::g_state.log_panel_open = false;
      ctx->Yield(3);

      const char* const kClosed = "##StatusBar/" ICON_FA_CHEVRON_RIGHT " Log";
      const char* const kOpen = "##StatusBar/" ICON_FA_CHEVRON_DOWN " Log";

      IM_CHECK(ctx->ItemExists(kClosed));
      IM_CHECK(!ctx->ItemExists(kOpen));
      IM_CHECK(!PanelIsUp(ctx));

      ctx->ItemClick(kClosed);
      ctx->Yield(4);
      IM_CHECK(gui::g_state.log_panel_open);
      IM_CHECK(PanelIsUp(ctx));
      IM_CHECK(ctx->ItemExists(kOpen));
      IM_CHECK(!ctx->ItemExists(kClosed));

      ctx->ItemClick(kOpen);
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.log_panel_open);
      IM_CHECK(!PanelIsUp(ctx));
      IM_CHECK(ctx->ItemExists(kClosed));
    };
  }

  // P54. The GUI level combo takes effect immediately — it is not a preference that a later Apply
  // picks up — because the reason a user reaches for it is a message they want to see NOW. The
  // assertion reads the sink's own level rather than the state field the combo writes: the field
  // moving is not the claim, the sink following it is.
  //
  // The Core combo's other half (that it only reaches a server when there is one) is not driven
  // here: with no server the branch is a no-op with nothing to observe, and standing up a real
  // server to watch a log level change would be a run-lifecycle test wearing this one's name.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "log_panel", "the_gui_level_combo_moves_the_sink_at_once");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedLogPanel scoped;
      gui::g_state.log_panel_open = true;
      ctx->Yield(4);
      IM_CHECK(PanelIsUp(ctx));

      // "Error" is index 5 of Trace / Debug / Verbose / Info / Warning / Error / Off. The item is
      // addressed by its label; the combo BUTTON is not in the item registry (BeginCombo does not
      // report one), so the popup is opened through ComboClick's window-scoped path.
      //
      // The effect is read as "what still reaches the sink", not as the level number the combo
      // wrote. That is deliberate on two counts: the number would be the field compared against
      // itself, and this project remaps spdlog's severities (GUI_LOG_INFO expands to spdlog's WARN,
      // see the macro block in gui_logger.hpp), so a numeric expectation here would encode the
      // remap rather than the behaviour.
      ctx->SetRef("//##LogPanel");
      ctx->ComboClick("##GuiLevel/Error");
      ctx->SetRef("");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.gui_log_level, 5);

      const size_t before_suppressed = gui::g_imgui_log_sink->Size();
      GUI_LOG_INFO("an informational line, below the selected level");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_imgui_log_sink->Size(), before_suppressed);  // dropped

      ctx->SetRef("//##LogPanel");
      ctx->ComboClick("##GuiLevel/Info");
      ctx->SetRef("");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.gui_log_level, 3);

      GUI_LOG_INFO("the same line, now at or above the selected level");
      ctx->Yield(2);
      IM_CHECK_GT(gui::g_imgui_log_sink->Size(), before_suppressed);  // let through
    };
  }

  // P55. Clear empties the buffer the content area reads from. Asserted on the sink's own size
  // rather than on the rendered rows: the rows are clipped text with no ids of their own, and the
  // buffer is what the panel draws from, so an empty buffer is the whole claim.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "log_panel", "clear_empties_what_the_panel_is_reading");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ScopedLogPanel scoped;
      gui::g_state.log_panel_open = true;
      ctx->Yield(4);
      IM_CHECK(PanelIsUp(ctx));

      gui::GetGuiLogger().set_level(spdlog::level::trace);  // nothing filtered while seeding
      GUI_LOG_INFO("log panel test line");
      ctx->Yield(2);
      IM_CHECK_GT(gui::g_imgui_log_sink->Size(), (size_t)0);  // the premise: there is something to clear

      ctx->ItemClick("**/Clear");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_imgui_log_sink->Size(), (size_t)0);
    };
  }
}
