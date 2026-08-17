// The window's chrome: the top bar's shape, the collapse strips, the splitter between a side panel
// and the viewport, and where the log panel sits in the stack.
//
// What this suite is for. These are the parts of the shell that have no state of their own — they
// are about where things are and what is on top of what, which is a property of a rendered frame
// and of nothing else. Two of them are load-bearing in a way that is easy to miss: a top-bar slot
// whose width changes with its label makes every control to its right jump while a simulation
// starts and stops, and a floating window that cannot come forward is a log panel the user cannot
// read once they have clicked anything else.
//
// Deliberately NOT here, with where each lives instead. What the Run slot DOES is
// functional/test_run_lifecycle.cpp — which states in so many words that the slot's width across
// its three labels is a separate proposition belonging to the top bar's chrome, i.e. here. What the
// log panel CONTAINS is functional/test_log_panel.cpp. Whether the Colors button opens the Colors
// window is functional/test_color_window.cpp.
//
// What a user sees when these break: a toolbar that shifts sideways the moment they press Run, a
// collapsed panel they cannot get back, or a log panel that vanishes behind the side panels as soon
// as they touch anything.

#include <cstring>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/gui_constants.hpp"
#include "gui/log_sink.hpp"  // ImGuiLogSink — the panel's second gate
// imgui_internal.h is normally an anti-pattern. Z-order has no public reading: it is the ORDER of
// ImGuiContext::Windows, and the two rules relied on here (BringWindowToDisplayFront splices to the
// back; a window created with NoBringToFrontOnFocus is pushed to the front, i.e. the bottom) are
// documented in the convention block at the top of src/gui/app_panels.cpp. An ImGui upgrade that
// changes either must update both.
#include "imgui_internal.h"
#include "test_gui_shared.hpp"

namespace {

// The top bar's run slot, under each of its three labels.
const char* const kRunBtn = "##TopBar/" ICON_FA_PLAY " Run";
const char* const kStopBtn = "##TopBar/" ICON_FA_STOP " Stop";
const char* const kStoppingBtn = "##TopBar/" ICON_FA_STOP " Stopping...";

// Index of a window in ImGui's submission-order list, or -1. Later index means visually higher.
int WindowStackIndex(const char* name) {
  ImGuiContext* g = ImGui::GetCurrentContext();
  for (int i = 0; i < g->Windows.Size; ++i) {
    if (std::strcmp(g->Windows[i]->Name, name) == 0) {
      return i;
    }
  }
  return -1;
}

}  // namespace

void RegisterShellChromeTests(ImGuiTestEngine* engine) {
  // P1 / P13. The collapse toggle and the strip that brings the panel back, as one round trip —
  // half of it is not a feature: a panel that collapses and cannot be restored is a panel the user
  // has lost.
  //
  // The strip's button is clicked by POSITION rather than by item path, and stays that way now that
  // the button is an ordinary widget in the collapsed panel's own window: the proposition is that a
  // real pointer at the place a user would aim at reaches it, which is exactly what an unreachable
  // expand button breaks. The geometry below is therefore derived the same way the drawing code
  // derives it. Its occluded counterpart — a click under a floating window correctly NOT reaching
  // the background — is the entry card's, in functional/test_entry_management.cpp.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "shell_chrome", "collapsing_the_left_panel_hides_it_and_the_strip_brings_it_back");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.left_panel_collapsed);
      IM_CHECK(ctx->GetWindowByRef("##LeftPanel") != nullptr);

      // The strip spans the space between the top bar and the status bar, with a square button
      // centred vertically in it (RenderCollapsedStrip, src/gui/app_panels.cpp). kCollapseBtnSize is
      // file-local there and is mirrored here rather than exported for one test.
      constexpr float kCollapseBtnSize = 20.0f;

      // The toggle's label carries the chevron that points the way it will move, so the path
      // depends on the current state — expanded here.
      ctx->ItemClick("##TopBar/" ICON_FA_CHEVRON_LEFT "##left_panel_toggle");
      ctx->Yield(3);
      IM_CHECK(gui::g_state.left_panel_collapsed);
      // What "collapsed" means is that the panel gives its column up to the preview, and the width
      // is what says so. It used to be checked as "the window stops being submitted at all", which
      // was true of the fixed-coordinate layout but is not a property of collapsing: the panel is a
      // dock node now, and a docked window that stops being submitted takes its node out of the
      // layout entirely — the column would go to the preview and the strip would end up drawn on top
      // of it rather than beside it.
      ImGuiWindow* left = ctx->GetWindowByRef("##LeftPanel");
      IM_CHECK(left != nullptr);
      IM_CHECK_LE(left->Size.x, kCollapseBtnSize);

      const ImGuiViewport* vp = ImGui::GetMainViewport();
      const float strip_h = vp->Size.y - gui::kTopBarHeight - gui::kStatusBarHeight;
      const float btn_y = gui::kTopBarHeight + (strip_h - kCollapseBtnSize) * 0.5f;
      ctx->MouseMoveToPos(ImVec2(vp->Pos.x + kCollapseBtnSize * 0.5f, vp->Pos.y + btn_y + kCollapseBtnSize * 0.5f));
      ctx->MouseClick(0);
      ctx->Yield(3);

      IM_CHECK(!gui::g_state.left_panel_collapsed);
      // The other half of the round trip: the column comes back at its default width, not at some
      // width the collapse left behind.
      left = ctx->GetWindowByRef("##LeftPanel");
      IM_CHECK(left != nullptr);
      IM_CHECK_EQ(left->Size.x, gui::kLeftPanelWidth);
    };
  }

  // The side panels are dock nodes, so the seam between a panel and the viewport is a splitter the
  // user can drag. That is the whole point of the docking substrate, and nothing else in the suite
  // states it — a regression that froze the splitter (or left the panels placed by arithmetic again)
  // would show up as every other case still passing.
  //
  // The preview is checked alongside the panel because the two are one proposition: the viewport is
  // placed from the central dock node's rectangle, so a panel that resizes while the preview stays
  // put means the preview is still being derived from the width constants.
  //
  // The preview WINDOW's rectangle is what is read, not g_preview_vp: the GL viewport is only
  // published when there is something to draw into it, and this case has no simulation result. The
  // two come from the same central-node rect one line apart in RenderPreviewPanel.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "shell_chrome", "dragging_the_left_splitter_resizes_the_panel_and_the_preview");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      ImGuiWindow* left = ctx->GetWindowByRef("##LeftPanel");
      IM_CHECK(left != nullptr);
      IM_CHECK_EQ(left->Size.x, gui::kLeftPanelWidth);
      ImGuiWindow* preview = ctx->GetWindowByRef("##PreviewPanel");
      IM_CHECK(preview != nullptr);
      const float preview_x_before = preview->Pos.x;
      const float preview_w_before = preview->Size.x;

      // The splitter sits in the gap immediately right of the panel; aim at its middle, which is
      // half of style.DockingSeparatorSize past the panel's right edge.
      const ImGuiViewport* vp = ImGui::GetMainViewport();
      const float seam_x = gui::kLeftPanelWidth + ImGui::GetStyle().DockingSeparatorSize * 0.5f;
      const float seam_y = vp->Size.y * 0.5f;
      constexpr float kDragBy = 60.0f;

      ctx->MouseMoveToPos(ImVec2(vp->Pos.x + seam_x, vp->Pos.y + seam_y));
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(ImVec2(vp->Pos.x + seam_x + kDragBy, vp->Pos.y + seam_y));
      ctx->MouseUp(0);
      ctx->Yield(3);

      left = ctx->GetWindowByRef("##LeftPanel");
      IM_CHECK(left != nullptr);
      // Not an exact width: the splitter lands where the pointer left it, and ImGui truncates the
      // resulting node sizes. What must be true is that the drag moved the panel, and moved it the
      // way it was dragged.
      IM_CHECK_GT(left->Size.x, gui::kLeftPanelWidth);
      preview = ctx->GetWindowByRef("##PreviewPanel");
      IM_CHECK(preview != nullptr);
      IM_CHECK_GT(preview->Pos.x, preview_x_before);
      IM_CHECK_LT(preview->Size.x, preview_w_before);

      // Drag back, for two reasons. It states that a splitter drag is reversible — a resize that
      // quantises differently in each direction leaves the user unable to get their layout back. And
      // it is how this case cleans up after itself: the dock layout outlives ResetTestState, so a
      // panel left 60 px wider would be the width every later case (including the ones comparing
      // against reference images) runs at. Restoring by dragging rather than by rebuilding the
      // layout keeps the panels docked throughout; a rebuild costs the next case a frame in which
      // the panels' child windows are not submitted.
      ctx->MouseMoveToPos(ImVec2(vp->Pos.x + seam_x + kDragBy, vp->Pos.y + seam_y));
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(ImVec2(vp->Pos.x + seam_x, vp->Pos.y + seam_y));
      ctx->MouseUp(0);
      ctx->Yield(3);

      left = ctx->GetWindowByRef("##LeftPanel");
      IM_CHECK(left != nullptr);
      IM_CHECK_EQ(left->Size.x, gui::kLeftPanelWidth);
      preview = ctx->GetWindowByRef("##PreviewPanel");
      IM_CHECK(preview != nullptr);
      IM_CHECK_EQ(preview->Pos.x, preview_x_before);
      IM_CHECK_EQ(preview->Size.x, preview_w_before);
    };
  }

  // P3. The run slot holds three different labels over a run's lifetime, and the button is sized so
  // that all three occupy the same rectangle — otherwise every control to its right slides sideways
  // twice per run, which reads as the toolbar flinching.
  //
  // The three states are reached the way the product reaches them (a run intent plus the in-flight
  // stop latch), not by writing the label: a width that only holds when a test poses the button
  // would hold in no real run.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "shell_chrome", "the_run_slot_keeps_one_rectangle_across_its_three_labels");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Slot {
        const char* name;
        gui::RunIntent intent;
        bool stop_inflight;
        const char* button;
      };
      const Slot kSlots[] = {
        { "idle", gui::RunIntent::kNone, false, kRunBtn },
        { "simulating", gui::RunIntent::kRunning, false, kStopBtn },
        { "stop draining", gui::RunIntent::kStopping, true, kStoppingBtn },
      };

      ResetTestState();
      ctx->Yield(3);

      float width = -1.0f;
      float right_neighbour_x = -1.0f;
      for (const Slot& s : kSlots) {
        gui::g_state.run_intent = s.intent;
        gui::g_state.dirty = false;
        gui::g_stop_inflight.store(s.stop_inflight);
        ctx->Yield(3);

        const ImGuiTestItemInfo btn = ctx->ItemInfo(s.button, ImGuiTestOpFlags_NoError);
        if (btn.ID == 0) {
          IM_ERRORF("%s: the run slot is not showing %s", s.name, s.button);
          continue;
        }
        // The neighbour is what the user actually notices moving. Revert is always submitted (it is
        // hidden by alpha rather than by omission), so it is a stable landmark in all three states.
        const ImGuiTestItemInfo neighbour = ctx->ItemInfo("##TopBar/Revert", ImGuiTestOpFlags_NoError);
        if (width < 0.0f) {
          width = btn.RectFull.GetWidth();
          right_neighbour_x = neighbour.RectFull.Min.x;
          continue;
        }
        if (btn.RectFull.GetWidth() != width) {
          IM_ERRORF("%s: the slot is %.1f px wide, the first state measured %.1f", s.name,
                    static_cast<double>(btn.RectFull.GetWidth()), static_cast<double>(width));
        }
        if (neighbour.RectFull.Min.x != right_neighbour_x) {
          IM_ERRORF("%s: the control to the slot's right moved to x=%.1f from %.1f", s.name,
                    static_cast<double>(neighbour.RectFull.Min.x), static_cast<double>(right_neighbour_x));
        }

        if (ctx->IsError()) {
          break;
        }
      }
      IM_CHECK_GT(width, 0.0f);  // a run of three misses would leave this unset

      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_stop_inflight.store(false);
      ctx->Yield(2);
    };
  }

  // P56. The six background panels carry NoBringToFrontOnFocus so a stray click cannot raise one
  // over a modal; the log panel deliberately does NOT, because it is a floating window the user is
  // meant to be able to bring forward and read. That asymmetry is the proposition — and it is
  // invisible in any state, since both windows are "open" either way.
  //
  // The panel is gated twice in this harness (a CLI flag and the presence of a sink), and neither
  // gate is reset by ResetTestState, so both are restored by a guard rather than at the end of the
  // body — an assertion failure part-way through would otherwise leak the log panel into every case
  // that runs after this one.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "shell_chrome", "the_log_panel_can_come_forward_over_the_side_panels");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      struct LogPanelGateGuard {
        bool prev_enable;
        std::shared_ptr<gui::ImGuiLogSink> prev_sink;
        LogPanelGateGuard() : prev_enable(g_enable_log_panel), prev_sink(gui::g_imgui_log_sink) {
          g_enable_log_panel = true;
          if (!gui::g_imgui_log_sink) {
            gui::g_imgui_log_sink = std::make_shared<gui::ImGuiLogSink>();
          }
        }
        ~LogPanelGateGuard() {
          gui::g_imgui_log_sink = prev_sink;
          g_enable_log_panel = prev_enable;
          gui::g_state.log_panel_open = false;
        }
      } gate_guard;

      gui::g_state.log_panel_open = true;
      ctx->Yield(4);

      // Try to raise a background panel. Without its flag this would splice ##LeftPanel to the back
      // of the list, i.e. above the log panel.
      ctx->WindowFocus("##LeftPanel");
      ctx->Yield(2);

      const int log_idx = WindowStackIndex("##LogPanel");
      const int left_idx = WindowStackIndex("##LeftPanel");
      IM_CHECK_GE(log_idx, 0);
      IM_CHECK_GE(left_idx, 0);
      IM_CHECK_GT(log_idx, left_idx);
    };
  }
}
