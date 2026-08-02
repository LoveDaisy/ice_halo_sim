// The one overlay-label case that needs a real ImGui frame: what it asserts is which DRAW LIST
// DrawOverlayLabels writes into (window vs foreground), which only exists inside a frame.
//
// The other 38 call ComputeOverlayLabels directly and inspect the returned labels, so they moved
// to test/unit-correctness/gui/test_overlay_labels.cpp.

#include <vector>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"
#include "test_gui_shared.hpp"

void RegisterOverlayLabelTests(ImGuiTestEngine* engine) {
  // Test F: Modal z-order regression — DrawOverlayLabels must target the current
  // window's draw list (not foreground), so modals correctly occlude labels.
  // F9: ImGui API calls must go through GuiFunc (main thread); TestFunc reads
  // captured data across threads via file-static struct.
  {
    static struct OverlayZorderCapture {
      int fg_before = -1;
      int fg_after = -1;
      int wnd_before = -1;
      int wnd_after = -1;
      bool begin_succeeded = false;
      bool done = false;
    } g_capture;

    ImGuiTest* t = IM_REGISTER_TEST(engine, "overlay_labels", "modal_does_not_leak_to_foreground");
    t->GuiFunc = [](ImGuiTestContext*) {
      if (g_capture.done) {
        return;
      }

      static std::vector<lumice::gui::OverlayLabel> labels;
      if (labels.empty()) {
        lumice::gui::OverlayLabel lbl{};
        lbl.screen_x = 100.0f;
        lbl.screen_y = 100.0f;
        lbl.text = "H";
        lbl.color = IM_COL32(255, 255, 255, 255);
        lbl.has_bg = false;
        lbl.group = 0;
        labels.push_back(lbl);
      }

      g_capture.fg_before = ImGui::GetForegroundDrawList()->VtxBuffer.Size;

      ImGui::SetNextWindowSize(ImVec2(200, 100), ImGuiCond_Always);
      bool ok = ImGui::Begin("##TestHostWindow", nullptr, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse);
      g_capture.begin_succeeded = ok;
      if (ok) {
        g_capture.wnd_before = ImGui::GetWindowDrawList()->VtxBuffer.Size;
        // Mock viewport rect aligned with the host window size set above.
        lumice::gui::DrawOverlayLabels(labels, 0.0f, 0.0f, 200.0f, 100.0f);
        g_capture.wnd_after = ImGui::GetWindowDrawList()->VtxBuffer.Size;
      }
      ImGui::End();

      g_capture.fg_after = ImGui::GetForegroundDrawList()->VtxBuffer.Size;
      g_capture.done = true;
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_capture = {};
      ctx->Yield(3);
      IM_CHECK(g_capture.done);
      IM_CHECK(g_capture.begin_succeeded);
      IM_CHECK_GT(g_capture.wnd_after, g_capture.wnd_before);
      IM_CHECK_EQ(g_capture.fg_after, g_capture.fg_before);
    };
  }
}
