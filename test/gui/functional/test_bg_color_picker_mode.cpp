// The Sky Color eyedropper, driven through the real panel.
//
// What this suite is for. The mapping from a cursor position to a photo pixel is pure and is
// asserted where it belongs — unit-correctness/gui/test_bg_color_picker.cpp holds the uv square's
// boundary, the absence of a second Y flip, and the byte-exact sRGB read. What that file
// structurally cannot see is everything the mode IS: a button that has to be reachable and
// correctly disabled, a display-time override that has to reach the published PreviewParams and
// leave GuiState alone, four existing mouse gestures that have to stop while it is armed, and a
// key that has to cancel it. All four need a live frame, so they live here.
//
// What a user sees when these break: the eyedropper samples a colour the preview was not showing
// them (overlays still drawn over the photo, or the render still blended into it); the sky colour
// lands somewhere other than where they clicked; or the click that takes the colour also swings
// the camera, because the confirm fires on mouse-down and the rest of that press falls through to
// the orbit handler — which is the defect the last case below was written against.
//
// Deliberately NOT here. No reference image and no PSNR: every proposition is a widget outcome or
// a published parameter, so this is `functional/`, not `visual/`, and it owns nothing that
// scripts/regen_gui_test_refs.py needs to know about.

#include <cmath>
#include <cstdint>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// The full label, icon glyph included: the test engine derives the item id from the whole
// string, so the "##" suffix alone does not address it.
const char* const kPickButton = "**/" ICON_FA_EYE_DROPPER "##display_sky_pick";

// A photograph synthesized at exactly the viewport's device-pixel size, so the contain fit is the
// identity and image pixel (col,row) is device pixel (col,row) of the viewport. Colour encodes
// position in 4-pixel blocks, which makes the expected value at any click computable from the
// click's own coordinates WITHOUT going through the production mapping — including its origin, the
// thing a quadrant-sized pattern cannot see.
// See BlockColor: 4-device-pixel blocks, so a click landing a pixel off the intended spot still
// reads the intended colour, while the 8x6-point shift a wrong sampling origin would introduce is
// two blocks or more and cannot pass.
struct ProbeBg {
  std::vector<unsigned char> rgb;
  int width = 0;
  int height = 0;
  bool requested = false;
  bool done = false;
};
ProbeBg g_probe_bg;

constexpr int kBlock = 4;

void BlockColor(int col, int row, unsigned char out[3]) {
  out[0] = static_cast<unsigned char>((col / kBlock * 37) % 256);
  out[1] = static_cast<unsigned char>((row / kBlock * 53) % 256);
  out[2] = 96;
}

void BuildProbeImage(int w, int h) {
  g_probe_bg.width = w;
  g_probe_bg.height = h;
  g_probe_bg.rgb.assign(static_cast<size_t>(w) * h * 3, 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      BlockColor(x, y, &g_probe_bg.rgb[(static_cast<size_t>(y) * w + x) * 3]);
    }
  }
}

// The one piece of main-thread work these cases need: the GL upload. A request rather than a
// direct call because a TestFunc runs on the test engine's coroutine, and a GL call from there is
// a call on the wrong thread.
void ProbeGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_probe_bg.requested && !g_probe_bg.done) {
    gui::g_preview.UploadBgTexture(g_probe_bg.rgb.data(), g_probe_bg.width, g_probe_bg.height);
    // The CPU mirror the eyedropper reads. In the app LoadAndUploadBgImage writes both together;
    // it is static and its only public entry opens a file dialog, so the probe fills both here.
    gui::g_state.bg_pixels = g_probe_bg.rgb;
    gui::g_state.bg_pixel_w = g_probe_bg.width;
    gui::g_state.bg_pixel_h = g_probe_bg.height;
    g_probe_bg.done = true;
  }
}

void UploadProbeImage(ImGuiTestContext* ctx) {
  g_probe_bg.done = false;
  g_probe_bg.requested = true;
  ctx->Yield(3);
  IM_CHECK(g_probe_bg.done);
}

// Put a shown background in place, sized to the live viewport. Two uploads, not one: the panel
// publishes no viewport at all until it has a texture or a background to draw
// (RenderPreviewPanel's outer `if`), so the size the real image must match is not knowable until
// SOME background is already loaded. The first upload is a throwaway that makes vp_w/vp_h appear.
void InstallProbeBackground(ImGuiTestContext* ctx) {
  ResetTestState();
  ctx->Yield(2);
  BuildProbeImage(4, 4);
  UploadProbeImage(ctx);
  gui::g_state.bg_show = true;
  ctx->Yield(3);
  IM_CHECK_GT(gui::g_preview_vp.vp_w, 0);

  BuildProbeImage(gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
  UploadProbeImage(ctx);
  IM_CHECK(gui::g_preview.HasBackground());
  gui::g_state.bg_show = true;
  ctx->Yield(3);
}

// The preview window's top-left in screen points — the origin the sampler measures against.
ImVec2 PreviewOrigin(ImGuiTestContext* ctx) {
  ImGuiWindow* w = ctx->GetWindowByRef("//##PreviewPanel");
  IM_CHECK_SILENT_RETV(w != nullptr, ImVec2(0, 0));
  return w->Pos;
}

}  // namespace

void RegisterBgColorPickerModeTests(ImGuiTestEngine* engine) {
  // AC1 — the button exists, is disabled with no photo / a hidden photo, and toggles the mode.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg_color_picker", "button_gating_and_toggle");
    t->GuiFunc = ProbeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists(kPickButton));
      IM_CHECK_EQ(gui::BgPhotoOnScreen(gui::g_state), false);  // no photo yet

      InstallProbeBackground(ctx);
      IM_CHECK_EQ(gui::BgPhotoOnScreen(gui::g_state), true);

      IM_CHECK_EQ(gui::g_bg_pick.active, false);
      ctx->ItemClick(kPickButton);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_bg_pick.active, true);
      ctx->ItemClick(kPickButton);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_bg_pick.active, false);

      // Hiding the photo makes it unavailable again, and drops an armed mode.
      ctx->ItemClick(kPickButton);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_bg_pick.active, true);
      gui::g_state.bg_show = false;
      ctx->Yield(2);
      IM_CHECK_EQ(gui::BgPhotoOnScreen(gui::g_state), false);
      IM_CHECK_EQ(gui::g_bg_pick.active, false);
    };
  }

  // AC2 — while picking, the frame published to the renderer is the photo alone; leaving restores.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg_color_picker", "pick_mode_publishes_the_photo_alone");
    t->GuiFunc = ProbeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      InstallProbeBackground(ctx);
      // Turn every overlay on so "off" below is a change, not the default.
      gui::g_state.show_horizon_line = true;
      gui::g_state.show_grid_line = true;
      gui::g_state.show_sun_circles_line = true;
      gui::g_state.show_lens_border_line = true;
      gui::g_state.bg_alpha = 0.5f;
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_preview_vp.params.overlay.show_horizon, true);
      IM_CHECK_GT(gui::g_preview_vp.params.bg.alpha, 0.0f);

      ctx->ItemClick(kPickButton);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_preview_vp.params.bg.alpha, 0.0f);
      IM_CHECK_EQ(gui::g_preview_vp.params.overlay.show_horizon, false);
      IM_CHECK_EQ(gui::g_preview_vp.params.overlay.show_grid, false);
      IM_CHECK_EQ(gui::g_preview_vp.params.overlay.show_sun_circles, false);
      IM_CHECK_EQ(gui::g_preview_vp.params.overlay.show_lens_border, false);
      IM_CHECK_EQ(gui::g_preview_vp.curve_labels.empty(), true);
      // The document is untouched — this is a display-time override, not a state change.
      IM_CHECK_EQ(gui::g_state.show_horizon_line, true);
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.5f);

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_bg_pick.active, false);
      IM_CHECK_EQ(gui::g_preview_vp.params.overlay.show_horizon, true);
      IM_CHECK_GT(gui::g_preview_vp.params.bg.alpha, 0.0f);
    };
  }

  // AC3 wiring + AC4's value — the click writes the pixel under the cursor, measured against an
  // oracle built from the click's own coordinates rather than from the production mapping.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg_color_picker", "click_writes_the_pixel_under_the_cursor");
    t->GuiFunc = ProbeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      InstallProbeBackground(ctx);
      const ImVec2 origin = PreviewOrigin(ctx);
      const float dpi_x = gui::g_preview_vp.dpi_scale_x;
      const float dpi_y = gui::g_preview_vp.dpi_scale_y;

      // Four spread-out points; each also pins the mapping's handedness, since a mirrored map
      // would land in a different block.
      const float kFrac[4][2] = { { 0.30f, 0.25f }, { 0.72f, 0.25f }, { 0.30f, 0.68f }, { 0.72f, 0.68f } };
      for (const auto& f : kFrac) {
        if (!gui::g_bg_pick.active) {
          ctx->ItemClick(kPickButton);
          ctx->Yield(2);
        }
        const float px = static_cast<float>(gui::g_preview_vp.vp_w) * f[0] / dpi_x;
        const float py = static_cast<float>(gui::g_preview_vp.vp_h) * f[1] / dpi_y;
        ctx->MouseMoveToPos(ImVec2(origin.x + px, origin.y + py));
        ctx->Yield(2);
        ctx->MouseClick(0);
        ctx->Yield(2);

        const int col = static_cast<int>(std::floor(px * dpi_x));
        const int row = static_cast<int>(std::floor(py * dpi_y));
        unsigned char want[3];
        BlockColor(col, row, want);
        const float* got = gui::g_state.renderer.background;
        // Within one block: MouseMoveToPos may land a device pixel off, which must not matter,
        // while the 8x6-point offset a wrong origin would introduce is two blocks or more.
        for (int c = 0; c < 3; ++c) {
          const int got_byte = static_cast<int>(std::lround(got[c] * 255.0f));
          if (std::abs(got_byte - static_cast<int>(want[c])) > 0) {
            IM_ERRORF("at frac (%.2f,%.2f) channel %d: got %d, want %d (block col=%d row=%d)", f[0], f[1], c, got_byte,
                      static_cast<int>(want[c]), col, row);
            break;
          }
        }
        // Non-fatal: the next iteration re-arms the mode from whatever state this one left, so one
        // point failing here must not decide the verdict for the three that follow it.
        if (gui::g_bg_pick.active) {
          IM_ERRORF("at frac (%.2f,%.2f): confirming a pick did not leave pick mode", f[0], f[1]);
        }
        // Once anything above has reported, every ctx-> call no-ops, so the remaining points would
        // report echoes of this failure rather than findings of their own. Stop at the first.
        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // AC5 — while picking, neither the camera nor the photo can be dragged or zoomed.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg_color_picker", "gestures_are_locked_out_while_picking");
    t->GuiFunc = ProbeGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      InstallProbeBackground(ctx);
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;  // a lens the camera drag is live for
      ctx->Yield(2);

      const ImVec2 origin = PreviewOrigin(ctx);
      const ImVec2 a(origin.x + gui::g_preview_vp.vp_w / gui::g_preview_vp.dpi_scale_x * 0.40f,
                     origin.y + gui::g_preview_vp.vp_h / gui::g_preview_vp.dpi_scale_y * 0.40f);
      const ImVec2 b(a.x + 60.0f, a.y + 40.0f);

      // Baseline: the camera drag DOES move the view when not picking, so the lockout below is a
      // real difference rather than a gesture that never worked in this harness.
      const float az0 = gui::g_state.renderer.azimuth;
      ctx->MouseMoveToPos(a);
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(b);
      ctx->Yield(2);
      ctx->MouseUp(0);
      ctx->Yield(2);
      IM_CHECK_NE(gui::g_state.renderer.azimuth, az0);

      ctx->ItemClick(kPickButton);
      ctx->Yield(2);
      const float az1 = gui::g_state.renderer.azimuth;
      const float pan1 = gui::g_state.bg_offset_x;
      const float zoom1 = gui::g_state.bg_scale;

      ctx->MouseMoveToPos(a);
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(b);
      ctx->Yield(2);
      ctx->MouseUp(0);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, az1);
      IM_CHECK_EQ(gui::g_state.bg_offset_x, pan1);
      IM_CHECK_EQ(gui::g_state.bg_scale, zoom1);
    };
  }
}
