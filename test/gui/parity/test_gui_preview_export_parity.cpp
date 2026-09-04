// Screen<->export parity — the gate that makes "the Screenshot writes the pixels the screen is
// showing" a proposition something can turn red on.
//
// The proposition. One frame publishes one set of inputs (`g_preview_vp.params`, its
// `curve_labels`, its device size and DPI). Two production paths consume that one set:
//   (1) the screen, through RenderPreviewFrameAndBlit — the preview's persistent FBO, blitted 1:1
//       back onto the viewport rectangle of the default framebuffer, read back here through the
//       g_fullframe_capture sub-region hook;
//   (2) the export, through RenderExportToRgba — a freshly allocated FBO of the same size, read
//       back by the same glReadPixels helper.
// Both call the same core (RenderFrameContentToBoundFbo, src/gui/export_fbo_renderer.cpp). The
// comparison below is what says they still do.
//
// Why this file exists at all. PR #304 collapsed the two paths onto that shared core, but nothing
// in the tree was asserting the collapse. What `functional/test_export.cpp` has is
// `DriveRepeatAndComparePsnr` — the SAME export path run twice and compared to itself. That
// measures whether the export is stable; it cannot see whether the export agrees with the screen,
// and two paths can be stably, reproducibly, identically wrong relative to each other while every
// such case stays green. Both user-visible defects PR #304 fixed (the default Screenshot dropping
// every overlay label; Retina export text coming out at half the screen's size) lived in exactly
// that blind spot. gui-polish-v10 had promised this comparison as a mandatory acceptance
// criterion and it was landed as the self-comparison instead; this is that gate, rebuilt.
//
// What it does NOT subsume, and must not be "simplified" into. `DriveRepeatAndComparePsnr` stays
// where it is and keeps its own job (export determinism across two invocations). And this case
// deliberately does NOT re-derive the export's labels: it hands the export arm the very
// `CurveLabelSet` objects RenderPreviewPanel published for the screen. Recomputing them — what
// `test_export.cpp`'s `RunExportRequest` does, and the reason its labelled case compares at ~33.6
// dB rather than exactly — would fold the label builders' own run-to-run float jitter into this
// measurement and force a calibrated threshold onto a question that has an exact answer. The
// determinism of the label recompute is that other suite's proposition, not this one's.
//
// ================================ Honest boundary: the DPI axis ==============================
//
// THIS GATE CANNOT SEE A DPI MISMATCH, and that is structural rather than an oversight. On macOS
// test/gui/test_gui_main.cpp pins `GLFW_COCOA_RETINA_FRAMEBUFFER = GLFW_FALSE` so the harness
// window's framebuffer size equals its window size (the on-screen reference groups need a
// deterministic capture size, so that hint is not negotiable from here); everywhere else the
// harness runs at a 1:1 surface too. So `g_preview_vp.dpi_scale_*` is 1.0 in every gui_test case
// that will ever run, and the halved-export-text defect could be reintroduced without this
// comparison moving a byte. The DPI axis is covered by `export/label_size_scales_with_the_target_dpi`
// in test/gui/functional/test_export.cpp, which drives RenderExportToRgba at an explicit dpi_scale
// of 2 and measures the rendered glyph extent. The two cases are complements: that one varies the
// DPI and looks at one arm, this one holds the DPI at 1 and compares two arms.
//
// ================================ Why the inset is zero, and stays =============================
//
// The capture hook reads the DEFAULT framebuffer, after ImGui_ImplOpenGL3_RenderDrawData, so the
// obvious hazard is chrome ImGui paints over the blit along the rectangle's edge — pixels that do
// not belong to the preview at all and that the export has no counterpart for. `kInsetPx` exists
// to skip such a ring, and the answer to how wide it must be is zero, on two independent grounds
// that agree:
//   * White box. `##PreviewPanel` is opened with `ImGuiWindowFlags_NoBackground`
//     (src/gui/app_panels.cpp), and ImGui's RenderWindowOuterBorders draws its `AddRect` only
//     when that flag is ABSENT; the fallback branch beside it needs `ImGuiChildFlags_ResizeX/Y`,
//     which a top-level window does not carry. So no window border is emitted over this
//     rectangle, whatever style.WindowBorderSize says.
//   * Measured. At `kInsetPx = 0` the two arms come back byte-identical over all 900x912 pixels
//     of the viewport, this suite's whole rectangle, with nothing skipped.
// Keep it at 0. Widening it is how this gate would quietly stop watching its own edges — and the
// edge is where a blit-rectangle or DPI-rounding regression shows up FIRST. If a future change
// really does paint chrome inside this rectangle, the honest fix is to say which pixels and why,
// here, with a number that came from a measurement like the one above; it is not a dial to turn
// until a red goes away.
//
// The one condition under which a nonzero inset was ever observed is out of this harness's reach:
// a one-off probe that forced `GLFW_COCOA_RETINA_FRAMEBUFFER` on saw the outermost 1 px differ at
// dpi 2, where `vp_x = (int)(panel_x * dpi_scale_x)` truncates and the neighbours' edges can land
// inside the rounded rectangle. See the DPI section above for why no gui_test case can reach that.
//
// Byte-exact is the right bar here, and it is a measured fact rather than an aspiration: with the
// two arms fed one snapshot of one frame's inputs, they run identical GL work on identical data in
// the same process, so any difference is a real divergence between the paths. Do not answer a red
// by fitting a PSNR threshold to it — that would rebuild the very self-satisfying ruler this file
// exists to replace.
//
// ==================================== Cadence and assets ====================================
//
// No committed reference image, so this case must never enter scripts/regen_gui_test_refs.py's
// GROUPS registry — the same property that puts it under test/gui/parity/ rather than
// test/gui/visual/ (doc/testing-architecture.md §4.10). Like the rest of that tag it runs on a
// developer machine with a real GL context via ./scripts/test.sh {quick,full,pr} and in no CI job
// today (§7.5).

#include <cstdio>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/overlay_labels.hpp"
#include "imgui.h"
#include "test_gui_shared.hpp"

namespace {

// Pixels skipped on each of the four sides. Zero, measured and argued in the "Why the inset is
// zero" block above — a named knob at 0, not a leftover: it is the one place a future harness that
// really does paint chrome over this rectangle would state how much, and it keeps the diagnostic
// line below reporting how many pixels were actually compared.
constexpr int kInsetPx = 0;

// One off-screen render of a snapshot the test took from g_preview_vp, answered in RGBA.
//
// The snapshot is the mechanism, not an optimization: the export must consume the inputs the
// captured FRAME published, so the TestFunc copies them out next to the capture and the GuiFunc
// renders from the copy instead of re-reading the (by then, newer) global.
struct PreviewExportRequest {
  bool upload_requested = false;
  bool upload_done = false;

  bool export_requested = false;
  bool export_done = false;
  bool export_ok = false;

  // Snapshot inputs.
  gui::PreviewParams params;
  std::vector<gui::CurveLabelSet> curve_labels;
  int dst_w = 0;
  int dst_h = 0;
  float dpi_x = 1.0f;
  float dpi_y = 1.0f;

  // Output.
  std::vector<unsigned char> rgba;

  void Reset() { *this = PreviewExportRequest{}; }
};

PreviewExportRequest g_req;

// UploadTexture and RenderExportToRgba both need the GL context, which is current only on the
// render thread; TestFunc runs on the test engine's coroutine. Same request/answer scaffolding
// every GL-touching suite here uses, and a non-capturing function pointer because
// ImGuiTestGuiFunc is a raw `void (*)(ImGuiTestContext*)`.
void PreviewExportGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.upload_requested && !g_req.upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_req.upload_done = true;
  }
  if (g_req.export_requested && !g_req.export_done) {
    g_req.rgba = gui::RenderExportToRgba(gui::g_preview, g_req.params, g_req.dst_w, g_req.dst_h, g_req.curve_labels,
                                         g_req.dpi_x, g_req.dpi_y);
    g_req.export_ok = !g_req.rgba.empty();
    g_req.export_done = true;
    g_req.export_requested = false;
  }
}

// Byte-for-byte over the inset interior. Returns the number of differing pixels and reports the
// first one — a coordinate plus both sides' RGBA is what makes a red diagnosable without a second
// run, and "which pixel" is the first question a real divergence gets asked.
int CountInsetPixelDiffs(const std::vector<unsigned char>& screen, const std::vector<unsigned char>& exported, int w,
                         int h) {
  int diffs = 0;
  bool reported = false;
  for (int y = kInsetPx; y < h - kInsetPx; ++y) {
    for (int x = kInsetPx; x < w - kInsetPx; ++x) {
      const size_t i = (static_cast<size_t>(y) * w + x) * 4;
      if (screen[i] == exported[i] && screen[i + 1] == exported[i + 1] && screen[i + 2] == exported[i + 2] &&
          screen[i + 3] == exported[i + 3]) {
        continue;
      }
      ++diffs;
      if (!reported) {
        reported = true;
        fprintf(stderr,
                "[preview_export_parity] first mismatch at (%d,%d): screen=(%u,%u,%u,%u) "
                "export=(%u,%u,%u,%u)\n",
                x, y, screen[i], screen[i + 1], screen[i + 2], screen[i + 3], exported[i], exported[i + 1],
                exported[i + 2], exported[i + 3]);
      }
    }
  }
  return diffs;
}

}  // namespace

void RegisterPreviewExportParityTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "preview_export_parity", "the_screen_and_the_export_read_the_same_fbo");
  t->GuiFunc = PreviewExportGuiFunc;
  t->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    g_req.Reset();

    // The overlay families are off by default (gui_state.hpp), and a scene with no label in it
    // would be blind to both of the defects this gate was built after. Lines and labels both: the
    // label anchors come from the same annotation overlay the lines are drawn from.
    gui::g_state.show_horizon_line = true;
    gui::g_state.show_horizon_label = true;
    gui::g_state.show_grid_line = true;
    gui::g_state.show_grid_label = true;

    // Something for the projection to sample. A blank texture would make "the two arms show the
    // same content" trivially true over most of the frame.
    g_req.upload_requested = true;
    ctx->Yield(2);
    IM_CHECK(g_req.upload_done);
    IM_CHECK(gui::g_preview.HasTexture());
    // The viewport is published by RenderPreviewPanel a frame behind the upload, and the top bar
    // syncs renderer state into g_preview_vp.params at draw time — one frame further still.
    ctx->Yield(3);

    // Off the preview, and off every widget: a hover highlight is drawn after the blit and would
    // land inside the compared region if the cursor happened to rest on the panel.
    ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
    ctx->Yield(2);

    IM_CHECK(gui::g_preview_vp.active);
    const int vp_x = gui::g_preview_vp.vp_x;
    const int vp_y = gui::g_preview_vp.vp_y;
    const int vp_w = gui::g_preview_vp.vp_w;
    const int vp_h = gui::g_preview_vp.vp_h;
    IM_CHECK(vp_w > 2 * kInsetPx);
    IM_CHECK(vp_h > 2 * kInsetPx);
    // Not a boundary check: an empty list means the labelled half of this comparison is vacuous,
    // and the case would keep passing while covering strictly less than it claims to.
    IM_CHECK(!gui::g_preview_vp.curve_labels.empty());

    // The screen arm. g_preview_vp's rectangle is already device pixels with a bottom-left origin
    // — the space glReadPixels and the g_fullframe_capture rect protocol both work in — so it is
    // passed through untouched. That absence of a coordinate conversion is why this is the right
    // capture point: a conversion here would be a second implementation of the placement the
    // comparison is supposed to be checking.
    g_fullframe_capture.Reset();
    g_fullframe_capture.rect_x = vp_x;
    g_fullframe_capture.rect_y = vp_y;
    g_fullframe_capture.rect_w = vp_w;
    g_fullframe_capture.rect_h = vp_h;
    g_fullframe_capture.requested.store(true);
    for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
      ctx->Yield(1);
    }
    IM_CHECK(g_fullframe_capture.done.load());
    IM_CHECK_EQ(g_fullframe_capture.width, vp_w);
    IM_CHECK_EQ(g_fullframe_capture.height, vp_h);
    const std::vector<unsigned char> screen = g_fullframe_capture.pixels;
    IM_CHECK_EQ(screen.size(), static_cast<size_t>(vp_w) * vp_h * 4);

    // The export arm, from the snapshot taken here — with no ctx interaction between the capture
    // above and these copies, so what the export renders is what the captured frame published.
    g_req.export_requested = false;
    g_req.export_done = false;
    g_req.export_ok = false;
    g_req.rgba.clear();
    g_req.params = gui::g_preview_vp.params;
    g_req.curve_labels = gui::g_preview_vp.curve_labels;
    g_req.dst_w = vp_w;
    g_req.dst_h = vp_h;
    g_req.dpi_x = gui::g_preview_vp.dpi_scale_x;
    g_req.dpi_y = gui::g_preview_vp.dpi_scale_y;
    g_req.export_requested = true;
    ctx->Yield(2);
    IM_CHECK(g_req.export_done);
    IM_CHECK(g_req.export_ok);
    IM_CHECK_EQ(g_req.rgba.size(), screen.size());

    const int diffs = CountInsetPixelDiffs(screen, g_req.rgba, vp_w, vp_h);
    const int compared = (vp_w - 2 * kInsetPx) * (vp_h - 2 * kInsetPx);
    fprintf(stderr, "[preview_export_parity] %dx%d inset %d px: %d/%d pixels differ\n", vp_w, vp_h, kInsetPx, diffs,
            compared);
    IM_CHECK_EQ(diffs, 0);
  };
}
