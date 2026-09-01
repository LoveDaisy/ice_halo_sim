// Writing a picture out: the Screenshot command and the two panorama presets beside it.
//
// What this suite is for. Everything here renders through an off-screen framebuffer that nothing
// else in the process is looking at, and that is the whole design: the exported image must be a
// function of the document and the exposure alone, never of what happens to be on screen at the
// moment the user picks the menu item. That property has exactly one honest way to be checked —
// render for real and read the pixels back — so every case below needs a live GL context and a
// real frame, and none of them can be moved down a layer.
//
// Deliberately NOT here. Where the menu items live and when each is clickable is the top bar's
// business and is pinned in functional/test_file_ops.cpp; which pixel a sky direction lands on for
// each projection is unit-correctness/gui/test_preview_renderer.cpp plus the committed reference
// images in visual/test_gui_lens_projection.cpp. This file asks a narrower question than either:
// given a preview that already renders correctly, does the export reproduce it.
//
// What a user sees when these break: the saved PNG is blank, or the wrong size, or has the
// application's own toolbar baked into a corner of the sky; the second export of a session fails
// or comes out different from the first; the image on screen is bright and the file on disk is
// black because the exposure the user dialled in never reached the exporter.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gl_capture.hpp"
#include "gui/gl_common.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "test_gui_shared.hpp"

namespace {

// ---------------------------------------------------------------------------------------------
// Main-thread scaffolding.
//
// PreviewRenderer::UploadTexture, RenderExportToRgba and ExportPreviewPng all need the GL context,
// which is current only on the render thread. TestFunc runs on the test engine's coroutine worker
// with no context bound, so every GL call here is a request the GuiFunc fulfils on the next frame
// and answers through a plain struct. The GuiFuncs are non-capturing function pointers because
// ImGuiTestGuiFunc is a raw `void (*)(ImGuiTestContext*)` and rejects anything else.
// ---------------------------------------------------------------------------------------------

// A 64x32 field of one uniform XYZ value. The XYZ upload path is the one the exposure uniform
// actually scales — the 8-bit RGB path is intensity-invariant by construction — and a uniform
// field makes mean luminance a reading of the exposure and nothing else, at any projection.
void UploadUniformXyzTexture(float value) {
  constexpr int kW = 64;
  constexpr int kH = 32;
  std::vector<float> xyz(static_cast<size_t>(kW) * kH * 3, value);
  gui::g_preview.UploadXyzTexture(xyz.data(), kW, kH);
}

// The self-owned-ImDrawList probe: build a draw list by hand, hand it to ImDrawData::AddDrawList,
// render it into a framebuffer this function owns, read it back.
struct SpikeState {
  bool requested = false;
  bool done = false;
  bool readback_ok = false;
  // GL_FRAMEBUFFER_COMPLETE, or whatever the driver said instead. Recorded rather than asserted on
  // the render thread, where there is no test context to report through.
  unsigned int fbo_status = 0;
  int buffer_size = 0;
  bool drawn_region_has_red = false;
  unsigned char far_r = 0;
  unsigned char far_g = 0;
  unsigned char far_b = 0;

  void Reset() { *this = SpikeState{}; }
};

SpikeState g_spike_state;

void RunSpikeFboImDrawListEndToEnd() {
  constexpr int kW = 256;
  constexpr int kH = 128;

  GLint prev_fbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);

  GLuint fbo = 0;
  GLuint rbo = 0;
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glGenRenderbuffers(1, &rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, kW, kH);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbo);

  // Asked before anything is drawn, because this case exists to give a ONE-LINE diagnosis and an
  // incomplete FBO does not produce one on its own: none of the four calls above returns a status,
  // and a readback from an incomplete framebuffer can come back "successfully" full of zeroes. That
  // path ends in "the red rectangle is missing", which reads as an ImGui draw-list regression — the
  // exact wrong conclusion on a machine whose driver simply refused the RGBA8 renderbuffer.
  g_spike_state.fbo_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

  glViewport(0, 0, kW, kH);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  ImDrawList dl(ImGui::GetDrawListSharedData());
  // ImGui internal API (underscore prefix); verified for v1.91.8-docking (commit f6a6076).
  dl._ResetForNewFrame();
  dl.PushTextureID(ImGui::GetIO().Fonts->TexID);
  dl.PushClipRect(ImVec2(0.0f, 0.0f), ImVec2((float)kW, (float)kH));
  dl.AddRectFilled(ImVec2(10.0f, 10.0f), ImVec2(30.0f, 30.0f), IM_COL32(255, 0, 0, 255));
  dl.PopClipRect();
  dl.PopTextureID();

  ImDrawData draw_data;
  draw_data.Clear();
  draw_data.DisplayPos = ImVec2(0.0f, 0.0f);
  draw_data.DisplaySize = ImVec2((float)kW, (float)kH);
  draw_data.FramebufferScale = ImVec2(1.0f, 1.0f);
  draw_data.AddDrawList(&dl);

  ImGui_ImplOpenGL3_RenderDrawData(&draw_data);

  std::vector<unsigned char> rgba;
  g_spike_state.readback_ok = lumice::gui::ReadbackGlRegionToRgba(0, 0, kW, kH, rgba);
  g_spike_state.buffer_size = static_cast<int>(rgba.size());

  if (g_spike_state.readback_ok && g_spike_state.buffer_size == kW * kH * 4) {
    auto px = [&](int x, int y, int ch) -> unsigned char {
      return rgba[static_cast<size_t>(y) * kW * 4 + static_cast<size_t>(x) * 4 + ch];
    };
    bool found = false;
    for (int y = 10; y <= 30 && !found; ++y) {
      for (int x = 10; x <= 30 && !found; ++x) {
        if (px(x, y, 0) > 150) {
          found = true;
        }
      }
    }
    g_spike_state.drawn_region_has_red = found;
    g_spike_state.far_r = px(200, 100, 0);
    g_spike_state.far_g = px(200, 100, 1);
    g_spike_state.far_b = px(200, 100, 2);
  }

  glBindFramebuffer(GL_FRAMEBUFFER, prev_fbo);
  glDeleteRenderbuffers(1, &rbo);
  glDeleteFramebuffers(1, &fbo);

  g_spike_state.done = true;
  g_spike_state.requested = false;
}

// Synthetic-texture upload + the real ExportPreviewPng command, both on request.
void ExportGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_export_test.upload_requested && !g_export_test.upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_export_test.upload_done = true;
  }
  if (g_export_test.export_requested && !g_export_test.export_done) {
    g_export_test.export_result =
        gui::ExportPreviewPng(g_export_test.export_path.c_str(), gui::g_preview, gui::g_preview_vp);
    g_export_test.export_done = true;
    g_export_test.export_requested = false;
  }
  if (g_spike_state.requested && !g_spike_state.done) {
    RunSpikeFboImDrawListEndToEnd();
  }
}

// One off-screen render, described by the caller and answered in RGBA.
struct ExportRequest {
  bool requested = false;
  bool done = false;

  // Inputs.
  bool with_overlay = false;
  float exposure_offset = 0.0f;
  int dst_w = 0;
  int dst_h = 0;
  // Which of the two panorama presets to apply on top of the live preview params, or -1 to render
  // the live params untouched (what Screenshot does). The value is a lens id so the cases read the
  // way the menu items do.
  int preset_lens = -1;

  // Outputs.
  bool ok = false;
  std::vector<unsigned char> rgba;
  int rgba_w = 0;
  int rgba_h = 0;

  void Reset() { *this = ExportRequest{}; }
};

ExportRequest g_req;

// Mirrors app.cpp's BuildExportParams: the export's exposure is the SUM of the manual offset and
// the adaptive lane, normalised by the snapshot intensity. Duplicated rather than called because
// the production function reads the live GUI state and these cases sweep the offset independently
// of any slider.
gui::PreviewParams BuildExportParamsFor(float exposure_offset) {
  gui::PreviewParams params = gui::g_preview_vp.params;
  const float ev_total = exposure_offset + gui::g_state.ev_auto;
  params.exposure.intensity_factor = std::pow(2.0f, ev_total);
  const float norm_intensity = gui::g_state.snapshot_intensity;
  params.exposure.intensity_scale = norm_intensity > 0 ? params.exposure.intensity_factor / norm_intensity : 0.0f;
  return params;
}

void RunExportRequest() {
  const int w = g_req.dst_w > 0 ? g_req.dst_w : gui::g_preview_vp.vp_w;
  const int h = g_req.dst_h > 0 ? g_req.dst_h : gui::g_preview_vp.vp_h;

  gui::PreviewParams params = BuildExportParamsFor(g_req.exposure_offset);
  if (g_req.preset_lens == gui::kLensTypeDualFisheyeEqualArea) {
    gui::ConfigureDualFisheyeExportParams(params);
  } else if (g_req.preset_lens == gui::kLensTypeRectangular) {
    gui::ConfigureEquirectExportParams(params);
  }

  // Every label the export draws is now a CurveLabelSet built from core's anchors — the horizon
  // included, since the GUI's own walk is gone. An empty list is how this path says "no overlay".
  std::vector<gui::CurveLabelSet> curve_labels;
  if (g_req.with_overlay) {
    // Turn enough of the overlay on that the labelled path really has something to draw.
    gui::g_state.show_horizon_line = true;
    gui::g_state.show_horizon_label = true;
    gui::g_state.show_grid_line = true;
    gui::g_state.show_grid_label = true;
    // Refresh, not Update: this is a one-shot render with no run of frames to settle over, and the
    // cache is the export's own for the same reason app.cpp's is the preview's.
    gui::AnnotationOverlayCache& cache = gui::PreviewAnnotationOverlay();
    cache.Refresh(gui::MakeAnnotationViewKey(gui::AnnotationViewInputFor(gui::g_state, gui::g_state.renderer), w, h));
    curve_labels.push_back(
        gui::BuildHorizonLabelSet(cache, gui::g_state, static_cast<float>(w), static_cast<float>(h)));
    curve_labels.push_back(gui::BuildGridLabelSet(cache, gui::g_state, static_cast<float>(w), static_cast<float>(h)));
  }

  auto buf = gui::RenderExportToRgba(gui::g_preview, params, w, h, curve_labels);
  g_req.ok = !buf.empty();
  g_req.rgba = std::move(buf);
  g_req.rgba_w = w;
  g_req.rgba_h = h;
  g_req.done = true;
  g_req.requested = false;
}

void ExportRequestGuiFunc(ImGuiTestContext* /*ctx*/) {
  ExportGuiFunc(nullptr);
  if (g_req.requested && !g_req.done) {
    RunExportRequest();
  }
}

// ---------------------------------------------------------------------------------------------
// Comparison.
// ---------------------------------------------------------------------------------------------

// PSNR over the whole RGBA buffer. Identical buffers report a large finite number rather than an
// infinity so the value can be printed and compared without special cases; alpha is included and
// contributes nothing in practice (the FBO's alpha is a constant 1.0).
double ComputePsnrRgba(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
  if (a.empty() || a.size() != b.size()) {
    return -1.0;
  }
  double mse = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const int d = static_cast<int>(a[i]) - static_cast<int>(b[i]);
    mse += static_cast<double>(d * d);
  }
  mse /= static_cast<double>(a.size());
  if (mse == 0.0) {
    return 1e30;
  }
  return 10.0 * std::log10(255.0 * 255.0 / mse);
}

double ComputeMeanLuma(const std::vector<unsigned char>& rgba) {
  if (rgba.empty()) {
    return 0.0;
  }
  double acc = 0.0;
  const size_t px_count = rgba.size() / 4;
  for (size_t i = 0; i < px_count; ++i) {
    const double r = rgba[i * 4 + 0] / 255.0;
    const double g = rgba[i * 4 + 1] / 255.0;
    const double b = rgba[i * 4 + 2] / 255.0;
    acc += (r + g + b) / 3.0;
  }
  return acc / static_cast<double>(px_count);
}

// A pixel field that is not black, so "the image has content" is a statement about the render and
// not about a lucky background colour.
void SeedSynthPreview(ImGuiTestContext* ctx) {
  g_export_test.Reset();
  g_export_test.upload_requested = true;
  ctx->Yield(2);
  IM_CHECK(g_export_test.upload_done);
  // The preview viewport is filled by RenderPreviewPanel, one frame behind the upload.
  ctx->Yield(2);
}

// Output size each menu item asks for. Screenshot takes the viewport; the two panoramas size
// themselves from the source texture, and Equirectangular additionally forces a strict 2:1 —
// mirroring DoExportEquirectangularPng, whose sizing is what a wrong aspect would show up as.
void ResolveExportSize(int preset_lens, int* dst_w, int* dst_h) {
  if (preset_lens == gui::kLensTypeDualFisheyeEqualArea) {
    *dst_w = gui::g_preview.GetTextureWidth();
    *dst_h = gui::g_preview.GetTextureHeight();
    return;
  }
  if (preset_lens == gui::kLensTypeRectangular) {
    const int short_res = std::min(gui::g_preview.GetTextureWidth() / 2, gui::g_preview.GetTextureHeight());
    *dst_w = 2 * short_res;
    *dst_h = short_res;
    return;
  }
  *dst_w = 0;  // 0 = "use the live viewport", resolved in RunExportRequest
  *dst_h = 0;
}

// Render the same description twice and report how far the two results are apart. Shared by the
// three determinism cases, which differ only in which preset they ask for — they stay three
// registrations because a lens whose export drifts should be named in the report, not hidden
// behind whichever one of the three ran first.
double DriveRepeatAndComparePsnr(ImGuiTestContext* ctx, int preset_lens, bool with_overlay) {
  int dst_w = 0;
  int dst_h = 0;
  ResolveExportSize(preset_lens, &dst_w, &dst_h);

  g_req.Reset();
  g_req.preset_lens = preset_lens;
  g_req.with_overlay = with_overlay;
  g_req.dst_w = dst_w;
  g_req.dst_h = dst_h;
  g_req.requested = true;
  ctx->Yield(2);
  if (!g_req.done || !g_req.ok) {
    return -1.0;
  }
  const std::vector<unsigned char> first = std::move(g_req.rgba);

  g_req.Reset();
  g_req.preset_lens = preset_lens;
  g_req.with_overlay = with_overlay;
  g_req.dst_w = dst_w;
  g_req.dst_h = dst_h;
  g_req.requested = true;
  ctx->Yield(2);
  if (!g_req.done || !g_req.ok) {
    return -1.0;
  }
  return ComputePsnrRgba(first, g_req.rgba);
}

}  // namespace

void RegisterExportPreviewTests(ImGuiTestEngine* engine) {
  // -------------------------------------------------------------------------------------------
  // The Screenshot command, end to end through the file system.
  // -------------------------------------------------------------------------------------------

  // One export, and everything a user would check about the file it produced: it exists, a decoder
  // accepts it, it is the size of what was on screen, and it is not blank. These were three
  // registrations asserting three of those four things about three separate exports of the same
  // scene; they are one export and one proposition — "Screenshot saved the preview" — and reading
  // them apart never told anyone anything the merged case does not.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "screenshot_writes_a_readable_png_of_the_preview");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedSynthPreview(ctx);
      IM_CHECK(gui::g_preview.HasTexture());

      const int expected_w = gui::g_preview_vp.vp_w;
      const int expected_h = gui::g_preview_vp.vp_h;
      IM_CHECK(expected_w > 0);
      IM_CHECK(expected_h > 0);

      const std::string tmp_path = GuiTestTempPath("lumice_export_screenshot.png").string();
      // Cleared on the way IN as well as on the way out, because IM_CHECK expands to a return: any
      // assertion below leaves the trailing std::remove unreached, and the next run would then read
      // a file this case did not write. Same self-healing shape as test_file_ops.cpp's.
      std::remove(tmp_path.c_str());
      g_export_test.export_path = tmp_path;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      std::vector<unsigned char> img;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      IM_CHECK(lumice::test::LoadPng(tmp_path.c_str(), img, img_w, img_h, img_ch));
      IM_CHECK_EQ(img_w, expected_w);
      IM_CHECK_EQ(img_h, expected_h);

      bool has_content = false;
      for (size_t i = 0; i < img.size() && !has_content; ++i) {
        has_content = img[i] != 0;
      }
      IM_CHECK(has_content);

      std::remove(tmp_path.c_str());
    };
  }

  // The exporter binds its own framebuffer and has to give the previous binding back. Nothing in
  // the first export's own result shows whether it did — the symptom is the SECOND export, which
  // renders into whatever it inherited.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "a_second_screenshot_succeeds_after_the_first");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedSynthPreview(ctx);

      const std::string path_a = GuiTestTempPath("lumice_export_twice_1.png").string();
      const std::string path_b = GuiTestTempPath("lumice_export_twice_2.png").string();
      // Both cleared up front — see the case above. Here it also removes a way for the case to pass
      // vacuously: comparing two PNGs left behind by an earlier run would agree perfectly.
      std::remove(path_a.c_str());
      std::remove(path_b.c_str());
      g_export_test.export_path = path_a;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      g_export_test.export_done = false;
      g_export_test.export_result = false;
      g_export_test.export_path = path_b;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      std::vector<unsigned char> img_a;
      std::vector<unsigned char> img_b;
      int w_a = 0;
      int h_a = 0;
      int ch_a = 0;
      int w_b = 0;
      int h_b = 0;
      int ch_b = 0;
      IM_CHECK(lumice::test::LoadPng(path_a.c_str(), img_a, w_a, h_a, ch_a));
      IM_CHECK(lumice::test::LoadPng(path_b.c_str(), img_b, w_b, h_b, ch_b));
      IM_CHECK_EQ(w_a, w_b);
      IM_CHECK_EQ(h_a, h_b);

      std::remove(path_a.c_str());
      std::remove(path_b.c_str());
    };
  }

  // The overlay compositing in export_fbo_renderer.cpp builds an ImDrawList it owns and feeds it to
  // ImDrawData::AddDrawList — two ImGui internals with no compile-time contract. This case is the
  // canary for a Dear ImGui upgrade: it draws a red square through that exact path into a
  // framebuffer of its own and reads it back, so a version that changes the semantics fails here
  // with a one-line diagnosis instead of somewhere downstream as "the overlay stopped appearing".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "a_self_owned_drawlist_still_reaches_a_bound_fbo");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_spike_state.Reset();
      g_spike_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_spike_state.done);
      // Before the pixels: this is the one failure that would otherwise be reported as a missing
      // rectangle. Named with its status code so the line itself says which way the FBO was
      // rejected.
      if (g_spike_state.fbo_status != GL_FRAMEBUFFER_COMPLETE) {
        IM_ERRORF(
            "the off-screen framebuffer is not complete (glCheckFramebufferStatus = 0x%04X); "
            "the pixel assertions below would be about a framebuffer that was never usable",
            g_spike_state.fbo_status);
      }
      IM_CHECK(g_spike_state.readback_ok);
      IM_CHECK_EQ(g_spike_state.buffer_size, 256 * 128 * 4);
      IM_CHECK(g_spike_state.drawn_region_has_red);
      // ...and only where it was asked to draw: a far corner stays at the clear colour.
      IM_CHECK_LT(static_cast<int>(g_spike_state.far_r), 10);
      IM_CHECK_LT(static_cast<int>(g_spike_state.far_g), 10);
      IM_CHECK_LT(static_cast<int>(g_spike_state.far_b), 10);
    };
  }

  // -------------------------------------------------------------------------------------------
  // Determinism, one case per menu item.
  //
  // "Export the same thing twice, get the same file" is what makes every other assertion in this
  // suite meaningful — a comparison against a second render is worthless if a second render is
  // allowed to differ. The three menu items get three cases because they configure the projection
  // differently and a failure has to name which one drifted.
  // -------------------------------------------------------------------------------------------

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "screenshot_renders_the_same_pixels_twice");
    t->GuiFunc = ExportRequestGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedSynthPreview(ctx);
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      const double psnr = DriveRepeatAndComparePsnr(ctx, /*preset_lens=*/-1, /*with_overlay=*/false);
      // Same context, same inputs: expect bit-identical. The bar sits at 50 dB rather than at
      // "identical" only to survive a driver that reorders a float sum.
      IM_CHECK(psnr >= 50.0);
    };
  }

  // The overlay arm of the same claim, separate because its tolerance is genuinely different:
  // ImGui rebuilds the font atlas vertex list on every AddText call and the subpixel placement
  // moves slightly between calls. Measured ~33.6 dB on macOS arm64; 30 dB keeps the case well
  // clear of an algorithmic regression (which lands under 20 dB) at ~1.2% of full-scale MSE.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "the_labelled_export_renders_the_same_pixels_twice");
    t->GuiFunc = ExportRequestGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedSynthPreview(ctx);

      const double psnr = DriveRepeatAndComparePsnr(ctx, /*preset_lens=*/-1, /*with_overlay=*/true);
      fprintf(stderr, "[export] labelled determinism PSNR = %.2f dB\n", psnr);
      IM_CHECK(psnr >= 30.0);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_dual_fisheye", "dual_fisheye_renders_the_same_pixels_twice");
    t->GuiFunc = ExportRequestGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedSynthPreview(ctx);
      // The preset sizes itself from the source texture, which the synthetic upload above set.
      IM_CHECK(gui::g_preview.GetTextureWidth() > 0);
      IM_CHECK(gui::g_preview.GetTextureHeight() > 0);

      const double psnr = DriveRepeatAndComparePsnr(ctx, gui::kLensTypeDualFisheyeEqualArea, /*with_overlay=*/false);
      IM_CHECK(psnr >= 50.0);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_equirect", "equirectangular_renders_the_same_pixels_twice");
    t->GuiFunc = ExportRequestGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      SeedSynthPreview(ctx);
      IM_CHECK(gui::g_preview.GetTextureWidth() > 0);
      IM_CHECK(gui::g_preview.GetTextureHeight() > 0);

      const double psnr = DriveRepeatAndComparePsnr(ctx, gui::kLensTypeRectangular, /*with_overlay=*/false);
      IM_CHECK(psnr >= 50.0);
    };
  }

  // -------------------------------------------------------------------------------------------
  // Isolation from the screen.
  // -------------------------------------------------------------------------------------------

  // The export must not see the application's own chrome. This paints a bright rectangle onto the
  // DEFAULT framebuffer over the region the preview occupies, then exports with it up and with it
  // down: the two buffers have to agree, and the corner pixel must not be the colour that was
  // painted. Two assertions rather than one because a whole-image metric can absorb an 80x80 patch
  // and still clear a 40 dB bar, and the direct colour read is what actually says "the toolbar is
  // not in the sky".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "the_export_ignores_what_the_default_framebuffer_shows");
    static bool s_draw_chrome = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      ExportRequestGuiFunc(nullptr);
      if (s_draw_chrome) {
        ImGui::GetForegroundDrawList()->AddRectFilled(ImVec2(0.0f, 0.0f), ImVec2(80.0f, 80.0f),
                                                      IM_COL32(255, 0, 255, 255));
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_draw_chrome = false;
      SeedSynthPreview(ctx);

      g_req.Reset();
      g_req.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);
      const std::vector<unsigned char> clean = std::move(g_req.rgba);

      s_draw_chrome = true;
      ctx->Yield(2);

      g_req.Reset();
      g_req.requested = true;
      ctx->Yield(2);
      s_draw_chrome = false;
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);

      IM_CHECK_EQ(clean.size(), g_req.rgba.size());
      IM_CHECK(ComputePsnrRgba(clean, g_req.rgba) >= 40.0);

      IM_CHECK(g_req.rgba_w >= 10);
      IM_CHECK(g_req.rgba_h >= 10);
      const size_t off = (5 * static_cast<size_t>(g_req.rgba_w) + 5) * 4;
      const int r = g_req.rgba[off + 0];
      const int g = g_req.rgba[off + 1];
      const int b = g_req.rgba[off + 2];
      IM_CHECK(!(r > 220 && g < 40 && b > 220));  // not the magenta that was painted on screen
    };
  }

  // -------------------------------------------------------------------------------------------
  // Exposure.
  //
  // The exported brightness is the sum of two lanes — the manual EV slider and the adaptive
  // ev_auto — and the bug this guards against was the export reading only one of them, so a file
  // saved while adaptive brightness was working came out dark. Two cases: the export follows EV at
  // all, and the two lanes are the same lane. Only the manual lane needs its own "does it get
  // brighter" case; once the two are byte-identical, the auto lane's direction follows.
  //
  // These do not repeat per projection. What would make one lens' exposure differ from another's
  // is a preset overwriting the exposure sub-struct, and that is asserted directly, one case down.
  // -------------------------------------------------------------------------------------------

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "manual_ev_brightens_the_export");
    static bool s_seeded = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      ExportRequestGuiFunc(nullptr);
      if (!s_seeded) {
        UploadUniformXyzTexture(0.08f);
        // snapshot_intensity is the normaliser the shader divides by; the poller writes it from
        // its own thread in production, so it is set here on the render thread beside the upload.
        gui::g_state.snapshot_intensity = 1.0f;
        gui::g_state.p99_raw_y = 0.0f;
        s_seeded = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_req.Reset();
      s_seeded = false;
      ctx->Yield(3);
      IM_CHECK(s_seeded);
      ctx->Yield(2);

      gui::g_state.ev_auto = 0.0f;
      g_req.Reset();
      g_req.exposure_offset = 0.0f;
      g_req.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);
      const double mean_ev0 = ComputeMeanLuma(g_req.rgba);

      g_req.Reset();
      g_req.exposure_offset = 2.0f;
      g_req.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);
      const double mean_ev2 = ComputeMeanLuma(g_req.rgba);

      fprintf(stderr, "[export] manual EV mean %.4f -> %.4f (%.2fx)\n", mean_ev0, mean_ev2,
              mean_ev0 > 0.0 ? mean_ev2 / mean_ev0 : 0.0);

      // A floor first: without it, "brighter" would also be satisfied by two near-black images.
      IM_CHECK(mean_ev0 > 5.0 / 255.0);
      // +2 EV is 4x in linear light; the bar is 1.3x because sRGB encoding and highlight clipping
      // both compress the top of the range and the exact factor is not the claim.
      IM_CHECK(mean_ev2 > 1.3 * mean_ev0);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "the_two_ev_lanes_are_interchangeable");
    static bool s_seeded = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      ExportRequestGuiFunc(nullptr);
      if (!s_seeded) {
        UploadUniformXyzTexture(0.08f);
        gui::g_state.snapshot_intensity = 1.0f;
        gui::g_state.p99_raw_y = 0.0f;
        s_seeded = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_req.Reset();
      s_seeded = false;
      ctx->Yield(3);
      IM_CHECK(s_seeded);
      ctx->Yield(2);

      // +2 EV on the manual slider.
      gui::g_state.ev_auto = 0.0f;
      g_req.Reset();
      g_req.exposure_offset = 2.0f;
      g_req.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);
      const std::vector<unsigned char> manual = std::move(g_req.rgba);

      // The same +2 EV, arriving from the adaptive lane instead.
      gui::g_state.ev_auto = 2.0f;
      g_req.Reset();
      g_req.exposure_offset = 0.0f;
      g_req.requested = true;
      ctx->Yield(2);
      gui::g_state.ev_auto = 0.0f;  // do not leak an exposure into the next case
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);

      IM_CHECK_EQ(manual.size(), g_req.rgba.size());
      const double psnr = ComputePsnrRgba(manual, g_req.rgba);
      fprintf(stderr, "[export] manual-vs-auto EV parity PSNR = %.2f dB\n", psnr);
      IM_CHECK(psnr >= 40.0);
    };
  }

  // -------------------------------------------------------------------------------------------
  // What the panorama presets are allowed to change.
  //
  // The field-by-field half of this claim — a preset overwrites the view and the decorations and
  // inherits everything else, the exposure included — needs no frame and no GL and is asserted in
  // unit-correctness/gui/test_export_params.cpp. It is the premise the compressions above rest on:
  // determinism and exposure are each asserted once and do not repeat per projection, and a preset
  // provably leaving the exposure alone is what makes that sound.
  // -------------------------------------------------------------------------------------------

  // The pixel-layer direction of the same claim, which does need a frame. With the live preview
  // already set to the preset's canonical view, applying the preset is a no-op and the two renders
  // must agree exactly. Do NOT "fix" a failure here by dropping the preset call from the second
  // path: that compares the render to itself and the case stops detecting anything.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_params_decompose", "applying_a_matching_preset_changes_no_pixel");
    t->GuiFunc = ExportRequestGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      gui::g_state.renderer.fov = 180.0f;
      gui::g_state.renderer.elevation = 0.0f;
      gui::g_state.renderer.azimuth = 0.0f;
      gui::g_state.renderer.roll = 0.0f;
      gui::g_state.renderer.visible = gui::kVisibleFull;
      gui::g_state.show_horizon_line = false;
      gui::g_state.show_horizon_label = false;
      gui::g_state.show_grid_line = false;
      gui::g_state.show_grid_label = false;
      gui::g_state.show_sun_circles_line = false;
      gui::g_state.show_sun_circles_label = false;
      gui::g_state.bg_show = false;

      SeedSynthPreview(ctx);
      // One extra frame: the top bar syncs renderer/state into g_preview_vp.params at draw time.
      ctx->Yield(3);
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      g_req.Reset();
      g_req.preset_lens = -1;  // live params, untouched
      g_req.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);
      const std::vector<unsigned char> live = std::move(g_req.rgba);

      g_req.Reset();
      g_req.preset_lens = gui::kLensTypeDualFisheyeEqualArea;  // the preset the menu item applies
      g_req.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_req.done);
      IM_CHECK(g_req.ok);

      IM_CHECK_EQ(live.size(), g_req.rgba.size());
      const double psnr = ComputePsnrRgba(live, g_req.rgba);
      fprintf(stderr, "[export] preview-vs-preset parity PSNR = %.2f dB\n", psnr);
      IM_CHECK(psnr >= 40.0);
    };
  }
}
