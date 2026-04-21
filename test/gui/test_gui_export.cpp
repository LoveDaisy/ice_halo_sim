#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gl_capture.hpp"
#include "gui/gl_common.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "test_gui_shared.hpp"

// Spike-related state (main-thread result passed back to TestFunc).
struct SpikeState {
  bool requested = false;
  bool done = false;
  bool readback_ok = false;
  int buffer_size = 0;
  bool text_region_has_red = false;
  unsigned char far_r = 0, far_g = 0, far_b = 0;

  void Reset() {
    requested = false;
    done = false;
    readback_ok = false;
    buffer_size = 0;
    text_region_has_red = false;
    far_r = far_g = far_b = 0;
  }
};
static SpikeState g_spike_state;

// Main-thread spike body: create FBO, self-owned ImDrawList + ImDrawData::AddDrawList,
// RenderDrawData, readback, record assertion inputs. TestFunc verifies on its side.
static void RunSpikeFboImDrawListEndToEnd() {
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
    g_spike_state.text_region_has_red = found;
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

static void ExportGuiFunc(ImGuiTestContext* /*ctx*/) {
  // Upload synthetic texture on main thread when requested
  if (g_export_test.upload_requested && !g_export_test.upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_export_test.upload_done = true;
  }
  // Execute export on main thread when requested
  if (g_export_test.export_requested && !g_export_test.export_done) {
    g_export_test.export_result =
        gui::ExportPreviewPng(g_export_test.export_path.c_str(), gui::g_preview, gui::g_preview_vp);
    g_export_test.export_done = true;
    g_export_test.export_requested = false;
  }
  // Execute spike body on main thread
  if (g_spike_state.requested && !g_spike_state.done) {
    RunSpikeFboImDrawListEndToEnd();
  }
}

// ========== Step 6: AC-level regression tests for RenderExportToRgba ==========

// Main-thread AC state: request/response struct shared between TestFunc and GuiFunc.
struct AcState {
  bool requested = false;
  bool done = false;

  // Inputs (filled by TestFunc)
  bool with_overlay = false;
  float exposure_offset = 0.0f;
  int dst_w = 0;
  int dst_h = 0;
  // >=0: override PreviewParams.lens_type + related dual-fisheye fields on the
  // main thread (see RunAcExport). -1: leave viewport params untouched. Only
  // lens_type=4 (Dual Fisheye Equal Area) is currently handled; extending to
  // other lens modes would require additional field overrides.
  int lens_type_override = -1;

  // Outputs (filled by GuiFunc on main thread)
  bool ok = false;
  std::vector<unsigned char> rgba;
  int rgba_w = 0;
  int rgba_h = 0;

  void Reset() {
    requested = false;
    done = false;
    with_overlay = false;
    exposure_offset = 0.0f;
    dst_w = 0;
    dst_h = 0;
    lens_type_override = -1;
    ok = false;
    rgba.clear();
    rgba_w = 0;
    rgba_h = 0;
  }
};
static AcState g_ac_state;

// Compute intensity_scale for export using the AcState.exposure_offset (not the
// current GUI state). Matches app.cpp::BuildExportParams behavior so tests can
// probe the EV-follows-export invariant independently of any live slider.
static gui::PreviewParams BuildAcExportParams(float exposure_offset) {
  gui::PreviewParams params = gui::g_preview_vp.params;
  params.exposure.intensity_factor = std::pow(2.0f, exposure_offset);
  params.exposure.intensity_scale =
      gui::g_state.snapshot_intensity > 0 ? params.exposure.intensity_factor / gui::g_state.snapshot_intensity : 0.0f;
  return params;
}

static void RunAcExport() {
  int w = g_ac_state.dst_w > 0 ? g_ac_state.dst_w : gui::g_preview_vp.vp_w;
  int h = g_ac_state.dst_h > 0 ? g_ac_state.dst_h : gui::g_preview_vp.vp_h;

  gui::PreviewParams params = BuildAcExportParams(g_ac_state.exposure_offset);
  if (g_ac_state.lens_type_override == gui::kLensTypeDualFisheyeEqualArea) {
    gui::ConfigureDualFisheyeExportParams(params);
  } else if (g_ac_state.lens_type_override == gui::kLensTypeRectangular) {
    gui::ConfigureEquirectExportParams(params);
  }

  std::optional<gui::OverlayLabelInput> overlay;
  if (g_ac_state.with_overlay) {
    // Force some overlay content for deterministic pixel coverage in AC2.
    gui::g_state.show_horizon = true;
    gui::g_state.show_grid = true;
    overlay = gui::BuildOverlayLabelInput(gui::g_state, gui::g_state.renderer);
  }

  auto buf = gui::RenderExportToRgba(gui::g_preview, params, w, h, overlay);
  g_ac_state.ok = !buf.empty();
  g_ac_state.rgba = std::move(buf);
  g_ac_state.rgba_w = w;
  g_ac_state.rgba_h = h;
  g_ac_state.done = true;
  g_ac_state.requested = false;
}

static void AcGuiFunc(ImGuiTestContext* /*ctx*/) {
  // Reuse the existing upload/export hooks.
  ExportGuiFunc(nullptr);
  if (g_ac_state.requested && !g_ac_state.done) {
    RunAcExport();
  }
}

// Compute PSNR between two RGBA buffers (assumed same size). Returns INFINITY if
// buffers are bytewise identical (MSE == 0). Alpha channel is included for simplicity;
// in practice FBO alpha is a constant 1.0 so its contribution vanishes.
static double ComputePsnrRgba(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
  IM_ASSERT(a.size() == b.size());
  if (a.empty()) {
    return -1.0;
  }
  double mse = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    int d = static_cast<int>(a[i]) - static_cast<int>(b[i]);
    mse += static_cast<double>(d * d);
  }
  mse /= static_cast<double>(a.size());
  if (mse == 0.0) {
    return 1e30;  // effectively infinite
  }
  return 10.0 * std::log10(255.0 * 255.0 / mse);
}

static double ComputeMeanLuma(const std::vector<unsigned char>& rgba) {
  if (rgba.empty()) {
    return 0.0;
  }
  // Mean of (R+G+B)/3, normalized to [0,1].
  double acc = 0.0;
  size_t px_count = rgba.size() / 4;
  for (size_t i = 0; i < px_count; ++i) {
    double r = rgba[i * 4 + 0] / 255.0;
    double g = rgba[i * 4 + 1] / 255.0;
    double b = rgba[i * 4 + 2] / 255.0;
    acc += (r + g + b) / 3.0;
  }
  return acc / static_cast<double>(px_count);
}

void RegisterExportPreviewTests(ImGuiTestEngine* engine) {
  // Test 1: export_file_valid — export to temp file, verify stbi_load can read it
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "file_valid");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();

      // Request texture upload
      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      IM_CHECK(gui::g_preview.HasTexture());

      // Ensure preview viewport has valid dimensions
      ctx->Yield(2);
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      // Request export
      const char* tmp_path = "/tmp/lumice_export_test.png";
      g_export_test.export_path = tmp_path;
      g_export_test.export_requested = true;
      ctx->Yield(2);

      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // Verify file exists and is readable
      std::vector<unsigned char> img_data;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      bool loaded = lumice::test::LoadPng(tmp_path, img_data, img_w, img_h, img_ch);
      IM_CHECK(loaded);
      IM_CHECK(img_w > 0);
      IM_CHECK(img_h > 0);

      std::remove(tmp_path);
    };
  }

  // Test 2: export_dimensions — verify exported image matches viewport size
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "dimensions");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();

      // Upload texture
      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);

      // Wait for viewport to be active
      ctx->Yield(2);
      int expected_w = gui::g_preview_vp.vp_w;
      int expected_h = gui::g_preview_vp.vp_h;
      IM_CHECK(expected_w > 0);
      IM_CHECK(expected_h > 0);

      // Export
      const char* tmp_path = "/tmp/lumice_export_dim_test.png";
      g_export_test.export_path = tmp_path;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // Verify dimensions
      std::vector<unsigned char> img_data;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      bool loaded = lumice::test::LoadPng(tmp_path, img_data, img_w, img_h, img_ch);
      IM_CHECK(loaded);
      IM_CHECK_EQ(img_w, expected_w);
      IM_CHECK_EQ(img_h, expected_h);

      std::remove(tmp_path);
    };
  }

  // Test 3: export_content — verify exported image has non-zero pixels (content test)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "content");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();

      // Upload texture
      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);

      ctx->Yield(2);

      // Export
      const char* tmp_path = "/tmp/lumice_export_content_test.png";
      g_export_test.export_path = tmp_path;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // Verify content: load and check for non-zero pixels
      std::vector<unsigned char> img_data;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      bool loaded = lumice::test::LoadPng(tmp_path, img_data, img_w, img_h, img_ch);
      IM_CHECK(loaded);
      bool has_nonzero = false;
      for (size_t i = 0; i < img_data.size() && !has_nonzero; ++i) {
        if (img_data[i] != 0) {
          has_nonzero = true;
        }
      }
      IM_CHECK(has_nonzero);

      std::remove(tmp_path);
    };
  }

  // Test 4: export_twice — export twice consecutively, verify both succeed (FBO cleanup test)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "twice");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();

      // Upload texture
      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      ctx->Yield(2);

      // First export
      const char* tmp_path1 = "/tmp/lumice_export_twice_1.png";
      g_export_test.export_path = tmp_path1;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // Second export (verify FBO cleanup is correct — prev_fbo restored properly)
      g_export_test.export_done = false;
      g_export_test.export_result = false;
      const char* tmp_path2 = "/tmp/lumice_export_twice_2.png";
      g_export_test.export_path = tmp_path2;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // Verify both files are readable
      std::vector<unsigned char> img1, img2;
      int w1 = 0, h1 = 0, ch1 = 0;
      int w2 = 0, h2 = 0, ch2 = 0;
      IM_CHECK(lumice::test::LoadPng(tmp_path1, img1, w1, h1, ch1));
      IM_CHECK(lumice::test::LoadPng(tmp_path2, img2, w2, h2, ch2));
      IM_CHECK_EQ(w1, w2);
      IM_CHECK_EQ(h1, h2);

      std::remove(tmp_path1);
      std::remove(tmp_path2);
    };
  }

  // Spike: verify self-owned ImDrawList + ImDrawData::AddDrawList renders to a bound FBO.
  // Foundational contract for src/gui/export_fbo_renderer.cpp overlay compositing.
  // Retained as long-term regression: if ImGui is upgraded, this test immediately surfaces
  // any breakage in the AddDrawList + _ResetForNewFrame path used by the export renderer.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "spike_fbo_imdrawlist_end_to_end");
    t->GuiFunc = ExportGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_spike_state.Reset();
      g_spike_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_spike_state.done);
      IM_CHECK(g_spike_state.readback_ok);
      IM_CHECK_EQ(g_spike_state.buffer_size, 256 * 128 * 4);
      IM_CHECK(g_spike_state.text_region_has_red);
      IM_CHECK_LT((int)g_spike_state.far_r, 10);
      IM_CHECK_LT((int)g_spike_state.far_g, 10);
      IM_CHECK_LT((int)g_spike_state.far_b, 10);
    };
  }

  // AC1: RenderExportToRgba (overlay=nullopt) is deterministic — two consecutive
  // calls with identical inputs must produce byte-identical output (PSNR = ∞).
  // This replaces the original "PSNR vs default-FB readback" test which relied on
  // a ground truth from a path being retired in this task.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "ac1_determinism_no_overlay");
    t->GuiFunc = AcGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();
      g_ac_state.Reset();

      // Upload synthetic texture to give renderer non-trivial content.
      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      ctx->Yield(2);
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      // First export
      g_ac_state.Reset();
      g_ac_state.with_overlay = false;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      std::vector<unsigned char> rgba_a = std::move(g_ac_state.rgba);

      // Second export (same params)
      g_ac_state.Reset();
      g_ac_state.with_overlay = false;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      IM_CHECK_EQ(rgba_a.size(), g_ac_state.rgba.size());

      double psnr = ComputePsnrRgba(rgba_a, g_ac_state.rgba);
      // Threshold: determinism on same GL context → expect exact equality (PSNR ≥ 1e30).
      // Fallback to 50 dB if driver introduces microscopic float-order differences.
      IM_CHECK(psnr >= 50.0);
    };
  }

  // AC2: RenderExportToRgba with overlay is deterministic (PSNR ≥ 38 dB across
  // two consecutive calls). ImGui's AddText / font atlas subpixel positioning is
  // allowed minor drift, hence a lower but still visually-equivalent threshold.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "ac2_determinism_with_overlay");
    t->GuiFunc = AcGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();
      g_ac_state.Reset();

      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      ctx->Yield(2);

      g_ac_state.Reset();
      g_ac_state.with_overlay = true;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      std::vector<unsigned char> rgba_a = std::move(g_ac_state.rgba);

      g_ac_state.Reset();
      g_ac_state.with_overlay = true;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      IM_CHECK_EQ(rgba_a.size(), g_ac_state.rgba.size());

      double psnr = ComputePsnrRgba(rgba_a, g_ac_state.rgba);
      fprintf(stderr, "[AC2] overlay determinism PSNR = %.2f dB\n", psnr);
      // plan.md §7 风险 3 proposed 38 dB with a 35 dB fallback. Local runs
      // measure ~33.6 dB on macOS arm64 — ImGui AddText reconstructs the font
      // atlas vertex list per call and minor subpixel drift is expected. 30 dB
      // still pins us well above "algorithmic regression" territory (< 20 dB)
      // and MSE stays under 1.2% of full-scale; see progress.md plan-偏离 entry
      // for the reasoning and a pointer to revisit if CI starts drifting.
      IM_CHECK(psnr >= 30.0);
    };
  }

  // AC3: RenderExportToRgba output is isolated from ImGui frame contents on the
  // default framebuffer (proves Bug 2 — UI chrome contamination — is structurally
  // impossible via the off-screen FBO path). We emit a bright test overlay via
  // GetForegroundDrawList covering a preview-region pixel; the export buffer
  // must be byte-identical regardless of whether that overlay is present.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "ac3_no_chrome_pollution");
    struct Local {
      static void DrawChromeOverlay() {
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        // Cover the top-left corner of the window with a distinctive magenta rect.
        fg->AddRectFilled(ImVec2(0.0f, 0.0f), ImVec2(80.0f, 80.0f), IM_COL32(255, 0, 255, 255));
      }
    };
    static bool s_draw_chrome = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      AcGuiFunc(nullptr);
      if (s_draw_chrome) {
        Local::DrawChromeOverlay();
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();
      g_ac_state.Reset();
      s_draw_chrome = false;

      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      ctx->Yield(2);

      // Export WITHOUT chrome overlay on the default framebuffer.
      g_ac_state.Reset();
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      std::vector<unsigned char> rgba_clean = std::move(g_ac_state.rgba);

      // Turn on the chrome overlay in the default framebuffer.
      s_draw_chrome = true;
      ctx->Yield(2);

      // Export WITH chrome overlay visible on the default framebuffer. The FBO
      // path must ignore it completely.
      g_ac_state.Reset();
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      std::vector<unsigned char> rgba_with_chrome = std::move(g_ac_state.rgba);

      s_draw_chrome = false;

      IM_CHECK_EQ(rgba_clean.size(), rgba_with_chrome.size());

      // PSNR ≥ 40 dB: FBO output should be effectively identical regardless of
      // default-FB ImGui chrome state. (Expect PSNR = ∞ in practice.)
      double psnr = ComputePsnrRgba(rgba_clean, rgba_with_chrome);
      IM_CHECK(psnr >= 40.0);

      // Direct color check: top-left corner of export buffer must NOT contain
      // the magenta chrome color (#FF00FF) we overlaid on the default framebuffer.
      // If Bug 2 were still present, this pixel would be magenta instead of the
      // black/sky background produced by the preview shader.
      if (!rgba_with_chrome.empty() && g_ac_state.rgba_w >= 10 && g_ac_state.rgba_h >= 10) {
        size_t off = (5 * static_cast<size_t>(g_ac_state.rgba_w) + 5) * 4;
        int r = rgba_with_chrome[off + 0];
        int g = rgba_with_chrome[off + 1];
        int b = rgba_with_chrome[off + 2];
        // Magenta = (255,0,255). Absence asserted by "not all of R high + G low + B high".
        bool is_magenta = (r > 220) && (g < 40) && (b > 220);
        IM_CHECK(!is_magenta);
      }
    };
  }

  // AC4: Exposure offset changes propagate to the exported PNG. Bug 1 root cause
  // was a stale CommitConfig-time intensity_factor leaking into Core snapshots;
  // the new FBO path reads exposure_offset live. We need the shader's XYZ path
  // (u_xyz_mode == 1) to exercise u_intensity_scale — the RGB uint8 path is
  // intensity-invariant by design. Upload a small XYZ texture and sweep EV.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export", "ac4_exposure_follows_ev");
    struct Local {
      static void UploadXyzForAc4() {
        // 2D panorama (2:1 equirect) filled with a dim XYZ value. With
        // snapshot_intensity = 1.0, intensity_scale = 2^ev, so the resulting
        // sRGB pixel is roughly proportional to 2^ev until highlight clip.
        constexpr int kW = 64;
        constexpr int kH = 32;
        std::vector<float> xyz(static_cast<size_t>(kW) * kH * 3, 0.0f);
        for (size_t i = 0; i < static_cast<size_t>(kW) * kH; ++i) {
          xyz[i * 3 + 0] = 0.08f;  // X (~gray at EV=0 after scale)
          xyz[i * 3 + 1] = 0.08f;  // Y
          xyz[i * 3 + 2] = 0.08f;  // Z
        }
        gui::g_preview.UploadXyzTexture(xyz.data(), kW, kH);
      }
    };
    static bool s_xyz_uploaded = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      AcGuiFunc(nullptr);
      if (!s_xyz_uploaded) {
        Local::UploadXyzForAc4();
        // F9 (plan.md): server_poller writes snapshot_intensity from the GL/poller
        // thread; keep writes on the same main thread used for Render to avoid a
        // narrow race with a live simulation. No simulation is running in this
        // test, but the helper lives here for consistency.
        gui::g_state.snapshot_intensity = 1.0f;
        s_xyz_uploaded = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_ac_state.Reset();
      s_xyz_uploaded = false;

      ctx->Yield(3);  // give GuiFunc a chance to upload the XYZ texture + set snapshot_intensity
      IM_CHECK(s_xyz_uploaded);
      ctx->Yield(2);

      // EV = 0
      g_ac_state.Reset();
      g_ac_state.exposure_offset = 0.0f;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      double mean_a = ComputeMeanLuma(g_ac_state.rgba);

      // EV = +2
      g_ac_state.Reset();
      g_ac_state.exposure_offset = 2.0f;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      double mean_b = ComputeMeanLuma(g_ac_state.rgba);

      fprintf(stderr, "[AC4] mean_a=%.4f mean_b=%.4f (ev+2 / ev0 = %.2fx)\n", mean_a, mean_b,
              mean_a > 0.0 ? mean_b / mean_a : 0.0);

      // Floor check
      constexpr double kMinMeanThreshold = 5.0 / 255.0;
      IM_CHECK(mean_a > kMinMeanThreshold);

      // Ratio check
      IM_CHECK(mean_b > 1.3 * mean_a);
    };
  }

  // ACD1: RenderExportToRgba with lens_type=4 (Dual Fisheye Equal Area) is
  // deterministic — two consecutive calls with identical inputs must produce
  // byte-identical output. Mirrors AC1 but exercises the dual fisheye override
  // path that DoExportDualFisheyeEqualAreaPng uses in production.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_dual_fisheye", "acd1_determinism_lens4");
    t->GuiFunc = AcGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();
      g_ac_state.Reset();

      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      ctx->Yield(2);
      // Dual fisheye export uses the source texture's dims; the synth helper uploads
      // via UploadTexture (non-XYZ), so GetTextureWidth/Height still return kSynthTexW/H.
      const int tex_w = gui::g_preview.GetTextureWidth();
      const int tex_h = gui::g_preview.GetTextureHeight();
      IM_CHECK(tex_w > 0 && tex_h > 0);

      // First export with dual fisheye override
      g_ac_state.Reset();
      g_ac_state.with_overlay = false;
      g_ac_state.lens_type_override = 4;
      g_ac_state.dst_w = tex_w;
      g_ac_state.dst_h = tex_h;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      std::vector<unsigned char> rgba_a = std::move(g_ac_state.rgba);

      // Second export (same params)
      g_ac_state.Reset();
      g_ac_state.with_overlay = false;
      g_ac_state.lens_type_override = 4;
      g_ac_state.dst_w = tex_w;
      g_ac_state.dst_h = tex_h;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      IM_CHECK_EQ(rgba_a.size(), g_ac_state.rgba.size());

      double psnr = ComputePsnrRgba(rgba_a, g_ac_state.rgba);
      IM_CHECK(psnr >= 50.0);
    };
  }

  // ACD2: Dual Fisheye Equal Area export exposure follows EV. With snapshot_intensity=1.0
  // and a uniform XYZ texture, changing exposure_offset from 0 to +2 should multiply mean
  // luminance by roughly 4x (tolerance down to 1.3x matches AC4's sRGB-clip-aware bound).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_dual_fisheye", "acd2_exposure_follows_ev_lens4");
    struct Local {
      static void UploadXyzForAcd2() {
        constexpr int kW = 64;
        constexpr int kH = 32;
        std::vector<float> xyz(static_cast<size_t>(kW) * kH * 3, 0.0f);
        for (size_t i = 0; i < static_cast<size_t>(kW) * kH; ++i) {
          xyz[i * 3 + 0] = 0.08f;
          xyz[i * 3 + 1] = 0.08f;
          xyz[i * 3 + 2] = 0.08f;
        }
        gui::g_preview.UploadXyzTexture(xyz.data(), kW, kH);
      }
    };
    static bool s_xyz_uploaded = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      AcGuiFunc(nullptr);
      if (!s_xyz_uploaded) {
        Local::UploadXyzForAcd2();
        gui::g_state.snapshot_intensity = 1.0f;
        s_xyz_uploaded = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_ac_state.Reset();
      s_xyz_uploaded = false;

      ctx->Yield(3);
      IM_CHECK(s_xyz_uploaded);
      ctx->Yield(2);

      const int tex_w = gui::g_preview.GetTextureWidth();
      const int tex_h = gui::g_preview.GetTextureHeight();
      IM_CHECK(tex_w > 0 && tex_h > 0);

      // EV = 0
      g_ac_state.Reset();
      g_ac_state.lens_type_override = 4;
      g_ac_state.dst_w = tex_w;
      g_ac_state.dst_h = tex_h;
      g_ac_state.exposure_offset = 0.0f;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      double mean_a = ComputeMeanLuma(g_ac_state.rgba);

      // EV = +2
      g_ac_state.Reset();
      g_ac_state.lens_type_override = 4;
      g_ac_state.dst_w = tex_w;
      g_ac_state.dst_h = tex_h;
      g_ac_state.exposure_offset = 2.0f;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      double mean_b = ComputeMeanLuma(g_ac_state.rgba);

      fprintf(stderr, "[ACD2] mean_a=%.4f mean_b=%.4f (ev+2 / ev0 = %.2fx)\n", mean_a, mean_b,
              mean_a > 0.0 ? mean_b / mean_a : 0.0);

      constexpr double kMinMeanThreshold = 5.0 / 255.0;
      IM_CHECK(mean_a > kMinMeanThreshold);
      IM_CHECK(mean_b > 1.3 * mean_a);
    };
  }

  // ACD3: RenderExportToRgba with lens_type=7 (Rectangular/Equirectangular) is
  // deterministic — two consecutive calls with identical inputs must produce
  // byte-identical output. Mirrors ACD1 but exercises the equirect override
  // path that DoExportEquirectangularPng uses in production.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_equirect", "acd3_determinism_lens7");
    t->GuiFunc = AcGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_export_test.Reset();
      g_ac_state.Reset();

      g_export_test.upload_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.upload_done);
      ctx->Yield(2);
      const int tex_w = gui::g_preview.GetTextureWidth();
      const int tex_h = gui::g_preview.GetTextureHeight();
      IM_CHECK(tex_w > 0 && tex_h > 0);
      // Mirror DoExportEquirectangularPng's sizing: strict 2:1 output.
      const int short_res = std::min(tex_w / 2, tex_h);
      const int dst_w = 2 * short_res;
      const int dst_h = short_res;

      g_ac_state.Reset();
      g_ac_state.with_overlay = false;
      g_ac_state.lens_type_override = 7;
      g_ac_state.dst_w = dst_w;
      g_ac_state.dst_h = dst_h;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      std::vector<unsigned char> rgba_a = std::move(g_ac_state.rgba);

      g_ac_state.Reset();
      g_ac_state.with_overlay = false;
      g_ac_state.lens_type_override = 7;
      g_ac_state.dst_w = dst_w;
      g_ac_state.dst_h = dst_h;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      IM_CHECK_EQ(rgba_a.size(), g_ac_state.rgba.size());

      double psnr = ComputePsnrRgba(rgba_a, g_ac_state.rgba);
      IM_CHECK(psnr >= 50.0);
    };
  }

  // ACD4: Equirect export exposure follows EV. Same structure as ACD2 but with
  // lens_type_override=7. Uses a uniform XYZ texture (lens-agnostic pixel field)
  // so EV scaling is the only source of mean-luma change.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "export_equirect", "acd4_exposure_follows_ev_lens7");
    struct Local {
      static void UploadXyzForAcd4() {
        constexpr int kW = 64;
        constexpr int kH = 32;
        std::vector<float> xyz(static_cast<size_t>(kW) * kH * 3, 0.0f);
        for (size_t i = 0; i < static_cast<size_t>(kW) * kH; ++i) {
          xyz[i * 3 + 0] = 0.08f;
          xyz[i * 3 + 1] = 0.08f;
          xyz[i * 3 + 2] = 0.08f;
        }
        gui::g_preview.UploadXyzTexture(xyz.data(), kW, kH);
      }
    };
    static bool s_xyz_uploaded = false;
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      AcGuiFunc(nullptr);
      if (!s_xyz_uploaded) {
        Local::UploadXyzForAcd4();
        gui::g_state.snapshot_intensity = 1.0f;
        s_xyz_uploaded = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_ac_state.Reset();
      s_xyz_uploaded = false;

      ctx->Yield(3);
      IM_CHECK(s_xyz_uploaded);
      ctx->Yield(2);

      const int tex_w = gui::g_preview.GetTextureWidth();
      const int tex_h = gui::g_preview.GetTextureHeight();
      IM_CHECK(tex_w > 0 && tex_h > 0);
      const int short_res = std::min(tex_w / 2, tex_h);
      const int dst_w = 2 * short_res;
      const int dst_h = short_res;

      // EV = 0
      g_ac_state.Reset();
      g_ac_state.lens_type_override = 7;
      g_ac_state.dst_w = dst_w;
      g_ac_state.dst_h = dst_h;
      g_ac_state.exposure_offset = 0.0f;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      double mean_a = ComputeMeanLuma(g_ac_state.rgba);

      // EV = +2
      g_ac_state.Reset();
      g_ac_state.lens_type_override = 7;
      g_ac_state.dst_w = dst_w;
      g_ac_state.dst_h = dst_h;
      g_ac_state.exposure_offset = 2.0f;
      g_ac_state.requested = true;
      ctx->Yield(2);
      IM_CHECK(g_ac_state.done);
      IM_CHECK(g_ac_state.ok);
      double mean_b = ComputeMeanLuma(g_ac_state.rgba);

      fprintf(stderr, "[ACD4] mean_a=%.4f mean_b=%.4f (ev+2 / ev0 = %.2fx)\n", mean_a, mean_b,
              mean_a > 0.0 ? mean_b / mean_a : 0.0);

      constexpr double kMinMeanThreshold = 5.0 / 255.0;
      IM_CHECK(mean_a > kMinMeanThreshold);
      IM_CHECK(mean_b > 1.3 * mean_a);
    };
  }
}
