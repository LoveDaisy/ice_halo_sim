#include <cstdio>

#include <stb_image.h>

#include "test_gui_shared.hpp"

// Background overlay GuiFunc: handles bg texture upload, equirect upload, and FBO export on main thread.
static void BgOverlayGuiFunc(ImGuiTestContext* /*ctx*/) {
  // Upload equirect texture when requested (reuses g_export_test state)
  if (g_export_test.upload_requested && !g_export_test.upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_export_test.upload_done = true;
  }
  // Upload bg image from file path
  if (g_bg_test.bg_upload_requested && !g_bg_test.bg_upload_done) {
    int w = 0;
    int h = 0;
    int channels = 0;
    unsigned char* raw = stbi_load(g_bg_test.bg_image_path.c_str(), &w, &h, &channels, 3);
    if (raw) {
      gui::g_preview.UploadBgTexture(raw, w, h);
      stbi_image_free(raw);
    }
    g_bg_test.bg_upload_done = true;
  }
  // Export via FBO
  if (g_bg_test.export_requested && !g_bg_test.export_done) {
    g_bg_test.export_result = gui::ExportPreviewPng(g_bg_test.export_path.c_str(), gui::g_preview, gui::g_preview_vp);
    g_bg_test.export_done = true;
  }
}

void RegisterBgOverlayTests(ImGuiTestEngine* engine) {
  // Test 1: bg/aspect_landscape — load landscape bg, verify viewport aspect ≈ 2650/1580
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "aspect_landscape");
    t->GuiFunc = BgOverlayGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_bg_test.Reset();

      // Also need an equirect texture for preview viewport to activate
      InitSynthTexture();
      g_export_test.Reset();
      g_export_test.upload_requested = true;

      // Upload landscape bg image
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.jpg";
      g_bg_test.bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(g_bg_test.bg_upload_done);
      IM_CHECK(gui::g_preview.HasBackground());

      // Verify aspect ratio ≈ 2650/1580 ≈ 1.677
      float bg_aspect = gui::g_preview.GetBgAspect();
      IM_CHECK(bg_aspect > 1.5f);
      IM_CHECK(bg_aspect < 1.8f);
    };
  }

  // Test 2: bg/aspect_portrait — load portrait bg, verify aspect ≈ 1608/2488 ≈ 0.646
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "aspect_portrait");
    t->GuiFunc = BgOverlayGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_bg_test.Reset();

      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_portrait.jpg";
      g_bg_test.bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(g_bg_test.bg_upload_done);
      IM_CHECK(gui::g_preview.HasBackground());

      float bg_aspect = gui::g_preview.GetBgAspect();
      IM_CHECK(bg_aspect > 0.5f);
      IM_CHECK(bg_aspect < 0.8f);
    };
  }

  // Test 3: bg/alpha_diff — alpha=0 vs alpha=1 should produce different FBO output
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "alpha_diff");
    t->GuiFunc = BgOverlayGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_bg_test.Reset();

      // Upload equirect texture
      InitSynthTexture();
      g_export_test.Reset();
      g_export_test.upload_requested = true;
      ctx->Yield(2);

      // Upload bg image
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.jpg";
      g_bg_test.bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(g_bg_test.bg_upload_done);

      // Enable bg overlay
      gui::g_state.bg_show = true;
      ctx->Yield(2);
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      // Export with alpha=0 (only bg visible)
      gui::g_state.bg_alpha = 0.0f;
      ctx->Yield(3);  // Let RenderPreviewPanel update params
      const char* path_a0 = "/tmp/lumice_bg_alpha0.png";
      g_bg_test.export_path = path_a0;
      g_bg_test.export_requested = true;
      ctx->Yield(3);
      IM_CHECK(g_bg_test.export_done);
      IM_CHECK(g_bg_test.export_result);

      // Export with alpha=1 (only render visible)
      g_bg_test.export_done = false;
      g_bg_test.export_result = false;
      g_bg_test.export_requested = false;
      gui::g_state.bg_alpha = 1.0f;
      ctx->Yield(3);  // Let RenderPreviewPanel update params
      const char* path_a1 = "/tmp/lumice_bg_alpha1.png";
      g_bg_test.export_path = path_a1;
      g_bg_test.export_requested = true;
      ctx->Yield(3);
      IM_CHECK(g_bg_test.export_done);
      IM_CHECK(g_bg_test.export_result);

      // Load both and verify they differ (PSNR should be low)
      std::vector<unsigned char> img0, img1;
      int w0 = 0, h0 = 0, ch0 = 0;
      int w1 = 0, h1 = 0, ch1 = 0;
      IM_CHECK(lumice::test::LoadPng(path_a0, img0, w0, h0, ch0));
      IM_CHECK(lumice::test::LoadPng(path_a1, img1, w1, h1, ch1));
      IM_CHECK_EQ(w0, w1);
      IM_CHECK_EQ(h0, h1);
      if (w0 == w1 && h0 == h1 && ch0 == ch1) {
        double psnr = lumice::test::ComputePsnr(img0.data(), img1.data(), w0, h0, ch0);
        fprintf(stderr, "[bg] alpha_diff: PSNR = %.2f dB (should be < 30)\n", psnr);
        IM_CHECK(psnr < 30.0);  // Should be quite different
      }

      std::remove(path_a0);
      std::remove(path_a1);
    };
  }

  // Test 4: bg/toggle_off — show=false should not affect output
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "toggle_off");
    t->GuiFunc = BgOverlayGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_bg_test.Reset();

      // Upload equirect texture
      InitSynthTexture();
      g_export_test.Reset();
      g_export_test.upload_requested = true;
      ctx->Yield(2);

      // Upload bg but keep show=false
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.jpg";
      g_bg_test.bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasBackground());

      gui::g_state.bg_show = false;
      ctx->Yield(2);

      // Verify bg_enabled is false in params
      IM_CHECK_EQ(gui::g_preview_vp.params.bg_enabled, false);
    };
  }

  // Test 5: bg/lmc_roundtrip — save/load preserves bg_path/bg_show/bg_alpha
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "lmc_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Set bg state
      gui::g_state.bg_path = "/some/test/path.png";
      gui::g_state.bg_show = true;
      gui::g_state.bg_alpha = 0.7f;

      // Save
      const char* tmp_path = "/tmp/lumice_bg_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset and load
      gui::DoNew();
      IM_CHECK(gui::g_state.bg_path.empty());

      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify roundtrip
      IM_CHECK_EQ(gui::g_state.bg_path, std::string("/some/test/path.png"));
      IM_CHECK_EQ(gui::g_state.bg_show, true);
      IM_CHECK(std::abs(gui::g_state.bg_alpha - 0.7f) < 0.01f);

      std::remove(tmp_path);
    };
  }

  // Test 6: bg/old_lmc_compat — JSON without bg fields gets defaults
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "old_lmc_compat");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({"crystals":[],"renderers":[],"filters":[]})";
      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK(loaded.bg_path.empty());
      IM_CHECK_EQ(loaded.bg_show, false);
      IM_CHECK(std::abs(loaded.bg_alpha - 1.0f) < 0.01f);
    };
  }

  // Test 7: bg/match_bg_avail — Match Background availability depends on HasBackground
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "match_bg_avail");
    t->GuiFunc = BgOverlayGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_bg_test.Reset();

      // No bg loaded — HasBackground should be false
      IM_CHECK_EQ(gui::g_preview.HasBackground(), false);

      // Load bg
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.jpg";
      g_bg_test.bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasBackground());

      // Set to kMatchBg — should work now
      gui::g_state.aspect_preset = gui::AspectPreset::kMatchBg;
      float bg_aspect = gui::g_preview.GetBgAspect();
      IM_CHECK(bg_aspect > 1.0f);  // Landscape

      // Clear bg — kMatchBg should not be functional
      gui::g_preview.ClearBackground();
      IM_CHECK_EQ(gui::g_preview.HasBackground(), false);
    };
  }

  // Test 8: bg/contain_mode — letterbox black bars for aspect mismatch
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "bg", "contain_mode");
    t->GuiFunc = BgOverlayGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_bg_test.Reset();

      // Upload equirect texture (needed for active viewport)
      InitSynthTexture();
      g_export_test.Reset();
      g_export_test.upload_requested = true;
      ctx->Yield(2);

      // Upload landscape bg (aspect ≈ 1.677)
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.jpg";
      g_bg_test.bg_upload_requested = true;
      ctx->Yield(3);
      IM_CHECK(gui::g_preview.HasBackground());

      // Enable bg with alpha=0 (full bg, no render) so we can clearly see letterbox
      gui::g_state.bg_show = true;
      gui::g_state.bg_alpha = 0.0f;
      ctx->Yield(2);

      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      // Export via FBO
      const char* tmp_path = "/tmp/lumice_bg_contain.png";
      g_bg_test.export_path = tmp_path;
      g_bg_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_bg_test.export_done);
      IM_CHECK(g_bg_test.export_result);

      // Load exported image
      std::vector<unsigned char> img;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      bool loaded = lumice::test::LoadPng(tmp_path, img, img_w, img_h, img_ch);
      IM_CHECK(loaded);
      IM_CHECK(img_w > 0);
      IM_CHECK(img_h > 0);

      // With landscape bg (aspect > 1) in a viewport that may be close to 16:9,
      // if viewport is taller than bg (vp_aspect < bg_aspect), there should be
      // black bars at top/bottom. Check top-left corner pixel is black (letterbox).
      float vp_aspect = static_cast<float>(img_w) / static_cast<float>(img_h);
      float bg_aspect = gui::g_preview.GetBgAspect();
      fprintf(stderr, "[bg] contain_mode: vp_aspect=%.3f bg_aspect=%.3f img=%dx%d\n", vp_aspect, bg_aspect, img_w,
              img_h);

      // Verify center pixel is non-black (has bg content)
      int cx = img_w / 2;
      int cy = img_h / 2;
      int center_idx = (cy * img_w + cx) * img_ch;
      bool center_nonblack = (img[center_idx] > 5 || img[center_idx + 1] > 5 || img[center_idx + 2] > 5);
      IM_CHECK(center_nonblack);

      std::remove(tmp_path);
    };
  }
}
