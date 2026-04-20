#include <chrono>
#include <cstdio>
#include <cstring>

#include "test_gui_shared.hpp"

// Rebuild the crystal mesh (if changed) and render it into g_crystal_renderer's
// FBO so tests can capture from it. Test-only helper; the modal guard mirrors
// the previous in-panel logic so the edit modal's FBO content is not overwritten.
static void DriveCrystalPreviewFboForTest() {
  if (gui::IsEditModalCrystalTabActive()) {
    return;
  }
  // Visual smoke tests configure entry [0][0] only. If a multi-entry visual test
  // is later added, change this helper to accept layer/entry indices.
  if (gui::g_state.layers.empty() || gui::g_state.layers[0].entries.empty()) {
    return;
  }
  const auto& cr = gui::g_state.layers[0].entries[0].crystal;
  int hash = gui::CrystalParamHash(cr);
  if (hash != gui::g_crystal_mesh_hash) {
    int result = gui::BuildAndUploadCrystalMesh(cr);
    if (result != 0) {
      gui::g_crystal_mesh_hash = hash;
    }
  }
  auto style = static_cast<gui::CrystalStyle>(gui::g_crystal_style);
  gui::g_crystal_renderer.Render(gui::g_crystal_rotation, gui::g_crystal_zoom, style);
}

// GuiFunc that renders GUI and captures crystal texture when requested.
//
// Note: the left panel no longer updates g_crystal_renderer's FBO every frame
// (task-remove-bottom-preview). Tests using this GuiFunc drive the FBO here
// on the selected entry's crystal before sampling the texture.
static void ScreenshotGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_capture.capture_requested && !g_capture.capture_done) {
    DriveCrystalPreviewFboForTest();
    int w = gui::g_crystal_renderer.Width();
    int h = gui::g_crystal_renderer.Height();
    auto tex_id = static_cast<unsigned int>(gui::g_crystal_renderer.GetTextureId());
    g_capture.pixels = lumice::test::ReadTexturePixels(tex_id, w, h);
    g_capture.width = w;
    g_capture.height = h;
    g_capture.capture_done = true;
  }
}

// Screenshot tests
void RegisterScreenshotTests(ImGuiTestEngine* engine) {
  // Smoke: capture crystal preview, verify non-black
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "screenshot", "smoke");
    t->GuiFunc = ScreenshotGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_capture.Reset();

      // Crystal tab is the default first tab — no need to switch.
      // Yield 2 frames: frame 1 triggers UpdateMesh (g_crystal_mesh_id == -1) + FBO render,
      // frame 2 ensures resolve is complete.
      ctx->Yield(2);

      // Request capture on main thread
      g_capture.capture_requested = true;
      // Yield to let GuiFunc run and perform the capture
      ctx->Yield(2);

      IM_CHECK(g_capture.capture_done);
      IM_CHECK_EQ(static_cast<int>(g_capture.pixels.size()), g_capture.width * g_capture.height * 4);

      // Save to temp file
      const char* tmp_path = "/tmp/lumice_crystal_test.png";
      bool saved = lumice::test::SavePng(tmp_path, g_capture.pixels.data(), g_capture.width, g_capture.height, 4);
      IM_CHECK(saved);

      // Verify not all black: at least some non-zero pixels
      bool has_nonzero = false;
      for (size_t i = 0; i < g_capture.pixels.size() && !has_nonzero; ++i) {
        if (g_capture.pixels[i] != 0) {
          has_nonzero = true;
        }
      }
      IM_CHECK(has_nonzero);

      // Verify file is non-empty
      FILE* f = fopen(tmp_path, "rb");
      IM_CHECK(f != nullptr);
      if (f) {
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        fclose(f);
        IM_CHECK(file_size > 0);
      }

      // Cleanup
      std::remove(tmp_path);
    };
  }

  // PSNR: capture crystal preview and compare against reference image
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "screenshot", "crystal_psnr");
    t->GuiFunc = ScreenshotGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_capture.Reset();

      // Crystal tab is default — yield 2 frames for mesh update + FBO render
      ctx->Yield(2);

      // Request capture on main thread
      g_capture.capture_requested = true;
      ctx->Yield(2);

      IM_CHECK(g_capture.capture_done);

      // Load reference image
      const char* ref_path = LUMICE_TEST_REF_DIR "/crystal_prism_default.png";
      std::vector<unsigned char> ref_data;
      int ref_w = 0;
      int ref_h = 0;
      int ref_ch = 0;
      bool loaded = lumice::test::LoadPng(ref_path, ref_data, ref_w, ref_h, ref_ch);
      IM_CHECK(loaded);
      IM_CHECK_EQ(ref_w, g_capture.width);
      IM_CHECK_EQ(ref_h, g_capture.height);

      // Convert capture to match reference channels if needed (RGBA→RGB or vice versa)
      const unsigned char* cmp_data = g_capture.pixels.data();
      std::vector<unsigned char> converted;
      int cmp_channels = 4;  // capture is always RGBA
      if (ref_ch == 4) {
        cmp_channels = 4;
      } else if (ref_ch == 3) {
        // Strip alpha channel for comparison
        converted = lumice::test::StripAlpha(g_capture.pixels.data(), g_capture.width, g_capture.height);
        cmp_data = converted.data();
        cmp_channels = 3;
      }

      // Same-platform same-hardware should produce near-identical results
      constexpr double kPsnrThreshold = 40.0;  // dB; deterministic GL render should be >50 dB
      double psnr = lumice::test::ComputePsnr(cmp_data, ref_data.data(), ref_w, ref_h, cmp_channels);
      IM_CHECK(psnr >= 0.0);
      fprintf(stderr, "[screenshot] crystal_psnr: PSNR = %.2f dB (threshold = %.1f dB)\n", psnr, kPsnrThreshold);
      IM_CHECK(psnr > kPsnrThreshold);
    };
  }

  // PSNR: left-panel capture vs reference (ImGui default-framebuffer readback)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "screenshot", "left_panel_psnr");
    // No GuiFunc: capture happens in main loop's post-RenderDrawData hook.
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_left_panel_capture.Reset();

      // Eliminate hover state: move cursor off-screen so no card is highlighted.
      // Otherwise reference image fixes a hover frame and subsequent no-hover runs
      // produce spurious PSNR failures.
      ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
      ctx->Yield(3);

      g_left_panel_capture.requested.store(true);

      // Poll up to 10 frames for the main-loop hook to complete readback.
      for (int i = 0; i < 10 && !g_left_panel_capture.done.load(); ++i) {
        ctx->Yield(1);
      }
      IM_CHECK(g_left_panel_capture.done.load());

      // "Non-zero" gate before PSNR: detects silent failure (helper returned zeros,
      // or timing window missed ImGui draw) that would otherwise pass a black-image PSNR.
      bool has_nonzero = false;
      for (size_t i = 0; i < g_left_panel_capture.pixels.size() && !has_nonzero; ++i) {
        if (g_left_panel_capture.pixels[i] != 0) {
          has_nonzero = true;
        }
      }
      IM_CHECK(has_nonzero);

      fprintf(stderr, "[screenshot] left_panel_psnr: captured size = %dx%d\n", g_left_panel_capture.width,
              g_left_panel_capture.height);

      // Save to tmp for reference-image generation (uncomment std::remove below to keep).
      const char* tmp_path = "/tmp/lumice_left_panel_test.png";
      auto rgb = lumice::test::StripAlpha(g_left_panel_capture.pixels.data(), g_left_panel_capture.width,
                                          g_left_panel_capture.height);
      lumice::test::SavePng(tmp_path, rgb.data(), g_left_panel_capture.width, g_left_panel_capture.height, 3);

      const char* ref_path = LUMICE_TEST_REF_DIR "/left_panel_default.png";
      std::vector<unsigned char> ref_data;
      int ref_w = 0;
      int ref_h = 0;
      int ref_ch = 0;
      bool loaded = lumice::test::LoadPng(ref_path, ref_data, ref_w, ref_h, ref_ch);
      if (!loaded) {
        fprintf(stderr, "[screenshot] left_panel_psnr: reference not found at %s\n", ref_path);
        fprintf(stderr, "[screenshot] Run test once, then copy %s to %s\n", tmp_path, ref_path);
        IM_CHECK(loaded);
        return;
      }
      IM_CHECK_EQ(ref_w, g_left_panel_capture.width);
      IM_CHECK_EQ(ref_h, g_left_panel_capture.height);

      constexpr double kPsnrThreshold = 40.0;
      double psnr = lumice::test::ComputePsnr(rgb.data(), ref_data.data(), ref_w, ref_h, ref_ch);
      IM_CHECK(psnr >= 0.0);
      fprintf(stderr, "[screenshot] left_panel_psnr: PSNR = %.2f dB (threshold = %.1f dB)\n", psnr, kPsnrThreshold);
      IM_CHECK(psnr > kPsnrThreshold);

      std::remove(tmp_path);
    };
  }
}

// Visual tests: crystal preview + render preview data correctness
void RegisterVisualTests(ImGuiTestEngine* engine) {
  // Crystal preview: Pyramid (HiddenLine, default style after ResetTestState)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "crystal_pyramid");
    t->GuiFunc = ScreenshotGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Left panel now renders cards directly (no tabs)
      ctx->Yield();

      // Switch to Pyramid type
      if (!gui::g_state.layers.empty() && !gui::g_state.layers[0].entries.empty()) {
        gui::g_state.layers[0].entries[0].crystal.type = gui::CrystalType::kPyramid;
      }
      // upper_h/lower_h default to 0.2 after Step 1 change
      gui::g_crystal_mesh_hash = -1;  // Force mesh rebuild

      // Yield 3 frames: state change → mesh rebuild → FBO render
      ctx->Yield(3);

      // Capture
      g_capture.Reset();
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);

      // Save to /tmp for reference image generation
      const char* tmp_path = "/tmp/lumice_visual_crystal_pyramid.png";
      auto rgb = lumice::test::StripAlpha(g_capture.pixels.data(), g_capture.width, g_capture.height);
      lumice::test::SavePng(tmp_path, rgb.data(), g_capture.width, g_capture.height, 3);

      // Load reference and compare
      const char* ref_path = LUMICE_TEST_REF_DIR "/crystal_pyramid_default.png";
      std::vector<unsigned char> ref_data;
      int ref_w = 0, ref_h = 0, ref_ch = 0;
      bool loaded = lumice::test::LoadPng(ref_path, ref_data, ref_w, ref_h, ref_ch);
      if (!loaded) {
        fprintf(stderr, "[visual] crystal_pyramid: reference not found at %s\n", ref_path);
        fprintf(stderr, "[visual] Run test once, then copy %s to %s\n", tmp_path, ref_path);
        IM_CHECK(loaded);
        return;
      }
      IM_CHECK_EQ(ref_w, g_capture.width);
      IM_CHECK_EQ(ref_h, g_capture.height);

      constexpr double kPsnrThreshold = 40.0;
      double psnr = lumice::test::ComputePsnr(rgb.data(), ref_data.data(), ref_w, ref_h, ref_ch);
      IM_CHECK(psnr >= 0.0);
      fprintf(stderr, "[visual] crystal_pyramid: PSNR = %.2f dB\n", psnr);
      IM_CHECK(psnr > kPsnrThreshold);

      std::remove(tmp_path);
    };
  }

  // Crystal preview: Wireframe style
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "crystal_wireframe");
    t->GuiFunc = ScreenshotGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Left panel now renders cards directly (no tabs)
      ctx->Yield();

      gui::g_crystal_style = 0;  // Wireframe
      ctx->Yield(3);

      g_capture.Reset();
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);

      const char* tmp_path = "/tmp/lumice_visual_crystal_wireframe.png";
      auto rgb = lumice::test::StripAlpha(g_capture.pixels.data(), g_capture.width, g_capture.height);
      lumice::test::SavePng(tmp_path, rgb.data(), g_capture.width, g_capture.height, 3);

      const char* ref_path = LUMICE_TEST_REF_DIR "/crystal_wireframe.png";
      std::vector<unsigned char> ref_data;
      int ref_w = 0, ref_h = 0, ref_ch = 0;
      bool loaded = lumice::test::LoadPng(ref_path, ref_data, ref_w, ref_h, ref_ch);
      if (!loaded) {
        fprintf(stderr, "[visual] crystal_wireframe: reference not found at %s\n", ref_path);
        IM_CHECK(loaded);
        return;
      }
      IM_CHECK_EQ(ref_w, g_capture.width);
      IM_CHECK_EQ(ref_h, g_capture.height);

      constexpr double kPsnrThreshold = 40.0;
      double psnr = lumice::test::ComputePsnr(rgb.data(), ref_data.data(), ref_w, ref_h, ref_ch);
      IM_CHECK(psnr >= 0.0);
      fprintf(stderr, "[visual] crystal_wireframe: PSNR = %.2f dB\n", psnr);
      IM_CHECK(psnr > kPsnrThreshold);

      std::remove(tmp_path);
    };
  }

  // Crystal preview: Shaded style
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "crystal_shaded");
    t->GuiFunc = ScreenshotGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Left panel now renders cards directly (no tabs)
      ctx->Yield();

      gui::g_crystal_style = 3;  // Shaded
      ctx->Yield(3);

      g_capture.Reset();
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);

      const char* tmp_path = "/tmp/lumice_visual_crystal_shaded.png";
      auto rgb = lumice::test::StripAlpha(g_capture.pixels.data(), g_capture.width, g_capture.height);
      lumice::test::SavePng(tmp_path, rgb.data(), g_capture.width, g_capture.height, 3);

      const char* ref_path = LUMICE_TEST_REF_DIR "/crystal_shaded.png";
      std::vector<unsigned char> ref_data;
      int ref_w = 0, ref_h = 0, ref_ch = 0;
      bool loaded = lumice::test::LoadPng(ref_path, ref_data, ref_w, ref_h, ref_ch);
      if (!loaded) {
        fprintf(stderr, "[visual] crystal_shaded: reference not found at %s\n", ref_path);
        IM_CHECK(loaded);
        return;
      }
      IM_CHECK_EQ(ref_w, g_capture.width);
      IM_CHECK_EQ(ref_h, g_capture.height);

      constexpr double kPsnrThreshold = 40.0;
      double psnr = lumice::test::ComputePsnr(rgb.data(), ref_data.data(), ref_w, ref_h, ref_ch);
      IM_CHECK(psnr >= 0.0);
      fprintf(stderr, "[visual] crystal_shaded: PSNR = %.2f dB\n", psnr);
      IM_CHECK(psnr > kPsnrThreshold);

      std::remove(tmp_path);
    };
  }

  // Render preview: texture upload/readback data correctness
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "preview_load");
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      // Upload texture on main thread when requested
      if (g_capture.capture_requested && !g_capture.capture_done) {
        gui::g_preview.UploadTexture(g_capture.pixels.data(), g_capture.width, g_capture.height);
        g_capture.capture_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Create synthetic 256x256 RGB gradient
      constexpr int kTexW = 256;
      constexpr int kTexH = 256;
      std::vector<unsigned char> tex_data(kTexW * kTexH * 3);
      for (int y = 0; y < kTexH; ++y) {
        for (int x = 0; x < kTexW; ++x) {
          int idx = (y * kTexW + x) * 3;
          tex_data[idx + 0] = static_cast<unsigned char>(x);      // R = x
          tex_data[idx + 1] = static_cast<unsigned char>(y);      // G = y
          tex_data[idx + 2] = static_cast<unsigned char>(x ^ y);  // B = x XOR y
        }
      }

      // Request upload via GuiFunc (GL call on main thread)
      g_capture.Reset();
      g_capture.pixels = tex_data;
      g_capture.width = kTexW;
      g_capture.height = kTexH;
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);

      // Verify state
      IM_CHECK(gui::g_preview.HasTexture());
      const unsigned char* readback = gui::g_preview.GetTextureData();
      IM_CHECK(readback != nullptr);

      // Exact byte comparison (UploadTexture stores CPU copy directly)
      bool match = (memcmp(readback, tex_data.data(), tex_data.size()) == 0);
      if (!match) {
        // Fallback: compute PSNR
        double psnr = lumice::test::ComputePsnr(readback, tex_data.data(), kTexW, kTexH, 3);
        IM_CHECK(psnr >= 0.0);
        fprintf(stderr, "[visual] preview_load: data mismatch, PSNR = %.2f dB\n", psnr);
      }
      IM_CHECK(match);
    };
  }

  // Render preview: .lmc roundtrip with texture
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "preview_lmc_roundtrip");
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      if (g_capture.capture_requested && !g_capture.capture_done) {
        gui::g_preview.UploadTexture(g_capture.pixels.data(), g_capture.width, g_capture.height);
        g_capture.capture_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Create synthetic texture
      constexpr int kTexW = 256;
      constexpr int kTexH = 256;
      std::vector<unsigned char> original(kTexW * kTexH * 3);
      for (int y = 0; y < kTexH; ++y) {
        for (int x = 0; x < kTexW; ++x) {
          int idx = (y * kTexW + x) * 3;
          original[idx + 0] = static_cast<unsigned char>((x + y) % 256);
          original[idx + 1] = static_cast<unsigned char>((x * 2) % 256);
          original[idx + 2] = static_cast<unsigned char>((y * 3) % 256);
        }
      }

      // Upload via GuiFunc
      g_capture.Reset();
      g_capture.pixels = original;
      g_capture.width = kTexW;
      g_capture.height = kTexH;
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);
      IM_CHECK(gui::g_preview.HasTexture());

      // Save with texture
      const char* tmp_path = "/tmp/lumice_visual_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, true);
      IM_CHECK(save_ok);

      // Clear
      gui::DoNew();
      IM_CHECK(!gui::g_preview.HasTexture());

      // Load
      std::vector<unsigned char> loaded_tex;
      int loaded_w = 0, loaded_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, loaded_tex, loaded_w, loaded_h);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(loaded_w, kTexW);
      IM_CHECK_EQ(loaded_h, kTexH);
      IM_CHECK(!loaded_tex.empty());

      // Re-upload loaded texture via GuiFunc
      g_capture.Reset();
      g_capture.pixels = loaded_tex;
      g_capture.width = loaded_w;
      g_capture.height = loaded_h;
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);
      IM_CHECK(gui::g_preview.HasTexture());

      // Exact comparison: PNG is lossless, roundtrip should be identical
      const unsigned char* readback = gui::g_preview.GetTextureData();
      IM_CHECK(readback != nullptr);
      size_t data_size = static_cast<size_t>(kTexW) * kTexH * 3;
      bool match = (memcmp(readback, original.data(), data_size) == 0);
      if (!match) {
        double psnr = lumice::test::ComputePsnr(readback, original.data(), kTexW, kTexH, 3);
        IM_CHECK(psnr >= 0.0);
        fprintf(stderr, "[visual] preview_lmc_roundtrip: mismatch, PSNR = %.2f dB\n", psnr);
        IM_CHECK(psnr >= 60.0);  // Fallback threshold
      } else {
        fprintf(stderr, "[visual] preview_lmc_roundtrip: exact match\n");
      }

      // Cleanup
      std::remove(tmp_path);
    };
  }

  // Test: UpdateCpuTextureData → Save → Load preserves correct data and dimensions.
  // Regression test for bug where UpdateCpuTextureData did not update tex_width_/tex_height_,
  // causing SaveLmcFile to use stale dimensions from a previous UploadTexture/UploadXyzTexture call.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "save_texture_dimension_consistency");
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      if (g_capture.capture_requested && !g_capture.capture_done) {
        gui::g_preview.UploadTexture(g_capture.pixels.data(), g_capture.width, g_capture.height);
        g_capture.capture_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Step 1: Upload a 256x256 texture via UploadTexture (simulates normal XYZ upload setting tex_width_/tex_height_)
      constexpr int kOldW = 256;
      constexpr int kOldH = 256;
      std::vector<unsigned char> old_tex(kOldW * kOldH * 3, 0);
      g_capture.Reset();
      g_capture.pixels = old_tex;
      g_capture.width = kOldW;
      g_capture.height = kOldH;
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), kOldW);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), kOldH);

      // Step 2: Call UpdateCpuTextureData with DIFFERENT dimensions (simulates RefreshCpuTextureForSave)
      constexpr int kNewW = 128;
      constexpr int kNewH = 128;
      std::vector<unsigned char> new_tex(kNewW * kNewH * 3);
      for (int y = 0; y < kNewH; ++y) {
        for (int x = 0; x < kNewW; ++x) {
          int idx = (y * kNewW + x) * 3;
          new_tex[idx + 0] = static_cast<unsigned char>((x * 7 + y * 3) % 256);
          new_tex[idx + 1] = static_cast<unsigned char>((x * 11 + y * 5) % 256);
          new_tex[idx + 2] = static_cast<unsigned char>((x * 13 + y * 17) % 256);
        }
      }
      gui::g_preview.UpdateCpuTextureData(new_tex.data(), kNewW, kNewH);

      // Verify dimensions were updated
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), kNewW);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), kNewH);

      // Step 3: Save with texture
      const char* tmp_path = "/tmp/lumice_save_dim_test.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, true);
      IM_CHECK(save_ok);

      // Step 4: Load back
      gui::GuiState loaded_state;
      std::vector<unsigned char> loaded_tex;
      int loaded_w = 0, loaded_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, loaded_state, loaded_tex, loaded_w, loaded_h);
      IM_CHECK(load_ok);

      // Step 5: Verify loaded texture matches UpdateCpuTextureData input (not the old UploadTexture data)
      IM_CHECK_EQ(loaded_w, kNewW);
      IM_CHECK_EQ(loaded_h, kNewH);
      IM_CHECK(!loaded_tex.empty());

      size_t expected_size = static_cast<size_t>(kNewW) * kNewH * 3;
      IM_CHECK_EQ(loaded_tex.size(), expected_size);

      bool match = (memcmp(loaded_tex.data(), new_tex.data(), expected_size) == 0);
      if (!match) {
        double psnr = lumice::test::ComputePsnr(loaded_tex.data(), new_tex.data(), kNewW, kNewH, 3);
        fprintf(stderr, "[visual] save_texture_dimension_consistency: mismatch, PSNR = %.2f dB\n", psnr);
        IM_CHECK(psnr >= 60.0);
      } else {
        fprintf(stderr, "[visual] save_texture_dimension_consistency: exact match\n");
      }

      // Cleanup
      std::remove(tmp_path);
    };
  }

  // End-to-end: Run simulation → Save → Open → verify visual consistency.
  // Tests the actual user workflow: the reopened file should look the same as the live display.
  // Regression test for PostSnapshot using stale CommitConfig-time EV instead of current GUI EV.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "save_open_visual_consistency");
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      // Upload texture on main thread when requested
      if (g_capture.capture_requested && !g_capture.capture_done) {
        gui::g_preview.UploadTexture(g_capture.pixels.data(), g_capture.width, g_capture.height);
        g_capture.capture_done = true;
      }
      // FBO export on main thread when requested
      if (g_export_test.export_requested && !g_export_test.export_done) {
        g_export_test.export_result =
            gui::ExportPreviewPng(g_export_test.export_path, gui::g_preview, gui::g_preview_vp);
        g_export_test.export_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // --- Phase 1: Start simulation and wait for texture data ---
      gui::g_server = LUMICE_CreateServer();
      gui::g_state.sim.infinite = true;
      gui::g_state.sim.max_hits = 8;
      {
        auto& r = gui::g_state.renderer;
        r.lens_type = 0;  // Linear
        r.fov = 120.0f;
        r.sim_resolution_index = 0;
        r.visible = 2;
        r.exposure_offset = 0.0f;
      }
      gui::DoRun();

      // Wait for first texture upload (up to 10s)
      auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.texture_upload_count == 0) {
        ctx->Yield();
        IM_CHECK(std::chrono::steady_clock::now() < timeout);
      }
      // Let a few more frames accumulate for stable data
      for (int i = 0; i < 30; i++) {
        ctx->Yield();
      }
      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      // --- Phase 2: Stop simulation, then capture (so Save uses same data as display) ---
      gui::DoStop();
      ctx->Yield(5);  // Let the final poller sync complete

      const char* path_a = "/tmp/lumice_save_visual_A.png";
      g_export_test.Reset();
      g_export_test.export_path = path_a;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // --- Phase 3: Save ---
      const char* lmc_path = "/tmp/lumice_save_visual_test.lmc";
      gui::g_state.current_file_path = lmc_path;
      gui::DoSave();

      // --- Phase 4: Destroy server and Open the saved file ---
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;

      std::vector<unsigned char> tex_data;
      int tex_w = 0, tex_h = 0;
      bool load_ok = gui::LoadLmcFile(lmc_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);
      IM_CHECK(!tex_data.empty());

      // Upload loaded texture (GuiFunc will handle GL)
      g_capture.Reset();
      g_capture.pixels = tex_data;
      g_capture.width = tex_w;
      g_capture.height = tex_h;
      g_capture.capture_requested = true;
      ctx->Yield(2);

      // --- Phase 5: Capture loaded preview (screenshot B) ---
      const char* path_b = "/tmp/lumice_save_visual_B.png";
      g_export_test.Reset();
      g_export_test.export_path = path_b;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // --- Phase 6: Compare screenshots A and B ---
      std::vector<unsigned char> img_a, img_b;
      int wa = 0, ha = 0, ca = 0;
      int wb = 0, hb = 0, cb = 0;
      IM_CHECK(lumice::test::LoadPng(path_a, img_a, wa, ha, ca));
      IM_CHECK(lumice::test::LoadPng(path_b, img_b, wb, hb, cb));
      IM_CHECK_EQ(wa, wb);
      IM_CHECK_EQ(ha, hb);

      // Compare in RGB (strip alpha if needed)
      std::vector<unsigned char> rgb_a, rgb_b;
      int ch = std::min(ca, cb);
      if (ca == 4) {
        rgb_a = lumice::test::StripAlpha(img_a.data(), wa, ha);
      } else {
        rgb_a = img_a;
      }
      if (cb == 4) {
        rgb_b = lumice::test::StripAlpha(img_b.data(), wb, hb);
      } else {
        rgb_b = img_b;
      }

      // PSNR threshold: the save path gets a fresh snapshot from the server (slightly more accumulated
      // data than the poller's last upload), plus float→uint8 quantization (~48 dB theoretical max).
      // 28 dB catches major conversion errors (wrong EV, wrong matrix) while allowing for timing noise.
      constexpr double kPsnrThreshold = 28.0;
      double psnr = lumice::test::ComputePsnr(rgb_a.data(), rgb_b.data(), wa, ha, 3);
      fprintf(stderr, "[visual] save_open_visual_consistency: PSNR = %.2f dB (threshold = %.1f dB)\n", psnr,
              kPsnrThreshold);
      IM_CHECK(psnr > kPsnrThreshold);

      // Cleanup
      std::remove(path_a);
      std::remove(path_b);
      std::remove(lmc_path);
    };
  }

  // Thumbnail: default entry produces a non-zero texture after a few frames
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "thumbnail_default_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Let ProcessUpdateQueue run for several frames to render the default entry thumbnail
      ctx->Yield(10);

      auto tex = gui::g_thumbnail_cache.GetTexture(0, 0);
      // Thumbnail should be rendered for the default entry (non-zero texture ID)
      IM_CHECK(tex != 0);
    };
  }

  // Thumbnail: adding an entry triggers thumbnail generation
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "thumbnail_new_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(5);

      // Add a new entry
      gui::g_state.layers[0].entries.emplace_back();
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(10);

      // Both entries should have thumbnails
      auto tex0 = gui::g_thumbnail_cache.GetTexture(0, 0);
      auto tex1 = gui::g_thumbnail_cache.GetTexture(0, 1);
      IM_CHECK(tex0 != 0);
      IM_CHECK(tex1 != 0);
    };
  }
}
