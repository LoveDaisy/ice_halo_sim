#include <cstdio>
#include <cstring>

#include "test_gui_shared.hpp"

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
}
