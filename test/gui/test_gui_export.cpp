#include <cstdio>
#include <cstring>
#include <vector>

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
}
