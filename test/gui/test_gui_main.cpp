#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <cstdio>

#include "gui/gl_common.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gl_init.h"
#include "gui/panels.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "imgui_te_context.h"
#include "imgui_te_engine.h"
#include "imgui_te_exporters.h"
#include "test_screenshot.hpp"

namespace gui = lumice::gui;

// Shared state for screenshot tests: GL calls must happen on main thread (GuiFunc),
// verification happens on test thread (TestFunc).
struct ScreenshotCapture {
  std::vector<unsigned char> pixels;
  int width = 0;
  int height = 0;
  std::atomic<bool> capture_requested{ false };
  std::atomic<bool> capture_done{ false };

  void Reset() {
    pixels.clear();
    width = 0;
    height = 0;
    capture_requested = false;
    capture_done = false;
  }
};
static ScreenshotCapture g_capture;

// Reset all global state for test isolation
static void ResetTestState() {
  // Document state (delegates to DoNew: g_state, g_preview, g_crystal_mesh_id/hash)
  gui::DoNew();

  // UI view state
  gui::ResetCrystalView();
  gui::g_crystal_style = 1;
  gui::g_panel_collapsed = false;
  gui::g_preview_vp.active = false;

  // Runtime state
  gui::g_show_unsaved_popup = false;
  gui::g_pending_action = gui::PendingAction::kNone;
  gui::g_server = nullptr;
  gui::g_last_poll_time = std::chrono::steady_clock::now();
  gui::ResetPendingDeleteState();

  // Test state
  g_capture.Reset();
}

// Register smoke tests
static void RegisterSmokeTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "gui_smoke", "default_state");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Verify default state: 1 crystal, 1 renderer, 1 scattering layer
    IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
    IM_CHECK_EQ(static_cast<int>(gui::g_state.renderers.size()), 1);
    IM_CHECK_EQ(gui::g_state.selected_crystal, 0);
    IM_CHECK_EQ(gui::g_state.selected_renderer, 0);
    IM_CHECK_EQ(gui::g_state.dirty, false);
    IM_CHECK_EQ(gui::g_state.sim_state, gui::GuiState::SimState::kIdle);
  };
}

// P0 tests
static void RegisterP0Tests(ImGuiTestEngine* engine) {
  // P0: New
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_file", "new");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Modify state: add crystal, set dirty
      gui::CrystalConfig c;
      c.id = gui::g_state.next_crystal_id++;
      gui::g_state.crystals.push_back(c);
      gui::g_state.dirty = true;

      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.dirty, true);

      // DoNew resets
      gui::DoNew();

      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
      IM_CHECK_EQ(gui::g_state.dirty, false);
      IM_CHECK_EQ(gui::g_state.selected_crystal, 0);
      IM_CHECK_EQ(gui::g_preview.HasTexture(), false);
    };
  }

  // P0: Save/Open roundtrip
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_file", "save_open_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Modify state
      gui::g_state.crystals[0].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[0].prism_h = 2.0f;
      gui::g_state.crystals[0].upper_h = 0.3f;
      gui::g_state.crystals[0].lower_h = 0.4f;
      gui::g_state.sun.altitude = 30.0f;
      gui::g_state.sim.max_hits = 12;

      // Save
      const char* tmp_path = "/tmp/lumice_gui_test.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.crystals[0].type, gui::CrystalType::kPrism);

      // Load
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify roundtrip
      IM_CHECK_EQ(gui::g_state.crystals[0].type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::g_state.crystals[0].prism_h, 2.0f);
      IM_CHECK_EQ(gui::g_state.crystals[0].upper_h, 0.3f);
      IM_CHECK_EQ(gui::g_state.crystals[0].lower_h, 0.4f);
      IM_CHECK_EQ(gui::g_state.sun.altitude, 30.0f);
      IM_CHECK_EQ(gui::g_state.sim.max_hits, 12);
      IM_CHECK(tex_data.empty());  // save_texture=false

      // Cleanup
      std::remove(tmp_path);
    };
  }

  // P0: Run/Stop UI state
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p0_sim", "run_stop_ui");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Scene 1: kIdle — "Run" button should exist
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Run"));

      // Scene 2: kSimulating — "Stop" button should exist
      gui::g_state.sim_state = gui::GuiState::SimState::kSimulating;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Stop"));

      // Scene 3: kDone
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Run"));  // Back to Run

      // Scene 4: kModified — Revert button should appear
      gui::g_state.sim_state = gui::GuiState::SimState::kModified;
      ctx->Yield();
      IM_CHECK(ctx->ItemExists("##TopBar/Revert"));
    };
  }
}

// P1 tests
static void RegisterP1Tests(ImGuiTestEngine* engine) {
  // P1: Crystal Add/Delete
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_crystal", "add_delete");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);  // Let GUI render

      // Verify initial state: 1 crystal
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);

      // Scenario A: Add a crystal, then delete the new (unreferenced) one
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);
      IM_CHECK_EQ(gui::g_state.selected_crystal, 1);

      // Delete the new crystal (not referenced by scattering) — should delete directly
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal/Del##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);

      // Scenario B: Try to delete the referenced crystal
      // Need at least 2 crystals for Del to be enabled
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal/Add##crystal");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 2);

      // Select the first crystal (referenced by scattering)
      gui::g_state.selected_crystal = 0;
      ctx->Yield();

      // Click Del — should open confirmation popup
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal/Del##crystal");
      ctx->Yield(2);

      // Click Delete in the popup to confirm
      ctx->ItemClick("Delete Crystal?/Delete");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
    };
  }

  // P1: Filter Add/Delete
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_filter", "add_delete");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Verify initial: no filters
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 0);

      // Switch to Filter tab
      ctx->ItemClick("##LeftPanel/ConfigTabs/Filter");
      ctx->Yield();

      // Add a filter
      ctx->ItemClick("##LeftPanel/ConfigTabs/Filter/Add##filter");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 1);
      IM_CHECK_EQ(gui::g_state.selected_filter, 0);

      // Delete the filter (unreferenced) — direct delete
      ctx->ItemClick("##LeftPanel/ConfigTabs/Filter/Del##filter");
      IM_CHECK_EQ(static_cast<int>(gui::g_state.filters.size()), 0);
    };
  }

  // P1: Unsaved Changes Popup
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_file", "unsaved_popup");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Set dirty
      gui::g_state.dirty = true;
      ctx->Yield();

      // Click New — should trigger unsaved popup
      ctx->ItemClick("##TopBar/New");
      ctx->Yield(2);

      // Click "Don't Save" in the popup
      ctx->ItemClick("Unsaved Changes/Don't Save");

      // Verify state was reset
      IM_CHECK_EQ(gui::g_state.dirty, false);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.crystals.size()), 1);
    };
  }
}

// P2 tests
static void RegisterP2Tests(ImGuiTestEngine* engine) {
  // P2: MarkDirty
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_state", "mark_dirty");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      IM_CHECK_EQ(gui::g_state.dirty, false);

      // Mark dirty
      gui::g_state.MarkDirty();
      IM_CHECK_EQ(gui::g_state.dirty, true);

      // DoNew resets dirty
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.dirty, false);
    };
  }

  // P2: Lens switch — full-sky resets elevation/azimuth/roll
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p2_render", "lens_switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Switch to Render tab so RenderRenderTab() runs
      ctx->ItemClick("##LeftPanel/ConfigTabs/Render");
      ctx->Yield();

      // Set non-zero view params
      gui::g_state.renderers[0].elevation = 45.0f;
      gui::g_state.renderers[0].azimuth = -30.0f;
      gui::g_state.renderers[0].roll = 10.0f;

      // Switch to full-sky lens type (index 4 = Dual Fisheye Equal Area)
      gui::g_state.renderers[0].lens_type = 4;

      // Need frame renders to trigger RenderRenderTab's full-sky reset
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderers[0].elevation, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderers[0].azimuth, 0.0f);
      IM_CHECK_EQ(gui::g_state.renderers[0].roll, 0.0f);
    };
  }
}

// GuiFunc that renders GUI and captures crystal texture when requested
static void ScreenshotGuiFunc(ImGuiTestContext* /*ctx*/) {
  // Normal GUI rendering is handled by the main loop.
  // Capture crystal texture on main thread when requested.
  if (g_capture.capture_requested && !g_capture.capture_done) {
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
static void RegisterScreenshotTests(ImGuiTestEngine* engine) {
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
}

// Visual tests: crystal preview + render preview data correctness
static void RegisterVisualTests(ImGuiTestEngine* engine) {
  // Crystal preview: Pyramid (HiddenLine, default style after ResetTestState)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "crystal_pyramid");
    t->GuiFunc = ScreenshotGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Ensure Crystal tab is active (previous tests may have switched to Render tab)
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
      ctx->Yield();

      // Switch to Pyramid type
      gui::g_state.crystals[0].type = gui::CrystalType::kPyramid;
      // upper_h/lower_h default to 0.2 after Step 1 change
      gui::g_crystal_mesh_id = -1;  // Force mesh rebuild

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

      // Ensure Crystal tab is active
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
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

      // Ensure Crystal tab is active
      ctx->ItemClick("##LeftPanel/ConfigTabs/Crystal");
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
}

int main(int /*argc*/, char** /*argv*/) {
  // GLFW init
  glfwSetErrorCallback(gui::GlfwErrorCallback);
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);  // Hidden window mode

  GLFWwindow* window =
      glfwCreateWindow(gui::kInitWindowWidth, gui::kInitWindowHeight, "LumiceGUITests", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);  // No VSync for tests

  if (!gui::InitGLLoader()) {
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  // ImGui setup
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = nullptr;

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  // Initialize GUI state
  gui::g_state = gui::InitDefaultState();

  if (!gui::g_preview.Init()) {
    fprintf(stderr, "Failed to initialize preview renderer\n");
    return 1;
  }
  if (!gui::g_crystal_renderer.Init(256, 256)) {
    fprintf(stderr, "Failed to initialize crystal renderer\n");
    return 1;
  }
  gui::ResetCrystalView();

  // Setup test engine
  ImGuiTestEngine* engine = ImGuiTestEngine_CreateContext();
  ImGuiTestEngineIO& test_io = ImGuiTestEngine_GetIO(engine);
  test_io.ConfigVerboseLevel = ImGuiTestVerboseLevel_Info;
  test_io.ConfigVerboseLevelOnError = ImGuiTestVerboseLevel_Debug;
  test_io.ConfigRunSpeed = ImGuiTestRunSpeed_Fast;
  test_io.ConfigNoThrottle = true;

  ImGuiTestEngine_Start(engine, ImGui::GetCurrentContext());
  ImGuiTestEngine_InstallDefaultCrashHandler();

  // Register and queue all tests
  RegisterSmokeTests(engine);
  RegisterP0Tests(engine);
  RegisterP1Tests(engine);
  RegisterP2Tests(engine);
  RegisterScreenshotTests(engine);
  RegisterVisualTests(engine);
  ImGuiTestEngine_QueueTests(engine, ImGuiTestGroup_Tests);

  // Main loop — runs until all tests complete
  while (true) {
    glfwPollEvents();

    if (glfwWindowShouldClose(window)) {
      if (ImGuiTestEngine_TryAbortEngine(engine)) {
        break;
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Render GUI panels (needed for test engine to find widgets)
    // Must match src/gui/main.cpp main loop render calls
    int win_w = 0;
    int win_h = 0;
    glfwGetWindowSize(window, &win_w, &win_h);
    auto layout_width = static_cast<float>(win_w);
    auto layout_height = static_cast<float>(win_h);

    gui::RenderTopBar(layout_width);
    gui::RenderLeftPanel(layout_height);
    gui::RenderPreviewPanel(window, layout_width, layout_height);
    gui::RenderFloatingLensBar(layout_width);
    gui::RenderStatusBar(layout_width, layout_height);
    gui::RenderUnsavedPopup(window);

    ImGui::Render();

    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    ImGuiTestEngine_PostSwap(engine);

    // Exit when all tests are done
    if (!ImGuiTestEngine_IsTestQueueEmpty(engine)) {
      continue;
    }
    if (!test_io.IsRunningTests) {
      break;
    }
  }

  // Get results
  ImGuiTestEngine_PrintResultSummary(engine);
  int count_tested = 0;
  int count_success = 0;
  ImGuiTestEngine_GetResult(engine, count_tested, count_success);
  fprintf(stderr, "[GUI Tests] %d/%d tests passed\n", count_success, count_tested);

  // Cleanup
  ImGuiTestEngine_Stop(engine);

  gui::g_crystal_renderer.Destroy();
  gui::g_preview.Destroy();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGuiTestEngine_DestroyContext(engine);
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return (count_tested == count_success) ? 0 : 1;
}
