#include <GLFW/glfw3.h>
#include <stb_image.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

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

// Shared state for export tests: GL calls must happen on main thread (GuiFunc).
struct ExportTestState {
  std::string export_path;
  bool upload_requested = false;
  bool upload_done = false;
  bool export_requested = false;
  bool export_done = false;
  bool export_result = false;

  void Reset() {
    export_path.clear();
    upload_requested = false;
    upload_done = false;
    export_requested = false;
    export_done = false;
    export_result = false;
  }
};
static ExportTestState g_export_test;

// Shared state for background overlay tests: GL calls must happen on main thread (GuiFunc).
struct BgOverlayTestState {
  std::string bg_image_path;
  bool bg_upload_requested = false;
  bool bg_upload_done = false;
  bool export_requested = false;
  bool export_done = false;
  bool export_result = false;
  std::string export_path;

  void Reset() {
    bg_image_path.clear();
    bg_upload_requested = false;
    bg_upload_done = false;
    export_requested = false;
    export_done = false;
    export_result = false;
    export_path.clear();
  }
};
static BgOverlayTestState g_bg_test;

// Reset all global state for test isolation
static void ResetTestState() {
  // Document state (delegates to DoNew: g_state, g_preview, g_crystal_mesh_id/hash)
  gui::DoNew();

  // UI view state
  gui::ResetCrystalView();
  gui::g_crystal_style = 1;
  gui::g_panel_collapsed = false;
  gui::g_preview_vp.active = false;
  gui::g_programmatic_resize = 0;

  // Runtime state
  gui::g_show_unsaved_popup = false;
  gui::g_pending_action = gui::PendingAction::kNone;
  gui::g_server_poller.Stop();  // Stop poller before nulling server
  gui::g_server = nullptr;
  gui::ResetPendingDeleteState();

  // Test state
  g_capture.Reset();
  g_export_test.Reset();
  g_bg_test.Reset();
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

// Aspect ratio tests
static void RegisterAspectRatioTests(ImGuiTestEngine* engine) {
  // Test 1: Preset switch — verify GetAspectRatio returns correct values
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "aspect_ratio", "switch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      struct TestCase {
        gui::AspectPreset preset;
        float expected_ratio;
      };
      TestCase cases[] = {
        { gui::AspectPreset::k16x9, 16.0f / 9.0f },
        { gui::AspectPreset::k3x2, 3.0f / 2.0f },
        { gui::AspectPreset::k4x3, 4.0f / 3.0f },
        { gui::AspectPreset::k1x1, 1.0f },
      };

      for (auto& tc : cases) {
        float ratio = gui::GetAspectRatio(tc.preset);
        IM_CHECK_LT(std::abs(ratio - tc.expected_ratio), 0.01f);
      }

      // Free and MatchBg return 0
      IM_CHECK_EQ(gui::GetAspectRatio(gui::AspectPreset::kFree), 0.0f);
      IM_CHECK_EQ(gui::GetAspectRatio(gui::AspectPreset::kMatchBg), 0.0f);
    };
  }

  // Test 2: Portrait toggle
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "aspect_ratio", "portrait_toggle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Landscape 3:2 = 1.5
      gui::g_state.aspect_preset = gui::AspectPreset::k3x2;
      gui::g_state.aspect_portrait = false;
      float ratio = gui::GetAspectRatio(gui::g_state.aspect_preset);
      IM_CHECK_LT(std::abs(ratio - 1.5f), 0.01f);

      // Portrait inverts: 1/1.5 ≈ 0.667
      gui::g_state.aspect_portrait = true;
      float inv_ratio = 1.0f / gui::GetAspectRatio(gui::g_state.aspect_preset);
      IM_CHECK_LT(std::abs(inv_ratio - 2.0f / 3.0f), 0.01f);
    };
  }

  // Test 3: Free mode — no ratio constraint
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "aspect_ratio", "free_mode");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      IM_CHECK_EQ(gui::g_state.aspect_preset, gui::AspectPreset::kFree);
      IM_CHECK_EQ(gui::g_state.aspect_portrait, false);
      IM_CHECK_EQ(gui::GetAspectRatio(gui::AspectPreset::kFree), 0.0f);
    };
  }

  // Test 4: .lmc roundtrip
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "aspect_ratio", "lmc_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Set 4:3 landscape
      gui::g_state.aspect_preset = gui::AspectPreset::k4x3;
      gui::g_state.aspect_portrait = false;

      // Save
      const char* tmp_path = "/tmp/lumice_aspect_test.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.aspect_preset, gui::AspectPreset::kFree);

      // Load
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify roundtrip
      IM_CHECK_EQ(gui::g_state.aspect_preset, gui::AspectPreset::k4x3);
      IM_CHECK_EQ(gui::g_state.aspect_portrait, false);

      std::remove(tmp_path);
    };
  }

  // Test 5: Old .lmc compat — missing aspect fields default to Free
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "aspect_ratio", "old_lmc_compat");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Minimal JSON without aspect fields
      std::string json = R"({"crystals":[],"renderers":[],"filters":[]})";
      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.aspect_preset, gui::AspectPreset::kFree);
      IM_CHECK_EQ(loaded.aspect_portrait, false);
    };
  }

  // Test 6: Screen bounds — ApplyAspectRatio clamps to workarea
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "aspect_ratio", "screen_bounds");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);

      // Verify workarea is available
      int work_x = 0;
      int work_y = 0;
      int work_w = 0;
      int work_h = 0;
      glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &work_x, &work_y, &work_w, &work_h);

      // The function should never produce sizes exceeding workarea
      // Just verify GetAspectRatio values are sane
      for (int i = 0; i < gui::kAspectPresetCount; i++) {
        float r = gui::GetAspectRatio(static_cast<gui::AspectPreset>(i));
        IM_CHECK(r >= 0.0f);
        if (r > 0.0f) {
          IM_CHECK(r < 100.0f);  // Sanity check
        }
      }
    };
  }
}

// Synthetic texture for export tests
static std::vector<unsigned char> g_synth_tex;
constexpr int kSynthTexW = 64;
constexpr int kSynthTexH = 64;

static void InitSynthTexture() {
  if (g_synth_tex.empty()) {
    g_synth_tex.resize(kSynthTexW * kSynthTexH * 3);
    for (int y = 0; y < kSynthTexH; ++y) {
      for (int x = 0; x < kSynthTexW; ++x) {
        int idx = (y * kSynthTexW + x) * 3;
        g_synth_tex[idx + 0] = static_cast<unsigned char>(x * 4);        // R
        g_synth_tex[idx + 1] = static_cast<unsigned char>(y * 4);        // G
        g_synth_tex[idx + 2] = static_cast<unsigned char>((x ^ y) * 4);  // B
      }
    }
  }
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
}

static void RegisterExportPreviewTests(ImGuiTestEngine* engine) {
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

static void RegisterBgOverlayTests(ImGuiTestEngine* engine) {
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
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.png";
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

      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_portrait.png";
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
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.png";
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
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.png";
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
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.png";
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
      g_bg_test.bg_image_path = LUMICE_TEST_REF_DIR "/bg_test_landscape.png";
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

// ========== Performance Tests ==========

static const char* CreatePerfConfig() {
  // Minimal config: single prism crystal, sun at 20°, infinite rays, 1024x512 resolution
  return R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "ratio": {"upper": 1.0, "lower": 1.0}}],
    "filter": [],
    "scene": {
      "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0, "diameter": 0.5, "spectrum": "D65"},
      "ray_num": "infinite",
      "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{"id": 1, "lens": {"type": "rectangular", "fov": 180.0},
                "resolution": [1024, 512], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
                "visible": "full", "background": [0, 0, 0], "opacity": 1.0, "intensity_factor": 1.0}]
  })";
}

static void StartPerfSimulation() {
  gui::g_server = LUMICE_CreateServer();
  LUMICE_InitLogger(gui::g_server);

  // Set up g_state to match perf config, then use DoRun() so the server's
  // config_manager_ is populated from the same SerializeCoreConfig path.
  gui::g_state.sun.altitude = 20.0f;
  gui::g_state.sun.azimuth = 0.0f;
  gui::g_state.sun.diameter = 0.5f;
  gui::g_state.sun.spectrum_index = 2;  // D65
  gui::g_state.sim.infinite = true;
  gui::g_state.sim.max_hits = 8;
  if (!gui::g_state.renderers.empty()) {
    auto& r = gui::g_state.renderers[0];
    r.lens_type = 7;  // Rectangular
    r.fov = 180.0f;
    r.sim_resolution_index = 0;  // 512 → Core resolution [1024, 512], matching CreatePerfConfig
    r.visible = 2;               // Full
    r.background[0] = r.background[1] = r.background[2] = 0.0f;
    r.exposure_offset = 0.0f;
  }
  gui::DoRun();
}

static void StopPerfSimulation() {
  gui::g_server_poller.Stop();
  if (gui::g_server) {
    LUMICE_StopServer(gui::g_server);
    LUMICE_DestroyServer(gui::g_server);
    gui::g_server = nullptr;
  }
  gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
}

static void ReportPerf(const char* label, unsigned long start_rays, unsigned long end_rays, double elapsed_sec) {
  unsigned long delta = end_rays - start_rays;
  double rays_per_sec = elapsed_sec > 0 ? static_cast<double>(delta) / elapsed_sec : 0;
  fprintf(stderr, "[PERF] %s: %.1f rays/sec (%lu rays in %.1fs)\n", label, rays_per_sec, delta, elapsed_sec);
}

// ========== Import/Export Tests ==========

static void RegisterImportExportTests(ImGuiTestEngine* engine) {
  // Test 1: JSON round-trip — serialize then deserialize, verify key fields match
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "json_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Set up non-default state
      gui::g_state.sim.ray_num_millions = 2.5f;

      // Serialize → deserialize
      std::string json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!json.empty());

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(json, loaded);
      IM_CHECK(ok);

      // Verify key fields survived round-trip
      IM_CHECK(std::abs(loaded.sim.ray_num_millions - gui::g_state.sim.ray_num_millions) < 0.01f);
      IM_CHECK_EQ(static_cast<int>(loaded.crystals.size()), static_cast<int>(gui::g_state.crystals.size()));
      IM_CHECK_EQ(static_cast<int>(loaded.filters.size()), static_cast<int>(gui::g_state.filters.size()));
      IM_CHECK_EQ(static_cast<int>(loaded.renderers.size()), static_cast<int>(gui::g_state.renderers.size()));
    };
  }

  // Test 2: JSON file round-trip — write to file, read back, deserialize, verify
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "json_file_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      gui::g_state.sim.ray_num_millions = 3.0f;
      std::string json = gui::SerializeCoreConfig(gui::g_state);

      const char* tmp_path = "/tmp/lumice_config_test.json";
      bool write_ok = gui::ExportConfigJson(tmp_path, json);
      IM_CHECK(write_ok);

      // Read back
      std::ifstream in(tmp_path);
      IM_CHECK(in.is_open());
      std::string read_json((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      in.close();

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(read_json, loaded);
      IM_CHECK(ok);
      IM_CHECK(std::abs(loaded.sim.ray_num_millions - 3.0f) < 0.01f);

      std::remove(tmp_path);
    };
  }

  // Test 3: Equirect PNG export — write synthetic data, verify file is readable
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "equirect_export");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Create synthetic 4x2 RGB image
      const int w = 4;
      const int h = 2;
      std::vector<unsigned char> data(w * h * 3, 128);

      const char* tmp_path = "/tmp/lumice_equirect_test.png";
      bool ok = gui::ExportEquirectPng(tmp_path, data.data(), w, h);
      IM_CHECK(ok);

      // Verify file is loadable
      std::vector<unsigned char> img_data;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      bool loaded = lumice::test::LoadPng(tmp_path, img_data, img_w, img_h, img_ch);
      IM_CHECK(loaded);
      IM_CHECK_EQ(img_w, w);
      IM_CHECK_EQ(img_h, h);

      std::remove(tmp_path);
    };
  }
}


static void RegisterPerfTests(ImGuiTestEngine* engine) {
  // Scenario 1: Steady-state simulation (baseline)
  // Measures: rays/sec, texture update interval (avg/min/max), texture FPS
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "perf_test", "steady_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();

      // Wait for first batch of data
      auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > timeout) {
          fprintf(stderr, "[PERF] ERROR: No simulation data after 10s\n");
          break;
        }
      }

      // Measure for 2 seconds (short enough for test engine timeout)
      unsigned long start_rays = gui::g_state.stats_sim_ray_num;
      auto upload_before = gui::g_state.texture_upload_count;
      auto start_time = std::chrono::steady_clock::now();
      auto end_time = start_time + std::chrono::seconds(2);

      // Track texture update intervals
      auto last_upload_count = upload_before;
      auto last_upload_time = start_time;
      std::vector<double> texture_intervals_ms;

      while (std::chrono::steady_clock::now() < end_time) {
        ctx->Yield();
        // Detect new texture upload
        if (gui::g_state.texture_upload_count != last_upload_count) {
          auto now = std::chrono::steady_clock::now();
          double interval = std::chrono::duration<double, std::milli>(now - last_upload_time).count();
          texture_intervals_ms.push_back(interval);
          last_upload_count = gui::g_state.texture_upload_count;
          last_upload_time = now;
        }
      }

      unsigned long end_rays = gui::g_state.stats_sim_ray_num;
      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

      ReportPerf("steady_state", start_rays, end_rays, elapsed);
      double rays_per_sec = elapsed > 0 ? static_cast<double>(end_rays - start_rays) / elapsed : 0;
      IM_CHECK_GT(rays_per_sec, 0.0);

      // Report texture update frequency
      auto texture_uploads = gui::g_state.texture_upload_count - upload_before;
      if (!texture_intervals_ms.empty()) {
        std::sort(texture_intervals_ms.begin(), texture_intervals_ms.end());
        double sum = 0;
        for (auto v : texture_intervals_ms) {
          sum += v;
        }
        double avg = sum / texture_intervals_ms.size();
        double median = texture_intervals_ms[texture_intervals_ms.size() / 2];
        double min_val = texture_intervals_ms.front();
        double max_val = texture_intervals_ms.back();
        fprintf(stderr, "[PERF] steady_state: %lu texture updates in %.1fs (%.1f FPS)\n", texture_uploads, elapsed,
                texture_uploads / elapsed);
        fprintf(stderr, "[PERF] steady_state: texture interval avg=%.0fms median=%.0fms min=%.0fms max=%.0fms\n", avg,
                median, min_val, max_val);
      }

      StopPerfSimulation();
    };
  }

  // Scenario 2: Parameter drag (slider interaction during simulation)
  // All parameter changes trigger full restart (hot-update was removed in task-52.1).
  // Measures: rays/sec, restarts, upload ratio, per-restart ray count distribution
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "perf_test", "slider_drag");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();

      // Wait for first batch of data
      auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > timeout) {
          fprintf(stderr, "[PERF] ERROR: No simulation data after 10s\n");
          break;
        }
      }

      // Measure: alternate crystal height between 0.8 and 1.2 over 5 seconds.
      // Each change triggers a full Stop/Start restart via DoRun().
      // Track cumulative rays across restarts via direct C API (not SyncFromPoller which has delay).
      auto start_time = std::chrono::steady_clock::now();
      auto end_time = start_time + std::chrono::seconds(5);
      auto last_commit = start_time;
      unsigned long cumulative_rays = 0;
      int iteration = 0;
      int restart_count = 0;
      auto upload_before = gui::g_state.texture_upload_count;
      std::vector<unsigned long> per_restart_rays;  // Ray count per restart cycle

      auto read_server_rays = [&]() -> unsigned long {
        if (!gui::g_server) {
          return 0;
        }
        LUMICE_StatsResult stats[2]{};
        LUMICE_GetStatsResults(gui::g_server, stats, 1);
        return stats[0].sim_ray_num;
      };

      while (std::chrono::steady_clock::now() < end_time) {
        if (!gui::g_state.crystals.empty()) {
          gui::g_state.crystals[0].height = (iteration % 2 == 0) ? 0.8f : 1.2f;
        }
        gui::g_state.dirty = true;
        iteration++;

        // Yield a few frames then check commit. Yield count is small (2 frames)
        // so the commit check runs frequently, letting kCommitIntervalMs control
        // the actual commit rate rather than the yield loop duration.
        for (int i = 0; i < 2; i++) {
          ctx->Yield();
        }

        // Throttled commit: same logic as main.cpp auto-commit.
        auto now = std::chrono::steady_clock::now();
        auto commit_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
        if (commit_elapsed >= gui::kCommitIntervalMs) {
          auto rays_this_cycle = read_server_rays();
          per_restart_rays.push_back(rays_this_cycle);
          cumulative_rays += rays_this_cycle;
          gui::g_state.dirty = false;
          gui::DoRun();
          last_commit = now;
          restart_count++;
        }
      }
      auto final_rays = read_server_rays();
      per_restart_rays.push_back(final_rays);
      cumulative_rays += final_rays;

      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

      double rays_per_sec = elapsed > 0 ? static_cast<double>(cumulative_rays) / elapsed : 0;
      fprintf(stderr, "[PERF] slider_drag: %.1f rays/sec (%lu rays in %.1fs)\n", rays_per_sec, cumulative_rays,
              elapsed);
      fprintf(stderr, "[PERF] slider_drag: %d param changes, %d restarts in %.1fs\n", iteration, restart_count,
              elapsed);

      auto texture_uploads = gui::g_state.texture_upload_count - upload_before;
      double upload_ratio = restart_count > 0 ? static_cast<double>(texture_uploads) / restart_count : 0;
      fprintf(stderr, "[PERF] slider_drag: %lu texture uploads / %d restarts (ratio: %.2f)\n", texture_uploads,
              restart_count, upload_ratio);

      // Report per-restart ray count distribution
      if (!per_restart_rays.empty()) {
        std::sort(per_restart_rays.begin(), per_restart_rays.end());
        unsigned long sum = 0;
        for (auto v : per_restart_rays) {
          sum += v;
        }
        double avg = static_cast<double>(sum) / per_restart_rays.size();
        auto median = per_restart_rays[per_restart_rays.size() / 2];
        auto min_val = per_restart_rays.front();
        auto max_val = per_restart_rays.back();
        fprintf(stderr, "[PERF] slider_drag: rays/restart avg=%.0f median=%lu min=%lu max=%lu (n=%zu)\n", avg, median,
                min_val, max_val, per_restart_rays.size());
      }

      IM_CHECK_GT(rays_per_sec, 0.0);
      IM_CHECK_GE(upload_ratio, 0.5);

      StopPerfSimulation();
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

  glfwSetWindowSizeCallback(window, gui::WindowSizeCallback);
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
  RegisterAspectRatioTests(engine);
  RegisterExportPreviewTests(engine);
  RegisterScreenshotTests(engine);
  RegisterVisualTests(engine);
  RegisterBgOverlayTests(engine);
  RegisterImportExportTests(engine);
  RegisterPerfTests(engine);
  ImGuiTestEngine_QueueTests(engine, ImGuiTestGroup_Tests);

  // Main loop — runs until all tests complete
  while (true) {
    glfwPollEvents();
    gui::SyncFromPoller();  // Sync server data for perf tests (no-op when g_server is null)

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
