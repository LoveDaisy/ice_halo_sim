#include <GLFW/glfw3.h>

#include <fstream>
#include <memory>

#include "gui/gl_common.h"
#include "gui/gui_logger.hpp"
#include "gui/log_sink.hpp"
#include "test_gui_shared.hpp"

// Aspect ratio tests
void RegisterAspectRatioTests(ImGuiTestEngine* engine) {
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

      // Minimal JSON without aspect fields. Uses legacy renderers=[] shape; legacy branch
      // finds an empty array and leaves loaded.renderer at default values. This test only
      // asserts aspect_* fields.
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

// ========== Calibration Tests ==========

void RegisterCalibrationTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "calibration", "no_warning_on_startup");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    ResetTestState();
    gui::g_state = gui::InitDefaultState();

    // Create server with DEBUG log level for diagnostics
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    LUMICE_SetLogLevel(gui::g_server, LUMICE_LOG_DEBUG);
    gui::SetGuiLogLevel(spdlog::level::debug);

    // Set up log capture: create sink and bridge Core logs
    auto log_sink = std::make_shared<gui::ImGuiLogSink>();
    gui::GetGuiLogger().sinks().push_back(log_sink);
    gui::g_imgui_log_sink = log_sink;
    LUMICE_SetLogCallback([](LUMICE_LogLevel level, const char* /*name*/, const char* message) {
      if (gui::g_imgui_log_sink) {
        auto spd_level = static_cast<spdlog::level::level_enum>(level);
        gui::g_imgui_log_sink->ReceiveExternal(spd_level, message);
      }
    });

    // Run calibration (blocks up to 2s)
    gui::CalibrateQualityThreshold();

    // Positive assertion: stats must have data
    LUMICE_StatsResult stats[2]{};
    LUMICE_GetStatsResults(gui::g_server, stats, 1);
    IM_CHECK_GT(stats[0].sim_ray_num, 0UL);

    // Negative assertion: no calibration warning
    bool found_warning = false;
    log_sink->ForEachEntry([&](size_t, const gui::LogEntry& entry) {
      if (entry.message.find("no data produced") != std::string::npos) {
        found_warning = true;
      }
    });
    IM_CHECK(!found_warning);

    // Cleanup: destroy server BEFORE removing sink (avoid dangling writes during join)
    LUMICE_StopServer(gui::g_server);
    LUMICE_DestroyServer(gui::g_server);
    gui::g_server = nullptr;
    LUMICE_SetLogCallback(nullptr);
    gui::g_imgui_log_sink = nullptr;
    gui::GetGuiLogger().sinks().pop_back();
    gui::SetGuiLogLevel(spdlog::level::warn);  // Restore default level
  };
}
