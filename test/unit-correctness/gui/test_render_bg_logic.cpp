// Non-pixel cases lifted out of the render and background VISUAL suites.
//
// Their siblings in test/gui/visual/test_gui_render.cpp and test_gui_bg.cpp compare rendered
// frames against reference images; these four never look at a pixel, so they belong in the
// windowless target instead:
//   AspectRatio.old_lmc_compat / Bg.old_lmc_compat — a legacy .lmc document that predates the
//     aspect / background fields must deserialize to the factory values for them
//   AspectRatio.screen_bounds — every preset's ratio is finite and in range
//   Calibration.no_warning_on_startup — CalibrateQualityThreshold() runs a real 100k-ray sim
//     through the C API and produces data, so startup never logs "no data produced". A real
//     simulation is orthogonal to needing a window: gui_unit_test links lumice_obj, so the whole
//     C API is available here.

#include <gtest/gtest.h>
#include <spdlog/common.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_logger.hpp"
#include "gui/gui_state.hpp"
#include "gui/log_sink.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

// Test 5: Old .lmc compat — missing aspect fields default to Free
TEST(AspectRatio, old_lmc_compat) {
  // Minimal JSON without aspect fields. Uses legacy renderers=[] shape; legacy branch
  // finds an empty array and leaves loaded.renderer at default values. This test only
  // asserts aspect_* fields.
  std::string json = R"({"crystals":[],"renderers":[],"filters":[]})";
  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  EXPECT_TRUE(ok);
  EXPECT_EQ(loaded.aspect_preset, gui::AspectPreset::kFree);
  EXPECT_EQ(loaded.aspect_portrait, false);
}

// Test 6: Screen bounds — ApplyAspectRatio clamps to workarea
TEST(AspectRatio, screen_bounds) {
  // The monitor-workarea query that used to open this case wrote four locals nobody read; what
  // the case actually asserts is that every preset's ratio is sane, which needs no monitor.
  for (int i = 0; i < gui::kAspectPresetCount; i++) {
    float r = gui::GetAspectRatio(static_cast<gui::AspectPreset>(i));
    EXPECT_TRUE(r >= 0.0f);
    if (r > 0.0f) {
      EXPECT_TRUE(r < 100.0f);  // Sanity check
    }
  }
}

// Test 6: bg/old_lmc_compat — JSON without bg fields gets defaults
TEST(Bg, old_lmc_compat) {
  // Empty legacy-format JSON: exercises DeserializeGuiStateJson fallback paths
  // (crystals/renderers/filters as ID-arrays). With renderers=[], the legacy branch
  // finds an empty vector and leaves loaded.renderer at default-constructed values;
  // this test only asserts bg_* fields, so the renderer branch is exercised but not
  // asserted against.
  std::string json = R"({"crystals":[],"renderers":[],"filters":[]})";
  gui::GuiState loaded;
  bool ok = gui::DeserializeGuiStateJson(json, loaded);
  EXPECT_TRUE(ok);
  EXPECT_TRUE(loaded.bg_path.empty());
  EXPECT_EQ(loaded.bg_show, false);
  EXPECT_TRUE(std::abs(loaded.bg_alpha - 1.0f) < 0.01f);
}

TEST(Calibration, no_warning_on_startup) {
  // No ResetTestState(): that is gui_test harness scaffolding, and this case installs everything
  // CalibrateQualityThreshold() reads — g_state and g_server — explicitly, right here.
  gui::g_state = gui::InitDefaultState();

  // Create server with DEBUG log level for diagnostics
  gui::g_server = LUMICE_CreateServer();
  EXPECT_TRUE(gui::g_server != nullptr);
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
  EXPECT_GT(stats[0].sim_ray_num, 0UL);

  // Negative assertion: no calibration warning
  bool found_warning = false;
  log_sink->ForEachEntry([&](size_t, const gui::LogEntry& entry) {
    if (entry.message.find("no data produced") != std::string::npos) {
      found_warning = true;
    }
  });
  EXPECT_TRUE(!found_warning);

  // Cleanup: destroy server BEFORE removing sink (avoid dangling writes during join)
  LUMICE_StopServer(gui::g_server);
  LUMICE_DestroyServer(gui::g_server);
  gui::g_server = nullptr;
  LUMICE_SetLogCallback(nullptr);
  gui::g_imgui_log_sink = nullptr;
  gui::GetGuiLogger().sinks().pop_back();
  gui::SetGuiLogLevel(spdlog::level::warn);  // Restore default level
}
