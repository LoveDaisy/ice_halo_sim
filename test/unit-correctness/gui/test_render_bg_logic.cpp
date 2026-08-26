// The aspect-preset and background fields, asked the questions that need no window.
//
// Three subjects live here, and what unites them is that none of them looks at a pixel:
//   - a legacy .lmc document that predates the aspect / background fields must deserialize to the
//     factory values for them;
//   - each aspect preset reports the ratio its own name promises;
//   - CalibrateQualityThreshold() runs a real 100k-ray sim through the C API and produces data, so
//     startup never logs "no data produced". A real simulation is orthogonal to needing a window:
//     gui_unit_test links lumice_obj, so the whole C API is available here.
//
// The questions that DO need a frame are asked elsewhere and are not restated here: whether the
// background image actually reaches the shader and the exported pixels is
// test/gui/functional/test_background_overlay.cpp, whether the "screen too small" warning is drawn
// is test/gui/functional/test_view_display_controls.cpp, and whether either field survives a
// document round trip is composition-correctness/gui/test_document_roundtrip_chain.cpp.

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
#include "support/scoped_result_frame.hpp"

namespace gui = lumice::gui;

// A legacy .lmc that predates the aspect and background fields must deserialize to the factory
// values for both — one document, both field groups, because a reader that half-populated the
// legacy branch would satisfy either half alone. Uses the legacy renderers=[] shape, so the legacy
// branch finds an empty array and leaves loaded.renderer at its default values.
TEST(LegacyLmcCompat, AbsentAspectAndBackgroundFieldsTakeTheFactoryValues) {
  gui::GuiState loaded;
  EXPECT_TRUE(gui::DeserializeGuiStateJson(R"({"crystals":[],"renderers":[],"filters":[]})", loaded));
  EXPECT_EQ(loaded.aspect_preset, gui::AspectPreset::kFree);
  EXPECT_EQ(loaded.aspect_portrait, false);
  EXPECT_TRUE(loaded.bg_path.empty());
  EXPECT_EQ(loaded.bg_show, false);
  EXPECT_TRUE(std::abs(loaded.bg_alpha - 1.0f) < 0.01f);
  // The pan/zoom identity. These three are load-bearing beyond "a default came back": the
  // renderer has no compat branch for a document that predates them, so an .lmc missing these
  // keys renders unchanged ONLY IF it lands on exactly (0, 0, 1) — see the bit-for-bit identity
  // case in test_preview_renderer.cpp for the other half of that argument.
  EXPECT_FLOAT_EQ(loaded.bg_offset_x, 0.0f);
  EXPECT_FLOAT_EQ(loaded.bg_offset_y, 0.0f);
  EXPECT_FLOAT_EQ(loaded.bg_scale, 1.0f);
}

// Every preset reports the ratio its own name promises — the exact number, not a sanity band.
//
// The band this replaces (0 <= r < 100 for every preset) could not see the failure the pairing
// actually invites: kAspectPresetNames is a presentation table sitting beside the enum, and a row
// inserted into one and not the other slides every later preset's ratio by one. A user who picks
// 3:2 and is handed 4:3 sees a wrong picture with a right-looking label on it, and every value in
// that shifted table is still inside the band.
//
// The two zero rows are as load-bearing as the five numbers: kFree and kMatchBg mean "no fixed
// ratio", and the whole Display-group aspect path keys off GetAspectRatio() == 0 to say so.
TEST(AspectPresets, EachPresetReportsTheRatioItsNamePromises) {
  struct Row {
    gui::AspectPreset preset;
    const char* name;
    float ratio;  // 0 = "no fixed ratio", the sentinel the caller branches on
  };
  const Row kRows[] = {
    { gui::AspectPreset::kFree, "Free", 0.0f },
    { gui::AspectPreset::k16x9, "16:9", 16.0f / 9.0f },
    { gui::AspectPreset::k3x2, "3:2", 3.0f / 2.0f },
    { gui::AspectPreset::k4x3, "4:3", 4.0f / 3.0f },
    { gui::AspectPreset::k1x1, "1:1", 1.0f },
    { gui::AspectPreset::k2x1, "2:1", 2.0f },
    { gui::AspectPreset::kMatchBg, "Match Background", 0.0f },
  };
  // A new preset must land a row here rather than pass by not being looked at.
  static_assert(sizeof(kRows) / sizeof(kRows[0]) == gui::kAspectPresetCount,
                "every AspectPreset needs a row: a preset with no expected ratio is untested");

  for (const Row& row : kRows) {
    // The displayed name is checked alongside the ratio because the shift described above moves
    // the two together; asserting only the ratio would leave "the label says 3:2" unstated.
    EXPECT_STREQ(gui::kAspectPresetNames[static_cast<int>(row.preset)], row.name);
    EXPECT_NEAR(gui::GetAspectRatio(row.preset), row.ratio, 1e-4f) << row.name;
  }
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
  LUMICE_StatsResult stats{};
  lumice::test::ScopedResultFrame frame(gui::g_server);
  LUMICE_FrameGetStats(frame.get(), &stats);
  EXPECT_GT(stats.sim_ray_num, 0UL);

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
