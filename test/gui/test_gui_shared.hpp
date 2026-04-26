#ifndef LUMICE_TEST_GUI_SHARED_HPP
#define LUMICE_TEST_GUI_SHARED_HPP

#include <atomic>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/crystal_renderer.hpp"
#include "gui/edit_modals.hpp"
#include "gui/file_io.hpp"
#include "gui/panels.hpp"
#include "imgui.h"
#include "imgui_te_context.h"
#include "imgui_te_engine.h"
#include "test_screenshot.hpp"

namespace gui = lumice::gui;

// ========== Shared state structs ==========

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

// Structurally equivalent to ScreenshotCapture; kept separate so each capture
// semantic (crystal FBO vs left-panel default-framebuffer) has its own lifecycle.
// If a third panel visual regression is added, consolidate into a single
// GenericRegionCaptureState<Tag> instead of copy-pasting again.
struct LeftPanelCaptureState {
  std::atomic<bool> requested{ false };
  std::atomic<bool> done{ false };
  std::vector<unsigned char> pixels;
  int width = 0;
  int height = 0;

  void Reset() {
    done.store(false);
    requested.store(false);
    pixels.clear();
    width = 0;
    height = 0;
  }
};

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

// ========== Extern global variables (defined in test_gui_main.cpp) ==========

extern ScreenshotCapture g_capture;
extern ExportTestState g_export_test;
extern BgOverlayTestState g_bg_test;
extern LeftPanelCaptureState g_left_panel_capture;
extern std::vector<unsigned char> g_synth_tex;
extern int g_core_log_level;
extern int g_gui_log_level;
extern bool g_enable_visible;
extern bool g_enable_vsync;
extern bool g_enable_frame_limit;
extern bool g_enable_main_loop_commit;
extern bool g_enable_log_panel;
extern int g_dorun_delay_ms;
extern int g_main_loop_restart_count;
extern unsigned long g_main_loop_cumulative_rays;

// ========== Shared constants ==========

constexpr int kSynthTexW = 64;
constexpr int kSynthTexH = 64;

// ========== Shared functions (defined in test_gui_main.cpp) ==========

void ResetTestState();
void InitSynthTexture();
void StartPerfSimulation();
void StopPerfSimulation();

// ========== Register function declarations ==========

void RegisterSmokeTests(ImGuiTestEngine* engine);
void RegisterP0Tests(ImGuiTestEngine* engine);
void RegisterP1Tests(ImGuiTestEngine* engine);
void RegisterP2Tests(ImGuiTestEngine* engine);
void RegisterAspectRatioTests(ImGuiTestEngine* engine);
void RegisterExportPreviewTests(ImGuiTestEngine* engine);
void RegisterScreenshotTests(ImGuiTestEngine* engine);
void RegisterVisualTests(ImGuiTestEngine* engine);
void RegisterBgOverlayTests(ImGuiTestEngine* engine);
void RegisterImportExportTests(ImGuiTestEngine* engine);
void RegisterCalibrationTests(ImGuiTestEngine* engine);
void RegisterPerfTests(ImGuiTestEngine* engine);
void RegisterP1InteractionTests(ImGuiTestEngine* engine);
void RegisterP1SliderBoundaryTests(ImGuiTestEngine* engine);
void RegisterP2InteractionRenderTests(ImGuiTestEngine* engine);
void RegisterP1RunningTests(ImGuiTestEngine* engine);
void RegisterP2InteractionModalTests(ImGuiTestEngine* engine);
void RegisterOverlayLabelTests(ImGuiTestEngine* engine);
void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine);
void RegisterCrystalRendererTests(ImGuiTestEngine* engine);

#endif  // LUMICE_TEST_GUI_SHARED_HPP
