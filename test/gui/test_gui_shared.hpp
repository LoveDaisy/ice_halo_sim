#ifndef LUMICE_TEST_GUI_SHARED_HPP
#define LUMICE_TEST_GUI_SHARED_HPP

#include <atomic>
#include <filesystem>
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
#include "include/lumice.h"
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

// Default-framebuffer capture with a caller-selectable rectangle. Deliberately NOT merged
// with LeftPanelCaptureState: that one derives its rect from left-panel geometry constants
// (plus a runtime Retina scale) inside the hook, whereas this one takes the rect from the
// caller. Merging would mean rewriting an already-validated hook for zero new capability.
//
// Rect protocol: rect_w <= 0 (the default) captures the whole default framebuffer
// (0, 0, fb_w, fb_h); setting a non-zero rect_w/rect_h before requested.store(true)
// captures that sub-region instead (framebuffer pixels, origin bottom-left — the caller
// applies its own Retina scaling if the rect comes from ImGui coordinates).
// The "FullFrame" name reflects the current sole consumer, not a restriction.
struct FullFrameCaptureState {
  std::atomic<bool> requested{ false };
  std::atomic<bool> done{ false };
  std::vector<unsigned char> pixels;
  int width = 0;
  int height = 0;
  int rect_x = 0;
  int rect_y = 0;
  int rect_w = 0;
  int rect_h = 0;

  void Reset() {
    done.store(false);
    requested.store(false);
    pixels.clear();
    width = 0;
    height = 0;
    rect_x = 0;
    rect_y = 0;
    rect_w = 0;
    rect_h = 0;
  }
};

struct AutoEvExportState {
  gui::PreviewViewport custom_vp;
  std::filesystem::path export_path;
  std::atomic<bool> requested{ false };
  std::atomic<bool> done{ false };
  bool result = false;

  void Reset() {
    custom_vp = gui::PreviewViewport{};
    export_path.clear();
    requested.store(false);
    done.store(false);
    result = false;
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
extern FullFrameCaptureState g_fullframe_capture;
extern AutoEvExportState g_auto_ev_export;
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
extern bool g_keep_export_png;

// ========== Shared constants ==========

constexpr int kSynthTexW = 64;
constexpr int kSynthTexH = 64;

// ========== Shared helpers ==========

// Rays a finite run will actually simulate for a given SimConfig::ray_num_millions.
// Mirrors the one conversion the GUI performs when it hands the budget to the core
// (src/gui/file_io.cpp, BuildScene -> LUMICE_SceneSetSimParams): float millions,
// promoted to double, times 1e6, truncated to LUMICE_RayCount.
//
// A test that waits for a run to COMPLETE and then asserts the exact ray count needs this
// number, and there is no getter to read it back from the scene. Keeping the expression in
// one place stops the two visual-regression suites from drifting apart; it stays a mirror of
// production rather than a shared owner, which is why callers should pin ray_num_millions to
// a value whose product with 1e6 is exact in float (i.e. a dyadic fraction such as 0.375f or
// 0.4375f). At those values truncation and rounding agree, so the mirror survives a change of
// rounding policy on the production side instead of silently going red — or, worse, staying
// green by coincidence.
inline unsigned long long ExpectedSimRayNum(float ray_num_millions) {
  return static_cast<unsigned long long>(static_cast<LUMICE_RayCount>(ray_num_millions * 1e6));
}

// ========== Shared functions (defined in test_gui_main.cpp) ==========

void ResetTestState();
void InitSynthTexture();
void StartPerfSimulation();
void StopPerfSimulation();

// ========== Register function declarations ==========

void RegisterP0Tests(ImGuiTestEngine* engine);
void RegisterP1Tests(ImGuiTestEngine* engine);
void RegisterP2Tests(ImGuiTestEngine* engine);
void RegisterAspectRatioTests(ImGuiTestEngine* engine);
void RegisterExportPreviewTests(ImGuiTestEngine* engine);
void RegisterScreenshotTests(ImGuiTestEngine* engine);
void RegisterVisualTests(ImGuiTestEngine* engine);
void RegisterBgOverlayTests(ImGuiTestEngine* engine);
void RegisterImportExportTests(ImGuiTestEngine* engine);
void RegisterColorWindowTests(ImGuiTestEngine* engine);
void RegisterCalibrationTests(ImGuiTestEngine* engine);
void RegisterPerfTests(ImGuiTestEngine* engine);
void RegisterP1InteractionTests(ImGuiTestEngine* engine);
void RegisterP1SliderBoundaryTests(ImGuiTestEngine* engine);
void RegisterP2InteractionRenderTests(ImGuiTestEngine* engine);
void RegisterP1RunningTests(ImGuiTestEngine* engine);
void RegisterP2InteractionModalTests(ImGuiTestEngine* engine);
void RegisterOverlayLabelTests(ImGuiTestEngine* engine);
void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine);
void RegisterAutoEvRegressionTests(ImGuiTestEngine* engine);
void RegisterLinkedEntriesTests(ImGuiTestEngine* engine);
void RegisterLifecycleTests(ImGuiTestEngine* engine);
void RegisterCompositePreviewTests(ImGuiTestEngine* engine);
void RegisterSamplingDensityStatsTests(ImGuiTestEngine* engine);
void RegisterPreviewAnimationTests(ImGuiTestEngine* engine);
void RegisterCaptureHarnessTests(ImGuiTestEngine* engine);
void RegisterLensProjectionTests(ImGuiTestEngine* engine);
void RegisterModalLayoutTests(ImGuiTestEngine* engine);
void RegisterDefaultsPanelTests(ImGuiTestEngine* engine);
void RegisterDefaultsPanelLayoutTests(ImGuiTestEngine* engine);

#endif  // LUMICE_TEST_GUI_SHARED_HPP
