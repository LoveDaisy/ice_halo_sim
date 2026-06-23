#ifndef LUMICE_GUI_APP_HPP
#define LUMICE_GUI_APP_HPP

#include <spdlog/sinks/basic_file_sink.h>

#include <memory>

#include "gui/crystal_preview.hpp"
#include "gui/crystal_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/log_sink.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"
#include "gui/server_poller.hpp"
#include "gui/thumbnail_cache.hpp"
#include "include/lumice.h"

struct GLFWwindow;

namespace lumice::gui {

// Stored preview viewport for deferred rendering (after ImGui::Render)
struct PreviewViewport {
  bool active = false;
  int vp_x = 0;
  int vp_y = 0;
  int vp_w = 0;
  int vp_h = 0;
  PreviewParams params;
};

enum class PendingAction { kNone, kNew, kOpen, kQuit };

// Global state — accessible for test engine
extern GuiState g_state;
extern PreviewRenderer g_preview;
extern CrystalRenderer g_crystal_renderer;
extern ThumbnailCache g_thumbnail_cache;
extern LUMICE_Server* g_server;
extern ServerPoller g_server_poller;
extern PreviewViewport g_preview_vp;
#if defined(__APPLE__)
// Tracks the backend the live g_server was constructed for (see app.cpp). Reset to
// false by any code that creates g_server directly via LUMICE_CreateServer (CPU
// default) outside MaybeReconstructServerForBackend — e.g. the perf-test harness —
// to keep the toggle-detection invariant honest.
extern bool g_server_is_metal;
#endif

// Aspect ratio state
extern int g_programmatic_resize;  // Counter: decremented by WindowSizeCallback, set by ApplyAspectRatio

// Unsaved changes popup state
extern bool g_show_unsaved_popup;
extern PendingAction g_pending_action;

// Queue a user-visible warning surfaced by RenderImportWarningPopup; consecutive
// calls within one import concatenate so all offending filters are reported.
void SetImportComplexFilterWarning(const std::string& msg);

// Test-only: read the pending warning text without opening the modal.
std::string PeekImportComplexFilterWarning();

// Test-only: clear any queued warning (live path auto-consumes via the popup).
void ClearImportComplexFilterWarning();

// Log sinks for GUI log panel
extern std::shared_ptr<ImGuiLogSink> g_imgui_log_sink;
extern std::shared_ptr<spdlog::sinks::basic_file_sink_mt> g_file_log_sink;
extern std::string g_log_file_path;

// GLFW callbacks
void GlfwErrorCallback(int error, const char* description);
void WindowSizeCallback(GLFWwindow* window, int width, int height);

// Aspect ratio helpers
float GetAspectRatio(AspectPreset preset);
void ApplyAspectRatio(GLFWwindow* window, AspectPreset preset, bool portrait, float override_ratio = 0.0f);

// Build an OverlayLabelInput from GuiState + RenderConfig. Shared between
// RenderPreviewPanel (live preview) and DoExportPreviewPng (off-screen FBO export)
// so both paths consume the same field-packing logic.
OverlayLabelInput BuildOverlayLabelInput(const GuiState& state, const RenderConfig& rc);

// Pick the coordinate grid step (in degrees) for a given FOV. Single source of
// truth shared between shader uniform (OverlayDecoration::grid_step) and label
// CPU loop (OverlayLabelInput::grid_step) so the two never drift.
// Caller guarantees fov > 0; the function does not validate.
float ComputeGridStep(float fov);

// Business operations
void DoSave();
void DoSaveAs();
void DoExportPreviewPng();
void DoExportDualFisheyeEqualAreaPng();
void DoExportEquirectangularPng();
void DoExportConfigJson();
void DoOpen();
void DoNew();
void CalibrateQualityThreshold();
void DoRun();
void DoStop();
void DoRevert();
void DoLoadBackground(GLFWwindow* window);
void DoClearBackground();
void SyncFromPoller();
void CheckUnsavedAndDo(PendingAction action);

// Panel rendering
void RenderTopBar(float window_width);
void RenderLeftPanel(float window_height);
void RenderRightPanel(GLFWwindow* window, float window_width, float window_height);
void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height);
void RenderStatusBar(float window_width, float window_height);
void RenderUnsavedPopup(GLFWwindow* window);
void RenderImportWarningPopup();
void RenderLogPanel(float window_width, float window_height);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_APP_HPP
