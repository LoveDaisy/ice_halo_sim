#ifndef LUMICE_GUI_APP_HPP
#define LUMICE_GUI_APP_HPP

#include "gui/crystal_renderer.hpp"
#include "gui/gui_state.hpp"
#include "gui/preview_renderer.hpp"
#include "gui/server_poller.hpp"
#include "include/lumice.h"

struct GLFWwindow;

namespace lumice::gui {

// Layout constants
constexpr int kInitWindowWidth = 1280;
constexpr int kInitWindowHeight = 720;
constexpr int kMinWindowWidth = 800;
constexpr int kMinWindowHeight = 600;
constexpr float kLeftPanelWidth = 380.0f;
constexpr float kTopBarHeight = 40.0f;
constexpr float kStatusBarHeight = 28.0f;

// Live-edit timing constants
constexpr int kTimingIntervalMs = 100;                // Base interval for commit and poll (ms)
constexpr int kCommitIntervalMs = kTimingIntervalMs;  // Min interval between auto-commits (T_commit)
constexpr int kPollIntervalMs = kTimingIntervalMs;    // Server poll interval (T_poll)

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
extern LUMICE_Server* g_server;
extern ServerPoller g_server_poller;
extern bool g_panel_collapsed;
extern PreviewViewport g_preview_vp;

// Crystal preview trackball state
extern float g_crystal_rotation[16];
extern float g_crystal_zoom;
extern int g_crystal_style;
extern int g_crystal_mesh_id;
extern int g_crystal_mesh_hash;

// Aspect ratio state
extern int g_programmatic_resize;  // Counter: decremented by WindowSizeCallback, set by ApplyAspectRatio
extern float g_aspect_bar_height;  // Cached actual height from ImGui layout

// Unsaved changes popup state
extern bool g_show_unsaved_popup;
extern PendingAction g_pending_action;

// GLFW callbacks
void GlfwErrorCallback(int error, const char* description);
void WindowSizeCallback(GLFWwindow* window, int width, int height);

// Aspect ratio helpers
float GetAspectRatio(AspectPreset preset);
void ApplyAspectRatio(GLFWwindow* window, AspectPreset preset, bool portrait, float override_ratio = 0.0f);

// Crystal preview helpers
int CrystalParamHash(const CrystalConfig& c);
void ResetCrystalView();
void ApplyTrackballRotation(float dx, float dy);

// Business operations
void DoSave();
void DoSaveAs();
void DoExportPreviewPng();
void DoOpen();
void DoNew();
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
void RenderFloatingLensBar(float window_width);
void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height);
void RenderStatusBar(float window_width, float window_height);
void RenderUnsavedPopup(GLFWwindow* window);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_APP_HPP
