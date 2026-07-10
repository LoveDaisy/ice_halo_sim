#ifndef LUMICE_GUI_APP_HPP
#define LUMICE_GUI_APP_HPP

#include <spdlog/sinks/basic_file_sink.h>

#include <atomic>
#include <filesystem>
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
// Tracks whether the live g_server was constructed for a GPU backend (Metal/CUDA)
// vs CPU (see app.cpp). Reset to false by any code that creates g_server directly
// via LUMICE_CreateServer (CPU default) outside MaybeReconstructServerForBackend —
// e.g. the perf-test harness — to keep the toggle-detection invariant honest.
extern bool g_server_is_gpu;

// Async Stop completion latch (blueprint §5/§8, 1.6). Set true synchronously by DoStop when it
// offloads the blocking `poller.Stop() + LUMICE_StopServer` sequence onto a background std::async
// thread; cleared by that thread when the backend has drained. Read (never written) by
// SyncFromPoller to advance run_intent kStopping→kStopped. JoinPendingStop() blocks until the
// background thread has returned and MUST run before any path that destroys/reconstructs the
// server or poller (use-after-free guard, R1).
extern std::atomic<bool> g_stop_inflight;
void JoinPendingStop();

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
void DoOpen(const std::filesystem::path& path);
void DoNew();
void CalibrateQualityThreshold();
void DoRun();
void DoStop();
void DoRevert();
void DoLoadBackground(GLFWwindow* window);
void DoClearBackground();
void SyncFromPoller();
void CheckUnsavedAndDo(PendingAction action);

// Single-owner sim_state reconcile (I2, blueprint §4/§5). Pure function of the last user intent,
// the epoch the GUI committed, the last backend observation (may be null before the first poll),
// and whether the config is dirty. No globals / GL / server access — declared here (not in a .cpp
// anonymous namespace) so the §1.3 truth table can be unit-tested directly. Called once per frame
// by SyncFromPoller as the ONLY sim_state writer.
GuiState::SimState ReconcileSimState(RunIntent intent, uint64_t committed_epoch, const PreviewSnapshot* snap,
                                     bool dirty);

// Display upload gate (blueprint §7 / I1/I6). Pure predicate deciding whether a poller snapshot's
// payload should be uploaded to GL this frame: it must be a fresh (unseen serial) non-empty payload
// whose epoch clears the display_epoch_floor. Declared here so the anti-flicker mechanism (§3.3) is
// headless-testable without a GL context.
bool ShouldUploadPayload(const PreviewSnapshot& snap, unsigned long long last_uploaded_texture_serial,
                         uint64_t display_epoch_floor);

// Effective per-frame composite/xyz upload decision (task-345.4). Folds server-side composite
// availability (`payload_is_composite`) with the user's display-time preference
// (`show_composite_preview`) into a single boolean the upload branch consumes. Pure predicate,
// no globals/GL — mirrors ShouldUploadPayload's testable-contract convention above so the
// truth-table can be unit-tested headlessly (see test_gui_composite_preview.cpp).
bool ShouldUseCompositeUpload(bool payload_is_composite, bool show_composite_preview);

// task-345.4 fire-branch gate: single predicate that both the SyncFromPoller upload branch AND
// the regression test consume, so the test proves the exact decision production uses. Returns
// true when the upload branch should fire this frame, which is EITHER the standard serial-dedup
// gate (fresh snapshot, epoch clears the floor) OR a display-mode flip that alone produces no
// new poller snapshot (the OR-branch — see SyncFromPoller comment). Pure predicate, no globals.
// SyncFromPoller cannot be driven end-to-end from a functional-test coroutine (no GL context —
// the g_preview.Upload*Texture() call in the branch is a GL call and SIGILLs on the worker
// thread), so a headless regression pins the decision at THIS seam and leaves the actual GL
// upload to the on-screen visual/ integration tests + owner AC5.
bool ShouldFireCompositeUpload(const PreviewSnapshot& snap, unsigned long long last_uploaded_texture_serial,
                               uint64_t display_epoch_floor, bool show_composite_preview,
                               bool last_uploaded_as_composite);

// task-348.3 AC3 (⑦): decide whether opening the Colors window should force
// show_composite_preview=true. True only when no color classes exist yet (nothing
// to remember, default to "on" so a newly-added first class is visible immediately);
// false when classes already exist (memory — caller must leave the existing preference
// untouched). Pure predicate; the caller is responsible for applying it ONLY on the
// false→true color_window_open transition (see RenderTopBar), not per-frame.
bool ShouldDefaultEnableColorsOnOpen(bool raypath_color_empty);

// task-348.3 AC1/AC2 shared writer: toggle the user preference `show_composite_preview`.
// Called from both the top-bar Colored button (app_panels.cpp) and the in-window
// "Enable colors" checkbox (color_window.cpp) so the two write sites cannot drift.
// Read side stays split (both sites read the ground truth `last_uploaded_as_composite`
// for their display state — see 345.4 read/write split contract).
void ToggleCompositePreview(GuiState& state);

// Panel rendering
void RenderTopBar(float window_width);
void RenderLeftPanel(float window_height);
void RenderRightPanel(GLFWwindow* window, float window_width, float window_height);
void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height);
void RenderStatusBar(float window_width, float window_height);
void RenderUnsavedPopup(GLFWwindow* window);
void RenderImportWarningPopup();
// Generic GUI warning modal (see app_panels.cpp). SetGuiWarning queues a message (idempotent
// while the same message is in-flight, so a persistent condition re-detected every debounced
// commit does not re-spam the modal). ClearGuiWarning re-arms it (called on a successful
// commit). RenderGuiWarningPopup shows it. Used e.g. when a filter edit exceeds the ABI bounds.
void SetGuiWarning(const std::string& msg);
void ClearGuiWarning();
std::string PeekGuiWarning();  // test accessor: current in-flight message ("" if none)
void RenderGuiWarningPopup();
void RenderLogPanel(float window_width, float window_height);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_APP_HPP
