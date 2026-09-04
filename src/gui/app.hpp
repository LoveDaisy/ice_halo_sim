#ifndef LUMICE_GUI_APP_HPP
#define LUMICE_GUI_APP_HPP

#include <spdlog/sinks/basic_file_sink.h>

#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "gui/annotation_overlay_cache.hpp"
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
  // Point-to-device-pixel ratio of the window the viewport above was measured in. Published here
  // because the deferred render needs it (the FBO is device pixels, the text inside it is points)
  // and because the screenshot export reads it to build its anchors in the same space the screen
  // does — the one place that decides the two are the same.
  float dpi_scale_x = 1.0f;
  float dpi_scale_y = 1.0f;
  // Overlay label anchors for this frame, in LOGICAL POINTS relative to the viewport's top-left.
  // Published, not drawn, by RenderPreviewPanel: the labels are part of the preview image now and
  // are rasterized with it in the deferred pass, not onto an ImGui window's draw list.
  //
  // LEVEL-TRIGGERED, exactly like `active` above and for the same reason: RenderPreviewPanel runs
  // unconditionally every frame and republishes both, so a stale set can never outlive the frame
  // that made it. Written only where `active` is set true, cleared where `active` is set false.
  std::vector<CurveLabelSet> curve_labels;
};

enum class PendingAction { kNone, kNew, kOpen, kQuit };

// task-cleanup-hardening AC4 (Save-偏离-E owner ruling = 提示需 Run):
// When the user invokes Save while sim_state == kModified (config edited since
// the last committed run), the on-screen preview is not a render of the
// current config. Rather than silently serialize "stale render + fresh config
// + dirty=false" and quietly clear Modified (the pre-353.5 behavior — the
// bug), we now front a prompt: "Config has been modified since the last run.
// Run first (to render the current config), Save anyway (freeze the last
// run's preview into the .lmc), or Cancel?" The kind identifies which
// deferred save path resumes after the user picks Save anyway.
enum class PendingSaveKind { kNone, kSave, kSaveAs };

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

// task-cleanup-hardening AC4: Save-modified prompt state. Set by DoSave /
// DoSaveAs when sim_state == kModified; consumed by RenderSaveModifiedPopup.
// PerformSave / PerformSaveAs bypass the check directly — the only caller
// besides RenderSaveModifiedPopup's own "Save anyway" branch is DoSave /
// DoSaveAs themselves (non-kModified fallthrough). RenderUnsavedPopup's
// "Save" button routes through DoSave() (code-review-01 M1), so it defers to
// this popup rather than bypassing the check.
extern bool g_show_save_modified_popup;
extern PendingSaveKind g_pending_save_kind;

// Export-config-JSON overwrite prompt state. Set by RequestConfigJsonExport when the target
// already exists; consumed by RenderExportOverwriteConfirmPopup, which resolves it through
// ConfirmPendingConfigJsonExport / CancelPendingConfigJsonExport. The JSON is built BEFORE the
// prompt (it is what the user asked to export) and held here until they answer, so the document
// they confirm is the one that gets written even if the state changes underneath.
extern bool g_show_export_overwrite_confirm_popup;
extern std::filesystem::path g_pending_export_json_path;
extern std::string g_pending_export_json_content;

// Queue a user-visible warning surfaced by RenderImportWarningPopup; consecutive
// calls within one import concatenate so all offending filters are reported.
void SetImportComplexFilterWarning(const std::string& msg);

// Report whatever degraded while the personal-defaults override file was last read, through the
// same one-shot warning popup an import degradation uses. Call it immediately after a
// MakeNewDocumentState() whose result the user is actually shown.
//
// Why it is a call rather than something MakeNewDocumentState does itself: user_defaults.cpp sits
// below the app layer and must not reach up to a popup. The counters exist there; the decision to
// put them in front of a user is the app's.
void SurfaceUserDefaultsDowngrades();

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

// Aspect ratio helpers. GetAspectRatio itself lives in gui_state.hpp, next to the AspectPreset
// enum: it is a pure function of the preset and file_io.cpp's export path needs it too.
void ApplyAspectRatio(GLFWwindow* window, AspectPreset preset, bool portrait, float override_ratio = 0.0f);

// The one AnnotationOverlayCache the live preview drives (app_panels.cpp), and the label set built
// from whatever it currently holds. Exposed so the off-screen export renders the SAME circles the
// preview is showing rather than recomputing them at its own moment — an export is a picture of
// what is on screen, and a second computation could settle on a different view.
AnnotationOverlayCache& PreviewAnnotationOverlay();
AnnotationViewInput AnnotationViewInputFor(const GuiState& state, const RenderConfig& rc);
// `vp_w`/`vp_h` are the target draw list's size; the anchors are scaled from the cache's own canvas
// into it, so a caller rendering at a size the cache was not built for still gets them in the right
// place (see the function's own note on the HiDPI case).
CurveLabelSet BuildSunCirclesLabelSet(const AnnotationOverlayCache& cache, const GuiState& state, float vp_w,
                                      float vp_h);
// The coordinate grid's twin of the above: same anchors-to-draw-list conversion, reading the grid
// half of the same cache result and carrying the grid's own colour, alpha and collision group.
CurveLabelSet BuildGridLabelSet(const AnnotationOverlayCache& cache, const GuiState& state, float vp_w, float vp_h);
// The horizon's. Same conversion again, reading the horizon half of the same result. It joins the
// GRID's collision group (the horizon is the parallel at altitude 0) while carrying its own colour
// and alpha, which is why it is a third set rather than a third family folded into the grid's.
CurveLabelSet BuildHorizonLabelSet(const AnnotationOverlayCache& cache, const GuiState& state, float vp_w, float vp_h);

// Pick the coordinate grid step (in degrees) for a given FOV. Single source of truth for every
// consumer that needs to know how dense the grid is: the annotation request the preview and the
// export both build from it (via the two expansions below), the exported config's grid arrays, and
// Caller guarantees fov > 0; the function does not validate.
float ComputeGridStep(float fov);

// Expand a grid step into the explicit angle list the coordinate grid draws — the parallels
// (constant elevation) and the meridians (constant azimuth), one function each.
//
// SINGLE SOURCE, and that is the whole point of them existing as functions. Three consumers need
// the same list: the live preview's annotation request, the off-screen export's, and the exported
// config's grid.elevation / grid.longitude arrays. If any two of them re-derived the range, the
// GUI would show one set of lines and the CLI would render another — precisely the divergence the
// annotation layer exists to close, and one no parity test would attribute to this decision.
//
// The ranges are the ones the GUI has always drawn: parallels over ±80° EXCLUDING 0° (the horizon
// owns that curve and is coloured separately), meridians over the half-open (-180°, 180°] so the
// anti-meridian appears once rather than twice.
// Caller guarantees step > 0; neither function validates.
std::vector<float> ComputeGridElevationAngles(float step);
std::vector<float> ComputeGridLongitudeAngles(float step);

// Business operations
//
// DoSave / DoSaveAs — the user-invoked save entry points (menu / keyboard
// shortcut). Gate on sim_state == kModified: if the last committed run does
// not reflect the current config, queue g_show_save_modified_popup and
// return; RenderSaveModifiedPopup resumes the actual serialization if the
// user picks Save anyway. When sim_state != kModified, fall through to
// PerformSave / PerformSaveAs directly (no popup).
//
// PerformSave / PerformSaveAs — internal, do the actual RefreshCpuTextureForSave
// + SaveLmcFile + dirty=false sequence. Exposed here (not just a .cpp static)
// because RenderSaveModifiedPopup's "Save anyway" branch calls them directly
// after the user has explicitly acknowledged the kModified warning.
// RenderUnsavedPopup's "Save" button does NOT call these directly — it routes
// through DoSave()/DoSaveAs() (code-review-01 M1) so the kModified gate still
// applies; when that defers to RenderSaveModifiedPopup, the pending
// New/Open/Quit action is threaded through g_pending_action and only resumes
// once "Save anyway" actually performs the save (see app_panels.cpp).
void DoSave();
void DoSaveAs();
void PerformSave();
void PerformSaveAs();
void DoExportPreviewPng();
void DoExportDualFisheyeEqualAreaPng();
void DoExportEquirectangularPng();
void DoExportConfigJson();

// The half of DoExportConfigJson that is not the file dialog: write `json_str` to `path`, OR — when
// something is already at `path` (ConfigJsonExportNeedsOverwriteConfirm) — hold both in the pending
// slots above and raise the confirmation prompt instead. Exporting is the one way the GUI can write
// over a document it did not author, and what it writes is only what the GUI can express, so that
// write is an explicit act rather than a side effect of picking a filename.
//
// Split out from the dialog wrapper so the decision is testable without NFD (the same shape as
// BuildExportJsonOrWarn). Does nothing on an empty path.
void RequestConfigJsonExport(const std::filesystem::path& path, const std::string& json_str);

// Resolve a pending export. Confirm writes the held JSON to the held path; Cancel drops both. Both
// clear the pending slots, and both are no-ops when nothing is pending.
void ConfirmPendingConfigJsonExport();
void CancelPendingConfigJsonExport();
void DoOpen();
void DoOpen(const std::filesystem::path& path);
void DoNew();
// Restore the background image named by `state.bg_path`, degrading gracefully when it cannot
// be loaded (path deleted, moved, or on a machine that never had it): background off, a
// kMatchBg aspect preset falls back to kFree, one warning logged — never a hard failure, since
// this runs on the startup path. No-op when bg_path is empty.
//
// Called after ResetFrontendState (which clears the background) by every path that can land a
// non-empty bg_path in the state: `.lmc` open, `.json` import, and New — the last of these
// only became reachable once bg_path could arrive from the user's personal defaults.
void LoadBackgroundWithDegrade(GuiState& state);
void CalibrateQualityThreshold();
// task-metal-gui-commit-backpressure: DoRun returns `true` when this call reached
// a terminal outcome that needs NO retry next tick — EITHER it issued a
// LUMICE_CommitScene, OR it hit an unrecoverable pre-commit validation
// failure (scene overflow) where retrying would not help. (So `true` is NOT
// strictly "CommitScene was called" — the overflow branch returns true
// without reaching it.) It returns `false` ONLY when the backpressure gate
// deferred this attempt because the current Run has not yet produced its first
// consumed batch (avoids the commit-outpaces-batch starvation that made Metal
// slider drag show 0 rays); the caller should keep g_state.dirty set so the next
// tick retries with the latest values.
// Callers that mirror main.cpp's 70ms throttle (dirty-clear / restart accounting)
// MUST gate those side effects on the returned bool — see main.cpp,
// test/gui/test_gui_main.cpp, test/gui/responsiveness/test_gui_perf.cpp.
// Callers outside the auto-commit throttle (button clicks, DoOpen/DoNew paths)
// are expected to run when the current Run is not RUNNING, so the gate
// short-circuits open and they may discard the return value. This expectation is
// NOT code-enforced: a non-throttle DoRun racing into the narrow
// RUNNING-but-first-batch-not-yet-landed window would be gated and its commit
// dropped with no automatic retry. That window is transient (first batch is
// ~O(100ms)) and no current non-throttle caller is reachable during it; revisit
// (add an explicit retry or assert) if a future UI path can trigger DoRun
// mid-first-batch.
//
// task-gui-feedback-affordances Step 2 (AC3) — `user_initiated` distinguishes
// user-clicked Run (top-bar Run button, "Run first" in the Save-Modified
// popup) from the main-loop 70ms auto-commit path. When true, DoRun calls
// ClearGuiWarning() BEFORE any conditional SetGuiWarning so a persistent
// overflow condition re-opens the warning modal on every explicit Run;
// when false, SetGuiWarning's identity-dedup is preserved so a slider drag
// producing a stuck-overflow commit each 70ms tick does not reopen the modal
// (which would freeze the UI). No default value — every call site must make
// this choice explicit so a future new caller cannot silently inherit the
// wrong dedup semantics.
bool DoRun(bool user_initiated);
void DoStop();
void DoRevert();
void DoLoadBackground(GLFWwindow* window);
void DoClearBackground();
void SyncFromPoller();
void CheckUnsavedAndDo(PendingAction action);

// Single frontend-reset owner (task-command-reset-owner, backlog #5; doc §4 "文档重置 owner",
// doc §5 I-reset-complete). Each command (`DoNew` / `DoOpen(.lmc±baked/.json)` / `DoRevert`)
// declares its intent via `FrontendResetReason` and delegates ALL preview / poller-staged /
// trackball / mesh-hash / effects-baseline resets to this owner instead of hand-picking a
// subset — cures the "each command resets a different subset, forgetting an adjacent layer"
// class of bug that led to task-349.1 → 350 → 351's three-round fix cycle.
//
// Subset per reason (逐条对照 as-built 精确复刻，see plan §3):
//   kNewDocument  : ClearTexture + ClearBackground + InvalidateStagedTexture +
//                   crystal_mesh_hash=0 + trackball reset + OnLayerStructureChanged
//   kOpenBaked    : Upload{Radiance,}Texture(baked) + ClearBackground + InvalidateStagedTexture +
//                   trackball reset + OnLayerStructureChanged
//   kOpenLmcBlank : ClearTexture + ClearBackground + InvalidateStagedTexture +
//                   trackball reset + OnLayerStructureChanged
//   kOpenJson     : ClearTexture + ClearBackground + InvalidateStagedTexture +
//                   trackball reset + OnLayerStructureChanged
//   kRevert       : InvalidateEffectsBaselines + OnLayerStructureChanged
//                   (no texture / staged / trackball / mesh-hash touch — Revert is
//                    config restore, not document switch)
//
// The `baked` texture payload MUST be non-null iff `reason == kOpenBaked` (owner asserts
// this invariant; passing an inconsistent pair is a caller bug). `bg_path` restore stays in
// `DoOpen(.lmc)` handler (it is per-file DATA recovery, not a document-switch reset).
enum class FrontendResetReason {
  kNewDocument,
  kOpenBaked,
  kOpenLmcBlank,
  kOpenJson,
  kRevert,
};

// Baked-texture payload for `kOpenBaked`. `data` must remain valid for the duration of the
// `ResetFrontendState` call; the owner does not retain the pointer.
struct FrontendTexturePayload {
  const unsigned char* data;
  int width;
  int height;
  // True when `data` holds the halo's radiance alone (a v>=4 .lmc), false when the sky colour is
  // already summed into it (pre-v4). Selects the PreviewRenderer upload entry point; getting it
  // wrong paints the sky twice or not at all. See PreviewRenderer::TextureMode.
  bool radiance_only;
};

void ResetFrontendState(GuiState& state, FrontendResetReason reason, const FrontendTexturePayload* baked = nullptr);

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

// Stats-apply gate. Pure predicate mirroring ShouldUploadPayload's texture-side pattern on the
// stats side: it keys on the stats' OWN generation stamp (snap.stats_epoch), not on the bundle
// epoch, because the bundle epoch is re-stamped on every poll while carried-forward stats are not.
// Without that distinction a restart republishes the previous run's ray/crystal/sampling counts
// under the newly committed epoch and the status bar shows them. The rays > 0 lower bound is
// retained unchanged — it is what keeps a zero from overwriting a value already on screen.
//
// The epoch parameter is spelled uint64_t while PreviewSnapshot::stats_epoch it is compared against
// is unsigned long long. That split is the module's existing convention, not an oversight: snapshot
// FIELDS follow epoch/payload_epoch (unsigned long long), while predicate epoch PARAMETERS follow
// ShouldUploadPayload's display_epoch_floor (uint64_t), because the argument sites are GuiState
// members and those are uint64_t. Same underlying type on every supported platform, so nothing
// narrows. Written down here because the convention is implicit enough that reviewing it twice
// produced the wrong answer once.
bool ShouldApplyStats(const PreviewSnapshot& snap, uint64_t committed_epoch);

// Effective per-frame composite/xyz upload decision (task-345.4). Folds server-side composite
// availability (`payload_is_composite`) with the user's display-time preference
// (`show_composite_preview`) into a single boolean the upload branch consumes. Pure predicate,
// no globals/GL — mirrors ShouldUploadPayload's testable-contract convention above so the
// truth-table can be unit-tested headlessly (asserted over its whole domain in
// test/composition-correctness/gui/test_run_lifecycle_chain.cpp; the payload and display-time
// contracts it feeds are in test/unit-correctness/gui/test_composite_preview.cpp).
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

// task-gui-feedback-affordances Step 1 (AC2): decide whether the top-bar Colors
// button should render with a distinct tint. True iff at least one color class
// is configured (derived state; single source is `raypath_color.empty()` — no
// new state source per scrum-353 GUI state governance). Pure predicate; the
// caller in RenderTopBar wraps ImGui::Button with PushStyleColor when true.
bool ShouldTintColorsButton(bool raypath_color_empty);

// ---- Sampling-density readout (status bar) ----
//
// The two counters behind this readout are NOT comparable across backends: the GPU route draws one
// geometry per batch by design, so its shape count sits orders of magnitude below the CPU route's
// for the same scene. That difference IS the sampling-density difference, made visible — it is not
// a regression, and it must never be turned into a cross-backend parity assertion.
//
// This sentence is the only thing standing between a user and a false bug report, and plain tooltip
// prose is exactly what gets deleted by a passing refactor without any signal. Hence: a named
// constant, asserted by test/unit-correctness/gui/test_sampling_density_stats.cpp
// (SamplingDensity.cross_backend_note_present) to still be present in the assembled tooltip.
// Reword it freely; deleting it is the thing the test is there to catch.
inline constexpr const char* kSamplingCrossBackendNote =
    "Not comparable across backends: the GPU route reuses one\n"
    "geometry per batch by design, so shape reads lower on GPU.";

// Format one sampling counter as a density relative to the rays actually traced. Pure function;
// the whole numeric contract of the readout lives here so the render call site cannot drift.
//
//   rays == 0                -> "n/a"   (cold start: nothing has been traced yet)
//   draws == 0 && rays > 0   -> "n/a"   (would be the second division by zero, see below)
//   draws >= rays            -> "1.00/ray"          (this dimension is resampled per ray)
//   0 < draws < rays         -> "1 per 5.4 x10^6 rays"  (this dimension is barely sampled)
//
// Both zero branches are the function's own responsibility, not the caller's. The `draws == 0` one
// in particular cannot be waved off as unreachable: reaching it needs only the two-term counter
// convention in trace_backend.hpp to change, and that is a DIFFERENT module's invariant — it would
// change with no signal whatsoever arriving here.
std::string FormatSamplingDensity(LUMICE_RayCount draws, LUMICE_RayCount rays);

// Same reading without the trailing " rays" ("1 per 5.4 x10^6"). The status bar is a single
// non-wrapping SameLine run sharing one row with the Log button, and measurement put the verbose
// form 477 px wide in its worst case against a 1024 px enforced minimum window width -- it did not
// fit. The word is kept in the tooltip, which has the room. Both spellings come out of one
// implementation so the numbers cannot drift apart.
std::string FormatSamplingDensityCompact(LUMICE_RayCount draws, LUMICE_RayCount rays);

// The exact status-bar segment text, and the exact hover-tooltip text. Split out as pure functions
// so a test can pin what the status bar says without doing OCR on a screenshot, while the
// screenshot test independently pins that this text reaches actual pixels.
std::string FormatSamplingSegment(LUMICE_RayCount crystals, LUMICE_RayCount orientations, LUMICE_RayCount rays);
std::string FormatSamplingTooltip(LUMICE_RayCount crystals, LUMICE_RayCount orientations, LUMICE_RayCount rays);

// task-348.3 AC1/AC2 shared writer: toggle the user preference `show_composite_preview`.
// Called from both the top-bar Colored checkbox (app_panels.cpp; icon-only Button in
// 348.3, reverted to a plain-text Checkbox in 349.3 #4) and the in-window "Enable
// colors" checkbox (color_window.cpp) so the two write sites cannot drift. Read side
// stays split (both sites read the ground truth `last_uploaded_as_composite` for
// their display state — see 345.4 read/write split contract).
void ToggleCompositePreview(GuiState& state);

// Panel rendering
void RenderTopBar(float window_width);
void RenderLeftPanel(float window_height);
void RenderRightPanel(GLFWwindow* window, float window_width, float window_height);
void RenderPreviewPanel(GLFWwindow* window, float window_width, float window_height);
void RenderStatusBar(float window_width, float window_height);
void RenderUnsavedPopup(GLFWwindow* window);
// task-cleanup-hardening AC4: Save-modified prompt. Rendered once per frame
// after RenderUnsavedPopup. Opens iff g_show_save_modified_popup is true;
// resumes the deferred save according to g_pending_save_kind on "Save anyway",
// or clears the pending state on "Cancel". A "Run first" button (when a live
// server exists and no run is inflight) invokes DoRun so the user can produce
// a fresh render matching the current config, then re-invoke Save.
void RenderSaveModifiedPopup(GLFWwindow* window);
void RenderImportWarningPopup();
// What that modal has to say, and why it is not a generic "file exists, overwrite?": the file being
// replaced may be a core config the GUI merely read, and what goes back is the GUI's own re-emission
// of it — everything the GUI cannot express is gone from the copy on disk. "Overwrite?" asks about a
// filename; the user has to be asked about the content.
//
// Named and declared here rather than written inline at the ImGui call so the wording is one thing
// that can be asserted (ConfigJsonExportContractChain.TheOverwritePromptSaysWhatIsLost). That test
// pins the sentence; the gui_test case pins that the modal renders at all. What neither can state is
// the join — the renderer's single use of this constant is what makes it one proposition.
extern const char* const kExportOverwriteWarningText;

// Opens iff g_show_export_overwrite_confirm_popup is true. Must be called every frame, outside any
// Begin/End block, like the other Render*Popup above it — an export that raised the prompt and
// found nobody rendering it would sit pending forever, which is the same as losing the command.
void RenderExportOverwriteConfirmPopup();
// Generic GUI warning modal (see app_panels.cpp). SetGuiWarning queues a message (idempotent
// while the same message is in-flight, so a persistent condition re-detected every debounced
// commit does not re-spam the modal). ClearGuiWarning re-arms it (called on a successful
// commit). RenderGuiWarningPopup shows it. Used e.g. when a filter edit exceeds the ABI bounds.
void SetGuiWarning(const std::string& msg);
void ClearGuiWarning();
std::string PeekGuiWarning();  // test accessor: current in-flight message ("" if none)
// task-gui-feedback-affordances Step 2 (AC3) test accessor: is a modal re-open
// pending on the next RenderGuiWarningPopup? True between SetGuiWarning (fresh
// or after ClearGuiWarning) and the frame that consumes it via OpenPopup. Lets
// tests observe DoRun(user_initiated=true)'s "force-reopen on every explicit
// Run" contract without spinning up a full ImGui frame.
bool IsGuiWarningPending();

namespace internal_test {
// task-gui-feedback-affordances Step 2 (AC3) test helper: consume the pending
// modal-open flag exactly as RenderGuiWarningPopup would on the next frame,
// without touching the in-flight message. Lets AC3 tests exercise the
// "OK dismissed the popup but the persistent overflow remains" state without
// running a full ImGui frame that would fight the popup's input capture.
void ConsumeGuiWarningPending();
}  // namespace internal_test
void RenderGuiWarningPopup();
void RenderLogPanel(float window_width, float window_height);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_APP_HPP
