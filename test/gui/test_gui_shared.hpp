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

// The one main-thread request the background suite (functional/test_background_overlay.cpp, its
// only consumer) cannot express any other way: decoding an image file and handing it to GL. It
// carried its own export fields until the shared RequestAndWaitPreviewExport helper existed, and
// that suite now exports through the helper; a second, private copy of that protocol is exactly the
// duplication the helper was promoted to remove.
struct BgOverlayTestState {
  std::string bg_image_path;
  bool bg_upload_requested = false;
  bool bg_upload_done = false;

  void Reset() {
    bg_image_path.clear();
    bg_upload_requested = false;
    bg_upload_done = false;
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

// Absolute path for a scratch file this suite writes (a capture PNG, a round-tripped .lmc, an
// exported JSON). Callers pass a bare filename; this decides the directory.
//
// It exists because a hardcoded "/tmp/..." is not a path on Windows. That is not hypothetical
// here: a gui_unit_test case that hardcoded one died on its first assertion for its entire life
// on Windows, and only surfaced because that target runs in CI on three platforms. gui_test's
// equivalents did not surface because gui_test runs in CI on one.
//
// The directory is, in order: --export-dir if given, else a per-process subdirectory of
// std::filesystem::temp_directory_path(). The per-process suffix is what makes two concurrent
// gui_test processes safe to run — a fixed filename under a shared directory is a data race
// between shards, and sharding the correctness pool is the reason this suite's runtime is worth
// attacking at all. The directory is created on first use.
//
// scripts/regen_gui_test_refs.py passes --export-dir explicitly and collects from there. It used
// to reconstruct "/tmp/<tmp_prefix><key>.png" from its own GROUPS table, i.e. it kept a second
// copy of a path the test picked; now it dictates the directory and only the FILENAME is shared
// (still "<tmp_prefix><key>.png", so the GROUPS table's meaning is unchanged).
std::filesystem::path GuiTestTempPath(const std::string& filename);

// ========== Shared functions (defined in test_gui_main.cpp) ==========

void ResetTestState();
// Pump frames until texture_upload_count has advanced past `baseline_upload_count`, or the
// wall-clock budget runs out. Returns whether it advanced.
//
// The sleep between yields is load-bearing and is why the budget is wall clock rather than a frame
// count: the poller runs on its own thread, and under --fixed-dt the frame pump would otherwise
// spin through the whole budget in less real time than one poll interval. That is also what keeps
// callers in the fast correctness pool instead of the real-timing one.
//
// Shared rather than copied: it began as a TU-private static in one suite, was copied into a
// second with a note saying the sibling was private and this one had a single caller, and the
// run-lifecycle suite is the third consumer.
bool WaitForSimRestartAtLeast(ImGuiTestContext* ctx, unsigned long long baseline_upload_count, int timeout_ms = 1500);
// Drive the main loop's off-screen preview-export hook (g_auto_ev_export, serviced in
// test_gui_main.cpp) with a caller-built viewport and wait for the FBO readback to land at
// `path`. Returns false if the readback did not complete within the poll budget or the
// export itself failed.
//
// Shared rather than copied: it started as a local static in the auto_ev suite, was copied
// once into the lens-projection suite with a note to promote it when a third caller
// appeared, and the sim-smoke suite is that third caller.
bool RequestAndWaitPreviewExport(ImGuiTestContext* ctx, const gui::PreviewViewport& vp, const std::string& path);
void InitSynthTexture();
void StartPerfSimulation();
void StopPerfSimulation();

// ===== A table-driven loop stops at its first failing row. Why, and what to write =====
//
// A non-fatal IM_ERRORF is not "IM_CHECK that keeps going". It routes through the same
// ImGuiTestEngine_Check as a fatal one and sets the test's status to Error just the same; the only
// thing it skips is the `return`. And every ImGuiTestContext action — ItemClick, ItemInputValue,
// ComboClick, ItemInfo, ItemExists, all of them — opens with `if (IsError()) return;`. So from the
// first reported row onward, a loop that keeps going drives nothing and reads nothing: it compares
// the state the GUI was left in, reports that too, and hands whoever reads the run a table of reds
// in which only the first is a cause and the rest are its shadow.
//
// So the reason to prefer IM_ERRORF over IM_CHECK in such a loop is the MESSAGE — naming the row,
// the value it landed on and the value expected, instead of "false is not true". It is not the
// sweep. Every loop here that drives ctx and reports non-fatally therefore ends with
//
//     if (ctx->IsError()) {
//       break;
//     }
//
// once per loop level, nesting included: breaking an inner loop still leaves the outer one driving.
// A loop that reports over data already in hand (comparing 16 floats of a pose matrix, say) needs
// none of this and should not have it — it really does report every bad element.
//
// The trigger is REPETITION, not the `for` keyword, and the difference is worth stating because a
// review pass already missed it once. A named lambda holding the body, called from a loop or called
// twice in a row —
//
//     cancel_by(/*by_escape=*/true);
//     cancel_by(/*by_escape=*/false);
//
// — is the same machine with one level of indirection: the first call reports the real failure, and
// every action in the second one silently does nothing, so the second call fails on a premise its
// own setup never got to establish. The guard goes at the CALL SITE (`return` between sequential
// calls, `break` in a loop) rather than at the top of the lambda: an early return inside it would
// stop the echo it produces itself, but not the caller's next fatal IM_CHECK reading state that was
// never driven. A fatal IM_CHECK inside a lambda has the same reach, since it returns only that
// lambda — which is why a lambda holding no IM_ERRORF at all still needs the guard around it.
//
// scripts/check_loop_fatal_asserts.py is the mechanical half of the same subject, from the other
// side: a FATAL assert in a loop body, which ends the whole case at the first bad row without
// saying so. The two rules point the same way — a table-driven loop reports one row and stops —
// and the script covers both this non-fatal-cascade half (a for-loop-nonfatal-cascade rule for the
// `for` shape, a lambda-call-cascade rule for the named-lambda-called-twice-in-a-row shape) and the
// fatal one.

// Whether the item ImGui submitted is greyed out.
//
// Declared here and defined in test_gui_main.cpp rather than written inline: ImGuiItemFlags_Disabled
// lives in imgui_internal.h, and a header every suite includes is the wrong place to drag that in.
// Ten functional suites had a byte-identical copy of this one line in their own anonymous
// namespace, which is ten places to remember the day the flag needs company (ImGuiItemFlags_
// ReadOnly, say) for the predicate to still mean "the user cannot operate this".
bool IsDisabled(const ImGuiTestItemInfo& info);

// Read the live rectangle of a named window out of the DEFAULT framebuffer, through
// g_fullframe_capture's sub-region protocol. The rectangle is clamped to the framebuffer, so a
// chrome window that overhangs an edge yields the part of it that was on screen. Returns false if
// the window is not submitted this frame, if the clamped rectangle is empty, or if the readback did
// not land.
//
// Output is RGBA, four bytes per pixel, origin bottom-left, sized in FRAMEBUFFER pixels — which is
// twice the ImGui size on a Retina display, so a caller indexing into it must use *out_w / *out_h
// rather than the window's ImGui size. The coordinate flip and the Retina scaling are the whole
// reason this is shared: the same arithmetic is open-coded at several capture sites, and getting
// the y flip subtly wrong yields a plausible-looking capture of the wrong band. New capture sites
// should call this rather than write it again.
//
// The caller is responsible for settling the frame first (and for parking the mouse if a tooltip
// would otherwise land in the rectangle).
bool CaptureWindowRect(ImGuiTestContext* ctx, const char* window_name, std::vector<unsigned char>* out_pixels,
                       int* out_w, int* out_h);

// Hands ImGui's popup stack back when a case leaves, by whichever exit.
//
// ResetTestState() reaches everything about a popup this suite opens except the one piece ImGui
// owns. Clearing the drawing side's statics stops the popup being submitted, but the entry
// OpenPopup pushed onto ImGuiContext::OpenPopupStack when it opened stays there, keyed to a window
// nobody draws any more — so IsPopupOpen, GetTopMostPopupModal and FindBlockingModal all keep
// answering for it in every case that runs afterwards. That entry has no other owner: the next
// case's ResetTestState() rebuilds GuiState and the modal statics and never touches ImGui's stack.
//
// Applies to all three popups this suite opens, not only the biggest one: the Edit Entry modal and
// the custom-spectrum editor are BeginPopupModal (so a leaked one also blocks input), and the sun-
// circles angle editor is a plain BeginPopup (which does not block, but leaves the same stack
// entry). One guard for all three rather than three near-identical objects — a case that opens any
// of them declares one of these.
//
// It has to be an object rather than statements at the end of the case body, for two mechanical
// reasons rather than as a matter of taste:
//   - a fatal IM_CHECK expands to `return` in the ENCLOSING function, so tail teardown runs only
//     when the case passes — and the assertions it sits behind are exactly the ones a real
//     regression trips;
//   - every ImGuiTestContext action (ItemClick, ItemClose, WindowMove, …) opens with
//     `if (IsError()) return;`, so `ctx->ItemClick(Cancel)` is already a no-op by the time a failed
//     case would reach it. Writing the teardown after the assert would not have helped. Note this
//     also applies after a NON-fatal IM_ERRORF, which sets the same error status without returning.
//     ctx->Yield() is the one exception — it pumps frames unconditionally, which is why the
//     destructor can still let the closing frame run.
struct ScopedPopups {
  explicit ScopedPopups(ImGuiTestContext* ctx) : ctx_(ctx) {}
  ~ScopedPopups();

  ScopedPopups(const ScopedPopups&) = delete;
  ScopedPopups& operator=(const ScopedPopups&) = delete;

 private:
  ImGuiTestContext* ctx_;
};

// Put the document inspector's crystal page on an entry, showing one of its three tabs.
//
// Every suite that used to reach the crystal / axis / filter editors did it by clicking an "Edit"
// button on an entry card, which opened a modal. Both are gone: the editors are the inspector's
// crystal page, and what decides which entry they edit is the tree's selection
// (doc/gui-layout-architecture.md §2). There is no dismissal counterpart on purpose — the page is
// always up and an edit is in the document the frame it is made.
//
// Shared rather than copied into each suite because five of them need it and the sequence is not
// obvious: the selection has to be pumped before the tab click, since the tab does not exist until
// the page is rendering.
void OpenEntryTab(ImGuiTestContext* ctx, int layer_idx, int entry_idx, const char* tab_ref);
void OpenCrystalTab(ImGuiTestContext* ctx, int layer_idx = 0, int entry_idx = 0);
void OpenAxisTab(ImGuiTestContext* ctx, int layer_idx = 0, int entry_idx = 0);
void OpenFilterTab(ImGuiTestContext* ctx, int layer_idx = 0, int entry_idx = 0);

// Path prefix of a display strip TAB BUTTON, for the one case that has to point at a tab rather
// than at something inside it. Note the second segment: BeginTabBar pushes an override ID, so a tab
// item's id is hashed under the tab bar's, NOT under the window's — "//##DisplayStrip/Grade" finds
// nothing, and finds it silently (ItemExists just answers false).
inline constexpr const char* kDisplayStripTabPrefix = "//##DisplayStrip/##DisplayStripTabs/";

// Select one of the display strip's tabs ("Grade" / "Overlays" / "Components") and pump the frames
// its contents need to be submitted.
//
// A case that reads anything in the strip has to call this first, and the reason is stronger than
// convenience: ImGui does not submit an unselected tab's contents at all, so those items do not
// exist rather than merely sitting out of view — `!ItemExists` would then be satisfied by "another
// tab is showing" as readily as by "the control is not offered", which is what several of these
// cases are actually asserting. ResetTestState puts the strip back on Grade, so a case that wants
// Grade need not call this; one that wants any other tab must.
void OpenDisplayStripTab(ImGuiTestContext* ctx, const char* tab_label);

// The document column's two halves are docked windows a few hundred pixels tall showing content
// that is routinely taller — a Pyramid with its Face Distance section expanded, a scene with more
// rows than the tree's half can show — so both scroll, by design. The two helpers below find an
// item in one of them regardless of where it currently sits.
//
// WHY A SCROLLED-AWAY ITEM CANNOT BE FOUND AT ALL, which is not obvious and is stronger than "it is
// off screen". A `**/name` lookup is resolved by LABEL, and a widget hands its label to the test
// engine on its last line — `IMGUI_TEST_ENGINE_ITEM_INFO(id, label, ...)` — which is AFTER the
// early-out it takes when its rectangle does not overlap the clip rect (see the tail of
// ImGui::Selectable, and every other widget built the same way). So for a clipped item the engine
// learns the id but never the label, and ItemInfo / ItemExists / ItemClick alike answer "no such
// item" rather than "not visible". Table rows are the same story one level up and worse: ImGui
// culls the ROW before submitting its contents, so the widgets are never created either.
//
// The consequence worth remembering: this affects the ACTION verbs too, not only the read-only
// queries. ItemClick scrolls to a target it has found; it cannot find one it has never been told
// the name of. The modal these pages replaced never hit any of this because it sized its content
// pane to fit its tallest tab — a luxury a docked column does not have.
ImGuiTestItemInfo InspectorItemInfo(ImGuiTestContext* ctx, const char* ref);
bool InspectorItemExists(ImGuiTestContext* ctx, const char* ref);

// Scroll the inspector until `ref` resolves and LEAVE it there, so an ItemClick / ItemInputValue
// can follow. Returns whether it resolved. The action-side counterpart to InspectorItemInfo — same
// relationship ScrollTreeTo has to the tree — and the reason it is separate is the last paragraph
// above: the action verbs cannot reach an item they were never told the name of, so a case that
// drives a control near the bottom of a page has to bring it on screen first rather than assume
// it. Deliberately does NOT restore the scroll, since its whole purpose is to set up an action.
bool ScrollInspectorTo(ImGuiTestContext* ctx, const char* ref);

// Open the combo at `combo_ref` and click `item_label` inside its popup.
//
// Two quirks make this a hand-rolled helper rather than ImGuiTestContext::ComboClick. That one
// splits its argument at the FIRST '/' and takes everything before it as the combo, so it can only
// address a combo that is a direct child of the current ref — which excludes every combo inside a
// PushID scope (the Colors window) or inside a property table (every inspector page). And
// `combo_ref` must be a LITERAL path, never a "**/" wildcard: ImGui::BeginCombo never calls
// IMGUI_TEST_ENGINE_ITEM_INFO, so a combo has no registered debug label and a wildcard search,
// which matches by label, can never find one.
//
// The ref is left where it was found, so a caller that had set one up keeps it.
void ComboPick(ImGuiTestContext* ctx, const char* combo_ref, const char* item_label);

// The tree half's scrolling region. Named here because the rows live in a child window, so the
// scroll that matters is the child's, not the tree window's.
inline constexpr const char* kTreeScrollRef = "##DocumentTree/##TreeScroll";

// Scroll the tree until `ref` resolves and LEAVE it there, so an ItemClick can follow. Returns
// whether it resolved. The counterpart to InspectorItemInfo for the other half — and unlike it,
// this one does not restore the scroll, because its whole purpose is to set up an action.
bool ScrollTreeTo(ImGuiTestContext* ctx, const char* ref);

// ========== Register function declarations ==========

void RegisterViewDisplayControlTests(ImGuiTestEngine* engine);
void RegisterExportPreviewTests(ImGuiTestEngine* engine);
void RegisterPreviewPixelTests(ImGuiTestEngine* engine);
void RegisterPreviewTextureTests(ImGuiTestEngine* engine);
void RegisterEntryManagementTests(ImGuiTestEngine* engine);
void RegisterBackgroundOverlayTests(ImGuiTestEngine* engine);
void RegisterFileOpsTests(ImGuiTestEngine* engine);
void RegisterColorWindowTests(ImGuiTestEngine* engine);
void RegisterFilterEditorTests(ImGuiTestEngine* engine);
void RegisterEditModalTests(ImGuiTestEngine* engine);
void RegisterSceneControlTests(ImGuiTestEngine* engine);
void RegisterExecutionClusterTests(ImGuiTestEngine* engine);
void RegisterShellChromeTests(ImGuiTestEngine* engine);
void RegisterDocumentColumnTests(ImGuiTestEngine* engine);
void RegisterInspectorNoHScrollTests(ImGuiTestEngine* engine);
void RegisterPropertyRowTests(ImGuiTestEngine* engine);
void RegisterLogPanelTests(ImGuiTestEngine* engine);
void RegisterOverlayControlTests(ImGuiTestEngine* engine);
void RegisterPreviewViewportTests(ImGuiTestEngine* engine);
void RegisterPerfTests(ImGuiTestEngine* engine);
void RegisterOverlayLabelTests(ImGuiTestEngine* engine);
void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine);
void RegisterRunLifecycleTests(ImGuiTestEngine* engine);
void RegisterStatusBarTests(ImGuiTestEngine* engine);
void RegisterThemeCoverageTests(ImGuiTestEngine* engine);
void RegisterPreviewAnimationTests(ImGuiTestEngine* engine);
void RegisterCaptureHarnessTests(ImGuiTestEngine* engine);
void RegisterSimE2eSmokeTests(ImGuiTestEngine* engine);
void RegisterLensProjectionTests(ImGuiTestEngine* engine);
void RegisterDefaultsPanelTests(ImGuiTestEngine* engine);
void RegisterDefaultsPanelLayoutTests(ImGuiTestEngine* engine);
void RegisterCrystalInspectorLayoutTests(ImGuiTestEngine* engine);
void RegisterDisplayStripLayoutTests(ImGuiTestEngine* engine);

#endif  // LUMICE_TEST_GUI_SHARED_HPP
