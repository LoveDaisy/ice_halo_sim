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

// ========== The trailing-label column ==========
//
// Several panels build their rows on one three-column model — [wide control][value][label] — and
// the claim these helpers exist to check is that the last two boundaries are single vertical
// lines, no matter which widget family drew the row. Two families share the model and neither can
// see the other: the hand-built [slider][input] rows in panels.cpp place their own label, while
// ImGui's Combo places its label itself at a spacing constant hardcoded inside BeginCombo. A row
// family that picks the other constant lands its controls and its label on lines 4px away from
// everyone else's, which is a defect no per-family test can state.
//
// Both numbers come out of geometry ImGui reports, never out of a constant copied from the panel
// code — an assertion built from kLabelColWidth and ItemSpacing.x would restate the layout code
// rather than check it.
struct LabelColumnRow {
  const char* name = "";       // for the failure message; the row as a user would name it
  float control_right = 0.0f;  // right edge of the row's last interactive control
  float label_left = 0.0f;     // left edge of the row's trailing label
};

// A hand-built row: the InputFloat/InputInt reports its own frame, and the trailing label is
// addressable because panels.cpp submits an InvisibleButton of the label's exact horizontal
// extent over it (see TextWithLabelProbe there).
inline LabelColumnRow MeasureWidgetRow(ImGuiTestContext* ctx, const char* name, const char* input_ref,
                                       const char* label_probe_ref) {
  LabelColumnRow row;
  row.name = name;
  row.control_right = ctx->ItemInfo(input_ref).RectFull.Max.x;
  row.label_left = ctx->ItemInfo(label_probe_ref).RectFull.Min.x;
  return row;
}

// A Combo row. ImGui places a combo's trailing label itself, and there is no item to read it
// from: BeginCombo registers its item with the NAV rectangle — the drawn frame, label excluded
// (`ItemAdd(total_bb, id, &bb)`, imgui_widgets.cpp; the test engine records the third argument) —
// and never calls the test engine's label hook at all, so the label is neither addressable nor
// inside the reported rectangle. What IS pinned down is where BeginCombo puts it: at
// `bb.Max.x + style.ItemInnerSpacing.x`, a constant hardcoded in that function. So the frame's
// right edge is measured and the label's left edge is that plus the one ImGui spacing — no
// constant from the panel code enters, which is what keeps this from restating the layout code.
// (Being unable to change that constant from outside is the reason the panel had to move to it.)
inline LabelColumnRow MeasureComboRow(ImGuiTestContext* ctx, const char* name, const char* combo_ref) {
  LabelColumnRow row;
  row.name = name;
  row.control_right = ctx->ItemInfo(combo_ref).RectFull.Max.x;
  row.label_left = row.control_right + ImGui::GetStyle().ItemInnerSpacing.x;
  return row;
}

// Both columns of every row, against the first row's.
//
// Non-fatal per row on purpose: the proposition is "all the families agree", so stopping at the
// first family that does not would report one number and hide which of the others is the odd one
// — the difference between "combo rows are 4px off" and "everything but combo rows is". The
// tolerance is sub-pixel: these are positions on a pixel grid, reached by different float
// arithmetic, so what is being asserted is the same column, not the same bit pattern.
inline void CheckLabelColumn(const char* group, const std::vector<LabelColumnRow>& rows) {
  IM_CHECK_GT(rows.size(), (size_t)1);  // one row cannot disagree with anything
  constexpr float kTolPx = 0.5f;
  const LabelColumnRow& ref = rows[0];
  for (size_t i = 1; i < rows.size(); ++i) {
    if (ImFabs(rows[i].control_right - ref.control_right) > kTolPx) {
      IM_ERRORF("%s: control right edge %s=%.1f vs %s=%.1f (delta %.1f px)", group, rows[i].name, rows[i].control_right,
                ref.name, ref.control_right, rows[i].control_right - ref.control_right);
    }
    if (ImFabs(rows[i].label_left - ref.label_left) > kTolPx) {
      IM_ERRORF("%s: label left edge %s=%.1f vs %s=%.1f (delta %.1f px)", group, rows[i].name, rows[i].label_left,
                ref.name, ref.label_left, rows[i].label_left - ref.label_left);
    }
  }
}

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
void RegisterShellChromeTests(ImGuiTestEngine* engine);
void RegisterLogPanelTests(ImGuiTestEngine* engine);
void RegisterOverlayControlTests(ImGuiTestEngine* engine);
void RegisterPreviewViewportTests(ImGuiTestEngine* engine);
void RegisterPerfTests(ImGuiTestEngine* engine);
void RegisterOverlayLabelTests(ImGuiTestEngine* engine);
void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine);
void RegisterRunLifecycleTests(ImGuiTestEngine* engine);
void RegisterStatusBarTests(ImGuiTestEngine* engine);
void RegisterPreviewAnimationTests(ImGuiTestEngine* engine);
void RegisterCaptureHarnessTests(ImGuiTestEngine* engine);
void RegisterSimE2eSmokeTests(ImGuiTestEngine* engine);
void RegisterLensProjectionTests(ImGuiTestEngine* engine);
void RegisterModalLayoutTests(ImGuiTestEngine* engine);
void RegisterDefaultsPanelTests(ImGuiTestEngine* engine);
void RegisterDefaultsPanelLayoutTests(ImGuiTestEngine* engine);
void RegisterThemeCoverageTests(ImGuiTestEngine* engine);
void RegisterThemeScanTests(ImGuiTestEngine* engine);

#endif  // LUMICE_TEST_GUI_SHARED_HPP
