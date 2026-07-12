#ifndef LUMICE_GUI_COLOR_WINDOW_HPP
#define LUMICE_GUI_COLOR_WINDOW_HPP

// task-342.3 Step 5-9: floating (non-modal) "Colors" window for editing the
// raypath color class pool.
//
// Design overview (see scratchpad/scrum-raypath-color-design2/task-gui-color-window/plan.md):
//   - Display-time edits (color / visible / solo / z_order / composite mode)
//     call LUMICE_SetRaypathColors directly — no epoch bump, no re-simulation.
//   - Structural edits (predicate text / combine any↔all / add/remove class or
//     ref) call GuiState::MarkStructHardDirty(), which the main-loop debounce
//     will pick up on its next FillLumiceConfig + CommitConfigStruct pass.
//   - z_order and the physical vector index are strictly decoupled: reorder
//     swaps z_order values only; the underlying vector position stays put so
//     GetColorClassLaneY(i) keeps binding to the same class.

#include <cstddef>
#include <vector>

#include "gui/raypath_segments.hpp"
#include "include/lumice.h"

namespace lumice::gui {

struct GuiState;
struct FilterConfig;
struct ColorClassConfig;

// Render the non-modal Colors window (Layer 3, floating). No-op when
// state.color_window_open is false.
void RenderColorWindow(GuiState& state, LUMICE_Server* server);

// task-color-migration (T1) — export the display-state push so the frame-tail
// reconciler (gui_state_reconcile.cpp) and DoRun/DoRevert repush discipline can
// drive it. Contract: server MUST be non-null; state.raypath_color may be empty
// (early-return false). Rebuilds the whole {classes[], z_order[]} arrays and
// calls LUMICE_SetRaypathColors + WakeForRefresh on the server poller. Returns
// true iff LUMICE_SetRaypathColors returned LUMICE_OK. On failure (settling
// window with class-count mismatch), the caller MUST NOT update its "last
// pushed" baseline — the next reconcile diff retries. See plan.md §3 D3.
bool PushDisplayState(const GuiState& state, LUMICE_Server* server);

// --- Pure helpers, exposed for direct unit-tests (Step 10). ---
//
// Swap the z_order values of two physical class slots (identified by their
// vector index). Never swaps the vector entries themselves — plan §3 decision 1
// requires the physical index to stay pinned so GetColorClassLaneY(i) keeps
// binding to the same class. No-op on out-of-range or self-swap.
void SwapZOrder(GuiState& state, size_t a, size_t b);

// Reassign z_order[] to a compact permutation [0, size) that preserves the
// current user-visible ordering. Called after delete-class (which leaves a
// hole in z_order values) to keep the "z_order is a permutation" invariant
// expected by LUMICE_SetRaypathColors.
void CompactZOrder(GuiState& state);

// Build a fresh ColorClassConfig from a physical filter's SoP. Single-factor
// rows become refs (whole-crystal iff row.text is empty); multi-factor AND
// rows are skipped and counted in `skipped_rows` (a LUMICE_ColorPredicate is a
// single atom; cross-ref combine:all is a separate mechanism).
// combine defaults to LUMICE_COLOR_COMBINE_ANY (blueprint case B: one filter
// UI-row imports to one class with rows OR'd).
ColorClassConfig BuildClassFromFilter(int layer_idx, int crystal_pool_id, const FilterConfig& f, int& skipped_rows);

// Validate a per-ref predicate text under the single-atom rule (single Factor,
// single alternative) — plan §3 decision 3. Empty/whitespace → valid (means
// whole-crystal). Returns the rejection message when factors!=1 or alts!=1.
GuiValidationResult ValidateSingleAtomText(const std::string& text);

// Orchestration wrapper: 500 ms throttled poll + resize + shared cache. Single
// source consumed by both the Colors window per-row warnings and the top-bar
// aggregate warning (a12) so the two indicators can't drift. Safe to call
// multiple times per frame — throttle key + shared static cache make same-frame
// calls idempotent (subsequent calls just resize + return the same vector).
// Returns a copy: the vector is tiny (one int per color class), so this trades
// a negligible copy for not exposing the internal throttle cache by reference
// across module boundaries.
std::vector<int> RefreshColorClassSignals(const GuiState& state, LUMICE_Server* server);

// task-fix-color-window-visibility-consistency: render-time derived
// "effective visibility" that mirrors the compositor's per-class participation
// rule (src/server/component_compositor.cpp GatherActiveClasses:55 —
// `any_solo ? cls.solo_ : cls.visible_`). Kept as a pure inline helper so the
// UI (`RenderColorWindow` eye icon, `NoVisibleMatchedColorClass`) cannot drift
// from the compositor's actual filter without recompiling. Do NOT persist —
// this is a per-frame derived quantity, never a stored field.
inline bool AnySolo(const std::vector<ColorClassConfig>& classes) {
  for (const auto& c : classes) {
    if (c.solo) {
      return true;
    }
  }
  return false;
}

inline bool EffectiveVisible(const ColorClassConfig& cls, bool any_solo) {
  return any_solo ? cls.solo : cls.visible;
}

// True when the composite would be empty at this instant — i.e. NO configured
// class (non-empty `match[]`) is simultaneously matched (`signal_flags[i]!=0`)
// AND effectively visible (`EffectiveVisible(cls, AnySolo(...))`). Drives the
// top-bar aggregate warning pip AND the Colors-window Enable-checkbox greying;
// merges the two prior "empty composite" states (fully unmatched vs. matched
// but every match hidden) into a single owner. Same signal source as the
// per-row warning in `RenderColorWindow`. Returns false when raypath_color is
// empty or when every class has empty match[] (nothing to warn about yet). A
// configured class whose index falls outside `signal_flags` is treated as
// unknown (not counted either way), not as a confirmed no-signal.
bool NoVisibleMatchedColorClass(const GuiState& state, const std::vector<int>& signal_flags);

// task-349.2 Step 3 (#6): shared tooltip text for the disabled "enable colors"
// controls (top-bar Colored toggle + Colors-window Enable checkbox). Single
// source so the two indicators cannot drift; kept as a header-scope constant
// rather than duplicated string literals per plan-review Suggestion 1. Wording
// covers both empty-composite roots: (a) no rays match any class; (b) matches
// exist but every matching class is currently hidden (visible=false, or solo'd
// out by another class).
inline constexpr const char* kColorsDisabledNoMatchTooltip =
    "No visible color class currently matches any rays -- the composite would be empty.\n"
    "Either no rays match any configured class (a physical filter may be blocking them,\n"
    "or the last run produced no matching data), or every matching class is currently\n"
    "hidden. Re-run, adjust the color classes, or make a matching class visible.";

// ---------------------------------------------------------------------------
// Exported for gui_test only — do not call from production code. Production
// code should go through RefreshColorClassSignals above, which adds the
// throttle + shared-cache semantics these tests need to bypass for
// deterministic, single-shot assertions.
// ---------------------------------------------------------------------------

// Set the ref's `match_all` flag WITHOUT clearing `predicate_text`. The core
// invariant this codifies: file_io.cpp FillColorPredicate reads `match_all`
// FIRST and emits UNSET (whole-crystal) whenever it is true, so the retained
// predicate_text does NOT leak into a commit while whole is checked. Toggling
// whole off then restores the same text the user last typed — no undo history.
void SetRefMatchAll(ColorClassRefConfig& ref, bool match_all);

// Handle a click on the visible/eye icon in the class list. Plain click toggles
// `visible` only; Alt+click implements exclusive solo:
//   - not currently solo → clear all others' solo, set this one's solo=true
//   - currently solo    → clear all solo (compositor's any_solo becomes false,
//                         falls back to per-class `visible`)
// UI never puts more than one class into the solo set. Compositor's
// `GatherActiveClasses` "any solo → take solo set, otherwise visible set"
// semantics are unchanged; this helper only shapes the UI-driven state.
// Out-of-range `phys` is a defensive no-op (mirrors SwapZOrder).
void HandleEyeClick(std::vector<ColorClassConfig>& classes, size_t phys, bool alt_down);

// Poll LUMICE_GetColorClassSignal into caller-owned buffer. Semantics on failure:
// if the C API rejects the class_count (settling window between GUI-side
// push_back and server-side commit), `out_flags` is LEFT UNCHANGED — the
// previously observed signal is preserved rather than clobbered to zero.
// `out_flags.resize(n, 1)` before the C call: new entries default to 1
// ("settling / not yet warned") so that adding a class does not immediately mark
// all pre-existing classes as unmatched.
void PollColorClassSignal(const GuiState& state, LUMICE_Server* server, std::vector<int>& out_flags);

// task-cleanup-hardening S5 — test-only accessors on the RefreshColorClassSignals
// throttle cache. Production code MUST NOT use these; they exist so gui_test can
// observe the (server, committed_epoch) invalidation keys and the cached flags
// size / poll-time directly, without needing two real servers whose signals
// happen to differ. AC2 verifies that a domain change (server pointer OR
// committed_epoch) forces the next RefreshColorClassSignals call to bypass the
// 500 ms throttle and re-poll immediately.
void GetColorClassSignalCacheKeysForTest(LUMICE_Server** server_out, uint64_t* epoch_out, size_t* flags_size_out,
                                         float* last_poll_time_out);
void ResetColorClassSignalCacheForTest();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_COLOR_WINDOW_HPP
