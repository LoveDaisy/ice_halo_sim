#ifndef LUMICE_GUI_COLOR_WINDOW_HPP
#define LUMICE_GUI_COLOR_WINDOW_HPP

// task-342.3 Step 5-9: floating (non-modal) "Colors" window for editing the
// raypath color class pool.
//
// Design overview (see scratchpad/scrum-raypath-color-design2/task-gui-color-window/plan.md):
//   - Display-time edits (color / visible / solo / z_order / composite mode)
//     call LUMICE_SetRaypathColors directly — no epoch bump, no re-simulation.
//   - Structural edits (predicate text / combine any↔all / add/remove class or
//     ref) call GuiState::MarkFilterDirty(), which the main-loop debounce
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

// Poll LUMICE_GetColorClassSignal into caller-owned buffer. Semantics on failure
// (task-348.1 fix): if the C API rejects the class_count (settling window between
// GUI-side push_back and server-side commit), `out_flags` is LEFT UNCHANGED —
// the previously observed signal is preserved rather than clobbered to zero.
// `out_flags.resize(n, 1)` before the C call: new entries default to 1
// ("settling / not yet warned") so that adding a class does not immediately mark
// all pre-existing classes as unmatched.
void PollColorClassSignal(const GuiState& state, LUMICE_Server* server, std::vector<int>& out_flags);

// Orchestration wrapper (task-348.1): 500 ms throttled poll + resize + shared
// cache. Single source consumed by both the Colors window per-row warnings and
// the top-bar aggregate warning (a12) so the two indicators can't drift. Safe
// to call multiple times per frame — throttle key + shared static cache make
// same-frame calls idempotent (subsequent calls just resize + return the same
// vector). Returns a copy (code-review-01 Minor 2): the vector is tiny (one int
// per color class), so this trades a negligible copy for not exposing the
// internal throttle cache by reference across module boundaries.
std::vector<int> RefreshColorClassSignals(const GuiState& state, LUMICE_Server* server);

// True when the user has configured at least one color class with non-empty
// `match[]`, AND every such class currently reports no signal (all warned).
// Drives the top-bar aggregate warning pip; matches the per-row warning
// condition in RenderColorWindow (`!match.empty() && signal_flags[phys]==0`)
// so the two never disagree. Returns false when raypath_color is empty or
// when every class has empty match[] (nothing to warn about yet).
bool AllConfiguredColorClassesUnmatched(const GuiState& state, const std::vector<int>& signal_flags);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_COLOR_WINDOW_HPP
