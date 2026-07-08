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

}  // namespace lumice::gui

#endif  // LUMICE_GUI_COLOR_WINDOW_HPP
