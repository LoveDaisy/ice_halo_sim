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

#include "include/lumice.h"

namespace lumice::gui {

struct GuiState;

// Render the non-modal Colors window (Layer 3, floating). No-op when
// state.color_window_open is false.
void RenderColorWindow(GuiState& state, LUMICE_Server* server);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_COLOR_WINDOW_HPP
