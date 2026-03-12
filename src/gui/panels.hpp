#ifndef LUMICE_GUI_PANELS_HPP
#define LUMICE_GUI_PANELS_HPP

namespace lumice::gui {

struct GuiState;

void RenderCrystalTab(GuiState& state);
void RenderSceneTab(GuiState& state);
void RenderRenderTab(GuiState& state);
void RenderFilterTab(GuiState& state);

// Reset pending-delete state (for test isolation)
void ResetPendingDeleteState();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PANELS_HPP
