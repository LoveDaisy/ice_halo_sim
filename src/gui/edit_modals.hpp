#ifndef LUMICE_GUI_EDIT_MODALS_HPP
#define LUMICE_GUI_EDIT_MODALS_HPP

namespace lumice::gui {

struct GuiState;
struct EditRequest;

// Process an edit request from the card UI. Must be called from the left panel
// after detecting a non-None EditRequest (before ResetEditRequest).
void OpenEditModal(const EditRequest& req, GuiState& state);

// Render all modal popups (Crystal/Axis/Filter). Called from the main loop
// outside any Begin/End block, in the same scope as RenderUnsavedPopup.
void RenderEditModals(GuiState& state);

// Returns true when the unified edit modal is open AND the Crystal tab is the
// active one. Used by visual-smoke tests to detect FBO contention with the
// modal's per-frame g_crystal_renderer.Render() call.
bool IsEditModalCrystalTabActive();

// Reset all modal-internal static state (active modal, edit buffers, pending flags).
// Called by test teardown (ResetTestState) to prevent state leakage between tests.
void ResetModalState();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EDIT_MODALS_HPP
