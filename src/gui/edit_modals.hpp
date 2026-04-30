#ifndef LUMICE_GUI_EDIT_MODALS_HPP
#define LUMICE_GUI_EDIT_MODALS_HPP

struct GLFWwindow;

namespace lumice::gui {

struct GuiState;
struct EditRequest;

// Process an edit request from the card UI. Must be called from the left panel
// after detecting a non-None EditRequest (before ResetEditRequest).
void OpenEditModal(const EditRequest& req, GuiState& state);

// Render all modal popups (Crystal/Axis/Filter). Called from the main loop
// outside any Begin/End block, in the same scope as RenderUnsavedPopup.
// `window` is used to clamp the modal's max size to the workarea of the
// monitor containing the window center; pass nullptr in headless test
// harnesses (the multi-monitor clamp then falls back to unbounded max).
void RenderEditModals(GuiState& state, GLFWwindow* window);

// Returns true when the unified edit modal is open (any tab). Used by
// visual-smoke tests to skip FBO drive while the modal owns the FBO via its
// per-frame g_crystal_renderer.Render() call — the preview is now a
// persistent left pane shared across Crystal / Axis / Filter tabs, so the
// gate condition is modal-open rather than crystal-tab-active.
bool IsEditModalOpen();

// (layer_idx, entry_idx) of the entry the open modal is bound to. Returns
// {-1, -1} as a safe sentinel when no modal is open, so callers may either
// check IsEditModalOpen() first (more readable) or just inspect the returned
// indices and treat negatives as "no target". This function is meant for UI
// lifecycle reverse-annotation (e.g. highlighting the source card while the
// modal is open), not for modal control flow.
struct EditModalTarget {
  int layer_idx;
  int entry_idx;
};
EditModalTarget GetEditModalTarget();

// Reset all modal-internal static state (active modal, edit buffers, pending flags).
// Called by test teardown (ResetTestState) to prevent state leakage between tests.
void ResetModalState();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EDIT_MODALS_HPP
