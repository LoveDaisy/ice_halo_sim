#ifndef LUMICE_GUI_EDIT_MODALS_HPP
#define LUMICE_GUI_EDIT_MODALS_HPP

struct GLFWwindow;

namespace lumice::gui {

struct GuiState;

// Render the document inspector's crystal page for entry (layer_idx, entry_idx): the sharing row,
// the crystal preview, and the Crystal / Axis / Filter tabs. Out-of-range indices draw nothing.
//
// This is the persistent-editor replacement for the three edit modals. It has no open or close
// event: the tree's selection IS the question of which entry is being edited, so the page reloads
// its buffers when that answer changes (or when the page reappears) and pushes them back into the
// document every frame. There is consequently no "confirm" — see doc/gui-layout-architecture.md §2.
void RenderCrystalInspector(GuiState& state, int layer_idx, int entry_idx);

// Reset the crystal editor's internal static state (edit buffers, the entry the buffers point at,
// the preview epoch). Called by test teardown (ResetTestState) to prevent state leakage between
// tests, and by the document-lifecycle paths that replace GuiState wholesale.
//
// The name keeps "Modal" although the modal is gone. Renaming it is a separate, mechanical change
// across every test teardown, and doing it inside the migration would have mixed a rename into a
// diff whose point is the behaviour change.
void ResetModalState();

// Render the custom-spectrum editor modal (independent of the Crystal/Axis/Filter
// per-entry editors above, which are no longer modal at all). The Sun panel's Spectrum combo calls
// OpenSpectrumModal() when the user picks "Custom..."; this function must be called each frame
// (from the main loop) to actually paint the popup while it is open.
void OpenSpectrumModal(GuiState& state);
void RenderSpectrumModal(GuiState& state);

// Returns true when the committed axis config of the entry the crystal page is bound to meets
// D-symmetry conditions (az uniform 360°, roll mean a multiple of 30°). Returns false when the page
// is not bound to an entry, or the entry index is invalid.
// Intended for GUI test assertions; production code should not call this.
bool IsCurrentModalDApplicable();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EDIT_MODALS_HPP
