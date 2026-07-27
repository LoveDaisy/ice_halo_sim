#ifndef LUMICE_GUI_DEFAULTS_PANEL_HPP
#define LUMICE_GUI_DEFAULTS_PANEL_HPP

// The "save current settings as my defaults" panel: a modal over the row set produced by
// defaults_diff.hpp.
//
// Three sections, in this order:
//   §1 Presets       — placeholder in this task; the preset library lands with 405.5.
//   §2 Changes       — rows whose current value differs from the effective default. Checked by
//                      default (opening this panel already means "adopt what I have"), unchecking
//                      excludes a row from the save.
//   §3 Other entries — everything else, read-only, collapsed by default. Carries the source of
//                      each value (factory vs mine) plus Revert / Reset all.
// §2 and §3 partition ONE row set (RowNeedsAdoption and its negation), so no key can appear twice.
//
// Modal on purpose: the row set is read once when the panel opens, and a modal makes that
// snapshot correct by construction — the user cannot edit the document behind an open panel, so
// there is no "the diff is stale" state to detect and re-sync.

#include "gui/gui_state.hpp"

namespace lumice::gui {

// Which section the panel starts expanded on. The entry point chooses; the panel does not
// hard-code one. 405.5 adds a second entry ("edit my presets") that opens on kPresets without
// touching the mechanism.
enum class DefaultsPanelSection {
  kPresets,        // §1
  kPendingChanges  // §2
};

// Window title, exported so tests address the modal by the same string the panel registers.
inline constexpr const char* kDefaultsPanelTitle = "Save Current as Defaults";

// Open the panel and rebuild its row set from the CURRENT state. Also clears the per-session
// unchecked set and the search filter: every opening starts from "adopt everything, show
// everything", which is the semantics the entry button promises.
void OpenDefaultsPanel(GuiState& state, DefaultsPanelSection initial_section);

// Render the panel; a no-op when it is not open. Must be called once per frame from the top-level
// frame loop (src/gui/main.cpp AND test/gui/test_gui_main.cpp) — a Render*Modal that only some
// loops call is invisible to whichever binary forgot it.
void RenderDefaultsPanel(GuiState& state);

// Drop every file-scope static this TU owns. Called from gui_test's ResetTestState() so one
// scenario's unchecked rows / search text cannot reach the next in a single-process suite.
void ResetDefaultsPanelTestState();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DEFAULTS_PANEL_HPP
