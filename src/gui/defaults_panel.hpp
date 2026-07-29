#ifndef LUMICE_GUI_DEFAULTS_PANEL_HPP
#define LUMICE_GUI_DEFAULTS_PANEL_HPP

// The "save current settings as my defaults" panel: a modal over the row set produced by
// defaults_diff.hpp.
//
// Two sections, in this order:
//   §1 Presets  — the preset library (a retuned built-in preset, namespace 2).
//   §2 Settings — ONE list of every key that could be a personal default, each with a checkbox
//                 that means exactly "this key is in my defaults". Save writes the checked rows'
//                 current values and removes the unchecked ones; a filter narrows the list.
//
// The single list replaces a two-section split ("changes to adopt" above "other entries") whose
// halves had different controls — a checkbox in one, a per-row Revert button in the other — for
// what turned out to be one decision. A user could not tell from the screen which of the two
// mechanisms would decide the fate of a given key, and Reset all had to reach across both. One
// list, one control, one meaning.
//
// Modal on purpose: the row set is read once when the panel opens, and a modal makes that
// snapshot correct by construction — the user cannot edit the document behind an open panel, so
// there is no "the diff is stale" state to detect and re-sync.

#include "gui/gui_state.hpp"

namespace lumice::gui {

// Which section the panel starts expanded on. The entry point chooses; the panel does not
// hard-code one.
enum class DefaultsPanelSection {
  kPresets,  // §1 — the preset library
  kSettings  // §2 — the merged settings list
};

// Window title, exported so tests address the modal by the same string the panel registers.
//
// Word-for-word the top-bar button that opens it (app_panels.cpp: ICON_FA_GEAR " Settings"). It
// used to read "Save Current as Defaults", which described the panel when Save was the only thing
// it did; the panel now EDITS these settings, and a title naming one of its buttons made the
// entry point and the window it opens look like two different features.
inline constexpr const char* kDefaultsPanelTitle = "Settings";

// Open the panel and rebuild its row set from the CURRENT state. Also recomputes the checkbox set
// from scratch and clears the search/filter controls: every opening starts from "here is what your
// defaults would be if you saved right now, showing everything".
void OpenDefaultsPanel(GuiState& state, DefaultsPanelSection initial_section);

// Render the panel; a no-op when it is not open. Must be called once per frame from the top-level
// frame loop (src/gui/main.cpp AND test/gui/test_gui_main.cpp) — a Render*Modal that only some
// loops call is invisible to whichever binary forgot it.
void RenderDefaultsPanel(GuiState& state);

// Drop every file-scope static this TU owns. Called from gui_test's ResetTestState() so one
// scenario's checkbox set / search text / filter cannot reach the next in a single-process suite.
void ResetDefaultsPanelTestState();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DEFAULTS_PANEL_HPP
