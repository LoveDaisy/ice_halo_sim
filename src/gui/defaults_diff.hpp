#ifndef LUMICE_GUI_DEFAULTS_DIFF_HPP
#define LUMICE_GUI_DEFAULTS_DIFF_HPP

// The row set behind the "save current settings as my defaults" panel, plus the three write-back
// operations that panel commits (adopt selected / revert one / reset all).
//
// GENERATED, NEVER ENUMERATED. A row set is produced by walking two JSON trees — the CURRENT
// GuiState and the EFFECTIVE DEFAULT (factory GuiState + the saved sparse override) — both
// serialized through SerializeGuiStateJson. There is deliberately no per-field metadata table and
// no hand-written key list: a second schema drifts the first time someone adds a GuiState field
// and forgets it, and this panel's entire job is to show the user every field that COULD be a
// default. A new serialized field appears here with zero changes to this file.
//
// The only hand-maintained knowledge is kDiffEngineExcludedRootKeys below, which is not a field
// list (see its own comment).
//
// Layering: this half calls into file_io.cpp (SerializeGuiStateJson) and user_defaults.cpp
// (GetActiveUserConfigDir / ReadOverlayJsonIfPresent / ApplyUserDefaultsOverlay /
// WriteUserDefaultsFile), so it links with lumice_gui_obj and is exercised from gui_test, not
// from unit_correctness_test — the same split user_defaults.hpp documents. It has NO ImGui
// dependency, though: the panel (defaults_panel.hpp) is a separate layer on top, and every
// assertion below can be made without rendering a frame.

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"

namespace lumice::gui {

// One candidate default: a leaf of the serialized GuiState document.
//
// `key_path` is dot-separated in the SERIALIZED shape ("renderer.lens_type", "overlay_grid_alpha"),
// not the C++ field shape — that is what the user sees in the override file, and several root keys
// are renames of their field ("aspect_ratio" <- aspect_preset, "overlay_grid_alpha" <- grid_alpha).
//
// Arrays are leaves: "renderer.background" is ONE row carrying the whole triple, never three rows.
// A single color edit must not flood the panel with three lines of noise.
struct DefaultDiffRow {
  std::string key_path;
  nlohmann::json current_value;
  nlohmann::json default_value;
  // Whether this key path is present in the saved override document ITSELF (not whether its value
  // happens to differ from factory). The two are independent: a user may deliberately save a value
  // that equals the factory one, and that is still "my default" — an explicit intent the panel has
  // to keep showing, otherwise the user cannot see (or revert) what they once saved.
  bool has_saved_override = false;
  // Interface reserved for 405.5 (preset-range clamp notices). Contract for that task, so it does
  // not have to be reverse-engineered from the panel: entries are already user-facing sentences,
  // ORDER IS PRIORITY (the panel shows the first one next to the row and the rest on hover), and
  // an empty vector means "no warning" — never a placeholder string. Always empty in this task;
  // the panel builds the column but has nothing to put in it yet.
  std::vector<std::string> warnings;
};

// Root keys the walk skips. This is NOT the "which fields may be defaults" list — that question is
// answered structurally by what SerializeGuiStateJson emits, and the answer for every OTHER root
// key is "yes". Exactly two special cases live here:
//   layers          — the whole of namespace 4. crystals / filters have no root key of their own
//                     (they are serialized inline under layers[].entries[]), and raypath_color is
//                     not serialized at all, so excluding this one key excludes the namespace.
//                     A key path into it would carry a document-local index, which cannot be a
//                     personal default.
//   schema_version  — a format version constant, not a GuiState field at all. It is identical on
//                     both sides so it would never produce a §2 row anyway; excluding it keeps it
//                     out of §3, where it would only read as "is this a setting?".
inline constexpr const char* kDiffEngineExcludedRootKeys[] = { "layers", "schema_version" };

// The whole row set for `current`, against the currently effective defaults (factory + whatever is
// saved in the active user-config directory). Sorted by key_path for a stable panel order.
//
// Reads the override file, so it is a "call on open / call after a write" operation, not a
// per-frame one (see defaults_panel.cpp, which caches the result for the life of the modal).
std::vector<DefaultDiffRow> BuildDefaultDiffRows(const GuiState& current);

// Display form of a JSON leaf. DISPLAY ONLY — comparison is done on the raw json values
// (RowNeedsAdoption below), so no amount of formatting loss can make two different values look
// (or be judged) equal.
std::string FormatDiffValue(const nlohmann::json& value);

// §2 (needs adoption) vs §3 (everything else). The two sections are exactly this predicate and its
// negation over one row set, so they are mutually exclusive and jointly complete by construction.
bool RowNeedsAdoption(const DefaultDiffRow& row);

// ------------------------------------------------------------------------------------------------
// Write-back. All three are surgical key-level edits of the existing document, never a wholesale
// replacement: the override file also carries the "presets" subtree (405.1/405.2 read it; its UI
// comes later), and a full rewrite here would silently delete a user's preset overrides.
// ------------------------------------------------------------------------------------------------

// Copy each accepted key path's CURRENT value into the override document. A key path that is
// absent from the current document (an optional key such as sun.custom_spectrum) is REMOVED from
// the override instead — "adopt what I have now" reads the same either way.
bool SaveAcceptedDefaults(const std::vector<std::string>& accepted_key_paths, const GuiState& current);

// Drop one key path from the override document, so it falls back to the factory value. Empty
// parent objects left behind are pruned, keeping the file readable by hand.
bool RevertOneDefault(const std::string& key_path);

// Drop every GuiState key from the override document. "presets" survives untouched — it belongs to
// namespace 2 and is not what this panel edits.
bool ResetAllDefaults();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DEFAULTS_DIFF_HPP
