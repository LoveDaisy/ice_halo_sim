#ifndef LUMICE_GUI_DEFAULTS_DIFF_HPP
#define LUMICE_GUI_DEFAULTS_DIFF_HPP

// The row set behind the "save current settings as my defaults" panel, plus the write-back
// operation that panel commits (one rule for the whole row set: keep what is checked, drop what is
// not).
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
#include <set>
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
  // The EFFECTIVE default: factory value with the saved override layered over it. This is what a
  // new document would actually start from, which is why it is the one the panel shows.
  nlohmann::json default_value;
  // The LITERAL factory value — GuiState{} serialized, with nothing layered on top.
  //
  // Equal to default_value exactly while this key has no saved override, which is why the two can
  // be confused. They must not be: once a user has saved a non-factory default for a key,
  // default_value IS that saved value, so "does this differ from factory" asked of default_value
  // answers "no" for every key the user has ever customised — the precise rows the question is
  // being asked about. The panel's "Differs from factory" filter compares against THIS field.
  nlohmann::json factory_value;
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
//                     both sides, so it can never differ; excluding it keeps it out of the list,
//                     where it would only read as "is this a setting?".
inline constexpr const char* kDiffEngineExcludedRootKeys[] = { "layers", "schema_version" };

// The raw (un-merged) override document in the active user-config directory. Empty object when
// personal defaults are disabled or unavailable, which is also what "nothing saved" looks like.
//
// Public so the panel takes its opening snapshot through the SAME directory resolution the diff
// engine and MakeNewDocumentState() use. Re-deriving it there from GetActiveUserConfigDir() +
// ReadOverlayJsonIfPresent() would put a second copy of the "no directory ⇒ empty" degradation
// rule in the panel, and two resolution paths for one question is the drift invariant I1 exists
// to prevent (see the note on GetActiveUserConfigDir in user_defaults.hpp).
nlohmann::json ReadActiveOverlayDoc();

// Writes `doc` verbatim to the active user-config directory's override file. Public for the same
// reason ReadActiveOverlayDoc is: the panel's Save is a whole-document commit (it builds the next
// document in memory, then writes it), so it needs that directory resolution once, in one place.
// A second inline GetActiveUserConfigDir() + WriteUserDefaultsFile() at the call site would be the
// same drift risk ReadActiveOverlayDoc's comment already flags for the read side.
//
// THE ONLY WRITER of the override file outside user_defaults.cpp's own WriteUserDefaultsFile, and
// meant to stay that way: a second write path is a second answer to "what did the user save", and
// this file has already carried one (an unreachable set of read-mutate-write wrappers left behind
// by the panel's move to the copy model, complete with its own "this is the only place…" comment).
// scripts/check_policies.py enforces the call-site count.
bool WriteActiveOverlayDoc(const nlohmann::json& doc);

// The whole row set for `current`, against the currently effective defaults (factory + whatever is
// saved in the active user-config directory). Sorted by key_path for a stable panel order.
//
// Reads the override file, so it is a "call on open / call after a write" operation, not a
// per-frame one (see defaults_panel.cpp, which caches the result for the life of the modal).
std::vector<DefaultDiffRow> BuildDefaultDiffRows(const GuiState& current);

// Same, against an EXPLICIT override document instead of whatever is on disk right now.
//
// This is the overload the panel uses. The panel is a pure editor: it takes one snapshot of the
// file when it opens and compares against that frozen document for the whole session, so nothing
// the user has not committed yet can move a row's "Source" or its opening checkbox state underneath
// them. The single-argument version above is exactly this one applied to ReadActiveOverlayDoc().
std::vector<DefaultDiffRow> BuildDefaultDiffRows(const GuiState& current, const nlohmann::json& overlay_doc);

// Whether `key_path` is written in `doc` itself (the in-document form of
// DefaultDiffRow::has_saved_override — see that field's comment for why presence, not value
// equality, is the question).
bool DocHasKeyPath(const nlohmann::json& doc, const std::string& key_path);

// Display form of a JSON leaf. DISPLAY ONLY — comparison is done on the raw json values
// (RowNeedsAdoption below), so no amount of formatting loss can make two different values look
// (or be judged) equal.
std::string FormatDiffValue(const nlohmann::json& value);

// Whether this row's current value differs from the default a new document would resolve for it.
//
// Was the §2/§3 partition when the panel had two sections; it is now one half of the OR that
// decides a row's INITIAL checkbox state (the other half is has_saved_override). Still a total,
// two-valued predicate over one row set, which is the property test_defaults_diff.cpp asserts.
bool RowNeedsAdoption(const DefaultDiffRow& row);

// Whether pressing Save right now would change what the override file holds for this key, given
// the row's current checkbox state. The panel tints such rows, so "the thing this panel is for" is
// visible per row instead of only through a filter that hides the others.
//
// A DIFFERENT question from both filters, and the distinction is the whole reason it exists:
//   "Differs from factory"  — compares the value with the built-in one. A key the user saved long
//                             ago and has not touched since differs from factory forever, yet
//                             saving would not move it.
//   "Edited this session"   — compares the checkbox with its state at open. Says nothing about a
//                             value edited in place while the checkbox stayed put.
//   this predicate          — compares what Save would write with what is on disk. That is the
//                             copy-vs-disk judgement the panel is built around.
//
// Both sides are "present with a value, or absent": checked means Save writes current_value,
// unchecked means Save removes the key, and has_saved_override says whether the file holds one
// today (in which case default_value IS that stored value — see the field's comment). Presence has
// to be compared as well as value, otherwise checking a key whose value already equals the stored
// one would read as no change when it would in fact ADD the key to the file, and vice versa.
bool RowWouldChangeOnSave(const DefaultDiffRow& row, bool checked);

// ------------------------------------------------------------------------------------------------
// The panel's write-back rule. Surgical key-level edits of an existing document, never a wholesale
// replacement: the override file also carries the "presets" subtree, and a full rewrite here would
// silently delete a user's preset overrides.
//
// Pure — it takes a document and changes it, it does no IO. That is what lets the panel's in-memory
// copy and the on-disk document be edited by the SAME code: the panel applies it to its working
// copy and writes once on Save.
// ------------------------------------------------------------------------------------------------

// Fold the panel's checkbox state into `doc`: every checked row gets its CURRENT value written,
// every unchecked row is removed. ONE rule over the WHOLE row set, which is what the checkbox now
// means — "this key is (not) in my defaults" — rather than two paths ("adopt this pending change"
// vs "revert this stored key") whose combined effect the user had to work out for themselves.
//
// A checked key path that is absent from the serialized current state (an optional key such as
// sun.custom_spectrum) is REMOVED instead — "adopt what I have now" reads the same either way.
//
// Keys `rows` does not mention are left alone. That is how the "presets" subtree survives (it never
// produces a row), and it also means a key some older version wrote is not swept up by a rule that
// never displayed it to the user.
//
// Returns false (leaving `doc` untouched) when `current` could not be serialized. The caller must
// not commit in that case: writing a document that is missing the very keys it was asked to adopt,
// while reporting a successful save, is a silent wrong answer. That failure is raised BEFORE the
// first mutation, which is what makes the whole call all-or-nothing.
bool ApplyCheckedRowsToDoc(nlohmann::json& doc, const std::vector<DefaultDiffRow>& rows,
                           const std::set<std::string>& checked_key_paths, const GuiState& current);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DEFAULTS_DIFF_HPP
