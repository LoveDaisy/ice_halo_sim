#include "gui/defaults_panel.hpp"

#include <cfloat>
#include <cstddef>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/destructive_style.hpp"
#include "gui/panels.hpp"
#include "gui/user_defaults.hpp"
#include "imgui.h"

namespace lumice::gui {

namespace {

// ------------------------------------------------------------------------------------------------
// Panel session state.
//
// All TU-local on purpose: none of it is document state, none of it is serialized, and none of it
// has Revert semantics — it is rebuilt from scratch every time the panel opens. Each is a value
// type that is replaced WHOLESALE (never partially cleared) on open and on reset; this scrum has
// already spent three code-review rounds on a module-level container that was cleared in one
// branch and refilled in another, and the shape below has no branch to get wrong.
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// The copy model. This panel is a pure EDITOR, not a live view of the file: it takes a copy of the
// override document when it opens, every edit changes only that copy, Save writes it once, and
// closing without saving throws it away.
//
// Why, and what it replaces: Revert and Reset all used to write the file the instant they were
// clicked while Save committed later, so pressing Reset all changed the file and left the panel
// looking identical — same rows, same checkboxes — and "will Save now include those diffs?" was
// unanswerable from the screen. One commit point removes the question.
//
// DEPENDENCY, stated because it is invisible from here: the panel is a BeginPopupModal, so the
// main UI cannot be edited while it is open and `current` cannot move underneath the copy. If this
// panel is ever made non-modal (a docked window, say), that premise is gone and the copy would
// need a reconciliation story with edits made behind it.
//
// TWO documents, not one, and the split is load-bearing:
//   g_snapshot_doc — what was on disk when the panel opened. FROZEN for the session. It anchors
//                    every judgement the list makes about a row (its effective default, its
//                    Source, its opening checkbox state), so nothing moves under the user while
//                    they work. Anchoring on the working copy instead would re-answer "is this
//                    mine" from a document the user has not committed.
//   g_copy_doc     — what pressing Save right now would write. Only §1's preset edits mutate it
//                    directly; the settings list expresses its intent as a checkbox set that is
//                    folded in once, at Save (see CommitCopy).
// ------------------------------------------------------------------------------------------------
nlohmann::json g_snapshot_doc = nlohmann::json::object();
nlohmann::json g_copy_doc = nlohmann::json::object();

// Row set, rebuilt on open and after a Save. Not rebuilt per frame: BuildDefaultDiffRows
// re-serializes the whole GuiState three times, and a per-frame call would do that 60x a second.
std::vector<DefaultDiffRow> g_rows;

// ------------------------------------------------------------------------------------------------
// The checkbox set. ONE meaning for every row in the list: "this key is in my defaults", i.e. what
// Save would leave the override file holding.
//
//   g_checked_keys         — the current answer, edited by clicking a checkbox.
//   g_initial_checked_keys — the same set frozen at open (and re-frozen after a successful Save).
//                            It is the BASELINE for "edited this session": a row counts as edited
//                            exactly when its checkbox state differs from this. Deriving that from
//                            the row's values instead would answer a different question — "differs
//                            from factory" — which is the other filter, and conflating the two is
//                            the confusion this task exists to remove.
//
// A selection set rather than 405.4's exclusion set: rows are no longer checked-by-default (a key
// that is neither changed nor already saved opens unchecked), so there is no default state for an
// exclusion set to be the complement of.
// ------------------------------------------------------------------------------------------------
std::set<std::string> g_checked_keys;
std::set<std::string> g_initial_checked_keys;

// Which rows the list shows, on top of the search box (they AND: a row must pass both).
//
// Three-way single choice rather than two independent toggles, because the two questions are NOT
// the same question and the panel must not let them read as one: a value can differ from the
// factory value without the user having touched it this session (they changed it in the main UI,
// or saved it long ago), and a row can be edited this session without differing from factory (the
// user checked a key whose value happens to be the built-in one). Two checkboxes would have raised
// a third question — whether ticking both means AND or OR — that has no natural answer.
enum class DefaultsRowFilter {
  kAll,
  kDiffersFromFactory,  // row.current_value != row.factory_value
  kEditedThisSession    // the checkbox moved since the panel opened
};
DefaultsRowFilter g_row_filter = DefaultsRowFilter::kAll;

ImGuiTextFilter g_search_filter;

// Consumed on the first frame after an open: which section starts expanded. nullopt on every
// other frame, so a section the user collapses stays collapsed while the panel is open.
std::optional<DefaultsPanelSection> g_pending_initial_section;

// Outcome of the last write, shown at the bottom of the panel. Deliberately carries NO directory
// path: this string is inside the visual-regression capture, and a machine-specific path would
// bake one developer's home directory into a committed reference image.
std::string g_status_message;

// §1's per-preset warning text, indexed by AxisPreset. The library's own channel, NOT
// DefaultDiffRow::warnings: BuildDefaultDiffRows walks the serialized GuiState tree, and the
// presets subtree is not in it — a preset can never produce a DefaultDiffRow to hang a warning on
// (scrum D1: "presets are not GuiState fields, they do not enter the diff"). Same visual form as
// the settings list's "!" column, different data source, on purpose.
//
// Whole-value replacement like every other piece of panel session state here: one array assigned
// on open, one element assigned per write.
struct PresetWarnings {
  std::string slots[static_cast<std::size_t>(AxisPreset::kCustom) + 1];
};
PresetWarnings g_preset_warnings;

// Live edit buffers for §1's std inputs, so a partially typed number ("0." on the way to "0.3")
// is not written — and clamped — on every keystroke. Committed on Enter / focus loss
// (IsItemDeactivatedAfterEdit), which is also when a clamp becomes a fair thing to report.
struct PresetStdBuffers {
  float slots[static_cast<std::size_t>(AxisPreset::kCustom) + 1] = {};
};
PresetStdBuffers g_preset_std_buffers;

bool IsRowChecked(const std::string& key_path) {
  return g_checked_keys.find(key_path) != g_checked_keys.end();
}

void RefreshRows(const GuiState& state) {
  // Built against the FROZEN snapshot, and nothing is re-answered from the working copy afterwards.
  // 405.4's model needed that second pass because a Revert click changed has_saved_override before
  // any write; with the checkbox as the only expression of that intent, the copy's GuiState half no
  // longer moves until Save, so BuildDefaultDiffRows' answer is already the right one.
  g_rows = BuildDefaultDiffRows(state, g_snapshot_doc);
}

// Rebuild the checkbox set from the rows: a key is checked at open exactly when it is already in
// the defaults, OR when the user's current value differs from what a new document would start with.
//
// The OR is the whole "what does this checkbox mean" answer. Reading it the other way round —
// which keys would be in my defaults if I saved right now — the first term keeps what is already
// saved, and the second adopts what the user has changed since. The three states AC2 names fall
// out of it: changed-not-saved and already-saved both open checked, neither opens unchecked.
//
// Whole-value assignment of BOTH sets, so the "edited this session" baseline cannot drift from the
// state it is supposed to be the baseline of.
void RecomputeCheckedKeys() {
  std::set<std::string> checked;
  for (const auto& row : g_rows) {
    if (RowNeedsAdoption(row) || row.has_saved_override) {
      checked.insert(row.key_path);
    }
  }
  g_initial_checked_keys = checked;
  g_checked_keys = std::move(checked);
}

// The two filter predicates, each answering its own question about one row.
bool PassesRowFilter(const DefaultDiffRow& row) {
  switch (g_row_filter) {
    case DefaultsRowFilter::kDiffersFromFactory:
      // Against the LITERAL factory value, never against row.default_value: for a key the user has
      // already saved a default for, default_value IS that saved value, so comparing with it would
      // report "same as factory" for precisely the keys they have customised.
      return row.current_value != row.factory_value;
    case DefaultsRowFilter::kEditedThisSession:
      return IsRowChecked(row.key_path) != (g_initial_checked_keys.find(row.key_path) != g_initial_checked_keys.end());
    case DefaultsRowFilter::kAll:
      break;
  }
  return true;
}

// Both narrowing controls, ANDed: a row is shown when it passes the search box AND the filter.
bool RowIsVisible(const DefaultDiffRow& row) {
  return g_search_filter.PassFilter(row.key_path.c_str()) && PassesRowFilter(row);
}

// What §1 shows for a preset: the value in the working copy, if it holds one.
//
// Reads the copy rather than GetUserAxisPresetZenithStdOverride: that accessor answers from the
// process-wide cache, which by design does not move until a Save lands, so the panel would show
// the user their own uncommitted edit as if it had not happened.
std::optional<float> CopyPresetZenithStd(const AxisPresetEntry& entry) {
  return ReadAxisPresetZenithStdFromDoc(g_copy_doc, entry.id);
}

// The zenith row §1 renders: the factory row with the copy's std substituted, clamped the way the
// loader would clamp it.
//
// The clamp matters for a value this session did not write — a hand-edited file can hold one
// outside the domain, and the document keeps it verbatim until the user commits something. Showing
// it raw would tell the user their Column button gives 25 when it actually gives 9.99999905.
AxisDist EffectiveCopyPresetZenith(const AxisPresetEntry& entry) {
  AxisDist zenith = entry.zenith;
  if (const auto stored = CopyPresetZenithStd(entry)) {
    const AxisPresetClampResult clamped = ClampAxisPresetZenithStdForSave(entry.id, *stored);
    if (clamped.accepted) {
      zenith.std = clamped.stored_value;
    }
  }
  return zenith;
}

// The document Save would write: the working copy with the checkbox set folded in.
//
// Split out from CommitCopy so the button can preview it (to tell "nothing to save" from "saved")
// with exactly the code that performs the write, rather than a second rule that agrees with it
// today. Returns false when the current state could not be serialized — see ApplyCheckedRowsToDoc.
bool BuildNextDoc(const GuiState& state, nlohmann::json& out) {
  out = g_copy_doc;
  return ApplyCheckedRowsToDoc(out, g_rows, g_checked_keys, state);
}

// The panel's ONE write. Every other action in this file edits g_copy_doc (or the checkbox set) and
// stops there.
//
// The checkbox set is folded into the copy HERE rather than when a box is clicked, because a
// checkbox is a statement about what Save should do, not an edit in its own right — ticking one
// must not look like a change to the document until the user commits it.
bool CommitCopy(const GuiState& state) {
  // Built beside the copy and only adopted once the write succeeds. On any failure below the copy
  // is left exactly as the user has it, so a second attempt (after fixing whatever blocked the
  // write) still carries their edits instead of a half-applied version of them.
  nlohmann::json next;
  if (!BuildNextDoc(state, next)) {
    return false;
  }
  if (!WriteActiveOverlayDoc(next)) {
    return false;
  }

  // ORDER IS PART OF THE CONTRACT — disk first, then memory, then the anchors:
  //  1. the write above has landed, so the process-wide preset cache may now follow it. Only the
  //     presets that actually changed this session are pushed — read directly off the two
  //     documents (next vs the frozen open-time snapshot) rather than tracked through a parallel
  //     "touched" shadow, so there is exactly one place a preset edit can go missing: the document
  //     itself. Re-parsing the whole document instead would be wrong for a different reason: it
  //     would re-clamp values this session never touched and file a duplicate downgrade notice for
  //     each, on a Save that had nothing to do with them.
  //  2. the copy becomes the new snapshot, because "what is on disk" is now literally this
  //     document. That re-anchors every judgement the list makes — a row the user just adopted now
  //     reads as Mine, which is the visible confirmation that the save landed.
  //  3. the caller then rebuilds the rows and re-freezes the checkbox baseline against the new
  //     anchor (see the Save button). Leaving the old baseline in place would report every row the
  //     save just committed as still "edited this session".
  for (const auto& entry : kAxisPresets) {
    const auto after = ReadAxisPresetZenithStdFromDoc(next, entry.id);
    const auto before = ReadAxisPresetZenithStdFromDoc(g_snapshot_doc, entry.id);
    if (after != before) {
      AdoptAxisPresetZenithStdOverrideInMemory(entry.id, after);
    }
  }
  g_copy_doc = std::move(next);
  g_snapshot_doc = g_copy_doc;
  return true;
}

// Section header whose open state is forced only on the frame an entry point requested it.
bool SectionHeader(const char* label, DefaultsPanelSection section, bool open_when_pending) {
  if (g_pending_initial_section.has_value()) {
    ImGui::SetNextItemOpen(open_when_pending && *g_pending_initial_section == section, ImGuiCond_Always);
  }
  return ImGui::CollapsingHeader(label);
}

// The search box and the filter, side by side. Together they answer "which rows am I looking at";
// keeping them on one line says so, and leaves the table the full height of the body.
void RenderListControls() {
  // "Search", not 405.4's "Filter": the word now belongs to the control beside it, and two things
  // called Filter that narrow the same list by different rules is the confusion this task is here
  // to remove, not to relocate.
  g_search_filter.Draw(ICON_FA_MAGNIFYING_GLASS " Search###defaults_search", 240.0f);

  // Labels ARE the definitions — the point owner made about this control is that a vague "only
  // show changes" switch is what produced the confusion in the first place, so each option says
  // which of the two meanings it carries, on screen and not only in a comment here.
  ImGui::SameLine();
  int filter = static_cast<int>(g_row_filter);
  ImGui::RadioButton("All###filter_all", &filter, static_cast<int>(DefaultsRowFilter::kAll));
  ImGui::SameLine();
  ImGui::RadioButton("Differs from factory###filter_differs", &filter,
                     static_cast<int>(DefaultsRowFilter::kDiffersFromFactory));
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Rows whose current value is not the built-in one — whether or not you saved it.");
  }
  ImGui::SameLine();
  ImGui::RadioButton("Edited this session###filter_edited", &filter,
                     static_cast<int>(DefaultsRowFilter::kEditedThisSession));
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Rows whose checkbox you changed since this panel opened.");
  }
  g_row_filter = static_cast<DefaultsRowFilter>(filter);
}

// The one settings table: every candidate default, one row each, one checkbox each.
void RenderSettingsTable() {
  constexpr ImGuiTableFlags kFlags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
  if (!ImGui::BeginTable("##defaults_settings_table", 6, kFlags)) {
    return;
  }
  ImGui::TableSetupColumn("##adopt", ImGuiTableColumnFlags_WidthFixed, 24.0f);
  ImGui::TableSetupColumn("Setting", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("Current", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("Effective default", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 110.0f);
  // The warning column (scrum D8). Built and sized here, filled by 405.5's preset-range clamp:
  // owner chose a column over a tooltip precisely because a tooltip is invisible to the reference
  // images, so the column has to exist in the captured layout before there is anything to put in
  // it.
  ImGui::TableSetupColumn("!", ImGuiTableColumnFlags_WidthFixed, 24.0f);
  ImGui::TableHeadersRow();

  for (const auto& row : g_rows) {
    if (!RowIsVisible(row)) {
      continue;
    }
    ImGui::TableNextRow();

    ImGui::TableNextColumn();
    bool checked = IsRowChecked(row.key_path);
    // The key path is the widget ID. It is unique by construction (it IS the row's identity), so
    // there is no index to keep in sync — the PushID(int) addressing trap that has bitten GUI
    // tests here before cannot arise. Every id this panel exposes uses "###" so the ID is the
    // short suffix alone: a test ref then does not have to carry the icon glyphs of a label.
    const std::string check_id = "###adopt_" + row.key_path;
    if (ImGui::Checkbox(check_id.c_str(), &checked)) {
      if (checked) {
        g_checked_keys.insert(row.key_path);
      } else {
        g_checked_keys.erase(row.key_path);
      }
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Checked: this setting is part of your personal defaults after you save.");
    }

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(row.key_path.c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(FormatDiffValue(row.current_value).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(FormatDiffValue(row.default_value).c_str());

    ImGui::TableNextColumn();
    // Source is "is this key written in my override file", not "does its value differ from
    // factory": a value deliberately saved that happens to equal the factory one is still the
    // user's, and hiding that would leave them unable to see what they saved. It reports the file
    // as it was when the panel opened — what the checkbox would CHANGE it to is the checkbox's own
    // job to say.
    //
    // A Selectable rather than plain text because the distinction needs a hover explanation, and
    // text carries no item to hover (nor an id, which is also what lets a test address the row's
    // source without reading pixels). NoAutoClosePopups: a stray click on a read-only cell must
    // not dismiss the panel.
    const std::string source_id =
        std::string(row.has_saved_override ? "Mine" : "Factory") + "###source_" + row.key_path;
    ImGui::Selectable(source_id.c_str(), false, ImGuiSelectableFlags_NoAutoClosePopups);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", row.has_saved_override ?
                                  "Saved in your personal defaults file. Un-check it to remove it on save." :
                                  "The built-in value. Nothing is saved for this setting.");
    }

    ImGui::TableNextColumn();
    if (!row.warnings.empty()) {
      ImGui::TextUnformatted(ICON_FA_TRIANGLE_EXCLAMATION);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", row.warnings.front().c_str());
      }
    }
  }
  ImGui::EndTable();
}

// Display format for a §1 std cell showing `value`, e.g. "%.7g".
//
// Built per-value from RoundTripPrecisionForAxisPresetStd rather than a fixed digit count: the
// domains are OPEN, so clamping Column's std lands on nextafter(10, 0) = 9.99999905 (needs 7
// digits) while clamping Lowitz's lands on nextafter(15, +inf) = 15.000001 (needs 8) — a fixed
// "%.7g" renders the first correctly but rounds the second to "15", directly under a line that
// says the value must stay GREATER than 15. Sharing the precision rule with FormatAxisPresetStd
// (used for the adjacent status/warning text) is what keeps the input box and the prose that
// explains it from ever disagreeing about what the stored value is.
std::string PresetStdDisplayFormat(float value) {
  char format[8];
  std::snprintf(format, sizeof(format), "%%.%dg", RoundTripPrecisionForAxisPresetStd(value));
  return format;
}

// Reload every §1 std input from the working copy (factory value, or what the copy holds). Called
// on open and after every committed edit, so the boxes show the value that would be stored rather
// than whatever the user last typed into them.
void RefreshPresetStdBuffers() {
  g_preset_std_buffers = PresetStdBuffers{};
  for (const auto& entry : kAxisPresets) {
    g_preset_std_buffers.slots[static_cast<std::size_t>(entry.id)] = EffectiveCopyPresetZenith(entry).std;
  }
}

// One read-only axis row: Type / Mean / Std as DISABLED typed controls rather than as text.
// Disabled controls, not plain labels, because the point is to show the shape of the value (a
// distribution picker, a number) while being honest that it cannot be changed here — a text cell
// would leave a user wondering where the editor for it is, and an enabled control would be a lie.
void RenderReadOnlyAxisRow(const char* axis_label, const AxisDist& dist) {
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TextUnformatted(axis_label);

  ImGui::TableNextColumn();
  ImGui::BeginDisabled();
  int type_index = static_cast<int>(dist.type);
  ImGui::SetNextItemWidth(-FLT_MIN);
  // SetNextComboPopupTopMost even though a disabled combo cannot open: this panel is a
  // BeginPopupModal, so the contract in panels.cpp applies to every combo it hosts, and the call
  // being already in place is what keeps a future decision to enable this cell from regressing
  // the popup back behind the modal.
  SetNextComboPopupTopMost();
  ImGui::Combo("##type", &type_index, kAxisDistTypeComboItems);
  ImGui::EndDisabled();

  ImGui::TableNextColumn();
  ImGui::BeginDisabled();
  float mean = dist.mean;
  ImGui::SetNextItemWidth(-FLT_MIN);
  ImGui::InputFloat("##mean", &mean, 0.0f, 0.0f, "%.1f");
  ImGui::EndDisabled();

  ImGui::TableNextColumn();
  ImGui::BeginDisabled();
  float std_value = dist.std;
  ImGui::SetNextItemWidth(-FLT_MIN);
  ImGui::InputFloat("##std", &std_value, 0.0f, 0.0f, PresetStdDisplayFormat(std_value).c_str());
  ImGui::EndDisabled();

  ImGui::TableNextColumn();  // warning cell: read-only rows never carry one
}

// The Zenith row of a preset that HAS an adjustable face: identical to the read-only row except
// the Std cell is live and the warning cell can fill.
void RenderEditableZenithRow(const AxisPresetEntry& entry) {
  const auto slot = static_cast<std::size_t>(entry.id);
  const AxisDist zenith = EffectiveCopyPresetZenith(entry);

  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TextUnformatted("Zenith");

  ImGui::TableNextColumn();
  ImGui::BeginDisabled();
  int type_index = static_cast<int>(zenith.type);
  ImGui::SetNextItemWidth(-FLT_MIN);
  SetNextComboPopupTopMost();
  ImGui::Combo("##type", &type_index, kAxisDistTypeComboItems);
  ImGui::EndDisabled();

  ImGui::TableNextColumn();
  ImGui::BeginDisabled();
  float mean = zenith.mean;
  ImGui::SetNextItemWidth(-FLT_MIN);
  ImGui::InputFloat("##mean", &mean, 0.0f, 0.0f, "%.1f");
  ImGui::EndDisabled();

  ImGui::TableNextColumn();
  ImGui::SetNextItemWidth(-FLT_MIN);
  const std::string std_id = std::string("###preset_std_") + entry.override_json_name;
  ImGui::InputFloat(std_id.c_str(), &g_preset_std_buffers.slots[slot], 0.0f, 0.0f,
                    PresetStdDisplayFormat(g_preset_std_buffers.slots[slot]).c_str());
  if (ImGui::IsItemDeactivatedAfterEdit()) {
    // Clamp WITHOUT writing: this is a §1 edit, and §1 edits go into the working copy like every
    // other edit in this panel. Leaving this one on the immediate-write path would only have
    // treated half the problem — and worse than not treating it at all, because the next Save
    // writes the copy over the whole file and would have silently undone it.
    const AxisPresetClampResult result = ClampAxisPresetZenithStdForSave(entry.id, g_preset_std_buffers.slots[slot]);
    // The message is empty on a clean value, which is also how the warning cell is cleared — one
    // assignment covers both directions, so a warning cannot outlive the value that caused it.
    g_preset_warnings.slots[slot] = result.message;
    if (!result.accepted) {
      g_status_message = result.message;
    } else {
      WriteAxisPresetZenithStdToDoc(g_copy_doc, entry.id, result.stored_value);
      if (result.clamped) {
        // Says "adjusted", not just the number. A status line reading "9.99999905" after the user
        // typed 25 reads as though nothing happened to their number — the icon in the warning
        // column carries the full reason, but the line the user is already looking at has to admit
        // the value changed.
        g_status_message = std::string("Adjusted to ") + FormatAxisPresetStd(result.stored_value) +
                           "; press Save to store it as the " + entry.label + " zenith std.";
      } else {
        g_status_message = std::string("Press Save to store ") + FormatAxisPresetStd(result.stored_value) + " as the " +
                           entry.label + " zenith std.";
      }
    }
    RefreshPresetStdBuffers();
  }

  ImGui::TableNextColumn();
  if (!g_preset_warnings.slots[slot].empty()) {
    // A Selectable rather than plain text, for the same reason the Source cell is one: the icon
    // needs a hover explanation, and text carries no item to hover — nor an id, which is also what
    // lets a test say "this preset is showing a warning" without reading pixels.
    // NoAutoClosePopups: a stray click on a read-only cell must not dismiss the panel.
    const std::string warning_id =
        ICON_FA_TRIANGLE_EXCLAMATION "###preset_warning_" + std::string(entry.override_json_name);
    ImGui::Selectable(warning_id.c_str(), false, ImGuiSelectableFlags_NoAutoClosePopups);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", g_preset_warnings.slots[slot].c_str());
    }
  }
}

// One preset: a collapsed row that opens onto its three axes. Collapsed by default — six presets
// times nine fields laid flat is a wall no one reads, and the question a user arrives with is
// about ONE preset.
void RenderPresetEntry(const AxisPresetEntry& entry) {
  const auto slot = static_cast<std::size_t>(entry.id);
  const bool has_override = CopyPresetZenithStd(entry).has_value();

  // Label carries "(mine)" for a preset the user has retuned, so the collapsed view already
  // answers "which of these have I changed" without opening all six.
  const std::string header_id =
      std::string(entry.label) + (has_override ? " (mine)" : "") + "###preset_" + AxisPresetLabel(entry.id);
  if (!ImGui::TreeNode(header_id.c_str())) {
    return;
  }

  if (!entry.has_adjustable_zenith_std) {
    // AC7: no input that would be ignored. Random is three uniform-360 axes — there is no narrow
    // distribution to retune, and a box that accepted a number and discarded it would be worse
    // than saying so.
    ImGui::TextWrapped(
        "%s has no adjustable value: it is defined as three full-360 uniform axes, so there is no "
        "spread to tune. The values below are shown for reference.",
        entry.label);
  } else {
    ImGui::TextWrapped("Zenith std must stay %s, otherwise this stops being recognised as %s.",
                       DescribeAxisPresetZenithStdDomain(entry.id).c_str(), entry.label);
  }

  constexpr ImGuiTableFlags kFlags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
  const std::string table_id = std::string("##preset_table_") + AxisPresetLabel(entry.id);
  if (ImGui::BeginTable(table_id.c_str(), 5, kFlags)) {
    ImGui::TableSetupColumn("Axis", ImGuiTableColumnFlags_WidthStretch, 0.6f);
    ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn("Mean", ImGuiTableColumnFlags_WidthStretch, 0.8f);
    ImGui::TableSetupColumn("Std", ImGuiTableColumnFlags_WidthStretch, 0.8f);
    ImGui::TableSetupColumn("!", ImGuiTableColumnFlags_WidthFixed, 24.0f);
    ImGui::TableHeadersRow();

    if (entry.has_adjustable_zenith_std) {
      ImGui::PushID("zenith");
      RenderEditableZenithRow(entry);
      ImGui::PopID();
    } else {
      ImGui::PushID("zenith");
      RenderReadOnlyAxisRow("Zenith", entry.zenith);
      ImGui::PopID();
    }
    ImGui::PushID("azimuth");
    RenderReadOnlyAxisRow("Azimuth", entry.azimuth);
    ImGui::PopID();
    ImGui::PushID("roll");
    RenderReadOnlyAxisRow("Roll", entry.roll);
    ImGui::PopID();
    ImGui::EndTable();
  }

  if (entry.has_adjustable_zenith_std) {
    const std::string restore_id =
        ICON_FA_ARROW_ROTATE_LEFT " Restore to factory###preset_restore_" + std::string(entry.override_json_name);
    ImGui::BeginDisabled(!has_override);
    if (ImGui::Button(restore_id.c_str())) {
      // Copy only, like the std edit above. The old code had a failure branch here because it
      // wrote the file; dropping the override from an in-memory document has none, which is also
      // why the warning is now cleared unconditionally: the value it described is gone from the
      // copy, so there is no state in which it still applies.
      EraseAxisPresetZenithStdFromDoc(g_copy_doc, entry.id);
      g_status_message = std::string(entry.label) + " will go back to its built-in value when you save.";
      g_preset_warnings.slots[slot].clear();
      RefreshPresetStdBuffers();
    }
    ImGui::EndDisabled();
  }

  ImGui::TreePop();
}

void RenderPresetLibrary() {
  ImGui::TextWrapped(
      "Retune a built-in preset so its button writes your value. The allowed range per preset is "
      "what keeps the preset recognisable as itself; a value outside it is adjusted to the nearest "
      "one that is, and said so.");
  for (const auto& entry : kAxisPresets) {
    // Custom is the classifier's "none of the above", not a built-in identity, so there is nothing
    // for a library to hold about it.
    if (entry.id == AxisPreset::kCustom) {
      continue;
    }
    RenderPresetEntry(entry);
  }
}

}  // namespace

void OpenDefaultsPanel(GuiState& state, DefaultsPanelSection initial_section) {
  state.defaults_panel_open = true;
  g_pending_initial_section = initial_section;
  // Whole-value resets, not "clear the parts that look stale": every opening of this panel is a
  // fresh decision, and a checkbox left over from an earlier opening would be an intent the user
  // stated about a session they have already closed.
  //
  // This is also where the previous session's working copy is discarded — closing the panel needs
  // no teardown of its own, because nothing survives this assignment. "Close = discard" is
  // therefore a property of the state's lifetime rather than a cleanup step someone can forget on
  // one of the ways out (the button today, an X or Esc later).
  // Belt-and-suspenders, not a fix for a live bug: ReadActiveOverlayDoc()'s current implementation
  // (ReadOverlayJsonIfPresent) already turns a hand-edited/interrupted-write file whose top level
  // is not an object into json::object() itself, so g_snapshot_doc is never actually non-object
  // today. But the copy model's every mutator (ApplyCheckedRowsToDoc and §1's preset writes)
  // assumes an object root, and that assumption should not
  // depend on an implementation detail two calls away that nothing here names. Establishing the
  // guarantee explicitly at the copy's single origin makes it true by construction of THIS
  // function, not by a property of whichever read function it happens to call today.
  g_snapshot_doc = ReadActiveOverlayDoc();
  if (!g_snapshot_doc.is_object()) {
    g_snapshot_doc = nlohmann::json::object();
  }
  g_copy_doc = g_snapshot_doc;
  g_search_filter.Clear();
  g_row_filter = DefaultsRowFilter::kAll;
  g_status_message.clear();
  g_preset_warnings = PresetWarnings{};
  RefreshPresetStdBuffers();
  RefreshRows(state);
  // After RefreshRows, never before: the checkbox set is a function of the rows.
  RecomputeCheckedKeys();
}

void RenderDefaultsPanel(GuiState& state) {
  if (!state.defaults_panel_open) {
    return;
  }
  if (!ImGui::IsPopupOpen(kDefaultsPanelTitle)) {
    ImGui::OpenPopup(kDefaultsPanelTitle);
  }

  // Fixed size on appearance rather than AlwaysAutoResize: the content is a table whose height is
  // a function of how many settings differ, and a modal that changes size with the diff would
  // make every visual-regression scene a different capture rectangle.
  ImGui::SetNextWindowSize(ImVec2(760.0f, 560.0f), ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal(kDefaultsPanelTitle, nullptr, ImGuiWindowFlags_NoSavedSettings)) {
    return;
  }

  ImGui::TextWrapped(
      "These are the settings a NEW document starts from. Adopting a change writes it to your "
      "personal defaults file; opening an existing file always uses the values in that file.");
  ImGui::Separator();

  RenderListControls();
  ImGui::Separator();

  // The two sections scroll inside a child; the action row below stays pinned. Without the split,
  // the settings list (40+ rows) pushes Save / Close off the bottom of a fixed-size modal — the
  // user would have to scroll past every setting to reach the button that commits their decision.
  // The reserved footer height is constant (one status line is budgeted whether or not there is a
  // message) so the body does not resize as the panel reports what it just did.
  const float footer_height =
      ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
  ImGui::BeginChild("##defaults_sections", ImVec2(0.0f, -footer_height));

  // §1 — the preset library (namespace 2). Its own section because a preset is not a GuiState
  // field: it never produces a row in the list below, and it is edited in place rather than
  // checked on or off.
  if (SectionHeader("Presets###defaults_presets", DefaultsPanelSection::kPresets, /*open_when_pending=*/true)) {
    RenderPresetLibrary();
  }

  // §2 — the settings list. The count is the number of rows CURRENTLY SHOWN against the total, so
  // a narrowed list says so in its own header rather than looking like a shorter settings set.
  int visible_count = 0;
  for (const auto& row : g_rows) {
    if (RowIsVisible(row)) {
      ++visible_count;
    }
  }
  char settings_label[96];
  snprintf(settings_label, sizeof(settings_label), "Settings (%d of %d)###defaults_settings", visible_count,
           static_cast<int>(g_rows.size()));
  if (SectionHeader(settings_label, DefaultsPanelSection::kSettings, /*open_when_pending=*/true)) {
    if (visible_count == 0) {
      ImGui::TextDisabled("No setting matches the search box and filter.");
    } else {
      RenderSettingsTable();
    }
  }

  ImGui::EndChild();

  // One frame of forced section state is enough; from here the user's own clicks own it.
  g_pending_initial_section.reset();

  ImGui::Separator();
  // Budgeted unconditionally (see footer_height): an empty line keeps the body height constant.
  ImGui::TextWrapped("%s", g_status_message.c_str());

  // Commit-and-stay rather than commit-and-close: after a save the adopted rows read as "Mine",
  // which IS the confirmation that the write landed, and the failure path (no writable config
  // directory) has somewhere to say so.
  if (ImGui::Button(ICON_FA_FLOPPY_DISK " Save as my defaults###defaults_save", ImVec2(200.0f, 0.0f))) {
    // Read-only preview of the document CommitCopy is about to write, so "nothing changed" is
    // decided by comparing that document with the one on disk rather than by counting clicks. The
    // count of checked rows would be the wrong measure twice over: it says nothing about §1's
    // preset edits, and a checkbox ticked and un-ticked again is a change by any click-counting
    // rule and no change at all by this one.
    nlohmann::json preview;
    const bool has_changes = BuildNextDoc(state, preview) && preview != g_snapshot_doc;
    const bool committed = CommitCopy(state);
    if (committed) {
      // Deliberately not "saved N settings". Under the merged model the checkbox set covers EVERY
      // row, so any count would have to pick a denominator ("all checked" is most of the list;
      // "newly checked" is a diff the user did not ask about) and the number would read as a
      // measure of what happened. It is not one — what happened is that the file now matches the
      // list, and the list is on screen.
      g_status_message = has_changes ? "Your personal defaults were updated." : "No changes to save.";
    } else {
      g_status_message = "Could not write your defaults file; nothing was saved.";
    }
    RefreshRows(state);
    if (committed) {
      // Re-freeze the baseline against the document that was just written — after RefreshRows, so
      // it is computed from the re-anchored rows. Skipped on failure, where the user's checkbox
      // set is still an uncommitted intent they may want to retry.
      RecomputeCheckedKeys();
    }
  }
  ImGui::SameLine();
  // Cancel semantics: the working copy is simply not committed. Nothing is written and nothing is
  // torn down here — the next OpenDefaultsPanel replaces every piece of session state wholesale,
  // so any other way out of this popup (an X, Esc) discards the copy identically without needing
  // its own path.
  if (ImGui::Button(ICON_FA_XMARK " Close###defaults_close", ImVec2(120.0f, 0.0f))) {
    state.defaults_panel_open = false;
    ImGui::CloseCurrentPopup();
  }

  // "Reset all" sits in the pinned action row rather than inside the list. It is not a per-row
  // operation: it clears EVERY row's checkbox at once, so burying it among the rows would
  // understate its reach — and a destructive control a user has to scroll 40 rows to find is also
  // one they cannot check the state of before pressing.
  ImGui::SameLine();
  const float reset_width = 220.0f;
  ImGui::SetCursorPosX(ImGui::GetWindowWidth() - reset_width - ImGui::GetStyle().WindowPadding.x);
  PushDestructiveStyle();
  if (ImGui::Button(ICON_FA_TRASH " Reset all my defaults###defaults_reset_all", ImVec2(reset_width, 0.0f))) {
    // "Un-check everything", and nothing else. It touches neither the file (405.5's write-through
    // behavior, which emptied the file while the screen did not move) NOR the working copy
    // (405.6's, which had to be paired with a separate pass over the checkbox set to stop Save
    // re-adopting the rows it had just cleared). With the checkbox as the single expression of
    // "this key is in my defaults", clearing the set IS the reset, and Save applies it like any
    // other state of the list — one rule, no second half to keep in step.
    //
    // Nothing is taken away: any row can be re-checked before saving, and closing without saving
    // discards this like every other edit.
    g_checked_keys.clear();
    g_status_message =
        "Every personal default will be removed when you save; new documents will start from factory values.";
  }
  PopDestructiveStyle();

  ImGui::EndPopup();
}

void ResetDefaultsPanelTestState() {
  g_rows.clear();
  g_snapshot_doc = nlohmann::json::object();
  g_copy_doc = nlohmann::json::object();
  g_checked_keys.clear();
  g_initial_checked_keys.clear();
  g_search_filter.Clear();
  g_row_filter = DefaultsRowFilter::kAll;
  g_pending_initial_section.reset();
  g_status_message.clear();
  g_preset_warnings = PresetWarnings{};
  g_preset_std_buffers = PresetStdBuffers{};
}

}  // namespace lumice::gui
