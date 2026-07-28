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
//   g_snapshot_doc — what was on disk when the panel opened. FROZEN for the session. It anchors the
//                    §2/§3 partition (RowNeedsAdoption compares against the defaults this document
//                    resolves), so rows do not jump between sections as the user edits. Anchoring
//                    the partition on the working copy instead would move a row out of §2 the
//                    moment the copy gained its value — with the user having pressed nothing.
//   g_copy_doc     — what pressing Save right now would write. Revert, Reset all and every §1
//                    preset edit mutate this and nothing else. It is also the source for each
//                    row's has_saved_override, which is how §3's Source cell and Revert button
//                    show an uncommitted Revert immediately (the visible feedback the old
//                    write-through model failed to give).
// ------------------------------------------------------------------------------------------------
nlohmann::json g_snapshot_doc = nlohmann::json::object();
nlohmann::json g_copy_doc = nlohmann::json::object();

// Row set, rebuilt on open and after every edit of the copy. Not rebuilt per frame: BuildDefaultDiffRows
// re-serializes the whole GuiState twice, and a per-frame call would do that 60x a second.
std::vector<DefaultDiffRow> g_rows;

// §2 rows the user UNCHECKED. Stored as the exclusion set rather than the selection set so a row
// is checked by default with no per-open initialization pass: pressing the entry button already
// means "adopt my current settings", and the checkbox exists to take individual rows back out.
std::set<std::string> g_excluded_keys;

ImGuiTextFilter g_search_filter;

// Consumed on the first frame after an open: which section starts expanded. nullopt on every
// other frame, so a user who collapses §2 keeps it collapsed while the panel stays open.
std::optional<DefaultsPanelSection> g_pending_initial_section;

// Outcome of the last write, shown at the bottom of the panel. Deliberately carries NO directory
// path: this string is inside the visual-regression capture, and a machine-specific path would
// bake one developer's home directory into a committed reference image.
std::string g_status_message;

// §1's per-preset warning text, indexed by AxisPreset. The library's own channel, NOT
// DefaultDiffRow::warnings: BuildDefaultDiffRows walks the serialized GuiState tree, and the
// presets subtree is not in it — a preset can never produce a DefaultDiffRow to hang a warning on
// (scrum D1: "presets are not GuiState fields, they do not enter the §2 diff"). Same visual form
// as §2's "!" column, different data source, on purpose.
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
  return g_excluded_keys.find(key_path) == g_excluded_keys.end();
}

void RefreshRows(const GuiState& state) {
  // Rows are built against the FROZEN snapshot (so the §2/§3 partition holds still for the whole
  // session), then each row's "is this key mine" is re-answered from the WORKING COPY (so an
  // uncommitted Revert or Reset all is visible immediately). The two data sources are the whole
  // point of the split — see the comment on g_snapshot_doc.
  g_rows = BuildDefaultDiffRows(state, g_snapshot_doc);
  for (auto& row : g_rows) {
    row.has_saved_override = DocHasKeyPath(g_copy_doc, row.key_path);
  }
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

// The panel's ONE write. Every other action in this file edits g_copy_doc and stops there.
//
// `accepted` are the §2 rows still checked; they are folded into the copy here rather than when
// the checkbox is clicked, because a checkbox is a statement about what Save should do, not an
// edit in its own right (unchecking one must not look like a change to the document).
bool CommitCopy(const GuiState& state, const std::vector<std::string>& accepted) {
  // Built beside the copy and only adopted once the write succeeds. On any failure below the copy
  // is left exactly as the user has it, so a second attempt (after fixing whatever blocked the
  // write) still carries their edits instead of a half-applied version of them.
  nlohmann::json next = g_copy_doc;
  if (!ApplyAcceptedDefaultsToDoc(next, accepted, state)) {
    return false;
  }
  if (!WriteOverlayDocument(next)) {
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
  //     document. That re-anchors the §2/§3 partition, which is what moves the adopted rows out
  //     of §2 — the visible confirmation that the save landed.
  //  3. the exclusion set is dropped: it described the previous set of pending rows, and after the
  //     re-anchor the rows it named are no longer pending.
  for (const auto& entry : kAxisPresets) {
    const auto after = ReadAxisPresetZenithStdFromDoc(next, entry.id);
    const auto before = ReadAxisPresetZenithStdFromDoc(g_snapshot_doc, entry.id);
    if (after != before) {
      AdoptAxisPresetZenithStdOverrideInMemory(entry.id, after);
    }
  }
  g_copy_doc = std::move(next);
  g_snapshot_doc = g_copy_doc;
  g_excluded_keys.clear();
  return true;
}

std::vector<std::string> CheckedPendingKeys() {
  std::vector<std::string> keys;
  for (const auto& row : g_rows) {
    if (RowNeedsAdoption(row) && IsRowChecked(row.key_path)) {
      keys.push_back(row.key_path);
    }
  }
  return keys;
}

// Section header whose open state is forced only on the frame an entry point requested it.
bool SectionHeader(const char* label, DefaultsPanelSection section, bool open_when_pending) {
  if (g_pending_initial_section.has_value()) {
    ImGui::SetNextItemOpen(open_when_pending && *g_pending_initial_section == section, ImGuiCond_Always);
  }
  return ImGui::CollapsingHeader(label);
}

// §3's header: it is never an entry target, so it is forced CLOSED on an initial-section frame
// (rather than left to whatever the previous opening left in ImGui's storage) — "§3 starts
// collapsed" is a product requirement, not a remembered preference.
bool OtherEntriesHeader(const char* label) {
  if (g_pending_initial_section.has_value()) {
    ImGui::SetNextItemOpen(false, ImGuiCond_Always);
  }
  return ImGui::CollapsingHeader(label);
}

void SetupCommonColumns(bool with_checkbox) {
  if (with_checkbox) {
    ImGui::TableSetupColumn("##adopt", ImGuiTableColumnFlags_WidthFixed, 24.0f);
  }
  ImGui::TableSetupColumn("Setting", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("Current", ImGuiTableColumnFlags_WidthStretch, 1.0f);
}

void RenderPendingTable() {
  constexpr ImGuiTableFlags kFlags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
  if (!ImGui::BeginTable("##defaults_pending_table", 5, kFlags)) {
    return;
  }
  SetupCommonColumns(/*with_checkbox=*/true);
  ImGui::TableSetupColumn("Effective default", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  // The warning column (scrum D8). Built and sized here, filled by 405.5's preset-range clamp:
  // owner chose a column over a tooltip precisely because a tooltip is invisible to the reference
  // images, so the column has to exist in the captured layout before there is anything to put in
  // it.
  ImGui::TableSetupColumn("!", ImGuiTableColumnFlags_WidthFixed, 24.0f);
  ImGui::TableHeadersRow();

  for (const auto& row : g_rows) {
    if (!RowNeedsAdoption(row) || !g_search_filter.PassFilter(row.key_path.c_str())) {
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
        g_excluded_keys.erase(row.key_path);
      } else {
        g_excluded_keys.insert(row.key_path);
      }
    }

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(row.key_path.c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(FormatDiffValue(row.current_value).c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(FormatDiffValue(row.default_value).c_str());

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

void RenderOtherTable(const GuiState& state) {
  constexpr ImGuiTableFlags kFlags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
  if (!ImGui::BeginTable("##defaults_other_table", 4, kFlags)) {
    return;
  }
  SetupCommonColumns(/*with_checkbox=*/false);
  ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 110.0f);
  ImGui::TableSetupColumn("##revert", ImGuiTableColumnFlags_WidthFixed, 80.0f);
  ImGui::TableHeadersRow();

  std::string revert_key;
  for (const auto& row : g_rows) {
    if (RowNeedsAdoption(row) || !g_search_filter.PassFilter(row.key_path.c_str())) {
      continue;
    }
    ImGui::TableNextRow();

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(row.key_path.c_str());

    ImGui::TableNextColumn();
    ImGui::TextUnformatted(FormatDiffValue(row.current_value).c_str());

    ImGui::TableNextColumn();
    // Source is "is this key written in my override file", not "does its value differ from
    // factory": a value deliberately saved that happens to equal the factory one is still the
    // user's, and hiding that would leave them unable to see (or revert) what they saved.
    //
    // A Selectable rather than plain text because the distinction needs a hover explanation, and
    // text carries no item to hover (nor an id, which is also what lets a test say "this key is
    // rendered in §3" without reading pixels). NoAutoClosePopups: a stray click on a read-only
    // cell must not dismiss the panel.
    const std::string source_id =
        std::string(row.has_saved_override ? "Mine" : "Factory") + "###source_" + row.key_path;
    ImGui::Selectable(source_id.c_str(), false, ImGuiSelectableFlags_NoAutoClosePopups);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", row.has_saved_override ? "Saved in your personal defaults file. Revert removes it." :
                                                       "The built-in value. Nothing is saved for this setting.");
    }

    ImGui::TableNextColumn();
    if (row.has_saved_override) {
      const std::string revert_id = ICON_FA_ARROW_ROTATE_LEFT " Revert###revert_" + row.key_path;
      if (ImGui::SmallButton(revert_id.c_str())) {
        revert_key = row.key_path;
      }
    }
  }
  ImGui::EndTable();

  // Applied after the loop: RefreshRows rebuilds g_rows, and mutating the container being iterated
  // would invalidate the loop's own references.
  if (!revert_key.empty()) {
    // Copy only — no file is touched until Save. There is no failure branch left to report: a
    // key-level erase from an in-memory document cannot fail the way a write to a directory that
    // vanished can.
    ApplyRevertToDoc(g_copy_doc, revert_key);
    g_status_message = "'" + revert_key + "' will go back to the factory value when you save.";
    RefreshRows(state);
  }
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
    // A Selectable rather than plain text, for the same reason §3's source cell is one: the icon
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
  // fresh decision, and a row unchecked in an earlier opening silently staying unchecked would
  // contradict the "checked by default" contract the entry button promises.
  //
  // This is also where the previous session's working copy is discarded — closing the panel needs
  // no teardown of its own, because nothing survives this assignment. "Close = discard" is
  // therefore a property of the state's lifetime rather than a cleanup step someone can forget on
  // one of the ways out (the button today, an X or Esc later).
  g_snapshot_doc = ReadActiveOverlayDoc();
  g_copy_doc = g_snapshot_doc;
  g_excluded_keys.clear();
  g_search_filter.Clear();
  g_status_message.clear();
  g_preset_warnings = PresetWarnings{};
  RefreshPresetStdBuffers();
  RefreshRows(state);
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

  g_search_filter.Draw(ICON_FA_MAGNIFYING_GLASS " Filter###defaults_search", 240.0f);
  ImGui::Separator();

  // The three sections scroll inside a child; the action row below stays pinned. Without the
  // split, expanding §3 (40+ rows) pushes Save / Close off the bottom of a fixed-size modal — the
  // user would have to scroll past every setting to reach the button that commits their decision.
  // The reserved footer height is constant (one status line is budgeted whether or not there is a
  // message) so the body does not resize as the panel reports what it just did.
  const float footer_height =
      ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
  ImGui::BeginChild("##defaults_sections", ImVec2(0.0f, -footer_height));

  // §1 — the preset library (405.5). Present so the section order and the "entry point decides
  // which section is expanded" mechanism are the ones 405.5 will extend, not ones it has to
  // introduce.
  if (SectionHeader("Presets###defaults_presets", DefaultsPanelSection::kPresets, /*open_when_pending=*/true)) {
    RenderPresetLibrary();
  }

  int pending_count = 0;
  for (const auto& row : g_rows) {
    if (RowNeedsAdoption(row)) {
      ++pending_count;
    }
  }
  char pending_label[96];
  snprintf(pending_label, sizeof(pending_label), "Changes to adopt (%d)###defaults_pending", pending_count);
  if (SectionHeader(pending_label, DefaultsPanelSection::kPendingChanges, /*open_when_pending=*/true)) {
    if (pending_count == 0) {
      ImGui::TextDisabled("Nothing differs from your current defaults.");
    } else {
      RenderPendingTable();
    }
  }

  char other_label[96];
  snprintf(other_label, sizeof(other_label), "Other entries (%d)###defaults_other",
           static_cast<int>(g_rows.size()) - pending_count);
  if (OtherEntriesHeader(other_label)) {
    RenderOtherTable(state);
  }

  ImGui::EndChild();

  // One frame of forced section state is enough; from here the user's own clicks own it.
  g_pending_initial_section.reset();

  ImGui::Separator();
  // Budgeted unconditionally (see footer_height): an empty line keeps the body height constant.
  ImGui::TextWrapped("%s", g_status_message.c_str());

  // Commit-and-stay rather than commit-and-close: after a save the adopted rows move from §2 to
  // §3 with source "Mine", which IS the confirmation that the write landed, and the failure path
  // (no writable config directory) has somewhere to say so.
  if (ImGui::Button(ICON_FA_FLOPPY_DISK " Save as my defaults###defaults_save", ImVec2(200.0f, 0.0f))) {
    // Save commits the WHOLE copy, not just the checked §2 rows: the Reverts, the Reset all and
    // the §1 preset edits are already in it. `accepted` being empty therefore does NOT mean
    // "nothing to do" — the old panel could say that only because those other edits had already
    // written themselves to disk behind the user's back.
    const std::vector<std::string> accepted = CheckedPendingKeys();
    // Read-only preview of what CommitCopy is about to write, so the status line can tell "nothing
    // in this session actually differs from what's on disk" apart from "the copy matched the
    // snapshot but there were accepted rows" — accepted.empty() alone conflates the two, since a
    // Revert/Reset/§1 edit followed by re-doing it back to the original value is empty-but-changed
    // by that test.
    nlohmann::json preview = g_copy_doc;
    const bool has_changes = ApplyAcceptedDefaultsToDoc(preview, accepted, state) && preview != g_snapshot_doc;
    if (CommitCopy(state, accepted)) {
      g_status_message = !has_changes ? "No changes to save." :
                         accepted.empty() ?
                                        "Your defaults were updated." :
                                        "Saved " + std::to_string(accepted.size()) + " setting(s) as your defaults.";
    } else {
      g_status_message = "Could not write your defaults file; nothing was saved.";
    }
    RefreshRows(state);
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

  // "Reset all" sits in the pinned action row rather than under §3's table. It is not a §3
  // operation: it removes EVERY personal default, including the ones §2 is offering to add, so
  // burying it under one section's rows would understate its reach — and a destructive control a
  // user has to scroll 40 rows to find is also one they cannot check the state of before pressing.
  ImGui::SameLine();
  const float reset_width = 220.0f;
  ImGui::SetCursorPosX(ImGui::GetWindowWidth() - reset_width - ImGui::GetStyle().WindowPadding.x);
  PushDestructiveStyle();
  if (ImGui::Button(ICON_FA_TRASH " Reset all my defaults###defaults_reset_all", ImVec2(reset_width, 0.0f))) {
    // Copy only, like every other edit here — this is the button whose old write-through behavior
    // made the panel dishonest, since it emptied the file while the screen did not move.
    ApplyResetAllToDoc(g_copy_doc);
    RefreshRows(state);
    // ...and un-check every pending row, because otherwise this button cannot do what it says.
    // §2's rows are checked by default, so Save would immediately re-adopt them and the "reset"
    // would write a full set of defaults straight back. With Save as the only write, that
    // contradiction is now reachable in one visible sequence (Reset all, then Save) instead of
    // being hidden behind a file that had already been emptied; the behavior AC2 asks for is that
    // Save then writes an EMPTY override set. Nothing is taken away — any row can be re-checked.
    std::set<std::string> excluded;
    for (const auto& row : g_rows) {
      if (RowNeedsAdoption(row)) {
        excluded.insert(row.key_path);
      }
    }
    g_excluded_keys = std::move(excluded);  // whole-value, like every other reset in this file
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
  g_excluded_keys.clear();
  g_search_filter.Clear();
  g_pending_initial_section.reset();
  g_status_message.clear();
  g_preset_warnings = PresetWarnings{};
  g_preset_std_buffers = PresetStdBuffers{};
}

}  // namespace lumice::gui
