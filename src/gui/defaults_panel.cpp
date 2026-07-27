#include "gui/defaults_panel.hpp"

#include <cfloat>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/destructive_style.hpp"
#include "gui/gui_logger.hpp"
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

// Row set, read once per open (and re-read after each write). Not rebuilt per frame: the panel is
// modal, so the document cannot change underneath it, and BuildDefaultDiffRows reads the override
// file — a per-frame call would re-read (and re-count the degradation of) that file 60x a second.
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
  g_rows = BuildDefaultDiffRows(state);
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

  // Applied after the loop: RevertOneDefault rebuilds g_rows, and mutating the container being
  // iterated would invalidate the loop's own references.
  if (!revert_key.empty()) {
    if (RevertOneDefault(revert_key)) {
      g_status_message = "Reverted '" + revert_key + "' to the factory value.";
    } else {
      g_status_message = "Could not write your defaults file; nothing was changed.";
    }
    RefreshRows(state);
  }
}

// Display format for every std cell in §1.
//
// %g rather than a fixed number of decimals, and 7 significant digits rather than 6, because of
// exactly one value: the clamp target. The domains are OPEN, so clamping Column's std lands on
// nextafter(10, 0) = 9.99999905 — which "%.3f" renders as "10.000" and "%.6g" as "10", directly
// under a line that says the value must stay LESS than 10. A user reading that has been told two
// contradictory things and has no way to tell which is true. 7 digits shows 9.999999, and leaves
// the values people actually type alone (0.3 stays "0.3", not "0.300000").
constexpr const char* kPresetStdDisplayFormat = "%.7g";

// Reload every §1 std input from what is actually in effect (factory value, or the user's stored
// override). Called on open and after every write, so the boxes show the stored truth rather than
// whatever the user last typed into them.
void RefreshPresetStdBuffers() {
  g_preset_std_buffers = PresetStdBuffers{};
  for (const auto& entry : kAxisPresets) {
    g_preset_std_buffers.slots[static_cast<std::size_t>(entry.id)] = EffectiveAxisPresetZenith(entry).std;
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
  ImGui::InputFloat("##std", &std_value, 0.0f, 0.0f, kPresetStdDisplayFormat);
  ImGui::EndDisabled();

  ImGui::TableNextColumn();  // warning cell: read-only rows never carry one
}

// The Zenith row of a preset that HAS an adjustable face: identical to the read-only row except
// the Std cell is live and the warning cell can fill.
void RenderEditableZenithRow(const AxisPresetEntry& entry) {
  const auto slot = static_cast<std::size_t>(entry.id);
  const AxisDist zenith = EffectiveAxisPresetZenith(entry);

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
  ImGui::InputFloat(std_id.c_str(), &g_preset_std_buffers.slots[slot], 0.0f, 0.0f, kPresetStdDisplayFormat);
  if (ImGui::IsItemDeactivatedAfterEdit()) {
    const AxisPresetWriteResult result = SaveAxisPresetZenithStdOverride(entry.id, g_preset_std_buffers.slots[slot]);
    // The message is empty on a clean write, which is also how the warning cell is cleared — one
    // assignment covers both directions, so a warning cannot outlive the value that caused it.
    g_preset_warnings.slots[slot] = result.message;
    if (!result.written) {
      g_status_message = result.message;
    } else if (result.clamped) {
      // Says "adjusted", not "saved". A status line reading "Saved 9.99999905" after the user typed
      // 25 is technically true and reads as though nothing happened to their number — the icon in
      // the warning column carries the full reason, but the line the user is already looking at has
      // to admit the value changed.
      g_status_message = std::string("Adjusted to ") + FormatAxisPresetStd(result.stored_value) + " and saved as the " +
                         entry.label + " zenith std.";
    } else {
      g_status_message =
          std::string("Saved ") + FormatAxisPresetStd(result.stored_value) + " as the " + entry.label + " zenith std.";
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
  const bool has_override = GetUserAxisPresetZenithStdOverride(entry.id).has_value();

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
      if (RevertOneAxisPresetOverride(entry.id)) {
        g_status_message = std::string(entry.label) + " was restored to its built-in value.";
      } else {
        g_status_message = "Could not write your defaults file; nothing was changed.";
      }
      // Cleared unconditionally: after a restore the value IS the factory one, so a warning about
      // the value that was just removed would describe something no longer there.
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
    const std::vector<std::string> accepted = CheckedPendingKeys();
    if (accepted.empty()) {
      g_status_message = "No changes were selected, so nothing was saved.";
    } else if (SaveAcceptedDefaults(accepted, state)) {
      g_status_message = "Saved " + std::to_string(accepted.size()) + " setting(s) as your defaults.";
      g_excluded_keys.clear();
    } else {
      g_status_message = "Could not write your defaults file; nothing was saved.";
    }
    RefreshRows(state);
  }
  ImGui::SameLine();
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
    if (ResetAllDefaults()) {
      g_status_message = "All personal defaults were removed; new documents start from factory values.";
    } else {
      g_status_message = "Could not write your defaults file; nothing was changed.";
    }
    RefreshRows(state);
  }
  PopDestructiveStyle();

  ImGui::EndPopup();
}

void ResetDefaultsPanelTestState() {
  g_rows.clear();
  g_excluded_keys.clear();
  g_search_filter.Clear();
  g_pending_initial_section.reset();
  g_status_message.clear();
  g_preset_warnings = PresetWarnings{};
  g_preset_std_buffers = PresetStdBuffers{};
}

}  // namespace lumice::gui
