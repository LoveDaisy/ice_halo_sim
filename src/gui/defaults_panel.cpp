#include "gui/defaults_panel.hpp"

#include <cstdio>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/defaults_diff.hpp"
#include "gui/destructive_style.hpp"
#include "gui/gui_logger.hpp"
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
    ImGui::TextDisabled("The preset library is not editable yet.");
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
}

}  // namespace lumice::gui
