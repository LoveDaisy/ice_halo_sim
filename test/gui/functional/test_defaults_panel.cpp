// The Settings modal: the panel where a user decides what a NEW document starts from.
//
// What this suite is for. The panel is a pure editor over two namespaces — the merged settings
// list and the preset library — and everything worth checking about it is a statement about what
// is ON SCREEN and what reaches the FILE, with a gap between the two that only exists while the
// panel is open. Three kinds of question live here and nowhere else:
//   * what a control SHOWS the moment the panel opens, which is a function of the file, the
//     document and the factory value at once, and is therefore not a property of any one of them;
//   * whether an edit stops at the working copy until Save is pressed — a claim about a click and
//     a file in the same breath, unfalsifiable without both;
//   * geometry: a frozen header row, two section headers that do not scroll, an action row pinned
//     to the bottom edge. None of these can be restated as "can I still click it": an item that
//     has scrolled off the top is perfectly clickable after the engine scrolls it back.
//
// Deliberately NOT here. Which rows exist and what each one's verdict is belongs to the diff
// engine (unit-correctness/gui/test_defaults_diff.cpp); what the store does with a document, and
// the precision rule the preset cells display through, belong to the store and its composition
// chain (unit-correctness/gui/test_user_defaults.cpp and
// composition-correctness/gui/test_user_defaults_chain.cpp); how the panel LOOKS is the
// defaults_panel_layout reference group under visual/. Nothing below re-asserts any of them —
// every case here starts from "those are right" and asks what the user can see and reach.
//
// What a user sees when these break: the settings they came for cannot be found from the main
// window; a checkbox says a setting is theirs when the file says otherwise; Reset all appears to
// clear everything and Save quietly puts it all back; a value typed into the table is accepted at
// a range the rest of the app refuses; the column headers scroll away halfway down a forty-row
// list; a preset they retuned reads as untouched, or a clamped value renders as the very number
// the panel just told them was not allowed.
//
// One thing this layer CANNOT answer, recorded rather than faked: the panel's status line is
// ImGui::TextWrapped, submitted with id 0, and the test engine only records items with an id. So
// "Adjusted to ...", "No changes to save." and "Your personal defaults were updated." are invisible
// here — every case below asserts the half of those propositions that has an observable (the value
// that got stored, the warning cell, the bytes in the file) and none of them pretends to check the
// sentence.

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/edit_modals.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/user_defaults.hpp"
#include "imgui_internal.h"
#include "imgui_te_utils.h"
#include "support/user_defaults_test_env.hpp"
#include "test_gui_shared.hpp"

namespace {

using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;
using lumice::test_user_defaults::ScopedUserConfigSource;
using nlohmann::json;

// ---------------------------------------------------------------------------------------------
// The fixture.
//
// Every case in this file needs the same four things in the same order, and the ORDER is the part
// that matters rather than the typing it saves: the config-source guard has to be installed BEFORE
// ResetTestState, because ResetTestState -> DoNew -> MakeNewDocumentState reads the process-wide
// source. Installed afterwards, the document under test would already carry whatever personal
// defaults the running machine has saved, and the case would be measuring that machine.
// ---------------------------------------------------------------------------------------------

class ScopedPanel {
 public:
  // `tag` names this case's own override directory and must be unique in this binary; two cases
  // sharing one silently share state.
  ScopedPanel(ImGuiTestContext* ctx, const char* tag)
      : ctx_(ctx), dir_(FreshOverlayDir(tag)), guard_(gui::UserConfigSource::kExplicitDir, dir_) {
    ResetTestState();
    ResetUserDefaultsChannels();
  }

  ScopedPanel(const ScopedPanel&) = delete;
  ScopedPanel& operator=(const ScopedPanel&) = delete;

  // Leaves the panel closed AND its preset nodes folded for whatever runs next in this
  // single-process suite.
  //
  // The fold state is the part that is easy to miss and expensive to get wrong. ImGui keeps a
  // TreeNode's open flag in the panel window's own storage, which outlives every reset this suite
  // performs: OpenDefaultsPanel re-derives the rows, the checkbox set and the section folds, and
  // the panel's own reset hook clears its TU statics, but neither owns ImGui's storage. So a case
  // that unfolds a preset hands the next one a library that is already open — and the visual
  // reference scenes downstream capture exactly one preset unfolded, which is a 16 dB difference
  // when a second one is showing. Measured, not assumed: with this teardown removed, the
  // presets_expanded and presets_warning references fail at 16.8 dB while the same scenes pass
  // when run on their own.
  //
  // In the destructor rather than at the end of each case because IM_CHECK expands to a return: a
  // case that fails halfway would otherwise hand its open nodes to the next one, and a cascade of
  // unrelated reds is the worst possible report.
  //
  // Honest limit, so the next person reading a red run does not chase it: the test context
  // short-circuits every action once an error is on record, so a case that FAILS gets no teardown
  // at all and can still leave a node unfolded. ResetTestState clears the panel's own statics for
  // the next case, but not ImGui's fold storage — so one genuine failure here may be followed by
  // reference scenes going red for no reason of their own. Fix the first red, then re-run.
  ~ScopedPanel() {
    // Reopened when a case closed the panel itself: the nodes are only reachable while it is up,
    // and whether this case ended with it open is not something the teardown should depend on.
    if (!gui::g_state.defaults_panel_open) {
      gui::OpenDefaultsPanel(gui::g_state, gui::DefaultsPanelSection::kPresets);
      ctx_->Yield(4);
    } else {
      ctx_->ItemOpen("**/###defaults_presets");
      ctx_->Yield(2);
    }
    for (const auto& entry : gui::kAxisPresets) {
      if (entry.id == gui::AxisPreset::kCustom) {
        continue;  // not a library entry; it has no node
      }
      ctx_->ItemClose((std::string("**/###preset_") + gui::AxisPresetLabel(entry.id)).c_str());
    }
    ctx_->Yield(2);

    // Through the button rather than by zeroing the flag: closing the way a user does also unwinds
    // ImGui's popup stack, and a popup left on the stack eats the next case's first clicks.
    ctx_->ItemClick("**/###defaults_close");
    ctx_->Yield(2);
    // Belt and braces for the one exit that does not answer the button (Esc re-opens the popup on
    // the next frame; see the case that pins it): the flag is what the OpenPopup guard reads.
    gui::g_state.defaults_panel_open = false;
    ctx_->Yield(2);
  }

  const std::filesystem::path& dir() const { return dir_; }

  void OpenOn(gui::DefaultsPanelSection section) const {
    gui::OpenDefaultsPanel(gui::g_state, section);
    ctx_->Yield(4);
  }

  void Close() const {
    if (gui::g_state.defaults_panel_open) {
      ctx_->ItemClick("**/###defaults_close");
      ctx_->Yield(2);
    }
  }

 private:
  ImGuiTestContext* ctx_;
  std::filesystem::path dir_;  // declared before guard_: the guard is constructed from it
  ScopedUserConfigSource guard_;
};

// A held mouse button that is released even when a check in the middle of a drag returns early.
//
// The drag cases below assert WHILE the button is down — that is the whole point of them — and
// IM_CHECK expands to a `return`, so a plain MouseDown/.../MouseUp pair would skip its own release
// on exactly the runs that fail. Defence in depth rather than the load-bearing mechanism: the
// engine already calls ImGuiTestEngine_ClearInput() at the top of every test for this ("Clear ImGui
// inputs to avoid key/mouse leaks from one test to another", imgui_te_engine.cpp). What the guard
// adds is that the release happens inside the case that pressed, so the widget under test still
// sees its release frame and hands its drag scratch back before anything else runs.
class ScopedMouseDown {
 public:
  ScopedMouseDown(ImGuiTestContext* ctx, const ImVec2& pos) : ctx_(ctx) {
    ctx_->MouseMoveToPos(pos);
    ctx_->MouseDown(0);
    ctx_->Yield(2);
  }

  ScopedMouseDown(const ScopedMouseDown&) = delete;
  ScopedMouseDown& operator=(const ScopedMouseDown&) = delete;

  ~ScopedMouseDown() { Release(); }

  void MoveTo(const ImVec2& pos) const {
    ctx_->MouseMoveToPos(pos);
    ctx_->Yield(2);
  }

  void Release() {
    if (released_) {
      return;
    }
    released_ = true;
    ctx_->MouseUp(0);
    ctx_->Yield(2);
  }

 private:
  ImGuiTestContext* ctx_;
  bool released_ = false;
};

// ---------------------------------------------------------------------------------------------
// Reading the panel.
// ---------------------------------------------------------------------------------------------

// Widget refs. The key path IS the widget id (every id this panel exposes uses "###", so the id is
// the short suffix alone), which makes these total functions of a row rather than an index that
// has to be kept in step with the row order.
std::string AdoptCheckboxRef(const std::string& key_path) {
  return "**/###adopt_" + key_path;
}

std::string SourceCellRef(const std::string& key_path) {
  return "**/###source_" + key_path;
}

// The three shapes a row's value cell can take. A slider cell is TWO items (the slider and its
// input box); a checkbox / colour / combo cell is one. The negative claim ("this row has no
// editor") has to cover all of them, so probing only the one the key would have used had it been
// registered proves nothing.
std::string ValueInputRef(const std::string& key_path) {
  return "**/##value_" + key_path + "_input";
}

std::string ValueWidgetRef(const std::string& key_path) {
  return "**/##value_" + key_path;
}

// The other half of a slider cell — the bar itself, which is the only one of the two that can be
// DRAGGED. ValueInputRef reaches the box beside it, and every case that types a number goes through
// that one; a case about what a drag leaves behind cannot.
std::string ValueSliderRef(const std::string& key_path) {
  return "**/##value_" + key_path + "_slider";
}

// A point `t` of the way across an item, on its vertical centre line. Read off the item's own rect
// rather than written as coordinates: how wide a value cell is belongs to the table.
ImVec2 PointAcross(const ImGuiTestItemInfo& info, float t) {
  return ImVec2(info.RectFull.Min.x + (info.RectFull.Max.x - info.RectFull.Min.x) * t,
                (info.RectFull.Min.y + info.RectFull.Max.y) * 0.5f);
}

// The Origin cell, addressed BY THE VALUE IT SHOULD BE SHOWING: that cell's id is the "##" form,
// so the id hashes the whole label and the item exists only if the cell drew exactly `expected`.
std::string OriginCellRef(const std::string& key_path, const std::string& expected_text) {
  return "**/" + expected_text + "##origin_" + key_path;
}

// The TEXT an item actually drew, as the engine recorded it when the item was submitted.
//
// This is the only way to assert on drawn text in this panel, and it is available at all only for
// items whose id is the "###" form — the display half of such a label is not part of the id, so
// addressing the item cannot tell you what it said. Two of this panel's claims are exactly about
// that display half (the section header's "N of M" count, the Source cell's Mine/Factory), and
// without this they would be unfalsifiable.
//
// DebugLabel is a 32-byte truncation, so every use below looks for a prefix or an early substring
// rather than comparing whole strings. functional/test_edit_modal.cpp reads it the same way and for
// the same reason.
std::string DrawnLabel(ImGuiTestContext* ctx, const std::string& ref) {
  const ImGuiTestItemInfo info = ctx->ItemInfo(ref.c_str(), ImGuiTestOpFlags_NoError);
  return info.ID == 0 ? std::string() : std::string(info.DebugLabel);
}

bool StartsWith(const std::string& s, const char* prefix) {
  return s.rfind(prefix, 0) == 0;
}

// Narrow the list to (at most) the rows whose key contains `text`.
//
// Not a convenience. The test engine's wildcard lookup only finds items it can SEE — clipped items
// are unaware of their labels — and the merged list is 40+ rows in a fixed-height modal. Filtering
// first makes "is this key rendered" a question about the panel rather than about where the scroll
// happened to be, and it exercises the search box in every case instead of only in the one written
// for it.
void FilterTo(ImGuiTestContext* ctx, const char* text) {
  ctx->ItemInputValue("**/###defaults_search", text);
  ctx->Yield(3);
}

// Whether a row's checkbox is ticked, asked of the REAL widget rather than of the panel's internal
// set (which is TU-private, and which a panel could keep correct while rendering something else).
// The row must be on screen — narrow with FilterTo first.
bool RowIsChecked(ImGuiTestContext* ctx, const std::string& key_path) {
  return ctx->ItemIsChecked(AdoptCheckboxRef(key_path).c_str());
}

void ToggleRow(ImGuiTestContext* ctx, const std::string& key_path) {
  FilterTo(ctx, key_path.c_str());
  ctx->ItemClick(AdoptCheckboxRef(key_path).c_str());
  ctx->Yield(2);
}

void SaveDefaultsPanel(ImGuiTestContext* ctx) {
  ctx->ItemClick("**/###defaults_save");
  ctx->Yield(3);
}

std::vector<gui::DefaultDiffRow> CurrentRows() {
  return gui::BuildDefaultDiffRows(gui::g_state);
}

json ReadOverlayFile(const std::filesystem::path& dir) {
  std::ifstream in(dir / gui::kUserDefaultsFileName);
  if (!in.is_open()) {
    return json::object();
  }
  try {
    return json::parse(in);
  } catch (const std::exception&) {
    return json::object();
  }
}

// The override file as RAW BYTES, or nullopt when it does not exist.
//
// Byte-level rather than parsed: "the file did not change" is the claim, and comparing two parsed
// documents would call a rewrite with reordered keys or different spacing equal. A panel that
// rewrote the file on every click while preserving its meaning would still be the panel the copy
// model exists to remove.
std::optional<std::string> ReadOverlayBytes(const std::filesystem::path& dir) {
  std::ifstream in(dir / gui::kUserDefaultsFileName, std::ios::binary);
  if (!in.is_open()) {
    return std::nullopt;
  }
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// presets.axis.<name>.zenith_std, reporting an absent or malformed key as nullopt rather than
// throwing. A regression that DROPS the key is precisely what these cases exist to catch, and
// nlohmann's operator[] chain would answer it with an uncaught type_error that aborts this
// single-process binary and hides every case after it.
std::optional<float> ReadPresetStd(const json& doc, const char* name) {
  const auto presets = doc.find("presets");
  if (presets == doc.end() || !presets->is_object()) {
    return std::nullopt;
  }
  const auto axis = presets->find("axis");
  if (axis == presets->end() || !axis->is_object()) {
    return std::nullopt;
  }
  const auto node = axis->find(name);
  if (node == axis->end() || !node->is_object()) {
    return std::nullopt;
  }
  const auto value = node->find("zenith_std");
  if (value == node->end() || !value->is_number()) {
    return std::nullopt;
  }
  return value->get<float>();
}

// ---------------------------------------------------------------------------------------------
// Geometry helpers. imgui_internal.h is an anti-pattern in this suite generally and unavoidable
// here for one reason: "did this stay where it was drawn" is not a question the public API answers.
// ---------------------------------------------------------------------------------------------

// The settings table object. Its id is the one BeginTable computed: the table is created directly
// in the panel window with nothing pushed on the ID stack, so the window's own GetID reproduces it.
// Should the table ever be wrapped in a child again — the shared scrolling child this layout
// removed — the seed changes and this returns null. That null IS a regression signal, not a broken
// helper.
ImGuiTable* SettingsTable(ImGuiTestContext* ctx) {
  ImGuiWindow* win = ctx->GetWindowByRef(gui::kDefaultsPanelTitle);
  if (win == nullptr) {
    return nullptr;
  }
  return ImGui::TableFindByID(win->GetID("##defaults_settings_table"));
}

// Where the header cell of `column` was drawn this frame.
//
// Addressed by computed id rather than by a "**/Setting" path, and that is forced rather than
// stylistic: imgui_tables.cpp adds header cells through ItemAdd (so the engine does record their
// geometry, under the id) but never calls the test engine's label hook, and a "**/" path is
// resolved BY that label. TableGetHeaderID closes the gap by reproducing the PushID(column_n) +
// GetID(name) that TableHeadersRow used.
ImGuiTestItemInfo SettingsHeaderInfo(ImGuiTestContext* ctx, ImGuiTable* table, const char* column) {
  return ctx->ItemInfo(TableGetHeaderID(table, column));
}

// Put the settings list back at its top. ImGui keeps a scroll position per window ID and the
// table's inner window is not torn down when the panel closes, so a case that scrolls to the
// bottom hands the NEXT case a list already at the bottom. Rewinding here rather than at the end of
// whoever scrolled makes each case depend on nothing but itself.
void RewindSettingsList(ImGuiTestContext* ctx, ImGuiTable* table) {
  ctx->ScrollToTop(table->InnerWindow->ID);
  ctx->Yield(2);
}

// Scroll the list by the user's own gesture — point at a row and turn the wheel — rather than by
// writing a scroll value into a window this test would first have to name. WHICH region receives
// the wheel is half of what is under test: pointing at the list must scroll the list and nothing
// else, which is exactly what a SetScrollY() would assume rather than check.
//
// The mouse is placed once and left there: after the first turn the row it was addressed by has
// scrolled away, and re-addressing it every turn would fail on an item that no longer exists.
// Eight turns of ten lines reaches the far end of a 40-row list several times over.
void ScrollSettingsListDown(ImGuiTestContext* ctx, const std::string& hover_ref) {
  ctx->MouseMove(hover_ref.c_str());
  for (int i = 0; i < 8; ++i) {
    ctx->MouseWheelY(-10.0f);
    ctx->Yield(2);
  }
  ctx->Yield(2);
}

// Whether a row is actually DRAWN right now — not merely submitted.
//
// Deliberately not ItemExists, and the difference is what makes the scroll cases non-vacuous: a
// Checkbox registers its label with the engine even on the branch where ItemAdd CLIPPED it, so
// ItemExists stays true for a row that has scrolled out of view and would report "nothing
// scrolled" for a list that scrolled all the way to its end. The clipped rectangle is the part
// that collapses to nothing when the row leaves the visible band.
bool RowIsOnScreen(ImGuiTestContext* ctx, const std::string& row_ref) {
  return ctx->ItemInfo(row_ref.c_str(), ImGuiTestOpFlags_NoError).RectClipped.GetHeight() > 0.0f;
}

// The id of a value-cell widget, computed rather than searched for by label.
//
// Forced, not stylistic: ImGui::BeginCombo is the one control shape here that never calls the test
// engine's label hook, so the wildcard LABEL search cannot see a combo cell at all. Every item is
// recorded by ItemAdd under its id regardless, so going through the id covers all five control
// shapes uniformly — which is what a coverage claim over the whole row set needs.
//
// The seed is the TABLE's id, not its inner window's: imgui_tables.cpp pushes an override id for
// the whole body, so every cell widget hashes against that rather than against whatever window it
// happens to be drawn in.
ImGuiID SettingsCellID(ImGuiTestContext* ctx, const std::string& label) {
  ImGuiTable* table = SettingsTable(ctx);
  if (table == nullptr) {
    return 0;
  }
  return ImGui::GetIDWithSeed(label.c_str(), nullptr, table->ID);
}

bool AnyValueWidgetExists(ImGuiTestContext* ctx, const std::string& key_path) {
  for (const char* suffix : { "", "_input", "_slider" }) {
    const ImGuiID id = SettingsCellID(ctx, "##value_" + key_path + suffix);
    if (id != 0 && ctx->ItemExists(id)) {
      return true;
    }
  }
  return false;
}

// The id of one cell of one axis row inside one preset's table, reproduced from the ID stack the
// panel builds rather than searched for by label.
//
// Forced, and by a different mechanism from SettingsCellID's: the three axis rows draw their cells
// under the SAME labels ("##type" / "##mean" / "##std"), told apart only by the PushID the panel
// wraps each row in. A wildcard label search therefore has no unique match and comes back with
// nothing at all — measured, and it looks exactly like "the cell is drawn as text", which is this
// file's only claim about those cells.
//
// The chain from `host` is the preset's TreeNode (which pushes its own id until TreePop), the table
// (which pushes an override id for its whole body), and the row's PushID.
//
// `host` is passed in rather than derived from the panel window, and that is the one link that
// cannot be reproduced by hashing: BeginChild composes a window NAME out of its parent's name, its
// own string id and the hash of that id, and the child window's ID is the hash of the composed
// name — not the hash the caller can see. Taking it from a known item's ItemInfo().Window sidesteps
// the whole question, and the caller checks the derivation against that item's own id before using
// it on the cells it cannot address any other way.
ImGuiID PresetAxisCellID(ImGuiWindow* host, const char* preset_label, const char* axis, const char* cell) {
  if (host == nullptr) {
    return 0;
  }
  const std::string node = std::string("###preset_") + preset_label;
  const std::string table = std::string("##preset_table_") + preset_label;
  ImGuiID id = ImGui::GetIDWithSeed(node.c_str(), nullptr, host->ID);
  id = ImGui::GetIDWithSeed(table.c_str(), nullptr, id);
  id = ImGui::GetIDWithSeed(axis, nullptr, id);
  return ImGui::GetIDWithSeed(cell, nullptr, id);
}

// The text an input box is SHOWING, read by activating it.
//
// The only route to it: an InputFloat formats its value into a scratch buffer every frame and
// hands it to InputText, which keeps it only while the item is active. So the box is clicked (which
// is also what a user does before reading a number closely), the buffer is read, and Escape reverts
// the activation without committing an edit. Returns an empty string when nothing is active, which
// the caller must treat as a failure rather than as an empty box.
std::string ActivatedInputText(ImGuiTestContext* ctx, const char* ref) {
  ctx->ItemClick(ref);
  ctx->Yield(2);
  std::string out;
  ImGuiContext& g = *ImGui::GetCurrentContext();
  if (ImGuiInputTextState* state = ImGui::GetInputTextState(g.ActiveId)) {
    out.assign(state->TextA.Data, static_cast<size_t>(state->TextLen));
  }
  ctx->KeyPress(ImGuiKey_Escape);
  ctx->Yield(2);
  return out;
}

}  // namespace

void RegisterDefaultsPanelTests(ImGuiTestEngine* engine) {
  // ===============================================================================================
  // Getting in and out.
  // ===============================================================================================

  {
    // Discoverability, not reachability. The panel spent its first iteration behind two items in
    // the Save dropdown, where it was reachable by every automated measure and found by nobody: a
    // user looking for their settings does not open a Save menu. So the claim is specifically
    // "visible from the default state with NOTHING opened" — no menu clicked, no window toggled,
    // no scrolling — which is why this case yields from ResetTestState straight into ItemExists
    // and does not touch a control first.
    //
    // A test that opened the menu and found the entry there would pass on the version that failed
    // the owner. Keep the "no interaction before the assertion" shape if this is ever edited.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "the_settings_entry_is_in_the_top_bar_and_stays_during_a_run");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_topbar_entry");
      ctx->Yield(2);

      const char* kSettingsRef = "##TopBar/" ICON_FA_GEAR " Settings";
      IM_CHECK(ctx->ItemExists(kSettingsRef));

      // A simulation running must not take the entry away: settings are not a document operation,
      // and an entry that vanishes while the user watches a render is one they cannot find at the
      // moment they think to look for it.
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists(kSettingsRef));
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kSettingsRef)));
      gui::g_state.run_intent = gui::RunIntent::kNone;
      ctx->Yield(2);

      // Not a decoration: it opens the panel, on the settings section, which is the whole of the
      // button's contract.
      IM_CHECK(!gui::g_state.defaults_panel_open);
      ctx->ItemClick(kSettingsRef);
      ctx->Yield(4);
      IM_CHECK(gui::g_state.defaults_panel_open);
      IM_CHECK(ctx->ItemExists("**/###defaults_save"));
      IM_CHECK(ctx->ItemExists("**/###defaults_settings"));
    };
  }

  {
    // The old home of the entry, asserted empty. Deleting the two Save-menu items is the half of
    // "promote the entry" that a passing top-bar assertion says nothing about, and leaving them
    // would quietly restore the state the promotion was meant to end: two ways in, one of which
    // teaches the user to look in the wrong place.
    //
    // The menu's own gate table lives in test_file_ops.cpp and enumerates the six items that SHOULD
    // be there; it would pass unchanged with two more beside them, which is why this negative is
    // stated separately and here, next to the entry point it is about.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_save_menu_does_not_host_a_second_way_in");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      ctx->SetRef("//$FOCUSED");
      IM_CHECK(!ctx->ItemExists("**/Save Current as Defaults..."));
      IM_CHECK(!ctx->ItemExists("**/Edit My Presets..."));
      // The menu itself still works — this is a removal, not a broken popup.
      IM_CHECK(ctx->ItemExists("**/Config JSON..."));
      ctx->SetRef("");

      ctx->PopupCloseAll();
      ctx->Yield(2);
    };
  }

  {
    // The entry point chooses which section opens expanded, asserted through an observable
    // consequence — whether that section's rows are rendered — rather than left as a manual check
    // that only breaks once a second entry point tries to use it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_settings_entry_point_opens_the_settings_section");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_entry_settings");

      gui::g_state.bg_alpha = 0.42f;
      const auto rows = CurrentRows();
      const auto probe = std::find_if(rows.begin(), rows.end(),
                                      [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
      IM_CHECK(probe != rows.end());

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef(probe->key_path).c_str()));
      // ...and the OTHER section is collapsed, so "which section opens" is a real choice rather
      // than "everything is always open".
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
    };
  }

  {
    // The same contract for the preset section. Two cases rather than one because the two entry
    // points are two independent registrations of the same mechanism, and a single case would not
    // say which one stopped travelling.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_presets_entry_point_opens_the_preset_section");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_entry_presets");

      gui::g_state.bg_alpha = 0.42f;
      const auto rows = CurrentRows();
      const auto probe = std::find_if(rows.begin(), rows.end(),
                                      [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
      IM_CHECK(probe != rows.end());

      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      // A preset row is only findable when its section is expanded (the engine cannot see clipped
      // items), and the settings row is correspondingly out of reach.
      IM_CHECK(ctx->ItemExists("**/###preset_Column"));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef(probe->key_path).c_str()));
    };
  }

  {
    // Both sections still fold and unfold by hand. Each section's body is a child or a table that
    // only exists inside the `if (header)` branch, so a header that stopped reporting its state
    // would take the whole section with it — and the layout work that gave each section its own
    // scrolling region is exactly the kind of change that can do that.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "both_sections_still_fold_and_unfold_by_hand");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_folding");
      gui::g_state.renderer.fov = 95.0f;  // a row that is unambiguously present

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      const std::string probe_row = AdoptCheckboxRef("renderer.fov");
      IM_CHECK(ctx->ItemExists(probe_row.c_str()));

      ctx->ItemClick("**/###defaults_settings");  // fold
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists(probe_row.c_str()));
      ctx->ItemClick("**/###defaults_settings");  // unfold
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(probe_row.c_str()));

      ctx->ItemClick("**/###defaults_presets");  // unfold (it opened collapsed on kSettings)
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/###preset_Column"));
      ctx->ItemClick("**/###defaults_presets");  // fold
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
      FilterTo(ctx, "");
    };
  }

  {
    // The entry point owns the OPENING frame and then lets go. A section the user collapses
    // afterwards must stay collapsed — that "one frame only" half is what a SetNextItemOpen left
    // running every frame would break, and it is invisible to a test that merely opens the panel
    // and looks.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_entry_point_owns_only_the_opening_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_entry_one_frame");

      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      IM_CHECK(ctx->ItemExists("**/###preset_Column"));
      ctx->ItemClick("**/###defaults_presets");
      ctx->Yield(4);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
      // Several more frames: a forced-open state re-applied every frame would have snapped it back
      // by now, and one Yield would not have caught it.
      ctx->Yield(8);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
    };
  }

  {
    // A mixed batch of edits, closed WITHOUT Save, leaves the file BYTE-identical.
    //
    // Every kind of edit the panel offers is exercised in one session, because the claim is about
    // the panel and not about one button: if any single path still wrote through, this fails.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "closing_without_saving_writes_nothing_and_discards_the_copy");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_close_discards");

      // A file with something in every half, so each edit below has something real to change.
      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["renderer"]["fov"] = 95.0f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      doc["presets"]["axis"]["plate"]["zenith_std"] = 0.6f;  // the one the restore below drops
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();

      const auto before = ReadOverlayBytes(panel.dir());
      IM_CHECK(before.has_value());

      // An unsaved document change too, so the list has a row that opens checked because its value
      // differs on top of the ones that open checked because they are already saved.
      gui::g_state.sun.altitude = 33.0f;

      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.5f);  // a preset edit
      ctx->Yield(2);
      ctx->ItemClose("**/###preset_Column");
      ctx->ItemOpen("**/###preset_Plate");
      ctx->Yield(2);
      ctx->ItemClick("**/###preset_restore_plate");  // a preset restore
      ctx->Yield(2);
      ctx->ItemClose("**/###preset_Plate");
      ctx->Yield(2);

      ctx->ItemOpen("**/###defaults_settings");
      ctx->Yield(2);
      ToggleRow(ctx, "bg_alpha");  // un-check a key that IS in the defaults
      FilterTo(ctx, "");
      ctx->ItemClick("**/###defaults_reset_all");  // and the whole settings half at once
      ctx->Yield(2);

      panel.Close();

      // Byte for byte, not json-equal: a rewrite that preserved meaning would still be a write.
      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before);
      // ...and nothing leaked into the in-memory halves either — both directions of the preset edit
      // (a changed value, a dropped override) and the settings half.
      const auto column = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(column.has_value());
      IM_CHECK_EQ(*column, 0.3f);
      const auto plate = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kPlate);
      IM_CHECK(plate.has_value());
      IM_CHECK_EQ(*plate, 0.6f);
      IM_CHECK_EQ(gui::MakeNewDocumentState().bg_alpha, 0.42f);
    };
  }

  {
    // The title-bar X: the exit that has to prove BOTH halves. "Writes nothing" alone would be
    // satisfied by a panel that never closes (Esc satisfies it that way, and the case below says
    // so), so the file assertion is paired here with two that say the panel is actually gone — the
    // flag cleared AND the widgets off screen. They are not the same claim: ImGui closes the popup
    // itself on the frame the X is pressed, so a version that forgot to clear defaults_panel_open
    // would look closed for exactly one frame and be re-opened by the OpenPopup guard on the next.
    // Yielding several frames before asking is what makes that failure visible rather than a coin
    // flip.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_title_bar_x_discards_and_actually_closes");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_title_x");

      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      const auto before = ReadOverlayBytes(panel.dir());
      IM_CHECK(before.has_value());

      // Real pending edits in both halves, so "nothing was written" is a statement about a session
      // that had something to write rather than about an untouched panel.
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.5f);
      ctx->Yield(2);
      ctx->ItemClose("**/###preset_Column");
      ctx->ItemOpen("**/###defaults_settings");
      ctx->Yield(2);
      ToggleRow(ctx, "bg_alpha");
      FilterTo(ctx, "");
      ctx->Yield(2);

      // Addressed through the title constant rather than a "**/" wildcard: "#CLOSE" is ImGui's id
      // for EVERY window's title-bar X, so a wildcard could match some other window's and pass
      // while this panel's X does nothing.
      const std::string close_x_ref = std::string(gui::kDefaultsPanelTitle) + "/#CLOSE";
      ctx->ItemClick(close_x_ref.c_str());
      ctx->Yield(4);

      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before);
      IM_CHECK(!gui::g_state.defaults_panel_open);
      IM_CHECK(!ctx->ItemExists("**/###defaults_save"));

      // Re-opening starts from the file, not from the discarded copy — the edits above are GONE
      // rather than merely unwritten. Without this, "discard" would be indistinguishable from
      // "deferred": a copy that survived the X would be committed by the next session's Save, and
      // the file assertion above would still have passed at the moment it was made.
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      SaveDefaultsPanel(ctx);
      const auto after = ReadOverlayFile(panel.dir());
      IM_CHECK_EQ(ReadPresetStd(after, "column"), std::optional<float>(0.3f));
      IM_CHECK(after.contains("bg_alpha"));
    };
  }

  {
    // Esc is the exit no button owns, and it does NOT close this panel — the close happens during
    // navigation, earlier in the frame than the OpenPopup guard, so the p_open channel never
    // observes it and the panel is back on screen the next frame.
    //
    // Both halves are asserted, and the second is what keeps the first honest: a file is trivially
    // unwritten if Esc did nothing at all. Measured behavior on record rather than a wish — if this
    // ever becomes a real close, this case is where that decision has to be made explicitly.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "escape_writes_nothing_and_does_not_close_the_panel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_escape");

      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      const auto before = ReadOverlayBytes(panel.dir());
      IM_CHECK(before.has_value());

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(2);
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(4);

      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before);
      IM_CHECK(gui::g_state.defaults_panel_open);
      IM_CHECK(ctx->ItemExists("**/###defaults_save"));
    };
  }

  {
    // Opening the panel normalizes a top level that is not an object.
    //
    // The read path this rides on already does that itself, so no path this repo ships can reach a
    // non-object copy today. The point is that the panel's own guarantee should not depend on an
    // implementation detail two calls away that nothing here names: every mutator of the working
    // copy assumes an object root, and this pins the guarantee at the copy's single origin.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "opening_the_panel_normalizes_a_non_object_override_file");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_non_object");
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), json::array()));

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      SaveDefaultsPanel(ctx);
      panel.Close();

      IM_CHECK(ReadOverlayFile(panel.dir()).is_object());
    };
  }

  // ===============================================================================================
  // The settings list: one row per candidate default, one checkbox each.
  // ===============================================================================================

  {
    // Completeness: every row the diff engine produces is rendered, once, with BOTH of its
    // controls. The two answer different questions — what the file holds NOW versus what Save
    // would leave it holding — which is why both are addressable and both are required here.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "every_candidate_default_gets_one_row_with_both_controls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_completeness");

      // Edited AND saved keys both present, so the sweep covers rows in every state rather than a
      // list that happens to be uniform.
      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      gui::g_state.renderer.fov = 95.0f;

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());
      // No duplicates in the row set itself: "rendered once" would be unfalsifiable through the
      // widget tree alone, since two rows with the same key path would also share a widget id.
      std::set<std::string> unique_keys;
      for (const auto& row : rows) {
        unique_keys.insert(row.key_path);
      }
      IM_CHECK_EQ(unique_keys.size(), rows.size());

      int changed_seen = 0;
      int unchanged_seen = 0;
      for (const auto& row : rows) {
        // One key at a time: nothing is off screen, so "not found" means "not rendered" rather
        // than "scrolled past". Reported non-fatally so a missing row names itself rather than
        // printing "false is not true"; the sweep still ends at the first gap, because the context
        // stops responding once anything is on record (see ctx->IsError() in test_gui_shared.hpp).
        FilterTo(ctx, row.key_path.c_str());
        if (!ctx->ItemExists(AdoptCheckboxRef(row.key_path).c_str())) {
          IM_ERRORF("row '%s' has no adopt checkbox", row.key_path.c_str());
        }
        if (ctx->IsError()) {
          break;
        }
        if (!ctx->ItemExists(SourceCellRef(row.key_path).c_str())) {
          IM_ERRORF("row '%s' has no source cell", row.key_path.c_str());
        }
        if (gui::RowNeedsAdoption(row)) {
          ++changed_seen;
        } else {
          ++unchanged_seen;
        }

        if (ctx->IsError()) {
          break;
        }
      }
      // Non-vacuous in both directions: the sweep really did cover rows that differ from the
      // effective default and rows that do not.
      IM_CHECK(changed_seen > 0);
      IM_CHECK(unchanged_seen > 0);

      FilterTo(ctx, "");
    };
  }

  {
    // The search box narrows the list, AND the section header says how many rows are left.
    //
    // The count is the half a "does this row exist" assertion cannot see, and it is the half the
    // user reads: a narrowed list with no count looks like a shorter settings set rather than a
    // filtered view of the same one. It is only readable at all because the header's id is the
    // "###" form, so the drawn text is not part of the id — see DrawnLabel.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "the_search_box_narrows_the_list_and_says_how_many_are_left");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_search");
      gui::g_state.renderer.fov = 95.0f;
      gui::g_state.bg_alpha = 0.42f;

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      const int total = static_cast<int>(CurrentRows().size());
      IM_CHECK(total > 1);

      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));
      {
        char expected[64];
        ImFormatString(expected, IM_ARRAYSIZE(expected), "Settings (%d of %d)", total, total);
        IM_CHECK(StartsWith(DrawnLabel(ctx, "**/###defaults_settings"), expected));
      }

      FilterTo(ctx, "renderer.fov");
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));
      {
        // The exact key, so the visible count is 1 and the header has to say a different number
        // from the total — a header wired to g_rows.size() twice would pass the check above and
        // fail here, which is the point of asking twice.
        char expected[64];
        ImFormatString(expected, IM_ARRAYSIZE(expected), "Settings (1 of %d)", total);
        IM_CHECK(StartsWith(DrawnLabel(ctx, "**/###defaults_settings"), expected));
      }

      // Clearing restores the full set — a filter that could not be undone would strand the user
      // in a partial view of their own settings.
      FilterTo(ctx, "");
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));
    };
  }

  {
    // The two filters are two DIFFERENT questions, and the trap this exists to catch is their
    // collapsing into one "only show changes" switch. So the two samples are chosen to be
    // orthogonal — each hits exactly one filter:
    //   A (renderer.fov)  value differs from factory, checkbox NOT touched this session
    //   B (bg_show)       value IS the factory one, checkbox touched this session
    // A single toggle cannot separate these, whichever of the two it means.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_two_filters_ask_two_different_questions");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_filters");
      gui::g_state.renderer.fov = 95.0f;                           // sample A
      IM_CHECK_EQ(gui::g_state.bg_show, gui::GuiState{}.bg_show);  // sample B starts at factory

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      // A opens checked (its value differs) and B opens unchecked, so toggling B is what makes it
      // "edited this session" — and A stays un-edited despite being checked. That asymmetry is the
      // point: "checked" and "edited" are not the same thing either.
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      ToggleRow(ctx, "bg_show");
      IM_CHECK(RowIsChecked(ctx, "bg_show"));
      FilterTo(ctx, "");

      // Filter 1 — the value layer. A hits, B misses.
      ctx->ItemClick("**/###filter_differs");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_show").c_str()));

      // ...and the search box ANDs with it rather than replacing it: a key that passes the search
      // but fails the filter stays hidden.
      FilterTo(ctx, "bg_show");
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_show").c_str()));
      FilterTo(ctx, "");

      // Filter 2 — the operation layer. B hits, A misses. Both directions, so neither filter can be
      // an alias of the other.
      ctx->ItemClick("**/###filter_edited");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("bg_show").c_str()));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));

      // A row hidden by a filter is still a row: Save applies the whole list, not the visible part
      // of it. A filter that silently narrowed what gets written would be a data-loss bug wearing a
      // view control's clothes.
      SaveDefaultsPanel(ctx);
      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(saved.contains("renderer"));  // A was hidden, and still written
      IM_CHECK_EQ(saved["renderer"]["fov"].get<float>(), 95.0f);
      IM_CHECK(saved.contains("bg_show"));  // B was visible, and written

      // Back to All, so the next scenario starts from a full list.
      ctx->ItemClick("**/###filter_all");
      ctx->Yield(2);
    };
  }

  {
    // What the checkbox says when the panel opens. It means "this key is in my defaults", and its
    // opening value is the OR of the two conditions that can put it there. All three reachable
    // states are constructed in ONE document so the answers cannot be right for one reason and
    // wrong for another:
    //   (1) changed in the GUI, not saved   -> checked (saving now would add it)
    //   (2) already saved, value unchanged  -> checked (saving now would keep it)
    //   (3) neither                         -> unchecked
    // (1) and (2) are the two halves of the OR; (3) is what makes the OR falsifiable.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_checkbox_opens_on_whether_the_key_is_in_my_defaults");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_checkbox_open");

      // State (2): saved, and the document starts from that saved value, so it does NOT also
      // qualify through "differs from the effective default" — otherwise the case would pass with
      // the panel looking at only one of the two conditions.
      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.42f);

      // State (1): changed here and now, nothing on disk about it.
      gui::g_state.renderer.fov = 95.0f;

      // The three rows, taken from the engine so the premises are checked rather than assumed.
      const auto rows = CurrentRows();
      const gui::DefaultDiffRow* changed = nullptr;
      const gui::DefaultDiffRow* saved = nullptr;
      const gui::DefaultDiffRow* neither = nullptr;
      for (const auto& row : rows) {
        if (row.key_path == "renderer.fov") {
          changed = &row;
        } else if (row.key_path == "bg_alpha") {
          saved = &row;
        } else if (row.key_path == "bg_show") {
          neither = &row;
        }
      }
      IM_CHECK(changed != nullptr && saved != nullptr && neither != nullptr);
      IM_CHECK(gui::RowNeedsAdoption(*changed) && !changed->has_saved_override);
      IM_CHECK(!gui::RowNeedsAdoption(*saved) && saved->has_saved_override);
      IM_CHECK(!gui::RowNeedsAdoption(*neither) && !neither->has_saved_override);

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));
      FilterTo(ctx, "bg_show");
      IM_CHECK(!RowIsChecked(ctx, "bg_show"));
      FilterTo(ctx, "");
    };
  }

  {
    // The Source cell reports THE FILE, not the difference — and the two are told apart by the one
    // configuration where they disagree: a value deliberately saved that happens to equal the
    // factory one. It is still the user's, and a cell wired to "differs from factory" would call it
    // Factory and leave them unable to see what they saved.
    //
    // Asserted on the DRAWN text rather than on the cell's existence: the id is the "###" form, so
    // an item is found whichever word it rendered. The store-side half of this rule is pinned in
    // composition-correctness/gui/test_user_defaults_chain.cpp; what is left for this layer is
    // whether the cell on screen reports it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_source_cell_reports_the_file_not_the_difference");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_source_cell");

      // Saved, and saved to EXACTLY the factory value.
      const gui::GuiState factory;
      json doc;
      doc["bg_alpha"] = factory.bg_alpha;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.bg_alpha, factory.bg_alpha);

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      FilterTo(ctx, "bg_alpha");
      // The premise, asked of the row set: this key IS in the file and does NOT differ from
      // factory. Without it the assertion below could pass for the ordinary reason.
      const auto rows = CurrentRows();
      const auto row =
          std::find_if(rows.begin(), rows.end(), [](const gui::DefaultDiffRow& r) { return r.key_path == "bg_alpha"; });
      IM_CHECK(row != rows.end());
      IM_CHECK(row->has_saved_override);
      IM_CHECK(!gui::RowNeedsAdoption(*row));
      IM_CHECK(StartsWith(DrawnLabel(ctx, SourceCellRef("bg_alpha")), "Mine"));

      // The other half: a key the file says nothing about reads Factory even after the user has
      // changed its value, which is the mirror-image confusion.
      gui::g_state.renderer.fov = 95.0f;
      panel.Close();
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));                                      // it WOULD be saved...
      IM_CHECK(StartsWith(DrawnLabel(ctx, SourceCellRef("renderer.fov")), "Factory"));  // ...but is not yet
      FilterTo(ctx, "");
    };
  }

  {
    // Un-checking a saved row expresses "remove it from my defaults", and Save is the one place it
    // lands. Split in two on purpose: the click must NOT reach the file and must still be visible
    // on screen, and only then does Save move the bytes. The split is what survives the control
    // changing — an earlier design asserted the file the instant a per-row button was clicked.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "unchecking_a_row_reaches_the_file_only_through_save");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_uncheck");

      // Two personal defaults plus a preset subtree this half of the panel does not own. The
      // subtree is the point: a wholesale rewrite of the document would delete it silently.
      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["renderer"]["fov"] = 95.0f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));

      // Start from a document that already carries those defaults, so both keys open CHECKED
      // because they are already in the defaults (not because their value differs).
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.42f);

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));

      const auto before = ReadOverlayBytes(panel.dir());
      IM_CHECK(before.has_value());
      ctx->ItemClick(AdoptCheckboxRef("bg_alpha").c_str());
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before);
      IM_CHECK(!RowIsChecked(ctx, "bg_alpha"));
      // The Source cell still reads Mine: it reports the file, which has not changed yet. The
      // checkbox reports the intent. Two questions, two answers, both on screen at once.
      IM_CHECK(StartsWith(DrawnLabel(ctx, SourceCellRef("bg_alpha")), "Mine"));

      SaveDefaultsPanel(ctx);

      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(saved.contains("renderer"));  // the row that stayed checked stayed written
      IM_CHECK(saved.contains("presets"));   // ...and the sibling namespace is untouched
      // The removed key is back to factory for a new document — the file change is the whole point,
      // not a panel-local undo.
      IM_CHECK_EQ(gui::MakeNewDocumentState().bg_alpha, gui::GuiState{}.bg_alpha);
      FilterTo(ctx, "");
    };
  }

  {
    // Save writes the checked rows and REMOVES the unchecked ones, in one pass, and what it wrote
    // is what a new document actually reads. Two edited keys, exactly one of them un-checked, so
    // the case distinguishes "wrote everything" from "wrote what was asked".
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "save_writes_the_checked_rows_and_leaves_out_the_unchecked_ones");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_adoption");
      gui::g_state.bg_alpha = 0.42f;
      gui::g_state.renderer.fov = 95.0f;

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      ToggleRow(ctx, "bg_alpha");
      FilterTo(ctx, "");
      SaveDefaultsPanel(ctx);

      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(saved.contains("renderer"));
      IM_CHECK_EQ(saved["renderer"]["fov"].get<float>(), 95.0f);
      IM_CHECK(!saved.contains("bg_alpha"));

      // ...and the file is what a new document actually reads.
      const gui::GuiState fresh = gui::MakeNewDocumentState();
      IM_CHECK_EQ(fresh.renderer.fov, 95.0f);
      IM_CHECK_EQ(fresh.bg_alpha, gui::GuiState{}.bg_alpha);

      // After the save the adopted row no longer differs from the effective default, and the panel
      // now attributes it to the user — which IS the on-screen confirmation that the write landed,
      // since the panel deliberately stays open.
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(StartsWith(DrawnLabel(ctx, SourceCellRef("renderer.fov")), "Mine"));
      const auto rows = CurrentRows();
      const auto fov = std::find_if(rows.begin(), rows.end(),
                                    [](const gui::DefaultDiffRow& row) { return row.key_path == "renderer.fov"; });
      IM_CHECK(fov != rows.end());
      IM_CHECK(!gui::RowNeedsAdoption(*fov));
      IM_CHECK(fov->has_saved_override);
      FilterTo(ctx, "");
    };
  }

  {
    // Reset all clears EVERY row's checkbox and writes nothing until Save. The whole sweep, not a
    // spot check: "all" is the claim, and a reset that cleared only the rows it could see — or only
    // the ones checked for one of the two reasons — would satisfy any single-row assertion.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "reset_all_unchecks_every_row_and_writes_nothing_until_save");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_reset_all");

      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      // Checked for the OTHER reason too, so the sweep covers both halves of the OR.
      gui::g_state.sun.altitude = 33.0f;

      const auto before = ReadOverlayBytes(panel.dir());
      IM_CHECK(before.has_value());

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      // Non-vacuous: something really was checked before the click, for each of the two reasons.
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));
      FilterTo(ctx, "sun.altitude");
      IM_CHECK(RowIsChecked(ctx, "sun.altitude"));

      FilterTo(ctx, "");
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());
      for (const auto& row : rows) {
        FilterTo(ctx, row.key_path.c_str());
        if (RowIsChecked(ctx, row.key_path)) {
          IM_ERRORF("row '%s' survived Reset all still checked", row.key_path.c_str());
        }
        if (ctx->IsError()) {
          break;
        }
      }

      FilterTo(ctx, "");
      SaveDefaultsPanel(ctx);
      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(!saved.contains("sun"));
      // The preset library is a sibling namespace this button does not reach.
      IM_CHECK_EQ(ReadPresetStd(saved, "column").value_or(-1.0f), 0.3f);
    };
  }

  {
    // Reset all in BOTH directions — discarded on close, committed on Save — in one case so the
    // pair cannot drift apart. The second half re-opens the panel, because the first Close is
    // precisely what threw the first copy away.
    //
    // The committed half is the one with a trap in it: Save after Reset all must write an EMPTY
    // override set, not re-adopt the rows that were checked when the panel opened. A panel that
    // re-adopted them would make the button a no-op the user cannot see through.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "reset_all_is_discarded_on_close_and_committed_on_save");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_reset_both_ways");

      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["renderer"]["fov"] = 95.0f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();

      const auto before = ReadOverlayBytes(panel.dir());
      IM_CHECK(before.has_value());

      // (a) discarded.
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);
      // The panel shows the reset immediately — this is the feedback a write-through model owed the
      // user and could not give: the row is un-checked though the file still holds the value, and
      // its Source still says so.
      IM_CHECK(!RowIsChecked(ctx, "renderer.fov"));
      IM_CHECK(StartsWith(DrawnLabel(ctx, SourceCellRef("renderer.fov")), "Mine"));
      FilterTo(ctx, "");
      panel.Close();
      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before);

      // (b) committed.
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(2);
      SaveDefaultsPanel(ctx);
      panel.Close();

      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(!saved.contains("renderer"));
      IM_CHECK_EQ(ReadPresetStd(saved, "column").value_or(-1.0f), 0.3f);
      IM_CHECK_EQ(gui::MakeNewDocumentState().bg_alpha, gui::GuiState{}.bg_alpha);
    };
  }

  // ===============================================================================================
  // The Current value column: the field's REAL control, in the table.
  //
  // The claim throughout is NOT "a control appears in the cell" — it is that the control is the
  // SAME one, with the SAME constraint, as the main UI's. A second way to reach states the main UI
  // refuses is the failure mode; a cell that silently ignores input is the other.
  // ===============================================================================================

  {
    // An out-of-range entry lands in the same place whether it is typed into the table or into the
    // main UI's own control.
    //
    // Three fields, one per constraint SHAPE the registry has to express, rather than three
    // arbitrary ones:
    //   renderer.fov        — a bound that is a function of the state (the lens type's max FOV)
    //   overlay_grid_alpha  — a constant bound
    //   sim.max_hits        — an integer bound
    // The fourth shape (a float on a non-linear slider scale) has no representative: not one of the
    // candidate defaults is edited on such a scale in the main UI — those are all axis-distribution
    // fields, which are per-crystal and never defaults. Its stand-in is the state-dependent bound,
    // which is the harder of the two.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_value_cell_clamps_exactly_like_the_main_ui_control");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_cell_clamp");

      const float max_fov = LUMICE_MaxFov(static_cast<LUMICE_LensType>(gui::g_state.renderer.lens_type));
      IM_CHECK_GT(max_fov, 90.0f);  // the probe below has to be outside the domain to test anything

      // The domains themselves, before driving anything through them. Not redundant with the
      // drive-it-twice comparison, and the reason is a real limit on what that comparison can see:
      // the main UI calls the SAME control every frame and it clamps unconditionally, so a table
      // cell that allowed a wider range would have its value pulled back within a frame and the two
      // would still agree. Asserting the domain directly is what catches a bound that DRIFTED, as
      // opposed to a cell that ignored its input.
      //
      // fov is checked against the expression the main UI uses, at two lens types, because "the
      // bound follows the state" is the property — a registry that returned a constant would pass
      // one of these and fail the other.
      const gui::FieldEditorEntry* fov_entry = gui::FindFieldEditor("renderer.fov");
      IM_CHECK(fov_entry != nullptr);
      // Reported per lens rather than asserted fatally: "the bound follows the state" is a claim
      // about the pair, so a run has to say WHICH lens disagreed — stopping at the first would hide
      // the very comparison the two entries exist to make.
      for (const int lens : { gui::kLensTypeLinear, gui::kLensTypeGlobe }) {
        gui::GuiState probe_state;
        probe_state.renderer.lens_type = lens;
        const auto constraint = fov_entry->Constraint(probe_state);
        if (!constraint.has_numeric_domain) {
          IM_ERRORF("lens %d: renderer.fov reports no numeric domain", lens);
          continue;
        }
        if (constraint.min_value != 1.0) {
          IM_ERRORF("lens %d: renderer.fov lower bound is %.3f, expected 1", lens, constraint.min_value);
        }
        const float expected_max = LUMICE_MaxFov(static_cast<LUMICE_LensType>(lens));
        if (static_cast<float>(constraint.max_value) != expected_max) {
          IM_ERRORF("lens %d: renderer.fov upper bound is %.3f, but the main UI uses %.3f", lens, constraint.max_value,
                    expected_max);
        }
      }
      {
        const gui::GuiState probe_state;
        const auto alpha = gui::FindFieldEditor("overlay_grid_alpha")->Constraint(probe_state);
        IM_CHECK(alpha.has_numeric_domain);
        IM_CHECK_EQ(alpha.min_value, 0.0);
        IM_CHECK_EQ(alpha.max_value, 1.0);
        const auto hits = gui::FindFieldEditor("sim.max_hits")->Constraint(probe_state);
        IM_CHECK(hits.has_numeric_domain);
        IM_CHECK_EQ(hits.min_value, 1.0);
        IM_CHECK_EQ(hits.max_value, 64.0);
      }

      // ---- renderer.fov ----
      gui::g_state.renderer.fov = 90.0f;
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      ctx->ItemInputValue(ValueInputRef("renderer.fov").c_str(), 900.0f);
      ctx->Yield(3);
      const float fov_from_table = gui::g_state.renderer.fov;
      FilterTo(ctx, "");
      panel.Close();

      gui::g_state.renderer.fov = 90.0f;
      ctx->Yield(2);
      ctx->ItemInputValue("**/##FOV##view_input", 900.0f);
      ctx->Yield(3);
      IM_CHECK_EQ(fov_from_table, gui::g_state.renderer.fov);
      IM_CHECK_EQ(fov_from_table, max_fov);  // non-vacuous: both clamped, neither ignored the input

      // ---- overlay_grid_alpha (constant domain) ----
      gui::g_state.grid_alpha = 0.3f;
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_grid_alpha");
      ctx->ItemInputValue(ValueInputRef("overlay_grid_alpha").c_str(), 7.5f);
      ctx->Yield(3);
      const float alpha_from_table = gui::g_state.grid_alpha;
      FilterTo(ctx, "");
      panel.Close();

      gui::g_state.grid_alpha = 0.3f;
      ctx->Yield(2);
      // Addressed through its window rather than with a `**/` wildcard, unlike the FOV control
      // above. A wildcard lookup resolves a LABEL, and a clipped item is registered by id without
      // one (imgui_te_context.cpp says so where it tries to pan the window looking for the item);
      // the Overlay group sits past the right panel's fold at the harness window size, so the
      // wildcard finds nothing while the window-relative id resolves and scrolls to it. The FOV
      // control is above the fold and either form reaches it — this one has no choice.
      ctx->SetRef("##RightPanel");
      ctx->ItemInputValue("##Alpha##grid_input", 7.5f);
      ctx->SetRef("");
      ctx->Yield(3);
      IM_CHECK_EQ(alpha_from_table, gui::g_state.grid_alpha);
      IM_CHECK_EQ(alpha_from_table, 1.0f);

      // ---- sim.max_hits (integer domain) ----
      gui::g_state.sim.max_hits = 8;
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "sim.max_hits");
      ctx->ItemInputValue(ValueInputRef("sim.max_hits").c_str(), 4096);
      ctx->Yield(3);
      const int hits_from_table = gui::g_state.sim.max_hits;
      FilterTo(ctx, "");
      panel.Close();

      gui::g_state.sim.max_hits = 8;
      ctx->Yield(2);
      ctx->ItemInputValue("**/##Max hits_input", 4096);
      ctx->Yield(3);
      IM_CHECK_EQ(hits_from_table, gui::g_state.sim.max_hits);
      IM_CHECK_EQ(hits_from_table, 64);

      // ---- renderer.opacity: the same clamp, where nothing else can be doing it ----
      //
      // The three fields above all have a main-UI control that clamps them every frame, so their
      // final value is evidence about the table's cell only if the table wrote it first. opacity
      // has no control anywhere else in the app: whatever this cell leaves behind is what the field
      // holds, indefinitely. It is therefore the one field whose clamp this suite can attribute to
      // the table with no alternative explanation.
      gui::g_state.renderer.opacity = 0.5f;
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.opacity");
      ctx->ItemInputValue(ValueInputRef("renderer.opacity").c_str(), 5.0f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 1.0f);
      FilterTo(ctx, "");
      panel.Close();
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 1.0f);  // and it stays: nothing else touches it
    };
  }

  {
    // The control shapes where "out of range" is not a meaningful input — a checkbox and a combo
    // cannot be given an out-of-domain value; their constraint IS the shape of the control. What
    // has to be true instead is that the cell edits the SAME FIELD the main UI edits.
    //
    // Plus the applicability half: a field whose main-UI control is disabled in some configuration
    // must be disabled in the table in the SAME configuration, or the table becomes a second way to
    // reach states the main UI refuses.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "a_checkbox_and_a_combo_cell_write_what_the_main_ui_writes");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_cell_shapes");
      gui::g_state.show_grid_line = false;
      gui::g_state.renderer.visible = gui::kVisibleFull;
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      // A checkbox cell writes the field itself.
      FilterTo(ctx, "overlay_grid_line");
      ctx->ItemClick(ValueWidgetRef("overlay_grid_line").c_str());
      ctx->Yield(2);
      IM_CHECK(gui::g_state.show_grid_line);

      // A combo cell writes the enum the main UI's radio buttons write. Opened and picked by hand
      // rather than through ComboClick: that helper splits its path at the FIRST '/', which lands
      // on the "**/" wildcard every ref in this file starts with.
      FilterTo(ctx, "renderer.visible");
      const ImGuiID visible_combo = SettingsCellID(ctx, "##value_renderer.visible");
      IM_CHECK(visible_combo != 0);
      ctx->ItemClick(visible_combo);
      ctx->Yield(2);
      // The popup's entries ARE label-addressable (Selectable registers its label), so only the
      // combo button itself needs the id treatment.
      ImGuiWindow* combo_popup = ctx->GetWindowByRef("//$FOCUSED");
      IM_CHECK(combo_popup != nullptr);
      ctx->ItemClick((std::string("//") + combo_popup->Name + "/**/Upper").c_str());
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.visible, gui::kVisibleUpper);

      // Applicability: roll applies under a linear lens...
      FilterTo(ctx, "renderer.roll");
      IM_CHECK(ctx->ItemExists(ValueInputRef("renderer.roll").c_str()));
      IM_CHECK(!IsDisabled(ctx->ItemInfo(ValueInputRef("renderer.roll").c_str())));
      FilterTo(ctx, "");
      panel.Close();

      // ...and not under a full-sky one, in the table exactly as in the main UI.
      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      IM_CHECK(gui::LensIsFullSky(gui::g_state.renderer.lens_type));
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.roll");
      IM_CHECK(IsDisabled(ctx->ItemInfo(ValueInputRef("renderer.roll").c_str())));
      FilterTo(ctx, "");
    };
  }

  {
    // A field with no registered editor cannot be edited here, and does not merely LOOK
    // uneditable — the row is still drawn (it is still a default the user can hold), which is what
    // makes the negative claim non-vacuous: the checkbox is found, so the row was there, and the
    // absence of a value widget is a fact about the cell rather than about the scroll position.
    //
    // Every id shape a value cell could have taken is probed, and a registered neighbour is probed
    // the same way as the positive control.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_row_with_no_registered_editor_has_no_control_at_all");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_unregistered");

      // The registry's answer, asked directly. This IS the criterion the panel branches on, so it
      // is asserted rather than inferred from what got drawn.
      IM_CHECK(gui::FindFieldEditor("bg_path") == nullptr);
      IM_CHECK(gui::FindFieldEditor("overlay_sun_circle_angles") == nullptr);
      IM_CHECK(gui::FindFieldEditor("overlay_grid_alpha") != nullptr);

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      // Per key, non-fatally: the two are different unregistered SHAPES (a path string, an array),
      // and which of them grew an editor is the diagnosis.
      for (const char* unregistered : { "bg_path", "overlay_sun_circle_angles" }) {
        FilterTo(ctx, unregistered);
        if (!ctx->ItemExists(AdoptCheckboxRef(unregistered).c_str())) {
          IM_ERRORF("row '%s' was not rendered at all, so its cell says nothing", unregistered);
          continue;
        }
        if (AnyValueWidgetExists(ctx, unregistered)) {
          IM_ERRORF("row '%s' has an editor, but no field editor is registered for it", unregistered);
        }

        if (ctx->IsError()) {
          break;
        }
      }

      // The positive control, through the same probe: a registered row of the same list DOES carry
      // a value widget, so the probe can tell the two apart.
      FilterTo(ctx, "overlay_grid_alpha");
      IM_CHECK(AnyValueWidgetExists(ctx, "overlay_grid_alpha"));
      FilterTo(ctx, "");
    };
  }

  {
    // "Origin value" is the FACTORY value, not the effective (saved) default. Under the copy model
    // up to four values are in play for one key — factory, what is saved, what the GUI holds now,
    // what Save would write — and showing the saved value here made "I changed this but have not
    // saved" and "I saved this" render identically.
    //
    // Asserted through the drawn cell rather than through the row struct: that cell's id carries
    // the text it rendered, so this fails if the column is re-pointed at the saved default even
    // though the row struct still holds the right factory value. The two are deliberately made
    // different first, otherwise the claim is unfalsifiable.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_origin_column_shows_the_factory_value");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_origin");

      json doc;
      doc["overlay_grid_alpha"] = 0.75f;  // saved default, deliberately not the factory value
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_grid_alpha");

      const auto rows = CurrentRows();
      const auto row = std::find_if(rows.begin(), rows.end(),
                                    [](const gui::DefaultDiffRow& r) { return r.key_path == "overlay_grid_alpha"; });
      IM_CHECK(row != rows.end());
      IM_CHECK(row->has_saved_override);
      IM_CHECK(row->default_value != row->factory_value);  // the premise

      const std::string factory_text = gui::FormatDiffValue(row->factory_value);
      const std::string saved_text = gui::FormatDiffValue(row->default_value);
      IM_CHECK(ctx->ItemExists(OriginCellRef("overlay_grid_alpha", factory_text).c_str()));
      IM_CHECK(!ctx->ItemExists(OriginCellRef("overlay_grid_alpha", saved_text).c_str()));
      FilterTo(ctx, "");
    };
  }

  {
    // The two notice icons are separate, and each has a producer that can actually fire. They
    // answer different questions ("you changed this" versus "what was loaded is out of range"), a
    // row can legitimately carry both, and a priority rule would hide whichever the user needed.
    //
    // The out-of-range icon's REACHABILITY is the finding this pins, because it inverts the obvious
    // guess: the shared slider control ends with an unconditional clamp and the main UI calls it
    // every frame, so a hand-edited out-of-range alpha is pulled back into its domain before this
    // panel ever sees it. The reachable fields are the ones with no main-UI control at all. Both
    // directions are asserted — the field that keeps the poison and two that cannot — so "this
    // notice can fire" is not confused with "this notice fires for everything".
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_two_note_icons_are_independent");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_notes");

      // A hand-edited defaults file, out of range on three fields.
      json doc;
      doc["renderer"]["opacity"] = 3.0f;  // no main-UI control => nothing clamps it
      doc["overlay_grid_alpha"] = 7.0f;   // its own slider clamps it every frame
      doc["renderer"]["fov"] = 4000.0f;   // the per-frame renderer invariant clamps it
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 3.0f);  // the poison did land
      ctx->Yield(4);  // let the main UI render — this is where the other two get pulled back

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      // Positive: the field nothing clamps still holds its out-of-range value, and says so.
      FilterTo(ctx, "renderer.opacity");
      IM_CHECK(ctx->ItemExists("**/###note_range_renderer.opacity"));
      IM_CHECK(!ctx->ItemExists("**/###note_edited_renderer.opacity"));

      // Negative, twice, for the two different mechanisms that pull a value back in range.
      FilterTo(ctx, "overlay_grid_alpha");
      IM_CHECK_EQ(gui::g_state.grid_alpha, 1.0f);
      IM_CHECK(!ctx->ItemExists("**/###note_range_overlay_grid_alpha"));
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(!ctx->ItemExists("**/###note_range_renderer.fov"));

      // The other icon: editing a value HERE marks that row, and only that row.
      ctx->ItemInputValue(ValueInputRef("renderer.fov").c_str(), 45.0f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.fov, 45.0f);
      IM_CHECK(ctx->ItemExists("**/###note_edited_renderer.fov"));
      FilterTo(ctx, "renderer.elevation");
      IM_CHECK(!ctx->ItemExists("**/###note_edited_renderer.elevation"));

      // Both at once, on one row: the out-of-range field, once edited, carries the pencil AND keeps
      // the warning until the edit takes it back into the domain. Editing it to a value inside
      // [0,1] clears the warning and leaves the pencil, which is the pair's whole point.
      FilterTo(ctx, "renderer.opacity");
      ctx->ItemInputValue(ValueInputRef("renderer.opacity").c_str(), 0.25f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 0.25f);
      IM_CHECK(ctx->ItemExists("**/###note_edited_renderer.opacity"));
      IM_CHECK(!ctx->ItemExists("**/###note_range_renderer.opacity"));
      FilterTo(ctx, "");
    };
  }

  {
    // The round trip the whole Current value column exists for: change a setting the user has never
    // touched in the main UI, from the table, and have it land in the defaults file.
    //
    // Also pins the two consequences of an in-cell edit that are decisions rather than mechanics:
    // the row joins the checked set (otherwise the edit would be silently dropped at Save), and
    // nothing is written until Save is actually pressed.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_cell_edit_adopts_its_row_and_lands_only_on_save");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_cell_roundtrip");
      gui::g_state = gui::MakeNewDocumentState();

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_zenith_nadir_radius_px");
      IM_CHECK(!RowIsChecked(ctx, "overlay_zenith_nadir_radius_px"));  // untouched rows open unchecked

      ctx->ItemInputValue(ValueInputRef("overlay_zenith_nadir_radius_px").c_str(), 12.5f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, 12.5f);
      // The edit adopted the row: an edit that Save would discard is the failure this avoids.
      IM_CHECK(RowIsChecked(ctx, "overlay_zenith_nadir_radius_px"));
      // ...but nothing has been written yet.
      IM_CHECK(!ReadOverlayBytes(panel.dir()).has_value() ||
               !gui::DocHasKeyPath(ReadOverlayFile(panel.dir()), "overlay_zenith_nadir_radius_px"));

      SaveDefaultsPanel(ctx);
      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(gui::DocHasKeyPath(saved, "overlay_zenith_nadir_radius_px"));
      IM_CHECK_EQ(saved["overlay_zenith_nadir_radius_px"].get<float>(), 12.5f);
      // Saved => the "you changed this here" pencil is retired: it would now be a lie.
      IM_CHECK(!ctx->ItemExists("**/###note_edited_overlay_zenith_nadir_radius_px"));
      FilterTo(ctx, "");
    };
  }

  {
    // A colour cell commits on RELEASE, not per frame.
    //
    // Regression guard for a "logically distinct control, structurally identical per-frame-commit
    // bug" family: a field type must gate its write-back on the interaction actually ending, not on
    // ImGui's per-frame "value changed" return. Sliders get that from IsItemDeactivatedAfterEdit();
    // the colour field carried the identical bug because the picker popup is drag-continuous just
    // like a slider, but its return value was taken as the commit signal directly.
    //
    // This is the case that CAN tell the two apart: every other cell case drives its control with a
    // single-frame ItemInputValue, which never holds a frame in between and so cannot distinguish
    // "changed on release" from "changed every frame".
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_colour_cell_commits_on_release_not_per_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_colour_commit");
      gui::g_state = gui::MakeNewDocumentState();

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_grid_color");

      const float base[3] = { gui::g_state.grid_color[0], gui::g_state.grid_color[1], gui::g_state.grid_color[2] };

      // The swatch button id, reproducing ColorEdit4's internal PushID(label) + ColorButton
      // ("##ColorButton", ...) chain against the table-seeded group id — see SettingsCellID's own
      // comment for why this has to be computed rather than found by a label search.
      const ImGuiID group_id = SettingsCellID(ctx, "##value_overlay_grid_color");
      IM_CHECK(group_id != 0);
      const ImGuiID swatch_id = ImGui::GetIDWithSeed("##ColorButton", nullptr, group_id);
      ctx->ItemClick(swatch_id);
      ctx->Yield(2);

      // Inside the picker popup now. "sv" is the saturation/value square — an ordinary
      // InvisibleButton, and unlike the swatch it IS registered with the test engine.
      IM_CHECK(ctx->ItemExists("**/sv"));
      const ImGuiTestItemInfo sv = ctx->ItemInfo("**/sv");
      const ImVec2 sv_lo = sv.RectFull.Min;
      const ImVec2 sv_hi = sv.RectFull.Max;
      // Two corners rather than "wherever the mouse already is": guarantees a real S/V delta
      // regardless of the document's starting grid_color.
      const ImVec2 start(sv_lo.x + (sv_hi.x - sv_lo.x) * 0.15f, sv_lo.y + (sv_hi.y - sv_lo.y) * 0.15f);
      const ImVec2 end(sv_lo.x + (sv_hi.x - sv_lo.x) * 0.85f, sv_lo.y + (sv_hi.y - sv_lo.y) * 0.85f);

      ctx->MouseMoveToPos(start);
      ctx->MouseDown(0);
      ctx->Yield(2);
      // Held, not yet released: the pre-fix bug wrote every one of these frames straight into
      // grid_color — this is the assertion that would have caught it.
      IM_CHECK_EQ(gui::g_state.grid_color[0], base[0]);
      IM_CHECK_EQ(gui::g_state.grid_color[1], base[1]);
      IM_CHECK_EQ(gui::g_state.grid_color[2], base[2]);

      ctx->MouseMoveToPos(end);
      ctx->Yield(2);
      // Still held after moving further — same claim a second time, so a fix that merely
      // special-cased the first frame of the drag would not pass.
      IM_CHECK_EQ(gui::g_state.grid_color[0], base[0]);
      IM_CHECK_EQ(gui::g_state.grid_color[1], base[1]);
      IM_CHECK_EQ(gui::g_state.grid_color[2], base[2]);

      ctx->MouseUp(0);
      ctx->Yield(2);
      // Released: the commit lands now, and the drag really did reach a different colour (sanity
      // that the corner-to-corner move landed on the square rather than missing it).
      IM_CHECK(gui::g_state.grid_color[0] != base[0] || gui::g_state.grid_color[1] != base[1] ||
               gui::g_state.grid_color[2] != base[2]);

      ctx->PopupCloseAll();
      FilterTo(ctx, "");
    };
  }

  {
    // A float cell's SLIDER lands what the drag left behind — on release, and only then.
    //
    // The sibling of the colour case above, and it fails on the OPPOSITE half of the same rule. A
    // slider does get a trustworthy "the interaction ended" signal out of ImGui, so the release
    // frame is recognised; what is easy to lose is the VALUE by then. The cell keeps a working copy
    // so that the frames of a drag never reach the document, and if that copy is refreshed from the
    // document on every frame it is also refreshed on the release frame — where the slider, having
    // only reported that it is finished, writes nothing more. The commit then compares the pre-drag
    // value against itself, finds no change, and drops the whole drag on the floor. The user-visible
    // shape of that is precise and was the reported bug: the box beside the slider works, and the
    // slider does not.
    //
    // Which is also why the case has to be a real press/move/release rather than the single-frame
    // ItemInputValue every other value-cell case uses: that helper drives the INPUT BOX, i.e. the
    // one path that never had the bug.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_float_slider_cell_lands_the_drag_on_release");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_float_slider_drag");
      gui::g_state = gui::MakeNewDocumentState();

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_zenith_nadir_radius_px");

      const float base = gui::g_state.zenith_nadir_radius_px;  // 8.0, in a [2, 20] domain
      const ImGuiTestItemInfo slider = ctx->ItemInfo(ValueSliderRef("overlay_zenith_nadir_radius_px").c_str());
      // Asked before the drag, so that "this test is not addressing the widget it thinks it is"
      // reports as itself instead of as "the value did not land" — telling those two apart is the
      // whole reason this case can be trusted when it goes red.
      IM_CHECK(slider.ID != 0);

      {
        ScopedMouseDown drag(ctx, PointAcross(slider, 0.15f));
        // Pressing an ImGui slider already moves its value to the press position, so a cell that
        // wrote per frame would have written a different number here — this is not a vacuous
        // "nothing happened yet" assertion.
        IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, base);

        drag.MoveTo(PointAcross(slider, 0.85f));
        // Still held after moving the other way across the bar: a fix that special-cased only the
        // first frame of a drag would not survive this second look.
        IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, base);

        drag.Release();
      }
      // Released: the drag lands, and it lands where the mouse was — not merely "somewhere else".
      // 0.85 of the way along [2, 20] is ~17; the bound below is loose enough to survive the grab
      // rectangle's own width and tight enough that a stale or half-way value fails it.
      IM_CHECK_NE(gui::g_state.zenith_nadir_radius_px, base);
      IM_CHECK_GT(gui::g_state.zenith_nadir_radius_px, 12.0f);
      IM_CHECK_LE(gui::g_state.zenith_nadir_radius_px, 20.0f);

      // The keyboard path is the half that always worked; asserted here so a fix that traded one
      // for the other cannot pass. Typing after a drag is also the real sequence a user performs.
      ctx->ItemInputValue(ValueInputRef("overlay_zenith_nadir_radius_px").c_str(), 5.5f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, 5.5f);

      FilterTo(ctx, "");
    };
  }

  {
    // The integer cell says the same thing, and is not covered by the float one: it is a separate
    // control (SliderInt + InputInt) behind a separate factory, and the two carried the bug
    // independently rather than through shared code.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "an_int_slider_cell_lands_the_drag_on_release");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_int_slider_drag");
      gui::g_state = gui::MakeNewDocumentState();

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "sim.max_hits");

      const int base = gui::g_state.sim.max_hits;  // 8, in a [1, 64] domain
      const ImGuiTestItemInfo slider = ctx->ItemInfo(ValueSliderRef("sim.max_hits").c_str());
      IM_CHECK(slider.ID != 0);

      {
        ScopedMouseDown drag(ctx, PointAcross(slider, 0.15f));
        IM_CHECK_EQ(gui::g_state.sim.max_hits, base);

        drag.MoveTo(PointAcross(slider, 0.85f));
        IM_CHECK_EQ(gui::g_state.sim.max_hits, base);

        drag.Release();
      }
      // ~0.85 of [1, 64] is in the mid-fifties.
      IM_CHECK_NE(gui::g_state.sim.max_hits, base);
      IM_CHECK_GT(gui::g_state.sim.max_hits, 32);
      IM_CHECK_LE(gui::g_state.sim.max_hits, 64);

      ctx->ItemInputValue(ValueInputRef("sim.max_hits").c_str(), 12);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.sim.max_hits, 12);

      FilterTo(ctx, "");
    };
  }

  {
    // Two slider cells on screen AT THE SAME TIME each drag on their own, and this is a claim about
    // the storage rather than about the widgets.
    //
    // The working copy a slider cell keeps across the frames of a drag cannot live in one shared
    // variable per field kind: every visible row of that kind renders on every frame, so the second
    // row's render would see the first row's "I am being dragged" flag, decline to refresh, take the
    // first row's in-progress value as its own — and then clear the flag, so the NEXT frame the row
    // actually being dragged would refresh from the document and lose the drag. The scratch is
    // therefore keyed per field, and this is the case that would go red if it stopped being.
    //
    // Both rows have to be on screen for that to be reachable at all, which is what the shared
    // "overlay_zenith_nadir" filter buys: it leaves the radius (a [2, 20] float) and the alpha (a
    // [0, 1] float) rendering side by side.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "two_slider_cells_do_not_share_one_drag_scratch");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_slider_scratch_keying");
      gui::g_state = gui::MakeNewDocumentState();

      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_zenith_nadir");

      const float alpha_base = gui::g_state.zenith_nadir_alpha;  // 0.6, in [0, 1]
      const ImGuiTestItemInfo radius = ctx->ItemInfo(ValueSliderRef("overlay_zenith_nadir_radius_px").c_str());
      IM_CHECK(radius.ID != 0);
      const ImGuiTestItemInfo alpha = ctx->ItemInfo(ValueSliderRef("overlay_zenith_nadir_alpha").c_str());
      IM_CHECK(alpha.ID != 0);

      {
        ScopedMouseDown drag(ctx, PointAcross(radius, 0.9f));
        drag.MoveTo(PointAcross(radius, 0.75f));
        drag.Release();
      }
      IM_CHECK_GT(gui::g_state.zenith_nadir_radius_px, 12.0f);
      // The neighbour was rendering throughout and was never touched.
      IM_CHECK_EQ(gui::g_state.zenith_nadir_alpha, alpha_base);
      const float radius_landed = gui::g_state.zenith_nadir_radius_px;

      {
        ScopedMouseDown drag(ctx, PointAcross(alpha, 0.1f));
        drag.MoveTo(PointAcross(alpha, 0.2f));
        drag.Release();
      }
      // The alpha drag lands its OWN value — a scratch shared with the radius would have offered a
      // number from the other field's domain, clamped into [0, 1] as 1.0.
      IM_CHECK_NE(gui::g_state.zenith_nadir_alpha, alpha_base);
      IM_CHECK_LT(gui::g_state.zenith_nadir_alpha, 0.4f);
      // ...and does not disturb what the radius already committed.
      IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, radius_landed);

      FilterTo(ctx, "");
    };
  }

  // ===============================================================================================
  // Geometry: what stays put while the list moves.
  //
  // Two INDEPENDENT mechanisms, and neither substitutes for the other:
  //   the table's frozen header row       -> TableSetupScrollFreeze(0, 1)
  //   the section headers standing still  -> they are not inside any scrolling region at all
  // A single case covering both would not say which one broke.
  // ===============================================================================================

  {
    // The settings table's header row does not move, and its columns do not shift, while the body
    // scrolls under it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_table_header_stays_put_while_the_body_scrolls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_frozen_header");
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());

      ImGuiTable* table = SettingsTable(ctx);
      IM_CHECK(table != nullptr);
      IM_CHECK(table->InnerWindow != nullptr);
      RewindSettingsList(ctx, table);

      // The premise, asserted rather than assumed: the list is longer than the box it was given, so
      // there is something for the wheel to do. Without it, "the header did not move" would be just
      // as true of a list too short to scroll at all.
      IM_CHECK_GT(table->InnerWindow->ScrollMax.y, 0.0f);
      IM_CHECK_EQ(table->InnerWindow->Scroll.y, 0.0f);

      // Two header cells rather than one, at opposite ends of the row: a header that stayed at the
      // right Y while its columns slid sideways would satisfy a single-cell check.
      const auto setting_before = SettingsHeaderInfo(ctx, table, "Setting");
      const auto source_before = SettingsHeaderInfo(ctx, table, "Source");
      IM_CHECK(setting_before.ID != 0);
      IM_CHECK(source_before.ID != 0);

      const std::string top_row = AdoptCheckboxRef(rows.front().key_path);
      IM_CHECK(RowIsOnScreen(ctx, top_row));

      ScrollSettingsListDown(ctx, top_row);

      // The list moved, and it is the LIST that moved: the wheel landed on the table's own scroll
      // region, and the row that was under the header is no longer drawn.
      table = SettingsTable(ctx);
      IM_CHECK(table != nullptr);
      IM_CHECK(table->InnerWindow != nullptr);
      IM_CHECK_GT(table->InnerWindow->Scroll.y, 0.0f);
      IM_CHECK(!RowIsOnScreen(ctx, top_row));

      // ...and the header did not, in either axis, and is still drawn (a header scrolled out of the
      // clip rect would keep its RectFull and report an empty RectClipped).
      const auto setting_after = SettingsHeaderInfo(ctx, table, "Setting");
      const auto source_after = SettingsHeaderInfo(ctx, table, "Source");
      IM_CHECK_EQ(setting_after.RectFull.Min.y, setting_before.RectFull.Min.y);
      IM_CHECK_EQ(setting_after.RectFull.Min.x, setting_before.RectFull.Min.x);
      IM_CHECK_EQ(source_after.RectFull.Min.x, source_before.RectFull.Min.x);
      IM_CHECK_GT(setting_after.RectClipped.GetHeight(), 0.0f);
      IM_CHECK_GT(source_after.RectClipped.GetHeight(), 0.0f);
    };
  }

  {
    // The two section headers do not move while a section's body scrolls, and the panel window
    // itself never scrolls.
    //
    // The second half is the two-layer-scroll trap: a table given its own ScrollY while a shared
    // child was still wrapped around it would look right in a screenshot and feel wrong under the
    // wheel. ScrollMax.y == 0 on the panel says there is exactly one scrolling region under the
    // pointer, and it is the section's.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "the_section_headers_do_not_scroll_and_the_panel_never_does");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_section_headers");
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      // Both sections expanded, so both headers are on screen at once and the one above the
      // scrolling body is as much at risk as the one below it.
      ctx->ItemOpen("**/###defaults_presets");
      ctx->Yield(3);

      ImGuiTable* table = SettingsTable(ctx);
      IM_CHECK(table != nullptr);
      IM_CHECK(table->InnerWindow != nullptr);
      RewindSettingsList(ctx, table);
      IM_CHECK_GT(table->InnerWindow->ScrollMax.y, 0.0f);

      const auto presets_before = ctx->ItemInfo("**/###defaults_presets");
      const auto settings_before = ctx->ItemInfo("**/###defaults_settings");
      IM_CHECK(presets_before.ID != 0);
      IM_CHECK(settings_before.ID != 0);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());
      const std::string top_row = AdoptCheckboxRef(rows.front().key_path);
      IM_CHECK(RowIsOnScreen(ctx, top_row));

      ScrollSettingsListDown(ctx, top_row);
      IM_CHECK(!RowIsOnScreen(ctx, top_row));  // non-vacuous: something did scroll

      const auto presets_after = ctx->ItemInfo("**/###defaults_presets");
      const auto settings_after = ctx->ItemInfo("**/###defaults_settings");
      IM_CHECK_EQ(presets_after.RectFull.Min.y, presets_before.RectFull.Min.y);
      IM_CHECK_EQ(settings_after.RectFull.Min.y, settings_before.RectFull.Min.y);
      // Neither header is merely "at the same coordinates while clipped away".
      IM_CHECK_GT(presets_after.RectClipped.GetHeight(), 0.0f);
      IM_CHECK_GT(settings_after.RectClipped.GetHeight(), 0.0f);

      // The panel window has nothing to scroll: every scrolling region is inside a section.
      ImGuiWindow* win = ctx->GetWindowByRef(gui::kDefaultsPanelTitle);
      IM_CHECK(win != nullptr);
      IM_CHECK_EQ(win->ScrollMax.y, 0.0f);
    };
  }

  {
    // The action row is pinned to the bottom edge and does not ride the list's height.
    //
    // The sections are sized to their content, so without the pin the row floats up under a short
    // list and sits at a different height on every keystroke in the search box — a destructive
    // button that moves while the user is reading is worse than one sitting in dead space.
    //
    // Driven by typing the search box one character at a time, because that is the gesture that
    // changes the visible row count continuously; the assertion is that all three buttons keep the
    // Y they had with the full list.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_action_row_stays_pinned_while_the_list_shrinks");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_pinned_actions");
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);

      const char* kActions[] = { "**/###defaults_save", "**/###defaults_close", "**/###defaults_reset_all" };
      float baseline[IM_ARRAYSIZE(kActions)] = {};
      // Per action, non-fatally, for the same reason the comparison loop below is: three buttons
      // share the row and a run should name every one that is missing, not only the leftmost.
      for (int i = 0; i < IM_ARRAYSIZE(kActions); ++i) {
        const auto info = ctx->ItemInfo(kActions[i], ImGuiTestOpFlags_NoError);
        if (info.ID == 0) {
          IM_ERRORF("action '%s' is not in the panel at all", kActions[i]);
          continue;
        }
        baseline[i] = info.RectFull.Min.y;

        if (ctx->IsError()) {
          break;
        }
      }

      // The premise: the list really does shrink as this is typed, so the row above the action row
      // is changing height. Checked at the end against the row count the header reports.
      const int total = static_cast<int>(CurrentRows().size());
      IM_CHECK(total > 1);

      const char* kTyped = "renderer.fov";
      std::string so_far;
      for (const char* c = kTyped; *c != '\0'; ++c) {
        so_far.push_back(*c);
        FilterTo(ctx, so_far.c_str());
        for (int i = 0; i < IM_ARRAYSIZE(kActions); ++i) {
          const auto info = ctx->ItemInfo(kActions[i], ImGuiTestOpFlags_NoError);
          // Reported rather than asserted, so the line names the prefix length and the two Y values
          // instead of "false is not true". Only the FIRST offending prefix is reported either way:
          // the test context short-circuits every action once an error is on record, which is also
          // the most useful one — it is where the row started moving.
          if (info.ID == 0) {
            IM_ERRORF("action '%s' vanished at search prefix '%s'", kActions[i], so_far.c_str());
          } else if (info.RectFull.Min.y != baseline[i]) {
            IM_ERRORF("action '%s' moved from y=%.1f to y=%.1f at search prefix '%s'", kActions[i], baseline[i],
                      info.RectFull.Min.y, so_far.c_str());
          }

          if (ctx->IsError()) {
            break;
          }
        }

        if (ctx->IsError()) {
          break;
        }
      }

      // ...and the list really was down to one row by the end, so the pin was tested against the
      // largest change the search box can make rather than against a list that never moved.
      {
        char expected[64];
        ImFormatString(expected, IM_ARRAYSIZE(expected), "Settings (1 of %d)", total);
        IM_CHECK(StartsWith(DrawnLabel(ctx, "**/###defaults_settings"), expected));
      }
      FilterTo(ctx, "");
    };
  }

  // ===============================================================================================
  // The preset library.
  // ===============================================================================================

  {
    // A std edit is clamped to the preset's open domain, and the warning cell follows the VALUE
    // rather than the event: 25 is above Column's domain and must be adjusted with the warning
    // appearing; 0.3 is inside it and must be stored verbatim with the warning GONE. A one-sided
    // case would be satisfied by a panel that clamped everything, and a warning that outlived the
    // value that caused it would point at a number that is no longer there.
    //
    // Each state is asserted after a Save, because a preset edit is an edit of the working copy
    // like every other edit in this panel. The warning assertions are not: they are pure UI state
    // and never depended on the file.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "a_preset_std_edit_clamps_and_the_warning_follows_the_value");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_preset_clamp");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(3);

      ctx->ItemInputValue("**/###preset_std_column", 25.0f);
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/###preset_warning_column"));
      SaveDefaultsPanel(ctx);
      {
        const auto stored = ReadPresetStd(ReadOverlayFile(panel.dir()), "column");
        IM_CHECK(stored.has_value());
        IM_CHECK(*stored < gui::kColumnPlateParryZenithStdUpperBound);
        IM_CHECK(*stored > 0.0f);
        // The clamp is reported once, when the panel adjusts the value — not again on the Save that
        // commits it. A second notice would tell the user something new had gone wrong.
        IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
      }

      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists("**/###preset_warning_column"));
      SaveDefaultsPanel(ctx);
      {
        const auto stored = ReadPresetStd(ReadOverlayFile(panel.dir()), "column");
        IM_CHECK(stored.has_value());
        IM_CHECK_EQ(*stored, 0.3f);
      }
    };
  }

  {
    // A clamped std is SHOWN with enough digits to read back as the value that was stored.
    //
    // The domains are open, so a clamp lands on a neighbouring float: Lowitz's lower bound clamps
    // to nextafter(15, +inf) = 15.000001, which a fixed 7-digit format renders as "15" — directly
    // under a line telling the user the value must stay GREATER than 15. That is the shape of the
    // defect this guards, and it is why the precision is computed per value rather than fixed.
    //
    // Read by activating the box, which is the only way its text is observable at all (see
    // ActivatedInputText) and is also what a user does before reading a number closely. The rule
    // itself is pinned off-screen in composition-correctness/gui/test_user_defaults_chain.cpp;
    // what is left for this layer is whether the box on screen goes through it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_clamped_std_is_shown_with_enough_digits_to_read_back");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_preset_precision");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Lowitz");
      ctx->Yield(3);

      // Below Lowitz's lower bound, so the clamp lands just INSIDE it — the case where the extra
      // digit is the difference between a true reading and a lie.
      ctx->ItemInputValue("**/###preset_std_lowitz", 1.0f);
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/###preset_warning_lowitz"));

      const std::string shown = ActivatedInputText(ctx, "**/###preset_std_lowitz");
      IM_CHECK(!shown.empty());
      // The claim, stated as the reader's own test rather than against a number: what the box shows
      // must READ BACK as a value inside the domain. "15" parses to exactly the excluded bound, so
      // this fails on the defect and passes only on a rendering that carries the extra digit.
      IM_CHECK_STR_NE(shown.c_str(), "15");
      IM_CHECK_GT(std::strtof(shown.c_str(), nullptr), gui::kLowitzZenithStdLowerBound);

      // ...and it is the value that would actually be STORED, not a prettier neighbour of it. Read
      // after a Save because the edit is in the working copy until then — the process-wide cache
      // still answers the factory value while the panel is open, which is a different case's
      // subject and would make a comparison against it meaningless here.
      SaveDefaultsPanel(ctx);
      const auto stored = ReadPresetStd(ReadOverlayFile(panel.dir()), "lowitz");
      IM_CHECK(stored.has_value());
      IM_CHECK_EQ(std::strtof(shown.c_str(), nullptr), *stored);
    };
  }

  {
    // Restore to factory drops THIS preset's override and leaves its neighbour alone — and, like
    // every other edit here, reaches the file only through Save. The store-side function is proved
    // elsewhere; this proves the button is wired to it and to the right preset.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "restore_to_factory_is_wired_per_preset");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_preset_restore");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(2);
      ctx->ItemOpen("**/###preset_Plate");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_plate", 0.5f);
      ctx->Yield(2);
      SaveDefaultsPanel(ctx);

      const auto before_restore = ReadOverlayBytes(panel.dir());
      IM_CHECK(before_restore.has_value());
      ctx->ItemClick("**/###preset_restore_column");
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before_restore);
      SaveDefaultsPanel(ctx);

      const json saved = ReadOverlayFile(panel.dir());
      IM_CHECK(!ReadPresetStd(saved, "column").has_value());
      const auto survivor = ReadPresetStd(saved, "plate");
      IM_CHECK(survivor.has_value());
      IM_CHECK_EQ(*survivor, 0.5f);

      // The input box now reads the factory value back, not the number the user last typed.
      IM_CHECK_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std,
                  gui::AxisPresetEntryFor(gui::AxisPreset::kColumn).zenith.std);
    };
  }

  {
    // A preset with no adjustable face offers NO input that would be ignored — a box that accepted
    // a number and discarded it would be worse than saying so in words.
    //
    // A structural sweep over the library table rather than a look at one name: the std input and
    // the restore button must exist for every adjustable preset and for none of the others, derived
    // from the same table the panel renders from, so this cannot drift into checking a stale list
    // of four names. It also pins the library's membership — the classifier's "none of the above"
    // is not a built-in identity and has nothing for a library to hold about it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_preset_with_no_adjustable_face_offers_no_input");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_preset_membership");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);

      int adjustable_seen = 0;
      int fixed_seen = 0;
      // Reported per preset rather than asserted fatally: "which preset is wrong" is the entire
      // content of a failure here, and a fatal assert would also leave whichever node it was
      // inspecting unfolded, which the teardown then has to undo for a case that never reached its
      // own end. It does not let the sweep continue past the first bad preset — the context stops
      // responding once anything is on record (see ctx->IsError() in test_gui_shared.hpp).
      for (const auto& entry : gui::kAxisPresets) {
        const std::string node_ref = std::string("**/###preset_") + gui::AxisPresetLabel(entry.id);
        if (entry.id == gui::AxisPreset::kCustom) {
          if (ctx->ItemExists(node_ref.c_str())) {
            IM_ERRORF("'%s' is the classifier's 'none of the above', not a library entry",
                      gui::AxisPresetLabel(entry.id));
          }
          continue;
        }
        if (!ctx->ItemExists(node_ref.c_str())) {
          IM_ERRORF("preset '%s' is missing from the library", gui::AxisPresetLabel(entry.id));
          continue;
        }
        ctx->ItemOpen(node_ref.c_str());
        ctx->Yield(2);

        if (entry.has_adjustable_zenith_std) {
          ++adjustable_seen;
          const std::string std_ref = std::string("**/###preset_std_") + entry.override_json_name;
          const std::string restore_ref = std::string("**/###preset_restore_") + entry.override_json_name;
          if (!ctx->ItemExists(std_ref.c_str())) {
            IM_ERRORF("preset '%s' is adjustable but offers no std input", entry.label);
          }
          if (ctx->IsError()) {
            break;
          }
          if (!ctx->ItemExists(restore_ref.c_str())) {
            IM_ERRORF("preset '%s' is adjustable but offers no restore button", entry.label);
          }
          if (ctx->IsError()) {
            break;
          }
        } else {
          ++fixed_seen;
          // Nothing keyed on a json name can exist for it — it has none. Probing the id an
          // adjustable preset would carry, spelled from the label instead, since that is the only
          // name this preset has.
          const std::string std_ref = std::string("**/###preset_std_") + gui::AxisPresetLabel(entry.id);
          if (ctx->ItemExists(std_ref.c_str())) {
            IM_ERRORF("preset '%s' has nothing to tune but offers an input anyway", entry.label);
          }
        }
        // Either branch above may have just reported non-fatally; ItemClose/Yield below drive ctx
        // unconditionally and must not run on top of that (see ctx->IsError() in
        // test_gui_shared.hpp).
        if (ctx->IsError()) {
          break;
        }
        ctx->ItemClose(node_ref.c_str());
        ctx->Yield(1);
      }
      IM_CHECK_EQ(adjustable_seen, 4);  // Column / Plate / Parry / Lowitz
      IM_CHECK_EQ(fixed_seen, 1);       // Random

      // Nothing was written by merely looking at the library.
      IM_CHECK(ReadOverlayFile(panel.dir()).empty());
    };
  }

  {
    // The axes a preset does not let you edit are DISABLED CONTROLS, not text.
    //
    // The distinction is the whole point of drawing them that way: a control shows the shape of the
    // value (a distribution picker, a number) while being honest that it cannot be changed here. A
    // text cell would leave the user hunting for the editor; an enabled control would be a lie.
    //
    // Falsifiable because the two shapes differ in what reaches the test engine at all:
    // ImGui::Text* items are submitted with id 0 and are invisible here, so an item that EXISTS is
    // already evidence it is a control, and the disabled flag is the second half.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "the_axes_a_preset_does_not_let_you_edit_are_disabled_controls");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_readonly_axes");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      // Column, the FIRST preset in the library, and that choice is forced rather than arbitrary:
      // the section's body is a bounded child, so a preset further down the list has its table
      // below the visible band — and a clipped item is unaware of its label, i.e. invisible to the
      // engine's search. Measured, on the last preset in the list: all three cells came back "not
      // an item at all", which is the same answer a text cell would give and would have read as
      // this case's own subject failing.
      //
      // Column also puts the contrast in one table: its Zenith std is the one editable cell in the
      // library, and the cells around it are the read-only ones. Assert both and the claim is
      // "these are disabled" rather than "everything here happens to be disabled".
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(3);

      // The anchor: the one cell in this table that CAN be addressed by label, because its id is
      // unique. Two jobs. It is asked first and fatally, because every probe below is a report
      // rather than an abort and the test context short-circuits once ANY error is on record — a
      // collapsed node would otherwise produce six "drawn as text?" lines that are all collateral
      // from one missing table. And its window is the seed the other cells' ids are derived from.
      const auto anchor = ctx->ItemInfo("**/###preset_std_column");
      IM_CHECK(anchor.ID != 0);
      IM_CHECK(anchor.Window != nullptr);
      IM_CHECK(!IsDisabled(anchor));  // the sibling that IS editable, so "disabled" below is a fact
                                      // about which cells are read-only rather than an artifact of
                                      // how this case reads them

      // The derivation, checked against the one id this case already knows, before it is trusted on
      // the cells it is the only way to reach. A silently wrong chain would report every cell as
      // "not an item at all" — which is indistinguishable from the defect under test.
      IM_CHECK_EQ(PresetAxisCellID(anchor.Window, "Column", "zenith", "###preset_std_column"), anchor.ID);

      // Every read-only cell of the two axes Column does not let you tune, one report each: which
      // of the six stopped being a control is the diagnosis, and a fatal assertion would name only
      // the first — and, worse, would leave the remaining probes unrun, since the test context
      // short-circuits every action once an error is on record.
      for (const char* axis : { "azimuth", "roll" }) {
        for (const char* cell : { "##type", "##mean", "##std" }) {
          const ImGuiID id = PresetAxisCellID(anchor.Window, "Column", axis, cell);
          if (id == 0 || !ctx->ItemExists(id)) {
            IM_ERRORF("read-only cell %s/%s is not an item at all — drawn as text?", axis, cell);
            continue;
          }
          if (!IsDisabled(ctx->ItemInfo(id))) {
            IM_ERRORF("read-only cell %s/%s is enabled — this row cannot be edited here", axis, cell);
          }

          if (ctx->IsError()) {
            break;
          }
        }

        if (ctx->IsError()) {
          break;
        }
      }

      // The tunable row's own neighbours are read-only too: a Zenith the user can retune still does
      // not let them change its distribution family or its mean.
      for (const char* cell : { "##type", "##mean" }) {
        const ImGuiID id = PresetAxisCellID(anchor.Window, "Column", "zenith", cell);
        if (id == 0 || !ctx->ItemExists(id)) {
          IM_ERRORF("read-only cell zenith/%s is not an item at all — drawn as text?", cell);
          continue;
        }
        if (!IsDisabled(ctx->ItemInfo(id))) {
          IM_ERRORF("read-only cell zenith/%s is enabled — only the std is tunable here", cell);
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  {
    // A retuned preset says so in its COLLAPSED title, so the user can see which of the six they
    // have changed without opening all six. Both directions, since the suffix is the whole signal:
    // absent before the edit, present after it.
    //
    // Read through the drawn label — the node's id is the "###" form, so it is found either way.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_retuned_preset_says_so_in_its_collapsed_title");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_preset_mine_suffix");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);

      const char* kColumn = "**/###preset_Column";
      const char* kPlate = "**/###preset_Plate";
      IM_CHECK_STR_EQ(DrawnLabel(ctx, kColumn).c_str(), "Column###preset_Column");
      IM_CHECK_STR_EQ(DrawnLabel(ctx, kPlate).c_str(), "Plate###preset_Plate");

      ctx->ItemOpen(kColumn);
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(3);
      ctx->ItemClose(kColumn);
      ctx->Yield(2);

      // The edited one is marked, and only the edited one — a suffix driven by "any preset was
      // touched" would mark both.
      IM_CHECK_STR_EQ(DrawnLabel(ctx, kColumn).c_str(), "Column (mine)###preset_Column");
      IM_CHECK_STR_EQ(DrawnLabel(ctx, kPlate).c_str(), "Plate###preset_Plate");

      // It follows the COPY, not the file: the mark is there before Save, because the panel is
      // reporting what it holds rather than what is on disk.
      IM_CHECK(!ReadPresetStd(ReadOverlayFile(panel.dir()), "column").has_value());
    };
  }

  {
    // A preset edit closed without Save reaches NEITHER the process-wide cache NOR the file. The
    // cache half matters on its own: it is what the axis modal's preset buttons read, so a leak
    // there would hand the user an uncommitted value through a completely different surface.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "a_preset_edit_closed_without_save_reaches_neither_cache_nor_file");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_preset_discard");

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(panel.dir(), doc));
      gui::g_state = gui::MakeNewDocumentState();

      const auto before_bytes = ReadOverlayBytes(panel.dir());
      IM_CHECK(before_bytes.has_value());
      const auto before_cache = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(before_cache.has_value());
      IM_CHECK_EQ(*before_cache, 0.3f);

      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.7f);
      ctx->Yield(3);
      // The cache still answers the old value while the panel is open: the edit is uncommitted, not
      // ignored (the collapsed-title case above is where "the panel does show it" is asserted).
      IM_CHECK_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std, 0.3f);
      panel.Close();

      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before_bytes);
      IM_CHECK_EQ(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn), before_cache);

      // Restore-to-factory is the other direction of the same claim: it must not silently drop a
      // stored override either.
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemClick("**/###preset_restore_column");
      ctx->Yield(3);
      panel.Close();

      IM_CHECK_EQ(ReadOverlayBytes(panel.dir()), before_bytes);
      IM_CHECK_EQ(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn), before_cache);
    };
  }

  {
    // A Save that could not write must leave the process-wide preset cache exactly as it was.
    // Otherwise this session resolves one value for the Column button while the next launch reads
    // the older one off disk, and the user watches their setting "come back" with no event to
    // attribute it to.
    //
    // WHY THIS IS A gui_test AND NOT A UNIT TEST. The contract is not a property of any single
    // function: it is the ORDER in which the commit composes two of them — write the document, and
    // only if that succeeded push the changed presets into the cache. Extracting the decision into
    // a pure ShouldAdopt(bool write_succeeded, ...) is perfectly possible, and that is exactly the
    // problem: the unit test would then assert that write_succeeded == false yields no adoptions,
    // i.e. restate the `if` it was extracted from, while the side that can actually break — does
    // the production call site really write first, and does it really leave before the adopt
    // loop — moves out of view and reports as covered. So this drives the real Save button twice:
    // once where the write fails and once where it lands.
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "defaults_panel", "a_save_that_could_not_write_leaves_the_preset_cache_alone");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // Not the shared fixture: this case has to take the config source AWAY mid-session, which is
      // the one thing the fixture's guard is there to hold steady. The directory is still a fresh
      // one, and the harness baseline is restored by the inner guards' destructors.
      const auto dir = FreshOverlayDir("panel_save_fails");
      std::optional<std::string> before_bytes;
      {
        ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
        ResetTestState();
        ResetUserDefaultsChannels();

        json doc;
        doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
        IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
        gui::g_state = gui::MakeNewDocumentState();

        gui::OpenDefaultsPanel(gui::g_state, gui::DefaultsPanelSection::kPresets);
        ctx->Yield(4);
        ctx->ItemOpen("**/###preset_Column");
        ctx->Yield(2);
        ctx->ItemInputValue("**/###preset_std_column", 0.7f);
        ctx->Yield(3);
        before_bytes = ReadOverlayBytes(dir);
      }
      IM_CHECK(before_bytes.has_value());
      const auto before_cache = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(before_cache.has_value());
      IM_CHECK_EQ(*before_cache, 0.3f);

      // No writable directory at all — the same shape as a config directory that has become
      // read-only between opening the panel and pressing Save.
      {
        ScopedUserConfigSource disabled(gui::UserConfigSource::kDisabled);
        SaveDefaultsPanel(ctx);
        IM_CHECK_EQ(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn), before_cache);
        IM_CHECK_EQ(ReadOverlayBytes(dir), before_bytes);
      }

      // The same button, with somewhere to write again: the edit lands, in the file AND in the
      // cache. This is what makes the assertions above a claim about the FAILED write rather than
      // about a click that never reached the commit at all — and it is the half that turns red if
      // the adopt loop is moved ahead of the write instead of behind it.
      {
        ScopedUserConfigSource writable(gui::UserConfigSource::kExplicitDir, dir);
        SaveDefaultsPanel(ctx);
        IM_CHECK_EQ(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).value_or(-1.0f), 0.7f);
        IM_CHECK_EQ(ReadPresetStd(ReadOverlayFile(dir), "column").value_or(-1.0f), 0.7f);
        // By hand, because this case does not use ScopedPanel and therefore does not get its
        // teardown: the Column node opened above is ImGui window storage that outlives this case,
        // and the visual reference scenes downstream capture exactly one preset unfolded.
        ctx->ItemClose("**/###preset_Column");
        ctx->Yield(2);
        if (gui::g_state.defaults_panel_open) {
          ctx->ItemClick("**/###defaults_close");
          ctx->Yield(2);
        }
      }
    };
  }

  // ===============================================================================================
  // Across surfaces: the library is READ by the axis modal and WRITTEN only here.
  // ===============================================================================================

  {
    // Retune Column in the library, then press the Column button in a crystal's axis modal and
    // confirm the crystal got the tuned value AND is still classified as Column. This is the one
    // case that crosses both surfaces — each half passing on its own would not prove the button
    // reads what the panel wrote.
    //
    // The explicit Save matters: without it the case would assert that an edit the user never saved
    // reached the axis modal, which is the opposite of what the copy model promises.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "a_retuned_preset_reaches_the_axis_modal_button");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_library_to_modal");
      panel.OpenOn(gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(3);
      SaveDefaultsPanel(ctx);
      panel.Close();

      gui::EditRequest req{ gui::EditTarget::kAxis, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, gui::g_state);
      ctx->Yield(4);
      ctx->ItemClick("**/Column");
      ctx->Yield(2);
      // Committed rather than read out of the modal's buffer: the buffer is TU-private by design
      // (OK/Cancel atomicity), and "the value reached the document" is the stronger claim anyway.
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(3);

      const auto& edited = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
      IM_CHECK_EQ(edited.zenith.std, 0.3f);
      IM_CHECK_EQ(static_cast<int>(gui::ClassifyAxisPreset(edited.zenith, edited.azimuth, edited.roll)),
                  static_cast<int>(gui::AxisPreset::kColumn));
    };
  }

  {
    // The axis modal never writes back to the library. It once carried a row of "save this zenith
    // std as the <preset> default" buttons; the product answer is now that a preset is retuned
    // where it is listed, and this modal only reads.
    //
    // Asserted rather than left to "the code is gone": a widget id nobody looks for is exactly the
    // kind of thing that comes back by accident. Note what this must NOT be shortened to —
    // asserting only that the buttons are absent, without the sibling case above asserting the read
    // path still works, would pass just as well if the preset feature had been deleted outright.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "the_axis_modal_never_writes_back_to_the_library");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_modal_no_writeback");

      gui::EditRequest req{ gui::EditTarget::kAxis, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, gui::g_state);
      ctx->Yield(4);

      // The same starting point the read case used: Column, then a std the user tuned. If any
      // write-through survived, this is the edit it would carry into the library.
      ctx->ItemClick("**/Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Zenith/##Std_input", 0.3f);
      ctx->Yield(2);

      // Every preset that once had a button, not just the one an older case happened to click.
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_column"));
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_plate"));
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_lowitz"));

      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(3);

      // The crystal took the edit; the library did not. OK rather than Cancel on purpose: a
      // discarded edit could not have reached the file either way, so it would prove nothing.
      const auto& edited = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
      IM_CHECK_EQ(edited.zenith.std, 0.3f);
      IM_CHECK(!ReadPresetStd(ReadOverlayFile(panel.dir()), "column").has_value());
    };
  }

  {
    // The end of the whole feature, through the panel's own button: what Save wrote is what a NEW
    // document starts from, and an OPENED file still wins over it.
    //
    // The second half is the invariant that makes a .lmc portable — a file someone sends you must
    // render the same on your machine whatever you have saved as a personal default. The store
    // proves that rule off-screen; this proves the two commands a user actually presses are wired
    // to it, starting from a file this panel wrote rather than one the test hand-assembled.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel",
                                    "what_the_panel_saved_starts_new_documents_and_an_opened_file_still_wins");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedPanel panel(ctx, "panel_end_to_end");

      // A .lmc that states bg_alpha explicitly — written BEFORE any personal default exists, the
      // way a file someone sends you would be.
      const float kFileValue = 0.11f;
      gui::g_state.bg_alpha = kFileValue;
      const std::filesystem::path lmc_path = panel.dir() / "portability_probe.lmc";
      IM_CHECK(gui::SaveLmcFile(lmc_path, gui::g_state, gui::g_preview, /*save_texture=*/false));

      // Now save a DIFFERENT value as the personal default, through the panel's own button.
      gui::g_state.bg_alpha = 0.77f;
      panel.OpenOn(gui::DefaultsPanelSection::kSettings);
      SaveDefaultsPanel(ctx);
      panel.Close();
      IM_CHECK_EQ(ReadOverlayFile(panel.dir())["bg_alpha"].get<float>(), 0.77f);

      // New document: the personal default applies. Without this the assertion below would be
      // vacuous — a file value "winning" over a default that never arrived proves nothing.
      gui::DoNew();
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.77f);

      // Opening the file: the file's value wins.
      gui::DoOpen(lmc_path);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.bg_alpha, kFileValue);
    };
  }
}
