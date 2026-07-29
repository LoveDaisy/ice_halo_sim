// Defaults panel interaction tests — the UI half of the "save current settings as my defaults"
// feature (src/gui/defaults_panel.*).
//
// Every case here drives the real widgets through ImGuiTestContext and then asserts on the
// override FILE (or on a freshly built document), never on the panel's own internal state: the
// question these tests answer is "did clicking this actually change what a new document starts
// from", and a panel that updated only its own vector would satisfy any in-memory assertion.
//
// Coverage:
//   AC1  — the merged list renders EVERY candidate default, once each: one row, one checkbox
//   AC2  — what a checkbox says on open, over all three states a row can be in
//   AC3  — Reset all un-checks every row and writes nothing until Save
//   AC4  — the two filters are two different questions, each with a hit and a miss
//   AC7  — adoption round-trip: uncheck one row, save, and the unchecked key is NOT in the file
//   AC8  — un-checking a saved row / Reset all, asserted file-side, with the presets subtree kept
//   AC9  — invariant I1 through the full GUI path: an opened .lmc beats the personal default
//   AC10 — the search box narrows the list, and clearing it restores every row
//   plus the entry-point contract: which section an entry point opens expanded
//
// The panel's entry and exit controls add three more, all of them claims about what a user can
// SEE rather than about what the code can reach:
//   the top-bar Settings button is visible with nothing opened, and the Save menu no longer
//   hosts the entry; the title-bar X discards like Close AND actually closes; and the axis
//   modal reads the preset library without writing to it
//
// The copy model (the panel is a pure editor: edits go into an in-memory copy, Save writes it once,
// closing discards it) adds three more, and rewrites the timing half of several of the above:
//   copy AC1 — a mixed batch of edits, closed WITHOUT Save, leaves the file byte-identical
//   copy AC2 — Reset all in both directions: discarded on close, and committed on Save
//   copy AC3 — a preset edit closed without Save reaches neither the cache nor the file
//
// Where a case used to click and read the file, it now clicks, asserts the file has NOT moved,
// then saves and asserts it has. That is the assertion the old model could not make.
//
// Two cases from the two-section model were RETIRED here rather than migrated — see the block
// comment above the AC1 case for why neither is a claim the merged list can still make.

#include <algorithm>
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
#include "test_gui_shared.hpp"
#include "user_defaults_test_env.hpp"

namespace {

using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;
using lumice::test_user_defaults::ScopedUserConfigSource;
using nlohmann::json;

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

// Widget refs. The key path IS the widget id (see defaults_panel.cpp), so these are total
// functions of a row rather than an index that has to be kept in step with the row order.
std::string AdoptCheckboxRef(const std::string& key_path) {
  return "**/###adopt_" + key_path;
}

// The per-row source cell ("Mine" / "Factory"). Every row in the merged list has one, next to its
// checkbox — the two answer different questions (what the file holds NOW vs what Save would leave
// it holding), which is why both are addressable.
std::string SourceCellRef(const std::string& key_path) {
  return "**/###source_" + key_path;
}

// Leave the panel closed for whatever runs next in this single-process suite. Closing through the
// button (rather than by zeroing the flag) also keeps ImGui's popup stack unwound the same way a
// user would leave it.
void CloseDefaultsPanel(ImGuiTestContext* ctx) {
  if (gui::g_state.defaults_panel_open) {
    ctx->ItemClick("**/###defaults_close");
    ctx->Yield(2);
  }
}

void OpenPanelOn(ImGuiTestContext* ctx, gui::DefaultsPanelSection section) {
  gui::OpenDefaultsPanel(gui::g_state, section);
  ctx->Yield(4);
}

// Narrow the panel to (at most) the rows whose key contains `text`.
//
// Not a convenience: the test engine's wildcard lookup only finds items it can SEE ("clipped
// items are unaware of their labels" — imgui_te_context.cpp), and the merged list is 40+ rows in a
// fixed-height modal. Filtering first makes "is this key rendered" a question about the panel
// rather than about where the scroll happened to be, and it exercises the search box on every case
// instead of only in the one written for it.
void FilterTo(ImGuiTestContext* ctx, const char* text) {
  ctx->ItemInputValue("**/###defaults_search", text);
  ctx->Yield(3);
}

// Whether a row's checkbox is ticked, asked of the REAL widget rather than of the panel's internal
// set (which is TU-private, and which a panel could keep correct while rendering something else).
// The row must be on screen — narrow with FilterTo first, or ItemInfo reports a missing item.
bool RowIsChecked(ImGuiTestContext* ctx, const std::string& key_path) {
  return ctx->ItemIsChecked(AdoptCheckboxRef(key_path).c_str());
}

// Click a row's checkbox, first narrowing the list to it so the click cannot land on a row that
// happens to be scrolled into that position.
void ToggleRow(ImGuiTestContext* ctx, const std::string& key_path) {
  FilterTo(ctx, key_path.c_str());
  ctx->ItemClick(AdoptCheckboxRef(key_path).c_str());
  ctx->Yield(2);
}

// Read presets.axis.<name>.zenith_std, reporting an absent or malformed key as nullopt rather
// than throwing. A regression that DROPS the key is precisely what these cases exist to catch, and
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

std::vector<gui::DefaultDiffRow> CurrentRows() {
  return gui::BuildDefaultDiffRows(gui::g_state);
}

// The override file as RAW BYTES, or nullopt when it does not exist.
//
// Byte-level rather than parsed: "the file did not change" is the claim, and comparing two parsed
// documents would call a rewrite with reordered keys or different spacing equal. A panel that
// rewrote the file on every click while preserving its meaning would still be the panel this
// change exists to remove.
std::optional<std::string> ReadOverlayBytes(const std::filesystem::path& dir) {
  std::ifstream in(dir / gui::kUserDefaultsFileName, std::ios::binary);
  if (!in.is_open()) {
    return std::nullopt;
  }
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void SaveDefaultsPanel(ImGuiTestContext* ctx) {
  ctx->ItemClick("**/###defaults_save");
  ctx->Yield(3);
}

// Scroll the settings list by the user's own gesture — point at a row of it and turn the wheel —
// rather than by writing a scroll value into a window this test would first have to name. WHICH
// region receives the wheel is half of what is under test: pointing at the list must scroll the
// list and nothing else, which is exactly the claim a SetScrollY() would assume rather than check.
//
// The mouse is placed once and left there: after the first turn the row it was addressed by has
// scrolled away, and re-addressing it every turn would fail on an item that no longer exists.
//
// Eight turns of ten lines reaches the far end of a 40-row list several times over — the claims
// below are about the bottom of the scroll range, not about a nudge.
void ScrollSettingsListDown(ImGuiTestContext* ctx, const std::string& hover_ref) {
  ctx->MouseMove(hover_ref.c_str());
  for (int i = 0; i < 8; ++i) {
    ctx->MouseWheelY(-10.0f);
    ctx->Yield(2);
  }
  ctx->Yield(2);
}

// The settings table object, which is what the geometric claims about its header have to go
// through.
//
// The id is the one BeginTable computed: the table is created directly in the panel window with
// nothing pushed on the ID stack, so the window's own GetID reproduces it. Should the table ever be
// wrapped in a child again — the shared scrolling child this task removed — the seed changes and
// this returns null. That null IS the regression signal, not a broken helper: measured by putting
// the old shared child back, which turns both scroll cases below red here while the folding case
// stays green.
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
// geometry, under the id) but never calls the test engine's label hook — IMGUI_TEST_ENGINE_ITEM_INFO
// appears throughout imgui_widgets.cpp and not once in imgui_tables.cpp — and a "**/" path is
// resolved BY that label. So the id is the entire gap, and TableGetHeaderID closes it by
// reproducing the PushID(column_n) + GetID(name) that TableHeadersRow used.
//
// imgui_internal.h is an anti-pattern in this suite generally (test_gui_interaction.cpp carries the
// same caveat for its z-order assertions) and unavoidable for the same reason here: "did this stay
// where it was drawn" is not a question the public API answers.
ImGuiTestItemInfo SettingsHeaderInfo(ImGuiTestContext* ctx, ImGuiTable* table, const char* column) {
  return ctx->ItemInfo(TableGetHeaderID(table, column));
}

// Put the settings list back at its top.
//
// Needed because ImGui keeps a scroll position per window ID, and the table's inner window is not
// torn down when the panel closes: in this single-process suite a case that scrolls the list to the
// bottom hands the NEXT case a list already at the bottom, where its own "the top row is on screen"
// premise is false. Rewinding here rather than at the end of whoever scrolled makes each case
// depend on nothing but itself.
void RewindSettingsList(ImGuiTestContext* ctx, ImGuiTable* table) {
  ctx->ScrollToTop(table->InnerWindow->ID);
  ctx->Yield(2);
}

// The three shapes a row's value-cell widget id can take. A slider cell is TWO items (the slider
// and its input box, ids built by PrepareSliderLayout); a checkbox / colour / combo cell is one.
// Kept as a set because the negative claim ("this row has no editor") has to cover all of them —
// checking only the one the key would have used had it been registered proves nothing.
std::string ValueInputRef(const std::string& key_path) {
  return "**/##value_" + key_path + "_input";
}

std::string ValueWidgetRef(const std::string& key_path) {
  return "**/##value_" + key_path;
}

// The id of a value-cell widget, computed rather than searched for by label.
//
// Forced, not stylistic: ImGui::BeginCombo is the one control shape here that never calls
// IMGUI_TEST_ENGINE_ITEM_INFO, so the engine's wildcard LABEL search cannot see a combo cell at all
// (test_gui_interaction.cpp carries the same note for the main UI's lens combo). Every item is
// recorded by ItemAdd under its id regardless, so going through the id covers all five control
// shapes uniformly — which is what a coverage claim over the whole row set needs.
// The seed is the TABLE's id, not its inner window's: imgui_tables.cpp pushes an override id
// (PushOverrideID(table->ID)) for the whole body, so every cell widget hashes against that rather
// than against whatever window it happens to be drawn in.
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

// The Origin cell, addressed BY THE VALUE IT SHOULD BE SHOWING (see the "##origin_" comment in
// defaults_panel.cpp): the item's id hashes the whole label, so this exists only if the cell drew
// exactly `expected`.
std::string OriginCellRef(const std::string& key_path, const std::string& expected_text) {
  return "**/" + expected_text + "##origin_" + key_path;
}

// Whether a row of the settings list is actually DRAWN right now — not merely submitted.
//
// Deliberately not ItemExists, and the difference is what makes the scroll cases below non-vacuous:
// a Checkbox registers its label with the test engine even on the branch where ItemAdd CLIPPED it
// (imgui_widgets.cpp calls IMGUI_TEST_ENGINE_ITEM_INFO before returning false), so ItemExists stays
// true for a row that has scrolled out of view and would have reported "nothing scrolled" for a
// list that scrolled all the way to its end. The clipped rectangle is the part that collapses to
// nothing when the row leaves the visible band.
bool RowIsOnScreen(ImGuiTestContext* ctx, const std::string& row_ref) {
  return ctx->ItemInfo(row_ref.c_str(), ImGuiTestOpFlags_NoError).RectClipped.GetHeight() > 0.0f;
}

}  // namespace

void RegisterDefaultsPanelTests(ImGuiTestEngine* engine) {
  // ================================================================================
  // AC1 — the merged list is complete: every row, once, with the same two controls
  //
  // This is the successor to 405.4's ac6_sections_are_mutually_exclusive_on_screen, which asserted
  // "a checkbox XOR a source cell" per row. That XOR was a statement about the two-section layout,
  // not about the data: §2 rows had a checkbox and §3 rows had a source cell, so exactly one of
  // the two probes could find any given key. In the merged list EVERY row has both, so the old
  // assertion would fail — correctly, and permanently. What it was actually protecting (no key
  // missing, no key rendered twice) is what this case asserts instead, in the shape the merged
  // list can still make it: both controls present, on every row, exactly once.
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac1_merged_list_renders_every_row_once");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_ac1");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // Edited AND saved keys both present, so the sweep below covers rows in every state rather
      // than a list that happens to be uniform.
      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();
      gui::g_state.renderer.fov = 95.0f;

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

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
        // One key at a time (see FilterTo): nothing is off-screen, so "not found" means "not
        // rendered" rather than "scrolled past".
        FilterTo(ctx, row.key_path.c_str());
        IM_CHECK(ctx->ItemExists(AdoptCheckboxRef(row.key_path).c_str()));
        IM_CHECK(ctx->ItemExists(SourceCellRef(row.key_path).c_str()));
        if (gui::RowNeedsAdoption(row)) {
          ++changed_seen;
        } else {
          ++unchanged_seen;
        }
      }
      // Non-vacuous in both directions: the sweep really did cover rows that differ from the
      // effective default and rows that do not, which used to be the two sections.
      IM_CHECK(changed_seen > 0);
      IM_CHECK(unchanged_seen > 0);

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // Discoverability, not reachability. The panel spent its first iteration behind two items in
    // the Save dropdown, where it was reachable by every automated measure and found by nobody:
    // a user looking for their settings does not open a Save menu. So the claim under test is
    // specifically "visible from the default state with NOTHING opened" — no menu clicked, no
    // window toggled, no scrolling — which is why this case yields from ResetTestState straight
    // into ItemExists and does not touch a single control first.
    //
    // A test that opened the menu and found the entry there would pass on the version that failed
    // the owner. That is the trap this case exists to avoid, so keep the "no interaction before
    // the assertion" shape if it is ever edited.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "settings_entry_is_visible_without_opening_anything");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_topbar_entry");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();
      ctx->Yield(2);

      const char* kSettingsRef = "##TopBar/" ICON_FA_GEAR " Settings";
      IM_CHECK(ctx->ItemExists(kSettingsRef));

      // A simulation running must not take the entry away: settings are not a document operation,
      // and an entry that vanishes while the user is watching a render is one they cannot find at
      // the moment they think to look for it.
      gui::g_state.run_intent = gui::RunIntent::kRunning;
      ctx->Yield(2);
      IM_CHECK(ctx->ItemExists(kSettingsRef));
      auto info = ctx->ItemInfo(kSettingsRef);
      IM_CHECK((info.ItemFlags & ImGuiItemFlags_Disabled) == 0);
      gui::g_state.run_intent = gui::RunIntent::kNone;
      ctx->Yield(2);

      // Not a decoration: it opens the panel, on the settings section.
      IM_CHECK(!gui::g_state.defaults_panel_open);
      ctx->ItemClick(kSettingsRef);
      ctx->Yield(4);
      IM_CHECK(gui::g_state.defaults_panel_open);
      IM_CHECK(ctx->ItemExists("**/###defaults_save"));
      IM_CHECK(ctx->ItemExists("**/###defaults_settings"));

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // The old home of the entry, asserted empty. Deleting the two Save-menu items is the half of
    // "promote the entry" that a passing top-bar assertion says nothing about, and leaving them
    // would quietly restore the state the promotion was meant to end: two ways in, one of which
    // teaches the user to look in the wrong place.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "save_menu_no_longer_hosts_the_entry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("##TopBar/Save");
      ctx->Yield(2);
      IM_CHECK(!ctx->ItemExists("**/Save Current as Defaults..."));
      IM_CHECK(!ctx->ItemExists("**/Edit My Presets..."));
      // The menu itself still works — this is a removal, not a broken popup.
      IM_CHECK(ctx->ItemExists("**/Config JSON..."));

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
    };
  }

  {
    // The "entry point decides the initial section" mechanism. Asserted through an observable
    // consequence — whether the settings list's rows are on screen — rather than left as a manual
    // check that only breaks once a second entry point tries to use it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "entry_point_decides_the_expanded_section");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_entry");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::g_state.bg_alpha = 0.42f;
      const auto rows = CurrentRows();
      const auto probe = std::find_if(rows.begin(), rows.end(),
                                      [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
      IM_CHECK(probe != rows.end());

      // Opened on §2: the settings list is expanded, so its rows are rendered.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef(probe->key_path).c_str()));
      CloseDefaultsPanel(ctx);

      // Opened on §1: the SAME row is no longer rendered, i.e. the section choice actually
      // travelled from the entry point into the panel.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef(probe->key_path).c_str()));
      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // AC7 — adoption round-trip
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac7_adoption_round_trip_respects_unchecked_rows");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_ac7");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::g_state.bg_alpha = 0.42f;
      gui::g_state.renderer.fov = 95.0f;

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      // Both edited keys open checked (their current value differs from the effective default);
      // uncheck exactly one of them, leaving the other alone.
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      ctx->ItemClick(AdoptCheckboxRef("bg_alpha").c_str());
      ctx->Yield(2);
      ctx->ItemClick("**/###defaults_save");
      ctx->Yield(3);

      const json saved = ReadOverlayFile(dir);
      IM_CHECK(saved.contains("renderer"));
      IM_CHECK_EQ(saved["renderer"]["fov"].get<float>(), 95.0f);
      // The unchecked row must be absent from the file, not merely absent from the panel.
      IM_CHECK(!saved.contains("bg_alpha"));

      // ...and the file is what a new document actually reads.
      const gui::GuiState fresh = gui::MakeNewDocumentState();
      IM_CHECK_EQ(fresh.renderer.fov, 95.0f);
      IM_CHECK_EQ(fresh.bg_alpha, gui::GuiState{}.bg_alpha);

      // After the save the adopted row no longer differs from the effective default, and the panel
      // now attributes it to the user.
      const auto rows = CurrentRows();
      const auto fov = std::find_if(rows.begin(), rows.end(),
                                    [](const gui::DefaultDiffRow& row) { return row.key_path == "renderer.fov"; });
      IM_CHECK(fov != rows.end());
      IM_CHECK(!gui::RowNeedsAdoption(*fov));
      IM_CHECK(fov->has_saved_override);

      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // AC8 — un-checking a saved row / Reset all, file-side, presets preserved
  //
  // 405.4 clicked a per-row Revert button here. That button is gone: un-checking the row expresses
  // the same intent through the one control the merged list has, and Save is the one place it
  // lands. The file-side claims below are unchanged — they are what the case was ever about.
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac8_uncheck_and_reset_all_preserve_presets");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_ac8");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // Two personal defaults plus a preset-library subtree this panel does not own. The subtree
      // is the point: a wholesale rewrite of the document would delete it silently, and 405.5's
      // UI does not exist yet to notice.
      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["renderer"]["fov"] = 95.0f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      // Start from a document that already carries those defaults, so both keys open CHECKED
      // because they are already in the defaults (not because their value differs).
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.42f);

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));

      // The click must NOT reach the file, and it must still be visible on screen. (405.4 asserted
      // the file the instant its Revert button was clicked; 405.6 split the claim in two, and the
      // split is what survives the control changing.)
      const auto before_uncheck = ReadOverlayBytes(dir);
      IM_CHECK(before_uncheck.has_value());
      ctx->ItemClick(AdoptCheckboxRef("bg_alpha").c_str());
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(dir), before_uncheck);
      IM_CHECK(!RowIsChecked(ctx, "bg_alpha"));
      // The Source cell still reads "Mine": it reports the file, which has not changed yet. The
      // checkbox reports the intent. Two questions, two answers, both on screen.
      IM_CHECK(ctx->ItemExists(SourceCellRef("bg_alpha").c_str()));

      SaveDefaultsPanel(ctx);

      json saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(saved.contains("renderer"));
      IM_CHECK(saved.contains("presets"));
      // The reverted key is back to factory for a new document — the file change is the whole
      // point, not a panel-local undo.
      IM_CHECK_EQ(gui::MakeNewDocumentState().bg_alpha, gui::GuiState{}.bg_alpha);

      // Same shape for Reset all: click, file unmoved, save, file emptied of GuiState keys.
      const auto before_reset = ReadOverlayBytes(dir);
      IM_CHECK(before_reset.has_value());
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(dir), before_reset);

      SaveDefaultsPanel(ctx);

      saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("renderer"));
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(saved.contains("presets"));
      IM_CHECK_EQ(saved["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
      const gui::GuiState after_reset = gui::MakeNewDocumentState();
      IM_CHECK_EQ(after_reset.renderer.fov, gui::RenderConfig{}.fov);

      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // AC9 — invariant I1 through the full GUI path
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac9_opened_file_beats_the_personal_default");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_ac9");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // A .lmc that states bg_alpha explicitly — written BEFORE any personal default exists, the
      // way a file someone sends you would be.
      const float kFileValue = 0.11f;
      gui::g_state.bg_alpha = kFileValue;
      const std::filesystem::path lmc_path = dir / "i1_probe.lmc";
      IM_CHECK(gui::SaveLmcFile(lmc_path, gui::g_state, gui::g_preview, /*save_texture=*/false));

      // Now save a DIFFERENT value as the personal default, through the panel.
      gui::g_state.bg_alpha = 0.77f;
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      ctx->ItemClick("**/###defaults_save");
      ctx->Yield(3);
      CloseDefaultsPanel(ctx);
      IM_CHECK_EQ(ReadOverlayFile(dir)["bg_alpha"].get<float>(), 0.77f);

      // New document: the personal default applies (otherwise the assertion below is vacuous).
      gui::DoNew();
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.77f);

      // Opening the file: the file's value wins. This is I1 — a .lmc must render the same on
      // every machine, whatever its owner saved as a personal default.
      gui::DoOpen(lmc_path);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.bg_alpha, kFileValue);
    };
  }

  // ================================================================================
  // AC10 (functional half) — the search box over the merged list
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac10_search_filters_the_merged_list");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_ac10");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::g_state.renderer.fov = 95.0f;
      gui::g_state.bg_alpha = 0.42f;

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));

      ctx->ItemInputValue("**/###defaults_search", "renderer");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));

      // Clearing restores the full set — a filter that could not be undone would strand the user
      // in a partial view of their own settings.
      ctx->ItemInputValue("**/###defaults_search", "");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));

      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // §1 — the preset library (405.5). Driven through the real widgets, asserted on the override
  // FILE: a panel that only updated its own buffers would satisfy any in-memory check.
  // ================================================================================

  {
    // AC3 — the clamp, both states, through the input box a user actually types into. 25 is above
    // Column's (0, 10) and must be adjusted with the warning cell appearing; 0.3 is inside it and
    // must be stored verbatim with NO warning. A one-sided test would be satisfied by a panel
    // that clamped everything.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "preset_std_edit_clamps_and_warns_both_states");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_preset_clamp");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(3);

      // MIGRATED for the copy model: 405.5 read the file straight after each blur, because the
      // blur wrote it. A §1 edit is now an edit of the panel's copy, so each state is asserted the
      // same way every other edit in this panel is — save, THEN read. The warning-cell assertions
      // are untouched: they are pure UI state and never depended on the file.
      ctx->ItemInputValue("**/###preset_std_column", 25.0f);
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/###preset_warning_column"));
      SaveDefaultsPanel(ctx);
      {
        const auto stored = ReadPresetStd(ReadOverlayFile(dir), "column");
        IM_CHECK(stored.has_value());
        IM_CHECK(*stored < gui::kColumnPlateParryZenithStdUpperBound);
        IM_CHECK(*stored > 0.0f);
        // The clamp is reported once, when the panel adjusts the value — not again on the Save
        // that commits it. A second notice would tell the user something new had gone wrong.
        IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
      }

      // In domain: the value survives exactly, and the warning from the previous edit is gone —
      // a warning that outlived the value that caused it would point at a number that is no
      // longer there.
      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists("**/###preset_warning_column"));
      SaveDefaultsPanel(ctx);
      {
        const auto stored = ReadPresetStd(ReadOverlayFile(dir), "column");
        IM_CHECK(stored.has_value());
        IM_CHECK_EQ(*stored, 0.3f);
      }

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC5 through the UI — Restore to factory removes the key and leaves the neighbouring preset
    // alone. The store-side case (test_gui_user_defaults.cpp) proves the function; this proves
    // the button is wired to it and to the right preset.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "preset_restore_button_is_wired_per_preset");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_preset_restore");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(2);
      ctx->ItemOpen("**/###preset_Plate");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_plate", 0.5f);
      ctx->Yield(2);

      // MIGRATED for the copy model: the two edits above and the restore below are all copy edits
      // now, so the file is only consulted after a Save. The restore click is additionally checked
      // for NOT reaching the file, which is the property this task adds.
      SaveDefaultsPanel(ctx);
      const auto before_restore = ReadOverlayBytes(dir);
      IM_CHECK(before_restore.has_value());

      ctx->ItemClick("**/###preset_restore_column");
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(dir), before_restore);
      SaveDefaultsPanel(ctx);

      const json saved = ReadOverlayFile(dir);
      IM_CHECK(!ReadPresetStd(saved, "column").has_value());
      const auto survivor = ReadPresetStd(saved, "plate");
      IM_CHECK(survivor.has_value());
      IM_CHECK_EQ(*survivor, 0.5f);

      // The input box now reads the factory value back, not the number the user last typed.
      IM_CHECK_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std,
                  gui::AxisPresetEntryFor(gui::AxisPreset::kColumn).zenith.std);

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC7 — Random offers no input that would be ignored. A structural assertion on the widget
    // tree rather than a look at the picture: the std input and the restore button exist for
    // every adjustable preset and for none of the others, derived from the same table the panel
    // renders from, so this cannot drift into checking a stale list of four names.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "preset_without_adjustable_face_offers_no_input");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_preset_random");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);

      int adjustable_seen = 0;
      int fixed_seen = 0;
      for (const auto& entry : gui::kAxisPresets) {
        // Custom is the classifier's "none of the above", not a library entry.
        if (entry.id == gui::AxisPreset::kCustom) {
          continue;
        }
        const std::string node_ref = std::string("**/###preset_") + gui::AxisPresetLabel(entry.id);
        IM_CHECK(ctx->ItemExists(node_ref.c_str()));
        ctx->ItemOpen(node_ref.c_str());
        ctx->Yield(2);

        if (entry.has_adjustable_zenith_std) {
          ++adjustable_seen;
          const std::string std_ref = std::string("**/###preset_std_") + entry.override_json_name;
          const std::string restore_ref = std::string("**/###preset_restore_") + entry.override_json_name;
          IM_CHECK(ctx->ItemExists(std_ref.c_str()));
          IM_CHECK(ctx->ItemExists(restore_ref.c_str()));
        } else {
          ++fixed_seen;
          // Nothing keyed on a json name can exist for it — it has none. Probing the two ids an
          // adjustable preset would carry, spelled from the label instead, since that is the only
          // name this preset has.
          const std::string std_ref = std::string("**/###preset_std_") + gui::AxisPresetLabel(entry.id);
          IM_CHECK(!ctx->ItemExists(std_ref.c_str()));
        }
        ctx->ItemClose(node_ref.c_str());
        ctx->Yield(1);
      }
      IM_CHECK_EQ(adjustable_seen, 4);  // Column / Plate / Parry / Lowitz
      IM_CHECK_EQ(fixed_seen, 1);       // Random

      // Nothing was written by merely looking at the library.
      IM_CHECK(ReadOverlayFile(dir).empty());

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC1/AC2 through the two real surfaces, end to end: retune Column in the library, then press
    // the Column button in the axis modal and confirm the crystal got the tuned value AND is
    // still classified as Column. This is the one case that crosses both halves of the feature —
    // each half passing on its own would not prove the button reads what the panel wrote.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "library_edit_reaches_the_axis_modal_button");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_preset_to_modal");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.3f);
      ctx->Yield(3);
      // MIGRATED for the copy model: an explicit Save, where 405.5 relied on the blur itself
      // committing. Without it this case would now assert that an edit the user never saved
      // reached the axis modal — the opposite of what the copy model promises (and what the
      // discard case below it asserts).
      SaveDefaultsPanel(ctx);
      CloseDefaultsPanel(ctx);

      // Now the other surface: the axis tab of a crystal's edit modal.
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
    // The axis modal no longer writes to the preset library. It once carried a row of "save this
    // zenith std as the <preset> default" buttons; the product answer is now that a preset is
    // retuned where it is listed, and this modal only reads.
    //
    // Asserted rather than left to "the code is gone": a widget id nobody looks for is exactly the
    // kind of thing that comes back by accident. The positive half of the same contract — that
    // editing the library still reaches this modal's buttons — is
    // library_edit_reaches_the_axis_modal_button above; the two together say "read yes, write no",
    // which is the whole claim. Note what this case must NOT be shortened to: asserting only that
    // the button is absent, without its sibling asserting the read path still works, would pass
    // just as well if the preset feature had been deleted outright.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "axis_modal_does_not_write_to_the_library");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_no_gesture");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::EditRequest req{ gui::EditTarget::kAxis, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, gui::g_state);
      ctx->Yield(4);

      // Same starting point the gesture case used: Column, then a std the user tuned. If any
      // write-through survived, this is the edit it would carry into the library.
      ctx->ItemClick("**/Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Zenith/##Std_input", 0.3f);
      ctx->Yield(2);

      // The controls themselves are gone — every preset that once had a button, not just the one
      // the old case happened to click.
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_column"));
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_plate"));
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_lowitz"));

      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(3);

      // The crystal took the edit; the library did not. OK rather than Cancel on purpose: a
      // discarded edit could not have reached the file either way, so it would prove nothing.
      const auto& edited = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
      IM_CHECK_EQ(edited.zenith.std, 0.3f);
      IM_CHECK(!ReadPresetStd(ReadOverlayFile(dir), "column").has_value());
    };
  }

  // ================================================================================
  // The copy model — the panel is a pure editor with one commit point
  // ================================================================================

  {
    // copy AC1 — a mixed batch of edits, closed WITHOUT Save, leaves the file BYTE-identical.
    //
    // Every kind of edit the panel offers is exercised in one session, because the claim is about
    // the panel and not about one button: if any single path still wrote through, this fails.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "copy_ac1_closing_without_save_writes_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_copy_ac1");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // A file with something in every half, so each edit below has something real to change.
      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["renderer"]["fov"] = 95.0f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      doc["presets"]["axis"]["plate"]["zenith_std"] = 0.6f;  // the one the restore below drops
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();

      const auto before = ReadOverlayBytes(dir);
      IM_CHECK(before.has_value());

      // An unsaved GuiState change, so the list has a row that opens checked because its value
      // differs, on top of the ones that open checked because they are already saved.
      gui::g_state.sun.altitude = 33.0f;

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.5f);  // §1 edit
      ctx->Yield(2);
      ctx->ItemClose("**/###preset_Column");
      ctx->ItemOpen("**/###preset_Plate");
      ctx->Yield(2);
      ctx->ItemClick("**/###preset_restore_plate");  // §1 restore
      ctx->Yield(2);
      ctx->ItemClose("**/###preset_Plate");
      ctx->Yield(2);

      ctx->ItemOpen("**/###defaults_settings");
      ctx->Yield(2);
      ToggleRow(ctx, "bg_alpha");  // un-check a key that IS in the defaults
      FilterTo(ctx, "");

      ctx->ItemClick("**/###defaults_reset_all");  // the whole GuiState half
      ctx->Yield(2);

      CloseDefaultsPanel(ctx);

      // Byte-for-byte, not json-equal: a rewrite that preserved meaning would still be a write.
      IM_CHECK_EQ(ReadOverlayBytes(dir), before);
      // ...and nothing leaked into the in-memory halves either — both directions of the §1 edit
      // (a changed value, a dropped override) and the GuiState half.
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
    // copy AC2 — Reset all in BOTH directions. One discard, one commit, in one case so that the
    // pair cannot drift apart; the second half re-opens the panel, because the first Close is
    // precisely what threw the first copy away.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "copy_ac2_reset_all_discarded_then_committed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_copy_ac2");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["renderer"]["fov"] = 95.0f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();

      const auto before = ReadOverlayBytes(dir);
      IM_CHECK(before.has_value());

      // (a) discarded.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);
      // The panel shows the reset immediately — this is the feedback the old write-through model
      // owed the user and could not give: the row is un-checked though the file still holds the
      // value, and its Source still says so.
      IM_CHECK(!RowIsChecked(ctx, "renderer.fov"));
      IM_CHECK(ctx->ItemExists(SourceCellRef("renderer.fov").c_str()));
      CloseDefaultsPanel(ctx);
      IM_CHECK_EQ(ReadOverlayBytes(dir), before);

      // (b) committed. Save after Reset all must write an EMPTY override set — not re-adopt the
      // rows that were checked when the panel opened, which would put a full set of defaults
      // straight back and make the button a no-op the user cannot see through.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(2);
      SaveDefaultsPanel(ctx);
      CloseDefaultsPanel(ctx);

      const json saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(!saved.contains("renderer"));
      // The preset library is a sibling namespace this button does not reach.
      IM_CHECK_EQ(ReadPresetStd(saved, "column").value_or(-1.0f), 0.3f);
      IM_CHECK_EQ(gui::MakeNewDocumentState().bg_alpha, gui::GuiState{}.bg_alpha);
    };
  }

  {
    // copy AC3 — a preset edit closed without Save reaches NEITHER the process-wide cache nor the
    // file. The cache half matters on its own: it is what the axis modal's preset buttons read, so
    // a leak there would hand the user an uncommitted value through a completely different surface.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "copy_ac3_preset_edit_discarded_on_close");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_copy_ac3");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();

      const auto before_bytes = ReadOverlayBytes(dir);
      IM_CHECK(before_bytes.has_value());
      const auto before_cache = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(before_cache.has_value());
      IM_CHECK_EQ(*before_cache, 0.3f);

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/###preset_std_column", 0.7f);
      ctx->Yield(3);
      // The panel itself shows the edit — it is uncommitted, not ignored.
      IM_CHECK_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std, 0.3f);
      CloseDefaultsPanel(ctx);

      IM_CHECK_EQ(ReadOverlayBytes(dir), before_bytes);
      IM_CHECK_EQ(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn), before_cache);

      // Restore-to-factory is the other direction of the same claim: it must not silently drop a
      // stored override either.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      ctx->ItemOpen("**/###preset_Column");
      ctx->Yield(2);
      ctx->ItemClick("**/###preset_restore_column");
      ctx->Yield(3);
      CloseDefaultsPanel(ctx);

      IM_CHECK_EQ(ReadOverlayBytes(dir), before_bytes);
      IM_CHECK_EQ(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn), before_cache);
    };
  }

  // RETIRED: copy_ac4_source_follows_copy_section_follows_snapshot.
  //
  // It asserted that a row's Source cell and Revert button tracked the WORKING COPY while its
  // §2/§3 membership tracked the opening SNAPSHOT — a distinction that only existed because the
  // panel had two mechanisms whose feedback had to be told apart. Both halves are gone: there are
  // no sections to move between, and there is no per-row operation that edits the copy before
  // Save, so Source is answered from the snapshot for the whole session (RefreshRows no longer
  // re-derives it from the copy at all). The claim underneath it — an uncommitted intent must be
  // visible on screen without touching the file — is now made by ac8 and copy AC2, both of which
  // assert the checkbox flips while the file bytes do not.

  {
    // Esc is the OTHER way out of a BeginPopupModal, and the one no button owns. AC1 is a claim
    // about closing the panel, not about one particular control, so the exit path that has no code
    // of its own is the one worth pinning: it must not write either.
    //
    // What this does NOT claim is that Esc is a well-behaved Close. The panel re-opens on the next
    // frame (state.defaults_panel_open is still set, so RenderDefaultsPanel calls OpenPopup again)
    // and the working copy survives with it — pre-existing behavior, unchanged by the copy model,
    // and the entry/exit controls are 408.5's subject. The assertion below is deliberately about
    // the file, which is what this task is responsible for.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "copy_escape_exit_writes_nothing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_copy_escape");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();
      const auto before = ReadOverlayBytes(dir);
      IM_CHECK(before.has_value());

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(2);
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(4);

      IM_CHECK_EQ(ReadOverlayBytes(dir), before);
      // The measured state after Esc, asserted so this case cannot pass vacuously (a file is
      // trivially unwritten if Esc did nothing at all) and so the pre-existing behavior is on
      // record for 408.5, which owns the exit controls: the flag is still set and the panel is
      // back on screen, i.e. Esc does not currently close this panel.
      IM_CHECK(gui::g_state.defaults_panel_open);
      IM_CHECK(ctx->ItemExists("**/###defaults_save"));

      // Leave the panel closed for whatever runs next in this single-process suite, whichever
      // state Esc left it in.
      CloseDefaultsPanel(ctx);
      gui::g_state.defaults_panel_open = false;
      ctx->Yield(2);
    };
  }

  {
    // The title-bar X: the third way out, and the one that has to prove BOTH halves. "Writes
    // nothing" alone would be satisfied by a panel that never closes (Esc satisfies it that way,
    // and the case above says so), so the file assertion is paired here with two that say the
    // panel is actually gone — the flag cleared AND the widgets off screen. They are not the same
    // claim: ImGui closes the popup itself on the frame the X is pressed, so a version that forgot
    // to clear defaults_panel_open would look closed for exactly one frame and be re-opened by the
    // OpenPopup guard on the next. Yielding several frames before asking is what makes that
    // failure visible rather than a coin flip.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "title_x_discards_and_actually_closes");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_title_x");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();
      const auto before = ReadOverlayBytes(dir);
      IM_CHECK(before.has_value());

      // Real pending edits in both halves, so "nothing was written" is a statement about a session
      // that had something to write, not about an untouched panel.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
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

      // Discard, same as the Close button: the working copy was never committed.
      IM_CHECK_EQ(ReadOverlayBytes(dir), before);
      // ...and unlike Esc, the panel stays closed.
      IM_CHECK(!gui::g_state.defaults_panel_open);
      IM_CHECK(!ctx->ItemExists("**/###defaults_save"));

      // Re-opening starts from the file, not from the discarded copy — the edits above are gone
      // rather than merely unwritten. Without this, "discard" would be indistinguishable from
      // "deferred": a copy that survived the X would be committed by the next session's Save, and
      // the file assertion above would still have passed at the moment it was made.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      SaveDefaultsPanel(ctx);
      CloseDefaultsPanel(ctx);
      const auto after = ReadOverlayFile(dir);
      IM_CHECK_EQ(ReadPresetStd(after, "column"), std::optional<float>(0.3f));
      IM_CHECK(after.contains("bg_alpha"));
    };
  }

  {
    // The entry point contract for §1, matching the one already asserted for the settings list:
    // kPresets must open the panel with the preset section expanded, not merely open it. No menu
    // item points here any more — the top-bar Settings button opens on kSettings and the preset
    // library is a section inside — but the section argument is still the panel's API, and
    // OpenDefaultsPanel is what this asserts about.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "presets_entry_point_opens_that_section");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_preset_entry");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      // A preset row is only findable when §1 is expanded (the engine cannot see clipped items).
      IM_CHECK(ctx->ItemExists("**/###preset_Column"));
      CloseDefaultsPanel(ctx);

      // ...and the OTHER entry point leaves §1 collapsed, so "which section opens" is a real
      // choice rather than "everything is always open".
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
      CloseDefaultsPanel(ctx);
    };
  }

  {
    // Guards OpenDefaultsPanel's explicit is_object() normalization of g_snapshot_doc/g_copy_doc.
    // ReadOverlayJsonIfPresent (what ReadActiveOverlayDoc calls) already turns a hand-edited or
    // interrupted-write file whose top level is not an object into json::object() itself, so this
    // scenario cannot currently reach a non-object copy through any path this repo ships — but the
    // panel's normalization should not depend on that being true two calls away forever, so this
    // pins the panel's OWN guarantee rather than the upstream implementation detail it happens to
    // ride on today. This is the ordinary Save path with no interaction beyond Open/Save.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "copy_open_normalizes_a_non_object_override_file");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_copy_non_object");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      IM_CHECK(gui::WriteUserDefaultsFile(dir, json::array()));

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      SaveDefaultsPanel(ctx);
      CloseDefaultsPanel(ctx);

      const json saved = ReadOverlayFile(dir);
      IM_CHECK(saved.is_object());
    };
  }

  // ================================================================================
  // AC2 — what the checkbox says when the panel opens, over all three states
  // ================================================================================
  {
    // The checkbox means "this key is in my defaults", and its opening value is the OR of the two
    // conditions that can put it there. All three reachable states are constructed in one document
    // so the answers cannot be right for one reason and wrong for another:
    //   (1) changed in the GUI, not saved   -> checked (saving now would add it)
    //   (2) already saved, value unchanged  -> checked (saving now would keep it)
    //   (3) neither                         -> unchecked
    // (1) and (2) are the two halves of the OR, and (3) is the state that makes the OR falsifiable.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac2_checkbox_opens_on_in_my_defaults");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_ac2_states");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // State (2): saved, and the document starts from that saved value, so it does NOT also
      // qualify through "differs from the effective default" — otherwise the case would pass with
      // the panel looking at only one of the two conditions.
      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
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

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));
      FilterTo(ctx, "bg_show");
      IM_CHECK(!RowIsChecked(ctx, "bg_show"));

      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // AC3 — Reset all clears every checkbox, and writes nothing until Save
  // ================================================================================
  {
    // The whole sweep, not a spot check: "all" is the claim, and a Reset that cleared only the
    // rows it could see (or only the ones that were checked for one of the two reasons) would
    // satisfy any single-row assertion.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac3_reset_all_unchecks_every_row_without_writing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_ac3_reset");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["bg_alpha"] = 0.42f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();
      // Checked for the OTHER reason too, so the sweep covers both halves of the OR.
      gui::g_state.sun.altitude = 33.0f;

      const auto before = ReadOverlayBytes(dir);
      IM_CHECK(before.has_value());

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      // Non-vacuous: something really was checked before the click.
      FilterTo(ctx, "bg_alpha");
      IM_CHECK(RowIsChecked(ctx, "bg_alpha"));
      FilterTo(ctx, "sun.altitude");
      IM_CHECK(RowIsChecked(ctx, "sun.altitude"));

      FilterTo(ctx, "");
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);

      // Nothing on disk moved (AC3's second half, inherited from the copy model).
      IM_CHECK_EQ(ReadOverlayBytes(dir), before);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());
      for (const auto& row : rows) {
        FilterTo(ctx, row.key_path.c_str());
        IM_CHECK(!RowIsChecked(ctx, row.key_path));
      }

      FilterTo(ctx, "");
      SaveDefaultsPanel(ctx);
      const json saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(!saved.contains("sun"));
      // The preset library is a sibling namespace this button does not reach.
      IM_CHECK_EQ(ReadPresetStd(saved, "column").value_or(-1.0f), 0.3f);

      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // AC4 — the two filters are two DIFFERENT questions
  // ================================================================================
  {
    // The trap this case exists to catch is the two meanings of "has changes" collapsing into one
    // switch. So the two samples are chosen to be orthogonal — each hits exactly one filter:
    //   A (renderer.fov)  value differs from factory, checkbox NOT touched this session
    //   B (bg_show)       value IS the factory one, checkbox touched this session
    // A single "only show changes" toggle cannot separate these, whichever of the two it means.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac4_filters_separate_value_from_edit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_ac4_filter");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::g_state.renderer.fov = 95.0f;                           // sample A
      IM_CHECK_EQ(gui::g_state.bg_show, gui::GuiState{}.bg_show);  // sample B starts at factory

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

      // A opens checked (its value differs) and B opens unchecked, so toggling B is what makes it
      // "edited this session" — and A stays un-edited despite being checked. That asymmetry is the
      // point: "checked" and "edited" are not the same thing either.
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(RowIsChecked(ctx, "renderer.fov"));
      ToggleRow(ctx, "bg_show");
      IM_CHECK(RowIsChecked(ctx, "bg_show"));
      FilterTo(ctx, "");

      // Filter 1 — value layer. A hits, B misses.
      ctx->ItemClick("**/###filter_differs");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_show").c_str()));

      // ...and the search box ANDs with it rather than replacing it: a key that passes the search
      // but fails the filter stays hidden.
      FilterTo(ctx, "bg_show");
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_show").c_str()));
      FilterTo(ctx, "");

      // Filter 2 — operation layer. B hits, A misses. Both directions, so neither filter can be
      // an alias of the other.
      ctx->ItemClick("**/###filter_edited");
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef("bg_show").c_str()));
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("renderer.fov").c_str()));

      // A row hidden by a filter is still a row: Save applies the whole list, not the visible part
      // of it. (A filter that silently narrowed what gets written would be a data-loss bug wearing
      // a view control's clothes.)
      SaveDefaultsPanel(ctx);
      const json saved = ReadOverlayFile(dir);
      IM_CHECK(saved.contains("renderer"));  // A was hidden, and still written
      IM_CHECK_EQ(saved["renderer"]["fov"].get<float>(), 95.0f);
      IM_CHECK(saved.contains("bg_show"));  // B was visible, and written

      // Back to All, so the next scenario in this single-process suite starts from a full list.
      ctx->ItemClick("**/###filter_all");
      ctx->Yield(2);
      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // Sticky headers — what stays put while the list scrolls
  //
  // Three claims, all of them geometric, and none of them expressible as "can I still click it":
  // an item that has scrolled off the top is still perfectly clickable after the engine scrolls it
  // back, so a reachability test would pass on the panel this change exists to fix.
  //
  // Two INDEPENDENT mechanisms are under test and neither substitutes for the other:
  //   the table's frozen header row     -> TableSetupScrollFreeze(0, 1)
  //   the two section headers standing still -> they are not inside any scrolling region at all
  // A single case covering both would not say which one broke.
  // ================================================================================
  {
    // AC1 — the settings table's header row does not move, and the columns do not shift, while the
    // body scrolls under it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "sticky_ac1_table_header_survives_scrolling");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_sticky_ac1");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());

      ImGuiTable* table = SettingsTable(ctx);
      IM_CHECK(table != nullptr);
      IM_CHECK(table->InnerWindow != nullptr);
      RewindSettingsList(ctx, table);

      // The premise this case rests on, asserted rather than assumed: the list is longer than the
      // box it was given, so there is something for the wheel to do. Without it, "the header did not
      // move" would be just as true of a list too short to scroll at all.
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

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC2 — the two section headers do not move while a section's body scrolls, and the panel
    // window itself never scrolls.
    //
    // The second half is the "two-layer scroll" trap the change had to avoid: a table given its own
    // ScrollY while the old shared child was still wrapped around it would look correct in a
    // screenshot and feel wrong under the wheel. ScrollMax.y == 0 says there is exactly one
    // scrolling region under the pointer, and it is the section's.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "sticky_ac2_section_headers_do_not_scroll");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_sticky_ac2");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
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

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC3 — the layout change did not cost the panel its folding behavior. Two separate claims:
    //
    //   (a) both CollapsingHeaders still fold and unfold on a click. Each section's body is now a
    //       child/table that only exists inside the `if (header)` branch, so a header that stopped
    //       reporting its state would take the whole section with it.
    //   (b) the entry point's initial section still wins on the opening frame, and ONLY on that
    //       frame — a section the user collapses afterwards must stay collapsed. That "one frame
    //       only" half is what an SetNextItemOpen(ImGuiCond_Always) left running every frame would
    //       break, and it is invisible to a test that merely opens the panel and looks.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "sticky_ac3_sections_still_fold_and_honor_entry_point");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_sticky_ac3");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // A row that is unambiguously present, narrowed to so the settings body is one short list.
      gui::g_state.renderer.fov = 95.0f;

      // (a) §2 first: opened expanded by the entry point, then folded and unfolded by hand.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      const std::string probe_row = AdoptCheckboxRef("renderer.fov");
      IM_CHECK(ctx->ItemExists(probe_row.c_str()));

      ctx->ItemClick("**/###defaults_settings");  // fold
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists(probe_row.c_str()));
      ctx->ItemClick("**/###defaults_settings");  // unfold
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists(probe_row.c_str()));

      // ...and §1, whose body is the other of the two new scrolling regions.
      ctx->ItemClick("**/###defaults_presets");  // unfold (it opened collapsed on kSettings)
      ctx->Yield(3);
      IM_CHECK(ctx->ItemExists("**/###preset_Column"));
      ctx->ItemClick("**/###defaults_presets");  // fold
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);

      // (b) the entry point decides the opening frame — and then lets go. Collapsing §1 while the
      // panel is open must stick for as long as it stays open.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      IM_CHECK(ctx->ItemExists("**/###preset_Column"));
      ctx->ItemClick("**/###defaults_presets");
      ctx->Yield(4);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
      // Several more frames: a forced-open state re-applied every frame would have snapped it back
      // by now, and one Yield would not have caught it.
      ctx->Yield(8);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));

      CloseDefaultsPanel(ctx);
    };
  }

  // ================================================================================
  // Inline cell editing. The claim under test throughout is NOT "a control appears in the cell" —
  // it is that the control is the SAME one, with the SAME constraint, as the main UI's.
  // ================================================================================

  {
    // AC1 — an out-of-range entry lands in the same place whether it is typed into the table or
    // into the main UI's own control.
    //
    // Three fields, one per constraint SHAPE the registry has to express, rather than three
    // arbitrary fields:
    //   renderer.fov        — a bound that is a function of the state (the lens type's max FOV)
    //   overlay_grid_alpha  — a constant bound
    //   sim.max_hits        — an integer bound, through the newly exported SliderIntWithInput
    // The fourth shape the issue asked for (a float on a non-linear slider scale) has no
    // representative: not one of the 42 candidate defaults is edited on a kSqrt/kLog scale in the
    // main UI — those are all axis-distribution fields, which are per-crystal and never defaults.
    // Its stand-in here is the state-dependent bound, which is the harder of the two.
    //
    // Each field is driven twice from the same starting value, and the final assertion is that the
    // two agree AND that they agree on the CLAMPED value rather than on "nothing happened" — a
    // panel whose cell silently ignored input would otherwise pass by matching a main UI that was
    // never touched either.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac1_table_clamps_like_the_main_ui");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_inline_ac1");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      const float max_fov = LUMICE_MaxFov(static_cast<LUMICE_LensType>(gui::g_state.renderer.lens_type));
      IM_CHECK_GT(max_fov, 90.0f);  // the probe below has to be outside the domain to test anything

      // ---- the domains themselves, before driving anything through them ----
      //
      // Not redundant with the drive-it-twice comparison below, and the reason is a real limit on
      // what that comparison can see: the main UI calls the SAME control every frame and it clamps
      // unconditionally, so a table cell that allowed a wider range would have its value pulled
      // back within a frame and the two would still agree. Asserting the domain directly is what
      // catches a bound that drifted, as opposed to a cell that ignored its input.
      //
      // fov is checked against the expression the main UI uses (LUMICE_MaxFov of the current lens)
      // rather than against a number, and at two lens types, because "the bound follows the state"
      // is the property — a registry that returned a constant would pass one of these and fail the
      // other.
      const gui::FieldEditorEntry* fov_entry = gui::FindFieldEditor("renderer.fov");
      IM_CHECK(fov_entry != nullptr);
      for (const int lens : { gui::kLensTypeLinear, gui::kLensTypeGlobe }) {
        gui::GuiState probe_state;
        probe_state.renderer.lens_type = lens;
        const auto constraint = fov_entry->Constraint(probe_state);
        IM_CHECK(constraint.has_numeric_domain);
        IM_CHECK_EQ(constraint.min_value, 1.0);
        IM_CHECK_EQ(static_cast<float>(constraint.max_value), LUMICE_MaxFov(static_cast<LUMICE_LensType>(lens)));
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
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.fov");
      ctx->ItemInputValue(ValueInputRef("renderer.fov").c_str(), 900.0f);
      ctx->Yield(3);
      const float fov_from_table = gui::g_state.renderer.fov;
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);

      gui::g_state.renderer.fov = 90.0f;
      ctx->Yield(2);
      ctx->ItemInputValue("**/##FOV##view_input", 900.0f);
      ctx->Yield(3);
      const float fov_from_main_ui = gui::g_state.renderer.fov;

      IM_CHECK_EQ(fov_from_table, fov_from_main_ui);
      IM_CHECK_EQ(fov_from_table, max_fov);  // non-vacuous: both clamped, neither ignored the input

      // ---- overlay_grid_alpha (constant domain) ----
      gui::g_state.grid_alpha = 0.3f;
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_grid_alpha");
      ctx->ItemInputValue(ValueInputRef("overlay_grid_alpha").c_str(), 7.5f);
      ctx->Yield(3);
      const float alpha_from_table = gui::g_state.grid_alpha;
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);

      gui::g_state.grid_alpha = 0.3f;
      ctx->Yield(2);
      ctx->ItemInputValue("**/##Alpha##grid_input", 7.5f);
      ctx->Yield(3);
      const float alpha_from_main_ui = gui::g_state.grid_alpha;

      IM_CHECK_EQ(alpha_from_table, alpha_from_main_ui);
      IM_CHECK_EQ(alpha_from_table, 1.0f);

      // ---- sim.max_hits (integer domain) ----
      gui::g_state.sim.max_hits = 8;
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "sim.max_hits");
      ctx->ItemInputValue(ValueInputRef("sim.max_hits").c_str(), 4096);
      ctx->Yield(3);
      const int hits_from_table = gui::g_state.sim.max_hits;
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);

      gui::g_state.sim.max_hits = 8;
      ctx->Yield(2);
      ctx->ItemInputValue("**/##Max hits_input", 4096);
      ctx->Yield(3);
      const int hits_from_main_ui = gui::g_state.sim.max_hits;

      IM_CHECK_EQ(hits_from_table, hits_from_main_ui);
      IM_CHECK_EQ(hits_from_table, 64);

      // ---- renderer.opacity: the same clamp, where nothing else can be doing it ----
      //
      // The three fields above all have a main-UI control that clamps them every frame, so their
      // final value is evidence about the table's cell only if the table wrote it first. opacity
      // has no control anywhere else in the app: whatever this cell leaves behind is what the
      // field holds, indefinitely. It is therefore the one field whose clamp this suite can
      // attribute to the table with no alternative explanation.
      gui::g_state.renderer.opacity = 0.5f;
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.opacity");
      ctx->ItemInputValue(ValueInputRef("renderer.opacity").c_str(), 5.0f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 1.0f);
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 1.0f);  // and it stays: nothing else touches it
    };
  }

  {
    // AC1, second half — the cell edits the SAME FIELD the main UI edits, for the control shapes
    // where "out of range" is not a meaningful input (a checkbox and a combo cannot be given an
    // out-of-domain value; their constraint IS the shape of the control).
    //
    // Also the applicability half: a field whose main-UI control is disabled in some configuration
    // must be disabled in the table in the SAME configuration, or the table becomes a second way to
    // reach states the main UI refuses — which is the cross-field question the issue asked this
    // task to take a position on.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac1_bool_combo_and_applicability");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_inline_ac1b");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::g_state.show_grid_line = false;
      gui::g_state.renderer.visible = gui::kVisibleFull;
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

      // A checkbox cell writes the field itself.
      FilterTo(ctx, "overlay_grid_line");
      ctx->ItemClick(ValueWidgetRef("overlay_grid_line").c_str());
      ctx->Yield(2);
      IM_CHECK(gui::g_state.show_grid_line);

      // A combo cell writes the enum the main UI's radio buttons write.
      //
      // Opened and picked by hand rather than through ComboClick: that helper splits its path at
      // the FIRST '/', which lands on the "**/" wildcard every ref in this file starts with.
      FilterTo(ctx, "renderer.visible");
      const ImGuiID visible_combo = SettingsCellID(ctx, "##value_renderer.visible");
      IM_CHECK(visible_combo != 0);
      ctx->ItemClick(visible_combo);
      ctx->Yield(2);
      // The popup's entries ARE label-addressable (Selectable registers its label), so only the
      // combo button itself needs the id treatment. Addressed through the popup window's own name,
      // the way ComboClick does it internally.
      ImGuiWindow* combo_popup = ctx->GetWindowByRef("//$FOCUSED");
      IM_CHECK(combo_popup != nullptr);
      ctx->ItemClick((std::string("//") + combo_popup->Name + "/**/Upper").c_str());
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.renderer.visible, gui::kVisibleUpper);

      // Applicability: roll applies under a linear lens...
      FilterTo(ctx, "renderer.roll");
      IM_CHECK(ctx->ItemExists(ValueInputRef("renderer.roll").c_str()));
      IM_CHECK((ctx->ItemInfo(ValueInputRef("renderer.roll").c_str()).ItemFlags & ImGuiItemFlags_Disabled) == 0);
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);

      // ...and not under a full-sky one, in the table exactly as in the main UI.
      gui::g_state.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
      IM_CHECK(gui::LensIsFullSky(gui::g_state.renderer.lens_type));
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "renderer.roll");
      IM_CHECK((ctx->ItemInfo(ValueInputRef("renderer.roll").c_str()).ItemFlags & ImGuiItemFlags_Disabled) != 0);
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC2 — a field with no registered editor cannot be edited here, and does not merely LOOK
    // uneditable.
    //
    // The row still exists (it is still a default the user can hold), which is what makes the
    // negative claim non-vacuous: the checkbox is found, so the row was drawn, and the absence of a
    // value widget is a fact about the cell rather than about the scroll position. Every id shape a
    // value cell could have taken is probed, and a registered neighbour is probed the same way as
    // the positive control.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac2_unregistered_field_is_read_only");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_inline_ac2");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // The registry's answer, asked directly. This IS the mechanical criterion the panel branches
      // on, so it is asserted rather than inferred from what got drawn.
      IM_CHECK(gui::FindFieldEditor("bg_path") == nullptr);
      IM_CHECK(gui::FindFieldEditor("overlay_sun_circle_angles") == nullptr);
      IM_CHECK(gui::FindFieldEditor("overlay_grid_alpha") != nullptr);

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

      for (const char* unregistered : { "bg_path", "overlay_sun_circle_angles" }) {
        FilterTo(ctx, unregistered);
        IM_CHECK(ctx->ItemExists(AdoptCheckboxRef(unregistered).c_str()));  // the row IS on screen
        IM_CHECK(!AnyValueWidgetExists(ctx, unregistered));
      }

      // The positive control, through the same probe: a registered row of the same list DOES carry
      // a value widget, so the probe can tell the two apart.
      FilterTo(ctx, "overlay_grid_alpha");
      IM_CHECK(AnyValueWidgetExists(ctx, "overlay_grid_alpha"));

      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC3 — coverage as a COUNTABLE deliverable, not "the mechanism works".
    //
    // Every leaf of the serialized document is named below with the control it must be edited
    // with, and the case fails on the first row whose classification disagrees. A count would not
    // do: it passes just as well when two fields swap classes, and it says nothing at all about
    // WHICH field regressed. The two deliberately-unregistered leaves are named here too, so
    // "read-only" is an asserted decision rather than an omission that looks like one.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac3_registry_covers_every_row");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      const auto dir = FreshOverlayDir("panel_inline_ac3");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      using Kind = gui::FieldEditorKind;
      struct Expected {
        const char* key_path;
        bool registered;
        Kind kind;  // ignored when !registered
      };
      // Order follows SerializeGuiStateJson so the two can be read side by side.
      static const Expected kExpected[] = {
        { "sun.altitude", true, Kind::kFloatSlider },
        { "sun.diameter", true, Kind::kFloatSlider },
        { "sun.spectrum", true, Kind::kCombo },
        { "sim.ray_num_millions", true, Kind::kFloatSlider },
        { "sim.max_hits", true, Kind::kIntSlider },
        { "sim.infinite", true, Kind::kCheckbox },
        { "renderer.lens_type", true, Kind::kCombo },
        { "renderer.fov", true, Kind::kFloatSlider },
        { "renderer.elevation", true, Kind::kFloatSlider },
        { "renderer.azimuth", true, Kind::kFloatSlider },
        { "renderer.roll", true, Kind::kFloatSlider },
        // The named special case: serialized as a VALUE (1024) while its control edits an index.
        // Registered as editable — the registry translates — rather than left read-only.
        { "renderer.sim_resolution", true, Kind::kCombo },
        { "renderer.visible", true, Kind::kCombo },
        { "renderer.front", true, Kind::kCheckbox },
        { "renderer.background", true, Kind::kColor },
        { "renderer.ray_color", true, Kind::kColor },
        { "renderer.opacity", true, Kind::kFloatSlider },
        { "renderer.exposure_offset", true, Kind::kFloatSlider },
        { "aspect_ratio", true, Kind::kCombo },
        { "aspect_portrait", true, Kind::kCheckbox },
        { "bg_path", false, Kind::kCheckbox },
        { "bg_show", true, Kind::kCheckbox },
        { "bg_alpha", true, Kind::kFloatSlider },
        { "overlay_horizon_line", true, Kind::kCheckbox },
        { "overlay_horizon_label", true, Kind::kCheckbox },
        { "overlay_grid_line", true, Kind::kCheckbox },
        { "overlay_grid_label", true, Kind::kCheckbox },
        { "overlay_sun_circles_line", true, Kind::kCheckbox },
        { "overlay_sun_circles_label", true, Kind::kCheckbox },
        { "overlay_sun_circle_angles", false, Kind::kCheckbox },
        { "overlay_horizon_color", true, Kind::kColor },
        { "overlay_grid_color", true, Kind::kColor },
        { "overlay_sun_circles_color", true, Kind::kColor },
        { "overlay_horizon_alpha", true, Kind::kFloatSlider },
        { "overlay_grid_alpha", true, Kind::kFloatSlider },
        { "overlay_sun_circles_alpha", true, Kind::kFloatSlider },
        { "overlay_zenith_nadir_line", true, Kind::kCheckbox },
        { "overlay_zenith_nadir_color", true, Kind::kColor },
        { "overlay_zenith_nadir_alpha", true, Kind::kFloatSlider },
        { "overlay_zenith_nadir_radius_px", true, Kind::kFloatSlider },
        { "right_panel_collapsed", true, Kind::kCheckbox },
        { "modal_layout_vertical", true, Kind::kCheckbox },
      };

      // Against the REAL row set of a factory document, so this cannot drift from what the panel
      // actually lists: a field added to the serializer makes the row set larger than the table
      // below and fails here, which is the reminder to classify it.
      gui::g_state = gui::MakeNewDocumentState();
      const auto rows = CurrentRows();
      std::set<std::string> row_keys;
      for (const auto& row : rows) {
        row_keys.insert(row.key_path);
      }
      IM_CHECK_EQ(row_keys.size(), std::size(kExpected));

      for (const auto& expected : kExpected) {
        IM_CHECK_SILENT(row_keys.count(expected.key_path) == 1);
        const gui::FieldEditorEntry* entry = gui::FindFieldEditor(expected.key_path);
        if (expected.registered) {
          IM_CHECK_SILENT(entry != nullptr);
          IM_CHECK_SILENT(entry->kind == expected.kind);
        } else {
          IM_CHECK_SILENT(entry == nullptr);
        }
      }

      // Nothing is registered that the document does not produce: an entry for a key that no longer
      // exists is dead weight the panel can never reach, and it would silently survive the loop
      // above.
      for (const auto& key : gui::RegisteredFieldEditorKeyPaths()) {
        IM_CHECK_SILENT(row_keys.count(key) == 1);
      }
    };
  }

  {
    // AC4 — "Origin value" is the FACTORY value, not the effective (saved) default.
    //
    // Asserted through the drawn cell rather than through the row struct: the cell's id carries the
    // text it rendered, so this fails if the column is re-pointed at default_value even though the
    // row struct still holds the right factory value. The two values are deliberately made
    // different first, otherwise the claim is unfalsifiable.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac4_origin_column_is_the_factory_value");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_inline_ac4");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["overlay_grid_alpha"] = 0.75f;  // saved default, deliberately not the factory 0.3
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_grid_alpha");

      const auto rows = CurrentRows();
      const auto row = std::find_if(rows.begin(), rows.end(),
                                    [](const gui::DefaultDiffRow& r) { return r.key_path == "overlay_grid_alpha"; });
      IM_CHECK(row != rows.end());
      IM_CHECK(row->has_saved_override);
      // The premise: the saved default and the factory value really do differ here.
      IM_CHECK(row->default_value != row->factory_value);

      const std::string factory_text = gui::FormatDiffValue(row->factory_value);
      const std::string saved_text = gui::FormatDiffValue(row->default_value);
      IM_CHECK(ctx->ItemExists(OriginCellRef("overlay_grid_alpha", factory_text).c_str()));
      IM_CHECK(!ctx->ItemExists(OriginCellRef("overlay_grid_alpha", saved_text).c_str()));

      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
    };
  }

  {
    // AC5b — the row tint's predicate, over the four states the issue enumerates.
    //
    // Asserted on the predicate rather than on pixels, deliberately and with the gap named: ImGui
    // keeps a table's per-row background colour only for the row being submitted, so there is no
    // frame-independent handle on "row N was tinted". What binds the predicate to what is PAINTED
    // is the defaults_panel_layout reference group — the pending_changes scene contains tinted rows
    // at a 40 dB floor, so a call site that stopped tinting turns that scene red.
    //
    // The fourth state is the one this whole predicate exists for: a key saved long ago, untouched
    // since. It differs from factory forever, yet Save would not move it, so it must NOT be tinted.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac5b_row_tint_predicate_four_states");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      gui::DefaultDiffRow row;
      row.key_path = "probe";

      // (1) nothing saved, nothing adopted — Save writes nothing for this key.
      row.current_value = 0.5f;
      row.default_value = 0.5f;
      row.factory_value = 0.5f;
      row.has_saved_override = false;
      IM_CHECK(!gui::RowWouldChangeOnSave(row, /*checked=*/false));

      // (2) the same row, adopted: Save would ADD the key. Tinted even though the value equals the
      // factory one — presence is a change.
      IM_CHECK(gui::RowWouldChangeOnSave(row, /*checked=*/true));

      // (3) changed in the GUI, never saved: adopted by default, and Save would write the new
      // value.
      row.current_value = 0.9f;
      IM_CHECK(gui::RowWouldChangeOnSave(row, /*checked=*/true));
      // ...and un-checking it takes it back to "Save writes nothing", which is also no change.
      IM_CHECK(!gui::RowWouldChangeOnSave(row, /*checked=*/false));

      // (4) saved earlier, untouched since: differs from factory, equals what is on disk. NOT
      // tinted — the state that separates this predicate from the "Differs from factory" filter.
      row.current_value = 0.9f;
      row.default_value = 0.9f;  // has_saved_override ⇒ default_value IS the stored value
      row.factory_value = 0.5f;
      row.has_saved_override = true;
      IM_CHECK(!gui::RowWouldChangeOnSave(row, /*checked=*/true));
      // Un-checking it would REMOVE the key from the file — a change.
      IM_CHECK(gui::RowWouldChangeOnSave(row, /*checked=*/false));
    };
  }

  {
    // AC5 — the two notice icons are separate, and each has a producer that can actually fire.
    //
    // The out-of-range icon's reachability is the finding this case pins, because it inverts the
    // obvious guess: SliderWithInput ends with an unconditional clamp and the main UI calls it
    // every frame, so a hand-edited out-of-range ALPHA is pulled back into its domain before this
    // panel ever sees it. The reachable fields are the ones with no main-UI control at all. Both
    // directions are asserted — the field that keeps the poison and two that cannot — so "this
    // notice can fire" is not confused with "this notice fires for everything".
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac5_note_icons_are_distinct");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_inline_ac5");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // A hand-edited defaults file, out of range on three fields.
      json doc;
      doc["renderer"]["opacity"] = 3.0f;  // no main-UI control ⇒ nothing clamps it
      doc["overlay_grid_alpha"] = 7.0f;   // its own slider clamps it every frame
      doc["renderer"]["fov"] = 4000.0f;   // the per-frame renderer invariant clamps it
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 3.0f);  // the poison did land
      ctx->Yield(4);  // let the main UI render — this is where the other two get pulled back

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);

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

      // Both at once, on one row: the out-of-range field, once edited, carries the pencil AND
      // keeps the warning until the edit takes it back into the domain. Editing it to a value
      // inside [0,1] clears the warning and leaves the pencil, which is the pair's whole point —
      // they are independent.
      FilterTo(ctx, "renderer.opacity");
      ctx->ItemInputValue(ValueInputRef("renderer.opacity").c_str(), 0.25f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.renderer.opacity, 0.25f);
      IM_CHECK(ctx->ItemExists("**/###note_edited_renderer.opacity"));
      IM_CHECK(!ctx->ItemExists("**/###note_range_renderer.opacity"));

      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
    };
  }

  {
    // The round trip the whole feature exists for: change a setting the user has never touched in
    // the main UI, from the table, and have it land in the defaults file.
    //
    // Also pins the two consequences of an in-cell edit that are decisions rather than mechanics:
    // the row joins the checked set (otherwise the edit would be silently dropped at Save), and
    // nothing is written until Save is actually pressed.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_edit_round_trips_through_save");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_inline_roundtrip");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();
      gui::g_state = gui::MakeNewDocumentState();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_zenith_nadir_radius_px");
      IM_CHECK(!RowIsChecked(ctx, "overlay_zenith_nadir_radius_px"));  // untouched rows open unchecked

      ctx->ItemInputValue(ValueInputRef("overlay_zenith_nadir_radius_px").c_str(), 12.5f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.zenith_nadir_radius_px, 12.5f);
      // The edit adopted the row: an edit that Save would discard is the failure this avoids.
      IM_CHECK(RowIsChecked(ctx, "overlay_zenith_nadir_radius_px"));
      // ...but nothing has been written yet.
      IM_CHECK(!ReadOverlayBytes(dir).has_value() ||
               !gui::DocHasKeyPath(ReadOverlayFile(dir), "overlay_zenith_nadir_radius_px"));

      SaveDefaultsPanel(ctx);
      const json saved = ReadOverlayFile(dir);
      IM_CHECK(gui::DocHasKeyPath(saved, "overlay_zenith_nadir_radius_px"));
      IM_CHECK_EQ(saved["overlay_zenith_nadir_radius_px"].get<float>(), 12.5f);
      // Saved ⇒ the "you changed this here" pencil is retired: it would now be a lie.
      IM_CHECK(!ctx->ItemExists("**/###note_edited_overlay_zenith_nadir_radius_px"));

      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
    };
  }

  {
    // Regression guard for a "logically distinct control, structurally the same per-frame-commit
    // bug" family: a field type must gate its write-back on the interaction actually ending, not on
    // ImGui's per-frame "value changed" return. Sliders (see the inline_ tests above) get this from
    // IsItemDeactivatedAfterEdit(); ColorField carried the identical bug because ColorEdit3's picker
    // popup is drag-continuous just like a slider, but its return value was taken as the commit
    // signal directly. This pins the field-value side of it: the underlying GuiState field must NOT
    // change while the picker's sv-square is held, and must change the instant it is released — the
    // same "commit on release, not on delta" contract the inline_ tests above already pin for
    // sliders via a single-frame ItemInputValue (which cannot distinguish "changed on release" from
    // "changed every frame", since it never holds a frame in between — this test is the one that
    // can).
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "inline_ac1_color_commits_on_release_not_per_frame");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state = gui::MakeNewDocumentState();

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kSettings);
      FilterTo(ctx, "overlay_grid_color");

      const float base[3] = { gui::g_state.grid_color[0], gui::g_state.grid_color[1], gui::g_state.grid_color[2] };

      // The swatch button id, reproducing ColorEdit4's internal `PushID(label)` + `ColorButton
      // ("##ColorButton", ...)` chain against the table-seeded group id — see SettingsCellID's own
      // comment for why this has to be computed rather than found by a "**/" label search.
      const ImGuiID group_id = SettingsCellID(ctx, "##value_overlay_grid_color");
      IM_CHECK(group_id != 0);
      const ImGuiID swatch_id = ImGui::GetIDWithSeed("##ColorButton", nullptr, group_id);
      ctx->ItemClick(swatch_id);
      ctx->Yield(2);

      // Inside the picker popup now. "sv" is ColorPicker4's saturation/value square — an ordinary
      // InvisibleButton, unlike the swatch it IS registered with the test engine (test_gui_lifecycle
      // .cpp's raypath-color test relies on the same "**/sv" lookup for the same reason).
      IM_CHECK(ctx->ItemExists("**/sv"));
      const ImGuiTestItemInfo sv = ctx->ItemInfo("**/sv");
      const ImVec2 sv_lo = sv.RectFull.Min;
      const ImVec2 sv_hi = sv.RectFull.Max;
      ctx->LogWarning("DIAG sv rect (%.1f,%.1f)-(%.1f,%.1f) window=%s", sv_lo.x, sv_lo.y, sv_hi.x, sv_hi.y,
                      sv.Window ? sv.Window->Name : "null");
      // Two corners rather than "wherever the mouse already is": guarantees a real S/V delta
      // regardless of MakeNewDocumentState()'s starting grid_color, the same guarantee the
      // lifecycle precedent gets for free by always landing on the S=1,V=1 corner.
      const ImVec2 start(sv_lo.x + (sv_hi.x - sv_lo.x) * 0.15f, sv_lo.y + (sv_hi.y - sv_lo.y) * 0.15f);
      const ImVec2 end(sv_lo.x + (sv_hi.x - sv_lo.x) * 0.85f, sv_lo.y + (sv_hi.y - sv_lo.y) * 0.85f);

      ctx->MouseMoveToPos(start);
      ctx->MouseDown(0);
      ctx->Yield(2);
      // Held, not yet released: the pre-fix bug wrote every one of these frames straight into
      // grid_color (ColorEdit3 bound the live field directly) — this is the assertion that would
      // have caught it.
      IM_CHECK_EQ(gui::g_state.grid_color[0], base[0]);
      IM_CHECK_EQ(gui::g_state.grid_color[1], base[1]);
      IM_CHECK_EQ(gui::g_state.grid_color[2], base[2]);

      ctx->MouseMoveToPos(end);
      ctx->Yield(2);
      // Still held after moving further — same claim, a second time, so a fix that merely
      // special-cased the first frame of the drag would not pass.
      IM_CHECK_EQ(gui::g_state.grid_color[0], base[0]);
      IM_CHECK_EQ(gui::g_state.grid_color[1], base[1]);
      IM_CHECK_EQ(gui::g_state.grid_color[2], base[2]);

      ctx->MouseUp(0);
      ctx->Yield(2);
      // Released: the commit lands now, and the drag actually reached a different colour (sanity
      // that the corner-to-corner move landed on the square rather than missing it).
      IM_CHECK(gui::g_state.grid_color[0] != base[0] || gui::g_state.grid_color[1] != base[1] ||
               gui::g_state.grid_color[2] != base[2]);

      ctx->PopupCloseAll();
      FilterTo(ctx, "");
      CloseDefaultsPanel(ctx);
    };
  }
}
