// Defaults panel interaction tests — the UI half of the "save current settings as my defaults"
// feature (src/gui/defaults_panel.*).
//
// Every case here drives the real widgets through ImGuiTestContext and then asserts on the
// override FILE (or on a freshly built document), never on the panel's own internal state: the
// question these tests answer is "did clicking this actually change what a new document starts
// from", and a panel that updated only its own vector would satisfy any in-memory assertion.
//
// Coverage:
//   AC6  — §2 / §3 are mutually exclusive ON SCREEN, row by row (not by eyeballing a screenshot)
//   AC7  — adoption round-trip: uncheck one row, save, and the unchecked key is NOT in the file
//   AC8  — Revert one row / Reset all, asserted file-side, with the presets subtree preserved
//   AC9  — invariant I1 through the full GUI path: an opened .lmc beats the personal default
//   AC10 — the search box filters both sections, and clearing it restores every row
//   plus the entry-point contract: which section an entry point opens expanded
//
// The copy model (the panel is a pure editor: edits go into an in-memory copy, Save writes it once,
// closing discards it) adds four more, and rewrites the timing half of several of the above:
//   copy AC1 — a mixed batch of edits, closed WITHOUT Save, leaves the file byte-identical
//   copy AC2 — Reset all in both directions: discarded on close, and committed on Save
//   copy AC3 — a preset edit closed without Save reaches neither the cache nor the file
//   copy AC4 — §3's Source / Revert follow the COPY, while the §2/§3 partition follows the
//              snapshot the panel opened with
//
// Where a case used to click and read the file, it now clicks, asserts the file has NOT moved,
// then saves and asserts it has. That is the assertion the old model could not make.

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/defaults_panel.hpp"
#include "gui/edit_modals.hpp"
#include "gui/user_defaults.hpp"
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

std::string RevertButtonRef(const std::string& key_path) {
  return "**/###revert_" + key_path;
}

// §3's per-row source cell. §3 has no checkbox by design (its rows already equal the effective
// default, so "adopt" would be a no-op), so this is the widget that says "this key is rendered in
// §3" — the counterpart to §2's checkbox.
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
// items are unaware of their labels" — imgui_te_context.cpp), and §3 alone is 40+ rows in a
// fixed-height modal. Filtering first makes "is this key rendered in §2 or §3" a question about
// the panel rather than about where the scroll happened to be, and it exercises the search box on
// every case instead of only in the one written for it.
void FilterTo(ImGuiTestContext* ctx, const char* text) {
  ctx->ItemInputValue("**/###defaults_search", text);
  ctx->Yield(3);
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

}  // namespace

void RegisterDefaultsPanelTests(ImGuiTestEngine* engine) {
  // ================================================================================
  // AC6 — the two sections are mutually exclusive on screen
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac6_sections_are_mutually_exclusive_on_screen");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // The guard precedes ResetTestState because ResetTestState -> DoNew ->
      // MakeNewDocumentState reads the process-wide source: installed afterwards, the
      // document under test would already carry the running machine's saved defaults.
      const auto dir = FreshOverlayDir("panel_ac6");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      // Edit a few settings so §2 is non-empty; without this the exclusivity check would hold
      // vacuously with every row in §3.
      gui::g_state.bg_alpha = 0.42f;
      gui::g_state.renderer.fov = 95.0f;

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      ctx->ItemOpen("**/###defaults_other");
      ctx->Yield(3);

      const auto rows = CurrentRows();
      IM_CHECK(!rows.empty());
      int pending_seen = 0;
      int other_seen = 0;
      for (const auto& row : rows) {
        // One key at a time (see FilterTo): with both sections showing only this key, whichever
        // section renders it is unambiguous and nothing is off-screen.
        FilterTo(ctx, row.key_path.c_str());
        // A §2 row is addressable by its checkbox and a §3 row is not (the section has none), so
        // "which section is this key rendered in" is a machine-readable question.
        const bool in_pending = ctx->ItemExists(AdoptCheckboxRef(row.key_path).c_str());
        // §3 renders the source of every row it holds, so the key is visible there and only there.
        const bool in_other = ctx->ItemExists(SourceCellRef(row.key_path).c_str());
        IM_CHECK(in_pending != in_other);  // exactly one section, never both, never neither
        if (gui::RowNeedsAdoption(row)) {
          IM_CHECK(in_pending);
          ++pending_seen;
        } else {
          IM_CHECK(in_other);
          ++other_seen;
        }
      }
      IM_CHECK(pending_seen > 0);
      IM_CHECK(other_seen > 0);

      CloseDefaultsPanel(ctx);
    };
  }

  {
    // The "entry point decides the initial section" mechanism (405.5 attaches its own entry to
    // it). Asserted through an observable consequence — whether §2's rows are on screen — rather
    // than left as a manual check that only breaks once 405.5 tries to use it.
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
      const auto pending = std::find_if(rows.begin(), rows.end(),
                                        [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
      IM_CHECK(pending != rows.end());

      // Opened on §2: its rows are rendered, and §3 is collapsed (its rows are not).
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      IM_CHECK(ctx->ItemExists(AdoptCheckboxRef(pending->key_path).c_str()));
      const auto other = std::find_if(rows.begin(), rows.end(),
                                      [](const gui::DefaultDiffRow& row) { return !gui::RowNeedsAdoption(row); });
      IM_CHECK(other != rows.end());
      CloseDefaultsPanel(ctx);

      // Opened on §1: the SAME row is no longer rendered, i.e. the section choice actually
      // travelled from the entry point into the panel.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPresets);
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef(pending->key_path).c_str()));
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

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      // Uncheck exactly one row, leaving the rest checked (the panel opens with everything
      // checked — pressing the entry button already means "adopt what I have").
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

      // After the save the adopted row is no longer a pending change: current == effective
      // default, and the panel now attributes it to the user.
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
  // AC8 — Revert one row / Reset all, file-side, presets preserved
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac8_revert_and_reset_all_preserve_presets");
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

      // Start from a document that already carries those defaults, so both keys sit in §3 with
      // source "Mine" and therefore have a Revert button.
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.42f);

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      ctx->ItemOpen("**/###defaults_other");
      ctx->Yield(2);
      FilterTo(ctx, "bg_alpha");

      // MIGRATED for the copy model (405.4 asserted the file the instant Revert was clicked).
      // Split in two: the click must NOT reach the file, and it must still be visible on screen.
      const auto before_revert = ReadOverlayBytes(dir);
      IM_CHECK(before_revert.has_value());
      ctx->ItemClick(RevertButtonRef("bg_alpha").c_str());
      ctx->Yield(3);
      IM_CHECK_EQ(ReadOverlayBytes(dir), before_revert);
      // ...and the feedback the old write-through model failed to give: the row already reads as
      // Factory (no Revert button left on it) even though the file still holds the value.
      IM_CHECK(!ctx->ItemExists(RevertButtonRef("bg_alpha").c_str()));
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
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
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
  // AC10 (functional half) — the search box
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "ac10_search_filters_both_sections");
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

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
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
    // Step 3's gesture, the other direction: a crystal's live zenith std saved INTO the library.
    // A preset is not a GuiState field, so nothing in the session could infer this intent — the
    // gesture is the only way an override gets produced from a crystal the user is editing.
    //
    // This case reads the file straight after the click and that is DELIBERATE, not an oversight
    // left over from before the panel became a pure editor. The gesture is not part of a panel
    // session: the defaults panel is not even open, so there is no copy to edit and no Close for
    // "discard" to attach to. A one-shot button that commits when pressed is the only semantics
    // available to it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "axis_modal_gesture_saves_into_the_library");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_gesture");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      gui::EditRequest req{ gui::EditTarget::kAxis, /*layer_idx=*/0, /*entry_idx=*/0 };
      gui::OpenEditModal(req, gui::g_state);
      ctx->Yield(4);

      // Start from Column, then tighten the std to the value the beta user works at.
      ctx->ItemClick("**/Column");
      ctx->Yield(2);
      ctx->ItemInputValue("**/Zenith/##Std_input", 0.3f);
      ctx->Yield(2);

      ctx->ItemClick("**/###save_as_preset_column");
      ctx->Yield(3);

      const auto gestured = ReadPresetStd(ReadOverlayFile(dir), "column");
      IM_CHECK(gestured.has_value());
      IM_CHECK_EQ(*gestured, 0.3f);
      // Random is not a gesture target — it has no adjustable face, so a button for it would
      // write nothing and say nothing.
      IM_CHECK(!ctx->ItemExists("**/###save_as_preset_random"));

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
      // Cancel discards the crystal edit, but the gesture is NOT part of that buffer — it wrote to
      // the library, which is a separate persistent thing. Asserted because the opposite would be
      // an easy and invisible mistake to make.
      const auto after_cancel = ReadPresetStd(ReadOverlayFile(dir), "column");
      IM_CHECK(after_cancel.has_value());
      IM_CHECK_EQ(*after_cancel, 0.3f);
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

      // An unsaved GuiState change, so §2 has a checked pending row on top of everything else.
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

      ctx->ItemOpen("**/###defaults_other");
      ctx->Yield(2);
      FilterTo(ctx, "bg_alpha");
      ctx->ItemClick(RevertButtonRef("bg_alpha").c_str());  // §3 revert
      ctx->Yield(2);
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
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);
      // The panel shows the reset immediately — this is the feedback the old model owed the user
      // and could not give: §3's rows have lost their Revert buttons though the file still holds
      // the values.
      ctx->ItemOpen("**/###defaults_other");
      ctx->Yield(2);
      FilterTo(ctx, "renderer.fov");
      IM_CHECK(ctx->ItemExists(SourceCellRef("renderer.fov").c_str()));
      IM_CHECK(!ctx->ItemExists(RevertButtonRef("renderer.fov").c_str()));
      CloseDefaultsPanel(ctx);
      IM_CHECK_EQ(ReadOverlayBytes(dir), before);

      // (b) committed. Save after Reset all must write an EMPTY override set — not re-adopt §2's
      // checked rows, which would put a full set of defaults straight back and make the button a
      // no-op the user cannot see through.
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
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

  {
    // copy AC4 — which document each judgement reads.
    //
    // Two states of one row, in one session: copy == disk, then copy != disk. The Source cell and
    // the Revert button follow the COPY (they answer "what would Save leave me with"), while which
    // SECTION the row is in follows the snapshot the panel opened with, so rows do not move under
    // the user while they edit.
    //
    // The last assertion is the case the acceptance criterion names explicitly: a value that
    // differs from factory but is already stored in the defaults counts as NOT pending. bg_alpha
    // is exactly that — 0.42 against a factory value, and saved.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_panel", "copy_ac4_source_follows_copy_section_follows_snapshot");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto dir = FreshOverlayDir("panel_copy_ac4");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
      ResetTestState();
      ResetUserDefaultsChannels();

      json doc;
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::g_state = gui::MakeNewDocumentState();
      IM_CHECK_EQ(gui::g_state.bg_alpha, 0.42f);
      IM_CHECK_NE(gui::g_state.bg_alpha, gui::GuiState{}.bg_alpha);

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      ctx->ItemOpen("**/###defaults_other");
      ctx->Yield(2);
      FilterTo(ctx, "bg_alpha");

      // State 1: copy == disk. Differs from factory, but it is already saved, so it is NOT a
      // pending change — it sits in §3 (no adopt checkbox) and reads as Mine.
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));
      IM_CHECK(ctx->ItemExists(SourceCellRef("bg_alpha").c_str()));
      IM_CHECK(ctx->ItemExists(RevertButtonRef("bg_alpha").c_str()));

      // State 2: copy != disk. The Revert button is gone the moment the copy loses the key —
      // before anything is written.
      ctx->ItemClick(RevertButtonRef("bg_alpha").c_str());
      ctx->Yield(3);
      IM_CHECK(!ctx->ItemExists(RevertButtonRef("bg_alpha").c_str()));
      IM_CHECK(ReadOverlayFile(dir).contains("bg_alpha"));  // ...and the disk still has it

      // The row did NOT move to §2, though its value now differs from what the copy would resolve.
      // Section membership is anchored to the opening snapshot on purpose: a row that jumped
      // sections on a click the user has not committed would be exactly the disorientation this
      // panel is being fixed to remove.
      IM_CHECK(!ctx->ItemExists(AdoptCheckboxRef("bg_alpha").c_str()));
      IM_CHECK(ctx->ItemExists(SourceCellRef("bg_alpha").c_str()));

      CloseDefaultsPanel(ctx);
    };
  }

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

      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
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
    // The entry point contract for §1, matching the one already asserted for §2: "Edit My
    // Presets..." must open the panel with the preset section expanded, not merely open it.
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
      OpenPanelOn(ctx, gui::DefaultsPanelSection::kPendingChanges);
      IM_CHECK(!ctx->ItemExists("**/###preset_Column"));
      CloseDefaultsPanel(ctx);
    };
  }
}
