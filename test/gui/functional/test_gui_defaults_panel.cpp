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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "gui/defaults_diff.hpp"
#include "gui/defaults_panel.hpp"
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

std::vector<gui::DefaultDiffRow> CurrentRows() {
  return gui::BuildDefaultDiffRows(gui::g_state);
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
      ctx->ItemClick(RevertButtonRef("bg_alpha").c_str());
      ctx->Yield(3);

      json saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(saved.contains("renderer"));
      IM_CHECK(saved.contains("presets"));
      // The reverted key is back to factory for a new document — the file change is the whole
      // point, not a panel-local undo.
      IM_CHECK_EQ(gui::MakeNewDocumentState().bg_alpha, gui::GuiState{}.bg_alpha);

      ctx->ItemClick("**/###defaults_reset_all");
      ctx->Yield(3);

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
}
