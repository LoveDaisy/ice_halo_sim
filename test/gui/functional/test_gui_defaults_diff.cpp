// Defaults diff-engine tests — the pure-logic half of the defaults panel (src/gui/defaults_diff.*).
//
// Nothing here renders a frame: every case calls the engine directly and asserts on the returned
// row set or on the override file it wrote. They live in gui_test only because the engine links
// against file_io.cpp / user_defaults.cpp (lumice_gui_obj), which unit_correctness_test does not
// pull in — the same Part1/Part2 split user_defaults.hpp documents.
//
// Coverage:
//   AC1 — generativity: every serialized root key is either excluded by name or produces rows,
//         so a newly serialized GuiState field appears in the panel with no engine change
//   AC2 — the two sections partition one row set (no key in both, no key missing, no duplicates)
//   AC3 — namespace 4 (layers / crystals / filters / raypath_color) never reaches the row set
//   AC4 — an array is ONE row, not one row per element
//   AC5 — "effective default" layers the saved override over factory, and enums read as names
//   plus the write-back contract (adopt / revert / reset all) that AC7 and AC8 drive from the UI

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

#include "gui/defaults_diff.hpp"
#include "gui/user_defaults.hpp"
#include "test_gui_shared.hpp"
#include "user_defaults_test_env.hpp"

namespace {

using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;
using lumice::test_user_defaults::ScopedUserConfigSource;
using nlohmann::json;

const gui::DefaultDiffRow* FindRow(const std::vector<gui::DefaultDiffRow>& rows, const std::string& key_path) {
  const auto it =
      std::find_if(rows.begin(), rows.end(), [&](const gui::DefaultDiffRow& row) { return row.key_path == key_path; });
  return it == rows.end() ? nullptr : &(*it);
}

// A single comparable string for a whole row set — used by AC3, where the claim is "nothing
// changed at all", not "this one field is still equal".
std::string RowSetDigest(const std::vector<gui::DefaultDiffRow>& rows) {
  json dump = json::array();
  for (const auto& row : rows) {
    dump.push_back({ { "key", row.key_path },
                     { "current", row.current_value },
                     { "default", row.default_value },
                     { "mine", row.has_saved_override } });
  }
  return dump.dump();
}

// A state whose values differ from factory across several shapes (scalar / enum / array / nested),
// so the section split is exercised in both directions rather than on an all-equal document.
gui::GuiState MakeEditedState() {
  gui::GuiState state = gui::InitDefaultState();
  state.bg_alpha = 0.42f;
  state.renderer.fov = 95.0f;
  state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
  state.renderer.background[1] = 0.5f;
  state.sun.altitude = 33.0f;
  state.aspect_portrait = !state.aspect_portrait;
  return state;
}

std::filesystem::path OverlayFilePath(const std::filesystem::path& dir) {
  return dir / gui::kUserDefaultsFileName;
}

json ReadOverlayFile(const std::filesystem::path& dir) {
  std::ifstream in(OverlayFilePath(dir));
  if (!in.is_open()) {
    return json::object();
  }
  try {
    return json::parse(in);
  } catch (const std::exception&) {
    return json::object();
  }
}

}  // namespace

void RegisterDefaultsDiffTests(ImGuiTestEngine* engine) {
  // ================================================================================
  // AC1 — generativity, as a permanent coverage assertion
  // ================================================================================
  {
    // The one-time red/green probe for AC1 (add a field, watch a row appear, remove it again)
    // cannot be committed — a field that exists only to be observed is not a field. This is its
    // permanent proxy: it re-derives the covered set from the SERIALIZER on every run, so a root
    // key that starts being emitted must either land in the exclusion table or show up as rows.
    // A future "special-case this key name" branch inside the walk turns it red.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "ac1_every_serialized_root_key_is_covered");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

      const gui::GuiState state = gui::InitDefaultState();
      const auto rows = gui::BuildDefaultDiffRows(state);
      IM_CHECK(!rows.empty());

      const json root = json::parse(gui::SerializeGuiStateJson(state));
      IM_CHECK(root.is_object());

      std::set<std::string> excluded;
      for (const char* key : gui::kDiffEngineExcludedRootKeys) {
        excluded.insert(key);
      }

      for (auto it = root.begin(); it != root.end(); ++it) {
        const std::string& key = it.key();
        const bool covered = std::any_of(rows.begin(), rows.end(), [&](const gui::DefaultDiffRow& row) {
          return row.key_path == key || row.key_path.rfind(key + ".", 0) == 0;
        });
        if (excluded.count(key) != 0) {
          IM_CHECK(!covered);  // an excluded key must not sneak in through a nested path either
        } else {
          IM_CHECK(covered);
        }
      }

      // Leaf-level spot checks alongside the root-level sweep: the root sweep would still pass if
      // someone taught the walk to stop at "renderer" and emit one opaque row for the whole
      // object, which is exactly the shape this panel must not have.
      IM_CHECK(FindRow(rows, "renderer.lens_type") != nullptr);
      IM_CHECK(FindRow(rows, "renderer.fov") != nullptr);
      IM_CHECK(FindRow(rows, "sun.altitude") != nullptr);
      IM_CHECK(FindRow(rows, "sim.max_hits") != nullptr);
      // The rename cases (field name != JSON key). If the engine ever resolves eligibility by
      // literal field name these are the ~18 rows that would silently disappear.
      IM_CHECK(FindRow(rows, "overlay_grid_alpha") != nullptr);
      IM_CHECK(FindRow(rows, "overlay_horizon_line") != nullptr);
      IM_CHECK(FindRow(rows, "aspect_ratio") != nullptr);
    };
  }

  // ================================================================================
  // AC2 — §2 and §3 partition the row set
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "ac2_sections_partition_the_row_set");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("diff_ac2");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      // Two shapes: an untouched document (§2 empty) and an edited one (§2 populated), so the
      // partition is asserted with each section both empty and non-empty.
      std::vector<gui::GuiState> states = { gui::InitDefaultState(), MakeEditedState() };
      for (const auto& state : states) {
        const auto rows = gui::BuildDefaultDiffRows(state);
        IM_CHECK(!rows.empty());

        std::set<std::string> pending;
        std::set<std::string> other;
        for (const auto& row : rows) {
          if (gui::RowNeedsAdoption(row)) {
            pending.insert(row.key_path);
          } else {
            other.insert(row.key_path);
          }
        }
        // Complete: every row is in exactly one section, and no key path is duplicated anywhere
        // in the row set (which is what "the same key must not appear twice" means concretely).
        IM_CHECK_EQ(pending.size() + other.size(), rows.size());
        std::set<std::string> intersection;
        std::set_intersection(pending.begin(), pending.end(), other.begin(), other.end(),
                              std::inserter(intersection, intersection.begin()));
        IM_CHECK(intersection.empty());
      }

      // Non-vacuous: the edited state really does produce §2 rows, otherwise the partition above
      // would hold trivially with §2 empty in every arm.
      const auto edited_rows = gui::BuildDefaultDiffRows(MakeEditedState());
      const auto pending_count =
          std::count_if(edited_rows.begin(), edited_rows.end(),
                        [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
      IM_CHECK(pending_count > 0);
    };
  }

  // ================================================================================
  // AC3 — namespace 4 never reaches the row set
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "ac3_collection_edits_produce_no_rows");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

      gui::GuiState state = gui::InitDefaultState();
      const std::string before = RowSetDigest(gui::BuildDefaultDiffRows(state));

      // layers: a second layer with its own entry.
      state.layers.push_back(state.layers[0]);
      state.layers[0].probability = 0.25f;
      // crystals: reachable only through the entry that references it (the crystal is serialized
      // inline under layers[].entries[]).
      auto& crystal = state.crystals[state.layers[0].entries[0].crystal_id];
      crystal.height = gui::ShapeDist(crystal.height.center * 2.0f + 1.0f);
      // filters: attach one to the first entry.
      {
        gui::FilterConfig f;
        f.name = "ac3";
        f.SetRaypath(gui::RaypathParams{ "3-5-1" });
        gui::SetFilter(state, state.layers[0].entries[0], f);
      }
      // raypath_color: not serialized at all, but assert it rather than assume it.
      state.raypath_color.emplace_back();

      const std::string after = RowSetDigest(gui::BuildDefaultDiffRows(state));
      IM_CHECK(before == after);

      // Detection power: the same helper DOES notice an ordinary namespace-1 edit, so the
      // equality above is a property of the exclusion, not of a digest that never changes.
      state.bg_alpha = state.bg_alpha + 0.25f;
      IM_CHECK(RowSetDigest(gui::BuildDefaultDiffRows(state)) != before);
    };
  }

  // ================================================================================
  // AC4 — an array is one row
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "ac4_array_edit_produces_exactly_one_row");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

      gui::GuiState state = gui::InitDefaultState();
      const auto baseline = gui::BuildDefaultDiffRows(state);
      const auto baseline_pending = std::count_if(
          baseline.begin(), baseline.end(), [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });

      state.renderer.background[1] = state.renderer.background[1] + 0.5f;
      const auto rows = gui::BuildDefaultDiffRows(state);
      const auto pending = std::count_if(rows.begin(), rows.end(),
                                         [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
      IM_CHECK_EQ(pending, baseline_pending + 1);

      const gui::DefaultDiffRow* row = FindRow(rows, "renderer.background");
      IM_CHECK(row != nullptr);
      IM_CHECK(gui::RowNeedsAdoption(*row));
      IM_CHECK(row->current_value.is_array());
      IM_CHECK_EQ(row->current_value.size(), static_cast<size_t>(3));
      // Explicitly NOT per-element rows — one color edit must not print three lines.
      IM_CHECK(FindRow(rows, "renderer.background.0") == nullptr);
      IM_CHECK(FindRow(rows, "renderer.background.1") == nullptr);
    };
  }

  // ================================================================================
  // AC5 — effective default = factory + saved override; enums read as names
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "ac5_effective_default_layers_the_saved_override");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("diff_ac5");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      gui::GuiState state = gui::InitDefaultState();
      state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;

      // State 1: no override file. The effective default is the factory value, so the edited
      // lens type is a §2 row.
      {
        const auto rows = gui::BuildDefaultDiffRows(state);
        const gui::DefaultDiffRow* row = FindRow(rows, "renderer.lens_type");
        IM_CHECK(row != nullptr);
        IM_CHECK(gui::RowNeedsAdoption(*row));
        IM_CHECK_STR_EQ(row->default_value.get<std::string>().c_str(), "linear");
        IM_CHECK(!row->has_saved_override);
      }

      // State 2: the same value saved as a personal default. Same current value, different
      // effective default -> the row moves to §3 and is attributed to the user.
      {
        json doc;
        doc["renderer"]["lens_type"] = "fisheye_equal_area";
        IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

        const auto rows = gui::BuildDefaultDiffRows(state);
        const gui::DefaultDiffRow* row = FindRow(rows, "renderer.lens_type");
        IM_CHECK(row != nullptr);
        IM_CHECK(!gui::RowNeedsAdoption(*row));
        IM_CHECK_STR_EQ(row->default_value.get<std::string>().c_str(), "fisheye_equal_area");
        IM_CHECK(row->has_saved_override);
      }

      // Enum readability, one row per name table (kLensTypeJsonNames / kVisibleJsonNames /
      // kAspectPresetJsonNames): the panel must show "fisheye_equal_area", never "1".
      {
        const auto rows = gui::BuildDefaultDiffRows(state);
        for (const char* key : { "renderer.lens_type", "renderer.visible", "aspect_ratio" }) {
          const gui::DefaultDiffRow* row = FindRow(rows, key);
          IM_CHECK(row != nullptr);
          IM_CHECK(row->current_value.is_string());
          IM_CHECK(!gui::FormatDiffValue(row->current_value).empty());
        }
      }
    };
  }

  {
    // "Source" is presence in the override document, not "differs from factory". A default the
    // user deliberately saved that happens to equal the factory value is still theirs — otherwise
    // they can neither see nor revert it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "ac5_source_tracks_the_file_not_the_value");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("diff_source");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      const gui::GuiState factory = gui::InitDefaultState();
      json doc;
      doc["bg_alpha"] = gui::GuiState{}.bg_alpha;  // saved, and identical to factory
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      const auto rows = gui::BuildDefaultDiffRows(factory);
      const gui::DefaultDiffRow* saved = FindRow(rows, "bg_alpha");
      IM_CHECK(saved != nullptr);
      IM_CHECK(!gui::RowNeedsAdoption(*saved));
      IM_CHECK(saved->has_saved_override);

      const gui::DefaultDiffRow* untouched = FindRow(rows, "bg_show");
      IM_CHECK(untouched != nullptr);
      IM_CHECK(!untouched->has_saved_override);
    };
  }

  {
    // Display formatting is decoupled from comparison: two values that FORMAT identically at
    // %.6g must still be a §2 row. Otherwise a rounding choice in the panel would decide what
    // gets written to disk.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "format_is_decoupled_from_comparison");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

      gui::GuiState state = gui::InitDefaultState();
      // 1e-7 relative difference: identical under %.6g, different as doubles.
      state.renderer.fov = state.renderer.fov * (1.0f + 1e-7f);
      const auto rows = gui::BuildDefaultDiffRows(state);
      const gui::DefaultDiffRow* row = FindRow(rows, "renderer.fov");
      IM_CHECK(row != nullptr);
      if (row->current_value != row->default_value) {
        // The float really did change (guards against the multiplication being a no-op at this
        // magnitude, which would make the assertion below vacuous).
        IM_CHECK(gui::RowNeedsAdoption(*row));
        IM_CHECK_STR_EQ(gui::FormatDiffValue(row->current_value).c_str(),
                        gui::FormatDiffValue(row->default_value).c_str());
      }

      // The formatter's own contract, checked directly.
      IM_CHECK_STR_EQ(gui::FormatDiffValue(json(true)).c_str(), "true");
      IM_CHECK_STR_EQ(gui::FormatDiffValue(json("fisheye_equal_area")).c_str(), "fisheye_equal_area");
      IM_CHECK_STR_EQ(gui::FormatDiffValue(json(0.25)).c_str(), "0.25");
      IM_CHECK_STR_EQ(gui::FormatDiffValue(json()).c_str(), "(absent)");
    };
  }

  // ================================================================================
  // Write-back contract (the engine half of AC7 / AC8)
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "write_back_is_surgical_and_keeps_presets");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("diff_write");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      // A pre-existing document with a preset-library subtree this panel does not own.
      json existing;
      existing["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
      existing["bg_show"] = true;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, existing));

      gui::GuiState state = MakeEditedState();
      IM_CHECK(gui::SaveAcceptedDefaults({ "renderer.lens_type", "bg_alpha" }, state));

      json saved = ReadOverlayFile(dir);
      IM_CHECK_STR_EQ(saved["renderer"]["lens_type"].get<std::string>().c_str(), "fisheye_equal_area");
      IM_CHECK_EQ(saved["bg_alpha"].get<float>(), 0.42f);
      // Untouched keys of both kinds survive: the sibling namespace-2 subtree AND a namespace-1
      // key this call did not mention.
      IM_CHECK(saved.contains("presets"));
      IM_CHECK_EQ(saved["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
      IM_CHECK_EQ(saved["bg_show"].get<bool>(), true);
      // A key that was never adopted must not be written just because it was serialized.
      IM_CHECK(!saved.contains("sun"));

      // Revert removes exactly one key and prunes the object it emptied.
      IM_CHECK(gui::RevertOneDefault("renderer.lens_type"));
      saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("renderer"));
      IM_CHECK(saved.contains("bg_alpha"));
      IM_CHECK(saved.contains("presets"));

      // Reset all drops every GuiState key and keeps the preset library.
      IM_CHECK(gui::ResetAllDefaults());
      saved = ReadOverlayFile(dir);
      IM_CHECK(!saved.contains("bg_alpha"));
      IM_CHECK(!saved.contains("bg_show"));
      IM_CHECK(saved.contains("presets"));
      IM_CHECK_EQ(saved["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
      IM_CHECK_EQ(saved.size(), static_cast<size_t>(1));
    };
  }

  {
    // No writable config directory (--no-user-config, or an OS that gave us nowhere to write):
    // every write reports failure instead of silently claiming success, and nothing is created.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "defaults_diff", "write_back_reports_failure_without_a_directory");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

      const gui::GuiState state = MakeEditedState();
      IM_CHECK(!gui::SaveAcceptedDefaults({ "bg_alpha" }, state));
      IM_CHECK(!gui::RevertOneDefault("bg_alpha"));
      IM_CHECK(!gui::ResetAllDefaults());

      // And the row set still builds — the panel must remain usable (read-only) in this state.
      IM_CHECK(!gui::BuildDefaultDiffRows(state).empty());
    };
  }
}
