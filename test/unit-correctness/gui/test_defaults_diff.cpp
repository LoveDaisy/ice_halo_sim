// Defaults diff-engine tests — the pure-logic half of the defaults panel (src/gui/defaults_diff.*).
//
// Nothing here renders a frame: every case calls the engine directly and asserts on the returned
// row set or on the override file it wrote. They live in gui_unit_test rather than in
// unit_correctness_test because the engine links against file_io.cpp / user_defaults.cpp
// (lumice_gui_obj), which unit_correctness_test does not pull in — the same Part1/Part2 split
// user_defaults.hpp documents. gui_unit_test links lumice_gui_obj with no window, no GL context
// and no ImGui test engine, so these cases need none of what gui_test exists to provide.
//
// Coverage:
//   AC1 — generativity: every serialized root key is either excluded by name or produces rows,
//         so a newly serialized GuiState field appears in the panel with no engine change
//   AC2 — RowNeedsAdoption is total and two-valued over one row set (no key in both halves, no
//         key missing, no duplicates) — the property the panel's opening checkbox state rests on
//   AC3 — namespace 4 (layers / crystals / filters / raypath_color) never reaches the row set
//   AC4 — an array is ONE row, not one row per element
//   AC5 — "effective default" layers the saved override over factory, and enums read as names
//   plus factory_value (the LITERAL factory value, which is NOT the effective default once a key
//   has been saved) and ApplyCheckedRowsToDoc (the panel's one write-back rule)

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state_tiers.hpp"
#include "gui/user_defaults.hpp"
#include "support/user_defaults_test_env.hpp"

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

// ================================================================================
// AC1 — generativity, as a permanent coverage assertion
// ================================================================================

// The one-time red/green probe for AC1 (add a field, watch a row appear, remove it again)
// cannot be committed — a field that exists only to be observed is not a field. This is its
// permanent proxy: it re-derives the covered set from the SERIALIZER on every run, so a root
// key that starts being emitted must either land in the exclusion table or show up as rows.
// A future "special-case this key name" branch inside the walk turns it red.
TEST(DefaultsDiff, ac1_every_serialized_root_key_is_covered) {
  ResetUserDefaultsChannels();
  ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

  const gui::GuiState state = gui::InitDefaultState();
  const auto rows = gui::BuildDefaultDiffRows(state);
  EXPECT_TRUE(!rows.empty());

  const json root = json::parse(gui::SerializeGuiStateJson(state));
  EXPECT_TRUE(root.is_object());

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
      EXPECT_TRUE(!covered);  // an excluded key must not sneak in through a nested path either
    } else {
      EXPECT_TRUE(covered);
    }
  }

  // Leaf-level spot checks alongside the root-level sweep: the root sweep would still pass if
  // someone taught the walk to stop at "renderer" and emit one opaque row for the whole
  // object, which is exactly the shape this panel must not have.
  EXPECT_TRUE(FindRow(rows, "renderer.lens_type") != nullptr);
  EXPECT_TRUE(FindRow(rows, "renderer.fov") != nullptr);
  EXPECT_TRUE(FindRow(rows, "sun.altitude") != nullptr);
  EXPECT_TRUE(FindRow(rows, "sim.max_hits") != nullptr);
  // The rename cases (field name != JSON key). If the engine ever resolves eligibility by
  // literal field name these are the ~18 rows that would silently disappear.
  EXPECT_TRUE(FindRow(rows, "overlay_grid_alpha") != nullptr);
  EXPECT_TRUE(FindRow(rows, "overlay_horizon_line") != nullptr);
  EXPECT_TRUE(FindRow(rows, "aspect_ratio") != nullptr);
}

// ================================================================================
// AC2 — RowNeedsAdoption partitions the row set
//
// Written when the predicate decided which of two SECTIONS a row lived in. The sections are
// gone; the predicate is not — it is one half of the OR that decides a row's opening checkbox
// state — and this is still the property that guarantees the panel asks it of every row exactly
// once. Kept unchanged on purpose: it is a statement about the engine, not about the layout.
// ================================================================================
TEST(DefaultsDiff, ac2_sections_partition_the_row_set) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("diff_ac2");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  // Two shapes: an untouched document (§2 empty) and an edited one (§2 populated), so the
  // partition is asserted with each section both empty and non-empty.
  std::vector<gui::GuiState> states = { gui::InitDefaultState(), MakeEditedState() };
  for (const auto& state : states) {
    const auto rows = gui::BuildDefaultDiffRows(state);
    EXPECT_TRUE(!rows.empty());

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
    EXPECT_EQ(pending.size() + other.size(), rows.size());
    std::set<std::string> intersection;
    std::set_intersection(pending.begin(), pending.end(), other.begin(), other.end(),
                          std::inserter(intersection, intersection.begin()));
    EXPECT_TRUE(intersection.empty());
  }

  // Non-vacuous: the edited state really does produce §2 rows, otherwise the partition above
  // would hold trivially with §2 empty in every arm.
  const auto edited_rows = gui::BuildDefaultDiffRows(MakeEditedState());
  const auto pending_count = std::count_if(edited_rows.begin(), edited_rows.end(),
                                           [](const gui::DefaultDiffRow& row) { return gui::RowNeedsAdoption(row); });
  EXPECT_TRUE(pending_count > 0);
}

// ================================================================================
// AC3 — namespace 4 never reaches the row set
// ================================================================================
TEST(DefaultsDiff, ac3_collection_edits_produce_no_rows) {
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
  EXPECT_TRUE(before == after);

  // Detection power: the same helper DOES notice an ordinary namespace-1 edit, so the
  // equality above is a property of the exclusion, not of a digest that never changes.
  state.bg_alpha = state.bg_alpha + 0.25f;
  EXPECT_TRUE(RowSetDigest(gui::BuildDefaultDiffRows(state)) != before);
}

// ================================================================================
// AC4 — an array is one row
// ================================================================================
TEST(DefaultsDiff, ac4_array_edit_produces_exactly_one_row) {
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
  EXPECT_EQ(pending, baseline_pending + 1);

  const gui::DefaultDiffRow* row = FindRow(rows, "renderer.background");
  ASSERT_TRUE(row != nullptr);
  EXPECT_TRUE(gui::RowNeedsAdoption(*row));
  EXPECT_TRUE(row->current_value.is_array());
  EXPECT_EQ(row->current_value.size(), static_cast<size_t>(3));
  // Explicitly NOT per-element rows — one color edit must not print three lines.
  EXPECT_TRUE(FindRow(rows, "renderer.background.0") == nullptr);
  EXPECT_TRUE(FindRow(rows, "renderer.background.1") == nullptr);
}

// ================================================================================
// AC5 — effective default = factory + saved override; enums read as names
// ================================================================================
TEST(DefaultsDiff, ac5_effective_default_layers_the_saved_override) {
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
    ASSERT_TRUE(row != nullptr);
    EXPECT_TRUE(gui::RowNeedsAdoption(*row));
    EXPECT_STREQ(row->default_value.get<std::string>().c_str(), "linear");
    EXPECT_TRUE(!row->has_saved_override);
  }

  // State 2: the same value saved as a personal default. Same current value, different
  // effective default -> the row moves to §3 and is attributed to the user.
  {
    json doc;
    doc["renderer"]["lens_type"] = "fisheye_equal_area";
    EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

    const auto rows = gui::BuildDefaultDiffRows(state);
    const gui::DefaultDiffRow* row = FindRow(rows, "renderer.lens_type");
    ASSERT_TRUE(row != nullptr);
    EXPECT_TRUE(!gui::RowNeedsAdoption(*row));
    EXPECT_STREQ(row->default_value.get<std::string>().c_str(), "fisheye_equal_area");
    EXPECT_TRUE(row->has_saved_override);
  }

  // Enum readability, one row per name table (kLensTypeJsonNames / kVisibleJsonNames /
  // kAspectPresetJsonNames): the panel must show "fisheye_equal_area", never "1".
  {
    const auto rows = gui::BuildDefaultDiffRows(state);
    for (const char* key : { "renderer.lens_type", "renderer.visible", "aspect_ratio" }) {
      const gui::DefaultDiffRow* row = FindRow(rows, key);
      ASSERT_TRUE(row != nullptr);
      EXPECT_TRUE(row->current_value.is_string());
      EXPECT_TRUE(!gui::FormatDiffValue(row->current_value).empty());
    }
  }
}

// "Source" is presence in the override document, not "differs from factory". A default the
// user deliberately saved that happens to equal the factory value is still theirs — otherwise
// they can neither see nor revert it.
TEST(DefaultsDiff, ac5_source_tracks_the_file_not_the_value) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("diff_source");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  const gui::GuiState factory = gui::InitDefaultState();
  json doc;
  doc["bg_alpha"] = gui::GuiState{}.bg_alpha;  // saved, and identical to factory
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const auto rows = gui::BuildDefaultDiffRows(factory);
  const gui::DefaultDiffRow* saved = FindRow(rows, "bg_alpha");
  ASSERT_TRUE(saved != nullptr);
  EXPECT_TRUE(!gui::RowNeedsAdoption(*saved));
  EXPECT_TRUE(saved->has_saved_override);

  const gui::DefaultDiffRow* untouched = FindRow(rows, "bg_show");
  ASSERT_TRUE(untouched != nullptr);
  EXPECT_TRUE(!untouched->has_saved_override);
}

// Display formatting is decoupled from comparison: two values that FORMAT identically at
// %.6g must still be a §2 row. Otherwise a rounding choice in the panel would decide what
// gets written to disk.
TEST(DefaultsDiff, format_is_decoupled_from_comparison) {
  ResetUserDefaultsChannels();
  ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

  gui::GuiState state = gui::InitDefaultState();
  // 1e-7 relative difference: identical under %.6g, different as doubles.
  state.renderer.fov = state.renderer.fov * (1.0f + 1e-7f);
  const auto rows = gui::BuildDefaultDiffRows(state);
  const gui::DefaultDiffRow* row = FindRow(rows, "renderer.fov");
  ASSERT_TRUE(row != nullptr);
  if (row->current_value != row->default_value) {
    // The float really did change (guards against the multiplication being a no-op at this
    // magnitude, which would make the assertion below vacuous).
    EXPECT_TRUE(gui::RowNeedsAdoption(*row));
    EXPECT_STREQ(gui::FormatDiffValue(row->current_value).c_str(), gui::FormatDiffValue(row->default_value).c_str());
  }

  // The formatter's own contract, checked directly.
  EXPECT_STREQ(gui::FormatDiffValue(json(true)).c_str(), "true");
  EXPECT_STREQ(gui::FormatDiffValue(json("fisheye_equal_area")).c_str(), "fisheye_equal_area");
  EXPECT_STREQ(gui::FormatDiffValue(json(0.25)).c_str(), "0.25");
  EXPECT_STREQ(gui::FormatDiffValue(json()).c_str(), "(absent)");
  // An array formats element-wise under the same rule, not through dump(): a float color
  // widened to double comes out of dump() as "[0.800000011920929, ...]", which reads as if the
  // extra digits were the setting.
  EXPECT_STREQ(gui::FormatDiffValue(json::array({ 0.800000011920929, 0.2, 1.0 })).c_str(), "[0.8, 0.2, 1]");
}

// factory_value vs default_value. They are equal until the key is saved, and that is exactly
// why the panel cannot use default_value for its "differs from factory" filter: once a
// non-factory default IS saved, default_value becomes the saved value, and the comparison
// would report "same as factory" for the one row the user definitely customised.
TEST(DefaultsDiff, factory_value_is_not_the_effective_default) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("diff_factory_value");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  const float kFactoryAlpha = gui::GuiState{}.bg_alpha;
  const float kSavedAlpha = kFactoryAlpha + 0.25f;

  // State 1: nothing saved. The two agree, which is the case that makes the confusion possible.
  {
    gui::GuiState state = gui::InitDefaultState();
    const auto rows = gui::BuildDefaultDiffRows(state);
    const gui::DefaultDiffRow* row = FindRow(rows, "bg_alpha");
    ASSERT_TRUE(row != nullptr);
    EXPECT_TRUE(row->factory_value == row->default_value);
    EXPECT_EQ(row->factory_value.get<float>(), kFactoryAlpha);
  }

  // State 2: a non-factory default saved, and the document sitting on it. The effective
  // default follows the file; factory_value does not move.
  {
    json doc;
    doc["bg_alpha"] = kSavedAlpha;
    EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

    gui::GuiState state = gui::InitDefaultState();
    state.bg_alpha = kSavedAlpha;
    const auto rows = gui::BuildDefaultDiffRows(state);
    const gui::DefaultDiffRow* row = FindRow(rows, "bg_alpha");
    ASSERT_TRUE(row != nullptr);
    EXPECT_EQ(row->default_value.get<float>(), kSavedAlpha);
    EXPECT_EQ(row->factory_value.get<float>(), kFactoryAlpha);
    EXPECT_TRUE(row->factory_value != row->default_value);
    // The two filter questions, side by side on ONE row, answering differently: the value
    // differs from factory, and the row does not need adopting (it already matches the
    // effective default). A panel that used default_value for both would say "no" twice.
    EXPECT_TRUE(row->current_value != row->factory_value);
    EXPECT_TRUE(!gui::RowNeedsAdoption(*row));
  }
}

// ================================================================================
// The panel's write-back rule: one pass over the row set, driven by the checkbox state
// ================================================================================
TEST(DefaultsDiff, apply_checked_rows_adopts_and_removes_in_one_pass) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("diff_checked_rows");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  // A document holding one key that will be un-checked, plus a preset subtree the rule must
  // not reach (it produces no row, so it is never named).
  json doc;
  doc["bg_show"] = false;
  doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = MakeEditedState();
  const auto rows = gui::BuildDefaultDiffRows(state, doc);
  EXPECT_TRUE(!rows.empty());
  EXPECT_TRUE(FindRow(rows, "bg_show") != nullptr);
  EXPECT_TRUE(FindRow(rows, "renderer.fov") != nullptr);

  // Checked: one key that is NOT in the document. Everything else — including bg_show, which
  // is — is left out of the set, i.e. un-checked.
  json next = doc;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(next, rows, { "renderer.fov" }, state));

  EXPECT_TRUE(next.contains("renderer"));
  EXPECT_EQ(next["renderer"]["fov"].get<float>(), state.renderer.fov);
  EXPECT_TRUE(!next.contains("bg_show"));  // un-checked -> removed, in the same pass
  // Untouched by construction rather than by a special case: no row names it.
  EXPECT_EQ(next["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
  // And nothing else was adopted just because it was serialized.
  EXPECT_TRUE(!next.contains("sun"));
  EXPECT_TRUE(!next.contains("bg_alpha"));

  // Un-checking EVERYTHING is Reset all: every GuiState key goes, the sibling subtree stays.
  json emptied = doc;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(emptied, rows, {}, state));
  EXPECT_TRUE(!emptied.contains("bg_show"));
  EXPECT_EQ(emptied.size(), static_cast<size_t>(1));
  EXPECT_TRUE(emptied.contains("presets"));
}

// A checked key that the CURRENT state does not serialize (an optional key) is removed rather
// than written as null — "adopt what I have now" reads the same either way, and a null in the
// file would be read back as a value.
TEST(DefaultsDiff, apply_checked_rows_removes_an_absent_optional_key) {
  ResetUserDefaultsChannels();
  ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

  const gui::GuiState state = gui::InitDefaultState();
  const json current = json::parse(gui::SerializeGuiStateJson(state));
  // The premise, asserted rather than assumed: this key really is one the default state does
  // not serialize. If it ever becomes unconditional, this line says so instead of the case
  // below quietly testing nothing.
  EXPECT_TRUE(!current.contains("sun") || !current["sun"].contains("custom_spectrum"));

  // The row is hand-built rather than taken from BuildDefaultDiffRows: whether a row for an
  // absent key EXISTS depends on the overlay loader putting the key back into the effective
  // default, which is a different mechanism from the one under test here. ApplyCheckedRowsToDoc
  // takes the rows and the current state as separate inputs, so the branch can be driven
  // directly.
  gui::DefaultDiffRow row;
  row.key_path = "sun.custom_spectrum";
  const std::vector<gui::DefaultDiffRow> rows = { row };

  // The emptied parent is pruned with the key...
  {
    json doc;
    doc["sun"]["custom_spectrum"] = json::array({ 400.0, 1.0 });
    EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(doc, rows, { "sun.custom_spectrum" }, state));
    EXPECT_TRUE(!doc.contains("sun"));
  }
  // ...but a parent that still holds something is not.
  {
    json doc;
    doc["sun"]["custom_spectrum"] = json::array({ 400.0, 1.0 });
    doc["sun"]["altitude"] = 33.0f;
    EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(doc, rows, { "sun.custom_spectrum" }, state));
    EXPECT_TRUE(doc.contains("sun"));
    EXPECT_TRUE(!doc["sun"].contains("custom_spectrum"));
    EXPECT_EQ(doc["sun"]["altitude"].get<float>(), 33.0f);
  }
}

// ================================================================================
// Write-back contract, asserted on the pair the panel's Save actually calls
// (ApplyCheckedRowsToDoc -> WriteActiveOverlayDoc)
// ================================================================================

// Surgical writes, asserted FILE-side: what the panel commits must edit the keys its rows name
// and leave every other key of the document exactly where it was.
//
// The sibling "presets" subtree is the first-order case (it belongs to the preset library, and a
// wholesale rewrite here would delete a user's preset overrides), but it is not the only one: a
// key written by an OLDER version, whose GuiState field has since been removed, produces no row
// either. Both survive for the same structural reason — the rule only ever touches keys a row
// names — so both are asserted here, the second one because it is the case no other test covers
// and the one that would silently start disappearing if the rule were ever rewritten as
// "keep the keys we know about".
//
// Runs the pair together, through the file, rather than asserting on the in-memory document
// alone: apply_checked_rows_adopts_and_removes_in_one_pass already pins the pure rule, and what
// is added here is that the document survives the trip to disk unchanged.
TEST(DefaultsDiff, live_write_back_is_surgical_and_keeps_keys_no_row_names) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("diff_live_write");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  // A pre-existing document with a preset-library subtree this panel does not own, plus a
  // GuiState key that IS in today's schema (so it produces a row) and one that is not.
  json existing;
  existing["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
  existing["bg_show"] = true;
  existing["a_key_no_current_field_serializes"] = 7;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, existing));

  const gui::GuiState state = MakeEditedState();
  const auto rows = gui::BuildDefaultDiffRows(state, existing);
  // The premise both halves of this case rest on, asserted rather than assumed: one of the two
  // extra keys is named by a row and the other is not.
  ASSERT_TRUE(FindRow(rows, "bg_show") != nullptr);
  ASSERT_TRUE(FindRow(rows, "a_key_no_current_field_serializes") == nullptr);

  json next = existing;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(next, rows, { "renderer.lens_type", "bg_alpha" }, state));
  EXPECT_TRUE(gui::WriteActiveOverlayDoc(next));

  const json saved = ReadOverlayFile(dir);
  EXPECT_STREQ(saved["renderer"]["lens_type"].get<std::string>().c_str(), "fisheye_equal_area");
  EXPECT_EQ(saved["bg_alpha"].get<float>(), 0.42f);
  // Untouched keys of both kinds survive: the sibling namespace-2 subtree AND the key no row
  // names.
  EXPECT_TRUE(saved.contains("presets"));
  EXPECT_EQ(saved["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
  EXPECT_EQ(saved.value("a_key_no_current_field_serializes", 0), 7);
  // bg_show, by contrast, HAS a row and was left un-checked, so the same pass removes it. That
  // is the line separating "not mentioned" from "mentioned and declined".
  EXPECT_TRUE(!saved.contains("bg_show"));
  // A key that was never checked must not be written just because it was serialized.
  EXPECT_TRUE(!saved.contains("sun"));
}

// No writable config directory (--no-user-config, or an OS that gave us nowhere to write):
// the write reports failure instead of silently claiming success, and nothing is created.
TEST(DefaultsDiff, live_write_back_reports_failure_without_a_directory) {
  ResetUserDefaultsChannels();
  ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);

  const gui::GuiState state = MakeEditedState();
  json doc;
  doc["bg_alpha"] = state.bg_alpha;
  EXPECT_TRUE(!gui::WriteActiveOverlayDoc(doc));
  // Nothing was created to write into, so the read side still answers "nothing saved" rather
  // than reporting a document this call invented.
  EXPECT_TRUE(gui::ReadActiveOverlayDoc().empty());

  // And the row set still builds — the panel must remain usable (read-only) in this state.
  EXPECT_TRUE(!gui::BuildDefaultDiffRows(state).empty());
}

// ================================================================================
// Key-path handling gaps carried over from 405.4 (both surfaced by that task's review as
// Minor, both on mechanisms this task's preset key paths lean on harder).
// ================================================================================

// The dot is the path separator, so a key whose own NAME contains a dot would be split into
// two tokens and written to the wrong place — silently, since both halves are valid object
// keys. Nothing in the code enforced the "no default-able key contains a dot" premise; it held
// only because someone had checked the field names by eye. This asserts it over the live
// registry, so a future field named "foo.bar" turns red here instead of corrupting a file.
TEST(DefaultsDiff, no_default_eligible_key_contains_the_path_separator) {
  int checked = 0;
  for (const auto& entry : gui::kFieldTierTable) {
    const auto verdict = gui::ResolveDefaultEligibility(entry.name);
    if (verdict.eligibility != gui::DefaultEligibility::kEligible) {
      continue;
    }
    ++checked;
    EXPECT_TRUE(std::string_view(entry.name).find('.') == std::string_view::npos);
  }
  // The registry is the source; an empty loop would make the assertion above vacuous.
  EXPECT_GT(checked, 0);

  // Root names are only half of it: a NESTED key with a dot ("renderer": {"a.b": 1}) would
  // split just as wrongly. Checked against the serialized tree rather than against the
  // emitted key paths, because once a dotted name is joined into "renderer.a.b" the damage is
  // no longer visible in the string — the two readings are indistinguishable by then.
  int keys_seen = 0;
  const std::function<void(const json&)> check_keys = [&](const json& node) {
    if (!node.is_object()) {
      return;
    }
    for (auto it = node.begin(); it != node.end(); ++it) {
      ++keys_seen;
      EXPECT_TRUE(it.key().find('.') == std::string::npos);
      check_keys(*it);
    }
  };
  check_keys(json::parse(gui::SerializeGuiStateJson(MakeEditedState())));
  EXPECT_GT(keys_seen, 0);

  // The preset library's own key path (presets.axis.<name>.zenith_std) deepens the reliance
  // on this rule, so its components are held to it too.
  for (const auto& entry : gui::kAxisPresets) {
    if (entry.override_json_name != nullptr) {
      EXPECT_TRUE(std::string_view(entry.override_json_name).find('.') == std::string_view::npos);
    }
  }
}

// A hand-edited file can put a scalar where a group of settings belongs ("renderer": 3).
// Honoring the user's write then means replacing that node — which DISCARDS whatever was
// there. That is a data loss, and it used to happen with no counter, no notice and no log:
// the same silent-discard family the scrum's I3 invariant exists to close.
TEST(DefaultsDiff, replacing_a_non_object_path_node_is_not_silent) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("setbypath_notice");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  json doc;
  doc["renderer"] = 3;  // a scalar where an object belongs
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = MakeEditedState();
  // Built BEFORE the drain below, deliberately: this is the panel's opening snapshot, and
  // overlaying a document whose "renderer" is a scalar is itself a degradation the loader is
  // entitled to report. Draining after it keeps the assertions below about the WRITE and
  // nothing else.
  const auto rows = gui::BuildDefaultDiffRows(state, doc);
  gui::TakeUserDefaultsDowngradeCount();
  gui::TakeUserDefaultsDowngradeNotices();

  json next = doc;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(next, rows, { "renderer.fov" }, state));
  EXPECT_TRUE(gui::WriteActiveOverlayDoc(next));

  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
  const auto notices = gui::TakeUserDefaultsDowngradeNotices();
  EXPECT_EQ(notices.size(), static_cast<size_t>(1));
  EXPECT_TRUE(notices[0].find("renderer") != std::string::npos);

  // The write still landed — the notice reports the loss, it does not refuse the edit.
  std::ifstream in(dir / gui::kUserDefaultsFileName);
  EXPECT_TRUE(in.is_open());
  const json saved = json::parse(in);
  EXPECT_TRUE(saved["renderer"].is_object());
  EXPECT_TRUE(saved["renderer"].contains("fov"));

  // Both counters are consumed on read, like every other channel in this family.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
  EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty());
}

// The negative half of the case above: a well-formed document must produce NO notice. Without
// this, a notice that fired on every write would still pass the test above while burying the
// real ones in noise.
TEST(DefaultsDiff, an_ordinary_write_reports_no_downgrade) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("setbypath_quiet");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  const gui::GuiState state = MakeEditedState();
  const auto rows = gui::BuildDefaultDiffRows(state, json::object());
  gui::TakeUserDefaultsDowngradeCount();
  gui::TakeUserDefaultsDowngradeNotices();

  json next = json::object();
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(next, rows, { "renderer.fov", "bg_alpha" }, state));
  EXPECT_TRUE(gui::WriteActiveOverlayDoc(next));
  // The premise: the write really did create the nested object the notice would have described,
  // so a silent zero here is "no loss to report" rather than "no write happened".
  EXPECT_TRUE(next["renderer"].is_object());
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
  EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty());
}
