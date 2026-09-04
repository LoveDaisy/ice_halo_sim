// Defaults diff-engine tests — the pure-logic half of the defaults panel (src/gui/defaults_diff.*),
// together with the two Settings-panel cases that ask about the panel's DATA rather than its UI
// (which keys the field-editor registry covers, and when the row tint predicate fires). Their 40
// widget-driving siblings stay in test/gui/functional/test_defaults_panel.cpp.
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
//   has been saved), ApplyCheckedRowsToDoc (the panel's one write-back rule), the field-editor
//   registry's coverage of the real row set, and RowWouldChangeOnSave

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_constants.hpp"
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

// Every case starts from drained channels and from an override source of its own: the downgrade
// counter and the notice list are consumed-on-read, so a leftover from a previous case reads as
// this one's own degradation, and a case left pointing at the machine's real config directory
// would answer with whatever personal defaults that machine has saved.
class DefaultsDiff : public ::testing::Test {
 protected:
  void SetUp() override { ResetUserDefaultsChannels(); }

  // No override file can exist, and none can be written.
  void NoUserConfig() { guard_.emplace(gui::UserConfigSource::kDisabled); }

  // A fresh, empty override directory this case owns, optionally seeded with a document.
  const std::filesystem::path& FreshUserConfig(const char* tag, const json* seed = nullptr) {
    dir_ = FreshOverlayDir(tag);
    guard_.emplace(gui::UserConfigSource::kExplicitDir, dir_);
    if (seed != nullptr) {
      EXPECT_TRUE(gui::WriteUserDefaultsFile(dir_, *seed));
    }
    return dir_;
  }

 private:
  std::filesystem::path dir_;
  std::optional<ScopedUserConfigSource> guard_;
};

}  // namespace

// ================================================================================
// AC1 — generativity, as a permanent coverage assertion
// ================================================================================

// The one-time red/green probe for AC1 (add a field, watch a row appear, remove it again)
// cannot be committed — a field that exists only to be observed is not a field. This is its
// permanent proxy: it re-derives the covered set from the SERIALIZER on every run, so a root
// key that starts being emitted must either land in the exclusion table or show up as rows.
// A future "special-case this key name" branch inside the walk turns it red.
//
// The leaf level is covered by registry_covers_every_row below rather than by spot checks here: it
// names every leaf the walk must produce, so a walk taught to stop at "renderer" and emit one
// opaque row for the whole object — the shape this panel must not have — fails there.
TEST_F(DefaultsDiff, ac1_every_serialized_root_key_is_covered) {
  NoUserConfig();

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
}

// ================================================================================
// AC2 — RowNeedsAdoption partitions the row set
//
// Written when the predicate decided which of two SECTIONS a row lived in. The sections are
// gone; the predicate is not — it is one half of the OR that decides a row's opening checkbox
// state — and this is still the property that guarantees the panel asks it of every row exactly
// once. Kept unchanged on purpose: it is a statement about the engine, not about the layout.
// ================================================================================
TEST_F(DefaultsDiff, ac2_sections_partition_the_row_set) {
  FreshUserConfig("diff_ac2");

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
TEST_F(DefaultsDiff, ac3_collection_edits_produce_no_rows) {
  NoUserConfig();

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
TEST_F(DefaultsDiff, ac4_array_edit_produces_exactly_one_row) {
  NoUserConfig();

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
// AC5 — effective default = factory + saved override, and how it differs from the factory value
//
// Three questions the panel asks of a row, folded into one case because they are three readings of
// ONE state transition (a key saved) and split apart they were three copies of the same staging:
//   - the effective default follows the file, so an already-saved value is not pending adoption
//   - "source" is presence in the document, not "differs from factory" — a deliberately saved
//     value equal to factory is still the user's, or they can neither see nor revert it
//   - factory_value does NOT move with the save, which is why the panel cannot use default_value
//     for its "differs from factory" filter: that comparison would report "same as factory" for
//     the one row the user definitely customised
// ================================================================================
// Opening the Settings panel serializes the CURRENT state, so every lens type the combo can reach
// has to survive that walk — including the three highest enum values, which for a time indexed past
// the end of the writer's name table and took the process down before any row was built.
//
// All eleven are probed rather than only those three: the table is indexed by the enum value, so a
// wrong entry anywhere lands a lens on its neighbour's name, and this path reports that as a
// perfectly healthy row.
TEST_F(DefaultsDiff, ac5_every_lens_type_survives_the_settings_panel_diff_walk) {
  FreshUserConfig("diff_lens_types");

  struct Row {
    int value;
    const char* spelling;
  };
  constexpr Row kRows[] = {
    Row{ gui::kLensTypeLinear, "linear" },
    Row{ gui::kLensTypeFisheyeEqualArea, "fisheye_equal_area" },
    Row{ gui::kLensTypeFisheyeEquidist, "fisheye_equidistant" },
    Row{ gui::kLensTypeFisheyeStereographic, "fisheye_stereographic" },
    Row{ gui::kLensTypeDualFisheyeEqualArea, "dual_fisheye_equal_area" },
    Row{ gui::kLensTypeDualFisheyeEquidist, "dual_fisheye_equidistant" },
    Row{ gui::kLensTypeDualFisheyeStereographic, "dual_fisheye_stereographic" },
    Row{ gui::kLensTypeRectangular, "rectangular" },
    Row{ gui::kLensTypeFisheyeOrthographic, "fisheye_orthographic" },
    Row{ gui::kLensTypeDualFisheyeOrthographic, "dual_fisheye_orthographic" },
    Row{ gui::kLensTypeGlobe, "globe" },
  };
  static_assert(sizeof(kRows) / sizeof(kRows[0]) == gui::kLensTypeCount,
                "this case must probe every lens type, not a prefix of them");

  for (const Row& row : kRows) {
    gui::GuiState state = gui::InitDefaultState();
    state.renderer.lens_type = row.value;

    const auto rows = gui::BuildDefaultDiffRows(state);
    const gui::DefaultDiffRow* lens = FindRow(rows, "renderer.lens_type");
    // Non-fatal per row: a missing row on one lens type must not hide the other ten readings, which
    // together are what says whether the table is short or merely wrong somewhere.
    if (lens == nullptr) {
      ADD_FAILURE() << row.spelling << ": the diff walk produced no renderer.lens_type row";
      continue;
    }
    EXPECT_EQ(lens->current_value.get<std::string>(), row.spelling) << "value: " << row.value;
    EXPECT_EQ(lens->factory_value.get<std::string>(), "linear") << "value: " << row.value;
  }
}

TEST_F(DefaultsDiff, ac5_effective_default_layers_the_saved_override_over_an_unmoving_factory_value) {
  const auto& dir = FreshUserConfig("diff_ac5");

  const float kFactoryAlpha = gui::GuiState{}.bg_alpha;
  gui::GuiState state = gui::InitDefaultState();
  state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;

  // State 1: no override file. The effective default is the factory value, so the edited lens type
  // is pending adoption and the two default readings agree — the case that makes the confusion
  // between them possible in the first place.
  {
    const auto rows = gui::BuildDefaultDiffRows(state);
    const gui::DefaultDiffRow* lens = FindRow(rows, "renderer.lens_type");
    ASSERT_TRUE(lens != nullptr);
    EXPECT_TRUE(gui::RowNeedsAdoption(*lens));
    EXPECT_STREQ(lens->default_value.get<std::string>().c_str(), "linear");
    EXPECT_TRUE(!lens->has_saved_override);
    const gui::DefaultDiffRow* alpha = FindRow(rows, "bg_alpha");
    ASSERT_TRUE(alpha != nullptr);
    EXPECT_TRUE(alpha->factory_value == alpha->default_value);
    EXPECT_EQ(alpha->factory_value.get<float>(), kFactoryAlpha);
  }

  // State 2: a document saving three keys — the edited lens type (same value, now also the
  // effective default), a non-factory bg_alpha the document then sits on, and bg_show at exactly
  // its factory value.
  const float kSavedAlpha = kFactoryAlpha + 0.25f;
  json doc;
  doc["renderer"]["lens_type"] = "fisheye_equal_area";
  doc["bg_alpha"] = kSavedAlpha;
  doc["bg_show"] = gui::GuiState{}.bg_show;  // saved, and identical to factory
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
  state.bg_alpha = kSavedAlpha;

  const auto rows = gui::BuildDefaultDiffRows(state);
  const gui::DefaultDiffRow* lens = FindRow(rows, "renderer.lens_type");
  ASSERT_TRUE(lens != nullptr);
  EXPECT_TRUE(!gui::RowNeedsAdoption(*lens)) << "the saved value became the effective default";
  EXPECT_STREQ(lens->default_value.get<std::string>().c_str(), "fisheye_equal_area");
  EXPECT_TRUE(lens->has_saved_override);

  const gui::DefaultDiffRow* alpha = FindRow(rows, "bg_alpha");
  ASSERT_TRUE(alpha != nullptr);
  EXPECT_EQ(alpha->default_value.get<float>(), kSavedAlpha);
  EXPECT_EQ(alpha->factory_value.get<float>(), kFactoryAlpha) << "factory_value moved with the save";
  // The two filter questions, side by side on ONE row, answering differently: the value differs
  // from factory, and the row does not need adopting. A panel using default_value for both would
  // say "no" twice.
  EXPECT_TRUE(alpha->current_value != alpha->factory_value);
  EXPECT_TRUE(!gui::RowNeedsAdoption(*alpha));

  // Presence, not value: bg_show was saved at its factory value and is still the user's.
  const gui::DefaultDiffRow* saved_at_factory = FindRow(rows, "bg_show");
  ASSERT_TRUE(saved_at_factory != nullptr);
  EXPECT_TRUE(saved_at_factory->has_saved_override);
  EXPECT_TRUE(!gui::RowNeedsAdoption(*saved_at_factory));
  const gui::DefaultDiffRow* untouched = FindRow(rows, "aspect_portrait");
  ASSERT_TRUE(untouched != nullptr);
  EXPECT_TRUE(!untouched->has_saved_override);

  // Enum readability, one row per name table (kLensTypeJsonNames / kVisibleJsonNames /
  // kAspectPresetJsonNames): the panel must show "fisheye_equal_area", never "1".
  for (const char* key : { "renderer.lens_type", "renderer.visible", "aspect_ratio" }) {
    const gui::DefaultDiffRow* row = FindRow(rows, key);
    if (row == nullptr) {
      ADD_FAILURE() << key << ": no row for this enum key";
      continue;  // no row to check for this key; the rest still get checked
    }
    EXPECT_TRUE(row->current_value.is_string());
    EXPECT_TRUE(!gui::FormatDiffValue(row->current_value).empty());
  }
}

// Display formatting is decoupled from comparison: two values that FORMAT identically at
// %.6g must still be a §2 row. Otherwise a rounding choice in the panel would decide what
// gets written to disk.
TEST_F(DefaultsDiff, format_is_decoupled_from_comparison) {
  NoUserConfig();

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

// ================================================================================
// The panel's write-back rule: one pass over the row set, driven by the checkbox state
// ================================================================================
TEST_F(DefaultsDiff, apply_checked_rows_adopts_and_removes_in_one_pass_and_survives_the_trip_to_disk) {
  // A pre-existing document with three kinds of key beside each other: one that will be
  // un-checked, a preset-library subtree this panel does not own, and a key written by an OLDER
  // version whose GuiState field has since been removed. The last two produce no row, so the rule
  // never names them — and both survive for that one structural reason, which is why they are
  // asserted together. The removed-field key is the case no other test covers, and the one that
  // would silently start disappearing if the rule were rewritten as "keep the keys we know about".
  json doc;
  doc["bg_show"] = false;
  doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;
  doc["a_key_no_current_field_serializes"] = 7;
  const auto& dir = FreshUserConfig("diff_checked_rows", &doc);

  const gui::GuiState state = MakeEditedState();
  const auto rows = gui::BuildDefaultDiffRows(state, doc);
  EXPECT_TRUE(!rows.empty());
  // The premise the halves below rest on, asserted rather than assumed: of the three extra keys,
  // one is named by a row and two are not.
  ASSERT_TRUE(FindRow(rows, "bg_show") != nullptr);
  ASSERT_TRUE(FindRow(rows, "renderer.fov") != nullptr);
  ASSERT_TRUE(FindRow(rows, "a_key_no_current_field_serializes") == nullptr);

  // Checked: keys that are NOT in the document. Everything else — including bg_show, which is —
  // is left out of the set, i.e. un-checked.
  json next = doc;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(next, rows, { "renderer.fov", "renderer.lens_type", "bg_alpha" }, state));

  EXPECT_TRUE(next.contains("renderer"));
  EXPECT_EQ(next["renderer"]["fov"].get<float>(), state.renderer.fov);
  EXPECT_TRUE(!next.contains("bg_show"));  // un-checked -> removed, in the same pass
  // Untouched by construction rather than by a special case: no row names them.
  EXPECT_EQ(next["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
  EXPECT_EQ(next.value("a_key_no_current_field_serializes", 0), 7);
  // And nothing else was adopted just because it was serialized.
  EXPECT_TRUE(!next.contains("sun"));

  // Now the same document through the pair the panel's Save actually calls, asserted FILE-side:
  // the trip to disk must change nothing about the outcome above.
  EXPECT_TRUE(gui::WriteActiveOverlayDoc(next));
  const json saved = ReadOverlayFile(dir);
  EXPECT_STREQ(saved["renderer"]["lens_type"].get<std::string>().c_str(), "fisheye_equal_area");
  EXPECT_EQ(saved["bg_alpha"].get<float>(), 0.42f);
  EXPECT_EQ(saved["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);
  EXPECT_EQ(saved.value("a_key_no_current_field_serializes", 0), 7);
  // bg_show, by contrast, HAS a row and was left un-checked, so the same pass removed it. That is
  // the line separating "not mentioned" from "mentioned and declined".
  EXPECT_TRUE(!saved.contains("bg_show"));
  EXPECT_TRUE(!saved.contains("sun"));

  // Un-checking EVERYTHING is Reset all: every GuiState key goes, the keys no row names stay.
  json emptied = doc;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(emptied, rows, {}, state));
  EXPECT_TRUE(!emptied.contains("bg_show"));
  EXPECT_EQ(emptied.size(), static_cast<size_t>(2));
  EXPECT_TRUE(emptied.contains("presets"));
  EXPECT_TRUE(emptied.contains("a_key_no_current_field_serializes"));
}

// A checked key that the CURRENT state does not serialize (an optional key) is removed rather
// than written as null — "adopt what I have now" reads the same either way, and a null in the
// file would be read back as a value.
TEST_F(DefaultsDiff, apply_checked_rows_removes_an_absent_optional_key) {
  NoUserConfig();

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
//
// The surgical-write half runs on the staging the apply case above already builds, and is asserted
// there: the pure rule and the trip to disk are the same document under one set of checked rows,
// and staging them separately said the same setup twice. What is left for this section is the
// state where the pair cannot run at all.

// No writable config directory (--no-user-config, or an OS that gave us nowhere to write):
// the write reports failure instead of silently claiming success, and nothing is created.
TEST_F(DefaultsDiff, live_write_back_reports_failure_without_a_directory) {
  NoUserConfig();

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
TEST_F(DefaultsDiff, no_default_eligible_key_contains_the_path_separator) {
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
//
// The negative half is in the same case rather than beside it: a notice that fired on EVERY write
// would satisfy the positive half while burying the real ones in noise, and the two halves differ
// only in whether the seeded document is well-formed.
TEST_F(DefaultsDiff, a_replaced_non_object_path_node_is_reported_and_an_ordinary_write_is_not) {
  json doc;
  doc["renderer"] = 3;  // a scalar where an object belongs
  const auto& dir = FreshUserConfig("setbypath_notice", &doc);

  const gui::GuiState state = MakeEditedState();
  // Built BEFORE the drain below, deliberately: this is the panel's opening snapshot, and
  // overlaying a document whose "renderer" is a scalar is itself a degradation the loader is
  // entitled to report. Draining after it keeps the assertions below about the WRITE and
  // nothing else.
  const auto rows = gui::BuildDefaultDiffRows(state, doc);
  ResetUserDefaultsChannels();

  json next = doc;
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(next, rows, { "renderer.fov" }, state));
  EXPECT_TRUE(gui::WriteActiveOverlayDoc(next));

  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
  const auto notices = gui::TakeUserDefaultsDowngradeNotices();
  // ASSERT, not EXPECT: the very regression this case exists to catch is "no notice was filed",
  // and indexing an empty vector on the next line would take this single-process binary down with
  // a SIGSEGV — hiding every case after it behind a crash instead of a diagnosis. Measured while
  // red-probing this assertion.
  ASSERT_EQ(notices.size(), static_cast<size_t>(1));
  EXPECT_TRUE(notices[0].find("renderer") != std::string::npos);

  // The write still landed — the notice reports the loss, it does not refuse the edit.
  const json saved = ReadOverlayFile(dir);
  EXPECT_TRUE(saved["renderer"].is_object());
  EXPECT_TRUE(saved["renderer"].contains("fov"));
  // Both counters are consumed on read, like every other channel in this family.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
  EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty());

  // The negative half: a well-formed document produces NO notice, and the write really did create
  // the nested object the notice would have described — so a silent zero reads as "no loss to
  // report" rather than as "no write happened".
  json quiet = json::object();
  const auto quiet_rows = gui::BuildDefaultDiffRows(state, quiet);
  ResetUserDefaultsChannels();
  EXPECT_TRUE(gui::ApplyCheckedRowsToDoc(quiet, quiet_rows, { "renderer.fov", "bg_alpha" }, state));
  EXPECT_TRUE(gui::WriteActiveOverlayDoc(quiet));
  EXPECT_TRUE(quiet["renderer"].is_object());
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
  EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty());
}

// ================================================================================
// The Settings panel's two data-only questions (its 40 widget-driving siblings are in
// test/gui/functional/test_defaults_panel.cpp)
// ================================================================================

// AC3 — coverage as a COUNTABLE deliverable, not "the mechanism works".
//
// Every leaf of the serialized document is named below with the control it must be edited
// with, and the case fails on the first row whose classification disagrees. A count would not
// do: it passes just as well when two fields swap classes, and it says nothing at all about
// WHICH field regressed. The two deliberately-unregistered leaves are named here too, so
// "read-only" is an asserted decision rather than an omission that looks like one.
TEST_F(DefaultsDiff, registry_covers_every_row) {
  // The freshly emptied explicit dir is what makes MakeNewDocumentState() deterministic.
  FreshUserConfig("panel_registry");

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
    // No editor: the GUI has no tint control anywhere (owner-decided; see
    // GuiState::RenderConfig::ray_color's own comment). Still serialized, so it still shows up as
    // a read-only row rather than disappearing.
    { "renderer.ray_color", false, Kind::kColor },
    { "renderer.exposure_offset", true, Kind::kFloatSlider },
    // v4.16. Registered as of the Display group's Mode combo landing: the field now has a control
    // in the main UI, so the defaults panel offers the same inline editor rather than a read-only
    // row. Two values only (Relative / Absolute), so it is a combo like renderer.visible.
    { "renderer.ev_mode", true, Kind::kCombo },
    { "aspect_ratio", true, Kind::kCombo },
    { "aspect_portrait", true, Kind::kCheckbox },
    { "bg_path", false, Kind::kCheckbox },
    { "bg_show", true, Kind::kCheckbox },
    { "bg_alpha", true, Kind::kFloatSlider },
    { "bg_offset_x", true, Kind::kFloatSlider },
    { "bg_offset_y", true, Kind::kFloatSlider },
    { "bg_scale", true, Kind::kFloatSlider },
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
    // The reference-point markers: three rows per marker, spelled out rather than generated from
    // MarkerFieldKey. This table is a hand-written mirror ON PURPOSE — generating it from the same
    // function the registry and the serializer call would compare one line of code against itself,
    // and the event it exists to catch is precisely a key that changed shape in one place.
    { "overlay_marker_zenith_line", true, Kind::kCheckbox },
    { "overlay_marker_zenith_label", true, Kind::kCheckbox },
    { "overlay_marker_zenith_color", true, Kind::kColor },
    { "overlay_marker_nadir_line", true, Kind::kCheckbox },
    { "overlay_marker_nadir_label", true, Kind::kCheckbox },
    { "overlay_marker_nadir_color", true, Kind::kColor },
    { "overlay_marker_sun_line", true, Kind::kCheckbox },
    { "overlay_marker_sun_label", true, Kind::kCheckbox },
    { "overlay_marker_sun_color", true, Kind::kColor },
    { "overlay_marker_subsun_line", true, Kind::kCheckbox },
    { "overlay_marker_subsun_label", true, Kind::kCheckbox },
    { "overlay_marker_subsun_color", true, Kind::kColor },
    { "overlay_marker_anthelion_line", true, Kind::kCheckbox },
    { "overlay_marker_anthelion_label", true, Kind::kCheckbox },
    { "overlay_marker_anthelion_color", true, Kind::kColor },
    { "overlay_marker_antisolar_line", true, Kind::kCheckbox },
    { "overlay_marker_antisolar_label", true, Kind::kCheckbox },
    { "overlay_marker_antisolar_color", true, Kind::kColor },
    { "overlay_markers_alpha", true, Kind::kFloatSlider },
    { "overlay_markers_radius_px", true, Kind::kFloatSlider },
    { "overlay_markers_section_open", true, Kind::kCheckbox },
    { "overlay_lens_border_line", true, Kind::kCheckbox },
    { "overlay_lens_border_color", true, Kind::kColor },
    { "overlay_lens_border_alpha", true, Kind::kFloatSlider },
    { "right_panel_collapsed", true, Kind::kCheckbox },
    { "modal_layout_vertical", true, Kind::kCheckbox },
  };

  // Against the REAL row set of a factory document, so this cannot drift from what the panel
  // actually lists: a field added to the serializer makes the row set larger than the table
  // above and fails here, which is the reminder to classify it.
  // A local document, not gui::g_state: BuildDefaultDiffRows takes the state it reads as a
  // parameter, so routing through the global would only add a way for a neighbouring case to
  // change this one's answer.
  const auto rows = gui::BuildDefaultDiffRows(gui::MakeNewDocumentState());
  std::set<std::string> row_keys;
  for (const auto& row : rows) {
    row_keys.insert(row.key_path);
  }
  EXPECT_EQ(row_keys.size(), std::size(kExpected));

  for (const auto& expected : kExpected) {
    EXPECT_TRUE(row_keys.count(expected.key_path) == 1);
    const gui::FieldEditorEntry* entry = gui::FindFieldEditor(expected.key_path);
    if (expected.registered) {
      if (entry == nullptr) {
        ADD_FAILURE() << expected.key_path << ": expected a registered field editor entry";
        continue;  // no entry to check the kind of; the rest still get checked
      }
      EXPECT_TRUE(entry->kind == expected.kind);
    } else {
      EXPECT_TRUE(entry == nullptr);
    }
  }

  // Nothing is registered that the document does not produce: an entry for a key that no longer
  // exists is dead weight the panel can never reach, and it would silently survive the loop
  // above.
  for (const auto& key : gui::RegisteredFieldEditorKeyPaths()) {
    EXPECT_TRUE(row_keys.count(key) == 1);
  }
}

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
TEST_F(DefaultsDiff, row_tint_predicate_over_four_states) {
  gui::DefaultDiffRow row;
  row.key_path = "probe";

  // (1) nothing saved, nothing adopted — Save writes nothing for this key.
  row.current_value = 0.5f;
  row.default_value = 0.5f;
  row.factory_value = 0.5f;
  row.has_saved_override = false;
  EXPECT_TRUE(!gui::RowWouldChangeOnSave(row, /*checked=*/false));

  // (2) the same row, adopted: Save would ADD the key. Tinted even though the value equals the
  // factory one — presence is a change.
  EXPECT_TRUE(gui::RowWouldChangeOnSave(row, /*checked=*/true));

  // (3) changed in the GUI, never saved: adopted by default, and Save would write the new
  // value.
  row.current_value = 0.9f;
  EXPECT_TRUE(gui::RowWouldChangeOnSave(row, /*checked=*/true));
  // ...and un-checking it takes it back to "Save writes nothing", which is also no change.
  EXPECT_TRUE(!gui::RowWouldChangeOnSave(row, /*checked=*/false));

  // (4) saved earlier, untouched since: differs from factory, equals what is on disk. NOT
  // tinted — the state that separates this predicate from the "Differs from factory" filter.
  row.current_value = 0.9f;
  row.default_value = 0.9f;  // has_saved_override ⇒ default_value IS the stored value
  row.factory_value = 0.5f;
  row.has_saved_override = true;
  EXPECT_TRUE(!gui::RowWouldChangeOnSave(row, /*checked=*/true));
  // Un-checking it would REMOVE the key from the file — a change.
  EXPECT_TRUE(gui::RowWouldChangeOnSave(row, /*checked=*/false));
}

// ================================================================================
// The Visibility group's enablement rule, as a rule rather than as a grid of examples.
//
// `renderer.visible` (the Upper/Full/Lower radio group) and `renderer.front` (the Front checkbox)
// are gated in the field-editor registry by NotUnderFullSky and NotUnderFullSkyOrGlobe. Before task
// 431.1 the main UI did not read either: app_panels.cpp spelled the same two conditions out again
// as a hand-paired `BeginDisabled` nest, and the only thing pinning the pair was nine near-identical
// gui_test cases, one per (lens, expectation) cell somebody thought to write down.
//
// This is the WHAT — the semantics of the two registry gates over every lens type. Its sibling
// `view_display_controls/the_visibility_row_is_gated_per_lens_by_two_rules` in
// test/gui/functional/test_view_display_controls.cpp is the WHERE — that the widgets
// app_panels.cpp actually draws carry exactly these values. The split is
// the link boundary AGENTS.md draws: the registry gate is pure logic and evaluates with no ImGui
// context, so it runs here and therefore on all three CI platforms; "is the real widget greyed"
// needs a live frame and can only run where gui_test does.
//
// Scope of the claim, stated so it is not read as wider than it is: LensIsFullSky is the repo's
// single source of truth for the full-sky set (gui_constants.hpp), and the expectation below calls
// it, exactly as the retired inline code did. So this pins WHICH gate each field carries and what
// that gate computes — not the membership of kFullSkyLensTypes, which is a policy statement guarded
// by its own static_assert rather than an invariant anything can derive.
// ================================================================================
// AC2/AC4 — universally quantified over kLensTypeCount x {visible, front}, so a lens added to the
// enum is covered the moment it exists, with no cell to remember to write. The expectation is
// re-derived from LensIsFullSky / kLensTypeGlobe rather than read back out of ConstraintFor: a
// predicate compared against itself agrees with itself no matter what it computes.
//
// This also carries AC4's dynamic half. The retired inline gate was
//   radios disabled  <=>  full_sky
//   Front  disabled  <=>  full_sky || (!full_sky && is_globe)  ==  full_sky || is_globe
// which is what the two EXPECT_EQ lines below assert of the registry, cell for cell.
//
// What makes `< kLensTypeCount` quantify over the whole enum rather than a prefix of it is not
// asserted here, because it is already asserted where it belongs: gui_state.hpp pins
// kLensTypeGlobe == kLensTypeCount - 1 with a static_assert, alongside the two that pin
// kLensTypeNames and kLensTypePresentationOrder against the same count. That header is included
// above, so the guarantee this loop rests on is enforced at compile time in this very TU — a
// runtime case restating it would be strictly weaker than the check already running.
TEST(VisibilityEnablement, inline_ac2_registry_gate_matches_semantics_for_every_lens) {
  for (int lens = 0; lens < lumice::gui::kLensTypeCount; ++lens) {
    lumice::gui::GuiState state;
    state.renderer.lens_type = lens;

    const bool full_sky = lumice::gui::LensIsFullSky(lens);
    const bool is_globe = (lens == lumice::gui::kLensTypeGlobe);

    const lumice::gui::FieldEditorConstraint visible_c = lumice::gui::ConstraintFor("renderer.visible", state);
    const lumice::gui::FieldEditorConstraint front_c = lumice::gui::ConstraintFor("renderer.front", state);

    EXPECT_EQ(visible_c.enabled, !full_sky) << "renderer.visible, lens=" << lens;
    EXPECT_EQ(front_c.enabled, !(full_sky || is_globe)) << "renderer.front, lens=" << lens;
  }
}
