// User-defaults store tests.
//
// Covers the half of src/gui/user_defaults.hpp that calls into file_io.cpp, which is only
// linkable from a target that pulls in lumice_gui_obj (unit_correctness_test links lumice_obj
// alone). Nothing here needs a window, a GL context or an ImGui frame, so gui_unit_test — the
// windowless target of this same layer — is its home:
//   AC2 — one-hand recomputation of `kView \ SerializeGuiStateJson` against the real serializer
//   AC3 — sparse override round-trip (singleton config + preset library), untouched keys stay factory
//   AC4 — invariant I1: personal defaults reach New, never a value present in a loaded file
//   AC5 — a broken override file degrades (unknown key / wrong type / invalid JSON), never crashes
//   D8  — load-time clamp of preset values into the classifier's tolerance domain, never silent
//   bg_path — a personal default pointing at a missing image degrades instead of blocking startup
//
// The eligibility resolver and the platform config-dir helpers are pure and header-only; they
// are covered in test/unit-correctness/gui/test_user_defaults_eligibility.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gui/app.hpp"
#include "gui/axis_presets.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state_tiers.hpp"
#include "gui/user_defaults.hpp"
#include "support/user_defaults_test_env.hpp"

namespace {

using nlohmann::json;

// Isolation policy (fresh temp directory per case, scope guards that restore the harness
// baseline) lives in one place for every test source that touches the store; see
// user_defaults_test_env.hpp for why it is shared rather than copied.
using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;
using lumice::test_user_defaults::ScopedNoUserConfigDirEnv;
using lumice::test_user_defaults::ScopedUserConfigSource;
using lumice::test_user_defaults::WriteRawOverlay;

// Compare two states over the fields a personal default is allowed to reach. Written against the
// serializer rather than as a field list so it cannot go stale: SerializeGuiStateJson is the same
// surface the override file itself is expressed in, so "the overlay changed nothing observable"
// and "the two documents serialize identically" are the same statement.
bool SerializesIdentically(const gui::GuiState& a, const gui::GuiState& b) {
  return gui::SerializeGuiStateJson(a) == gui::SerializeGuiStateJson(b);
}

// Read presets.axis.<name>.zenith_std, reporting an absent or malformed key as nullopt.
//
// Not defensiveness for its own sake: `doc["presets"]["axis"]["column"]["zenith_std"].get<float>()`
// on a document that LOST the key throws an uncaught json type_error, which aborts this
// single-process binary and takes every case after it with it — measured, while red-probing the
// surgical-write assertion below. The regression these cases exist to catch is exactly "the key
// is gone", so the read has to survive it and report.
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

// Read the override file from disk directly, bypassing the production reader. The write-side
// cases assert on what LANDED, and going through ReadOverlayJsonIfPresent would let a reader bug
// and a writer bug cancel out into a passing test.
json ReadOverlayDoc(const std::filesystem::path& dir) {
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

// Put one preset override on disk, the way the panel's Save does: read the whole document, touch
// the one key through its production owner, write the whole document back.
//
// The cases below use this to STAGE a precondition rather than to assert on it — the thing under
// test is what happens next (a reload, a revert, a classification). It deliberately goes through
// WriteAxisPresetZenithStdToDoc instead of spelling the JSON path by hand: a test that hardcoded
// `doc["presets"]["axis"][...]` would keep passing after the document shape moved, staging a file
// the production reader no longer looks at.
//
// No clamp: callers pass in-domain values, and routing the staging through the clamp would make a
// case that meant to stage 0.3 quietly stage something else if a domain ever moved.
bool SeedPresetOverrideOnDisk(const std::filesystem::path& dir, gui::AxisPreset preset, float stored_value) {
  json doc = ReadOverlayDoc(dir);
  gui::WriteAxisPresetZenithStdToDoc(doc, preset, stored_value);
  return gui::WriteUserDefaultsFile(dir, doc);
}

// Drop one preset override the way the panel's Restore button and Save do: erase the key from the
// working document through its production owner, write the whole document, and only THEN let the
// process-wide cache follow — disk first, memory second, the order defaults_panel.cpp's CommitCopy
// calls out as part of the contract.
//
// Spelled out here rather than called through one store-side function because there is no longer
// one: the disk-side revert wrapper this file used to call was a second write path that the panel
// itself never went through, and the live path composes these three primitives at its own call
// site. The assertions below are about the DOCUMENT shape the erase leaves behind, which is
// EraseAxisPresetZenithStdFromDoc's own contract and the half a wholesale rewrite would break.
//
// ⛔ Do NOT reuse this helper to assert the disk-first ORDER itself. It reproduces that order to
// build a starting state, so an assertion about ordering made through it would be checking this
// file's copy of the sequence, not the panel's. The order contract belongs to CommitCopy and is
// pinned in gui_test (defaults_panel/a_save_that_could_not_write_leaves_the_preset_cache_alone) —
// a second place asserting
// it here is exactly the parallel-implementation shape this task deleted.
bool RestoreOnePresetOnDisk(const std::filesystem::path& dir, gui::AxisPreset preset) {
  json doc = ReadOverlayDoc(dir);
  gui::EraseAxisPresetZenithStdFromDoc(doc, preset);
  if (!gui::WriteUserDefaultsFile(dir, doc)) {
    return false;
  }
  gui::AdoptAxisPresetZenithStdOverrideInMemory(preset, std::nullopt);
  return true;
}

}  // namespace

// Every case starts from drained channels: the downgrade counter and the notice list are
// consumed-on-read, so a leftover from a previous case reads as this one's own degradation.
class UserDefaults : public ::testing::Test {
 protected:
  void SetUp() override { ResetUserDefaultsChannels(); }

  // A fresh, empty override directory plus the document to find in it. Staging the precondition is
  // one statement in every case below; what is under test is always what happens next.
  static std::filesystem::path DirWith(const char* name, const json& doc) {
    const auto dir = FreshOverlayDir(name);
    EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
    return dir;
  }
};

// ================================================================================
// AC2 — `kView \ serialized` recomputed against the real serializer
// ================================================================================
TEST_F(UserDefaults, ac2_kview_difference_set_recomputed) {
  // Expected JSON root key for each FieldTier::kView field, or nullptr when the field is
  // expected to have NO serialized representation. This table is the recomputation: it is
  // checked in both directions against a real SerializeGuiStateJson output, so a field that
  // starts (or stops) being serialized flips a verdict here rather than silently widening
  // or narrowing what may be stored as a personal default.
  //
  // The field->key mapping is not identity everywhere (aspect_preset -> "aspect_ratio",
  // show_* -> "overlay_*"), which is exactly why this cannot be a name-equality check.
  struct ViewFieldKey {
    const char* field;
    const char* json_key;  // nullptr = expected to be absent from the schema
  };
  static constexpr ViewFieldKey kViewFieldKeys[] = {
    { "aspect_preset", "aspect_ratio" },
    { "aspect_portrait", "aspect_portrait" },
    { "bg_path", "bg_path" },
    { "bg_show", "bg_show" },
    { "bg_alpha", "bg_alpha" },
    { "bg_offset_x", "bg_offset_x" },
    { "bg_offset_y", "bg_offset_y" },
    { "bg_scale", "bg_scale" },
    { "show_horizon_line", "overlay_horizon_line" },
    { "show_horizon_label", "overlay_horizon_label" },
    { "show_grid_line", "overlay_grid_line" },
    { "show_grid_label", "overlay_grid_label" },
    { "show_sun_circles_line", "overlay_sun_circles_line" },
    { "show_sun_circles_label", "overlay_sun_circles_label" },
    { "sun_circle_angles", "overlay_sun_circle_angles" },
    { "horizon_color", "overlay_horizon_color" },
    { "grid_color", "overlay_grid_color" },
    { "sun_circles_color", "overlay_sun_circles_color" },
    { "horizon_alpha", "overlay_horizon_alpha" },
    { "grid_alpha", "overlay_grid_alpha" },
    { "sun_circles_alpha", "overlay_sun_circles_alpha" },
    { "show_zenith_nadir_line", "overlay_zenith_nadir_line" },
    { "zenith_nadir_color", "overlay_zenith_nadir_color" },
    { "zenith_nadir_alpha", "overlay_zenith_nadir_alpha" },
    { "zenith_nadir_radius_px", "overlay_zenith_nadir_radius_px" },
    { "show_lens_border_line", "overlay_lens_border_line" },
    { "lens_border_color", "overlay_lens_border_color" },
    { "lens_border_alpha", "overlay_lens_border_alpha" },
    { "right_panel_collapsed", "right_panel_collapsed" },
    { "modal_layout_vertical", "modal_layout_vertical" },
    { "left_panel_collapsed", nullptr },
    { "gui_log_level", nullptr },
    { "core_log_level", nullptr },
    { "log_to_file", nullptr },
    { "log_panel_open", nullptr },
  };

  const json root = json::parse(gui::SerializeGuiStateJson(gui::GuiState{}));
  EXPECT_TRUE(root.is_object());

  // 1. The table must enumerate the kView tier exactly — no field skipped, none invented.
  std::set<std::string> table_fields;
  for (const auto& row : kViewFieldKeys) {
    table_fields.insert(row.field);
  }
  std::set<std::string> tier_view_fields;
  for (const auto& entry : gui::kFieldTierTable) {
    if (entry.tier == gui::FieldTier::kView) {
      tier_view_fields.insert(entry.name);
    }
  }
  EXPECT_EQ(table_fields.size(), tier_view_fields.size());
  EXPECT_TRUE(table_fields == tier_view_fields);

  // 2. Each claim holds against the actual serializer output.
  std::set<std::string> recomputed_difference;
  for (const auto& row : kViewFieldKeys) {
    if (row.json_key == nullptr) {
      recomputed_difference.insert(row.field);
      continue;
    }
    EXPECT_TRUE(root.contains(row.json_key));
  }

  // 3. The recomputed difference set must equal the production constant that eligibility
  //    is derived from. This is the assertion the design doc's hand-eyeballed list is
  //    replaced by.
  std::set<std::string> constant_difference;
  for (const char* name : gui::kUnserializedViewFields) {
    constant_difference.insert(name);
  }
  EXPECT_TRUE(recomputed_difference == constant_difference);
  EXPECT_EQ(recomputed_difference.size(), static_cast<size_t>(5));
}

// ================================================================================
// AC3 — sparse round-trip
// ================================================================================
TEST_F(UserDefaults, ac3_sparse_overlay_round_trip) {
  const auto dir = FreshOverlayDir("ac3");

  // A genuinely sparse document: one namespace-1 singleton key, one namespace-1 kView key,
  // and one namespace-2 preset-library entry. Everything else must come back factory.
  json doc;
  doc["renderer"]["lens_type"] = "fisheye_equal_area";
  doc["bg_alpha"] = 0.25f;
  doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;  // the tighter value users asked for
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  EXPECT_EQ(state.renderer.lens_type, gui::kLensTypeFisheyeEqualArea);
  EXPECT_EQ(state.bg_alpha, 0.25f);

  // An untouched key reads back as the factory value, not as whatever the writer omitted.
  const gui::GuiState factory{};
  EXPECT_EQ(state.aspect_portrait, factory.aspect_portrait);
  EXPECT_EQ(state.sun.altitude, factory.sun.altitude);
  EXPECT_EQ(state.renderer.fov, factory.renderer.fov);

  // Preset-library half, read through its own query interface.
  const auto column = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(column.has_value());
  EXPECT_EQ(*column, 0.3f);
  // A preset with no entry stays "no override" rather than defaulting to something.
  EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kLowitz).has_value());

  // A well-formed file is not a degradation, and an in-domain value is not a clamp.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(0));

  // The most common path of all: no override file yet. It must be a silent no-op, NOT a
  // degradation — counting it would fire the "your defaults were downgraded" notice on every
  // fresh install.
  const gui::GuiState untouched = gui::MakeNewDocumentState(FreshOverlayDir("absent"));
  const gui::GuiState seeded = gui::InitDefaultState();
  EXPECT_EQ(untouched.renderer.lens_type, seeded.renderer.lens_type);
  EXPECT_EQ(untouched.bg_alpha, seeded.bg_alpha);
  EXPECT_EQ(untouched.crystals.size(), seeded.crystals.size());
  EXPECT_EQ(untouched.layers.size(), seeded.layers.size());
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
}

// Eligibility must be enforced on the READ path, not merely recorded as metadata: the
// override file is a text file the user can hand-edit. One case per namespace would be three
// copies of the same staging; the point is that NONE of them reaches a new document, so they
// are smuggled together and the document is checked against the seeded contents as a whole.
//
// raypath_color is included because it is namespace 4 (kCollectionFields) exactly like
// layers/crystals/filters, but MakeNewDocumentState only cleared the other three, so a
// hand-edited file could put a colour-class list in every new document.
TEST_F(UserDefaults, ac3_ineligible_keys_cannot_be_smuggled_in) {
  const auto dir = FreshOverlayDir("ineligible");

  json doc;
  // namespace 3: an ordinary serializable scalar the deserializer would happily read.
  doc["use_gpu_backend"] = true;
  // namespace 4: collections. A hand-edited file must not make New start with someone
  // else's scene. Bare-array wire form for raypath_color (DeserializeGuiStateJson accepts it
  // directly under that key; see file_io.cpp's parse block).
  doc["layers"] = json::array();
  doc["layers"].push_back({ { "prob", 0.5f }, { "entries", json::array() } });
  doc["layers"].push_back({ { "prob", 0.5f }, { "entries", json::array() } });
  doc["raypath_color"] = json::array();
  doc["raypath_color"].push_back({ { "color", { 1.0f, 0.0f, 0.0f } } });
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  EXPECT_EQ(state.use_gpu_backend, gui::GuiState{}.use_gpu_backend);
  // Seeded contents only: exactly one layer with one entry, one crystal, no filters, no colours.
  EXPECT_EQ(state.layers.size(), static_cast<size_t>(1));
  EXPECT_EQ(state.layers[0].entries.size(), static_cast<size_t>(1));
  EXPECT_EQ(state.crystals.size(), static_cast<size_t>(1));
  EXPECT_EQ(state.filters.size(), static_cast<size_t>(0));
  EXPECT_EQ(state.raypath_color.size(), static_cast<size_t>(0));
}

// --no-user-config and --user-config <empty dir> must be the same document. Both are "the
// user has saved nothing", reached by two different routes, and a user who cannot get the
// same result twice has no way to establish a factory baseline.
TEST_F(UserDefaults, switch_disabled_equals_empty_explicit_dir) {
  const auto empty_dir = FreshOverlayDir("switch_empty");

  gui::GuiState disabled;
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);
    disabled = gui::MakeNewDocumentState();
  }
  gui::GuiState empty_explicit;
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, empty_dir);
    empty_explicit = gui::MakeNewDocumentState();
  }
  EXPECT_TRUE(SerializesIdentically(disabled, empty_explicit));
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);

  // ...and the comparison above is not vacuous: the SAME directory holding a real override
  // produces a DIFFERENT document through the same code path. Without this arm, a broken
  // switch that ignored explicit_dir would satisfy the equality assertion perfectly.
  json doc;
  doc["renderer"]["lens_type"] = "fisheye_equal_area";
  doc["bg_alpha"] = 0.25f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(empty_dir, doc));
  gui::GuiState overridden;
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, empty_dir);
    overridden = gui::MakeNewDocumentState();
  }
  EXPECT_TRUE(!SerializesIdentically(disabled, overridden));
  EXPECT_EQ(overridden.renderer.lens_type, gui::kLensTypeFisheyeEqualArea);
  EXPECT_EQ(overridden.bg_alpha, 0.25f);

  // And with the switch back to disabled, that same on-disk file is invisible again.
  gui::GuiState disabled_again;
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);
    disabled_again = gui::MakeNewDocumentState();
  }
  EXPECT_TRUE(SerializesIdentically(disabled, disabled_again));

  // Last, the harness's own baseline, asserted from inside the harness with NO guard in force.
  // This binary installs kTestHarnessUserConfigDefault before any test runs (gui_unit_test_env.cpp;
  // gui_test does the same from its main()); were that install dropped or flipped, the developer's
  // real config directory would reach this document instead. It is read through the switch's only
  // observable effect, since the source enum has no getter by design (one owner). The `overridden`
  // arm above is this line's detection power: a real file IS reachable through this same call when
  // the switch says so, so agreeing with `disabled` here is a fact about the baseline, not about
  // MakeNewDocumentState being inert.
  EXPECT_TRUE(SerializesIdentically(disabled, gui::MakeNewDocumentState()));
}

// Two ways of pointing --user-config somewhere unusable. Both must land on the factory document
// rather than on a crash: a corrupt file routes into the existing degradation path instead of
// around it, and a path to a REGULAR FILE (the flag takes a directory, and users point it at the
// user_defaults.json itself) is survivable — it simply has nothing to read.
TEST_F(UserDefaults, switch_explicit_dir_that_cannot_be_read_degrades_to_the_factory_document) {
  const auto broken = FreshOverlayDir("switch_broken");
  WriteRawOverlay(broken, "{ this is not json");
  const auto not_a_dir_parent = FreshOverlayDir("switch_not_a_dir");
  const std::filesystem::path not_a_dir = not_a_dir_parent / "plain_file.txt";
  {
    std::ofstream out(not_a_dir, std::ios::trunc);
    out << "not a directory";
  }

  struct Case {
    const char* name;
    std::filesystem::path path;
    int expect_downgrades;  // a file that parses as nothing is not a degradation to report
  };
  const Case kCases[] = {
    { "a corrupt override file", broken, 1 },
    { "a path to a regular file", not_a_dir, 0 },
  };

  const gui::GuiState factory = gui::InitDefaultState();
  for (const Case& c : kCases) {
    ResetUserDefaultsChannels();
    gui::GuiState state;
    {
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, c.path);
      state = gui::MakeNewDocumentState();
    }
    EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), c.expect_downgrades) << c.name;
    EXPECT_EQ(state.renderer.lens_type, factory.renderer.lens_type) << c.name;
    EXPECT_EQ(state.bg_alpha, factory.bg_alpha) << c.name;
    EXPECT_EQ(state.crystals.size(), factory.crystals.size()) << c.name;
  }
}

// ================================================================================
// AC4 — invariant I1
// ================================================================================
TEST_F(UserDefaults, ac4_file_value_beats_personal_default) {
  const auto dir = FreshOverlayDir("ac4");

  // Personal default: X.
  json doc;
  doc["renderer"]["lens_type"] = "fisheye_equal_area";
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  // Sanity: the personal default really does reach a NEW document (otherwise the negative
  // assertions below would pass vacuously).
  const gui::GuiState fresh = gui::MakeNewDocumentState(dir);
  EXPECT_EQ(fresh.renderer.lens_type, gui::kLensTypeFisheyeEqualArea);

  // Forward: a file that states Y explicitly loads as Y, never as the personal default X.
  // This is invariant I1 — otherwise a .lmc someone sends you renders differently on your
  // machine, silently.
  json file_doc = json::parse(gui::SerializeGuiStateJson(gui::InitDefaultState()));
  file_doc["renderer"]["lens_type"] = "linear";
  gui::GuiState loaded;
  EXPECT_TRUE(gui::DeserializeGuiStateJson(file_doc.dump(), loaded));
  EXPECT_EQ(loaded.renderer.lens_type, gui::kLensTypeLinear);

  // Reverse: an OLD file that is simply MISSING the key falls back to the FACTORY value,
  // not to the personal default.
  //
  // This is the "must not be left ambiguous" call in the AC, and it goes this way because
  // the alternative breaks reproducibility in a way nothing surfaces: if a missing key fell
  // back to the personal default, two people opening the same old file would render
  // different images with no indication why. "New document" is a decision the user just
  // made; "opened an old file" is not. Only the former takes personal defaults.
  json old_doc = file_doc;
  old_doc["renderer"].erase("lens_type");
  gui::GuiState old_loaded;
  EXPECT_TRUE(gui::DeserializeGuiStateJson(old_doc.dump(), old_loaded));
  EXPECT_EQ(old_loaded.renderer.lens_type, gui::RenderConfig{}.lens_type);
  EXPECT_TRUE(old_loaded.renderer.lens_type != gui::kLensTypeFisheyeEqualArea);

  // The same pair on a second, differently-shaped key (a root scalar rather than a nested enum),
  // which is also the programmatic form of the manual walkthrough this feature used to be signed
  // off by: "set a user_defaults.json, start the GUI, observe New picking up the default and .lmc
  // Open unaffected". A one-time human check leaves no trace and has to be redone by hand every
  // time this file changes.
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, json{ { "bg_alpha", 0.33f } }));
  EXPECT_EQ(gui::MakeNewDocumentState(dir).bg_alpha, 0.33f);
  json lmc_doc = json::parse(gui::SerializeGuiStateJson(gui::InitDefaultState()));
  lmc_doc.erase("bg_alpha");
  gui::GuiState opened;
  EXPECT_TRUE(gui::DeserializeGuiStateJson(lmc_doc.dump(), opened));
  EXPECT_EQ(opened.bg_alpha, gui::GuiState{}.bg_alpha);
  EXPECT_TRUE(opened.bg_alpha != 0.33f);
}

// ================================================================================
// AC5 — degradation, never a crash
// ================================================================================
// A broken override file must cost the user their personal defaults and nothing else: the app still
// opens on a usable document, and the fact that something was dropped is reported. The line the
// rows below draw is between "this build does not understand it" and "nobody could" — a key from a
// NEWER build is the normal shape of a file that has travelled, so ignoring it is correct and is
// not a degradation. Everything else is, and the whole overlay goes, including well-formed siblings:
// a half-applied set of defaults is harder to reason about, and to revert, than none at all.
TEST_F(UserDefaults, ABrokenOverrideFileCostsTheDefaultsAndSaysSoWithoutBreakingTheDocument) {
  struct DegradeCase {
    const char* name;
    const char* raw;
    bool expect_bg_alpha_applied;
    int expect_downgrades;
  };
  const DegradeCase kCases[] = {
    { "a key this build does not know", R"({"totally_unknown_key": 1, "bg_alpha": 0.75})", true, 0 },
    // bg_alpha is a float, so a string there makes the deserializer throw mid-merge — and the
    // well-formed sibling beside it goes too.
    { "a value of the wrong type", R"({"bg_alpha": "not_a_number", "aspect_portrait": true})", false, 1 },
    { "a truncated document", R"({"bg_alpha": 0.5, )", false, 1 },
    { "an array where an object belongs", "[1, 2, 3]", false, 1 },
    // The preset-library half degrades down the same channel. It is a row rather than a case of
    // its own because the verdict it needs is the one every row already carries.
    { "a preset value of the wrong type", R"({"presets": {"axis": {"column": {"zenith_std": "wide"}}}})", false, 1 },
  };

  const gui::GuiState factory{};
  for (const DegradeCase& c : kCases) {
    ResetUserDefaultsChannels();
    const auto dir = FreshOverlayDir("degrade");
    WriteRawOverlay(dir, c.raw);

    const gui::GuiState state = gui::MakeNewDocumentState(dir);
    EXPECT_EQ(state.bg_alpha, c.expect_bg_alpha_applied ? 0.75f : factory.bg_alpha) << c.name;
    EXPECT_EQ(state.aspect_portrait, factory.aspect_portrait) << c.name;
    EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), c.expect_downgrades) << c.name;
    // Still a usable new document, whatever was thrown away.
    EXPECT_EQ(state.layers.size(), static_cast<size_t>(1)) << c.name;
    // Nothing here leaves a preset override behind, and a REJECTED value is a drop rather than a
    // clamp: it is counted, but it must not also arrive as a "we adjusted this for you" notice.
    EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value()) << c.name;
    EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty()) << c.name;
  }
}

// ================================================================================
// D8 — load-time clamp into the classifier's tolerance domain, with a visible trace
// ================================================================================
// A stored preset value outside its classifier's domain is pulled back INSIDE it, not onto the
// boundary: the domains are open, so landing exactly on 10 fails the classifier's strict `< 10` and
// demotes the preset to Custom — which is the very thing the clamp exists to prevent. Both
// directions are covered because the domains point opposite ways, and both ends of Column's are
// because a zero or negative std would otherwise reach the sampler.
TEST_F(UserDefaults, AnOutOfDomainPresetValueIsPulledStrictlyInsideItsDomain) {
  constexpr float kInf = std::numeric_limits<float>::infinity();
  struct ClampCase {
    const char* name;
    const char* preset_key;
    gui::AxisPreset preset;
    float stored;
    float lower;    // the domain, open at both ends
    float upper;    // kInf for a preset whose domain has no upper end
    float landing;  // the value that must come back — the nearest float strictly inside
  };
  const ClampCase kCases[] = {
    // Hand-edited, or written by a build whose classifier was wider.
    { "above Column's upper bound", "column", gui::AxisPreset::kColumn, 25.0f, 0.0f,
      gui::kColumnPlateParryZenithStdUpperBound, std::nextafter(gui::kColumnPlateParryZenithStdUpperBound, 0.0f) },
    // Zero and below are outside too: Column/Plate/Parry's domain is open at BOTH ends, and a
    // non-positive std would otherwise reach the sampler.
    { "below Plate's lower end of zero", "plate", gui::AxisPreset::kPlate, -3.0f, 0.0f,
      gui::kColumnPlateParryZenithStdUpperBound, std::nextafter(0.0f, kInf) },
    { "below Lowitz's lower bound", "lowitz", gui::AxisPreset::kLowitz, 5.0f, gui::kLowitzZenithStdLowerBound, kInf,
      std::nextafter(gui::kLowitzZenithStdLowerBound, kInf) },
  };

  for (const ClampCase& c : kCases) {
    ResetUserDefaultsChannels();
    json doc;
    doc["presets"]["axis"][c.preset_key]["zenith_std"] = c.stored;
    gui::MakeNewDocumentState(DirWith("clamp", doc));

    const auto stored = gui::GetUserAxisPresetZenithStdOverride(c.preset);
    if (!stored.has_value()) {
      ADD_FAILURE() << c.name << ": no stored override for this preset";
      continue;  // no value to compare for this row; the rest still get checked
    }
    EXPECT_EQ(*stored, c.landing) << c.name;
    EXPECT_GT(*stored, c.lower) << c.name;
    EXPECT_LT(*stored, c.upper) << c.name;

    // The clamp must leave a trace naming the key and the value it replaced. At load time the user
    // is not looking at the preset panel, so a silent clamp is indistinguishable from a silent drop
    // — the exact failure family this whole channel exists to end. And the channel is
    // consumed-on-read, like the downgrade counter it mirrors.
    const auto notices = gui::TakeUserDefaultsDowngradeNotices();
    if (notices.size() != static_cast<size_t>(1)) {
      ADD_FAILURE() << c.name << ": expected exactly 1 downgrade notice, got " << notices.size();
      continue;  // no notice to index for this row; the rest still get checked
    }
    EXPECT_NE(notices[0].find(c.preset_key), std::string::npos) << c.name;
    EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty()) << c.name;
  }
}

// code-review round 2 Major: ApplyAxisPresetOverridesFromJson only ever added/updated slots,
// never cleared one whose entry disappeared from the file. MakeNewDocumentState() is called
// repeatedly within one process (main.cpp startup, every DoNew(), every DoOpen() .json
// import), so a stale g_axis_overrides slot would keep answering with a deleted override
// until the process restarts. Both cases below call MakeNewDocumentState() twice against the
// same directory within a single test case (deliberately not draining the channels between the
// two calls) to reproduce the same-process, file-changed-underneath scenario. The second case
// covers a distinct branch from the first: passing an explicit override_dir means `dir`
// is truthy on both calls and never exercises the branch where GetUserConfigDir() ITSELF
// resolves to nullopt — the fix resets g_axis_overrides only inside `if (dir)`, so a
// process whose config directory becomes unavailable mid-run (removed out from under it,
// permissions revoked, disk full — its own doc comment lists these) would keep answering with
// the first call's override forever.
TEST_F(UserDefaults, d8_preset_override_does_not_leak_across_calls) {
  struct Case {
    const char* name;
    bool via_unavailable_config_dir;
  };
  const Case kCases[] = {
    { "the override disappears from the file", false },
    { "the config directory becomes unavailable", true },
  };

  for (const Case& c : kCases) {
    ResetUserDefaultsChannels();
    const auto dir = FreshOverlayDir("d8_leak");
    json doc;
    doc["presets"]["axis"]["column"]["zenith_std"] = 3.0f;
    EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
    gui::MakeNewDocumentState(dir);
    const auto first = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
    if (!first.has_value()) {
      ADD_FAILURE() << c.name << ": no stored override after the first call";
      continue;  // no value to compare for this row; the rest still get checked
    }
    EXPECT_EQ(*first, 3.0f) << c.name;

    if (c.via_unavailable_config_dir) {
      // The no-arg production call path (main.cpp / DoNew() / DoOpen() all call
      // MakeNewDocumentState() with no argument), with GetUserConfigDir() forced to nullopt.
      //
      // The kAutoDetect guard is what keeps this arm testing what it says. The harness baseline
      // is kDisabled, and under kDisabled the no-arg path returns nullopt without ever calling
      // GetUserConfigDir() — the arm would still pass (same verdict, by a different route) while
      // silently no longer exercising "GetUserConfigDir() went unavailable mid-process", the
      // regression it was written for. Restoring kAutoDetect for this scope re-arms
      // ScopedNoUserConfigDirEnv, which only has an effect on that branch.
      ScopedUserConfigSource auto_detect(gui::UserConfigSource::kAutoDetect);
      ScopedNoUserConfigDirEnv no_config_dir;
      gui::MakeNewDocumentState(std::nullopt);
    } else {
      // The user edits the file underneath the running process and removes the override — no
      // call to ResetUserAxisPresetOverrides() here, matching production (only
      // ApplyAxisPresetOverridesFromJson's internal reset should apply).
      EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, json::object()));
      gui::MakeNewDocumentState(dir);
    }
    EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value()) << c.name;
  }
}

// AC2 — the core claim of D8, MEASURED rather than reasoned about: a retuned preset is still
// the same preset to ClassifyAxisPreset, which is what keeps its button highlighted and its
// preview on the typical-view branch. Four presets, each at a value inside its own domain
// (the domains point in opposite directions, so one example would prove nothing about the
// other side).
TEST_F(UserDefaults, preset_identity_survives_retuning) {
  const auto dir = FreshOverlayDir("preset_identity");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  struct Case {
    gui::AxisPreset preset;
    float tuned_std;
  };
  // Column / Plate / Parry at 0.3 (the beta user's habit, tighter than factory); Lowitz at 20,
  // which is inside its (15, inf) domain and well away from the factory 40.
  const Case cases[] = {
    { gui::AxisPreset::kColumn, 0.3f },
    { gui::AxisPreset::kPlate, 0.3f },
    { gui::AxisPreset::kParry, 0.3f },
    { gui::AxisPreset::kLowitz, 20.0f },
  };

  for (const auto& c : cases) {
    const auto& entry = gui::AxisPresetEntryFor(c.preset);
    // Staged straight into the in-memory cache: every tuned_std here sits inside its own
    // domain, so this is the same state a committed edit would leave behind, minus the IO the
    // claim under test does not involve.
    gui::AdoptAxisPresetZenithStdOverrideInMemory(c.preset, c.tuned_std);

    // Exactly what the modal's preset button assembles into its edit buffer.
    const gui::AxisDist zenith = gui::EffectiveAxisPresetZenith(entry);
    EXPECT_EQ(zenith.std, c.tuned_std);
    EXPECT_EQ(static_cast<int>(gui::ClassifyAxisPreset(zenith, entry.azimuth, entry.roll)), static_cast<int>(c.preset));

    // The preview's typical-view branch is selected by the classified preset, so it follows
    // from the line above; asserted anyway because "preview goes somewhere else" is the
    // symptom a user would report, and it should fail HERE rather than as a screenshot.
    float rotation[16] = {};
    float typical[16] = {};
    gui::DefaultPreviewRotation(gui::ClassifyAxisPreset(zenith, entry.azimuth, entry.roll), nullptr, rotation);
    gui::DefaultPreviewRotation(c.preset, nullptr, typical);
    for (int i = 0; i < 16; ++i) {
      EXPECT_EQ(rotation[i], typical[i]);
    }
  }
}

// AC3 (store half) — an out-of-domain edit is clamped AND says so, in both directions, and an
// in-domain edit is left exactly as typed. Both states asserted: a clamp that fired on every
// write would satisfy a one-sided test while making the feature useless.
//
// Aimed at ClampAxisPresetZenithStdForSave, the single owner of both the clamp and its wording
// (defaults_panel.cpp is its only production caller). This case is the whole repo's only direct
// check of that wording, so the message assertions below are load-bearing, not decoration.
TEST_F(UserDefaults, preset_write_clamps_and_reports_both_states) {
  const auto dir = FreshOverlayDir("preset_write_clamp");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  // Above Column's (0, 10).
  const auto high = gui::ClampAxisPresetZenithStdForSave(gui::AxisPreset::kColumn, 25.0f);
  EXPECT_TRUE(high.accepted);
  EXPECT_TRUE(high.clamped);
  EXPECT_TRUE(high.stored_value < gui::kColumnPlateParryZenithStdUpperBound);
  EXPECT_TRUE(!high.message.empty());
  EXPECT_TRUE(high.message.find("Column") != std::string::npos);
  EXPECT_TRUE(high.message.find("25") != std::string::npos);
  // The copy must not claim the bound is physical — it is where the neighbouring criterion
  // starts. Plate's [10, 15) dead zone is the case that makes the distinction matter.
  EXPECT_TRUE(high.message.find("not a physical limit") != std::string::npos);
  // Still Column afterwards, which is the point of clamping rather than rejecting. Asserted on
  // the clamped value actually taking effect, not on the clamp result alone.
  gui::AdoptAxisPresetZenithStdOverrideInMemory(gui::AxisPreset::kColumn, high.stored_value);
  const auto& column = gui::AxisPresetEntryFor(gui::AxisPreset::kColumn);
  EXPECT_EQ(
      static_cast<int>(gui::ClassifyAxisPreset(gui::EffectiveAxisPresetZenith(column), column.azimuth, column.roll)),
      static_cast<int>(gui::AxisPreset::kColumn));

  // Below Lowitz's (15, inf) — the opposite side.
  const auto low = gui::ClampAxisPresetZenithStdForSave(gui::AxisPreset::kLowitz, 5.0f);
  EXPECT_TRUE(low.accepted);
  EXPECT_TRUE(low.clamped);
  EXPECT_TRUE(low.stored_value > gui::kLowitzZenithStdLowerBound);

  // In domain: stored verbatim, nothing reported.
  const auto ok = gui::ClampAxisPresetZenithStdForSave(gui::AxisPreset::kColumn, 0.3f);
  EXPECT_TRUE(ok.accepted);
  EXPECT_TRUE(!ok.clamped);
  EXPECT_EQ(ok.stored_value, 0.3f);
  EXPECT_TRUE(ok.message.empty());
}

// AC1 + AC5 — the whole life of a preset override on one staging: retune Column to the value the
// beta user asked for and confirm a fresh process reads it back (AC1), then restore it to factory
// and confirm the revert was surgical (AC5) — byte-identical to kAxisPresets, key gone from the
// file with its empty parents pruned, and NOTHING else disturbed, neither the other preset nor the
// GuiState half of the same file. That last part is what a wholesale rewrite would break.
//
// MakeNewDocumentState() is the "restart" throughout: it is the exact call startup makes, and it
// re-reads the file from disk. Write and revert share one file and one session on purpose — they
// are one user's sequence, and staging them separately said the same setup twice.
TEST_F(UserDefaults, preset_override_round_trips_across_a_reload_and_reverting_one_is_surgical) {
  const auto dir = FreshOverlayDir("preset_lifecycle");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  // A file that already carries a GuiState default, so the write path has a neighbour to
  // preserve rather than an empty document to overwrite.
  json seed;
  seed["bg_alpha"] = 0.42f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, seed));

  // Two independent read-modify-write cycles through WriteAxisPresetZenithStdToDoc, the owner
  // of the document shape the panel's Save also goes through.
  EXPECT_TRUE(SeedPresetOverrideOnDisk(dir, gui::AxisPreset::kColumn, 0.3f));
  EXPECT_TRUE(SeedPresetOverrideOnDisk(dir, gui::AxisPreset::kPlate, 0.5f));

  // What LANDED, read through this file's own raw reader rather than the production one so a
  // writer bug and a reader bug cannot cancel out into a pass. Taken after the SECOND write, which
  // is also how this says the second write did not take the first one with it: without that, the
  // whole case still passes when the write is a wholesale rewrite of the presets subtree —
  // measured with exactly that mutation.
  {
    const json after_two_writes = ReadOverlayDoc(dir);
    const auto kept_column = ReadPresetStd(after_two_writes, "column");
    const auto kept_plate = ReadPresetStd(after_two_writes, "plate");
    ASSERT_TRUE(kept_column.has_value());
    ASSERT_TRUE(kept_plate.has_value());
    EXPECT_EQ(*kept_column, 0.3f);
    EXPECT_EQ(*kept_plate, 0.5f);
    EXPECT_TRUE(after_two_writes.contains("bg_alpha"));
    EXPECT_EQ(after_two_writes.value("bg_alpha", 0.0f), 0.42f);
  }

  // The session picks the file up the way startup does — which is also what loads the value the
  // memory-side assertion after the revert is about, so that one is about a value that was really
  // there rather than one which never arrived.
  gui::MakeNewDocumentState(dir);
  const auto loaded = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(*loaded, 0.3f);
  // ...and that is what pressing the Column button would write into the crystal.
  EXPECT_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std, 0.3f);

  // Drop the in-memory state the way a restart would, then re-resolve: the value has to come back
  // off the disk rather than out of the cache the write left warm.
  gui::ResetUserAxisPresetOverrides();
  EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
  gui::MakeNewDocumentState(dir);
  const auto reloaded = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(reloaded.has_value());
  EXPECT_EQ(*reloaded, 0.3f);

  EXPECT_TRUE(RestoreOnePresetOnDisk(dir, gui::AxisPreset::kColumn));

  // Factory again, field by field against the table itself.
  const auto& column = gui::AxisPresetEntryFor(gui::AxisPreset::kColumn);
  EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
  const gui::AxisDist restored = gui::EffectiveAxisPresetZenith(column);
  EXPECT_EQ(static_cast<int>(restored.type), static_cast<int>(column.zenith.type));
  EXPECT_EQ(restored.mean, column.zenith.mean);
  EXPECT_EQ(restored.std, column.zenith.std);

  const json doc = ReadOverlayDoc(dir);
  EXPECT_TRUE(!ReadPresetStd(doc, "column").has_value());
  // Untouched: the other preset and the GuiState key sharing the file.
  const auto survivor = ReadPresetStd(doc, "plate");
  ASSERT_TRUE(survivor.has_value());
  EXPECT_EQ(*survivor, 0.5f);
  EXPECT_EQ(doc.value("bg_alpha", 0.0f), 0.42f);

  // Reverting the last preset prunes the now-empty parents, so a hand-opened file does not
  // accumulate `"presets": {"axis": {}}` skeletons.
  EXPECT_TRUE(RestoreOnePresetOnDisk(dir, gui::AxisPreset::kPlate));
  const json pruned = ReadOverlayDoc(dir);
  EXPECT_TRUE(!pruned.contains("presets"));
  EXPECT_EQ(pruned.value("bg_alpha", 0.0f), 0.42f);
}

// AC7 — Random has no adjustable face, so nothing about it may reach the file. Asserted at the
// STORE, not only at the UI: the UI not drawing an input is the first defense, and a defense
// that exists only in a widget is one a future refactor removes without noticing.
TEST_F(UserDefaults, preset_without_adjustable_face_is_never_written) {
  const auto dir = FreshOverlayDir("preset_random");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  json working = json::object();
  for (const auto preset : { gui::AxisPreset::kRandom, gui::AxisPreset::kCustom }) {
    const auto result = gui::ClampAxisPresetZenithStdForSave(preset, 3.0f);
    EXPECT_TRUE(!result.accepted);         // refused before any caller could commit it
    EXPECT_TRUE(!result.message.empty());  // refused out loud, not silently dropped
    EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(preset).has_value());
    // Both document mutators are no-ops for these presets, asserted in both directions: a write
    // must not invent a key, and an erase must not leave a `presets`/`axis` skeleton behind. They
    // are the primitives the panel's §1 controls call, so this is the store-level defense in the
    // place the panel would actually hit it.
    // One assertion per direction, not one after both: checking only the net effect of write+erase
    // would still pass if a future write DID invent a key, so long as the erase removed it again.
    gui::WriteAxisPresetZenithStdToDoc(working, preset, 3.0f);
    EXPECT_TRUE(working.empty());  // a write must not invent a key
    gui::EraseAxisPresetZenithStdFromDoc(working, preset);
    EXPECT_TRUE(working.empty());  // an erase must not leave a skeleton behind
  }

  // Nothing reached the file either.
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, working));
  const json doc = ReadOverlayDoc(dir);
  EXPECT_TRUE(!doc.contains("presets"));

  // And the table itself agrees about which presets those are, so this test cannot pass by
  // testing a set that has quietly stopped matching the one the UI renders.
  int adjustable = 0;
  for (const auto& entry : gui::kAxisPresets) {
    if (entry.has_adjustable_zenith_std) {
      ++adjustable;
      EXPECT_TRUE(entry.override_json_name != nullptr);
    } else {
      EXPECT_TRUE(entry.override_json_name == nullptr);
    }
  }
  EXPECT_EQ(adjustable, 4);  // Column / Plate / Parry / Lowitz
}

// MOVED, not retired: "a failed write leaves the in-memory preset value alone" now lives in
// test/gui/functional/test_defaults_panel.cpp as
// defaults_panel/a_save_that_could_not_write_leaves_the_preset_cache_alone.
//
// It used to be asserted here against a disk-side revert wrapper the panel never called. The
// contract is not a property of any one function — it is the ORDER in which the panel's Save
// composes two of them (write the document, and only if that succeeded push the values into the
// process-wide cache), so the only place it can be observed is the composition itself. See the
// comment on that case for why it is driven through the real Save button.

// AC4 — a load-time clamp must REACH the user, not merely be counted. 405.2 built the
// counters; until this task nothing consumed them, so a value silently adjusted at startup
// was indistinguishable from one that had been dropped. Asserted through the same one-shot
// warning surface an import degradation uses, from the production DoNew() path.
TEST_F(UserDefaults, ALoadTimeClampTellsTheUserWhichValueItReplacedAndWithWhat) {
  struct SurfacedCase {
    const char* name;
    const char* preset_key;
    const char* preset_label;  // as the warning names it
    float stored;
    const char* stored_text;
    const char* clamped_text;
  };
  const SurfacedCase kCases[] = {
    { "above Column's upper bound", "column", "Column", 25.0f, "25", "9.99999" },
    // The mirror direction, and not a courtesy duplicate: the panel's std cell renders with a
    // fixed "%.7g" while this notice uses adaptive precision. nextafter(10, 0) needs 7
    // significant digits and happened to survive the fixed format; nextafter(15, +inf) needs 8 and
    // collapsed to bare "15" under it — printing the clamped value as the very boundary the
    // neighbouring sentence says it must stay above. Only this direction could catch that.
    { "below Lowitz's lower bound", "lowitz", "Lowitz", 5.0f, "5", "15.000001" },
  };

  for (const SurfacedCase& c : kCases) {
    ResetUserDefaultsChannels();
    gui::ClearImportComplexFilterWarning();
    json doc;
    doc["presets"]["axis"][c.preset_key]["zenith_std"] = c.stored;
    const auto dir = DirWith("clamp_surfaced", doc);
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

    gui::DoNew();

    const std::string warning = gui::PeekImportComplexFilterWarning();
    EXPECT_FALSE(warning.empty()) << c.name;
    EXPECT_NE(warning.find(c.preset_label), std::string::npos) << c.name;
    EXPECT_NE(warning.find(c.stored_text), std::string::npos) << c.name;
    // And the value they actually got, not merely that something was wrong.
    EXPECT_NE(warning.find(c.clamped_text), std::string::npos) << c.name << ": got \"" << warning << "\"";
    gui::ClearImportComplexFilterWarning();
  }

  // The negative half: a clean override file, and a first run with no file at all, must produce
  // NO popup. Without them, a notice that fired on every New would satisfy the rows above while
  // training the user to dismiss the dialog unread — which is the same as not having one.
  json clean;
  clean["presets"]["axis"]["column"]["zenith_std"] = 0.3f;  // inside the domain
  clean["bg_alpha"] = 0.42f;
  for (const auto& quiet_dir : { DirWith("clamp_quiet", clean), FreshOverlayDir("clamp_quiet_empty") }) {
    ResetUserDefaultsChannels();
    gui::ClearImportComplexFilterWarning();
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, quiet_dir);
    gui::DoNew();
    EXPECT_TRUE(gui::PeekImportComplexFilterWarning().empty()) << quiet_dir;
  }
}

// The JSON-import path reads the override file too, but DeserializeFromJson opens with
// `state = GuiState{}` — so nothing of the personal defaults survives it. Reporting a
// degradation there would describe a document the user is not getting; leaving the counters
// filled would misattribute them to the NEXT New. Draining is the only correct handling, and
// this pins it: the import must leave the channel empty.
TEST_F(UserDefaults, json_import_does_not_leak_downgrades_into_the_next_new) {
  gui::ClearImportComplexFilterWarning();
  const auto dir = FreshOverlayDir("import_leak");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  json doc;
  doc["presets"]["axis"]["column"]["zenith_std"] = 25.0f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  // A minimal but valid CLI config, so DoOpen takes the import branch to completion.
  const std::filesystem::path config = dir / "imported.json";
  {
    std::ofstream out(config, std::ios::trunc);
    out << R"({"sun": {"altitude": 25.0}})";
  }
  gui::DoOpen(config);
  // Whatever the import path read is gone, reported or not.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
  EXPECT_TRUE(gui::TakeUserDefaultsDowngradeNotices().empty());

  // The decisive part: the NEXT New reports its own load, and the file is clean by then. HOW the
  // file gets cleaned is not the subject here, so it is emptied directly rather than through a
  // preset-shaped edit.
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, json::object()));
  gui::ClearImportComplexFilterWarning();
  gui::DoNew();
  EXPECT_TRUE(gui::PeekImportComplexFilterWarning().empty());
}

// ================================================================================
// bg_path as a personal default
// ================================================================================
TEST_F(UserDefaults, bg_path_default_degrades_when_missing) {
  const auto dir = FreshOverlayDir("bg");

  json doc;
  doc["bg_path"] = (dir / "no_such_background.png").string();
  doc["bg_show"] = true;
  doc["aspect_ratio"] = "match_background";
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  gui::GuiState state = gui::MakeNewDocumentState(dir);
  EXPECT_TRUE(!state.bg_path.empty());  // the default did land in the state...
  EXPECT_EQ(state.bg_show, true);       // ...and asked for the background to be shown
  EXPECT_EQ(static_cast<int>(state.aspect_preset), static_cast<int>(gui::AspectPreset::kMatchBg));

  // ...but the file is gone. The New path must degrade rather than throw or hang: this runs
  // during startup, where a personal default pointing at a since-deleted image would
  // otherwise be able to stop the app from opening.
  gui::LoadBackgroundWithDegrade(state);
  EXPECT_EQ(state.bg_show, false);
  EXPECT_EQ(static_cast<int>(state.aspect_preset), static_cast<int>(gui::AspectPreset::kFree));

  // An empty bg_path is a plain no-op — the overwhelmingly common case must not be made to
  // pay for the degrade path (and must not clobber an unrelated aspect preset).
  gui::GuiState empty{};
  empty.bg_show = true;
  empty.aspect_preset = gui::AspectPreset::kMatchBg;
  gui::LoadBackgroundWithDegrade(empty);
  EXPECT_EQ(empty.bg_show, true);
  EXPECT_EQ(static_cast<int>(empty.aspect_preset), static_cast<int>(gui::AspectPreset::kMatchBg));
}

// Structural gate: the degrade step must be wired into every path that can land a non-empty
// bg_path. Calling the helper directly (above) proves it works; this proves it is reached.
// Same technique as the reset-primitive single-owner gate, which now lives in
// test/composition-correctness/gui/test_document_switch_chain.cpp as
// DocumentSwitchChain.TheResetPrimitivesHaveExactlyOneCallSite.
TEST_F(UserDefaults, bg_degrade_is_wired_into_every_document_path) {
  std::ifstream in(LUMICE_GUI_APP_CPP_PATH);
  EXPECT_TRUE(in.is_open());
  const std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_TRUE(!src.empty());

  int calls = 0;
  size_t pos = 0;
  const std::string needle = "LoadBackgroundWithDegrade(g_state);";
  while ((pos = src.find(needle, pos)) != std::string::npos) {
    ++calls;
    pos += needle.size();
  }
  // Three document-entry paths: DoOpen(.lmc), DoOpen(.json import), DoNew.
  EXPECT_EQ(calls, 3);

  // And the old inline copy must be gone — LoadAndUploadBgImage should now only be reached
  // from the shared helper and from the explicit "Load Background..." command.
  int raw_loads = 0;
  pos = 0;
  const std::string raw = "LoadAndUploadBgImage(";
  while ((pos = src.find(raw, pos)) != std::string::npos) {
    ++raw_loads;
    pos += raw.size();
  }
  EXPECT_EQ(raw_loads, 3);  // 1 definition + LoadBackgroundWithDegrade + DoLoadBackground
}
