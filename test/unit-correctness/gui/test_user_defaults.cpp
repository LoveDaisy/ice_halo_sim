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
// pinned in gui_test (save_failure_leaves_the_preset_cache_untouched) — a second place asserting
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

// ================================================================================
// AC2 — `kView \ serialized` recomputed against the real serializer
// ================================================================================
TEST(UserDefaults, ac2_kview_difference_set_recomputed) {
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
TEST(UserDefaults, ac3_sparse_overlay_round_trip) {
  ResetUserDefaultsChannels();
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
}

// The most common path of all: no override file yet. It must be a silent no-op, NOT a
// degradation — counting it would fire the "your defaults were downgraded" notice on every
// fresh install.
TEST(UserDefaults, ac3_absent_file_is_not_a_downgrade) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("absent");

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  const gui::GuiState factory = gui::InitDefaultState();
  EXPECT_EQ(state.renderer.lens_type, factory.renderer.lens_type);
  EXPECT_EQ(state.bg_alpha, factory.bg_alpha);
  EXPECT_EQ(state.crystals.size(), factory.crystals.size());
  EXPECT_EQ(state.layers.size(), factory.layers.size());
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
}

// Eligibility must be enforced on the READ path, not merely recorded as metadata: the
// override file is a text file the user can hand-edit.
TEST(UserDefaults, ac3_ineligible_keys_cannot_be_smuggled_in) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("ineligible");

  json doc;
  // namespace 3: an ordinary serializable scalar the deserializer would happily read.
  doc["use_gpu_backend"] = true;
  // namespace 4: a collection. A hand-edited file must not make New start with someone
  // else's scene.
  doc["layers"] = json::array();
  doc["layers"].push_back({ { "prob", 0.5f }, { "entries", json::array() } });
  doc["layers"].push_back({ { "prob", 0.5f }, { "entries", json::array() } });
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  const gui::GuiState factory{};
  EXPECT_EQ(state.use_gpu_backend, factory.use_gpu_backend);
  // Seeded contents only: exactly one layer with one entry, one crystal, no filters.
  EXPECT_EQ(state.layers.size(), static_cast<size_t>(1));
  EXPECT_EQ(state.layers[0].entries.size(), static_cast<size_t>(1));
  EXPECT_EQ(state.crystals.size(), static_cast<size_t>(1));
  EXPECT_EQ(state.filters.size(), static_cast<size_t>(0));
}

// code-review round 1 Major: raypath_color is namespace 4 (kCollectionFields) exactly like
// layers/crystals/filters, but MakeNewDocumentState only cleared the other three. A
// hand-edited override file could smuggle a color-class list into every new document.
TEST(UserDefaults, ac3_raypath_color_cannot_be_smuggled_in) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("raypath_color_smuggle");

  json doc;
  // Bare-array wire form (DeserializeGuiStateJson accepts it directly under
  // "raypath_color"; see file_io.cpp's parse block).
  doc["raypath_color"] = json::array();
  doc["raypath_color"].push_back({ { "color", { 1.0f, 0.0f, 0.0f } } });
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  const gui::GuiState factory{};
  EXPECT_EQ(state.raypath_color.size(), factory.raypath_color.size());
  EXPECT_EQ(state.raypath_color.size(), static_cast<size_t>(0));
}

// --no-user-config and --user-config <empty dir> must be the same document. Both are "the
// user has saved nothing", reached by two different routes, and a user who cannot get the
// same result twice has no way to establish a factory baseline.
TEST(UserDefaults, switch_disabled_equals_empty_explicit_dir) {
  ResetUserDefaultsChannels();
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
}

// --user-config pointing at a directory whose file is corrupt: the switch must route into
// the existing degradation path, not around it.
TEST(UserDefaults, switch_explicit_dir_with_broken_file_degrades) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("switch_broken");
  WriteRawOverlay(dir, "{ this is not json");

  gui::GuiState state;
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
    state = gui::MakeNewDocumentState();
  }
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
  const gui::GuiState factory = gui::InitDefaultState();
  EXPECT_EQ(state.renderer.lens_type, factory.renderer.lens_type);
  EXPECT_EQ(state.bg_alpha, factory.bg_alpha);
}

// --user-config <path to a regular file> — the flag takes a DIRECTORY, and a user who points
// it at the user_defaults.json itself must get a degraded startup, not a crash.
TEST(UserDefaults, switch_explicit_dir_pointing_at_a_file_is_survivable) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("switch_not_a_dir");
  const std::filesystem::path file = dir / "plain_file.txt";
  {
    std::ofstream out(file, std::ios::trunc);
    out << "not a directory";
  }

  gui::GuiState state;
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, file);
    state = gui::MakeNewDocumentState();
  }
  const gui::GuiState factory = gui::InitDefaultState();
  EXPECT_EQ(state.renderer.lens_type, factory.renderer.lens_type);
  EXPECT_EQ(state.crystals.size(), factory.crystals.size());
}

// The harness's own baseline, asserted from inside the harness. This binary installs
// kTestHarnessUserConfigDefault before any test runs (gui_unit_test_env.cpp; gui_test does the
// same from its main()); if that install were dropped or flipped, this is the case that says so.
// It reads the switch through its only observable effect — whether a real override file on disk
// reaches a new document — because the source enum itself has no getter, by design (one owner).
TEST(UserDefaults, switch_harness_baseline_is_isolated) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("switch_baseline");
  json doc;
  doc["bg_alpha"] = 0.125f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  // No ScopedUserConfigSource here on purpose: this is the process state the harness install
  // left behind, i.e. what every other scenario in this binary runs under.
  const gui::GuiState state = gui::MakeNewDocumentState();
  const gui::GuiState factory = gui::InitDefaultState();
  EXPECT_EQ(state.bg_alpha, factory.bg_alpha);

  // Detection power for the assertion above: the file IS readable and DOES carry a value
  // distinguishable from factory — it simply must not be consulted at the baseline.
  {
    ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
    const gui::GuiState reachable = gui::MakeNewDocumentState();
    EXPECT_EQ(reachable.bg_alpha, 0.125f);
    EXPECT_TRUE(reachable.bg_alpha != factory.bg_alpha);
  }
}

// ================================================================================
// AC4 — invariant I1
// ================================================================================
TEST(UserDefaults, ac4_file_value_beats_personal_default) {
  ResetUserDefaultsChannels();
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
}

// A real-GUI manual walkthrough ("set a user_defaults.json, start the GUI, observe New picking
// up the default and .lmc Open unaffected") is a one-time human check that leaves no trace and
// has to be redone by hand every time this file changes. This test programmatically covers the
// same assertion pair instead — the two halves already exist separately in
// ac3_sparse_overlay_round_trip and ac4_file_value_beats_personal_default; this is the pair in
// one place, self-contained.
TEST(UserDefaults, m2_new_document_picks_up_personal_default_and_lmc_open_is_unaffected) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("m2_walkthrough");

  json doc;
  doc["bg_alpha"] = 0.33f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  // "New document picks up the personal default."
  const gui::GuiState new_doc = gui::MakeNewDocumentState(dir);
  EXPECT_EQ(new_doc.bg_alpha, 0.33f);

  // "Opening an .lmc is unaffected" — an explicit file that does not mention bg_alpha must
  // load the FACTORY value, never the personal default that New just picked up in this same
  // process. DeserializeGuiStateJson is the .lmc/.json load path; it never consults
  // MakeNewDocumentState() or the personal-default store.
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
TEST(UserDefaults, ac5_unknown_key_is_ignored_silently) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("ac5_unknown");

  json doc;
  doc["totally_unknown_key"] = 1;
  doc["bg_alpha"] = 0.75f;  // a known key alongside it must still apply
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  EXPECT_EQ(state.bg_alpha, 0.75f);
  // Forward compatibility: a key this build does not know about is the normal shape of a
  // file written by a NEWER build. Ignoring it is correct and is not a degradation.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
}

TEST(UserDefaults, ac5_wrong_type_degrades_without_partial_apply) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("ac5_type");

  // `bg_alpha` is a float; a string there makes the deserializer throw mid-merge.
  WriteRawOverlay(dir, R"({"bg_alpha": "not_a_number", "aspect_portrait": true})");

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  const gui::GuiState factory{};
  // The WHOLE overlay is discarded, including the well-formed sibling key: a half-applied
  // set of defaults is harder to reason about (and to revert) than none at all.
  EXPECT_EQ(state.bg_alpha, factory.bg_alpha);
  EXPECT_EQ(state.aspect_portrait, factory.aspect_portrait);
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
  // Still a usable new document.
  EXPECT_EQ(state.layers.size(), static_cast<size_t>(1));
}

TEST(UserDefaults, ac5_invalid_json_degrades) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("ac5_json");

  WriteRawOverlay(dir, R"({"bg_alpha": 0.5, )");  // truncated

  const gui::GuiState state = gui::MakeNewDocumentState(dir);
  const gui::GuiState factory{};
  EXPECT_EQ(state.bg_alpha, factory.bg_alpha);
  EXPECT_EQ(state.layers.size(), static_cast<size_t>(1));
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);

  // A JSON scalar / array at the root is also not a usable document.
  ResetUserDefaultsChannels();
  WriteRawOverlay(dir, "[1, 2, 3]");
  const gui::GuiState state2 = gui::MakeNewDocumentState(dir);
  EXPECT_EQ(state2.bg_alpha, factory.bg_alpha);
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
}

// ================================================================================
// D8 — load-time clamp into the classifier's tolerance domain, with a visible trace
// ================================================================================
TEST(UserDefaults, d8_preset_override_clamped_with_notice) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("d8_upper");

  // Column's domain is (0, 10); 25 is above it (hand-edited, or from a build whose
  // classifier was wider).
  json doc;
  doc["presets"]["axis"]["column"]["zenith_std"] = 25.0f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
  gui::MakeNewDocumentState(dir);

  const auto column = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(column.has_value());
  // Clamped to the largest float STRICTLY below the bound: landing on 10.0 exactly would
  // fail the classifier's strict `< 10` and demote the preset to Custom.
  EXPECT_EQ(*column, std::nextafter(gui::kColumnPlateParryZenithStdUpperBound, 0.0f));
  EXPECT_TRUE(*column < gui::kColumnPlateParryZenithStdUpperBound);

  // The clamp must leave a trace naming the key and both values — at load time the user is
  // not looking at the preset panel, so a silent clamp is indistinguishable from a silent
  // drop (the exact failure family this system exists to avoid).
  const auto notices = gui::TakeUserDefaultsDowngradeNotices();
  EXPECT_EQ(notices.size(), static_cast<size_t>(1));
  EXPECT_TRUE(notices[0].find("column") != std::string::npos);
  EXPECT_TRUE(notices[0].find("25") != std::string::npos);
  // Consumed-on-read, like the shape-dist downgrade counter it mirrors.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(0));
}

TEST(UserDefaults, d8_lowitz_clamps_on_the_lower_side) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("d8_lower");

  // Lowitz's domain is (15, inf) — the opposite side from Column/Plate/Parry.
  json doc;
  doc["presets"]["axis"]["lowitz"]["zenith_std"] = 5.0f;
  // Non-positive is outside Column's (0, 10) too — the domain is open at BOTH ends there,
  // and a zero/negative std would otherwise reach the sampler.
  doc["presets"]["axis"]["plate"]["zenith_std"] = -3.0f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
  gui::MakeNewDocumentState(dir);

  const auto lowitz = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kLowitz);
  ASSERT_TRUE(lowitz.has_value());
  EXPECT_EQ(*lowitz, std::nextafter(gui::kLowitzZenithStdLowerBound, std::numeric_limits<float>::infinity()));
  EXPECT_TRUE(*lowitz > gui::kLowitzZenithStdLowerBound);

  const auto plate = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kPlate);
  ASSERT_TRUE(plate.has_value());
  EXPECT_TRUE(*plate > 0.0f);
  EXPECT_TRUE(*plate < gui::kColumnPlateParryZenithStdUpperBound);

  EXPECT_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(2));
}

// code-review round 2 Major: ApplyAxisPresetOverridesFromJson only ever added/updated slots,
// never cleared one whose entry disappeared from the file. MakeNewDocumentState() is called
// repeatedly within one process (main.cpp startup, every DoNew(), every DoOpen() .json
// import), so a stale g_axis_overrides slot would keep answering with a deleted override
// until the process restarts. This test calls MakeNewDocumentState() twice against the same
// directory within a single test case (deliberately not calling ResetUserDefaultsChannels()
// between the two calls) to reproduce the same-process, file-changed-underneath scenario.
TEST(UserDefaults, d8_preset_override_does_not_leak_across_calls) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("d8_leak");

  json doc;
  doc["presets"]["axis"]["column"]["zenith_std"] = 3.0f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
  gui::MakeNewDocumentState(dir);
  const auto first = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, 3.0f);

  // The user edits the file underneath the running process and removes the override —
  // no call to ResetUserAxisPresetOverrides() here, matching production (only
  // ApplyAxisPresetOverridesFromJson's internal reset should apply).
  json empty_doc = json::object();
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, empty_doc));
  gui::MakeNewDocumentState(dir);
  const auto second = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  EXPECT_TRUE(!second.has_value());
}

// code-review round 4 Major: the leak test above always passes an explicit override_dir, so
// `dir` is truthy on both calls and never exercises the branch where GetUserConfigDir() itself
// (not an explicit override_dir) resolves to nullopt. That is exactly the gap: the previous fix
// reset g_axis_overrides only inside `if (dir)`, so a process where the FIRST
// MakeNewDocumentState() call loaded a preset override and a LATER call's GetUserConfigDir()
// becomes unavailable (its own doc comment says this can happen at runtime — directory removed
// out from under the process, permissions revoked, disk full) would keep answering with the
// first call's override forever. This test forces that exact flip via the no-arg production
// call path (main.cpp / DoNew() / DoOpen() all call MakeNewDocumentState() with no argument).
TEST(UserDefaults, d8_preset_override_does_not_leak_when_config_dir_becomes_unavailable) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("d8_leak_no_dir");

  json doc;
  doc["presets"]["axis"]["column"]["zenith_std"] = 3.0f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));
  gui::MakeNewDocumentState(dir);
  const auto first = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, 3.0f);

  // Same process, same override still loaded — but this call's GetUserConfigDir() now
  // resolves to nullopt, the no-arg production call path, not an explicit override_dir.
  //
  // The kAutoDetect guard is what keeps this test testing what it says. The harness baseline
  // is kDisabled, and under kDisabled the no-arg path returns nullopt without ever calling
  // GetUserConfigDir() — the case would still pass (same verdict, by a different route) while
  // silently no longer exercising "GetUserConfigDir() went unavailable mid-process", the
  // regression it was written for. Restoring kAutoDetect for this scope re-arms
  // ScopedNoUserConfigDirEnv, which only has an effect on that branch.
  {
    ScopedUserConfigSource auto_detect(gui::UserConfigSource::kAutoDetect);
    ScopedNoUserConfigDirEnv no_config_dir;
    gui::MakeNewDocumentState(std::nullopt);
  }
  const auto second = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  EXPECT_TRUE(!second.has_value());
}

TEST(UserDefaults, d8_malformed_preset_value_degrades) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("d8_bad");

  WriteRawOverlay(dir, R"({"presets": {"axis": {"column": {"zenith_std": "wide"}}}})");
  gui::MakeNewDocumentState(dir);

  EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
  // A rejected value is a drop, not a clamp — the two channels must not be conflated.
  EXPECT_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(0));
}

// AC1 — the round trip a user actually performs: retune Column to the value the beta user
// asked for, then confirm a fresh process would read it back. MakeNewDocumentState() is the
// "restart" here: it is the exact call startup makes, and it re-reads the file from disk.
TEST(UserDefaults, preset_write_round_trips_across_a_reload) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("preset_roundtrip");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  EXPECT_TRUE(SeedPresetOverrideOnDisk(dir, gui::AxisPreset::kColumn, 0.3f));

  // On disk under the key 405.2's reader already looks for. Asserted through this file's own
  // raw reader rather than the production one, so a writer bug and a reader bug cannot cancel.
  const json doc = ReadOverlayDoc(dir);
  EXPECT_TRUE(doc.contains("presets"));
  const auto stored_column = ReadPresetStd(doc, "column");
  ASSERT_TRUE(stored_column.has_value());
  EXPECT_EQ(*stored_column, 0.3f);

  // The session picks it up the way startup does.
  gui::MakeNewDocumentState(dir);
  const auto loaded = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(*loaded, 0.3f);

  // Drop the in-memory state the way a restart would, then re-resolve from the file.
  gui::ResetUserAxisPresetOverrides();
  EXPECT_TRUE(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
  gui::MakeNewDocumentState(dir);

  const auto reloaded = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
  ASSERT_TRUE(reloaded.has_value());
  EXPECT_EQ(*reloaded, 0.3f);

  // And that is what pressing the Column button would write into the crystal.
  EXPECT_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std, 0.3f);
}

// AC2 — the core claim of D8, MEASURED rather than reasoned about: a retuned preset is still
// the same preset to ClassifyAxisPreset, which is what keeps its button highlighted and its
// preview on the typical-view branch. Four presets, each at a value inside its own domain
// (the domains point in opposite directions, so one example would prove nothing about the
// other side).
TEST(UserDefaults, preset_identity_survives_retuning) {
  ResetUserDefaultsChannels();
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
TEST(UserDefaults, preset_write_clamps_and_reports_both_states) {
  ResetUserDefaultsChannels();
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

// AC5 — restore one preset to factory: byte-identical to kAxisPresets, key gone from the file
// (with its empty parents pruned), and NOTHING else disturbed — neither another preset nor the
// GuiState half of the same file. The last part is what a wholesale rewrite would break.
TEST(UserDefaults, preset_restore_to_factory_is_surgical) {
  ResetUserDefaultsChannels();
  const auto dir = FreshOverlayDir("preset_restore");
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
  // Load them into the session, so the memory-side assertion after the revert below is about a
  // value that was really there rather than one that never arrived.
  gui::MakeNewDocumentState(dir);
  EXPECT_TRUE(gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());

  // Asserted after the SECOND write, before any revert: the second write must not have taken
  // the first one with it. Without this line the whole case still passes when the write is a
  // wholesale rewrite of the presets subtree — measured with exactly that mutation, which is
  // why the assertion is here and not left implied by the revert checks below.
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
TEST(UserDefaults, preset_without_adjustable_face_is_never_written) {
  ResetUserDefaultsChannels();
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
// test/gui/functional/test_gui_defaults_panel.cpp as
// save_failure_leaves_the_preset_cache_untouched.
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
TEST(UserDefaults, load_time_clamp_reaches_the_user) {
  ResetUserDefaultsChannels();
  gui::ClearImportComplexFilterWarning();
  const auto dir = FreshOverlayDir("clamp_surfaced");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  json doc;
  doc["presets"]["axis"]["column"]["zenith_std"] = 25.0f;  // above Column's (0, 10)
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  gui::DoNew();

  const std::string warning = gui::PeekImportComplexFilterWarning();
  EXPECT_TRUE(!warning.empty());
  EXPECT_TRUE(warning.find("Column") != std::string::npos);
  EXPECT_TRUE(warning.find("25") != std::string::npos);
  // The clamped value itself, so the user learns what they actually got rather than only that
  // something was wrong.
  EXPECT_TRUE(warning.find("9.99999") != std::string::npos);
  gui::ClearImportComplexFilterWarning();
}

// The other clamp direction: Lowitz's domain is a LOWER bound (must stay greater than 15),
// the mirror image of Column's upper bound above. A prior code-review round found that the
// §1 panel's std cell used a fixed "%.7g" while this load-time notice (via FormatAxisPresetStd)
// used adaptive precision — nextafter(10, 0) needs 7 digits and happened to work under the
// fixed format, but nextafter(15, +inf) needs 8 and collapsed to "15" under it, printing the
// clamped value as the boundary itself. Column's direction alone could not have caught that:
// both call sites need exercising on the direction that actually needs the extra digit.
TEST(UserDefaults, load_time_clamp_reaches_the_user_lowitz_lower_bound) {
  ResetUserDefaultsChannels();
  gui::ClearImportComplexFilterWarning();
  const auto dir = FreshOverlayDir("clamp_surfaced_lowitz");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  json doc;
  doc["presets"]["axis"]["lowitz"]["zenith_std"] = 5.0f;  // below Lowitz's (15, inf)
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  gui::DoNew();

  const std::string warning = gui::PeekImportComplexFilterWarning();
  EXPECT_TRUE(!warning.empty());
  EXPECT_TRUE(warning.find("Lowitz") != std::string::npos);
  EXPECT_TRUE(warning.find("5") != std::string::npos);
  // The full clamped value, not the boundary alone: nextafter(15, +inf) needs 8 significant
  // digits ("15.000001"), so a warning reading bare "15" would mean the display collapsed the
  // stored value into the very boundary the neighbouring sentence says it must stay above.
  EXPECT_TRUE(warning.find("15.000001") != std::string::npos);
  gui::ClearImportComplexFilterWarning();
}

// The negative half of AC4: a clean override file must produce NO popup. Without this, a
// notice that fired on every New would satisfy the case above while training the user to
// dismiss the dialog unread — which is the same as not having one.
TEST(UserDefaults, a_clean_load_surfaces_nothing) {
  ResetUserDefaultsChannels();
  gui::ClearImportComplexFilterWarning();
  const auto dir = FreshOverlayDir("clamp_quiet");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

  json doc;
  doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;  // inside the domain
  doc["bg_alpha"] = 0.42f;
  EXPECT_TRUE(gui::WriteUserDefaultsFile(dir, doc));

  gui::DoNew();
  EXPECT_TRUE(gui::PeekImportComplexFilterWarning().empty());

  // A first run with no file at all is the most common case of all, and must be just as quiet.
  const auto empty_dir = FreshOverlayDir("clamp_quiet_empty");
  ScopedUserConfigSource empty_guard(gui::UserConfigSource::kExplicitDir, empty_dir);
  gui::DoNew();
  EXPECT_TRUE(gui::PeekImportComplexFilterWarning().empty());
}

// The JSON-import path reads the override file too, but DeserializeFromJson opens with
// `state = GuiState{}` — so nothing of the personal defaults survives it. Reporting a
// degradation there would describe a document the user is not getting; leaving the counters
// filled would misattribute them to the NEXT New. Draining is the only correct handling, and
// this pins it: the import must leave the channel empty.
TEST(UserDefaults, json_import_does_not_leak_downgrades_into_the_next_new) {
  ResetUserDefaultsChannels();
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
TEST(UserDefaults, bg_path_default_degrades_when_missing) {
  ResetUserDefaultsChannels();
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
// Same technique as the reset-primitive single-owner gate in test_gui_import_export.cpp.
TEST(UserDefaults, bg_degrade_is_wired_into_every_document_path) {
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
