// User-defaults store tests.
//
// Covers the half of src/gui/user_defaults.hpp that calls into file_io.cpp, which is only
// linkable from gui_test (unit_correctness_test links lumice_obj, not lumice_gui_obj):
//   AC2 — one-hand recomputation of `kView \ SerializeGuiStateJson` against the real serializer
//   AC3 — sparse override round-trip (singleton config + preset library), untouched keys stay factory
//   AC4 — invariant I1: personal defaults reach New, never a value present in a loaded file
//   AC5 — a broken override file degrades (unknown key / wrong type / invalid JSON), never crashes
//   D8  — load-time clamp of preset values into the classifier's tolerance domain, never silent
//   bg_path — a personal default pointing at a missing image degrades instead of blocking startup
//
// The eligibility resolver and the platform config-dir helpers are pure and header-only; they
// are covered in test/unit-correctness/gui/test_user_defaults_eligibility.cpp.

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
#include "gui/gui_state_tiers.hpp"
#include "gui/user_defaults.hpp"
#include "test_gui_shared.hpp"
#include "user_defaults_test_env.hpp"

namespace {

using nlohmann::json;

// Isolation policy (fresh temp directory per case, scope guards that restore the harness
// baseline) lives in one place for every gui_test source that touches the store; see
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

}  // namespace

void RegisterUserDefaultsTests(ImGuiTestEngine* engine) {
  // ================================================================================
  // AC2 — `kView \ serialized` recomputed against the real serializer
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac2_kview_difference_set_recomputed");
    t->TestFunc = [](ImGuiTestContext*) {
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
      IM_CHECK(root.is_object());

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
      IM_CHECK_EQ(table_fields.size(), tier_view_fields.size());
      IM_CHECK(table_fields == tier_view_fields);

      // 2. Each claim holds against the actual serializer output.
      std::set<std::string> recomputed_difference;
      for (const auto& row : kViewFieldKeys) {
        if (row.json_key == nullptr) {
          recomputed_difference.insert(row.field);
          continue;
        }
        IM_CHECK(root.contains(row.json_key));
      }

      // 3. The recomputed difference set must equal the production constant that eligibility
      //    is derived from. This is the assertion the design doc's hand-eyeballed list is
      //    replaced by.
      std::set<std::string> constant_difference;
      for (const char* name : gui::kUnserializedViewFields) {
        constant_difference.insert(name);
      }
      IM_CHECK(recomputed_difference == constant_difference);
      IM_CHECK_EQ(recomputed_difference.size(), static_cast<size_t>(5));
    };
  }

  // ================================================================================
  // AC3 — sparse round-trip
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac3_sparse_overlay_round_trip");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("ac3");

      // A genuinely sparse document: one namespace-1 singleton key, one namespace-1 kView key,
      // and one namespace-2 preset-library entry. Everything else must come back factory.
      json doc;
      doc["renderer"]["lens_type"] = "fisheye_equal_area";
      doc["bg_alpha"] = 0.25f;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;  // the tighter value users asked for
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      IM_CHECK_EQ(state.renderer.lens_type, gui::kLensTypeFisheyeEqualArea);
      IM_CHECK_EQ(state.bg_alpha, 0.25f);

      // An untouched key reads back as the factory value, not as whatever the writer omitted.
      const gui::GuiState factory{};
      IM_CHECK_EQ(state.aspect_portrait, factory.aspect_portrait);
      IM_CHECK_EQ(state.sun.altitude, factory.sun.altitude);
      IM_CHECK_EQ(state.renderer.fov, factory.renderer.fov);

      // Preset-library half, read through its own query interface.
      const auto column = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(column.has_value());
      IM_CHECK_EQ(*column, 0.3f);
      // A preset with no entry stays "no override" rather than defaulting to something.
      IM_CHECK(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kLowitz).has_value());

      // A well-formed file is not a degradation, and an in-domain value is not a clamp.
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(0));
    };
  }

  {
    // The most common path of all: no override file yet. It must be a silent no-op, NOT a
    // degradation — counting it would fire the "your defaults were downgraded" notice on every
    // fresh install.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac3_absent_file_is_not_a_downgrade");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("absent");

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      const gui::GuiState factory = gui::InitDefaultState();
      IM_CHECK_EQ(state.renderer.lens_type, factory.renderer.lens_type);
      IM_CHECK_EQ(state.bg_alpha, factory.bg_alpha);
      IM_CHECK_EQ(state.crystals.size(), factory.crystals.size());
      IM_CHECK_EQ(state.layers.size(), factory.layers.size());
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
    };
  }

  {
    // Eligibility must be enforced on the READ path, not merely recorded as metadata: the
    // override file is a text file the user can hand-edit.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac3_ineligible_keys_cannot_be_smuggled_in");
    t->TestFunc = [](ImGuiTestContext*) {
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
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      const gui::GuiState factory{};
      IM_CHECK_EQ(state.use_gpu_backend, factory.use_gpu_backend);
      // Seeded contents only: exactly one layer with one entry, one crystal, no filters.
      IM_CHECK_EQ(state.layers.size(), static_cast<size_t>(1));
      IM_CHECK_EQ(state.layers[0].entries.size(), static_cast<size_t>(1));
      IM_CHECK_EQ(state.crystals.size(), static_cast<size_t>(1));
      IM_CHECK_EQ(state.filters.size(), static_cast<size_t>(0));
    };
  }

  // code-review round 1 Major: raypath_color is namespace 4 (kCollectionFields) exactly like
  // layers/crystals/filters, but MakeNewDocumentState only cleared the other three. A
  // hand-edited override file could smuggle a color-class list into every new document.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac3_raypath_color_cannot_be_smuggled_in");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("raypath_color_smuggle");

      json doc;
      // Bare-array wire form (DeserializeGuiStateJson accepts it directly under
      // "raypath_color"; see file_io.cpp's parse block).
      doc["raypath_color"] = json::array();
      doc["raypath_color"].push_back({ { "color", { 1.0f, 0.0f, 0.0f } } });
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      const gui::GuiState factory{};
      IM_CHECK_EQ(state.raypath_color.size(), factory.raypath_color.size());
      IM_CHECK_EQ(state.raypath_color.size(), static_cast<size_t>(0));
    };
  }

  // ================================================================================
  // The process-wide source switch (--user-config / --no-user-config)
  //
  // These exercise the NO-ARG MakeNewDocumentState() — the call production actually makes from
  // main.cpp / DoNew() / DoOpen() — against each of the three UserConfigSource states. The tests
  // above all pass an explicit override_dir, which deliberately bypasses the switch entirely.
  // ================================================================================
  {
    // --no-user-config and --user-config <empty dir> must be the same document. Both are "the
    // user has saved nothing", reached by two different routes, and a user who cannot get the
    // same result twice has no way to establish a factory baseline.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "switch_disabled_equals_empty_explicit_dir");
    t->TestFunc = [](ImGuiTestContext*) {
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
      IM_CHECK(SerializesIdentically(disabled, empty_explicit));
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);

      // ...and the comparison above is not vacuous: the SAME directory holding a real override
      // produces a DIFFERENT document through the same code path. Without this arm, a broken
      // switch that ignored explicit_dir would satisfy the equality assertion perfectly.
      json doc;
      doc["renderer"]["lens_type"] = "fisheye_equal_area";
      doc["bg_alpha"] = 0.25f;
      IM_CHECK(gui::WriteUserDefaultsFile(empty_dir, doc));
      gui::GuiState overridden;
      {
        ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, empty_dir);
        overridden = gui::MakeNewDocumentState();
      }
      IM_CHECK(!SerializesIdentically(disabled, overridden));
      IM_CHECK_EQ(overridden.renderer.lens_type, gui::kLensTypeFisheyeEqualArea);
      IM_CHECK_EQ(overridden.bg_alpha, 0.25f);

      // And with the switch back to disabled, that same on-disk file is invisible again.
      gui::GuiState disabled_again;
      {
        ScopedUserConfigSource guard(gui::UserConfigSource::kDisabled);
        disabled_again = gui::MakeNewDocumentState();
      }
      IM_CHECK(SerializesIdentically(disabled, disabled_again));
    };
  }

  {
    // --user-config pointing at a directory whose file is corrupt: the switch must route into
    // the existing degradation path, not around it.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "switch_explicit_dir_with_broken_file_degrades");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("switch_broken");
      WriteRawOverlay(dir, "{ this is not json");

      gui::GuiState state;
      {
        ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
        state = gui::MakeNewDocumentState();
      }
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
      const gui::GuiState factory = gui::InitDefaultState();
      IM_CHECK_EQ(state.renderer.lens_type, factory.renderer.lens_type);
      IM_CHECK_EQ(state.bg_alpha, factory.bg_alpha);
    };
  }

  {
    // --user-config <path to a regular file> — the flag takes a DIRECTORY, and a user who points
    // it at the user_defaults.json itself must get a degraded startup, not a crash.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "switch_explicit_dir_pointing_at_a_file_is_survivable");
    t->TestFunc = [](ImGuiTestContext*) {
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
      IM_CHECK_EQ(state.renderer.lens_type, factory.renderer.lens_type);
      IM_CHECK_EQ(state.crystals.size(), factory.crystals.size());
    };
  }

  {
    // The harness's own baseline, asserted from inside the harness. gui_test's main() installs
    // kTestHarnessUserConfigDefault before any test runs, and every scenario reaches
    // MakeNewDocumentState() through ResetTestState() -> DoNew(); if that install were dropped or
    // flipped, this is the case that says so. It reads the switch through its only observable
    // effect — whether a real override file on disk reaches a new document — because the source
    // enum itself has no getter, by design (one owner: main()).
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "switch_gui_test_baseline_is_isolated");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("switch_baseline");
      json doc;
      doc["bg_alpha"] = 0.125f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      // No ScopedUserConfigSource here on purpose: this is the process state gui_test's main()
      // left behind, i.e. what every other scenario in this binary runs under.
      const gui::GuiState state = gui::MakeNewDocumentState();
      const gui::GuiState factory = gui::InitDefaultState();
      IM_CHECK_EQ(state.bg_alpha, factory.bg_alpha);

      // Detection power for the assertion above: the file IS readable and DOES carry a value
      // distinguishable from factory — it simply must not be consulted at the baseline.
      {
        ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
        const gui::GuiState reachable = gui::MakeNewDocumentState();
        IM_CHECK_EQ(reachable.bg_alpha, 0.125f);
        IM_CHECK(reachable.bg_alpha != factory.bg_alpha);
      }
    };
  }

  // ================================================================================
  // AC4 — invariant I1
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac4_file_value_beats_personal_default");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("ac4");

      // Personal default: X.
      json doc;
      doc["renderer"]["lens_type"] = "fisheye_equal_area";
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      // Sanity: the personal default really does reach a NEW document (otherwise the negative
      // assertions below would pass vacuously).
      const gui::GuiState fresh = gui::MakeNewDocumentState(dir);
      IM_CHECK_EQ(fresh.renderer.lens_type, gui::kLensTypeFisheyeEqualArea);

      // Forward: a file that states Y explicitly loads as Y, never as the personal default X.
      // This is invariant I1 — otherwise a .lmc someone sends you renders differently on your
      // machine, silently.
      json file_doc = json::parse(gui::SerializeGuiStateJson(gui::InitDefaultState()));
      file_doc["renderer"]["lens_type"] = "linear";
      gui::GuiState loaded;
      IM_CHECK(gui::DeserializeGuiStateJson(file_doc.dump(), loaded));
      IM_CHECK_EQ(loaded.renderer.lens_type, gui::kLensTypeLinear);

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
      IM_CHECK(gui::DeserializeGuiStateJson(old_doc.dump(), old_loaded));
      IM_CHECK_EQ(old_loaded.renderer.lens_type, gui::RenderConfig{}.lens_type);
      IM_CHECK(old_loaded.renderer.lens_type != gui::kLensTypeFisheyeEqualArea);
    };
  }

  // A real-GUI manual walkthrough ("set a user_defaults.json, start the GUI, observe New picking
  // up the default and .lmc Open unaffected") is a one-time human check that leaves no trace and
  // has to be redone by hand every time this file changes. This test programmatically covers the
  // same assertion pair instead — the two halves already exist separately in
  // ac3_sparse_overlay_round_trip and ac4_file_value_beats_personal_default; this is the pair in
  // one place, self-contained.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults",
                                    "m2_new_document_picks_up_personal_default_and_lmc_open_is_unaffected");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("m2_walkthrough");

      json doc;
      doc["bg_alpha"] = 0.33f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      // "New document picks up the personal default."
      const gui::GuiState new_doc = gui::MakeNewDocumentState(dir);
      IM_CHECK_EQ(new_doc.bg_alpha, 0.33f);

      // "Opening an .lmc is unaffected" — an explicit file that does not mention bg_alpha must
      // load the FACTORY value, never the personal default that New just picked up in this same
      // process. DeserializeGuiStateJson is the .lmc/.json load path; it never consults
      // MakeNewDocumentState() or the personal-default store.
      json lmc_doc = json::parse(gui::SerializeGuiStateJson(gui::InitDefaultState()));
      lmc_doc.erase("bg_alpha");
      gui::GuiState opened;
      IM_CHECK(gui::DeserializeGuiStateJson(lmc_doc.dump(), opened));
      IM_CHECK_EQ(opened.bg_alpha, gui::GuiState{}.bg_alpha);
      IM_CHECK(opened.bg_alpha != 0.33f);
    };
  }

  // ================================================================================
  // AC5 — degradation, never a crash
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac5_unknown_key_is_ignored_silently");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("ac5_unknown");

      json doc;
      doc["totally_unknown_key"] = 1;
      doc["bg_alpha"] = 0.75f;  // a known key alongside it must still apply
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      IM_CHECK_EQ(state.bg_alpha, 0.75f);
      // Forward compatibility: a key this build does not know about is the normal shape of a
      // file written by a NEWER build. Ignoring it is correct and is not a degradation.
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac5_wrong_type_degrades_without_partial_apply");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("ac5_type");

      // `bg_alpha` is a float; a string there makes the deserializer throw mid-merge.
      WriteRawOverlay(dir, R"({"bg_alpha": "not_a_number", "aspect_portrait": true})");

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      const gui::GuiState factory{};
      // The WHOLE overlay is discarded, including the well-formed sibling key: a half-applied
      // set of defaults is harder to reason about (and to revert) than none at all.
      IM_CHECK_EQ(state.bg_alpha, factory.bg_alpha);
      IM_CHECK_EQ(state.aspect_portrait, factory.aspect_portrait);
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
      // Still a usable new document.
      IM_CHECK_EQ(state.layers.size(), static_cast<size_t>(1));
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "ac5_invalid_json_degrades");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("ac5_json");

      WriteRawOverlay(dir, R"({"bg_alpha": 0.5, )");  // truncated

      const gui::GuiState state = gui::MakeNewDocumentState(dir);
      const gui::GuiState factory{};
      IM_CHECK_EQ(state.bg_alpha, factory.bg_alpha);
      IM_CHECK_EQ(state.layers.size(), static_cast<size_t>(1));
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);

      // A JSON scalar / array at the root is also not a usable document.
      ResetUserDefaultsChannels();
      WriteRawOverlay(dir, "[1, 2, 3]");
      const gui::GuiState state2 = gui::MakeNewDocumentState(dir);
      IM_CHECK_EQ(state2.bg_alpha, factory.bg_alpha);
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
    };
  }

  // ================================================================================
  // D8 — load-time clamp into the classifier's tolerance domain, with a visible trace
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "d8_preset_override_clamped_with_notice");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("d8_upper");

      // Column's domain is (0, 10); 25 is above it (hand-edited, or from a build whose
      // classifier was wider).
      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 25.0f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::MakeNewDocumentState(dir);

      const auto column = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(column.has_value());
      // Clamped to the largest float STRICTLY below the bound: landing on 10.0 exactly would
      // fail the classifier's strict `< 10` and demote the preset to Custom.
      IM_CHECK_EQ(*column, std::nextafter(gui::kColumnPlateParryZenithStdUpperBound, 0.0f));
      IM_CHECK(*column < gui::kColumnPlateParryZenithStdUpperBound);

      // The clamp must leave a trace naming the key and both values — at load time the user is
      // not looking at the preset panel, so a silent clamp is indistinguishable from a silent
      // drop (the exact failure family this system exists to avoid).
      const auto notices = gui::TakeUserDefaultsDowngradeNotices();
      IM_CHECK_EQ(notices.size(), static_cast<size_t>(1));
      IM_CHECK(notices[0].find("column") != std::string::npos);
      IM_CHECK(notices[0].find("25") != std::string::npos);
      // Consumed-on-read, like the shape-dist downgrade counter it mirrors.
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(0));
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "d8_lowitz_clamps_on_the_lower_side");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("d8_lower");

      // Lowitz's domain is (15, inf) — the opposite side from Column/Plate/Parry.
      json doc;
      doc["presets"]["axis"]["lowitz"]["zenith_std"] = 5.0f;
      // Non-positive is outside Column's (0, 10) too — the domain is open at BOTH ends there,
      // and a zero/negative std would otherwise reach the sampler.
      doc["presets"]["axis"]["plate"]["zenith_std"] = -3.0f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::MakeNewDocumentState(dir);

      const auto lowitz = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kLowitz);
      IM_CHECK(lowitz.has_value());
      IM_CHECK_EQ(*lowitz, std::nextafter(gui::kLowitzZenithStdLowerBound, std::numeric_limits<float>::infinity()));
      IM_CHECK(*lowitz > gui::kLowitzZenithStdLowerBound);

      const auto plate = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kPlate);
      IM_CHECK(plate.has_value());
      IM_CHECK(*plate > 0.0f);
      IM_CHECK(*plate < gui::kColumnPlateParryZenithStdUpperBound);

      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(2));
    };
  }

  // code-review round 2 Major: ApplyAxisPresetOverridesFromJson only ever added/updated slots,
  // never cleared one whose entry disappeared from the file. MakeNewDocumentState() is called
  // repeatedly within one process (main.cpp startup, every DoNew(), every DoOpen() .json
  // import), so a stale g_axis_overrides slot would keep answering with a deleted override
  // until the process restarts. This test calls MakeNewDocumentState() twice against the same
  // directory within a single test case (deliberately not calling ResetUserDefaultsChannels()
  // between the two calls) to reproduce the same-process, file-changed-underneath scenario.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "d8_preset_override_does_not_leak_across_calls");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("d8_leak");

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 3.0f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::MakeNewDocumentState(dir);
      const auto first = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(first.has_value());
      IM_CHECK_EQ(*first, 3.0f);

      // The user edits the file underneath the running process and removes the override —
      // no call to ResetUserAxisPresetOverrides() here, matching production (only
      // ApplyAxisPresetOverridesFromJson's internal reset should apply).
      json empty_doc = json::object();
      IM_CHECK(gui::WriteUserDefaultsFile(dir, empty_doc));
      gui::MakeNewDocumentState(dir);
      const auto second = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(!second.has_value());
    };
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
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults",
                                    "d8_preset_override_does_not_leak_when_config_dir_becomes_unavailable");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("d8_leak_no_dir");

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 3.0f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));
      gui::MakeNewDocumentState(dir);
      const auto first = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(first.has_value());
      IM_CHECK_EQ(*first, 3.0f);

      // Same process, same override still loaded — but this call's GetUserConfigDir() now
      // resolves to nullopt, the no-arg production call path, not an explicit override_dir.
      //
      // The kAutoDetect guard is what keeps this test testing what it says. gui_test's baseline
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
      IM_CHECK(!second.has_value());
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "d8_malformed_preset_value_degrades");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("d8_bad");

      WriteRawOverlay(dir, R"({"presets": {"axis": {"column": {"zenith_std": "wide"}}}})");
      gui::MakeNewDocumentState(dir);

      IM_CHECK(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 1);
      // A rejected value is a drop, not a clamp — the two channels must not be conflated.
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeNotices().size(), static_cast<size_t>(0));
    };
  }

  // ================================================================================
  // Preset library — the WRITE side (405.5). The read side above could resolve an override;
  // these cover the functions that produce one, and the property the whole design exists for:
  // a retuned preset is still classified as that preset.
  // ================================================================================

  {
    // AC1 — the round trip a user actually performs: retune Column to the value the beta user
    // asked for, then confirm a fresh process would read it back. MakeNewDocumentState() is the
    // "restart" here: it is the exact call startup makes, and it re-reads the file from disk.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "preset_write_round_trips_across_a_reload");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("preset_roundtrip");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      const auto result = gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kColumn, 0.3f);
      IM_CHECK(result.written);
      IM_CHECK(!result.clamped);
      IM_CHECK_EQ(result.stored_value, 0.3f);
      IM_CHECK(result.message.empty());  // a clean write says nothing — the warning cell stays empty

      // On disk under the key 405.2's reader already looks for, not merely in memory.
      const json doc = ReadOverlayDoc(dir);
      IM_CHECK(doc.contains("presets"));
      IM_CHECK_EQ(doc["presets"]["axis"]["column"]["zenith_std"].get<float>(), 0.3f);

      // Drop the in-memory state the way a restart would, then re-resolve from the file.
      gui::ResetUserAxisPresetOverrides();
      IM_CHECK(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
      gui::MakeNewDocumentState(dir);

      const auto reloaded = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(reloaded.has_value());
      IM_CHECK_EQ(*reloaded, 0.3f);

      // And that is what pressing the Column button would write into the crystal.
      IM_CHECK_EQ(gui::EffectiveAxisPresetZenith(gui::AxisPresetEntryFor(gui::AxisPreset::kColumn)).std, 0.3f);
    };
  }

  {
    // AC2 — the core claim of D8, MEASURED rather than reasoned about: a retuned preset is still
    // the same preset to ClassifyAxisPreset, which is what keeps its button highlighted and its
    // preview on the typical-view branch. Four presets, each at a value inside its own domain
    // (the domains point in opposite directions, so one example would prove nothing about the
    // other side).
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "preset_identity_survives_retuning");
    t->TestFunc = [](ImGuiTestContext*) {
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
        const auto result = gui::SaveAxisPresetZenithStdOverride(c.preset, c.tuned_std);
        IM_CHECK(result.written);
        IM_CHECK(!result.clamped);

        // Exactly what the modal's preset button assembles into its edit buffer.
        const gui::AxisDist zenith = gui::EffectiveAxisPresetZenith(entry);
        IM_CHECK_EQ(zenith.std, c.tuned_std);
        IM_CHECK_EQ(static_cast<int>(gui::ClassifyAxisPreset(zenith, entry.azimuth, entry.roll)),
                    static_cast<int>(c.preset));

        // The preview's typical-view branch is selected by the classified preset, so it follows
        // from the line above; asserted anyway because "preview goes somewhere else" is the
        // symptom a user would report, and it should fail HERE rather than as a screenshot.
        float rotation[16] = {};
        float typical[16] = {};
        gui::DefaultPreviewRotation(gui::ClassifyAxisPreset(zenith, entry.azimuth, entry.roll), nullptr, rotation);
        gui::DefaultPreviewRotation(c.preset, nullptr, typical);
        for (int i = 0; i < 16; ++i) {
          IM_CHECK_EQ(rotation[i], typical[i]);
        }
      }
    };
  }

  {
    // AC3 (store half) — an out-of-domain edit is clamped AND says so, in both directions, and an
    // in-domain edit is left exactly as typed. Both states asserted: a clamp that fired on every
    // write would satisfy a one-sided test while making the feature useless.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "preset_write_clamps_and_reports_both_states");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("preset_write_clamp");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      // Above Column's (0, 10).
      const auto high = gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kColumn, 25.0f);
      IM_CHECK(high.written);
      IM_CHECK(high.clamped);
      IM_CHECK(high.stored_value < gui::kColumnPlateParryZenithStdUpperBound);
      IM_CHECK(!high.message.empty());
      IM_CHECK(high.message.find("Column") != std::string::npos);
      IM_CHECK(high.message.find("25") != std::string::npos);
      // The copy must not claim the bound is physical — it is where the neighbouring criterion
      // starts. Plate's [10, 15) dead zone is the case that makes the distinction matter.
      IM_CHECK(high.message.find("not a physical limit") != std::string::npos);
      // Still Column afterwards, which is the point of clamping rather than rejecting.
      const auto& column = gui::AxisPresetEntryFor(gui::AxisPreset::kColumn);
      IM_CHECK_EQ(static_cast<int>(
                      gui::ClassifyAxisPreset(gui::EffectiveAxisPresetZenith(column), column.azimuth, column.roll)),
                  static_cast<int>(gui::AxisPreset::kColumn));

      // Below Lowitz's (15, inf) — the opposite side.
      const auto low = gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kLowitz, 5.0f);
      IM_CHECK(low.written);
      IM_CHECK(low.clamped);
      IM_CHECK(low.stored_value > gui::kLowitzZenithStdLowerBound);

      // In domain: stored verbatim, nothing reported.
      const auto ok = gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kColumn, 0.3f);
      IM_CHECK(ok.written);
      IM_CHECK(!ok.clamped);
      IM_CHECK_EQ(ok.stored_value, 0.3f);
      IM_CHECK(ok.message.empty());
    };
  }

  {
    // AC5 — restore one preset to factory: byte-identical to kAxisPresets, key gone from the file
    // (with its empty parents pruned), and NOTHING else disturbed — neither another preset nor the
    // GuiState half of the same file. The last part is what a wholesale rewrite would break.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "preset_restore_to_factory_is_surgical");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("preset_restore");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      // A file that already carries a GuiState default, so the write path has a neighbour to
      // preserve rather than an empty document to overwrite.
      json seed;
      seed["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, seed));

      IM_CHECK(gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kColumn, 0.3f).written);
      IM_CHECK(gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kPlate, 0.5f).written);

      IM_CHECK(gui::RevertOneAxisPresetOverride(gui::AxisPreset::kColumn));

      // Factory again, field by field against the table itself.
      const auto& column = gui::AxisPresetEntryFor(gui::AxisPreset::kColumn);
      IM_CHECK(!gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn).has_value());
      const gui::AxisDist restored = gui::EffectiveAxisPresetZenith(column);
      IM_CHECK_EQ(static_cast<int>(restored.type), static_cast<int>(column.zenith.type));
      IM_CHECK_EQ(restored.mean, column.zenith.mean);
      IM_CHECK_EQ(restored.std, column.zenith.std);

      const json doc = ReadOverlayDoc(dir);
      IM_CHECK(!doc["presets"]["axis"].contains("column"));
      // Untouched: the other preset and the GuiState key sharing the file.
      IM_CHECK_EQ(doc["presets"]["axis"]["plate"]["zenith_std"].get<float>(), 0.5f);
      IM_CHECK_EQ(doc["bg_alpha"].get<float>(), 0.42f);

      // Reverting the last preset prunes the now-empty parents, so a hand-opened file does not
      // accumulate `"presets": {"axis": {}}` skeletons.
      IM_CHECK(gui::RevertOneAxisPresetOverride(gui::AxisPreset::kPlate));
      const json pruned = ReadOverlayDoc(dir);
      IM_CHECK(!pruned.contains("presets"));
      IM_CHECK_EQ(pruned["bg_alpha"].get<float>(), 0.42f);
    };
  }

  {
    // AC7 — Random has no adjustable face, so nothing about it may reach the file. Asserted at the
    // STORE, not only at the UI: the UI not drawing an input is the first defense, and a defense
    // that exists only in a widget is one a future refactor removes without noticing.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "preset_without_adjustable_face_is_never_written");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("preset_random");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      for (const auto preset : { gui::AxisPreset::kRandom, gui::AxisPreset::kCustom }) {
        const auto result = gui::SaveAxisPresetZenithStdOverride(preset, 3.0f);
        IM_CHECK(!result.written);
        IM_CHECK(!result.message.empty());  // refused out loud, not silently dropped
        IM_CHECK(!gui::GetUserAxisPresetZenithStdOverride(preset).has_value());
        IM_CHECK(!gui::RevertOneAxisPresetOverride(preset));
      }

      // Nothing was created at all — not an empty `presets` skeleton either.
      const json doc = ReadOverlayDoc(dir);
      IM_CHECK(!doc.contains("presets"));

      // And the table itself agrees about which presets those are, so this test cannot pass by
      // testing a set that has quietly stopped matching the one the UI renders.
      int adjustable = 0;
      for (const auto& entry : gui::kAxisPresets) {
        if (entry.has_adjustable_zenith_std) {
          ++adjustable;
          IM_CHECK(entry.override_json_name != nullptr);
        } else {
          IM_CHECK(entry.override_json_name == nullptr);
        }
      }
      IM_CHECK_EQ(adjustable, 4);  // Column / Plate / Parry / Lowitz
    };
  }

  {
    // A failed write must leave the in-memory value alone. Otherwise this session resolves one
    // value while the next launch reads the older one off disk, and the user sees their setting
    // "come back" with no event to attribute it to.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "preset_write_failure_leaves_memory_untouched");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("preset_write_fail");
      {
        ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
        IM_CHECK(gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kColumn, 0.3f).written);
      }

      // No writable directory at all — the same shape as a read-only config dir.
      ScopedUserConfigSource disabled(gui::UserConfigSource::kDisabled);
      const auto result = gui::SaveAxisPresetZenithStdOverride(gui::AxisPreset::kColumn, 7.0f);
      IM_CHECK(!result.written);
      const auto still = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(still.has_value());
      IM_CHECK_EQ(*still, 0.3f);  // NOT 7.0 — the failed write changed nothing

      IM_CHECK(!gui::RevertOneAxisPresetOverride(gui::AxisPreset::kColumn));
      const auto after_failed_revert = gui::GetUserAxisPresetZenithStdOverride(gui::AxisPreset::kColumn);
      IM_CHECK(after_failed_revert.has_value());
      IM_CHECK_EQ(*after_failed_revert, 0.3f);  // a failed revert must not report success in memory
    };
  }

  {
    // AC4 — a load-time clamp must REACH the user, not merely be counted. 405.2 built the
    // counters; until this task nothing consumed them, so a value silently adjusted at startup
    // was indistinguishable from one that had been dropped. Asserted through the same one-shot
    // warning surface an import degradation uses, from the production DoNew() path.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "load_time_clamp_reaches_the_user");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      gui::ClearImportComplexFilterWarning();
      const auto dir = FreshOverlayDir("clamp_surfaced");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 25.0f;  // above Column's (0, 10)
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      gui::DoNew();

      const std::string warning = gui::PeekImportComplexFilterWarning();
      IM_CHECK(!warning.empty());
      IM_CHECK(warning.find("Column") != std::string::npos);
      IM_CHECK(warning.find("25") != std::string::npos);
      // The clamped value itself, so the user learns what they actually got rather than only that
      // something was wrong.
      IM_CHECK(warning.find("9.99999") != std::string::npos);
      gui::ClearImportComplexFilterWarning();
    };
  }

  {
    // The negative half of AC4: a clean override file must produce NO popup. Without this, a
    // notice that fired on every New would satisfy the case above while training the user to
    // dismiss the dialog unread — which is the same as not having one.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "a_clean_load_surfaces_nothing");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      gui::ClearImportComplexFilterWarning();
      const auto dir = FreshOverlayDir("clamp_quiet");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 0.3f;  // inside the domain
      doc["bg_alpha"] = 0.42f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      gui::DoNew();
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());

      // A first run with no file at all is the most common case of all, and must be just as quiet.
      const auto empty_dir = FreshOverlayDir("clamp_quiet_empty");
      ScopedUserConfigSource empty_guard(gui::UserConfigSource::kExplicitDir, empty_dir);
      gui::DoNew();
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
    };
  }

  {
    // The JSON-import path reads the override file too, but DeserializeFromJson opens with
    // `state = GuiState{}` — so nothing of the personal defaults survives it. Reporting a
    // degradation there would describe a document the user is not getting; leaving the counters
    // filled would misattribute them to the NEXT New. Draining is the only correct handling, and
    // this pins it: the import must leave the channel empty.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "json_import_does_not_leak_downgrades_into_the_next_new");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      gui::ClearImportComplexFilterWarning();
      const auto dir = FreshOverlayDir("import_leak");
      ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);

      json doc;
      doc["presets"]["axis"]["column"]["zenith_std"] = 25.0f;
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      // A minimal but valid CLI config, so DoOpen takes the import branch to completion.
      const std::filesystem::path config = dir / "imported.json";
      {
        std::ofstream out(config, std::ios::trunc);
        out << R"({"sun": {"altitude": 25.0}})";
      }
      gui::DoOpen(config);
      // Whatever the import path read is gone, reported or not.
      IM_CHECK_EQ(gui::TakeUserDefaultsDowngradeCount(), 0);
      IM_CHECK(gui::TakeUserDefaultsDowngradeNotices().empty());

      // The decisive part: the NEXT New reports its own load, and the file is clean by then.
      IM_CHECK(gui::RevertOneAxisPresetOverride(gui::AxisPreset::kColumn));
      gui::ClearImportComplexFilterWarning();
      gui::DoNew();
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
    };
  }

  // ================================================================================
  // bg_path as a personal default
  // ================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "bg_path_default_degrades_when_missing");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetUserDefaultsChannels();
      const auto dir = FreshOverlayDir("bg");

      json doc;
      doc["bg_path"] = (dir / "no_such_background.png").string();
      doc["bg_show"] = true;
      doc["aspect_ratio"] = "match_background";
      IM_CHECK(gui::WriteUserDefaultsFile(dir, doc));

      gui::GuiState state = gui::MakeNewDocumentState(dir);
      IM_CHECK(!state.bg_path.empty());  // the default did land in the state...
      IM_CHECK_EQ(state.bg_show, true);  // ...and asked for the background to be shown
      IM_CHECK_EQ(static_cast<int>(state.aspect_preset), static_cast<int>(gui::AspectPreset::kMatchBg));

      // ...but the file is gone. The New path must degrade rather than throw or hang: this runs
      // during startup, where a personal default pointing at a since-deleted image would
      // otherwise be able to stop the app from opening.
      gui::LoadBackgroundWithDegrade(state);
      IM_CHECK_EQ(state.bg_show, false);
      IM_CHECK_EQ(static_cast<int>(state.aspect_preset), static_cast<int>(gui::AspectPreset::kFree));

      // An empty bg_path is a plain no-op — the overwhelmingly common case must not be made to
      // pay for the degrade path (and must not clobber an unrelated aspect preset).
      gui::GuiState empty{};
      empty.bg_show = true;
      empty.aspect_preset = gui::AspectPreset::kMatchBg;
      gui::LoadBackgroundWithDegrade(empty);
      IM_CHECK_EQ(empty.bg_show, true);
      IM_CHECK_EQ(static_cast<int>(empty.aspect_preset), static_cast<int>(gui::AspectPreset::kMatchBg));
    };
  }

  {
    // Structural gate: the degrade step must be wired into every path that can land a non-empty
    // bg_path. Calling the helper directly (above) proves it works; this proves it is reached.
    // Same technique as the reset-primitive single-owner gate in test_gui_import_export.cpp.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "user_defaults", "bg_degrade_is_wired_into_every_document_path");
    t->TestFunc = [](ImGuiTestContext*) {
      std::ifstream in(LUMICE_GUI_APP_CPP_PATH);
      IM_CHECK(in.is_open());
      const std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      IM_CHECK(!src.empty());

      int calls = 0;
      size_t pos = 0;
      const std::string needle = "LoadBackgroundWithDegrade(g_state);";
      while ((pos = src.find(needle, pos)) != std::string::npos) {
        ++calls;
        pos += needle.size();
      }
      // Three document-entry paths: DoOpen(.lmc), DoOpen(.json import), DoNew.
      IM_CHECK_EQ(calls, 3);

      // And the old inline copy must be gone — LoadAndUploadBgImage should now only be reached
      // from the shared helper and from the explicit "Load Background..." command.
      int raw_loads = 0;
      pos = 0;
      const std::string raw = "LoadAndUploadBgImage(";
      while ((pos = src.find(raw, pos)) != std::string::npos) {
        ++raw_loads;
        pos += raw.size();
      }
      IM_CHECK_EQ(raw_loads, 3);  // 1 definition + LoadBackgroundWithDegrade + DoLoadBackground
    };
  }
}
