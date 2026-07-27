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
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "gui/gui_state_tiers.hpp"
#include "gui/user_defaults.hpp"
#include "test_gui_shared.hpp"

namespace {

using nlohmann::json;

// Every test writes its override file into a fresh directory under the system temp dir, so no
// test ever reads (or writes) the developer's real OS config directory. That isolation is
// hand-rolled here on purpose: the --user-config / --no-user-config switch that will make it a
// process-wide property is a separate task, and until it lands the only safe injection point is
// MakeNewDocumentState's override_dir parameter.
std::filesystem::path FreshOverlayDir(const char* tag) {
  std::filesystem::path dir = std::filesystem::temp_directory_path() / (std::string("lumice_user_defaults_") + tag);
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  return dir;
}

// Reset every consumable channel so a test starts from a known state regardless of what ran
// before it in this single-process test binary.
void ResetUserDefaultsChannels() {
  gui::ResetUserAxisPresetOverrides();
  gui::TakeUserDefaultsDowngradeCount();
  gui::TakeUserDefaultsClampNotices();
}

void WriteRawOverlay(const std::filesystem::path& dir, std::string_view text) {
  std::ofstream out(dir / gui::kUserDefaultsFileName, std::ios::trunc);
  out << text;
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
      IM_CHECK_EQ(gui::TakeUserDefaultsClampNotices().size(), static_cast<size_t>(0));
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
      const auto notices = gui::TakeUserDefaultsClampNotices();
      IM_CHECK_EQ(notices.size(), static_cast<size_t>(1));
      IM_CHECK(notices[0].find("column") != std::string::npos);
      IM_CHECK(notices[0].find("25") != std::string::npos);
      // Consumed-on-read, like the shape-dist downgrade counter it mirrors.
      IM_CHECK_EQ(gui::TakeUserDefaultsClampNotices().size(), static_cast<size_t>(0));
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

      IM_CHECK_EQ(gui::TakeUserDefaultsClampNotices().size(), static_cast<size_t>(2));
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
      IM_CHECK_EQ(gui::TakeUserDefaultsClampNotices().size(), static_cast<size_t>(0));
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
