#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>

#include "config/config_manager.hpp"
#include "config/filter_config.hpp"
#include "test_gui_shared.hpp"

// ========== Import/Export Tests ==========

void RegisterImportExportTests(ImGuiTestEngine* engine) {
  // Test 1: Core JSON path invariants — SerializeCoreConfig → DeserializeFromJson.
  // NOTE: This is NOT a GUI-state round-trip. SerializeCoreConfig always emits a
  // fixed full-sphere renderer (lens="dual_fisheye_equal_area", fov=180, visible="full",
  // background=[0,0,0]) regardless of GUI state — Core only consumes this shape.
  // GUI-state round-trip is covered by renderer_new_format_roundtrip / lmc_full_roundtrip.
  // exposure_offset / opacity are not asserted: exposure_offset's 2^x → log2 round-trip
  // introduces floating-point precision drift.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "json_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Set up non-default state
      gui::g_state.sim.ray_num_millions = 2.5f;

      // Serialize → deserialize
      std::string json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!json.empty());

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(json, loaded);
      IM_CHECK(ok);

      // State-derived fields survive the round-trip.
      IM_CHECK(std::abs(loaded.sim.ray_num_millions - gui::g_state.sim.ray_num_millions) < 0.01f);
      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), static_cast<int>(gui::g_state.layers.size()));

      // Renderer fields that Core hardcodes: lens is always dual_fisheye_equal_area
      // (kLensTypeNames[4] == "Dual Fisheye Equal Area"; keep this index in sync
      // if kLensTypeNames is reordered) and fov is always 180. These assertions lock
      // the Core-path contract itself.
      IM_CHECK_EQ(loaded.renderer.lens_type, 4);
      IM_CHECK_EQ(loaded.renderer.fov, 180.0f);
    };
  }

  // Test 2: JSON file round-trip — write to file, read back, deserialize, verify
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "json_file_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      gui::g_state.sim.ray_num_millions = 3.0f;
      std::string json = gui::SerializeCoreConfig(gui::g_state);

      const char* tmp_path = "/tmp/lumice_config_test.json";
      bool write_ok = gui::ExportConfigJson(tmp_path, json);
      IM_CHECK(write_ok);

      // Read back
      std::ifstream in(tmp_path);
      IM_CHECK(in.is_open());
      std::string read_json((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      in.close();

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(read_json, loaded);
      IM_CHECK(ok);
      IM_CHECK(std::abs(loaded.sim.ray_num_millions - 3.0f) < 0.01f);

      std::remove(tmp_path);
    };
  }

  // Test 3: Old format backward compat — ID-referenced crystals/scattering → layers/entries
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "old_format_compat");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Pre-copy-model .lmc GUI-state format (verified via `git show ade8fc2^`):
      // lowercase "type", nested "shape" block, scattering entries reference crystals
      // by crystal_id. DeserializeGuiStateJson's legacy branch (the
      // root.contains("crystals") && root.contains("scattering") path) reads exactly
      // this shape. Optional fields (wedge angles, indices, axis) are omitted because
      // this test only asserts crystal type, the primary shape dimensions (height for
      // prism; prism_h/upper_h/lower_h for pyramid), proportion, filter absence and
      // sun altitude.
      std::string json = R"({
        "crystals": [
          {"id": 1, "type": "prism", "shape": {"height": 2.0}},
          {"id": 2, "type": "pyramid",
           "shape": {"prism_h": 1.0, "upper_h": 0.3, "lower_h": 0.4}}
        ],
        "scattering": [
          {"prob": 0.8, "entries": [
            {"crystal_id": 1, "proportion": 60.0},
            {"crystal_id": 2, "proportion": 40.0, "filter_id": -1}
          ]}
        ],
        "sun": {"altitude": 25.0}
      })";

      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);

      // Verify migration to copy model
      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(loaded.layers[0].entries.size()), 2);
      IM_CHECK_EQ(loaded.layers[0].entries[0].crystal.type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(loaded.layers[0].entries[0].crystal.height, 2.0f);
      IM_CHECK_EQ(loaded.layers[0].entries[0].proportion, 60.0f);
      IM_CHECK_EQ(loaded.layers[0].entries[1].crystal.type, gui::CrystalType::kPyramid);
      IM_CHECK(!loaded.layers[0].entries[1].filter.has_value());
      IM_CHECK_EQ(loaded.sun.altitude, 25.0f);
    };
  }

  // Test 4: Full .lmc roundtrip with all EntryCard fields
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "lmc_full_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Set up a state with multiple entries, filter, and pyramid crystal
      auto& entry0 = gui::g_state.layers[0].entries[0];
      entry0.crystal.type = gui::CrystalType::kPyramid;
      entry0.crystal.prism_h = 2.0f;
      entry0.crystal.upper_h = 0.3f;
      entry0.crystal.lower_h = 0.4f;
      // Non-default face_distance to exercise the conditional-write branch in SerializeCrystal.
      entry0.crystal.face_distance[0] = 1.2f;
      entry0.crystal.face_distance[3] = 0.9f;
      // Non-default axis distributions to verify axis round-trip on all three axes.
      // All three use the same SerializeAxisDist/FillAxisDist path, but covering each
      // individually guards against future per-axis special-casing (e.g. roll default-omission).
      entry0.crystal.zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 25.0f, 3.0f };
      entry0.crystal.azimuth = gui::AxisDist{ gui::AxisDistType::kUniform, 10.0f, 20.0f };
      entry0.crystal.roll = gui::AxisDist{ gui::AxisDistType::kLaplacian, 5.0f, 2.0f };
      entry0.proportion = 75.0f;
      gui::FilterConfig f;
      f.param = gui::RaypathParams{ "3-1-5" };
      entry0.filter = f;

      // Add a second entry
      gui::EntryCard extra;
      extra.crystal.type = gui::CrystalType::kPrism;
      extra.crystal.height = 3.0f;
      extra.proportion = 25.0f;
      gui::g_state.layers[0].entries.push_back(extra);

      // Save
      const char* tmp_path = "/tmp/lumice_full_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset and load
      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify all fields survived round-trip
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      auto& loaded0 = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(loaded0.crystal.type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(loaded0.crystal.prism_h, 2.0f);
      IM_CHECK_EQ(loaded0.crystal.upper_h, 0.3f);
      IM_CHECK_EQ(loaded0.crystal.lower_h, 0.4f);
      IM_CHECK_EQ(loaded0.crystal.face_distance[0], 1.2f);
      IM_CHECK_EQ(loaded0.crystal.face_distance[3], 0.9f);
      IM_CHECK_EQ(loaded0.crystal.zenith.type, gui::AxisDistType::kGauss);
      IM_CHECK_EQ(loaded0.crystal.zenith.mean, 25.0f);
      IM_CHECK_EQ(loaded0.crystal.zenith.std, 3.0f);
      IM_CHECK_EQ(loaded0.crystal.azimuth.type, gui::AxisDistType::kUniform);
      IM_CHECK_EQ(loaded0.crystal.azimuth.mean, 10.0f);
      IM_CHECK_EQ(loaded0.crystal.azimuth.std, 20.0f);
      IM_CHECK_EQ(loaded0.crystal.roll.type, gui::AxisDistType::kLaplacian);
      IM_CHECK_EQ(loaded0.crystal.roll.mean, 5.0f);
      IM_CHECK_EQ(loaded0.crystal.roll.std, 2.0f);
      IM_CHECK_EQ(loaded0.proportion, 75.0f);
      IM_CHECK(loaded0.filter.has_value());
      IM_CHECK_EQ(loaded0.filter->RaypathText(), std::string("3-1-5"));

      auto& loaded1 = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(loaded1.crystal.type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(loaded1.crystal.height, 3.0f);
      IM_CHECK_EQ(loaded1.proportion, 25.0f);

      std::remove(tmp_path);
    };
  }

  // Test: modal_layout_vertical roundtrip.
  // Ensures the new .lmc field survives save/load and defaults correctly on legacy files.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "modal_layout_vertical_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, false);  // ResetTestState pins legacy false

      // Save with horizontal (false) — distinguishable from the new production
      // default (true) so the load step below can prove it was actually read.
      const char* tmp_path = "/tmp/lumice_modal_layout_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset to the new production default via DoNew (bypasses ResetTestState pin).
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, true);  // new production default
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, false);  // saved value survives

      std::remove(tmp_path);
    };
  }

  // Test 4a: renderer copy-model new-format round-trip.
  // Fields covered (per RenderConfig in gui_state.hpp):
  //   lens_type, fov, elevation, azimuth, roll, sim_resolution_index, visible,
  //   background[3], ray_color[3], opacity, exposure_offset
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "renderer_new_format_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Populate every RenderConfig field with a non-default value.
      auto& r = gui::g_state.renderer;
      r.lens_type = 3;
      r.fov = 115.0f;
      r.elevation = 12.0f;
      r.azimuth = -24.0f;
      r.roll = 7.5f;
      r.sim_resolution_index = 2;
      r.visible = 1;
      r.background[0] = 0.1f;
      r.background[1] = 0.2f;
      r.background[2] = 0.3f;
      r.ray_color[0] = 0.8f;
      r.ray_color[1] = 0.6f;
      r.ray_color[2] = 0.4f;
      r.opacity = 0.75f;
      r.exposure_offset = 1.5f;

      std::string json = gui::SerializeGuiStateJson(gui::g_state);
      IM_CHECK(!json.empty());

      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);

      IM_CHECK_EQ(loaded.renderer.lens_type, 3);
      IM_CHECK_EQ(loaded.renderer.fov, 115.0f);
      IM_CHECK_EQ(loaded.renderer.elevation, 12.0f);
      IM_CHECK_EQ(loaded.renderer.azimuth, -24.0f);
      IM_CHECK_EQ(loaded.renderer.roll, 7.5f);
      IM_CHECK_EQ(loaded.renderer.sim_resolution_index, 2);
      IM_CHECK_EQ(loaded.renderer.visible, 1);
      IM_CHECK_EQ(loaded.renderer.background[0], 0.1f);
      IM_CHECK_EQ(loaded.renderer.background[1], 0.2f);
      IM_CHECK_EQ(loaded.renderer.background[2], 0.3f);
      IM_CHECK_EQ(loaded.renderer.ray_color[0], 0.8f);
      IM_CHECK_EQ(loaded.renderer.ray_color[1], 0.6f);
      IM_CHECK_EQ(loaded.renderer.ray_color[2], 0.4f);
      IM_CHECK_EQ(loaded.renderer.opacity, 0.75f);
      IM_CHECK_EQ(loaded.renderer.exposure_offset, 1.5f);
    };
  }

  // Test 4b: legacy .lmc renderer format (vector + selected_renderer_id + next_renderer_id)
  // must still load correctly into the new copy-model renderer field.
  // Fields covered: all RenderConfig fields (same list as Test 4a).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "renderer_legacy_format_compat");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Historical .lmc GUI-state format: renderers: [ { id, ... } ]
      // Simulates pre-task-renderer-inline output; all renderer fields non-default.
      std::string json = R"({
        "layers": [],
        "renderers": [
          {
            "id": 7,
            "lens_type": "fisheye_stereographic",
            "fov": 135.0,
            "elevation": 11.0,
            "azimuth": -21.0,
            "roll": 3.5,
            "sim_resolution": 2048,
            "visible": "lower",
            "background": [0.11, 0.22, 0.33],
            "ray_color": [0.9, 0.7, 0.5],
            "opacity": 0.8,
            "exposure_offset": -1.25
          }
        ],
        "selected_renderer_id": 7,
        "next_renderer_id": 8
      })";

      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);

      // Values must equal the first (and only) element of the legacy renderers array.
      IM_CHECK_EQ(loaded.renderer.lens_type, 3);  // index of "fisheye_stereographic"
      IM_CHECK_EQ(loaded.renderer.fov, 135.0f);
      IM_CHECK_EQ(loaded.renderer.elevation, 11.0f);
      IM_CHECK_EQ(loaded.renderer.azimuth, -21.0f);
      IM_CHECK_EQ(loaded.renderer.roll, 3.5f);
      IM_CHECK_EQ(loaded.renderer.sim_resolution_index, 2);  // 2048 → index 2 in kSimResolutions={512,1024,2048,4096}
      IM_CHECK_EQ(loaded.renderer.visible, 1);               // "lower" → 1
      IM_CHECK(std::abs(loaded.renderer.background[0] - 0.11f) < 1e-5f);
      IM_CHECK(std::abs(loaded.renderer.background[1] - 0.22f) < 1e-5f);
      IM_CHECK(std::abs(loaded.renderer.background[2] - 0.33f) < 1e-5f);
      IM_CHECK(std::abs(loaded.renderer.ray_color[0] - 0.9f) < 1e-5f);
      IM_CHECK(std::abs(loaded.renderer.ray_color[1] - 0.7f) < 1e-5f);
      IM_CHECK(std::abs(loaded.renderer.ray_color[2] - 0.5f) < 1e-5f);
      IM_CHECK(std::abs(loaded.renderer.opacity - 0.8f) < 1e-5f);
      IM_CHECK_EQ(loaded.renderer.exposure_offset, -1.25f);
    };
  }

  // Test 4c: Core JSON with empty render array — must not crash; renderer keeps defaults.
  // Covers the boundary added during task-renderer-inline DeserializeFromJson migration.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "core_json_empty_render");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({
        "crystal": [],
        "filter": [],
        "scene": {"scattering": [], "ray_num": 1000, "max_hits": 4,
                   "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}},
        "render": []
      })";

      gui::GuiState loaded;
      bool ok = gui::DeserializeFromJson(json, loaded);
      IM_CHECK(ok);
      // Default RenderConfig field values (from gui_state.hpp).
      // Warning log path exists but is not asserted automatically; observe via logs manually.
      IM_CHECK_EQ(loaded.renderer.lens_type, 0);
      IM_CHECK_EQ(loaded.renderer.fov, 90.0f);
      IM_CHECK_EQ(loaded.renderer.sim_resolution_index, 1);
      IM_CHECK_EQ(loaded.renderer.visible, 2);
      IM_CHECK_EQ(loaded.renderer.opacity, 1.0f);
    };
  }

  // Test 4d: legacy format with multiple renderers — GUI now single-renderer, take first.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "renderer_legacy_multi_takes_first");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({
        "layers": [],
        "renderers": [
          {"id": 1, "lens_type": "linear", "fov": 90.0},
          {"id": 2, "lens_type": "fisheye_equal_area", "fov": 180.0}
        ]
      })";
      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      // Must equal the first entry, not the second.
      IM_CHECK_EQ(loaded.renderer.lens_type, 0);  // "linear" → 0
      IM_CHECK_EQ(loaded.renderer.fov, 90.0f);
    };
  }

  // task-overlay-line-label-toggle: Test E — legacy `overlay_<x>` key (single
  // visibility) deserializes into both line and label = legacy_value.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "overlay_legacy_key_fallback");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({
        "layers": [],
        "renderer": {"lens_type": "linear", "fov": 90.0},
        "overlay_horizon": true,
        "overlay_grid": false,
        "overlay_sun_circles": true
      })";
      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.show_horizon_line, true);
      IM_CHECK_EQ(loaded.show_horizon_label, true);
      IM_CHECK_EQ(loaded.show_grid_line, false);
      IM_CHECK_EQ(loaded.show_grid_label, false);
      IM_CHECK_EQ(loaded.show_sun_circles_line, true);
      IM_CHECK_EQ(loaded.show_sun_circles_label, true);
    };
  }

  // task-overlay-line-label-toggle: Test F — new keys take precedence over the
  // legacy key when both are present (mixed-key scenario for hand-edited JSON).
  // Cover horizon AND grid to distinguish "all overlays handled correctly"
  // from "only horizon special-cased".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "overlay_new_keys_take_precedence");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({
        "layers": [],
        "renderer": {"lens_type": "linear", "fov": 90.0},
        "overlay_horizon": true,
        "overlay_horizon_line": true,
        "overlay_horizon_label": false,
        "overlay_grid": true,
        "overlay_grid_line": false,
        "overlay_grid_label": true
      })";
      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.show_horizon_line, true);
      IM_CHECK_EQ(loaded.show_horizon_label, false);
      IM_CHECK_EQ(loaded.show_grid_line, false);
      IM_CHECK_EQ(loaded.show_grid_label, true);
    };
  }

  // task-overlay-line-label-toggle: Test G — round-trip preserves all six
  // line/label fields independently (no collapsing back to single key on write).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "overlay_roundtrip_preserves_split");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      gui::g_state.show_horizon_line = true;
      gui::g_state.show_horizon_label = false;
      gui::g_state.show_grid_line = false;
      gui::g_state.show_grid_label = true;
      gui::g_state.show_sun_circles_line = true;
      gui::g_state.show_sun_circles_label = true;

      std::string json = gui::SerializeGuiStateJson(gui::g_state);
      IM_CHECK(!json.empty());

      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.show_horizon_line, true);
      IM_CHECK_EQ(loaded.show_horizon_label, false);
      IM_CHECK_EQ(loaded.show_grid_line, false);
      IM_CHECK_EQ(loaded.show_grid_label, true);
      IM_CHECK_EQ(loaded.show_sun_circles_line, true);
      IM_CHECK_EQ(loaded.show_sun_circles_label, true);
    };
  }

  // task-data-model-and-serialization: AC #7 — multi-raypath OR end-to-end.
  // GUI raypath_text "3-5; 1-3" → SerializeCoreConfig → ConfigManager parses
  // out 3 filters: 2 simple raypaths (ids 1, 2) + 1 complex (id 3) referencing
  // them. The main filter referenced by the scattering entry is the complex.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "multi_raypath_or_e2e");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Build a GuiState with one entry whose filter has a multi-segment raypath.
      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      gui::EntryCard entry;
      entry.crystal.type = gui::CrystalType::kPrism;
      entry.crystal.height = 1.0f;
      entry.crystal.face_distance[0] = 1.0f;
      entry.crystal.face_distance[1] = 1.0f;
      entry.crystal.face_distance[2] = 1.0f;
      entry.crystal.face_distance[3] = 1.0f;
      entry.crystal.face_distance[4] = 1.0f;
      entry.crystal.face_distance[5] = 1.0f;
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      f.param = gui::RaypathParams{ "3-5; 1-3" };
      entry.filter = f;
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      // Serialize → parse via core ConfigManager.
      std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      auto parsed = nlohmann::json::parse(core_json);
      lumice::ConfigManager m;
      lumice::from_json(parsed, m);

      // Expect 3 filters: id=1 (raypath {3,5}), id=2 (raypath {1,3}), id=3 (complex).
      IM_CHECK_EQ(static_cast<int>(m.filters_.size()), 3);
      IM_CHECK(m.filters_.count(1) == 1);
      IM_CHECK(m.filters_.count(2) == 1);
      IM_CHECK(m.filters_.count(3) == 1);

      // Simple filter 1 — first segment "3-5".
      const auto& f1 = m.filters_.at(1);
      const auto& s1 = std::get<lumice::SimpleFilterParam>(f1.param_);
      const auto& rp1 = std::get<lumice::RaypathFilterParam>(s1);
      IM_CHECK_EQ(static_cast<int>(rp1.raypath_.size()), 2);
      IM_CHECK_EQ(static_cast<int>(rp1.raypath_[0]), 3);
      IM_CHECK_EQ(static_cast<int>(rp1.raypath_[1]), 5);

      // Simple filter 2 — second segment "1-3".
      const auto& f2 = m.filters_.at(2);
      const auto& s2 = std::get<lumice::SimpleFilterParam>(f2.param_);
      const auto& rp2 = std::get<lumice::RaypathFilterParam>(s2);
      IM_CHECK_EQ(static_cast<int>(rp2.raypath_.size()), 2);
      IM_CHECK_EQ(static_cast<int>(rp2.raypath_[0]), 1);
      IM_CHECK_EQ(static_cast<int>(rp2.raypath_[1]), 3);

      // Complex filter 3 — composition of [[1], [2]].
      const auto& fc = m.filters_.at(3);
      const auto& cp = std::get<lumice::ComplexFilterParam>(fc.param_);
      IM_CHECK_EQ(static_cast<int>(cp.filters_.size()), 2);
      IM_CHECK_EQ(static_cast<int>(cp.filters_[0].size()), 1);
      IM_CHECK_EQ(static_cast<int>(cp.filters_[0][0].first), 1);
      IM_CHECK_EQ(static_cast<int>(cp.filters_[1].size()), 1);
      IM_CHECK_EQ(static_cast<int>(cp.filters_[1][0].first), 2);

      // Scattering entry should reference the complex filter (id 3).
      IM_CHECK(parsed.contains("scene"));
      IM_CHECK(parsed["scene"]["scattering"][0]["entries"][0]["filter"] == 3);
    };
  }

  // task-data-model-and-serialization: AC #8 — single-segment raypath stays a
  // RaypathFilterParam (no forced upgrade to ComplexFilterParam).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "single_raypath_no_complex_upgrade");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      gui::EntryCard entry;
      entry.crystal.type = gui::CrystalType::kPrism;
      entry.crystal.height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        entry.crystal.face_distance[i] = 1.0f;
      }
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      f.param = gui::RaypathParams{ "3-1-5" };
      entry.filter = f;
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      auto parsed = nlohmann::json::parse(core_json);
      lumice::ConfigManager m;
      lumice::from_json(parsed, m);

      // Exactly 1 simple raypath, no complex.
      IM_CHECK_EQ(static_cast<int>(m.filters_.size()), 1);
      const auto& f1 = m.filters_.begin()->second;
      const auto& sp = std::get<lumice::SimpleFilterParam>(f1.param_);
      IM_CHECK(std::holds_alternative<lumice::RaypathFilterParam>(sp));
      const auto& rp = std::get<lumice::RaypathFilterParam>(sp);
      IM_CHECK_EQ(static_cast<int>(rp.raypath_.size()), 3);
    };
  }

  // task-data-model-and-serialization: per-type GUI JSON round-trip — every
  // FilterParamVariant alternative survives SerializeFilterForGui →
  // ParseFilterFromGuiJson via the .lmc save/load path.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "per_type_filter_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Build one entry per filter type.
      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;

      auto make_entry = [](gui::FilterParamVariant param) {
        gui::EntryCard e;
        e.crystal.type = gui::CrystalType::kPrism;
        e.crystal.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          e.crystal.face_distance[i] = 1.0f;
        }
        e.proportion = 25.0f;
        gui::FilterConfig f;
        f.param = std::move(param);
        e.filter = f;
        return e;
      };

      layer.entries.push_back(make_entry(gui::RaypathParams{ "3-1-5" }));
      layer.entries.push_back(make_entry(gui::EntryExitParams{ "7", "4" }));
      layer.entries.push_back(make_entry(gui::DirectionParams{ 30.0f, -10.0f }));
      gui::g_state.layers.push_back(layer);

      const char* tmp_path = "/tmp/lumice_per_type_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 3);

      const auto& f0 = *gui::g_state.layers[0].entries[0].filter;
      IM_CHECK(std::holds_alternative<gui::RaypathParams>(f0.param));
      IM_CHECK_EQ(std::get<gui::RaypathParams>(f0.param).raypath_text, std::string("3-1-5"));

      const auto& f1 = *gui::g_state.layers[0].entries[1].filter;
      IM_CHECK(std::holds_alternative<gui::EntryExitParams>(f1.param));
      const auto& ee = std::get<gui::EntryExitParams>(f1.param);
      IM_CHECK_EQ(ee.entry_text, std::string("7"));
      IM_CHECK_EQ(ee.exit_text, std::string("4"));

      const auto& f2 = *gui::g_state.layers[0].entries[2].filter;
      IM_CHECK(std::holds_alternative<gui::DirectionParams>(f2.param));
      const auto& dp = std::get<gui::DirectionParams>(f2.param);
      IM_CHECK_EQ(dp.az, 30.0f);
      IM_CHECK_EQ(dp.el, -10.0f);

      std::remove(tmp_path);
    };
  }

  // task-data-model-and-serialization: legacy v=1 .lmc / GUI JSON without
  // `type` field falls back to RaypathParams (default raypath_text "").
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "legacy_no_type_falls_back_to_raypath");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // A v=1-style payload: filter object lacks `type` and uses raypath_text directly.
      std::string json = R"({
        "schema_version": 1,
        "layers": [
          { "prob": 0.0, "entries": [
            { "crystal": { "type": "prism", "shape": { "height": 1.0, "face_distance": [1,1,1,1,1,1] } },
              "proportion": 100.0,
              "filter": { "id": 1, "action": "filter_in", "raypath_text": "3-1-5", "sym_p": true, "sym_b": true, "sym_d": true }
            }
          ]}
        ],
        "renderer": {"lens_type": "linear", "fov": 90.0}
      })";
      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(loaded.layers[0].entries.size()), 1);
      IM_CHECK(loaded.layers[0].entries[0].filter.has_value());
      const auto& f = *loaded.layers[0].entries[0].filter;
      IM_CHECK(std::holds_alternative<gui::RaypathParams>(f.param));
      IM_CHECK_EQ(std::get<gui::RaypathParams>(f.param).raypath_text, std::string("3-1-5"));
    };
  }

  // task-per-type-entry-exit (issue 178.4): end-to-end equivalence between
  // GUI EE filter → SerializeCoreConfig output and a hand-crafted reference
  // core JSON object. Validates AC-6 (`type` / `action` / `symmetry` /
  // `entry` / `exit` / `id` field-by-field equality regardless of insertion
  // order via nlohmann::json::operator==).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "entry_exit_serialize_core_equivalent");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Build a single-entry GuiState carrying an EntryExitParams filter.
      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 1.0f;

      gui::EntryCard e;
      e.crystal.type = gui::CrystalType::kPrism;
      e.crystal.height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        e.crystal.face_distance[i] = 1.0f;
      }
      e.proportion = 100.0f;

      gui::FilterConfig fc;
      fc.action = 1;     // filter_out
      fc.sym_p = true;   // → "P" in symmetry suffix
      fc.sym_b = false;  // omitted
      fc.sym_d = true;   // → "D"
      fc.param = gui::EntryExitParams{ /*entry_text=*/"2", /*exit_text=*/"5" };
      e.filter = fc;

      layer.entries.push_back(e);
      gui::g_state.layers.push_back(layer);

      const std::string s = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!s.empty());

      const auto j = nlohmann::json::parse(s);
      IM_CHECK(j.contains("filter"));
      IM_CHECK(j["filter"].is_array());
      IM_CHECK_EQ(static_cast<int>(j["filter"].size()), 1);

      // Field-level assertions (also catches symmetry string ordering bugs).
      const auto& jf = j["filter"][0];
      IM_CHECK_STR_EQ(jf["type"].get<std::string>().c_str(), "entry_exit");
      IM_CHECK_STR_EQ(jf["action"].get<std::string>().c_str(), "filter_out");
      IM_CHECK_STR_EQ(jf["symmetry"].get<std::string>().c_str(), "PD");
      IM_CHECK_EQ(jf["entry"].get<int>(), 2);
      IM_CHECK_EQ(jf["exit"].get<int>(), 5);
      IM_CHECK_EQ(jf["id"].get<int>(), 1);

      // Whole-object equivalence with hand-crafted reference (json::operator==
      // is field-set + value equality, insertion-order independent).
      const nlohmann::json expected = {
        { "id", 1 },          { "type", "entry_exit" }, { "action", "filter_out" },
        { "symmetry", "PD" }, { "entry", 2 },           { "exit", 5 },
      };
      IM_CHECK(jf == expected);
    };
  }

  // task-per-type-direction (issue 178.5): end-to-end equivalence between
  // GUI Direction filter → SerializeCoreConfig output and a hand-crafted
  // reference core JSON object. Validates AC-7 (`type` / `action` /
  // `symmetry` / `az` / `el` / `radii` / `id` field-by-field equality
  // regardless of insertion order via nlohmann::json::operator==).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "direction_serialize_core_equivalent");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Build a single-entry GuiState carrying a DirectionParams filter.
      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 1.0f;

      gui::EntryCard e;
      e.crystal.type = gui::CrystalType::kPrism;
      e.crystal.height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        e.crystal.face_distance[i] = 1.0f;
      }
      e.proportion = 100.0f;

      gui::FilterConfig fc;
      fc.action = 1;     // filter_out
      fc.sym_p = true;   // → "P" in symmetry suffix
      fc.sym_b = false;  // omitted
      fc.sym_d = true;   // → "D"
      fc.param = gui::DirectionParams{ /*az=*/30.0f, /*el=*/45.0f };
      e.filter = fc;

      layer.entries.push_back(e);
      gui::g_state.layers.push_back(layer);

      const std::string s = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!s.empty());

      const auto j = nlohmann::json::parse(s);
      IM_CHECK(j.contains("filter"));
      IM_CHECK(j["filter"].is_array());
      IM_CHECK_EQ(static_cast<int>(j["filter"].size()), 1);

      // Field-level assertions (also catches symmetry string ordering bugs).
      // Note: `symmetry` field is still serialized for Direction filters even
      // though the GUI hides the P/B/D Checkboxes (H4 contract — UI hides
      // controls but data round-trips unchanged for cross-type compatibility).
      const auto& jf = j["filter"][0];
      IM_CHECK_STR_EQ(jf["type"].get<std::string>().c_str(), "direction");
      IM_CHECK_STR_EQ(jf["action"].get<std::string>().c_str(), "filter_out");
      IM_CHECK_STR_EQ(jf["symmetry"].get<std::string>().c_str(), "PD");
      IM_CHECK_EQ(jf["az"].get<float>(), 30.0f);
      IM_CHECK_EQ(jf["el"].get<float>(), 45.0f);
      // GUI no longer owns radii; serialization layer injects the fixed
      // default (kDirectionDefaultRadiiDeg, file_io.cpp).
      IM_CHECK_EQ(jf["radii"].get<float>(), 5.0f);
      IM_CHECK_EQ(jf["id"].get<int>(), 1);

      // Whole-object equivalence with hand-crafted reference.
      const nlohmann::json expected = {
        { "id", 1 },     { "type", "direction" }, { "action", "filter_out" }, { "symmetry", "PD" },
        { "az", 30.0f }, { "el", 45.0f },         { "radii", 5.0f },
      };
      IM_CHECK(jf == expected);
    };
  }

  // task-filter-modal-polish-v1: v2 .lmc-style Direction filter JSON
  // (with explicit radii) must load successfully into a v3 DirectionParams
  // (radii silently dropped — GUI struct lacks the field). Re-serializing
  // (SerializeGuiStateJson) must omit "radii" from the filter object.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "direction_v2_radii_dropped_on_load");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      const std::string v2_lmc = R"({
        "schema_version": 2,
        "layers": [{
          "prob": 0.0,
          "entries": [{
            "crystal": {"type": "prism", "shape": {"height": 1.0}},
            "proportion": 100.0,
            "filter": {
              "type": "direction",
              "action": "filter_in",
              "az": 22.0, "el": 5.0, "radii": 7.5
            }
          }]
        }]
      })";

      gui::GuiState restored;
      bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
      IM_CHECK(ok);
      IM_CHECK(restored.layers[0].entries[0].filter.has_value());
      const auto& f = *restored.layers[0].entries[0].filter;
      IM_CHECK(std::holds_alternative<gui::DirectionParams>(f.param));
      const auto& dp = std::get<gui::DirectionParams>(f.param);
      IM_CHECK_EQ(dp.az, 22.0f);
      IM_CHECK_EQ(dp.el, 5.0f);
      // radii silently discarded — DirectionParams no longer has the field.

      // Re-serializing must omit "radii" from the .lmc filter object.
      const std::string written = gui::SerializeGuiStateJson(restored);
      const auto j = nlohmann::json::parse(written);
      const auto& jf = j["layers"][0]["entries"][0]["filter"];
      IM_CHECK_STR_EQ(jf["type"].get<std::string>().c_str(), "direction");
      IM_CHECK(!jf.contains("radii"));
      IM_CHECK_EQ(jf["az"].get<float>(), 22.0f);
      IM_CHECK_EQ(jf["el"].get<float>(), 5.0f);
    };
  }

  // task-filter-modal-polish-v1: legacy v2 .lmc Entry-Exit filter (with
  // int "entry" / "exit" fields) must load successfully and translate to
  // the v3 string representation. Re-serialization writes the new
  // entry_text / exit_text fields.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "entry_exit_v2_int_translates_to_text_on_load");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      const std::string v2_lmc = R"({
        "schema_version": 2,
        "layers": [{
          "prob": 0.0,
          "entries": [{
            "crystal": {"type": "prism", "shape": {"height": 1.0}},
            "proportion": 100.0,
            "filter": {
              "type": "entry_exit",
              "action": "filter_in",
              "entry": 7, "exit": 4
            }
          }]
        }]
      })";

      gui::GuiState restored;
      bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
      IM_CHECK(ok);
      IM_CHECK(restored.layers[0].entries[0].filter.has_value());
      const auto& f = *restored.layers[0].entries[0].filter;
      IM_CHECK(std::holds_alternative<gui::EntryExitParams>(f.param));
      const auto& ee = std::get<gui::EntryExitParams>(f.param);
      IM_CHECK_EQ(ee.entry_text, std::string("7"));
      IM_CHECK_EQ(ee.exit_text, std::string("4"));

      // Re-serialization writes the new entry_text / exit_text fields.
      const std::string written = gui::SerializeGuiStateJson(restored);
      const auto j = nlohmann::json::parse(written);
      const auto& jf = j["layers"][0]["entries"][0]["filter"];
      IM_CHECK_STR_EQ(jf["type"].get<std::string>().c_str(), "entry_exit");
      IM_CHECK_STR_EQ(jf["entry_text"].get<std::string>().c_str(), "7");
      IM_CHECK_STR_EQ(jf["exit_text"].get<std::string>().c_str(), "4");
    };
  }
}
