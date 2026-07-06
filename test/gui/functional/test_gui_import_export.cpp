#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>

#include "IconsFontAwesome6.h"
#include "gui/raypath_segments.hpp"  // ParseSummandText / SumOfProducts (SoP round-trip tests)
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

  // task-gui-ms-prob-footguns: layer.probability round-trip must byte-preserve
  // hand-written values including the "footgun" case (last-layer prob>0). The
  // GUI's disable/warning logic is display-only; it must not silently rewrite
  // the loaded value. Locks: file value == deserialized state.probability.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "last_layer_prob_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Two layers, both with prob>0 (last layer would trigger the CLI
      // warning, but load must still preserve the value byte-for-byte).
      gui::g_state.layers[0].probability = 0.3f;
      gui::Layer new_layer;
      gui::EntryCard e;
      e.crystal_id = 0;
      new_layer.entries.push_back(e);
      new_layer.probability = 0.45f;  // last layer, non-zero — the footgun value
      gui::g_state.layers.push_back(std::move(new_layer));

      std::string json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!json.empty());

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(json, loaded);
      IM_CHECK(ok);

      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 2);
      IM_CHECK_EQ(loaded.layers[0].probability, 0.3f);
      IM_CHECK_EQ(loaded.layers[1].probability, 0.45f);  // NOT silently zeroed

      // Second serialization path (code-review Major-1): the C struct commit
      // path (FillLumiceConfig -> LUMICE_CommitConfigStruct) is what actually
      // feeds the simulator. Assert it preserves the last-layer footgun prob>0
      // too — the display disable/warning logic must not zero the stored value
      // on the path that reaches core. This path is commit-only (no reverse
      // deserialize), so it is a forward-fidelity check, not a round-trip.
      // (The .lmc save path SerializeGuiStateJson uses the identical
      // `jl["prob"] = layer.probability` float primitive as SerializeCoreConfig
      // above, so its prob fidelity is covered by equivalence.)
      // Feed the DESERIALIZED state (not the original g_state) so this is the
      // full end-to-end chain: file JSON -> deserialize -> loaded -> C struct
      // -> core (code-review r2 Minor-1).
      LUMICE_Config cfg;
      gui::FillLumiceConfig(loaded, &cfg);
      IM_CHECK_EQ(cfg.scatter_count, 2);
      IM_CHECK_EQ(cfg.scattering[0].probability, 0.3f);
      IM_CHECK_EQ(cfg.scattering[1].probability, 0.45f);
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
      IM_CHECK_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[0]).type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[0]).height, 2.0f);
      IM_CHECK_EQ(loaded.layers[0].entries[0].proportion, 60.0f);
      IM_CHECK_EQ(gui::CrystalOf(loaded, loaded.layers[0].entries[1]).type, gui::CrystalType::kPyramid);
      IM_CHECK(!loaded.layers[0].entries[1].filter_id.has_value());
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
      gui::CrystalOf(gui::g_state, entry0).type = gui::CrystalType::kPyramid;
      gui::CrystalOf(gui::g_state, entry0).prism_h = 2.0f;
      gui::CrystalOf(gui::g_state, entry0).upper_h = 0.3f;
      gui::CrystalOf(gui::g_state, entry0).lower_h = 0.4f;
      // Non-default face_distance to exercise the conditional-write branch in SerializeCrystal.
      gui::CrystalOf(gui::g_state, entry0).face_distance[0] = 1.2f;
      gui::CrystalOf(gui::g_state, entry0).face_distance[3] = 0.9f;
      // Non-default axis distributions to verify axis round-trip on all three axes.
      // All three use the same SerializeAxisDist/FillAxisDist path, but covering each
      // individually guards against future per-axis special-casing (e.g. roll default-omission).
      gui::CrystalOf(gui::g_state, entry0).zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 25.0f, 3.0f };
      gui::CrystalOf(gui::g_state, entry0).azimuth = gui::AxisDist{ gui::AxisDistType::kUniform, 10.0f, 20.0f };
      gui::CrystalOf(gui::g_state, entry0).roll = gui::AxisDist{ gui::AxisDistType::kLaplacian, 5.0f, 2.0f };
      entry0.proportion = 75.0f;
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, entry0, f);

      // Add a second entry with its own crystal pool slot (ID-pool model).
      gui::CrystalConfig extra_crystal;
      extra_crystal.type = gui::CrystalType::kPrism;
      extra_crystal.height = 3.0f;
      gui::EntryCard extra;
      extra.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(extra_crystal);
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
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).prism_h, 2.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).upper_h, 0.3f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).lower_h, 0.4f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).face_distance[0], 1.2f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).face_distance[3], 0.9f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).zenith.type, gui::AxisDistType::kGauss);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).zenith.mean, 25.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).zenith.std, 3.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).azimuth.type, gui::AxisDistType::kUniform);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).azimuth.mean, 10.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).azimuth.std, 20.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).roll.type, gui::AxisDistType::kLaplacian);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).roll.mean, 5.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).roll.std, 2.0f);
      IM_CHECK_EQ(loaded0.proportion, 75.0f);
      IM_CHECK(loaded0.filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.filters[*loaded0.filter_id].RaypathText(), std::string("3-1-5"));

      auto& loaded1 = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded1).type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded1).height, 3.0f);
      IM_CHECK_EQ(loaded1.proportion, 25.0f);

      std::remove(tmp_path);
    };
  }

  // Test: custom-spectrum end-to-end round-trip (task-323).
  // Covers all 4 file_io write paths + 1 load path introduced by the discrete-spectrum work:
  //   1. GUI project save (root["sun"]["spectrum"]="custom" + "custom_spectrum" array)
  //   2. GUI project load (reads back the array, restores kCustomSpectrumIndex)
  //   3. SerializeCoreConfig (light_source.spectrum emitted as a JSON array)
  //   4. FillLumiceConfig (LUMICE_Config.spectrum_entries[]/spectrum_count populated)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "custom_spectrum_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Configure a 3-entry custom spectrum.
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };

      // (1)+(2) .lmc save/load round-trip.
      const char* tmp_path = "/tmp/lumice_custom_spectrum_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);
      gui::DoNew();
      IM_CHECK(gui::g_state.sun.spectrum_index != gui::kCustomSpectrumIndex);  // reset baseline
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), (size_t)3);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[0].wavelength, 450.0f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[0].weight, 0.5f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[1].wavelength, 550.0f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[2].weight, 0.7f);
      std::remove(tmp_path);

      // (3) SerializeCoreConfig emits an array-form spectrum. Matches core light_config.cpp
      // SpectrumToJson shape ([{wavelength, weight}, ...]).
      std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      auto parsed = nlohmann::json::parse(core_json);
      const auto& spec = parsed["scene"]["light_source"]["spectrum"];
      IM_CHECK(spec.is_array());
      IM_CHECK_EQ(spec.size(), (size_t)3);
      IM_CHECK_EQ(spec[0]["wavelength"].get<float>(), 450.0f);
      IM_CHECK_EQ(spec[1]["weight"].get<float>(), 1.0f);
      IM_CHECK_EQ(spec[2]["wavelength"].get<float>(), 650.0f);

      // (4) FillLumiceConfig populates spectrum_entries[]/spectrum_count.
      LUMICE_Config cfg{};
      gui::FillLumiceConfig(gui::g_state, &cfg);
      IM_CHECK_EQ(cfg.spectrum_count, 3);
      IM_CHECK_EQ(cfg.spectrum_entries[0].wavelength, 450.0f);
      IM_CHECK_EQ(cfg.spectrum_entries[0].weight, 0.5f);
      IM_CHECK_EQ(cfg.spectrum_entries[2].wavelength, 650.0f);
    };
  }

  // Test: legacy core-JSON import with array-form spectrum.
  // Regression guard for the historical silent-drop bug in file_io.cpp (task-323 Step 5 #5):
  // arrays were previously discarded without warning; must now populate custom_spectrum.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "core_json_array_spectrum_import");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Baseline: reset state has a preset spectrum, not custom.
      IM_CHECK(gui::g_state.sun.spectrum_index != gui::kCustomSpectrumIndex);

      // Build a minimal core-JSON with an array-form spectrum and import it.
      nlohmann::json root;
      nlohmann::json cr;
      cr["id"] = 1;
      cr["type"] = "prism";
      cr["shape"]["height"] = 1.0f;
      cr["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
      cr["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
      cr["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
      root["crystal"] = nlohmann::json::array({ cr });
      root["scene"]["light_source"]["type"] = "sun";
      root["scene"]["light_source"]["altitude"] = 20.0f;
      root["scene"]["light_source"]["diameter"] = 0.5f;
      root["scene"]["light_source"]["spectrum"] = nlohmann::json::array(
          { { { "wavelength", 480.0f }, { "weight", 0.9f } }, { { "wavelength", 620.0f }, { "weight", 1.0f } } });
      root["scene"]["ray_num"] = 1000;
      root["scene"]["max_hits"] = 7;
      root["scene"]["scattering"] = nlohmann::json::array(
          { { { "prob", 1.0f },
              { "entries", nlohmann::json::array({ { { "crystal", 1 }, { "proportion", 1.0f } } }) } } });

      // Import via the real Core-JSON entry point (mirrors the GUI "Open .json"
      // flow: app.cpp:498 DeserializeFromJson), NOT the binary .lmc loader.
      bool load_ok = gui::DeserializeFromJson(root.dump(), gui::g_state);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), (size_t)2);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[0].wavelength, 480.0f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[1].wavelength, 620.0f);
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
      r.front = true;
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
      IM_CHECK_EQ(loaded.renderer.front, true);
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
      IM_CHECK_EQ(loaded.renderer.front, false);             // no "front" key → default false
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
      IM_CHECK_EQ(loaded.renderer.front, false);
      IM_CHECK_EQ(loaded.renderer.opacity, 1.0f);
    };
  }

  // Test 4e: legacy "visible": "front" maps to base=full + front=true
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "renderer_legacy_front_compat");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({
        "layers": [],
        "renderer": {
          "lens_type": "fisheye_equal_area",
          "fov": 180.0,
          "visible": "front"
        }
      })";

      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.renderer.visible, gui::kVisibleFull);
      IM_CHECK_EQ(loaded.renderer.front, true);
    };
  }

  // Test 4f: legacy "visible": "front" in core-JSON format (DeserializeFromJson path)
  // Covers the root["render"][0]["visible"]=="front" branch at file_io.cpp:1038-1041,
  // which is distinct from the GUI-JSON "renderer" object path tested by renderer_legacy_front_compat.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "core_json_legacy_front_compat");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      std::string json = R"({
        "crystal": [],
        "filter": [],
        "scene": {"scattering": [], "ray_num": 1000, "max_hits": 4,
                   "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}},
        "render": [
          {
            "lens": {"type": "fisheye_equal_area", "fov": 180.0},
            "visible": "front"
          }
        ]
      })";

      gui::GuiState loaded;
      bool ok = gui::DeserializeFromJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.renderer.visible, gui::kVisibleFull);
      IM_CHECK_EQ(loaded.renderer.front, true);
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

  // task-gui-zenith-nadir-marker: zenith/nadir overlay round-trip — 4 new fields
  // (line toggle, color[3], alpha, radius_px) preserved through Serialize → Deserialize.
  // Missing-key fallback covered separately by overlay_legacy_key_fallback.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "zenith_nadir_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      gui::g_state.show_zenith_nadir_line = true;
      gui::g_state.zenith_nadir_color[0] = 0.1f;
      gui::g_state.zenith_nadir_color[1] = 0.5f;
      gui::g_state.zenith_nadir_color[2] = 0.9f;
      gui::g_state.zenith_nadir_alpha = 0.42f;
      gui::g_state.zenith_nadir_radius_px = 12.5f;

      std::string json = gui::SerializeGuiStateJson(gui::g_state);
      IM_CHECK(!json.empty());

      gui::GuiState loaded;
      bool ok = gui::DeserializeGuiStateJson(json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(loaded.show_zenith_nadir_line, true);
      IM_CHECK_EQ(loaded.zenith_nadir_color[0], 0.1f);
      IM_CHECK_EQ(loaded.zenith_nadir_color[1], 0.5f);
      IM_CHECK_EQ(loaded.zenith_nadir_color[2], 0.9f);
      IM_CHECK_EQ(loaded.zenith_nadir_alpha, 0.42f);
      IM_CHECK_EQ(loaded.zenith_nadir_radius_px, 12.5f);

      // Legacy file (without these keys) must fall back to defaults — verifies
      // no exception and no spurious enabled state for users on older .lmc files.
      std::string legacy_json = R"({
        "layers": [],
        "renderer": {"lens_type": "linear", "fov": 90.0}
      })";
      gui::GuiState legacy_loaded;
      IM_CHECK(gui::DeserializeGuiStateJson(legacy_json, legacy_loaded));
      IM_CHECK_EQ(legacy_loaded.show_zenith_nadir_line, false);
      IM_CHECK_EQ(legacy_loaded.zenith_nadir_color[0], 0.8f);
      IM_CHECK_EQ(legacy_loaded.zenith_nadir_color[1], 0.2f);
      IM_CHECK_EQ(legacy_loaded.zenith_nadir_color[2], 0.2f);
      IM_CHECK_EQ(legacy_loaded.zenith_nadir_alpha, 0.6f);
      IM_CHECK_EQ(legacy_loaded.zenith_nadir_radius_px, 8.0f);
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
      gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
      gui::CrystalOf(gui::g_state, entry).height = 1.0f;
      gui::CrystalOf(gui::g_state, entry).face_distance[0] = 1.0f;
      gui::CrystalOf(gui::g_state, entry).face_distance[1] = 1.0f;
      gui::CrystalOf(gui::g_state, entry).face_distance[2] = 1.0f;
      gui::CrystalOf(gui::g_state, entry).face_distance[3] = 1.0f;
      gui::CrystalOf(gui::g_state, entry).face_distance[4] = 1.0f;
      gui::CrystalOf(gui::g_state, entry).face_distance[5] = 1.0f;
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-5; 1-3" });
      gui::SetFilter(gui::g_state, entry, f);
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      // Serialize and inspect the emitted core JSON directly. NOTE: this test
      // validates the serialized JSON shape only (filter array layout, ids,
      // composition structure) — not ConfigManager round-trip semantics. If
      // round-trip behavior needs guarding in the future, add a dedicated
      // core-side test or a config-deserialization C API (see backlog).
      std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      auto parsed = nlohmann::json::parse(core_json);

      // SerializeCoreConfig (src/gui/file_io.cpp:578-626) starts next_filter_id
      // at 1 and push_back's filters in SerializeFilterForCore order — multi-
      // segment raypath emits children first (ids 1..N) then the complex
      // (id N+1), so array index matches id - 1.
      IM_CHECK(parsed.contains("filter") && parsed["filter"].is_array());
      IM_CHECK_EQ(static_cast<int>(parsed["filter"].size()), 3);

      // Filter 1: raypath [3, 5] (segment "3-5").
      IM_CHECK_EQ(parsed["filter"][0]["id"].get<int>(), 1);
      IM_CHECK_STR_EQ(parsed["filter"][0]["type"].get<std::string>().c_str(), "raypath");
      IM_CHECK_EQ(static_cast<int>(parsed["filter"][0]["raypath"].size()), 2);
      IM_CHECK_EQ(parsed["filter"][0]["raypath"][0].get<int>(), 3);
      IM_CHECK_EQ(parsed["filter"][0]["raypath"][1].get<int>(), 5);

      // Filter 2: raypath [1, 3] (segment "1-3").
      IM_CHECK_EQ(parsed["filter"][1]["id"].get<int>(), 2);
      IM_CHECK_STR_EQ(parsed["filter"][1]["type"].get<std::string>().c_str(), "raypath");
      IM_CHECK_EQ(static_cast<int>(parsed["filter"][1]["raypath"].size()), 2);
      IM_CHECK_EQ(parsed["filter"][1]["raypath"][0].get<int>(), 1);
      IM_CHECK_EQ(parsed["filter"][1]["raypath"][1].get<int>(), 3);

      // Filter 3: complex, composition [[1], [2]].
      IM_CHECK_EQ(parsed["filter"][2]["id"].get<int>(), 3);
      IM_CHECK_STR_EQ(parsed["filter"][2]["type"].get<std::string>().c_str(), "complex");
      IM_CHECK(parsed["filter"][2].contains("composition"));
      IM_CHECK_EQ(static_cast<int>(parsed["filter"][2]["composition"].size()), 2);
      IM_CHECK_EQ(static_cast<int>(parsed["filter"][2]["composition"][0].size()), 1);
      IM_CHECK_EQ(parsed["filter"][2]["composition"][0][0].get<int>(), 1);
      IM_CHECK_EQ(static_cast<int>(parsed["filter"][2]["composition"][1].size()), 1);
      IM_CHECK_EQ(parsed["filter"][2]["composition"][1][0].get<int>(), 2);

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
      gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
      gui::CrystalOf(gui::g_state, entry).height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
      }
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, entry, f);
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      // Inspect serialized JSON directly — same shape-only contract as the
      // multi_raypath_or_e2e test above.
      std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      auto parsed = nlohmann::json::parse(core_json);

      // Exactly 1 simple raypath filter, no complex upgrade.
      IM_CHECK(parsed.contains("filter") && parsed["filter"].is_array());
      IM_CHECK_EQ(static_cast<int>(parsed["filter"].size()), 1);
      IM_CHECK_STR_EQ(parsed["filter"][0]["type"].get<std::string>().c_str(), "raypath");
      IM_CHECK_EQ(static_cast<int>(parsed["filter"][0]["raypath"].size()), 3);
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

      auto make_entry = [](gui::Factor param) {
        // ID-pool model: each entry needs its own crystal + filter pool slot.
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::EntryCard e;
        e.crystal_id = static_cast<int>(gui::g_state.crystals.size());
        gui::g_state.crystals.push_back(c);
        e.proportion = 25.0f;
        gui::FilterConfig f;
        if (std::holds_alternative<gui::RaypathParams>(param)) {
          f.SetRaypath(std::get<gui::RaypathParams>(std::move(param)));
        } else {
          f.SetEntryExit(std::get<gui::EntryExitParams>(std::move(param)));
        }
        gui::SetFilter(gui::g_state, e, f);
        return e;
      };

      // Build one entry per remaining filter type (raypath, entry_exit).
      layer.entries.push_back(make_entry(gui::RaypathParams{ "3-1-5" }));
      layer.entries.push_back(make_entry(gui::EntryExitParams{ "7", "4" }));
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

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      const auto& f0 = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK(f0.IsRaypath());
      IM_CHECK_EQ(f0.RaypathText(), std::string("3-1-5"));

      const auto& f1 = gui::g_state.filters[*gui::g_state.layers[0].entries[1].filter_id];
      IM_CHECK(f1.IsEntryExit());
      const auto& ee = f1.EntryExitParamsValue();
      IM_CHECK_EQ(ee.entry_text, std::string("7"));
      IM_CHECK_EQ(ee.exit_text, std::string("4"));

      std::remove(tmp_path);
    };
  }

  // T2 (task-serialization-bidirectional, AC1): any sum-of-products (cross-type
  // OR + AND + internal multi-value + multiple rows) survives GUI -> .lmc -> GUI
  // with FilterConfig::operator== (text-layer) equality. Covers SerializeFilterForGui
  // ("summands") and ParseFilterFromGuiJson's new-form reader.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "sop_lmc_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // One entry per SoP filter shape. Each filter is built from canonical row
      // texts (the operator== identity), so the round-trip must reproduce them.
      const std::vector<std::vector<std::string>> shapes = {
        { "3-1-5" },                       // degenerate single raypath
        { "3-5", "1-3", "2-6" },           // multi-row raypath OR
        { "entry:7 & exit:4" },            // single EE
        { "entry:3 & exit:5 & len:2-6" },  // EE with length range
        { "entry:", "exit:5" },            // wildcard EE rows
        { "3-1-5", "entry:3 & exit:5" },   // cross-type OR
        { "entry:3 & 7-1" },               // AND clause (EE + raypath)
        { "entry:3,4 & 7-1", "2-6" },      // cross-type OR + AND + internal multi
      };

      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      std::vector<gui::FilterConfig> originals;
      for (const auto& rows : shapes) {
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::EntryCard e;
        e.crystal_id = static_cast<int>(gui::g_state.crystals.size());
        gui::g_state.crystals.push_back(c);
        e.proportion = 100.0f;
        gui::FilterConfig f;
        gui::SumOfProducts sop;
        for (const auto& row : rows) {
          sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
        }
        f.param = std::move(sop);
        originals.push_back(f);
        gui::SetFilter(gui::g_state, e, f);
        layer.entries.push_back(e);
      }
      gui::g_state.layers.push_back(layer);

      const char* tmp_path = "/tmp/lumice_sop_roundtrip.lmc";
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), static_cast<int>(shapes.size()));
      for (size_t i = 0; i < shapes.size(); ++i) {
        const auto& e = gui::g_state.layers[0].entries[i];
        IM_CHECK(e.filter_id.has_value());
        const auto& loaded = gui::g_state.filters[*e.filter_id];
        // AC1: field-equal at the operator== (text) layer.
        IM_CHECK(loaded == originals[i]);
      }
      std::remove(tmp_path);
    };
  }

  // T7 (task-serialization-bidirectional, SUGGESTION-1 GAP): a non-degenerate
  // SoP (multi-summand and/or multi-factor) must render a non-crash entry-card
  // summary. FilterSummary (panels.cpp) previously called DegenerateFactor(),
  // which asserts/UB on a non-degenerate SoP. Assert it returns a non-empty
  // string without crashing. (Full multi-summand editor UI is 333.4.)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "non_degenerate_sop_summary_no_crash");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      auto make_sop = [](const std::vector<std::string>& rows) {
        gui::FilterConfig f;
        gui::SumOfProducts sop;
        for (const auto& row : rows) {
          sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
        }
        f.param = std::move(sop);
        return f;
      };

      // Multi-summand OR.
      {
        gui::FilterConfig f = make_sop({ "3-5", "1-3", "2-6" });
        std::string s = gui::FilterSummary(std::optional<gui::FilterConfig>{ f });
        IM_CHECK(!s.empty());
        IM_CHECK(s.find("(+2 more)") != std::string::npos);
      }
      // Multi-factor AND (single row, non-degenerate: 1 row but 2 factors).
      {
        gui::FilterConfig f = make_sop({ "entry:3 & 7-1" });
        std::string s = gui::FilterSummary(std::optional<gui::FilterConfig>{ f });
        IM_CHECK(!s.empty());
      }
      // Mixed OR + AND.
      {
        gui::FilterConfig f = make_sop({ "entry:3,4 & 7-1", "2-6" });
        std::string s = gui::FilterSummary(std::optional<gui::FilterConfig>{ f });
        IM_CHECK(!s.empty());
      }
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
      IM_CHECK(loaded.layers[0].entries[0].filter_id.has_value());
      const auto& f = loaded.filters[*loaded.layers[0].entries[0].filter_id];
      IM_CHECK(f.IsRaypath());
      IM_CHECK_EQ(f.RaypathText(), std::string("3-1-5"));
    };
  }

  // T4 (task-serialization-bidirectional, AC3): a legacy v2 .lmc raypath filter
  // with ';' multi-segment sugar upgrades losslessly to a multi-row SoP via the
  // canonical FromLegacyRaypath fan-out (one OR row per segment).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "legacy_v2_multisegment_raypath_upgrades_to_sop");
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
            "filter": {"type": "raypath", "action": "filter_in", "raypath_text": "3-5;1-3"}
          }]
        }]
      })";

      gui::GuiState restored;
      bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
      IM_CHECK(ok);
      IM_CHECK(restored.layers[0].entries[0].filter_id.has_value());
      const auto& f = restored.filters[*restored.layers[0].entries[0].filter_id];
      // Split into two OR rows: "3-5" and "1-3".
      IM_CHECK_EQ(static_cast<int>(f.param.size()), 2);
      IM_CHECK_STR_EQ(f.param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(f.param[1].text.c_str(), "1-3");
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
      gui::CrystalOf(gui::g_state, e).type = gui::CrystalType::kPrism;
      gui::CrystalOf(gui::g_state, e).height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        gui::CrystalOf(gui::g_state, e).face_distance[i] = 1.0f;
      }
      e.proportion = 100.0f;

      gui::FilterConfig fc;
      fc.action = 1;     // filter_out
      fc.sym_p = true;   // → "P" in symmetry suffix
      fc.sym_b = false;  // omitted
      fc.sym_d = true;   // → "D"
      fc.SetEntryExit(gui::EntryExitParams{ /*entry_text=*/"2", /*exit_text=*/"5" });
      gui::SetFilter(gui::g_state, e, fc);

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

  // filter-direction-hide (issue 180.2): .lmc with type="direction" filter
  // must degrade to empty RaypathParams after the Direction type is removed
  // from the GUI. Mirrors Crystal's unknown-type fallback path (#179).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "direction_lmc_degrades_to_raypath");
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
              "az": 30.0, "el": 15.0
            }
          }]
        }]
      })";

      gui::GuiState restored;
      bool ok = gui::DeserializeGuiStateJson(v2_lmc, restored);
      IM_CHECK(ok);
      IM_CHECK_EQ(static_cast<int>(restored.layers[0].entries.size()), 1);

      // direction type → unknown-type fallback → empty RaypathParams
      IM_CHECK(restored.layers[0].entries[0].filter_id.has_value());
      const auto& f = restored.filters[*restored.layers[0].entries[0].filter_id];
      IM_CHECK(f.IsRaypath());
      IM_CHECK(f.RaypathText().empty());
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
      IM_CHECK(restored.layers[0].entries[0].filter_id.has_value());
      const auto& f = restored.filters[*restored.layers[0].entries[0].filter_id];
      IM_CHECK(f.IsEntryExit());
      const auto& ee = f.EntryExitParamsValue();
      IM_CHECK_EQ(ee.entry_text, std::string("7"));
      IM_CHECK_EQ(ee.exit_text, std::string("4"));

      // Re-serialization writes the v3 sum-of-products form: type "sop" +
      // "summands" array of canonical row texts (the legacy EE int fields upgrade
      // to the "entry:<e> & exit:<x>" grammar row).
      const std::string written = gui::SerializeGuiStateJson(restored);
      const auto j = nlohmann::json::parse(written);
      const auto& jf = j["layers"][0]["entries"][0]["filter"];
      IM_CHECK_STR_EQ(jf["type"].get<std::string>().c_str(), "sop");
      IM_CHECK(jf["summands"].is_array());
      IM_CHECK_EQ(static_cast<int>(jf["summands"].size()), 1);
      IM_CHECK_STR_EQ(jf["summands"][0].get<std::string>().c_str(), "entry:7 & exit:4");
    };
  }

  // T13 — AC7 serialization equivalence: Remove EE Filter via GUI → OK →
  // entry.filter_id = nullopt → SerializeCoreConfig emits no "filter" field.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "entry_exit_remove_ok_no_filter_in_json");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Pre-populate entry.filter with an EE filter.
      {
        gui::FilterConfig fc;
        fc.action = 0;
        fc.sym_p = true;
        fc.sym_b = true;
        fc.sym_d = true;
        gui::EntryExitParams ee;
        ee.entry_text = "3";
        ee.exit_text = "6";
        fc.SetEntryExit(ee);
        gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], fc);
      }
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      // H5 (333.4): the FilterEditType radio was retired; Remove Filter is a
      // single button (##filter). The pre-populated EE filter shows up as one
      // row already; Remove Filter arms the intent flag, OK writes nullopt.
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // filter must be nullopt after Remove + OK.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      // SerializeCoreConfig must not emit any "filter" field (no EE residue).
      const std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!core_json.empty());
      const auto j = nlohmann::json::parse(core_json);
      // No top-level "filter" array, or it's empty.
      const bool no_filter = !j.contains("filter") || j["filter"].empty();
      IM_CHECK(no_filter);
    };
  }

  // task-serialization-bidirectional (explore-271 reject->reconstruct): a
  // multi-segment raypath complex filter (N simple raypaths + 1 OR-of-singletons
  // complex) reconstructs into an N-row sum-of-products (one raypath factor per
  // row). Pre-uplift this merged back into a single ';'-joined RaypathParams; the
  // SoP model now surfaces each OR row explicitly (semantically equivalent, and
  // re-serialize byte-equivalent). No warning (fully representable).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "complex_raypath_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::ClearImportComplexFilterWarning();

      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      gui::EntryCard entry;
      gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
      gui::CrystalOf(gui::g_state, entry).height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
      }
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-5;1-3" });
      gui::SetFilter(gui::g_state, entry, f);
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      const std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!core_json.empty());

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(core_json, loaded);
      IM_CHECK(ok);

      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(loaded.layers[0].entries.size()), 1);
      const auto& loaded_entry = loaded.layers[0].entries[0];
      IM_CHECK(loaded_entry.filter_id.has_value());
      const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
      // Reconstructed as a 2-row SoP: row 0 = raypath "3-5", row 1 = raypath "1-3".
      IM_CHECK_EQ(static_cast<int>(loaded_filter.param.size()), 2);
      IM_CHECK_STR_EQ(loaded_filter.param[0].text.c_str(), "3-5");
      IM_CHECK_STR_EQ(loaded_filter.param[1].text.c_str(), "1-3");
      // No warning should have fired for a well-formed GUI-emitted complex.
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
      // Re-serialize equivalence: the reconstructed filter emits the SAME core
      // filter array (2 simple raypaths + 1 complex).
      const auto j_orig = nlohmann::json::parse(core_json);
      const auto j_reser = nlohmann::json::parse(gui::SerializeCoreConfig(loaded));
      IM_CHECK(j_orig["filter"] == j_reser["filter"]);
    };
  }

  // task-serialization-bidirectional (explore-271 reject->reconstruct): an EE
  // multi-value (cartesian product) complex filter reconstructs into one EE
  // factor per (entry,exit) pair, one row per clause. "3,4" x "5,6" -> 4 rows
  // (3,5),(3,6),(4,5),(4,6). Pre-uplift this re-factorized into comma lists;
  // the SoP model keeps each pair as its own row (re-serialize byte-equivalent).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "complex_ee_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::ClearImportComplexFilterWarning();

      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      gui::EntryCard entry;
      gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
      gui::CrystalOf(gui::g_state, entry).height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
      }
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      gui::EntryExitParams ee;
      ee.entry_text = "3,4";
      ee.exit_text = "5,6";
      f.SetEntryExit(ee);
      gui::SetFilter(gui::g_state, entry, f);
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      const std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!core_json.empty());
      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(core_json, loaded);
      IM_CHECK(ok);

      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      const auto& loaded_entry = loaded.layers[0].entries[0];
      IM_CHECK(loaded_entry.filter_id.has_value());
      const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
      // 4 clauses -> 4 rows, each a single EE factor. Serialize order was e outer,
      // x inner: (3,5),(3,6),(4,5),(4,6).
      IM_CHECK_EQ(static_cast<int>(loaded_filter.param.size()), 4);
      IM_CHECK_STR_EQ(loaded_filter.param[0].text.c_str(), "entry:3 & exit:5");
      IM_CHECK_STR_EQ(loaded_filter.param[1].text.c_str(), "entry:3 & exit:6");
      IM_CHECK_STR_EQ(loaded_filter.param[2].text.c_str(), "entry:4 & exit:5");
      IM_CHECK_STR_EQ(loaded_filter.param[3].text.c_str(), "entry:4 & exit:6");
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
      const auto j_orig = nlohmann::json::parse(core_json);
      const auto j_reser = nlohmann::json::parse(gui::SerializeCoreConfig(loaded));
      IM_CHECK(j_orig["filter"] == j_reser["filter"]);
    };
  }

  // task-serialization-bidirectional (explore-271 reject->reconstruct): a
  // wildcard entry (empty text) crossed with multiple exits serializes to a
  // complex filter (pair_count = 1 x N > 1). Reconstruct decodes the absent
  // "entry" field back to an empty (wildcard) string per row — exercising
  // DecodeEEFaceFromJson's absent-field branch. Two clauses -> two rows.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "complex_ee_wildcard_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::ClearImportComplexFilterWarning();

      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      gui::EntryCard entry;
      gui::CrystalOf(gui::g_state, entry).type = gui::CrystalType::kPrism;
      gui::CrystalOf(gui::g_state, entry).height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        gui::CrystalOf(gui::g_state, entry).face_distance[i] = 1.0f;
      }
      entry.proportion = 100.0f;
      gui::FilterConfig f;
      gui::EntryExitParams ee;
      ee.entry_text = "";  // wildcard entry
      ee.exit_text = "5,6";
      f.SetEntryExit(ee);
      gui::SetFilter(gui::g_state, entry, f);
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      const std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!core_json.empty());
      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(core_json, loaded);
      IM_CHECK(ok);

      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      const auto& loaded_entry = loaded.layers[0].entries[0];
      IM_CHECK(loaded_entry.filter_id.has_value());
      const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
      // 2 clauses -> 2 rows; entry omitted (wildcard) so the canonical text is
      // "entry: & exit:<N>" (empty entry_text after the "entry:" anchor).
      IM_CHECK_EQ(static_cast<int>(loaded_filter.param.size()), 2);
      IM_CHECK_STR_EQ(loaded_filter.param[0].text.c_str(), "entry: & exit:5");
      IM_CHECK_STR_EQ(loaded_filter.param[1].text.c_str(), "entry: & exit:6");
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
      const auto j_orig = nlohmann::json::parse(core_json);
      const auto j_reser = nlohmann::json::parse(gui::SerializeCoreConfig(loaded));
      IM_CHECK(j_orig["filter"] == j_reser["filter"]);
    };
  }

  // task-serialization-bidirectional (explore-271 reject->RECONSTRUCT): a true
  // AND-of-products complex filter (a composition product with >1 child id,
  // expressing AND) IS now representable — it reconstructs into a single OR row
  // whose factors are the ANDed children. Pre-uplift this was loudly rejected;
  // the SoP model makes it a first-class reconstruct (warning empty).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "and_of_products_complex_reconstructs");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::ClearImportComplexFilterWarning();

      // Hand-authored core JSON: 2 raypath simples + 1 complex with an
      // AND-of-products composition ([[1,2]] — id 1 AND id 2 in the same clause).
      const std::string core_json = R"({
        "crystal": [
          {"id": 1, "type": "Prism", "height": 1.0,
           "face_distance": [1,1,1,1,1,1]}
        ],
        "filter": [
          {"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "raypath", "action": "filter_in", "raypath": [1, 3]},
          {"id": 3, "type": "complex", "action": "filter_in",
           "composition": [[1, 2]]}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5},
          "ray_num": 1000,
          "max_hits": 8,
          "scattering": [
            {"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 3}]}
          ]
        }
      })";

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(core_json, loaded);
      IM_CHECK(ok);
      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      const auto& loaded_entry = loaded.layers[0].entries[0];
      // The complex reconstructs — the entry references it.
      IM_CHECK(loaded_entry.filter_id.has_value());
      const auto& lf = loaded.filters[*loaded_entry.filter_id];
      // One clause with two raypath terms -> one row with two raypath factors.
      IM_CHECK_EQ(static_cast<int>(lf.param.size()), 1);
      IM_CHECK_EQ(static_cast<int>(lf.param[0].factors.size()), 2);
      IM_CHECK(std::holds_alternative<gui::RaypathParams>(lf.param[0].factors[0]));
      IM_CHECK(std::holds_alternative<gui::RaypathParams>(lf.param[0].factors[1]));
      IM_CHECK_STR_EQ(lf.param[0].text.c_str(), "3-5 & 1-3");
      // No warning — fully representable now.
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
      // Re-serialize equivalence: back to the identical 2 raypath + 1 complex form.
      const auto j_orig = nlohmann::json::parse(core_json);
      const auto j_reser = nlohmann::json::parse(gui::SerializeCoreConfig(loaded));
      IM_CHECK(j_orig["filter"] == j_reser["filter"]);
    };
  }

  // task-serialization-bidirectional: genuinely non-representable complex inputs
  // still loudly reject (GUI `Factor` has only raypath / entry_exit arms). These
  // KEEP the explore-271 anti-silent-miscull contract for the unsupported cases:
  //   (a) child simple type not in {raypath, entry_exit} (e.g. "direction")
  //   (b) a term id that points to another complex filter (nested complex)
  //   (c) a term id with no matching child in the pool (dangling reference)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "unsupported_complex_still_rejects");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);

      // Drives DeserializeFromJson with a hand-authored core JSON, asserts the
      // referenced complex filter is NOT materialized and a warning fires.
      auto expect_reject = [](const std::string& core_json, int entry_filter_id) {
        IM_UNUSED(entry_filter_id);
        ResetTestState();
        gui::ClearImportComplexFilterWarning();
        gui::GuiState loaded = gui::InitDefaultState();
        bool ok = gui::DeserializeFromJson(core_json, loaded);
        IM_CHECK(ok);
        IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
        const auto& e = loaded.layers[0].entries[0];
        IM_CHECK(!e.filter_id.has_value());  // unsupported complex not materialized
        IM_CHECK(!gui::PeekImportComplexFilterWarning().empty());
        gui::ClearImportComplexFilterWarning();
      };

      // (a) child type "direction" — unknown to the GUI Factor variant.
      expect_reject(R"({
        "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
        "filter": [
          {"id": 1, "type": "direction", "action": "filter_in", "az": 30.0, "el": 15.0},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
          "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 2}]}]
        }
      })",
                    2);

      // (b) nested complex: a term id points to another complex filter.
      expect_reject(R"({
        "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
        "filter": [
          {"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]},
          {"id": 3, "type": "complex", "action": "filter_in", "composition": [[2]]}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
          "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 3}]}]
        }
      })",
                    3);

      // (c) dangling reference: term id has no matching child in the pool.
      expect_reject(R"({
        "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
        "filter": [
          {"id": 1, "type": "complex", "action": "filter_in", "composition": [[99]]}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
          "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 1}]}]
        }
      })",
                    1);

      // (d) empty clause: a composition product with no terms (pins the explicit
      // product.empty() reject added in TryReconstructComplexFilter; code-review-01 Minor 4).
      expect_reject(R"({
        "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
        "filter": [
          {"id": 1, "type": "raypath", "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[]]}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
          "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 2}]}]
        }
      })",
                    2);

      // (e) child filter with a MISSING "type" field — malformed core-JSON. Must loud-reject,
      // NOT silently rebuild as a match-all raypath (code-review-04 Major 1).
      expect_reject(R"({
        "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
        "filter": [
          {"id": 1, "action": "filter_in", "raypath": [3, 5]},
          {"id": 2, "type": "complex", "action": "filter_in", "composition": [[1]]}
        ],
        "scene": {
          "light_source": {"altitude": 20.0, "diameter": 0.5}, "ray_num": 1000, "max_hits": 8,
          "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 100.0, "filter": 2}]}]
        }
      })",
                    2);
    };
  }

  // 327.4 cross-check: the GUI filter expansion has two twins — SerializeFilterForCore
  // (GUI -> JSON) and ExpandFilterToStruct (GUI -> C struct, via FillLumiceConfig). For the
  // same GuiState the struct built directly must field-match the struct obtained by JSON
  // round-trip. Guards the twins against drift (plan §3-2 / review Major 1).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "filter_expand_struct_vs_json");
    t->TestFunc = [](ImGuiTestContext*) {
      // Core comparison: given a fully-built GuiState (single crystal, single
      // filter, single entry) the struct path (ExpandFilterToStruct) and the JSON
      // path (SerializeFilterForCore) must produce field-identical LUMICE_Config.
      auto run_cross_check = [](const gui::FilterConfig& f) {
        gui::g_state.filters.clear();
        gui::g_state.layers.clear();
        gui::g_state.filters.push_back(f);
        gui::Layer layer;
        layer.probability = 1.0f;
        gui::EntryCard e;
        e.crystal_id = 0;
        e.filter_id = 0;
        e.proportion = 100.0f;
        layer.entries.push_back(e);
        gui::g_state.layers.push_back(layer);

        LUMICE_Config from_struct{};
        IM_CHECK(gui::FillLumiceConfig(gui::g_state, &from_struct));  // struct path (ExpandFilterToStruct)
        std::string json = gui::SerializeCoreConfig(gui::g_state);    // JSON path (SerializeFilterForCore)
        LUMICE_Config from_json{};
        IM_CHECK_EQ(LUMICE_ParseConfigString(json.c_str(), &from_json), LUMICE_OK);

        IM_CHECK_EQ(from_struct.filter_count, from_json.filter_count);
        IM_CHECK_EQ(from_struct.composition_count, from_json.composition_count);
        for (int i = 0; i < from_struct.filter_count; i++) {
          const LUMICE_FilterParam& a = from_struct.filters[i];
          const LUMICE_FilterParam& b = from_json.filters[i];
          IM_CHECK_EQ(a.id, b.id);
          IM_CHECK_EQ(a.type, b.type);
          IM_CHECK_EQ(a.action, b.action);
          IM_CHECK_EQ(a.symmetry, b.symmetry);
          if (a.type == LUMICE_FILTER_TYPE_RAYPATH) {
            IM_CHECK_EQ(a.raypath_count, b.raypath_count);
            for (int k = 0; k < a.raypath_count; k++) {
              IM_CHECK_EQ(a.raypath[k], b.raypath[k]);
            }
          } else if (a.type == LUMICE_FILTER_TYPE_ENTRY_EXIT) {
            IM_CHECK_EQ(a.ee_entry, b.ee_entry);
            IM_CHECK_EQ(a.ee_exit, b.ee_exit);
            IM_CHECK_EQ(a.ee_min_len, b.ee_min_len);
            IM_CHECK_EQ(a.ee_max_len, b.ee_max_len);
          } else if (a.type == LUMICE_FILTER_TYPE_COMPLEX) {
            const LUMICE_ComplexComposition& ca = from_struct.compositions[a.composition_index];
            const LUMICE_ComplexComposition& cb = from_json.compositions[b.composition_index];
            IM_CHECK_EQ(ca.clause_count, cb.clause_count);
            for (int cl = 0; cl < ca.clause_count; cl++) {
              IM_CHECK_EQ(ca.term_counts[cl], cb.term_counts[cl]);
              for (int tt = 0; tt < ca.term_counts[cl]; tt++) {
                IM_CHECK_EQ(ca.clauses[cl][tt], cb.clauses[cl][tt]);
              }
            }
          }
        }
      };

      auto fresh_state = []() {
        ResetTestState();
        gui::g_state.crystals.clear();
        gui::g_state.filters.clear();
        gui::g_state.layers.clear();
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::g_state.crystals.push_back(c);
      };

      // Degenerate / type-internal-OR cases (AC4 byte-equivalence guard): built via
      // the compat SetRaypath / SetEntryExit writers, exactly as pre-uplift.
      auto cross_check = [&](gui::Factor param) {
        fresh_state();
        gui::FilterConfig f;
        if (std::holds_alternative<gui::RaypathParams>(param)) {
          f.SetRaypath(std::get<gui::RaypathParams>(std::move(param)));
        } else {
          f.SetEntryExit(std::get<gui::EntryExitParams>(std::move(param)));
        }
        run_cross_check(f);
      };

      // Full sum-of-products cases (new capability): each canonical summand text
      // becomes an OR row; ParseSummandText builds the AND-of-factors parse cache.
      auto cross_check_sop = [&](const std::vector<std::string>& rows) {
        fresh_state();
        gui::FilterConfig f;
        gui::SumOfProducts sop;
        for (const auto& row : rows) {
          sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
        }
        f.param = std::move(sop);
        run_cross_check(f);
      };

      cross_check(gui::RaypathParams{ "3-1-5" });          // single-segment -> 1 simple raypath
      cross_check(gui::RaypathParams{ "3-5; 1-4; 2-6" });  // multi-segment -> 3 simple + 1 complex
      cross_check(gui::EntryExitParams{ "3", "5" });       // single pair -> 1 simple EE
      gui::EntryExitParams ee_multi{ "3,5", "1" };         // 2x1 -> 2 simple + 1 complex
      ee_multi.length_mode = 3;
      ee_multi.min_len = 2;
      ee_multi.max_len = 6;
      cross_check(ee_multi);

      // Wildcard EE matrix: empty entry/exit strings map to LUMICE_EE_WILDCARD_SENTINEL and
      // must survive struct↔JSON cross-check on both single and multi-value sides.
      cross_check(gui::EntryExitParams{ "", "" });     // both wildcard -> 1 simple EE
      cross_check(gui::EntryExitParams{ "", "5" });    // entry wildcard + single exit -> 1 simple EE
      cross_check(gui::EntryExitParams{ "3", "" });    // single entry + exit wildcard -> 1 simple EE
      cross_check(gui::EntryExitParams{ "3,5", "" });  // 2 entry values × wildcard exit -> 2 simple + 1 complex
      cross_check(gui::EntryExitParams{ "", "3,5" });  // wildcard entry × 2 exit values -> 2 simple + 1 complex

      // Sum-of-products cases (T1, task-serialization-bidirectional):
      // 1. cross-type OR: raypath row + EE row -> 2 clauses, 1 term each, mixed type.
      cross_check_sop({ "3-1-5", "entry:3 & exit:5" });
      // 2. AND clause: one row with an EE factor AND a raypath factor -> 1 clause, 2 terms.
      cross_check_sop({ "entry:3 & 7-1" });
      // 3. cross-type OR + AND + internal multi-value (Cartesian distribution):
      //    "entry:3,4 & 7-1" -> 2 clauses (EE3&rp, EE4&rp) each 2 terms; "2-6" -> +1 clause.
      cross_check_sop({ "entry:3,4 & 7-1", "2-6" });

      // Overflow: a raypath with more than LUMICE_MAX_CONFIG_CLAUSES OR segments exceeds the
      // composition ABI bounds -> FillLumiceConfig must return false (graceful degradation),
      // so app.cpp keeps the prior committed state instead of committing a truncated config.
      {
        ResetTestState();
        gui::g_state.crystals.clear();
        gui::g_state.filters.clear();
        gui::g_state.layers.clear();
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::g_state.crystals.push_back(c);
        std::string too_many_segments;  // LUMICE_MAX_CONFIG_CLAUSES + 1 OR segments
        for (int i = 0; i < LUMICE_MAX_CONFIG_CLAUSES + 1; ++i) {
          if (i) {
            too_many_segments += ";";
          }
          too_many_segments += "3-5";
        }
        gui::FilterConfig f;
        f.name = "OverflowFilter";  // named so overflow.filter_name assertion below is meaningful
        f.SetRaypath(gui::RaypathParams{ too_many_segments });
        gui::g_state.filters.push_back(f);
        gui::Layer layer;
        layer.probability = 1.0f;
        gui::EntryCard e;
        e.crystal_id = 0;
        e.filter_id = 0;
        e.proportion = 100.0f;
        layer.entries.push_back(e);
        gui::g_state.layers.push_back(layer);
        LUMICE_Config over{};
        gui::FilterOverflowInfo overflow;
        IM_CHECK(!gui::FillLumiceConfig(gui::g_state, &over, &overflow));  // over ABI bounds -> false
        // "no partial writes on overflow" contract (ExpandFilterToStruct doc): the overflowing
        // filter bails before any filter/composition is written for it.
        IM_CHECK_EQ(over.filter_count, 0);
        IM_CHECK_EQ(over.composition_count, 0);
        // Overflow identity: the first (layer 0, entry 0) reference is captured with the
        // FilterConfig::name so the caller can locate the offending filter for the user.
        IM_CHECK_EQ(overflow.layer_index, 0);
        IM_CHECK_EQ(overflow.entry_index, 0);
        IM_CHECK_EQ(overflow.filter_name, std::string("OverflowFilter"));
        // The locator string DoRun embeds in the modal + Log message (named filter form).
        IM_CHECK_STR_EQ(gui::FormatOverflowLocator(overflow).c_str(), "filter \"OverflowFilter\", Layer 1 / Entry 1");
      }

      // Overflow (new trigger, task-serialization-bidirectional): a single OR row
      // with more than LUMICE_MAX_CONFIG_TERMS AND factors exceeds the per-clause
      // term ABI bound. Pre-uplift this was unreachable (clauses were always
      // singletons); a multi-factor AND row can now hit it. Must return false with
      // no partial write.
      {
        ResetTestState();
        gui::g_state.crystals.clear();
        gui::g_state.filters.clear();
        gui::g_state.layers.clear();
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::g_state.crystals.push_back(c);
        // LUMICE_MAX_CONFIG_TERMS + 1 raypath factors ANDed together in one row.
        std::string too_many_terms;
        for (int i = 0; i < LUMICE_MAX_CONFIG_TERMS + 1; ++i) {
          if (i) {
            too_many_terms += " & ";
          }
          too_many_terms += std::to_string(i + 1) + "-" + std::to_string(i + 2);
        }
        gui::FilterConfig f;
        f.param = gui::SumOfProducts{ gui::SummandText{ too_many_terms, gui::ParseSummandText(too_many_terms) } };
        gui::g_state.filters.push_back(f);
        gui::Layer layer;
        layer.probability = 1.0f;
        gui::EntryCard e;
        e.crystal_id = 0;
        e.filter_id = 0;
        e.proportion = 100.0f;
        layer.entries.push_back(e);
        gui::g_state.layers.push_back(layer);
        LUMICE_Config over{};
        IM_CHECK(!gui::FillLumiceConfig(gui::g_state, &over));  // term count > 8 -> false
        IM_CHECK_EQ(over.filter_count, 0);
        IM_CHECK_EQ(over.composition_count, 0);
      }

      // Overflow (code-review-01 Major 2): the cross-factor Cartesian is capped
      // BEFORE materialization. Multiple multi-alternative factors whose product
      // exceeds LUMICE_MAX_CONFIG_CLAUSES must be rejected gracefully without ever
      // building the (potentially exponential) clause tree.
      {
        ResetTestState();
        gui::g_state.crystals.clear();
        gui::g_state.filters.clear();
        gui::g_state.layers.clear();
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::g_state.crystals.push_back(c);
        // Three raypath factors ANDed, each carrying 4 ';'-OR alternatives:
        // 4 * 4 * 4 = 64 would-be clauses >> LUMICE_MAX_CONFIG_CLAUSES(16), but only
        // 3 factors (<= term cap) — so this exercises the clause-product path, not
        // the term-count path.
        gui::SummandText row;
        row.text = "1;2;3;4 & 1;2;3;4 & 1;2;3;4";
        row.factors = {
          gui::Factor{ gui::RaypathParams{ "1;2;3;4" } },
          gui::Factor{ gui::RaypathParams{ "1;2;3;4" } },
          gui::Factor{ gui::RaypathParams{ "1;2;3;4" } },
        };
        gui::FilterConfig f;
        f.param = gui::SumOfProducts{ row };
        gui::g_state.filters.push_back(f);
        gui::Layer layer;
        layer.probability = 1.0f;
        gui::EntryCard e;
        e.crystal_id = 0;
        e.filter_id = 0;
        e.proportion = 100.0f;
        layer.entries.push_back(e);
        gui::g_state.layers.push_back(layer);
        LUMICE_Config over{};
        IM_CHECK(!gui::FillLumiceConfig(gui::g_state, &over));  // Cartesian > 16 clauses -> false
        IM_CHECK_EQ(over.filter_count, 0);
        IM_CHECK_EQ(over.composition_count, 0);

        // JSON-twin overflow behavior (code-review-02 Minor 2): SerializeFilterForCore is a
        // total function, so it degrades the overflowing filter to a BOUNDED match-all
        // stand-in (not the exponential tree, not a crash) — parses back as a single simple
        // filter with no complex. The production export path (DoExportConfigJson) rejects
        // overflow upstream via FillLumiceConfig before this is ever written (Major 1).
        std::string core_json = gui::SerializeCoreConfig(gui::g_state);
        IM_CHECK(!core_json.empty());
        LUMICE_Config from_json{};
        IM_CHECK_EQ(LUMICE_ParseConfigString(core_json.c_str(), &from_json), LUMICE_OK);
        IM_CHECK_EQ(from_json.filter_count, 1);       // bounded match-all stand-in, not 64 clauses
        IM_CHECK_EQ(from_json.composition_count, 0);  // no complex emitted for the stand-in
      }
    };
  }

  // Export overflow rejection (code-review-03 Major 1): the pure BuildExportJsonOrWarn — the
  // logic DoExportConfigJson delegates to — must REFUSE to produce a config for an over-limit
  // filter (never silently write a semantically-opposite match-all export) and hand back a
  // locator-bearing warning. Pins the critical fix into the regression gate rather than
  // relying on a one-time code read.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "export_json_rejects_overflow_filter");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();
      gui::g_state.crystals.clear();
      gui::g_state.filters.clear();
      gui::g_state.layers.clear();
      gui::CrystalConfig c;
      c.type = gui::CrystalType::kPrism;
      c.height = 1.0f;
      for (int i = 0; i < 6; ++i) {
        c.face_distance[i] = 1.0f;
      }
      gui::g_state.crystals.push_back(c);
      // 3 raypath factors x 4 alternatives = 64 clauses >> LUMICE_MAX_CONFIG_CLAUSES.
      gui::SummandText row;
      row.text = "1;2;3;4 & 1;2;3;4 & 1;2;3;4";
      row.factors = {
        gui::Factor{ gui::RaypathParams{ "1;2;3;4" } },
        gui::Factor{ gui::RaypathParams{ "1;2;3;4" } },
        gui::Factor{ gui::RaypathParams{ "1;2;3;4" } },
      };
      gui::FilterConfig f;
      f.name = "BigFilter";
      f.param = gui::SumOfProducts{ row };
      gui::g_state.filters.push_back(f);
      gui::Layer layer;
      layer.probability = 1.0f;
      gui::EntryCard e;
      e.crystal_id = 0;
      e.filter_id = 0;
      e.proportion = 100.0f;
      layer.entries.push_back(e);
      gui::g_state.layers.push_back(layer);

      // Overflow → refuse: no JSON produced, warning names the offending filter.
      std::string json;
      std::string warning;
      IM_CHECK(!gui::BuildExportJsonOrWarn(gui::g_state, &json, &warning));
      IM_CHECK(json.empty());  // out_json left untouched — nothing to write
      IM_CHECK(!warning.empty());
      IM_CHECK(warning.find("BigFilter") != std::string::npos);  // FormatOverflowLocator names it

      // Sanity: a valid (degenerate) filter exports fine — no warning, JSON produced.
      gui::g_state.filters[0].SetRaypath(gui::RaypathParams{ "3-5" });
      std::string ok_json;
      std::string ok_warning;
      IM_CHECK(gui::BuildExportJsonOrWarn(gui::g_state, &ok_json, &ok_warning));
      IM_CHECK(!ok_json.empty());
      IM_CHECK(ok_warning.empty());
    };
  }

  // FormatOverflowLocator format contract (pure function, no GUI): the unnamed form drops the
  // filter clause and keeps 1-based Layer/Entry, and the indices are presented as +1.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "overflow_locator_format");
    t->TestFunc = [](ImGuiTestContext*) {
      gui::FilterOverflowInfo named;
      named.layer_index = 2;
      named.entry_index = 0;
      named.filter_name = "MyFilter";
      IM_CHECK_STR_EQ(gui::FormatOverflowLocator(named).c_str(), "filter \"MyFilter\", Layer 3 / Entry 1");
      gui::FilterOverflowInfo unnamed;
      unnamed.layer_index = 0;
      unnamed.entry_index = 4;
      // Empty filter_name -> no "filter \"...\"," clause, just the position.
      IM_CHECK_STR_EQ(gui::FormatOverflowLocator(unnamed).c_str(), "Layer 1 / Entry 5");
    };
  }

  // DoRun's Log-panel dedup gate (`PeekGuiWarning() != warning_msg`): a persistent overflow
  // re-detected on every ~70ms debounce commit writes the Log line ONLY on the first detection
  // (message unchanged -> gate closed), and re-opens (gate open) when the located filter/layer
  // changes or after a successful commit clears the warning. This exercises the exact predicate
  // DoRun uses so AC② (Log dedup) has automated coverage rather than only on-screen verification.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "overflow_log_dedup_gate");
    t->TestFunc = [](ImGuiTestContext*) {
      gui::ClearGuiWarning();
      const std::string msg_a = "This filter has too many OR segments / values to apply (limit " +
                                std::to_string(LUMICE_MAX_CONFIG_CLAUSES) +
                                "; filter \"A\", Layer 1 / Entry 1).\nkept.";
      // First detection: gate open -> would write the Log line.
      IM_CHECK(gui::PeekGuiWarning() != msg_a);
      gui::SetGuiWarning(msg_a);
      // Same overflow re-detected on the next debounce commit: gate closed -> Log NOT re-written.
      IM_CHECK(gui::PeekGuiWarning() == msg_a);
      // User switches which filter overflows (new locator): gate open again -> re-logs.
      const std::string msg_b = "This filter has too many OR segments / values to apply (limit " +
                                std::to_string(LUMICE_MAX_CONFIG_CLAUSES) +
                                "; filter \"B\", Layer 2 / Entry 1).\nkept.";
      IM_CHECK(gui::PeekGuiWarning() != msg_b);
      gui::SetGuiWarning(msg_b);
      // A successful commit clears the warning: the same message would log again afterwards.
      gui::ClearGuiWarning();
      IM_CHECK(gui::PeekGuiWarning() != msg_a);
      gui::ClearGuiWarning();
    };
  }

  // 327.4 anti-spam: SetGuiWarning is idempotent while the same message is in-flight, so an
  // over-bounds filter re-detected on every debounced commit does NOT re-open the modal and
  // freeze interaction. ClearGuiWarning (a successful commit) re-arms it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "gui_warning_dedup");
    t->TestFunc = [](ImGuiTestContext*) {
      gui::ClearGuiWarning();
      IM_CHECK(gui::PeekGuiWarning().empty());
      gui::SetGuiWarning("over-bounds A");
      IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), "over-bounds A");
      gui::SetGuiWarning("over-bounds A");  // same message -> idempotent, no re-open
      IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), "over-bounds A");
      gui::SetGuiWarning("over-bounds B");  // different -> updates (would re-open)
      IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), "over-bounds B");
      gui::ClearGuiWarning();  // successful commit re-arms
      IM_CHECK(gui::PeekGuiWarning().empty());
      gui::SetGuiWarning("over-bounds A");  // same message after a clear -> warns again
      IM_CHECK_STR_EQ(gui::PeekGuiWarning().c_str(), "over-bounds A");
      gui::ClearGuiWarning();
    };
  }
}
