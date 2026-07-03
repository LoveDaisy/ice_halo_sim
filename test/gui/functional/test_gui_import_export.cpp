#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>

#include "IconsFontAwesome6.h"
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
      LUMICE_Config cfg;
      gui::FillLumiceConfig(gui::g_state, &cfg);
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
      f.param = gui::RaypathParams{ "3-1-5" };
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
      f.param = gui::RaypathParams{ "3-5; 1-3" };
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
      f.param = gui::RaypathParams{ "3-1-5" };
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

      auto make_entry = [](gui::FilterParamVariant param) {
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
        f.param = std::move(param);
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
      IM_CHECK(std::holds_alternative<gui::RaypathParams>(f0.param));
      IM_CHECK_EQ(std::get<gui::RaypathParams>(f0.param).raypath_text, std::string("3-1-5"));

      const auto& f1 = gui::g_state.filters[*gui::g_state.layers[0].entries[1].filter_id];
      IM_CHECK(std::holds_alternative<gui::EntryExitParams>(f1.param));
      const auto& ee = std::get<gui::EntryExitParams>(f1.param);
      IM_CHECK_EQ(ee.entry_text, std::string("7"));
      IM_CHECK_EQ(ee.exit_text, std::string("4"));

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
      IM_CHECK(loaded.layers[0].entries[0].filter_id.has_value());
      const auto& f = loaded.filters[*loaded.layers[0].entries[0].filter_id];
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
      fc.param = gui::EntryExitParams{ /*entry_text=*/"2", /*exit_text=*/"5" };
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
      IM_CHECK(std::holds_alternative<gui::RaypathParams>(f.param));
      IM_CHECK(std::get<gui::RaypathParams>(f.param).raypath_text.empty());
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
        fc.param = ee;
        gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], fc);
      }
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      ctx->ItemClick("**/Entry-Exit##filter_type");
      ctx->Yield(2);

      ctx->ItemClick("**/Remove Filter##filter_ee");
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

  // task-gui-complex-filter-import-roundtrip: degenerate multi-segment raypath
  // complex filter survives core-JSON round-trip. GUI emits N simple raypaths
  // + 1 complex referencing them on serialize; import must reverse-map back to
  // a multi-segment RaypathParams instead of dropping the complex (explore-271).
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
      f.param = gui::RaypathParams{ "3-5;1-3" };
      gui::SetFilter(gui::g_state, entry, f);
      layer.entries.push_back(entry);
      gui::g_state.layers.push_back(layer);

      const std::string core_json = gui::SerializeCoreConfig(gui::g_state);
      IM_CHECK(!core_json.empty());

      gui::GuiState loaded = gui::InitDefaultState();
      bool ok = gui::DeserializeFromJson(core_json, loaded);
      IM_CHECK(ok);

      // The scattering entry must reference a filter slot in the loaded state's
      // filter pool, and that filter must be a multi-segment RaypathParams.
      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(loaded.layers[0].entries.size()), 1);
      const auto& loaded_entry = loaded.layers[0].entries[0];
      IM_CHECK(loaded_entry.filter_id.has_value());
      const auto& loaded_filter = loaded.filters[*loaded_entry.filter_id];
      IM_CHECK(loaded_filter.IsRaypath());
      IM_CHECK_STR_EQ(loaded_filter.RaypathText().c_str(), "3-5;1-3");
      // No warning should have fired for a well-formed GUI-emitted complex.
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
    };
  }

  // task-gui-complex-filter-import-roundtrip: degenerate EE multi-value
  // (cartesian product) complex filter survives round-trip. entries × exits
  // factorize back to entry_text / exit_text.
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
      f.param = ee;
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
      IM_CHECK(std::holds_alternative<gui::EntryExitParams>(loaded_filter.param));
      const auto& p = std::get<gui::EntryExitParams>(loaded_filter.param);
      // Order is sorted by reconstruct; both ASCII "3" < "4" and "5" < "6".
      IM_CHECK_STR_EQ(p.entry_text.c_str(), "3,4");
      IM_CHECK_STR_EQ(p.exit_text.c_str(), "5,6");
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
    };
  }

  // task-gui-complex-filter-import-roundtrip (code-review Major): a wildcard
  // entry (empty text) crossed with multiple exits still serializes to a
  // complex filter (pair_count = 1 x N > 1). The reconstruct path must decode
  // the absent "entry" field back to an empty (wildcard) string — exercising
  // DecodeEEFaceFromJson's absent-field branch, which Test B's all-specific
  // faces did not cover.
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
      f.param = ee;
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
      IM_CHECK(std::holds_alternative<gui::EntryExitParams>(loaded_filter.param));
      const auto& p = std::get<gui::EntryExitParams>(loaded_filter.param);
      IM_CHECK_STR_EQ(p.entry_text.c_str(), "");  // wildcard preserved
      IM_CHECK_STR_EQ(p.exit_text.c_str(), "5,6");
      IM_CHECK(gui::PeekImportComplexFilterWarning().empty());
    };
  }

  // task-gui-complex-filter-import-roundtrip: a true AND-of-products complex
  // filter (composition entry with >1 child id, expressing AND semantics) is
  // not representable in the GUI's flat FilterConfig — import must drop it,
  // leave the referencing scattering entry without a filter, AND queue a
  // user-visible warning via SetImportComplexFilterWarning (no silent miscull).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "non_degenerate_complex_ignored_but_warned");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();
      gui::ClearImportComplexFilterWarning();

      // Hand-authored core JSON: 2 raypath simples + 1 complex with an
      // AND-of-products composition ([[1,2]] — id 1 AND id 2 in the same
      // product), which the GUI cannot represent.
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
      // Filter id=3 is the complex — it must not be materialized; entry's
      // filter_id must be unset (no silent miscull on the rendered image).
      IM_CHECK(!loaded_entry.filter_id.has_value());
      // The loud warning must have fired so the user sees the modal.
      const std::string warning = gui::PeekImportComplexFilterWarning();
      IM_CHECK(!warning.empty());
      gui::ClearImportComplexFilterWarning();
    };
  }
}
