#include <cstdio>
#include <fstream>

#include "test_gui_shared.hpp"

// ========== Import/Export Tests ==========

void RegisterImportExportTests(ImGuiTestEngine* engine) {
  // Test 1: JSON round-trip — serialize then deserialize, verify key fields match
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

      // Verify key fields survived round-trip
      IM_CHECK(std::abs(loaded.sim.ray_num_millions - gui::g_state.sim.ray_num_millions) < 0.01f);
      IM_CHECK_EQ(static_cast<int>(loaded.layers.size()), static_cast<int>(gui::g_state.layers.size()));
      IM_CHECK_EQ(static_cast<int>(loaded.renderers.size()), static_cast<int>(gui::g_state.renderers.size()));
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

      // Old format: crystals[] + scattering[] with ID references
      std::string json = R"({
        "crystals": [
          {"id": 1, "type": "Prism", "height": 2.0},
          {"id": 2, "type": "Pyramid", "height": 1.5, "upper_h": 0.3, "lower_h": 0.4, "prism_h": 1.0}
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
      entry0.proportion = 75.0f;
      gui::FilterConfig f;
      f.raypath_text = "3-1-5";
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
      IM_CHECK_EQ(loaded0.proportion, 75.0f);
      IM_CHECK(loaded0.filter.has_value());

      auto& loaded1 = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(loaded1.crystal.type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(loaded1.crystal.height, 3.0f);
      IM_CHECK_EQ(loaded1.proportion, 25.0f);

      std::remove(tmp_path);
    };
  }

  // Test 5: Equirect PNG export — write synthetic data, verify file is readable
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "equirect_export");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Create synthetic 4x2 RGB image
      const int w = 4;
      const int h = 2;
      std::vector<unsigned char> data(w * h * 3, 128);

      const char* tmp_path = "/tmp/lumice_equirect_test.png";
      bool ok = gui::ExportEquirectPng(tmp_path, data.data(), w, h);
      IM_CHECK(ok);

      // Verify file is loadable
      std::vector<unsigned char> img_data;
      int img_w = 0;
      int img_h = 0;
      int img_ch = 0;
      bool loaded = lumice::test::LoadPng(tmp_path, img_data, img_w, img_h, img_ch);
      IM_CHECK(loaded);
      IM_CHECK_EQ(img_w, w);
      IM_CHECK_EQ(img_h, h);

      std::remove(tmp_path);
    };
  }
}
