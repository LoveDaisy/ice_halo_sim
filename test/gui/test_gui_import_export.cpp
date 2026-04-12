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
      IM_CHECK_EQ(static_cast<int>(loaded.crystals.size()), static_cast<int>(gui::g_state.crystals.size()));
      IM_CHECK_EQ(static_cast<int>(loaded.filters.size()), static_cast<int>(gui::g_state.filters.size()));
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

  // Test 3: Equirect PNG export — write synthetic data, verify file is readable
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
