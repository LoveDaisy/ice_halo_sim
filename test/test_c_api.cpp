#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include "include/lumice.h"

TEST(CrystalMeshApi, PrismVerticesAndEdges) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "prism", "shape": {"height": 1.0}})";
  LUMICE_ErrorCode err = LUMICE_GetCrystalMesh(nullptr, json, &mesh);
  EXPECT_EQ(err, LUMICE_OK);
  EXPECT_EQ(mesh.vertex_count, 12);
  // 18 wireframe edges: 6 top + 6 bottom + 6 vertical (internal diagonals excluded)
  EXPECT_EQ(mesh.edge_count, 18);
}

TEST(CrystalMeshApi, PyramidVerticesAndEdges) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "pyramid", "shape": {"prism_h": 1.0, "upper_h": 0.5, "lower_h": 0.5}})";
  LUMICE_ErrorCode err = LUMICE_GetCrystalMesh(nullptr, json, &mesh);
  EXPECT_EQ(err, LUMICE_OK);
  EXPECT_GT(mesh.vertex_count, 0);
  EXPECT_GT(mesh.edge_count, 0);
  // Pyramid has more vertices and edges than a prism
  EXPECT_GT(mesh.vertex_count, 12);
}

TEST(CrystalMeshApi, NullArgs) {
  LUMICE_CrystalMesh mesh{};
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, nullptr, &mesh), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, "{}", nullptr), LUMICE_ERR_NULL_ARG);
}

TEST(CrystalMeshApi, InvalidJson) {
  LUMICE_CrystalMesh mesh{};
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, "not json", &mesh), LUMICE_ERR_INVALID_JSON);
}

TEST(CrystalMeshApi, MissingFields) {
  LUMICE_CrystalMesh mesh{};
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, R"({"type": "prism"})", &mesh), LUMICE_ERR_MISSING_FIELD);
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, R"({"shape": {}})", &mesh), LUMICE_ERR_MISSING_FIELD);
}

TEST(CrystalMeshApi, UnknownType) {
  LUMICE_CrystalMesh mesh{};
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, R"({"type": "cube", "shape": {}})", &mesh), LUMICE_ERR_INVALID_VALUE);
}


// =============== ParseConfigApi Tests ===============

// Helper: build a minimal valid JSON (ConfigToJson format) for testing.
static std::string MakeMinimalConfigJson() {
  nlohmann::json root;

  // One prism crystal
  nlohmann::json cr;
  cr["id"] = 1;
  cr["type"] = "prism";
  cr["shape"]["height"] = 1.5f;
  cr["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
  cr["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  root["crystal"] = nlohmann::json::array({ cr });

  // Scene
  nlohmann::json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = 20.0f;
  scene["light_source"]["azimuth"] = 0.0f;
  scene["light_source"]["diameter"] = 0.5f;
  scene["light_source"]["spectrum"] = "D65";
  scene["ray_num"] = 1000000ul;
  scene["max_hits"] = 8;
  scene["scattering"] = nlohmann::json::array();
  root["scene"] = scene;

  // One renderer
  nlohmann::json rn;
  rn["id"] = 1;
  rn["lens"]["type"] = "dual_fisheye_equal_area";
  rn["lens"]["fov"] = 180.0f;
  rn["resolution"] = { 800, 400 };
  rn["view"]["elevation"] = 0.0f;
  rn["view"]["azimuth"] = 0.0f;
  rn["view"]["roll"] = 0.0f;
  rn["visible"] = "full";
  rn["background"] = { 0.0f, 0.0f, 0.0f };
  rn["opacity"] = 1.0f;
  rn["intensity_factor"] = 1.0f;
  rn["norm_mode"] = 0;
  root["render"] = nlohmann::json::array({ rn });

  root["filter"] = nlohmann::json::array();
  return root.dump();
}

// Helper: build a full config JSON with pyramid, filters, scattering.
static std::string MakeFullConfigJson() {
  nlohmann::json root;

  // Prism crystal
  nlohmann::json cr1;
  cr1["id"] = 1;
  cr1["type"] = "prism";
  cr1["shape"]["height"] = 1.3f;
  cr1["shape"]["face_distance"] = { 1.0f, 0.8f, 1.0f, 0.8f, 1.0f, 0.8f };
  cr1["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 5.0f } };
  cr1["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr1["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };

  // Pyramid crystal
  nlohmann::json cr2;
  cr2["id"] = 2;
  cr2["type"] = "pyramid";
  cr2["shape"]["prism_h"] = 1.0f;
  cr2["shape"]["upper_h"] = 0.5f;
  cr2["shape"]["lower_h"] = 0.5f;
  cr2["shape"]["upper_indices"] = { 1, 0, 1 };
  cr2["shape"]["lower_indices"] = { 1, 0, 1 };
  cr2["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 15.0f } };
  cr2["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr2["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  root["crystal"] = nlohmann::json::array({ cr1, cr2 });

  // Filter
  nlohmann::json flt;
  flt["id"] = 1;
  flt["type"] = "raypath";
  flt["action"] = "filter_in";
  flt["raypath"] = { 3, 5 };
  flt["symmetry"] = "PB";
  root["filter"] = nlohmann::json::array({ flt });

  // Scene with scattering
  nlohmann::json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = 15.0f;
  scene["light_source"]["azimuth"] = 0.0f;
  scene["light_source"]["diameter"] = 0.5f;
  scene["light_source"]["spectrum"] = "D50";
  scene["ray_num"] = "infinite";
  scene["max_hits"] = 8;

  nlohmann::json entry;
  entry["crystal"] = 1;
  entry["proportion"] = 0.7f;
  entry["filter"] = 1;
  nlohmann::json entry2;
  entry2["crystal"] = 2;
  entry2["proportion"] = 0.3f;
  nlohmann::json layer;
  layer["prob"] = 1.0f;
  layer["entries"] = nlohmann::json::array({ entry, entry2 });
  scene["scattering"] = nlohmann::json::array({ layer });
  root["scene"] = scene;

  // Renderer
  nlohmann::json rn;
  rn["id"] = 1;
  rn["lens"]["type"] = "dual_fisheye_equal_area";
  rn["resolution"] = { 1024, 512 };
  rn["opacity"] = 0.9f;
  rn["intensity_factor"] = 2.0f;
  rn["norm_mode"] = 1;
  root["render"] = nlohmann::json::array({ rn });

  return root.dump();
}


TEST(ParseConfigApi, MinimalPrismConfig) {
  auto json = MakeMinimalConfigJson();
  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &config), LUMICE_OK);

  EXPECT_EQ(config.crystal_count, 1);
  EXPECT_EQ(config.crystals[0].id, 1);
  EXPECT_EQ(config.crystals[0].type, 0);  // prism
  EXPECT_FLOAT_EQ(config.crystals[0].height, 1.5f);

  // Default face_distance = all 1.0
  for (int k = 0; k < 6; k++) {
    EXPECT_FLOAT_EQ(config.crystals[0].face_distance[k], 1.0f);
  }

  // Axis
  EXPECT_EQ(config.crystals[0].zenith.type, 0);  // gauss
  EXPECT_FLOAT_EQ(config.crystals[0].zenith.mean, 90.0f);
  EXPECT_FLOAT_EQ(config.crystals[0].zenith.std, 10.0f);

  // Scene
  EXPECT_FLOAT_EQ(config.sun_altitude, 20.0f);
  EXPECT_STREQ(config.spectrum, "D65");
  EXPECT_EQ(config.infinite, 0);
  EXPECT_EQ(config.ray_num, 1000000ul);
  EXPECT_EQ(config.max_hits, 8);

  // Renderer
  EXPECT_EQ(config.renderer_count, 1);
  EXPECT_EQ(config.renderers[0].resolution_w, 800);
  EXPECT_EQ(config.renderers[0].resolution_h, 400);
  EXPECT_FLOAT_EQ(config.renderers[0].opacity, 1.0f);
}


TEST(ParseConfigApi, FullConfigWithPyramidAndFilter) {
  auto json = MakeFullConfigJson();
  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &config), LUMICE_OK);

  // Two crystals
  EXPECT_EQ(config.crystal_count, 2);

  // Prism with custom face_distance
  EXPECT_EQ(config.crystals[0].type, 0);
  EXPECT_FLOAT_EQ(config.crystals[0].face_distance[1], 0.8f);
  EXPECT_FLOAT_EQ(config.crystals[0].face_distance[2], 1.0f);

  // Pyramid
  EXPECT_EQ(config.crystals[1].type, 1);
  EXPECT_FLOAT_EQ(config.crystals[1].prism_h, 1.0f);
  EXPECT_FLOAT_EQ(config.crystals[1].upper_h, 0.5f);
  EXPECT_EQ(config.crystals[1].upper_indices[0], 1);

  // Filter
  EXPECT_EQ(config.filter_count, 1);
  EXPECT_EQ(config.filters[0].action, 0);  // filter_in
  EXPECT_EQ(config.filters[0].raypath_count, 2);
  EXPECT_EQ(config.filters[0].raypath[0], 3);
  EXPECT_EQ(config.filters[0].raypath[1], 5);
  EXPECT_EQ(config.filters[0].symmetry, 3);  // P=1 | B=2 = 3

  // Scene
  EXPECT_STREQ(config.spectrum, "D50");
  EXPECT_EQ(config.infinite, 1);

  // Scattering
  EXPECT_EQ(config.scatter_count, 1);
  EXPECT_FLOAT_EQ(config.scattering[0].probability, 1.0f);
  EXPECT_EQ(config.scattering[0].entry_count, 2);
  EXPECT_EQ(config.scattering[0].entries[0].crystal_id, 1);
  EXPECT_FLOAT_EQ(config.scattering[0].entries[0].proportion, 0.7f);
  EXPECT_EQ(config.scattering[0].entries[0].filter_id, 1);
  EXPECT_EQ(config.scattering[0].entries[1].filter_id, -1);  // no filter
}


TEST(ParseConfigApi, ParseModifyCommit) {
  auto json = MakeMinimalConfigJson();
  LUMICE_Config config{};
  ASSERT_EQ(LUMICE_ParseConfigString(json.c_str(), &config), LUMICE_OK);

  // Modify ray_num
  config.ray_num = 5000000;
  config.infinite = 0;

  // Commit to a real server
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);

  int reused = -1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(server, &config, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0);  // First commit, not reused

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}


TEST(ParseConfigApi, ParseConfigFile) {
  auto json = MakeMinimalConfigJson();

  // Write to temp file (cross-platform: use std::filesystem::temp_directory_path)
  auto tmp_path = std::filesystem::temp_directory_path() / "lumice_test_config.json";
  {
    std::ofstream f(tmp_path);
    f << json;
  }

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigFile(tmp_path.u8string().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.crystal_count, 1);
  EXPECT_FLOAT_EQ(config.crystals[0].height, 1.5f);

  std::filesystem::remove(tmp_path);
}


TEST(ParseConfigApi, NullArgs) {
  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(nullptr, &config), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ParseConfigString("{}", nullptr), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ParseConfigFile(nullptr, &config), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ParseConfigFile("/tmp/test.json", nullptr), LUMICE_ERR_NULL_ARG);
}


TEST(ParseConfigApi, InvalidJson) {
  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString("not json at all", &config), LUMICE_ERR_INVALID_JSON);
  EXPECT_EQ(LUMICE_ParseConfigString("{invalid", &config), LUMICE_ERR_INVALID_JSON);
}


TEST(ParseConfigApi, MissingCrystalSection) {
  LUMICE_Config config{};
  // Valid JSON but missing "crystal" key
  EXPECT_EQ(LUMICE_ParseConfigString(R"({"scene": {}})", &config), LUMICE_ERR_MISSING_FIELD);
}


TEST(ParseConfigApi, FileNotFound) {
  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigFile("/tmp/nonexistent_lumice_config_12345.json", &config), LUMICE_ERR_FILE_NOT_FOUND);
}


TEST(ParseConfigApi, UnsupportedFilterType) {
  nlohmann::json root;
  root["crystal"] = nlohmann::json::array();
  root["scene"] = { { "ray_num", 1000 } };
  root["filter"] = nlohmann::json::array({ { { "id", 1 }, { "type", "direction" }, { "action", "filter_in" } } });

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}


TEST(ParseConfigApi, ArraySpectrumNotSupported) {
  nlohmann::json root;
  nlohmann::json cr;
  cr["id"] = 1;
  cr["type"] = "prism";
  cr["shape"]["height"] = 1.0f;
  cr["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
  cr["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  root["crystal"] = nlohmann::json::array({ cr });
  root["scene"]["light_source"]["spectrum"] = nlohmann::json::array({ { { "wavelength", 550 }, { "weight", 1.0 } } });
  root["scene"]["ray_num"] = 1000;

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}


TEST(ParseConfigApi, SpectrumEnumerations) {
  // Test all supported spectrum strings
  for (const char* sp : { "D65", "D50", "A", "E" }) {
    auto json_str = MakeMinimalConfigJson();
    auto root = nlohmann::json::parse(json_str);
    root["scene"]["light_source"]["spectrum"] = sp;

    LUMICE_Config config{};
    ASSERT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_OK) << "Failed for spectrum: " << sp;
    EXPECT_STREQ(config.spectrum, sp);
  }

  // Unknown spectrum string
  auto json_str = MakeMinimalConfigJson();
  auto root = nlohmann::json::parse(json_str);
  root["scene"]["light_source"]["spectrum"] = "UnknownIlluminant";

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}
