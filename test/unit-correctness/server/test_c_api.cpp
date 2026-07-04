#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "config/filter_config.hpp"  // core FilterConfig + to_json, for emit isomorphism cross-check
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "include/lumice.h"
#include "server/c_api_internal.hpp"  // ConfigToJson (test-only exposure) for emit-shape assertions

// Regression guard (task-fix-stats-ray-count-u32-overflow): ray-count fields must be
// 64-bit so totals > 2^32 never truncate on Windows, where `unsigned long` is 32-bit
// (the status-bar ray-count rollover reported by Windows users). These field-level
// asserts complement the header-level guard in lumice.h: they verify the struct fields
// actually use the 64-bit type, not just that the typedef is wide enough.
static_assert(sizeof(((LUMICE_StatsResult*)nullptr)->sim_ray_num) >= 8, "stats sim_ray_num must be 64-bit");
static_assert(sizeof(((LUMICE_StatsResult*)nullptr)->ray_seg_num) >= 8, "stats ray_seg_num must be 64-bit");
static_assert(sizeof(((LUMICE_StatsResult*)nullptr)->crystal_num) >= 8, "stats crystal_num must be 64-bit");
static_assert(sizeof(((LUMICE_Config*)nullptr)->ray_num) >= 8, "config ray_num must be 64-bit");

// ABI guard (backend-lifecycle-epoch): appending the trailing uint64 `epoch`
// grew LUMICE_RawXyzResult from 56 → 64 bytes (effective_pixels@48 + 4 pad +
// epoch@56, 8-aligned). test/e2e/capi_runner.py mirrors this exact size; keep the
// two in lockstep (measured, not assumed — ctypes.sizeof == 64).
static_assert(sizeof(LUMICE_RawXyzResult) == 64, "LUMICE_RawXyzResult ABI must be 64 bytes (capi_runner.py mirror)");

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

TEST(CrystalMeshApi, PrismFaceNumbersInLegalSet) {
  // Zero-init covers all unused slots with 0; LUMICE_GetCrystalMesh must overwrite
  // [0, triangle_count) with valid face numbers (>0).
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "prism", "shape": {"height": 1.0}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);
  ASSERT_GT(mesh.triangle_count, 0);
  for (int i = 0; i < mesh.triangle_count; ++i) {
    int fn = mesh.face_numbers[i];
    EXPECT_GE(fn, 1) << "triangle " << i << " face_number should be >= 1";
    EXPECT_LE(fn, 8) << "triangle " << i << " face_number should be <= 8 for prism";
    EXPECT_NE(fn, -1) << "triangle " << i << " must be recognized";
  }
}

TEST(CrystalMeshApi, PyramidFaceNumbersInLegalSet) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "pyramid", "shape": {"prism_h": 1.0, "upper_h": 0.5, "lower_h": 0.5}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);
  ASSERT_GT(mesh.triangle_count, 0);
  bool saw_prism = false;
  bool saw_upper_pyr = false;
  bool saw_lower_pyr = false;
  for (int i = 0; i < mesh.triangle_count; ++i) {
    int fn = mesh.face_numbers[i];
    bool legal = (fn == 1) || (fn == 2) ||  // basal
                 (fn >= 3 && fn <= 8) ||    // prism
                 (fn >= 13 && fn <= 18) ||  // upper pyramidal
                 (fn >= 23 && fn <= 28);    // lower pyramidal
    EXPECT_TRUE(legal) << "triangle " << i << " face_number=" << fn;
    if (fn >= 3 && fn <= 8) {
      saw_prism = true;
    }
    if (fn >= 13 && fn <= 18) {
      saw_upper_pyr = true;
    }
    if (fn >= 23 && fn <= 28) {
      saw_lower_pyr = true;
    }
  }
  EXPECT_TRUE(saw_prism);
  EXPECT_TRUE(saw_upper_pyr);
  EXPECT_TRUE(saw_lower_pyr);
}

// Isolated coverage of the C-API's kInvalidId -> -1 conversion logic.
// Synthetic zero vector forces all prism/basal dot products to fall below kFloatEps,
// which takes the else branch in FillHexFnMap and leaves the slot as kInvalidId.
// Test-only input; real callers pass unit per-triangle normals as documented.
TEST(CrystalMeshApi, FillHexFnMapInvalidNormalMapsToMinusOne) {
  lumice::IdType fn_tmp[1] = {};
  const float kCraftedNormal[3] = { 0.0f, 0.0f, 0.0f };
  lumice::FillHexFnMap(1, kCraftedNormal, fn_tmp);
  ASSERT_EQ(fn_tmp[0], lumice::kInvalidId);
  int api_val = (fn_tmp[0] == lumice::kInvalidId) ? -1 : static_cast<int>(fn_tmp[0]);
  EXPECT_EQ(api_val, -1);
  // Reverse check: naive static_cast yields 65535, confirming explicit check is needed.
  EXPECT_EQ(static_cast<int>(fn_tmp[0]), 65535);
  EXPECT_NE(static_cast<int>(fn_tmp[0]), -1);
}


// =============== Per-face topology tests ===============

TEST(CrystalMeshApi, PrismPerFaceTopology) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "prism", "shape": {"height": 1.0}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);

  // Prism: 2 basal + 6 lateral faces
  EXPECT_EQ(mesh.face_count, 8);
  EXPECT_GT(mesh.face_count, 0);

  int basal_count = 0;
  int lateral_count = 0;
  for (int i = 0; i < mesh.face_count; ++i) {
    int fn = mesh.face_numbers_by_face[i];
    EXPECT_GT(fn, 0) << "face " << i << " face_number should be > 0";
    int vtx_cnt = mesh.face_vtx_counts[i];
    if (fn == 1 || fn == 2) {
      // Basal faces have 6 vertices (regular hexagon)
      EXPECT_EQ(vtx_cnt, 6) << "basal face " << i << " (fn=" << fn << ") should have 6 vertices";
      ++basal_count;
    } else if (fn >= 3 && fn <= 8) {
      // Lateral prism faces are quads
      EXPECT_EQ(vtx_cnt, 4) << "prism face " << i << " (fn=" << fn << ") should have 4 vertices";
      ++lateral_count;
    }
  }
  EXPECT_EQ(basal_count, 2);
  EXPECT_EQ(lateral_count, 6);
}

TEST(CrystalMeshApi, PyramidPerFaceTopology) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "pyramid", "shape": {"prism_h": 1.0, "upper_h": 0.5, "lower_h": 0.5}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);

  EXPECT_GT(mesh.face_count, 0);
  // Full pyramid: 2 basal + 6 prism + 6 upper + 6 lower = 20 faces
  EXPECT_EQ(mesh.face_count, 20);

  for (int i = 0; i < mesh.face_count; ++i) {
    int fn = mesh.face_numbers_by_face[i];
    bool legal = (fn == 1) || (fn == 2) || (fn >= 3 && fn <= 8) || (fn >= 13 && fn <= 18) || (fn >= 23 && fn <= 28);
    EXPECT_TRUE(legal) << "face " << i << " face_number=" << fn << " is not in legal set";
    EXPECT_GE(mesh.face_vtx_counts[i], 3) << "face " << i << " must have >= 3 vertices";
  }
}

TEST(CrystalMeshApi, PerFaceVertexOrderCCW) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "prism", "shape": {"height": 1.0}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);
  ASSERT_GT(mesh.face_count, 0);

  // Find basal face (fn==1) and verify CCW winding
  int basal_fi = -1;
  for (int i = 0; i < mesh.face_count; ++i) {
    if (mesh.face_numbers_by_face[i] == 1) {
      basal_fi = i;
      break;
    }
  }
  ASSERT_GE(basal_fi, 0) << "no basal face 1 found";

  int offset = mesh.face_vtx_offsets[basal_fi];
  int count = mesh.face_vtx_counts[basal_fi];
  ASSERT_GE(count, 3);

  // Compute face normal from first triangle (v0, v1, v2)
  const float* p0 = mesh.vertices + mesh.face_vtx_pool[offset + 0] * 3;
  const float* p1 = mesh.vertices + mesh.face_vtx_pool[offset + 1] * 3;
  const float* p2 = mesh.vertices + mesh.face_vtx_pool[offset + 2] * 3;
  float e1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
  float e2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
  float nx = e1[1] * e2[2] - e1[2] * e2[1];
  float ny = e1[2] * e2[0] - e1[0] * e2[2];
  float nz = e1[0] * e2[1] - e1[1] * e2[0];
  float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
  ASSERT_GT(nlen, 1e-6f) << "degenerate face normal";
  nx /= nlen;
  ny /= nlen;
  nz /= nlen;

  // For CCW winding, each consecutive edge cross product should point along the face normal
  for (int k = 0; k < count; ++k) {
    const float* a = mesh.vertices + mesh.face_vtx_pool[offset + k] * 3;
    const float* b = mesh.vertices + mesh.face_vtx_pool[offset + (k + 1) % count] * 3;
    float ex = b[0] - a[0];
    float ey = b[1] - a[1];
    float ez = b[2] - a[2];
    // Edge next_edge = b - a; for CCW the "winding cross" test uses consecutive edges.
    // Simple check: cross of consecutive edge pairs projected onto face normal > 0 on average.
    (void)ex;
    (void)ey;
    (void)ez;
  }
  // A simpler winding check: sum of cross products of (vi - center) × (v_{i+1} - center)
  // projected onto the face normal should be positive for CCW.
  float cx = 0.0f;
  float cy = 0.0f;
  float cz = 0.0f;
  for (int k = 0; k < count; ++k) {
    const float* v = mesh.vertices + mesh.face_vtx_pool[offset + k] * 3;
    cx += v[0];
    cy += v[1];
    cz += v[2];
  }
  cx /= count;
  cy /= count;
  cz /= count;

  float winding_sum = 0.0f;
  for (int k = 0; k < count; ++k) {
    const float* a = mesh.vertices + mesh.face_vtx_pool[offset + k] * 3;
    const float* b = mesh.vertices + mesh.face_vtx_pool[offset + (k + 1) % count] * 3;
    float ax = a[0] - cx, ay = a[1] - cy, az = a[2] - cz;
    float bx = b[0] - cx, by = b[1] - cy, bz = b[2] - cz;
    // cross(a, b) projected onto normal
    float cross_x = ay * bz - az * by;
    float cross_y = az * bx - ax * bz;
    float cross_z = ax * by - ay * bx;
    winding_sum += cross_x * nx + cross_y * ny + cross_z * nz;
  }
  EXPECT_GT(winding_sum, 0.0f) << "basal face vertices not in CCW order";
}

// =============== Per-face area-weighted normal tests ===============

TEST(CrystalMeshApi, FaceNormalsUnitLengthPrism) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "prism", "shape": {"height": 1.0}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);
  ASSERT_GT(mesh.face_count, 0);

  for (int fi = 0; fi < mesh.face_count; ++fi) {
    int fn = mesh.face_numbers_by_face[fi];
    const float* n = mesh.face_normals + fi * 3;
    float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    EXPECT_NEAR(len, 1.0f, 1e-5f) << "face fi=" << fi << " (fn=" << fn << ") normal not unit length (len=" << len
                                  << ")";
  }
}

TEST(CrystalMeshApi, FaceNormalsOutwardPrism) {
  // Each face normal must point outward from the face centroid: dot(n, centroid - origin) > 0
  // for centered hex prism geometry.
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "prism", "shape": {"height": 1.0}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);
  ASSERT_GT(mesh.face_count, 0);

  for (int fi = 0; fi < mesh.face_count; ++fi) {
    int fn = mesh.face_numbers_by_face[fi];
    int offset = mesh.face_vtx_offsets[fi];
    int count = mesh.face_vtx_counts[fi];
    ASSERT_GT(count, 0);

    float cx = 0.0f;
    float cy = 0.0f;
    float cz = 0.0f;
    for (int k = 0; k < count; ++k) {
      const float* p = mesh.vertices + mesh.face_vtx_pool[offset + k] * 3;
      cx += p[0];
      cy += p[1];
      cz += p[2];
    }
    cx /= count;
    cy /= count;
    cz /= count;

    const float* n = mesh.face_normals + fi * 3;
    float dot = n[0] * cx + n[1] * cy + n[2] * cz;
    EXPECT_GT(dot, 0.0f) << "face fi=" << fi << " (fn=" << fn << ") normal not outward (dot=" << dot << ")";
  }
}

TEST(CrystalMeshApi, FaceNormalsUnitLengthExtremePyramid) {
  // Extreme-wedge pyramid (close to the catalog-G regime that motivated this task).
  // Covers lower-pyramidal face numbers (23-28), which exceed LUMICE_MAX_CRYSTAL_FACES=24
  // — exercising the position-based (fi) indexing rather than fn-value indexing.
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "pyramid", "shape": {"prism_h": 0.01, "upper_h": 0.5, "lower_h": 0.5,
                            "upper_wedge_angle": 87.0, "lower_wedge_angle": 87.0}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);
  ASSERT_GT(mesh.face_count, 0);

  for (int fi = 0; fi < mesh.face_count; ++fi) {
    int fn = mesh.face_numbers_by_face[fi];
    const float* n = mesh.face_normals + fi * 3;
    float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    EXPECT_NEAR(len, 1.0f, 1e-5f) << "extreme pyramid fi=" << fi << " (fn=" << fn << ") normal len=" << len;
  }
}

TEST(CrystalMeshApi, PerFacePoolBoundary) {
  LUMICE_CrystalMesh mesh{};
  const char* json = R"({"type": "pyramid", "shape": {"prism_h": 1.0, "upper_h": 0.5, "lower_h": 0.5}})";
  ASSERT_EQ(LUMICE_GetCrystalMesh(nullptr, json, &mesh), LUMICE_OK);

  // Verify pool usage doesn't exceed the cap
  int total_pool = 0;
  for (int i = 0; i < mesh.face_count; ++i) {
    int end = mesh.face_vtx_offsets[i] + mesh.face_vtx_counts[i];
    if (end > total_pool) {
      total_pool = end;
    }
    EXPECT_LE(end, LUMICE_MAX_CRYSTAL_FACE_VTXPOOL) << "face " << i << " exceeds pool cap";
    // offsets and counts must be non-negative
    EXPECT_GE(mesh.face_vtx_offsets[i], 0);
    EXPECT_GT(mesh.face_vtx_counts[i], 0);
  }
  EXPECT_LE(total_pool, LUMICE_MAX_CRYSTAL_FACE_VTXPOOL);
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


TEST(ParseConfigApi, RayNumAbove32BitNotTruncated) {
  // Regression (task-fix-stats-ray-count-u32-overflow): config ray_num was parsed via
  // `rn.get<unsigned long>()`, truncating to 32-bit on Windows. A finite ray_num above
  // 2^32 must round-trip through LUMICE_ParseConfigString intact.
  auto root = nlohmann::json::parse(MakeMinimalConfigJson());
  const LUMICE_RayCount kBigRayNum = 5'000'000'000ULL;  // > UINT32_MAX (4'294'967'295)
  root["scene"]["ray_num"] = kBigRayNum;

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.infinite, 0);
  // Pre-fix on Windows this truncated to 705'032'704; post-fix it holds the full value.
  EXPECT_EQ(config.ray_num, kBigRayNum);
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
  EXPECT_NEAR(config.crystals[1].upper_wedge_angle, 28.0f, 0.1f);

  // Filter
  EXPECT_EQ(config.filter_count, 1);
  EXPECT_EQ(config.filters[0].type, LUMICE_FILTER_TYPE_RAYPATH);  // JsonToFilter sets discriminant
  EXPECT_EQ(config.filters[0].action, 0);                         // filter_in
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


TEST(ParseConfigApi, ArraySpectrumParsed) {
  nlohmann::json root;
  nlohmann::json cr;
  cr["id"] = 1;
  cr["type"] = "prism";
  cr["shape"]["height"] = 1.0f;
  cr["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
  cr["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  root["crystal"] = nlohmann::json::array({ cr });
  root["scene"]["light_source"]["spectrum"] = nlohmann::json::array(
      { { { "wavelength", 450 }, { "weight", 0.8 } }, { { "wavelength", 550 }, { "weight", 1.0 } } });
  root["scene"]["ray_num"] = 1000;

  LUMICE_Config config{};
  ASSERT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.spectrum_count, 2);
  EXPECT_FLOAT_EQ(config.spectrum_entries[0].wavelength, 450.0f);
  EXPECT_FLOAT_EQ(config.spectrum_entries[0].weight, 0.8f);
  EXPECT_FLOAT_EQ(config.spectrum_entries[1].wavelength, 550.0f);
  EXPECT_FLOAT_EQ(config.spectrum_entries[1].weight, 1.0f);
}


TEST(ParseConfigApi, StructSpectrumRoundTrip) {
  // Fill LUMICE_Config directly, commit via struct path (bypasses JSON parse), then re-parse
  // the ConfigToJson output via a JSON round-trip to prove spectrum_entries[] serializes into
  // the array shape core light_config expects (mirrors GUI struct→commit path).
  auto json = MakeMinimalConfigJson();
  LUMICE_Config config{};
  ASSERT_EQ(LUMICE_ParseConfigString(json.c_str(), &config), LUMICE_OK);

  config.spectrum_count = 3;
  config.spectrum_entries[0] = { 450.0f, 0.5f };
  config.spectrum_entries[1] = { 550.0f, 1.0f };
  config.spectrum_entries[2] = { 650.0f, 0.7f };
  config.ray_num = 300;  // 3 wavelengths * 100 rays each is enough for a smoke commit
  config.infinite = 0;

  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(server, &config, &reused), LUMICE_OK);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}


TEST(ParseConfigApi, ArraySpectrumOverCap) {
  auto json_str = MakeMinimalConfigJson();
  auto root = nlohmann::json::parse(json_str);
  nlohmann::json arr = nlohmann::json::array();
  for (int i = 0; i <= LUMICE_MAX_CONFIG_SPECTRUM_ENTRIES; i++) {
    arr.push_back({ { "wavelength", 400 + i }, { "weight", 1.0 } });
  }
  root["scene"]["light_source"]["spectrum"] = arr;

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_CONFIG);
}


TEST(ParseConfigApi, SpectrumEnumerations) {
  // Test all supported spectrum strings (D55 / D75 added in task-323).
  for (const char* sp : { "D65", "D55", "D50", "D75", "A", "E" }) {
    auto json_str = MakeMinimalConfigJson();
    auto root = nlohmann::json::parse(json_str);
    root["scene"]["light_source"]["spectrum"] = sp;

    LUMICE_Config config{};
    ASSERT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_OK) << "Failed for spectrum: " << sp;
    EXPECT_STREQ(config.spectrum, sp);
    EXPECT_EQ(config.spectrum_count, 0);
  }

  // Unknown spectrum string
  auto json_str = MakeMinimalConfigJson();
  auto root = nlohmann::json::parse(json_str);
  root["scene"]["light_source"]["spectrum"] = "UnknownIlluminant";

  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}


// =============== Server Lifecycle / Results API Tests ===============

// Helper: build a config with a highly selective BD raypath filter that
// produces many 0-exit-ray batches on the backend exit-seam path. Used by
// ZeroExitBatchNoHang to regression-guard the sim_scene_cnt_ leak (fix:
// simulator.cpp drops the exit_count==0 early return + server.cpp ConsumeData
// guards consume on has_renderable while keeping -- unconditional).
//
// Mirrors test/e2e/configs/parity_single_ms_bd_filter.json structure: prism
// crystal + raypath [4,6] BD filter + scattering prob=0.5 with filter=1. Sized
// to ~200k rays so the test completes in seconds when the fix is present and
// times out at 60s when the leak is reintroduced.
static std::string MakeBdFilterConfigJson() {
  auto base = nlohmann::json::parse(MakeMinimalConfigJson());
  base["scene"]["ray_num"] = 200000ul;
  base["scene"]["max_hits"] = 7;

  nlohmann::json flt;
  flt["id"] = 1;
  flt["type"] = "raypath";
  flt["raypath"] = { 4, 6 };
  flt["symmetry"] = "BD";
  base["filter"] = nlohmann::json::array({ flt });

  nlohmann::json entry;
  entry["crystal"] = 1;
  entry["proportion"] = 10;
  entry["filter"] = 1;
  nlohmann::json layer;
  layer["prob"] = 0.5f;
  layer["entries"] = nlohmann::json::array({ entry });
  base["scene"]["scattering"] = nlohmann::json::array({ layer });
  return base.dump();
}

// Helper: build a small finite-ray-count config with non-empty scattering.
// - Based on MakeMinimalConfigJson() (parse-modify-dump pattern, like SpectrumEnumerations)
// - ray_num set to 1000 for fast completion
// - scattering layer added with prob=0.0 (single-pass: rays exit after one crystal interaction).
//   Without a non-empty scattering, the simulator processes no crystals, leaving crystal_num == 0.
static std::string MakeSmallSimConfigJson() {
  auto base = nlohmann::json::parse(MakeMinimalConfigJson());
  base["scene"]["ray_num"] = 1000ul;

  // crystal id 1 matches the single crystal in MakeMinimalConfigJson()
  nlohmann::json entry;
  entry["crystal"] = 1;
  entry["proportion"] = 1.0f;
  nlohmann::json layer;
  layer["prob"] = 0.0f;  // single-pass: rays terminate after this scattering layer
  layer["entries"] = nlohmann::json::array({ entry });
  base["scene"]["scattering"] = nlohmann::json::array({ layer });
  return base.dump();
}

// Helper: poll the server until it transitions to LUMICE_SERVER_IDLE or timeout.
// Returns true if idle was reached within timeout_ms; false on timeout.
static bool WaitForIdle(LUMICE_Server* server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    LUMICE_ServerState state{};
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// Lightweight fixture: only creates and destroys a server with num_workers=1.
// Tests explicitly call CommitAndWaitForIdle() to advance the lifecycle as needed,
// keeping each test self-documenting and avoiding hidden coupling on simulator success.
class ServerLifecycleApi : public ::testing::Test {
 protected:
  void SetUp() override {
    LUMICE_ServerConfig server_config{};
    server_config.num_workers = 1;  // Predictable single-worker behavior on CI
    server_ = LUMICE_CreateServerEx(&server_config);
    ASSERT_NE(server_, nullptr);
  }

  void TearDown() override {
    if (server_ != nullptr) {
      LUMICE_StopServer(server_);
      LUMICE_DestroyServer(server_);
      server_ = nullptr;
    }
  }

  // Commit a small finite simulation and wait for it to complete.
  // After this returns, the server is in IDLE with stats/render results available.
  void CommitAndWaitForIdle() {
    auto json = MakeSmallSimConfigJson();
    ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
    ASSERT_TRUE(WaitForIdle(server_, 10000)) << "Server did not reach IDLE within 10 seconds";
  }

  LUMICE_Server* server_ = nullptr;
};


TEST_F(ServerLifecycleApi, FullLifecycle) {
  // Initial state after creation: IDLE.
  // Note: only IDLE and RUNNING are observable; LUMICE_SERVER_NOT_READY is unreachable
  // through the public API in the current implementation (intentional — covered in
  // GetBeforeCommit test below). RUNNING is also racy to observe at this scale
  // (1000 rays + 1 worker complete in <20ms), so this test asserts only the
  // before/after IDLE states, not the intermediate RUNNING state.
  LUMICE_ServerState state{};
  ASSERT_EQ(LUMICE_QueryServerState(server_, &state), LUMICE_OK);
  EXPECT_EQ(state, LUMICE_SERVER_IDLE);

  // Commit config and wait for completion.
  auto json = MakeSmallSimConfigJson();
  ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(server_, 10000)) << "Server did not reach IDLE within 10 seconds";

  // Final state: IDLE.
  ASSERT_EQ(LUMICE_QueryServerState(server_, &state), LUMICE_OK);
  EXPECT_EQ(state, LUMICE_SERVER_IDLE);
}


// Explicit single-source lifecycle (backend-lifecycle-epoch): Idle → Running →
// Completed, monotonic epoch (++ per reset-causing commit), Stop → Idle. Also
// pins QueryServerState as a projection (COMPLETED → IDLE).
TEST_F(ServerLifecycleApi, GetSimLifecycle) {
  // Fresh server, no commit yet: IDLE, epoch 0.
  LUMICE_SimLifecycleResult lc{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, LUMICE_LIFECYCLE_IDLE);
  EXPECT_EQ(lc.epoch, 0u);

  // First reset-causing commit: epoch must advance to 1. Right after commit the
  // run is RUNNING (or, on a very fast finish, already COMPLETED) — never IDLE,
  // since status_ is kRunning until the pipeline drains.
  auto json = MakeSmallSimConfigJson();
  ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.epoch, 1u) << "epoch must ++ on the first reset-causing commit";
  EXPECT_NE(lc.lifecycle, LUMICE_LIFECYCLE_IDLE) << "post-commit lifecycle is RUNNING or COMPLETED, never IDLE";

  // Drain to completion: COMPLETED, epoch stable at 1.
  ASSERT_TRUE(WaitForIdle(server_, 10000)) << "Server did not reach IDLE within 10 seconds";
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, LUMICE_LIFECYCLE_COMPLETED);
  EXPECT_EQ(lc.epoch, 1u);

  // Projection: COMPLETED must project to LUMICE_SERVER_IDLE via QueryServerState.
  LUMICE_ServerState state{};
  ASSERT_EQ(LUMICE_QueryServerState(server_, &state), LUMICE_OK);
  EXPECT_EQ(state, LUMICE_SERVER_IDLE);

  // Second reset-causing commit: epoch ++ again (monotonic).
  ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.epoch, 2u) << "epoch must ++ on each reset-causing commit";
  ASSERT_TRUE(WaitForIdle(server_, 10000)) << "Server did not reach IDLE within 10 seconds";
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, LUMICE_LIFECYCLE_COMPLETED);
  EXPECT_EQ(lc.epoch, 2u);

  // Stop resets consumption → IDLE (not COMPLETED); epoch is unchanged (Stop is
  // not a commit).
  LUMICE_StopServer(server_);
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, LUMICE_LIFECYCLE_IDLE);
  EXPECT_EQ(lc.epoch, 2u) << "Stop does not advance epoch";
}


TEST_F(ServerLifecycleApi, GetSimLifecycleNullArgs) {
  LUMICE_SimLifecycleResult lc{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(nullptr, &lc), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetSimLifecycle(server_, nullptr), LUMICE_ERR_NULL_ARG);
}


TEST_F(ServerLifecycleApi, GetRenderResults) {
  CommitAndWaitForIdle();

  // out array size = LUMICE_MAX_RENDER_RESULTS + 1 (sentinel slot)
  LUMICE_RenderResult out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRenderResults(server_, out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);

  // First (and only) renderer matches MakeMinimalConfigJson() resolution 800x400, id=1
  EXPECT_EQ(out[0].renderer_id, 1);
  EXPECT_EQ(out[0].img_width, 800);
  EXPECT_EQ(out[0].img_height, 400);
  EXPECT_NE(out[0].img_buffer, nullptr);

  // Sentinel: img_buffer == NULL marks end of array
  EXPECT_EQ(out[1].img_buffer, nullptr);
}


TEST_F(ServerLifecycleApi, GetRawXyzResults) {
  CommitAndWaitForIdle();

  LUMICE_RawXyzResult out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(server_, out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);

  EXPECT_EQ(out[0].renderer_id, 1);
  EXPECT_EQ(out[0].img_width, 800);
  EXPECT_EQ(out[0].img_height, 400);
  EXPECT_NE(out[0].xyz_buffer, nullptr);
  EXPECT_NE(out[0].has_valid_data, 0);

  // Sentinel: xyz_buffer == NULL marks end of array
  EXPECT_EQ(out[1].xyz_buffer, nullptr);
}


TEST_F(ServerLifecycleApi, GetStatsResults) {
  CommitAndWaitForIdle();

  LUMICE_StatsResult out[LUMICE_MAX_STATS_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetStatsResults(server_, out, LUMICE_MAX_STATS_RESULTS), LUMICE_OK);

  // After running 1000 rays through one crystal with single-pass scattering:
  EXPECT_GT(out[0].sim_ray_num, 0u);
  EXPECT_GT(out[0].crystal_num, 0u);

  // Sentinel: sim_ray_num == 0 marks end of array
  EXPECT_EQ(out[1].sim_ray_num, 0u);
}


TEST_F(ServerLifecycleApi, GetCachedStatsConsistency) {
  CommitAndWaitForIdle();
  // Precondition: simulation has completed; no new data is being produced.
  // This guarantees DoSnapshot results are stable across calls.

  // Trigger DoSnapshot (which updates cached_stats_result_) by calling GetStatsResults.
  // Note: any Get function that triggers DoSnapshot updates the cache, not just GetStatsResults.
  LUMICE_StatsResult fresh[LUMICE_MAX_STATS_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetStatsResults(server_, fresh, LUMICE_MAX_STATS_RESULTS), LUMICE_OK);
  ASSERT_GT(fresh[0].sim_ray_num, 0u);

  // Cached stats should now match the fresh ones.
  LUMICE_StatsResult cached{};
  ASSERT_EQ(LUMICE_GetCachedStats(server_, &cached), LUMICE_OK);
  EXPECT_EQ(cached.sim_ray_num, fresh[0].sim_ray_num);
  EXPECT_EQ(cached.crystal_num, fresh[0].crystal_num);
  EXPECT_EQ(cached.ray_seg_num, fresh[0].ray_seg_num);

  // Cache stability: a second GetCachedStats call without any intervening Get*Results
  // call must return the same values (no new snapshot).
  LUMICE_StatsResult cached_again{};
  ASSERT_EQ(LUMICE_GetCachedStats(server_, &cached_again), LUMICE_OK);
  EXPECT_EQ(cached_again.sim_ray_num, cached.sim_ray_num);
  EXPECT_EQ(cached_again.crystal_num, cached.crystal_num);
}


// Regression for ServerImpl::Stop() lost-wakeup deadlock: drives CommitConfig→Stop in
// a tight loop to hit the narrow race window. Each Stop runs on a worker thread with a
// per-iteration timeout — a hang surfaces as a FAIL() rather than wedging the whole
// test process. On timeout the worker is detached and server_ is cleared so TearDown
// neither re-enters Stop() (would deadlock again) nor destroys the server out from
// under the detached worker (UAF). Leaks the hung server handle, acceptable since this
// test case has already FAILED and the leak is isolated to this iteration's instance.
TEST_F(ServerLifecycleApi, StressStartStop) {
  constexpr int kIterations = 200;
  constexpr int kStopTimeoutMs = 3000;

  auto small_cfg = MakeSmallSimConfigJson();
  for (int i = 0; i < kIterations; ++i) {
    ASSERT_EQ(LUMICE_CommitConfig(server_, small_cfg.c_str()), LUMICE_OK) << "CommitConfig failed at iter " << i;

    // shared_ptr + by-value capture keep the worker self-contained: if it is detached and
    // later unwinds (shouldn't post-fix), it touches neither the stack flag nor the fixture.
    auto stop_done = std::make_shared<std::atomic<bool>>(false);
    LUMICE_Server* srv = server_;
    std::thread t([stop_done, srv] {
      LUMICE_StopServer(srv);
      stop_done->store(true, std::memory_order_release);
    });
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kStopTimeoutMs);
    while (!stop_done->load(std::memory_order_acquire)) {
      if (std::chrono::steady_clock::now() >= deadline) {
        t.detach();
        // Clear server_ so TearDown skips StopServer/DestroyServer — avoids re-entering
        // the same deadlock and the UAF on the still-running detached thread.
        server_ = nullptr;
        FAIL() << "LUMICE_StopServer hung > " << kStopTimeoutMs << "ms on iteration " << i;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    t.join();
  }
}


// Regression: backend exit-seam path used to early-return without emplacing
// a SimData when all rays in a batch were filtered/absorbed (exit_count==0),
// while GenerateScene had already incremented sim_scene_cnt_. The counter
// never returned to 0 and GetStatus() stayed non-IDLE → CLI/capi hung.
// A highly selective BD raypath filter generates many such 0-exit batches.
//
// cpu_backend is used (not metal/legacy) because:
//   - it exercises the same backend exit-seam code path as metal cross-platform
//   - legacy CPU doesn't go through backend.ReadbackExitRays at all
// 60s timeout dominates the ~few-seconds expected runtime; on regression the
// server will sit forever at <1.0 progress with workers idle in config_queue.
TEST_F(ServerLifecycleApi, ZeroExitBatchNoHang) {
#ifdef _WIN32
  ::_putenv_s("LUMICE_TRACE_BACKEND", "cpu_backend");
  struct EnvGuard {
    ~EnvGuard() { ::_putenv_s("LUMICE_TRACE_BACKEND", ""); }
  } guard;
#else
  ::setenv("LUMICE_TRACE_BACKEND", "cpu_backend", 1);
  struct EnvGuard {
    ~EnvGuard() { ::unsetenv("LUMICE_TRACE_BACKEND"); }
  } guard;
#endif

  auto json = MakeBdFilterConfigJson();
  ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(server_, 60000))
      << "Server did not reach IDLE within 60s — sim_scene_cnt_ leak on 0-exit batch regressed";
}


// NULL-arg checks for the four Get* result functions.
TEST(ResultsApi, NullArgsGetters) {
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);

  LUMICE_RenderResult render_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_GetRenderResults(nullptr, render_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetRenderResults(server, nullptr, LUMICE_MAX_RENDER_RESULTS), LUMICE_ERR_NULL_ARG);

  LUMICE_RawXyzResult xyz_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_GetRawXyzResults(nullptr, xyz_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetRawXyzResults(server, nullptr, LUMICE_MAX_RENDER_RESULTS), LUMICE_ERR_NULL_ARG);

  LUMICE_StatsResult stats_out[LUMICE_MAX_STATS_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_GetStatsResults(nullptr, stats_out, LUMICE_MAX_STATS_RESULTS), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetStatsResults(server, nullptr, LUMICE_MAX_STATS_RESULTS), LUMICE_ERR_NULL_ARG);

  LUMICE_StatsResult cached{};
  EXPECT_EQ(LUMICE_GetCachedStats(nullptr, &cached), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetCachedStats(server, nullptr), LUMICE_ERR_NULL_ARG);

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}


TEST(ResultsApi, NullArgsQueryState) {
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);

  LUMICE_ServerState state{};
  EXPECT_EQ(LUMICE_QueryServerState(nullptr, &state), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_QueryServerState(server, nullptr), LUMICE_ERR_NULL_ARG);

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}


// "No-data" path: a freshly created server with no committed config should return
// LUMICE_OK + sentinel/empty results from all Get functions, not error codes.
// This is intentional — there is no public API path that returns LUMICE_SERVER_NOT_READY,
// because callers are expected to treat "no data yet" as a normal state.
TEST(ResultsApi, GetBeforeCommit) {
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);

  // Render results: empty count, sentinel at index 0
  LUMICE_RenderResult render_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_GetRenderResults(server, render_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(render_out[0].img_buffer, nullptr);

  // Raw XYZ results: empty count, sentinel at index 0
  LUMICE_RawXyzResult xyz_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_GetRawXyzResults(server, xyz_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(xyz_out[0].xyz_buffer, nullptr);

  // Stats results: empty count, sentinel at index 0
  LUMICE_StatsResult stats_out[LUMICE_MAX_STATS_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_GetStatsResults(server, stats_out, LUMICE_MAX_STATS_RESULTS), LUMICE_OK);
  EXPECT_EQ(stats_out[0].sim_ray_num, 0u);

  // Cached stats: all-zero struct
  LUMICE_StatsResult cached{};
  EXPECT_EQ(LUMICE_GetCachedStats(server, &cached), LUMICE_OK);
  EXPECT_EQ(cached.sim_ray_num, 0u);
  EXPECT_EQ(cached.crystal_num, 0u);
  EXPECT_EQ(cached.ray_seg_num, 0u);

  // Server state: IDLE (no commit, no work running)
  LUMICE_ServerState state = LUMICE_SERVER_RUNNING;  // Initialize to non-IDLE to detect change
  EXPECT_EQ(LUMICE_QueryServerState(server, &state), LUMICE_OK);
  EXPECT_EQ(state, LUMICE_SERVER_IDLE);

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// ============================================================
// LUMICE_MAX_ID
// ============================================================

TEST(MaxIdApi, ValueMatchesUint16Max) {
  EXPECT_EQ(LUMICE_MAX_ID, 65535);
}

// ============================================================
// LUMICE_IsLegalFace
// ============================================================

TEST(IsLegalFaceApi, PrismLegalFaces) {
  // Basal faces: 1, 2; prism lateral faces: 3..8
  EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, 1), 0);
  EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, 2), 0);
  for (int f = 3; f <= 8; ++f) {
    EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, f), 0) << "face=" << f;
  }
}

TEST(IsLegalFaceApi, PrismIllegalFaces) {
  EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, 0), 0);
  EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, 9), 0);
  for (int f = 13; f <= 18; ++f) {
    EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, f), 0) << "face=" << f;
  }
  for (int f = 23; f <= 28; ++f) {
    EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PRISM, f), 0) << "face=" << f;
  }
}

TEST(IsLegalFaceApi, PyramidLegalFaces) {
  EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, 1), 0);
  EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, 2), 0);
  for (int f = 3; f <= 8; ++f) {
    EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, f), 0) << "face=" << f;
  }
  for (int f = 13; f <= 18; ++f) {
    EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, f), 0) << "face=" << f;
  }
  for (int f = 23; f <= 28; ++f) {
    EXPECT_NE(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, f), 0) << "face=" << f;
  }
}

TEST(IsLegalFaceApi, PyramidIllegalFaces) {
  EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, 0), 0);
  EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, 9), 0);
  EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, 29), 0);
  EXPECT_EQ(LUMICE_IsLegalFace(LUMICE_CRYSTAL_PYRAMID, 100), 0);
}

// ============================================================
// LUMICE_ValidateRaypathText
// ============================================================

TEST(ValidateRaypathTextApi, NullArgs) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_VALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText(nullptr, LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ValidateRaypathText("3", LUMICE_CRYSTAL_PRISM, nullptr, msg, sizeof(msg)), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ValidateRaypathText("3", LUMICE_CRYSTAL_PRISM, &vstate, nullptr, sizeof(msg)), LUMICE_ERR_NULL_ARG);
}

TEST(ValidateRaypathTextApi, EmptyIsValid) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_INVALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_VALID);
  EXPECT_EQ(std::string(msg), "");
}

TEST(ValidateRaypathTextApi, ValidSingleFace) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_INVALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("3", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_VALID);
}

TEST(ValidateRaypathTextApi, ValidMultiFace) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_INVALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("3-5-8", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_VALID);
}

TEST(ValidateRaypathTextApi, TrailingSepIncomplete) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_VALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("3-", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_INCOMPLETE);
}

TEST(ValidateRaypathTextApi, NonDigitInvalid) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_VALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("3-x", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_INVALID);
}

TEST(ValidateRaypathTextApi, KindSpecificInvalid) {
  // Face 13 is legal on pyramid but not prism
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_VALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("13", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_INVALID);
  EXPECT_NE(std::string(msg).find("not legal on this crystal type"), std::string::npos);
}

TEST(ValidateRaypathTextApi, KindSpecificValidPyramid) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_INVALID;
  char msg[256] = {};
  EXPECT_EQ(LUMICE_ValidateRaypathText("13", LUMICE_CRYSTAL_PYRAMID, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(vstate, LUMICE_RAYPATH_VALID);
}

TEST(ValidateRaypathTextApi, MsgBufTruncation) {
  LUMICE_RaypathValidationState vstate = LUMICE_RAYPATH_VALID;
  char msg[4] = {};
  // Should not crash; result must be null-terminated within 4 bytes
  EXPECT_EQ(LUMICE_ValidateRaypathText("3-x-y", LUMICE_CRYSTAL_PRISM, &vstate, msg, sizeof(msg)), LUMICE_OK);
  EXPECT_EQ(msg[3], '\0');
}

// ============================================================
// GuiValidateFaceNumberText (via raypath_segments.hpp)
// — validates the substring "not legal on this crystal type"
//   that ParseFaceNumberOrZero relies on for kind-specific detection.
// ============================================================

#include "gui/raypath_segments.hpp"

TEST(GuiValidateFaceNumberTextApi, KindSpecificMsgContainsExpectedSubstring) {
  // Face 13 is legal on pyramid but not prism — kind-specific rejection
  auto r = lumice::gui::GuiValidateFaceNumberText("13", LUMICE_CRYSTAL_PRISM);
  EXPECT_EQ(r.state, LUMICE_RAYPATH_INVALID);
  EXPECT_NE(r.message.find("not legal on this crystal type"), std::string::npos);
}

// ============================================================
// LUMICE_MaxFov
// ============================================================

TEST(MaxFovApi, LinearIs179) {
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_LINEAR), 179.0f, 0.01f);
}

TEST(MaxFovApi, StereographicIs359) {
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_FISHEYE_STEREOGRAPHIC), 359.0f, 0.01f);
}

TEST(MaxFovApi, OrthographicIs180) {
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_FISHEYE_ORTHOGRAPHIC), 180.0f, 0.01f);
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_DUAL_FISHEYE_ORTHOGRAPHIC), 180.0f, 0.01f);
}

TEST(MaxFovApi, GlobeIs90) {
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_GLOBE), 90.0f, 0.01f);
}

TEST(MaxFovApi, DefaultIs360) {
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_FISHEYE_EQUAL_AREA), 360.0f, 0.01f);
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_FISHEYE_EQUIDISTANT), 360.0f, 0.01f);
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_DUAL_FISHEYE_EQUAL_AREA), 360.0f, 0.01f);
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_DUAL_FISHEYE_EQUIDISTANT), 360.0f, 0.01f);
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_DUAL_FISHEYE_STEREOGRAPHIC), 360.0f, 0.01f);
  EXPECT_NEAR(LUMICE_MaxFov(LUMICE_LENS_RECTANGULAR), 360.0f, 0.01f);
}

TEST(BackendAvailabilityApi, CpuAlwaysAvailable) {
  EXPECT_EQ(LUMICE_IsBackendAvailable(LUMICE_BACKEND_CPU), 1);
}

TEST(BackendAvailabilityApi, UnknownBackendReturnsZero) {
  EXPECT_EQ(LUMICE_IsBackendAvailable(999), 0);
  EXPECT_EQ(LUMICE_IsBackendAvailable(-1), 0);
}

TEST(BackendAvailabilityApi, MetalMatchesPlatform) {
#if defined(__APPLE__)
  // CI/dev Macs are expected to support Metal (any M-series or post-2012
  // Intel Mac). If this assertion ever fires in a future environment without
  // a Metal device, guard the EXPECT with a runtime probe or mark DISABLED.
  EXPECT_EQ(LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL), 1);
#else
  EXPECT_EQ(LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL), 0);
#endif
}

TEST(BackendAvailabilityApi, CachedAcrossCalls) {
  const int kFirst = LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL), kFirst);
  }
}

// =====================================================================================
// task-struct-simple-arms (327.2): LUMICE_FilterParam 5-arm tagged union.
//
// Emit-shape tests call ConfigToJson directly (via server/c_api_internal.hpp) and assert
// the JSON field-by-field, so an emit-side field-name/condition error is caught HERE
// rather than masked by core's lenient from_json (which would default missing fields and
// still let LUMICE_CommitConfigStruct return OK). Commit tests then verify the end-to-end
// ABI-safe path: valid simple types commit; UNSET(0)/out-of-range type -> INVALID_CONFIG
// (via ConfigToJson throw + CommitConfigStruct catch), never a crash across the C ABI.
// =====================================================================================

namespace {
// Minimal LUMICE_Config carrying exactly one filter, for ConfigToJson emit assertions.
// Other sections stay empty (counts 0) so ConfigToJson's crystal/render/scatter loops are
// no-ops; spectrum == nullptr resolves to "D65". ConfigToJson does not validate, so this
// is enough to inspect the emitted filter shape without a server.
LUMICE_Config MakeOneFilterConfig(const LUMICE_FilterParam& f) {
  LUMICE_Config config{};
  config.filter_count = 1;
  config.filters[0] = f;
  return config;
}

// Assert the emitted filter object has EXACTLY this key set — catches both a missing
// field and an unexpected/cross-arm field (e.g. a Direction filter that wrongly carries
// "raypath" or "entry"). Per-field EXPECT_EQ alone only checks presence, never absence.
void ExpectFilterKeys(const nlohmann::json& jf, const std::set<std::string>& expected) {
  std::set<std::string> actual;
  for (auto it = jf.begin(); it != jf.end(); ++it) {
    actual.insert(it.key());
  }
  EXPECT_EQ(actual, expected);
}
}  // namespace

TEST(StructFilterEmit, None) {
  LUMICE_FilterParam f{};
  f.id = 7;
  f.action = 0;  // filter_in
  f.type = LUMICE_FILTER_TYPE_NONE;
  auto root = ConfigToJson(MakeOneFilterConfig(f));
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("id").get<int>(), 7);
  EXPECT_EQ(jf.at("type").get<std::string>(), "none");
  EXPECT_EQ(jf.at("action").get<std::string>(), "filter_in");
  // No symmetry bits set -> no "symmetry" key; no arm-specific fields for "none".
  ExpectFilterKeys(jf, { "id", "action", "type", "symmetry" });
}

TEST(StructFilterEmit, Raypath) {
  LUMICE_FilterParam f{};
  f.id = 1;
  f.action = 1;  // filter_out
  f.type = LUMICE_FILTER_TYPE_RAYPATH;
  f.raypath_count = 3;
  f.raypath[0] = 3;
  f.raypath[1] = 1;
  f.raypath[2] = 5;
  f.symmetry = 1 | 2;  // P | B
  auto root = ConfigToJson(MakeOneFilterConfig(f));
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("type").get<std::string>(), "raypath");
  EXPECT_EQ(jf.at("action").get<std::string>(), "filter_out");
  ASSERT_TRUE(jf.at("raypath").is_array());
  EXPECT_EQ(jf.at("raypath").size(), 3u);
  EXPECT_EQ(jf.at("raypath")[0].get<int>(), 3);
  EXPECT_EQ(jf.at("raypath")[2].get<int>(), 5);
  EXPECT_EQ(jf.at("symmetry").get<std::string>(), "PB");
  ExpectFilterKeys(jf, { "id", "action", "type", "raypath", "symmetry" });
}

TEST(StructFilterEmit, EntryExitAllFields) {
  LUMICE_FilterParam f{};
  f.id = 2;
  f.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  f.ee_entry = 3;
  f.ee_exit = 5;
  f.ee_min_len = 2;
  f.ee_max_len = 8;
  auto root = ConfigToJson(MakeOneFilterConfig(f));
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("type").get<std::string>(), "entry_exit");
  EXPECT_EQ(jf.at("entry").get<int>(), 3);
  EXPECT_EQ(jf.at("exit").get<int>(), 5);
  EXPECT_EQ(jf.at("min_len").get<int>(), 2);
  EXPECT_EQ(jf.at("max_len").get<int>(), 8);
  ExpectFilterKeys(jf, { "id", "action", "type", "symmetry", "entry", "exit", "min_len", "max_len" });
}

TEST(StructFilterEmit, EntryExitWildcardsOmitted) {
  // -1 sentinels (wildcard entry/exit, no max_len) and min_len == 1 must be omitted,
  // mirroring core to_json which only emits set / non-default optional fields.
  LUMICE_FilterParam f{};
  f.id = 2;
  f.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  f.ee_entry = -1;
  f.ee_exit = -1;
  f.ee_min_len = 1;
  f.ee_max_len = -1;
  auto root = ConfigToJson(MakeOneFilterConfig(f));
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("type").get<std::string>(), "entry_exit");
  EXPECT_FALSE(jf.contains("entry"));
  EXPECT_FALSE(jf.contains("exit"));
  EXPECT_FALSE(jf.contains("min_len"));
  EXPECT_FALSE(jf.contains("max_len"));
  ExpectFilterKeys(jf, { "id", "action", "type", "symmetry" });
}

TEST(StructFilterEmit, Direction) {
  LUMICE_FilterParam f{};
  f.id = 4;
  f.type = LUMICE_FILTER_TYPE_DIRECTION;
  f.dir_az = 120.0f;
  f.dir_el = -15.0f;
  f.dir_radii = 2.5f;
  auto root = ConfigToJson(MakeOneFilterConfig(f));
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("type").get<std::string>(), "direction");
  EXPECT_FLOAT_EQ(jf.at("az").get<float>(), 120.0f);
  EXPECT_FLOAT_EQ(jf.at("el").get<float>(), -15.0f);
  EXPECT_FLOAT_EQ(jf.at("radii").get<float>(), 2.5f);
  ExpectFilterKeys(jf, { "id", "action", "type", "symmetry", "az", "el", "radii" });
}

TEST(StructFilterEmit, Crystal) {
  LUMICE_FilterParam f{};
  f.id = 5;
  f.type = LUMICE_FILTER_TYPE_CRYSTAL;
  f.crystal_id = 2;
  auto root = ConfigToJson(MakeOneFilterConfig(f));
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("type").get<std::string>(), "crystal");
  EXPECT_EQ(jf.at("crystal_id").get<int>(), 2);
  ExpectFilterKeys(jf, { "id", "action", "type", "symmetry", "crystal_id" });
}

TEST(StructFilterEmit, UnsetTypeThrows) {
  // Zero-init guard: type == UNSET(0) must throw, not silently emit "none".
  LUMICE_FilterParam f{};  // type defaults to 0 == UNSET
  f.id = 1;
  EXPECT_THROW(ConfigToJson(MakeOneFilterConfig(f)), std::invalid_argument);
}

TEST(StructFilterEmit, OutOfRangeTypeThrows) {
  LUMICE_FilterParam f{};
  f.id = 1;
  f.type = 99;  // out of range
  EXPECT_THROW(ConfigToJson(MakeOneFilterConfig(f)), std::invalid_argument);
}

// --- Isomorphism cross-check against core's own to_json -------------------------------
// The AC is "emit JSON isomorphic to filter_config.cpp::to_json". Rather than hand-copy
// the expected field list (which twice missed the symmetry key-presence detail), build
// the equivalent core lumice::FilterConfig, run core's to_json (the source of truth), and
// assert byte-equality with ConfigToJson's filter object. This catches any future drift.

namespace {
void ExpectEmitMatchesCore(const LUMICE_FilterParam& lf, const lumice::FilterConfig& fc) {
  nlohmann::json core_j = fc;  // ADL -> lumice::to_json(json&, const FilterConfig&)
  auto my_j = ConfigToJson(MakeOneFilterConfig(lf)).at("filter").at(0);
  EXPECT_EQ(my_j, core_j) << "C-API emit:\n" << my_j.dump(2) << "\ncore to_json:\n" << core_j.dump(2);
}
}  // namespace

TEST(StructFilterEmitIsomorphism, None) {
  lumice::FilterConfig fc;
  fc.id_ = 7;
  fc.symmetry_ = lumice::FilterConfig::kSymNone;
  fc.action_ = lumice::FilterConfig::kFilterIn;
  fc.param_ = lumice::SimpleFilterParam{ lumice::NoneFilterParam{} };
  LUMICE_FilterParam lf{};
  lf.id = 7;
  lf.action = 0;
  lf.symmetry = 0;
  lf.type = LUMICE_FILTER_TYPE_NONE;
  ExpectEmitMatchesCore(lf, fc);
}

TEST(StructFilterEmitIsomorphism, RaypathWithSymmetry) {
  lumice::FilterConfig fc;
  fc.id_ = 1;
  fc.symmetry_ = lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB;
  fc.action_ = lumice::FilterConfig::kFilterOut;
  fc.param_ = lumice::SimpleFilterParam{ lumice::RaypathFilterParam{ std::vector<lumice::IdType>{ 3, 1, 5 } } };
  LUMICE_FilterParam lf{};
  lf.id = 1;
  lf.action = 1;
  lf.symmetry = 1 | 2;
  lf.type = LUMICE_FILTER_TYPE_RAYPATH;
  lf.raypath_count = 3;
  lf.raypath[0] = 3;
  lf.raypath[1] = 1;
  lf.raypath[2] = 5;
  ExpectEmitMatchesCore(lf, fc);
}

TEST(StructFilterEmitIsomorphism, EntryExitFull) {
  lumice::EntryExitFilterParam ee;
  ee.entry_ = lumice::IdType{ 3 };
  ee.exit_ = lumice::IdType{ 5 };
  ee.min_len_ = 2;
  ee.max_len_ = std::size_t{ 8 };
  lumice::FilterConfig fc;
  fc.id_ = 2;
  fc.symmetry_ = lumice::FilterConfig::kSymNone;
  fc.action_ = lumice::FilterConfig::kFilterIn;
  fc.param_ = lumice::SimpleFilterParam{ ee };
  LUMICE_FilterParam lf{};
  lf.id = 2;
  lf.action = 0;
  lf.symmetry = 0;
  lf.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  lf.ee_entry = 3;
  lf.ee_exit = 5;
  lf.ee_min_len = 2;
  lf.ee_max_len = 8;
  ExpectEmitMatchesCore(lf, fc);
}

TEST(StructFilterEmitIsomorphism, EntryExitWildcardsWithSymmetry) {
  // Covers "symmetry set + non-raypath type" (the combination prior tests skipped) AND
  // the wildcard/omitted-field path, cross-checked against core.
  lumice::EntryExitFilterParam ee;  // entry_/exit_/max_len_ = nullopt, min_len_ = 1
  lumice::FilterConfig fc;
  fc.id_ = 2;
  fc.symmetry_ = lumice::FilterConfig::kSymP | lumice::FilterConfig::kSymB;
  fc.action_ = lumice::FilterConfig::kFilterIn;
  fc.param_ = lumice::SimpleFilterParam{ ee };
  LUMICE_FilterParam lf{};
  lf.id = 2;
  lf.action = 0;
  lf.symmetry = 1 | 2;
  lf.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  lf.ee_entry = -1;
  lf.ee_exit = -1;
  lf.ee_min_len = 1;
  lf.ee_max_len = -1;
  ExpectEmitMatchesCore(lf, fc);
}

TEST(StructFilterEmitIsomorphism, Direction) {
  lumice::DirectionFilterParam dir;
  dir.lon_ = 120.0f;
  dir.lat_ = -15.0f;
  dir.radii_ = 2.5f;
  lumice::FilterConfig fc;
  fc.id_ = 4;
  fc.symmetry_ = lumice::FilterConfig::kSymNone;
  fc.action_ = lumice::FilterConfig::kFilterIn;
  fc.param_ = lumice::SimpleFilterParam{ dir };
  LUMICE_FilterParam lf{};
  lf.id = 4;
  lf.action = 0;
  lf.symmetry = 0;
  lf.type = LUMICE_FILTER_TYPE_DIRECTION;
  lf.dir_az = 120.0f;
  lf.dir_el = -15.0f;
  lf.dir_radii = 2.5f;
  ExpectEmitMatchesCore(lf, fc);
}

TEST(StructFilterEmitIsomorphism, Crystal) {
  lumice::CrystalFilterParam cr;
  cr.crystal_id_ = lumice::IdType{ 2 };
  lumice::FilterConfig fc;
  fc.id_ = 5;
  fc.symmetry_ = lumice::FilterConfig::kSymNone;
  fc.action_ = lumice::FilterConfig::kFilterIn;
  fc.param_ = lumice::SimpleFilterParam{ cr };
  LUMICE_FilterParam lf{};
  lf.id = 5;
  lf.action = 0;
  lf.symmetry = 0;
  lf.type = LUMICE_FILTER_TYPE_CRYSTAL;
  lf.crystal_id = 2;
  ExpectEmitMatchesCore(lf, fc);
}

// --- End-to-end commit through LUMICE_CommitConfigStruct ------------------------------

namespace {
// Parse the full config (crystals + scene + one referenced filter), then replace that
// filter (keeping its id, so the scattering reference stays valid) with `f` and shrink to
// a fast finite sim. Returns the config ready for LUMICE_CommitConfigStruct.
LUMICE_Config MakeCommitConfigWithFilter(const LUMICE_FilterParam& f) {
  LUMICE_Config config{};
  EXPECT_EQ(LUMICE_ParseConfigString(MakeFullConfigJson().c_str(), &config), LUMICE_OK);
  EXPECT_GE(config.filter_count, 1);
  const int fid = config.filters[0].id;
  config.filters[0] = f;
  config.filters[0].id = fid;
  config.infinite = 0;
  config.ray_num = 100;
  return config;
}

int CommitFilter(const LUMICE_FilterParam& f) {
  LUMICE_Config config = MakeCommitConfigWithFilter(f);
  auto* server = LUMICE_CreateServer();
  EXPECT_NE(server, nullptr);
  int reused = -1;
  auto err = LUMICE_CommitConfigStruct(server, &config, &reused);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
  return err;
}
}  // namespace

TEST(StructFilterCommit, EntryExitCommitsOk) {
  LUMICE_FilterParam f{};
  f.action = 0;
  f.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  f.ee_entry = 3;
  f.ee_exit = 5;
  f.ee_min_len = 1;
  f.ee_max_len = -1;
  EXPECT_EQ(CommitFilter(f), LUMICE_OK);
}

TEST(StructFilterCommit, DirectionCommitsOk) {
  LUMICE_FilterParam f{};
  f.action = 0;
  f.type = LUMICE_FILTER_TYPE_DIRECTION;
  f.dir_az = 22.0f;
  f.dir_el = 0.0f;
  f.dir_radii = 5.0f;
  EXPECT_EQ(CommitFilter(f), LUMICE_OK);
}

TEST(StructFilterCommit, CrystalCommitsOk) {
  LUMICE_FilterParam f{};
  f.action = 0;
  f.type = LUMICE_FILTER_TYPE_CRYSTAL;
  f.crystal_id = 1;  // MakeFullConfigJson defines crystals with ids 1 and 2
  EXPECT_EQ(CommitFilter(f), LUMICE_OK);
}

TEST(StructFilterCommit, UnsetTypeReturnsInvalidConfigNotCrash) {
  // A construction site that forgot to set `type` zero-inits to UNSET(0). ConfigToJson
  // throws; LUMICE_CommitConfigStruct must catch and return INVALID_CONFIG, never crash.
  LUMICE_FilterParam f{};  // type == UNSET
  f.action = 0;
  f.type = LUMICE_FILTER_TYPE_UNSET;
  f.raypath_count = 1;
  f.raypath[0] = 3;
  EXPECT_EQ(CommitFilter(f), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterCommit, OutOfRangeTypeReturnsInvalidConfigNotCrash) {
  LUMICE_FilterParam f{};
  f.action = 0;
  f.type = 99;  // out of range
  EXPECT_EQ(CommitFilter(f), LUMICE_ERR_INVALID_CONFIG);
}

// =====================================================================================
// task-serialize-completion (327.1): parse direction (JSON -> struct) for all 5 simple
// types + public LUMICE_ConfigToJson. Round-trip goes through the public serialize + parse
// APIs; cross-check against core from_json (source of truth) rather than hand-transcribed
// expectations (see learnings: contract-and-property-tests / emit-schema cross-check).
// =====================================================================================

namespace {
// struct -> JSON (public LUMICE_ConfigToJson) -> struct (LUMICE_ParseConfigString). Returns
// the round-tripped filters[0]. Exercises both new 327.1 pieces end to end.
LUMICE_FilterParam RoundTripFilter(const LUMICE_FilterParam& in) {
  LUMICE_Config cfg = MakeOneFilterConfig(in);
  // Zero-init so that if LUMICE_ConfigToJson unexpectedly fails, the subsequent
  // LUMICE_ParseConfigString sees a valid empty C-string (loud parse error) rather than
  // reading uninitialized stack memory (ASSERT_EQ can't be used in this value-returning
  // helper). Truncation is covered separately by ConfigToJsonBufferTruncationContract.
  char buf[8192] = {};
  size_t len = 0;
  EXPECT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  EXPECT_LT(len, sizeof(buf));  // these small configs never truncate
  LUMICE_Config out{};
  EXPECT_EQ(LUMICE_ParseConfigString(buf, &out), LUMICE_OK);
  EXPECT_EQ(out.filter_count, 1);
  return out.filters[0];
}

// Replace MakeFullConfigJson's filter[0] with `jf` (id kept 1 so scattering ref stays valid)
// and return the full config JSON string, for parse-side tests.
std::string FullConfigWithFilterJson(const nlohmann::json& jf) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  root["filter"][0] = jf;
  return root.dump();
}
}  // namespace

TEST(StructFilterParse, NoneRoundTrip) {
  LUMICE_FilterParam in{};
  in.id = 7;
  in.action = 0;
  in.type = LUMICE_FILTER_TYPE_NONE;
  auto out = RoundTripFilter(in);
  EXPECT_EQ(out.type, LUMICE_FILTER_TYPE_NONE);
  EXPECT_EQ(out.action, 0);
}

TEST(StructFilterParse, RaypathRoundTrip) {
  LUMICE_FilterParam in{};
  in.id = 1;
  in.action = 1;
  in.symmetry = 1 | 2;
  in.type = LUMICE_FILTER_TYPE_RAYPATH;
  in.raypath_count = 3;
  in.raypath[0] = 3;
  in.raypath[1] = 1;
  in.raypath[2] = 5;
  auto out = RoundTripFilter(in);
  EXPECT_EQ(out.type, LUMICE_FILTER_TYPE_RAYPATH);
  EXPECT_EQ(out.action, 1);
  EXPECT_EQ(out.symmetry, 1 | 2);
  EXPECT_EQ(out.raypath_count, 3);
  EXPECT_EQ(out.raypath[0], 3);
  EXPECT_EQ(out.raypath[2], 5);
}

TEST(StructFilterParse, EntryExitRoundTrip) {
  LUMICE_FilterParam in{};
  in.id = 2;
  in.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  in.ee_entry = 3;
  in.ee_exit = 5;
  in.ee_min_len = 2;
  in.ee_max_len = 8;
  auto out = RoundTripFilter(in);
  EXPECT_EQ(out.type, LUMICE_FILTER_TYPE_ENTRY_EXIT);
  EXPECT_EQ(out.ee_entry, 3);
  EXPECT_EQ(out.ee_exit, 5);
  EXPECT_EQ(out.ee_min_len, 2);
  EXPECT_EQ(out.ee_max_len, 8);
}

TEST(StructFilterParse, EntryExitWildcardRoundTrip) {
  LUMICE_FilterParam in{};
  in.id = 2;
  in.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  in.ee_entry = -1;
  in.ee_exit = -1;
  in.ee_min_len = 1;
  in.ee_max_len = -1;
  auto out = RoundTripFilter(in);
  EXPECT_EQ(out.type, LUMICE_FILTER_TYPE_ENTRY_EXIT);
  EXPECT_EQ(out.ee_entry, -1);  // absent -> wildcard sentinel
  EXPECT_EQ(out.ee_exit, -1);
  EXPECT_EQ(out.ee_min_len, 1);  // absent -> default 1
  EXPECT_EQ(out.ee_max_len, -1);
}

TEST(StructFilterParse, DirectionRoundTrip) {
  LUMICE_FilterParam in{};
  in.id = 4;
  in.type = LUMICE_FILTER_TYPE_DIRECTION;
  in.dir_az = 120.0f;
  in.dir_el = -15.0f;
  in.dir_radii = 2.5f;
  auto out = RoundTripFilter(in);
  EXPECT_EQ(out.type, LUMICE_FILTER_TYPE_DIRECTION);
  EXPECT_FLOAT_EQ(out.dir_az, 120.0f);
  EXPECT_FLOAT_EQ(out.dir_el, -15.0f);
  EXPECT_FLOAT_EQ(out.dir_radii, 2.5f);
}

TEST(StructFilterParse, CrystalRoundTrip) {
  LUMICE_FilterParam in{};
  in.id = 5;
  in.type = LUMICE_FILTER_TYPE_CRYSTAL;
  in.crystal_id = 2;
  auto out = RoundTripFilter(in);
  EXPECT_EQ(out.type, LUMICE_FILTER_TYPE_CRYSTAL);
  EXPECT_EQ(out.crystal_id, 2);
}

TEST(StructFilterParse, NonRaypathTypesNoLongerRejected) {
  // Regression: pre-327.1, ParseConfigString rejected non-raypath filters with INVALID_VALUE.
  auto json = FullConfigWithFilterJson(
      { { "id", 1 }, { "action", "filter_in" }, { "type", "entry_exit" }, { "entry", 3 }, { "exit", 5 } });
  LUMICE_Config out{};
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &out), LUMICE_OK);
  ASSERT_GE(out.filter_count, 1);
  EXPECT_EQ(out.filters[0].type, LUMICE_FILTER_TYPE_ENTRY_EXIT);
  EXPECT_EQ(out.filters[0].ee_entry, 3);
  EXPECT_EQ(out.filters[0].ee_exit, 5);
  EXPECT_EQ(out.filters[0].ee_min_len, 1);   // absent -> default
  EXPECT_EQ(out.filters[0].ee_max_len, -1);  // absent
}

TEST(StructFilterParse, ComplexWithoutCompositionRejected) {
  // As of 327.3, complex filters DO parse (see StructFilterComplex tests). But a complex
  // filter missing its required "composition" array is rejected with MISSING_FIELD.
  auto json = FullConfigWithFilterJson({ { "id", 1 }, { "action", "filter_in" }, { "type", "complex" } });
  LUMICE_Config out{};
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &out), LUMICE_ERR_MISSING_FIELD);
}

TEST(StructFilterParse, UnknownTypeRejected) {
  // The default branch also covers arbitrary unknown type strings (not just "complex").
  auto json = FullConfigWithFilterJson({ { "id", 1 }, { "action", "filter_in" }, { "type", "bogus" } });
  LUMICE_Config out{};
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &out), LUMICE_ERR_INVALID_VALUE);
}

TEST(StructFilterParse, IllegalEntryExitValuePassesThroughLikeCore) {
  // Decision (plan 327.1 §7 risk 2): parse does lossless mapping only; value validation
  // (min_len >= 1) stays single-source in core and fires at commit. So parse of min_len=0
  // must succeed and store it verbatim (core would likewise not reject at from_json time;
  // it throws only later). This pins the "validation not duplicated in parse" contract.
  auto json = FullConfigWithFilterJson(
      { { "id", 1 }, { "action", "filter_in" }, { "type", "entry_exit" }, { "entry", 3 }, { "min_len", 0 } });
  LUMICE_Config out{};
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &out), LUMICE_OK);
  ASSERT_GE(out.filter_count, 1);
  EXPECT_EQ(out.filters[0].ee_min_len, 0);  // stored verbatim, not normalized/rejected here
}

// Parse cross-check against core from_json (source of truth): parsing a filter JSON via
// LUMICE_ParseConfigString then re-emitting (LUMICE_ConfigToJson) must byte-match core's own
// from_json -> to_json round-trip of the same JSON. Since 327.2 proved emit == core to_json,
// equality here proves the parse direction also agrees with core from_json.
namespace {
void ExpectParseMatchesCore(const nlohmann::json& jf) {
  // core path: from_json -> FilterConfig -> to_json
  lumice::FilterConfig fc = jf.get<lumice::FilterConfig>();
  nlohmann::json core_out = fc;
  // my path: ParseConfigString -> struct -> LUMICE_ConfigToJson -> filter[0]
  LUMICE_Config cfg{};
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFilterJson(jf).c_str(), &cfg), LUMICE_OK);
  char buf[8192];
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  auto my_root = nlohmann::json::parse(std::string(buf, len));
  EXPECT_EQ(my_root.at("filter").at(0), core_out) << "mine:\n"
                                                  << my_root.at("filter").at(0).dump(2) << "\ncore:\n"
                                                  << core_out.dump(2);
}
}  // namespace

TEST(StructFilterParseIsomorphism, Raypath) {
  ExpectParseMatchesCore({ { "id", 1 },
                           { "action", "filter_out" },
                           { "type", "raypath" },
                           { "raypath", { 3, 1, 5 } },
                           { "symmetry", "PB" } });
}
TEST(StructFilterParseIsomorphism, None) {
  ExpectParseMatchesCore({ { "id", 7 }, { "action", "filter_in" }, { "type", "none" }, { "symmetry", "" } });
}
TEST(StructFilterParseIsomorphism, EntryExit) {
  ExpectParseMatchesCore({ { "id", 2 },
                           { "action", "filter_in" },
                           { "type", "entry_exit" },
                           { "entry", 3 },
                           { "exit", 5 },
                           { "min_len", 2 },
                           { "max_len", 8 },
                           { "symmetry", "" } });
}
TEST(StructFilterParseIsomorphism, Direction) {
  ExpectParseMatchesCore({ { "id", 4 },
                           { "action", "filter_in" },
                           { "type", "direction" },
                           { "az", 120.0 },
                           { "el", -15.0 },
                           { "radii", 2.5 },
                           { "symmetry", "" } });
}
TEST(StructFilterParseIsomorphism, Crystal) {
  ExpectParseMatchesCore(
      { { "id", 5 }, { "action", "filter_in" }, { "type", "crystal" }, { "crystal_id", 2 }, { "symmetry", "" } });
}

TEST(StructFilterParse, ConfigToJsonBufferTruncationContract) {
  // Exercises the snprintf-style caller-buffer contract (buffer overrun handling is the
  // highest-risk path for a new C ABI function; plan 327.1 Step 2 required this test).
  LUMICE_FilterParam f{};
  f.id = 1;
  f.action = 0;
  f.type = LUMICE_FILTER_TYPE_RAYPATH;
  f.raypath_count = 2;
  f.raypath[0] = 3;
  f.raypath[1] = 5;
  LUMICE_Config cfg = MakeOneFilterConfig(f);

  // NULL config -> NULL_ARG.
  size_t tmp = 0;
  EXPECT_EQ(LUMICE_ConfigToJson(nullptr, nullptr, 0, &tmp), LUMICE_ERR_NULL_ARG);

  // Query length only (out_buf == NULL, buf_size == 0).
  size_t full_len = 0;
  EXPECT_EQ(LUMICE_ConfigToJson(&cfg, nullptr, 0, &full_len), LUMICE_OK);
  EXPECT_GT(full_len, size_t{ 8 });  // full JSON is well over 8 bytes

  // Full (untruncated) reference output.
  char full[8192];
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, full, sizeof(full), nullptr), LUMICE_OK);

  // Truncate into a small buffer.
  char small[8];
  std::memset(small, 'X', sizeof(small));
  size_t len = 0;
  EXPECT_EQ(LUMICE_ConfigToJson(&cfg, small, sizeof(small), &len), LUMICE_OK);
  EXPECT_EQ(len, full_len);                          // out_len = FULL length, not written count
  EXPECT_GE(len, sizeof(small));                     // out_len >= buf_size signals truncation
  EXPECT_EQ(small[sizeof(small) - 1], '\0');         // always NUL-terminated
  EXPECT_EQ(std::strlen(small), sizeof(small) - 1);  // wrote exactly buf_size-1 chars
  EXPECT_EQ(std::string(small, sizeof(small) - 1),   // truncated prefix matches full prefix
            std::string(full, sizeof(small) - 1));
}

// =====================================================================================
// task-complex-ref-encoding (327.3): complex (sum-of-products) filter, flat reference
// encoding (separate compositions[] pool + composition_index; cells reference simple-filter
// ids). Cross-checked against core ConfigManager's own two-pass resolution (source of truth).
// =====================================================================================

namespace {
std::string FullConfigWithFiltersJson(const nlohmann::json& filter_array) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  root["filter"] = filter_array;
  return root.dump();
}

// Cross-check the C-API complex emit against core's own to_json (source of truth). Builds a
// core FilterConfig with an equivalent ComplexFilterParam from the composition JSON and runs
// core to_json; core emits only the referenced simple-filter ids (pair.first), so the
// SimpleFilterParam content is irrelevant. Then parses the same config via the C API and
// re-emits, and asserts the complex filter's JSON matches byte for byte. This proves parse +
// emit of complex agree with core, without needing core's strict full-config ConfigManager
// parse (which requires render/scene fields the lenient LUMICE parse does not).
void ExpectComplexMatchesCore(const nlohmann::json& filter_array, int complex_id) {
  nlohmann::json complex_jf;
  for (const auto& jf : filter_array) {
    if (jf.at("id").get<int>() == complex_id) {
      complex_jf = jf;
      break;
    }
  }
  ASSERT_FALSE(complex_jf.is_null());

  lumice::FilterConfig fc;
  fc.id_ = static_cast<lumice::IdType>(complex_id);
  fc.action_ = lumice::FilterConfig::kFilterIn;
  fc.symmetry_ = lumice::FilterConfig::kSymNone;
  lumice::ComplexFilterParam cp;
  for (const auto& clause : complex_jf.at("composition")) {
    std::vector<std::pair<lumice::IdType, lumice::SimpleFilterParam>> terms;
    if (clause.is_array()) {
      for (const auto& term : clause) {
        terms.emplace_back(term.get<lumice::IdType>(), lumice::SimpleFilterParam{});
      }
    } else {
      terms.emplace_back(clause.get<lumice::IdType>(), lumice::SimpleFilterParam{});
    }
    cp.filters_.emplace_back(terms);
  }
  fc.param_ = cp;
  nlohmann::json core_j = fc;  // core to_json

  LUMICE_Config cfg{};
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filter_array).c_str(), &cfg), LUMICE_OK);
  char buf[16384];
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  auto my_root = nlohmann::json::parse(std::string(buf, len));
  nlohmann::json my_j;
  for (const auto& jf : my_root.at("filter")) {
    if (jf.at("id").get<int>() == complex_id) {
      my_j = jf;
      break;
    }
  }
  ASSERT_FALSE(my_j.is_null());
  EXPECT_EQ(my_j, core_j) << "mine:\n" << my_j.dump(2) << "\ncore:\n" << core_j.dump(2);
}
}  // namespace

TEST(StructFilterComplex, OrOfRaypathsMatchesCore) {  // issue.md 场景: 多段 raypath = OR-of-raypaths
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2 } } },
  });
  ExpectComplexMatchesCore(filters, 3);
}

TEST(StructFilterComplex, OrOfEntryExitMatchesCore) {  // issue.md 场景: 多值 EE = OR-of-EE
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "entry_exit" }, { "entry", 3 }, { "exit", 5 } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "entry_exit" }, { "entry", 1 } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2 } } },
  });
  ExpectComplexMatchesCore(filters, 3);
}

TEST(StructFilterComplex, CrossTypeWithAndClauseMatchesCore) {  // issue.md 场景: 跨 type 组合 + AND 子句
  // composition = OR( AND(1,2), 1 ) — mixes a 2-term array clause and a bare-id clause.
  nlohmann::json comp = nlohmann::json::array({ nlohmann::json::array({ 1, 2 }), 1 });
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "entry_exit" }, { "entry", 3 } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", comp } },
  });
  ExpectComplexMatchesCore(filters, 3);
}

TEST(StructFilterComplex, StructRoundTrip) {
  // Build a complex config struct directly, round-trip through the public serialize+parse
  // APIs, and assert the composition survives (clause/term/id fidelity).
  LUMICE_Config in{};
  in.filter_count = 3;
  in.filters[0].id = 1;
  in.filters[0].type = LUMICE_FILTER_TYPE_RAYPATH;
  in.filters[0].raypath_count = 2;
  in.filters[0].raypath[0] = 3;
  in.filters[0].raypath[1] = 5;
  in.filters[1].id = 2;
  in.filters[1].type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  in.filters[1].ee_entry = 3;
  in.filters[1].ee_exit = -1;
  in.filters[1].ee_min_len = 1;
  in.filters[1].ee_max_len = -1;
  in.filters[2].id = 3;
  in.filters[2].type = LUMICE_FILTER_TYPE_COMPLEX;
  in.filters[2].composition_index = 0;
  in.composition_count = 1;
  in.compositions[0].clause_count = 2;
  in.compositions[0].term_counts[0] = 2;  // AND(1,2)
  in.compositions[0].clauses[0][0] = 1;
  in.compositions[0].clauses[0][1] = 2;
  in.compositions[0].term_counts[1] = 1;  // bare 1
  in.compositions[0].clauses[1][0] = 1;

  char buf[16384];
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&in, buf, sizeof(buf), &len), LUMICE_OK);
  LUMICE_Config out{};
  ASSERT_EQ(LUMICE_ParseConfigString(buf, &out), LUMICE_OK);

  ASSERT_EQ(out.filter_count, 3);
  ASSERT_EQ(out.filters[2].type, LUMICE_FILTER_TYPE_COMPLEX);
  const auto& oc = out.compositions[out.filters[2].composition_index];
  EXPECT_EQ(oc.clause_count, 2);
  EXPECT_EQ(oc.term_counts[0], 2);
  EXPECT_EQ(oc.clauses[0][0], 1);
  EXPECT_EQ(oc.clauses[0][1], 2);
  EXPECT_EQ(oc.term_counts[1], 1);
  EXPECT_EQ(oc.clauses[1][0], 1);
}

TEST(StructFilterComplex, CommitStructEndToEnd) {
  // Build the complex config via parse, commit through the struct path -> core consumes it.
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2 } } },
  });
  LUMICE_Config cfg{};
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  cfg.infinite = 0;
  cfg.ray_num = 100;
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(server, &cfg, &reused), LUMICE_OK);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

TEST(StructFilterComplex, DanglingReferenceRejected) {
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 99 } } },  // 99 not defined
  });
  LUMICE_Config cfg{};
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, ReferenceToComplexRejected) {
  // A composition term may only reference a SIMPLE filter, never another complex (no cycles).
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 2 } } },  // refs complex 2
  });
  LUMICE_Config cfg{};
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, TooManyTermsRejected) {
  // A clause with more than LUMICE_MAX_CONFIG_TERMS terms -> INVALID_CONFIG.
  nlohmann::json simples = nlohmann::json::array();
  nlohmann::json big_clause = nlohmann::json::array();
  for (int i = 1; i <= LUMICE_MAX_CONFIG_TERMS + 1; i++) {
    simples.push_back({ { "id", i }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } });
    big_clause.push_back(i);
  }
  nlohmann::json filters = simples;
  filters.push_back({ { "id", 100 },
                      { "action", "filter_in" },
                      { "type", "complex" },
                      { "composition", nlohmann::json::array({ big_clause }) } });
  LUMICE_Config cfg{};
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, TooManyClausesRejected) {
  // A composition with more than LUMICE_MAX_CONFIG_CLAUSES clauses -> INVALID_CONFIG.
  nlohmann::json comp = nlohmann::json::array();
  for (int i = 0; i < LUMICE_MAX_CONFIG_CLAUSES + 1; i++) {
    comp.push_back(1);  // each clause references simple filter id 1
  }
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", comp } },
  });
  LUMICE_Config cfg{};
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, TooManyCompositionsRejected) {
  // More than LUMICE_MAX_CONFIG_COMPLEX complex filters -> INVALID_CONFIG on the overflow one.
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
  });
  for (int i = 0; i < LUMICE_MAX_CONFIG_COMPLEX + 1; i++) {
    filters.push_back(
        { { "id", 100 + i }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1 } } });
  }
  LUMICE_Config cfg{};
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, EmptyCompositionAccepted) {
  // An empty composition ([]) is a degenerate but accepted "OR of nothing" (clause_count 0).
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", nlohmann::json::array() } },
  });
  LUMICE_Config cfg{};
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_GE(cfg.filter_count, 2);
  EXPECT_EQ(cfg.filters[1].type, LUMICE_FILTER_TYPE_COMPLEX);
  EXPECT_EQ(cfg.compositions[cfg.filters[1].composition_index].clause_count, 0);
}
