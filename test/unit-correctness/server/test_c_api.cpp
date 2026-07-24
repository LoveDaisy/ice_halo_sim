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
#include <vector>

#include "config/filter_config.hpp"         // core FilterConfig + to_json, for emit isomorphism cross-check
#include "config/raypath_color_config.hpp"  // core RaypathColorConfig + to_json, for color-class isomorphism
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "include/lumice.h"
#include "include/lumice_config_scope.hpp"  // lumice::ConfigOwningGuard RAII for raypath_color
#include "server/c_api_internal.hpp"        // ConfigToJson (test-only exposure) for emit-shape assertions

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

// ABI guards for the other structs test/e2e/capi_runner.py mirrors. Added after
// task-cuda-ctypes-teardown-crash: LUMICE_RenderResult grew from 24 → 32 bytes
// in task-345.3 when composite_p99_y was appended (float@24 + 4 pad, 8-aligned),
// but the ctypes mirror was not updated. Result: LUMICE_GetRenderResults wrote
// 8 bytes past the Python-allocated (LUMICE_RenderResult * 1)() buffer on every
// poll, corrupting the Python heap and aborting under _ctypes teardown or the
// next glibc malloc/free check. LUMICE_ServerConfig has the same failure mode
// (12 vs 8 byte drift after preferred_backend was appended). Static-assert both
// so a future field addition breaks the build instead of silently corrupting
// pytest heaps.
static_assert(sizeof(LUMICE_RenderResult) == 32, "LUMICE_RenderResult ABI must be 32 bytes (capi_runner.py mirror)");
static_assert(sizeof(LUMICE_ServerConfig) == 12, "LUMICE_ServerConfig ABI must be 12 bytes (capi_runner.py mirror)");

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
  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &config), LUMICE_OK);

  EXPECT_EQ(config.crystal_count, 1);
  EXPECT_EQ(config.crystals[0].id, 1);
  EXPECT_EQ(config.crystals[0].type, 0);  // prism
  EXPECT_FLOAT_EQ(config.crystals[0].height.center, 1.5f);

  // Default face_distance = all NO_RANDOM 1.0
  for (int k = 0; k < 6; k++) {
    EXPECT_EQ(config.crystals[0].face_distance[k].type, LUMICE_DIST_NO_RANDOM);
    EXPECT_FLOAT_EQ(config.crystals[0].face_distance[k].center, 1.0f);
  }

  // Axis
  EXPECT_EQ(config.crystals[0].zenith.type, LUMICE_DIST_GAUSS);
  EXPECT_FLOAT_EQ(config.crystals[0].zenith.center, 90.0f);
  EXPECT_FLOAT_EQ(config.crystals[0].zenith.spread, 10.0f);

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

  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.infinite, 0);
  // Pre-fix on Windows this truncated to 705'032'704; post-fix it holds the full value.
  EXPECT_EQ(config.ray_num, kBigRayNum);
}


TEST(ParseConfigApi, FullConfigWithPyramidAndFilter) {
  auto json = MakeFullConfigJson();
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &config), LUMICE_OK);

  // Two crystals
  EXPECT_EQ(config.crystal_count, 2);

  // Prism with custom face_distance
  EXPECT_EQ(config.crystals[0].type, 0);
  EXPECT_FLOAT_EQ(config.crystals[0].face_distance[1].center, 0.8f);
  EXPECT_FLOAT_EQ(config.crystals[0].face_distance[2].center, 1.0f);

  // Pyramid
  EXPECT_EQ(config.crystals[1].type, 1);
  EXPECT_FLOAT_EQ(config.crystals[1].prism_h.center, 1.0f);
  EXPECT_FLOAT_EQ(config.crystals[1].upper_h.center, 0.5f);
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
  lumice::ConfigOwningGuard config_guard(config);
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

  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigFile(tmp_path.u8string().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.crystal_count, 1);
  EXPECT_FLOAT_EQ(config.crystals[0].height.center, 1.5f);

  std::filesystem::remove(tmp_path);
}


TEST(ParseConfigApi, NullArgs) {
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigString(nullptr, &config), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ParseConfigString("{}", nullptr), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ParseConfigFile(nullptr, &config), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ParseConfigFile("/tmp/test.json", nullptr), LUMICE_ERR_NULL_ARG);
}


TEST(ParseConfigApi, InvalidJson) {
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigString("not json at all", &config), LUMICE_ERR_INVALID_JSON);
  EXPECT_EQ(LUMICE_ParseConfigString("{invalid", &config), LUMICE_ERR_INVALID_JSON);
}


TEST(ParseConfigApi, MissingCrystalSection) {
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  // Valid JSON but missing "crystal" key
  EXPECT_EQ(LUMICE_ParseConfigString(R"({"scene": {}})", &config), LUMICE_ERR_MISSING_FIELD);
}


TEST(ParseConfigApi, FileNotFound) {
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigFile("/tmp/nonexistent_lumice_config_12345.json", &config), LUMICE_ERR_FILE_NOT_FOUND);
}


TEST(ParseConfigApi, UnsupportedFilterType) {
  nlohmann::json root;
  root["crystal"] = nlohmann::json::array();
  root["scene"] = { { "ray_num", 1000 } };
  root["filter"] = nlohmann::json::array({ { { "id", 1 }, { "type", "direction" }, { "action", "filter_in" } } });

  LUMICE_Config config{};

  lumice::ConfigOwningGuard config_guard(config);
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

  lumice::ConfigOwningGuard config_guard(config);
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
  lumice::ConfigOwningGuard config_guard(config);
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

  lumice::ConfigOwningGuard config_guard(config);
  EXPECT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_CONFIG);
}


TEST(ParseConfigApi, SpectrumEnumerations) {
  // Test all supported spectrum strings (D55 / D75 added in task-323).
  for (const char* sp : { "D65", "D55", "D50", "D75", "A", "E" }) {
    auto json_str = MakeMinimalConfigJson();
    auto root = nlohmann::json::parse(json_str);
    root["scene"]["light_source"]["spectrum"] = sp;

    LUMICE_Config config{};

    lumice::ConfigOwningGuard config_guard(config);
    ASSERT_EQ(LUMICE_ParseConfigString(root.dump().c_str(), &config), LUMICE_OK) << "Failed for spectrum: " << sp;
    EXPECT_STREQ(config.spectrum, sp);
    EXPECT_EQ(config.spectrum_count, 0);
  }

  // Unknown spectrum string
  auto json_str = MakeMinimalConfigJson();
  auto root = nlohmann::json::parse(json_str);
  root["scene"]["light_source"]["spectrum"] = "UnknownIlluminant";

  LUMICE_Config config{};

  lumice::ConfigOwningGuard config_guard(config);
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

// task-gui-feedback-affordances Step 7 (AC1): LUMICE_GetColorOverflowInfo
// exposes the component-bit overflow count captured synchronously during
// CommitConfig. A well-formed color config (no overflow) reads 0; a config
// with > 64 distinct predicates on one placement reads the number dropped.
TEST_F(ServerLifecycleApi, GetColorOverflowInfoZeroWhenNoOverflow) {
  auto json = MakeSmallSimConfigJson();
  ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
  LUMICE_ColorOverflowInfo info{};
  ASSERT_EQ(LUMICE_GetColorOverflowInfo(server_, &info), LUMICE_OK);
  EXPECT_EQ(info.component_overflow_count, 0);
  // The three GPU-only async caps are 0 here: no overflow, and this CPU-backend
  // config never touches the device buffer-layout limits. task-color-degrade-gui-surfacing.
  EXPECT_EQ(info.symmetry_group_overflow_count, 0);
  EXPECT_EQ(info.or_summand_overflow_count, 0);
  EXPECT_EQ(info.color_class_overflow_count, 0);
}

TEST_F(ServerLifecycleApi, GetColorOverflowInfoReportsPredicateDrops) {
  // Build a config where the color config has 65 unique predicates on the
  // single placement (layer 0, crystal 1). 65 - 64 (ComponentTable::kMaxBits)
  // = 1 predicate must be dropped to kNoBit.
  auto base = nlohmann::json::parse(MakeSmallSimConfigJson());
  nlohmann::json classes = nlohmann::json::array();
  nlohmann::json cls;
  cls["color"] = { 1.0, 0.0, 0.0 };
  nlohmann::json matches = nlohmann::json::array();
  for (int k = 0; k < 65; ++k) {
    // Each ref carries a structurally-unique EE predicate (distinct min_len),
    // so dedup does NOT collapse them; every ref consumes one component bit.
    // Fields sit at the ref's top level (Design-2 RaypathColorRef inline
    // predicate schema — see raypath_color_config.cpp from_json).
    nlohmann::json ref;
    ref["layer"] = 0;
    ref["crystal"] = 1;
    ref["type"] = "entry_exit";
    ref["entry"] = 1;
    ref["exit"] = 1;
    ref["min_len"] = k + 1;
    matches.push_back(ref);
  }
  cls["match"] = matches;
  classes.push_back(cls);
  base["raypath_color"]["classes"] = classes;
  const std::string json = base.dump();

  ASSERT_EQ(LUMICE_CommitConfig(server_, json.c_str()), LUMICE_OK);
  LUMICE_ColorOverflowInfo info{};
  ASSERT_EQ(LUMICE_GetColorOverflowInfo(server_, &info), LUMICE_OK);
  EXPECT_EQ(info.component_overflow_count, 1);  // 65 - 64 = 1 predicate dropped
  // GPU-only async caps stay 0 on this CPU-backend, sync-only commit path.
  EXPECT_EQ(info.symmetry_group_overflow_count, 0);
  EXPECT_EQ(info.or_summand_overflow_count, 0);
  EXPECT_EQ(info.color_class_overflow_count, 0);
}

TEST_F(ServerLifecycleApi, GetColorOverflowInfoNullArgs) {
  LUMICE_ColorOverflowInfo info{};
  EXPECT_EQ(LUMICE_GetColorOverflowInfo(nullptr, &info), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetColorOverflowInfo(server_, nullptr), LUMICE_ERR_NULL_ARG);
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
// task-344: caller-owned out-param (LUMICE_Config is now non-copyable — raypath_color owns
// a heap allocation; returning by value would leave two aliased copies). Caller must attach
// a lumice::ConfigOwningGuard to `out` before calling.
void FillOneFilterConfig(LUMICE_Config* out, const LUMICE_FilterParam& f) {
  out->filter_count = 1;
  out->filters[0] = f;
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
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  auto root = ConfigToJson(config);
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
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  auto root = ConfigToJson(config);
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
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  auto root = ConfigToJson(config);
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
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  auto root = ConfigToJson(config);
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
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  auto root = ConfigToJson(config);
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
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  auto root = ConfigToJson(config);
  const auto& jf = root.at("filter").at(0);
  EXPECT_EQ(jf.at("type").get<std::string>(), "crystal");
  EXPECT_EQ(jf.at("crystal_id").get<int>(), 2);
  ExpectFilterKeys(jf, { "id", "action", "type", "symmetry", "crystal_id" });
}

TEST(StructFilterEmit, UnsetTypeThrows) {
  // Zero-init guard: type == UNSET(0) must throw, not silently emit "none".
  LUMICE_FilterParam f{};  // type defaults to 0 == UNSET
  f.id = 1;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  EXPECT_THROW(ConfigToJson(config), std::invalid_argument);
}

TEST(StructFilterEmit, OutOfRangeTypeThrows) {
  LUMICE_FilterParam f{};
  f.id = 1;
  f.type = 99;  // out of range
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  EXPECT_THROW(ConfigToJson(config), std::invalid_argument);
}

// --- Isomorphism cross-check against core's own to_json -------------------------------
// The AC is "emit JSON isomorphic to filter_config.cpp::to_json". Rather than hand-copy
// the expected field list (which twice missed the symmetry key-presence detail), build
// the equivalent core lumice::FilterConfig, run core's to_json (the source of truth), and
// assert byte-equality with ConfigToJson's filter object. This catches any future drift.

namespace {
void ExpectEmitMatchesCore(const LUMICE_FilterParam& lf, const lumice::FilterConfig& fc) {
  nlohmann::json core_j = fc;  // ADL -> lumice::to_json(json&, const FilterConfig&)
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneFilterConfig(&config, lf);
  auto my_j = ConfigToJson(config).at("filter").at(0);
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
// task-344: caller-owned out-param. Populates `out` via LUMICE_ParseConfigString then
// overrides filter[0] and finiteness. Caller must attach ConfigOwningGuard first.
void FillCommitConfigWithFilter(LUMICE_Config* out, const LUMICE_FilterParam& f) {
  EXPECT_EQ(LUMICE_ParseConfigString(MakeFullConfigJson().c_str(), out), LUMICE_OK);
  EXPECT_GE(out->filter_count, 1);
  const int fid = out->filters[0].id;
  out->filters[0] = f;
  out->filters[0].id = fid;
  out->infinite = 0;
  out->ray_num = 100;
}

int CommitFilter(const LUMICE_FilterParam& f) {
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillCommitConfigWithFilter(&config, f);
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
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  FillOneFilterConfig(&cfg, in);
  // Zero-init so that if LUMICE_ConfigToJson unexpectedly fails, the subsequent
  // LUMICE_ParseConfigString sees a valid empty C-string (loud parse error) rather than
  // reading uninitialized stack memory (ASSERT_EQ can't be used in this value-returning
  // helper). Truncation is covered separately by ConfigToJsonBufferTruncationContract.
  char buf[8192] = {};
  size_t len = 0;
  EXPECT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  EXPECT_LT(len, sizeof(buf));  // these small configs never truncate
  LUMICE_Config out{};
  lumice::ConfigOwningGuard out_guard(out);
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
  lumice::ConfigOwningGuard out_guard(out);
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
  lumice::ConfigOwningGuard out_guard(out);
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &out), LUMICE_ERR_MISSING_FIELD);
}

TEST(StructFilterParse, UnknownTypeRejected) {
  // The default branch also covers arbitrary unknown type strings (not just "complex").
  auto json = FullConfigWithFilterJson({ { "id", 1 }, { "action", "filter_in" }, { "type", "bogus" } });
  LUMICE_Config out{};
  lumice::ConfigOwningGuard out_guard(out);
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
  lumice::ConfigOwningGuard out_guard(out);
  EXPECT_EQ(LUMICE_ParseConfigString(json.c_str(), &out), LUMICE_OK);
  ASSERT_GE(out.filter_count, 1);
  EXPECT_EQ(out.filters[0].ee_min_len, 0);  // stored verbatim, not normalized/rejected here
}

// =====================================================================================
// v4.10 distribution leaf: LUMICE_Distribution round-trips for shape scalars
// (struct -> LUMICE_ConfigToJson -> LUMICE_ParseConfigString), across all six distribution
// types INCLUDING no_random (AC3), plus geom_clock struct-path equivalence (AC5) and the
// LUMICE_API_VERSION compile-time guard (AC4).
// =====================================================================================

// AC4: LUMICE_API_VERSION exists and is usable in a caller static_assert.
static_assert(LUMICE_API_VERSION >= 410, "LUMICE_API_VERSION regressed below v4.10");

namespace {
// struct -> JSON (public LUMICE_ConfigToJson) -> struct (LUMICE_ParseConfigString). Returns the
// round-tripped crystals[0]. Mirrors RoundTripFilter for the crystal / distribution path.
LUMICE_CrystalParam RoundTripCrystal(const LUMICE_CrystalParam& in) {
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  cfg.crystal_count = 1;
  cfg.crystals[0] = in;
  char buf[8192] = {};
  size_t len = 0;
  EXPECT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  EXPECT_LT(len, sizeof(buf));  // these small configs never truncate
  LUMICE_Config out{};
  lumice::ConfigOwningGuard out_guard(out);
  EXPECT_EQ(LUMICE_ParseConfigString(buf, &out), LUMICE_OK);
  EXPECT_EQ(out.crystal_count, 1);
  return out.crystals[0];
}

// A prism crystal with `height` set to `d`; axis + all six face_distance left NO_RANDOM defaults.
LUMICE_CrystalParam MakePrismWithHeight(const LUMICE_Distribution& d) {
  LUMICE_CrystalParam cr{};  // zero-init => every distribution is NO_RANDOM
  cr.id = 1;
  cr.type = 0;  // prism
  cr.height = d;
  for (int i = 0; i < 6; i++) {
    cr.face_distance[i] = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
  }
  return cr;
}
}  // namespace

TEST(DistributionRoundTrip, AllTypesHeight) {
  struct Case {
    int type;
    float center;
    float spread;
  };
  // Covers ALL six LUMICE_DIST_* including NO_RANDOM (AC3): height goes struct -> JSON -> struct.
  const Case cases[] = {
    { LUMICE_DIST_NO_RANDOM, 1.5f, 0.0f },  { LUMICE_DIST_UNIFORM, 2.0f, 0.3f },
    { LUMICE_DIST_GAUSS, 1.2f, 0.1f },      { LUMICE_DIST_ZIGZAG, 0.9f, 0.2f },
    { LUMICE_DIST_LAPLACIAN, 1.1f, 0.05f }, { LUMICE_DIST_GAUSS_LEGACY, 1.3f, 0.15f },
  };
  for (const auto& c : cases) {
    LUMICE_Distribution in{ c.type, c.center, c.spread };
    auto out = RoundTripCrystal(MakePrismWithHeight(in)).height;
    EXPECT_EQ(out.type, c.type) << "type=" << c.type;
    EXPECT_FLOAT_EQ(out.center, c.center) << "type=" << c.type;
    // NO_RANDOM serializes as a bare number (no spread on the wire); it round-trips as spread 0.
    // For the randomized types spread must survive verbatim.
    if (c.type != LUMICE_DIST_NO_RANDOM) {
      EXPECT_FLOAT_EQ(out.spread, c.spread) << "type=" << c.type;
    }
  }
}

// AC3 focused case: a NO_RANDOM shape scalar serializes to a bare JSON number and parses back.
// Before Step 3 taught JsonToDistribution the is_number() branch, this failed with
// LUMICE_ERR_MISSING_FIELD (bare number has no "type"/"mean"/"std"). Red/green evidence in
// this task's progress log.
TEST(DistributionRoundTrip, NoRandomHeight) {
  auto out = RoundTripCrystal(MakePrismWithHeight({ LUMICE_DIST_NO_RANDOM, 1.5f, 0.0f })).height;
  EXPECT_EQ(out.type, LUMICE_DIST_NO_RANDOM);
  EXPECT_FLOAT_EQ(out.center, 1.5f);
}

// Per-face independence: a mix of distribution types across the 6 faces must survive round-trip
// (not collapsed by a "all default" fast path).
TEST(DistributionRoundTrip, FaceDistancePerFaceMixed) {
  LUMICE_CrystalParam cr{};
  cr.id = 1;
  cr.type = 0;
  cr.height = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
  cr.face_distance[0] = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
  cr.face_distance[1] = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 0.8f, 0.05f };
  cr.face_distance[2] = LUMICE_Distribution{ LUMICE_DIST_UNIFORM, 1.2f, 0.1f };
  cr.face_distance[3] = LUMICE_Distribution{ LUMICE_DIST_ZIGZAG, 0.9f, 0.2f };
  cr.face_distance[4] = LUMICE_Distribution{ LUMICE_DIST_LAPLACIAN, 1.1f, 0.03f };
  cr.face_distance[5] = LUMICE_Distribution{ LUMICE_DIST_GAUSS_LEGACY, 1.0f, 0.15f };
  auto out = RoundTripCrystal(cr);
  EXPECT_EQ(out.face_distance[0].type, LUMICE_DIST_NO_RANDOM);
  EXPECT_FLOAT_EQ(out.face_distance[0].center, 1.0f);
  EXPECT_EQ(out.face_distance[1].type, LUMICE_DIST_GAUSS);
  EXPECT_FLOAT_EQ(out.face_distance[1].center, 0.8f);
  EXPECT_FLOAT_EQ(out.face_distance[1].spread, 0.05f);
  EXPECT_EQ(out.face_distance[2].type, LUMICE_DIST_UNIFORM);
  EXPECT_FLOAT_EQ(out.face_distance[2].center, 1.2f);
  EXPECT_EQ(out.face_distance[3].type, LUMICE_DIST_ZIGZAG);
  EXPECT_EQ(out.face_distance[4].type, LUMICE_DIST_LAPLACIAN);
  EXPECT_EQ(out.face_distance[5].type, LUMICE_DIST_GAUSS_LEGACY);
  EXPECT_FLOAT_EQ(out.face_distance[5].spread, 0.15f);
}

// Pyramid shape scalars (prism_h/upper_h/lower_h) are distributions too; verify no_random +
// a randomized type survive round-trip on the pyramid arm.
TEST(DistributionRoundTrip, PyramidShapeScalars) {
  LUMICE_CrystalParam cr{};
  cr.id = 1;
  cr.type = 1;  // pyramid
  cr.prism_h = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
  cr.upper_h = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 0.5f, 0.02f };
  cr.lower_h = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 0.5f, 0.0f };
  cr.upper_wedge_angle = 28.0f;
  cr.lower_wedge_angle = 28.0f;
  for (int i = 0; i < 6; i++) {
    cr.face_distance[i] = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
  }
  auto out = RoundTripCrystal(cr);
  EXPECT_EQ(out.prism_h.type, LUMICE_DIST_NO_RANDOM);
  EXPECT_FLOAT_EQ(out.prism_h.center, 1.0f);
  EXPECT_EQ(out.upper_h.type, LUMICE_DIST_GAUSS);
  EXPECT_FLOAT_EQ(out.upper_h.center, 0.5f);
  EXPECT_FLOAT_EQ(out.upper_h.spread, 0.02f);
  EXPECT_EQ(out.lower_h.type, LUMICE_DIST_NO_RANDOM);
  EXPECT_FLOAT_EQ(out.upper_wedge_angle, 28.0f);
}

// AC5: geom_clock reaches the scene JSON via the struct path, with the same "0 => omit" wire
// convention core proj_config.cpp uses; and it survives a struct -> JSON -> struct round-trip.
TEST(GeomClockStructPath, EmittedWhenSet) {
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  cfg.geom_clock = 30;
  char buf[8192] = {};
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  auto j = nlohmann::json::parse(buf);
  ASSERT_TRUE(j.contains("scene"));
  ASSERT_TRUE(j["scene"].contains("geom_clock"));
  EXPECT_EQ(j["scene"]["geom_clock"].get<int>(), 30);
}

TEST(GeomClockStructPath, OmittedWhenZero) {
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  cfg.geom_clock = 0;  // zero-init default: disabled
  char buf[8192] = {};
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  auto j = nlohmann::json::parse(buf);
  ASSERT_TRUE(j.contains("scene"));
  EXPECT_FALSE(j["scene"].contains("geom_clock"));
}

TEST(GeomClockStructPath, RoundTrip) {
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  cfg.geom_clock = 16;
  char buf[8192] = {};
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, buf, sizeof(buf), &len), LUMICE_OK);
  LUMICE_Config out{};
  lumice::ConfigOwningGuard out_guard(out);
  ASSERT_EQ(LUMICE_ParseConfigString(buf, &out), LUMICE_OK);
  EXPECT_EQ(out.geom_clock, 16);
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
  lumice::ConfigOwningGuard cfg_guard(cfg);
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
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  FillOneFilterConfig(&cfg, f);

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

  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filter_array).c_str(), &cfg), LUMICE_OK);
  // Query full length first, then allocate a right-sized buffer — the pre-v4.9 fixed 16KB
  // buffer overflowed once compositions with N > ~100 OR-clauses became possible.
  size_t needed = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, nullptr, 0, &needed), LUMICE_OK);
  std::vector<char> buf(needed + 1, '\0');
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&cfg, buf.data(), buf.size(), &len), LUMICE_OK);
  auto my_root = nlohmann::json::parse(std::string(buf.data(), len));
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
  lumice::ConfigOwningGuard in_guard(in);
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
  // v4.9: compositions[i] owns heap ptrs; populate via LUMICE_CompositionSetClauses
  // rather than direct field writes. `clauses = OR(AND(1,2), 1)` in clause-major flat form:
  //   term_counts = [2, 1], term_ids = [1, 2,   1]
  int rt_term_counts[2] = { 2, 1 };
  int rt_term_ids[3] = { 1, 2, 1 };
  ASSERT_EQ(LUMICE_CompositionSetClauses(&in.compositions[0], 2, rt_term_counts, rt_term_ids), LUMICE_OK);

  char buf[16384];
  size_t len = 0;
  ASSERT_EQ(LUMICE_ConfigToJson(&in, buf, sizeof(buf), &len), LUMICE_OK);
  LUMICE_Config out{};
  lumice::ConfigOwningGuard out_guard(out);
  ASSERT_EQ(LUMICE_ParseConfigString(buf, &out), LUMICE_OK);

  ASSERT_EQ(out.filter_count, 3);
  ASSERT_EQ(out.filters[2].type, LUMICE_FILTER_TYPE_COMPLEX);
  const auto& oc = out.compositions[out.filters[2].composition_index];
  EXPECT_EQ(oc.clause_count, 2);
  int c0_n = 0;
  const int* c0_terms = LUMICE_CompositionClauseTerms(&oc, 0, &c0_n);
  ASSERT_NE(c0_terms, nullptr);
  EXPECT_EQ(c0_n, 2);
  EXPECT_EQ(c0_terms[0], 1);
  EXPECT_EQ(c0_terms[1], 2);
  int c1_n = 0;
  const int* c1_terms = LUMICE_CompositionClauseTerms(&oc, 1, &c1_n);
  ASSERT_NE(c1_terms, nullptr);
  EXPECT_EQ(c1_n, 1);
  EXPECT_EQ(c1_terms[0], 1);
}

TEST(StructFilterComplex, CommitStructEndToEnd) {
  // Build the complex config via parse, commit through the struct path -> core consumes it.
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2 } } },
  });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
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

TEST(StructFilterComplex, ComplexFilterCommitReusesOnNonRendererChange) {
  // The mechanism behind the 327.4 jank fix: a config carrying a COMPLEX filter can reuse
  // consumers (out_reused == 1) on a subsequent non-renderer change, so the live-preview
  // buffer is NOT torn. Before, GUI multi-segment/multi-value (complex) filters were forced
  // onto the JSON commit path, which has no out_reused signal and always rebuilt — tearing
  // the buffer on every crystal/sun slider drag. Routing complex through the typed struct
  // (327.3/327.4) restores the reuse signal. The GUI-integration timing (poller not stopped
  // on drag) is verified on-screen; this locks the underlying core contract headlessly.
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2 } } },
  });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  cfg.infinite = 0;
  cfg.ray_num = 100;

  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  ASSERT_EQ(LUMICE_CommitConfigStruct(server, &cfg, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0);  // first commit builds consumers

  // Change only a non-renderer field (sun altitude); the complex filter is unchanged.
  cfg.sun_altitude += 5.0f;
  reused = -1;
  ASSERT_EQ(LUMICE_CommitConfigStruct(server, &cfg, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 1);  // consumers reused despite the complex filter -> buffer not torn

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

TEST(StructFilterComplex, DanglingReferenceRejected) {
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 99 } } },  // 99 not defined
  });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
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
  lumice::ConfigOwningGuard cfg_guard(cfg);
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
  lumice::ConfigOwningGuard cfg_guard(cfg);
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
  lumice::ConfigOwningGuard cfg_guard(cfg);
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
  lumice::ConfigOwningGuard cfg_guard(cfg);
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, EmptyCompositionAccepted) {
  // An empty composition ([]) is a degenerate but accepted "OR of nothing" (clause_count 0).
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", nlohmann::json::array() } },
  });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_GE(cfg.filter_count, 2);
  EXPECT_EQ(cfg.filters[1].type, LUMICE_FILTER_TYPE_COMPLEX);
  EXPECT_EQ(cfg.compositions[cfg.filters[1].composition_index].clause_count, 0);
}

TEST(StructFilterComplex, EmptyClauseWithinCompositionCommitEndToEnd) {
  // Regression (code-review-01/02, round 1+2, Major): LUMICE_CompositionSetClauses only
  // allocates term_ids when total_terms > 0, so a composition where every clause has 0 terms
  // (clause_count > 0, total_terms == 0) legitimately ends up with term_ids == nullptr while
  // term_counts stays allocated (non-null). Round 1 fixed LUMICE_CommitConfigStruct's read-side
  // check but left SetClauses's own entry-point null-check requiring term_ids unconditionally
  // non-null whenever clause_count > 0 — which rejected exactly this legitimate shape earlier,
  // at LUMICE_ParseConfigString time, for the two real production callers (JsonToComplexComposition
  // / ExpandFilterToStruct) whose std::vector<int> term_ids accumulator is left empty (and thus
  // .data() == nullptr) when total_terms == 0. Round 2 fixes SetClauses itself to only require
  // term_ids non-null once total_terms is known to be > 0, restoring pre-v4.9 JSON round-trip
  // behavior for this shape end-to-end (Parse -> Commit), covering both `[[]]` (one empty
  // clause) and `[[],[]]` (two empty clauses) per round 1's original ask.
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 },
        { "action", "filter_in" },
        { "type", "complex" },
        { "composition", nlohmann::json::array({ nlohmann::json::array() }) } },  // [[]]
      { { "id", 3 },
        { "action", "filter_in" },
        { "type", "complex" },
        { "composition", nlohmann::json::array({ nlohmann::json::array(), nlohmann::json::array() }) } },  // [[],[]]
  });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_GE(cfg.filter_count, 3);
  EXPECT_EQ(cfg.filters[1].type, LUMICE_FILTER_TYPE_COMPLEX);
  EXPECT_EQ(cfg.filters[2].type, LUMICE_FILTER_TYPE_COMPLEX);

  const auto& comp1 = cfg.compositions[cfg.filters[1].composition_index];
  EXPECT_EQ(comp1.clause_count, 1);
  EXPECT_NE(comp1.term_counts, nullptr);
  EXPECT_EQ(comp1.term_counts[0], 0);
  EXPECT_EQ(comp1.term_ids, nullptr);

  const auto& comp2 = cfg.compositions[cfg.filters[2].composition_index];
  EXPECT_EQ(comp2.clause_count, 2);
  EXPECT_NE(comp2.term_counts, nullptr);
  EXPECT_EQ(comp2.term_counts[0], 0);
  EXPECT_EQ(comp2.term_counts[1], 0);
  EXPECT_EQ(comp2.term_ids, nullptr);

  cfg.infinite = 0;
  cfg.ray_num = 100;
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(server, &cfg, &reused), LUMICE_OK);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// task-host-abi-cpu-caps AC1 (§4 Step 8): a `complex` filter that OR's N > 16 real raypath
// simple filters (well past the pre-v4.9 LUMICE_MAX_CONFIG_CLAUSES=16 cap) survives the full
// C-API round-trip (JSON parse ↔ ConfigToJson emit ↔ core to_json cross-check). Uses the same
// ExpectComplexMatchesCore helper as OrOfRaypathsMatchesCore, so failure would flag any
// difference between the new pointer-storage emit and core's to_json output byte for byte.
TEST(StructFilterComplex, ManyOrClausesMatchesCore) {
  constexpr int kN = 200;  // >> old cap 16, well under new cap 4096
  nlohmann::json filters = nlohmann::json::array();
  nlohmann::json comp = nlohmann::json::array();
  for (int i = 1; i <= kN; i++) {
    // Distinct real raypaths — a mix of 1-face and 2-face patterns; face numbers cycle
    // over the valid prism range [1, 8] so each generated raypath is a legal simple filter.
    int f0 = 1 + ((i - 1) % 8);
    int f1 = 1 + (i % 8);
    filters.push_back({ { "id", i }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { f0, f1 } } });
    comp.push_back(i);
  }
  const int complex_id = kN + 1;
  filters.push_back(
      { { "id", complex_id }, { "action", "filter_in" }, { "type", "complex" }, { "composition", comp } });
  // Cross-check: core's own to_json vs C-API round-trip must agree even at 200 OR clauses.
  ExpectComplexMatchesCore(filters, complex_id);
}

// task-host-abi-cpu-caps AC1 end-to-end (§4 Step 8): the same N > 16 OR-clauses complex filter
// commits successfully to a real server through the struct path (Config → C ABI →
// core ConfigManager). If the pre-v4.9 clause-count-16 cap silently truncated, the composition
// would round-trip with fewer clauses and either LUMICE_ERR_INVALID_CONFIG or an incorrect
// commit would surface here.
TEST(StructFilterComplex, ManyOrClausesCommitStructEndToEnd) {
  constexpr int kN = 200;
  nlohmann::json filters = nlohmann::json::array();
  nlohmann::json comp = nlohmann::json::array();
  for (int i = 1; i <= kN; i++) {
    int f0 = 1 + ((i - 1) % 8);
    int f1 = 1 + (i % 8);
    filters.push_back({ { "id", i }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { f0, f1 } } });
    comp.push_back(i);
  }
  const int complex_id = kN + 1;
  filters.push_back(
      { { "id", complex_id }, { "action", "filter_in" }, { "type", "complex" }, { "composition", comp } });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.compositions[cfg.filters[kN].composition_index].clause_count, kN);
  cfg.infinite = 0;
  cfg.ray_num = 100;
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(server, &cfg, &reused), LUMICE_OK);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// task-host-abi-cpu-caps §7 risk 2 regression (§4 Step 8): parse two different compositions
// into the SAME LUMICE_Config in sequence; the second parse must fully replace the first
// (correct clause_count, correct term ids, no residual state from the first). This is the
// direct regression witness for the double-free / leak hazard introduced by making
// LUMICE_ComplexComposition an owning type — the create-or-replace contract in
// LUMICE_CompositionSetClauses + release-before-memset in JsonToConfig together ensure the
// first parse's heap allocations are released before the second parse overwrites the pointers.
// Value-semantics only (mirrors ConsecutiveParseIntoSameConfigOverridesCorrectly);
// leak detection itself lives outside this test (valgrind / asan, plan §7 risk 2).
TEST(StructFilterComplex, ConsecutiveParseIntoSameCompositionOverridesCorrectly) {
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);

  // First parse: a 3-clause OR of raypaths.
  nlohmann::json first_filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 6, 7 } } },
      { { "id", 10 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2, 3 } } },
  });
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(first_filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.composition_count, 1);
  ASSERT_EQ(cfg.compositions[0].clause_count, 3);
  ASSERT_NE(cfg.compositions[0].term_ids, nullptr);
  ASSERT_NE(cfg.compositions[0].term_counts, nullptr);

  // Second parse into the SAME cfg: a single-clause AND (different clause_count AND different
  // term shape). Verifies the CREATE-OR-REPLACE contract on both LUMICE_Config-level Release
  // (via JsonToConfig) and per-record LUMICE_CompositionSetClauses (via JsonToComplexComposition).
  nlohmann::json second_filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 20 },
        { "action", "filter_in" },
        { "type", "complex" },
        { "composition", nlohmann::json::array({ nlohmann::json::array({ 1, 2 }) }) } },
  });
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithFiltersJson(second_filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.composition_count, 1);
  EXPECT_EQ(cfg.compositions[0].clause_count, 1);
  int c0_n = 0;
  const int* c0_terms = LUMICE_CompositionClauseTerms(&cfg.compositions[0], 0, &c0_n);
  ASSERT_NE(c0_terms, nullptr);
  EXPECT_EQ(c0_n, 2);
  EXPECT_EQ(c0_terms[0], 1);
  EXPECT_EQ(c0_terms[1], 2);
}

// =====================================================================================
// Raypath Color Classes (task-342.2, Design 2) — C-API emit / parse / setter tests.
// Emit tests call ConfigToJson directly (server/c_api_internal.hpp) and assert the JSON
// field by field. Parse tests round-trip JSON -> LUMICE_Config. The setter (AC2/AC3) and
// the JSON-vs-struct pixel equivalence (AC1) drive a real server.
// =====================================================================================

namespace {

// Minimal LUMICE_Config carrying exactly one color class, for ConfigToJson emit assertions.
// task-344: `out` must be a caller-owned LUMICE_Config with a lifetime-bound
// lumice::ConfigOwningGuard already attached. This function allocates raypath_color via
// LUMICE_ConfigCreateColorClasses on `out` — the guard's destructor releases it.
void FillOneColorClassConfig(LUMICE_Config* out, const LUMICE_ColorClass& cls, int mode = LUMICE_COLOR_MODE_DOMINANT) {
  LUMICE_ColorClass* classes = LUMICE_ConfigCreateColorClasses(out, 1);
  ASSERT_NE(classes, nullptr);
  classes[0] = cls;
  out->raypath_color_mode = mode;
}

// A whole-crystal (match-all) class on {layer 0, crystal 1}: predicate zero-init => UNSET =>
// match-all. Red, visible, combine=any — all C-API/core defaults except color.
LUMICE_ColorClass MakeWholeCrystalClass() {
  LUMICE_ColorClass cls{};
  cls.color[0] = 1.0f;
  cls.combine = LUMICE_COLOR_COMBINE_ANY;
  cls.visible = 1;
  cls.solo = 0;
  cls.match_count = 1;
  cls.match[0].layer = 0;
  cls.match[0].crystal = 1;
  return cls;
}

const nlohmann::json& EmitFirstColorClass(const nlohmann::json& root) {
  return root.at("raypath_color").at("classes").at(0);
}

std::set<std::string> JsonKeySet(const nlohmann::json& j) {
  std::set<std::string> ks;
  for (auto it = j.begin(); it != j.end(); ++it) {
    ks.insert(it.key());
  }
  return ks;
}

nlohmann::json ColorRefJson(int layer, int crystal) {
  nlohmann::json r;
  r["layer"] = layer;
  r["crystal"] = crystal;
  return r;
}

nlohmann::json ColorClassJson(std::vector<float> rgb, nlohmann::json match) {
  nlohmann::json c;
  c["color"] = rgb;
  c["match"] = std::move(match);
  return c;
}

// Full config JSON (crystal 1, one scattering layer entry crystal 1) with a raypath_color
// section attached — the base sim actually tags surviving rays so a composite is produced.
std::string FullConfigWithRaypathColorJson(const nlohmann::json& rc) {
  auto root = nlohmann::json::parse(MakeSmallSimConfigJson());
  root["raypath_color"] = rc;
  return root.dump();
}

// Two-class color sim config: class0 = red whole-crystal (match-all, always fires),
// class1 = green {crystal 1, entry_exit min_len>=3} (a subset). Guarantees red pixels.
std::string MakeColorSimConfigJson() {
  nlohmann::json rc;
  rc["mode"] = "dominant";
  nlohmann::json c0 = ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json r1 = ColorRefJson(0, 1);
  r1["type"] = "entry_exit";
  r1["min_len"] = 3;
  nlohmann::json c1 = ColorClassJson({ 0.0f, 1.0f, 0.0f }, nlohmann::json::array({ r1 }));
  rc["classes"] = nlohmann::json::array({ c0, c1 });
  return FullConfigWithRaypathColorJson(rc);
}

// AC1 sibling of MakeColorSimConfigJson: same shape but class1's entry_exit ref carries
// symmetry="P". If either commit path drops the symmetry bit during struct<->JSON round-trip,
// the two composites diverge — this is the pixel-level AC1 pin for the new field.
std::string MakeColorSimConfigWithSymmetryJson() {
  nlohmann::json rc;
  rc["mode"] = "dominant";
  nlohmann::json c0 = ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json r1 = ColorRefJson(0, 1);
  r1["type"] = "entry_exit";
  r1["min_len"] = 3;
  r1["symmetry"] = "P";  // non-default: prism symmetry expansion for the entry_exit predicate
  nlohmann::json c1 = ColorClassJson({ 0.0f, 1.0f, 0.0f }, nlohmann::json::array({ r1 }));
  rc["classes"] = nlohmann::json::array({ c0, c1 });
  return FullConfigWithRaypathColorJson(rc);
}

}  // namespace

// ---- Emit shape (StructColorClassEmit) ----

TEST(StructColorClassEmit, MatchAllDefaultOmitsPredicateType) {
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, MakeWholeCrystalClass());
  auto root = ConfigToJson(config);
  ASSERT_TRUE(root.contains("raypath_color"));
  EXPECT_EQ(root.at("raypath_color").at("mode").get<std::string>(), "dominant");
  const auto& jc = EmitFirstColorClass(root);
  // Defaults (combine=any, visible=true, solo=false) omitted; only color + match present.
  EXPECT_EQ(JsonKeySet(jc), (std::set<std::string>{ "color", "match" }));
  const auto& ref = jc.at("match").at(0);
  // UNSET predicate => match-all => NO "type" key (only layer/crystal on the wire).
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal" }));
  EXPECT_EQ(ref.at("layer").get<int>(), 0);
  EXPECT_EQ(ref.at("crystal").get<int>(), 1);
}

TEST(StructColorClassEmit, RaypathPredicate) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_RAYPATH;
  cls.match[0].predicate.raypath_count = 2;
  cls.match[0].predicate.raypath[0] = 3;
  cls.match[0].predicate.raypath[1] = 5;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "type", "raypath" }));
  EXPECT_EQ(ref.at("type").get<std::string>(), "raypath");
  ASSERT_EQ(ref.at("raypath").size(), 2u);
  EXPECT_EQ(ref.at("raypath")[0].get<int>(), 3);
  EXPECT_EQ(ref.at("raypath")[1].get<int>(), 5);
}

TEST(StructColorClassEmit, EntryExitPredicateAllFields) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  cls.match[0].predicate.ee_entry = 3;
  cls.match[0].predicate.ee_exit = 5;
  cls.match[0].predicate.ee_min_len = 2;
  cls.match[0].predicate.ee_max_len = 8;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref),
            (std::set<std::string>{ "layer", "crystal", "type", "entry", "exit", "min_len", "max_len" }));
  EXPECT_EQ(ref.at("type").get<std::string>(), "entry_exit");
  EXPECT_EQ(ref.at("entry").get<int>(), 3);
  EXPECT_EQ(ref.at("exit").get<int>(), 5);
  EXPECT_EQ(ref.at("min_len").get<int>(), 2);
  EXPECT_EQ(ref.at("max_len").get<int>(), 8);
}

TEST(StructColorClassEmit, EntryExitWildcardsOmitted) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_ENTRY_EXIT;
  cls.match[0].predicate.ee_entry = -1;
  cls.match[0].predicate.ee_exit = -1;
  cls.match[0].predicate.ee_min_len = 1;
  cls.match[0].predicate.ee_max_len = -1;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "type" }));
  EXPECT_EQ(ref.at("type").get<std::string>(), "entry_exit");
}

TEST(StructColorClassEmit, DirectionPredicate) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_DIRECTION;
  cls.match[0].predicate.dir_az = 120.0f;
  cls.match[0].predicate.dir_el = -15.0f;
  cls.match[0].predicate.dir_radii = 2.5f;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "type", "az", "el", "radii" }));
  EXPECT_EQ(ref.at("type").get<std::string>(), "direction");
  EXPECT_FLOAT_EQ(ref.at("az").get<float>(), 120.0f);
  EXPECT_FLOAT_EQ(ref.at("el").get<float>(), -15.0f);
  EXPECT_FLOAT_EQ(ref.at("radii").get<float>(), 2.5f);
}

TEST(StructColorClassEmit, CrystalPredicate) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_CRYSTAL;
  cls.match[0].predicate.crystal_id = 7;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "type", "crystal_id" }));
  EXPECT_EQ(ref.at("type").get<std::string>(), "crystal");
  EXPECT_EQ(ref.at("crystal_id").get<int>(), 7);
}

TEST(StructColorClassEmit, CombineAllVisibleFalseSoloTrueEmitted) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.combine = LUMICE_COLOR_COMBINE_ALL;
  cls.visible = 0;  // A4: zero-init default would also be 0 (invisible)
  cls.solo = 1;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& jc = EmitFirstColorClass(root);
  EXPECT_EQ(jc.at("combine").get<std::string>(), "all");
  EXPECT_FALSE(jc.at("visible").get<bool>());
  EXPECT_TRUE(jc.at("solo").get<bool>());
}

TEST(StructColorClassEmit, ZeroInitClassIsInvisible) {
  // A4 regression: LUMICE_ColorClass{} zero-inits visible=0, which the emitter writes as
  // "visible":false (OPPOSITE of core's default true). Long-term guard for the GUI trap.
  LUMICE_ColorClass cls{};
  cls.match_count = 1;
  cls.match[0].crystal = 1;
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& jc = EmitFirstColorClass(root);
  ASSERT_TRUE(jc.contains("visible"));
  EXPECT_FALSE(jc.at("visible").get<bool>());
}

TEST(StructColorClassEmit, ZeroCountOmitsKey) {
  // AC4: no color classes => no "raypath_color" key => JSON byte-shape identical to pre-v4.7.
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  config.raypath_color_count = 0;
  auto root = ConfigToJson(config);
  EXPECT_FALSE(root.contains("raypath_color"));
}

TEST(StructColorClassEmit, ModeAdditiveAndPainterEmitted) {
  {
    LUMICE_Config config{};
    lumice::ConfigOwningGuard config_guard(config);
    FillOneColorClassConfig(&config, MakeWholeCrystalClass(), LUMICE_COLOR_MODE_ADDITIVE);
    auto add = ConfigToJson(config);
    EXPECT_EQ(add.at("raypath_color").at("mode").get<std::string>(), "additive");
  }
  {
    LUMICE_Config config{};
    lumice::ConfigOwningGuard config_guard(config);
    FillOneColorClassConfig(&config, MakeWholeCrystalClass(), LUMICE_COLOR_MODE_PAINTER);
    auto pnt = ConfigToJson(config);
    EXPECT_EQ(pnt.at("raypath_color").at("mode").get<std::string>(), "painter");
  }
}

// task-356.2: symmetry emit — single bit, combined bits, match-all + symmetry, default omission.
TEST(StructColorClassEmit, SymmetrySingleBitEmitted) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_RAYPATH;
  cls.match[0].predicate.raypath_count = 1;
  cls.match[0].predicate.raypath[0] = 3;
  cls.match[0].predicate.symmetry = 1;  // P
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "type", "raypath", "symmetry" }));
  EXPECT_EQ(ref.at("symmetry").get<std::string>(), "P");
}

TEST(StructColorClassEmit, SymmetryCombinedBitsEmitted) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_CRYSTAL;
  cls.match[0].predicate.crystal_id = 1;
  cls.match[0].predicate.symmetry = 1 | 2 | 4;  // PBD
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(ref.at("symmetry").get<std::string>(), "PBD");
}

TEST(StructColorClassEmit, SymmetryWithMatchAll) {
  // Match-all (UNSET) + non-default symmetry: legal state — arm fields omitted, symmetry emitted.
  // Direct pin on the Step 3 emit refactor (each arm's early `return` became `break` so the
  // shared symmetry-emit tail is reached from UNSET too).
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_UNSET;
  cls.match[0].predicate.symmetry = 2;  // B
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "symmetry" }));
  EXPECT_EQ(ref.at("symmetry").get<std::string>(), "B");
}

// AC3 pin: symmetry == 0 must NOT produce a "symmetry" key (byte-identical wire vs pre-v4.9).
// StructColorClassEmit.MatchAllDefaultOmitsPredicateType covers this for UNSET; here we pin
// it explicitly for a typed arm (raypath).
TEST(StructColorClassEmit, SymmetryOmittedWhenDefaultOnTypedArm) {
  LUMICE_ColorClass cls = MakeWholeCrystalClass();
  cls.match[0].predicate.type = LUMICE_FILTER_TYPE_RAYPATH;
  cls.match[0].predicate.raypath_count = 1;
  cls.match[0].predicate.raypath[0] = 3;
  // cls.match[0].predicate.symmetry left at 0 (zero-init).
  LUMICE_Config config{};
  lumice::ConfigOwningGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& ref = EmitFirstColorClass(root).at("match").at(0);
  EXPECT_EQ(JsonKeySet(ref), (std::set<std::string>{ "layer", "crystal", "type", "raypath" }));
  EXPECT_FALSE(ref.contains("symmetry"));
}

// ---- Emit/parse isomorphism cross-checked against core (StructColorClassEmitIsomorphism) ----

TEST(StructColorClassEmitIsomorphism, RoundTripThroughCore) {
  // Exercise match-all + entry_exit(min/max) + raypath arms and non-default class fields.
  // Read BOTH the source JSON and the C-API-reemitted JSON with core's from_json, then
  // compare their canonical to_json — semantic isomorphism robust to the array/object shape.
  nlohmann::json c0 = ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json ee = ColorRefJson(0, 1);
  ee["type"] = "entry_exit";
  ee["min_len"] = 2;
  ee["max_len"] = 2;
  ee["symmetry"] = "PD";  // task-356.2: cover symmetry survives struct<->JSON via core canonical form
  nlohmann::json rp = ColorRefJson(1, 2);
  rp["type"] = "raypath";
  rp["raypath"] = { 3, 5 };
  rp["symmetry"] = "B";
  nlohmann::json c1 = ColorClassJson({ 0.0f, 1.0f, 0.0f }, nlohmann::json::array({ ee, rp }));
  c1["combine"] = "all";
  c1["visible"] = false;
  c1["solo"] = true;
  nlohmann::json rc;
  rc["mode"] = "additive";
  rc["classes"] = nlohmann::json::array({ c0, c1 });

  lumice::RaypathColorConfig from_src = rc.get<lumice::RaypathColorConfig>();
  nlohmann::json core_src = from_src;  // core canonical form

  LUMICE_Config cfg{};

  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 2);
  ASSERT_EQ(cfg.raypath_color_mode, LUMICE_COLOR_MODE_ADDITIVE);
  auto emitted = ConfigToJson(cfg).at("raypath_color");
  lumice::RaypathColorConfig from_emit = emitted.get<lumice::RaypathColorConfig>();
  nlohmann::json core_emit = from_emit;

  EXPECT_EQ(core_src, core_emit) << "C-API emit:\n" << core_emit.dump(2) << "\ncore:\n" << core_src.dump(2);
}

// ---- Parse (ParseConfigApi raypath_color) ----

TEST(ParseConfigApi, RaypathColorArrayFormParsed) {
  nlohmann::json rc =
      nlohmann::json::array({ ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) })) });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  EXPECT_EQ(cfg.raypath_color_count, 1);
  // doc §4.8: bare-array wire form now
  // defaults to painter — kept in lockstep with core RaypathColorConfig::from_json.
  EXPECT_EQ(cfg.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER);
  ASSERT_EQ(cfg.raypath_color[0].match_count, 1);
  EXPECT_EQ(cfg.raypath_color[0].match[0].crystal, 1);
  EXPECT_EQ(cfg.raypath_color[0].match[0].predicate.type, LUMICE_FILTER_TYPE_UNSET);  // match-all
  EXPECT_EQ(cfg.raypath_color[0].visible, 1);  // core default true, not zero-init
  EXPECT_EQ(cfg.raypath_color[0].combine, LUMICE_COLOR_COMBINE_ANY);
}

TEST(ParseConfigApi, RaypathColorObjectFormParsed) {
  nlohmann::json rc;
  rc["mode"] = "painter";
  nlohmann::json c0 = ColorClassJson({ 0.2f, 0.4f, 0.6f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  c0["combine"] = "all";
  c0["visible"] = false;
  c0["solo"] = true;
  rc["classes"] = nlohmann::json::array({ c0 });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  EXPECT_EQ(cfg.raypath_color_count, 1);
  EXPECT_EQ(cfg.raypath_color_mode, LUMICE_COLOR_MODE_PAINTER);
  EXPECT_EQ(cfg.raypath_color[0].combine, LUMICE_COLOR_COMBINE_ALL);
  EXPECT_EQ(cfg.raypath_color[0].visible, 0);
  EXPECT_EQ(cfg.raypath_color[0].solo, 1);
  EXPECT_FLOAT_EQ(cfg.raypath_color[0].color[2], 0.6f);
}

TEST(ParseConfigApi, RaypathColorPredicateArms) {
  nlohmann::json ee = ColorRefJson(0, 1);
  ee["type"] = "entry_exit";
  ee["min_len"] = 2;
  ee["max_len"] = 4;
  nlohmann::json dir = ColorRefJson(0, 1);
  dir["type"] = "direction";
  dir["az"] = 22.0f;
  dir["el"] = 33.0f;
  dir["radii"] = 4.0f;
  nlohmann::json cry = ColorRefJson(0, 1);
  cry["type"] = "crystal";
  cry["crystal_id"] = 9;
  nlohmann::json rc = nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ ee, dir, cry })) });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 1);
  ASSERT_EQ(cfg.raypath_color[0].match_count, 3);
  const auto& m = cfg.raypath_color[0].match;
  EXPECT_EQ(m[0].predicate.type, LUMICE_FILTER_TYPE_ENTRY_EXIT);
  EXPECT_EQ(m[0].predicate.ee_min_len, 2);
  EXPECT_EQ(m[0].predicate.ee_max_len, 4);
  EXPECT_EQ(m[1].predicate.type, LUMICE_FILTER_TYPE_DIRECTION);
  EXPECT_FLOAT_EQ(m[1].predicate.dir_az, 22.0f);
  EXPECT_EQ(m[2].predicate.type, LUMICE_FILTER_TYPE_CRYSTAL);
  EXPECT_EQ(m[2].predicate.crystal_id, 9);
}

// task-356.2 AC2 parse side: symmetry parses correctly on every named arm (raypath /
// entry_exit / direction / crystal) and on match-all (UNSET). Direct pin on Step 3
// parse refactor — each named arm falls through to the shared symmetry tail.
TEST(ParseConfigApi, RaypathColorPredicateSymmetryParsed) {
  nlohmann::json rp = ColorRefJson(0, 1);
  rp["type"] = "raypath";
  rp["raypath"] = { 3, 5 };
  rp["symmetry"] = "PB";  // bits 1|2 = 3
  nlohmann::json ee = ColorRefJson(0, 1);
  ee["type"] = "entry_exit";
  ee["min_len"] = 2;
  ee["symmetry"] = "P";  // bit 1
  nlohmann::json dir = ColorRefJson(0, 1);
  dir["type"] = "direction";
  dir["az"] = 22.0f;
  dir["el"] = 33.0f;
  dir["radii"] = 4.0f;
  dir["symmetry"] = "B";  // bit 2
  nlohmann::json cry = ColorRefJson(0, 1);
  cry["type"] = "crystal";
  cry["crystal_id"] = 9;
  cry["symmetry"] = "PBD";  // bits 1|2|4 = 7
  // Match-all + symmetry (no "type" key, non-default symmetry).
  nlohmann::json ma = ColorRefJson(0, 1);
  ma["symmetry"] = "D";  // bit 4
  nlohmann::json rc =
      nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ rp, ee, dir, cry, ma })) });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 1);
  ASSERT_EQ(cfg.raypath_color[0].match_count, 5);
  const auto& m = cfg.raypath_color[0].match;
  EXPECT_EQ(m[0].predicate.type, LUMICE_FILTER_TYPE_RAYPATH);
  EXPECT_EQ(m[0].predicate.symmetry, 3);
  EXPECT_EQ(m[1].predicate.type, LUMICE_FILTER_TYPE_ENTRY_EXIT);
  EXPECT_EQ(m[1].predicate.symmetry, 1);
  EXPECT_EQ(m[2].predicate.type, LUMICE_FILTER_TYPE_DIRECTION);
  EXPECT_EQ(m[2].predicate.symmetry, 2);
  EXPECT_EQ(m[3].predicate.type, LUMICE_FILTER_TYPE_CRYSTAL);
  EXPECT_EQ(m[3].predicate.symmetry, 7);
  // Match-all (no "type" key) still gets symmetry parsed from the shared tail.
  EXPECT_EQ(m[4].predicate.type, LUMICE_FILTER_TYPE_UNSET);
  EXPECT_EQ(m[4].predicate.symmetry, 4);
}

// task-356.2 AC3 zero-regression: missing "symmetry" key must yield symmetry == 0 on every
// arm (including match-all). Pins the "default omission" side of the parse round-trip.
TEST(ParseConfigApi, RaypathColorPredicateSymmetryOmittedDefaultsToZero) {
  nlohmann::json rp = ColorRefJson(0, 1);
  rp["type"] = "raypath";
  rp["raypath"] = { 3, 5 };
  nlohmann::json ma = ColorRefJson(0, 1);
  nlohmann::json rc = nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ rp, ma })) });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color[0].match_count, 2);
  EXPECT_EQ(cfg.raypath_color[0].match[0].predicate.symmetry, 0);
  EXPECT_EQ(cfg.raypath_color[0].match[1].predicate.symmetry, 0);
}

TEST(ParseConfigApi, RaypathColorComplexPredicateRejected) {
  nlohmann::json bad = ColorRefJson(0, 1);
  bad["type"] = "complex";
  nlohmann::json rc = nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ bad })) });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_ERR_INVALID_VALUE);
}

TEST(ParseConfigApi, RaypathColorClassesOverCapRejected) {
  nlohmann::json classes = nlohmann::json::array();
  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_CLASSES + 1; i++) {
    classes.push_back(ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ ColorRefJson(0, 1) })));
  }
  nlohmann::json rc;
  rc["classes"] = classes;
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(ParseConfigApi, RaypathColorRefsOverCapRejected) {
  nlohmann::json match = nlohmann::json::array();
  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_REFS + 1; i++) {
    match.push_back(ColorRefJson(0, 1));
  }
  nlohmann::json rc = nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, match) });
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  EXPECT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

// ---- Server-driven: AC1 (pixel equivalence), AC2 (no restart), AC3 (rejection) ----

namespace {
// Index of the brightest-red pixel (max R) in an sRGB buffer, or -1 if none lit.
int BrightestRedPixel(const uint8_t* buf, int w, int h) {
  int best = -1;
  int best_r = 0;
  for (int p = 0; p < w * h; p++) {
    const int r = buf[p * 3 + 0];
    if (r > best_r) {
      best_r = r;
      best = p;
    }
  }
  return best;
}
}  // namespace

TEST(RaypathColorApi, JsonAndStructCommitPixelEquivalent) {
  // AC1: the same config committed via JSON string vs via LUMICE_ParseConfigString +
  // LUMICE_CommitConfigStruct must produce a byte-identical composite (fixed seed).
  const std::string json = MakeColorSimConfigJson();

  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 12345u;

  LUMICE_Server* a = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(a, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(a, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(a, 10000));

  LUMICE_Server* b = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(b, nullptr);
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(json.c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 2);
  ASSERT_EQ(LUMICE_CommitConfigStruct(b, &cfg, nullptr), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(b, 10000));

  LUMICE_RenderResult oa[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult ob[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(a, oa, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(oa[0].img_buffer, nullptr);
  // Copy A's pixels before B's Get* call (composite buffer only valid until the next Get*).
  const int w = oa[0].img_width;
  const int h = oa[0].img_height;
  const size_t nbytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
  std::vector<uint8_t> a_px(oa[0].img_buffer, oa[0].img_buffer + nbytes);

  ASSERT_EQ(LUMICE_GetCompositeResults(b, ob, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(ob[0].img_buffer, nullptr);
  ASSERT_EQ(ob[0].img_width, w);
  ASSERT_EQ(ob[0].img_height, h);

  // Sanity: composite is non-trivial (class0 is match-all red, so there ARE lit pixels).
  uint64_t sum = 0;
  for (uint8_t v : a_px) {
    sum += v;
  }
  EXPECT_GT(sum, 0u) << "composite unexpectedly all-black — equivalence would be vacuous";

  EXPECT_EQ(std::memcmp(a_px.data(), ob[0].img_buffer, nbytes), 0)
      << "JSON-commit and struct-commit composites must be byte-identical";

  LUMICE_StopServer(a);
  LUMICE_DestroyServer(a);
  LUMICE_StopServer(b);
  LUMICE_DestroyServer(b);
}

// task-356.2 AC1 (硬约束) with symmetry: same shape as JsonAndStructCommitPixelEquivalent
// but the entry_exit ref carries symmetry="P". If either commit path drops the symmetry bit,
// the two composites diverge — pixel-level pin that the new field survives struct<->JSON.
TEST(RaypathColorApi, JsonAndStructCommitPixelEquivalentWithSymmetry) {
  const std::string json = MakeColorSimConfigWithSymmetryJson();

  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 12345u;

  LUMICE_Server* a = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(a, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(a, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(a, 10000));

  LUMICE_Server* b = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(b, nullptr);
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  ASSERT_EQ(LUMICE_ParseConfigString(json.c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 2);
  // Direct assertion that the struct path carries the symmetry bit (redundant with the
  // parse-side test but pins this fixture's precondition — a struct with symmetry=0 here
  // would make the "byte-identical" check vacuous).
  ASSERT_EQ(cfg.raypath_color[1].match_count, 1);
  ASSERT_EQ(cfg.raypath_color[1].match[0].predicate.symmetry, 1);  // "P"
  ASSERT_EQ(LUMICE_CommitConfigStruct(b, &cfg, nullptr), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(b, 10000));

  LUMICE_RenderResult oa[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult ob[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(a, oa, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(oa[0].img_buffer, nullptr);
  const int w = oa[0].img_width;
  const int h = oa[0].img_height;
  const size_t nbytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
  std::vector<uint8_t> a_px(oa[0].img_buffer, oa[0].img_buffer + nbytes);

  ASSERT_EQ(LUMICE_GetCompositeResults(b, ob, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(ob[0].img_buffer, nullptr);
  ASSERT_EQ(ob[0].img_width, w);
  ASSERT_EQ(ob[0].img_height, h);

  uint64_t sum = 0;
  for (uint8_t v : a_px) {
    sum += v;
  }
  EXPECT_GT(sum, 0u) << "composite unexpectedly all-black — equivalence would be vacuous";

  EXPECT_EQ(std::memcmp(a_px.data(), ob[0].img_buffer, nbytes), 0)
      << "JSON-commit and struct-commit composites must be byte-identical (with symmetry='P')";

  LUMICE_StopServer(a);
  LUMICE_DestroyServer(a);
  LUMICE_StopServer(b);
  LUMICE_DestroyServer(b);
}

TEST(RaypathColorApi, SetRaypathColorsDoesNotRestartSim) {
  // AC2: after reaching steady state, SetRaypathColors changes the composite but must NOT
  // advance epoch nor clear the accumulator, and the new color must actually show.
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 777u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_SimLifecycleResult lc0{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(s, &lc0), LUMICE_OK);
  LUMICE_RayCount rc0 = 0;
  ASSERT_EQ(LUMICE_GetSimRayCount(s, &rc0), LUMICE_OK);

  LUMICE_RenderResult before[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, before, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(before[0].img_buffer, nullptr);
  const int w = before[0].img_width;
  const int h = before[0].img_height;
  const size_t nbytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
  std::vector<uint8_t> before_px(before[0].img_buffer, before[0].img_buffer + nbytes);
  const int red_p = BrightestRedPixel(before_px.data(), w, h);
  ASSERT_GE(red_p, 0) << "no red pixel to recolor — class0 (match-all red) should light pixels";

  // Recolor class0 red->blue and class1 green->blue; both stay visible. dominant unchanged.
  LUMICE_ColorClassDisplay disp[2]{};
  disp[0].color[2] = 1.0f;  // class0 blue
  disp[0].visible = 1;
  disp[1].color[2] = 1.0f;  // class1 blue
  disp[1].visible = 1;
  ASSERT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

  // Epoch unchanged, accumulator not cleared.
  LUMICE_SimLifecycleResult lc1{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(s, &lc1), LUMICE_OK);
  EXPECT_EQ(lc1.epoch, lc0.epoch) << "SetRaypathColors must not advance epoch (no re-sim)";
  LUMICE_RayCount rc1 = 0;
  ASSERT_EQ(LUMICE_GetSimRayCount(s, &rc1), LUMICE_OK);
  EXPECT_GE(rc1, rc0) << "accumulator must not be cleared/reset";

  // New color shows: the formerly-red pixel is now blue.
  LUMICE_RenderResult after[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, after, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(after[0].img_buffer, nullptr);
  const uint8_t* ap = after[0].img_buffer;
  EXPECT_GT(ap[red_p * 3 + 2], 0) << "recolored pixel must now be blue";
  EXPECT_EQ(ap[red_p * 3 + 0], 0) << "recolored pixel must no longer be red";
  EXPECT_NE(std::memcmp(before_px.data(), ap, nbytes), 0) << "composite must actually change";

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// task-342.4 Step 1 regression: RawXyz then Composite in the same tick must
// both reflect the current data generation. Before the DoSnapshot()/GetRawXyz
// unification, RawXyz would consume snapshot_dirty_ first and Composite's
// DoSnapshot() would then early-return, leaving cached_composite_results_ stale
// (empty or last-tick's pixels). See plan §3 keypoint 1.
TEST(RaypathColorApi, RawXyzThenCompositeSeesFreshGeneration) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 4242u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Same-tick double consume: RawXyz first (previously the dirty-flag hog),
  // Composite second. Both must see this-generation results.
  LUMICE_RawXyzResult raw[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, raw, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(raw[0].xyz_buffer, nullptr);
  ASSERT_NE(raw[0].has_valid_data, 0);
  const unsigned long long gen_after_rawxyz = raw[0].snapshot_generation;
  EXPECT_GT(gen_after_rawxyz, 0ull) << "generation must advance on first consume";

  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp[0].img_buffer, nullptr) << "composite must be populated (class0 is match-all red)";
  const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
  uint64_t sum = 0;
  for (size_t i = 0; i < nbytes; ++i) {
    sum += comp[0].img_buffer[i];
  }
  EXPECT_GT(sum, 0u) << "composite must reflect the current-tick data, not an all-zero stale cache";

  // A second RawXyz call in the same tick must not regress the generation
  // (no new dirty snapshot arrived; both cached values are consistent).
  LUMICE_RawXyzResult raw2[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, raw2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_GE(raw2[0].snapshot_generation, gen_after_rawxyz);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// task-342.4 Step 1 regression: reverse order (Composite then RawXyz). Before
// the fix, Composite's DoSnapshot() would consume the dirty flag but NOT bump
// snapshot_generation_ (that lived only in RawXyz's Phase-1), so RawXyz would
// see generation stuck forever and the poller's has_new_snapshot check would
// permanently return false. See plan §3 keypoint 1.
TEST(RaypathColorApi, CompositeThenRawXyzGenerationStillAdvances) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 5151u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp[0].img_buffer, nullptr);

  LUMICE_RawXyzResult raw[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, raw, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(raw[0].xyz_buffer, nullptr);
  EXPECT_NE(raw[0].has_valid_data, 0) << "RawXyz must see the snapshot that Composite consumed";
  EXPECT_GT(raw[0].snapshot_generation, 0ull)
      << "generation must advance even when Composite consumed the dirty flag first";

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// code-review-02 Major 2 regression: deterministically reproduce the generation-drift
// scenario ServerPoller::PopulateCompositePayload() (server_poller.cpp) guards against — a
// Get*Results call landing strictly between an initial LUMICE_GetRawXyzResults() capture and
// a later re-check must be observable as a newer snapshot_generation, and that drift must be
// exactly what a "recheck != captured" comparison flags. Uses LUMICE_SetRaypathColors() to
// deterministically arm snapshot_dirty_ without depending on background-thread timing (AC2:
// it forces the next DoSnapshot() to re-run even without new ray data) — this is functionally
// identical to a background batch commit landing in that window, but reproducible on demand.
TEST(RaypathColorApi, CompositeGenerationDriftDetectableViaRecheck) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 8181u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Baseline (no drift armed yet): mirrors PopulateCompositePayload's happy path, where the
  // recheck observes the same generation that was captured before the composite call.
  LUMICE_RawXyzResult raw1[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, raw1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  const unsigned long long captured_xyz_generation = raw1[0].snapshot_generation;

  LUMICE_RenderResult comp1[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, comp1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp1[0].img_buffer, nullptr);
  const size_t nbytes = static_cast<size_t>(comp1[0].img_width) * static_cast<size_t>(comp1[0].img_height) * 3;
  std::vector<uint8_t> comp1_px(comp1[0].img_buffer, comp1[0].img_buffer + nbytes);

  LUMICE_RawXyzResult regen_check1[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, regen_check1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(regen_check1[0].snapshot_generation, captured_xyz_generation)
      << "no drift armed yet: recheck must observe the same generation that was captured before Composite";

  // Deterministically arm a new dirty event exactly in the window PopulateCompositePayload's
  // recheck is meant to catch: after xyz was "captured" above, but before the recheck below.
  LUMICE_ColorClassDisplay disp[2]{};
  disp[0].color[2] = 1.0f;  // class0 red->blue
  disp[0].visible = 1;
  disp[1].color[2] = 1.0f;  // class1 green->blue
  disp[1].visible = 1;
  ASSERT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

  // Plays the role of PollOnce()'s LUMICE_GetCompositeResults(): consumes the freshly-armed
  // dirty flag and materializes a generation newer than captured_xyz_generation.
  LUMICE_RenderResult comp2[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, comp2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp2[0].img_buffer, nullptr);
  EXPECT_NE(std::memcmp(comp1_px.data(), comp2[0].img_buffer, nbytes), 0)
      << "comp2 must reflect the recolor, proving it genuinely belongs to a newer generation "
         "than captured_xyz_generation rather than a stale cache hit";

  // Plays the role of PopulateCompositePayload's regen_check: must observe a generation newer
  // than captured_xyz_generation — exactly the condition PopulateCompositePayload uses to
  // decide whether to drop this tick's composite (server_poller.cpp).
  LUMICE_RawXyzResult regen_check2[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, regen_check2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_GT(regen_check2[0].snapshot_generation, captured_xyz_generation)
      << "drift must be detectable: recheck.snapshot_generation != captured_xyz_generation is "
         "the exact condition PopulateCompositePayload checks to drop a mismatched composite";

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// task-345.2 Step 2 regression: the atomic combined getter must pair xyz +
// composite from a SINGLE snapshot — i.e., composite_out[0] MUST belong to
// xyz_out[0].snapshot_generation, even when repeated churn arms a fresh dirty
// event between each call. Contrast with CompositeGenerationDriftDetectableViaRecheck
// above, which proves the OPPOSITE property for the three-call sequence
// (xyz → composite → recheck) that the fix eliminates from the poller path.
TEST(RaypathColorApi, RawXyzAndCompositeSameGenerationUnderChurn) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 1919u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  unsigned long long prev_generation = 0ull;
  for (int round = 0; round < 8; ++round) {
    // Arm a fresh dirty event before every combined call (deterministic
    // stand-in for background ConsumeData batch churn in the GUI hot path).
    LUMICE_ColorClassDisplay disp[2]{};
    // Rotate blue channel per round so every LUMICE_SetRaypathColors is a
    // genuine display-state change (never a no-op that skips setting dirty).
    disp[0].color[2] = static_cast<float>(round % 3) / 2.0f;
    disp[0].visible = 1;
    disp[1].color[2] = static_cast<float>((round + 1) % 3) / 2.0f;
    disp[1].visible = 1;
    ASSERT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

    LUMICE_RawXyzResult xyz[LUMICE_MAX_RENDER_RESULTS + 1]{};
    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    ASSERT_EQ(LUMICE_GetRawXyzAndCompositeResults(s, xyz, LUMICE_MAX_RENDER_RESULTS, comp, LUMICE_MAX_RENDER_RESULTS),
              LUMICE_OK)
        << "round " << round;
    ASSERT_NE(xyz[0].xyz_buffer, nullptr) << "round " << round;
    ASSERT_NE(comp[0].img_buffer, nullptr) << "round " << round << ": raypath_color is configured";

    // Same-generation invariant (structural property of one-DoSnapshot() call).
    // The composite that the getter returns MUST correspond to this call's xyz
    // snapshot_generation — no cross-generation mix is possible when there is
    // exactly ONE DoSnapshot() trigger inside the atomic call.
    EXPECT_GT(xyz[0].snapshot_generation, prev_generation)
        << "round " << round << ": SetRaypathColors armed dirty → generation must advance";
    prev_generation = xyz[0].snapshot_generation;

    // Cross-check: an immediate independent LUMICE_GetRawXyzResults() call
    // right after the combined call must observe the same generation
    // (nothing else bumps it in this thread), i.e. the combined call did
    // NOT leave the server in a "generation drift is armed for the next
    // observer" state.
    LUMICE_RawXyzResult recheck[LUMICE_MAX_RENDER_RESULTS + 1]{};
    ASSERT_EQ(LUMICE_GetRawXyzResults(s, recheck, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_EQ(recheck[0].snapshot_generation, xyz[0].snapshot_generation)
        << "round " << round << ": no drift armed between combined call and immediate recheck";
  }

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// The combined getter's xyz/composite outputs must byte-match what the two
// individual getters would return when called in the ordinary "no churn armed
// between them" case — proves the merged code path preserves the semantics of
// its two callees, not just adds a new atomic guarantee.
TEST(RaypathColorApi, RawXyzAndCompositeMatchesIndividualGettersNoChurn) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 2929u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_RawXyzResult xyz_c[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult comp_c[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzAndCompositeResults(s, xyz_c, LUMICE_MAX_RENDER_RESULTS, comp_c, LUMICE_MAX_RENDER_RESULTS),
            LUMICE_OK);
  ASSERT_NE(xyz_c[0].xyz_buffer, nullptr);
  ASSERT_NE(comp_c[0].img_buffer, nullptr);
  const int width = xyz_c[0].img_width;
  const int height = xyz_c[0].img_height;
  const size_t rgb_bytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  const size_t xyz_floats = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  std::vector<uint8_t> comp_copy(comp_c[0].img_buffer, comp_c[0].img_buffer + rgb_bytes);
  std::vector<float> xyz_copy(xyz_c[0].xyz_buffer, xyz_c[0].xyz_buffer + xyz_floats);
  const unsigned long long combined_generation = xyz_c[0].snapshot_generation;

  // Idle server → no dirty → the individual getters must land on the SAME
  // frozen snapshot the combined call just materialized (generation stable).
  LUMICE_RawXyzResult xyz_i[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetRawXyzResults(s, xyz_i, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(xyz_i[0].snapshot_generation, combined_generation);
  EXPECT_EQ(std::memcmp(xyz_i[0].xyz_buffer, xyz_copy.data(), xyz_floats * sizeof(float)), 0);

  LUMICE_RenderResult comp_i[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, comp_i, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp_i[0].img_buffer, nullptr);
  EXPECT_EQ(std::memcmp(comp_i[0].img_buffer, comp_copy.data(), rgb_bytes), 0);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// NULL-arg guard mirrors the two individual getters.
TEST(RaypathColorApi, RawXyzAndCompositeRejectsNullArgs) {
  LUMICE_Server* s = LUMICE_CreateServer();
  ASSERT_NE(s, nullptr);
  LUMICE_RawXyzResult xyz[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(
      LUMICE_GetRawXyzAndCompositeResults(nullptr, xyz, LUMICE_MAX_RENDER_RESULTS, comp, LUMICE_MAX_RENDER_RESULTS),
      LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetRawXyzAndCompositeResults(s, nullptr, LUMICE_MAX_RENDER_RESULTS, comp, LUMICE_MAX_RENDER_RESULTS),
            LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetRawXyzAndCompositeResults(s, xyz, LUMICE_MAX_RENDER_RESULTS, nullptr, LUMICE_MAX_RENDER_RESULTS),
            LUMICE_ERR_NULL_ARG);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, SetRaypathColorsRejectsBadArgsAllOrNothing) {
  // AC3: class_count mismatch and non-permutation z_order are rejected without mutating state.
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 999u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Capture the current composite to prove rejections leave it untouched.
  LUMICE_RenderResult before[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, before, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(before[0].img_buffer, nullptr);
  const size_t nbytes = static_cast<size_t>(before[0].img_width) * static_cast<size_t>(before[0].img_height) * 3;
  std::vector<uint8_t> before_px(before[0].img_buffer, before[0].img_buffer + nbytes);

  // count mismatch (active is 2). Pass a 1-element array + count 1 (no OOB read).
  LUMICE_ColorClassDisplay one[1]{};
  one[0].color[2] = 1.0f;
  one[0].visible = 1;
  EXPECT_EQ(LUMICE_SetRaypathColors(s, one, 1, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_ERR_INVALID_CONFIG);

  // Non-permutation z_order with matching count: duplicate rank {0,0} and out-of-range {0,2}.
  LUMICE_ColorClassDisplay two[2]{};
  two[0].color[2] = 1.0f;
  two[0].visible = 1;
  two[1].color[2] = 1.0f;
  two[1].visible = 1;
  const int dup[2] = { 0, 0 };
  EXPECT_EQ(LUMICE_SetRaypathColors(s, two, 2, dup, LUMICE_COLOR_MODE_DOMINANT), LUMICE_ERR_INVALID_CONFIG);
  const int oob[2] = { 0, 2 };
  EXPECT_EQ(LUMICE_SetRaypathColors(s, two, 2, oob, LUMICE_COLOR_MODE_DOMINANT), LUMICE_ERR_INVALID_CONFIG);

  // All rejections were all-or-nothing: composite unchanged (still red, not blue).
  LUMICE_RenderResult mid[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, mid, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(mid[0].img_buffer, nullptr);
  EXPECT_EQ(std::memcmp(before_px.data(), mid[0].img_buffer, nbytes), 0)
      << "rejected SetRaypathColors must not mutate the active table";

  // A valid permutation {1,0} succeeds.
  const int perm[2] = { 1, 0 };
  EXPECT_EQ(LUMICE_SetRaypathColors(s, two, 2, perm, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, SetRaypathColorsNullAndInvalidMode) {
  LUMICE_ColorClassDisplay disp[2]{};
  disp[0].visible = 1;
  disp[1].visible = 1;
  // Null server.
  EXPECT_EQ(LUMICE_SetRaypathColors(nullptr, disp, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_ERR_NULL_ARG);

  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Null classes with positive count.
  EXPECT_EQ(LUMICE_SetRaypathColors(s, nullptr, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_ERR_NULL_ARG);
  // Out-of-range composite mode.
  EXPECT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, 99), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, -1), LUMICE_ERR_INVALID_VALUE);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, CommitConfigStructOverCapRejected) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);

  // raypath_color_count over the class cap: rejected before any simulation touch.
  // v4.8: leave raypath_color == nullptr and only set the count field so the count-bounds
  // branch fires before any dereference (this test never intended to allocate an
  // out-of-cap array, and Create would reject the count anyway).
  LUMICE_Config over_classes{};
  lumice::ConfigOwningGuard over_classes_guard(over_classes);
  over_classes.raypath_color_count = LUMICE_MAX_CONFIG_COLOR_CLASSES + 1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(s, &over_classes, nullptr), LUMICE_ERR_INVALID_CONFIG);

  // match_count over the ref cap: rejected in the per-class bounds loop. v4.8: raypath_color
  // is a heap pointer, so we must Create a 1-class array before writing match_count — the
  // previous inline-array pattern (`over_refs.raypath_color[0].match_count = ...`) would
  // deref a nullptr under the new ABI.
  LUMICE_Config over_refs{};
  lumice::ConfigOwningGuard over_refs_guard(over_refs);
  LUMICE_ColorClass* classes = LUMICE_ConfigCreateColorClasses(&over_refs, 1);
  ASSERT_NE(classes, nullptr);
  classes[0].match_count = LUMICE_MAX_CONFIG_COLOR_REFS + 1;
  EXPECT_EQ(LUMICE_CommitConfigStruct(s, &over_refs, nullptr), LUMICE_ERR_INVALID_CONFIG);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// task-344 regression 1: v4.8 ABI added a defensive null check to LUMICE_CommitConfigStruct
// so the per-class match_count bounds loop never dereferences a null raypath_color pointer.
// Trigger it by setting count > 0 while leaving the pointer nullptr.
TEST(RaypathColorApi, CommitConfigStructRejectsNullArrayWithNonzeroCount) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);

  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);
  cfg.raypath_color_count = 1;  // > 0
  cfg.raypath_color = nullptr;  // but no allocation
  EXPECT_EQ(LUMICE_CommitConfigStruct(s, &cfg, nullptr), LUMICE_ERR_INVALID_CONFIG);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// task-344 regression 2: connect JsonToConfig's Release-before-memset fix. Parsing two
// different color configs into the same LUMICE_Config must land the SECOND set of classes
// verbatim (i.e. the first parse's classes must not leak into the second). This test only
// verifies value semantics — removing the added Release call would still let the second
// parse pass (it just leaks the first allocation). Leak detection itself lives in the
// task's plan §6 valgrind cross-check, not here.
TEST(RaypathColorApi, ConsecutiveParseIntoSameConfigOverridesCorrectly) {
  LUMICE_Config cfg{};
  lumice::ConfigOwningGuard cfg_guard(cfg);

  // First parse: 2 classes.
  nlohmann::json c0 = ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json c1 = ColorClassJson({ 0.0f, 1.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json rc_two;
  rc_two["mode"] = "dominant";
  rc_two["classes"] = nlohmann::json::array({ c0, c1 });
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc_two).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 2);
  ASSERT_NE(cfg.raypath_color, nullptr);

  // Second parse into the SAME cfg: only 1 class, with a distinct color.
  nlohmann::json c2 = ColorClassJson({ 0.0f, 0.0f, 1.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json rc_one;
  rc_one["mode"] = "additive";
  rc_one["classes"] = nlohmann::json::array({ c2 });
  ASSERT_EQ(LUMICE_ParseConfigString(FullConfigWithRaypathColorJson(rc_one).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 1);
  ASSERT_NE(cfg.raypath_color, nullptr);
  EXPECT_EQ(cfg.raypath_color_mode, LUMICE_COLOR_MODE_ADDITIVE);
  EXPECT_FLOAT_EQ(cfg.raypath_color[0].color[0], 0.0f);
  EXPECT_FLOAT_EQ(cfg.raypath_color[0].color[1], 0.0f);
  EXPECT_FLOAT_EQ(cfg.raypath_color[0].color[2], 1.0f);
}

// task-342.3 Step 2: LUMICE_GetColorClassSignal (AC4 empty-arc detector).
TEST(RaypathColorApi, GetColorClassSignalBasic) {
  // MakeColorSimConfigJson: class0 = red match-all (always fires),
  //                        class1 = green {entry_exit min_len>=3} (subset that fires for prism).
  // Both should report signal=1 after sim drains.
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 12345u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Trigger a snapshot so lane data is materialized (mirror the GUI polling contract).
  LUMICE_RenderResult composite[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, composite, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);

  int flags[2] = { -1, -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, flags, 2), LUMICE_OK);
  EXPECT_EQ(flags[0], 1) << "class0 (match-all red) must have signal after sim";
  EXPECT_EQ(flags[1], 1) << "class1 (entry_exit min_len>=3) must have signal after sim";

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, GetColorClassSignalEmptyClassReportsZero) {
  // A class whose predicate matches nothing (raypath referring to face IDs the crystal
  // doesn't have, e.g. large indices on a hex prism) must report signal=0.
  nlohmann::json rc;
  rc["mode"] = "dominant";
  nlohmann::json c0 = ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  // class1: impossible raypath [99, 99] — face 99 does not exist on a hex prism.
  nlohmann::json r1 = ColorRefJson(0, 1);
  r1["type"] = "raypath";
  r1["raypath"] = nlohmann::json::array({ 99, 99 });
  nlohmann::json c1 = ColorClassJson({ 0.0f, 1.0f, 0.0f }, nlohmann::json::array({ r1 }));
  rc["classes"] = nlohmann::json::array({ c0, c1 });
  const std::string json = FullConfigWithRaypathColorJson(rc);

  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 4321u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_RenderResult composite[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_GetCompositeResults(s, composite, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);

  int flags[2] = { -1, -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, flags, 2), LUMICE_OK);
  EXPECT_EQ(flags[0], 1) << "class0 (match-all) must have signal";
  EXPECT_EQ(flags[1], 0) << "class1 (impossible raypath) must be empty";

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, GetColorClassSignalCountMismatchRejected) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  int one[1] = { -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, one, 1), LUMICE_ERR_INVALID_CONFIG) << "active=2, count=1 must be rejected";
  int three[3] = { -1, -1, -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, three, 3), LUMICE_ERR_INVALID_CONFIG) << "active=2, count=3 must be rejected";

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, GetColorClassSignalNullAndZeroCount) {
  // Null server.
  int flags[2] = { -1, -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(nullptr, flags, 2), LUMICE_ERR_NULL_ARG);

  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);

  // Zero color-classes committed (minimal config with no raypath_color).
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeMinimalConfigJson().c_str()), LUMICE_OK);
  // class_count=0 with any out_flags (including nullptr) must be OK no-op.
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, nullptr, 0), LUMICE_OK);

  // Non-null server, positive count, but null out_flags → NULL_ARG.
  ASSERT_EQ(LUMICE_CommitConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, nullptr, 2), LUMICE_ERR_NULL_ARG);
  // Negative count → INVALID_VALUE.
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, flags, -1), LUMICE_ERR_INVALID_VALUE);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}
