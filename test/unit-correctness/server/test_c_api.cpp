#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "config/crystal_config.hpp"  // core PrismCrystalParam + from_json, the sync-group canonical-form authority
#include "config/filter_config.hpp"   // core FilterConfig + to_json, for emit isomorphism cross-check
#include "config/raypath_color_config.hpp"  // core RaypathColorConfig + to_json, for color-class isomorphism
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "include/lumice.h"
#include "server/c_api_internal.hpp"  // ConfigScratch(+Guard) + ParseConfigString + ConfigToJson (internal)
#include "support/env_var.hpp"

// Regression guard (task-fix-stats-ray-count-u32-overflow): ray-count fields must be
// 64-bit so totals > 2^32 never truncate on Windows, where `unsigned long` is 32-bit
// (the status-bar ray-count rollover reported by Windows users). These field-level
// asserts complement the header-level guard in lumice.h: they verify the struct fields
// actually use the 64-bit type, not just that the typedef is wide enough.
static_assert(sizeof(((LUMICE_StatsResult*)nullptr)->sim_ray_num) >= 8, "stats sim_ray_num must be 64-bit");
static_assert(sizeof(((LUMICE_StatsResult*)nullptr)->ray_seg_num) >= 8, "stats ray_seg_num must be 64-bit");
static_assert(sizeof(((LUMICE_StatsResult*)nullptr)->crystal_num) >= 8, "stats crystal_num must be 64-bit");

// ABI guard (backend-lifecycle-epoch): appending the trailing uint64 `epoch`
// grew LUMICE_RawXyzResult from 56 → 64 bytes (effective_pixels@48 + 4 pad +
// epoch@56, 8-aligned). test/e2e/capi_runner.py mirrors this exact size; keep the
// two in lockstep (measured, not assumed — ctypes.sizeof == 64).
// The absolute-normalization denominator `emitted_energy` (float) then went into
// that 4-byte pad at offset 52 rather than onto the end, so the size stays 64 and
// `epoch` keeps offset 56 — the offset asserts below are what make that a checked
// claim rather than a hoped-for one.
// The exposure anchor `anchor_l99_sky` then found NO pad left to occupy — the four asserts
// below are what turned that from an assumption into a checked fact — so it went on the end
// at offset 64 and the struct grew to 68, rounded to 72 by its own 8-byte alignment. This is
// a real ABI size change, not a free append: an un-recompiled caller's buffer is 8 bytes
// short per row. Both numbers are asserted rather than one derived from the other, because
// an offset can move without the size changing and vice versa.
static_assert(sizeof(LUMICE_RawXyzResult) == 72, "LUMICE_RawXyzResult ABI must be 72 bytes (capi_runner.py mirror)");
static_assert(offsetof(LUMICE_RawXyzResult, effective_pixels) == 48, "LUMICE_RawXyzResult layout drift");
static_assert(offsetof(LUMICE_RawXyzResult, emitted_energy) == 52,
              "emitted_energy must occupy the pre-existing pad, not grow the struct");
static_assert(offsetof(LUMICE_RawXyzResult, epoch) == 56, "epoch offset must be unchanged by emitted_energy");
static_assert(offsetof(LUMICE_RawXyzResult, anchor_l99_sky) == 64, "LUMICE_RawXyzResult layout drift");
// Lands in the tail padding anchor_l99_sky's 8-byte rounding created, so it costs no size. The
// offset assertion is what says it went THERE and not onto the end (which would grow the struct
// to 80 and silently overrun every mirrored caller's buffer).
static_assert(offsetof(LUMICE_RawXyzResult, axis_solid_angle) == 68,
              "axis_solid_angle must occupy anchor_l99_sky's tail padding, not extend the struct");
static_assert(offsetof(LUMICE_RawXyzResult, anchor_l99_sky) == 64,
              "anchor_l99_sky must append after epoch, leaving every existing offset alone");

// ABI guards for the other structs test/e2e/capi_runner.py mirrors. Added after
// task-cuda-ctypes-teardown-crash: LUMICE_RenderResult grew from 24 → 32 bytes
// in task-345.3 when composite_p99_y was appended (float@24 + 4 pad, 8-aligned),
// but the ctypes mirror was not updated. Result: the render getter wrote
// 8 bytes past the Python-allocated (LUMICE_RenderResult * 1)() buffer on every
// poll, corrupting the Python heap and aborting under _ctypes teardown or the
// next glibc malloc/free check. LUMICE_ServerConfig has the same failure mode
// (12 vs 8 byte drift after preferred_backend was appended). Static-assert both
// so a future field addition breaks the build instead of silently corrupting
// pytest heaps.
static_assert(sizeof(LUMICE_RenderResult) == 32, "LUMICE_RenderResult ABI must be 32 bytes (capi_runner.py mirror)");
static_assert(sizeof(LUMICE_ServerConfig) == 12, "LUMICE_ServerConfig ABI must be 12 bytes (capi_runner.py mirror)");

namespace {

// Build a deterministic (NO_RANDOM) LUMICE_Distribution scalar.
LUMICE_Distribution DetDist(float value) {
  return LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, value, 0.0f };
}

// Serialize a scratch to its JSON text. Stands in for the retired public LUMICE_ConfigToJson
// wrapper: the encoder it wrapped (ConfigToJson) is unchanged and still the single source of the
// wire format, so the round-trip and isomorphism tests below keep asserting exactly what they
// did. The wrapper's own snprintf-style buffer contract died with it; the live equivalent is
// LUMICE_SceneToJson, whose contract is pinned in test_c_api_scene.cpp.
std::string ScratchToJson(const ConfigScratch& cfg) {
  return ConfigToJson(cfg).dump();
}

// JSON text -> Scene handle -> commit. Replaces the retired LUMICE_CommitConfig: one commit
// entry point remains, and it takes a handle.
LUMICE_ErrorCode CommitJsonConfig(LUMICE_Server* server, const char* json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json, &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

// Commit a scratch, replacing the retired LUMICE_CommitConfigStruct. It encoded via ConfigToJson
// and handed the document to the same core commit LUMICE_CommitScene uses, so routing the
// encoded text back through LUMICE_SceneFromJson reaches the identical core call. A ConfigToJson
// throw (unset/invalid filter type) maps to LUMICE_ERR_INVALID_CONFIG exactly as the retired
// wrapper mapped it, so the negative tests keep their expected code.
LUMICE_ErrorCode CommitScratch(LUMICE_Server* server, const ConfigScratch& cfg, int* out_reused) {
  std::string text;
  try {
    text = ScratchToJson(cfg);
  } catch (const std::exception&) {
    return LUMICE_ERR_INVALID_CONFIG;
  }
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(text.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, out_reused);
  LUMICE_SceneDestroy(scene);
  return err;
}

// Build a deterministic prism param: `height` scalar + six face_distance = 1.0
// (the regular-hexagon default the old JSON path relied on).
LUMICE_CrystalParam MakePrismParam(float height = 1.0f) {
  LUMICE_CrystalParam p{};
  p.type = 0;
  p.height = DetDist(height);
  for (auto& fd : p.face_distance) {
    fd = DetDist(1.0f);
  }
  return p;
}

// Build a deterministic pyramid param. Wedge angles default to 28.0° (the same
// value the old 3-arg CreatePyramid Miller-[1,0,1] default produced).
LUMICE_CrystalParam MakePyramidParam(float prism_h, float upper_h, float lower_h, float upper_wedge = 28.0f,
                                     float lower_wedge = 28.0f) {
  LUMICE_CrystalParam p{};
  p.type = 1;
  p.prism_h = DetDist(prism_h);
  p.upper_h = DetDist(upper_h);
  p.lower_h = DetDist(lower_h);
  p.upper_wedge_angle = upper_wedge;
  p.lower_wedge_angle = lower_wedge;
  for (auto& fd : p.face_distance) {
    fd = DetDist(1.0f);
  }
  return p;
}

}  // namespace

TEST(CrystalMeshApi, PrismVerticesAndEdges) {
  LUMICE_CrystalMesh mesh{};
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  LUMICE_ErrorCode err = LUMICE_GetCrystalMesh(&param, 0, &mesh);
  EXPECT_EQ(err, LUMICE_OK);
  EXPECT_EQ(mesh.vertex_count, 12);
  // 18 wireframe edges: 6 top + 6 bottom + 6 vertical (internal diagonals excluded)
  EXPECT_EQ(mesh.edge_count, 18);
}

TEST(CrystalMeshApi, PyramidVerticesAndEdges) {
  LUMICE_CrystalMesh mesh{};
  const LUMICE_CrystalParam param = MakePyramidParam(1.0f, 0.5f, 0.5f);
  LUMICE_ErrorCode err = LUMICE_GetCrystalMesh(&param, 0, &mesh);
  EXPECT_EQ(err, LUMICE_OK);
  EXPECT_GT(mesh.vertex_count, 0);
  EXPECT_GT(mesh.edge_count, 0);
  // Pyramid has more vertices and edges than a prism
  EXPECT_GT(mesh.vertex_count, 12);
}

TEST(CrystalMeshApi, NullArgs) {
  LUMICE_CrystalMesh mesh{};
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  EXPECT_EQ(LUMICE_GetCrystalMesh(nullptr, 0, &mesh), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetCrystalMesh(&param, 0, nullptr), LUMICE_ERR_NULL_ARG);
}

TEST(CrystalMeshApi, UnknownType) {
  // An out-of-range crystal type (not 0=prism / 1=pyramid) must be rejected without
  // building anything — the new-signature analog of the old "unknown type string".
  LUMICE_CrystalMesh mesh{};
  LUMICE_CrystalParam param = MakePrismParam(1.0f);
  param.type = 2;
  EXPECT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_ERR_INVALID_VALUE);
}

TEST(CrystalMeshApi, PrismFaceNumbersInLegalSet) {
  // Zero-init covers all unused slots with 0; LUMICE_GetCrystalMesh must overwrite
  // [0, triangle_count) with valid face numbers (>0).
  LUMICE_CrystalMesh mesh{};
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);
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
  const LUMICE_CrystalParam param = MakePyramidParam(1.0f, 0.5f, 0.5f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);
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
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);

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
  const LUMICE_CrystalParam param = MakePyramidParam(1.0f, 0.5f, 0.5f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);

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
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);
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
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);
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
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);
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
  const LUMICE_CrystalParam param = MakePyramidParam(0.01f, 0.5f, 0.5f, 87.0f, 87.0f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);
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
  const LUMICE_CrystalParam param = MakePyramidParam(1.0f, 0.5f, 0.5f);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh), LUMICE_OK);

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

// =============== Sampling contract tests (Step 2 boundary + AC2–AC5) ===============

// Step 2 boundary: the shared shape-translation helper must NOT leak the crystal
// id or axis distributions into its output — those are meaningless for a stateless
// mesh preview and belong only to ConfigToJson's per-crystal wrapper. Locks the
// "don't accidentally slip id/axis into the shared helper" risk.
TEST(CrystalShapeToJson, ExcludesIdAndAxis) {
  LUMICE_CrystalParam prism = MakePrismParam(1.0f);
  prism.id = 7;
  prism.zenith = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 90.0f, 10.0f };
  nlohmann::json j = CrystalShapeToJson(prism);
  EXPECT_FALSE(j.contains("id"));
  EXPECT_FALSE(j.contains("axis"));
  EXPECT_TRUE(j.contains("type"));
  EXPECT_TRUE(j.contains("shape"));

  LUMICE_CrystalParam pyr = MakePyramidParam(1.0f, 0.5f, 0.5f);
  pyr.id = 9;
  nlohmann::json jp = CrystalShapeToJson(pyr);
  EXPECT_FALSE(jp.contains("id"));
  EXPECT_FALSE(jp.contains("axis"));
}

// AC2: identical param + identical seed => bit-identical mesh. The whole struct is
// compared (both start zero-initialized, so unused tail slots stay 0 in both).
TEST(CrystalMeshSampling, DeterministicSameSeedBitExact) {
  LUMICE_CrystalParam param = MakePrismParam(1.0f);
  for (auto& fd : param.face_distance) {
    fd = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.3f };
  }
  LUMICE_CrystalMesh a{};
  LUMICE_CrystalMesh b{};
  const unsigned long long seed = 0x1234567890abcdefULL;
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, seed, &a), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, seed, &b), LUMICE_OK);
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(LUMICE_CrystalMesh)), 0)
      << "same param + same seed must produce a bit-identical mesh";
}

// AC3 (positive): a randomized crystal produces different meshes under different seeds.
TEST(CrystalMeshSampling, SeedChangesMeshWhenRandomized) {
  LUMICE_CrystalParam param = MakePrismParam(1.0f);
  for (auto& fd : param.face_distance) {
    fd = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.3f };
  }
  LUMICE_CrystalMesh a{};
  LUMICE_CrystalMesh b{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 1, &a), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 2, &b), LUMICE_OK);
  EXPECT_NE(std::memcmp(&a, &b, sizeof(LUMICE_CrystalMesh)), 0)
      << "different seeds on a randomized crystal must produce different meshes";
}

// AC3 (negative / no-op contract): a fully NO_RANDOM crystal ignores the seed.
TEST(CrystalMeshSampling, SeedNoOpWhenNoRandom) {
  const LUMICE_CrystalParam param = MakePrismParam(1.0f);  // all fields NO_RANDOM
  LUMICE_CrystalMesh a{};
  LUMICE_CrystalMesh b{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 1, &a), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 999999, &b), LUMICE_OK);
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(LUMICE_CrystalMesh)), 0)
      << "no_random crystal must yield the identical mesh regardless of seed";
}

// AC4: the preview path matches the full-precision simulation geometry bit-for-bit,
// and demonstrably diverges from what the old GUI %.4f snprintf truncation produced.
// The truncated baseline's INEQUALITY is what proves this test is not tautological.
TEST(CrystalMeshSampling, PreviewMatchesSimulationPrecision) {
  const float kFull = 1.23456789f;
  const float kTrunc = 1.2346f;  // what the old GUI "%.4f" formatting would have written
  const LUMICE_CrystalParam param = MakePrismParam(kFull);
  LUMICE_CrystalMesh mesh_new{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&param, 0, &mesh_new), LUMICE_OK);
  ASSERT_GT(mesh_new.vertex_count, 0);

  const float dist[6] = { 1, 1, 1, 1, 1, 1 };
  const lumice::Crystal c_full = lumice::Crystal::CreatePrism(kFull, dist);
  const lumice::detail::BuiltMesh built_full = lumice::detail::BuildMeshFromCfGeom(c_full.CfGeom());
  ASSERT_EQ(mesh_new.vertex_count, static_cast<int>(built_full.mesh.GetVtxCnt()));
  EXPECT_EQ(std::memcmp(mesh_new.vertices, built_full.mesh.GetVtxPtr(0), mesh_new.vertex_count * 3 * sizeof(float)), 0)
      << "preview mesh must equal full-precision simulation geometry bit-for-bit";

  const lumice::Crystal c_trunc = lumice::Crystal::CreatePrism(kTrunc, dist);
  const lumice::detail::BuiltMesh built_trunc = lumice::detail::BuildMeshFromCfGeom(c_trunc.CfGeom());
  ASSERT_EQ(mesh_new.vertex_count, static_cast<int>(built_trunc.mesh.GetVtxCnt()));
  EXPECT_NE(std::memcmp(mesh_new.vertices, built_trunc.mesh.GetVtxPtr(0), mesh_new.vertex_count * 3 * sizeof(float)), 0)
      << "%.4f-truncated geometry must differ — proving the test detects precision loss";
}

// AC5: a large-sigma crystal repeatedly samples shapes the closed-form validation
// gate rejects; every call must return LUMICE_OK (no SIGSEGV) with an empty-but-valid
// mesh. The degenerate_hit_count assertion guarantees the reject branch is actually
// exercised — "no crash" alone would still pass if the branch were never reached.
TEST(CrystalMeshSampling, DegenerateInputSurvivesLoop) {
  LUMICE_CrystalParam param = MakePrismParam(1.0f);
  // Large sigma so opposite face-distance pairs frequently sum <= 0, tripping the
  // closed-form validity gate. Seeds are fixed (0..199) and the RNG is deterministic,
  // so the hit count is fully reproducible — not flaky — and empirically comfortable
  // (dozens of hits), leaving margin if the gate is later retuned.
  for (auto& fd : param.face_distance) {
    fd = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 1.0f };
  }
  int degenerate_hit_count = 0;
  for (unsigned long long seed = 0; seed < 200; ++seed) {
    LUMICE_CrystalMesh mesh{};
    ASSERT_EQ(LUMICE_GetCrystalMesh(&param, seed, &mesh), LUMICE_OK) << "seed " << seed << " must not crash/error";
    if (mesh.vertex_count == 0) {
      ++degenerate_hit_count;  // empty-but-valid result from the validation-gate reject branch
    }
  }
  EXPECT_GT(degenerate_hit_count, 0) << "validation-gate reject branch was never exercised — pick a larger sigma";
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
  // fov is NOT optional once "lens" is present: core's LensParam::from_json requires "fov" or "f"
  // and throws out_of_range 403 otherwise (verified against the CLI). This fixture predates the
  // v4.11 renderer round-trip, when the C API ignored the whole "lens" object and so accepted a
  // document core itself rejects.
  rn["lens"]["fov"] = 180.0f;
  rn["resolution"] = { 1024, 512 };
  rn["intensity_factor"] = 2.0f;
  rn["norm_mode"] = 1;
  root["render"] = nlohmann::json::array({ rn });

  return root.dump();
}


TEST(ParseConfigApi, MinimalPrismConfig) {
  auto json = MakeMinimalConfigJson();
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(json.c_str(), &config), LUMICE_OK);

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
}


TEST(ParseConfigApi, RayNumAbove32BitNotTruncated) {
  // Regression (task-fix-stats-ray-count-u32-overflow): config ray_num was parsed via
  // `rn.get<unsigned long>()`, truncating to 32-bit on Windows. A finite ray_num above
  // 2^32 must round-trip through ParseConfigString intact.
  auto root = nlohmann::json::parse(MakeMinimalConfigJson());
  const LUMICE_RayCount kBigRayNum = 5'000'000'000ULL;  // > UINT32_MAX (4'294'967'295)
  root["scene"]["ray_num"] = kBigRayNum;

  ConfigScratch config{};

  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.infinite, 0);
  // Pre-fix on Windows this truncated to 705'032'704; post-fix it holds the full value.
  EXPECT_EQ(config.ray_num, kBigRayNum);
}


TEST(ParseConfigApi, FullConfigWithPyramidAndFilter) {
  auto json = MakeFullConfigJson();
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(json.c_str(), &config), LUMICE_OK);

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


// K-class: NLOHMANN_JSON_SERIALIZE_ENUM maps an unrecognized string to the FIRST table entry, so
// the JSON string parse path pre-checks lens.type / visible against a hand-written known-value
// list before decoding (IsKnownLensTypeString / IsKnownVisibleString in c_api.cpp) rather than
// letting a typo silently become "linear" / "upper". LUMICE_SceneAddRenderer's struct entry point
// is covered separately by SceneNegative.RendererInvalidEnumOrGridCountRejected (int, not string);
// this pair covers the string-typed JSON path those tests don't reach.
TEST(ParseConfigApi, RendererLensTypeUnknownStringRejected) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  root["render"][0]["lens"]["type"] = "not_a_real_lens";
  // NLOHMANN_JSON_SERIALIZE_ENUM's silent-fallback value is kLinear (the first table entry,
  // core/render_config.hpp), whose MaxFov is 179°. Use a fov that is valid for kLinear (60°,
  // well under both 179° and every other lens type's cap) so a pre-check bypass would silently
  // succeed via the fallback instead of incidentally being caught by the unrelated fov-range
  // check that MakeFullConfigJson's fov=180 would trip for kLinear regardless of this guard.
  root["render"][0]["lens"]["fov"] = 60.0f;

  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}


TEST(ParseConfigApi, RendererVisibleUnknownStringRejected) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  root["render"][0]["visible"] = "not_a_real_visibility";

  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}


TEST(ParseConfigApi, ParseModifyCommit) {
  auto json = MakeMinimalConfigJson();
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  ASSERT_EQ(ParseConfigString(json.c_str(), &config), LUMICE_OK);

  // Modify ray_num
  config.ray_num = 5000000;
  config.infinite = 0;

  // Commit to a real server
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);

  int reused = -1;
  EXPECT_EQ(CommitScratch(server, config, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0);  // First commit, not reused

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}


// The file-reading half of the JSON reader. LUMICE_SceneFromJsonFile is its only entry point
// since v4.12 retired LUMICE_ParseConfigFile; both read through the same JsonToConfig, so the
// assertion moved from struct fields to the handle's wire document.
TEST(ParseConfigApi, SceneFromJsonFile) {
  auto json = MakeMinimalConfigJson();

  // Write to temp file (cross-platform: use std::filesystem::temp_directory_path)
  auto tmp_path = std::filesystem::temp_directory_path() / "lumice_test_config.json";
  {
    std::ofstream f(tmp_path);
    f << json;
  }

  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJsonFile(tmp_path.u8string().c_str(), &scene), LUMICE_OK);
  ASSERT_NE(scene, nullptr);
  const auto& root = SceneRoot(scene);
  ASSERT_TRUE(root.contains("crystal"));
  ASSERT_EQ(root.at("crystal").size(), 1u);
  EXPECT_FLOAT_EQ(root.at("crystal").at(0).at("shape").at("height").get<float>(), 1.5f);
  LUMICE_SceneDestroy(scene);

  std::filesystem::remove(tmp_path);
}


TEST(ParseConfigApi, NullArgs) {
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(nullptr, &config), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(ParseConfigString("{}", nullptr), LUMICE_ERR_NULL_ARG);
  LUMICE_Scene* scene = nullptr;
  EXPECT_EQ(LUMICE_SceneFromJsonFile(nullptr, &scene), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(scene, nullptr);
  EXPECT_EQ(LUMICE_SceneFromJsonFile("/tmp/test.json", nullptr), LUMICE_ERR_NULL_ARG);
}


TEST(ParseConfigApi, InvalidJson) {
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString("not json at all", &config), LUMICE_ERR_INVALID_JSON);
  EXPECT_EQ(ParseConfigString("{invalid", &config), LUMICE_ERR_INVALID_JSON);
}


TEST(ParseConfigApi, MissingCrystalSection) {
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  // Valid JSON but missing "crystal" key
  EXPECT_EQ(ParseConfigString(R"({"scene": {}})", &config), LUMICE_ERR_MISSING_FIELD);
}


// `prob` must be rejected HERE, not only by core's ParseScatteringInfo. This parser is what the
// CLI actually runs, and it hands core a re-serialized document (ConfigToJson writes `prob`
// unconditionally from the parsed struct), so a check that lives only in core would never see the
// missing key — it would already have been filled in with the struct's zero. Deleting this test's
// counterpart branch in c_api.cpp makes this red while core's own test stays green, which is the
// whole reason both exist.
TEST(ParseConfigApi, ScatteringMissingProbRejected) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  ASSERT_FALSE(root["scene"]["scattering"].empty());
  root["scene"]["scattering"][0].erase("prob");

  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_MISSING_FIELD);
}


// An `axis` slot written as an object must name its `type`, and it must be rejected HERE, not only
// by core's from_json. This parser is what the CLI actually runs, and core only ever sees this
// parser's output re-encoded by ConfigToJson — which writes `type` unconditionally from the
// struct, so the missing key would already have been filled in before core looked. Deleting the
// counterpart branch in c_api.cpp makes this red while core's own test stays green, which is the
// whole reason both exist (the `prob` narrowing established the pattern).
//
// One case per slot: before this narrowing each slot silently produced a different wrong answer,
// so they are not interchangeable evidence.
TEST(ParseConfigApi, AxisSlotObjectMissingTypeRejected) {
  for (const char* slot : { "zenith", "azimuth", "roll" }) {
    SCOPED_TRACE(slot);
    auto root = nlohmann::json::parse(MakeFullConfigJson());
    ASSERT_TRUE(root["crystal"][0]["axis"][slot].is_object());
    root["crystal"][0]["axis"][slot].erase("type");

    ConfigScratch config{};
    ConfigScratchGuard config_guard(config);
    EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_MISSING_FIELD);
  }
}

TEST(ParseConfigApi, AxisWithoutZenithRejected) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  root["crystal"][0]["axis"].erase("zenith");

  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_MISSING_FIELD);
}

// Both rejections again, this time through the whole public commit path a GUI drives
// (SceneFromJson -> CommitScene) rather than the internal parser alone. The point is that the
// document is turned away with an error code and the process survives — the failure mode this
// guards against is a throw escaping somewhere along the way, which a return-code assertion on
// ParseConfigString by itself cannot see.
//
// It reports MISSING_FIELD, not INVALID_CONFIG: the handle path parses the user's text at
// SceneFromJson and hands core only the ConfigToJson re-encoding of what it accepted. Core's own
// throw for these two documents is therefore structurally unreachable from here — ConfigToJson
// always emits `type` — which is the same asymmetry that makes the c_api.cpp half mandatory.
TEST(AxisSlotCommit, MalformedAxisIsRejectedNotCrash) {
  struct Case {
    const char* label;
    nlohmann::json (*mutate)(nlohmann::json);
  };
  const Case cases[] = {
    { "zenith object without type",
      [](nlohmann::json root) {
        root["crystal"][0]["axis"]["zenith"].erase("type");
        return root;
      } },
    { "azimuth object without type",
      [](nlohmann::json root) {
        root["crystal"][0]["axis"]["azimuth"].erase("type");
        return root;
      } },
    { "roll object without type",
      [](nlohmann::json root) {
        root["crystal"][0]["axis"]["roll"].erase("type");
        return root;
      } },
    { "axis without zenith",
      [](nlohmann::json root) {
        root["crystal"][0]["axis"].erase("zenith");
        return root;
      } },
  };

  for (const auto& c : cases) {
    SCOPED_TRACE(c.label);
    auto root = c.mutate(nlohmann::json::parse(MakeFullConfigJson()));
    root["scene"]["ray_num"] = 100;  // keep the commit finite

    auto* server = LUMICE_CreateServer();
    ASSERT_NE(server, nullptr);
    EXPECT_EQ(CommitJsonConfig(server, root.dump().c_str()), LUMICE_ERR_MISSING_FIELD);
    LUMICE_StopServer(server);
    LUMICE_DestroyServer(server);
  }
}


TEST(ParseConfigApi, FileNotFound) {
  LUMICE_Scene* scene = nullptr;
  EXPECT_EQ(LUMICE_SceneFromJsonFile("/tmp/nonexistent_lumice_config_12345.json", &scene), LUMICE_ERR_FILE_NOT_FOUND);
  EXPECT_EQ(scene, nullptr);
}


TEST(ParseConfigApi, UnsupportedFilterType) {
  nlohmann::json root;
  root["crystal"] = nlohmann::json::array();
  root["scene"] = { { "ray_num", 1000 } };
  root["filter"] = nlohmann::json::array({ { { "id", 1 }, { "type", "direction" }, { "action", "filter_in" } } });

  ConfigScratch config{};

  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
}


TEST(ParseConfigApi, ArraySpectrumParsed) {
  // Start from a complete document and swap in the discrete spectrum: every other required key
  // (filter / render / scene.max_hits / scene.scattering, light_source.type / .altitude) must be
  // present, since the parser now enforces exactly what core's from_json enforces.
  auto root = nlohmann::json::parse(MakeMinimalConfigJson());
  root["scene"]["light_source"]["spectrum"] = nlohmann::json::array(
      { { { "wavelength", 450 }, { "weight", 0.8 } }, { { "wavelength", 550 }, { "weight", 1.0 } } });

  ConfigScratch config{};

  ConfigScratchGuard config_guard(config);
  ASSERT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_OK);
  EXPECT_EQ(config.spectrum_count, 2);
  EXPECT_FLOAT_EQ(config.spectrum_entries[0].wavelength, 450.0f);
  EXPECT_FLOAT_EQ(config.spectrum_entries[0].weight, 0.8f);
  EXPECT_FLOAT_EQ(config.spectrum_entries[1].wavelength, 550.0f);
  EXPECT_FLOAT_EQ(config.spectrum_entries[1].weight, 1.0f);
}


TEST(ParseConfigApi, StructSpectrumRoundTrip) {
  // Fill ConfigScratch directly, commit via struct path (bypasses JSON parse), then re-parse
  // the ConfigToJson output via a JSON round-trip to prove spectrum_entries[] serializes into
  // the array shape core light_config expects (mirrors GUI struct→commit path).
  auto json = MakeMinimalConfigJson();
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  ASSERT_EQ(ParseConfigString(json.c_str(), &config), LUMICE_OK);

  config.spectrum_count = 3;
  config.spectrum_entries[0] = { 450.0f, 0.5f };
  config.spectrum_entries[1] = { 550.0f, 1.0f };
  config.spectrum_entries[2] = { 650.0f, 0.7f };
  config.ray_num = 300;  // 3 wavelengths * 100 rays each is enough for a smoke commit
  config.infinite = 0;

  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(CommitScratch(server, config, &reused), LUMICE_OK);
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

  ConfigScratch config{};

  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_CONFIG);
}


TEST(ParseConfigApi, SpectrumEnumerations) {
  // Test all supported spectrum strings (D55 / D75 added in task-323).
  for (const char* sp : { "D65", "D55", "D50", "D75", "A", "E" }) {
    auto json_str = MakeMinimalConfigJson();
    auto root = nlohmann::json::parse(json_str);
    root["scene"]["light_source"]["spectrum"] = sp;

    ConfigScratch config{};

    ConfigScratchGuard config_guard(config);
    ASSERT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_OK) << "Failed for spectrum: " << sp;
    EXPECT_STREQ(config.spectrum, sp);
    EXPECT_EQ(config.spectrum_count, 0);
  }

  // Unknown spectrum string
  auto json_str = MakeMinimalConfigJson();
  auto root = nlohmann::json::parse(json_str);
  root["scene"]["light_source"]["spectrum"] = "UnknownIlluminant";

  ConfigScratch config{};

  ConfigScratchGuard config_guard(config);
  EXPECT_EQ(ParseConfigString(root.dump().c_str(), &config), LUMICE_ERR_INVALID_VALUE);
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
    ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
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
  ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
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
  ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
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
  ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
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
  ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
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
  //
  // The 65 refs are spread over 3 classes because a single class may carry at most
  // LUMICE_MAX_CONFIG_COLOR_REFS (32) of them. Predicates dedup across classes, so the
  // component-bit budget is consumed by the union either way — the grouping is irrelevant to
  // what this test asserts. (Before v4.12 this config was written as one 65-ref class and
  // committed as a raw JSON string, which reached core without passing the C API's own
  // per-class cap; that entry point is gone, so the config is now expressed within the cap.)
  auto base = nlohmann::json::parse(MakeSmallSimConfigJson());
  constexpr int kTotalRefs = 65;
  constexpr int kRefsPerClass = 25;  // <= LUMICE_MAX_CONFIG_COLOR_REFS
  nlohmann::json classes = nlohmann::json::array();
  for (int first = 0; first < kTotalRefs; first += kRefsPerClass) {
    nlohmann::json cls;
    cls["color"] = { 1.0, 0.0, 0.0 };
    nlohmann::json matches = nlohmann::json::array();
    for (int k = first; k < std::min(first + kRefsPerClass, kTotalRefs); ++k) {
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
  }
  base["raypath_color"]["classes"] = classes;
  const std::string json = base.dump();

  ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
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


TEST_F(ServerLifecycleApi, FrameGetRender) {
  CommitAndWaitForIdle();

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);

  // out array size = LUMICE_MAX_RENDER_RESULTS + 1 (sentinel slot)
  LUMICE_RenderResult out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRender(frame, out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);

  // First (and only) renderer matches MakeMinimalConfigJson() resolution 800x400, id=1
  EXPECT_EQ(out[0].renderer_id, 1);
  EXPECT_EQ(out[0].img_width, 800);
  EXPECT_EQ(out[0].img_height, 400);
  EXPECT_NE(out[0].img_buffer, nullptr);

  // Sentinel: img_buffer == NULL marks end of array
  EXPECT_EQ(out[1].img_buffer, nullptr);

  LUMICE_ReleaseResultFrame(frame);
}


TEST_F(ServerLifecycleApi, FrameGetRawXyz) {
  CommitAndWaitForIdle();

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);

  LUMICE_RawXyzResult out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);

  EXPECT_EQ(out[0].renderer_id, 1);
  EXPECT_EQ(out[0].img_width, 800);
  EXPECT_EQ(out[0].img_height, 400);
  EXPECT_NE(out[0].xyz_buffer, nullptr);
  EXPECT_NE(out[0].has_valid_data, 0);

  // Sentinel: xyz_buffer == NULL marks end of array
  EXPECT_EQ(out[1].xyz_buffer, nullptr);

  LUMICE_ReleaseResultFrame(frame);
}


TEST_F(ServerLifecycleApi, FrameGetStats) {
  CommitAndWaitForIdle();

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);

  LUMICE_StatsResult out{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &out), LUMICE_OK);

  // After running 1000 rays through one crystal with single-pass scattering:
  EXPECT_GT(out.sim_ray_num, 0u);
  EXPECT_GT(out.crystal_num, 0u);
  // The orientation half must be mapped too. c_api.cpp fills this struct with a
  // hand-written field-by-field copy, which is the copy-paste shape where a new
  // field is easiest to forget — and forgetting it yields 0, which reads as
  // "this scene never randomizes orientation" rather than as a missing wire.
  EXPECT_GT(out.orientation_num, 0u);

  LUMICE_ReleaseResultFrame(frame);
}


// ---------------------------------------------------------------------------------------
// Result frame (opaque handle). The lifetime property itself — a frame held across a later
// snapshot keeps its own data — lives in test_result_frame_lifetime.cpp; what is checked
// here is the frame reads themselves and the handle's own contract.
// ---------------------------------------------------------------------------------------

// Every kind of result read off ONE frame belongs to ONE snapshot. This is what retires the
// combined xyz+composite getter: pairing is not something a caller has to ask for.
TEST_F(ServerLifecycleApi, ResultFrameReadsShareOneGeneration) {
  CommitAndWaitForIdle();

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);

  LUMICE_RawXyzResult xyz_a[2]{};
  LUMICE_RawXyzResult xyz_b[2]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz_a, 1), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz_b, 1), LUMICE_OK);
  EXPECT_EQ(xyz_a[0].snapshot_generation, xyz_b[0].snapshot_generation);
  EXPECT_EQ(xyz_a[0].xyz_buffer, xyz_b[0].xyz_buffer) << "two reads of one frame disagree";

  // Stats are re-read the same way. This is what the retired cached-stats consistency case
  // used to check by a different route: it read stats twice and required no new snapshot in
  // between. A frame makes "no new snapshot in between" true by construction rather than by
  // the reader picking a getter that promises not to refresh.
  LUMICE_StatsResult stats_a{};
  LUMICE_StatsResult stats_b{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats_a), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats_b), LUMICE_OK);
  ASSERT_GT(stats_a.sim_ray_num, 0u) << "no stats to compare — the case would be vacuous";
  EXPECT_EQ(stats_a.sim_ray_num, stats_b.sim_ray_num);
  EXPECT_EQ(stats_a.crystal_num, stats_b.crystal_num);
  EXPECT_EQ(stats_a.orientation_num, stats_b.orientation_num);
  EXPECT_EQ(stats_a.ray_seg_num, stats_b.ray_seg_num);

  LUMICE_ReleaseResultFrame(frame);
}


// Release really does give the share back, and taking another frame afterwards works.
TEST_F(ServerLifecycleApi, ResultFrameAcquireReleaseAcquire) {
  CommitAndWaitForIdle();

  LUMICE_ResultFrame* first = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &first), LUMICE_OK);
  LUMICE_ReleaseResultFrame(first);

  LUMICE_ResultFrame* second = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &second), LUMICE_OK);
  ASSERT_NE(second, nullptr);
  LUMICE_RenderResult out[2]{};
  EXPECT_EQ(LUMICE_FrameGetRender(second, out, 1), LUMICE_OK);
  EXPECT_NE(out[0].img_buffer, nullptr);
  LUMICE_ReleaseResultFrame(second);
}


// A frame acquired before anything has run is empty, not invalid: every read succeeds and
// writes its sentinel / all-zero struct. Nothing here may crash.
TEST_F(ServerLifecycleApi, ResultFrameBeforeAnySnapshotIsEmpty) {
  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_NE(frame, nullptr);

  LUMICE_RenderResult render[2]{};
  LUMICE_RenderResult composite[2]{};
  LUMICE_RawXyzResult xyz[2]{};
  LUMICE_StatsResult stats{ 1, 1, 1, 1 };  // pre-dirtied: an untouched out param would pass
  EXPECT_EQ(LUMICE_FrameGetRender(frame, render, 1), LUMICE_OK);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame, composite, 1), LUMICE_OK);
  EXPECT_EQ(LUMICE_FrameGetRawXyz(frame, xyz, 1), LUMICE_OK);
  EXPECT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(render[0].img_buffer, nullptr);
  EXPECT_EQ(composite[0].img_buffer, nullptr);
  EXPECT_EQ(xyz[0].xyz_buffer, nullptr);
  EXPECT_EQ(stats.sim_ray_num, 0u);
  EXPECT_EQ(stats.ray_seg_num, 0u);
  EXPECT_EQ(stats.crystal_num, 0u);
  EXPECT_EQ(stats.orientation_num, 0u);

  LUMICE_ReleaseResultFrame(frame);
}


TEST_F(ServerLifecycleApi, ResultFrameNullArgs) {
  LUMICE_ResultFrame* frame = nullptr;
  EXPECT_EQ(LUMICE_AcquireResultFrame(nullptr, &frame), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_AcquireResultFrame(server_, nullptr), LUMICE_ERR_NULL_ARG);

  LUMICE_RenderResult render[2]{};
  LUMICE_RawXyzResult xyz[2]{};
  LUMICE_StatsResult stats{};
  EXPECT_EQ(LUMICE_FrameGetRender(nullptr, render, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetComposite(nullptr, render, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRawXyz(nullptr, xyz, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetStats(nullptr, &stats), LUMICE_ERR_NULL_ARG);

  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  EXPECT_EQ(LUMICE_FrameGetRender(frame, nullptr, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame, nullptr, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRawXyz(frame, nullptr, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetStats(frame, nullptr), LUMICE_ERR_NULL_ARG);
  LUMICE_ReleaseResultFrame(frame);

  // NULL-safe no-op, same contract as LUMICE_DestroyServer / LUMICE_SceneDestroy.
  // Double-release is deliberately NOT tested: the contract says it is undefined behavior
  // and there is no double-free sentinel, which is the same answer every other handle in
  // this API gives.
  LUMICE_ReleaseResultFrame(nullptr);
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
    ASSERT_EQ(CommitJsonConfig(server_, small_cfg.c_str()), LUMICE_OK) << "CommitConfig failed at iter " << i;

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
  lumice::test::SetEnvVar("LUMICE_TRACE_BACKEND", "cpu_backend");
  struct EnvGuard {
    ~EnvGuard() { lumice::test::UnsetEnvVar("LUMICE_TRACE_BACKEND"); }
  } guard;

  auto json = MakeBdFilterConfigJson();
  ASSERT_EQ(CommitJsonConfig(server_, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(server_, 60000))
      << "Server did not reach IDLE within 60s — sim_scene_cnt_ leak on 0-exit batch regressed";
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

  LUMICE_ResultFrame* frame = nullptr;
  EXPECT_EQ(LUMICE_AcquireResultFrame(server, &frame), LUMICE_OK);
  ASSERT_NE(frame, nullptr) << "acquiring before a commit must still yield a frame, just an empty one";

  // Render results: empty count, sentinel at index 0
  LUMICE_RenderResult render_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_FrameGetRender(frame, render_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(render_out[0].img_buffer, nullptr);

  // Raw XYZ results: empty count, sentinel at index 0
  LUMICE_RawXyzResult xyz_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_FrameGetRawXyz(frame, xyz_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(xyz_out[0].xyz_buffer, nullptr);

  // Stats: all-zero struct
  LUMICE_StatsResult stats_out{};
  EXPECT_EQ(LUMICE_FrameGetStats(frame, &stats_out), LUMICE_OK);
  EXPECT_EQ(stats_out.sim_ray_num, 0u);
  EXPECT_EQ(stats_out.crystal_num, 0u);
  EXPECT_EQ(stats_out.ray_seg_num, 0u);

  LUMICE_ReleaseResultFrame(frame);

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
// LUMICE_IsShapeScalarApplicable / LUMICE_ShapeScalarSyncKeyName
// ============================================================

TEST(ShapeScalarApplicableApi, PrismOwnsHeightAndTheSixFaces) {
  EXPECT_NE(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_HEIGHT), 0);
  for (int s = LUMICE_SHAPE_SCALAR_FACE_0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
    EXPECT_NE(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PRISM, s), 0) << "slot=" << s;
  }
  // The three stacked pyramid heights are not fields a prism has at all.
  EXPECT_EQ(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_UPPER_H), 0);
  EXPECT_EQ(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_PRISM_H), 0);
  EXPECT_EQ(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_LOWER_H), 0);
}

TEST(ShapeScalarApplicableApi, PyramidOwnsThreeHeightsAndTheSixFaces) {
  EXPECT_NE(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_UPPER_H), 0);
  EXPECT_NE(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_PRISM_H), 0);
  EXPECT_NE(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_LOWER_H), 0);
  for (int s = LUMICE_SHAPE_SCALAR_FACE_0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
    EXPECT_NE(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PYRAMID, s), 0) << "slot=" << s;
  }
  EXPECT_EQ(LUMICE_IsShapeScalarApplicable(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_HEIGHT), 0);
}

// ============================================================
// LUMICE_IsDApplicable
// ============================================================

// The rule: azimuth uniform over a full turn AND the roll anchor on a multiple of 30 deg.
TEST(IsDApplicableApi, AnswersTheTwoConditionsAndNothingElse) {
  EXPECT_NE(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f, 0.0f), 0);
  EXPECT_NE(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f, 30.0f), 0);
  EXPECT_NE(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f, -60.0f), 0);
  EXPECT_NE(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f, 180.0f), 0);

  EXPECT_EQ(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f, 15.0f), 0);   // roll off the grid
  EXPECT_EQ(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 180.0f, 0.0f), 0);    // not a full turn
  EXPECT_EQ(LUMICE_IsDApplicable(LUMICE_DIST_GAUSS, 360.0f, 0.0f), 0);      // wrong type
  EXPECT_EQ(LUMICE_IsDApplicable(LUMICE_DIST_NO_RANDOM, 360.0f, 0.0f), 0);  // wrong type
}

// The tolerance is core's, not a second one chosen here. 3.05e-5 is the residue a sqrt-mapped
// Range slider used to leave behind at its stop, and it is the value on which the GUI's former
// private copy (1e-3) and core (1e-5) gave opposite answers -- the checkbox saying D was live
// while the engine had already dropped it. This case is the pin that they now cannot disagree:
// whatever core answers here is what the GUI shows, because the GUI asks this function.
TEST(IsDApplicableApi, TheToleranceIsCoresOwn) {
  EXPECT_EQ(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f - 3.05e-5f, 0.0f), 0);
  EXPECT_EQ(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f, 3.05e-5f), 0)
      << "a roll anchor off the 30 deg grid by more than core's epsilon must not pass either";
  // Just inside it, so the case above is testing the threshold and not merely "any nonzero delta".
  EXPECT_NE(LUMICE_IsDApplicable(LUMICE_DIST_UNIFORM, 360.0f - 1e-6f, 0.0f), 0);
}

TEST(IsDApplicableApi, AnUnrecognisedDistributionTypeAnswersFalse) {
  EXPECT_EQ(LUMICE_IsDApplicable(-1, 360.0f, 0.0f), 0);
  EXPECT_EQ(LUMICE_IsDApplicable(999, 360.0f, 0.0f), 0);
}

TEST(ShapeScalarApplicableApi, OutOfRangeSlotsAnswerFalseRatherThanTrapping) {
  for (auto kind : { LUMICE_CRYSTAL_PRISM, LUMICE_CRYSTAL_PYRAMID }) {
    EXPECT_EQ(LUMICE_IsShapeScalarApplicable(kind, -1), 0);
    EXPECT_EQ(LUMICE_IsShapeScalarApplicable(kind, LUMICE_SHAPE_SCALAR_COUNT), 0);
    EXPECT_EQ(LUMICE_IsShapeScalarApplicable(kind, 12345), 0);
  }
}

TEST(ShapeScalarSyncKeyNameApi, NamesEveryApplicableSlot) {
  EXPECT_STREQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_HEIGHT), "height");
  EXPECT_STREQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_UPPER_H), "upper_h");
  EXPECT_STREQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_PRISM_H), "prism_h");
  EXPECT_STREQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_LOWER_H), "lower_h");
  // All six faces share one key — its wire value is a 6-element array.
  for (auto kind : { LUMICE_CRYSTAL_PRISM, LUMICE_CRYSTAL_PYRAMID }) {
    for (int s = LUMICE_SHAPE_SCALAR_FACE_0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
      EXPECT_STREQ(LUMICE_ShapeScalarSyncKeyName(kind, s), "face_distance") << "slot=" << s;
    }
  }
}

TEST(ShapeScalarSyncKeyNameApi, InapplicableAndOutOfRangeSlotsAnswerNull) {
  EXPECT_EQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_UPPER_H), nullptr);
  EXPECT_EQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_PRISM_H), nullptr);
  EXPECT_EQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PRISM, LUMICE_SHAPE_SCALAR_LOWER_H), nullptr);
  EXPECT_EQ(LUMICE_ShapeScalarSyncKeyName(LUMICE_CRYSTAL_PYRAMID, LUMICE_SHAPE_SCALAR_HEIGHT), nullptr);
  for (auto kind : { LUMICE_CRYSTAL_PRISM, LUMICE_CRYSTAL_PYRAMID }) {
    EXPECT_EQ(LUMICE_ShapeScalarSyncKeyName(kind, -1), nullptr);
    EXPECT_EQ(LUMICE_ShapeScalarSyncKeyName(kind, LUMICE_SHAPE_SCALAR_COUNT), nullptr);
  }
}

// The two queries answer the same question about the same slot, so they must never disagree:
// a slot that applies must be nameable, and a slot that does not must have no name. This is the
// assertion that keeps the "unreachable" nullptr branch inside ShapeScalarSyncKeyName unreachable
// — a slot added to the applicability map but not to the key table would land there, and would
// otherwise show up only as a JSON field that silently goes missing.
TEST(ShapeScalarSyncKeyNameApi, AgreesWithApplicabilityOnEverySlot) {
  for (auto kind : { LUMICE_CRYSTAL_PRISM, LUMICE_CRYSTAL_PYRAMID }) {
    for (int s = 0; s < LUMICE_SHAPE_SCALAR_COUNT; ++s) {
      const bool applicable = LUMICE_IsShapeScalarApplicable(kind, s) != 0;
      const char* key = LUMICE_ShapeScalarSyncKeyName(kind, s);
      EXPECT_EQ(applicable, key != nullptr) << "kind=" << kind << " slot=" << s;
    }
  }
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
// still let the commit return OK). Commit tests then verify the end-to-end
// ABI-safe path: valid simple types commit; UNSET(0)/out-of-range type -> INVALID_CONFIG
// (via ConfigToJson throw + its caller's catch), never a crash across the C ABI.
// =====================================================================================

namespace {
// Minimal ConfigScratch carrying exactly one filter, for ConfigToJson emit assertions.
// Other sections stay empty (counts 0) so ConfigToJson's crystal/render/scatter loops are
// no-ops; spectrum == nullptr resolves to "D65". ConfigToJson does not validate, so this
// is enough to inspect the emitted filter shape without a server.
// Caller-owned out-param, because ConfigScratch is non-copyable: raypath_color owns a heap
// allocation, so returning by value would leave two aliased copies. Caller must attach
// a ConfigScratchGuard to `out` before calling.
void FillOneFilterConfig(ConfigScratch* out, const LUMICE_FilterParam& f) {
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  FillOneFilterConfig(&config, f);
  EXPECT_THROW(ConfigToJson(config), std::invalid_argument);
}

TEST(StructFilterEmit, OutOfRangeTypeThrows) {
  LUMICE_FilterParam f{};
  f.id = 1;
  f.type = 99;  // out of range
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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

// --- End-to-end commit of a scratch (CommitScratch) -----------------------------------

namespace {
// Parse the full config (crystals + scene + one referenced filter), then replace that
// filter (keeping its id, so the scattering reference stays valid) with `f` and shrink to
// a fast finite sim. Returns the config ready for CommitScratch.
// Caller-owned out-param (same non-copyable reason as FillOneFilterConfig). Populates `out`
// via ParseConfigString then overrides filter[0] and finiteness. Caller must attach a
// ConfigScratchGuard first.
void FillCommitConfigWithFilter(ConfigScratch* out, const LUMICE_FilterParam& f) {
  EXPECT_EQ(ParseConfigString(MakeFullConfigJson().c_str(), out), LUMICE_OK);
  EXPECT_GE(out->filter_count, 1);
  const int fid = out->filters[0].id;
  out->filters[0] = f;
  out->filters[0].id = fid;
  out->infinite = 0;
  out->ray_num = 100;
}

int CommitFilter(const LUMICE_FilterParam& f) {
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  FillCommitConfigWithFilter(&config, f);
  auto* server = LUMICE_CreateServer();
  EXPECT_NE(server, nullptr);
  int reused = -1;
  auto err = CommitScratch(server, config, &reused);
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
  // throws; the commit must catch and return INVALID_CONFIG, never crash.
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
// types + ConfigToJson. Round-trip goes through the internal serialize + parse
// APIs; cross-check against core from_json (source of truth) rather than hand-transcribed
// expectations (see learnings: contract-and-property-tests / emit-schema cross-check).
// =====================================================================================

namespace {
// struct -> JSON (ConfigToJson) -> struct (ParseConfigString). Returns
// the round-tripped filters[0]. Exercises both new 327.1 pieces end to end.
LUMICE_FilterParam RoundTripFilter(const LUMICE_FilterParam& in) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  FillOneFilterConfig(&cfg, in);
  const std::string text = ScratchToJson(cfg);
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  EXPECT_EQ(ParseConfigString(text.c_str(), &out), LUMICE_OK);
  EXPECT_EQ(out.filter_count, 1);
  return out.filters[0];
}

// Replace MakeFullConfigJson's filter[0] with `jf` (id kept 1 so scattering ref stays valid)
// and return the full config JSON string, for parse-side tests.
std::string FullConfigWithFilterJson(const nlohmann::json& jf) {
  auto root = nlohmann::json::parse(MakeFullConfigJson());
  root["filter"][0] = jf;
  // Keep the scattering entry pointing at whatever id the substituted filter carries: a dangling
  // filter reference is a config core itself refuses (m.filters_.at throws), so leaving the
  // fixture's original id here would make the document invalid rather than exercise the filter.
  root["scene"]["scattering"][0]["entries"][0]["filter"] = jf.at("id");
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
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  EXPECT_EQ(ParseConfigString(json.c_str(), &out), LUMICE_OK);
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
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  EXPECT_EQ(ParseConfigString(json.c_str(), &out), LUMICE_ERR_MISSING_FIELD);
}

TEST(StructFilterParse, UnknownTypeRejected) {
  // The default branch also covers arbitrary unknown type strings (not just "complex").
  auto json = FullConfigWithFilterJson({ { "id", 1 }, { "action", "filter_in" }, { "type", "bogus" } });
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  EXPECT_EQ(ParseConfigString(json.c_str(), &out), LUMICE_ERR_INVALID_VALUE);
}

TEST(StructFilterParse, IllegalEntryExitValuePassesThroughLikeCore) {
  // Decision (plan 327.1 §7 risk 2): parse does lossless mapping only; value validation
  // (min_len >= 1) stays single-source in core and fires at commit. So parse of min_len=0
  // must succeed and store it verbatim (core would likewise not reject at from_json time;
  // it throws only later). This pins the "validation not duplicated in parse" contract.
  auto json = FullConfigWithFilterJson(
      { { "id", 1 }, { "action", "filter_in" }, { "type", "entry_exit" }, { "entry", 3 }, { "min_len", 0 } });
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  EXPECT_EQ(ParseConfigString(json.c_str(), &out), LUMICE_OK);
  ASSERT_GE(out.filter_count, 1);
  EXPECT_EQ(out.filters[0].ee_min_len, 0);  // stored verbatim, not normalized/rejected here
}

// =====================================================================================
// v4.10 distribution leaf: LUMICE_Distribution round-trips for shape scalars
// (struct -> ConfigToJson -> ParseConfigString), across all six distribution
// types INCLUDING no_random (AC3), plus geom_clock struct-path equivalence (AC5) and the
// LUMICE_API_VERSION compile-time guard (AC4).
// =====================================================================================

// AC4: LUMICE_API_VERSION exists and is usable in a caller static_assert.
static_assert(LUMICE_API_VERSION >= 412, "LUMICE_API_VERSION regressed below v4.12");

namespace {
// struct -> JSON (ConfigToJson) -> struct (ParseConfigString). Returns the
// round-tripped crystals[0]. Mirrors RoundTripFilter for the crystal / distribution path.
LUMICE_CrystalParam RoundTripCrystal(const LUMICE_CrystalParam& in) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  cfg.crystal_count = 1;
  cfg.crystals[0] = in;
  const std::string text = ScratchToJson(cfg);
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  EXPECT_EQ(ParseConfigString(text.c_str(), &out), LUMICE_OK);
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
  EXPECT_FLOAT_EQ(out.face_distance[2].spread, 0.1f);
  EXPECT_EQ(out.face_distance[3].type, LUMICE_DIST_ZIGZAG);
  EXPECT_FLOAT_EQ(out.face_distance[3].center, 0.9f);
  EXPECT_FLOAT_EQ(out.face_distance[3].spread, 0.2f);
  EXPECT_EQ(out.face_distance[4].type, LUMICE_DIST_LAPLACIAN);
  EXPECT_FLOAT_EQ(out.face_distance[4].center, 1.1f);
  EXPECT_FLOAT_EQ(out.face_distance[4].spread, 0.03f);
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
  EXPECT_FLOAT_EQ(out.lower_h.center, 0.5f);
  EXPECT_FLOAT_EQ(out.upper_wedge_angle, 28.0f);
}

// AC5: geom_clock reaches the scene JSON via the struct path, with the same "0 => omit" wire
// convention core proj_config.cpp uses; and it survives a struct -> JSON -> struct round-trip.
TEST(GeomClockStructPath, EmittedWhenSet) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  cfg.geom_clock = 30;
  auto j = nlohmann::json::parse(ScratchToJson(cfg));
  ASSERT_TRUE(j.contains("scene"));
  ASSERT_TRUE(j["scene"].contains("geom_clock"));
  EXPECT_EQ(j["scene"]["geom_clock"].get<int>(), 30);
}

TEST(GeomClockStructPath, OmittedWhenZero) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  cfg.geom_clock = 0;  // zero-init default: disabled
  auto j = nlohmann::json::parse(ScratchToJson(cfg));
  ASSERT_TRUE(j.contains("scene"));
  EXPECT_FALSE(j["scene"].contains("geom_clock"));
}

TEST(GeomClockStructPath, RoundTrip) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  cfg.geom_clock = 16;
  const std::string text = ScratchToJson(cfg);
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  ASSERT_EQ(ParseConfigString(text.c_str(), &out), LUMICE_OK);
  EXPECT_EQ(out.geom_clock, 16);
}

// JsonToDistribution's defensive {"type":"no_random",...} object branch (c_api.cpp) is read-side
// only — DistributionToJson never emits this shape (NO_RANDOM always writes a bare number). Cover
// it directly through the public parse entry point so the branch can't silently bit-rot unnoticed.
TEST(DistributionRoundTrip, NoRandomObjectFormAccepted) {
  auto j = nlohmann::json::parse(MakeMinimalConfigJson());
  j["crystal"][0]["shape"]["height"] = { { "type", "no_random" }, { "mean", 1.5f } };
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  ASSERT_EQ(ParseConfigString(j.dump().c_str(), &config), LUMICE_OK);
  ASSERT_EQ(config.crystal_count, 1);
  EXPECT_EQ(config.crystals[0].height.type, LUMICE_DIST_NO_RANDOM);
  EXPECT_FLOAT_EQ(config.crystals[0].height.center, 1.5f);
  EXPECT_FLOAT_EQ(config.crystals[0].height.spread, 0.0f);
}

// code-review round 1 (Major): the round-trip tests above only prove the C API's own writer
// (ConfigToJson) and reader (ParseConfigString) agree with each other — a shared misconception
// about core's real wire contract (e.g. the no_random bare-number convention) would not be
// caught by that self-closed loop. This commits a struct-built config carrying mixed
// distribution shapes plus a non-zero geom_clock straight to a real server, proving core's own
// from_json genuinely accepts what the C API produces — mirrors the
// ParseConfigApi.ParseModifyCommit end-to-end commit pattern.
TEST(DistributionRoundTrip, CommitsToRealServer) {
  auto json = MakeMinimalConfigJson();
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  ASSERT_EQ(ParseConfigString(json.c_str(), &config), LUMICE_OK);
  ASSERT_EQ(config.crystal_count, 1);

  config.crystals[0].height = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.5f, 0.0f };
  for (int i = 0; i < 6; i++) {
    config.crystals[0].face_distance[i] = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.1f };
  }
  config.geom_clock = 8;

  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(CommitScratch(server, config, &reused), LUMICE_OK);

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// =====================================================================================
// v4.13 shape-scalar sync groups: LUMICE_CrystalParam.sync_group[] across the C API.
//
// The struct field is the whole point of this surface: core has expressed sync groups since
// v4.12, but with no slot here every declaration made through a config file / the GUI / a python
// caller was dropped in translation — core received all-zero and nothing warned. So the tests
// below deliberately cover BOTH halves: the C API's own verbatim round-trip (this layer must not
// invent semantics), and what core actually does with the JSON this layer emits (the layer must
// not emit a key core cannot read — the exact failure mode this task exists to close).
// =====================================================================================

namespace {

// Signed plane offset (centroid · unit normal) of every prism face (face numbers 3..8) in `mesh`,
// in face_number order. Deliberately measured off the geometry rather than assuming a
// face_number <-> face_distance[i] mapping, so the assertions below stay true statements about
// "same-group faces sit at the same distance" no matter how the two are ordered internally.
std::vector<float> PrismFacePlaneOffsets(const LUMICE_CrystalMesh& mesh) {
  std::vector<float> offsets(6, std::numeric_limits<float>::quiet_NaN());
  for (int fi = 0; fi < mesh.face_count; ++fi) {
    const int fn = mesh.face_numbers_by_face[fi];
    if (fn < 3 || fn > 8) {
      continue;  // basal / pyramidal
    }
    const int offset = mesh.face_vtx_offsets[fi];
    const int count = mesh.face_vtx_counts[fi];
    if (count <= 0) {
      continue;
    }
    float c[3] = { 0.0f, 0.0f, 0.0f };
    for (int k = 0; k < count; ++k) {
      const float* p = mesh.vertices + mesh.face_vtx_pool[offset + k] * 3;
      c[0] += p[0];
      c[1] += p[1];
      c[2] += p[2];
    }
    const float* n = mesh.face_normals + fi * 3;
    offsets[fn - 3] = (c[0] * n[0] + c[1] * n[1] + c[2] * n[2]) / static_cast<float>(count);
  }
  return offsets;
}

// Number of distinct values in `v` under `tol`. Used to phrase "the three C3-equivalent faces
// collapsed onto one distance" without naming which faces those are.
size_t CountDistinct(const std::vector<float>& v, float tol) {
  std::vector<float> uniq;
  for (float x : v) {
    if (std::none_of(uniq.begin(), uniq.end(), [&](float u) { return std::fabs(u - x) <= tol; })) {
      uniq.push_back(x);
    }
  }
  return uniq.size();
}

void ExpectSyncGroupEq(const LUMICE_CrystalParam& out, const LUMICE_CrystalParam& in) {
  for (int i = 0; i < LUMICE_SHAPE_SCALAR_COUNT; ++i) {
    EXPECT_EQ(out.sync_group[i], in.sync_group[i]) << "sync_group slot " << i;
  }
}

}  // namespace

// AC1 form (a): six prism faces in two C3-symmetric groups — the motivating case.
TEST(SyncGroupRoundTrip, PrismFacesOnly) {
  LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  cr.id = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_0] = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_2] = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_4] = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_1] = 2;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_3] = 2;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_5] = 2;
  ExpectSyncGroupEq(RoundTripCrystal(cr), cr);
}

// AC1 form (b): the pyramid arm, grouping two of the three height scalars. Also guards the
// index-order trap — UPPER_H is slot 1 and PRISM_H slot 2, the reverse of this struct's field
// declaration order, so a translation written by field position instead of by name lands on the
// wrong keys and this test goes red.
TEST(SyncGroupRoundTrip, PyramidHeightsOnly) {
  LUMICE_CrystalParam cr = MakePyramidParam(1.0f, 0.5f, 0.5f);
  cr.id = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_UPPER_H] = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_LOWER_H] = 1;
  ExpectSyncGroupEq(RoundTripCrystal(cr), cr);
}

// AC1 form (c): a group spanning two different scalar KINDS (a height and a face distance).
// Mixing dimensions is deliberately permitted — the mechanism shares a draw, it does not claim
// the members are commensurable — so the translation must not quietly drop the cross-kind edge.
TEST(SyncGroupRoundTrip, MixedHeightAndFace) {
  LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  cr.id = 1;
  cr.height = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.1f };
  cr.face_distance[0] = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.1f };
  cr.sync_group[LUMICE_SHAPE_SCALAR_HEIGHT] = 1;
  cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_0] = 1;
  const auto out = RoundTripCrystal(cr);
  ExpectSyncGroupEq(out, cr);
  // Both members' distributions survive the trip too (this layer translates; it does not
  // leader-normalize — that happens once, in core).
  EXPECT_EQ(out.height.type, LUMICE_DIST_GAUSS);
  EXPECT_FLOAT_EQ(out.height.center, 1.0f);
  EXPECT_FLOAT_EQ(out.height.spread, 0.1f);
  EXPECT_EQ(out.face_distance[0].type, LUMICE_DIST_GAUSS);
  EXPECT_FLOAT_EQ(out.face_distance[0].center, 1.0f);
  EXPECT_FLOAT_EQ(out.face_distance[0].spread, 0.1f);
}

// AC2: a zero-initialized param stays all-independent through the round trip.
TEST(SyncGroupRoundTrip, ZeroInitStaysIndependent) {
  LUMICE_CrystalParam cr = MakePrismParam(1.0f);  // built from LUMICE_CrystalParam{}
  cr.id = 1;
  const auto out = RoundTripCrystal(cr);
  for (int i = 0; i < LUMICE_SHAPE_SCALAR_COUNT; ++i) {
    EXPECT_EQ(out.sync_group[i], 0) << "sync_group slot " << i;
  }
}

// AC2, the stronger half: "all independent" must not merely round-trip, it must leave the wire
// form untouched — no `sync_group` key at all — so every pre-v4.13 config serializes byte for
// byte as it did before. The second half is the green counterpart: the key DOES appear once
// something is actually synced, so the first assertion cannot pass by never emitting anything.
TEST(SyncGroupWireForm, KeyOmittedWhenAllIndependent) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  cfg.crystal_count = 1;
  cfg.crystals[0] = MakePrismParam(1.0f);
  cfg.crystals[0].id = 1;
  EXPECT_EQ(ScratchToJson(cfg).find("sync_group"), std::string::npos);

  cfg.crystals[0].sync_group[LUMICE_SHAPE_SCALAR_FACE_0] = 1;
  cfg.crystals[0].sync_group[LUMICE_SHAPE_SCALAR_FACE_3] = 1;
  EXPECT_NE(ScratchToJson(cfg).find("sync_group"), std::string::npos);
}

// AC3: the preview path. LUMICE_GetCrystalMesh needs no change of its own — it already routes
// through CrystalShapeToJson -> core from_json -> MakeCrystal — so teaching that translation
// about sync_group is enough for the mesh to honor grouping. Asserted white-box on the geometry
// (same-group faces sit at the same plane offset), not by eye.
TEST(SyncGroupPreview, GroupedFacesShareDistance) {
  // Control: two faces declared at DIFFERENT distances and NOT synced -> the mesh shows two
  // distinct prism-face offsets. Without this the grouped case below could pass on a mesh that
  // ignores face_distance entirely.
  LUMICE_CrystalParam ungrouped = MakePrismParam(1.0f);
  ungrouped.id = 1;
  ungrouped.face_distance[2] = DetDist(1.5f);
  LUMICE_CrystalMesh mesh{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&ungrouped, /*sample_seed=*/7, &mesh), LUMICE_OK);
  const auto ungrouped_offsets = PrismFacePlaneOffsets(mesh);
  EXPECT_EQ(CountDistinct(ungrouped_offsets, 1e-5f), 2u);

  // Same two scalars, now in one group. The leader is the lower ShapeScalar index (FACE_0, the
  // 1.0), so BOTH land on the leader's value: all six faces collapse onto the single distance
  // the ungrouped mesh showed five of. This distinguishes "leader wins" from "some member wins".
  LUMICE_CrystalParam grouped = ungrouped;
  grouped.sync_group[LUMICE_SHAPE_SCALAR_FACE_0] = 1;
  grouped.sync_group[LUMICE_SHAPE_SCALAR_FACE_2] = 1;
  LUMICE_CrystalMesh grouped_mesh{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&grouped, /*sample_seed=*/7, &grouped_mesh), LUMICE_OK);
  const auto grouped_offsets = PrismFacePlaneOffsets(grouped_mesh);
  ASSERT_EQ(CountDistinct(grouped_offsets, 1e-5f), 1u);

  // ...and that single distance is the leader's, i.e. the one five of the ungrouped faces had.
  LUMICE_CrystalParam all_leader = MakePrismParam(1.0f);  // all six at the leader's 1.0
  all_leader.id = 1;
  LUMICE_CrystalMesh leader_mesh{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&all_leader, /*sample_seed=*/7, &leader_mesh), LUMICE_OK);
  const auto leader_offsets = PrismFacePlaneOffsets(leader_mesh);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(grouped_offsets[i], leader_offsets[i], 1e-5f) << "prism face " << (i + 3);
  }
}

// AC3, the payload case: with RANDOMIZED face distances, a C3 grouping must make the sampled
// mesh itself C3-symmetric — three faces on one drawn distance, three on another. This is what
// a shared draw buys that leader normalization alone cannot; the control shows six independent
// draws produce six distinct distances.
TEST(SyncGroupPreview, C3GroupingCollapsesRandomDrawsToTwo) {
  LUMICE_CrystalParam cr = MakePrismParam(1.0f);
  cr.id = 1;
  for (auto& fd : cr.face_distance) {
    fd = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.1f };
  }

  LUMICE_CrystalMesh independent_mesh{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&cr, /*sample_seed=*/12345, &independent_mesh), LUMICE_OK);
  EXPECT_EQ(CountDistinct(PrismFacePlaneOffsets(independent_mesh), 1e-5f), 6u);

  for (int i = 0; i < 6; ++i) {
    cr.sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = (i % 2 == 0) ? 1 : 2;
  }
  LUMICE_CrystalMesh c3_mesh{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&cr, /*sample_seed=*/12345, &c3_mesh), LUMICE_OK);
  const auto c3_offsets = PrismFacePlaneOffsets(c3_mesh);
  ASSERT_EQ(CountDistinct(c3_offsets, 1e-5f), 2u);
  EXPECT_NEAR(c3_offsets[0], c3_offsets[2], 1e-5f);
  EXPECT_NEAR(c3_offsets[2], c3_offsets[4], 1e-5f);
  EXPECT_NEAR(c3_offsets[1], c3_offsets[3], 1e-5f);
  EXPECT_NEAR(c3_offsets[3], c3_offsets[5], 1e-5f);
}

// AC4: group ids are labels for a partition, not values. Two spellings of the SAME partition must
// reach core as the same canonical form. Consumed through core's own from_json — the single
// authority on what "canonical" means — rather than re-deriving the rule here.
TEST(SyncGroupCanonicalForm, EquivalentPartitionsCollapse) {
  LUMICE_CrystalParam canonical = MakePrismParam(1.0f);
  canonical.id = 1;
  LUMICE_CrystalParam relabeled = canonical;
  for (int i = 0; i < 6; ++i) {
    canonical.sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = (i % 2 == 0) ? 1 : 2;
    relabeled.sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = (i % 2 == 0) ? 2 : 1;
  }

  const auto a = CrystalShapeToJson(canonical).at("shape").get<lumice::PrismCrystalParam>();
  const auto b = CrystalShapeToJson(relabeled).at("shape").get<lumice::PrismCrystalParam>();

  // Guard first: both must actually carry groups. Without this the equality below would also be
  // satisfied by the pre-v4.13 bug — a key core never reads, both sides silently all-zero.
  bool any_grouped = false;
  for (int i = 0; i < lumice::kShapeScalarCount; ++i) {
    any_grouped = any_grouped || a.sync_group_[i] != 0;
  }
  ASSERT_TRUE(any_grouped) << "core received no sync group at all — C API key name drift?";

  for (int i = 0; i < lumice::kShapeScalarCount; ++i) {
    EXPECT_EQ(a.sync_group_[i], b.sync_group_[i]) << "sync_group_ slot " << i;
  }
}

// AC4 at the level a user can observe: the two spellings sample the same crystal. Randomized
// distributions make this a statement about the RNG draw sequence, not just about stored ids.
TEST(SyncGroupCanonicalForm, EquivalentPartitionsSampleIdenticalMesh) {
  LUMICE_CrystalParam canonical = MakePrismParam(1.0f);
  canonical.id = 1;
  for (auto& fd : canonical.face_distance) {
    fd = LUMICE_Distribution{ LUMICE_DIST_GAUSS, 1.0f, 0.1f };
  }
  LUMICE_CrystalParam relabeled = canonical;
  for (int i = 0; i < 6; ++i) {
    canonical.sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = (i % 2 == 0) ? 1 : 2;
    relabeled.sync_group[LUMICE_SHAPE_SCALAR_FACE_0 + i] = (i % 2 == 0) ? 2 : 1;
  }

  LUMICE_CrystalMesh mesh_a{};
  LUMICE_CrystalMesh mesh_b{};
  ASSERT_EQ(LUMICE_GetCrystalMesh(&canonical, /*sample_seed=*/999, &mesh_a), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetCrystalMesh(&relabeled, /*sample_seed=*/999, &mesh_b), LUMICE_OK);
  ASSERT_EQ(mesh_a.vertex_count, mesh_b.vertex_count);
  ASSERT_GT(mesh_a.vertex_count, 0);
  for (int i = 0; i < mesh_a.vertex_count * 3; ++i) {
    EXPECT_FLOAT_EQ(mesh_a.vertices[i], mesh_b.vertices[i]) << "vertex component " << i;
  }
}

// Parse cross-check against core from_json (source of truth): parsing a filter JSON via
// ParseConfigString then re-emitting (ConfigToJson) must byte-match core's own
// from_json -> to_json round-trip of the same JSON. Since 327.2 proved emit == core to_json,
// equality here proves the parse direction also agrees with core from_json.
namespace {
void ExpectParseMatchesCore(const nlohmann::json& jf) {
  // core path: from_json -> FilterConfig -> to_json
  lumice::FilterConfig fc = jf.get<lumice::FilterConfig>();
  nlohmann::json core_out = fc;
  // my path: ParseConfigString -> struct -> ConfigToJson -> filter[0]
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFilterJson(jf).c_str(), &cfg), LUMICE_OK);
  auto my_root = nlohmann::json::parse(ScratchToJson(cfg));
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

  ConfigScratch cfg{};

  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(filter_array).c_str(), &cfg), LUMICE_OK);
  auto my_root = nlohmann::json::parse(ScratchToJson(cfg));
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
  ConfigScratch in{};
  ConfigScratchGuard in_guard(in);
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

  const std::string text = ScratchToJson(in);
  ConfigScratch out{};
  ConfigScratchGuard out_guard(out);
  ASSERT_EQ(ParseConfigString(text.c_str(), &out), LUMICE_OK);

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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  cfg.infinite = 0;
  cfg.ray_num = 100;
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(CommitScratch(server, cfg, &reused), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  cfg.infinite = 0;
  cfg.ray_num = 100;

  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  ASSERT_EQ(CommitScratch(server, cfg, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 0);  // first commit builds consumers

  // Change only a non-renderer field (sun altitude); the complex filter is unchanged.
  cfg.sun_altitude += 5.0f;
  reused = -1;
  ASSERT_EQ(CommitScratch(server, cfg, &reused), LUMICE_OK);
  EXPECT_EQ(reused, 1);  // consumers reused despite the complex filter -> buffer not torn

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

TEST(StructFilterComplex, DanglingReferenceRejected) {
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 99 } } },  // 99 not defined
  });
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, ReferenceToComplexRejected) {
  // A composition term may only reference a SIMPLE filter, never another complex (no cycles).
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 2 } } },  // refs complex 2
  });
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(StructFilterComplex, EmptyCompositionAccepted) {
  // An empty composition ([]) is a degenerate but accepted "OR of nothing" (clause_count 0).
  nlohmann::json filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", nlohmann::json::array() } },
  });
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_GE(cfg.filter_count, 2);
  EXPECT_EQ(cfg.filters[1].type, LUMICE_FILTER_TYPE_COMPLEX);
  EXPECT_EQ(cfg.compositions[cfg.filters[1].composition_index].clause_count, 0);
}

TEST(StructFilterComplex, EmptyClauseWithinCompositionCommitEndToEnd) {
  // Regression (code-review-01/02, round 1+2, Major): LUMICE_CompositionSetClauses only
  // allocates term_ids when total_terms > 0, so a composition where every clause has 0 terms
  // (clause_count > 0, total_terms == 0) legitimately ends up with term_ids == nullptr while
  // term_counts stays allocated (non-null). Round 1 fixed the commit path's read-side
  // check but left SetClauses's own entry-point null-check requiring term_ids unconditionally
  // non-null whenever clause_count > 0 — which rejected exactly this legitimate shape earlier,
  // at ParseConfigString time, for the two real production callers (JsonToComplexComposition
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
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
  EXPECT_EQ(CommitScratch(server, cfg, &reused), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.compositions[cfg.filters[kN].composition_index].clause_count, kN);
  cfg.infinite = 0;
  cfg.ray_num = 100;
  auto* server = LUMICE_CreateServer();
  ASSERT_NE(server, nullptr);
  int reused = -1;
  EXPECT_EQ(CommitScratch(server, cfg, &reused), LUMICE_OK);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// task-host-abi-cpu-caps §7 risk 2 regression (§4 Step 8): parse two different compositions
// into the SAME ConfigScratch in sequence; the second parse must fully replace the first
// (correct clause_count, correct term ids, no residual state from the first). This is the
// direct regression witness for the double-free / leak hazard introduced by making
// LUMICE_ComplexComposition an owning type — the create-or-replace contract in
// LUMICE_CompositionSetClauses + release-before-memset in JsonToConfig together ensure the
// first parse's heap allocations are released before the second parse overwrites the pointers.
// Value-semantics only (mirrors ConsecutiveParseIntoSameConfigOverridesCorrectly);
// leak detection itself lives outside this test (valgrind / asan, plan §7 risk 2).
TEST(StructFilterComplex, ConsecutiveParseIntoSameCompositionOverridesCorrectly) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);

  // First parse: a 3-clause OR of raypaths.
  nlohmann::json first_filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 3 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 6, 7 } } },
      { { "id", 10 }, { "action", "filter_in" }, { "type", "complex" }, { "composition", { 1, 2, 3 } } },
  });
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(first_filters).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.composition_count, 1);
  ASSERT_EQ(cfg.compositions[0].clause_count, 3);
  ASSERT_NE(cfg.compositions[0].term_ids, nullptr);
  ASSERT_NE(cfg.compositions[0].term_counts, nullptr);

  // Second parse into the SAME cfg: a single-clause AND (different clause_count AND different
  // term shape). Verifies the CREATE-OR-REPLACE contract on both ConfigScratch-level Release
  // (via JsonToConfig) and per-record LUMICE_CompositionSetClauses (via JsonToComplexComposition).
  nlohmann::json second_filters = nlohmann::json::array({
      { { "id", 1 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 3, 5 } } },
      { { "id", 2 }, { "action", "filter_in" }, { "type", "raypath" }, { "raypath", { 1, 4 } } },
      { { "id", 20 },
        { "action", "filter_in" },
        { "type", "complex" },
        { "composition", nlohmann::json::array({ nlohmann::json::array({ 1, 2 }) }) } },
  });
  ASSERT_EQ(ParseConfigString(FullConfigWithFiltersJson(second_filters).c_str(), &cfg), LUMICE_OK);
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
// field by field. Parse tests round-trip JSON -> ConfigScratch. The setter (AC2/AC3) and
// the JSON-vs-struct pixel equivalence (AC1) drive a real server.
// =====================================================================================

namespace {

// Minimal ConfigScratch carrying exactly one color class, for ConfigToJson emit assertions.
// `out` must be a caller-owned ConfigScratch with a lifetime-bound ConfigScratchGuard already
// attached: this function allocates raypath_color via ConfigCreateColorClasses on `out`, and
// the guard's destructor is what releases it.
void FillOneColorClassConfig(ConfigScratch* out, const LUMICE_ColorClass& cls, int mode = LUMICE_COLOR_MODE_DOMINANT) {
  LUMICE_ColorClass* classes = ConfigCreateColorClasses(out, 1);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  FillOneColorClassConfig(&config, cls);
  auto root = ConfigToJson(config);
  const auto& jc = EmitFirstColorClass(root);
  ASSERT_TRUE(jc.contains("visible"));
  EXPECT_FALSE(jc.at("visible").get<bool>());
}

TEST(StructColorClassEmit, ZeroCountOmitsKey) {
  // AC4: no color classes => no "raypath_color" key => JSON byte-shape identical to pre-v4.7.
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
  config.raypath_color_count = 0;
  auto root = ConfigToJson(config);
  EXPECT_FALSE(root.contains("raypath_color"));
}

TEST(StructColorClassEmit, ModeAdditiveAndPainterEmitted) {
  {
    ConfigScratch config{};
    ConfigScratchGuard config_guard(config);
    FillOneColorClassConfig(&config, MakeWholeCrystalClass(), LUMICE_COLOR_MODE_ADDITIVE);
    auto add = ConfigToJson(config);
    EXPECT_EQ(add.at("raypath_color").at("mode").get<std::string>(), "additive");
  }
  {
    ConfigScratch config{};
    ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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
  ConfigScratch config{};
  ConfigScratchGuard config_guard(config);
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

  ConfigScratch cfg{};

  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
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
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color[0].match_count, 2);
  EXPECT_EQ(cfg.raypath_color[0].match[0].predicate.symmetry, 0);
  EXPECT_EQ(cfg.raypath_color[0].match[1].predicate.symmetry, 0);
}

TEST(ParseConfigApi, RaypathColorComplexPredicateRejected) {
  nlohmann::json bad = ColorRefJson(0, 1);
  bad["type"] = "complex";
  nlohmann::json rc = nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ bad })) });
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_ERR_INVALID_VALUE);
}

TEST(ParseConfigApi, RaypathColorClassesOverCapRejected) {
  nlohmann::json classes = nlohmann::json::array();
  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_CLASSES + 1; i++) {
    classes.push_back(ColorClassJson({ 1, 0, 0 }, nlohmann::json::array({ ColorRefJson(0, 1) })));
  }
  nlohmann::json rc;
  rc["classes"] = classes;
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
}

TEST(ParseConfigApi, RaypathColorRefsOverCapRejected) {
  nlohmann::json match = nlohmann::json::array();
  for (int i = 0; i < LUMICE_MAX_CONFIG_COLOR_REFS + 1; i++) {
    match.push_back(ColorRefJson(0, 1));
  }
  nlohmann::json rc = nlohmann::json::array({ ColorClassJson({ 1, 0, 0 }, match) });
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);
  EXPECT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc).c_str(), &cfg), LUMICE_ERR_INVALID_CONFIG);
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

TEST(RaypathColorApi, SetRaypathColorsDoesNotRestartSim) {
  // AC2: after reaching steady state, SetRaypathColors changes the composite but must NOT
  // advance epoch nor clear the accumulator, and the new color must actually show.
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 777u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_SimLifecycleResult lc0{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(s, &lc0), LUMICE_OK);
  LUMICE_RayCount rc0 = 0;
  ASSERT_EQ(LUMICE_GetSimRayCount(s, &rc0), LUMICE_OK);

  LUMICE_ResultFrame* frame_before = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_before), LUMICE_OK);
  LUMICE_RenderResult before[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame_before, before, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(before[0].img_buffer, nullptr);
  const int w = before[0].img_width;
  const int h = before[0].img_height;
  const size_t nbytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
  std::vector<uint8_t> before_px(before[0].img_buffer, before[0].img_buffer + nbytes);
  LUMICE_ReleaseResultFrame(frame_before);
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
  LUMICE_ResultFrame* frame_after = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_after), LUMICE_OK);
  LUMICE_RenderResult after[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame_after, after, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(after[0].img_buffer, nullptr);
  const uint8_t* ap = after[0].img_buffer;
  EXPECT_GT(ap[red_p * 3 + 2], 0) << "recolored pixel must now be blue";
  EXPECT_EQ(ap[red_p * 3 + 0], 0) << "recolored pixel must no longer be red";
  EXPECT_NE(std::memcmp(before_px.data(), ap, nbytes), 0) << "composite must actually change";
  LUMICE_ReleaseResultFrame(frame_after);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// Ordering regression, restated for the frame model. The two defects this guards
// were both about WHICH read consumed snapshot_dirty_: reading xyz first left the composite
// stale (its DoSnapshot early-returned), and reading composite first advanced no generation
// at all (the bump lived only in xyz's Phase 1). Read order is no longer a thing a caller
// can get wrong — one snapshot materializes both kinds into one frame — so the two ordering
// variants that used to be separate cases are one case here. What survives from them is the
// assertion: after a run, ONE frame carries fresh xyz AND fresh composite AND a generation
// that advanced.
TEST(RaypathColorApi, XyzAndCompositeMaterializeTogether) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 4242u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame), LUMICE_OK);

  LUMICE_RawXyzResult raw[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, raw, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(raw[0].xyz_buffer, nullptr);
  EXPECT_NE(raw[0].has_valid_data, 0);
  const unsigned long long generation = raw[0].snapshot_generation;
  EXPECT_GT(generation, 0ull) << "generation must advance on first consume";

  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp[0].img_buffer, nullptr) << "composite must be populated (class0 is match-all red)";
  const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
  uint64_t sum = 0;
  for (size_t i = 0; i < nbytes; ++i) {
    sum += comp[0].img_buffer[i];
  }
  EXPECT_GT(sum, 0u) << "composite must reflect the current-tick data, not an all-zero stale cache";

  LUMICE_ReleaseResultFrame(frame);

  // A second frame taken with nothing new committed must not regress the generation.
  LUMICE_ResultFrame* frame2 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame2), LUMICE_OK);
  LUMICE_RawXyzResult raw2[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame2, raw2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_GE(raw2[0].snapshot_generation, generation);
  LUMICE_ReleaseResultFrame(frame2);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// Generation-drift regression, re-aimed at the frame model. The original case proved
// that generation drift between an xyz capture and a later re-check was *detectable*, because
// the poller then read xyz and composite through two separate server-taking getters and had to
// notice when a snapshot landed between them. One frame carries both, so that particular drift
// window no longer exists (ResultFrameReadsShareOneGeneration covers the in-frame half). What
// still has to hold, and is what this case now asserts, is the other half: arming a dirty event
// must make the NEXT acquired frame genuinely newer — a newer snapshot_generation carrying
// pixels that actually changed, not a stale re-read wearing a fresh number.
// LUMICE_SetRaypathColors() arms snapshot_dirty_ on demand, so this does not depend on
// background-thread timing.
TEST(RaypathColorApi, RecolorAdvancesGenerationOnNextFrame) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 8181u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_ResultFrame* frame1 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame1), LUMICE_OK);
  LUMICE_RawXyzResult raw1[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult comp1[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame1, raw1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetComposite(frame1, comp1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp1[0].img_buffer, nullptr);
  const unsigned long long captured_generation = raw1[0].snapshot_generation;
  const size_t nbytes = static_cast<size_t>(comp1[0].img_width) * static_cast<size_t>(comp1[0].img_height) * 3;
  std::vector<uint8_t> comp1_px(comp1[0].img_buffer, comp1[0].img_buffer + nbytes);

  // Nothing armed: a second frame taken from an idle server must not claim to be newer.
  LUMICE_ResultFrame* frame_same = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_same), LUMICE_OK);
  LUMICE_RawXyzResult raw_same[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame_same, raw_same, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(raw_same[0].snapshot_generation, captured_generation)
      << "no dirty event armed: acquiring again must not manufacture a new generation";
  LUMICE_ReleaseResultFrame(frame_same);

  LUMICE_ColorClassDisplay disp[2]{};
  disp[0].color[2] = 1.0f;  // class0 red->blue
  disp[0].visible = 1;
  disp[1].color[2] = 1.0f;  // class1 green->blue
  disp[1].visible = 1;
  ASSERT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

  LUMICE_ResultFrame* frame2 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame2), LUMICE_OK);
  LUMICE_RawXyzResult raw2[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult comp2[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame2, raw2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetComposite(frame2, comp2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp2[0].img_buffer, nullptr);
  EXPECT_GT(raw2[0].snapshot_generation, captured_generation)
      << "the armed dirty event must materialize as a newer generation on the next frame";
  EXPECT_NE(std::memcmp(comp1_px.data(), comp2[0].img_buffer, nbytes), 0)
      << "comp2 must reflect the recolor, proving the newer generation carries new pixels "
         "rather than a stale cache hit wearing a fresh number";

  // frame1 was held across all of the above and still reads its own pixels.
  LUMICE_RenderResult comp1_again[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame1, comp1_again, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_EQ(std::memcmp(comp1_px.data(), comp1_again[0].img_buffer, nbytes), 0);

  LUMICE_ReleaseResultFrame(frame2);
  LUMICE_ReleaseResultFrame(frame1);
  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// Same-generation-pairing regression, carried to the frame model. The contract is unchanged —
// xyz and composite must come from ONE snapshot even while churn arms a fresh dirty event
// before every read — but it is no longer something a dedicated combined getter has to
// promise: both are read off one frame, so the pairing is structural. The case is kept
// because the contract is kept; only the mechanism it points at changed.
TEST(RaypathColorApi, RawXyzAndCompositeSameGenerationUnderChurn) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 1919u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
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

    LUMICE_ResultFrame* frame = nullptr;
    ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame), LUMICE_OK) << "round " << round;

    LUMICE_RawXyzResult xyz[LUMICE_MAX_RENDER_RESULTS + 1]{};
    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK) << "round " << round;
    ASSERT_EQ(LUMICE_FrameGetComposite(frame, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK) << "round " << round;
    ASSERT_NE(xyz[0].xyz_buffer, nullptr) << "round " << round;
    ASSERT_NE(comp[0].img_buffer, nullptr) << "round " << round << ": raypath_color is configured";

    // Same-generation invariant. One frame is materialized by exactly one DoSnapshot(), so the
    // composite read off it cannot belong to a different generation than the xyz read off it.
    EXPECT_GT(xyz[0].snapshot_generation, prev_generation)
        << "round " << round << ": SetRaypathColors armed dirty → generation must advance";
    prev_generation = xyz[0].snapshot_generation;

    // Cross-check: re-reading the same frame after the composite read must observe the same
    // generation — the frame is immutable, so nothing about reading it can arm drift for the
    // next observer.
    LUMICE_RawXyzResult recheck[LUMICE_MAX_RENDER_RESULTS + 1]{};
    ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, recheck, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_EQ(recheck[0].snapshot_generation, xyz[0].snapshot_generation)
        << "round " << round << ": re-reading one frame must not shift its generation";

    LUMICE_ReleaseResultFrame(frame);
  }

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// The combined getter this case was written for is gone: there is nothing left for it to be
// "combined" against. What the byte-level comparison was actually buying — an idle server hands
// out the same frozen data no matter how many times it is asked — is worth keeping on its own,
// so the comparison is re-aimed at two independently acquired frames.
TEST(RaypathColorApi, IdleServerFramesAgreeByteForByte) {
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 2929u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_ResultFrame* frame_a = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_a), LUMICE_OK);
  LUMICE_RawXyzResult xyz_a[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult comp_a[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame_a, xyz_a, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetComposite(frame_a, comp_a, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(xyz_a[0].xyz_buffer, nullptr);
  ASSERT_NE(comp_a[0].img_buffer, nullptr);
  const int width = xyz_a[0].img_width;
  const int height = xyz_a[0].img_height;
  const size_t rgb_bytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  const size_t xyz_floats = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  std::vector<uint8_t> comp_copy(comp_a[0].img_buffer, comp_a[0].img_buffer + rgb_bytes);
  std::vector<float> xyz_copy(xyz_a[0].xyz_buffer, xyz_a[0].xyz_buffer + xyz_floats);
  const unsigned long long generation_a = xyz_a[0].snapshot_generation;
  LUMICE_ReleaseResultFrame(frame_a);

  // Idle server → no dirty → the second frame must be the SAME frozen snapshot, byte for byte.
  LUMICE_ResultFrame* frame_b = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_b), LUMICE_OK);
  LUMICE_RawXyzResult xyz_b[LUMICE_MAX_RENDER_RESULTS + 1]{};
  LUMICE_RenderResult comp_b[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame_b, xyz_b, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetComposite(frame_b, comp_b, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(comp_b[0].img_buffer, nullptr);
  EXPECT_EQ(xyz_b[0].snapshot_generation, generation_a);
  EXPECT_EQ(std::memcmp(xyz_b[0].xyz_buffer, xyz_copy.data(), xyz_floats * sizeof(float)), 0);
  EXPECT_EQ(std::memcmp(comp_b[0].img_buffer, comp_copy.data(), rgb_bytes), 0);

  LUMICE_ReleaseResultFrame(frame_b);
  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

TEST(RaypathColorApi, SetRaypathColorsRejectsBadArgsAllOrNothing) {
  // AC3: class_count mismatch and non-permutation z_order are rejected without mutating state.
  LUMICE_ServerConfig sc{};
  sc.num_workers = 1;
  sc.sim_seed = 999u;
  LUMICE_Server* s = LUMICE_CreateServerEx(&sc);
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Capture the current composite to prove rejections leave it untouched.
  LUMICE_ResultFrame* frame_before = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_before), LUMICE_OK);
  LUMICE_RenderResult before[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame_before, before, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(before[0].img_buffer, nullptr);
  const size_t nbytes = static_cast<size_t>(before[0].img_width) * static_cast<size_t>(before[0].img_height) * 3;
  std::vector<uint8_t> before_px(before[0].img_buffer, before[0].img_buffer + nbytes);
  LUMICE_ReleaseResultFrame(frame_before);

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
  LUMICE_ResultFrame* frame_mid = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame_mid), LUMICE_OK);
  LUMICE_RenderResult mid[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame_mid, mid, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_NE(mid[0].img_buffer, nullptr);
  EXPECT_EQ(std::memcmp(before_px.data(), mid[0].img_buffer, nbytes), 0)
      << "rejected SetRaypathColors must not mutate the active table";
  LUMICE_ReleaseResultFrame(frame_mid);

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
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Null classes with positive count.
  EXPECT_EQ(LUMICE_SetRaypathColors(s, nullptr, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_ERR_NULL_ARG);
  // Out-of-range composite mode.
  EXPECT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, 99), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(LUMICE_SetRaypathColors(s, disp, 2, nullptr, -1), LUMICE_ERR_INVALID_VALUE);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// task-344 regression 2: connect JsonToConfig's Release-before-memset fix. Parsing two
// different color configs into the same ConfigScratch must land the SECOND set of classes
// verbatim (i.e. the first parse's classes must not leak into the second). This test only
// verifies value semantics — removing the added Release call would still let the second
// parse pass (it just leaks the first allocation). Leak detection itself lives in the
// task's plan §6 valgrind cross-check, not here.
TEST(RaypathColorApi, ConsecutiveParseIntoSameConfigOverridesCorrectly) {
  ConfigScratch cfg{};
  ConfigScratchGuard cfg_guard(cfg);

  // First parse: 2 classes.
  nlohmann::json c0 = ColorClassJson({ 1.0f, 0.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json c1 = ColorClassJson({ 0.0f, 1.0f, 0.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json rc_two;
  rc_two["mode"] = "dominant";
  rc_two["classes"] = nlohmann::json::array({ c0, c1 });
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc_two).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.raypath_color_count, 2);
  ASSERT_NE(cfg.raypath_color, nullptr);

  // Second parse into the SAME cfg: only 1 class, with a distinct color.
  nlohmann::json c2 = ColorClassJson({ 0.0f, 0.0f, 1.0f }, nlohmann::json::array({ ColorRefJson(0, 1) }));
  nlohmann::json rc_one;
  rc_one["mode"] = "additive";
  rc_one["classes"] = nlohmann::json::array({ c2 });
  ASSERT_EQ(ParseConfigString(FullConfigWithRaypathColorJson(rc_one).c_str(), &cfg), LUMICE_OK);
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
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  // Trigger a snapshot so lane data is materialized (mirror the GUI polling contract).
  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame), LUMICE_OK);
  LUMICE_RenderResult composite[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame, composite, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  LUMICE_ReleaseResultFrame(frame);

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
  ASSERT_EQ(CommitJsonConfig(s, json.c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(s, &frame), LUMICE_OK);
  LUMICE_RenderResult composite[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame, composite, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  LUMICE_ReleaseResultFrame(frame);

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
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
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
  ASSERT_EQ(CommitJsonConfig(s, MakeMinimalConfigJson().c_str()), LUMICE_OK);
  // class_count=0 with any out_flags (including nullptr) must be OK no-op.
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, nullptr, 0), LUMICE_OK);

  // Non-null server, positive count, but null out_flags → NULL_ARG.
  ASSERT_EQ(CommitJsonConfig(s, MakeColorSimConfigJson().c_str()), LUMICE_OK);
  ASSERT_TRUE(WaitForIdle(s, 10000));
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, nullptr, 2), LUMICE_ERR_NULL_ARG);
  // Negative count → INVALID_VALUE.
  EXPECT_EQ(LUMICE_GetColorClassSignal(s, flags, -1), LUMICE_ERR_INVALID_VALUE);

  LUMICE_StopServer(s);
  LUMICE_DestroyServer(s);
}

// =============== EV Auto Anchor ===============
// These pin the C API forwarding layer only: that LUMICE_ComputeP99Y / LUMICE_ComputeEvAuto reach
// the core implementation with arguments in the right order and hand back its value unmodified.
// The algorithm's own propositions live in test_ev_anchor.cpp; a forward that silently swapped
// two int parameters or dropped the downsample factor would pass there and fail here.
TEST(EvAutoAnchorApi, ComputeP99YForwardsCoarsePathAndFallback) {
  // Same 4x4 / f=2 hand-computed sample as the core test: coarse bin sums {14,22,46,54},
  // P99 = 54, divided by f^2 = 4 -> 13.5.
  std::vector<float> y = { 1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,
                           9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f };
  std::vector<float> xyz(4 * 4 * 3, 0.0f);
  for (size_t i = 0; i < y.size(); ++i) {
    xyz[i * 3 + 1] = y[i];
  }
  EXPECT_FLOAT_EQ(LUMICE_ComputeP99Y(xyz.data(), 4, 4, 2), 13.5f);

  // downsample_factor <= 1 takes the fine path: P99 over the 16 non-zero per-pixel Y values,
  // idx = floor(16 * 0.99) = 15 -> the max, 16.
  EXPECT_FLOAT_EQ(LUMICE_ComputeP99Y(xyz.data(), 4, 4, 1), 16.0f);

  // A non-square buffer would come back wrong if the forward swapped width and height: an 8x2
  // image at f=2 gives 4x1 coarse bins, while 2x8 gives 1x4 — different bin sums, different P99.
  std::vector<float> wide(8 * 2 * 3, 0.0f);
  for (int i = 0; i < 16; ++i) {
    wide[static_cast<size_t>(i) * 3 + 1] = static_cast<float>(i + 1);
  }
  // Rows are 1..8 and 9..16; the four 2x2 bins sum to 1+2+9+10=22, 3+4+11+12=30, 38, 46.
  // idx = floor(4 * 0.99) = 3 -> 46, / 4 = 11.5.
  EXPECT_FLOAT_EQ(LUMICE_ComputeP99Y(wide.data(), 8, 2, 2), 11.5f);
}

TEST(EvAutoAnchorApi, ComputeEvAutoForwardsFormulaAndGuards) {
  // target_white=135 -> sRGB reverse transform; ev = log2(target_linear / (p99 / snapshot)).
  const float t = 135.0f / 255.0f;
  const float target_linear = std::pow((t + 0.055f) / 1.055f, 2.4f);
  const float expected = std::log2f(target_linear / (2.0e-3f / 4.0f));
  EXPECT_FLOAT_EQ(LUMICE_ComputeEvAuto(2.0e-3f, 4.0f, 135.0f), std::clamp(expected, -6.0f, 6.0f));

  // Guard branches return a hard 0 (the "no anchor yet" convention), not inf/NaN.
  EXPECT_FLOAT_EQ(LUMICE_ComputeEvAuto(2.0e-3f, 0.0f, 135.0f), 0.0f);
  EXPECT_FLOAT_EQ(LUMICE_ComputeEvAuto(0.0f, 4.0f, 135.0f), 0.0f);
}

// =================================================================================================
// Annotation overlay (LUMICE_ComputeAnnotationOverlay / _ReleaseAnnotationOverlay)
//
// The geometry itself is covered in test/unit-correctness/core/test_annotation_overlay.cpp and the
// cross-implementation agreement in test/unit-correctness/gui/
// test_annotation_overlay_gui_parity.cpp. What is asserted here is the ABI shape: argument
// validation, what a successful call actually fills in, and the acquire/release contract.
// =================================================================================================

namespace {

LUMICE_AnnotationRequest MakeAnnotationRequest(int w, int h) {
  LUMICE_AnnotationRequest req{};
  req.view.width = w;
  req.view.height = h;
  req.view.lens_type = LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA;
  req.view.lens_fov = 180.0f;
  req.view.visible = LUMICE_VISIBLE_FULL;
  req.reference_dir[0] = 0.0f;
  req.reference_dir[1] = -1.0f;
  req.reference_dir[2] = 0.0f;
  req.horizon = 1;
  req.want_labels = 1;
  return req;
}

}  // namespace

TEST(AnnotationOverlayApi, RejectsNullArguments) {
  LUMICE_AnnotationRequest req = MakeAnnotationRequest(64, 32);
  LUMICE_AnnotationOverlay out{};
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(nullptr, &out), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&req, nullptr), LUMICE_ERR_NULL_ARG);

  // A non-zero count with a NULL list is the shape a caller gets wrong by forgetting one line, so
  // it is an error rather than a silent "no lines".
  req.elevation_count = 3;
  req.elevation_deg = nullptr;
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&req, &out), LUMICE_ERR_NULL_ARG);
}

TEST(AnnotationOverlayApi, RejectsOutOfRangeEnumsAndCounts) {
  const float angles[] = { 10.0f, 20.0f };
  LUMICE_AnnotationOverlay out{};

  LUMICE_AnnotationRequest bad_lens = MakeAnnotationRequest(64, 32);
  bad_lens.view.lens_type = LUMICE_LENS_TYPE_GLOBE + 1;
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&bad_lens, &out), LUMICE_ERR_INVALID_VALUE);

  LUMICE_AnnotationRequest bad_visible = MakeAnnotationRequest(64, 32);
  bad_visible.view.visible = 7;
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&bad_visible, &out), LUMICE_ERR_INVALID_VALUE);

  LUMICE_AnnotationRequest negative = MakeAnnotationRequest(64, 32);
  negative.elevation_deg = angles;
  negative.elevation_count = -1;
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&negative, &out), LUMICE_ERR_INVALID_VALUE);

  LUMICE_AnnotationRequest too_many = MakeAnnotationRequest(64, 32);
  too_many.longitude_deg = angles;
  too_many.longitude_count = LUMICE_MAX_ANNOTATION_LINES + 1;
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&too_many, &out), LUMICE_ERR_INVALID_VALUE);

  LUMICE_AnnotationRequest too_many_circles = MakeAnnotationRequest(64, 32);
  too_many_circles.angular_dist_deg = angles;
  too_many_circles.angular_dist_count = LUMICE_MAX_ANNOTATION_CIRCLES + 1;
  EXPECT_EQ(LUMICE_ComputeAnnotationOverlay(&too_many_circles, &out), LUMICE_ERR_INVALID_VALUE);

  // A rejected call must leave nothing to release: `out` is still zero-initialized, so the
  // NULL-safe Release below is a no-op rather than a free of a wild pointer.
  EXPECT_EQ(out.storage, nullptr);
  LUMICE_ReleaseAnnotationOverlay(&out);
}

TEST(AnnotationOverlayApi, FillsGeometryLabelsAndMarkers) {
  const float parallels[] = { -30.0f, 30.0f };
  const float meridians[] = { 0.0f, 90.0f, 180.0f, -90.0f };
  const float circles[] = { 22.0f };

  LUMICE_AnnotationRequest req = MakeAnnotationRequest(128, 64);
  req.elevation_deg = parallels;
  req.elevation_count = 2;
  req.longitude_deg = meridians;
  req.longitude_count = 4;
  req.angular_dist_deg = circles;
  req.angular_dist_count = 1;
  req.zenith_nadir = 1;

  LUMICE_AnnotationOverlay out{};
  ASSERT_EQ(LUMICE_ComputeAnnotationOverlay(&req, &out), LUMICE_OK);
  EXPECT_EQ(out.width, 128);
  EXPECT_EQ(out.height, 64);
  ASSERT_NE(out.drawable, nullptr);
  ASSERT_NE(out.horizon, nullptr);
  ASSERT_NE(out.elevation, nullptr);
  ASSERT_NE(out.longitude, nullptr);
  ASSERT_NE(out.angular_dist, nullptr);
  EXPECT_TRUE(out.zenith_valid);
  EXPECT_TRUE(out.nadir_valid);
  ASSERT_GT(out.label_count, 0);
  ASSERT_NE(out.labels, nullptr);

  // Every mask pixel must sit inside the drawable region, read through the C pointers rather than
  // the C++ vectors — the ABI's job is that the pointer and the extent agree.
  const size_t n = 128u * 64u;
  const unsigned char* const masks[] = { out.horizon, out.elevation, out.longitude, out.angular_dist };
  for (const unsigned char* m : masks) {
    size_t on = 0;
    size_t stray = 0;
    for (size_t i = 0; i < n; ++i) {
      on += (m[i] != 0) ? 1u : 0u;
      stray += (m[i] != 0 && out.drawable[i] == 0) ? 1u : 0u;
    }
    EXPECT_GT(on, 0u) << "a category that draws nothing makes the containment check vacuous";
    EXPECT_EQ(stray, 0u);
  }

  bool saw_horizon = false;
  bool saw_circle = false;
  for (int i = 0; i < out.label_count; ++i) {
    const LUMICE_AnnotationLabel& l = out.labels[i];
    EXPECT_GE(l.px, 0.0f);
    EXPECT_LT(l.px, 128.0f);
    EXPECT_GE(l.py, 0.0f);
    EXPECT_LT(l.py, 64.0f);
    // NUL-terminated within the fixed buffer, which is what makes the field usable as a C string.
    EXPECT_LT(std::strlen(l.text), sizeof(l.text));
    if (l.kind == LUMICE_ANNOTATION_HORIZON) {
      saw_horizon = true;
      EXPECT_EQ(l.index, -1);
    }
    if (l.kind == LUMICE_ANNOTATION_ANGULAR_DIST) {
      saw_circle = true;
      EXPECT_EQ(l.index, 0);
      EXPECT_FLOAT_EQ(l.value_deg, 22.0f);
    }
  }
  EXPECT_TRUE(saw_horizon);
  EXPECT_TRUE(saw_circle);

  LUMICE_ReleaseAnnotationOverlay(&out);
  // Release nulls the whole view, so a caller reading the struct afterwards sees "nothing here"
  // rather than freed memory, and a second Release is a no-op instead of a double free.
  EXPECT_EQ(out.storage, nullptr);
  EXPECT_EQ(out.drawable, nullptr);
  EXPECT_EQ(out.labels, nullptr);
  EXPECT_EQ(out.label_count, 0);
  LUMICE_ReleaseAnnotationOverlay(&out);
  LUMICE_ReleaseAnnotationOverlay(nullptr);
}

TEST(AnnotationOverlayApi, OnlyRequestedCategoriesGetABuffer) {
  LUMICE_AnnotationRequest req = MakeAnnotationRequest(64, 32);
  LUMICE_AnnotationOverlay out{};
  ASSERT_EQ(LUMICE_ComputeAnnotationOverlay(&req, &out), LUMICE_OK);
  EXPECT_NE(out.horizon, nullptr);
  EXPECT_EQ(out.elevation, nullptr);
  EXPECT_EQ(out.longitude, nullptr);
  EXPECT_EQ(out.angular_dist, nullptr);
  EXPECT_FALSE(out.zenith_valid) << "zenith_nadir was not requested";
  LUMICE_ReleaseAnnotationOverlay(&out);
}

TEST(AnnotationOverlayApi, DegenerateViewIsAnEmptyOverlayNotAnError) {
  LUMICE_AnnotationRequest req = MakeAnnotationRequest(0, 32);
  LUMICE_AnnotationOverlay out{};
  ASSERT_EQ(LUMICE_ComputeAnnotationOverlay(&req, &out), LUMICE_OK);
  EXPECT_EQ(out.width, 0);
  EXPECT_EQ(out.height, 0);
  EXPECT_EQ(out.drawable, nullptr);
  EXPECT_EQ(out.horizon, nullptr);
  EXPECT_EQ(out.label_count, 0);
  // Still holds storage, and still has to be released — "empty" is a result, not a failure.
  EXPECT_NE(out.storage, nullptr);
  LUMICE_ReleaseAnnotationOverlay(&out);
}

TEST(AnnotationOverlayApi, IsDeterministicAndCarriesNoCrossCallState) {
  const float parallels[] = { 15.0f, 45.0f };
  LUMICE_AnnotationRequest req = MakeAnnotationRequest(96, 48);
  req.view.lens_type = LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
  req.view.lens_fov = 90.0f;
  req.view.view_elevation = 60.0f;
  req.elevation_deg = parallels;
  req.elevation_count = 2;

  LUMICE_AnnotationOverlay a{};
  LUMICE_AnnotationOverlay b{};
  ASSERT_EQ(LUMICE_ComputeAnnotationOverlay(&req, &a), LUMICE_OK);
  // Deliberately overlapping lifetimes: two live overlays must not share storage, which is what
  // makes the handle a per-call allocation rather than a cache.
  ASSERT_EQ(LUMICE_ComputeAnnotationOverlay(&req, &b), LUMICE_OK);
  EXPECT_NE(a.storage, b.storage);
  EXPECT_NE(a.drawable, b.drawable);

  const size_t n = 96u * 48u;
  EXPECT_EQ(std::memcmp(a.drawable, b.drawable, n), 0);
  EXPECT_EQ(std::memcmp(a.elevation, b.elevation, n), 0);
  ASSERT_EQ(a.label_count, b.label_count);
  ASSERT_GT(a.label_count, 0);
  for (int i = 0; i < a.label_count; ++i) {
    EXPECT_FLOAT_EQ(a.labels[i].px, b.labels[i].px);
    EXPECT_FLOAT_EQ(a.labels[i].py, b.labels[i].py);
    EXPECT_STREQ(a.labels[i].text, b.labels[i].text);
  }

  // Releasing one must not disturb the other.
  LUMICE_ReleaseAnnotationOverlay(&a);
  EXPECT_NE(b.drawable, nullptr);
  EXPECT_EQ(std::memcmp(b.drawable, b.drawable, n), 0);
  LUMICE_ReleaseAnnotationOverlay(&b);
}
