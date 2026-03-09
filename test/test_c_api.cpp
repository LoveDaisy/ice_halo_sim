#include <gtest/gtest.h>

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
