#include <gtest/gtest.h>

#include <cstddef>
#include <fstream>

#include "config/config_manager.hpp"
#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "util/logger.hpp"

extern std::string config_file_name;
using namespace lumice;

namespace {

class V3TestCrystal : public ::testing::Test {
 protected:
  void SetUp() override {
    std::ifstream f(config_file_name);
    f >> config_json_;
  }

  nlohmann::json config_json_;
};

TEST_F(V3TestCrystal, CrystalCacheData) {
  auto crystal = Crystal::CreatePrism(1.3);

  auto n = crystal.TotalTriangles();
  ASSERT_EQ(n, 20u);

  // All normals should be unit vectors
  const auto* face_n = crystal.GetTriangleNormal();
  for (size_t i = 0; i < n; i++) {
    float len = Norm3(face_n + i * 3);
    ASSERT_NEAR(len, 1.0f, 1e-5);
  }

  // All areas should be positive
  const auto* face_area = crystal.GetTirangleArea();
  float total_area = 0;
  for (size_t i = 0; i < n; i++) {
    ASSERT_GT(face_area[i], 0.0f);
    total_area += face_area[i];
  }
  // Expected total surface area of prism h=1.3: 2 * basal_area + 6 * side_area
  // Basal area = 6 * (sqrt(3)/4 * 0.5^2) = 6 * sqrt(3)/16 = 3*sqrt(3)/8 ≈ 0.649519
  // Side area = 0.5 * 1.3 = 0.65 each, 6 sides = 3.9
  // Total ≈ 2 * 0.649519 + 3.9 ≈ 5.199
  ASSERT_NEAR(total_area, 5.199038f, 1e-3);

  // fn_map: check basal and prism faces are assigned correctly
  int basal_1_cnt = 0;
  int basal_2_cnt = 0;
  int prism_cnt = 0;
  for (size_t i = 0; i < n; i++) {
    auto fn = crystal.GetFn(static_cast<int>(i));
    if (fn == 1) {
      basal_1_cnt++;
    } else if (fn == 2) {
      basal_2_cnt++;
    } else if (fn >= 3 && fn <= 8) {
      prism_cnt++;
    } else {
      FAIL() << "Unexpected fn=" << fn << " for triangle " << i;
    }
  }
  ASSERT_GT(basal_1_cnt, 0);
  ASSERT_GT(basal_2_cnt, 0);
  ASSERT_EQ(basal_1_cnt + basal_2_cnt + prism_cnt, static_cast<int>(n));
}

TEST_F(V3TestCrystal, PolygonFaceDataPrism) {
  auto crystal = Crystal::CreatePrism(1.3);

  // Prism has 8 planes: 2 basal + 6 prism
  ASSERT_EQ(crystal.PolygonFaceCount(), 8u);

  const auto* pn = crystal.GetPolygonFaceNormal();
  const auto* pd = crystal.GetPolygonFaceDist();
  const auto* tri_id = crystal.GetPolygonFaceTriId();
  ASSERT_NE(pn, nullptr);
  ASSERT_NE(pd, nullptr);
  ASSERT_NE(tri_id, nullptr);

  auto tri_cnt = crystal.TotalTriangles();
  const auto* face_n = crystal.GetTriangleNormal();

  for (size_t i = 0; i < crystal.PolygonFaceCount(); i++) {
    // Normals should be unit vectors
    float len = Norm3(pn + i * 3);
    EXPECT_NEAR(len, 1.0f, 1e-5) << "polygon face " << i;

    // tri_id should be in range
    EXPECT_GE(tri_id[i], 0);
    EXPECT_LT(tri_id[i], static_cast<int>(tri_cnt));

    // Polygon face normal should match its representative triangle normal
    float dot = Dot3(pn + i * 3, face_n + tri_id[i] * 3);
    EXPECT_GT(dot, 0.999f) << "polygon face " << i << " tri_id " << tri_id[i];
  }
}

TEST_F(V3TestCrystal, PolygonFaceDataPyramid) {
  auto crystal = Crystal::CreatePyramid(0.3f, 1.0f, 0.3f);

  // Full pyramid has 20 planes: 2 basal + 6 prism + 6 upper + 6 lower
  ASSERT_EQ(crystal.PolygonFaceCount(), 20u);

  const auto* pn = crystal.GetPolygonFaceNormal();
  const auto* pd = crystal.GetPolygonFaceDist();
  const auto* tri_id = crystal.GetPolygonFaceTriId();

  auto tri_cnt = crystal.TotalTriangles();
  const auto* face_n = crystal.GetTriangleNormal();

  for (size_t i = 0; i < crystal.PolygonFaceCount(); i++) {
    float len = Norm3(pn + i * 3);
    EXPECT_NEAR(len, 1.0f, 1e-5) << "polygon face " << i;

    EXPECT_GE(tri_id[i], 0);
    EXPECT_LT(tri_id[i], static_cast<int>(tri_cnt));

    float dot = Dot3(pn + i * 3, face_n + tri_id[i] * 3);
    EXPECT_GT(dot, 0.999f) << "polygon face " << i << " tri_id " << tri_id[i];
  }
}

TEST_F(V3TestCrystal, PrismMesh) {
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c1 = CreatePrismMesh(1.5f);
  auto c2 = CreatePrismMesh(1.5f, dist);

  ASSERT_EQ(c1.GetTriangleCnt(), c2.GetTriangleCnt());
  ASSERT_EQ(c1.GetVtxCnt(), c2.GetVtxCnt());

  // Check all vertices are identical.
  const auto* vtx1 = c1.GetVtxPtr(0);
  const auto* vtx2 = c2.GetVtxPtr(0);
  for (size_t i = 0; i < c1.GetVtxCnt(); i++) {
    bool match = false;
    for (size_t j = 0; j < c2.GetVtxCnt(); j++) {
      if (FloatEqualZero(DiffNorm3(vtx1 + i * 3, vtx2 + j * 3))) {
        match = true;
        break;
      }
    }
    ASSERT_TRUE(match);
  }
  for (size_t i = 0; i < c2.GetVtxCnt(); i++) {
    bool match = false;
    for (size_t j = 0; j < c1.GetVtxCnt(); j++) {
      if (FloatEqualZero(DiffNorm3(vtx2 + i * 3, vtx1 + j * 3))) {
        match = true;
        break;
      }
    }
    ASSERT_TRUE(match);
  }
}

TEST_F(V3TestCrystal, PyramidMesh) {
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c1 = CreatePyramidMesh(0.2f, 1.4f, 0.8f);
  auto c2 = CreatePyramidMesh(1, 1, 1, 1, 0.2f, 1.4f, 0.8f, dist);

  ASSERT_EQ(c1.GetTriangleCnt(), c2.GetTriangleCnt());
  ASSERT_EQ(c1.GetVtxCnt(), c2.GetVtxCnt());

  // Check all vertices are identical.
  const auto* vtx1 = c1.GetVtxPtr(0);
  const auto* vtx2 = c2.GetVtxPtr(0);
  for (size_t i = 0; i < c1.GetVtxCnt(); i++) {
    bool match = false;
    for (size_t j = 0; j < c2.GetVtxCnt(); j++) {
      if (FloatEqualZero(DiffNorm3(vtx1 + i * 3, vtx2 + j * 3))) {
        match = true;
        break;
      }
    }
    ASSERT_TRUE(match);
  }
  for (size_t i = 0; i < c2.GetVtxCnt(); i++) {
    bool match = false;
    for (size_t j = 0; j < c1.GetVtxCnt(); j++) {
      if (FloatEqualZero(DiffNorm3(vtx2 + i * 3, vtx1 + j * 3))) {
        match = true;
        break;
      }
    }
    ASSERT_TRUE(match);
  }
}

}  // namespace