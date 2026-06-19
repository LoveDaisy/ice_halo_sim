#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

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

TEST_F(V3TestCrystal, PyramidFaceDistanceNormalDirection) {
  // Pyramidal face normal direction should be independent of face distance.
  // After the fix, polygon face normals for pyramidal faces must be identical
  // between default dist and custom dist (the plane equation normal is determined
  // solely by Miller index / alpha, not by dist).
  float dist_default[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  float dist_custom[6]{ 0.5f, 1.0f, 1.5f, 0.8f, 1.2f, 0.7f };

  auto c_default = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist_default);
  auto c_custom = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist_custom);

  auto pf_cnt_default = c_default.PolygonFaceCount();
  auto pf_cnt_custom = c_custom.PolygonFaceCount();
  ASSERT_EQ(pf_cnt_default, 20u);  // 2 basal + 6 prism + 6 upper pyr + 6 lower pyr
  ASSERT_EQ(pf_cnt_custom, 20u);

  const auto* pn_default = c_default.GetPolygonFaceNormal();
  const auto* pn_custom = c_custom.GetPolygonFaceNormal();

  // Polygon faces are ordered by plane equation index: 2 basal, 6 prism, 6 upper pyr, 6 lower pyr.
  // Pyramidal faces are indices 8..19. Their normals must be identical between the two crystals.
  for (size_t i = 8; i < 20; i++) {
    float dot = Dot3(pn_default + i * 3, pn_custom + i * 3);
    EXPECT_NEAR(dot, 1.0f, 1e-5f) << "Pyramidal polygon face " << i << ": normal direction changed with face distance";
  }
}

TEST_F(V3TestCrystal, PyramidNonDefaultFaceDistanceMeshValid) {
  // Pyramid with asymmetric face distances should produce a valid mesh.
  float dist[6]{ 0.5f, 1.0f, 1.5f, 0.8f, 1.2f, 0.7f };
  auto crystal = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist);

  EXPECT_GT(crystal.TotalTriangles(), 0u);
  EXPECT_GT(crystal.TotalVertices(), 0u);

  // All normals should be unit vectors
  const auto* face_n = crystal.GetTriangleNormal();
  for (size_t i = 0; i < crystal.TotalTriangles(); i++) {
    float len = Norm3(face_n + i * 3);
    EXPECT_NEAR(len, 1.0f, 1e-4f) << "triangle " << i;
  }

  // All areas should be positive
  const auto* face_area = crystal.GetTirangleArea();
  for (size_t i = 0; i < crystal.TotalTriangles(); i++) {
    EXPECT_GT(face_area[i], 0.0f) << "triangle " << i;
  }

  // Vertices should differ from default dist
  float dist_default[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c_default = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist_default);
  bool any_different = false;
  const auto* vtx = crystal.GetTriangleVtx();
  const auto* vtx_default = c_default.GetTriangleVtx();
  for (size_t i = 0; i < crystal.TotalTriangles() * 9 && !any_different; i++) {
    if (std::abs(vtx[i] - vtx_default[i]) > 1e-5f) {
      any_different = true;
    }
  }
  EXPECT_TRUE(any_different) << "Non-default face distance should produce different vertices";
}

TEST_F(V3TestCrystal, PyramidZeroFaceDistanceBoundary) {
  // dist=0 for one face should not crash, producing a valid (possibly degenerate) mesh.
  float dist[6]{ 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto crystal = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist);

  // Should not crash; mesh may be degenerate but must have vertices
  EXPECT_GT(crystal.TotalVertices(), 0u);
}

TEST_F(V3TestCrystal, PyramidFaceDistanceZRange) {
  // Verify that the z range (basal positions) is reasonable for non-default face distance.
  float dist[6]{ 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f };
  auto crystal = Crystal::CreatePyramid(1, 1, 1, 1, 0.5f, 1.0f, 0.5f, dist);

  float dist_default[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c_default = Crystal::CreatePyramid(1, 1, 1, 1, 0.5f, 1.0f, 0.5f, dist_default);

  // Find z range for both crystals
  auto find_z_range = [](const Crystal& c) -> std::pair<float, float> {
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    const auto* vtx = c.GetTriangleVtx();
    for (size_t i = 0; i < c.TotalTriangles() * 3; i++) {
      float z = vtx[i * 3 + 2];
      z_min = std::min(z_min, z);
      z_max = std::max(z_max, z);
    }
    return { z_min, z_max };
  };

  auto [z_min, z_max] = find_z_range(crystal);
  auto [z_min_d, z_max_d] = find_z_range(c_default);

  // Smaller face distance → smaller crystal → smaller z range
  EXPECT_LT(z_max, z_max_d + 1e-5f);
  EXPECT_GT(z_min, z_min_d - 1e-5f);
  // z range should be positive (crystal has height)
  EXPECT_GT(z_max - z_min, 0.1f);
}

// ====== Wedge angle API tests ======

TEST_F(V3TestCrystal, WedgeAngleVsMillerIndexConsistency) {
  // For common Miller indices, the wedge angle overload should produce identical results.
  struct TestCase {
    int i1, i4;
    const char* label;
  };
  TestCase cases[] = {
    { 1, 1, "{1,0,-1,1}" },
    { 2, 1, "{2,0,-2,1}" },
    { 3, 2, "{3,0,-3,2}" },
  };

  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (const auto& tc : cases) {
    float alpha = std::atan(math::kSqrt3_2 * tc.i4 / tc.i1 / kIceCrystalC) * math::kRadToDegree;

    auto c_miller = Crystal::CreatePyramid(tc.i1, tc.i4, tc.i1, tc.i4, 0.3f, 1.0f, 0.3f, dist);
    auto c_angle = Crystal::CreatePyramid(alpha, alpha, 0.3f, 1.0f, 0.3f, dist);

    EXPECT_EQ(c_miller.TotalTriangles(), c_angle.TotalTriangles()) << tc.label;
    EXPECT_EQ(c_miller.TotalVertices(), c_angle.TotalVertices()) << tc.label;

    // Verify vertex-level bit-exact consistency
    const auto* vtx_m = c_miller.GetTriangleVtx();
    const auto* vtx_a = c_angle.GetTriangleVtx();
    for (size_t i = 0; i < c_miller.TotalTriangles() * 9; i++) {
      EXPECT_EQ(vtx_m[i], vtx_a[i]) << tc.label << " vtx[" << i << "]";
    }
  }
}

TEST_F(V3TestCrystal, WedgeAngleConvenienceOverload) {
  // Convenience overload (no dist) should match explicit default dist.
  float alpha = std::atan(math::kSqrt3_2 / kIceCrystalC) * math::kRadToDegree;
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  auto c1 = Crystal::CreatePyramid(alpha, alpha, 0.3f, 1.0f, 0.3f);
  auto c2 = Crystal::CreatePyramid(alpha, alpha, 0.3f, 1.0f, 0.3f, dist);

  EXPECT_EQ(c1.TotalTriangles(), c2.TotalTriangles());
  EXPECT_EQ(c1.TotalVertices(), c2.TotalVertices());
}

TEST_F(V3TestCrystal, WedgeAngleNormal) {
  // alpha=45 degrees: a typical case, crystal should be valid.
  auto crystal = Crystal::CreatePyramid(45.0f, 45.0f, 0.3f, 1.0f, 0.3f);
  EXPECT_GT(crystal.TotalTriangles(), 0u);
  EXPECT_GT(crystal.TotalVertices(), 0u);
  // Full pyramid: 20 polygon faces
  EXPECT_EQ(crystal.PolygonFaceCount(), 20u);
}

TEST_F(V3TestCrystal, WedgeAngleDegenerateSmall) {
  // alpha < 0.1 degree: pyramid segment should be skipped, degenerating to prism.
  auto c_degenerate = Crystal::CreatePyramid(0.05f, 0.05f, 0.3f, 1.0f, 0.3f);
  auto c_prism = Crystal::CreatePrism(1.0f);

  // Should degenerate to prism (8 polygon faces: 2 basal + 6 prism)
  EXPECT_EQ(c_degenerate.TotalTriangles(), c_prism.TotalTriangles());
  EXPECT_EQ(c_degenerate.TotalVertices(), c_prism.TotalVertices());
  EXPECT_EQ(c_degenerate.PolygonFaceCount(), 8u);
}

TEST_F(V3TestCrystal, WedgeAngleDegenerateLarge) {
  // alpha > 89.9 degrees: pyramid segment should be skipped (face degenerates to basal).
  auto c_degenerate = Crystal::CreatePyramid(90.0f, 90.0f, 0.3f, 1.0f, 0.3f);
  auto c_prism = Crystal::CreatePrism(1.0f);

  EXPECT_EQ(c_degenerate.TotalTriangles(), c_prism.TotalTriangles());
  EXPECT_EQ(c_degenerate.TotalVertices(), c_prism.TotalVertices());
  EXPECT_EQ(c_degenerate.PolygonFaceCount(), 8u);
}

TEST_F(V3TestCrystal, WedgeAngleDegenerateNegative) {
  // Negative alpha or alpha >= 180: should also degenerate.
  auto c1 = Crystal::CreatePyramid(-5.0f, -5.0f, 0.3f, 1.0f, 0.3f);
  auto c2 = Crystal::CreatePyramid(180.0f, 180.0f, 0.3f, 1.0f, 0.3f);
  auto c_prism = Crystal::CreatePrism(1.0f);

  EXPECT_EQ(c1.TotalTriangles(), c_prism.TotalTriangles());
  EXPECT_EQ(c2.TotalTriangles(), c_prism.TotalTriangles());
}

TEST_F(V3TestCrystal, MillerIndexI1ZeroDegenerateToPrism) {
  // i1=0 is invalid (would cause integer division by zero). Should degenerate to prism.
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c_degenerate = Crystal::CreatePyramid(0, 1, 0, 1, 0.3f, 1.0f, 0.3f, dist);
  auto c_prism = Crystal::CreatePrism(1.0f);

  EXPECT_EQ(c_degenerate.TotalTriangles(), c_prism.TotalTriangles());
  EXPECT_EQ(c_degenerate.TotalVertices(), c_prism.TotalVertices());
  EXPECT_EQ(c_degenerate.PolygonFaceCount(), 8u);
}

TEST_F(V3TestCrystal, WedgeAngleBoundaryValid) {
  // alpha=1 and alpha=80 are near the boundary, should produce valid 20-face pyramid meshes.
  auto c1 = Crystal::CreatePyramid(1.0f, 1.0f, 0.3f, 1.0f, 0.3f);
  EXPECT_GT(c1.TotalTriangles(), 0u);
  EXPECT_GT(c1.TotalVertices(), 0u);
  EXPECT_EQ(c1.PolygonFaceCount(), 20u);

  auto c2 = Crystal::CreatePyramid(80.0f, 80.0f, 0.3f, 1.0f, 0.3f);
  EXPECT_GT(c2.TotalTriangles(), 0u);
  EXPECT_GT(c2.TotalVertices(), 0u);
  EXPECT_EQ(c2.PolygonFaceCount(), 20u);

  // All triangle areas should be positive (no degenerate triangles)
  const auto* area1 = c1.GetTirangleArea();
  for (size_t i = 0; i < c1.TotalTriangles(); i++) {
    EXPECT_GT(area1[i], 0.0f) << "alpha=1 triangle " << i;
  }
  const auto* area2 = c2.GetTirangleArea();
  for (size_t i = 0; i < c2.TotalTriangles(); i++) {
    EXPECT_GT(area2[i], 0.0f) << "alpha=80 triangle " << i;
  }
}

// ---- detail:: helper function tests ----

TEST(DetailComputeSigmaA, RollMeanMultiples) {
  EXPECT_EQ(lumice::detail::ComputeSigmaA(0.0f), 0);
  EXPECT_EQ(lumice::detail::ComputeSigmaA(30.0f), 5);
  EXPECT_EQ(lumice::detail::ComputeSigmaA(60.0f), 4);
  EXPECT_EQ(lumice::detail::ComputeSigmaA(90.0f), 3);
  EXPECT_EQ(lumice::detail::ComputeSigmaA(120.0f), 2);
  EXPECT_EQ(lumice::detail::ComputeSigmaA(150.0f), 1);
}

TEST(DetailIsRollMeanAtMultipleOf30, Boundary) {
  // Construct AxisDistribution with specific roll mean
  AxisDistribution d{};
  d.roll_dist.type = DistributionType::kNoRandom;

  d.roll_dist.mean = 0.0f;
  EXPECT_TRUE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  d.roll_dist.mean = 30.0f;
  EXPECT_TRUE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  d.roll_dist.mean = 60.0f;
  EXPECT_TRUE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  d.roll_dist.mean = 15.0f;
  EXPECT_FALSE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  // 29.9f is clearly outside the 1e-5 tolerance of any multiple of 30
  d.roll_dist.mean = 29.9f;
  EXPECT_FALSE(lumice::detail::IsRollMeanAtMultipleOf30(d));
}

TEST(DetailIsDApplicable, Conditions) {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.std = 360.0f;
  d.azimuth_dist.mean = 0.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.mean = 0.0f;

  // az=kUniform/360 + roll=0 → true
  EXPECT_TRUE(lumice::detail::IsDApplicable(d));

  // roll=15 → false
  d.roll_dist.mean = 15.0f;
  EXPECT_FALSE(lumice::detail::IsDApplicable(d));

  // az=kGaussian → false
  d.roll_dist.mean = 0.0f;
  d.azimuth_dist.type = DistributionType::kGaussian;
  EXPECT_FALSE(lumice::detail::IsDApplicable(d));
}

// D4 invariant: every triangle's normal must align with at least one polygon-face
// normal (dot > 1-1e-3), ensuring PolygonFaceOfTri always finds a match on valid
// crystals and HitSurface's polygon-face normal equals the per-triangle normal.
TEST_F(V3TestCrystal, EveryTriangleMapsToCoplanarPolygon) {
  auto prism = Crystal::CreatePrism(1.3);
  auto pyramid = Crystal::CreatePyramid(0.3f, 1.0f, 0.3f);

  for (const auto* crystal : { &prism, &pyramid }) {
    const float* tn = crystal->GetTriangleNormal();
    const float* pn = crystal->GetPolygonFaceNormal();
    for (size_t t = 0; t < crystal->TotalTriangles(); t++) {
      bool found = false;
      for (size_t p = 0; p < crystal->PolygonFaceCount(); p++) {
        if (Dot3(tn + t * 3, pn + p * 3) > 1.0f - 1e-3f) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "triangle " << t << " has no coplanar polygon face";
    }
  }
}

// AC-5: GetFn(IdType poly_idx) returns the same fn that GetFn(int tri_id) gives
// for the polygon's representative triangle, and yields kInvalidId on out-of-range
// or kInvalidId input. Covers the new polygon-face overload used by simulator
// after the from_face_/to_face_ split.
TEST(CrystalGetFnByPolygonFace, MatchesTriangleOverloadAndBoundaries) {
  auto crystal = Crystal::CreatePrism(1.0f);
  ASSERT_GT(crystal.PolygonFaceCount(), 0u);

  const int* poly_tri = crystal.GetPolygonFaceTriId();
  for (size_t p = 0; p < crystal.PolygonFaceCount(); p++) {
    IdType fn_poly = crystal.GetFn(static_cast<IdType>(p));
    IdType fn_tri = crystal.GetFn(poly_tri[p]);
    EXPECT_EQ(fn_poly, fn_tri) << "polygon " << p << " fn mismatch vs tri " << poly_tri[p];
    EXPECT_NE(fn_poly, kInvalidId) << "polygon " << p << " should resolve to a known fn";
  }

  // Out-of-range polygon index returns kInvalidId.
  EXPECT_EQ(crystal.GetFn(static_cast<IdType>(crystal.PolygonFaceCount())), kInvalidId);
  // kInvalidId input returns kInvalidId.
  EXPECT_EQ(crystal.GetFn(kInvalidId), kInvalidId);
}

// task-geometry-gen-numerical-robustness Step 3: prism_h=0 pyramid (apex-on-apex)
// must not leak prism/basal Fn entries or zero-area triangles into the mesh.
// Probe data showed Triangulate already skips collapsed prism faces (<3 vertices)
// upstream; this test asserts the post-Step-2 invariant.
TEST_F(V3TestCrystal, PrismHZeroNoLeftoverPrismOrBasal) {
  const float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (float wedge : { 60.0f, 75.0f, 87.0f, 87.5f, 88.0f }) {
    auto crystal = Crystal::CreatePyramid(wedge, wedge, 1.0f, 0.0f, 1.0f, dist);

    // Polygon-face hygiene: 12 = 6 upper pyramid + 6 lower pyramid; no basal, no prism.
    EXPECT_EQ(crystal.PolygonFaceCount(), 12u) << "wedge=" << wedge;
    for (size_t p = 0; p < crystal.PolygonFaceCount(); p++) {
      int fn = static_cast<int>(crystal.GetFn(static_cast<IdType>(p)));
      EXPECT_TRUE((fn >= 13 && fn <= 18) || (fn >= 23 && fn <= 28))
          << "wedge=" << wedge << " poly " << p << " unexpected Fn=" << fn;
    }

    // Triangle-level hygiene: no leftover Fn outside the pyramid set, no zero-area
    // triangles (max area > 0, no triangle below 1e-6 × max_area).
    auto tri_cnt = crystal.TotalTriangles();
    const float* area = crystal.GetTirangleArea();
    float max_area = 0.0f;
    for (size_t t = 0; t < tri_cnt; t++) {
      max_area = std::max(max_area, area[t]);
    }
    EXPECT_GT(max_area, 0.0f) << "wedge=" << wedge;
    for (size_t t = 0; t < tri_cnt; t++) {
      EXPECT_GT(area[t], 1e-6f * max_area) << "wedge=" << wedge << " tri " << t << " near-zero area " << area[t];
      int fn = static_cast<int>(crystal.GetFn(static_cast<int>(t)));
      EXPECT_TRUE((fn >= 13 && fn <= 18) || (fn >= 23 && fn <= 28))
          << "wedge=" << wedge << " tri " << t << " unexpected Fn=" << fn;
    }
  }
}

// task-geometry-gen-numerical-robustness Step 1: white-box wedge sweep diagnostic.
// TEMPORARY — DISABLED_ prefix so it does not run by default; invoke with
// `--gtest_also_run_disabled_tests --gtest_filter=*WedgeSweepDiagnostic*` to capture data.
// Will be replaced/folded into PyramidWedgeSweepNoFalseBasal in Step 6.
TEST(GeometryGenDiagnostic, DISABLED_WedgeSweepDiagnostic) {
  const float wedges[] = { 60.0f, 75.0f, 85.0f, 87.0f, 87.4f, 87.5f, 88.0f, 89.0f, 89.9f };
  // B-ring config geometry: prism_h=0, upper_h=lower_h=1.0, symmetric dist[6]=1
  const float h1 = 1.0f;
  const float h2 = 0.0f;  // prism_h=0 (degenerate prism section)
  const float h3 = 1.0f;
  const float dist[6]{ 1, 1, 1, 1, 1, 1 };

  std::printf("\n========== Step 1 white-box: wedge sweep (prism_h=0, upper/lower_h=1.0) ==========\n");
  for (float wedge : wedges) {
    std::printf("\n----- wedge = %.4f deg -----\n", wedge);

    float coef[kMaxHexCrystalPlanes * 4];
    auto plane_cnt = FillHexCrystalCoef(wedge, wedge, h1, h2, h3, dist, coef);
    std::printf("  FillHexCrystalCoef: plane_cnt = %zu\n", plane_cnt);

    // (a) Per-plane normal length |n| — is the relativization basis non-trivial?
    std::printf("  Plane normal lengths |n|:");
    for (size_t p = 0; p < plane_cnt; p++) {
      float ln = Norm3(coef + p * 4);
      std::printf(" [p%zu]=%.4e", p, ln);
    }
    std::printf("\n");

    // (b) SolvePlanes det for all triples; check raw det vs scale-invariant |det|/(|n1||n2||n3|).
    int triples_total = 0;
    int triples_kept = 0;
    int triples_dropped_singular = 0;
    float min_raw_det_kept = std::numeric_limits<float>::infinity();
    float min_rel_det_kept = std::numeric_limits<float>::infinity();
    float min_raw_det_dropped = std::numeric_limits<float>::infinity();
    float min_rel_det_dropped = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < plane_cnt; i++) {
      for (size_t j = i + 1; j < plane_cnt; j++) {
        for (size_t k = j + 1; k < plane_cnt; k++) {
          const float* c1 = coef + i * 4;
          const float* c2 = coef + j * 4;
          const float* c3 = coef + k * 4;
          float det = c1[0] * c2[1] * c3[2] + c1[1] * c2[2] * c3[0] + c1[2] * c2[0] * c3[1] - c1[2] * c2[1] * c3[0] -
                      c1[0] * c2[2] * c3[1] - c1[1] * c2[0] * c3[2];
          float n1 = Norm3(c1);
          float n2 = Norm3(c2);
          float n3 = Norm3(c3);
          float denom = n1 * n2 * n3;
          float rel = denom > 0 ? std::fabs(det) / denom : 0.0f;
          float xyz[3];
          bool solved = SolvePlanes(c1, c2, c3, xyz);
          triples_total++;
          if (solved) {
            triples_kept++;
            min_raw_det_kept = std::min(min_raw_det_kept, std::fabs(det));
            min_rel_det_kept = std::min(min_rel_det_kept, rel);
          } else {
            triples_dropped_singular++;
            min_raw_det_dropped = std::min(min_raw_det_dropped, std::fabs(det));
            min_rel_det_dropped = std::min(min_rel_det_dropped, rel);
          }
        }
      }
    }
    std::printf("  SolvePlanes triples: total=%d kept=%d singular-dropped=%d\n", triples_total, triples_kept,
                triples_dropped_singular);
    std::printf("    min |det| kept    = %.4e   min rel-det kept    = %.4e\n", min_raw_det_kept, min_rel_det_kept);
    if (triples_dropped_singular > 0) {
      std::printf("    min |det| dropped = %.4e   min rel-det dropped = %.4e\n", min_raw_det_dropped,
                  min_rel_det_dropped);
    }

    // (c) SolveConvexPolyhedronVtx: vertex count.
    auto [vtx, vtx_cnt] = SolveConvexPolyhedronVtx(static_cast<int>(plane_cnt), coef);
    std::printf("  SolveConvexPolyhedronVtx: vtx_cnt = %d\n", vtx_cnt);

    // (d) CollectSurfaceVtx: per-plane |Dot3+d| of all vertices (smallest non-zero is the
    //     closest "should this vtx be on this plane?" decision). Dump min/max for diagnostic.
    auto faces = CollectSurfaceVtx(vtx_cnt, vtx.get(), static_cast<int>(plane_cnt), coef);
    std::printf("  CollectSurfaceVtx: face_groups = %zu\n", faces.size());
    // Per-plane vertex incidence: sum of |Dot3(coef, vtx) + d| under and over the FloatEqualZero
    // (=kFloatEps=1e-5) threshold tells us if any vertex is borderline-incident.
    for (size_t p = 0; p < plane_cnt; p++) {
      float min_abs_val = std::numeric_limits<float>::infinity();
      float min_abs_val_normalized = std::numeric_limits<float>::infinity();
      int borderline_count = 0;  // 0.1*kFloatEps < |val| < 10*kFloatEps
      float norm_len = Norm3(coef + p * 4);
      for (int v = 0; v < vtx_cnt; v++) {
        float val = Dot3(coef + p * 4, vtx.get() + v * 3) + coef[p * 4 + 3];
        float abs_val = std::fabs(val);
        if (abs_val < min_abs_val) {
          min_abs_val = abs_val;
        }
        float abs_val_norm = norm_len > 0 ? abs_val / norm_len : abs_val;
        if (abs_val_norm < min_abs_val_normalized) {
          min_abs_val_normalized = abs_val_norm;
        }
        if (abs_val > 1e-6f && abs_val < 1e-4f) {
          borderline_count++;
        }
      }
      if (borderline_count > 0) {
        std::printf("    plane %zu: |n|=%.4e min|val|=%.4e min|val|/|n|=%.4e borderline(1e-6..1e-4)=%d\n", p, norm_len,
                    min_abs_val, min_abs_val_normalized, borderline_count);
      }
    }

    // (e) Final Crystal: poly_face_cnt and normal z (which one is fake basal n=(0,0,±1)?).
    auto crystal = Crystal::CreatePyramid(wedge, wedge, h1, h2, h3, dist);
    std::printf("  Crystal.PolygonFaceCount = %zu\n", crystal.PolygonFaceCount());
    const float* pn = crystal.GetPolygonFaceNormal();
    int n_basal_like = 0;
    for (size_t p = 0; p < crystal.PolygonFaceCount(); p++) {
      if (std::fabs(pn[p * 3 + 2]) > 0.99f) {
        n_basal_like++;
        std::printf("    poly %zu: n = (%.4f, %.4f, %.4f) fn=%d  <-- basal-like\n", p, pn[p * 3 + 0], pn[p * 3 + 1],
                    pn[p * 3 + 2], static_cast<int>(crystal.GetFn(static_cast<IdType>(p))));
      }
    }
    std::printf("  basal-like polygon faces (|n_z|>0.99): %d\n", n_basal_like);
  }
  std::printf("\n========== end Step 1 diagnostic ==========\n\n");
}

}  // namespace
