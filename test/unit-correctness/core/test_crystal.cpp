#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <utility>

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

const std::set<int> kExpectedUpper = { 13, 14, 15, 16, 17, 18 };
const std::set<int> kExpectedLower = { 23, 24, 25, 26, 27, 28 };

// Returns the set of fn values in [lo..hi] across all polygon faces of c.
std::set<int> CollectFnSet(const Crystal& c, int lo, int hi) {
  std::set<int> fns;
  for (size_t p = 0; p < c.PolygonFaceCount(); p++) {
    int fn = static_cast<int>(c.GetFn(static_cast<IdType>(p)));
    if (fn >= lo && fn <= hi) {
      fns.insert(fn);
    }
  }
  return fns;
}

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
// normal (dot > 1-1e-3) on standard crystals. This is the geometric premise that
// lets PolygonFaceOfTri's argmax (kFaceCoplanarFloor=1e-2 slack) always find a
// match, and that HitSurface's polygon-face normal equals the per-triangle normal.
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

// task-geometry-gen-numerical-robustness Step 6a: wedge sweep topology sentinel.
// Replaces the Step 1 DISABLED_WedgeSweepDiagnostic (white-box probe), promoting
// its discovery to a regression guard. For prism_h=0 upper_h=lower_h=1.0
// bipyramid across the full wedge range:
//   - PolygonFaceCount stays at 12 (6 upper + 6 lower pyramid; no fake basal)
//   - No polygon face has |n_z| > 0.99 except real pyramid faces, which would
//     show up as fn 13-18 / 23-28; an extra basal-like face with fn 1 or 2
//     is the B-ring bug signature.
// The wedge list MUST be float (not int braced-init), or 87.5 / 89.9 narrowing
// fails to compile and silent removal would drop coverage of the 87.4 -> 87.5
// critical point that Step 1 located.
TEST_F(V3TestCrystal, PyramidWedgeSweepNoFalseBasal) {
  const float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (float wedge : { 60.0f, 75.0f, 80.0f, 85.0f, 87.0f, 87.5f, 88.0f, 89.0f, 89.9f }) {
    auto c = Crystal::CreatePyramid(wedge, wedge, 1.0f, 0.0f, 1.0f, dist);
    EXPECT_EQ(c.PolygonFaceCount(), 12u) << "wedge=" << wedge;
    const float* pn = c.GetPolygonFaceNormal();
    for (size_t p = 0; p < c.PolygonFaceCount(); p++) {
      int fn = static_cast<int>(c.GetFn(static_cast<IdType>(p)));
      EXPECT_FALSE(fn == 1 || fn == 2) << "wedge=" << wedge << " poly " << p << " is a fake basal (fn=" << fn << ")";
      // pyramid faces at extreme wedge have |n_z| close to 1 (sin(wedge)),
      // so we can't filter by n_z alone; the Fn check above is the precise sentinel.
      (void)pn;  // silence unused; kept for future per-normal diagnostics.
    }
  }
}

// task-geometry-gen-numerical-robustness Step 4: at extreme wedge (>= 88.5 deg)
// the float32 SolveConvexPolyhedronVtx collapsed apex/anti-apex into the basal
// ring, dropping TotalVertices from 8 to 6 and corrupting downstream face groups.
// The double-precision pipeline restores the 8-vertex topology end-to-end.
TEST_F(V3TestCrystal, ExtremeWedgeVertexNoCollapse) {
  const float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (float wedge : { 88.0f, 88.5f, 89.0f, 89.5f, 89.9f }) {
    auto c = Crystal::CreatePyramid(wedge, wedge, 1.0f, 0.0f, 1.0f, dist);
    EXPECT_EQ(c.TotalVertices(), 8u) << "wedge=" << wedge;
    EXPECT_EQ(c.PolygonFaceCount(), 12u) << "wedge=" << wedge;
  }
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

// Step 1 white-box diagnostic (`GeometryGenDiagnostic.DISABLED_WedgeSweepDiagnostic`)
// was removed in Step 6: its single-point discovery (BuildPolygonFaceData grouping
// flips at wedge = 87.44 deg) is now permanently guarded by
// `PyramidWedgeSweepNoFalseBasal` above. The raw sweep data lives in
// scratchpad/task-geometry-gen-numerical-robustness/progress.md (2026-06-19 entry).

// task-280.2 fillhexfnmap-extreme-sentinel — Sentinel A:
// At extreme wedge (89.0/89.5 deg) full pyramid (h1=0.3, h2=1.0, h3=0.3), the
// 20 polygon faces must yield Fn sets exactly {13..18} (upper) and {23..28}
// (lower). Detection power: with first-match instead of argmax in
// FillHexFnMap's pri loop, a face whose normal aligns with ref_j=4 (60 deg)
// would still satisfy Dot3(n, ref_3) = cos(89 deg) * cos(60 deg) ~ 0.0087 >
// kFloatEps and be claimed by j=3, collapsing fn=14 into fn=13 -- the set
// equality below would fail. The explicit size()==6 assertion makes the
// 6-fold completeness intent obvious in failure output.
TEST_F(V3TestCrystal, FillHexFnMapExtremeWedgeFullPyramidSixFold) {
  for (float wedge : { 89.0f, 89.5f }) {
    auto c = Crystal::CreatePyramid(wedge, wedge, 0.3f, 1.0f, 0.3f);
    ASSERT_EQ(c.PolygonFaceCount(), 20u) << "wedge=" << wedge;
    auto upper = CollectFnSet(c, 13, 18);
    auto lower = CollectFnSet(c, 23, 28);
    EXPECT_EQ(upper.size(), 6u) << "upper fn count wedge=" << wedge;
    EXPECT_EQ(lower.size(), 6u) << "lower fn count wedge=" << wedge;
    EXPECT_EQ(upper, kExpectedUpper) << "upper fn set wedge=" << wedge;
    EXPECT_EQ(lower, kExpectedLower) << "lower fn set wedge=" << wedge;
  }
}

// task-280.2 Sentinel B: extreme wedge bipyramid (prism_h=0) at 89.0/89.5 deg
// fills the 1-deg gap left by PrismHZeroNoLeftoverPrismOrBasal (sweeps 60..88
// deg) and PyramidWedgeSweepNoFalseBasal (sweeps to 89.9 deg but only checks
// fn != 1/2). This asserts full 6-fold completeness of the Fn allocation.
TEST_F(V3TestCrystal, FillHexFnMapExtremeWedgeBipyramidSixFold) {
  const float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  for (float wedge : { 89.0f, 89.5f }) {
    auto c = Crystal::CreatePyramid(wedge, wedge, 1.0f, 0.0f, 1.0f, dist);
    ASSERT_EQ(c.PolygonFaceCount(), 12u) << "wedge=" << wedge;
    auto upper = CollectFnSet(c, 13, 18);
    auto lower = CollectFnSet(c, 23, 28);
    EXPECT_EQ(upper.size(), 6u) << "upper fn count bipyramid wedge=" << wedge;
    EXPECT_EQ(lower.size(), 6u) << "lower fn count bipyramid wedge=" << wedge;
    EXPECT_EQ(upper, kExpectedUpper) << "upper fn set bipyramid wedge=" << wedge;
    EXPECT_EQ(lower, kExpectedLower) << "lower fn set bipyramid wedge=" << wedge;
  }
}

// task-280.2 Sentinel C: multi-Miller-axis pyramid coverage. Fn allocation
// must not depend on the specific alpha implied by Miller (i1, i4). Covers
// four axes at temperate wedge angles where geometry generation is robust.
TEST_F(V3TestCrystal, FillHexFnMapMillerAxisPyramidSixFold) {
  const float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  struct MillerCase {
    int i1;
    int i4;
    const char* label;
  };
  const MillerCase cases[] = {
    { 1, 1, "(1,1)" },
    { 1, 2, "(1,2)" },
    { 2, 1, "(2,1)" },
    { 3, 2, "(3,2)" },
  };
  for (const auto& mc : cases) {
    auto c = Crystal::CreatePyramid(mc.i1, mc.i4, mc.i1, mc.i4, 0.3f, 1.0f, 0.3f, dist);
    ASSERT_EQ(c.PolygonFaceCount(), 20u) << "miller=" << mc.label;
    auto upper = CollectFnSet(c, 13, 18);
    auto lower = CollectFnSet(c, 23, 28);
    EXPECT_EQ(upper.size(), 6u) << "upper fn count miller=" << mc.label;
    EXPECT_EQ(lower.size(), 6u) << "lower fn count miller=" << mc.label;
    EXPECT_EQ(upper, kExpectedUpper) << "upper fn set miller=" << mc.label;
    EXPECT_EQ(lower, kExpectedLower) << "lower fn set miller=" << mc.label;
  }
}

}  // namespace
