#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <random>
#include <set>
#include <utility>

#include "config/config_manager.hpp"
#include "config/crystal_config.hpp"
#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/geo3d_closedform.hpp"
#include "core/math.hpp"
#include "core/simulator.hpp"
#include "core/trace_ops.hpp"
#include "util/logger.hpp"

extern std::string config_file_name;
using namespace lumice;

namespace {

// The Crystal no longer caches a triangle mesh — entry-point sampling reads
// cf_geom_ corners directly, and the only triangulating consumer (C-API export)
// rebuilds a Mesh on demand. These test-only helpers reconstruct the same fan
// triangulation from cf_geom_ so the geometric invariants below can be asserted
// exactly as before: detail::BuildEntrySubTris yields per-triangle
// vtx/normal/area/face_id (the old GetTriangleVtx/Normal/Area/PolygonFaceOfTri,
// same fan order), and detail::BuildMeshFromCfGeom yields the deduped V/F counts
// (the old GetMesh()).
std::vector<detail::EntrySubTri> SubTris(const Crystal& c) {
  const CrystalGeom& cf = c.CfGeom();
  std::vector<detail::EntrySubTri> sub(detail::CountEntrySubTris(cf));
  if (!sub.empty()) {
    detail::BuildEntrySubTris(cf, sub.data());
  }
  return sub;
}
size_t TriCnt(const Crystal& c) {
  return detail::CountEntrySubTris(c.CfGeom());
}
size_t DedupVtxCnt(const Crystal& c) {
  return detail::BuildMeshFromCfGeom(c.CfGeom()).mesh.GetVtxCnt();
}

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

  const auto sub = SubTris(crystal);
  const size_t n = sub.size();
  ASSERT_EQ(n, 20u);

  // All normals should be unit vectors
  for (size_t i = 0; i < n; i++) {
    float len = Norm3(sub[i].n);
    ASSERT_NEAR(len, 1.0f, 1e-5);
  }

  // All areas should be positive
  float total_area = 0;
  for (size_t i = 0; i < n; i++) {
    ASSERT_GT(sub[i].area, 0.0f);
    total_area += sub[i].area;
  }
  // Expected total surface area of prism h=1.3: 2 * basal_area + 6 * side_area
  // Basal area = 6 * (sqrt(3)/4 * 0.5^2) = 6 * sqrt(3)/16 = 3*sqrt(3)/8 ≈ 0.649519
  // Side area = 0.5 * 1.3 = 0.65 each, 6 sides = 3.9
  // Total ≈ 2 * 0.649519 + 3.9 ≈ 5.199
  ASSERT_NEAR(total_area, 5.199038f, 1e-3);

  // fn_map: check basal and prism faces are assigned correctly.
  // Chain tri → polygon-face → fn (both hops via the parametric slot table
  // populated by PopulateFromCfGeom from cf_geom_.face_number).
  int basal_1_cnt = 0;
  int basal_2_cnt = 0;
  int prism_cnt = 0;
  for (size_t i = 0; i < n; i++) {
    IdType poly = sub[i].face_id;
    auto fn = crystal.GetFn(poly);
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
  ASSERT_NE(pn, nullptr);
  ASSERT_NE(pd, nullptr);

  const auto sub = SubTris(crystal);

  for (size_t i = 0; i < crystal.PolygonFaceCount(); i++) {
    // Normals should be unit vectors
    float len = Norm3(pn + i * 3);
    EXPECT_NEAR(len, 1.0f, 1e-5) << "polygon face " << i;

    // Every polygon face must contribute at least one triangle whose normal
    // aligns with the polygon plane normal. Uses the fan sub-triangle's compact
    // present-face id (from cf_geom_, same numbering PopulateFromCfGeom uses) as
    // the inverse map — replaces the pre-7C GetPolygonFaceTriId() which exposed
    // a single argmax-selected representative triangle.
    bool found = false;
    for (size_t t = 0; t < sub.size(); t++) {
      if (sub[t].face_id == static_cast<IdType>(i)) {
        float dot = Dot3(pn + i * 3, sub[t].n);
        EXPECT_GT(dot, 0.999f) << "polygon face " << i << " tri " << t;
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "polygon face " << i << " has no member triangle";
  }
}

TEST_F(V3TestCrystal, PolygonFaceDataPyramid) {
  auto crystal = Crystal::CreatePyramid(0.3f, 1.0f, 0.3f);

  // Full pyramid has 20 planes: 2 basal + 6 prism + 6 upper + 6 lower
  ASSERT_EQ(crystal.PolygonFaceCount(), 20u);

  const auto* pn = crystal.GetPolygonFaceNormal();
  const auto* pd = crystal.GetPolygonFaceDist();
  ASSERT_NE(pn, nullptr);
  ASSERT_NE(pd, nullptr);

  const auto sub = SubTris(crystal);

  for (size_t i = 0; i < crystal.PolygonFaceCount(); i++) {
    float len = Norm3(pn + i * 3);
    EXPECT_NEAR(len, 1.0f, 1e-5) << "polygon face " << i;

    // Same "at least one member triangle aligns with the polygon normal"
    // invariant as PolygonFaceDataPrism (see the rationale there).
    bool found = false;
    for (size_t t = 0; t < sub.size(); t++) {
      if (sub[t].face_id == static_cast<IdType>(i)) {
        float dot = Dot3(pn + i * 3, sub[t].n);
        EXPECT_GT(dot, 0.999f) << "polygon face " << i << " tri " << t;
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "polygon face " << i << " has no member triangle";
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

  const auto sub = SubTris(crystal);
  EXPECT_GT(sub.size(), 0u);
  EXPECT_GT(DedupVtxCnt(crystal), 0u);

  // All normals should be unit vectors
  for (size_t i = 0; i < sub.size(); i++) {
    float len = Norm3(sub[i].n);
    EXPECT_NEAR(len, 1.0f, 1e-4f) << "triangle " << i;
  }

  // All areas should be positive
  for (size_t i = 0; i < sub.size(); i++) {
    EXPECT_GT(sub[i].area, 0.0f) << "triangle " << i;
  }

  // Vertices should differ from default dist
  float dist_default[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c_default = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist_default);
  const auto sub_default = SubTris(c_default);
  ASSERT_EQ(sub.size(), sub_default.size());
  bool any_different = false;
  for (size_t t = 0; t < sub.size() && !any_different; t++) {
    for (int j = 0; j < 9; j++) {
      if (std::abs(sub[t].v[j] - sub_default[t].v[j]) > 1e-5f) {
        any_different = true;
        break;
      }
    }
  }
  EXPECT_TRUE(any_different) << "Non-default face distance should produce different vertices";
}

TEST_F(V3TestCrystal, PyramidZeroFaceDistanceBoundary) {
  // dist=0 for one face should not crash, producing a valid (possibly degenerate) mesh.
  float dist[6]{ 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto crystal = Crystal::CreatePyramid(1, 1, 1, 1, 0.3f, 1.0f, 0.3f, dist);

  // Should not crash; mesh may be degenerate but must have vertices
  EXPECT_GT(DedupVtxCnt(crystal), 0u);
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
    const auto sub = SubTris(c);
    for (const auto& s : sub) {
      for (int k = 0; k < 3; k++) {
        float z = s.v[k * 3 + 2];
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
      }
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

    const auto sub_m = SubTris(c_miller);
    const auto sub_a = SubTris(c_angle);
    EXPECT_EQ(sub_m.size(), sub_a.size()) << tc.label;
    EXPECT_EQ(DedupVtxCnt(c_miller), DedupVtxCnt(c_angle)) << tc.label;

    // Verify vertex-level bit-exact consistency
    ASSERT_EQ(sub_m.size(), sub_a.size());
    for (size_t t = 0; t < sub_m.size(); t++) {
      for (int j = 0; j < 9; j++) {
        EXPECT_EQ(sub_m[t].v[j], sub_a[t].v[j]) << tc.label << " tri " << t << " v[" << j << "]";
      }
    }
  }
}

TEST_F(V3TestCrystal, WedgeAngleConvenienceOverload) {
  // Convenience overload (no dist) should match explicit default dist.
  float alpha = std::atan(math::kSqrt3_2 / kIceCrystalC) * math::kRadToDegree;
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  auto c1 = Crystal::CreatePyramid(alpha, alpha, 0.3f, 1.0f, 0.3f);
  auto c2 = Crystal::CreatePyramid(alpha, alpha, 0.3f, 1.0f, 0.3f, dist);

  EXPECT_EQ(TriCnt(c1), TriCnt(c2));
  EXPECT_EQ(DedupVtxCnt(c1), DedupVtxCnt(c2));
}

TEST_F(V3TestCrystal, WedgeAngleNormal) {
  // alpha=45 degrees: a typical case, crystal should be valid.
  auto crystal = Crystal::CreatePyramid(45.0f, 45.0f, 0.3f, 1.0f, 0.3f);
  EXPECT_GT(TriCnt(crystal), 0u);
  EXPECT_GT(DedupVtxCnt(crystal), 0u);
  // Full pyramid: 20 polygon faces
  EXPECT_EQ(crystal.PolygonFaceCount(), 20u);
}

TEST_F(V3TestCrystal, WedgeAngleDegenerateSmall) {
  // alpha < 0.1 degree: pyramid segment should be skipped, degenerating to prism.
  auto c_degenerate = Crystal::CreatePyramid(0.05f, 0.05f, 0.3f, 1.0f, 0.3f);
  auto c_prism = Crystal::CreatePrism(1.0f);

  // Should degenerate to prism (8 polygon faces: 2 basal + 6 prism)
  EXPECT_EQ(TriCnt(c_degenerate), TriCnt(c_prism));
  EXPECT_EQ(DedupVtxCnt(c_degenerate), DedupVtxCnt(c_prism));
  EXPECT_EQ(c_degenerate.PolygonFaceCount(), 8u);
}

TEST_F(V3TestCrystal, WedgeAngleDegenerateLarge) {
  // alpha > 89.9 degrees: pyramid segment should be skipped (face degenerates to basal).
  auto c_degenerate = Crystal::CreatePyramid(90.0f, 90.0f, 0.3f, 1.0f, 0.3f);
  auto c_prism = Crystal::CreatePrism(1.0f);

  EXPECT_EQ(TriCnt(c_degenerate), TriCnt(c_prism));
  EXPECT_EQ(DedupVtxCnt(c_degenerate), DedupVtxCnt(c_prism));
  EXPECT_EQ(c_degenerate.PolygonFaceCount(), 8u);
}

TEST_F(V3TestCrystal, WedgeAngleDegenerateNegative) {
  // Negative alpha or alpha >= 180: should also degenerate.
  auto c1 = Crystal::CreatePyramid(-5.0f, -5.0f, 0.3f, 1.0f, 0.3f);
  auto c2 = Crystal::CreatePyramid(180.0f, 180.0f, 0.3f, 1.0f, 0.3f);
  auto c_prism = Crystal::CreatePrism(1.0f);

  EXPECT_EQ(TriCnt(c1), TriCnt(c_prism));
  EXPECT_EQ(TriCnt(c2), TriCnt(c_prism));
}

TEST_F(V3TestCrystal, MillerIndexI1ZeroDegenerateToPrism) {
  // i1=0 is invalid (would cause integer division by zero). Should degenerate to prism.
  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  auto c_degenerate = Crystal::CreatePyramid(0, 1, 0, 1, 0.3f, 1.0f, 0.3f, dist);
  auto c_prism = Crystal::CreatePrism(1.0f);

  EXPECT_EQ(TriCnt(c_degenerate), TriCnt(c_prism));
  EXPECT_EQ(DedupVtxCnt(c_degenerate), DedupVtxCnt(c_prism));
  EXPECT_EQ(c_degenerate.PolygonFaceCount(), 8u);
}

TEST_F(V3TestCrystal, WedgeAngleBoundaryValid) {
  // alpha=1 and alpha=80 are near the boundary, should produce valid 20-face pyramid meshes.
  auto c1 = Crystal::CreatePyramid(1.0f, 1.0f, 0.3f, 1.0f, 0.3f);
  EXPECT_GT(TriCnt(c1), 0u);
  EXPECT_GT(DedupVtxCnt(c1), 0u);
  EXPECT_EQ(c1.PolygonFaceCount(), 20u);

  auto c2 = Crystal::CreatePyramid(80.0f, 80.0f, 0.3f, 1.0f, 0.3f);
  EXPECT_GT(TriCnt(c2), 0u);
  EXPECT_GT(DedupVtxCnt(c2), 0u);
  EXPECT_EQ(c2.PolygonFaceCount(), 20u);

  // All triangle areas should be positive (no degenerate triangles)
  const auto sub1 = SubTris(c1);
  for (size_t i = 0; i < sub1.size(); i++) {
    EXPECT_GT(sub1[i].area, 0.0f) << "alpha=1 triangle " << i;
  }
  const auto sub2 = SubTris(c2);
  for (size_t i = 0; i < sub2.size(); i++) {
    EXPECT_GT(sub2[i].area, 0.0f) << "alpha=80 triangle " << i;
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

  d.roll_dist.center = 0.0f;
  EXPECT_TRUE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  d.roll_dist.center = 30.0f;
  EXPECT_TRUE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  d.roll_dist.center = 60.0f;
  EXPECT_TRUE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  d.roll_dist.center = 15.0f;
  EXPECT_FALSE(lumice::detail::IsRollMeanAtMultipleOf30(d));

  // 29.9f is clearly outside the 1e-5 tolerance of any multiple of 30
  d.roll_dist.center = 29.9f;
  EXPECT_FALSE(lumice::detail::IsRollMeanAtMultipleOf30(d));
}

TEST(DetailIsDApplicable, Conditions) {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.spread = 360.0f;
  d.azimuth_dist.center = 0.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.center = 0.0f;

  // az=kUniform/360 + roll=0 → true
  EXPECT_TRUE(lumice::detail::IsDApplicable(d));

  // roll=15 → false
  d.roll_dist.center = 15.0f;
  EXPECT_FALSE(lumice::detail::IsDApplicable(d));

  // az=kGaussian → false
  d.roll_dist.center = 0.0f;
  d.azimuth_dist.type = DistributionType::kGaussian;
  EXPECT_FALSE(lumice::detail::IsDApplicable(d));
}

// D4 invariant: every triangle's normal must align with at least one polygon-face
// normal (dot > 1-1e-3) on standard crystals. This is the geometric premise that
// each fan sub-triangle is coplanar with its owning polygon face, and that
// HitSurface's polygon-face normal equals the per-triangle normal.
TEST_F(V3TestCrystal, EveryTriangleMapsToCoplanarPolygon) {
  auto prism = Crystal::CreatePrism(1.3);
  auto pyramid = Crystal::CreatePyramid(0.3f, 1.0f, 0.3f);

  for (const auto* crystal : { &prism, &pyramid }) {
    const auto sub = SubTris(*crystal);
    const float* pn = crystal->GetPolygonFaceNormal();
    for (size_t t = 0; t < sub.size(); t++) {
      bool found = false;
      for (size_t p = 0; p < crystal->PolygonFaceCount(); p++) {
        if (Dot3(sub[t].n, pn + p * 3) > 1.0f - 1e-3f) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "triangle " << t << " has no coplanar polygon face";
    }
  }
}

// AC-5: GetFn(IdType poly_idx) yields legal per-polygon face numbers and
// kInvalidId on out-of-range or kInvalidId input. Since Step 7A the tri
// overload (GetFn(int)) is deleted — the parametric slot table populated by
// PopulateFromCfGeom is the single source of truth, so "two overloads must
// agree" is no longer testable (there is only one overload). The invariants
// worth guarding here are: every present polygon face resolves to a legal fn,
// and out-of-range / kInvalidId poly_idx return kInvalidId.
TEST(CrystalGetFnByPolygonFace, LegalPerPolyFnAndBoundaries) {
  auto crystal = Crystal::CreatePrism(1.0f);
  ASSERT_GT(crystal.PolygonFaceCount(), 0u);

  for (size_t p = 0; p < crystal.PolygonFaceCount(); p++) {
    IdType fn_poly = crystal.GetFn(static_cast<IdType>(p));
    EXPECT_NE(fn_poly, kInvalidId) << "polygon " << p << " should resolve to a known fn";
    EXPECT_TRUE(IsLegalFace(CrystalKind::kPrism, static_cast<int>(fn_poly)))
        << "polygon " << p << " fn=" << fn_poly << " not legal for prism";
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
      // Primary sentinel is PolygonFaceCount()==12u above: the task-276 fake
      // flat-top basal shows up as a geometry/face-count defect. The actual
      // fake basal was numbered fn=13/23 by FillHexFnMap (it collides with the
      // upper/lower-cone +x faces) — and 13/23 are *legal* pyramid cone faces,
      // so fn alone cannot distinguish it (no EXPECT on fn 13/23 is possible).
      // The fn==1||fn==2 check below is only a cheap secondary guard against a
      // *different*, basal-numbered degeneration (a real pyramid cap never has
      // fn 1/2). |n_z| ≈ sin(wedge) ≈ 1 at extreme wedge, so n_z can't filter
      // either; the geometric sentinel is the face count, not the normal.
      EXPECT_FALSE(fn == 1 || fn == 2) << "wedge=" << wedge << " poly " << p << " is a fake basal (fn=" << fn << ")";
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
    EXPECT_EQ(DedupVtxCnt(c), 8u) << "wedge=" << wedge;
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
    const auto sub = SubTris(crystal);
    float max_area = 0.0f;
    for (const auto& s : sub) {
      max_area = std::max(max_area, s.area);
    }
    EXPECT_GT(max_area, 0.0f) << "wedge=" << wedge;
    for (size_t t = 0; t < sub.size(); t++) {
      EXPECT_GT(sub[t].area, 1e-6f * max_area)
          << "wedge=" << wedge << " tri " << t << " near-zero area " << sub[t].area;
      IdType poly = sub[t].face_id;
      int fn = static_cast<int>(crystal.GetFn(poly));
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

// task-280.6 zero-volume degenerate-crystal guard: when prism_h=0 collides with
// wedge > kMaxPyramidAlpha (89.9 deg), both pyramidal caps are dropped, leaving
// upper/lower basal faces at z=0 — a zero-thickness hexagon. Without the guard
// in FillHexCrystalCoef, Triangulate's Vec3FromTo(body_center, face_center)
// collapses to (0,0,0) and Normalize3 returns NaN (explore-280.3 reproduce).
// Detection power: removing the guard makes the crystal non-empty while
// face normals contain NaN — both branches of the EXPECT below would flip.
TEST_F(V3TestCrystal, FillHexCrystalCoefZeroVolumeGuardNoNan) {
  const float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

  // Issue scenario: wedge=89.95° (> 89.9° kMaxPyramidAlpha) + prism_h=0 → guard
  // trips → empty crystal (zero triangles), no NaN normals.
  auto c = Crystal::CreatePyramid(89.95f, 89.95f, 1.0f, 0.0f, 1.0f, dist);
  EXPECT_EQ(TriCnt(c), 0u) << "zero-volume guard should yield empty crystal, not NaN-poisoned mesh";

  // Regression: normal pyramid (prism_h>0, wedge=60°) must be unaffected.
  auto c_normal = Crystal::CreatePyramid(60.0f, 60.0f, 1.0f, 1.0f, 1.0f, dist);
  const auto sub_normal = SubTris(c_normal);
  ASSERT_GT(sub_normal.size(), 0u);
  for (size_t t = 0; t < sub_normal.size(); t++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_FALSE(std::isnan(sub_normal[t].n[j])) << "normal pyramid triangle " << t << " has NaN normal";
    }
  }

  // Edge case: wedge=89.9° (exactly at kMaxPyramidAlpha, pyramidal caps NOT
  // dropped per "alpha <= 89.9" condition) with prism_h=0. Per explore-280.3
  // wedge ≤ 89.9° is safe at any prism_h, so geometry should still build.
  // Accept either outcome (guard fires or not), but if any triangles exist,
  // their normals must be finite — that is the real invariant we care about.
  auto c_boundary = Crystal::CreatePyramid(89.9f, 89.9f, 1.0f, 0.0f, 1.0f, dist);
  const auto sub_boundary = SubTris(c_boundary);
  for (size_t t = 0; t < sub_boundary.size(); t++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_FALSE(std::isnan(sub_boundary[t].n[j])) << "boundary wedge=89.9° triangle " << t << " has NaN normal";
    }
  }
}

// ==========================================================================
// Pyramid empty-feasible-region fuzz.
//
// FillHexCrystalCoef's pyramid branch solves basal `d` by intersecting three
// non-basal planes at a time and clamping z_max/z_min to whichever candidate
// vertices lie inside the polyhedron. When random face_distance perturbations
// make the half-space intersection empty, the triple-plane loop never enters
// its IsInPolyhedron3D branch; without a guard z_max/z_min retain their
// sentinel initial values (lowest()/max()) and both basal-d coefficients
// come out as +inf. This fuzz asserts the strong invariant: for every sample,
// the coefficients FillHexCrystalCoef reports (the first plane_cnt*4 floats
// of out_coef) must be finite — either the pyramid solves cleanly, or the
// guard trips and returns plane_cnt==0 (no coefficients to inspect).
//
// A weak "guard-fires-sometimes" check accompanies the strong invariant:
// under this sampling distribution the guard is expected to fire on a
// non-trivial fraction of samples, so a run that reports zero fires would
// mean either the sampling drifted away from the diagnostic domain or the
// guard branch was silently unreachable. Under these alpha ranges
// (0.1° < alpha < 89.9°) has_upper/has_lower are always true, so the
// pre-existing zero-volume `!has_upper && !has_lower && h2<eps` early-return
// cannot fire; any plane_cnt==0 outcome here is unambiguously attributable
// to the empty-feasible-region guard.
//
// Sampling distribution matches bench_geom_closedform.cpp::BM_DumpPlaneSets
// (see bench/bench_geom_closedform.cpp — verified against the red/green
// diagnostic dump this test protects against). NOTE: if the bench-side
// distribution changes, hit-rate assertions here will silently drift out of
// alignment with the diagnostic — keep the two in sync when touching either.
// The bench uses N=1500/arm; this fuzz uses N=1000/arm which is enough to
// sit comfortably away from the small-sample noise floor around a ~13.6%
// empirical fire rate. Seed is deterministic and independent of the bench
// seed so this test does not depend on the bench binary being run.
//
// The uniform_real_distribution / normal_distribution mappings from raw
// mt19937 output are not standardized across libstdc++ / libc++, so the
// exact fire count may drift across platforms. The rate-band assertion is
// deliberately wide ([5%, 30%]) to accommodate that; the finiteness
// invariant is what carries the load.
struct PyramidFeasibilityResult {
  int total = 0;
  int guard_fired = 0;      // plane_cnt == 0 (guard tripped, coefficients skipped)
  int solved = 0;           // plane_cnt > 0 with finite coefficients
  int nonfinite_leaks = 0;  // plane_cnt > 0 with non-finite coefficient — a BUG
};

enum class PyramidFuzzArm { kDirectWedge = 0, kMillerIndex = 1 };

PyramidFeasibilityResult RunPyramidFeasibilityFuzz(uint32_t seed, PyramidFuzzArm arm, int n, double sigma) {
  PyramidFeasibilityResult r{};
  std::mt19937 rng(seed);
  std::normal_distribution<double> d_noise(1.0, sigma);
  std::uniform_real_distribution<double> h_dist(0.2, 2.0);
  std::uniform_real_distribution<double> a_dist(1.0, 89.5);
  std::uniform_int_distribution<int> miller_idx(1, 4);
  for (int s = 0; s < n; s++) {
    float dist[6];
    for (float& d : dist) {
      d = static_cast<float>(d_noise(rng));
    }
    const auto h2 = static_cast<float>(h_dist(rng));
    const auto h1 = static_cast<float>(h_dist(rng) / 2.0);
    float au = 0;
    float al = 0;
    if (arm == PyramidFuzzArm::kDirectWedge) {
      au = static_cast<float>(a_dist(rng));
      al = static_cast<float>(a_dist(rng));
    } else {
      const int i1 = miller_idx(rng);
      const int i4 = miller_idx(rng);
      au = static_cast<float>(std::atan(math::kSqrt3_2 * i4 / i1 / kIceCrystalC) * math::kRadToDegree);
      al = au;
    }
    float coef[kMaxHexCrystalPlanes * 4];
    const size_t plane_cnt = FillHexCrystalCoef(au, al, h1, h2, h1, dist, coef);
    r.total++;
    if (plane_cnt == 0) {
      r.guard_fired++;
      continue;
    }
    bool all_finite = true;
    for (size_t i = 0; i < plane_cnt * 4; i++) {
      if (!std::isfinite(coef[i])) {
        all_finite = false;
        break;
      }
    }
    if (all_finite) {
      r.solved++;
    } else {
      r.nonfinite_leaks++;
      ADD_FAILURE() << "non-finite coefficient survived FillHexCrystalCoef: arm=" << static_cast<int>(arm)
                    << " sample=" << s << " au=" << au << " al=" << al << " h1=" << h1 << " h2=" << h2 << " dist=["
                    << dist[0] << "," << dist[1] << "," << dist[2] << "," << dist[3] << "," << dist[4] << "," << dist[5]
                    << "] plane_cnt=" << plane_cnt;
    }
  }
  return r;
}

// Direct-wedge arm: alpha sweeps 1°-89.5°, including the extreme-flat tail
// (>87°) that the B-ring bug family lives in. Under sigma=0.8 empirical fire
// rate ~13.6% (matches the diagnostic dump 89/700 for this arm).
TEST_F(V3TestCrystal, PyramidFeasibilityFuzzDirectWedge_sigma08) {
  const auto r = RunPyramidFeasibilityFuzz(0xFED08A00u, PyramidFuzzArm::kDirectWedge, 1000, 0.8);
  std::cerr << "[observability] direct-wedge sigma=0.8 guard_fired=" << r.guard_fired << " solved=" << r.solved
            << " nonfinite_leaks=" << r.nonfinite_leaks << " total=" << r.total << "\n";
  EXPECT_EQ(r.nonfinite_leaks, 0) << "empty-feasible-region guard must not leak +inf coefficients";
  EXPECT_GE(r.guard_fired, 50) << "guard fired on < 5% of samples — either distribution drifted "
                                  "away from the diagnostic domain, or the branch is unreachable "
                                  "(guard_fired="
                               << r.guard_fired << " / " << r.total << ")";
  EXPECT_LE(r.guard_fired, 300) << "guard fired on > 30% of samples — condition may be over-tight "
                                   "(guard_fired="
                                << r.guard_fired << " / " << r.total << ")";
  EXPECT_GT(r.solved, 500) << "healthy pyramids should still dominate (solved=" << r.solved << " / " << r.total << ")";
}

// Miller-index arm: alpha = atan(sqrt(3)/2 * i4/i1 / c), i1,i4 ∈ [1,4].
// Under sigma=0.8 empirical fire rate ~14.4% (matches the diagnostic dump
// 101/700 for this arm). Both pyramid construction paths must be covered
// independently — a majority-healthy result on one arm cannot statistically
// hide a bug on the other.
TEST_F(V3TestCrystal, PyramidFeasibilityFuzzMillerIndex_sigma08) {
  const auto r = RunPyramidFeasibilityFuzz(0xFED08A01u, PyramidFuzzArm::kMillerIndex, 1000, 0.8);
  std::cerr << "[observability] miller-index sigma=0.8 guard_fired=" << r.guard_fired << " solved=" << r.solved
            << " nonfinite_leaks=" << r.nonfinite_leaks << " total=" << r.total << "\n";
  EXPECT_EQ(r.nonfinite_leaks, 0) << "empty-feasible-region guard must not leak +inf coefficients";
  EXPECT_GE(r.guard_fired, 50) << "guard fired on < 5% of samples (Miller-index arm) "
                                  "(guard_fired="
                               << r.guard_fired << " / " << r.total << ")";
  EXPECT_LE(r.guard_fired, 300) << "guard fired on > 30% of samples (Miller-index arm) "
                                   "(guard_fired="
                                << r.guard_fired << " / " << r.total << ")";
  EXPECT_GT(r.solved, 500);
}

// Green-arm sanity: at sigma=0.3 the same distribution is well-behaved and
// the guard should almost never fire (bench dump: direct-wedge 694/700 solid,
// Miller-index 700/700). This is the fuzz-layer analogue of the red/green
// pair validated externally via BM_DumpPlaneSets; it guards against a guard
// that is silently over-tight and mis-degrades healthy pyramids.
TEST_F(V3TestCrystal, PyramidFeasibilityFuzzDirectWedge_sigma03_GreenBaseline) {
  const auto r = RunPyramidFeasibilityFuzz(0xFED03A00u, PyramidFuzzArm::kDirectWedge, 1000, 0.3);
  std::cerr << "[observability] direct-wedge sigma=0.3 (green) guard_fired=" << r.guard_fired << " solved=" << r.solved
            << " nonfinite_leaks=" << r.nonfinite_leaks << " total=" << r.total << "\n";
  EXPECT_EQ(r.nonfinite_leaks, 0);
  EXPECT_LE(r.guard_fired, 30) << "guard fired on > 3% of green-baseline samples — condition too tight "
                                  "(guard_fired="
                               << r.guard_fired << " / " << r.total << ")";
  EXPECT_GT(r.solved, 950);
}

// ==========================================================================
// Face-distance mesh-manifold regression coverage.
//
// Pre-fix on hex prisms, 4+ planes converging at near-coincident corners could
// escape the SolveConvexPolyhedronVtxD absolute-tolerance vertex dedup and
// produce non-manifold meshes (V=14/F=16 for a hex whose V should be 12), which
// downstream corrupted the polygon-face slots and SIGSEGV'd through
// Crystal::GetFn(IdType). Guard those invariants at three levels: closed-
// manifold fuzz, hand-picked degenerate/negative-d survival/rejection cases,
// and a direct GetFn(IdType) out-of-range unit test.
// ==========================================================================

// IsClosedTriMesh is declared in core/crystal.hpp and shared with the
// production gate in crystal.cpp (Crystal factory boundary) — this test file
// asserts against the real implementation instead of a hand-copied one, so a
// future tightening of the predicate (e.g. added manifold checks per
// doc/numerical-robustness.md) can't silently drift out of sync with what
// these tests actually verify.

// Shared fuzz driver for the prism face_distance matrix. Callers pass the
// distribution kind + spread; the helper draws `n` samples, builds a prism per
// sample, and buckets each outcome into exactly one of three states:
//   - healthy               : V/F satisfy IsClosedTriMesh and PolygonFaceCount()==8
//                             (6 side faces + 2 basal faces — the canonical hex)
//   - degenerate_but_legal  : V/F satisfy IsClosedTriMesh but PolygonFaceCount()<8
//                             (side face(s) collapsed — still a legal closed
//                             convex mesh, just with fewer than 8 polygon faces;
//                             this is the "legal but reduced" path that the
//                             filter/render pipeline must still traverse
//                             without crashing)
//   - rejected              : V==0 && F==0 (RejectMalformed folded a non-manifold
//                             mesh at the factory boundary — the correct outcome
//                             when opposite-pair sum ≤ 0 makes the body actually
//                             degenerate)
// The three buckets partition the sample space; any Crystal that ships with a
// V/F pair that neither satisfies IsClosedTriMesh nor is (0,0) is a bug and
// ADD_FAILURE fires immediately with the offending dist[] logged.
struct PrismFuzzResult {
  int healthy = 0;
  int degenerate_but_legal = 0;
  int rejected = 0;
  // Indirect coverage for the negative-face_distance reject path (AC3) under
  // random sampling, alongside the deterministic cases in
  // FaceDistanceRejectRealDegenerate: how many iterations drew at least one
  // negative dist[i], and how many of those were rejected (V==0 && F==0).
  int neg_input = 0;
  int neg_input_rejected = 0;
};

enum class PrismFuzzKind { kGaussian, kUniform };

PrismFuzzResult RunPrismFuzz(uint32_t seed, PrismFuzzKind kind, float mean, float std, int n) {
  PrismFuzzResult r{};
  std::mt19937 rng(seed);
  // kUniform matches CrystalMaker's semantics: (U-0.5)*std + mean, so std is
  // the full-width range around mean (verified against math.cpp:370). kGaussian
  // uses standard normal * std + mean (math.cpp:372-374).
  std::uniform_real_distribution<float> uni(mean - 0.5f * std, mean + 0.5f * std);
  std::normal_distribution<float> gauss(mean, std);
  for (int i = 0; i < n; i++) {
    float dist[6];
    bool has_neg = false;
    for (auto& x : dist) {
      x = (kind == PrismFuzzKind::kGaussian) ? gauss(rng) : uni(rng);
      if (x < 0.f) {
        has_neg = true;
      }
    }
    if (has_neg) {
      r.neg_input++;
    }
    Crystal c = Crystal::CreatePrism(1.2f, dist);
    const auto built = detail::BuildMeshFromCfGeom(c.CfGeom());
    const size_t v = built.mesh.GetVtxCnt();
    const size_t f = built.mesh.GetTriangleCnt();
    if (v == 0 && f == 0) {
      r.rejected++;
      if (has_neg) {
        r.neg_input_rejected++;
      }
      continue;
    }
    if (!IsClosedTriMesh(v, f)) {
      // ADD_FAILURE_AT (not ASSERT_TRUE) so the enclosing non-void helper
      // remains legal; the outer TEST_F then EXPECT_FALSE(HasFailure()) if
      // it wants to short-circuit. Bail out of the loop to avoid drowning
      // the log in duplicate failures on the same seed.
      ADD_FAILURE() << "Non-manifold Crystal escaped the factory gate on iter=" << i << " V=" << v << " F=" << f
                    << " dist=[" << dist[0] << "," << dist[1] << "," << dist[2] << "," << dist[3] << "," << dist[4]
                    << "," << dist[5] << "]";
      break;
    }
    if (c.PolygonFaceCount() < 8u) {
      r.degenerate_but_legal++;
    } else {
      r.healthy++;
    }
  }
  return r;
}

// gauss(1.0, std) baselines from the 377.1 diagnosis matrix (12500 samples at
// std=0.50 — the reference tier for these proportions). At N=10000 we allow
// ±3% absolute so the sample-size difference between the diagnosis run and
// this unit-test fuzz does not turn into a flake.
TEST_F(V3TestCrystal, PrismEulerFuzzGauss_std015) {
  const auto r = RunPrismFuzz(0xFACED151u, PrismFuzzKind::kGaussian, 1.0f, 0.15f, 10000);
  // Small std → geometry stays inside the healthy path. Allow a tiny tail for
  // the near-boundary numerical edge (<1% expected).
  EXPECT_LT(r.degenerate_but_legal, 100) << "std=0.15 degenerate_but_legal=" << r.degenerate_but_legal;
  EXPECT_LT(r.rejected, 100) << "std=0.15 rejected=" << r.rejected;
  EXPECT_GT(r.healthy, 9800);
}

TEST_F(V3TestCrystal, PrismEulerFuzzGauss_std030) {
  // 377.1 baseline: ~13.6% degenerate_but_legal.
  const auto r = RunPrismFuzz(0xFACED152u, PrismFuzzKind::kGaussian, 1.0f, 0.30f, 10000);
  EXPECT_GT(r.degenerate_but_legal, 1000) << r.degenerate_but_legal;  // > 10%
  EXPECT_LT(r.degenerate_but_legal, 1700) << r.degenerate_but_legal;  // < 17%
  EXPECT_GT(r.healthy, 8000);
}

TEST_F(V3TestCrystal, PrismEulerFuzzGauss_std050) {
  // 377.1 baseline: ~49.2% degenerate_but_legal. This is the AC5 anchor — if
  // std ever gets silently clamped, this proportion collapses to the std=0.15
  // level and the test goes red immediately.
  const auto r = RunPrismFuzz(0xFACED153u, PrismFuzzKind::kGaussian, 1.0f, 0.50f, 10000);
  EXPECT_GT(r.degenerate_but_legal, 4400) << r.degenerate_but_legal;  // > 44%
  EXPECT_LT(r.degenerate_but_legal, 5400) << r.degenerate_but_legal;  // < 54%

  // AC3 indirect coverage (plan Step 2): at this std, gauss(1.0, 0.5) is
  // unbounded and must draw negative face_distance values on some iterations
  // — the reject path (opposite-pair sum <= 0) must fire on a non-trivial,
  // non-overwhelming fraction of them (some negative-containing draws still
  // keep a positive opposite-pair sum and legitimately survive, per
  // FaceDistanceReverseSurvives).
  ASSERT_GT(r.neg_input, 0) << "std=0.50 must draw at least one negative dist[i]";
  EXPECT_GT(r.neg_input_rejected, 0) << "reject path never fired on any negative-containing draw";
  EXPECT_LT(r.neg_input_rejected, r.neg_input)
      << "every negative-containing draw was rejected — reject judgment may be over-tight "
      << "(should only fire when opposite-pair sum <= 0, not on any negative d[i])";
}

// Uniform baseline (empirical): with (U-0.5)*std+mean semantics all d_i stay in
// [mean-0.5*std, mean+0.5*std] > 0 for std < 2*mean, so `rejected` is exactly
// zero (no opposite-pair-sum-≤0 outcome ever reachable) at these tiers. The
// remaining invariant is closed-manifold-or-reject, which the shared helper's
// inline ASSERT enforces on every iteration.
//
// A one-off empirical sweep (std in {0.5, 0.8, 1.0, 1.3, 1.6, 1.9, 1.99},
// N=10000, not committed as a permanent test) confirmed degenerate_but_legal
// is exactly 0 at std<=0.5 and only becomes non-trivial from std>=0.8 (1.7%),
// climbing to double digits by std~1.0-1.3. Bounded uniform noise simply does
// not reach the near-coincident-vertex combinations gauss(1.0, std) reaches
// at the same nominal std — the two distributions are not comparable at
// matched std values for this purpose, only at matched *variance* (uniform's
// stddev is width/sqrt(12), i.e. roughly 3.5x tighter than its literal
// "std" parameter suggests). Widening uniform's std into the >=0.8 range to
// chase a non-zero degenerate proportion also starts producing occasional
// `rejected` outcomes (observed at std=0.8/1.0/1.6/1.99) via the *same*
// closed-mesh Euler-check gate that rejects real-degenerate gaussian inputs
// — not the opposite-pair-sum path this comment's zero-rejection claim rests
// on — so it would invalidate the EXPECT_EQ(rejected, 0) invariant below at
// the same tier, not just add a new assertion. Redesigning the uniform std
// matrix to reach both goals at once is out of scope here: the 015/030/050
// tiers were chosen to mirror gaussian's tiers 1:1, and this fuzz layer's job
// is exercising the "uniform path never mis-triggers real-degenerate
// rejection" invariant — the reject-path coverage AC3 requires is already
// covered by gaussian's fuzz (std=0.50 neg_input assertions above) and the
// deterministic FaceDistanceRejectRealDegenerate cases, both of which fire
// through the identical CreatePrism -> RejectMalformed code path regardless
// of which distribution produced the input.
TEST_F(V3TestCrystal, PrismEulerFuzzUniform_std015) {
  const auto r = RunPrismFuzz(0xFACED154u, PrismFuzzKind::kUniform, 1.0f, 0.15f, 10000);
  EXPECT_EQ(r.rejected, 0) << "uniform tight-spread should never reject";
}

TEST_F(V3TestCrystal, PrismEulerFuzzUniform_std030) {
  const auto r = RunPrismFuzz(0xFACED155u, PrismFuzzKind::kUniform, 1.0f, 0.30f, 10000);
  EXPECT_EQ(r.rejected, 0);
  // Observed (N=10000, seed 0xFACED155): degenerate_but_legal=0 — the bounded
  // (U-0.5)*std+mean range [0.85, 1.15] never triggers wedge-collapse geometry
  // at this std, so no meaningful lower-bound assertion is possible here
  // without risking flakes on a proportion that is genuinely ~0, not merely
  // small. The load-bearing invariants stay existence-only (no rejection, no
  // manifold escape); this observed value is logged for future debugging so a
  // shift toward non-zero-but-still-low degenerate rates does not silently
  // disappear from view either.
  std::cerr << "[observability] uniform std=0.30 degenerate_but_legal=" << r.degenerate_but_legal << "\n";
}

TEST_F(V3TestCrystal, PrismEulerFuzzUniform_std050) {
  const auto r = RunPrismFuzz(0xFACED156u, PrismFuzzKind::kUniform, 1.0f, 0.50f, 10000);
  EXPECT_EQ(r.rejected, 0);
  // Observed (N=10000, seed 0xFACED156): degenerate_but_legal=0, same
  // rationale as std=0.30 above.
  std::cerr << "[observability] uniform std=0.50 degenerate_but_legal=" << r.degenerate_but_legal << "\n";
}

TEST_F(V3TestCrystal, FaceDistanceKnownMalformedInputsHealed) {
  // Deterministic pin for the actual root cause (SolveConvexPolyhedronVtxD's
  // vertex dedup): these are exact float32 face_distance combinations
  // diagnosed to reproduce a non-manifold mesh (V=14/F=16 or V=12/F=12) on
  // pre-fix HEAD — captured via hex-float bit patterns (not decimal, which
  // loses the precision that triggers the near-coincident-vertex path) during
  // root-cause diagnosis, replaying the exact SolveConvexPolyhedronVtxD input
  // that produced 4+ planes converging within the pre-fix absolute dedup
  // tolerance. Unlike the random-seed fuzz tests above (PrismEulerFuzzGauss/
  // Uniform), whose seeded 10000-sample runs have near-zero probability of
  // landing on the ~14-in-200k trigger set, these inputs deterministically
  // hit the fix on every run: if the scale-relative vertex-dedup tolerance in
  // SolveConvexPolyhedronVtxD is ever reverted or narrowed back toward the
  // old absolute tolerance, every case here fails immediately and
  // reproducibly, with no dependency on RNG state or memory layout.
  constexpr float kH = 1.2f;
  const float kKnownMalformedInputs[][6]{
    // clang-format off
    { 0x1.ac21bp+0f,  0x1.1d0d04p+0f, 0x1.c2325ap-1f, 0x1.18b52ep+0f, 0x1.eb8ef4p+0f, 0x1.a5ae5p-1f  },
    { 0x1.d9c724p-2f, 0x1.240ec2p+0f, 0x1.69eed8p-1f, 0x1.4feeaep+0f, 0x1.4a5544p-1f, 0x1.1b9f16p+0f },
    { 0x1.669738p+0f, 0x1.d40808p-3f, 0x1.7525ep-1f,  0x1.3bf1cap-1f, 0x1.129d3cp+0f, 0x1.2c18e2p+0f },
    { 0x1.06a254p+0f, 0x1.f0702p-1f,  0x1.03dc38p+0f, 0x1.c9dc68p-1f, 0x1.12e3fp+1f,  0x1.40dc9ep+0f },
    { 0x1.36916p-3f,  0x1.7051cp-4f,  0x1.4aa59cp+0f, 0x1.f07434p-1f, 0x1.95810cp+0f, 0x1.3a9366p-1f },
    { 0x1.d5cbdcp-2f, 0x1.8c5ca4p+0f, 0x1.32126ap+0f, 0x1.44c52cp+0f, 0x1.2b007p-4f,  0x1.665a6ep-1f },
    { 0x1.4be45cp+0f, 0x1.12d93p-1f,  0x1.570774p+0f, 0x1.9b3b64p-1f, 0x1.485f34p+0f, 0x1.7867ap+0f  },
    // clang-format on
  };
  for (size_t i = 0; i < std::size(kKnownMalformedInputs); i++) {
    Crystal c = Crystal::CreatePrism(kH, kKnownMalformedInputs[i]);
    const auto built = detail::BuildMeshFromCfGeom(c.CfGeom());
    const size_t v = built.mesh.GetVtxCnt();
    const size_t f = built.mesh.GetTriangleCnt();
    ASSERT_TRUE(IsClosedTriMesh(v, f)) << "known pre-fix-malformed input[" << i << "] still produces a non-manifold "
                                       << "mesh post-fix: V=" << v << " F=" << f;
  }
}

TEST_F(V3TestCrystal, FaceDistanceReverseSurvives) {
  // Anti-regression for "hide bugs in the parameter domain": these must not be
  // silently rejected as degenerate. Numbers come from the 377.1 diagnosis
  // matrix (single-edge zero → trapezoidal prism, full 1's → healthy hex).
  {
    float dist[6]{ 1, 1, 1, 1, 1, 1 };
    Crystal c = Crystal::CreatePrism(1.2f, dist);
    const auto built = detail::BuildMeshFromCfGeom(c.CfGeom());
    EXPECT_EQ(built.mesh.GetVtxCnt(), 12u);
    EXPECT_EQ(built.mesh.GetTriangleCnt(), 20u);
  }
  {
    // Single-edge zero: d0=0 collapses one prism plane onto the origin;
    // opposite-pair sum d0+d3=1>0 keeps the body convex → V=8, F=12 trapezoid.
    // Rejecting this would erase user-legitimate degenerate crystals and hide
    // real bugs "in the parameter domain".
    float dist[6]{ 0, 1, 1, 1, 1, 1 };
    Crystal c = Crystal::CreatePrism(1.2f, dist);
    const auto built = detail::BuildMeshFromCfGeom(c.CfGeom());
    EXPECT_EQ(built.mesh.GetVtxCnt(), 8u);
    EXPECT_EQ(built.mesh.GetTriangleCnt(), 12u);
  }
  {
    // Negative d with opposite-pair sum > 0: d0=-0.5 + d3=1 = 0.5 > 0. Not the
    // canonical hex shape (geometry shifts), but must NOT be rejected as
    // degenerate — negative face_distance is a legitimate input path unlocked
    // by the removal of std::abs at CrystalMaker.
    float dist[6]{ -0.5f, 1, 1, 1, 1, 1 };
    Crystal c = Crystal::CreatePrism(1.2f, dist);
    const auto built = detail::BuildMeshFromCfGeom(c.CfGeom());
    EXPECT_GT(built.mesh.GetVtxCnt(), 0u) << "negative-d with positive opposite-pair sum wrongly rejected";
    EXPECT_GT(built.mesh.GetTriangleCnt(), 0u);
    EXPECT_TRUE(IsClosedTriMesh(built.mesh.GetVtxCnt(), built.mesh.GetTriangleCnt()));
  }
}

TEST_F(V3TestCrystal, FaceDistanceRejectRealDegenerate) {
  // Anti-regression for "let real degenerate through": opposite pair sums ≤ 0
  // → zero-volume (or worse) body. Must be rejected as V=0/F=0 rather than
  // propagate as a partial mesh that downstream would wild-read. Cover both
  // zero-sum (canonical degenerate) and strictly-negative-sum (negative-d
  // wraps past the origin) inputs — the second bucket is the AC3 focus and
  // was not exercised before the negative-d path was unlocked at CrystalMaker.
  struct Case {
    const char* name;
    float dist[6];
  };
  const Case kCases[]{
    // Zero opposite-pair sum: flat plate.
    { "zero_sum_d0_d3", { 0.f, 1.f, 1.f, 0.f, 1.f, 1.f } },
    // Strictly negative opposite-pair sum: one plane wraps past the origin
    // faster than its opposite can compensate → non-manifold or empty solid.
    { "neg_sum_d0_d3", { -1.5f, 1.f, 1.f, 1.f, 1.f, 1.f } },
    { "neg_sum_d1_d4", { 1.f, -1.5f, 1.f, 1.f, 1.f, 1.f } },
    { "neg_sum_d2_d5", { 1.f, 1.f, -1.5f, 1.f, 1.f, 1.f } },
  };
  for (const auto& tc : kCases) {
    Crystal c = Crystal::CreatePrism(1.2f, tc.dist);
    const auto built = detail::BuildMeshFromCfGeom(c.CfGeom());
    EXPECT_EQ(built.mesh.GetVtxCnt(), 0u) << "case=" << tc.name;
    EXPECT_EQ(built.mesh.GetTriangleCnt(), 0u) << "case=" << tc.name;
  }
}

TEST_F(V3TestCrystal, GetFnPolyIdxOutOfRangeReturnsInvalidId) {
  // The IdType overload has both an outer bound (poly_idx < poly_face_cnt_) and
  // an inner defense (poly_face_tri_id_[poly_idx] must index into fn_map_).
  // Both branches: an out-of-range poly_idx must not read past poly_face_tri_id_.
  Crystal c = Crystal::CreatePrism(1.3f);
  const auto n = c.PolygonFaceCount();
  ASSERT_GT(n, 0u);
  // Outer bound: index equal to the count returns kInvalidId.
  EXPECT_EQ(c.GetFn(static_cast<IdType>(n)), kInvalidId);
  EXPECT_EQ(c.GetFn(static_cast<IdType>(n + 100)), kInvalidId);
  EXPECT_EQ(c.GetFn(kInvalidId), kInvalidId);
  // Valid ids in range must return a defined fn (not the sentinel).
  for (IdType i = 0; i < static_cast<IdType>(n); i++) {
    EXPECT_NE(c.GetFn(i), kInvalidId) << "poly_idx=" << i;
  }
}

TEST_F(V3TestCrystal, PyramidRandomFaceDistanceMoveGetFnLegal) {
  // Regression guard for the polygon-face count/stride mismatch previously
  // living in BuildPolygonFaceData: when the degenerate-representative-triangle
  // branch fires it shrank poly_face_cnt_ below the actual allocation stride;
  // Crystal's copy/move ctors then re-derived poly_face_d_ / poly_face_tri_id_
  // offsets from the shrunk count and read from wrong regions of
  // poly_face_data_. Symptoms: CPU GetFn(IdType) either returned kInvalidId
  // (bound-checked garbage tri) or, when the garbage happened to land in
  // fn_map_ range, silently wrong face numbers; Metal UploadCrystal wild-read
  // → SIGSEGV. After the fix poly_face_cnt_ ≡ actual allocation stride so
  // copy/move are structurally safe.
  //
  // Detection strategy: fixed-seed deterministic sweep over pyramid +
  // face_distance samples; for each surviving crystal exercise both move-ctor
  // and move-assignment (the real production paths — MakeCrystal returns by
  // value which forces move, and metal_trace_backend re-assigns current_crystal
  // each dispatch batch), then require every polygon face's GetFn to (a) not be
  // kInvalidId and (b) resolve to a legal pyramid face number. Pre-fix, at
  // least one of the swept crystals lands on the shrink path and one of these
  // two assertions fires; post-fix, both stay bit-for-bit intact regardless.
  //
  // PORTABILITY: the face_distance samples are drawn by mapping raw mt19937
  // output through a hand-written uniform transform — NOT std::normal_/
  // uniform_real_distribution, whose bit-exact output sequences are unspecified
  // and differ across libstdc++ (dev49/Linux) and libc++ (macOS). mt19937's
  // uint32 stream plus this manual mapping is identical on every stdlib, so the
  // fixed seed constructs the *same* crystals everywhere.
  //
  // Step 3 aftermath (2026-07-21): the anti-vacuous DegenerateShrinkCount()
  // assertion was dropped. The pyramid factory walks the closed-form path
  // (MakePyramidClosedForm → PopulateFromCfGeom) and never reaches
  // BuildPolygonFaceData, so there is no shrink counter to exercise. The
  // remaining assertions still guard PopulateFromCfGeom's move/assign behavior
  // under a wide fuzz — the failure symptoms they were originally added to catch
  // (kInvalidId / out-of-range fn) would still fire on any pointer-rebinding
  // regression in the copy/move ctors, regardless of which factory populated
  // the fields.
  std::mt19937 gen(42);
  // Map a fresh mt19937 draw to face_distance in [0.3, 1.7) — spread wide
  // enough to produce near-zero-area representative triangles (the shrink
  // trigger) while staying upstream-constructible.
  auto next_dist = [&gen]() {
    const float u = static_cast<float>(gen() >> 8) * (1.0f / 16777216.0f);  // [0,1)
    return 0.3f + u * 1.4f;
  };
  // Multiple h regimes — the SHRINK case requires an upstream-passable
  // mesh that still has a near-zero-area representative triangle for at
  // least one polygon plane; that condition is sensitive to the pyramid
  // wedge geometry as well as face_distance.
  struct HTuple {
    float h1, h2, h3;
  };
  const HTuple h_regimes[] = {
    { 0.3f, 1.0f, 0.3f },    // moderate wedge, close to production defaults
    { 0.1f, 1.2f, 0.5f },    // very thin upper wedge
    { 0.8f, 0.3f, 0.8f },    // wedge-heavy
    { 0.05f, 1.5f, 0.05f },  // extremely thin wedges
  };
  size_t swept = 0;
  for (int iter = 0; iter < 500; iter++) {
    float dist[6];
    for (int i = 0; i < 6; i++) {
      dist[i] = next_dist();
    }
    const auto& h = h_regimes[iter % 4];
    Crystal c = Crystal::CreatePyramid(1, 1, 1, 1, h.h1, h.h2, h.h3, dist);
    // Upstream rejects severely degenerate meshes (non-manifold Euler check)
    // by returning a zero-triangle Crystal — skip; the count/stride path never
    // runs there.
    if (c.PolygonFaceCount() == 0) {
      continue;
    }
    // Exercise both move-ctor (mirrors `MakeCrystal(...)` by-value return) and
    // move-assignment (mirrors `current_crystal = MakeCrystal(...)` in
    // metal_trace_backend.mm — the exact production call point the pre-fix
    // crash was observed at).
    const size_t face_cnt_before = c.PolygonFaceCount();
    Crystal moved(std::move(c));
    ASSERT_EQ(moved.PolygonFaceCount(), face_cnt_before) << "iter=" << iter << " (move-ctor count drift)";
    Crystal assigned;
    assigned = std::move(moved);
    ASSERT_EQ(assigned.PolygonFaceCount(), face_cnt_before) << "iter=" << iter << " (move-assign count drift)";
    for (size_t p = 0; p < assigned.PolygonFaceCount(); p++) {
      IdType fn = assigned.GetFn(static_cast<IdType>(p));
      ASSERT_NE(fn, kInvalidId) << "iter=" << iter << " poly_idx=" << p
                                << " — GetFn returned kInvalidId, indicating polygon face tri_id "
                                << "landed in a wrong region of poly_face_data_ (count/stride mismatch)";
      ASSERT_TRUE(IsLegalFace(CrystalKind::kPyramid, static_cast<int>(fn)))
          << "iter=" << iter << " poly_idx=" << p << " fn=" << fn
          << " — GetFn resolved to an out-of-range face number, indicating polygon face tri_id "
          << "landed in a plausible-but-wrong region of fn_map_";
    }
    swept++;
  }
  // Sanity: the fixed seed must have exercised at least a few surviving
  // crystals, otherwise the guard is vacuous.
  ASSERT_GT(swept, 20u) << "fixed-seed sweep produced too few surviving crystals; guard is vacuous";
}

// Step 1 of the closed-form representation swap adds a CrystalGeom flat-POD
// field, populated later (Steps 2/3) by the closed-form factory paths. Until
// wired, all crystals — including those built by the current factories —
// present cf_face_cnt_ == 0 (the "not populated" sentinel). This test pins:
//   1. Default-constructed Crystal has face_cnt == 0.
//   2. Copy/move preserve face_cnt (currently 0; guards the plumbing added in
//      Step 1 so a broken copy path is caught before Steps 2/3 start relying
//      on the field being carried through).
//   3. The capacity constants are large enough for the pyramid worst case
//      (defence in depth alongside the static_asserts in crystal.cpp).
TEST(CrystalGeomStep1, DefaultInstanceHasZeroFaceCount) {
  Crystal c;
  EXPECT_EQ(c.CfGeom().face_cnt, 0);
  Crystal cp = c;
  EXPECT_EQ(cp.CfGeom().face_cnt, 0);
  Crystal mv = std::move(cp);
  EXPECT_EQ(mv.CfGeom().face_cnt, 0);
  Crystal cp2 = Crystal::CreatePrism(1.0f);
  cp2 = c;
  EXPECT_EQ(cp2.CfGeom().face_cnt, 0);
}

TEST(CrystalGeomStep2, PrismFactoryPopulatesCfGeom) {
  // Post-Step-2 tripwire: CreatePrism now walks the closed-form path and must
  // populate cf_geom_ with the parametric 8-face prism layout (2 basal + 6
  // side). If a future refactor accidentally re-routes prism construction
  // through a mesh-only path, face_cnt drops to 0 and this test fails.
  Crystal prism = Crystal::CreatePrism(1.0f);
  EXPECT_EQ(prism.CfGeom().face_cnt, 8);
  EXPECT_EQ(prism.PolygonFaceCount(), 8u);
}

TEST(CrystalGeomStep3, PyramidFactoryPopulatesCfGeom) {
  // Post-Step-3 tripwire: both pyramid ctor paths (alpha and Miller) now walk
  // the closed-form path and must populate cf_geom_ with the parametric
  // 20-face pyramid layout (2 basal + 6 prism side + 6 upper cone + 6 lower
  // cone). If a future refactor accidentally re-routes pyramid construction
  // through a mesh-only path, face_cnt drops to 0 and this test fails.
  Crystal p_alpha = Crystal::CreatePyramid(0.5f, 1.0f, 0.5f);
  EXPECT_EQ(p_alpha.CfGeom().face_cnt, 20);
  EXPECT_EQ(p_alpha.PolygonFaceCount(), 20u);

  float dist[6]{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  Crystal p_miller = Crystal::CreatePyramid(1, 1, 1, 1, 0.5f, 1.0f, 0.5f, dist);
  EXPECT_EQ(p_miller.CfGeom().face_cnt, 20);
  EXPECT_EQ(p_miller.PolygonFaceCount(), 20u);
}

TEST(CrystalGeomStep1, CapacityConstantsBoundPyramidWorstCase) {
  EXPECT_GE(kCrystalGeomMaxFaces, 20);
  // Anchor to the closed-form evaluator's per-face capacity rather than a hard
  // literal, so this defence-in-depth check tracks the compile-time static_assert
  // in crystal.cpp automatically. The measured hexagonal-family worst case is 7
  // (see the kCrystalGeomMaxVtxPerFace comment in crystal.hpp).
  EXPECT_GE(kCrystalGeomMaxVtxPerFace, kClosedFormPyramidMaxFaceVtx);
  EXPECT_GE(kCrystalGeomMaxVtxPerFace, 7);
}

// ---------------------------------------------------------------------------
// Empty-Crystal state contract.
//
// Two producers can hand downstream code a Crystal that bounds no volume:
//   (a) `Crystal c;`             — the public default constructor;
//   (b) `return Crystal();`      — the closed-form validity gate's reject
//                                  branch in MakePrismClosedForm /
//                                  MakePyramidClosedForm (crystal.cpp), which
//                                  IS reachable from production input because
//                                  CrystalMaker feeds it randomized h /
//                                  face_distance draws.
// The tests below pin, at run time, that (a) and (b) produce a byte-for-byte
// identical state, and that that state is the "nothing populated" sentinel the
// rest of the codebase tests for (`face_cnt == 0` / `PolygonFaceCount() == 0`).
// Consumers therefore need exactly one defence, not two; and any future change
// that makes the reject branch return a *partially* populated Crystal (e.g. a
// plane set without corners) breaks these tests rather than surfacing as a
// downstream out-of-bounds read.
// ---------------------------------------------------------------------------

// Assert that `c` is in the canonical empty state, field by field. Kept as a
// helper so both producers are checked against the same predicate.
void ExpectCanonicalEmptyCrystal(const Crystal& c, const char* who) {
  const CrystalGeom& g = c.CfGeom();
  EXPECT_EQ(g.face_cnt, 0) << who;
  for (int slot = 0; slot < kCrystalGeomMaxFaces; slot++) {
    EXPECT_FALSE(g.face_present[slot]) << who << " slot=" << slot;
    EXPECT_EQ(g.face_vtx_cnt[slot], 0) << who << " slot=" << slot;
    EXPECT_EQ(g.face_number[slot], 0) << who << " slot=" << slot;
  }
  for (int i = 0; i < kCrystalGeomMaxFaces * 4; i++) {
    EXPECT_EQ(g.plane_coef[i], 0.0f) << who << " plane_coef[" << i << "]";
  }
  for (int i = 0; i < kCrystalGeomMaxFaces * 3; i++) {
    EXPECT_EQ(g.face_normal[i], 0.0f) << who << " face_normal[" << i << "]";
  }
  for (int i = 0; i < kCrystalGeomMaxFaces * kCrystalGeomMaxVtxPerFace * 3; i++) {
    EXPECT_EQ(g.face_vtx[i], 0.0f) << who << " face_vtx[" << i << "]";
  }
  // Polygon-face view derived from cf_geom_ — PopulateFromCfGeom never ran.
  EXPECT_EQ(c.PolygonFaceCount(), 0u) << who;
  EXPECT_EQ(c.GetPolygonFaceNormal(), nullptr) << who;
  EXPECT_EQ(c.GetPolygonFaceDist(), nullptr) << who;
  // Symmetry reduction is inapplicable (the factories set fn_period_ = 6 only
  // on the success path), and GetFn must reject every index rather than read
  // out of a null table.
  EXPECT_EQ(c.FnPeriod(), -1) << who;
  EXPECT_EQ(c.GetFn(0), kInvalidId) << who;
  EXPECT_EQ(c.config_id_, kInvalidId) << who;
  // On-demand triangulation of an empty geometry must yield an empty mesh, not
  // a degenerate one — this is what the GPU device-gen tri buffers size from.
  EXPECT_EQ(detail::CountEntrySubTris(g), 0u) << who;
  const auto built = detail::BuildMeshFromCfGeom(g);
  EXPECT_EQ(built.mesh.GetVtxCnt(), 0u) << who;
  EXPECT_EQ(built.mesh.GetTriangleCnt(), 0u) << who;
}

// Byte-for-byte comparison of the flat POD geometry, so "equivalent" means
// equivalent and not just "both report face_cnt == 0".
bool CfGeomBytesEqual(const CrystalGeom& a, const CrystalGeom& b) {
  return std::memcmp(&a, &b, sizeof(CrystalGeom)) == 0;
}

TEST(EmptyCrystalContract, DefaultConstructedStateIsPinnedFieldByField) {
  Crystal c;
  ExpectCanonicalEmptyCrystal(c, "default-constructed");

  // Copy / move must carry the empty state through unchanged — these are the
  // operations `std::vector<Crystal>` performs on reallocation, and the ones
  // the backends use to stage a resolved crystal.
  Crystal copied = c;  // NOLINT(performance-unnecessary-copy-initialization)
  ExpectCanonicalEmptyCrystal(copied, "copy-constructed-from-empty");
  Crystal moved = std::move(copied);
  ExpectCanonicalEmptyCrystal(moved, "move-constructed-from-empty");

  // Assigning an empty over a populated instance must fully clear it (a stale
  // poly_face_data_ left behind would be read with poly_face_cnt_ == 0 above,
  // but the pointer accessors are part of the pinned contract).
  Crystal overwritten = Crystal::CreatePrism(1.0f);
  ASSERT_GT(overwritten.PolygonFaceCount(), 0u) << "fixture precondition: prism must be populated";
  overwritten = c;
  ExpectCanonicalEmptyCrystal(overwritten, "copy-assigned-empty-over-populated");

  Crystal overwritten2 = Crystal::CreatePrism(1.0f);
  ASSERT_GT(overwritten2.PolygonFaceCount(), 0u) << "fixture precondition: prism must be populated";
  Crystal donor;
  overwritten2 = std::move(donor);
  ExpectCanonicalEmptyCrystal(overwritten2, "move-assigned-empty-over-populated");
}

TEST(EmptyCrystalContract, DegenerateFactoryRejectEqualsDefaultConstruction) {
  const Crystal reference;  // canonical empty state

  // Prism reject branch: h <= kFloatEps fails IsValidClosedFormPrism, taking
  // the silent `return Crystal();` path in MakePrismClosedForm.
  Crystal prism_reject = Crystal::CreatePrism(0.0f);
  ASSERT_EQ(prism_reject.CfGeom().face_cnt, 0)
      << "fixture precondition: CreatePrism(0.0f) must hit the validity-gate reject branch; "
      << "if this fires the gate changed and the rest of this test is vacuous";
  ExpectCanonicalEmptyCrystal(prism_reject, "prism-validity-gate-reject");
  EXPECT_TRUE(CfGeomBytesEqual(prism_reject.CfGeom(), reference.CfGeom()))
      << "prism reject branch produced a cf_geom_ that differs bytewise from default construction";

  // Prism reject branch reached through the *warning* arm instead (h > eps but
  // the opposite-pair sums cannot bound a solid): all six distances negative.
  {
    float dist[6]{ -1, -1, -1, -1, -1, -1 };
    Crystal prism_neg = Crystal::CreatePrism(1.2f, dist);
    ASSERT_EQ(prism_neg.CfGeom().face_cnt, 0)
        << "fixture precondition: all-negative face distances must fail the closed-form validity gate";
    ExpectCanonicalEmptyCrystal(prism_neg, "prism-all-negative-dist-reject");
    EXPECT_TRUE(CfGeomBytesEqual(prism_neg.CfGeom(), reference.CfGeom()));
  }

  // Pyramid reject branch: zero-volume input fails IsValidClosedFormPyramid
  // (present face count < 4) and takes `return Crystal();` in
  // MakePyramidClosedForm.
  {
    float dist[6]{ 0, 0, 0, 0, 0, 0 };
    Crystal pyr_reject = Crystal::CreatePyramid(28.0f, 28.0f, 0.0f, 0.0f, 0.0f, dist);
    ASSERT_EQ(pyr_reject.CfGeom().face_cnt, 0)
        << "fixture precondition: zero-volume pyramid must hit the validity-gate reject branch";
    ExpectCanonicalEmptyCrystal(pyr_reject, "pyramid-validity-gate-reject");
    EXPECT_TRUE(CfGeomBytesEqual(pyr_reject.CfGeom(), reference.CfGeom()));
  }
}

TEST(EmptyCrystalContract, MakeCrystalCanReturnTheEmptyStateFromRandomizedParams) {
  // Reachability evidence for the empty state in *production* input terms: the
  // parameters below are an ordinary randomized crystal config, not a
  // hand-built empty Crystal. A wide face_distance spread drives some draws
  // past the closed-form validity gate, so MakeCrystal — the single entry point
  // every backend builds crystals through — hands back the same empty state the
  // default constructor produces.
  //
  // Consumers must therefore treat "MakeCrystal returned an empty crystal" as a
  // reachable input, not as an impossible one.
  PrismCrystalParam param;
  param.h_ = Distribution{ DistributionType::kGaussian, 1.0f, 0.5f };
  for (auto& d : param.d_) {
    d = Distribution{ DistributionType::kGaussian, 1.0f, 0.8f };
  }

  RandomNumberGenerator rng(20260724u);
  size_t empty_cnt = 0;
  size_t populated_cnt = 0;
  const Crystal reference;
  for (int i = 0; i < 2000; i++) {
    Crystal c = MakeCrystal(rng, CrystalParam{ param });
    if (c.CfGeom().face_cnt == 0) {
      empty_cnt++;
      // Every empty crystal that reaches a consumer is the canonical state —
      // there is no second, partially-populated flavour to defend against.
      ASSERT_TRUE(CfGeomBytesEqual(c.CfGeom(), reference.CfGeom())) << "draw " << i;
      ASSERT_EQ(c.PolygonFaceCount(), 0u) << "draw " << i;
      ASSERT_EQ(c.FnPeriod(), -1) << "draw " << i;
    } else {
      populated_cnt++;
    }
  }
  EXPECT_GT(empty_cnt, 0u) << "the randomized sweep produced no degenerate draw — the reachability claim "
                           << "this test exists to pin is no longer exercised by these parameters";
  EXPECT_GT(populated_cnt, 0u) << "the sweep produced only degenerate draws — parameters are unrealistic "
                               << "and the test says nothing about production behaviour";
}

}  // namespace
