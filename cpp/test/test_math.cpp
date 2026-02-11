#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "core/geo3d.hpp"
#include "core/math.hpp"

extern std::string config_file_name;
using namespace icehalo;

namespace {

class V3TestMath : public ::testing::Test {
 protected:
  void SetUp() override {}
};

TEST_F(V3TestMath, Solve3Case1) {
  float coef[12]{
    1, 2, 3, 1,  //
    3, 4, 5, 2,  //
    5, 6, 9, 3,  //
  };

  float xyz[3];
  auto solved = v3::SolvePlanes(coef, coef + 4, coef + 8, xyz);

  ASSERT_TRUE(solved);
  ASSERT_NEAR(xyz[0], 0.0f, math::kFloatEps);
  ASSERT_NEAR(xyz[1], -1.0f / 2.0f, math::kFloatEps);
  ASSERT_NEAR(xyz[2], 0.0f, math::kFloatEps);
}

TEST_F(V3TestMath, Solve3Case2) {
  float coef[12]{
    0,     0,     1,     -0.863,  //
    0.407, 0,     0.217, -0.328,  //
    0.204, 0.353, 0.217, -0.328,  //
  };

  float xyz[3];
  auto solved = v3::SolvePlanes(coef, coef + 4, coef + 8, xyz);

  ASSERT_TRUE(solved);
  ASSERT_NEAR(xyz[0], 0.34577f, 2e-5);
  ASSERT_NEAR(xyz[1], 0.19884f, 2e-5);
  ASSERT_NEAR(xyz[2], 0.863f, 2e-5);
}

// ============================================================================
// SolveLines tests
// ============================================================================

TEST_F(V3TestMath, SolveLinesIntersect) {
  // y = x  =>  x - y = 0  =>  coef = [1, -1, 0]
  // y = -x + 2  =>  x + y - 2 = 0  =>  coef = [1, 1, -2]
  // Intersection at (1, 1)
  float coef1[3]{ 1, -1, 0 };
  float coef2[3]{ 1, 1, -2 };
  float res[2];

  ASSERT_TRUE(v3::SolveLines(coef1, coef2, res));
  EXPECT_NEAR(res[0], 1.0f, math::kFloatEps);
  EXPECT_NEAR(res[1], 1.0f, math::kFloatEps);
}

TEST_F(V3TestMath, SolveLinesPerpendicular) {
  // x = 3  =>  1*x + 0*y - 3 = 0
  // y = -2  =>  0*x + 1*y + 2 = 0
  // Intersection at (3, -2)
  float coef1[3]{ 1, 0, -3 };
  float coef2[3]{ 0, 1, 2 };
  float res[2];

  ASSERT_TRUE(v3::SolveLines(coef1, coef2, res));
  EXPECT_NEAR(res[0], 3.0f, math::kFloatEps);
  EXPECT_NEAR(res[1], -2.0f, math::kFloatEps);
}

TEST_F(V3TestMath, SolveLinesParallel) {
  // y = 2x + 1  =>  2x - y + 1 = 0
  // y = 2x + 3  =>  2x - y + 3 = 0
  float coef1[3]{ 2, -1, 1 };
  float coef2[3]{ 2, -1, 3 };
  float res[2];

  ASSERT_FALSE(v3::SolveLines(coef1, coef2, res));
  EXPECT_TRUE(std::isnan(res[0]));
  EXPECT_TRUE(std::isnan(res[1]));
}

TEST_F(V3TestMath, SolveLinesCoincident) {
  // Same line: x + y - 1 = 0
  float coef1[3]{ 1, 1, -1 };
  float coef2[3]{ 2, 2, -2 };  // Scaled version
  float res[2];

  ASSERT_FALSE(v3::SolveLines(coef1, coef2, res));
}

// ============================================================================
// IsInPolygon2 tests
// ============================================================================

TEST_F(V3TestMath, IsInPolygon2TriangleInside) {
  // Triangle with vertices (0,0), (2,0), (0,2)
  // Half-planes: y >= 0 => -y <= 0, x >= 0 => -x <= 0, x+y <= 2 => x+y-2 <= 0
  float coef[9]{
    0,  -1, 0,   // -y <= 0
    -1, 0,  0,   // -x <= 0
    1,  1,  -2,  // x + y - 2 <= 0
  };

  float inside[2]{ 0.5f, 0.5f };
  EXPECT_TRUE(v3::IsInPolygon2(3, coef, inside));
}

TEST_F(V3TestMath, IsInPolygon2TriangleOutside) {
  float coef[9]{
    0, -1, 0, -1, 0, 0, 1, 1, -2,
  };

  float outside[2]{ 1.5f, 1.5f };  // x + y = 3 > 2
  EXPECT_FALSE(v3::IsInPolygon2(3, coef, outside));
}

TEST_F(V3TestMath, IsInPolygon2TriangleBoundary) {
  float coef[9]{
    0, -1, 0, -1, 0, 0, 1, 1, -2,
  };

  float on_edge[2]{ 1.0f, 1.0f };                           // x + y = 2, exactly on boundary
  EXPECT_TRUE(v3::IsInPolygon2(3, coef, on_edge, true));    // boundary included
  EXPECT_FALSE(v3::IsInPolygon2(3, coef, on_edge, false));  // boundary excluded
}

TEST_F(V3TestMath, IsInPolygon2Square) {
  // Unit square [0,1] x [0,1]
  // x >= 0, x <= 1, y >= 0, y <= 1
  float coef[12]{
    -1, 0,  0,   // -x <= 0
    1,  0,  -1,  // x - 1 <= 0
    0,  -1, 0,   // -y <= 0
    0,  1,  -1,  // y - 1 <= 0
  };

  float center[2]{ 0.5f, 0.5f };
  EXPECT_TRUE(v3::IsInPolygon2(4, coef, center));

  float corner[2]{ 0.0f, 0.0f };
  EXPECT_TRUE(v3::IsInPolygon2(4, coef, corner, true));

  float outside[2]{ 1.5f, 0.5f };
  EXPECT_FALSE(v3::IsInPolygon2(4, coef, outside));
}

// ============================================================================
// IsInPolyhedron3 tests
// ============================================================================

TEST_F(V3TestMath, IsInPolyhedron3CubeInside) {
  // Unit cube [0,1]^3, defined by 6 half-spaces
  float coef[24]{
    -1, 0,  0,  0,   // -x <= 0
    1,  0,  0,  -1,  // x - 1 <= 0
    0,  -1, 0,  0,   // -y <= 0
    0,  1,  0,  -1,  // y - 1 <= 0
    0,  0,  -1, 0,   // -z <= 0
    0,  0,  1,  -1,  // z - 1 <= 0
  };

  float center[3]{ 0.5f, 0.5f, 0.5f };
  EXPECT_TRUE(v3::IsInPolyhedron3(6, coef, center));
}

TEST_F(V3TestMath, IsInPolyhedron3CubeOutside) {
  float coef[24]{
    -1, 0, 0, 0, 1, 0, 0, -1, 0, -1, 0, 0, 0, 1, 0, -1, 0, 0, -1, 0, 0, 0, 1, -1,
  };

  float outside[3]{ 1.5f, 0.5f, 0.5f };
  EXPECT_FALSE(v3::IsInPolyhedron3(6, coef, outside));
}

TEST_F(V3TestMath, IsInPolyhedron3CubeBoundary) {
  float coef[24]{
    -1, 0, 0, 0, 1, 0, 0, -1, 0, -1, 0, 0, 0, 1, 0, -1, 0, 0, -1, 0, 0, 0, 1, -1,
  };

  float on_face[3]{ 1.0f, 0.5f, 0.5f };  // On x=1 face
  EXPECT_TRUE(v3::IsInPolyhedron3(6, coef, on_face, true));
  EXPECT_FALSE(v3::IsInPolyhedron3(6, coef, on_face, false));

  float on_edge[3]{ 0.0f, 0.0f, 0.5f };  // On edge x=0,y=0
  EXPECT_TRUE(v3::IsInPolyhedron3(6, coef, on_edge, true));
  EXPECT_FALSE(v3::IsInPolyhedron3(6, coef, on_edge, false));
}

TEST_F(V3TestMath, IsInPolyhedron3Tetrahedron) {
  // Regular tetrahedron centered at origin (approximately)
  // Vertices: (1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1)
  // Half-spaces: inward-pointing normals
  float coef[16]{
    1,  1,  -1, -1,  // x + y - z - 1 <= 0
    1,  -1, 1,  -1,  // x - y + z - 1 <= 0
    -1, 1,  1,  -1,  // -x + y + z - 1 <= 0
    -1, -1, -1, -1,  // -x - y - z - 1 <= 0
  };

  float origin[3]{ 0.0f, 0.0f, 0.0f };
  EXPECT_TRUE(v3::IsInPolyhedron3(4, coef, origin));

  float far_out[3]{ 5.0f, 5.0f, 5.0f };
  EXPECT_FALSE(v3::IsInPolyhedron3(4, coef, far_out));
}

// ============================================================================
// Rotation tests
// ============================================================================

class V3TestRotation : public ::testing::Test {
 protected:
  static constexpr float kEps = 1e-5f;
};

TEST_F(V3TestRotation, DefaultIdentity) {
  v3::Rotation rot;
  float pt[3]{ 1.0f, 2.0f, 3.0f };
  float orig[3];
  std::memcpy(orig, pt, sizeof(pt));

  rot.Apply(pt);

  EXPECT_NEAR(pt[0], orig[0], kEps);
  EXPECT_NEAR(pt[1], orig[1], kEps);
  EXPECT_NEAR(pt[2], orig[2], kEps);
}

TEST_F(V3TestRotation, RotateAroundZ90) {
  // Rotate 90 degrees around Z axis: (1,0,0) -> (0,1,0)
  float z_axis[3]{ 0, 0, 1 };
  v3::Rotation rot(z_axis, math::kPi_2);

  float pt[3]{ 1.0f, 0.0f, 0.0f };
  rot.Apply(pt);

  EXPECT_NEAR(pt[0], 0.0f, kEps);
  EXPECT_NEAR(pt[1], 1.0f, kEps);
  EXPECT_NEAR(pt[2], 0.0f, kEps);
}

TEST_F(V3TestRotation, RotateAroundX180) {
  // Rotate 180 degrees around X axis: (0,1,0) -> (0,-1,0)
  float x_axis[3]{ 1, 0, 0 };
  v3::Rotation rot(x_axis, math::kPi);

  float pt[3]{ 0.0f, 1.0f, 0.0f };
  rot.Apply(pt);

  EXPECT_NEAR(pt[0], 0.0f, kEps);
  EXPECT_NEAR(pt[1], -1.0f, kEps);
  EXPECT_NEAR(pt[2], 0.0f, kEps);
}

TEST_F(V3TestRotation, InverseRestoresOriginal) {
  float y_axis[3]{ 0, 1, 0 };
  v3::Rotation rot(y_axis, 0.7f);

  float pt[3]{ 1.0f, 2.0f, 3.0f };
  float orig[3];
  std::memcpy(orig, pt, sizeof(pt));

  rot.Apply(pt);
  // After rotation, pt should be different
  EXPECT_GT(std::abs(pt[0] - orig[0]) + std::abs(pt[2] - orig[2]), kEps);

  // Apply inverse to restore
  rot.Inverse();
  rot.Apply(pt);

  EXPECT_NEAR(pt[0], orig[0], kEps);
  EXPECT_NEAR(pt[1], orig[1], kEps);
  EXPECT_NEAR(pt[2], orig[2], kEps);
}

TEST_F(V3TestRotation, ApplyInverseEquivalent) {
  float z_axis[3]{ 0, 0, 1 };
  v3::Rotation rot(z_axis, 1.2f);

  float pt1[3]{ 3.0f, 4.0f, 5.0f };
  float pt2[3]{ 3.0f, 4.0f, 5.0f };

  // Method 1: Apply, then Inverse + Apply
  rot.Apply(pt1);
  rot.Inverse();
  rot.Apply(pt1);

  // Method 2: Apply, then ApplyInverse (on a fresh rotation)
  v3::Rotation rot2(z_axis, 1.2f);
  rot2.Apply(pt2);
  rot2.ApplyInverse(pt2);

  EXPECT_NEAR(pt1[0], pt2[0], kEps);
  EXPECT_NEAR(pt1[1], pt2[1], kEps);
  EXPECT_NEAR(pt1[2], pt2[2], kEps);
}

TEST_F(V3TestRotation, ChainTwoRotations) {
  // Two 90-degree rotations around Z = one 180-degree rotation
  float z_axis[3]{ 0, 0, 1 };
  v3::Rotation rot(z_axis, math::kPi_2);
  rot.Chain(z_axis, math::kPi_2);

  float pt[3]{ 1.0f, 0.0f, 0.0f };
  rot.Apply(pt);

  // (1,0,0) rotated 180 around Z -> (-1,0,0)
  EXPECT_NEAR(pt[0], -1.0f, kEps);
  EXPECT_NEAR(pt[1], 0.0f, kEps);
  EXPECT_NEAR(pt[2], 0.0f, kEps);
}

TEST_F(V3TestRotation, MultiplePoints) {
  float z_axis[3]{ 0, 0, 1 };
  v3::Rotation rot(z_axis, math::kPi_2);

  float pts[6]{ 1, 0, 0, 0, 1, 0 };  // Two points
  rot.Apply(pts, 2);

  // (1,0,0) -> (0,1,0)
  EXPECT_NEAR(pts[0], 0.0f, kEps);
  EXPECT_NEAR(pts[1], 1.0f, kEps);
  EXPECT_NEAR(pts[2], 0.0f, kEps);

  // (0,1,0) -> (-1,0,0)
  EXPECT_NEAR(pts[3], -1.0f, kEps);
  EXPECT_NEAR(pts[4], 0.0f, kEps);
  EXPECT_NEAR(pts[5], 0.0f, kEps);
}

}  // namespace
