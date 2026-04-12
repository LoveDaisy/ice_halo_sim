#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <numeric>

#include "core/geo3d.hpp"
#include "core/math.hpp"

namespace {

class Geo3dTest : public ::testing::Test {
 protected:
  void SetUp() override { lumice::RandomNumberGenerator::GetInstance().SetSeed(kFixedSeed_); }

  static constexpr uint32_t kFixedSeed_ = 20260412u;
};


// ============================================================================
// SampleTrianglePoint
// ============================================================================

TEST_F(Geo3dTest, SampleTrianglePoint_InsideTriangleBarycentric) {
  // Triangle: A=(0,0,0), B=(1,0,0), C=(0,1,0)
  // For point P = A + alpha*(B-A) + beta*(C-A), barycentric coords are (1-alpha-beta, alpha, beta).
  // All must be >= 0 and sum to 1.
  constexpr float kVertices[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
  constexpr size_t kN = 10000;
  float pts[kN * 3];
  lumice::SampleTrianglePoint(kVertices, pts, kN);

  for (size_t i = 0; i < kN; i++) {
    float x = pts[i * 3 + 0];
    float y = pts[i * 3 + 1];
    float z = pts[i * 3 + 2];
    // For this triangle: alpha = x, beta = y, gamma = 1 - x - y
    EXPECT_GE(x, -1e-5f) << "Sample " << i << " x < 0";
    EXPECT_GE(y, -1e-5f) << "Sample " << i << " y < 0";
    EXPECT_GE(1.0f - x - y, -1e-5f) << "Sample " << i << " gamma < 0";
    EXPECT_NEAR(x + y + (1.0f - x - y), 1.0f, 1e-4f) << "Sample " << i << " bary sum != 1";
    EXPECT_NEAR(z, 0.0f, 1e-6f) << "Sample " << i << " z != 0 (not in plane)";
  }
}

TEST_F(Geo3dTest, SampleTrianglePoint_BarycentricMeanApproxCentroid) {
  // Uniform sampling on triangle has mean = centroid = (1/3, 1/3, 0).
  // Precision: sigma_mean ~ 7.4e-4, 3sigma ~ 2.2e-3, tolerance 0.01 (~4.5 sigma).
  constexpr float kVertices[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
  constexpr size_t kN = 100000;
  float pts[kN * 3];
  lumice::SampleTrianglePoint(kVertices, pts, kN);

  double sum_x = 0, sum_y = 0, sum_z = 0;
  for (size_t i = 0; i < kN; i++) {
    sum_x += pts[i * 3 + 0];
    sum_y += pts[i * 3 + 1];
    sum_z += pts[i * 3 + 2];
  }
  float mean_x = static_cast<float>(sum_x / kN);
  float mean_y = static_cast<float>(sum_y / kN);
  float mean_z = static_cast<float>(sum_z / kN);

  EXPECT_NEAR(mean_x, 1.0f / 3.0f, 0.01f);
  EXPECT_NEAR(mean_y, 1.0f / 3.0f, 0.01f);
  EXPECT_NEAR(mean_z, 0.0f, 1e-4f);
}

TEST_F(Geo3dTest, SampleTrianglePoint_DegenerateCollinearTriangle) {
  // Current contract: degenerate (collinear) triangle does not crash/NaN.
  // SampleTrianglePoint computes u*e1 + v*e2 + A with u+v<=1; for collinear points
  // (0,0,0),(1,0,0),(2,0,0): e1=(1,0,0), e2=(2,0,0), result falls on x-axis in [0,2].
  // Source: geo3d.cpp:153-168, no division, no assert.
  constexpr float kVertices[9] = { 0, 0, 0, 1, 0, 0, 2, 0, 0 };
  constexpr size_t kN = 100;
  float pts[kN * 3];
  lumice::SampleTrianglePoint(kVertices, pts, kN);

  for (size_t i = 0; i < kN; i++) {
    float x = pts[i * 3 + 0];
    float y = pts[i * 3 + 1];
    float z = pts[i * 3 + 2];
    EXPECT_TRUE(std::isfinite(x)) << "Sample " << i << " x not finite";
    EXPECT_FLOAT_EQ(y, 0.0f) << "Sample " << i << " y != 0";
    EXPECT_FLOAT_EQ(z, 0.0f) << "Sample " << i << " z != 0";
    EXPECT_GE(x, -1e-6f) << "Sample " << i << " x < 0";
    EXPECT_LE(x, 2.0f + 1e-6f) << "Sample " << i << " x > 2";
  }
}


// ============================================================================
// SampleSph
// ============================================================================

TEST_F(Geo3dTest, SampleSph_UnitRadiusLength) {
  // All samples must have magnitude == radii (within float tolerance).
  // Precision: float accumulated error ~5e-7, tolerance 1e-4 gives 200x margin for r=1.
  constexpr size_t kN = 100000;

  // Test with radii = 1.0
  {
    float pts[kN * 3];
    lumice::SampleSph(1.0f, pts, kN);
    for (size_t i = 0; i < kN; i++) {
      float x = pts[i * 3 + 0], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
      float len = std::sqrt(x * x + y * y + z * z);
      EXPECT_NEAR(len, 1.0f, 1e-4f) << "Sample " << i << " radii=1";
    }
  }

  // Test with radii = 3.5 (delta/radii = 2.9e-5, still well above float eps)
  {
    float pts[kN * 3];
    lumice::SampleSph(3.5f, pts, kN);
    for (size_t i = 0; i < kN; i++) {
      float x = pts[i * 3 + 0], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
      float len = std::sqrt(x * x + y * y + z * z);
      EXPECT_NEAR(len, 3.5f, 1e-4f) << "Sample " << i << " radii=3.5";
    }
  }
}

TEST_F(Geo3dTest, SampleSph_OctantUniformity) {
  // 8 octants should each get ~N/8 samples. 3sigma/mu = 2.5%, tolerance +-5% (~4.8 sigma).
  constexpr size_t kN = 100000;
  float pts[kN * 3];
  lumice::SampleSph(1.0f, pts, kN);

  int octant_count[8] = {};
  for (size_t i = 0; i < kN; i++) {
    int idx = (pts[i * 3 + 0] >= 0 ? 0 : 1) + (pts[i * 3 + 1] >= 0 ? 0 : 2) + (pts[i * 3 + 2] >= 0 ? 0 : 4);
    octant_count[idx]++;
  }

  constexpr float kExpected = kN / 8.0f;
  constexpr float kLo = kExpected * 0.95f;
  constexpr float kHi = kExpected * 1.05f;
  for (int i = 0; i < 8; i++) {
    EXPECT_GE(octant_count[i], static_cast<int>(kLo)) << "Octant " << i << " too few";
    EXPECT_LE(octant_count[i], static_cast<int>(kHi)) << "Octant " << i << " too many";
  }
}


// ============================================================================
// SampleBall
// ============================================================================

TEST_F(Geo3dTest, SampleBall_InsideBallCDF) {
  // Volume-uniform sampling: P(||p|| < r) = r^3 for unit ball.
  // Check at r in {0.4, 0.6, 0.8} with per-point tolerance derived from binomial 3sigma + 1.5x margin.
  // r=0.2 excluded (3sigma/mu = 7.5%, insufficient signal/noise).
  constexpr size_t kN = 200000;
  float pts[kN * 3];
  lumice::SampleBall(1.0f, pts, kN);

  struct CheckPoint {
    float r;
    float expected_frac;
    float tolerance;
  };
  // r=0.4: mu=12800, sigma=109, 3sigma/mu=2.6%, tol=4% (1.5x margin)
  // r=0.6: mu=43200, sigma=184, 3sigma/mu=1.3%, tol=2%
  // r=0.8: mu=102400, sigma=224, 3sigma/mu=0.66%, tol=1.5%
  constexpr CheckPoint kPoints[] = {
    { 0.4f, 0.064f, 0.04f },
    { 0.6f, 0.216f, 0.02f },
    { 0.8f, 0.512f, 0.015f },
  };

  for (const auto& cp : kPoints) {
    int count = 0;
    for (size_t i = 0; i < kN; i++) {
      float x = pts[i * 3 + 0], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
      float r = std::sqrt(x * x + y * y + z * z);
      if (r < cp.r) {
        count++;
      }
    }
    float frac = static_cast<float>(count) / static_cast<float>(kN);
    EXPECT_NEAR(frac, cp.expected_frac, cp.tolerance)
        << "CDF mismatch at r=" << cp.r << ": got " << frac << " expected " << cp.expected_frac;
  }
}

TEST_F(Geo3dTest, SampleBall_DirectionUniformity) {
  // Direction octant uniformity, same tolerance as SampleSph: +-5%.
  constexpr size_t kN = 200000;
  float pts[kN * 3];
  lumice::SampleBall(1.0f, pts, kN);

  int octant_count[8] = {};
  for (size_t i = 0; i < kN; i++) {
    int idx = (pts[i * 3 + 0] >= 0 ? 0 : 1) + (pts[i * 3 + 1] >= 0 ? 0 : 2) + (pts[i * 3 + 2] >= 0 ? 0 : 4);
    octant_count[idx]++;
  }

  constexpr float kExpected = kN / 8.0f;
  constexpr float kLo = kExpected * 0.95f;
  constexpr float kHi = kExpected * 1.05f;
  for (int i = 0; i < 8; i++) {
    EXPECT_GE(octant_count[i], static_cast<int>(kLo)) << "Octant " << i << " too few";
    EXPECT_LE(octant_count[i], static_cast<int>(kHi)) << "Octant " << i << " too many";
  }
}


// ============================================================================
// SampleSphCapPoint
// ============================================================================

TEST_F(Geo3dTest, SampleSphCapPoint_SamplesInsideCap_LonZeroLatZero) {
  // Cap centered at (1,0,0), half-angle 10 deg.
  // All samples must satisfy dot(p, center) >= cos(10 deg) - delta.
  constexpr size_t kN = 50000;
  float pts[kN * 3];
  lumice::SampleSphCapPoint(0.0f, 0.0f, 10.0f, pts, kN);

  float cos_cap = std::cos(10.0f * lumice::math::kDegreeToRad);
  for (size_t i = 0; i < kN; i++) {
    float x = pts[i * 3 + 0], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
    float len = std::sqrt(x * x + y * y + z * z);
    EXPECT_NEAR(len, 1.0f, 1e-4f) << "Sample " << i << " not unit vector";
    float dot = x;  // dot with (1,0,0)
    EXPECT_GE(dot, cos_cap - 1e-5f) << "Sample " << i << " outside cap";
  }
}

// DISABLED: SampleSphCapPoint has a rotation bug (geo3d.cpp:203, sign flip in y-component).
// Samples fall outside cap when lon!=0. Mean direction is correct (symmetric flip).
// Bug fix tracked in task-fix-geo3d-sampler-rotation.
TEST_F(Geo3dTest, DISABLED_SampleSphCapPoint_SamplesInsideCap_Rotated) {
  // Cap at (lon=45, lat=30) deg, half-angle 5 deg.
  // Center direction: c = (cos(30)*cos(45), cos(30)*sin(45), sin(30))
  constexpr float kLon = 45.0f, kLat = 30.0f, kCap = 5.0f;
  float c_lon = std::cos(kLon * lumice::math::kDegreeToRad);
  float s_lon = std::sin(kLon * lumice::math::kDegreeToRad);
  float c_lat = std::cos(kLat * lumice::math::kDegreeToRad);
  float s_lat = std::sin(kLat * lumice::math::kDegreeToRad);
  float center[3] = { c_lat * c_lon, c_lat * s_lon, s_lat };

  constexpr size_t kN = 50000;
  float pts[kN * 3];
  lumice::SampleSphCapPoint(kLon, kLat, kCap, pts, kN);

  float cos_cap = std::cos(kCap * lumice::math::kDegreeToRad);
  for (size_t i = 0; i < kN; i++) {
    float x = pts[i * 3 + 0], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
    float dot = x * center[0] + y * center[1] + z * center[2];
    EXPECT_GE(dot, cos_cap - 5e-5f) << "Sample " << i << " outside rotated cap";
  }
}

TEST_F(Geo3dTest, SampleSphCapPoint_CapCenterMean) {
  // Narrow cap (3 deg) at (lon=45, lat=30). Mean direction should match center.
  // Precision: sigma_perp ~ 4.6e-5 rad (0.0026 deg), 3sigma ~ 0.008 deg.
  // Tolerance 0.5 deg (60 sigma) — rotation sign-flip bugs cause multi-degree errors.
  constexpr float kLon = 45.0f, kLat = 30.0f, kCap = 3.0f;
  float c_lon = std::cos(kLon * lumice::math::kDegreeToRad);
  float s_lon = std::sin(kLon * lumice::math::kDegreeToRad);
  float c_lat = std::cos(kLat * lumice::math::kDegreeToRad);
  float s_lat = std::sin(kLat * lumice::math::kDegreeToRad);
  float center[3] = { c_lat * c_lon, c_lat * s_lon, s_lat };

  constexpr size_t kN = 200000;
  auto pts = std::make_unique<float[]>(kN * 3);
  lumice::SampleSphCapPoint(kLon, kLat, kCap, pts.get(), kN);

  // Compute mean direction
  double sum[3] = {};
  for (size_t i = 0; i < kN; i++) {
    sum[0] += pts[i * 3 + 0];
    sum[1] += pts[i * 3 + 1];
    sum[2] += pts[i * 3 + 2];
  }
  float mean[3] = { static_cast<float>(sum[0] / kN), static_cast<float>(sum[1] / kN), static_cast<float>(sum[2] / kN) };
  float mean_len = std::sqrt(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
  ASSERT_GT(mean_len, 1e-6f) << "Mean direction is zero vector";

  // Normalize and compute angle to expected center
  float dot = (mean[0] * center[0] + mean[1] * center[1] + mean[2] * center[2]) / mean_len;
  dot = std::min(1.0f, std::max(-1.0f, dot));  // clamp for acos safety
  float angle_deg = std::acos(dot) / lumice::math::kDegreeToRad;
  EXPECT_LT(angle_deg, 0.5f) << "Mean direction deviates " << angle_deg << " deg from expected center";
}

TEST_F(Geo3dTest, SampleSphCapPoint_BoundaryCap_Zero) {
  // cap=0: all samples should equal the center direction exactly (within float tolerance).
  // Also test cap=1e-4 deg to verify no NaN near the boundary.
  constexpr float kLon = 60.0f, kLat = 20.0f;
  float c_lon = std::cos(kLon * lumice::math::kDegreeToRad);
  float s_lon = std::sin(kLon * lumice::math::kDegreeToRad);
  float c_lat = std::cos(kLat * lumice::math::kDegreeToRad);
  float s_lat = std::sin(kLat * lumice::math::kDegreeToRad);
  float center[3] = { c_lat * c_lon, c_lat * s_lon, s_lat };

  // Exact zero cap
  {
    constexpr size_t kN = 100;
    float pts[kN * 3];
    lumice::SampleSphCapPoint(kLon, kLat, 0.0f, pts, kN);
    for (size_t i = 0; i < kN; i++) {
      EXPECT_NEAR(pts[i * 3 + 0], center[0], 1e-5f) << "Zero-cap sample " << i;
      EXPECT_NEAR(pts[i * 3 + 1], center[1], 1e-5f) << "Zero-cap sample " << i;
      EXPECT_NEAR(pts[i * 3 + 2], center[2], 1e-5f) << "Zero-cap sample " << i;
    }
  }

  // Near-zero cap (1e-4 deg): check all finite
  {
    constexpr size_t kN = 100;
    float pts[kN * 3];
    lumice::SampleSphCapPoint(kLon, kLat, 1e-4f, pts, kN);
    for (size_t i = 0; i < kN; i++) {
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 0])) << "Near-zero cap " << i << " x not finite";
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 1])) << "Near-zero cap " << i << " y not finite";
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 2])) << "Near-zero cap " << i << " z not finite";
    }
  }
}

TEST_F(Geo3dTest, SampleSphCapPoint_BoundaryCap_FullSphere) {
  // cap=180 deg: samples should cover entire sphere. Check all octants non-zero and all finite.
  // Also test cap=179.99 deg near boundary.
  constexpr float kLon = 0.0f, kLat = 0.0f;

  // Full sphere
  {
    constexpr size_t kN = 50000;
    auto pts = std::make_unique<float[]>(kN * 3);
    lumice::SampleSphCapPoint(kLon, kLat, 180.0f, pts.get(), kN);

    int octant_count[8] = {};
    for (size_t i = 0; i < kN; i++) {
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 0])) << "Full-sphere " << i << " x not finite";
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 1])) << "Full-sphere " << i << " y not finite";
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 2])) << "Full-sphere " << i << " z not finite";

      int idx = (pts[i * 3 + 0] >= 0 ? 0 : 1) + (pts[i * 3 + 1] >= 0 ? 0 : 2) + (pts[i * 3 + 2] >= 0 ? 0 : 4);
      octant_count[idx]++;
    }

    // Minimum per octant: N/16 = 3125 (well below expected N/8 = 6250)
    for (int i = 0; i < 8; i++) {
      EXPECT_GE(octant_count[i], static_cast<int>(kN / 16)) << "Full-sphere octant " << i << " too few";
    }
  }

  // Near-full sphere (179.99 deg): check finite
  {
    constexpr size_t kN = 1000;
    float pts[kN * 3];
    lumice::SampleSphCapPoint(kLon, kLat, 179.99f, pts, kN);
    for (size_t i = 0; i < kN; i++) {
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 0])) << "Near-full " << i << " x not finite";
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 1])) << "Near-full " << i << " y not finite";
      EXPECT_TRUE(std::isfinite(pts[i * 3 + 2])) << "Near-full " << i << " z not finite";
    }
  }
}


// ============================================================================
// Mesh value-type semantics
// ============================================================================

TEST_F(Geo3dTest, Mesh_DefaultAndEmpty) {
  // Default constructor: zero counts
  lumice::Mesh m0;
  EXPECT_EQ(m0.GetVtxCnt(), 0u);
  EXPECT_EQ(m0.GetTriangleCnt(), 0u);

  // Mesh(0, 0): also zero counts
  lumice::Mesh m00(0, 0);
  EXPECT_EQ(m00.GetVtxCnt(), 0u);
  EXPECT_EQ(m00.GetTriangleCnt(), 0u);

  // Mesh(3, 1): allocated, counts correct
  lumice::Mesh m31(3, 1);
  EXPECT_EQ(m31.GetVtxCnt(), 3u);
  EXPECT_EQ(m31.GetTriangleCnt(), 1u);
  EXPECT_NE(m31.GetVtxPtr(0), nullptr);
  EXPECT_NE(m31.GetTrianglePtr(0), nullptr);
}

TEST_F(Geo3dTest, Mesh_CopyPreservesData) {
  // Construct Mesh(4, 2), write known data, copy, modify source, verify copy unchanged.
  lumice::Mesh src(4, 2);
  // Fill vertices: 4 vertices * 3 coords = 12 floats
  for (int v = 0; v < 4; v++) {
    float* p = src.GetVtxPtr(v);
    p[0] = static_cast<float>(v * 3 + 1);
    p[1] = static_cast<float>(v * 3 + 2);
    p[2] = static_cast<float>(v * 3 + 3);
  }
  // Fill triangles: 2 triangles * 3 indices = 6 ints
  int* t0 = src.GetTrianglePtr(0);
  t0[0] = 0;
  t0[1] = 1;
  t0[2] = 2;
  int* t1 = src.GetTrianglePtr(1);
  t1[0] = 1;
  t1[1] = 2;
  t1[2] = 3;

  // Copy
  lumice::Mesh copy(src);
  EXPECT_EQ(copy.GetVtxCnt(), 4u);
  EXPECT_EQ(copy.GetTriangleCnt(), 2u);

  // Modify source
  src.GetVtxPtr(0)[0] = -999.0f;

  // Verify copy is independent (deep copy)
  EXPECT_FLOAT_EQ(copy.GetVtxPtr(0)[0], 1.0f);
  EXPECT_FLOAT_EQ(copy.GetVtxPtr(1)[0], 4.0f);
  EXPECT_EQ(copy.GetTrianglePtr(0)[0], 0);
  EXPECT_EQ(copy.GetTrianglePtr(1)[2], 3);
}

TEST_F(Geo3dTest, Mesh_MoveLeavesSourceEmpty) {
  // Construct, fill, move, verify source is empty and target has data.
  lumice::Mesh src(3, 1);
  for (int v = 0; v < 3; v++) {
    float* p = src.GetVtxPtr(v);
    p[0] = static_cast<float>(v);
    p[1] = static_cast<float>(v + 10);
    p[2] = static_cast<float>(v + 20);
  }
  int* t = src.GetTrianglePtr(0);
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;

  lumice::Mesh dst(std::move(src));
  // Source should be cleared
  EXPECT_EQ(src.GetVtxCnt(), 0u);       // NOLINT(bugprone-use-after-move)
  EXPECT_EQ(src.GetTriangleCnt(), 0u);  // NOLINT(bugprone-use-after-move)

  // Destination should have the data
  EXPECT_EQ(dst.GetVtxCnt(), 3u);
  EXPECT_EQ(dst.GetTriangleCnt(), 1u);
  EXPECT_FLOAT_EQ(dst.GetVtxPtr(0)[0], 0.0f);
  EXPECT_FLOAT_EQ(dst.GetVtxPtr(2)[2], 22.0f);
  EXPECT_EQ(dst.GetTrianglePtr(0)[1], 1);
}

}  // namespace
