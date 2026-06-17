#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "core/buffer.hpp"
#include "core/crystal.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"

using namespace lumice;

// GetReflectRatio has external linkage (not static, not in anonymous namespace)
namespace lumice {
float GetReflectRatio(float delta, float rr);
}  // namespace lumice


// ============================================================================
// IceRefractiveIndex::Get() tests
// Reference values computed from code's Sellmeier formula with kCoefAvr coefficients.
// Cross-checked against refractiveindex.info (ice Ih, ordinary ray).
// ============================================================================

TEST(IceRefractiveIndexTest, KnownWavelength589nm) {
  // 589nm (sodium D-line): n ≈ 1.3097 (refractiveindex.info: ~1.3098)
  double n = IceRefractiveIndex::Get(589.0);
  EXPECT_NEAR(n, 1.3097193039, 1e-6);
}

TEST(IceRefractiveIndexTest, KnownWavelength550nm) {
  // 550nm (green): n ≈ 1.3110
  double n = IceRefractiveIndex::Get(550.0);
  EXPECT_NEAR(n, 1.3110129171, 1e-6);
}

TEST(IceRefractiveIndexTest, BoundaryMin350nm) {
  double n = IceRefractiveIndex::Get(350.0);
  EXPECT_NEAR(n, 1.3246528270, 1e-6);
  EXPECT_GT(n, 1.0);
}

TEST(IceRefractiveIndexTest, BoundaryMax900nm) {
  double n = IceRefractiveIndex::Get(900.0);
  EXPECT_NEAR(n, 1.3031960565, 1e-6);
  EXPECT_GT(n, 1.0);
}

TEST(IceRefractiveIndexTest, OutOfRangeReturns1) {
  EXPECT_DOUBLE_EQ(IceRefractiveIndex::Get(349.0), 1.0);
  EXPECT_DOUBLE_EQ(IceRefractiveIndex::Get(901.0), 1.0);
  EXPECT_DOUBLE_EQ(IceRefractiveIndex::Get(0.0), 1.0);
  EXPECT_DOUBLE_EQ(IceRefractiveIndex::Get(-100.0), 1.0);
}


// ============================================================================
// GetReflectRatio() tests — Fresnel reflection coefficient
// ============================================================================

TEST(GetReflectRatioTest, NormalIncidence) {
  // At normal incidence: delta = 1, d_sqrt = 1
  // Rs = ((rr - 1)/(rr + 1))^2, Rp = ((1 - rr)/(1 + rr))^2 = Rs
  // Result = Rs (since Rs == Rp)
  float n = 1.3097f;
  float r = GetReflectRatio(1.0f, n);  // NOLINT(readability-identifier-naming) Fresnel notation
  float expected = ((n - 1.0f) / (n + 1.0f)) * ((n - 1.0f) / (n + 1.0f));
  EXPECT_NEAR(r, expected, 1e-5f);
  EXPECT_NEAR(r, 0.01798f, 1e-4f);
}

TEST(GetReflectRatioTest, KnownValues) {
  // delta = 0.5, rr = 1.5 -> d_sqrt = sqrt(0.5)
  // Rs = (1.5 - 0.7071)/(1.5 + 0.7071) = 0.7929/2.2071 = 0.35929, Rs^2 = 0.12909
  // Rp = (1 - 1.5*0.7071)/(1 + 1.5*0.7071) = (1 - 1.06066)/(1 + 1.06066) = -0.06066/2.06066
  //    = -0.02944, Rp^2 = 0.000867
  // R = (0.12909 + 0.000867) / 2 = 0.06498
  float d_sqrt = std::sqrt(0.5f);
  float rs = (1.5f - d_sqrt) / (1.5f + d_sqrt);  // NOLINT(readability-identifier-naming) Fresnel notation
  rs *= rs;
  float rp = (1.0f - 1.5f * d_sqrt) / (1.0f + 1.5f * d_sqrt);  // NOLINT(readability-identifier-naming) Fresnel notation
  rp *= rp;
  float expected = (rs + rp) / 2.0f;

  float r = GetReflectRatio(0.5f, 1.5f);  // NOLINT(readability-identifier-naming) Fresnel notation
  EXPECT_NEAR(r, expected, 1e-5f);
}


// ============================================================================
// HitSurface() tests
// ============================================================================

class HitSurfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    crystal_ = Crystal::CreatePrism(1.0f);
    n_ = static_cast<float>(IceRefractiveIndex::Get(589.0));
  }

  // Helper to call HitSurface for a single ray
  struct HitResult {
    float reflect_dir[3];
    float refract_dir[3];
    float reflect_weight;
    float refract_weight;
  };

  HitResult HitSingle(const float* dir, float weight, IdType to_face) {
    float dir_in[3] = { dir[0], dir[1], dir[2] };
    float w_in[1] = { weight };
    IdType to_face_in[1] = { to_face };
    float dir_out[6] = {};
    float w_out[2] = {};

    float_bf_t d_in(dir_in, 3 * sizeof(float));
    float_bf_t wt_in(w_in, sizeof(float));
    id_bf_t fi_in(to_face_in, sizeof(IdType));
    float_bf_t d_out(dir_out, 3 * sizeof(float));
    float_bf_t wt_out(w_out, sizeof(float));

    HitSurface(crystal_, n_, 1, d_in, wt_in, fi_in, d_out, wt_out);

    HitResult r{};
    std::memcpy(r.reflect_dir, dir_out, 3 * sizeof(float));
    std::memcpy(r.refract_dir, dir_out + 3, 3 * sizeof(float));
    r.reflect_weight = w_out[0];
    r.refract_weight = w_out[1];
    return r;
  }

  Crystal crystal_;
  float n_ = 0;
};

TEST_F(HitSurfaceTest, AirToIceNormalIncidence) {
  // Find a face normal and shoot a ray along -normal (air -> ice, cos_theta < 0)
  // HitSurface looks up the polygon-face normal indexed by RaySeg::to_face_.
  const float* norms = crystal_.GetPolygonFaceNormal();
  IdType fid = 0;
  float norm[3] = { norms[0], norms[1], norms[2] };

  // Ray direction = -normal (shooting into the crystal)
  float dir[3] = { -norm[0], -norm[1], -norm[2] };

  auto result = HitSingle(dir, 1.0f, fid);

  // Reflection should be along +normal
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(result.reflect_dir[j], norm[j], 1e-4f);
  }

  // Refraction at normal incidence: direction barely changes (d_refract ≈ d_in for small n-1)
  // More precisely: refract = rr * d_in - (rr - sqrt(d)) * cos * norm
  // At normal incidence with cos=-1, rr=1/n: refract = (1/n)*(-norm) - (1/n - 1)*(-1)*norm
  //   = -norm/n + (1 - 1/n)*norm = -norm/n + norm - norm/n = norm*(1 - 2/n)
  // Actually just check it's roughly in the same direction as input
  float dot = 0;
  for (int j = 0; j < 3; j++) {
    dot += result.refract_dir[j] * dir[j];
  }
  EXPECT_GT(dot, 0.0f);  // Refracted ray goes in same general direction as incident

  // Fresnel at normal incidence: R = ((n-1)/(n+1))^2 ≈ 0.018
  EXPECT_NEAR(result.reflect_weight, 0.018f, 0.002f);
  EXPECT_GT(result.refract_weight, 0.0f);
}

TEST_F(HitSurfaceTest, SnellsLaw30Degrees) {
  // Construct a ray at 30° to a face normal (air -> ice)
  // HitSurface looks up the polygon-face normal indexed by RaySeg::to_face_.
  const float* norms = crystal_.GetPolygonFaceNormal();
  IdType fid = 0;
  float norm[3] = { norms[0], norms[1], norms[2] };

  // Build a tangent vector perpendicular to norm
  float tangent[3];
  if (std::abs(norm[0]) < 0.9f) {
    float tmp[3] = { 1, 0, 0 };
    // tangent = tmp - (tmp.norm)*norm
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  } else {
    float tmp[3] = { 0, 1, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  }
  float t_len = Norm3(tangent);
  for (float& v : tangent) {
    v /= t_len;
  }

  // Ray direction: 30° from -normal toward tangent (air -> ice, cos_theta < 0)
  float theta_i = 30.0f * math::kDegreeToRad;
  float dir[3];
  for (int j = 0; j < 3; j++) {
    dir[j] = -std::cos(theta_i) * norm[j] + std::sin(theta_i) * tangent[j];
  }

  auto result = HitSingle(dir, 1.0f, fid);

  // Verify Snell's law: sin(theta_r) = sin(theta_i) / n
  // Compute refracted angle from dot product with -normal
  float cos_refract = 0;
  for (int j = 0; j < 3; j++) {
    cos_refract += result.refract_dir[j] * (-norm[j]);
  }
  cos_refract = std::abs(cos_refract);
  float sin_refract = std::sqrt(1.0f - cos_refract * cos_refract);

  float expected_sin_refract = std::sin(theta_i) / n_;
  EXPECT_NEAR(sin_refract, expected_sin_refract, 1e-4f);
}

TEST_F(HitSurfaceTest, TotalInternalReflection) {
  // Ice -> air (cos_theta > 0), angle > critical angle (~49.8°)
  // HitSurface looks up the polygon-face normal indexed by RaySeg::to_face_.
  const float* norms = crystal_.GetPolygonFaceNormal();
  IdType fid = 0;
  float norm[3] = { norms[0], norms[1], norms[2] };

  // Build tangent
  float tangent[3];
  if (std::abs(norm[0]) < 0.9f) {
    float tmp[3] = { 1, 0, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  } else {
    float tmp[3] = { 0, 1, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  }
  float t_len = Norm3(tangent);
  for (float& v : tangent) {
    v /= t_len;
  }

  // 60° from +normal (ice -> air, cos_theta > 0), well above critical angle ~49.8°
  float theta = 60.0f * math::kDegreeToRad;
  float dir[3];
  for (int j = 0; j < 3; j++) {
    dir[j] = std::cos(theta) * norm[j] + std::sin(theta) * tangent[j];
  }

  auto result = HitSingle(dir, 1.0f, fid);

  // Total reflection: refract weight should be -1
  EXPECT_FLOAT_EQ(result.refract_weight, -1.0f);
  // Reflection weight should be positive
  EXPECT_GT(result.reflect_weight, 0.0f);
}

TEST_F(HitSurfaceTest, EnergyConservation) {
  // Non-total-reflection case: reflect_weight + refract_weight ≈ input_weight
  // HitSurface looks up the polygon-face normal indexed by RaySeg::to_face_.
  const float* norms = crystal_.GetPolygonFaceNormal();
  IdType fid = 0;
  float norm[3] = { norms[0], norms[1], norms[2] };

  // Air -> ice, 20° incidence
  float tangent[3];
  if (std::abs(norm[0]) < 0.9f) {
    float tmp[3] = { 1, 0, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  } else {
    float tmp[3] = { 0, 1, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  }
  float t_len = Norm3(tangent);
  for (float& v : tangent) {
    v /= t_len;
  }

  float theta = 20.0f * math::kDegreeToRad;
  float dir[3];
  for (int j = 0; j < 3; j++) {
    dir[j] = -std::cos(theta) * norm[j] + std::sin(theta) * tangent[j];
  }

  float input_weight = 1.0f;
  auto result = HitSingle(dir, input_weight, fid);

  EXPECT_GT(result.refract_weight, 0.0f);
  EXPECT_NEAR(result.reflect_weight + result.refract_weight, input_weight, 1e-5f);
}

TEST_F(HitSurfaceTest, DirectionSwitchAirVsIce) {
  // Same face, same angle, but from air side (cos < 0) vs ice side (cos > 0)
  // HitSurface looks up the polygon-face normal indexed by RaySeg::to_face_.
  const float* norms = crystal_.GetPolygonFaceNormal();
  IdType fid = 0;
  float norm[3] = { norms[0], norms[1], norms[2] };

  float tangent[3];
  if (std::abs(norm[0]) < 0.9f) {
    float tmp[3] = { 1, 0, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  } else {
    float tmp[3] = { 0, 1, 0 };
    float d = tmp[0] * norm[0] + tmp[1] * norm[1] + tmp[2] * norm[2];
    tangent[0] = tmp[0] - d * norm[0];
    tangent[1] = tmp[1] - d * norm[1];
    tangent[2] = tmp[2] - d * norm[2];
  }
  float t_len = Norm3(tangent);
  for (float& v : tangent) {
    v /= t_len;
  }

  float theta = 20.0f * math::kDegreeToRad;

  // Air -> ice: cos_theta < 0, rr = 1/n
  float dir_air[3];
  for (int j = 0; j < 3; j++) {
    dir_air[j] = -std::cos(theta) * norm[j] + std::sin(theta) * tangent[j];
  }
  auto result_air = HitSingle(dir_air, 1.0f, fid);

  // Ice -> air: cos_theta > 0, rr = n
  float dir_ice[3];
  for (int j = 0; j < 3; j++) {
    dir_ice[j] = std::cos(theta) * norm[j] + std::sin(theta) * tangent[j];
  }
  auto result_ice = HitSingle(dir_ice, 1.0f, fid);

  // Both should have valid refraction (20° < critical angle)
  EXPECT_GT(result_air.refract_weight, 0.0f);
  EXPECT_GT(result_ice.refract_weight, 0.0f);

  // Fresnel coefficients should differ because rr differs
  EXPECT_NE(result_air.reflect_weight, result_ice.reflect_weight);
}


// ============================================================================
// Propagate() tests
// ============================================================================

class PropagateTest : public ::testing::Test {
 protected:
  void SetUp() override { crystal_ = Crystal::CreatePrism(1.0f); }

  Crystal crystal_;
};

TEST_F(PropagateTest, HorizontalRayHitsSideFace) {
  // Ray from center (0,0,0), horizontal direction (1,0,0)
  // Should hit a side face of the prism
  float dir[3] = { 1.0f, 0.0f, 0.0f };
  float pos[3] = { 0.0f, 0.0f, 0.0f };
  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };
  IdType src_fid[1] = { kInvalidId };

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(src_fid, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // Should hit some polygon face (fid != kInvalidId)
  EXPECT_NE(fid_out[0], kInvalidId);
  EXPECT_LT(fid_out[0], crystal_.PolygonFaceCount());

  // Output position should be different from input (ray traveled)
  float dist = DiffNorm3(pos_out, pos);
  EXPECT_GT(dist, 0.0f);
}

TEST_F(PropagateTest, VerticalRayHitsTopFace) {
  // Ray from center (0,0,0), vertical direction (0,0,1)
  // Should hit a top face
  float dir[3] = { 0.0f, 0.0f, 1.0f };
  float pos[3] = { 0.0f, 0.0f, 0.0f };
  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };
  IdType src_fid[1] = { kInvalidId };

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(src_fid, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  EXPECT_NE(fid_out[0], kInvalidId);

  // Output z should be positive (hit top face)
  EXPECT_GT(pos_out[2], 0.0f);
  // Output x,y should be near zero (vertical ray from center)
  EXPECT_NEAR(pos_out[0], 0.0f, 1e-4f);
  EXPECT_NEAR(pos_out[1], 0.0f, 1e-4f);
}

TEST_F(PropagateTest, NegativeWeightSkipped) {
  // w_in < 0 marks total reflection — Propagate should skip
  float dir[3] = { 1.0f, 0.0f, 0.0f };
  float pos[3] = { 0.0f, 0.0f, 0.0f };
  float w[1] = { -1.0f };
  float pos_out[3] = { -999.0f, -999.0f, -999.0f };
  IdType fid_out[1] = { kInvalidId };
  IdType src_fid[1] = { kInvalidId };

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(src_fid, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // PropagateSlab skips w<0 rays: output position = input position, fid = kInvalidId
  // (The skip preserves the initialized values from the gather phase in PropagateSlab,
  //  but for the original position since w<0 is skipped in the scatter phase)
  EXPECT_EQ(fid_out[0], kInvalidId);
}

TEST_F(PropagateTest, Step2SharedPosition) {
  // 4 directions, 2 positions, step=2
  // Rays 0,1 share pos[0], rays 2,3 share pos[1]
  float dirs[12] = {
    1.0f, 0.0f, 0.0f,  // ray 0
    0.0f, 1.0f, 0.0f,  // ray 1
    1.0f, 0.0f, 0.0f,  // ray 2 (same dir as 0)
    0.0f, 1.0f, 0.0f,  // ray 3 (same dir as 1)
  };
  float positions[6] = {
    0.0f, 0.0f, 0.0f,  // pos 0
    0.0f, 0.0f, 0.0f,  // pos 1 (same position for simplicity)
  };
  float weights[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
  float pos_out[12] = {};
  IdType fid_out[4] = { kInvalidId, kInvalidId, kInvalidId, kInvalidId };
  IdType src_fids[2] = { kInvalidId, kInvalidId };

  float_bf_t d_in(dirs, 3 * sizeof(float));
  float_bf_t p_in(positions, 3 * sizeof(float));
  float_bf_t wt_in(weights, sizeof(float));
  id_bf_t fi_src(src_fids, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 4, 2, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // All rays should hit something (from center, any direction hits a face)
  for (int i = 0; i < 4; i++) {
    EXPECT_NE(fid_out[i], kInvalidId) << "Ray " << i << " missed";
  }

  // Rays with same direction and same position should produce same results
  // Ray 0 (dir +x, pos 0) vs Ray 2 (dir +x, pos 1, which == pos 0)
  EXPECT_EQ(fid_out[0], fid_out[2]);
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(pos_out[j], pos_out[6 + j], 1e-4f);
  }
}

TEST_F(PropagateTest, ChunkBoundaryConsistency) {
  // Verify single-chunk (num=1) and multi-chunk (num=129 = 128+1) Propagate
  // produce identical face IDs and exit positions for the same active ray.
  // Replaces the pre-polygon-only SlabAndTrianglePathConsistency test (the two
  // paths have been merged — only the chunk loop remains to be exercised).
  float dir[3] = { 1.0f, 0.0f, 0.0f };
  float pos[3] = { 0.0f, 0.0f, 0.0f };
  float w[1] = { 1.0f };

  // Single-chunk reference (num=1)
  float pos_out_ref[3] = {};
  IdType fid_ref[1] = { kInvalidId };
  IdType src_fid_ref[1] = { kInvalidId };
  {
    float_bf_t d_in(dir, 3 * sizeof(float));
    float_bf_t p_in(pos, 3 * sizeof(float));
    float_bf_t wt_in(w, sizeof(float));
    id_bf_t fi_src(src_fid_ref, sizeof(IdType));
    float_bf_t p_out(pos_out_ref, 3 * sizeof(float));
    id_bf_t fi_out(fid_ref, sizeof(IdType));
    Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);
  }

  // Multi-chunk path (num=129 → chunk1=128, chunk2=1); only first ray is
  // active, the rest are padded with w=-1 so they get skipped in scatter.
  constexpr int kNum = 129;
  std::vector<float> dirs(kNum * 3, 0.0f);
  std::vector<float> positions(kNum * 3, 0.0f);
  std::vector<float> weights(kNum, -1.0f);
  std::vector<float> pos_out_chunk(kNum * 3, 0.0f);
  std::vector<IdType> fid_chunk(kNum, kInvalidId);
  std::vector<IdType> src_fid_chunk(kNum, kInvalidId);

  dirs[0] = dir[0];
  dirs[1] = dir[1];
  dirs[2] = dir[2];
  weights[0] = 1.0f;

  {
    float_bf_t d_in(dirs.data(), 3 * sizeof(float));
    float_bf_t p_in(positions.data(), 3 * sizeof(float));
    float_bf_t wt_in(weights.data(), sizeof(float));
    id_bf_t fi_src(src_fid_chunk.data(), sizeof(IdType));
    float_bf_t p_out(pos_out_chunk.data(), 3 * sizeof(float));
    id_bf_t fi_out(fid_chunk.data(), sizeof(IdType));
    Propagate(crystal_, kNum, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);
  }

  EXPECT_NE(fid_ref[0], kInvalidId);
  EXPECT_EQ(fid_ref[0], fid_chunk[0]);
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(pos_out_ref[j], pos_out_chunk[j], 1e-5f);
  }
}

TEST_F(PropagateTest, NoIntersection) {
  // Ray starting outside the crystal, pointing away
  // Use a point far from the crystal
  float dir[3] = { 1.0f, 0.0f, 0.0f };
  float pos[3] = { 100.0f, 100.0f, 100.0f };
  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };
  IdType src_fid[1] = { kInvalidId };

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(src_fid, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // PropagateSlab finds exit faces via half-space intersection
  // From outside the crystal, the slab method may still find an exit face
  // (it doesn't check if the ray starts inside). This is expected behavior
  // since Propagate assumes the ray starts inside the crystal.
  // We just verify no crash occurs.
  EXPECT_TRUE(true);  // No crash is the test
}

// Find the polygon face whose normal aligns with target_n (dot > 0.99).
// Returns kInvalidId if no polygon face matches.
static IdType FindPolygonFaceByNormal(const Crystal& crystal, const float* target_n) {
  const float* pn = crystal.GetPolygonFaceNormal();
  for (size_t p = 0; p < crystal.PolygonFaceCount(); p++) {
    if (Dot3(target_n, pn + p * 3) > 0.99f) {
      return static_cast<IdType>(p);
    }
  }
  return kInvalidId;
}

// AC-1: TIR on fn=3 (x=√3/4), ray starts at shared edge with fn=4 (0.5x+√3/2·y=√3/4).
// from_face_in = polygon face of fn=3 (TIR source); adjacent fn=4 should be hit.
//
// Geometry note: pos.x is shifted inward by δ = 3e-6 (well below kFloatEps=1e-5)
// so t_fn4 = δ is reliably positive across platforms while still less than the
// pre-fix +kFloatEps acceptance threshold (the bug witness). A pos exactly on
// the shared edge (δ=0) makes t_fn4 mathematically 0, which on x86_64 floating
// point can land slightly negative outside [-kFloatEps, 0] and fail the test.
TEST_F(PropagateTest, TIREdgeAdjacentFaceHit) {
  const float kSqrt3 = std::sqrt(3.0f);
  constexpr float kInwardShift = 3e-6f;
  float pos[3] = { kSqrt3 / 4.0f - kInwardShift, 0.25f, 0.0f };
  float dir[3] = { -0.5f, kSqrt3 / 2.0f, 0.0f };
  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };

  // Use the polygon face whose normal is fn=3 (normal ≈ (1, 0, 0)).
  const float kFn3Norm[3] = { 1.0f, 0.0f, 0.0f };
  IdType src_poly = FindPolygonFaceByNormal(crystal_, kFn3Norm);
  ASSERT_NE(src_poly, kInvalidId) << "Could not find fn=3 polygon face";

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(&src_poly, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // Should hit fn=4 (normal ≈ (0.5, √3/2, 0)); verify via dot product
  ASSERT_NE(fid_out[0], kInvalidId) << "Expected adjacent face hit, got no intersection";
  const float* norm_out = crystal_.GetPolygonFaceNormal() + fid_out[0] * 3;
  float dot = norm_out[0] * 0.5f + norm_out[1] * (kSqrt3 / 2.0f);
  EXPECT_NEAR(dot, 1.0f, 1e-3f);
}

// AC-2: same geometry as AC-1 but num=129 crosses the chunk boundary
// (chunk1=128, chunk2=1), exercising the per-chunk source-face exclusion
// path. The inward-shift rationale from AC-1 still applies: with δ=0 the
// computed t at the shared edge can land slightly negative on x86_64 and
// trip the source-face guard spuriously.
TEST_F(PropagateTest, TIREdgeAdjacentFaceHit_LargeNum) {
  const float kSqrt3 = std::sqrt(3.0f);
  constexpr float kInwardShift = 3e-6f;
  constexpr int kNum = 129;
  std::vector<float> dirs(kNum * 3, 0.0f);
  std::vector<float> positions(kNum * 3, 0.0f);
  std::vector<float> weights(kNum, -1.0f);
  std::vector<float> pos_out(kNum * 3, 0.0f);
  std::vector<IdType> fid_out(kNum, kInvalidId);
  std::vector<IdType> src_fids(kNum, kInvalidId);

  dirs[0] = -0.5f;
  dirs[1] = kSqrt3 / 2.0f;
  dirs[2] = 0.0f;
  positions[0] = kSqrt3 / 4.0f - kInwardShift;
  positions[1] = 0.25f;
  positions[2] = 0.0f;
  weights[0] = 1.0f;

  // Polygon face for fn=3 (normal ≈ (1, 0, 0)) as source face.
  const float kFn3Norm[3] = { 1.0f, 0.0f, 0.0f };
  src_fids[0] = FindPolygonFaceByNormal(crystal_, kFn3Norm);
  ASSERT_NE(src_fids[0], kInvalidId) << "Could not find fn=3 polygon face";

  float_bf_t d_in(dirs.data(), 3 * sizeof(float));
  float_bf_t p_in(positions.data(), 3 * sizeof(float));
  float_bf_t wt_in(weights.data(), sizeof(float));
  id_bf_t fi_src(src_fids.data(), sizeof(IdType));
  float_bf_t p_out(pos_out.data(), 3 * sizeof(float));
  id_bf_t fi_out(fid_out.data(), sizeof(IdType));

  Propagate(crystal_, kNum, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // Should hit fn=4 (normal ≈ (0.5, √3/2, 0))
  ASSERT_NE(fid_out[0], kInvalidId) << "Expected adjacent face hit, got no intersection";
  const float* norm_out = crystal_.GetPolygonFaceNormal() + fid_out[0] * 3;
  float dot = norm_out[0] * 0.5f + norm_out[1] * (kSqrt3 / 2.0f);
  EXPECT_NEAR(dot, 1.0f, 1e-3f);
}

// AC-3: guard test — source face must not be selected as exit even when t≈-ε.
// Ray starts at center of fn=3 face slightly pushed inward; direction is barely toward fn=3.
// fid_in_src = fn=3 triangle; verify fid_out != fn=3.
TEST_F(PropagateTest, PropagateSourceFaceNotSelected) {
  const float kSqrt3 = std::sqrt(3.0f);

  // Polygon face for fn=3 (normal ≈ (1, 0, 0)) as source face.
  const float kFn3Norm[3] = { 1.0f, 0.0f, 0.0f };
  IdType src_poly = FindPolygonFaceByNormal(crystal_, kFn3Norm);
  ASSERT_NE(src_poly, kInvalidId) << "Could not find fn=3 polygon face";

  // Place ray half-epsilon inside fn=3 (pos.x = √3/4 - kFloatEps/2), direction +x (outward).
  // t_fn3 = (kFloatEps/2) / 1.0 ∈ (0, kFloatEps): in the guard zone.
  //   - Correct code (source face uses +kFloatEps): t_fn3 < kFloatEps → fn=3 rejected → fid_out = kInvalidId.
  //   - Regressed code (all faces use -kFloatEps): t_fn3 > -kFloatEps → fn=3 accepted → fid_out = fn=3 poly.
  float pos[3] = { kSqrt3 / 4.0f - math::kFloatEps * 0.5f, 0.0f, 0.0f };
  float dir[3] = { 1.0f, 0.0f, 0.0f };

  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(&src_poly, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  // With dir=(+1,0,0) from a point ε/2 inside fn=3, the only face geometrically
  // reachable in the forward direction is fn=3 itself. A correctly-functioning
  // source-face guard must reject it, leaving fid_out=kInvalidId. Asserting
  // "kInvalidId" is strictly stronger than "not fn=3": it also catches regressions
  // where some unrelated face is spuriously selected (e.g. PropagateSlab returning
  // a stale far_face), which the earlier `if (fid_out>=0)` wrapping would miss.
  EXPECT_EQ(fid_out[0], kInvalidId) << "guard zone must produce no hit; "
                                       "fid_out (="
                                    << fid_out[0]
                                    << ") means either "
                                       "source face fn=3 was wrongly selected or some unreachable face was returned";
}

// AC-3 (a): polygon-only equivalence on a regular hexagonal prism.
// Six horizontal rays from the prism centre, each aimed along one of the
// six side-face outward normals (60° apart, in the x-y plane). Each ray
// must land on a side face whose outward normal aligns with the ray
// direction (dot > 0.9), i.e. the ray exits through the polygon face it
// is pointing at.
TEST_F(PropagateTest, PolygonOnlyPrism6SideFaces) {
  const float kSqrt3 = std::sqrt(3.0f);
  // Outward normals of the 6 hexagonal side faces (unit vectors, z=0).
  const float kDirs[6][3] = {
    { 1.0f, 0.0f, 0.0f },             //
    { 0.5f, kSqrt3 / 2.0f, 0.0f },    //
    { -0.5f, kSqrt3 / 2.0f, 0.0f },   //
    { -1.0f, 0.0f, 0.0f },            //
    { -0.5f, -kSqrt3 / 2.0f, 0.0f },  //
    { 0.5f, -kSqrt3 / 2.0f, 0.0f },   //
  };

  for (int i = 0; i < 6; i++) {
    float dir[3] = { kDirs[i][0], kDirs[i][1], kDirs[i][2] };
    float pos[3] = { 0.0f, 0.0f, 0.0f };
    float w[1] = { 1.0f };
    float pos_out[3] = {};
    IdType fid_out[1] = { kInvalidId };
    IdType src_fid[1] = { kInvalidId };

    float_bf_t d_in(dir, 3 * sizeof(float));
    float_bf_t p_in(pos, 3 * sizeof(float));
    float_bf_t wt_in(w, sizeof(float));
    id_bf_t fi_src(src_fid, sizeof(IdType));
    float_bf_t p_out(pos_out, 3 * sizeof(float));
    id_bf_t fi_out(fid_out, sizeof(IdType));

    Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

    ASSERT_NE(fid_out[0], kInvalidId) << "Ray " << i << " missed all faces";
    const float* norm = crystal_.GetPolygonFaceNormal() + fid_out[0] * 3;
    // Hit a side face (z-component of normal ≈ 0).
    EXPECT_NEAR(norm[2], 0.0f, 1e-4f) << "Ray " << i << " hit non-side face (norm.z=" << norm[2] << ")";
    // Outward normal aligns with the ray direction.
    float dot = norm[0] * dir[0] + norm[1] * dir[1] + norm[2] * dir[2];
    EXPECT_GT(dot, 0.9f) << "Ray " << i << " hit wrong side face (dot=" << dot << ")";
  }
}

// AC-3 (b): polygon-only equivalence on a hexagonal pyramid with both
// cone segments present. Parameters chosen so that both the upper and
// lower pyramid caps exist (h1=0.5, h3=0.5 are non-zero) and the prism
// middle segment is non-degenerate (h2=1.0). Vertical rays from the
// origin must hit a face on the corresponding cone (norm.z > 0.5 for
// the upward ray, < -0.5 for the downward ray).
TEST_F(PropagateTest, PolygonOnlyPyramidConeFaces) {
  Crystal pyramid = Crystal::CreatePyramid(0.5f, 1.0f, 0.5f);

  struct Case {
    float dir_z;
    float expected_norm_z_sign;
    const char* label;
  };
  const Case kCases[] = {
    { 1.0f, 1.0f, "up -> upper cone" },
    { -1.0f, -1.0f, "down -> lower cone" },
  };

  for (const auto& c : kCases) {
    float dir[3] = { 0.0f, 0.0f, c.dir_z };
    float pos[3] = { 0.0f, 0.0f, 0.0f };
    float w[1] = { 1.0f };
    float pos_out[3] = {};
    IdType fid_out[1] = { kInvalidId };
    IdType src_fid[1] = { kInvalidId };

    float_bf_t d_in(dir, 3 * sizeof(float));
    float_bf_t p_in(pos, 3 * sizeof(float));
    float_bf_t wt_in(w, sizeof(float));
    id_bf_t fi_src(src_fid, sizeof(IdType));
    float_bf_t p_out(pos_out, 3 * sizeof(float));
    id_bf_t fi_out(fid_out, sizeof(IdType));

    Propagate(pyramid, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

    ASSERT_NE(fid_out[0], kInvalidId) << c.label << ": ray missed all faces";
    const float* norm = pyramid.GetPolygonFaceNormal() + fid_out[0] * 3;
    // Hit face must belong to the expected cone segment (norm.z sign + magnitude > 0.5).
    EXPECT_GT(norm[2] * c.expected_norm_z_sign, 0.5f)
        << c.label << ": hit face norm.z=" << norm[2] << " (expected sign " << c.expected_norm_z_sign << ")";
  }
}

// AC-5: when from_face_in = kInvalidId (initial ray, no source face), Propagate
// applies no source-face guard — equivalent to the legacy fid_in_src = -1 path.
// The ray from the origin along +x must hit some polygon face.
TEST_F(PropagateTest, FromFaceNotSelf_InitialRayNoSource) {
  float dir[3] = { 1.0f, 0.0f, 0.0f };
  float pos[3] = { 0.0f, 0.0f, 0.0f };
  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };
  IdType src_fid[1] = { kInvalidId };  // no source — legacy "-1" sentinel

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(src_fid, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  ASSERT_NE(fid_out[0], kInvalidId) << "kInvalidId source should not exclude any face";
}

// AC-5: to_face_out from Propagate is a polygon face index — i.e. always in
// [0, PolygonFaceCount()), never a triangle id. Guarantees the new ABI invariant
// downstream consumers (HitSurface normal lookup, raypath recorder) rely on.
TEST_F(PropagateTest, ToFaceIsPolygonFaceIndex) {
  float dir[3] = { 1.0f, 0.0f, 0.0f };
  float pos[3] = { 0.0f, 0.0f, 0.0f };
  float w[1] = { 1.0f };
  float pos_out[3] = {};
  IdType fid_out[1] = { kInvalidId };
  IdType src_fid[1] = { kInvalidId };

  float_bf_t d_in(dir, 3 * sizeof(float));
  float_bf_t p_in(pos, 3 * sizeof(float));
  float_bf_t wt_in(w, sizeof(float));
  id_bf_t fi_src(src_fid, sizeof(IdType));
  float_bf_t p_out(pos_out, 3 * sizeof(float));
  id_bf_t fi_out(fid_out, sizeof(IdType));

  Propagate(crystal_, 1, 1, d_in, p_in, wt_in, fi_src, p_out, fi_out);

  ASSERT_NE(fid_out[0], kInvalidId);
  EXPECT_LT(fid_out[0], crystal_.PolygonFaceCount()) << "fid_out must be a polygon-face index, got " << fid_out[0]
                                                     << " (PolygonFaceCount=" << crystal_.PolygonFaceCount() << ")";
}
