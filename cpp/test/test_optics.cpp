#include "core/crystal.hpp"
#include "core/optics.hpp"
#include "gtest/gtest.h"
#include "util/log.hpp"

namespace {

class OpticsTest : public ::testing::Test {
 protected:
  void SetUp() override { crystal_ = icehalo::Crystal::CreateHexPrism(1.2f); }

  icehalo::CrystalPtrU crystal_;
};


TEST_F(OpticsTest, RefractIndex) {
  float wl[]{ 400.0f, 500.0f, 600.0f };
  float n[]{ 1.3194f, 1.3130f, 1.3094f };
  int idx = 0;
  for (const auto& curr_wl : wl) {
    EXPECT_NEAR(icehalo::IceRefractiveIndex::Get(curr_wl), n[idx++], 1e-4);
  }
}


TEST_F(OpticsTest, HitSurface) {
  constexpr float kN = 1.31;
  constexpr int kNum = 6;

  float dir_in[kNum * 3]{
    0.0f,      0.0f, -1.0f,       // Case 1: perpendicular incident
    0.707107f, 0.0f, -0.707107f,  // Case 2: incident at 45 degree
    0.792624f, 0.0f, 0.609711f,   // Case 3: incident at 45 degree, from inside out, total reflection
    0.0f,      0.0f, -1.0f,       // Case 4: perpendicular incident
    0.707107f, 0.0f, -0.707107f,  // Case 5: incident at 45 degree
    0.792624f, 0.0f, 0.609711f,   // Case 6: incident at 45 degree, from inside out, total reflection
  };
  float w_in[kNum]{
    1.0f,  // Case 1: full intensity
    1.0f,  // Case 2: full intensity
    1.0f,  // Case 3: full intensity
    0.5f,  // Case 4: half intensity
    0.5f,  // Case 5: half intensity
    0.5f,  // Case 6: half intensity
  };
  int face_id_in[kNum]{
    0,  // Case 1: top face. (face number 2)
    0,  // Case 2: top face. (face number 2)
    0,  // Case 3: top face. (face number 2)
    0,  // Case 4: top face. (face number 2)
    0,  // Case 5: top face. (face number 2)
    0,  // Case 6: top face. (face number 2)
  };

  float dir_out_e[2 * kNum * 3]{
    0.0f,      0.0f, 1.0f,        // Case 1: reflective
    0.0f,      0.0f, -1.0f,       // Case 1: refractive
    0.707107f, 0.0f, 0.707107f,   // Case 2: reflective
    0.539776f, 0.0f, -0.841809f,  // Case 2: refractive
    0.792624f, 0.0f, -0.609711f,  // Case 3: reflective, total reflection
    0.792624f, 0.0f, -0.609711f,  // Case 3: refractive, total reflection
    0.0f,      0.0f, 1.0f,        // Case 4: reflective
    0.0f,      0.0f, -1.0f,       // Case 4: refractive
    0.707107f, 0.0f, 0.707107f,   // Case 5: reflective
    0.539776f, 0.0f, -0.841809f,  // Case 5: refractive
    0.792624f, 0.0f, -0.609711f,  // Case 6: reflective, total reflection
    0.792624f, 0.0f, -0.609711f,  // Case 6: refractive, total reflection
  };
  float w_out_e[2 * kNum]{
    0.018009f,  // Case 1: reflective
    0.981991f,  // Case 1: refractive
    0.025038f,  // Case 2: reflective
    0.974962f,  // Case 2: refractive
    1.0f,       // Case 3: reflective, total reflection
    -1.0f,      // Case 3: refractive, total reflection
    0.009005f,  // Case 4: reflective
    0.490995f,  // Case 4: refractive
    0.012519f,  // Case 5: reflective
    0.487481f,  // Case 5: refractive
    0.5f,       // Case 6: reflective, total reflection
    -1.0f,      // Case 6: refractive, total reflection
  };

  float dir_out[2 * kNum * 3];
  float dir_out_new[2 * kNum * 3];
  float w_out[2 * kNum];
  float w_out_new[2 * kNum];

  icehalo::Optics::HitSurface(crystal_.get(), kN, kNum,  // input
                              dir_in, face_id_in, w_in,  // input
                              dir_out, w_out);           // output
  icehalo::v3::HitSurface(crystal_.get(), kN, kNum,      // input
                          dir_in, face_id_in, w_in,      // input
                          dir_out_new, w_out_new);       // output

  using icehalo::math::kFloatEps;
  for (int i = 0; i < kNum * 2; i++) {
    EXPECT_NEAR(w_out[i], w_out_e[i], kFloatEps) << "@(" << i << ")";
    EXPECT_NEAR(w_out[i], w_out_new[i], kFloatEps) << "@(" << i << ")";
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(dir_out[i * 3 + j], dir_out_e[i * 3 + j], kFloatEps) << "@(" << i << "," << j << ")";
      EXPECT_NEAR(dir_out[i * 3 + j], dir_out_new[i * 3 + j], kFloatEps) << "@(" << i << "," << j << ")";
    }
  }
}


TEST_F(OpticsTest, RayFaceIntersection) {
  auto c = icehalo::Crystal::CreateHexPrism(1.0f);
  auto face_num = c->TotalFaces();
  auto face_norm = c->GetFaceNorm();
  auto face_base = c->GetFaceBaseVector();
  auto face_point = c->GetFaceVertex();

  constexpr int kNum = 5;
  using icehalo::math::kSqrt3;
  float dir_in[kNum * 3]{
    kSqrt3 / 2,  0.5f,         0.0f,          // case 1
    1.0f,        0.0f,         0.0f,          // case 2
    0.5f,        0.0f,         -kSqrt3 / 2,   // case 3
    0.5f,        -kSqrt3 / 2,  0.0f,          // case 4
    0.35693541f, -0.18690710f, -0.91523923f,  // case 5
  };
  float p_in[kNum * 3]{
    -kSqrt3 / 2, 0.0f,           0.0f,  // case 1
    -0.5f,       kSqrt3 * 5 / 6, 0.8f,  // case 2
    0.0f,        0.0f,           0.0f,  // case 3
    kSqrt3 / 2,  -0.2f,          0.5f,  // case 4
    -0.1f,       0.82679492f,    0.8f,  // case 5
  };
  int id_in[kNum]{ 10, 8, 1, 4, 8 };

  float p_out[kNum * 3]{
    kSqrt3 / 4, 0.75f,     0.0f,   // case 1
    0.0f,       0.0f,      0.0f,   // case 2
    0.577350f,  0.0f,      -1.0f,  // case 3
    0.0f,       0.0f,      0.0f,   // case 4
    0.601984f,  0.459205f, -1.0f,  // case 5
  };
  int id_out[kNum]{ 6, -1, 16, -1, 16 };

  using icehalo::math::kFloatEps;
  for (int i = 0; i < kNum; i++) {
    float out_pt_simd[]{ 0, 0, 0 };
    float out_pt_normal[]{ 0, 0, 0 };
    int out_id_simd = -1;
    int out_id_normal = -1;
    icehalo::Optics::IntersectLineWithTriangles(p_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                                                face_base, face_point, face_norm,                  // input
                                                out_pt_normal, &out_id_normal);                    // output

    icehalo::Optics::IntersectLineWithTrianglesSimd(p_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                                                    face_base, face_point, face_norm,                  // input
                                                    out_pt_simd, &out_id_simd);                        // output
    EXPECT_EQ(out_id_normal, id_out[i]);
    EXPECT_EQ(out_id_normal, out_id_simd);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(out_pt_simd[j], out_pt_normal[j], kFloatEps);
      EXPECT_NEAR(out_pt_normal[j], p_out[i * 3 + j], kFloatEps);
    }
  }
}


TEST_F(OpticsTest, Propagate) {
  auto c = icehalo::Crystal::CreateHexPrism(1.0f);

  constexpr int kNum = 5;
  using icehalo::math::kSqrt3;
  float dir_in[kNum * 3]{
    kSqrt3 / 2,  0.5f,         0.0f,          // case 1
    1.0f,        0.0f,         0.0f,          // case 2
    0.5f,        0.0f,         -kSqrt3 / 2,   // case 3
    0.5f,        -kSqrt3 / 2,  0.0f,          // case 4
    0.35693541f, -0.18690710f, -0.91523923f,  // case 5
  };
  float pt_in[kNum * 3]{
    -kSqrt3 / 2, 0.0f,           0.0f,  // case 1
    -0.5f,       kSqrt3 * 5 / 6, 0.8f,  // case 2
    0.0f,        0.0f,           0.0f,  // case 3
    kSqrt3 / 2,  -0.2f,          0.5f,  // case 4
    -0.1f,       0.82679492f,    0.8f,  // case 5
  };
  float w_in[kNum]{
    1.0f,  // case 1
    1.0f,  // case 2
    1.0f,  // case 3
    1.0f,  // case 4
    1.0f,  // case 5
  };
  int id_in[kNum]{ 10, 8, 1, 4, 8 };

  float pt_out[kNum * 3]{
    kSqrt3 / 4, 0.75f,     0.0f,   // case 1
    0.0f,       0.0f,      0.0f,   // case 2
    0.577350f,  0.0f,      -1.0f,  // case 3
    0.0f,       0.0f,      0.0f,   // case 4
    0.601984f,  0.459205f, -1.0f,  // case 5
  };
  int id_out[kNum]{ 6, -1, 16, -1, 16 };

  float pt_out_result[kNum * 3]{};
  int id_out_result[kNum]{};


  icehalo::v3::Propagate(c.get(), kNum,                  // input
                         pt_in, dir_in, w_in, id_in,     // input
                         pt_out_result, id_out_result);  // output

  using icehalo::math::kFloatEps;
  for (int i = 0; i < kNum; i++) {
    EXPECT_EQ(id_out[i], id_out_result[i]);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(pt_out_result[i * 3 + j], pt_out[i * 3 + j], kFloatEps) << "@(" << i << "," << j << ")";
    }
  }
}


TEST_F(OpticsTest, RayPathHash) {
  icehalo::RayPathRecorder recorder1;
  icehalo::RayPathRecorder recorder2;
  recorder1 << 1 << 3 << 5 << icehalo::kInvalidId;
  ASSERT_EQ(recorder1.Hash(), 0x1fffe14181);

  recorder1.Clear();
  recorder1 << 1, 3, 5, icehalo::kInvalidId;
  ASSERT_EQ(recorder1.Hash(), 0x1fffe14181);

  recorder1.Clear();
  recorder1 << 1, 3;
  recorder2.Clear();
  recorder2 << 5, icehalo::kInvalidId;
  recorder1 << recorder2;
  ASSERT_EQ(recorder1.Hash(), 0x1fffe14181);

  recorder2.Clear();
  recorder2 << 5, icehalo::kInvalidId;
  recorder1.Clear();
  recorder1 << 1, 3;
  recorder1 >> recorder2;
  ASSERT_EQ(recorder2.Hash(), 0x1fffe14181);
}

}  // namespace
