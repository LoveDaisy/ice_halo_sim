#include "core/crystal.hpp"
#include "core/optics.hpp"
#include "gtest/gtest.h"
#include "process/simulation.hpp"

extern std::string config_file_name;

namespace {

class OpticsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    crystal_ = icehalo::Crystal::CreateHexPrism(1.2f);
    context_ = icehalo::ProjectContext::CreateFromFile(config_file_name.c_str());
  }

  icehalo::CrystalPtrU crystal_;
  icehalo::ProjectContextPtr context_;
};


TEST_F(OpticsTest, RefractIndex) {
  float wl[]{ 400, 500, 600 };
  float n[]{ 1.3194f, 1.3130f, 1.3094f };
  int idx = 0;
  for (const auto& curr_wl : wl) {
    EXPECT_NEAR(icehalo::IceRefractiveIndex::Get(curr_wl), n[idx++], 1e-4);
  }
}


TEST_F(OpticsTest, HitSurface0) {
  constexpr float kN = 1.31;
  constexpr int kNum = 3;

  float dir_in[kNum * 3]{
    0.0f,      0.0f, -1.0f,       // Case 1: perpendicular incident
    0.707107f, 0.0f, -0.707107f,  // Case 2: incident at 45 degree
    0.792624f, 0.0f, 0.609711f,   // Case 3: incident at 45 degree, from inside out, total reflection
  };
  float w_in[kNum]{
    1.0f,  // Case 1: full intensity
    1.0f,  // Case 2: full intensity
    1.0f,  // Case 3: full intensity
  };
  int face_id_in[kNum]{
    0,  // Case 1: top face. (face number 2)
    0,  // Case 2: top face. (face number 2)
    0,  // Case 3: top face. (face number 2)
  };

  float dir_out_e[2 * kNum * 3]{
    0.0f,      0.0f, 1.0f,        // Case 1: reflective
    0.0f,      0.0f, -1.0f,       // Case 1: refractive
    0.707107f, 0.0f, 0.707107f,   // Case 2: reflective
    0.539776f, 0.0f, -0.841809f,  // Case 2: refractive
    0.792624f, 0.0f, -0.609711f,  // Case 3: reflective, total reflection
    0.792624f, 0.0f, -0.609711f,  // Case 3: refractive, total reflection
  };
  float w_out_e[2 * kNum]{
    0.018009f,  // Case 1: reflective
    0.981991f,  // Case 1: refractive
    0.025038f,  // Case 2: reflective
    0.974962f,  // Case 2: refractive
    1.0f,       // Case 3: reflective, total reflection
    -1.0f,      // Case 3: refractive, total reflection
  };

  float dir_out[2 * kNum * 3];
  float w_out[2 * kNum];

  icehalo::Optics::HitSurface(crystal_.get(), kN, kNum,  // input
                              dir_in, face_id_in, w_in,  // input
                              dir_out, w_out);           // output

  for (int i = 0; i < kNum * 2; i++) {
    EXPECT_NEAR(w_out[i], w_out_e[i], icehalo::math::kFloatEps);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(dir_out[i * 3 + j], dir_out_e[i * 3 + j], icehalo::math::kFloatEps);
    }
  }
}


TEST_F(OpticsTest, HitSurface1) {
  constexpr float kN = 1.31;
  constexpr int kNum = 3;

  float dir_in[kNum * 3]{
    0.0f,      0.0f, -1.0f,       // Case 1: perpendicular incident
    0.707107f, 0.0f, -0.707107f,  // Case 2: incident at 45 degree
    0.792624f, 0.0f, 0.609711f,   // Case 3: incident at 45 degree, from inside out, total reflection
  };
  float w_in[kNum]{
    0.5f,  // Case 1: half intensity
    0.5f,  // Case 2: half intensity
    0.5f,  // Case 3: half intensity
  };
  int face_id_in[kNum]{
    0,  // Case 1: top face. (face number 2)
    0,  // Case 2: top face. (face number 2)
    0,  // Case 3: top face. (face number 2)
  };

  float dir_out_e[2 * kNum * 3]{
    0.0f,      0.0f, 1.0f,        // Case 1: reflective
    0.0f,      0.0f, -1.0f,       // Case 1: refractive
    0.707107f, 0.0f, 0.707107f,   // Case 2: reflective
    0.539776f, 0.0f, -0.841809f,  // Case 2: refractive
    0.792624f, 0.0f, -0.609711f,  // Case 3: reflective, total reflection
    0.792624f, 0.0f, -0.609711f,  // Case 3: refractive, total reflection
  };
  float w_out_e[2 * kNum]{
    0.009005f,  // Case 1: reflective
    0.490995f,  // Case 1: refractive
    0.012519f,  // Case 2: reflective
    0.487481f,  // Case 2: refractive
    0.5f,       // Case 3: reflective, total reflection
    -1.0f,      // Case 3: refractive, total reflection
  };

  float dir_out[2 * kNum * 3];
  float w_out[2 * kNum];

  icehalo::Optics::HitSurface(crystal_.get(), kN, kNum,  // input
                              dir_in, face_id_in, w_in,  // input
                              dir_out, w_out);           // output

  for (int i = 0; i < kNum * 2; i++) {
    EXPECT_NEAR(w_out[i], w_out_e[i], icehalo::math::kFloatEps);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(dir_out[i * 3 + j], dir_out_e[i * 3 + j], icehalo::math::kFloatEps);
    }
  }
}


TEST_F(OpticsTest, RayFaceIntersection0) {
  auto c = icehalo::Crystal::CreateHexPrism(1.0f);
  auto face_num = c->TotalFaces();
  auto face_norm = c->GetFaceNorm();
  auto face_base = c->GetFaceBaseVector();
  auto face_point = c->GetFaceVertex();

  float dir_in[3]{ icehalo::math::kSqrt3 / 2, 0.5f, 0.0f };
  float p_in[3]{ -icehalo::math::kSqrt3 / 2, 0.0f, 0.0f };
  float p_out[3]{ icehalo::math::kSqrt3 / 4, 0.75f, 0.0f };
  int id_in = 10;
  int id_out = 6;

  float p_result[3];
  int id_result = -1;

  icehalo::Optics::IntersectLineWithTriangles(p_in, dir_in, id_in, face_num,     // input
                                              face_base, face_point, face_norm,  // input
                                              p_result, &id_result);             // output

  EXPECT_EQ(id_out, id_result);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(p_out[i], p_result[i], icehalo::math::kFloatEps);
  }

  icehalo::Optics::IntersectLineWithTrianglesSimd(p_in, dir_in, id_in, face_num,     // input
                                                  face_base, face_point, face_norm,  // input
                                                  p_result, &id_result);             // output

  EXPECT_EQ(id_out, id_result);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(p_out[i], p_result[i], icehalo::math::kFloatEps);
  }
}


TEST_F(OpticsTest, RayFaceIntersection1) {
  auto c = icehalo::Crystal::CreateHexPrism(1.0f);
  auto face_num = c->TotalFaces();
  auto face_norm = c->GetFaceNorm();
  auto face_base = c->GetFaceBaseVector();
  auto face_point = c->GetFaceVertex();

  constexpr int num = 5;
  // clang-format off
  float dir_in[num * 3] {
    icehalo::math::kSqrt3 / 2, 0.5f, 0.0f,   // case 1
    1.0f, 0.0f, 0.0f,                        // case 2
    0.5f, 0.0f, -icehalo::math::kSqrt3 / 2,  // case 3
    0.5f, -icehalo::math::kSqrt3 / 2, 0.0f,  // case 4
    0.35693541f, -0.18690710f, -0.91523923f, // case 5
  };
  float p_in[num * 3] {
    -icehalo::math::kSqrt3 / 2, 0.0f, 0.0f,      // case 1
    -0.5f, icehalo::math::kSqrt3 * 5 / 6, 0.8f,  // case 1
    0.0f, 0.0f, 0.0f,                            // case 3
    icehalo::math::kSqrt3 / 2, -0.2f, 0.5f,      // case 4
    -0.1f, 0.82679492f, 0.8f,                    // case 5
  };
  // clang-format on
  int id_in[num]{ 10, 8, 1, 4, 8 };

  for (int i = 0; i < num; i++) {
    float test_pt[]{ 0, 0, 0 };
    float expect_pt[]{ 0, 0, 0 };
    int test_id = -1;
    int expect_id = -1;
    icehalo::Optics::IntersectLineWithTriangles(p_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                                                face_base, face_point, face_norm,                  // input
                                                expect_pt, &expect_id);                            // output

    icehalo::Optics::IntersectLineWithTrianglesSimd(p_in + i * 3, dir_in + i * 3, id_in[i], face_num,  // input
                                                    face_base, face_point, face_norm,                  // input
                                                    test_pt, &test_id);                                // output
    EXPECT_EQ(expect_id, test_id);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(test_pt[j], expect_pt[j], icehalo::math::kFloatEps);
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
