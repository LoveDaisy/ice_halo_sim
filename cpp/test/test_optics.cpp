#include "optics.h"
#include "crystal.h"

#include "gtest/gtest.h"


namespace {

class OpticsTest : public ::testing::Test {
protected:
  void SetUp() override {
    crystal = IceHalo::Crystal::createHexPrism(1.2f);
  }

  IceHalo::CrystalPtr crystal;
};


TEST_F(OpticsTest, HitSurface0) {
  constexpr float kN = 1.31;
  constexpr int kNum = 3;

  float dir_in[kNum * 3] = {
         0.0f,  0.0f,      -1.0f,      // Case 1: perpendicular incident
    0.707107f,  0.0f, -0.707107f,      // Case 2: incident at 45 degree
    0.792624f,  0.0f,  0.609711f,      // Case 3: incident at 45 degree, from inside out, total reflection
  };
  float w_in[kNum] = {
    1.0f,                     // Case 1: full intensity
    1.0f,                     // Case 2: full intensity
    1.0f,                     // Case 3: full intensity
  };
  int face_id_in[kNum] = {
    0,                        // Case 1: top face. (face number 2)
    0,                        // Case 2: top face. (face number 2)
    0,                        // Case 3: top face. (face number 2)
  };

  float dir_out_e[2 * kNum * 3] = {
         0.0f,  0.0f,       1.0f,        // Case 1: reflective
         0.0f,  0.0f,      -1.0f,        // Case 1: refractive
    0.707107f,  0.0f,  0.707107f,        // Case 2: reflective
    0.539776f,  0.0f, -0.841809f,        // Case 2: refractive
    0.792624f,  0.0f, -0.609711f,        // Case 3: reflective, total reflection
    0.792624f,  0.0f, -0.609711f,        // Case 3: refractive, total reflection
  };
  float w_out_e[2 * kNum] = {
    0.018009f,                 // Case 1: reflective
    0.981991f,                 // Case 1: refractive
    0.025038f,                 // Case 2: reflective
    0.974962f,                 // Case 2: refractive
         1.0f,                 // Case 3: reflective, total reflection
         0.0f,                 // Case 3: refractive, total reflection
  };

  float dir_out[2 * kNum * 3];
  float w_out[2 * kNum];

  IceHalo::Optics::HitSurface(crystal, kN, kNum,
                              dir_in, face_id_in, w_in,
                              dir_out, w_out);

  for (int i = 0; i < kNum * 2; i++) {
    EXPECT_NEAR(w_out[i], w_out_e[i], IceHalo::Math::kFloatEps);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(dir_out[i * 3 + j], dir_out_e[i * 3 + j], IceHalo::Math::kFloatEps);
    }
  }
}


TEST_F(OpticsTest, HitSurface1) {
  constexpr float kN = 1.31;
  constexpr int kNum = 3;

  float dir_in[kNum * 3] = {
         0.0f,  0.0f,      -1.0f,      // Case 1: perpendicular incident
    0.707107f,  0.0f, -0.707107f,      // Case 2: incident at 45 degree
    0.792624f,  0.0f,  0.609711f,      // Case 3: incident at 45 degree, from inside out, total reflection
  };
  float w_in[kNum] = {
    0.5f,                     // Case 1: full intensity
    0.5f,                     // Case 2: full intensity
    0.5f,                     // Case 3: full intensity
  };
  int face_id_in[kNum] = {
    0,                        // Case 1: top face. (face number 2)
    0,                        // Case 2: top face. (face number 2)
    0,                        // Case 3: top face. (face number 2)
  };

  float dir_out_e[2 * kNum * 3] = {
         0.0f,  0.0f,       1.0f,        // Case 1: reflective
         0.0f,  0.0f,      -1.0f,        // Case 1: refractive
    0.707107f,  0.0f,  0.707107f,        // Case 2: reflective
    0.539776f,  0.0f, -0.841809f,        // Case 2: refractive
    0.792624f,  0.0f, -0.609711f,        // Case 3: reflective, total reflection
    0.792624f,  0.0f, -0.609711f,        // Case 3: refractive, total reflection
  };
  float w_out_e[2 * kNum] = {
    0.009005f,                 // Case 1: reflective
    0.490995f,                 // Case 1: refractive
    0.012519f,                 // Case 2: reflective
    0.487481f,                 // Case 2: refractive
         0.5f,                 // Case 3: reflective, total reflection
         0.0f,                 // Case 3: refractive, total reflection
  };

  float dir_out[2 * kNum * 3];
  float w_out[2 * kNum];

  IceHalo::Optics::HitSurface(crystal, kN, kNum,
                              dir_in, face_id_in, w_in,
                              dir_out, w_out);

  for (int i = 0; i < kNum * 2; i++) {
    EXPECT_NEAR(w_out[i], w_out_e[i], IceHalo::Math::kFloatEps);
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(dir_out[i * 3 + j], dir_out_e[i * 3 + j], IceHalo::Math::kFloatEps);
    }
  }
}

}  // namespace
