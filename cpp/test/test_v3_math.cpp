#include <gtest/gtest.h>

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

}  // namespace
