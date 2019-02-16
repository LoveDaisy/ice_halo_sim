#include "context.h"
#include "mymath.h"

#include "gtest/gtest.h"

#include <string>

extern std::string config_file_name;

namespace {

class ContextTest : public ::testing::Test {
protected:
  void SetUp() override {
    context = IceHalo::SimulationContext::createFromFile(config_file_name.c_str());
  }

  IceHalo::SimulationContextPtr context;
};


TEST_F(ContextTest, CreateNotNull) {
  ASSERT_NE(context, nullptr);
}


TEST_F(ContextTest, CheckSunDir) {
  auto sunDir = context->getSunDir();
  EXPECT_NEAR(sunDir[0], 0.0f, IceHalo::Math::kFloatEps);
  EXPECT_NEAR(sunDir[1], -0.906308f, IceHalo::Math::kFloatEps);
  EXPECT_NEAR(sunDir[2], -0.422618f, IceHalo::Math::kFloatEps);
}


TEST_F(ContextTest, FillSunDir) {
  auto sunDir = context->getSunDir();
  auto sunD = context->getSunDiameter();

  constexpr int kRayNum = 100;
  float dir[3 * kRayNum];
  context->fillSunDir(dir, kRayNum);

  for (int i = 0; i < kRayNum; i++) {
    float a = std::acos(IceHalo::Math::dot3(sunDir, dir + i * 3));    // In rad
    a *= 180.0f / IceHalo::Math::kPi;    // To degree
    EXPECT_TRUE(a < sunD / 2);
  }
}

}  // namespace
