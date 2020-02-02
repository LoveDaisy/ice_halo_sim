#include <string>

#include "context.h"
#include "gtest/gtest.h"
#include "mymath.h"

extern std::string config_file_name;

namespace {

class ContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context = icehalo::ProjectContext::CreateFromFile(config_file_name.c_str());  // read from file
  }

  icehalo::ProjectContextPtr context;
};


TEST_F(ContextTest, CreateNotNull) {
  ASSERT_NE(context, nullptr);
}


TEST_F(ContextTest, CheckSunDir) {
  auto sun_dir = context->sun_ctx_.GetSunPosition();
  EXPECT_NEAR(sun_dir[0], 0.0f, icehalo::math::kFloatEps);
  EXPECT_NEAR(sun_dir[1], -0.906308f, icehalo::math::kFloatEps);
  EXPECT_NEAR(sun_dir[2], -0.422618f, icehalo::math::kFloatEps);
}


TEST_F(ContextTest, FillSunDir) {
  auto sun_dir = context->sun_ctx_.GetSunPosition();
  auto sun_d = context->sun_ctx_.GetSunDiameter();

  constexpr int kRayNum = 200;
  float dir[3 * kRayNum];
  icehalo::math::RandomSampler::SampleSphericalPointsCart(sun_dir, sun_d / 2, dir, kRayNum);

  for (int i = 0; i < kRayNum; i++) {
    float a = icehalo::math::Dot3(sun_dir, dir + i * 3);  // In rad
    EXPECT_TRUE(a > std::cos(sun_d / 2 * icehalo::math::kDegreeToRad) - 5e-7);
  }
}

}  // namespace
