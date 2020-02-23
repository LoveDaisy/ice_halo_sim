#include <string>

#include "context/context.hpp"
#include "core/mymath.hpp"
#include "gtest/gtest.h"

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
  auto sun_dir = context->sun_ctx_->GetSunPosition();
  EXPECT_NEAR(sun_dir[0], 0.0f, icehalo::math::kFloatEps);
  EXPECT_NEAR(sun_dir[1], -0.906308f, icehalo::math::kFloatEps);
  EXPECT_NEAR(sun_dir[2], -0.422618f, icehalo::math::kFloatEps);
}


TEST_F(ContextTest, FillSunDir) {
  auto sun_dir = context->sun_ctx_->GetSunPosition();
  auto sun_d = context->sun_ctx_->GetSunDiameter();

  constexpr int kRayNum = 200;
  float dir[3 * kRayNum];
  icehalo::math::RandomSampler::SampleSphericalPointsCart(sun_dir, sun_d / 2, dir, kRayNum);

  for (int i = 0; i < kRayNum; i++) {
    float a = icehalo::math::Dot3(sun_dir, dir + i * 3);  // In rad
    EXPECT_TRUE(a > std::cos(sun_d / 2 * icehalo::math::kDegreeToRad) - 5e-7);
  }
}

TEST_F(ContextTest, RayPathHash01) {
  auto ctx = context->GetCrystalContext(4);

  icehalo::RayPath ray_path_01{ 2, 3, 6, icehalo::kInvalidFaceNumber };
  icehalo::RayPath ray_path_02{ 2, 5, 8, icehalo::kInvalidFaceNumber };

  auto hash_01 = icehalo::RayPathRecorder::Hash(ray_path_01);
  auto hash_02 = icehalo::RayPathRecorder::Hash(ray_path_02);
  auto hash_01_normalized = icehalo::NormalizeRayPath(ray_path_01, ctx, icehalo::RenderSplitter::kDefaultSymmetry);
  auto hash_02_normalized = icehalo::NormalizeRayPath(ray_path_02, ctx, icehalo::RenderSplitter::kDefaultSymmetry);
  ASSERT_NE(hash_01, hash_02);
  ASSERT_EQ(hash_01_normalized, hash_02_normalized);
}

TEST_F(ContextTest, RayPathHash02) {
  auto ctx = context->GetCrystalContext(4);
  const auto sym_flag = icehalo::RenderSplitter::kDefaultSymmetry;

  std::vector<icehalo::RayPath> ray_path_list{
    icehalo::RayPath{ 2, 3, 6, icehalo::kInvalidFaceNumber },
    icehalo::RayPath{ 2, 8, 6, icehalo::kInvalidFaceNumber },
    icehalo::RayPath{ 2, 8, 6, icehalo::kInvalidFaceNumber, 1, 3, 5, icehalo::kInvalidFaceNumber },
  };

  for (const auto& rp0 : ray_path_list) {
    auto ext_ray_path_list = icehalo::MakeSymmetryExtension(rp0, ctx, sym_flag);
    auto normalized_hash = icehalo::NormalizeRayPath(rp0, ctx, sym_flag);
    for (const auto& rp : ext_ray_path_list) {
      auto curr_hash = icehalo::NormalizeRayPath(rp, ctx, sym_flag);
      EXPECT_EQ(normalized_hash, curr_hash);
    }
  }
}

}  // namespace
