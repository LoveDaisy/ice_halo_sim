#include <gtest/gtest.h>

#include <fstream>
#include <utility>
#include <variant>

#include "core/math.hpp"
#include "json.hpp"
#include "protocol/crystal_config.hpp"
#include "protocol/filter_config.hpp"

extern std::string config_file_name;
using namespace icehalo;

namespace {

class V3TestJson : public ::testing::Test {
 protected:
  void SetUp() override {
    std::ifstream f(config_file_name);
    f >> config_json_;
  }

  nlohmann::json config_json_;
};

TEST_F(V3TestJson, Distribution) {
  const auto& j_height = config_json_.at("crystal")[1].at("shape").at("height");
  auto d = j_height.get<Distribution>();

  ASSERT_EQ(d.type, DistributionType::kGaussian);
  ASSERT_NEAR(d.mean, 1.3, 1e-5);
  ASSERT_NEAR(d.std, 0.2, 1e-5);
}

TEST_F(V3TestJson, Crystal_PrismSimple) {
  const auto& j_crystal = config_json_.at("crystal")[1];
  auto c = j_crystal.get<v3::CrystalConfig>();

  ASSERT_EQ(c.id_, 2);
  ASSERT_TRUE(std::holds_alternative<v3::PrismCrystalParam>(c.param_));

  const auto& p = std::get<v3::PrismCrystalParam>(c.param_);
  ASSERT_EQ(p.h_.type, DistributionType::kGaussian);
  ASSERT_NEAR(p.h_.mean, 1.3, 1e-5);
  ASSERT_NEAR(p.h_.std, 0.2, 1e-5);
  for (const auto& x : p.d_) {
    ASSERT_EQ(x.type, DistributionType::kNoRandom);
    ASSERT_NEAR(x.mean, 1.0f, 1e-5);
    ASSERT_NEAR(x.std, 0.0f, 1e-5);
  }
}

TEST_F(V3TestJson, Crystal_PrismFull) {
  const auto& j_crystal = config_json_.at("crystal")[3];
  auto c = j_crystal.get<v3::CrystalConfig>();

  ASSERT_EQ(c.id_, 4);
  ASSERT_TRUE(std::holds_alternative<v3::PrismCrystalParam>(c.param_));

  const auto& p = std::get<v3::PrismCrystalParam>(c.param_);
  ASSERT_EQ(p.h_.type, DistributionType::kUniform);
  ASSERT_NEAR(p.h_.mean, 1.2, 1e-5);
  ASSERT_NEAR(p.h_.std, 0.4, 1e-5);
  for (const auto& x : p.d_) {
    ASSERT_EQ(x.type, DistributionType::kGaussian);
    ASSERT_NEAR(x.mean, 1.0f * math::kSqrt3_4, 1e-5);
    ASSERT_NEAR(x.std, 0.2f * math::kSqrt3_4, 1e-5);
  }

  const auto& axis = c.axis_;
  ASSERT_EQ(axis.azimuth_dist.type, DistributionType::kUniform);
  ASSERT_NEAR(axis.azimuth_dist.mean, 0, 1e-5);
  ASSERT_NEAR(axis.azimuth_dist.std, 360, 1e-5);
  ASSERT_EQ(axis.latitude_dist.type, DistributionType::kGaussian);
  ASSERT_NEAR(axis.latitude_dist.mean, 90, 1e-5);
  ASSERT_NEAR(axis.latitude_dist.std, 1.2, 1e-5);
  ASSERT_EQ(axis.roll_dist.type, DistributionType::kUniform);
  ASSERT_NEAR(axis.roll_dist.mean, 0, 1e-5);
  ASSERT_NEAR(axis.roll_dist.std, 360, 1e-5);
}

TEST_F(V3TestJson, Filter_None) {
  const auto& j_filter = config_json_.at("filter")[0];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 1);
  ASSERT_TRUE(std::holds_alternative<v3::NoneFilterParam>(f.param_));
}

}  // namespace
