#include <gtest/gtest.h>

#include <fstream>
#include <utility>
#include <variant>

#include "core/math.hpp"
#include "json.hpp"
#include "protocol/crystal_config.hpp"

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

TEST_F(V3TestJson, Crystal1) {
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

TEST_F(V3TestJson, Crystal3) {
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
    ASSERT_NEAR(x.mean, 1.0f, 1e-5);
    ASSERT_NEAR(x.std, 0.2f, 1e-5);
  }
}

}  // namespace
