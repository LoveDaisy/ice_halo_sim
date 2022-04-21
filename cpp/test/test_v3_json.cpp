#include <gtest/gtest.h>

#include <fstream>
#include <utility>
#include <variant>
#include <vector>

#include "core/core_def.hpp"
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

#define CHECK_DISTRIBUTION(d, t, m, s) \
  do {                                 \
    ASSERT_EQ(d.type, t);              \
    ASSERT_NEAR(d.mean, m, 1e-5);      \
    ASSERT_NEAR(d.std, s, 1e-5);       \
  } while (false)

TEST_F(V3TestJson, Distribution) {
  const auto& j_height = config_json_.at("crystal")[1].at("shape").at("height");
  auto d = j_height.get<Distribution>();

  CHECK_DISTRIBUTION(d, DistributionType::kGaussian, 1.3, 0.2);
}

TEST_F(V3TestJson, Crystal_PrismSimple) {
  const auto& j_crystal = config_json_.at("crystal")[1];
  auto c = j_crystal.get<v3::CrystalConfig>();

  ASSERT_EQ(c.id_, 2);
  ASSERT_TRUE(std::holds_alternative<v3::PrismCrystalParam>(c.param_));

  const auto& p = std::get<v3::PrismCrystalParam>(c.param_);
  CHECK_DISTRIBUTION(p.h_, DistributionType::kGaussian, 1.3, 0.2);
  for (const auto& x : p.d_) {
    CHECK_DISTRIBUTION(x, DistributionType::kNoRandom, 1.0f * math::kSqrt3_4, 0.0f);
  }
}

TEST_F(V3TestJson, Crystal_PrismFull) {
  const auto& j_crystal = config_json_.at("crystal")[3];
  auto c = j_crystal.get<v3::CrystalConfig>();

  ASSERT_EQ(c.id_, 4);
  ASSERT_TRUE(std::holds_alternative<v3::PrismCrystalParam>(c.param_));

  const auto& p = std::get<v3::PrismCrystalParam>(c.param_);
  CHECK_DISTRIBUTION(p.h_, DistributionType::kUniform, 1.2f, 0.4f);
  for (const auto& x : p.d_) {
    CHECK_DISTRIBUTION(x, DistributionType::kGaussian, 1.0f * math::kSqrt3_4, 0.2f * math::kSqrt3_4);
  }

  const auto& axis = c.axis_;
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
  CHECK_DISTRIBUTION(axis.latitude_dist, DistributionType::kGaussian, 90, 1.2);
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
}

TEST_F(V3TestJson, Crystal_PyramidSimple) {
  const auto& j_crystal = config_json_.at("crystal")[4];
  auto c = j_crystal.get<v3::CrystalConfig>();

  ASSERT_EQ(c.id_, 5);
  ASSERT_TRUE(std::holds_alternative<v3::PyramidCrystalParam>(c.param_));

  const auto& p = std::get<v3::PyramidCrystalParam>(c.param_);
  CHECK_DISTRIBUTION(p.h_pyr_u_, DistributionType::kNoRandom, 0.1, 0);
  CHECK_DISTRIBUTION(p.h_pyr_l_, DistributionType::kNoRandom, 0.5, 0);
  CHECK_DISTRIBUTION(p.h_prs_, DistributionType::kNoRandom, 1.2, 0);

  ASSERT_EQ(p.miller_indices_u_[0], 2);
  ASSERT_EQ(p.miller_indices_u_[1], 0);
  ASSERT_EQ(p.miller_indices_u_[2], 3);

  ASSERT_EQ(p.miller_indices_l_[0], 1);
  ASSERT_EQ(p.miller_indices_l_[1], 0);
  ASSERT_EQ(p.miller_indices_l_[2], 1);

  for (const auto& x : p.d_) {
    CHECK_DISTRIBUTION(x, DistributionType::kNoRandom, 1.0f * math::kSqrt3_4, 0.0f);
  }

  const auto& axis = c.axis_;
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
  CHECK_DISTRIBUTION(axis.latitude_dist, DistributionType::kNoRandom, 90, 0);
  CHECK_DISTRIBUTION(axis.azimuth_dist, DistributionType::kUniform, 0, 360);
}

TEST_F(V3TestJson, Filter_None) {
  const auto& j_filter = config_json_.at("filter")[0];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 1);
  ASSERT_TRUE(std::holds_alternative<v3::NoneFilterParam>(f.param_));
  ASSERT_EQ(f.action_, v3::FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, v3::FilterConfig::kSymNone);
}

TEST_F(V3TestJson, Filter_Raypath) {
  const auto& j_filter = config_json_.at("filter")[1];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 2);
  ASSERT_TRUE(std::holds_alternative<v3::RaypathFilterParam>(f.param_));
  ASSERT_EQ(f.action_, v3::FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, v3::FilterConfig::kSymP | v3::FilterConfig::kSymB | v3::FilterConfig::kSymD);

  const auto& p = std::get<v3::RaypathFilterParam>(f.param_);
  std::vector<v3::IdType> expect_raypath = { 3, 1, 5, 7, 4 };
  ASSERT_EQ(p.raypath_, expect_raypath);
}

TEST_F(V3TestJson, Filter_EntryExit) {
  const auto& j_filter = config_json_.at("filter")[2];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 4);
  ASSERT_TRUE(std::holds_alternative<v3::EntryExitFilterParam>(f.param_));
  ASSERT_EQ(f.action_, v3::FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, v3::FilterConfig::kSymNone);

  const auto& p = std::get<v3::EntryExitFilterParam>(f.param_);
  ASSERT_EQ(p.entry_, 3);
  ASSERT_EQ(p.exit_, 5);
}

TEST_F(V3TestJson, Filter_Direction) {
  const auto& j_filter = config_json_.at("filter")[3];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 5);
  ASSERT_TRUE(std::holds_alternative<v3::DirectionFilterParam>(f.param_));
  ASSERT_EQ(f.action_, v3::FilterConfig::kFilterOut);
  ASSERT_EQ(f.symmetry_, v3::FilterConfig::kSymNone);

  const auto& p = std::get<v3::DirectionFilterParam>(f.param_);
  ASSERT_NEAR(p.lat_, 25, 1e-5);
  ASSERT_NEAR(p.lon_, 180, 1e-5);
  ASSERT_NEAR(p.radii_, 0.5, 1e-5);
}

TEST_F(V3TestJson, Filter_Crystal) {
  const auto& j_filter = config_json_.at("filter")[4];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 6);
  ASSERT_TRUE(std::holds_alternative<v3::CrystalFilterParam>(f.param_));
  ASSERT_EQ(f.action_, v3::FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, v3::FilterConfig::kSymNone);

  const auto& p = std::get<v3::CrystalFilterParam>(f.param_);
  ASSERT_EQ(p.crystal_id_, 3);
}

TEST_F(V3TestJson, Filter_Complex) {
  const auto& j_filter = config_json_.at("filter")[5];
  auto f = j_filter.get<v3::FilterConfig>();

  ASSERT_EQ(f.id, 7);
  ASSERT_TRUE(std::holds_alternative<v3::ComplexFilterParam>(f.param_));
  ASSERT_EQ(f.action_, v3::FilterConfig::kFilterIn);
  ASSERT_EQ(f.symmetry_, v3::FilterConfig::kSymNone);

  const auto& p = std::get<v3::ComplexFilterParam>(f.param_);
  std::vector<std::vector<v3::IdType>> expect_composition = { { 1 }, { 3, 6 }, { 5 } };
  ASSERT_EQ(p.filters_, expect_composition);
}

}  // namespace
