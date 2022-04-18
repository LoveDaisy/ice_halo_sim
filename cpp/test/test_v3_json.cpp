#include <gtest/gtest.h>

#include <fstream>
#include <variant>

#include "core/math.hpp"
#include "json.hpp"
#include "protocol/crystal_config.hpp"

extern std::string config_file_name;

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
  auto d = j_height.get<icehalo::Distribution>();

  ASSERT_EQ(d.type, icehalo::DistributionType::kGaussian);
  ASSERT_NEAR(d.mean, 1.3, 1e-5);
  ASSERT_NEAR(d.std, 0.2, 1e-5);
}

TEST_F(V3TestJson, Crystal) {
	const auto& j_crystal = config_json_.at("crystal")[1];
	auto c = j_crystal.get<icehalo::v3::CrystalConfig>();

	ASSERT_EQ(c.id_, 2);
}

}  // namespace
