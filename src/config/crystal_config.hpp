#ifndef CONFIG_CRYSTAL_CONFIG_H_
#define CONFIG_CRYSTAL_CONFIG_H_

#include <memory>
#include <nlohmann/json.hpp>
#include <variant>

#include "core/math.hpp"

namespace lumice {

struct PrismCrystalParam {
  Distribution h_{ DistributionType::kNoRandom, 1.0f, 0.0f };  // Height, equal to c/a in HP2.0
  Distribution d_[6]{};                                        // Distance to center for prism faces
};

struct PyramidCrystalParam {
  Distribution h_prs_{};                                             // Prism height
  Distribution h_pyr_u_{ DistributionType::kNoRandom, 0.0f, 0.0f };  // Upper pyramidal relative height, from 0.0 to 1.0
  Distribution h_pyr_l_{ DistributionType::kNoRandom, 0.0f, 0.0f };  // Lower pyramidal relative height, from 0.0 to 1.0
  Distribution d_[6]{};                                              // Distance to center for prism faces
  float wedge_angle_u_ = 28.0f;  // Upper wedge angle (degrees). Default ≈ atan(√3/2 / 1.629), i.e. Miller {1,0,-1,1}
  float wedge_angle_l_ = 28.0f;  // Lower wedge angle (degrees)
};

using CrystalParam = std::variant<PrismCrystalParam, PyramidCrystalParam>;

struct CrystalConfig {
  IdType id_;
  CrystalParam param_;
  AxisDistribution axis_;
};

using CrystalConfigPtrU = std::unique_ptr<CrystalConfig>;
using CrystalConfigPtrS = std::shared_ptr<CrystalConfig>;

// convert to & from json object
void to_json(nlohmann::json& j, const CrystalConfig& c);
void from_json(const nlohmann::json& j, CrystalConfig& c);

}  // namespace lumice

#endif  // CONFIG_CRYSTAL_CONFIG_H_
