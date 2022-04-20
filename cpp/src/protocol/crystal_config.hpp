#ifndef SRC_CONTEXT_CRYSTAL_CONFIG_H_
#define SRC_CONTEXT_CRYSTAL_CONFIG_H_

#include <memory>
#include <variant>

#include "core/core_def.hpp"
#include "core/math.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

struct PrismCrystalParam {
  Distribution h_;     // Height, equal to c/a in HP2.0
  Distribution d_[6];  // Distance to center for prism faces
};

struct PyramidCrystalParam {
  Distribution h_prs_;       // Prism height
  Distribution h_pyr_u_;     // Upper pyramidal relative height, from 0.0 to 1.0
  Distribution h_pyr_l_;     // Lower pyramidal relative height, from 0.0 to 1.0
  Distribution d_[6];        // Distance to center for prism faces
  int miller_indices_u_[3];  // Miller indices, c1: -30 dgree, c2: 90 degree, c3: -150 degree
  int miller_indices_l_[3];  // Miller indices, c1: -30 dgree, c2: 90 degree, c3: -150 degree
  // Miller indices (i1, i2, -(i1+i2), i3)
  // c1: -30 dgree, c2: 90 degree, c3: -150 degree
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

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_CONTEXT_CRYSTAL_CONFIG_H_
