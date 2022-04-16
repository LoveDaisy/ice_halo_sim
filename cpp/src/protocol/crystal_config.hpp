#ifndef SRC_CONTEXT_CRYSTAL_CONFIG_H_
#define SRC_CONTEXT_CRYSTAL_CONFIG_H_

#include <memory>
#include <variant>

#include "core/math.hpp"

namespace icehalo {
namespace v3 {

struct PrismCrystalParam {
  Distribution h_;     // Height, equal to c/a in HP2.0
  Distribution d_[6];  // Distance to center for prism faces
};

struct PyramidCrystalParam {
  Distribution h_prs_;    // Prism height
  Distribution h_pyr_u_;  // Upper pyramidal relative height, from 0.0 to 1.0
  Distribution h_pyr_l_;  // Lower pyramidal relative height, from 0.0 to 1.0
  Distribution d_[6];     // Distance to center for prism faces
};

using CrystalParam = std::variant<PrismCrystalParam, PyramidCrystalParam>;


/**
 * @brief It is a part of Simulation Backend Protocol
 *
 */
struct CrystalConfig {
  int id_;
  CrystalParam param_;
  AxisDistribution axis_;
  SpaceRange range_;
};

using CrystalConfigPtrU = std::unique_ptr<CrystalConfig>;
using CrystalConfigPtrS = std::shared_ptr<CrystalConfig>;

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_CONTEXT_CRYSTAL_CONFIG_H_