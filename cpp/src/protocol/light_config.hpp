#ifndef SRC_CONTEXT_LIGHT_CONFIG_H_
#define SRC_CONTEXT_LIGHT_CONFIG_H_

#include <variant>
#include <vector>

#include "core/core_def.hpp"

namespace icehalo {
namespace v3 {

struct SunParam {
  float altitude_;  // Degree
  float diameter_;  // Degree
};

struct StreetLightParam {
  float distace_;   // meter
  float height_;    // meter
  float diameter_;  // meter. Treat steet light as a sphere (NOT a ball)
};

using LightSourceParam = std::variant<SunParam, StreetLightParam>;

struct WlParam {
  float wl_;
  float weight_;
};

struct LightSourceConfig {
  IdType id;
  LightSourceParam param_;
  std::vector<WlParam> wl_param_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_CONTEXT_LIGHT_CONFIG_H_
