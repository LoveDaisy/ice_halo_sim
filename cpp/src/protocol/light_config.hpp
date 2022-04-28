#ifndef SRC_CONTEXT_LIGHT_CONFIG_H_
#define SRC_CONTEXT_LIGHT_CONFIG_H_

#include <variant>
#include <vector>

#include "core/def.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

struct SunParam {
  float altitude_;  // Degree
  float azimuth_;   // Degree
  float diameter_;  // Degree
};

struct StreetLightParam {
  float distace_;   // meter
  float azimuth_;   // degree
  float height_;    // meter
  float diameter_;  // meter. Treat steet light as a sphere (NOT a ball)
};

using LightSourceParam = std::variant<SunParam, StreetLightParam>;

struct WlParam {
  float wl_;
  float weight_;
};

struct LightSourceConfig {
  IdType id_;
  LightSourceParam param_;
  std::vector<WlParam> wl_param_;
};

// convert to/from json object
void to_json(nlohmann::json& j, const LightSourceConfig& l);
void from_json(const nlohmann::json& j, LightSourceConfig& l);

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_CONTEXT_LIGHT_CONFIG_H_
