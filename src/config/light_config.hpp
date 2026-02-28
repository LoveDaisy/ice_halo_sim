#ifndef CONFIG_LIGHT_CONFIG_H_
#define CONFIG_LIGHT_CONFIG_H_

#include <nlohmann/json.hpp>
#include <variant>
#include <vector>

#include "core/def.hpp"
#include "util/illuminant_data.hpp"

namespace lumice {

struct SunParam {
  float altitude_;  // Degree
  float azimuth_;   // Degree
  float diameter_;  // Degree
};

struct WlParam {
  float wl_;
  float weight_;
};

using SpectrumConfig = std::variant<std::vector<WlParam>, IlluminantType>;

struct LightSourceConfig {
  IdType id_;
  SunParam param_;
  SpectrumConfig spectrum_;
};

// convert to/from json object
void to_json(nlohmann::json& j, const LightSourceConfig& l);
void from_json(const nlohmann::json& j, LightSourceConfig& l);

}  // namespace lumice

#endif  // CONFIG_LIGHT_CONFIG_H_
