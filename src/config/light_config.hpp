#ifndef CONFIG_LIGHT_CONFIG_H_
#define CONFIG_LIGHT_CONFIG_H_

#include <nlohmann/json.hpp>
#include <variant>
#include <vector>

#include "core/def.hpp"
#include "util/illuminant_data.hpp"

namespace lumice {

// Every field carries a default so that a partially filled `SunParam` is a well-defined value
// rather than an uninitialized read. The zeros are the same bytes `SunParam p{}` (the JSON
// decoder's starting point) already produced, so no default changed; what changed is that
// `SunParam s; s.altitude_ = x;` no longer leaves `azimuth_` holding whatever the stack held.
// That shape put a NaN into the annotation layer's sun direction, whose normalizer then fell
// back to the zenith and moved a whole family of circles — intermittently, since it depended
// on the previous frame's residue.
struct SunParam {
  float altitude_{};  // Degree
  float azimuth_{};   // Degree
  float diameter_{};  // Degree
};

struct WlParam {
  float wl_;
  float weight_;
};

using SpectrumConfig = std::variant<std::vector<WlParam>, IlluminantType>;

struct LightSourceConfig {
  SunParam param_;
  SpectrumConfig spectrum_;
};

// convert to/from json object
void to_json(nlohmann::json& j, const LightSourceConfig& l);
void from_json(const nlohmann::json& j, LightSourceConfig& l);

}  // namespace lumice

#endif  // CONFIG_LIGHT_CONFIG_H_
