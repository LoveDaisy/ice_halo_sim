#ifndef CONFIG_CONFIG_MANAGER_H_
#define CONFIG_CONFIG_MANAGER_H_

#include <map>
#include <nlohmann/json.hpp>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"

namespace lumice {

struct ConfigManager {
  std::map<IdType, CrystalConfig> crystals_;
  std::map<IdType, FilterConfig> filters_;
  std::map<IdType, RenderConfig> renderers_;
  SceneConfig scene_;
};

// convert to/from json object
void to_json(nlohmann::json& j, const ConfigManager& m);
void from_json(const nlohmann::json& j, ConfigManager& m);

}  // namespace lumice

#endif  // CONFIG_CONFIG_MANAGER_H_
