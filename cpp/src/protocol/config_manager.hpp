#ifndef PROTOCOL_CONFIG_MANAGER_H_
#define PROTOCOL_CONFIG_MANAGER_H_

#include <map>

#include "core/core_def.hpp"
#include "json.hpp"
#include "protocol/crystal_config.hpp"
#include "protocol/filter_config.hpp"
#include "protocol/light_config.hpp"
#include "protocol/proj_config.hpp"
#include "protocol/render_config.hpp"

namespace icehalo {
namespace v3 {

struct ConfigManager {
  std::map<IdType, LightSourceConfig> lights_;
  std::map<IdType, CrystalConfig> crystals_;
  std::map<IdType, FilterConfig> filters_;
  std::map<IdType, RenderConfig> renderers_;
  std::map<IdType, SceneConfig> scenes_;
  std::map<IdType, ProjConfig> projects_;
};

// convert to/from json object
void to_json(nlohmann::json& j, const ConfigManager& m);
void from_json(const nlohmann::json& j, ConfigManager& m);

}  // namespace v3
}  // namespace icehalo

#endif  // PROTOCOL_CONFIG_MANAGER_H_
