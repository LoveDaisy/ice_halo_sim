#ifndef SRC_CONTEXT_BACKEND_CONFIG_H_
#define SRC_CONTEXT_BACKEND_CONFIG_H_

#include <cstddef>
#include <map>
#include <memory>
#include <vector>

#include "core/core_def.hpp"
#include "protocol/crystal_config.hpp"
#include "protocol/filter_config.hpp"
#include "protocol/light_config.hpp"
#include "protocol/render_config.hpp"

/*
 * We define some public protocols (accessable outside the lib) here.
 * And also some of them can be used in simulator and renderer queues (internal usage).
 */

namespace icehalo {
namespace v3 {

struct ScatteringSetting {
  FilterConfig filter_id_;
  CrystalConfig crystal_id_;
  float crystal_proportion_;
};

struct MsInfo {
  float prob_;
  std::vector<ScatteringSetting> setting_;
};

struct SceneConfig {
  IdType id_;
  size_t ray_num_;          // For every single wavelength.
  std::vector<MsInfo> ms_;  // (prob, [scattering_info, ...])
};

struct ProjConfig {
  SceneConfig scene_;                  // One scene for one project.
  std::vector<RenderConfig> renders_;  // One project may have multipile renderer.
};

class ConfigManager {
 public:
  static ConfigManager& GetInstance();

  ConfigManager& AddLightSource(IdType id, LightSourceConfig config);
  ConfigManager& AddCrystal(IdType id, CrystalConfig config);
  ConfigManager& AddFilter(IdType id, FilterConfig config);
  ConfigManager& AddRenderer(IdType id, RenderConfig config);

  LightSourceConfig GetLightSource(IdType id);
  FilterConfig GetFilter(IdType id);
  CrystalConfig GetCrystal(IdType id);
  RenderConfig GetRenderer(IdType id);

 private:
  ConfigManager() {}

  std::map<IdType, LightSourceConfig> light_sources_;
  std::map<IdType, CrystalConfig> crystals_;
  std::map<IdType, FilterConfig> filters_;
  std::map<IdType, RenderConfig> renderers_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_CONTEXT_BACKEND_CONFIG_H_
