#ifndef CONFIG_PROJ_CONFIG_H_
#define CONFIG_PROJ_CONFIG_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

struct ScatteringSetting {
  FilterConfig filter_;
  CrystalConfig crystal_;
  float crystal_proportion_;
};

struct MsInfo {
  float prob_;
  std::vector<ScatteringSetting> setting_;
};

struct SceneConfig {
  IdType id_;
  size_t ray_num_;  // For every single wavelength.
  size_t max_hits_;
  LightSourceConfig light_source_;
  std::vector<MsInfo> ms_;  // (prob, [scattering_info, ...])
};

using SceneConfigPtrU = std::unique_ptr<SceneConfig>;
using SceneConfigPtrS = std::shared_ptr<SceneConfig>;

void to_json(nlohmann::json& j, const SceneConfig& s);

// =============== Project configuration ===============
struct ProjConfig {
  IdType id_;
  SceneConfig scene_;                    // One scene for one project.
  std::vector<RenderConfig> renderers_;  // One project may have multipile renderer.
};

using ProjConfigPtrU = std::unique_ptr<ProjConfig>;
using ProjConfigPtrS = std::shared_ptr<ProjConfig>;

void to_json(nlohmann::json& j, const ProjConfig& p);

}  // namespace v3
}  // namespace icehalo

#endif  // CONFIG_PROJ_CONFIG_H_
