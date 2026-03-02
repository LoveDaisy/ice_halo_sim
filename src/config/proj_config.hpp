#ifndef CONFIG_PROJ_CONFIG_H_
#define CONFIG_PROJ_CONFIG_H_

#include <cstddef>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"

namespace lumice {

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
  size_t ray_num_;  // For every single wavelength.
  size_t max_hits_;
  LightSourceConfig light_source_;
  std::vector<MsInfo> ms_;  // (prob, [scattering_info, ...])
};

using SceneConfigPtrU = std::unique_ptr<SceneConfig>;
using SceneConfigPtrS = std::shared_ptr<SceneConfig>;

void to_json(nlohmann::json& j, const SceneConfig& s);

}  // namespace lumice

#endif  // CONFIG_PROJ_CONFIG_H_
