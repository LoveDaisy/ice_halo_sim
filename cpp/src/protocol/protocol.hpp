#ifndef SRC_CONTEXT_BACKEND_CONFIG_H_
#define SRC_CONTEXT_BACKEND_CONFIG_H_

#include <cstddef>
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
  LightSourceConfig light_source_;
  size_t ray_num_;  // For every single wavelength.
  size_t max_hits_;
  std::vector<MsInfo> ms_;  // (prob, [scattering_info, ...])
};

struct ProjConfig {
  SceneConfig scene_;                  // One scene for one project.
  std::vector<RenderConfig> renders_;  // One project may have multipile renderer.
};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_CONTEXT_BACKEND_CONFIG_H_
