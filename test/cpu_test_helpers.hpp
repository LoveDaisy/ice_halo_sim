// Shared helpers for CPU backend tests (golden-analytic anchors, parity-cross-
// backend CPU-side fixtures). Cross-platform: no Metal/Apple guards.
//
// Mirrors test/metal_test_helpers.hpp's MakeMetalScene / MakeRectangularRender
// shapes so the CPU golden-ray anchors can borrow the same configuration that
// the Metal anchors already validate (cuda-backend-mvp subtask 1).
//
// Everything is `inline` so the header is safe to include from multiple TUs;
// no out-of-line definitions to avoid duplicate-symbol links.

#ifndef LUMICE_TEST_CPU_TEST_HELPERS_H_
#define LUMICE_TEST_CPU_TEST_HELPERS_H_

#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"

namespace lumice {
namespace cpu_test {

inline RenderConfig MakeRectangularRender() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kRectangular;
  cfg.lens_.fov_ = 360.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 32;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  return cfg;
}

// Single-crystal prism scene with `ms_layers` MS layers (last layer prob=0,
// earlier layers prob=0.6). Mirrors MakeMetalScene's wire shape so anchors
// derived for the Metal kernel reuse the same crystal geometry.
inline SceneConfig MakeCpuScene(size_t max_hits, size_t ms_layers) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  for (size_t mi = 0; mi < ms_layers; mi++) {
    MsInfo ms;
    ms.prob_ = (mi + 1 < ms_layers) ? 0.6f : 0.0f;
    ScatteringSetting s;
    s.crystal_.id_ = static_cast<IdType>(mi);
    PrismCrystalParam prism;
    prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    for (auto& d : prism.d_) {
      d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    }
    s.crystal_.param_ = prism;
    // Value-init the filter so its POD members (id_, symmetry_) are zeroed
    // rather than left as stack residue (same caveat as MakeMetalScene).
    s.filter_ = FilterConfig{};
    s.crystal_proportion_ = 1.0f;
    ms.setting_.push_back(std::move(s));
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

}  // namespace cpu_test
}  // namespace lumice

#endif  // LUMICE_TEST_CPU_TEST_HELPERS_H_
