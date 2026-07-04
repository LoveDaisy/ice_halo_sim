// Shared helpers for CUDA backend tests (parity-cross-backend, hi-wire injection
// scenes, rich-exit contract). Mirrors test/cpu_test_helpers.hpp and
// test/metal_test_helpers.hpp for the CUDA vantage point.
//
// The file body is guarded by `LUMICE_CUDA_ENABLED`; on hosts without CUDA it
// contributes zero symbols so shared parity_test binaries stay clean.
//
// Everything is `inline` so the header can be included from multiple TUs; no
// out-of-line definitions to avoid duplicate-symbol links.

#ifndef LUMICE_TEST_CUDA_TEST_HELPERS_H_
#define LUMICE_TEST_CUDA_TEST_HELPERS_H_

#if defined(LUMICE_CUDA_ENABLED)

#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"

namespace lumice {
namespace cuda_test {

// Fisheye-equal-area zenith render (64x64, fov=180) — the default frame used
// by MakePrismScene's rich-exit + crystal-count tests.
inline RenderConfig MakeRenderConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 64;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  return cfg;
}

// Full-sphere rectangular render — keeps every exit direction in field so the
// device-fused XYZ image responds to orientation (a narrow fisheye culls most
// random-orientation exits, collapsing the image toward constant).
inline RenderConfig MakeFullViewRender() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kRectangular;
  cfg.lens_.fov_ = 360.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 32;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kFull;
  return cfg;
}

// Deterministic prism scene (matches test_cpu_trace_backend.cpp::MakeSimpleScene
// shape so the geometry — unit prism, 8 polygon faces: 2 basal + 6 sides — is
// stable across the parity layer).
inline SceneConfig MakePrismScene(size_t max_hits) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  MsInfo ms;
  ms.prob_ = 0.0f;  // single MS final layer
  ScatteringSetting s;
  // Value-initialize the filter (NoneFilterParam pass-through). Without this the
  // POD members FilterConfig::symmetry_/action_/id_ stay INDETERMINATE (a bare
  // `ScatteringSetting s;` default-inits them), and the garbage action_/symmetry_
  // make the device emit gate's DeviceFilterCheck reject every exit → no XYZ
  // accumulation (mirrors test/cpu_test_helpers.hpp which sets this explicitly).
  s.filter_ = FilterConfig{};
  s.crystal_.id_ = 0;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.param_ = prism;
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

// Single-crystal random-axis prism scene. Random orientation makes the gen
// stream observable (different mixed_seed → different rotation). `final_prob`
// controls the final-layer emit-gate keep fraction: > 0 makes the gate stream
// observable via the device-fused image. `filter_` is value-initialized
// (pass-through) so the device emit gate does not reject every exit.
inline SceneConfig MakeRandomAxisScene(size_t max_hits, float final_prob) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  MsInfo ms;
  ms.prob_ = final_prob;
  ScatteringSetting s;
  s.filter_ = FilterConfig{};
  s.crystal_.id_ = 0;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  s.crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  s.crystal_.param_ = prism;
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

// Two-MS-layer scene tuned so the transit stream is cleanly OBSERVABLE at
// layer-1's d_dirs_ without the atomic-compaction confound:
//   - Point sun (diameter 0) + FIXED-axis layer 0 → every layer-0 ray is
//     identical → every continuation ray carries the SAME world direction. So
//     layer-1's d_dirs_[tid] = R1(tid)^-1 · const depends only on tid (the
//     transit orientation), independent of which continuation ray landed at
//     that tid → deterministic for a fixed (seed, transit_base).
//   - RANDOM-axis layer 1 → the transit kernel samples a per-tid orientation
//     R1(tid) from the transit PCG stream, so a non-zero transit hi moves it.
// Layer 0 continues (prob 0.6) into layer 1 (final, prob 0).
inline SceneConfig MakeTwoLayerScene(size_t max_hits) {
  SceneConfig scene = MakeRandomAxisScene(max_hits, /*final_prob=*/0.0f);  // layer-1 template (random axis)
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.0f };              // point sun (diameter 0)
  MsInfo layer0 = scene.ms_[0];
  layer0.prob_ = 0.6f;
  layer0.setting_[0].crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kNoRandom, 0.0f, 0.0f };
  layer0.setting_[0].crystal_.axis_.latitude_dist = Distribution{ DistributionType::kNoRandom, 20.0f, 0.0f };
  scene.ms_.insert(scene.ms_.begin(), std::move(layer0));
  return scene;
}

}  // namespace cuda_test
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)

#endif  // LUMICE_TEST_CUDA_TEST_HELPERS_H_
