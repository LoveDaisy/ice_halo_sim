// Shared helpers for Metal backend tests. Both test_metal_trace_backend.cpp
// (single/two-layer regression) and test_metal_trace_parity.cpp (parity
// harness) consume this header.
//
// Everything here is `inline` so the header is safe to include from multiple
// translation units; no out-of-line definitions to avoid duplicate-symbol
// links.

#ifndef LUMICE_TEST_METAL_TEST_HELPERS_H_
#define LUMICE_TEST_METAL_TEST_HELPERS_H_

#if defined(__APPLE__)

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"

namespace lumice {
namespace metal_test {

inline bool ShouldSkipMetalTests() {
  const char* env = std::getenv("LUMICE_SKIP_METAL_TESTS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

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

// Dual-fisheye equal-area RenderConfig matching the GUI's mandatory wire
// format (c_api.cpp:295: fov180 / full-globe / el=0). `overlap` mirrors
// kDualFisheyeOverlap on the GUI side (set 0 for the no-overlap variant).
inline RenderConfig MakeDualFisheyeEARender(float overlap = 0.0f) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kDualFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 128;  // wide enough for left+right circle layout
  cfg.resolution_[1] = 64;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 0.0f;  // GUI sets el=0 for dual-fisheye (full-globe; unused)
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kFull;
  cfg.overlap_ = overlap;
  return cfg;
}

inline SceneConfig MakeMetalScene(size_t max_hits, size_t ms_layers) {
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
    // rather than left as stack residue — uninitialized symmetry_ leaks into
    // FilterSpec::Create as a bit-pattern that can reject every ray.
    s.filter_ = FilterConfig{};
    s.crystal_proportion_ = 1.0f;
    ms.setting_.push_back(std::move(s));
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

// scrum-258.3 Step 6: per-layer filter+prob tests need explicit prob control.
// Uniform prob across all layers — for prob=0/1 boundary tests and prob=0.5
// split direction tests. Crystal/filter setup mirrors MakeMetalScene so the
// existing fast e2e + parity harness stays compatible.
inline SceneConfig MakeMetalSceneWithProb(size_t max_hits, size_t ms_layers,
                                          float prob) {
  SceneConfig scene = MakeMetalScene(max_hits, ms_layers);
  for (auto& ms : scene.ms_) {
    ms.prob_ = prob;
  }
  return scene;
}

// Multi-crystal MS scene for backend-multi-crystal-per-layer parity tests.
// Each MS layer carries `crystal_count` populations with proportion 0.7 / 0.3
// (non-uniform — exercises PartitionCrystalRayNum's largest-remainder path).
// All crystals are deterministic prisms (h=1.0) so MakeCrystal does not
// consume RNG and the oracle's RNG mirror can stay aligned with the backend.
inline SceneConfig MakeMultiCrystalScene(size_t max_hits, size_t ms_layers, size_t crystal_count) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  // Fixed proportions: 0.7 / 0.3 for crystal_count=2. For larger crystal_count
  // (uncommon in this task), spread the residual evenly among the tail.
  std::vector<float> proportions;
  proportions.reserve(crystal_count);
  if (crystal_count == 2) {
    proportions.push_back(0.7f);
    proportions.push_back(0.3f);
  } else {
    float head = 0.7f;
    float tail = (1.0f - head) / static_cast<float>(crystal_count - 1);
    proportions.push_back(head);
    for (size_t i = 1; i < crystal_count; i++) {
      proportions.push_back(tail);
    }
  }

  for (size_t mi = 0; mi < ms_layers; mi++) {
    MsInfo ms;
    ms.prob_ = (mi + 1 < ms_layers) ? 0.6f : 0.0f;
    for (size_t ci = 0; ci < crystal_count; ci++) {
      ScatteringSetting s;
      s.crystal_.id_ = static_cast<IdType>(mi * 10 + ci);
      PrismCrystalParam prism;
      prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
      for (auto& d : prism.d_) {
        d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
      }
      s.crystal_.param_ = prism;
      s.filter_ = FilterConfig{};
      s.crystal_proportion_ = proportions[ci];
      ms.setting_.push_back(std::move(s));
    }
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

inline double ChannelSum(const std::vector<float>& xyz, int channel) {
  double s = 0.0;
  for (size_t i = channel; i < xyz.size(); i += 3) {
    s += static_cast<double>(xyz[i]);
  }
  return s;
}

inline double RelErr(double a, double b) {
  double ref = std::max(std::abs(a), std::abs(b));
  if (ref == 0.0) {
    return 0.0;
  }
  return std::abs(a - b) / ref;
}

}  // namespace metal_test
}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // LUMICE_TEST_METAL_TEST_HELPERS_H_
