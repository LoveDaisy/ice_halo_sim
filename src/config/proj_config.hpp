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

// See doc/filter-architecture.md §1 (filter ↔ crystal single-key binding)
struct ScatteringSetting {
  FilterConfig filter_;
  CrystalConfig crystal_;
  float crystal_proportion_;
  // Ray-allocation weight q_i for this (layer, entry) — the share of the layer's rays this
  // entry is DEALT, as opposed to crystal_proportion_ (p_i), the share of the layer's ENERGY
  // it carries. Under SceneConfig::kProportional the two are the same knob and this field is
  // never read. Under kAdaptive a pilot pass (Neyman: q ∝ p·√E[e²]) fills it, the partition
  // deals rays by q, and every ray born into the entry has its weight scaled by
  // (p_i/ΣP)/(q_i/ΣQ) so the expected image is unchanged and only the variance moves.
  // Engine-internal: not a JSON key, not compared by ConfigSnapshot, not GUI-visible. The
  // -1 sentinel means "not delivered": a layer with ANY undelivered entry deals by p (see
  // ResolveLayerRayAllocation in core/simulator.hpp for why the fallback is per layer).
  float crystal_ray_alloc_weight_ = -1.0f;
};

struct MsInfo {
  float prob_;
  std::vector<ScatteringSetting> setting_;
};

struct SceneConfig {
  // How a layer's rays are dealt across its entries. kProportional (default) deals by
  // crystal_proportion_ — sampling share and energy share are one knob, which is Neyman
  // allocation only when every entry's per-ray energy statistics agree. kAdaptive deals by
  // ScatteringSetting::crystal_ray_alloc_weight_ (q_i) with a per-entry weight correction,
  // so an entry whose rays each carry far more energy (a filtered high-energy raypath at a
  // small proportion) gets the sample count its variance calls for. Opt-in on purpose: the
  // default must keep every existing config bit-identical.
  enum class RayAllocationMode { kProportional, kAdaptive };

  // Total rays across the whole spectrum, not per wavelength: ServerImpl::GenerateScene
  // derives the per-wavelength budget from this with PerWavelengthRayNum (ceil(total/n_wl),
  // see server/ray_num_semantics.hpp). kInfSize is the "run until stopped" sentinel and is
  // passed through undivided.
  size_t ray_num_;
  size_t max_hits_;
  // GPU K-shape pool clock (0 = disabled/opt-in default). Affects Metal/CUDA GPU
  // backends only; CPU-legacy uses its own LUMICE_GEOM_CLOCK. Unlike ray_num_ /
  // max_hits_ (consumed by all three backends), this field is read only by the
  // two GPU backends -- it is NOT a globally-effective simulation parameter.
  size_t geom_clock_ = 0;
  RayAllocationMode ray_allocation_ = RayAllocationMode::kProportional;
  LightSourceConfig light_source_;
  std::vector<MsInfo> ms_;  // (prob, [scattering_info, ...])
};

using SceneConfigPtrU = std::unique_ptr<SceneConfig>;
using SceneConfigPtrS = std::shared_ptr<SceneConfig>;

// kProportional FIRST on purpose: NLOHMANN_JSON_SERIALIZE_ENUM maps any unrecognized string
// to the first table entry, so a misspelled value lands on the same mode a MISSING key does —
// the safe default, and the parser warns about it (config_manager.cpp).
NLOHMANN_JSON_SERIALIZE_ENUM(        // declare
    SceneConfig::RayAllocationMode,  // type
    {
        { SceneConfig::RayAllocationMode::kProportional, "proportional" },
        { SceneConfig::RayAllocationMode::kAdaptive, "adaptive" },
    })

void to_json(nlohmann::json& j, const SceneConfig& s);

}  // namespace lumice

#endif  // CONFIG_PROJ_CONFIG_H_
