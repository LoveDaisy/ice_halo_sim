#include "server/stats.hpp"

#include "core/def.hpp"

namespace lumice {

void StatsConsumer::Consume(const SimData& data) {
  sim_rays_ += data.root_ray_count_;
  total_rays_ += data.rays_.size_;
  // The two halves of the crystal-geometry count aggregate differently, and
  // conflating them is what made this stat scale with the dispatch grain and
  // the worker-pool size. Stochastic draws are genuinely distinct geometries
  // per batch and per worker, so they SUM; the deterministic population is a
  // property of the committed scene that every batch and every worker reports
  // identically, so it is OVERWRITTEN (same discipline as the GPU
  // color-degrade tally). See SimData for the field-level contract.
  stochastic_crystal_samples_ += data.stochastic_crystal_sample_count_;
  deterministic_crystals_ = data.deterministic_crystal_count_;
}

void StatsConsumer::PrepareSnapshot() {
  snapshot_total_rays_ = total_rays_;
  snapshot_sim_rays_ = sim_rays_;
  snapshot_crystals_ = deterministic_crystals_ + stochastic_crystal_samples_;
}

Result StatsConsumer::GetResult() const {
  return StatsResult{ snapshot_total_rays_, snapshot_sim_rays_, snapshot_crystals_ };
}

void StatsConsumer::Reset() {
  total_rays_ = 0;
  sim_rays_ = 0;
  stochastic_crystal_samples_ = 0;
  deterministic_crystals_ = 0;
  snapshot_total_rays_ = 0;
  snapshot_sim_rays_ = 0;
  snapshot_crystals_ = 0;
}

}  // namespace lumice
