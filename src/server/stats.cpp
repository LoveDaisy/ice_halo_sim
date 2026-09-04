#include "server/stats.hpp"

#include "core/def.hpp"

namespace lumice {

void StatsConsumer::Consume(const SimData& data) {
  sim_rays_ += data.root_ray_count_;
  total_rays_ += data.ray_seg_count_;
  // The two halves of the crystal-geometry count aggregate differently, and
  // conflating them is what made this stat scale with the dispatch grain and
  // the worker-pool size. Stochastic draws are genuinely distinct geometries
  // per batch and per worker, so they SUM; the deterministic population is a
  // property of the committed scene that every batch and every worker reports
  // identically, so it is OVERWRITTEN (same discipline as the GPU
  // color-degrade tally). See SimData for the field-level contract.
  stochastic_crystal_samples_ += data.stochastic_crystal_sample_count_;
  deterministic_crystals_ = data.deterministic_crystal_count_;
  // Orientation count: identical split for identical reasons. Keeping the two
  // statistics on separate accumulators is the point — a scene of fixed shapes
  // under random axes reports a small crystal count and a large orientation one.
  stochastic_orientation_samples_ += data.stochastic_orientation_sample_count_;
  deterministic_orientations_ = data.deterministic_orientation_count_;
}

void StatsConsumer::PrepareSnapshot() {
  snapshot_total_rays_ = total_rays_;
  snapshot_sim_rays_ = sim_rays_;
  snapshot_crystals_ = deterministic_crystals_ + stochastic_crystal_samples_;
  snapshot_orientations_ = deterministic_orientations_ + stochastic_orientation_samples_;
}

Result StatsConsumer::GetResult() const {
  return StatsResult{ snapshot_total_rays_, snapshot_sim_rays_, snapshot_crystals_, snapshot_orientations_ };
}

void StatsConsumer::Reset() {
  total_rays_ = 0;
  sim_rays_ = 0;
  stochastic_crystal_samples_ = 0;
  deterministic_crystals_ = 0;
  stochastic_orientation_samples_ = 0;
  deterministic_orientations_ = 0;
  snapshot_total_rays_ = 0;
  snapshot_sim_rays_ = 0;
  snapshot_crystals_ = 0;
  snapshot_orientations_ = 0;
}

}  // namespace lumice
