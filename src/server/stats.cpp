#include "server/stats.hpp"

#include "core/def.hpp"

namespace lumice {

void StatsConsumer::Consume(const SimData& data) {
  for (const auto& r : data.rays_) {
    if (r.prev_ray_idx_ == kInfSize) {
      sim_rays_++;
    }
  }
  total_rays_ += data.rays_.size_;
  crystals_ += data.crystals_.size();
}

void StatsConsumer::PrepareSnapshot() {
  snapshot_total_rays_ = total_rays_;
  snapshot_sim_rays_ = sim_rays_;
  snapshot_crystals_ = crystals_;
}

Result StatsConsumer::GetResult() const {
  return StatsResult{ snapshot_total_rays_, snapshot_sim_rays_, snapshot_crystals_ };
}

void StatsConsumer::Reset() {
  total_rays_ = 0;
  sim_rays_ = 0;
  crystals_ = 0;
  snapshot_total_rays_ = 0;
  snapshot_sim_rays_ = 0;
  snapshot_crystals_ = 0;
}

}  // namespace lumice
