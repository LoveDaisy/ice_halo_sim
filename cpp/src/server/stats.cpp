#include "server/stats.hpp"

#include <set>

#include "core/def.hpp"

namespace icehalo {
namespace v3 {

void StatsConsumer::Consume(const SimData& data) {
  for (const auto& r : data.rays_) {
    if (r.prev_ray_idx_ == kInfSize) {
      sim_rays_++;
    }
  }
  total_rays_ += data.rays_.size_;
  crystals_ += data.crystals_.size();
}

Result StatsConsumer::GetResult() const {
  return StatsResult{ total_rays_, sim_rays_, crystals_ };
}

}  // namespace v3
}  // namespace icehalo
