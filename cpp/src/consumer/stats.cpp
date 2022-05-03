#include "consumer/stats.hpp"
#include <set>

namespace icehalo {
namespace v3 {

void Stats::Consume(const SimData& data) {
  std::set<size_t> idx_set;
  for (const auto& r : data.rays_) {
    idx_set.emplace(r.root_ray_idx_);
  }
  total_rays_ += idx_set.size();
}

Result Stats::GetResult() const {
  return StatsResult{ total_rays_ };
}

}  // namespace v3
}  // namespace icehalo
