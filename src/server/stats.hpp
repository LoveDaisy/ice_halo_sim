#ifndef CONSUMER_STATS_H_
#define CONSUMER_STATS_H_

#include <cstddef>

#include "server/consumer.hpp"
#include "server/server.hpp"

namespace lumice {

class StatsConsumer : public IConsume {
 public:
  void Consume(const SimData& data) override;
  void PrepareSnapshot() override;
  Result GetResult() const override;
  void Reset() override;

  // Live (un-snapshotted) accumulated sim ray count. Cheap O(1) read of the
  // running counter for progress polling (e.g. the --benchmark drain loop),
  // which needs sim_ray_num every iteration but NOT a rendered snapshot.
  // task-317: the drain-count benchmark poll used GetStatsResult(), which
  // unconditionally triggers DoSnapshot -> RenderConsumer sRGB (powf/pixel),
  // dominating wall-time and starving drain-window closure. Callers must hold
  // consumer_mutex_ (Consume() mutates sim_rays_ under it).
  size_t LiveSimRays() const { return sim_rays_; }

 private:
  size_t total_rays_ = 0;
  size_t sim_rays_ = 0;
  size_t crystals_ = 0;
  size_t snapshot_total_rays_ = 0;
  size_t snapshot_sim_rays_ = 0;
  size_t snapshot_crystals_ = 0;
};

}  // namespace lumice

#endif  // CONSUMER_STATS_H_
