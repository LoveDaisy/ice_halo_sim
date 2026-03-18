#ifndef CONSUMER_STATS_H_
#define CONSUMER_STATS_H_

#include <cstddef>

#include "server/consumer.hpp"
#include "server/server.hpp"

namespace lumice {

class StatsConsumer : public IConsume {
 public:
  void Consume(const SimData& data) override;
  void ResetAccumulation() override;
  void PrepareSnapshot() override;
  Result GetResult() const override;

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
