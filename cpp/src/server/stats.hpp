#ifndef CONSUMER_STATS_H_
#define CONSUMER_STATS_H_

#include <cstddef>

#include "include/result.hpp"
#include "server/consumer.hpp"

namespace icehalo {
namespace v3 {

class StatsConsumer : public IConsume {
 public:
  void Consume(const SimData& data) override;
  Result GetResult() const override;

 private:
  size_t total_rays_ = 0;
  size_t sim_rays_ = 0;
  size_t crystals_ = 0;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_STATS_H_
