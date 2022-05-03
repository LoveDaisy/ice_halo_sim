#ifndef CONSUMER_STATS_H_
#define CONSUMER_STATS_H_

#include <cstddef>

#include "consumer/consumer.hpp"
#include "include/result.hpp"

namespace icehalo {
namespace v3 {

class Stats : public IConsume {
 public:
  void Consume(const SimData& data) override;
  Result GetResult() const override;

 private:
  size_t total_rays_ = 0;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_STATS_H_
