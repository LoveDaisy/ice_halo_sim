#ifndef CONSUMER_SHOW_RAYS_H_
#define CONSUMER_SHOW_RAYS_H_

#include "consumer/consumer.hpp"

namespace icehalo {
namespace v3 {

class ShowRaysInfo : public IConsume {
 public:
  void Consume(const SimData& data) override;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_SHOW_RAYS_H_
