#ifndef CONSUMER_SHOW_RAYS_H_
#define CONSUMER_SHOW_RAYS_H_

#include "server/consumer.hpp"

namespace icehalo {

class ShowRayInfoConsumer : public IConsume {
 public:
  void Consume(const SimData& data) override;
};

}  // namespace icehalo

#endif  // CONSUMER_SHOW_RAYS_H_
