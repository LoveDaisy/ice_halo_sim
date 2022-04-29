#include "consumer/consumer.hpp"

#include "util/queue.hpp"

namespace icehalo {
namespace v3 {

Consumer::Consumer(QueuePtrS<SimData> data_queue) : data_queue_(data_queue), stop_(false) {}

void Consumer::RegisterConsumer(ConsumerPtrU consumer) {
  consumers_.emplace_back(std::move(consumer));
}

void Consumer::Run() {
  while (true) {
    auto data = data_queue_->Get();
    if (stop_ || data.rays_.Empty()) {
      break;
    }

    for (auto& c : consumers_) {
      c->Consume(data);
    }
    if (stop_) {
      break;
    }
  }
}

void Consumer::Stop() {
  stop_ = true;
}

}  // namespace v3
}  // namespace icehalo
