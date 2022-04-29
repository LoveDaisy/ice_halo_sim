#ifndef CONSUMER_CONSUMER_H_
#define CONSUMER_CONSUMER_H_

#include <atomic>
#include <memory>
#include <vector>

#include "protocol/sim_data.hpp"

namespace icehalo {
namespace v3 {

class IConsume {
 public:
  IConsume() = default;
  IConsume(const IConsume&) = delete;
  IConsume(IConsume&&) = delete;
  virtual ~IConsume() = default;

  IConsume& operator=(const IConsume&) = delete;
  IConsume& operator=(IConsume&&) = delete;

  virtual void Consume(const SimData& data) = 0;
};

using ConsumerPtrU = std::unique_ptr<IConsume>;
using ConsumerPtrS = std::shared_ptr<IConsume>;


template <class T>
class Queue;

template <class T>
using QueuePtrU = std::unique_ptr<Queue<T>>;
template <class T>
using QueuePtrS = std::shared_ptr<Queue<T>>;

class Consumer {
 public:
  Consumer(QueuePtrS<SimData> data_queue);

  void RegisterConsumer(ConsumerPtrU consumer);
  void Run();
  void Stop();

 private:
  std::vector<ConsumerPtrU> consumers_;
  QueuePtrS<SimData> data_queue_;
  std::atomic_bool stop_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_CONSUMER_H_
