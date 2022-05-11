#ifndef CONSUMER_CONSUMER_H_
#define CONSUMER_CONSUMER_H_

#include <atomic>
#include <memory>
#include <vector>

#include "config/sim_data.hpp"
#include "include/result.hpp"

namespace icehalo {
namespace v3 {

// =============== Interface ===============
class IConsume {
 public:
  IConsume() = default;
  IConsume(const IConsume&) = delete;
  IConsume(IConsume&&) = delete;
  virtual ~IConsume() = default;

  IConsume& operator=(const IConsume&) = delete;
  IConsume& operator=(IConsume&&) = delete;

  virtual void Consume(const SimData& data) = 0;
  virtual Result GetResult() const { return NoneResult{}; }
};

using ConsumerPtrU = std::unique_ptr<IConsume>;
using ConsumerPtrS = std::shared_ptr<IConsume>;

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_CONSUMER_H_
