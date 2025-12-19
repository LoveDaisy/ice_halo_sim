#ifndef CONSUMER_CONSUMER_H_
#define CONSUMER_CONSUMER_H_

#include <atomic>
#include <memory>
#include <vector>

#include "config/sim_data.hpp"
#include "include/server.hpp"

namespace icehalo {
namespace v3 {

// =============== Interface ===============
/**
 * @brief Consumer interface for processing simulation data
 * @details This interface defines the contract for consumers that process simulation data.
 *          Consumers are used to render images, collect statistics, or perform other operations
 *          on the simulation results.
 */
class IConsume {
 public:
  IConsume() = default;
  IConsume(const IConsume&) = delete;
  IConsume(IConsume&&) = delete;
  virtual ~IConsume() = default;

  IConsume& operator=(const IConsume&) = delete;
  IConsume& operator=(IConsume&&) = delete;

  /**
   * @brief Consume simulation data
   * @param data Simulation data to process
   * @note This method is called by the server for each simulation data batch
   */
  virtual void Consume(const SimData& data) = 0;

  /**
   * @brief Get processing result
   * @return Result object containing the processing result
   * @note Default implementation returns NoneResult
   * @note Override this method to return custom results (e.g., RenderResult, StatsResult)
   */
  virtual Result GetResult() const { return NoneResult{}; }
};

using ConsumerPtrU = std::unique_ptr<IConsume>;
using ConsumerPtrS = std::shared_ptr<IConsume>;

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_CONSUMER_H_
