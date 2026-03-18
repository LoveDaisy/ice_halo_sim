#ifndef CONSUMER_CONSUMER_H_
#define CONSUMER_CONSUMER_H_

#include <atomic>
#include <memory>
#include <vector>

#include "config/sim_data.hpp"
#include "server/server.hpp"

namespace lumice {

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
   * @brief Reset accumulated data to zero.
   * @details Called under consumer_mutex_ during hot config updates.
   *          Implementations must clear all accumulated simulation data (e.g., internal_xyz_,
   *          total_intensity_, ray counters) so that subsequent Consume() calls start fresh.
   *          Default implementation is empty (no-op), suitable for stateless consumers.
   */
  virtual void ResetAccumulation() {}

  /**
   * @brief Prepare a snapshot of the current state.
   * @details Called under consumer_mutex_ to create a consistent snapshot (memcpy).
   *          Default implementation is empty (no-op), suitable for consumers that don't need snapshotting.
   */
  virtual void PrepareSnapshot() {}

  /**
   * @brief Post-process snapshot data (e.g., XYZ→RGB conversion).
   * @details Called under snapshot_mutex_ (NOT consumer_mutex_) after PrepareSnapshot().
   *          Default implementation is empty (no-op).
   */
  virtual void PostSnapshot() {}

  /**
   * @brief Get processing result
   * @return Result object containing the processing result
   * @note Default implementation returns NoneResult
   * @note Override this method to return custom results (e.g., RenderResult, StatsResult)
   */
  virtual Result GetResult() const { return NoneResult{}; }
};

using ConsumerPtrS = std::shared_ptr<IConsume>;

}  // namespace lumice

#endif  // CONSUMER_CONSUMER_H_
