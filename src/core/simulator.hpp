#ifndef CORE_SIMULATOR_H_
#define CORE_SIMULATOR_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "util/logger.hpp"

namespace lumice {

class Filter;

template <class T>
class Queue;

template <class T>
using QueuePtrU = std::unique_ptr<Queue<T>>;
template <class T>
using QueuePtrS = std::shared_ptr<Queue<T>>;

struct SimBatch {
  size_t ray_num_ = 0;
  std::shared_ptr<const SceneConfig> scene_;
  uint64_t generation_ = 0;
};

class Simulator {
 public:
  enum State {
    kIdle,
    kRunning,
  };

  Simulator(QueuePtrS<SimBatch> config_queue, QueuePtrS<SimData> data_queue, uint32_t seed = 0);
  Simulator(const Simulator& other) = delete;
  Simulator(Simulator&& other) noexcept;
  ~Simulator() = default;

  Simulator& operator=(const Simulator& other) = delete;
  Simulator& operator=(Simulator&& other) noexcept;

  void Run();
  void Stop();
  bool IsIdle() const;
  void SetLogLevel(LogLevel level);

 private:
  using CrystalCache = std::vector<std::pair<const CrystalParam*, Crystal>>;
  struct SimWorkspace {
    RayBuffer buffer_data[2]{};
    RayBuffer init_data[2]{};
  };
  void SimulateOneWavelength(const SceneConfig& config, const WlParam& wl_param, size_t ray_num,
                             CrystalCache& crystal_cache, SimWorkspace& workspace, uint64_t generation,
                             std::vector<std::vector<double>>& ray_alloc_carry);

  static constexpr size_t kSmallBatchRayNum = 32;

  QueuePtrS<SimBatch> config_queue_;
  QueuePtrS<SimData> data_queue_;
  std::atomic_bool stop_;
  std::atomic_bool idle_;

  uint32_t seed_;
  RandomNumberGenerator rng_;
  Logger logger_{ "Simulator" };
};

// Distributes ray_num rays across crystals proportionally using per-crystal carry with
// largest-remainder correction. carry[ci] accumulates fractional remainders across calls,
// enabling fair allocation for crystals with proportion * ray_num < 1.
// Caller must ensure carry.size() == proportions.size(). Returns array with exact sum == ray_num.
std::unique_ptr<size_t[]> PartitionCrystalRayNum(const std::vector<float>& proportions, size_t ray_num,
                                                 std::vector<double>& carry);

// Per-batch ray dispatcher: decides each ray's final state (kNormal / kOutgoing /
// kContinue / kStopped) and routes kContinue rays into the next ms init buffer.
//
// Filter semantics (post task-query-filter-uplift-v2): the filter acts only as a
// branch gate controlling kContinue. Filter-fail rays are emitted as kOutgoing so
// that the consumer-side query filter sees the full unfiltered ray set.
//
// Internal: exposed for unit testing; not part of the public C API.
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const Filter* filter,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data);                            // output

// Build the local-to-world rotation matrix for a crystal sample.
// Implements the chain  R = Rz(az - pi) * Ry(-zenith) * Rz(roll),
// where zenith = pi/2 - latitude. Inputs are in radians and follow the convention
// produced by RandomSampler::SampleSphericalPointsSph: azimuth around +z,
// latitude = asin(u) ∈ [-pi/2, +pi/2]. See doc/coordinate-convention.md.
//
// Internal: exposed for unit testing; not part of the public C API.
Rotation BuildCrystalRotation(float azimuth_rad, float latitude_rad, float roll_rad);

}  // namespace lumice

#endif  // CORE_SIMULATOR_H_
