#ifndef CORE_SIMULATOR_H_
#define CORE_SIMULATOR_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "util/logger.hpp"

namespace lumice {

class FilterSpec;

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
  // Adaptive Brightness mode chosen at commit time. kOn → Design A CollectData (filter is
  // emit-gate); kOff → F1 CollectData_F1 (filter-fail outgoing rays routed to anchor buffer
  // instead of dropped, and may still continue to next MS level). When multiple renderers
  // exist with mixed modes the server emits kOff if any renderer requests it (OFF-superset).
  AdaptiveBrightnessMode ab_mode_ = AdaptiveBrightnessMode::kOn;
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
                             AdaptiveBrightnessMode ab_mode, CrystalCache& crystal_cache, SimWorkspace& workspace,
                             uint64_t generation, std::vector<std::vector<double>>& ray_alloc_carry);

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

// Per-batch ray dispatcher: classifies each ray via derived predicates
// (IsNormal() / IsOutgoing() / IsContinue() / IsTir() / IsFilterDropped()) and
// routes IsContinue() rays into the next ms init buffer. Segment kind is
// derived from (to_face_, w_, is_continue_, is_filter_dropped_); only the
// IsContinue() and IsFilterDropped() bits are written here.
//
// Filter semantics (Design A, see doc/filter-architecture.md §2): the filter
// is a simulator-side emit-gate. For outgoing candidates, filter-fail rays
// are marked IsFilterDropped() and excluded from both outgoing and continue;
// filter-pass rays then branch on prob between continue (next MS level) and
// outgoing. The consumer no longer applies a query filter.
//
// Internal: exposed for unit testing; not part of the public C API.
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data);                              // output

// F1 variant of CollectData (selected when AdaptiveBrightnessMode::kOff is active for the batch).
// Differences from Design-A CollectData:
//   - Filter is NOT an emit-gate. Outgoing candidates whose filter check fails are routed into
//     the anchor lane (collected by the caller via IsFilterDropped()) AND remain eligible for
//     prob-pass continue to the next MS layer. This is the source of F1's perf cost in
//     scenes with mid-trajectory filters.
//   - Filter-pass + prob-pass → continue (same as Design A).
//   - Filter-pass + prob-fail → emit to outgoing lane (same as Design A).
// is_continue_ and is_filter_dropped_ flags are still written so the caller's outgoing/anchor
// collection loop can dispatch by IsOutgoing() / IsFilterDropped().
//
// See doc/filter-architecture.md §7 (and scrum-adaptive-additivity-redesign explore SUMMARY)
// for design rationale. Internal: exposed for unit testing; not part of the public C API.
void CollectDataF1(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
                   RayBuffer* buffer_data, RayBuffer* init_data);                              // output

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
