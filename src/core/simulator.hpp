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
#include "core/trace_backend.hpp"
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
  // Snapshot of renderers active when this batch was emitted (task 252.3).
  // Captured by server.cpp's GenerateScene under scene_mutex_, alongside
  // active_scene_, so a concurrent CommitConfig cannot tear the (scene,
  // renders) pair. Non-null on the backend path; the legacy CPU path ignores
  // this field and tolerates null. shared_ptr<const ...> guarantees in-flight
  // SimBatches keep the snapshot alive after a later CommitConfig swap.
  std::shared_ptr<const std::vector<RenderConfig>> renders_;
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

  // Backend-routed wavelength step (task 252.3, TraceBackend seam integration).
  // Drives backend.BeginSession -> (TraceLayer -> Recombine)+ -> ReadbackImage
  // -> EndSession and emplaces a SimData carrying backend_xyz_ +
  // backend_total_intensity_. Only invoked when CanUseBackend() returns true.
  void SimulateOneWavelengthWithBackend(TraceBackend& backend, const SceneConfig& scene, const RenderConfig& render,
                                        const WlParam& wl_param, size_t ray_num, uint64_t generation);

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
// (IsNormal() / IsOutgoing() / IsContinue() / IsTir()) and routes
// IsContinue() rays into the next ms init buffer. Design A filter semantics:
// filter-fail = ray terminates (w_ set negative); filter-pass + prob-pass =
// continue; filter-pass + prob-fail = emit outgoing.
//
// Internal: exposed for unit testing; not part of the public C API.
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
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
