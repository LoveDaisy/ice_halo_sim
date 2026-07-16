#ifndef CORE_SIMULATOR_H_
#define CORE_SIMULATOR_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/backend/backend_kind.hpp"
#include "core/backend/trace_backend.hpp"
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
  // Snapshot of renderers active when this batch was emitted (task 252.3).
  // Captured by server.cpp's GenerateScene under scene_mutex_, alongside
  // active_scene_, so a concurrent CommitConfig cannot tear the (scene,
  // renders) pair. Non-null on the backend path; the legacy CPU path ignores
  // this field and tolerates null. shared_ptr<const ...> guarantees in-flight
  // SimBatches keep the snapshot alive after a later CommitConfig swap.
  std::shared_ptr<const std::vector<RenderConfig>> renders_;
  // Snapshot of the raypath_color config active when this batch was emitted
  // (task-engine-redirect-design2). Same locking discipline as `renders_` —
  // captured under scene_mutex_ in GenerateScene so (scene, renders,
  // raypath_color) is a consistent triple. NULL is treated as "no color
  // configured" (AC3 zero-cost path) by both CPU emit gates.
  std::shared_ptr<const RaypathColorConfig> raypath_color_;
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
  // Set the preferred trace backend for the next Run() entry. Thread-safe:
  // server thread writes via release, simulator thread reads via acquire at
  // the top of Run().
  void SetPreferredBackend(BackendKind backend);

  // Returns the seed actually handed to the trace backend (task 260.6).
  // When `seed_ != 0` this equals `seed_`; when `seed_ == 0` this is a
  // per-instance non-zero value derived from a global atomic counter so the
  // backend's device-gen path activates in the default multi-worker / random
  // mode. Stable across the simulator's lifetime — required for the backend's
  // `seeded_` idempotency contract. Exposed for unit tests only.
  uint32_t GetEffectiveSeed() const { return effective_seed_; }

 private:
  using CrystalCache = std::vector<std::pair<const CrystalParam*, Crystal>>;
  struct SimWorkspace {
    RayBuffer buffer_data[2]{};
    RayBuffer init_data[2]{};
  };
  void SimulateOneWavelength(const SceneConfig& config, const RaypathColorConfig* raypath_color,
                             const WlParam& wl_param, size_t ray_num, CrystalCache& crystal_cache,
                             SimWorkspace& workspace, uint64_t generation,
                             std::vector<std::vector<double>>& ray_alloc_carry);

  // Backend-routed wavelength step (TraceBackend seam, scrum-258.1 exit-seam).
  // Drives backend.BeginSession -> (TraceLayer -> Recombine)+ -> ReadbackExitRays
  // -> EndSession and emplaces a SimData whose outgoing_d_/w_ carry the
  // backend's world-space exit rays, routed through the legacy consumer
  // projection (same downstream path as Metal-OFF). Only invoked when
  // CanUseBackend() returns true.
  void SimulateOneWavelengthWithBackend(TraceBackend& backend, const SceneConfig& scene, const RenderConfig& render,
                                        std::shared_ptr<const RaypathColorConfig> raypath_color,
                                        const WlParam& wl_param, size_t ray_num, uint64_t generation);

  // scrum-312 (third-clock drain): for SupportsThirdClockDrain() backends the
  // device XYZ accumulator persists across per-batch sessions; this window holds
  // the host-side aggregation (Σ root rays / crystals) since the last drain so
  // the drained SimData carries correct normalization + stats. Drained on
  // display cadence (producer-pause / generation-change / run-exit / batch cap),
  // not per batch — see Run() and DrainDeviceXyz.
  struct XyzDrainWindow {
    bool pending = false;     // undrained device accumulation present
    size_t root_rays = 0;     // Σ ray_num over the window (normalization denom)
    size_t crystals = 0;      // Σ crystal_count over the window (stats)
    uint64_t generation = 0;  // generation the window belongs to
    int w = 0;                // render resolution of the window
    int h = 0;
    float wl = 0.0f;     // last wl (device-fused: not consumed downstream)
    uint32_t calls = 0;  // batches accumulated since last drain (cadence cap)
    // task-color-degrade-gui-surfacing: latest GPU color-degrade tally for this
    // window. OVERWRITTEN each batch (config constant, identical every batch),
    // NOT accumulated — see the store in SimulateOneWavelength's window branch.
    ColorDegradeCounts color_degrade_counts_{};
  };
  XyzDrainWindow xyz_win_;
  static constexpr uint32_t kDefaultXyzDrainBatches = 64;
  uint32_t xyz_drain_batches_ = kDefaultXyzDrainBatches;  // resolved from env at Run() entry
  // Readback the persistent device XYZ accumulator into a SimData (window-
  // aggregated root/crystal counts), enqueue it, and reset the window. No-op if
  // `backend` is null or nothing is pending (self-guarding so call sites stay
  // flat). Called only for SupportsThirdClockDrain() backends.
  void DrainDeviceXyz(TraceBackend* backend);

  static constexpr size_t kSmallBatchRayNum = 32;

  // Experiment knob (LUMICE_GEOM_CLOCK): rays served by one sampled
  // crystal shape on this path -- the legacy CPU geometry clock, i.e. D/K.
  // Defaults to kSmallBatchRayNum, which is where the shipped value comes from:
  // the geometry resample rides the ray-batching stride as a side effect, it was
  // never chosen for sampling quality. 1 = a fresh shape per ray (the oracle).
  // Resolved from env at Run() entry.
  //
  // SAFE RANGE [1, 64] -- values >= 128 corrupt the heap, because this doubles as
  // the ray-batch stride and SimulateOneWavelength's buffers only hold
  // ray_num*2 == 256 for a 128-ray SimBatch. See env_knobs.hpp GeomClock() for
  // the measured exit codes and the full mechanism before raising it.
  size_t geom_clock_ = kSmallBatchRayNum;

  QueuePtrS<SimBatch> config_queue_;
  QueuePtrS<SimData> data_queue_;
  std::atomic_bool stop_;
  std::atomic_bool idle_;

  uint32_t seed_;
  // Non-zero seed handed to TraceBackend in `SimulateOneWavelengthWithBackend`
  // so the device-gen path activates even when the user-facing `seed_` is 0
  // (default multi-worker random mode). See task 260.6.
  uint32_t effective_seed_;
  RandomNumberGenerator rng_;
  Logger logger_{ "Simulator" };
  // Preferred trace backend. release-write by ServerImpl::SetPreferredBackend,
  // acquire-read at Run() entry. env-var LUMICE_TRACE_BACKEND still wins.
  std::atomic<BackendKind> preferred_backend_{ BackendKind::kCpu };
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
//
// Design 2 (2026-07-08, doc/gui-custom-spectrum-and-raypath-color.md §4.0):
// the emit gate has two decoupled predicates, evaluated on the same ray:
//   - `spec` (physical filter) — decides whether the ray survives.
//   - `color_spec` (color predicate, non-destructive pass) — decides which
//     color component bits get OR'd into the ray's carried mask.
// Both come from `FilterSpec::Create` with `action_=kFilterIn`; the ONLY
// difference is which JSON they were built from (physical `filter` vs the
// synthetic ComplexFilterParam we assemble from `raypath_color[].match[]`).
// A null `color_spec` (default) leaves the mask untouched — zero cost when
// no `raypath_color` is configured (AC3/AC4 anchors). `color_bits[k]` is the
// global component bit for OR-summand k of the color spec (or
// ComponentTable::kNoBit for budget-overflowed predicates).
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data,                               // output
                 const FilterSpec* color_spec = nullptr,                                     // input
                 const std::vector<uint8_t>* color_bits = nullptr);                          // input

// Non-owning view of one per-symmetry color spec group produced by
// `BuildColorSpecGroups`. Referenced by the multi-group `CollectData` overload
// so the CPU emit gate can evaluate several symmetry-scoped color passes on
// each surviving ray, with physical-filter check + prob roll still done once.
struct ColorSpecGroup {
  const FilterSpec* spec;
  const std::vector<uint8_t>* bits;
};

// Multi-group overload of `CollectData` — same contract as the two-parameter
// overload above, except the non-destructive color pass runs `color_groups`
// (one per symmetry value produced by `BuildColorSpecGroups`) instead of a
// single `color_spec`/`color_bits` pair. Each group contributes its matched
// summand bits into the ray's carried component mask via OR.
//
// The physical filter check (`spec->Check`) and the MS `prob_` roll
// (`rng.GetUniform() < ms_info.prob_`) still execute exactly once per ray —
// they must not depend on `color_groups`, otherwise the RNG stream (and hence
// continue/emit routing) would drift with the color config.
//
// A null `color_groups` behaves identically to a null `color_spec` in the
// two-parameter overload (AC3 zero-cost path when no color predicates apply).
void CollectData(RandomNumberGenerator& rng, const MsInfo& ms_info, const FilterSpec* spec,  // input
                 RayBuffer* buffer_data, RayBuffer* init_data,                               // output
                 const std::vector<ColorSpecGroup>* color_groups);                           // input

// Build the local-to-world rotation matrix for a crystal sample.
// Implements the chain  R = Rz(az - pi) * Ry(-zenith) * Rz(roll),
// where zenith = pi/2 - latitude. Inputs are in radians and follow the convention
// produced by RandomSampler::SampleSphericalPointsSph: azimuth around +z,
// latitude = asin(u) ∈ [-pi/2, +pi/2]. See doc/coordinate-convention.md.
//
// Internal: exposed for unit testing; not part of the public C API.
Rotation BuildCrystalRotation(float azimuth_rad, float latitude_rad, float roll_rad);

namespace detail {
// Maps a triangle id to its polygon-face index via argmax of the dot product
// against polygon-face normals (kFaceCoplanarFloor=1e-2). Used by InitRay_p_fid
// to label the initial entry segment. Exposed for unit-test access; not part
// of the public C API.
IdType PolygonFaceOfTri(const Crystal& crystal, int tri_id);
}  // namespace detail

}  // namespace lumice

#endif  // CORE_SIMULATOR_H_
