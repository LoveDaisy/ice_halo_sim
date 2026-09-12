#ifndef CORE_SIMULATOR_H_
#define CORE_SIMULATOR_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/backend/backend_kind.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/chain_id_table.hpp"
#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/shared/ray_allocation_shared.hpp"
#include "util/logger.hpp"

namespace lumice {

class FilterSpec;
class RayAllocationOnline;

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
  // The online ray-allocation authority of the scene this batch belongs to
  // (scene.ray_allocation = adaptive on a RENDER commit); null on every other
  // batch — proportional scenes, analysis sessions — which then deal by p with
  // every correction 1.0f, the zero-cost path. Captured in GenerateScene under
  // scene_mutex_ with the three above, so a CommitConfig cannot pair a batch with
  // another scene's tally. Not a snapshot itself: the worker Loads one snapshot
  // from it when it STARTS the batch (not when the batch was queued — the queue
  // runs up to kMaxSceneCnt batches ahead, i.e. tens of millions of rays on the
  // GPU grain, and a q bound that early would never catch up with the run) and
  // Accumulates its tally into it when the batch is done. An in-flight batch of
  // a superseded scene keeps the old object alive and adds into that, so a new
  // scene's tally can never be polluted by a stale batch. Non-const, unlike the
  // three snapshots above, because the worker WRITES into it (Accumulate) — it is
  // a synchronized channel between the workers and the server, not a snapshot;
  // its own mutex is the whole of its thread contract.
  std::shared_ptr<RayAllocationOnline> ray_alloc_online_;
};

// The online authority of one adaptive scene's ray allocation: the cumulative
// per-(layer, entry) tally every batch of every worker adds into, and the
// immutable q snapshot derived from it that the next batch deals by. One object
// per committed adaptive scene, owned by the server and bound into every SimBatch
// of that scene (SimBatch::ray_alloc_online_); an analysis session binds none and
// so deals by p (doc/raypath-analysis-panel.md §10, unchanged).
//
// Two operations, both thread-safe, both O(entries):
//   Load()       — the snapshot to deal the NEXT batch by. Never null: the
//                  constructor publishes the cold-start deal (uniform over the
//                  p_i > 0 entries of each layer — the √K-bounded worst case the
//                  design chose over dealing by p, whose small-sample estimate
//                  of a rare entry is exactly what a pilot got wrong).
//   Accumulate() — add one batch's tally, recompute every layer's q from the
//                  CUMULATIVE tally (ComputeAdaptiveRayAllocationWeights, floor
//                  included) and publish a fresh snapshot. Cumulative rather
//                  than EMA on purpose: within a run the statistic is
//                  stationary, so the running total is the best estimate at
//                  every point and converges monotonically.
//
// Unbiasedness rests on the order of those two calls inside a batch: a batch
// Loads once at its start, traces and charges under that q, and only then
// Accumulates its own tally — so the q a ray is corrected by depends only on
// batches that finished before it was born (conditionally unbiased, batch by
// batch). Workers merge through the object's own mutex; the server neither
// reads nor writes the tally while workers run.
class RayAllocationOnline {
 public:
  explicit RayAllocationOnline(const SceneConfig& scene);

  std::shared_ptr<const RayAllocationSnapshot> Load() const;
  // `delta` must be shaped like the scene this object was built for; a batch
  // whose tally is empty (nothing dealt) is a no-op. Returns true when a
  // log milestone was crossed — the caller decides what to do with it.
  bool Accumulate(const RayAllocationTally& delta);

  // A copy of the cumulative tally and of what it last published, for the log
  // line and for tests. The energy shares the object was built with are what
  // the q it publishes are relative to.
  RayAllocationTally Cumulative() const;
  const std::vector<std::vector<float>>& Proportions() const { return p_; }

 private:
  std::vector<std::vector<float>> p_;  // [mi][ci], crystal_proportion_
  mutable std::mutex mutex_;
  RayAllocationTally cumulative_;
  std::shared_ptr<const RayAllocationSnapshot> current_;
  // Log cadence: a milestone is crossed when the first layer's cumulative dealt
  // count passes the next power of two (bounded to O(log N) lines per run).
  size_t next_log_rays_ = 1;
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

  // Is this Run() still driving a TraceBackend? False both when
  // Run() never got one (CreateBackend returned nullptr — CPU preference, or a
  // GPU preference this build/host cannot honour) and after a
  // BackendUnavailableError dropped it mid-Run(). Read across threads by the
  // server's producer (GenerateScene sizes its per-batch dispatch grain on it —
  // a GPU-sized batch on the legacy path traces one host-sampled wavelength per
  // 262144 rays), so it uses the same release/acquire pairing as
  // preferred_backend_ above. Defaults to true so the window between the ctor
  // and the worker thread reaching CreateBackend does not read as "fell back".
  bool BackendActive() const { return backend_active_.load(std::memory_order_acquire); }

  // Raypath-analysis mode for the next Run() entry (doc/raypath-analysis-panel.md
  // §3). While `enabled`, the legacy CPU path allocates the per-ray chain-id
  // column, interns every layer traversal into this Simulator's own
  // ChainIdInterningTable (cleared at Run() entry — one Run() is one analysis
  // session, and ids restart from 1) under `symmetry` (FilterConfig::kSym*
  // flags, applied uniformly to every layer; sigma_a / d_applicable are still
  // derived per layer from that layer's axis distribution exactly as
  // FilterSpec::Create does), and delivers SimData::outgoing_chain_id_ +
  // chain_id_table_delta_. The backend-routed paths (CpuTraceBackend, Metal,
  // CUDA) do not implement it: they leave both fields empty and Run() logs one
  // WARN per entry saying so. Same thread contract as SetPreferredBackend:
  // written by the server thread, snapshotted at the top of Run().
  void SetAnalysisChainId(bool enabled, uint8_t symmetry);

  // The analysis run's other session property: force the legacy CPU path for
  // the next Run(), ahead of BOTH the LUMICE_TRACE_BACKEND override and the
  // SetPreferredBackend preference (neither is consulted, and neither is
  // modified — the preference is still there for the next render session).
  // Same thread contract as SetAnalysisChainId: written by the server thread,
  // snapshotted at the top of Run().
  void SetAnalysisForceCpu(bool enabled);

  // The backend kind the most recent Run() entry actually resolved to — kCpu
  // for the legacy path (whether by preference, by force, by an unavailable
  // GPU, or by the mid-run BackendUnavailableError fallback, which re-publishes
  // it), kMetal / kCuda while that backend is live. kCpu before the first Run().
  // Written at the same two points as backend_active_ and read by the server
  // (Server::GetActiveBackend) — the observable answer to "did the force take".
  BackendKind ActiveBackend() const { return active_backend_.load(std::memory_order_acquire); }

  // Returns the seed actually handed to the trace backend (task 260.6).
  // When `seed_ != 0` this equals `seed_`; when `seed_ == 0` this is a
  // per-instance non-zero value derived from a global atomic counter so the
  // backend's device-gen path activates in the default multi-worker / random
  // mode. Stable across the simulator's lifetime — required for the backend's
  // `seeded_` idempotency contract. Exposed for unit tests only.
  uint32_t GetEffectiveSeed() const { return effective_seed_; }

  // Observation hook for the legacy CPU path's per-batch `all_data` buffer,
  // invoked SYNCHRONOUSLY on the simulator's own thread at the point the batch
  // is sealed, just before the SimData is queued. Exposed for unit tests only:
  // `all_data` never leaves the producer thread (the whole point of recycling
  // it in SimWorkspace), so a test that needs to inspect the raw ray segments
  // has no consumer-side seat to do it from any more.
  //
  // Contract: set only for the lifetime of a test fixture, from the same thread
  // that will read the results; production leaves it at nullptr and pays a
  // single predictable branch per batch. This is NOT a production extension
  // point — a real consumer belongs on the SimData/consumer seam, not here.
  // A plain function pointer + context, deliberately not std::function: the
  // hook is permanent interface surface on a hot-path production type.
  using AllDataObserverFn = void (*)(void* ctx, const RayBuffer& all_data);
  void SetAllDataObserverForTest(AllDataObserverFn fn, void* ctx) {
    all_data_observer_ = fn;
    all_data_observer_ctx_ = ctx;
  }


 private:
  using CrystalCache = std::vector<std::pair<const CrystalParam*, Crystal>>;
  struct SimWorkspace {
    RayBuffer buffer_data[2]{};
    RayBuffer init_data[2]{};
    // Per-batch accumulation buffer for the legacy CPU path. Lives here, next
    // to the two pools it already sits beside, so it is RECYCLED across batches
    // (Reset = grow-never-shrink) instead of being freshly allocated and then
    // handed to SimData every batch. At 2 MS layers that buffer is ~1.9 MB,
    // which is past the Windows CRT's 512 KB heap-cache threshold, so the old
    // per-batch construct/destruct pair went straight to VirtualAlloc /
    // VirtualFree — measured as the root cause of the 2-layer-MS Windows/Linux
    // throughput gap. Keeping it here is only sound because nothing downstream
    // reads its CONTENT: SimData carries the segment count alone
    // (`ray_seg_count_`), see the note there.
    RayBuffer all_data{};
  };
  // `emitted_weight` is the per-ray spectral weight to CHARGE THE NORMALIZATION
  // DENOMINATOR with, deliberately separate from `wl_param.weight_`, which is the
  // weight the rays are physically traced at. The two differ for an illuminant
  // spectrum, where the traced weight comes from a random wavelength draw and the
  // charged weight is that draw's expectation, so the denominator does not wobble
  // with the seed. It is a plain parameter rather than a WlParam field on purpose:
  // the per-ray-wl-pool call site deliberately passes a ZERO WlParam (so a dropped
  // per-ray wavelength renders black and loud instead of collapsing onto a flat
  // spectrum) while still needing a real emitted weight — folding the two into one
  // struct would make that call site express both meanings with one value. If a
  // further audit-only quantity ever joins it, group them into a batch-audit
  // struct rather than growing this parameter list again.
  // `ray_alloc`: the q snapshot this batch deals by — Loaded ONCE per SimBatch
  // by Run() from SimBatch::ray_alloc_online_ and held for the batch, so every
  // wavelength of a discrete spectrum deals by the same q; nullptr on every
  // proportional or analysis batch. `tally_out`: when non-null, this call's
  // per-(layer, entry) tally (core/shared/ray_allocation_shared.hpp for the
  // contract) is accumulated into it at the same "true exit" collection point
  // outgoing_w_ is built from, so what is measured is exactly what the consumer
  // is handed. Left untouched on a stop_ abort, when nothing is published either.
  void SimulateOneWavelength(const SceneConfig& config, const RaypathColorConfig* raypath_color,
                             const WlParam& wl_param, float emitted_weight, size_t ray_num, CrystalCache& crystal_cache,
                             SimWorkspace& workspace, uint64_t generation,
                             std::vector<std::vector<double>>& ray_alloc_carry, const RayAllocationSnapshot* ray_alloc,
                             RayAllocationTally* tally_out);

  // Backend-routed wavelength step (TraceBackend seam, scrum-258.1 exit-seam).
  // Drives backend.BeginSession -> (TraceLayer -> Recombine)+ -> ReadbackExitRays
  // -> EndSession and emplaces a SimData whose outgoing_d_/w_ carry the
  // backend's world-space exit rays, routed through the legacy consumer
  // projection (same downstream path as Metal-OFF). Only invoked when
  // CanUseBackend() returns true.
  // `emitted_weight`: see SimulateOneWavelength above — same contract.
  void SimulateOneWavelengthWithBackend(TraceBackend& backend, const SceneConfig& scene, const RenderConfig& render,
                                        std::shared_ptr<const RaypathColorConfig> raypath_color,
                                        const WlParam& wl_param, float emitted_weight, size_t ray_num,
                                        uint64_t generation, const RayAllocationSnapshot* ray_alloc,
                                        RayAllocationTally* tally_out);

  // scrum-312 (third-clock drain): for SupportsThirdClockDrain() backends the
  // device XYZ accumulator persists across per-batch sessions; this window holds
  // the host-side aggregation (Σ root rays / crystals) since the last drain so
  // the drained SimData carries correct normalization + stats. Drained on
  // display cadence (producer-pause / generation-change / run-exit / batch cap),
  // not per batch — see Run() and DrainDeviceXyz.
  struct XyzDrainWindow {
    bool pending = false;  // undrained device accumulation present
    size_t root_rays = 0;  // Σ ray_num over the window
    // Σ emitted_weight × ray_num over the window — the absolute normalization
    // denominator's window aggregate. Accumulated at exactly the same site and
    // under exactly the same condition as root_rays above; the two must never
    // drift apart or the third-clock (GPU) path would normalize against a
    // different quantity than the single-batch paths do.
    float emitted_energy = 0.0f;
    // Stats: Σ stochastic draws over the window (accumulated) alongside the
    // scene's deterministic slot count (OVERWRITTEN — config constant, same
    // discipline as color_degrade_counts_ below). See TraceBackend::
    // GetLastBatchStochasticCrystalSampleCount for why the two cannot be one.
    size_t stochastic_crystal_samples = 0;
    size_t deterministic_crystals = 0;
    // Orientation stats ride the window under the identical two-rule split (Σ
    // vs OVERWRITE). Separate from the crystal pair above because they are a
    // separate statistic -- see SimData for the field-level contract.
    size_t stochastic_orientation_samples = 0;
    size_t deterministic_orientations = 0;
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
  // One `RayAllocationOnline: layer L entry E: p= q= rays=` line per (layer, entry)
  // of `online`, at the cadence Accumulate reports (each doubling of the first
  // layer's dealt count). The only signal of the online q that crosses the process
  // boundary: the e2e mechanism test parses it, and the server's Stop() prints the
  // same shape once more as the run's final state.
  void LogRayAllocationMilestone(const RayAllocationOnline& online);

  static constexpr size_t kSmallBatchRayNum = 32;

  // Experiment knob (LUMICE_GEOM_CLOCK): rays served by one sampled
  // crystal shape on this path -- the legacy CPU geometry clock, i.e. D/K.
  // Defaults to kSmallBatchRayNum, which is where the shipped value comes from:
  // the geometry resample rides the ray-batching stride as a side effect, it was
  // never chosen for sampling quality. 1 = a fresh shape per ray (the oracle).
  // Resolved from env at Run() entry.
  //
  // This doubles as the ray-batch stride. It used to carry a heap-corruption
  // ceiling at the SimBatch size for that reason; ResetHitLoopBuffers removed it
  // by sizing the hit-loop buffer pair against buffer_data[0]'s capacity instead
  // of against the batch. Values past the dispatch granularity are now a no-op
  // rather than unsafe -- see env_knobs.hpp GeomClock() for the before/after
  // measurement and why raising this alone stops having an effect.
  size_t geom_clock_ = kSmallBatchRayNum;
  // Deterministic half of the reported crystal-geometry count for the config
  // currently in hand — a pure function of that config (see
  // trace_ops.hpp::DeterministicCrystalCount), so it is derived ONCE per
  // committed batch in Run() and merely read by the four sites that publish it.
  // Those sites all aggregate it by OVERWRITE, so recomputing per site was
  // harmless, but it left the invariant with four derivation points and no
  // single authority. Assigned in Run() right after the config is obtained
  // rather than gated on `generation != prev_generation`: `prev_generation`
  // starts at 0, so a generation-keyed cache would skip the first committed
  // config whenever generations are 0-based and silently publish 0.
  size_t deterministic_crystal_count_ = 0;
  // Same role for the orientation count, derived at the same single point in
  // Run() from the same committed config, for the same reasons.
  size_t deterministic_orientation_count_ = 0;

  QueuePtrS<SimBatch> config_queue_;
  QueuePtrS<SimData> data_queue_;
  std::atomic_bool stop_;
  std::atomic_bool idle_;

  uint32_t seed_;
  // Non-zero seed handed to TraceBackend in `SimulateOneWavelengthWithBackend`
  // so the device-gen path activates even when the user-facing `seed_` is 0
  // (default multi-worker random mode). See task 260.6.
  uint32_t effective_seed_;

  // See SetAllDataObserverForTest. nullptr in every production build path.
  AllDataObserverFn all_data_observer_ = nullptr;
  void* all_data_observer_ctx_ = nullptr;

  RandomNumberGenerator rng_;
  Logger logger_{ "Simulator" };
  // Preferred trace backend. release-write by ServerImpl::SetPreferredBackend,
  // acquire-read at Run() entry. env-var LUMICE_TRACE_BACKEND still wins.
  std::atomic<BackendKind> preferred_backend_{ BackendKind::kCpu };

  // Raypath-analysis session settings: the atomic is what SetAnalysisChainId
  // writes (one 2-byte trivially-copyable value, so enabled and symmetry can
  // never be observed torn), the plain copy is Run()'s snapshot of it, read on
  // the simulator thread only. Default: off; the symmetry only means anything
  // once SetAnalysisChainId turns it on, and that call always sets both. The
  // server's analysis run passes FilterConfig::kSymNone (chains recorded at
  // their finest, reduced when read — server.hpp RaypathAnalysisRequest); the
  // mechanism itself takes any P/B/D bit set, and tests exercise the others.
  struct ChainIdSession {
    bool enabled = false;
    uint8_t symmetry = FilterConfig::kSymNone;
  };
  std::atomic<ChainIdSession> analysis_chain_id_{ ChainIdSession{ false, FilterConfig::kSymNone } };
  ChainIdSession chain_id_session_{};
  // See SetAnalysisForceCpu / ActiveBackend.
  std::atomic_bool analysis_force_cpu_{ false };
  std::atomic<BackendKind> active_backend_{ BackendKind::kCpu };
  // Per-worker interning table (see chain_id_table.hpp for why per-worker is
  // enough). Only ever touched from inside Run() on the simulator thread.
  ChainIdInterningTable chain_id_table_;

  // Backing store for BackendActive() (see its declaration above for
  // the contract and the default's rationale). Written by the simulator thread at
  // exactly two points inside Run() — right after CreateBackend, and in the
  // BackendUnavailableError catch that resets `backend` — and nowhere else; those
  // are the only two places `backend`'s nullness changes.
  std::atomic_bool backend_active_{ true };
};

// Distributes ray_num rays across crystals proportionally using per-crystal carry with
// largest-remainder correction. carry[ci] accumulates fractional remainders across calls,
// enabling fair allocation for crystals with proportion * ray_num < 1.
// Caller must ensure carry.size() == proportions.size(). Returns array with exact sum == ray_num.
std::unique_ptr<size_t[]> PartitionCrystalRayNum(const std::vector<float>& proportions, size_t ray_num,
                                                 std::vector<double>& carry);

// Per-entry weight correction for dealing a layer's rays by `q` while the energy shares are
// `p`: correction_i = (p_i/ΣP) / (q_i/ΣQ), with ΣP = Σ max(0, p_i) and ΣQ = Σ max(0, q_i) —
// the same normalization PartitionCrystalRayNum applies to whatever it is handed, so the two
// agree entry for entry and Σ_i n_i · w · correction_i == N · w up to the partition's ±1-ray
// rounding, whatever raw scale `p` and `q` arrive in. Comparing the raw ratio p_i/q_i instead
// would fold a constant ΣP/ΣQ into every ray whenever the two vectors are not on the same
// scale — a global brightness bias, not a rounding error, and one a ±1-ray tolerance would
// not catch.
// An entry with q_i <= 0 is dealt no rays by PartitionCrystalRayNum, so its correction is
// never read; it is returned as 1.0f so the unread slot holds a finite value rather than a
// 0/0 NaN. Precondition: p.size() == q.size(). The single owner of this formula — every
// backend consumes the vector, none re-derives it.
std::vector<float> ComputeRayAllocationCorrection(const std::vector<float>& p, const std::vector<float>& q);

// The single owner of the first layer's Σ_ci n_ci · (correction_ci − 1) formula that feeds
// emitted_ray_equivalent / emitted_ray_equivalent_delta_this_batch_ (see the comment at the
// call site in simulator.cpp for why the (correction - 1) shape matters). `crystal_ray_num`
// must have at least `count` entries; `corrections` must have at least `count` entries too —
// both hold for every call site, which all derive `count` from the same ms_info.setting_.size()
// used to build both arrays. Every first_ms accumulation site (legacy Simulator,
// CpuTraceBackend, MetalTraceBackend, CudaTraceBackend) calls this rather than re-deriving the
// sum inline, so it enjoys the same single-authority status as ComputeRayAllocationCorrection.
double AccumulateFirstLayerEmittedRayEquivalentDelta(const size_t* crystal_ray_num,
                                                     const std::vector<float>& corrections, size_t count);

// What one MS layer's ray partition and per-entry weight correction are, resolved from the
// scene's allocation mode and the layer's entries. `partition_weights` is what
// PartitionCrystalRayNum is handed — p_i (ScatteringSetting::crystal_proportion_) under
// kProportional, q_i under a delivered kAdaptive layer. Named distinctly from `proportions`
// (the scene's crystal_proportion_, always an energy share) precisely because this field's
// dimension is mode-dependent: reusing that name here would make one identifier stand for two
// different quantities, which is exactly the p_i/q_i conflation this task exists to undo.
// `corrections` is what each ray born into entry ci is multiplied by.
struct LayerRayAllocation {
  std::vector<float> partition_weights;
  std::vector<float> corrections;
  // True when the layer is dealt by q (kAdaptive AND a snapshot was delivered). What a
  // backend gates its tally on: a layer dealt by p has nothing to measure for.
  bool adaptive = false;
};

// The one place that decides whether a layer deals by p or by q. `q_for_layer` is the
// layer's row of the RayAllocationSnapshot the batch Loaded (its size must equal the
// layer's entry count), or nullptr when no snapshot was delivered — every proportional
// scene, and every analysis session, whose SimBatch binds no RayAllocationOnline.
// kProportional, or kAdaptive with no snapshot, deals by crystal_proportion_ with every
// correction exactly 1.0f — the multiply is then an IEEE identity and the layer is
// bit-for-bit what it was before the mode existed. Delivery is all-or-nothing per LAYER
// by construction (a snapshot carries every entry of every layer); there is no "partly
// adaptive" layer.
LayerRayAllocation ResolveLayerRayAllocation(SceneConfig::RayAllocationMode mode, const MsInfo& layer,
                                             const std::vector<float>* q_for_layer);

// The q one layer is dealt by, from its energy shares `p` (crystal_proportion_) and
// the cumulative tally of the same layer (`stats.size() == p.size()`). Neyman
// allocation with a floor:
//   q_i = 0                                  if p_i <= 0   (a switched-off entry stays off)
//   q_i = max(raw_i / Σraw, 0.01 / K)        otherwise, raw_i = p_i · √(Σw²_i / rays_i)
// with K the number of p_i > 0 entries — the entries the floor is FOR; a p_i = 0 entry
// is not a share the floor divides among. The floor is not a zero-hit special case:
// Neyman on its own pushes a rare-but-ordinary-energy entry BELOW its proportional
// share (E[e²] carries the hit rate once more than E[e] does), and an entry the tally
// saw no exit from would otherwise be dealt nothing for the rest of the run — a fixed
// point nothing escapes (see the starvation test). 0.01/K keeps the main entries'
// shares within 0.1% of Neyman while guaranteeing every live entry 1% of a uniform
// deal. The vector is returned unnormalized past the floor: PartitionCrystalRayNum
// and ComputeRayAllocationCorrection normalize whatever they are handed, and the
// same Σ is what both see. An entry with rays_i == 0 has raw_i = 0 (nothing was
// measured, so nothing is claimed); when every raw_i is 0 the live entries are
// dealt uniformly at the floor — which is also the cold-start deal, before any
// batch has been measured.
std::vector<float> ComputeAdaptiveRayAllocationWeights(const std::vector<float>& p,
                                                       const std::vector<RayAllocationEntryTally>& stats);

// Whether a newly committed scene needs its online tally started over. True when
// anything the tally can depend on differs between the two scenes; false when
// only the fields it is indifferent to differ — ray_num_, geom_clock_ and the
// ray_allocation_ mode itself. Implemented by comparing the two scenes with the
// one SceneConfig equality this tree has (config_compare.hpp) after masking exactly
// those fields, rather than by a second list of the fields that DO matter: a scene
// field added later is therefore compared by default, which errs toward one extra
// cold start, never toward a stale q. CommitConfig is a high-frequency path (the
// GUI recommits every 70ms while a slider drags), and a recommit that changed
// nothing the statistic reads must carry the accumulated tally forward rather than
// throw it away.
bool RayAllocationInputsChanged(const SceneConfig& previous, const SceneConfig& next);

// The one writer of the `<prefix>: layer L entry E: p=… q=… rays=…` line, one per
// (layer, entry): q is reported as the layer's SHARE (Σ over live entries = 1) so
// it reads against p directly. Called by the worker at Accumulate's milestone
// cadence (prefix "RayAllocationOnline") and by the server at Stop() for the run's
// final state (prefix "RayAllocationOnline(final)").
void LogRayAllocationState(Logger& logger, const RayAllocationOnline& online, const char* prefix);

// Single owner of the hit-loop buffer-pair capacity contract. The pair is a
// producer/consumer ping-pong (buffer_data[0] holds a hit's input rays,
// buffer_data[1] receives their two-child fan-out), so their sizes are NOT
// independent: buffer_data[1] must absorb twice whatever buffer_data[0] can
// hold. Sizing the two separately is what let a fan-out write past the end of
// buffer_data[1] — see the definition in simulator.cpp for the full mechanism.
//
// `chain_id_enabled` is forwarded to both RayBuffer::Reset calls — the
// per-ray chain-id column exists on both halves of the pair or on neither.
//
// Internal: exposed for unit testing; not part of the public C API.
void ResetHitLoopBuffers(RayBuffer buffer_data[2], size_t ray_num, bool chain_id_enabled = false);

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
// NOTE ON THIS NAMESPACE'S REACH: CountEntrySubTris / BuildEntrySubTris / the
// EntrySubTri struct below are no longer test-or-simulator-only — the Metal and
// CUDA backends' host upload paths (metal_trace_backend.mm, cuda_trace_backend.cu)
// call them in PRODUCTION to build their device entry-sampler triangle SoA from
// cf_geom_. They are, in effect, a small geometry utility shared by three modules
// (simulator + both GPU backends), which sits uneasily with the `detail` name's
// "internal / test-visible" connotation. The migration to a proper shared home
// (e.g. crystal.hpp or a dedicated geometry header) is DELIBERATELY DEFERRED: it
// is a pure rename with no functional benefit today, and the reuse works as-is.
// TRACKED DEBT: if a fourth cross-module consumer appears, re-evaluate promoting
// these out of `detail` into a first-class shared API at that point.

// One fan sub-triangle of a present polygon face, built once per entry-sampler
// setup (CPU InitRay_p_fid per call, or a GPU backend's per-crystal host upload)
// from cf_geom_ corners. Carries its geometry (for the per-ray projected weight
// + point sampling) and the compact present-face id it belongs to (== the
// RaySeg::to_face_ value, matching PopulateFromCfGeom's numbering). Also exposed
// for unit-test access (constructing a synthetic CrystalGeom with a degenerate
// fan sub-triangle); not part of the public C API.
struct EntrySubTri {
  float v[9];      // 3 corners (fanned from corner 0)
  float n[3];      // unit winding normal Cross3(v1-v0, v2-v0), normalized (zero if area == 0)
  float area;      // 0.5 * |Cross3(v1-v0, v2-v0)|
  IdType face_id;  // compact present-face id (0..PolygonFaceCount()-1)
};

// Count the fan sub-triangles the present faces expand into (Σ max(vtx-2, 0)).
size_t CountEntrySubTris(const CrystalGeom& cf);

// Expand present faces into a flat fan sub-triangle table. Fan (0, k, k+1) for
// k = 1..vtx_cnt-2, matching BuildMeshFromCfGeom and the T1 analytic oracle
// (test/support/incidence_sampling_oracle.hpp). A sub-triangle whose corners
// are degenerate (collapsed/collinear, area == 0) gets a zero normal instead
// of a NaN one — its area already zeroes its selection weight downstream.
void BuildEntrySubTris(const CrystalGeom& cf, EntrySubTri* out);
}  // namespace detail

}  // namespace lumice

#endif  // CORE_SIMULATOR_H_
