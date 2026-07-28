#ifndef CORE_CPU_TRACE_BACKEND_H_
#define CORE_CPU_TRACE_BACKEND_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "config/color_gate_table.hpp"
#include "config/sim_data.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/trace_ops.hpp"

namespace lumice {

// CPU implementation of the TraceBackend seam.
//
// Mirrors Simulator::SimulateOneWavelength's loop structure (small-batch = 32
// rays per init+trace pass) per MS layer / per crystal-batch, but exposes the
// trace -> projection -> XYZ-accumulation as a coarse-grained `TraceLayer`.
//
// In this initial cut, the contract is: ONE TraceLayer call per (MS layer,
// crystal_id). Multi-crystal MS layers require multiple TraceLayer calls
// before invoking Recombine. The session-level crystal cache lives in
// `layer_crystals_` so deterministic-crystal layers reuse the same Crystal
// across TraceLayer calls.
class CpuLayerHandle : public LayerHandle {
 public:
  CpuLayerHandle() = default;
  size_t ContinuationCount() const override;
  LayerStats GetLayerStats() const override;

 private:
  friend class CpuTraceBackend;
  RayBuffer continuation_;
  LayerStats stats_{};
};

class CpuTraceBackend : public TraceBackend {
 public:
  CpuTraceBackend();
  ~CpuTraceBackend() override = default;

  // Seed contract: the first call with spec.seed != 0 seeds the RNG for
  // this backend instance's entire lifetime; subsequent calls with
  // spec.seed != 0 trigger an assertion. To use a different seed, destroy
  // and recreate the backend.
  void BeginSession(const SessionSpec& spec) override;
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override;
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override;
  // [TEST-ONLY] XYZ image accessor — no longer on the TraceBackend production
  // seam. Used by the CPU-vs-Metal parity harness; keep callers on the
  // concrete type, not a polymorphic base reference.
  void ReadbackImage(XyzImageData& out);
  // Exit seam (scrum-258.1+): buffer-egress contract — see TraceBackend.
  // Single-MS: returns the final layer's outgoing rays. Multi-MS semantics
  // (per-layer routing / filter / prob 分流) are owned by 258.3.
  // 258.2: returns `ExitRayRecord` carrying {dir, weight, path,
  // crystal_id, ms_layer_idx}; move-out — `exit_records_` is left empty.
  size_t ReadbackExitRays(std::vector<ExitRayRecord>& out) override;
  // task-268.4: per-layer destructive drain. On CPU this has identical
  // semantics to ReadbackExitRays (both move out `exit_records_`) — the
  // CPU backend never accumulated session-level state because the host-side
  // vector is rebuilt every layer anyway.
  size_t DrainExits(std::vector<ExitRayRecord>& out) override;
  void EndSession() override;

  // Diagnostic accessors (unit tests).
  size_t RootRayCount() const { return root_ray_count_; }
  float TotalLandedWeight() const { return total_landed_weight_; }

  // Stochastic crystal-geometry draws this batch made, summed across every
  // (layer, ci). See TraceBackend for the cross-backend contract.
  size_t GetLastBatchStochasticCrystalSampleCount() const override { return stochastic_sample_count_this_batch_; }

  // Stochastic ORIENTATION draws this batch made — rays, not geometries. See
  // TraceBackend for the cross-backend contract and why the two counts differ
  // by orders of magnitude on the same scene.
  size_t GetLastBatchStochasticOrientationSampleCount() const override {
    return stochastic_orientation_sample_count_this_batch_;
  }

 private:
  SessionSpec spec_{};
  RandomNumberGenerator rng_;
  Rotation camera_rot_;
  std::unique_ptr<float[]> xyz_buf_;
  int width_ = 0;
  int height_ = 0;

  size_t root_ray_count_ = 0;
  // Output register for GetLastBatchStochasticCrystalSampleCount: zeroed every
  // BeginSession, incremented per (layer, ci) whose params actually draw. Purely
  // per-batch — no cross-batch state is needed, because the shapes that ARE
  // reused across batches are the deterministic ones, and those are counted from
  // the config rather than from any batch.
  size_t stochastic_sample_count_this_batch_ = 0;
  // Output register for GetLastBatchStochasticOrientationSampleCount, same
  // per-batch lifecycle as the counter above. Incremented by the whole
  // population's ray count (not by 1) per (layer, ci) whose axis draws, because
  // orientation is resampled per ray with no reuse at all.
  size_t stochastic_orientation_sample_count_this_batch_ = 0;
  size_t ms_idx_ = 0;  // advances on each Recombine.
  float total_landed_weight_ = 0.0f;

  // Backend-owned storage for the device-resident handle returned by Recombine.
  // Lifetime: from the Recombine that writes it through to the NEXT Recombine
  // (or EndSession). The DeviceRayBatch::backend_ptr handed out by Recombine
  // points at &continuation_buf_.
  RayBuffer continuation_buf_;

  // Exit seam (scrum-258.1/258.2): session-level accumulator for rich
  // world-space exit records {dir, weight, path, crystal_id, ms_layer_idx},
  // appended-to by every TraceLayer in this session. Single-MS: equals the
  // only layer's outgoing set; multi-MS: union of every layer's outgoings
  // (per-layer routing / filter / prob owned by 258.3). BeginSession /
  // EndSession reset; ReadbackExitRays moves out and returns the count.
  std::vector<ExitRayRecord> exit_records_;

  // Design 2 (task-engine-redirect-design2): placement-scoped color-predicate
  // → component-bit table, built from SessionSpec::raypath_color once in
  // BeginSession. TraceLayer slices it per (ms_idx_, crystal_id) via
  // ColorGatePlacementFor and hands the placement to CollectData, which
  // evaluates the color predicates as a non-destructive pass and ORs the
  // matched bits into each surviving ray's mask (piped out through
  // ExitRayRecord::component_mask). Empty table (no raypath_color configured)
  // → zero-cost no-op in the emit gate.
  ColorGateTable color_gate_table_;

  bool in_session_ = false;
  // Track whether rng_ has been seeded by a prior BeginSession in this
  // backend's lifetime. Simulator calls BeginSession / EndSession per
  // SimBatch (server.cpp:77 `kDefaultRayNum=128`); without this guard every
  // batch would reset rng_ back to spec.seed, collapsing axis-sample
  // diversity to a single 128-ray sequence repeated thousands of times.
  // See progress.md DONE 2026-06-10 15:35 (task-filter-parity-rootcause-fix).
  // To reseed, destroy and recreate the backend instance.
  bool seeded_ = false;
  uint32_t seeded_seed_ = 0;  // seed value used when seeded_ was set
};

}  // namespace lumice

#endif  // CORE_CPU_TRACE_BACKEND_H_
