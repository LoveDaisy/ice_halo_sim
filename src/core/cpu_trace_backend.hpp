#ifndef CORE_CPU_TRACE_BACKEND_H_
#define CORE_CPU_TRACE_BACKEND_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "config/sim_data.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/trace_backend.hpp"

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

  void BeginSession(const SessionSpec& spec) override;
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override;
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override;
  // Test-only XYZ image accessor (scrum-258.1 Step 5: no longer on the
  // TraceBackend production seam). Used by the CPU-vs-Metal parity harness
  // — keep callers on the concrete type, not a polymorphic base reference.
  void ReadbackImage(XyzImageData& out);
  // Exit seam (scrum-258.1+): buffer-egress contract — see TraceBackend.
  // Single-MS: returns the final layer's outgoing rays. Multi-MS semantics
  // (per-layer routing / filter / prob 分流) are owned by 258.3.
  // 258.2: returns `ExitRayRecord` carrying {dir, weight, path,
  // crystal_id, ms_layer_idx}; move-out — `exit_records_` is left empty.
  size_t ReadbackExitRays(std::vector<ExitRayRecord>& out) override;
  void EndSession() override;

  // Diagnostic accessors (unit tests).
  size_t RootRayCount() const { return root_ray_count_; }
  float TotalLandedWeight() const { return total_landed_weight_; }

 private:
  SessionSpec spec_{};
  RandomNumberGenerator rng_;
  Rotation camera_rot_;
  std::unique_ptr<float[]> xyz_buf_;
  int width_ = 0;
  int height_ = 0;

  size_t root_ray_count_ = 0;
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

  bool in_session_ = false;
  // Track whether rng_ has been seeded by a prior BeginSession in this
  // backend's lifetime. Simulator calls BeginSession / EndSession per
  // SimBatch (server.cpp:77 `kDefaultRayNum=128`); without this guard every
  // batch would reset rng_ back to spec.seed, collapsing axis-sample
  // diversity to a single 128-ray sequence repeated thousands of times.
  // See progress.md DONE 2026-06-10 15:35 (task-filter-parity-rootcause-fix).
  bool seeded_ = false;
};

}  // namespace lumice

#endif  // CORE_CPU_TRACE_BACKEND_H_
