#ifndef CORE_METAL_TRACE_BACKEND_H_
#define CORE_METAL_TRACE_BACKEND_H_

// Metal backend for the TraceBackend seam (Apple platforms only).
//
// This header is intentionally pure C++ and does NOT depend on Metal/Metal.h:
// the Objective-C++ implementation (`metal_trace_backend.mm`) hides all Metal
// types behind an opaque pimpl. The seam validates the contract from a second,
// orthogonal vantage point (see core/backend/trace_backend.hpp design invariant #4) —
// recorder runs backend-local, continuation rays stay device-resident across
// the layer boundary, and only a 4-byte counter readback crosses host/device
// mid-session.

#if defined(__APPLE__)

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/backend/trace_backend.hpp"

namespace lumice {

class Logger;

// Runtime probe for Metal device availability. Returns true iff a Metal device
// is present: first tries MTLCreateSystemDefaultDevice(), and if that is nil
// falls back to MTLCopyAllDevices() being non-empty (mirrors EnsureDevice's
// two-step probe; on some Macs the default-device query can return nil while
// MTLCopyAllDevices still enumerates a usable device). Result is cached after
// the first call (std::call_once), so subsequent calls are a plain memory read;
// safe to call from any thread. Has no side effects on the cached result besides
// the one-time probe — distinct from MetalTraceBackend::Impl::EnsureDevice,
// which asserts on failure and retains the device reference.
bool MetalDeviceAvailable();

// Deeper Metal availability probe — trial-compiles the same MSL source as
// EnsurePso (kFilterMatchHelperSrc + kKernelSrc with MSL 3.0 + MTLMathModeSafe)
// and verifies all three kernel entry points (trace_layer_kernel,
// gen_root_kernel, transit_root_kernel) resolve via newFunctionWithName. This
// catches the macOS 26.5 failure mode where the library compiles but kernel
// entry points are not exposed — a scenario MetalDeviceAvailable cannot
// observe. The cached result drives LUMICE_IsBackendAvailable so the GUI's
// "Use GPU" checkbox (Metal on this host) is gated by the actual runtime success criterion
// rather than mere device presence. Result is cached after the first call
// (std::call_once, write-once then read-only — thread-safe).
bool MetalPipelineAvailable();

// MetalLayerHandle — the only host-visible scalar produced per TraceLayer is
// the continuation count (populated from a 4-byte device readback, equivalent
// to a 4-byte cudaMemcpy for CUDA backends). Continuation ray buffers
// themselves are owned by the backend's session-level pool (Impl::cont_*).
class MetalLayerHandle : public LayerHandle {
 public:
  explicit MetalLayerHandle(size_t continuation_count, LayerStats stats)
      : continuation_count_(continuation_count), stats_(stats) {}
  size_t ContinuationCount() const override { return continuation_count_; }
  LayerStats GetLayerStats() const override { return stats_; }

 private:
  size_t continuation_count_ = 0;
  LayerStats stats_{};
};

// MetalTraceBackend — Metal GPU backend for TraceBackend.
//
// v1 constraints:
//   - kRectangular lens and zenith view (el≈90°) only.
//   - Each MS layer must have exactly one crystal setting.
//
// Recombine honors RecombineSpec.shuffle: when true, a device-side
// shuffle_cont_kernel (Feistel gather) decorrelates the continuation pool's
// per-parent-CI grouping, mirroring legacy host Fisher-Yates
// (task-gpu-backend-recombine-shuffle). shuffle = false skips the permutation.
class MetalTraceBackend : public TraceBackend {
 public:
  explicit MetalTraceBackend(Logger* logger = nullptr);
  ~MetalTraceBackend() override;

  MetalTraceBackend(const MetalTraceBackend&) = delete;
  MetalTraceBackend(MetalTraceBackend&&) = delete;
  MetalTraceBackend& operator=(const MetalTraceBackend&) = delete;
  MetalTraceBackend& operator=(MetalTraceBackend&&) = delete;

  void BeginSession(const SessionSpec& spec) override;
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override;
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override;
  // [TEST-ONLY] XYZ image accessor — not on the TraceBackend production seam;
  // production exit egress uses DrainExits(). Used by the CPU-vs-Metal parity
  // harness; keep callers on the concrete type, not a polymorphic base reference.
  void ReadbackImage(XyzImageData& out);
  // [TEST-ONLY] task-358.3 (renamed from SetCaptureComponent / ReadbackComponent
  // Capture after Fork-C retirement, closing blueprint §6(c) decoupling): the
  // ray's per-layer `this_mask` is now purely Design-2 colour bits (Fork-C
  // physical-bit produce branch removed). SetCaptureRayMask(true) enables the
  // per-session capture ring — the emit gate appends (this_mask, weight) of
  // every emitted ray, and cross-layer carry stays on regardless. Must be
  // called before BeginSession. Default false → the append branch is skipped
  // (production zero-cost). ReadbackRayMask returns the accumulated (mask,
  // weight) pairs for CPU parity comparison.
  void SetCaptureRayMask(bool enable);
  void ReadbackRayMask(std::vector<uint64_t>& masks, std::vector<float>& weights) const;
  // [PARITY-ONLY] Exit seam buffer-egress contract — see base class.
  // Returns rich `ExitRayRecord` (36B each) — see core/exit_seam.hpp.
  // Not invoked on the production path; DrainExits is the production exit
  // seam. Retained as a parity testing contract — virtual override stays
  // for the parity harness.
  size_t ReadbackExitRays(std::vector<ExitRayRecord>& out) override;
  // task-268.4: per-layer destructive drain. Reads back the current exit
  // buffer contents AND resets the device slot counter so the buffer can
  // be recycled across MS layers. See base class doc for the contract.
  size_t DrainExits(std::vector<ExitRayRecord>& out) override;
  // task-metal-device-fused-consumer (S1): device-fused XYZ accumulation.
  // SupportsDeviceXyzAccum() = true for this backend — the trace kernel emits
  // directly into a session-resident W*H*3 atomic_float buffer, and the
  // simulator reads it back once per batch via ReadbackXyzAccum instead
  // of materialising per-exit records.
  bool SupportsDeviceXyzAccum() const override { return true; }
  void ReadbackXyzAccum(XyzImageData& xyz, float& landed_weight) override;
  // task-358.1 Step 4 (AC3 device-side Y-lane accumulation): copy the flattened
  // per-color-class Y accumulator to host and reset the device side for the
  // next window. See TraceBackend::ReadbackClassLanes for contract + layout.
  void ReadbackClassLanes(std::vector<float>& lane_data, size_t& class_count) override;
  // scrum-312.4: Metal joins the third clock (persistent cross-batch XYZ
  // accumulator + display-cadence drain). Unlike CUDA the payoff is architectural
  // uniformity, not throughput — unified-memory readback was already ~free.
  bool SupportsThirdClockDrain() const override { return true; }
  void EndSession() override;
  bool IsCompatible(const RenderConfig& render) const override;
  // scrum-268.8 (DR-3): per-ray wavelength pool size. Resolved from
  // LUMICE_WL_POOL_SIZE env var the first time BeginSession runs; constant
  // thereafter for the backend instance's lifetime.
  uint32_t WlPoolSize() const override;

  // K-shape pool: total distinct crystal INSTANCES built during this batch
  // across every (layer, ci) — the sum of P_ci over the whole scene. At
  // LUMICE_GPU_GEOM_CLOCK unset (P_ci = 1 per ci) this equals the cross-layer
  // instance count. Simulator reads it after the MS loop to fill
  // SimData.crystal_count_ for stats reporting.
  size_t GetLastBatchCrystalCount() const override;
  // task-color-degrade-gui-surfacing: per-config GPU color-degrade tally.
  ColorDegradeCounts GetLastColorDegradeCounts() const override;

  // [TEST-ONLY] Return the trace_layer_kernel PSO's
  // maxTotalThreadsPerThreadgroup (or 0 if BeginSession has not yet built the
  // PSO). Used by the scrum-267 task-fused-emit-gate occupancy regression
  // guard — the device-side emit gate added 64B of thread-local path scratch
  // + the DeviceFilterCheck call graph, and plan §9 mandates ≥1024
  // maxThreadsPerThreadgroup as the wavefront-stay-fused acceptance bar
  // (drop triggers R1 option B = split gate into its own dispatch).
  size_t TraceLayerKernelMaxThreadsForTest() const;

  // [TEST-ONLY] task-metal-gui-commit-backpressure O2 AC1 whitebox: expose
  // opaque device / trace_layer_kernel PSO pointer identity for the cross-
  // instance regression guard (two MetalTraceBackend instances must observe
  // the same underlying MTLDevice / MTLComputePipelineState pointers, proving
  // the process-level cache is shared). Returns nullptr before BeginSession
  // has populated Impl::device / Impl::pso via EnsureDevice / EnsurePso.
  // Void-pointer return keeps the header pure C++ (no MTLDevice leak).
  const void* GetDevicePtrForTest() const;
  const void* GetPsoPtrForTest() const;

  // scrum-328.2 Step 3: test-only observation + injection points migrated to
  // `MetalTraceBackendTestHooks` (see `metal_trace_backend_test_hooks.hpp`).
  // Known exception retained on this class: `TraceLayerKernelMaxThreadsForTest`
  // above — operational occupancy guard, not a per-ray white-box injection
  // point (plan §"Step 3 已知例外").
  friend class MetalTraceBackendTestHooks;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // CORE_METAL_TRACE_BACKEND_H_
