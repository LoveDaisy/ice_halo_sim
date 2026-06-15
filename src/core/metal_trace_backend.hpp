#ifndef CORE_METAL_TRACE_BACKEND_H_
#define CORE_METAL_TRACE_BACKEND_H_

// Metal backend for the TraceBackend seam (Apple platforms only).
//
// This header is intentionally pure C++ and does NOT depend on Metal/Metal.h:
// the Objective-C++ implementation (`metal_trace_backend.mm`) hides all Metal
// types behind an opaque pimpl. The seam validates the contract from a second,
// orthogonal vantage point (see core/trace_backend.hpp design invariant #4) —
// recorder runs backend-local, continuation rays stay device-resident across
// the layer boundary, and only a 4-byte counter readback crosses host/device
// mid-session.

#if defined(__APPLE__)

#include <cstddef>
#include <memory>
#include <vector>

#include "core/trace_backend.hpp"

namespace lumice {

class Logger;

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
//   - Device-side shuffle is not implemented; callers must set
//     RecombineSpec.shuffle = false before calling Recombine().
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
  // Test-only XYZ image accessor (scrum-258.1 Step 5: no longer on the
  // TraceBackend production seam). Used by the CPU-vs-Metal parity harness
  // — keep callers on the concrete type, not a polymorphic base reference.
  void ReadbackImage(XyzImageData& out);
  // Exit seam (scrum-258.1/258.2): buffer-egress contract — see base class.
  // 258.2: returns rich `ExitRayRecord` (36B each) — see core/exit_seam.hpp.
  size_t ReadbackExitRays(std::vector<ExitRayRecord>& out) override;
  // task-268.4: per-layer destructive drain. Reads back the current exit
  // buffer contents AND resets the device slot counter so the buffer can
  // be recycled across MS layers. See base class doc for the contract.
  size_t DrainExits(std::vector<ExitRayRecord>& out) override;
  void EndSession() override;
  bool IsCompatible(const RenderConfig& render) const override;

  // Test-only: return the trace_layer_kernel PSO's maxTotalThreadsPerThreadgroup
  // (or 0 if BeginSession has not yet built the PSO). Used by the scrum-267
  // task-fused-emit-gate occupancy regression guard — the device-side emit
  // gate added 64B of thread-local path scratch + the DeviceFilterCheck call
  // graph, and plan §9 mandates ≥1024 maxThreadsPerThreadgroup as the
  // wavefront-stay-fused acceptance bar (drop triggers R1 option B = split
  // gate into its own dispatch).
  size_t TraceLayerKernelMaxThreadsForTest() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // CORE_METAL_TRACE_BACKEND_H_
