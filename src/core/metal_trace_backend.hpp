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

#include "core/trace_backend.hpp"

namespace lumice {

// MetalLayerHandle — the only host-visible scalar produced per TraceLayer is
// the continuation count (populated from a 4-byte device readback, equivalent
// to a 4-byte cudaMemcpy for CUDA backends). Continuation ray buffers
// themselves are owned by the backend's session-level pool (Impl::cont_*).
class MetalLayerHandle : public LayerHandle {
 public:
  MetalLayerHandle(size_t continuation_count, LayerStats stats)
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
  MetalTraceBackend();
  ~MetalTraceBackend() override;

  MetalTraceBackend(const MetalTraceBackend&) = delete;
  MetalTraceBackend(MetalTraceBackend&&) = delete;
  MetalTraceBackend& operator=(const MetalTraceBackend&) = delete;
  MetalTraceBackend& operator=(MetalTraceBackend&&) = delete;

  void BeginSession(const SessionSpec& spec) override;
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override;
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override;
  void ReadbackImage(XyzImageData& out) override;
  void EndSession() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // CORE_METAL_TRACE_BACKEND_H_
