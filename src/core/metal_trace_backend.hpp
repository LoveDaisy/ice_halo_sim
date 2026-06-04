#ifndef CORE_METAL_TRACE_BACKEND_H_
#define CORE_METAL_TRACE_BACKEND_H_

// Pure C++ Metal-backend skeleton. Provided to validate the TraceBackend seam
// from a SECOND, orthogonal vantage point (see core/trace_backend.hpp design
// invariant #4). Compiles on Apple platforms only.
//
// This header intentionally does NOT depend on Metal/Metal.h. The real
// kernels live in a future companion `.mm` translation unit; the stub here
// only proves the contract can be implemented under a discrete-memory
// device-pointer model.

#if defined(__APPLE__)

#include <cstddef>
#include <memory>

#include "core/trace_backend.hpp"

namespace lumice {

// MetalLayerHandle — in a real implementation this would own:
//   id<MTLBuffer> continuation_rays_;          // d/p/w/tf/rec parallel SoA on device
//   id<MTLBuffer> outgoing_d_;                 // streaming-compact append buf
//   id<MTLBuffer> outgoing_w_;
//   uint32_t continuation_count_host_mirror_;  // sole host-visible scalar
// All of the above stay device-resident across the layer's Trace+Scatter
// kernels — the host only observes the count via a tiny readback.
class MetalLayerHandle : public LayerHandle {
 public:
  MetalLayerHandle() = default;
  size_t ContinuationCount() const override { return continuation_count_; }

  // In the real implementation, only this scalar is host-visible mid-session.
  // Continuation rays themselves live in device memory.
  size_t continuation_count_ = 0;
};

// Metal-skeleton backend.
//
// State machine + lifetime contracts are inherited from TraceBackend; see
// core/trace_backend.hpp. The methods below are intentionally stubs — they
// exist to prove the seam compiles and links under the Metal vantage point.
class MetalTraceBackend : public TraceBackend {
 public:
  MetalTraceBackend() = default;
  ~MetalTraceBackend() override = default;

  // Real impl: allocate persistent device buffers (xyz accumulator W*H*3,
  // continuation pool capped at AllocateAllData(...) bound), build pipeline
  // state objects for trace/scatter kernels, capture spec.scene/spec.render
  // pointers.
  void BeginSession(const SessionSpec& spec) override { (void)spec; }

  // Real impl: encode a single MTLComputeCommandEncoder that runs the fused
  // trace -> recorder -> projection -> XYZ scatter-add kernel chain.
  // No mid-pipeline kernel boundaries — that is the GPU performance lever
  // (design invariant #1, core/trace_backend.hpp). Returns a
  // MetalLayerHandle whose `continuation_count_host_mirror_` is the only
  // host-visible scalar (matching the CUDA contract: 4-byte readback).
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override {
    (void)roots;
    return std::make_unique<MetalLayerHandle>();
  }

  // Real impl: dispatch a stream-compaction kernel over the LayerHandle's
  // continuation buffer (M2 unified memory makes this conceptually cheap;
  // the contract still pretends a PCIe transfer separates host/device so the
  // discrete-CUDA path inherits the same seam unchanged — see
  // core/trace_backend.hpp design invariant #3). DeviceRayBatch::backend_ptr
  // would map to a freshly-bound id<MTLBuffer> that the caller MUST NOT
  // free; lifetime ends at the next Recombine OR EndSession.
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override {
    (void)handle;
    (void)spec;
    DeviceRayBatch d;
    d.backend_ptr = nullptr;  // would be an id<MTLBuffer> cast
    d.count = 0;
    return RootRaySource::FromDevice(d);
  }

  // Real impl: blitEncoder copyFromBuffer(xyz_accumulator) to a host-shared
  // buffer, then memcpy into out.data. This is the sole device->host
  // boundary, mirroring CUDA's PCIe XYZ readback (W*H*3*sizeof(float)).
  void ReadbackImage(XyzImageData& out) override { (void)out; }

  void EndSession() override {}
};

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // CORE_METAL_TRACE_BACKEND_H_
