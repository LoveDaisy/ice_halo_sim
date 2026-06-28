#ifndef CORE_CUDA_TRACE_BACKEND_H_
#define CORE_CUDA_TRACE_BACKEND_H_

// CUDA backend for the TraceBackend seam (NVIDIA GPUs).
//
// This header is pure C++ and does not include any CUDA Runtime headers; all
// device state lives behind an opaque pimpl in cuda_trace_backend.cu. The seam
// is validated from a third orthogonal vantage point (CPU unified-memory +
// Apple unified-memory + NVIDIA discrete-memory PCIe) — see
// core/backend/trace_backend.hpp design invariant #4.
//
// MVP scope (scrum-cuda-backend-mvp subtask 3): single MS, no filter, no prob
// 分流, host root upload, per-ray crystal orientation, WlPoolSize()=0
// (discrete-wl path). multi-MS / device root-gen / WlPool / GUI throughput
// live in follow-up subtasks.
//
// Build gate: header body is compiled only when LUMICE_CUDA_ENABLED is defined
// (set by CMake when LUMICE_CUDA_ENABLED=ON). Other translation units include
// this header unconditionally; the empty body keeps Mac/Windows host builds
// zero-regress.

#if defined(LUMICE_CUDA_ENABLED)

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/backend/trace_backend.hpp"

namespace lumice {

class Logger;

// Runtime probe for CUDA device availability. Returns true iff at least one
// CUDA-capable device is enumerated by the driver (cudaGetDeviceCount > 0). Has
// no side effects on the cached state besides the one-time probe; safe to call
// from any thread. Result is cached after the first call (std::call_once), so
// subsequent calls are a plain memory read. Distinct from
// CudaTraceBackend::BeginSession which sets the active device and allocates
// session buffers — that is the heavy-weight path.
bool CudaDeviceAvailable();

// CudaLayerHandle — opaque per-layer handle. Mirrors MetalLayerHandle: the only
// host-visible scalar produced per TraceLayer is the continuation count
// (populated from a 4-byte cudaMemcpy D2H). Continuation ray buffers live in
// the session-level Impl::cont_* pool when multi-MS lands; MVP returns 0
// continuation count because Recombine is a stub.
class CudaLayerHandle : public LayerHandle {
 public:
  explicit CudaLayerHandle(size_t continuation_count, LayerStats stats)
      : continuation_count_(continuation_count), stats_(stats) {}
  size_t ContinuationCount() const override { return continuation_count_; }
  LayerStats GetLayerStats() const override { return stats_; }

 private:
  size_t continuation_count_ = 0;
  LayerStats stats_{};
};

// CudaTraceBackend — NVIDIA GPU backend for TraceBackend.
//
// MVP constraints:
//   - Single MS only (no Recombine continuation; final-layer DrainExits is the
//     only egress).
//   - No DeviceFilter / prob 分流 — every traced ray is emitted.
//   - WlPoolSize() == M (296.6 DR-3) — one session covers the whole spectrum via
//     a device wl_pool; each ray carries a per-ray wl_idx and the exit records
//     feed SimData.outgoing_wl_ for host-side per-ray CMF (mirrors Metal).
//   - Per-ray crystal orientation — InitRayFirstMs samples one orientation
//     per root ray (halo-ring distribution comes from this); the kernel
//     indexes the per-ray d_rot_c2w buffer by tid before applying frame
//     invariant 6 (mirrors metal_trace_backend.mm's per-ray rot upload).
//   - HostRayBatch ingest only — RootRaySource::FromDevice never reaches
//     TraceLayer.
class CudaTraceBackend : public TraceBackend {
 public:
  explicit CudaTraceBackend(Logger* logger = nullptr);
  ~CudaTraceBackend() override;

  CudaTraceBackend(const CudaTraceBackend&) = delete;
  CudaTraceBackend(CudaTraceBackend&&) = delete;
  CudaTraceBackend& operator=(const CudaTraceBackend&) = delete;
  CudaTraceBackend& operator=(CudaTraceBackend&&) = delete;

  void BeginSession(const SessionSpec& spec) override;
  LayerHandlePtr TraceLayer(const RootRaySource& roots) override;
  RootRaySource Recombine(LayerHandlePtr handle, const RecombineSpec& spec) override;
  size_t ReadbackExitRays(std::vector<ExitRayRecord>& out) override;
  size_t DrainExits(std::vector<ExitRayRecord>& out) override;
  // S2 device-fused XYZ accumulation overrides — mirrors MetalTraceBackend.
  // HasDeviceXyzAccum reports true so the simulator routes per-batch egress
  // through ReadbackXyzAccum (W*H*3 D2H) instead of the per-exit DrainExits
  // round-trip. IsCompatible mirrors Metal's constraints (rectangular @ zenith
  // view, or dual_fisheye_equal_area @ fov≈180); other lens configs fall back
  // to legacy CPU via simulator backend dispatch.
  bool HasDeviceXyzAccum() const override;
  void ReadbackXyzAccum(XyzImageData& xyz, float& landed_weight) override;
  bool IsCompatible(const RenderConfig& render) const override;
  void EndSession() override;
  // Per-ray wavelength pool size M (296.6 DR-3). Non-zero routes the driving
  // loop (simulator.cpp) through the single-session per-ray-wl path. Resolved
  // from env on demand so it answers correctly even before BeginSession (the
  // driving loop queries it first). Mirrors MetalTraceBackend::WlPoolSize().
  uint32_t WlPoolSize() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)

#endif  // CORE_CUDA_TRACE_BACKEND_H_
