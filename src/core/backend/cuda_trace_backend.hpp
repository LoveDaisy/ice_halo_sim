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
#include <string>
#include <vector>

#include "core/backend/rng_probe_stream.hpp"
#include "core/backend/trace_backend.hpp"

namespace lumice {

class Logger;

// Runtime probe for CUDA device availability. Returns true iff at least one
// enumerated CUDA device meets the sm_61 PTX floor (a device below the floor,
// or none/driver-missing, yields false → legacy CPU fallback, never a crash).
// Safe to call from any thread; the probe runs once and is cached, so
// subsequent calls are a plain memory read. Distinct from
// CudaTraceBackend::BeginSession which sets the active device and allocates
// session buffers — that is the heavy-weight path.
bool CudaDeviceAvailable();

// One-shot human-readable summary of the CUDA device probe: device count,
// per-device name + compute capability, driver/runtime versions, and the
// selection verdict (incl. the sm capability floor). Logged once at backend
// routing time so an unavailable/degraded GPU is diagnosable from a single run
// rather than a second release cycle. Cached alongside CudaDeviceAvailable().
std::string CudaDeviceDiagnostics();

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
  // scrum-312: CUDA opts into third-clock drain (persistent device accumulator +
  // display-cadence readback) to shed the per-batch synchronous D2H readback tax.
  bool SupportsThirdClockDrain() const override { return true; }
  bool IsCompatible(const RenderConfig& render) const override;
  void EndSession() override;
  // Per-ray wavelength pool size M (296.6 DR-3). Non-zero routes the driving
  // loop (simulator.cpp) through the single-session per-ray-wl path. Resolved
  // from env on demand so it answers correctly even before BeginSession (the
  // driving loop queries it first). Mirrors MetalTraceBackend::WlPoolSize().
  uint32_t WlPoolSize() const override;

  // task-exit-seam-crystal-count: setting count of the final MS layer for the
  // current session. Backed by Impl::final_layer_crystals_ populated during
  // BeginSession — safe to read anytime the session is open.
  size_t GetLastBatchCrystalCount() const override;

  // [TEST-ONLY] task-gpu-rng-ray-index-uint64 white-box injection: pre-seed the
  // three per-session PCG ray-base counters (gen / transit / gate) INDEPENDENTLY
  // BEFORE the first TraceLayer so a dev49-only test can drive exactly one
  // device stream into a non-zero hi epoch (the other two held at hi==0) without
  // running >2^32 real rays. Per-stream bases let gate/transit be asserted in
  // isolation from gen. MUST be called AFTER BeginSession and BEFORE the first
  // TraceLayer, and only in test builds.
  void SetInitialRayBaseForTest(size_t gen_base, size_t transit_base, size_t gate_base);

  // [TEST-ONLY] task-gpu-rng-ray-index-uint64 white-box observation: copy the
  // first `count` device-gen'd ray directions (`d_dirs_`, crystal-local, 3
  // floats/ray) back to host. This is the DIRECT output of `gen_root_kernel` —
  // the exact kernel where `gen_ray_base_hi` mixes into the PCG seed that drives
  // the per-ray orientation sample — so it observes the hi wiring in isolation
  // from trace → emit → device-fused accumulation.
  //
  // Raw-TraceLayer harness coverage caveat (scrum-328.2 Step 0 recon):
  //   - `ReadbackXyzAccum` IS populated in raw-TraceLayer harness runs when
  //     the scene's `final_prob > 0` and the render is compatible
  //     (`GateStreamWireUp` in test_cuda_rich_exit.cpp exercises this).
  //   - `GenStreamWireUp` scenes deliberately set `final_prob=0.0f` (gen
  //     stream isolation): the emit path drops every ray so `ReadbackXyzAccum`
  //     stays zero and `DrainExits` returns empty — the design here is to
  //     observe `d_dirs_` directly, not to complain that the accum path is
  //     blind. The kernel wiring itself does write when `final_prob>0`.
  //
  // Contract: MUST be called AFTER a TraceLayer whose gen dispatch produced
  // >= `count` rays and BEFORE EndSession (which frees d_dirs_). Returns the
  // number of floats written (3 * count), or 0 if unavailable.
  size_t ReadbackGenDirsForTest(std::vector<float>& out, size_t count);

  // [TEST-ONLY] task-gpu-rng-ray-index-uint64 + scrum-328.2 Step 1: RNG-only
  // observation of a device PCG stream, isolated from ray physics + atomic-
  // compaction non-determinism. EnableRngProbeForTest allocates a per-ray probe
  // sink; the NEXT TraceLayer's kernel writes each thread's raw pcg_uniform
  // draw at probe[tid + ci_start] into it — but **only the kernel matching
  // the explicit `stream` argument** actually writes (host passes nullptr to
  // the other kernels), replacing the pre-scrum-328.2 implicit "whichever
  // kernel is executed next" routing:
  //   - RngProbeStream::kGen        → gen_root_kernel writes the first
  //                                    pcg_uniform draw of its per-ray PCG
  //                                    stream. (scrum-328.2 Step 1 new.)
  //   - RngProbeStream::kGateMs1    → trace_single_ms_kernel writes the emit
  //                                    gate's first draw when ms_mode==1
  //                                    (per-bounce, gate_ray_base_hi).
  //   - RngProbeStream::kGateFinal  → trace_single_ms_kernel writes the emit
  //                                    gate's first draw when ms_mode==0
  //                                    (final layer, gate_ray_base_final_hi).
  //                                    Reserved wiring; add the ms_mode==0
  //                                    probe write when a consumer needs it.
  //   - RngProbeStream::kTransit    → transit_multi_ms_kernel writes the
  //                                    transit stream's first draw.
  // `ci_start` addresses probe[tid + ci_start] so a multi-crystal-instance
  // dispatch series does not silently overwrite prior CIs' draws (the pre-
  // scrum-328.2 wiring indexed by raw `tid`, i.e. only single-ci scenes were
  // safe). The draw is a pure function of (mixed_seed, tid) — deterministic,
  // unlike d_dirs_/image which mix in the non-deterministic continuation ray
  // at that tid. A non-zero hi on the probed stream must move the draws;
  // hi==0 runs are bit-identical.
  void EnableRngProbeForTest(RngProbeStream stream, size_t count, size_t ci_start = 0);
  size_t ReadbackRngProbeForTest(std::vector<float>& out, size_t count);

  // [TEST-ONLY] scrum-328.2 Step 1 attempt-count observability. Allocates a
  // per-ray sink sibling to the RNG probe; gen_root_kernel unconditionally
  // writes each ray's kLatPathGenericReject inner-loop iteration count when
  // the sink is bound (mean(attempts) = 1/accept_ratio, the ONLY direct route
  // to a per-ray acceptance-rate observation on device). Independent of
  // EnableRngProbeForTest (no shared enable state), Metal-symmetric. Consumer:
  // scrum-328 near-pole area-measure sampler smoke tests / parity harness
  // (mean(attempts) → doc/near-pole-area-measure-sampling.md anchor 4.90 for
  // Laplacian b=5, 3.76 for Gaussian σ=5). `ci_start` mirrors the RNG probe
  // ci addressing so multi-ci scenes can be observed without overwrite.
  void EnableGenAttemptCountForTest(size_t count, size_t ci_start = 0);
  size_t ReadbackGenAttemptCountForTest(std::vector<int>& out, size_t count);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)

#endif  // CORE_CUDA_TRACE_BACKEND_H_
