// CUDA backend for TraceBackend (NVIDIA GPUs).
//
// MVP stub stage (scrum-cuda-backend-mvp subtask 3, M1):
//   - CMake gate + class skeleton.
//   - CudaDeviceAvailable() runtime probe.
//   - BeginSession throws BackendUnavailableError so the simulator falls back
//     to legacy CPU until M2 lands.
//
// Build gate: this entire translation unit is added to lumice_obj only when
// LUMICE_CUDA_ENABLED is ON (see CMakeLists.txt). Other backends are
// uninvolved.

#include "core/backend/cuda_trace_backend.hpp"

#if defined(LUMICE_CUDA_ENABLED)

#include <cuda_runtime.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "util/logger.hpp"

namespace lumice {

namespace {

// One-shot probe of cudaGetDeviceCount. Cached after the first call. Errors
// (driver not installed, no devices, runtime mismatch) all collapse to false;
// the simulator interprets that as "no CUDA backend available, fall back to
// legacy CPU".
bool ProbeCudaDevice() {
  int n = 0;
  cudaError_t err = cudaGetDeviceCount(&n);
  if (err != cudaSuccess) {
    // Drain the error so subsequent CUDA calls do not see it as sticky.
    (void)cudaGetLastError();
    return false;
  }
  return n > 0;
}

}  // namespace

bool CudaDeviceAvailable() {
  static std::once_flag flag;
  static bool cached = false;
  std::call_once(flag, []() { cached = ProbeCudaDevice(); });
  return cached;
}

// --- Impl -----------------------------------------------------------------
//
// Heavy-weight session state lives here. M1 only declares the field that
// indicates whether a session is open; M2 fills out device buffers / pinned
// staging / counters; M3 wires the kernel.

struct CudaTraceBackend::Impl {
  Logger* logger = nullptr;
  bool in_session = false;
};

CudaTraceBackend::CudaTraceBackend(Logger* logger) : impl_(std::make_unique<Impl>()) {
  impl_->logger = logger;
}

CudaTraceBackend::~CudaTraceBackend() = default;

void CudaTraceBackend::BeginSession(const SessionSpec& spec) {
  (void)spec;
  // M1: device lifecycle + buffer pool land in M2. Until then, advertise the
  // backend as unavailable so the simulator routes the run to legacy CPU.
  // BackendUnavailableError is the contract for "no half-open session" — the
  // simulator catches it, drops the backend instance, and continues.
  throw BackendUnavailableError(
      "CudaTraceBackend::BeginSession not yet implemented (scrum-cuda-backend-mvp M2)");
}

LayerHandlePtr CudaTraceBackend::TraceLayer(const RootRaySource& roots) {
  (void)roots;
  return std::make_unique<CudaLayerHandle>(0u, LayerStats{});
}

RootRaySource CudaTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  (void)handle;
  (void)spec;
  // MVP single-MS: Recombine is a no-op stub. Caller MUST NOT pass the result
  // back into TraceLayer; the simulator's single-MS path never calls Recombine
  // anyway.
  return RootRaySource::FromDevice(DeviceRayBatch{});
}

size_t CudaTraceBackend::ReadbackExitRays(std::vector<ExitRayRecord>& out) {
  out.clear();
  return 0;
}

size_t CudaTraceBackend::DrainExits(std::vector<ExitRayRecord>& out) {
  out.clear();
  return 0;
}

void CudaTraceBackend::EndSession() {
  impl_->in_session = false;
}

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
