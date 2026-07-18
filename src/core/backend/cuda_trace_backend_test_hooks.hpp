// CudaTraceBackend test-hooks (scrum-328.2 Step 3).
//
// Off-band harness for the CUDA backend: exposes the `*ForTest` observation +
// injection points that white-box tests need, while leaving `CudaTraceBackend`
// itself with a production-clean public interface (`grep -rn ForTest
// src/core/backend/cuda_trace_backend.hpp` should hit only the enumerated
// known exception `GetLastBatchCrystalCount` — which is production and NOT a
// test-only symbol; kept in the class body as-is).
//
// Access mechanism: this class is declared a `friend` of `CudaTraceBackend` so
// its methods can reach the private pimpl `impl_`. The method bodies live in
// `cuda_trace_backend.cu` (same TU as `CudaTraceBackend::Impl` — otherwise
// `impl_` would be an incomplete type). Constructing this class is free (a
// reference wrapper); callers hand it a live `CudaTraceBackend&` between
// `BeginSession` and `EndSession` and invoke the same methods that used to
// live directly on the backend.
//
// scrum-328.2 Step 3 收纳清单（枚举，与 plan.md 对齐）：
//   - SetInitialRayBase        (was `SetInitialRayBaseForTest`)
//   - ReadbackGenDirs          (was `ReadbackGenDirsForTest`)
//   - EnableRngProbe           (was `EnableRngProbeForTest`)
//   - ReadbackRngProbe         (was `ReadbackRngProbeForTest`)
//   - EnableGenAttemptCount    (was `EnableGenAttemptCountForTest`, Step 1 new)
//   - ReadbackGenAttemptCount  (was `ReadbackGenAttemptCountForTest`, Step 1 new)
#ifndef CORE_CUDA_TRACE_BACKEND_TEST_HOOKS_H_
#define CORE_CUDA_TRACE_BACKEND_TEST_HOOKS_H_

#if defined(LUMICE_CUDA_ENABLED)

#include <cstddef>
#include <vector>

#include "core/backend/rng_probe_stream.hpp"

namespace lumice {

class CudaTraceBackend;  // forward

class CudaTraceBackendTestHooks {
 public:
  explicit CudaTraceBackendTestHooks(CudaTraceBackend& backend) : backend_(backend) {}

  // Per-stream 64-bit PCG ray-base injection (gen / transit / gate).
  void SetInitialRayBase(size_t gen_base, size_t transit_base, size_t gate_base);

  // Direct read of gen_root_kernel's crystal-local ray directions (3 floats/ray).
  size_t ReadbackGenDirs(std::vector<float>& out, size_t count);

  // scrum-328.2 Step 1: explicit-stream RNG probe (raw pcg_uniform draw).
  void EnableRngProbe(RngProbeStream stream, size_t count, size_t ci_start = 0);
  size_t ReadbackRngProbe(std::vector<float>& out, size_t count);

  // scrum-328.2 Step 1: per-ray attempt-count observation (mean(attempts) =
  // 1/accept_ratio at the gen sampler; doc/near-pole-area-measure-sampling.md
  // anchors 4.90/3.76 for Laplacian b=5 / Gaussian σ=5).
  void EnableGenAttemptCount(size_t count, size_t ci_start = 0);
  size_t ReadbackGenAttemptCount(std::vector<int>& out, size_t count);

  // White-box observation of `CudaTraceBackend::Impl::BuildGeomPool` call
  // frequency. `EnableGeomPoolRebuildCount` flips the production zero-cost
  // counter on (call BEFORE any BeginSession to observe the first build);
  // `ReadbackGeomPoolRebuildCount` reads the count of `BuildGeomPool`
  // invocations since Impl construction (or last observation). Distinguishes
  // stochastic scenes (count grows with BeginSession cycles) from
  // deterministic scenes (count stays at 1 across cycles — the pool
  // build-once fast path). No device buffers to poke; no session-boundary
  // constraints.
  void EnableGeomPoolRebuildCount();
  uint32_t ReadbackGeomPoolRebuildCount() const;

 private:
  CudaTraceBackend& backend_;
};

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)

#endif  // CORE_CUDA_TRACE_BACKEND_TEST_HOOKS_H_
