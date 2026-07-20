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

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
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

  // White-box readback of a produced-geometry scalar vector from the FIRST
  // host slot crystal currently in the geometry pool (`Impl::pool_crystals_`
  // front). Copies up to `count` polygon-face distances (the quantity a
  // stochastic `d_[]`/`h_` config actually randomizes) into `out`; returns
  // the number of floats written (0 if the pool is empty). Call BETWEEN
  // BeginSession and EndSession so the pool reflects the current cycle's draw.
  // Lets a test assert that successive same-seed BeginSession cycles produce
  // DIFFERENT geometry — the direct AC1 evidence that `rng_` advances across
  // batches (Layer 1 seed-once) rather than resetting to a frozen shape.
  size_t ReadbackFirstPoolCrystalGeom(std::vector<float>& out, size_t count) const;

  // K-shape pool observability (CUDA sibling of Metal
  // `MetalTraceBackendTestHooks::ReadbackPoolShapeTable` /
  // `ReadbackRootPoolShape`, metal_trace_backend_test_hooks.hpp:63-64).
  //
  //   * ReadbackPoolShapeTable — D2H copy of `d_pool_shape_table_` as a flat
  //     vector of `{poly_off, poly_cnt, tri_off, tri_cnt}` rows. .size() ==
  //     Σ P_ci over every (layer, ci) resolved in this BeginSession (aka the
  //     value `GetLastBatchCrystalCount()` reports). The device buffer's
  //     backing storage `Impl::pool_shape_slot_cap_` is set to
  //     `pool_crystals_.size()` at every BuildGeomPool (freed to 0 on
  //     scene-change / no-K path), so it is EXACTLY the current batch's
  //     Σ P_ci — not a static allocation upper bound; stale rows from a
  //     prior larger-K batch cannot leak through. Precondition: called
  //     BETWEEN BeginSession and EndSession.
  //   * ReadbackRootPoolShape — D2H copy of the first `count` entries of
  //     `d_root_pool_shape_` as `(poly_off, poly_cnt)` pairs. The pool-slot
  //     each ray's root landed on for the last TraceLayer pass (gen_root_kernel
  //     for layer 0 / transit_multi_ms_kernel for layer≥1). Fail-hard: throws
  //     `std::out_of_range` if `count > root_cap_` — silent truncation would
  //     hide a caller size-accounting bug behind a plausible-looking short
  //     vector until it surfaces on-device as `cudaErrorIllegalMemoryAccess`,
  //     which is far more expensive to trace than a host-side throw here.
  //
  // No `PoolShapeCountThisBatch()` sibling: CUDA already exposes the same
  // value on the production surface as `GetLastBatchCrystalCount()` — tests
  // read that directly (no reason to duplicate the getter here).
  //
  // Design note: both methods D2H-copy into a fresh vector rather than
  // out-parameter form (the by-value form Metal uses for these two, since
  // both return small O(pool_size) vectors — cheap to construct/return). The
  // out-parameter form is reserved for the large per-ray probes.
  std::vector<std::array<uint32_t, 4>> ReadbackPoolShapeTable() const;
  std::vector<std::pair<uint32_t, uint32_t>> ReadbackRootPoolShape(size_t count) const;

 private:
  CudaTraceBackend& backend_;
};

}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)

#endif  // CORE_CUDA_TRACE_BACKEND_TEST_HOOKS_H_
