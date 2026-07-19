// MetalTraceBackend test-hooks (scrum-328.2 Step 3).
//
// Off-band harness for the Metal backend. Semantics mirror
// `CudaTraceBackendTestHooks` — see that header for the friend-class access
// mechanism + collection rationale. Known-exception `*ForTest` method
// deliberately retained on `MetalTraceBackend` itself:
//   - `TraceLayerKernelMaxThreadsForTest` — scrum-267 occupancy regression
//     guard, an operational-side observation (not a per-ray white-box injection
//     point), kept as-is per plan §"Step 3 已知例外".
//
// scrum-328.2 Step 3 收纳清单（枚举，与 plan.md 对齐）：
//   - SetInitialRayBase        (was `SetInitialRayBaseForTest`)
//   - ReadbackRootRot          (was `ReadbackRootRotForTest`)
//   - ReadbackGenDirs          (was `ReadbackGenDirsForTest`, Step 2 new)
//   - EnableGenAttemptCount    (was `EnableGenAttemptCountForTest`, Step 1 new)
//   - ReadbackGenAttemptCount  (was `ReadbackGenAttemptCountForTest`, Step 1 new)
#ifndef CORE_METAL_TRACE_BACKEND_TEST_HOOKS_H_
#define CORE_METAL_TRACE_BACKEND_TEST_HOOKS_H_

#if defined(__APPLE__)

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace lumice {

class MetalTraceBackend;  // forward

class MetalTraceBackendTestHooks {
 public:
  explicit MetalTraceBackendTestHooks(MetalTraceBackend& backend) : backend_(backend) {}

  // Per-stream 64-bit PCG ray-base injection (root_gen / transit; Metal's emit
  // gate uses tid directly, so no gate hi stream — see scrum-267 §3.6).
  void SetInitialRayBase(size_t root_base, size_t transit_base);

  // Read of the per-ray crystal→world rotation matrix pool (9 floats/ray).
  // For transit layers this is the transit kernel's sampled orientation.
  size_t ReadbackRootRot(std::vector<float>& out, size_t count);

  // scrum-328.2 Step 2: direct read of gen_root_kernel's crystal-local ray
  // directions (3 floats/ray). Unified memory → plain memcpy.
  size_t ReadbackGenDirs(std::vector<float>& out, size_t count);

  // scrum-328.2 Step 1 attempt-count observation (Metal-symmetric with CUDA;
  // near-pole acceptance-rate smoke consumer).
  void EnableGenAttemptCount(size_t count, size_t ci_start = 0);
  size_t ReadbackGenAttemptCount(std::vector<int>& out, size_t count);

  // K-shape geometry pool observability:
  //   * ReadbackPoolShapeTable — copy the host-side pool_shape_table_h_
  //     (per-shape {poly_off, poly_cnt, tri_off, tri_cnt}). Reflects the pool
  //     built by the most recent ResolveLayerCrystalForCi; .size() == P_ci.
  //   * ReadbackRootPoolShape — copy the first `count` (poly_off, poly_cnt)
  //     entries the last root pass wrote for its rays. Enables an AC1 probe
  //     that different rays land on different pool slots.
  //   * PoolShapeCountThisBatch — running sum of P_ci across every (layer,
  //     ci) resolve inside the current session (BeginSession resets to 0).
  //     Same value the CLI stat `Stats: crystals=N` reports at EndSession.
  std::vector<std::array<uint32_t, 4>> ReadbackPoolShapeTable();
  std::vector<std::pair<uint32_t, uint32_t>> ReadbackRootPoolShape(size_t count);
  size_t PoolShapeCountThisBatch() const;

 private:
  MetalTraceBackend& backend_;
};

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // CORE_METAL_TRACE_BACKEND_TEST_HOOKS_H_
