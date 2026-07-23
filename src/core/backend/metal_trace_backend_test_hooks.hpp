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

  // Direct read of the per-ray crystal-local entry point (3 floats/ray) the
  // gen_root / transit_root kernels wrote into root_p_buf (both dispatches bind
  // the same shared buffer, so this reads gen's samples after a gen TraceLayer
  // and transit's after a transit TraceLayer). Plain memcpy (unified memory).
  // The sibling entry-face id is ReadbackRootTf. Used by the per-ray white-box
  // gate that checks each captured (p, to_face) is geometrically consistent with
  // the captured direction (ReadbackGenDirs) and the crystal geometry.
  size_t ReadbackRootP(std::vector<float>& out, size_t count);

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

  // K-shape pool `path[]` locality probe: `rec_sink_buf` carries the per-ray
  // Σ float(path[k]) written at the tail of `trace_layer_kernel`. Under the
  // fixed contract, path[k] is a LOCAL polygon index within [0, PolygonFaceCount),
  // so `rec_sink[tid] <= max_hits × (PolygonFaceCount - 1)` regardless of the
  // shape the ray landed on. Pre-fix (path[k] absolute), rays landing on
  // shape s > 0 could sum poly_off × max_hits above this bound. Tests use
  // this to catch a regression to absolute-index storage without needing a
  // per-ray path readback.
  //
  // Convention note: this + ReadbackRootTf take an out-parameter vector and
  // return the actual element count copied. This diverges from the earlier
  // by-value Readback* methods above (ReadbackPoolShapeTable,
  // ReadbackRootPoolShape) — the out-param form lets tests reuse a pre-sized
  // vector across many batches without repeated allocation, which matters for
  // the large per-ray probes here (up to `num_rays` floats/uint32_t entries
  // per batch, vs. the O(pool_size) small vectors the earlier methods return).
  // Future large per-ray Readback* additions should follow this out-param
  // form; small O(pool_size) ones can keep by-value.
  size_t ReadbackRecSink(std::vector<float>& out, size_t count);
  // K-shape pool `root_tf` widened absolute-index readback. Each entry is the
  // pool-wide polygon index of the ray's initial hit face (or kInvalidId =
  // 0xffffffff on "triangle has no polygon backing"). Used by tests that
  // want to verify the widened 32-bit range without hitting sentinel
  // collisions. See ReadbackRecSink above for the out-param convention
  // rationale.
  size_t ReadbackRootTf(std::vector<uint32_t>& out, size_t count);

 private:
  MetalTraceBackend& backend_;
};

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // CORE_METAL_TRACE_BACKEND_TEST_HOOKS_H_
