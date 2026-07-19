// CUDA-only K-shape pool structural + locality tests (mirrors the Metal
// KShapePool_* suite in test_metal_trace_backend.cpp:400-972). Three tests
// live in this TU:
//
//   * Test A (KShapePool_PathIsLocalWithinPolygonFaceCount_AC1) —
//     `d_root_pool_shape_` + `ExitRayRecord::path[]` locality invariants.
//     K=8, ci_n=64, hex-prism (PolygonFaceCount=8) → P_ci=8, so pool_offs
//     span [0, 56] which stay < 256 (uint8_t path storage capacity); every
//     value in `ExitRayRecord::path[]` for a raw layer-0 exit MUST be < 8
//     under the LOCAL-index contract. Pre-fix (path[k] absolute), rays on
//     shape s > 0 would write poly_off + local into path[] → values ≥ 8 on
//     shapes with poly_off ≥ 8. This is the CUDA sibling of the Metal
//     absolute-`path[]`-index regression the K-shape pool was hardened
//     against; CUDA's
//     static claim (`d_root_pool_shape.` design comment `:1836-1842`) is
//     that the invariant holds by construction — this test turns that claim
//     into runtime evidence. Uses raw layer-0 `ExitRayRecord`s via
//     `ReadbackExitRays` (mid-layer emits go through DrainExits's
//     `ms_layer_idx != final_layer` verbatim branch → path[] is exactly
//     what the kernel wrote, no GetFn remap yet).
//
//   * Test B (KShapePool_TransitPicksMultipleShapes_AC1) — transit
//     multi-MS layer-1 K-shape pick coverage. Larger scene (ci_n=4096,
//     P_ci=512 per layer) so transit sees hundreds of continuation rays;
//     asserts `transit_multi_ms_kernel` distributed layer-1 rays across ≥2
//     distinct pool slots. The ms_layers=1-only pre-existing tests
//     never surface a transit-side regression.
//
//   * Test C (KShapePool_DefaultKnobUnsetGivesPCiOne_AC2) — knob-off
//     structural collapse: with `LUMICE_GPU_GEOM_CLOCK` unset, P_ci ≡ 1
//     (one pool shape per (layer, ci)) and `GetLastBatchCrystalCount()`
//     equals the total (layer, ci) count. Complements the driver-verified
//     four-file parity battery (11 passed knob-off) with a structural
//     assertion that pins the AC2 contract at the pool level (as a
//     white-box structural anchor, not a re-verification of bit-equivalence
//     — that continues to be enforced by the four-file parity battery).
//
// Build gate: `#if defined(LUMICE_CUDA_ENABLED)` — the TU is empty on
// non-CUDA hosts (compiles+links into unit_correctness_test with zero
// contributed symbols on Mac). Runtime: `ShouldSkipCudaTests()` mirrors
// test_cuda_geom_pool_rebuild.cpp / test_cuda_rich_exit.cpp so the tests
// self-skip when no CUDA device is enumerated (dev49-only body).

#include <gtest/gtest.h>

#if defined(LUMICE_CUDA_ENABLED)

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/cuda_trace_backend_test_hooks.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/exit_seam.hpp"
#include "core/raypath.hpp"
#include "cuda_test_helpers.hpp"

namespace lumice {
namespace {

using cuda_test::MakeRenderConfig;
using cuda_test::MakeStochasticPrismScene;
using cuda_test::MakeTwoLayerScene;

bool ShouldSkipCudaTests() {
  return !CudaDeviceAvailable();
}

// Two-layer hex-prism scene with stochastic h_/d_ overlaid on both layers.
// Layer 0 fixed-axis with prob 0.6 (~60% continuation into layer 1); layer 1
// random-axis, final. Stochastic geometry forces `SceneHasStochasticGeometry`
// → BeginSession rebuilds the K-shape pool every batch → the K knob is
// actually exercised (deterministic scenes collapse to P_ci=1 regardless of K).
SceneConfig MakeTwoLayerStochasticScene(size_t max_hits) {
  SceneConfig scene = MakeTwoLayerScene(max_hits);
  for (auto& ms : scene.ms_) {
    for (auto& s : ms.setting_) {
      PrismCrystalParam prism = std::get<PrismCrystalParam>(s.crystal_.param_);
      prism.h_ = Distribution{ DistributionType::kGaussian, 1.0f, 0.15f };
      for (auto& d : prism.d_) {
        d = Distribution{ DistributionType::kGaussian, 1.0f, 0.15f };
      }
      s.crystal_.param_ = prism;
    }
  }
  return scene;
}

// =============================================================================
// Test A — path[] locality (K=8, ci_n=64, P_ci=8, poly_off ∈ [0, 56] < 256)
// =============================================================================
TEST(CudaKShapePool, KShapePool_PathIsLocalWithinPolygonFaceCount_AC1) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  const size_t kMaxHits = 6;
  auto scene = MakeTwoLayerStochasticScene(kMaxHits);
  auto render = MakeRenderConfig();

  // ci_n = 64, K = 8 → P_ci = 8 per (layer, ci); 2 layers × 1 ci each = 16
  // total pool slots. Max poly_off = 7 * 8 = 56 < 256 → the LOCAL-vs-ABSOLUTE
  // ceiling below has no uint8_t-truncation aliasing blind spot.
  const size_t kCiN = 64;
  const size_t kK = 8;
  const size_t kExpectedPCiPerLayerCi = (kCiN + kK - 1) / kK;
  ASSERT_EQ(kExpectedPCiPerLayerCi, 8u);
  const uint32_t kPolygonFaceCount = 8u;

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 31;
  spec.ray_num = kCiN;

  CudaTraceBackend backend;
  CudaTraceBackendTestHooks hooks(backend);
  backend.BeginSession(spec);

  HostRayBatch host;
  host.count = kCiN;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h0, nullptr);

  // Layer-0 pool table sanity — every shape must have 8 polygon faces (the
  // LOCAL-index ceiling below assumes hex-prism topology). Table covers ALL
  // layers (BuildGeomPool builds every (layer, ci) at once in BeginSession),
  // so limit the poly_cnt check to the first P_ci entries which are layer 0.
  auto table = hooks.ReadbackPoolShapeTable();
  ASSERT_GE(table.size(), kExpectedPCiPerLayerCi)
      << "pool_shape_table missing rows: expected ≥ " << kExpectedPCiPerLayerCi << " got " << table.size();
  for (size_t s = 0; s < kExpectedPCiPerLayerCi; ++s) {
    ASSERT_EQ(table[s][1], kPolygonFaceCount)
        << "layer-0 shape[" << s << "] poly_cnt != 8 — hex-prism assumption broken.";
  }

  // ---- Test A.1 — per-ray root pool shape is a legal table entry ----------
  auto rays0 = hooks.ReadbackRootPoolShape(kCiN);
  ASSERT_EQ(rays0.size(), kCiN);

  std::set<std::pair<uint32_t, uint32_t>> valid;
  for (size_t s = 0; s < kExpectedPCiPerLayerCi; ++s) {
    valid.emplace(table[s][0], table[s][1]);
  }
  for (size_t i = 0; i < rays0.size(); ++i) {
    EXPECT_TRUE(valid.count(rays0[i]) > 0u)
        << "layer-0 ray " << i << " landed on (poly_off=" << rays0[i].first << ", poly_cnt=" << rays0[i].second
        << "), which is NOT a layer-0 pool_shape_table row";
  }

  // Sensitivity guard (mirrors Metal Test A `:833-858`): at K=8, P_ci=8, at
  // least one ray MUST land on a shape with `poly_off > 0` — otherwise the
  // path[] LOCAL-vs-ABSOLUTE distinction below is vacuous (both contracts
  // agree when poly_off == 0). If this fails on 64 rays across 8 slots
  // (uniform-random pick would give each slot ~8 rays), the K-shape picker
  // is degenerate — likely a `slot_base` / `p_ci` arithmetic drift.
  size_t rays0_on_nonzero_off = 0;
  for (const auto& r : rays0) {
    if (r.first > 0u) {
      ++rays0_on_nonzero_off;
    }
  }
  EXPECT_GT(rays0_on_nonzero_off, 0u) << "K=8/P_ci=8 with " << kCiN
                                      << " rays: expected some rays on poly_off > 0, got zero — "
                                         "K-shape picker collapsed to slot 0 (picker regression) or "
                                         "the scene isn't exercising per-ray pick.";

  // Test A.2 (path[] locality via raw layer-0 ExitRayRecord) — intentionally
  // dropped on CUDA. Since `SupportsDeviceXyzAccum() == true`,
  // `trace_single_ms_kernel` never writes `d_exit_` / `d_exit_count_`; all
  // mid-exits fold into `EmitToDeviceXyz` (`cuda_trace_backend.cu:947`), so
  // `ReadbackExitRays` returns 0 by construction. The Metal absolute-vs-LOCAL
  // `path[]` regression is also structurally impossible here — `path_rec[]`
  // is a thread-local `uint8[]` fed only by LOCAL `from_poly` / `hit_poly`
  // values bounds-checked against `poly_cnt`, and it never leaves the
  // kernel. Filter-consumed index-type errors are still caught (indirectly)
  // by the CUDA filter-parity battery. Re-adding a direct path[] readback
  // would need a dedicated test-only capture ring in the kernel — tracked
  // as a project backlog item ("[CUDA / K-shape 池] 补 path[] LOCAL/ABSOLUTE
  // 结构性观测（capture ring）") so this coverage gap does not vanish with
  // the task archive.

  backend.EndSession();
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

// =============================================================================
// Test B — transit_multi_ms_kernel per-ray shape coverage (layer ≥ 1)
// =============================================================================
TEST(CudaKShapePool, KShapePool_TransitPicksMultipleShapes_AC1) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  const size_t kMaxHits = 6;
  auto scene = MakeTwoLayerStochasticScene(kMaxHits);
  auto render = MakeRenderConfig();

  // Match Metal Test B (`test_metal_trace_backend.cpp:910-968`): 4096 rays so
  // ~2400 continue into layer 1 → plenty of transit rays for the ≥2-distinct-
  // slots signal to have statistical footing.
  const size_t kRayNum = 4096;

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 37;
  spec.ray_num = kRayNum;

  CudaTraceBackend backend;
  CudaTraceBackendTestHooks hooks(backend);
  backend.BeginSession(spec);

  HostRayBatch host;
  host.count = kRayNum;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h0, nullptr);
  const size_t cont0 = h0->ContinuationCount();
  ASSERT_GT(cont0, 32u) << "layer-0 produced only " << cont0
                        << " continuations — need > 32 for the transit ≥2-distinct-slot signal "
                           "to have statistical footing.";

  RecombineSpec rspec;
  rspec.shuffle = false;
  auto roots1 = backend.Recombine(std::move(h0), rspec);
  ASSERT_TRUE(roots1.is_device);
  ASSERT_EQ(roots1.device.count, cont0);

  auto h1 = backend.TraceLayer(roots1);
  ASSERT_NE(h1, nullptr);

  // Layer-1 root pool shape carrier holds transit_multi_ms_kernel's per-ray
  // picks. Table now covers both layers (BuildGeomPool ran once in
  // BeginSession).
  auto rays1 = hooks.ReadbackRootPoolShape(cont0);
  ASSERT_EQ(rays1.size(), cont0);

  auto table = hooks.ReadbackPoolShapeTable();
  ASSERT_GT(table.size(), 0u);

  std::set<std::pair<uint32_t, uint32_t>> valid(rays1.begin(), rays1.end());  // used to size the check
  std::set<std::pair<uint32_t, uint32_t>> table_valid;
  for (const auto& row : table) {
    table_valid.emplace(row[0], row[1]);
  }
  for (size_t i = 0; i < rays1.size(); ++i) {
    EXPECT_TRUE(table_valid.count(rays1[i]) > 0u)
        << "layer-1 ray " << i << " landed on (poly_off=" << rays1[i].first << ", poly_cnt=" << rays1[i].second
        << ") — NOT a row of the cumulative pool_shape_table. transit K-shape pick synthesized "
           "an out-of-table tuple.";
  }

  // AC1 core: transit distributes rays across ≥2 distinct pool slots. K=8 with
  // ~cont0 rays over P_ci ≥ 2 slots MUST spread; exactly-1 here would mean the
  // transit picker is broken (e.g. clamp-to-slot-0 regression) — a regression
  // that the ms_layers=1-only pre-existing tests would never catch.
  EXPECT_GE(valid.size(), 2u) << "K=8, layer-1 with " << cont0
                              << " transit rays: expected ≥ 2 distinct (poly_off, poly_cnt), got " << valid.size()
                              << " — transit_multi_ms_kernel K-shape pick is not distributing.";

  backend.EndSession();
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

// =============================================================================
// Test C — K=0 / knob-unset structural collapse (AC2 structural anchor)
// =============================================================================
TEST(CudaKShapePool, KShapePool_DefaultKnobUnsetGivesPCiOne_AC2) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");  // isolation from prior test order

  // Single-layer stochastic scene — one (layer, ci) pair. K=0 → P_ci = 1 →
  // pool table has exactly one row, and GetLastBatchCrystalCount() == 1.
  auto scene = MakeStochasticPrismScene(/*max_hits=*/4, /*gaussian=*/true);
  auto render = MakeRenderConfig();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 21;

  CudaTraceBackend backend;
  CudaTraceBackendTestHooks hooks(backend);
  backend.BeginSession(spec);

  HostRayBatch host;
  host.count = 64;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  backend.TraceLayer(RootRaySource::FromHost(host));

  auto table = hooks.ReadbackPoolShapeTable();
  EXPECT_EQ(table.size(), 1u) << "K=0 must collapse pool to a single shape (P_ci ≡ 1 per (layer, ci)).";
  EXPECT_EQ(backend.GetLastBatchCrystalCount(), 1u)
      << "K=0, single-(layer, ci) scene: Σ P_ci must be 1 (mirrors GetLastBatchCrystalCount contract "
         "for the knob-off configuration).";

  // Every ray MUST report the single shape's tuple; any spread means the
  // K=0 collapse is broken (P_ci somehow > 1, or the picker sampled a
  // ghost slot).
  auto rays = hooks.ReadbackRootPoolShape(host.count);
  ASSERT_EQ(rays.size(), host.count);
  std::set<std::pair<uint32_t, uint32_t>> distinct(rays.begin(), rays.end());
  EXPECT_EQ(distinct.size(), 1u) << "K=0: every ray must land on the sole pool shape (distinct = 1); got "
                                 << distinct.size();
  if (!rays.empty()) {
    EXPECT_EQ(rays.front().first, 0u) << "K=0 sole shape must have poly_off=0.";
  }

  backend.EndSession();
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
