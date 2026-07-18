// White-box unit test for CUDA geometry-pool rebuild behavior.
//
// Asserts the two invariants in `src/core/backend/cuda_trace_backend.cu`:
//   * `Impl::rng_` is seeded ONCE per Impl lifetime (mirrors CPU/Metal — see
//     the `rng_seeded_` field and its gate in BeginSession). An unconditional
//     `SetSeed(spec.seed)` at the top of every BeginSession would collapse
//     the per-batch RNG stream to a constant prefix, freezing crystal-shape
//     randomization end-to-end.
//   * The per-(layer,ci) geometry pool is REBUILT every BeginSession when
//     the scene carries any stochastic crystal parameter (see
//     `Impl::pool_stochastic_` and the `geom_pool_built_ = false`
//     invalidation just below the scene-change block in BeginSession); it
//     stays BUILT-ONCE when every crystal is deterministic (the fast path
//     preserved at `geom_pool_built_`).
//
// Observation channel: the production-zero-cost `geom_pool_rebuild_count_`
// counter (test-only enabled via CudaTraceBackendTestHooks). No device
// buffers to poke; no session-boundary constraints.
//
// Build gate: `#if defined(LUMICE_CUDA_ENABLED)` — the TU is empty on
// non-CUDA hosts (compiles and links into unit_correctness_test with zero
// contributed symbols on Mac). Runtime: `ShouldSkipCudaTests()` mirrors
// test_cuda_component_mask_parity.cpp / test_cuda_rich_exit.cpp so the
// tests self-skip when no CUDA device is enumerated (dev49-only body).

#include <gtest/gtest.h>

#if defined(LUMICE_CUDA_ENABLED)

#include <cstddef>
#include <cstdint>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/cuda_trace_backend_test_hooks.hpp"
#include "core/backend/trace_backend.hpp"
#include "cuda_test_helpers.hpp"

namespace lumice {
namespace {

using cuda_test::MakePrismScene;
using cuda_test::MakeRenderConfig;

bool ShouldSkipCudaTests() {
  return !CudaDeviceAvailable();
}

// Deterministic prism scene — reuses MakePrismScene from cuda_test_helpers.hpp
// (h_ = 1.0, all d_ = 1.0, no stochastic distributions). Path: fast build-once
// pool.
SceneConfig MakeDeterministicPrismScene(size_t max_hits) {
  return MakePrismScene(max_hits);
}

// Stochastic prism scene — `h_` and `d_[i]` all set to a uniform distribution.
// `IsDeterministic(param)` returns false, so `SceneHasStochasticGeometry` is
// true and BeginSession must force per-batch pool rebuild.
SceneConfig MakeStochasticPrismScene(size_t max_hits, bool gaussian) {
  SceneConfig scene = MakePrismScene(max_hits);
  auto& setting = scene.ms_[0].setting_[0];
  PrismCrystalParam prism = std::get<PrismCrystalParam>(setting.crystal_.param_);
  const DistributionType t = gaussian ? DistributionType::kGaussian : DistributionType::kUniform;
  // Height h_: mean 1.0, spread 0.15 — well-conditioned (never degenerate).
  prism.h_ = Distribution{ t, 1.0f, 0.15f };
  // Face distances d_[6]: mean 1.0, spread 0.15. Still well-conditioned; we
  // are exercising the RNG plumbing, not degenerate-face handling — the
  // negative-`d` / non-manifold rejection at the Crystal factory boundary is
  // validated separately and is deliberately out of scope for this test.
  for (auto& d : prism.d_) {
    d = Distribution{ t, 1.0f, 0.15f };
  }
  setting.crystal_.param_ = prism;
  return scene;
}

// Drive N BeginSession/EndSession cycles against `scene` and return the
// observed BuildGeomPool invocation count.
uint32_t CountRebuildsAcrossBatches(const SceneConfig& scene, const RenderConfig& render, uint32_t seed,
                                    size_t n_cycles) {
  CudaTraceBackend backend;
  CudaTraceBackendTestHooks hooks(backend);
  hooks.EnableGeomPoolRebuildCount();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;

  for (size_t i = 0; i < n_cycles; ++i) {
    backend.BeginSession(spec);
    backend.EndSession();
  }
  return hooks.ReadbackGeomPoolRebuildCount();
}

// -----------------------------------------------------------------------------
// Deterministic scene: BuildGeomPool must run exactly ONCE across an
// arbitrary number of BeginSession/EndSession cycles. This asserts the
// build-once fast path (via `geom_pool_built_`) is preserved end-to-end.
TEST(CudaGeomPoolRebuild, DeterministicScene_BuildsPoolOnce) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  auto scene = MakeDeterministicPrismScene(/*max_hits=*/4);
  auto render = MakeRenderConfig();

  const size_t kCycles = 5;
  uint32_t rebuilds = CountRebuildsAcrossBatches(scene, render, /*seed=*/42u, kCycles);
  EXPECT_EQ(rebuilds, 1u) << "Deterministic scene must build the pool once and reuse it across every "
                             "BeginSession cycle (build-once fast path via `geom_pool_built_`). Observed "
                          << rebuilds << " rebuild(s) across " << kCycles << " cycle(s).";
}

// -----------------------------------------------------------------------------
// AC1 (Layer 2) — uniform-stochastic scene: BuildGeomPool must run EVERY
// BeginSession cycle so each SimBatch draws fresh shapes. Combined with the
// Layer 1 seed-once gate, successive rebuilds advance rng_ instead of replaying
// the same prefix (see CrossCycleShapesDiffer below for the direct evidence
// that shapes actually change; this test isolates the rebuild-frequency
// invariant on its own).
TEST(CudaGeomPoolRebuild, StochasticScene_Uniform_RebuildsEveryBatch) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  auto scene = MakeStochasticPrismScene(/*max_hits=*/4, /*gaussian=*/false);
  auto render = MakeRenderConfig();

  const size_t kCycles = 5;
  uint32_t rebuilds = CountRebuildsAcrossBatches(scene, render, /*seed=*/42u, kCycles);
  EXPECT_EQ(rebuilds, static_cast<uint32_t>(kCycles))
      << "Stochastic (uniform) scene must rebuild the geometry pool every "
         "BeginSession (AC1). Observed "
      << rebuilds << " rebuild(s) across " << kCycles << " cycle(s).";
}

// -----------------------------------------------------------------------------
// Same as above but with a gaussian distribution — uniform and gaussian
// exercise different rng_ code paths in CrystalMaker (Get() dispatches on
// DistributionType), so both must be covered independently: testing one
// distribution type alone would miss regressions in the other's sampling path.
TEST(CudaGeomPoolRebuild, StochasticScene_Gaussian_RebuildsEveryBatch) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  auto scene = MakeStochasticPrismScene(/*max_hits=*/4, /*gaussian=*/true);
  auto render = MakeRenderConfig();

  const size_t kCycles = 5;
  uint32_t rebuilds = CountRebuildsAcrossBatches(scene, render, /*seed=*/42u, kCycles);
  EXPECT_EQ(rebuilds, static_cast<uint32_t>(kCycles))
      << "Stochastic (gaussian) scene must rebuild the geometry pool every "
         "BeginSession (AC1). Observed "
      << rebuilds << " rebuild(s) across " << kCycles << " cycle(s).";
}

// -----------------------------------------------------------------------------
// AC1 (Layer 1 seed-once) direct evidence — successive BeginSession cycles on
// the SAME backend instance with the SAME seed must draw DIFFERENT shapes.
// Prior to the fix, rng_ was reset every BeginSession, so cycle N shapes ==
// cycle 0 shapes; after the fix, rng_ advances across cycles.
//
// Observation channel: `ReadbackFirstPoolCrystalGeom` copies the produced
// polygon-face distances of the pool's first host crystal — the quantity a
// stochastic `d_[]`/`h_` config actually randomizes. Read BETWEEN BeginSession
// and EndSession each cycle so the vector reflects THAT cycle's draw, then
// compare across cycles.
//
// Why rebuild-count alone is insufficient (and this test is not redundant with
// the two above): if Layer 1 regressed to an unconditional per-BeginSession
// reseed while Layer 2 still rebuilt the pool every cycle, `rng_` would reset
// to the same prefix each time and every rebuild would draw the IDENTICAL
// shape — yet `geom_pool_rebuild_count_` would still equal kCycles. Only a
// direct geometry comparison distinguishes "rebuilt with a fresh draw" from
// "rebuilt with a frozen draw". This is the sole local (pre-dev49) defense for
// the Layer 1 seed-once invariant, so it must assert on geometry, not counts.
TEST(CudaGeomPoolRebuild, StochasticScene_ShapesDifferAcrossBatches) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  auto scene = MakeStochasticPrismScene(/*max_hits=*/4, /*gaussian=*/false);
  auto render = MakeRenderConfig();

  CudaTraceBackend backend;
  CudaTraceBackendTestHooks hooks(backend);

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42u;  // SAME seed every cycle — the point is that rng_ advances.

  const size_t kCycles = 4;
  std::vector<std::vector<float>> geoms;
  geoms.reserve(kCycles);
  for (size_t i = 0; i < kCycles; ++i) {
    backend.BeginSession(spec);
    std::vector<float> g;
    // 8 = MakePrismScene's polygon-face count (2 basal + 6 prism). If a future
    // copy uses a crystal with fewer faces, ReadbackFirstPoolCrystalGeom
    // truncates to the actual count rather than reading out of bounds.
    const size_t n = hooks.ReadbackFirstPoolCrystalGeom(g, /*count=*/8);
    backend.EndSession();
    ASSERT_GT(n, 0u) << "pool crystal geometry unavailable at cycle " << i
                     << " (empty pool — device-gen path must populate pool_crystals_)";
    geoms.push_back(std::move(g));
  }

  // Layer 1 direct evidence: with the SAME seed on the SAME instance, each
  // later cycle's geometry MUST differ from cycle 0. If Layer 1 regressed
  // (unconditional reseed), every cycle would replay the identical first draw
  // and all geoms would be bit-equal — the frozen-geometry bug this task fixes.
  size_t distinct_from_first = 0;
  for (size_t i = 1; i < kCycles; ++i) {
    if (geoms[i] != geoms[0]) {
      ++distinct_from_first;
    }
  }
  EXPECT_EQ(distinct_from_first, kCycles - 1)
      << "Stochastic scene must draw a DIFFERENT shape every BeginSession cycle "
         "(AC1, Layer 1 seed-once). Only "
      << distinct_from_first << " of " << (kCycles - 1)
      << " later cycles differed from cycle 0; fewer means rng_ is being reset "
         "per BeginSession (the frozen-geometry regression this task fixes).";
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
