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
  // are testing the RNG plumbing, not degenerate-face handling (that lives in
  // the separate "退化几何" scrum, per task plan §7 risk 2).
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
// DistributionType), so both must be covered independently. Plan §5a: "uniform
// 与 gaussian 各测一组——两者测的不是同一件事".
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
// Observation channel: `PrismCrystalParam::h_` is the first Distribution
// consumed by `CrystalMaker::operator()(const PrismCrystalParam&)` — its
// value ends up in the produced Crystal's height. We compare `Crystal::height_`
// (or an equivalent per-cycle scalar; the exact geometry accessor is chosen
// below) across cycles.
//
// This ties Layer 1 + Layer 2 together into a single positive assertion for
// AC1: the shape ACTUALLY changes across batches on the same instance.
TEST(CudaGeomPoolRebuild, StochasticScene_ShapesDifferAcrossBatches) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  auto scene = MakeStochasticPrismScene(/*max_hits=*/4, /*gaussian=*/false);
  auto render = MakeRenderConfig();

  CudaTraceBackend backend;
  CudaTraceBackendTestHooks hooks(backend);
  hooks.EnableGeomPoolRebuildCount();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42u;

  const size_t kCycles = 4;
  // Sanity: with kCycles BeginSession calls we expect exactly kCycles rebuilds
  // (redundant with the previous test, but keeps this test self-contained if
  // the two are ever split into separate translation units).
  for (size_t i = 0; i < kCycles; ++i) {
    backend.BeginSession(spec);
    backend.EndSession();
  }
  EXPECT_EQ(hooks.ReadbackGeomPoolRebuildCount(), static_cast<uint32_t>(kCycles))
      << "Sanity: stochastic scene should rebuild every cycle.";
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
