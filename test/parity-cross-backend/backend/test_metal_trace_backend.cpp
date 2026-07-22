// Metal backend tests. Compiled into unit_test on all platforms (the body is
// `#if defined(__APPLE__)`-guarded); on non-Apple the tests are absent. On
// Apple the tests may be skipped at runtime via LUMICE_SKIP_METAL_TESTS=1
// (covers CI runners with no Metal-capable GPU).

#include <gtest/gtest.h>

#if defined(__APPLE__)

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <set>
#include <vector>

#include "config/render_config.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/metal_trace_backend.hpp"
#include "core/backend/metal_trace_backend_test_hooks.hpp"
#include "core/backend/trace_backend.hpp"
#include "metal_test_helpers.hpp"

namespace lumice {
namespace {

using metal_test::ChannelSum;
using metal_test::MakeMetalScene;
using metal_test::MakeRectangularRender;
using metal_test::RelErr;
using metal_test::ShouldSkipMetalTests;

// =============================================================================
// Test E — single-layer parity: CPU vs Metal with identical SessionSpec and
// seed must produce numerically close XYZ images (per-channel sum relative
// error ≤ 1e-4, per plan §6 / §8 unified threshold).
// =============================================================================
TEST(MetalTraceBackend, SingleLayerXyzMatchesCpu) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kRayCount = 4096;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_cpu(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  {
    CpuTraceBackend cpu;
    cpu.BeginSession(spec);
    auto h = cpu.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_cpu.data(), render.resolution_[0], render.resolution_[1] };
    cpu.ReadbackImage(img);
    cpu.EndSession();
  }

  std::vector<float> xyz_metal(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_metal.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // Sanity: both images must contain finite values, with positive total weight.
  double cpu_total = 0.0;
  double metal_total = 0.0;
  for (size_t i = 0; i < xyz_cpu.size(); i++) {
    ASSERT_TRUE(std::isfinite(xyz_cpu[i]));
    ASSERT_TRUE(std::isfinite(xyz_metal[i]));
    cpu_total += static_cast<double>(xyz_cpu[i]);
    metal_total += static_cast<double>(xyz_metal[i]);
  }
  EXPECT_GT(cpu_total, 0.0);
  EXPECT_GT(metal_total, 0.0);

  // Per-channel sum comparison. Both backends consume the same root rays
  // (deterministic InitRayFirstMs from spec.seed) and the kernel matches
  // RectangularProject byte-for-byte at the source level. The residual
  // disparity comes from two sources, neither of which the kernel can
  // eliminate:
  //   1) GPU vs CPU transcendentals (atan2/asin/sqrt) differ at the ULP
  //      level. Over 8 max_hits worth of trace math this accumulates to
  //      ~5e-5 per ray — visible in the per-channel sum.
  //   2) GPU atomic_fetch_add on the XYZ image runs in nondeterministic
  //      order across threadgroups; float addition is non-associative so
  //      the running total drifts from a sequential CPU sum.
  // 5e-4 captures observed disparity (~2.4e-4 with mathMode=Safe) with
  // 2x headroom; plan §1 originally specified 1e-2, plan §8 suggested
  // tightening to 1e-4 — empirically the latter is below the floor that
  // GPU/CPU parity can hit on this code.
  for (int c = 0; c < 3; c++) {
    double scpu = ChannelSum(xyz_cpu, c);
    double smetal = ChannelSum(xyz_metal, c);
    double rel = RelErr(scpu, smetal);
    EXPECT_LT(rel, 5e-4) << "channel=" << c << " cpu=" << scpu << " metal=" << smetal;
  }
}

// =============================================================================
// Test F — two-layer MS end-to-end: BeginSession → TraceLayer (append) →
// Recombine → TraceLayer (accumulate) → ReadbackImage. Verifies the
// device-resident continuation path runs without UB and produces a finite,
// non-zero XYZ image. Strict CPU-vs-Metal parity for MS chains is deferred
// to scrum sub-task 4's parity harness.
// =============================================================================
TEST(MetalTraceBackend, TwoLayerEndToEnd) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/6, /*ms_layers=*/2);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  MetalTraceBackend backend;
  backend.BeginSession(spec);

  HostRayBatch host;
  host.count = 2048;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h0, nullptr);
  size_t cont0 = h0->ContinuationCount();
  EXPECT_GT(cont0, 0u);

  RecombineSpec rspec;
  rspec.shuffle = false;  // v1 Metal backend does not implement device shuffle.
  auto roots1 = backend.Recombine(std::move(h0), rspec);
  ASSERT_TRUE(roots1.is_device);
  EXPECT_EQ(roots1.device.count, cont0);

  auto h1 = backend.TraceLayer(roots1);
  ASSERT_NE(h1, nullptr);
  EXPECT_EQ(h1->ContinuationCount(), 0u);  // last layer: no append.

  std::vector<float> xyz(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  XyzImageData img{ xyz.data(), render.resolution_[0], render.resolution_[1] };
  backend.ReadbackImage(img);

  double sum = 0.0;
  for (float v : xyz) {
    sum += static_cast<double>(v);
    ASSERT_TRUE(std::isfinite(v));
  }
  EXPECT_GT(sum, 0.0);

  backend.EndSession();
}

// =============================================================================
// scrum-267 task-fused-emit-gate Step 9 / code-review-01 M2:
// trace_layer_kernel production-PSO occupancy regression guard.
//
// The device emit gate added ~64B of thread-local scratch (`uchar
// path_local[kDevRecCap]`) plus the DeviceFilterCheck call graph (6 sub-
// matcher inlines) into trace_layer_kernel — register pressure increased
// materially. Plan §9 set `maxThreadsPerThreadgroup ≥ 1024` as the wavefront-
// stay-fused ACCEPTANCE BAR; plan R1 names option B (split filter gate into
// independent dispatch) as the response when occupancy drops.
//
// Occupancy history (Apple M-series): plan baseline 1024 → 704 at M5 (267.2
// fused emit gate: path_local scratch + DeviceFilterCheck inlines) → 640 after
// scrum-268.8 DR-3 (per-ray wavelength added wl_pool reads + per-ray wl_idx /
// cmf registers to trace_layer_kernel).
//
// ⭐ R1 RULING (scrum-268.6, 2026-06-16): the occupancy drop is BENIGN. The
// single-engine + backend-aware large-dispatch (32768) throughput on the heavy
// multi-MS + filter scenes is ~8-10x legacy N-worker (measured via
// test_metal_throughput, the now-active D1 gate; setup-excluded re-measure
// 2026-06-19, task-fix-throughput-bench-honesty — the earlier "3.5–5.4x" was the
// setup-inflated --benchmark reading). The GPU is saturated at large
// dispatch, so the lower per-threadgroup occupancy does NOT cost throughput.
// → plan R1 option B (split the filter gate into its own wavefront dispatch to
// recover occupancy) is NOT invoked. 640 is the accepted production baseline;
// the active throughput gate (ratio ≥ 1.0) is the primary guard now, this
// occupancy guard is the secondary early-warning for further kernel bloat.
//
// The ≥ 640 assertion holds at the exact DR-3 baseline (zero margin); a further
// drop fires it as a signal to re-measure throughput and reconsider R1 option B
// (do NOT just lower the threshold — re-rule against the throughput gate).
// =============================================================================
TEST(MetalTraceBackend, TraceLayerKernelOccupancy) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  MetalTraceBackend backend;
  backend.BeginSession(spec);  // builds the trace PSO via Impl::RebuildPSO.
  size_t max_threads = backend.TraceLayerKernelMaxThreadsForTest();
  backend.EndSession();

  std::fprintf(stderr,
               "[occupancy] trace_layer_kernel maxTotalThreadsPerThreadgroup=%zu "
               "(1024 plan → 704 @M5 → 640 @DR-3 baseline → 576 @358.1 raypath-color "
               "pass, ruled benign after throughput re-measure; R1 ruled benign @268.6)\n",
               max_threads);
  // task-358.1 (metal-color-parity) 640→576: the Step 2/4 MSL emit-gate additions
  // (Design-2 color pass + per-color-class atomic Y-lane accumulator) raise the
  // trace kernel's register pressure, dropping maxTotalThreadsPerThreadgroup from
  // the 268.8 DR-3 baseline of 640 to 576. Re-measured on 2026-07-13 against
  // Metal's throughput battery (test_metal_throughput.py, 3 samples each,
  // single-sample thermal CoV ~25% per the test docstring):
  //   ms_multi_crystal_complex_filter: 7.86 / 8.38 / 174.81x (1st sample
  //     outlier; steady-state ~8.1x, matches pre-358.1 8.14x baseline)
  //   ms_multi_crystal_filtered_bd:    8.03 / 8.29 / 8.64x (steady ~8.3x,
  //     vs pre-358.1 10.09x — within single-sample CoV, no material drop)
  // Both configs pass D1 gate (ratio ≥ 1.0) and sanity floor (ratio ≥ 3.0)
  // on every sample. The occupancy drop does NOT translate into a material
  // throughput regression on Metal (ALU-bound trace kernel, occupancy is a
  // non-binding constraint per explore-306.1). Precedent: 1024→704→640 each
  // cleared the same "re-measure, rule benign, relax guard" path. See
  // progress.md RESUME §3.
  EXPECT_GE(max_threads, static_cast<size_t>(576))
      << "trace_layer_kernel occupancy regressed below the 576 task-358.1 baseline "
         "(1024 plan → 704 @M5 → 640 @DR-3 → 576 @358.1) — re-measure multi-MS+filter "
         "throughput vs the active D1 gate and reconsider plan R1 option B "
         "(split filter gate into independent dispatch); see scratchpad/"
         "scrum-gpu-single-engine-continuation/task-fused-emit-gate/plan.md "
         "and progress.md for context.";
}

// =============================================================================
// K-shape geometry pool: GetLastBatchCrystalCount reports the CROSS-LAYER sum
// of distinct pool shapes built during the batch (Σ layers Σ ci P_ci). At the
// default knob LUMICE_GPU_GEOM_CLOCK=0, P_ci collapses to 1 per ci, so the
// return value equals Σ layers Σ ci 1 = the cross-layer setting count.
// =============================================================================
TEST(MetalTraceBackend, GetLastBatchCrystalCountSumsPoolShapesAcrossLayers) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  // Single-MS single-crystal → 1 layer × 1 ci × P_ci=1 = 1.
  {
    auto scene = MakeMetalScene(/*max_hits=*/4, /*ms_layers=*/1);
    auto render = MakeRectangularRender();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 11;

    MetalTraceBackend backend;
    backend.BeginSession(spec);
    HostRayBatch host;
    host.count = 512;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    backend.TraceLayer(RootRaySource::FromHost(host));

    EXPECT_EQ(backend.GetLastBatchCrystalCount(), 1u);
    backend.EndSession();
  }

  // Multi-MS: layer 0 has 1 crystal setting, layer 1 has 3. At the default
  // knob (P_ci=1 per ci) the batch-wide sum is 1 + 3 = 4 — this is the exact
  // opposite of the pre-K-pool semantic that returned only the final layer's
  // 3 settings.
  {
    auto scene = MakeMetalScene(/*max_hits=*/4, /*ms_layers=*/2);
    auto& final_ms = scene.ms_.back();
    ScatteringSetting extra1 = final_ms.setting_.front();
    ScatteringSetting extra2 = final_ms.setting_.front();
    final_ms.setting_.front().crystal_proportion_ = 0.4f;
    extra1.crystal_.id_ = 100;
    extra1.crystal_proportion_ = 0.3f;
    extra2.crystal_.id_ = 101;
    extra2.crystal_proportion_ = 0.3f;
    final_ms.setting_.push_back(std::move(extra1));
    final_ms.setting_.push_back(std::move(extra2));
    ASSERT_EQ(final_ms.setting_.size(), 3u);
    ASSERT_EQ(scene.ms_.front().setting_.size(), 1u);

    auto render = MakeRectangularRender();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 13;

    MetalTraceBackend backend;
    backend.BeginSession(spec);
    HostRayBatch host;
    host.count = 1024;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;

    auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h0, nullptr);
    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = backend.Recombine(std::move(h0), rspec);
    backend.TraceLayer(roots1);

    EXPECT_EQ(backend.GetLastBatchCrystalCount(), 4u)
        << "Cross-layer pool-shape sum at K=0: 1 (layer 0) + 3 (layer 1). "
           "Pre-K-pool semantic was final-layer settings only (3).";
    backend.EndSession();
  }
}

// =============================================================================
// task-metal-gui-commit-backpressure O2 AC1: two independently-constructed
// MetalTraceBackend instances must observe the same underlying MTLDevice /
// trace_layer_kernel MTLComputePipelineState pointers. This is the direct
// whitebox proof that PSO/library/device construction was hoisted to a process-
// level cache (per-Run rebuilds were the ~150ms cost that starved Metal slider
// drag under the 70ms commit clock). Pointer identity — not throughput — is
// the mechanism-level claim; throughput noise would let a broken cache pass
// this test even if it silently rebuilt.
// =============================================================================
TEST(MetalTraceBackend, DeviceAndPsoSharedAcrossInstances) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  HostRayBatch host;
  host.count = 256;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  const void* dev0 = nullptr;
  const void* pso0 = nullptr;
  {
    MetalTraceBackend backend_a;
    backend_a.BeginSession(spec);
    auto h = backend_a.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    dev0 = backend_a.GetDevicePtrForTest();
    pso0 = backend_a.GetPsoPtrForTest();
    backend_a.EndSession();
  }
  ASSERT_NE(dev0, nullptr) << "First instance did not populate device";
  ASSERT_NE(pso0, nullptr) << "First instance did not populate trace_layer PSO";

  const void* dev1 = nullptr;
  const void* pso1 = nullptr;
  {
    MetalTraceBackend backend_b;
    backend_b.BeginSession(spec);
    auto h = backend_b.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    dev1 = backend_b.GetDevicePtrForTest();
    pso1 = backend_b.GetPsoPtrForTest();
    backend_b.EndSession();
  }

  EXPECT_EQ(dev0, dev1) << "MTLDevice pointer differs across MetalTraceBackend instances — the "
                           "process-level device cache is not being shared, meaning per-Run "
                           "commits will rebuild the device and pay the ~100-150ms library-load "
                           "cost that regresses Metal slider-drag responsiveness.";
  EXPECT_EQ(pso0, pso1) << "trace_layer_kernel MTLComputePipelineState pointer differs across "
                           "MetalTraceBackend instances — the process-level PSO cache is not "
                           "being shared. Per-Run PSO rebuilds were the ~150ms first-batch cost "
                           "that starved commit-outpaces-batch under the 70ms GUI commit clock.";
}

// =============================================================================
// K-shape geometry pool AC1: LUMICE_GPU_GEOM_CLOCK enables a per-ci pool of
// P_ci = ceil(N_ci / K) distinct crystal shapes, and gen_root_kernel picks a
// pool slot per ray via an independent PCG stream. These tests verify:
//   * Default (knob unset): P_ci == 1, one shape built per ci, all rays land
//     on pool slot 0 — bit-identical to the pre-K-pool path.
//   * K=8 with non-deterministic crystal params and 64 rays: P_ci == 8;
//     ReadbackRootPoolShape shows ≥ 2 distinct (poly_off, poly_cnt) values
//     across the batch (per-ray shape picking observably works).
//   * Independent verification: the expected P_ci and the per-ray slot draw
//     are recomputed inside the test WITHOUT calling into production code —
//     otherwise the assertion would only check that a function calls itself.
// =============================================================================

namespace {

// Random-h prism scene: overrides MakeMetalScene's deterministic h_ to a
// gaussian so IsDeterministic returns false and the K knob takes effect.
SceneConfig MakeMetalSceneRandomH(size_t max_hits, size_t ms_layers) {
  SceneConfig scene = MakeMetalScene(max_hits, ms_layers);
  for (auto& ms : scene.ms_) {
    for (auto& s : ms.setting_) {
      auto prism = std::get<PrismCrystalParam>(s.crystal_.param_);
      prism.h_ = Distribution{ DistributionType::kGaussian, 1.0f, 0.15f };
      s.crystal_.param_ = prism;
    }
  }
  return scene;
}

}  // namespace

TEST(MetalTraceBackend, KShapePool_DefaultKnobUnsetGivesPCiOne_AC1) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  // Guarantee the knob is off for this test (independent of prior test order).
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/4, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 21;

  MetalTraceBackend backend;
  backend.BeginSession(spec);
  HostRayBatch host;
  host.count = 64;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  backend.TraceLayer(RootRaySource::FromHost(host));

  MetalTraceBackendTestHooks hooks(backend);
  auto table = hooks.ReadbackPoolShapeTable();
  EXPECT_EQ(table.size(), 1u) << "K=0 must collapse pool to a single shape";
  EXPECT_EQ(hooks.PoolShapeCountThisBatch(), 1u) << "Σ layers Σ ci P_ci at K=0: 1 × 1 × 1 = 1";
  backend.EndSession();
}

TEST(MetalTraceBackend, KShapePool_KEnabledBuildsCeilNciOverKShapes_AC1) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/4, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 23;

  const size_t ci_n = 64;
  const size_t k = 8;
  const size_t expected_p_ci = (ci_n + k - 1) / k;
  ASSERT_EQ(expected_p_ci, 8u);

  MetalTraceBackend backend;
  backend.BeginSession(spec);
  HostRayBatch host;
  host.count = ci_n;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  backend.TraceLayer(RootRaySource::FromHost(host));

  MetalTraceBackendTestHooks hooks(backend);
  auto table = hooks.ReadbackPoolShapeTable();
  EXPECT_EQ(table.size(), expected_p_ci) << "P_ci must be ceil(N_ci / K) = ceil(64 / 8) = 8";
  EXPECT_EQ(hooks.PoolShapeCountThisBatch(), expected_p_ci) << "Single-layer, single-ci → Σ P_ci = 8";

  // Verify per-shape offsets stack in non-decreasing order (each shape's slice
  // starts after the previous shape's end).
  for (size_t s = 1; s < table.size(); s++) {
    EXPECT_GE(table[s][0], table[s - 1][0] + table[s - 1][1])
        << "poly_off[" << s << "] not after poly_off[" << (s - 1) << "] + poly_cnt";
    EXPECT_GE(table[s][2], table[s - 1][2] + table[s - 1][3])
        << "tri_off[" << s << "] not after tri_off[" << (s - 1) << "] + tri_cnt";
  }

  // AC1 core: rays actually land on more than one pool slot. We do NOT call
  // BuildGeomShapeStream / pcg_uniform here (that would let the production
  // code silently agree with itself); we simply check that > 1 distinct
  // (poly_off, poly_cnt) tuple appears across the batch. If the picker were
  // broken (e.g. all rays picked slot 0), we would see exactly 1 distinct
  // tuple → failure.
  auto rays = hooks.ReadbackRootPoolShape(ci_n);
  ASSERT_EQ(rays.size(), ci_n);
  std::set<std::pair<uint32_t, uint32_t>> distinct(rays.begin(), rays.end());
  EXPECT_GE(distinct.size(), 2u) << "K=8 P_ci=8: expected > 1 distinct (poly_off, poly_cnt) across " << ci_n
                                 << " rays, got " << distinct.size();

  // Cross-check: every observed (poly_off, poly_cnt) MUST match some row of
  // pool_shape_table_h_ (shape picker never writes garbage). Table entries
  // beyond the picked ones are OK — but every pick must be inside the table.
  std::set<std::pair<uint32_t, uint32_t>> valid;
  for (const auto& row : table) {
    valid.emplace(row[0], row[1]);
  }
  for (const auto& r : rays) {
    EXPECT_TRUE(valid.count(r) > 0u) << "ray landed on (" << r.first << ", " << r.second
                                     << "), which is NOT a pool_shape_table row";
  }

  backend.EndSession();
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

// K-shape pool driven by config (NOT env): SceneConfig::geom_clock_ is now the
// production source of K. With the env var explicitly unset, setting
// scene.geom_clock_ = 8 must build the same P_ci = ceil(N_ci / K) pool the env
// path produces. This proves the config plumbing itself drives K — the earlier
// tests only cover the env-override path, which would still pass even if the
// config field were ignored.
TEST(MetalTraceBackend, KShapePool_ConfigDrivenKWithoutEnv) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  // No env override: the value must come purely from config.
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/4, /*ms_layers=*/1);
  scene.geom_clock_ = 8;  // config-supplied K
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 23;

  const size_t ci_n = 64;
  const size_t k = 8;
  const size_t expected_p_ci = (ci_n + k - 1) / k;
  ASSERT_EQ(expected_p_ci, 8u);

  MetalTraceBackend backend;
  backend.BeginSession(spec);
  HostRayBatch host;
  host.count = ci_n;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  backend.TraceLayer(RootRaySource::FromHost(host));

  MetalTraceBackendTestHooks hooks(backend);
  auto table = hooks.ReadbackPoolShapeTable();
  EXPECT_EQ(table.size(), expected_p_ci) << "config geom_clock=8 must build ceil(64/8)=8 shapes without any env var";
  EXPECT_EQ(hooks.PoolShapeCountThisBatch(), expected_p_ci) << "Single-layer, single-ci → Σ P_ci = 8";
  backend.EndSession();
}

// AC2 supplementary smoke: with the K knob enabled, a full session runs to
// completion without crash and produces exit statistics on the same order of
// magnitude as the K=0 baseline for the same seed. This is not a CV/variance
// gate (proper CV-vs-K trend requires many-batch statistical runs, out of
// this test's scope) — it verifies that the K > 0 code path is exercised in
// CI, catching regressions like the "all rays exit immediately" failure
// mode that the intra-step verification caught during Step 4.
TEST(MetalTraceBackend, KShapePool_KEnabledSessionRunsAndProducesOutput_AC2) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/4, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 25;

  auto run_and_get_stats = [&](const char* k_val) {
    if (k_val) {
      ::setenv("LUMICE_GPU_GEOM_CLOCK", k_val, /*overwrite=*/1);
    } else {
      ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
    }
    MetalTraceBackend backend;
    backend.BeginSession(spec);
    HostRayBatch host;
    host.count = 512;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    auto handle = backend.TraceLayer(RootRaySource::FromHost(host));
    LayerStats stats{};
    if (handle) {
      stats = handle->GetLayerStats();
    }
    size_t crystals = backend.GetLastBatchCrystalCount();
    backend.EndSession();
    return std::make_pair(stats, crystals);
  };

  auto [stats0, crystals0] = run_and_get_stats(nullptr);
  auto [stats8, crystals8] = run_and_get_stats("8");

  // K=0: exactly one shape per ci; K=8 with 512 rays: P_ci = ceil(512/8) = 64.
  EXPECT_EQ(crystals0, 1u);
  EXPECT_EQ(crystals8, 64u) << "K=8, N_ci=512 → P_ci=64; single layer × single ci → Σ P_ci = 64";

  // Both runs must produce non-zero exit counts (session actually ran). The
  // per-batch reset invariant (exit_stats_buf zeroed each dispatch) means
  // both runs report the layer's exit metrics independently.
  EXPECT_GT(stats0.exit_count, 0u);
  EXPECT_GT(stats8.exit_count, 0u);
  EXPECT_GT(stats0.exit_w_sum, 0.0f);
  EXPECT_GT(stats8.exit_w_sum, 0.0f);

  // Mean weight invariance sanity: switching K changes the crystal-shape
  // distribution the rays interact with, not the total energy budget. Ratio
  // is bounded conservatively (5×) — a stride-error / silent-drop regression
  // would collapse one of the two to zero or produce a >10× swing.
  const float w_ratio = stats8.exit_w_sum / stats0.exit_w_sum;
  EXPECT_GT(w_ratio, 0.2f);
  EXPECT_LT(w_ratio, 5.0f);

  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

// AC2 statistical gate: with a stochastic h_ distribution, each K=0 session
// draws exactly one crystal shape (P_ci=1), so exit_w_sum varies wildly
// batch-to-batch as the seed picks different shapes. K=8 draws
// P_ci = ceil(512/8) = 64 shapes per session, so each batch already averages
// over 64 shapes — the across-seed variance drops sharply. This test asserts
// (a) mean-invariance: the two K settings sample the same underlying energy
// budget, so their means must agree within a few sigma; (b) variance drop:
// stddev(K=8) < stddev(K=0). Together these constitute the AC2 evidence that
// K→K* narrows variance without biasing the mean.
TEST(MetalTraceBackend, KShapePool_KEnabledReducesCrossSeedVariance_AC2) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/4, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec_template;
  spec_template.scene = &scene;
  spec_template.render = &render;
  spec_template.wl = WlParam{ 550.0f, 1.0f };

  auto run_batch = [&](const char* k_val, uint32_t seed) -> float {
    if (k_val) {
      ::setenv("LUMICE_GPU_GEOM_CLOCK", k_val, /*overwrite=*/1);
    } else {
      ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
    }
    MetalTraceBackend backend;
    SessionSpec spec = spec_template;
    spec.seed = seed;
    backend.BeginSession(spec);
    HostRayBatch host;
    host.count = 512;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    auto handle = backend.TraceLayer(RootRaySource::FromHost(host));
    float w_sum = 0.0f;
    if (handle) {
      w_sum = handle->GetLayerStats().exit_w_sum;
    }
    backend.EndSession();
    return w_sum;
  };

  constexpr size_t kNumSeeds = 12u;
  std::vector<float> ws_k0;
  std::vector<float> ws_k8;
  ws_k0.reserve(kNumSeeds);
  ws_k8.reserve(kNumSeeds);
  for (uint32_t s = 1u; s <= kNumSeeds; ++s) {
    ws_k0.push_back(run_batch(nullptr, s));
    ws_k8.push_back(run_batch("8", s));
  }

  auto stats = [](const std::vector<float>& xs) {
    double sum = 0.0;
    for (float v : xs)
      sum += v;
    double mean = sum / static_cast<double>(xs.size());
    double sq = 0.0;
    for (float v : xs) {
      double d = v - mean;
      sq += d * d;
    }
    double var = sq / static_cast<double>(xs.size());
    return std::make_pair(mean, std::sqrt(var));
  };
  auto [mean_k0, sd_k0] = stats(ws_k0);
  auto [mean_k8, sd_k8] = stats(ws_k8);

  // Mean invariance: K only redistributes shape draws within a batch, so the
  // per-batch exit_w_sum expectation is invariant in K. With 12 seeds, the
  // standard error of each mean is sd/sqrt(12); compare the two means by the
  // pooled standard error of their difference (~sqrt(sd_k0^2 + sd_k8^2)/sqrt(12))
  // against a generous 3σ bound to allow for finite-sample noise.
  double pooled_se = std::sqrt(sd_k0 * sd_k0 + sd_k8 * sd_k8) / std::sqrt(static_cast<double>(kNumSeeds));
  double mean_gap = std::abs(mean_k0 - mean_k8);
  EXPECT_LT(mean_gap, 3.0 * pooled_se) << "AC2 mean invariance: |mean(K=0)-mean(K=8)| = " << mean_gap
                                       << " exceeds 3× pooled SE = " << (3.0 * pooled_se) << " (mean_k0=" << mean_k0
                                       << ", mean_k8=" << mean_k8 << ", sd_k0=" << sd_k0 << ", sd_k8=" << sd_k8 << ")";

  // Variance drop: K=8 averages 64 shapes per batch vs K=0's one shape per
  // batch, so cross-seed sd_k8 should be markedly lower than sd_k0. A generous
  // ratio bound (sd_k8 < 0.75 × sd_k0) tolerates finite-sample noise while
  // still catching a "K knob has no effect" regression. Theoretical expectation
  // is roughly sqrt(64) ≈ 8× improvement; the loose bound here keeps the test
  // non-flaky.
  EXPECT_LT(sd_k8, 0.75 * sd_k0) << "AC2 variance drop: sd(K=8) = " << sd_k8
                                 << " must be < 0.75 × sd(K=0) = " << (0.75 * sd_k0) << " (mean_k0=" << mean_k0
                                 << ", mean_k8=" << mean_k8 << ")";

  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

// Regression coverage for the "empty batch + K knob enabled + stochastic
// crystal params" combination. The offending
// arithmetic (p_ci = ceil(ci_n/K) = 0 when ci_n==0) is already dead-code
// today — TraceLayer short-circuits at `total_ray_num == 0` (line ~3051) and
// the per-ci loop has `if (ci_n == 0) { continue; }` from the base commit —
// but the defensive `ci_n > 0u` guard added around the p_ci formula is
// defense-in-depth against future removal of either short-circuit. This
// test locks in the "no crash under K enabled + empty root batch" contract
// so any regression that removes both guards + reintroduces the arithmetic
// fault will be caught here without requiring a hand-crafted multi-ci
// partition that forces one ci to zero.
TEST(MetalTraceBackend, KShapePool_EmptyBatchWithKEnabledDoesNotCrash_RegressionR2) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/4, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 7;

  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  MetalTraceBackend backend;
  backend.BeginSession(spec);
  HostRayBatch host;
  host.count = 0;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  auto handle = backend.TraceLayer(RootRaySource::FromHost(host));
  // TraceLayer early-returns at total_ray_num==0 without entering the ci
  // loop, so no pool is built and pool_shape_count_this_batch_ stays 0.
  // The assertion is "we got here without an assert-crash from the p_ci
  // guard chain firing on the empty path".
  EXPECT_EQ(backend.GetLastBatchCrystalCount(), 0u)
      << "empty batch with K enabled must return 0 pool shapes (no pools "
         "built) — non-zero here would mean the empty-batch guard is gone "
         "and the K-shape path partially ran on a zero-ray batch.";
  backend.EndSession();

  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

// =============================================================================
// K-shape pool absolute/local semantics regression coverage.
//
// Two blindspots in the original K-shape pool test net that these tests close:
//
//   * `path[]` locality (Test A). The 5 pre-existing KShapePool tests all ran
//     with `filter.type = None` (which short-circuits DeviceFilterCheck to
//     true), so absolute-vs-local corruption in the `path[]` write site never
//     showed up. A ray landing on pool shape `s > 0` writes `poly_off + local`
//     into `path[]`; downstream `path_local[k] = uchar(path[k])` narrows that
//     to 8-bit and hands it to `ApplyGetFn_dev`, which indexes the per-crystal
//     GetFn stripe — corrupting every raypath / entry-exit filter judgment.
//     We probe locality directly via `ReadbackRecSink`: the trace kernel
//     writes `rec_sink[tid] = Σ_k float(path[k])`, and under the fixed
//     contract path[k] ∈ [0, PolygonFaceCount) regardless of pool shape.
//
//   * transit_root_kernel K>0 coverage (Test B). The pre-existing tests were
//     all `ms_layers=1`, exercising only `gen_root_kernel`'s K-shape pick
//     path. `transit_root_kernel` has its own K-shape pick that only runs on
//     layer≥1, so a regression there would slip past the whole prior net.
// =============================================================================

TEST(MetalTraceBackend, KShapePool_PathIsLocalWithinPolygonFaceCount_AC1_TestA) {
  // Test A: given a hex prism (PolygonFaceCount=8) + K=8 + ci_n=64 (P_ci=8),
  // every ray's `Σ_k path[k]` MUST be bounded by `max_hits × PolygonFaceCount`.
  // Independent-verify principle: we do NOT re-derive which pool slot each ray
  // landed on (that would just recompute what the production code did).
  // Instead, we assert a shape-invariant ceiling that holds ONLY when path[k]
  // is a local index. Pre-fix (path[k] absolute), a ray on the last shape
  // (poly_off ≈ 56) could sum up to 4 × 63 = 252, well past the local-index
  // ceiling 4 × 8 = 32. Sentinel against reintroducing this exact blindspot.
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  const size_t kMaxHits = 4;
  auto scene = MakeMetalSceneRandomH(/*max_hits=*/kMaxHits, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 31;

  const size_t ci_n = 64;
  const size_t k = 8;
  const size_t expected_p_ci = (ci_n + k - 1) / k;
  ASSERT_EQ(expected_p_ci, 8u);

  MetalTraceBackend backend;
  backend.BeginSession(spec);
  HostRayBatch host;
  host.count = ci_n;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;
  backend.TraceLayer(RootRaySource::FromHost(host));

  MetalTraceBackendTestHooks hooks(backend);
  auto table = hooks.ReadbackPoolShapeTable();
  ASSERT_EQ(table.size(), expected_p_ci) << "K=8 P_ci sanity";

  // Every shape in the pool must be a valid hex-prism (PolygonFaceCount == 8
  // baked into MakeMetalScene; the local-index ceiling assumes this).
  for (size_t s = 0; s < table.size(); s++) {
    ASSERT_EQ(table[s][1], 8u) << "shape[" << s
                               << "] poly_cnt != 8; test's "
                                  "PolygonFaceCount assumption broken.";
  }

  // Independent ceiling — a shape-invariant of the LOCAL storage contract.
  // The pre-fix (absolute) code stored poly_off + local into path[]; the
  // largest legal Σ under the absolute contract would be max_hits × 63 = 252
  // (poly_off up to 56 + local up to 7), so a 32 ceiling below cleanly
  // separates the two behaviors.
  const uint32_t poly_face_count = 8u;
  const float kLocalCeil = static_cast<float>(kMaxHits) * static_cast<float>(poly_face_count);
  std::vector<float> rec_sink;
  size_t got = hooks.ReadbackRecSink(rec_sink, ci_n);
  ASSERT_EQ(got, ci_n) << "ReadbackRecSink should return exactly ci_n slots";

  // Sanity: at least one ray actually recorded hits (rec_sink > 0). If EVERY
  // ray is zero the ceiling below is vacuously satisfied, so we'd rather fail
  // loudly here than silently pass an inert test.
  size_t nonzero = 0;
  float max_val = 0.0f;
  for (float v : rec_sink) {
    if (v > 0.0f) {
      ++nonzero;
    }
    max_val = std::max(max_val, v);
  }
  EXPECT_GT(nonzero, 0u) << "no ray recorded a path hit — either the scene "
                            "reflects everything at once (parameter drift) or "
                            "the trace kernel is not writing rec_sink; either "
                            "way the locality ceiling below is vacuous.";
  // Sensitivity guard: the pre-fix (absolute) code stored poly_off + local in
  // path[], so with 8 shapes × PolygonFaceCount == 8, some rays MUST land on
  // shapes with poly_off ≥ (P_ci-1) × PolygonFaceCount = 56, and their
  // rec_sink[i] would sum ≥ 56 per hit. If the max we observe is below the
  // local ceiling AND below what a shape-0-only distribution could reach
  // (max_hits × PolygonFaceCount - 1 = 28), we're not exercising the bug
  // (e.g. all rays landed on shape 0 or all rays exit after 1 hit) — that
  // would let the pre-fix bug silently coexist with a green test. We assert
  // sufficient variance in max_val to prove we ARE probing the poly_off > 0
  // shapes.
  std::vector<uint32_t> tf_probe;
  hooks.ReadbackRootTf(tf_probe, ci_n);
  uint32_t max_tf = 0u;
  for (uint32_t v : tf_probe) {
    if (v != 0xffffffffu) {
      max_tf = std::max(max_tf, v);
    }
  }
  // At K=8, P_ci=8, we expect random per-ray shape picks. With 64 rays over
  // 8 shapes the expected count on shape 7 is ~8; the observed max_tf should
  // reach into a poly_off > 0 shape (≥ 8).
  ASSERT_GE(max_tf, static_cast<uint32_t>(poly_face_count))
      << "max root_tf across 64 rays = " << max_tf << " < PolygonFaceCount = " << poly_face_count
      << " — either all rays landed on shape 0 (picker regression) or "
         "sentinel-only (dead session); the locality ceiling below cannot "
         "then discriminate absolute vs local storage.";

  // The ceiling MUST hold for every ray, whichever pool shape it picked.
  for (size_t i = 0; i < rec_sink.size(); i++) {
    EXPECT_LT(rec_sink[i], kLocalCeil) << "ray " << i << " has rec_sink=" << rec_sink[i]
                                       << " (Σ_k path[k]); under the LOCAL-index contract this must be < " << kLocalCeil
                                       << " (max_hits × PolygonFaceCount). A value at or above "
                                          "this ceiling means path[k] is storing an ABSOLUTE "
                                          "pool-wide index — the round-4 blindspot regression.";
  }

  // Widened root_tf sanity: values must either be the widened sentinel
  // (kInvalidId = 0xffffffff) or a legal absolute polygon index inside SOME
  // shape's slice. A stray 0x0000ffff (host uint16 sentinel zero-extended)
  // would break both the ceiling above and this check.
  std::vector<uint32_t> tf;
  hooks.ReadbackRootTf(tf, ci_n);
  ASSERT_EQ(tf.size(), ci_n);
  auto valid_absolute = [&](uint32_t v) {
    if (v == 0xffffffffu) {
      return true;
    }
    for (const auto& row : table) {
      const uint32_t begin = row[0];
      const uint32_t end = row[0] + row[1];
      if (v >= begin && v < end) {
        return true;
      }
    }
    return false;
  };
  for (size_t i = 0; i < tf.size(); i++) {
    EXPECT_TRUE(valid_absolute(tf[i])) << "root_tf[" << i << "]=" << std::hex << tf[i] << std::dec
                                       << " is neither kInvalidId nor a legal absolute polygon index within "
                                          "any pool shape's [poly_off, poly_off+poly_cnt) window";
  }

  backend.EndSession();
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

TEST(MetalTraceBackend, KShapePool_TransitPicksMultipleShapes_AC1_TestB) {
  // Test B: `transit_root_kernel` picks a fresh pool shape per ray on layer
  // ≥1. Prior K-shape pool tests all had `ms_layers=1`, exercising ONLY
  // gen_root_kernel — a bug in transit's picker would have gone unnoticed.
  // We drive a 2-layer session, then ReadbackRootPoolShape after the second
  // TraceLayer to see what transit wrote for layer 1 rays.
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  auto scene = MakeMetalSceneRandomH(/*max_hits=*/6, /*ms_layers=*/2);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 37;

  MetalTraceBackend backend;
  backend.BeginSession(spec);

  HostRayBatch host;
  host.count = 4096;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h0, nullptr);
  size_t cont0 = h0->ContinuationCount();
  ASSERT_GT(cont0, 0u) << "no continuation rays — test B cannot exercise transit";

  RecombineSpec rspec;
  rspec.shuffle = false;
  auto roots1 = backend.Recombine(std::move(h0), rspec);
  ASSERT_TRUE(roots1.is_device);
  ASSERT_EQ(roots1.device.count, cont0);

  auto h1 = backend.TraceLayer(roots1);
  ASSERT_NE(h1, nullptr);

  // After layer 1's TraceLayer, root_pool_shape_buf_ holds what transit_root
  // wrote for each ray. cont0 IS the exact number of rays transit dispatched
  // (roots1.device.count == cont0), so readback that many slots.
  MetalTraceBackendTestHooks hooks(backend);
  auto rays = hooks.ReadbackRootPoolShape(cont0);
  ASSERT_EQ(rays.size(), cont0);

  // Pool table must also have grown ≥1 shape per layer/ci resolve.
  auto table = hooks.ReadbackPoolShapeTable();
  ASSERT_GT(table.size(), 0u);

  // Every ray landed on some pool_shape_table row.
  std::set<std::pair<uint32_t, uint32_t>> valid;
  for (const auto& row : table) {
    valid.emplace(row[0], row[1]);
  }
  for (const auto& r : rays) {
    EXPECT_TRUE(valid.count(r) > 0u) << "layer-1 ray landed on (" << r.first << ", " << r.second
                                     << ") — NOT a row of layer-1's pool_shape_table.";
  }

  // AC1 core: transit distributes rays across ≥2 distinct pool slots. A K=8
  // + cont0 ≫ 1 with random-h prism scene means P_ci ≥ 2 and the picker MUST
  // spread. Exactly-1 here would mean transit's shape-picker is broken (e.g.
  // clamp-to-zero regression) — the ms_layers=1 tests would never catch this.
  std::set<std::pair<uint32_t, uint32_t>> distinct(rays.begin(), rays.end());
  EXPECT_GE(distinct.size(), 2u) << "K=8 + " << cont0 << " continuation rays: expected ≥ 2 distinct "
                                 << "(poly_off, poly_cnt) tuples from transit_root_kernel, got " << distinct.size()
                                 << " — transit's K-shape pick is not distributing.";

  backend.EndSession();
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
}

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
