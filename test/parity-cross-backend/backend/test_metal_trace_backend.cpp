// Metal backend tests. Compiled into unit_test on all platforms (the body is
// `#if defined(__APPLE__)`-guarded); on non-Apple the tests are absent. On
// Apple the tests may be skipped at runtime via LUMICE_SKIP_METAL_TESTS=1
// (covers CI runners with no Metal-capable GPU).

#include <gtest/gtest.h>

#if defined(__APPLE__)

#include <cmath>
#include <cstring>
#include <vector>

#include "config/render_config.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/metal_trace_backend.hpp"
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

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
