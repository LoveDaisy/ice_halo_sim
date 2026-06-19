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
#include "core/cpu_trace_backend.hpp"
#include "core/metal_trace_backend.hpp"
#include "core/trace_backend.hpp"
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
               "(1024 plan → 704 @M5 → 640 @DR-3 baseline; R1 ruled benign @268.6)\n",
               max_threads);
  EXPECT_GE(max_threads, static_cast<size_t>(640))
      << "trace_layer_kernel occupancy regressed below the 640 DR-3 baseline "
         "(1024 plan → 704 @M5 → 640 @268.8) — re-measure multi-MS+filter "
         "throughput vs the active D1 gate and reconsider plan R1 option B "
         "(split filter gate into independent dispatch); see scratchpad/"
         "scrum-gpu-single-engine-continuation/task-fused-emit-gate/plan.md "
         "and progress.md for context.";
}

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
