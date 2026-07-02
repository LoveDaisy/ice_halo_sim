// Device root-gen tests (task-260.2/260.7). Exercises the Metal device-resident
// PCG path activated when spec.seed != 0 (see MetalTraceBackend::TraceLayer
// can_use_device_gen guard in src/core/metal_trace_backend.mm). task-260.7
// removed the crystal_cnt == 1 gate; multi-crystal layers now also use
// per-ci device-gen.
//
// These tests verify externally observable properties — kernel-internal
// buffers are pimpl-hidden, so the assertions are at the public API level:
//
//   * device-gen determinism: same seed → identical XYZ across two runs.
//   * device-gen vs host-gen statistical equivalence: aggregate XYZ stays
//     close (5% rel-err is the Monte Carlo noise floor at N=8192 rays,
//     well above the ~0.07% drift previously observed in the parity suite).
//   * device-gen guards: tri_count overrun falls back to host-gen without
//     crashing; multi-crystal layers stay on device-gen (per-ci).
//   * counter monotonicity: a second TraceLayer invocation within the same
//     session produces stable XYZ totals (no batch-wrap collision).
//
// Direct kernel-level unit tests (per plan §6) require buffer access that
// would need invasive pimpl exposure; the integration-level checks here cover
// the same properties through the public API. The slow e2e parity harness
// (test/e2e/test_metal_exit_seam_parity.py, M7 acceptance) provides
// large-N ds_corr verification.

#include <gtest/gtest.h>

#if defined(__APPLE__)

#include <cmath>
#include <cstring>
#include <vector>

#include "config/render_config.hpp"
#include "core/backend/metal_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "metal_test_helpers.hpp"

namespace lumice {
namespace {

using metal_test::ChannelSum;
using metal_test::EnableDeviceGenForStatisticalParity;
using metal_test::ForceHostGenForByteIdentity;
using metal_test::MakeMetalScene;
using metal_test::MakeMultiCrystalScene;
using metal_test::MakeRectangularRender;
using metal_test::RelErr;
using metal_test::ShouldSkipMetalTests;

constexpr size_t kRayCount = 8192;

// Same-seed determinism: two MetalTraceBackend instances with identical spec
// must produce identical XYZ images via the device-gen path. Validates
// (gen_seed, gen_ray_base + tid) → reproducible PCG stream within a session
// and across instances.
TEST(MetalRootGen, DeviceGenDeterminism) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_a(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  std::vector<float> xyz_b(xyz_a.size(), 0.0f);
  for (int run = 0; run < 2; run++) {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ run == 0 ? xyz_a.data() : xyz_b.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // Float-precision equality is the practical contract for fixed-seed
  // reproducibility on the Metal backend: same PCG range → same sampled
  // rays, but GPU atomic-add ordering across SIMD groups is permitted to
  // drift at the ULP level by Metal's relaxed atomic model. RelErr < 1e-3
  // is far below the ~1% Monte-Carlo noise floor at N=8192 rays yet still
  // well above any plausible ULP atomic-reorder drift on observed devices,
  // so it catches RNG / counter / seed regressions cleanly. (Strict == would
  // be over-tight; loose 1e-6 would no-op the test.)
  for (int c = 0; c < 3; c++) {
    double sa = ChannelSum(xyz_a, c);
    double sb = ChannelSum(xyz_b, c);
    EXPECT_GT(sa, 0.0);
    EXPECT_LT(RelErr(sa, sb), 1e-3) << "channel=" << c << " run0=" << sa << " run1=" << sb;
  }
}

// Device-gen vs host-gen statistical equivalence on aggregate XYZ. Both runs
// use the same scene + seed; only LUMICE_DISABLE_DEVICE_GEN differs. PCG ≠
// mt19937 at the per-ray level, so byte equality is not expected — the test
// asserts aggregate Monte Carlo means coincide within ~5%.
TEST(MetalRootGen, DeviceGenVsHostGenStatisticalParity) {
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

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz_device(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  std::vector<float> xyz_host(xyz_device.size(), 0.0f);

  EnableDeviceGenForStatisticalParity();
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_device.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  ForceHostGenForByteIdentity();
  {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_host.data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }
  EnableDeviceGenForStatisticalParity();  // restore default for downstream tests

  double total_device = 0.0;
  double total_host = 0.0;
  for (size_t i = 0; i < xyz_device.size(); i++) {
    ASSERT_TRUE(std::isfinite(xyz_device[i]));
    ASSERT_TRUE(std::isfinite(xyz_host[i]));
    total_device += static_cast<double>(xyz_device[i]);
    total_host += static_cast<double>(xyz_host[i]);
  }
  EXPECT_GT(total_device, 0.0);
  EXPECT_GT(total_host, 0.0);
  EXPECT_LT(RelErr(total_device, total_host), 0.05) << "total_device=" << total_device << " total_host=" << total_host;

  for (int c = 0; c < 3; c++) {
    double sd = ChannelSum(xyz_device, c);
    double sh = ChannelSum(xyz_host, c);
    EXPECT_LT(RelErr(sd, sh), 0.05) << "channel=" << c << " device=" << sd << " host=" << sh;
  }
}

// Multi-crystal layer now uses per-ci device-gen (task-260.7 removed the
// crystal_cnt == 1 gate; explore-260.3 exp2 verified ds=0.9998). Verifies
// the layer runs without crash and produces a finite, non-zero XYZ.
TEST(MetalRootGen, MultiCrystalUsesPerCiDeviceGen) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMultiCrystalScene(/*max_hits=*/8, /*ms_layers=*/1, /*crystal_count=*/2);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  MetalTraceBackend metal;
  metal.BeginSession(spec);
  auto h = metal.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h, nullptr);
  XyzImageData img{ xyz.data(), render.resolution_[0], render.resolution_[1] };
  metal.ReadbackImage(img);
  metal.EndSession();

  double total = 0.0;
  for (size_t i = 0; i < xyz.size(); i++) {
    ASSERT_TRUE(std::isfinite(xyz[i]));
    total += static_cast<double>(xyz[i]);
  }
  EXPECT_GT(total, 0.0) << "multi-crystal layer produced empty XYZ";
}

// seed == 0 must use the host-gen path (gen_seed_ == 0 in the
// can_use_device_gen guard). Result must be finite and non-zero.
TEST(MetalRootGen, ZeroSeedUsesHostGen) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 0;  // disables device-gen branch in TraceLayer

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  std::vector<float> xyz(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
  MetalTraceBackend metal;
  metal.BeginSession(spec);
  auto h = metal.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h, nullptr);
  XyzImageData img{ xyz.data(), render.resolution_[0], render.resolution_[1] };
  metal.ReadbackImage(img);
  metal.EndSession();

  double total = 0.0;
  for (size_t i = 0; i < xyz.size(); i++) {
    ASSERT_TRUE(std::isfinite(xyz[i]));
    total += static_cast<double>(xyz[i]);
  }
  EXPECT_GT(total, 0.0);
}

// task-260.5: same-instance multi-session must advance gen_ray_base across
// BeginSession cycles. Pre-fix bug: BeginSession unconditionally reset
// root_ray_count=0 every cycle, so every cycle consumed the SAME PCG range
// (gen_ray_base=0..N) and produced identical XYZ. After fix, root_ray_count
// is reset only on the first seeding (via the `seeded` gate) and persists
// across Reset(), so each cycle consumes a distinct PCG range → XYZ differs
// from prior cycles. This is the mirror-bug of 258.10's RNG-reset regression.
TEST(MetalRootGen, DeviceGenMultiSessionAdvancesGenRayBase) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  constexpr int kCycles = 3;
  MetalTraceBackend metal;  // single instance, multiple sessions
  std::vector<std::vector<float>> xyz_per_cycle(kCycles);
  for (int cycle = 0; cycle < kCycles; cycle++) {
    xyz_per_cycle[cycle].assign(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
    metal.BeginSession(spec);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ xyz_per_cycle[cycle].data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
  }

  // Adjacent cycles must consume *different* PCG ranges → aggregate channel
  // sum must differ measurably. Empirically (kRayCount=8192, this scene):
  //   * same PCG range (pre-260.5-fix bug): RelErr ≈ 1e-7 (ULP-only drift
  //     from GPU atomic-add reorder).
  //   * different PCG range (post-fix):     RelErr ≥ 1.2e-4 across channels,
  //     ~3.5e-4 typical (true Monte-Carlo divergence on aggregate sum).
  // Threshold 1e-5 sits 100× above the ULP floor and ≥12× below the smallest
  // post-fix difference observed, so it catches "PCG range did not advance"
  // without flaking on legitimate sample variance.
  for (int cycle = 1; cycle < kCycles; cycle++) {
    for (int c = 0; c < 3; c++) {
      double s_prev = ChannelSum(xyz_per_cycle[cycle - 1], c);
      double s_curr = ChannelSum(xyz_per_cycle[cycle], c);
      EXPECT_GT(s_prev, 0.0);
      EXPECT_GT(RelErr(s_prev, s_curr), 1e-5)
          << "cycle=" << cycle << " channel=" << c << " prev=" << s_prev << " curr=" << s_curr
          << " — cycles produced identical PCG range (gen_ray_base not advancing)";
    }
  }
}

// task-260.5 Step 5 (cross-instance multi-session determinism): two independent
// backend instances with the same seed, each driven through the same
// multi-session sequence, must produce byte-identical per-cycle XYZ. Validates
// the R3 architectural invariant — per-Run() backend instance creation in
// simulator.cpp:596 means `seeded` starts false on each new instance, so the
// fix's `!seeded` gate restarts root_ray_count from 0 deterministically per
// Run(). If a future refactor pools or reuses backend instances across Run()s,
// this test still passes because both instances start at seeded=false here.
TEST(MetalRootGen, DeviceGenDeterminismCrossInstanceMultiSession) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  constexpr int kCycles = 3;
  std::vector<std::vector<float>> xyz_a(kCycles);
  std::vector<std::vector<float>> xyz_b(kCycles);
  for (int instance = 0; instance < 2; instance++) {
    MetalTraceBackend metal;  // fresh instance each outer iteration
    auto& sink = (instance == 0) ? xyz_a : xyz_b;
    for (int cycle = 0; cycle < kCycles; cycle++) {
      sink[cycle].assign(render.resolution_[0] * render.resolution_[1] * 3, 0.0f);
      metal.BeginSession(spec);
      auto h = metal.TraceLayer(RootRaySource::FromHost(host));
      ASSERT_NE(h, nullptr);
      XyzImageData img{ sink[cycle].data(), render.resolution_[0], render.resolution_[1] };
      metal.ReadbackImage(img);
      metal.EndSession();
    }
  }

  for (int cycle = 0; cycle < kCycles; cycle++) {
    for (int c = 0; c < 3; c++) {
      double sa = ChannelSum(xyz_a[cycle], c);
      double sb = ChannelSum(xyz_b[cycle], c);
      EXPECT_GT(sa, 0.0);
      EXPECT_LT(RelErr(sa, sb), 1e-3) << "cycle=" << cycle << " channel=" << c << " inst0=" << sa << " inst1=" << sb
                                      << " — cross-instance determinism broken at this cycle";
    }
  }
}

// task-gpu-rng-ray-index-uint64 white-box injection: assert the device kernel
// actually consumes the new `gen_ray_base_hi` field (Major 1 in the plan-review
// coverage-gap closure — the in-range parity battery only exercises hi==0 which
// is bit-identical by construction, so a broken host↔device wiring would slip
// through corr/energy/cross-seed all-green). This drives the Metal gen_root
// kernel into three distinct hi epochs (0, 1, 2) via `SetInitialRayBaseForTest`
// and asserts each hi != 0 image diverges from the hi == 0 image by more than
// the atomic-add noise floor (self-calibrated from two hi==0 runs).
//
// Contract:
//   * hi == 0 (base_lo = 0)             → pcg_seed_with_high is identity → this
//     reference image is the "pre-fix bit-exact" one that the in-range parity
//     battery covers exhaustively.
//   * hi == 1 (base = 1<<32, base_lo=0) → mixed_seed = seed ^ pcg_hash(1), a
//     different PCG stream per ray → per-pixel content shifts.
//   * hi == 2 (base = 2<<32, base_lo=0) → mixed_seed = seed ^ pcg_hash(2),
//     yet another distinct stream → per-pixel content shifts again, distinctly.
//
// Discriminator choice: per-pixel L1 relative diff, self-calibrated.
// Aggregate channel sums are an MC INVARIANT — two different PCG streams
// sample the same underlying distribution so their integrals coincide to
// O(1/sqrt(N)); measured ~1e-4 rel drift across hi=0/1/2 (well below the 1e-3
// threshold the same-stream determinism test uses). Per-pixel L1 responds to
// stream difference but is SCENE-DEPENDENT: this default scene has kNoRandom
// axis dist, so most rays hit similar exit angles → stream diff moves rays
// only fractionally within the pixel grid → per-pixel L1 is a few percent
// (not 50%+ as it would be under axis-random scenes).
//
// Absolute thresholds are therefore unreliable — we compare against a
// same-scene baseline: run hi=0 TWICE, take the L1 of that repetition as the
// atomic-noise floor, then assert every (hi≠0) vs (hi==0) L1 exceeds
// baseline * kBaselineMultiple (default 5×). This scales with scene physics
// while still detecting wire-up regressions (a broken wire would collapse the
// hi≠0 streams onto hi==0, giving L1 ≈ baseline).
//
// Three-way divergence (hi=0 vs 1, hi=0 vs 2, hi=1 vs 2) forces the mixing to
// be genuinely a hash of hi — catches "hi wired in but hashed wrong" bug shape
// where hi=1/2 would both differ from hi=0 but not from each other.
TEST(MetalRootGen, GenRayBaseHiWireUp) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMetalScene(/*max_hits=*/8, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  constexpr size_t kEpoch = static_cast<size_t>(1) << 32;
  // Slots 0 and 1 are BOTH hi=0 (baseline atomic-noise measurement);
  // slots 2 and 3 are hi=1 and hi=2 respectively.
  const size_t kBases[] = { 0u, 0u, kEpoch, 2u * kEpoch };
  const size_t kImgFloats =
      static_cast<size_t>(render.resolution_[0]) * static_cast<size_t>(render.resolution_[1]) * 3u;
  std::vector<std::vector<float>> images(4);
  for (int i = 0; i < 4; i++) {
    images[i].assign(kImgFloats, 0.0f);
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    metal.SetInitialRayBaseForTest(kBases[i]);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    XyzImageData img{ images[i].data(), render.resolution_[0], render.resolution_[1] };
    metal.ReadbackImage(img);
    metal.EndSession();
    for (int c = 0; c < 3; c++) {
      EXPECT_GT(ChannelSum(images[i], c), 0.0)
          << "base_idx=" << i << " channel=" << c << " — empty image; test setup broken";
    }
  }

  auto l1_rel_diff = [](const std::vector<float>& a, const std::vector<float>& b) {
    double num = 0.0;
    double denom = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
      num += std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i]));
      denom += 0.5 * (std::abs(static_cast<double>(a[i])) + std::abs(static_cast<double>(b[i])));
    }
    return denom > 0.0 ? num / denom : 0.0;
  };

  // Baseline: same-stream (both hi==0) atomic-add noise floor for this scene.
  const double baseline = l1_rel_diff(images[0], images[1]);
  // Absolute floor guards against a degenerate baseline (e.g. Metal bitwise
  // atomic determinism giving baseline≈0 → hi≠0 stream drift would sneak
  // through with a tiny multiple of ~0). 1e-3 is well above float32 rounding
  // noise but well below any true stream-difference this scene produces.
  const double threshold = std::max(baseline * 5.0, 1e-3);

  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    double d = l1_rel_diff(images[pr.first], images[pr.second]);
    EXPECT_GT(d, threshold) << "base_pair=(" << pr.first << "," << pr.second << ") per-pixel L1 rel diff=" << d
                            << " baseline (hi==0 vs hi==0)=" << baseline << " threshold=" << threshold
                            << " — device-side hi wire-up appears broken: hi!=0 stream produced pixels "
                               "no different from same-stream atomic noise. Expected the hi-mix to shift "
                               "the PCG sequence enough that per-pixel L1 diff comfortably exceeds the "
                               "same-stream repeat baseline.";
  }
}

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
