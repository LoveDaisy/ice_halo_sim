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
#include <numeric>
#include <vector>

#include "config/render_config.hpp"
#include "core/backend/metal_trace_backend.hpp"
#include "core/backend/metal_trace_backend_test_hooks.hpp"  // scrum-328.2 Step 3
#include "core/backend/trace_backend.hpp"
#include "core/math.hpp"
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
// stream difference but is SCENE-DEPENDENT: under a kNoRandom axis dist most
// rays hit similar exit angles → stream diff moves rays only fractionally
// within the pixel grid → per-pixel L1 is only a few percent (and on some
// scene/lens combos washes out to exactly 0, as the CUDA sibling test hit).
// This test therefore uses a RANDOM crystal axis so the gen PCG stream drives
// the orientation sample → different mixed_seed → different exit *directions* →
// ~50%+ per-pixel divergence, robustly observable and not scene-fragile.
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
  // Random crystal orientation so the gen PCG stream drives orientation →
  // different mixed_seed → different exit directions → robust ~50% per-pixel
  // divergence (see rationale comment above; mirrors the CUDA sibling test).
  scene.ms_[0].setting_[0].crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  scene.ms_[0].setting_[0].crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };

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
    MetalTraceBackendTestHooks(metal).SetInitialRayBase(/*root_base=*/kBases[i],
                                                        /*transit_base=*/0u);  // ms_layers=1 → root stream only
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

// transit stream — RNG-isolated observation via the layer-1 rotation matrices
// (root_rot), NOT the XYZ image. The transit orientation R(tid) =
// build_crystal_rotation(sample_lat_lon_roll(transit_mixed_seed, tid)) is a pure
// function of (transit_seed, tid), so it is deterministic per tid and free of
// the atomic-continuation-compaction confound that makes the image / root_d
// non-deterministic across identical 2-layer runs. Injects ONLY the transit
// base (root stays hi==0) so layer-0 generation + the continuation set are
// identical across runs; a non-zero transit hi must move R(tid) at every tid.
// Layer 1 uses a RANDOM axis so sample_lat_lon_roll actually consumes the
// transit stream (a kNoRandom axis would ignore the seed → no observable). This
// closes the transit_root_kernel coverage gap the same way the CUDA sibling
// (CudaRngHiWiring.TransitStreamWireUp) does; Metal has no gate hi stream (the
// emit gate uses tid directly, scrum-267 §3.6), so gen + transit cover it.
TEST(MetalRootGen, TransitStreamWireUp) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  auto scene = MakeMetalScene(/*max_hits=*/6, /*ms_layers=*/2);
  // Layer 1 (the continuation/transit layer) gets a random axis so the transit
  // PCG stream drives its orientation sample.
  scene.ms_[1].setting_[0].crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  scene.ms_[1].setting_[0].crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
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
  const size_t kBases[] = { 0u, 0u, kEpoch, 2u * kEpoch };
  std::vector<std::vector<float>> rots(4);
  size_t cont_count = 0;
  for (int i = 0; i < 4; i++) {
    MetalTraceBackend metal;
    metal.BeginSession(spec);
    MetalTraceBackendTestHooks hooks(metal);
    hooks.SetInitialRayBase(/*root_base=*/0u, /*transit_base=*/kBases[i]);
    auto h0 = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h0, nullptr);
    const size_t cont = h0->ContinuationCount();
    ASSERT_GT(cont, 16u) << "base_idx=" << i << " — too few continuation rays (" << cont << ") to observe transit.";
    if (i == 0) {
      cont_count = cont;
    }
    ASSERT_EQ(cont, cont_count) << "base_idx=" << i << " — continuation count varies across runs; root not isolated.";
    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = metal.Recombine(std::move(h0), rspec);
    auto h1 = metal.TraceLayer(roots1);
    ASSERT_NE(h1, nullptr);
    size_t n = hooks.ReadbackRootRot(rots[i], cont_count);
    metal.EndSession();
    ASSERT_EQ(n, 9u * cont_count) << "base_idx=" << i << " — transit rot readback returned " << n;
  }

  EXPECT_EQ(rots[0], rots[1]) << "hi==0 transit rotations non-deterministic — cannot distinguish wiring from noise.";
  auto frac_moved = [](const std::vector<float>& a, const std::vector<float>& b) {
    const size_t n = a.size() / 9u;
    size_t moved = 0u;
    for (size_t r = 0; r < n; r++) {
      double d2 = 0.0;
      for (int k = 0; k < 9; k++) {
        const double e = static_cast<double>(a[9 * r + k]) - static_cast<double>(b[9 * r + k]);
        d2 += e * e;
      }
      if (d2 > 1e-10) {
        moved++;
      }
    }
    return n > 0u ? static_cast<double>(moved) / static_cast<double>(n) : 0.0;
  };
  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    const double moved = frac_moved(rots[pr.first], rots[pr.second]);
    EXPECT_GT(moved, 0.9) << "transit stream: base_pair=(" << pr.first << "," << pr.second << ") only " << moved
                          << " of layer-1 transit rotations differ — transit hi wiring not reaching the device.";
  }
}

// scrum-328.2 Step 7 (AC5a) — Facility consumer smoke, updated by scrum-328.3
// (Gaussian tight-envelope kLatPathRayleigh) and scrum-328.4 (Laplacian tight-
// envelope kLatPathLaplacianTightEnvelope, propose Gamma(2,b) + accept sin(θ)/θ).
// Drives Laplacian b=5 and Gaussian σ=5 near-pole configs with the observability
// facility (EnableGenAttemptCount + ReadbackGenAttemptCount) and asserts
// mean(attempts) matches the current anchors within ±5%:
// - Laplacian b=5 → 1.007 (kLatPathLaplacianTightEnvelope, ~99.3% acceptance;
//                          scrum-328.4 Step 1 exp4 MC estimate 1.0075, device-
//                          measured 1.00745 with seed 42 / 65536 rays)
// - Gaussian σ=5  → 1.002 (kLatPathRayleigh tight-envelope, ~99.8% acceptance)
//
// Near-pole config: latitude_dist.mean = 90° (= zenith 0, per math.cpp:638),
// so cos(φ)/M rejection loop actually kicks in. Sample size 65536 → MC
// standard error on mean(attempts) < 1% for a distribution whose stdev is
// ~4, well below the ±5% tolerance.
struct AttemptStats {
  double mean;
  int max;
  size_t safety_valve_hits;  // rays that hit kMaxRejectionAttempts (1000)
};

inline AttemptStats ComputeAttemptStats(const std::vector<int>& attempts) {
  AttemptStats out{ 0.0, 0, 0 };
  if (attempts.empty()) {
    return out;
  }
  const long long sum = std::accumulate(attempts.begin(), attempts.end(), 0LL);
  out.mean = static_cast<double>(sum) / static_cast<double>(attempts.size());
  for (int a : attempts) {
    if (a > out.max) {
      out.max = a;
    }
    if (a >= 1000) {  // pcg_shared.h kMaxRejectionAttempts
      out.safety_valve_hits++;
    }
  }
  return out;
}

TEST(RngObservabilityFacilitySmoke, NearPoleAcceptanceRateMatchesDocAnchors) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  // 65536 rays × 1 layer × 1 crystal = 1 ci; sample size covers MC SE < 1%.
  constexpr size_t kSmokeRayCount = 65536;

  struct Case {
    const char* label;
    DistributionType lat_type;
    float anchor_mean_attempts;  // doc/near-pole-area-measure-sampling.md §附录
  };
  const Case kCases[] = {
    // Laplacian b=5 (mean=90°, i.e. zenith=0): scrum-328.4 routes this to
    // kLatPathLaplacianTightEnvelope (propose Gamma(2,b), accept sin(θ)/θ, M=1).
    // Anchor is the measured device mean(attempts) with seed 42 / 65536 rays,
    // matching the Python MC estimate 1.0075 from exp4 to 3 decimals.
    { "Laplacian b=5", DistributionType::kLaplacian, 1.007 },
    // Gaussian σ=5 (mean=90°): after scrum-328.3 relaxed the Rayleigh threshold
    // to colatitude_center<0.5° (σ-independent, capped at σ<60° per plan risk 1),
    // this config now routes to kLatPathRayleigh with the tight-envelope
    // sin(θ)/θ accept step. Anchor is the measured device mean(attempts) with
    // seed 42 / 65536 rays (captured during scrum-328.3 Step 4); Python MC in
    // doc/near-pole-area-measure-sampling.md §附录 predicts ≈ 1.002 (99.8%
    // acceptance). ±5% band absorbs both MC noise and the Python-vs-device
    // arithmetic tolerance.
    { "Gaussian σ=5", DistributionType::kGaussian, 1.002 },
  };

  for (const Case& c : kCases) {
    auto scene = MakeMetalScene(/*max_hits=*/1, /*ms_layers=*/1);
    // Near-pole: latitude_dist mean=90° (config zenith=0 equivalent), std=5°.
    // Azimuth/roll uniform full-range so lon/roll draws don't perturb attempts.
    scene.ms_[0].setting_[0].crystal_.axis_.latitude_dist = Distribution{ c.lat_type, 90.0f, 5.0f };
    scene.ms_[0].setting_[0].crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    scene.ms_[0].setting_[0].crystal_.axis_.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };

    auto render = MakeRectangularRender();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 42;

    HostRayBatch host;
    host.count = kSmokeRayCount;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;

    MetalTraceBackend metal;
    metal.BeginSession(spec);
    MetalTraceBackendTestHooks hooks(metal);
    hooks.EnableGenAttemptCount(kSmokeRayCount);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr) << c.label << " — gen dispatch returned null.";

    std::vector<int> attempts;
    size_t n = hooks.ReadbackGenAttemptCount(attempts, kSmokeRayCount);
    metal.EndSession();
    ASSERT_EQ(n, kSmokeRayCount) << c.label << " — attempt-count readback returned " << n;

    const AttemptStats stats = ComputeAttemptStats(attempts);

    // Sanity: no ray should saturate the safety valve at these parameters.
    // Any hit would contaminate mean(attempts) with a 1000-clamped outlier and
    // invalidate the anchor comparison.
    EXPECT_EQ(stats.safety_valve_hits, 0u)
        << c.label << " — " << stats.safety_valve_hits
        << " rays hit kMaxRejectionAttempts=1000; mean would be biased. max=" << stats.max;

    // ±5% band (plan §Step 1 test point). Anchors from
    // scratchpad/explore-near-pole-latitude-sampling-divergence/insights.md
    // and doc/near-pole-area-measure-sampling.md §附录, verified by CPU
    // instrumentation of math.cpp:503 rejection loop.
    const double lo = c.anchor_mean_attempts * 0.95;
    const double hi = c.anchor_mean_attempts * 1.05;
    EXPECT_GE(stats.mean, lo) << c.label << " — mean(attempts)=" << stats.mean << " below anchor±5% [" << lo << ", "
                              << hi << "] (anchor=" << c.anchor_mean_attempts << ").";
    EXPECT_LE(stats.mean, hi) << c.label << " — mean(attempts)=" << stats.mean << " above anchor±5% [" << lo << ", "
                              << hi << "] (anchor=" << c.anchor_mean_attempts << ").";
  }
}

// code-review round 1 Major#2 regression: two real crystal instances (ci=0/1)
// within ONE MS layer must write their gen-attempt-count windows into
// DISJOINT buffer regions. Before this fix, TraceLayer's ci-loop passed the
// SAME static `lat_attempts_ci_start_` base to every ci's EncodeGenRoot
// dispatch — with a non-zero base ci_start armed, this both (a) let the
// second ci's dispatch silently clobber the first ci's already-written
// window, and (b) allowed the kernel to write past the [0, count)
// allocation (EnableGenAttemptCount only sized the buffer for `count`, not
// `ci_start + count`). This test arms a non-zero base ci_start and asserts:
// the region before the base stays untouched, and the full multi-ci window
// after the base is entirely written (a collision/gap would leave the
// portion of the smaller ci's window that the larger ci's write clobbered —
// or, pre-fix, the tail past the too-small allocation — as unwritten zeros).
TEST(RngObservabilityFacilitySmoke, MultiCiAttemptWindowsDoNotOverwrite) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  constexpr size_t kTotal = 16000;      // MakeMultiCrystalScene: 0.7/0.3 split → 11200/4800
  constexpr size_t kCiStartBase = 37u;  // non-zero base offset (AC2 literal requirement)

  auto scene = MakeMultiCrystalScene(/*max_hits=*/1, /*ms_layers=*/1, /*crystal_count=*/2);
  for (auto& setting : scene.ms_[0].setting_) {
    // Off-pole Laplacian mean=80° (colatitude_center=10° > kPolarThresholdRad=0.5°) so
    // SelectLatPath routes to kLatPathGenericReject regardless of scrum-328.4's tight-
    // envelope path — the GenericReject rejection loop keeps `attempts` varying so a
    // trivial always-1 value would not distinguish "written" from "never-touched-but-
    // happens-to-read-as-1". Prior scrum-328.2 revision used mean=90° / b=5°, which
    // scrum-328.4 rerouted to LaplacianTightEnvelope with attempts≈1 (would silently
    // erode this test's discriminating power).
    setting.crystal_.axis_.latitude_dist = Distribution{ DistributionType::kLaplacian, 80.0f, 5.0f };
    setting.crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    setting.crystal_.axis_.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  }

  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kTotal;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  MetalTraceBackend metal;
  metal.BeginSession(spec);
  MetalTraceBackendTestHooks hooks(metal);
  hooks.EnableGenAttemptCount(kTotal, kCiStartBase);
  auto h = metal.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h, nullptr);

  std::vector<int> attempts;
  size_t n = hooks.ReadbackGenAttemptCount(attempts, kCiStartBase + kTotal);
  metal.EndSession();
  ASSERT_EQ(n, kCiStartBase + kTotal) << "readback returned fewer elements than allocated — capacity fix regressed.";

  for (size_t i = 0; i < kCiStartBase; i++) {
    ASSERT_EQ(attempts[i], 0) << "slot " << i << " before ci_start base was written — base offset not honored.";
  }

  size_t zero_count = 0;
  for (size_t i = kCiStartBase; i < kCiStartBase + kTotal; i++) {
    if (attempts[i] == 0) {
      zero_count++;
    }
  }
  EXPECT_EQ(zero_count, 0u)
      << zero_count << " of " << kTotal
      << " attempt-count slots in [ci_start, ci_start+total) were never written — multi-crystal-instance "
      << "windows are colliding instead of landing in disjoint per-ci slices.";
}

// scrum-328.3 Step 4(b) — uniform near-pole acceptance-rate smoke. The
// ComputeJacobianEnvelope kUniform branch was tightened from a loose M=1.0 to
// the exact bounded-uniform envelope M = cos(max(|mean|-std/2, 0)°). At
// mean=90° / std=15°, the proposal support is [82.5°, 97.5°] (folded to
// [82.5°, 90°] via NormalizeLatitude); E[cos(phi)] ≈ 0.06545 and
// M_new = cos(82.5°) ≈ 0.13053, giving a theoretical accept probability
// ≈ 0.501 and mean(attempts) ≈ 1.996. Old M=1.0 would give ≈ 15.3.
// Test asserts the new mean(attempts) lands in a band tight enough to reject
// the old-M baseline (≤ 6) and confirms no safety-valve hits.
TEST(RngObservabilityFacilitySmoke, NearPoleUniformAcceptanceRateBeatsBaseline) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  constexpr size_t kSmokeRayCount = 65536;
  auto scene = MakeMetalScene(/*max_hits=*/1, /*ms_layers=*/1);
  scene.ms_[0].setting_[0].crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 90.0f, 15.0f };
  scene.ms_[0].setting_[0].crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  scene.ms_[0].setting_[0].crystal_.axis_.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };

  auto render = MakeRectangularRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kSmokeRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  MetalTraceBackend metal;
  metal.BeginSession(spec);
  MetalTraceBackendTestHooks hooks(metal);
  hooks.EnableGenAttemptCount(kSmokeRayCount);
  auto h = metal.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h, nullptr);

  std::vector<int> attempts;
  size_t n = hooks.ReadbackGenAttemptCount(attempts, kSmokeRayCount);
  metal.EndSession();
  ASSERT_EQ(n, kSmokeRayCount);

  const AttemptStats stats = ComputeAttemptStats(attempts);
  EXPECT_EQ(stats.safety_valve_hits, 0u);

  // Theoretical mean(attempts) ≈ 1.996 with new tight M; ±15% band absorbs MC
  // noise + the exact fold-integral approximation. Tight enough to reject the
  // old M=1.0 baseline (which would give ~15).
  constexpr double kAnchor = 1.996;
  const double lo = kAnchor * 0.85;
  const double hi = kAnchor * 1.15;
  EXPECT_GE(stats.mean, lo) << "uniform-near-pole mean(attempts)=" << stats.mean << " below anchor±15% [" << lo << ", "
                            << hi << "] (anchor=" << kAnchor << ").";
  EXPECT_LE(stats.mean, hi) << "uniform-near-pole mean(attempts)=" << stats.mean << " above anchor±15% [" << lo << ", "
                            << hi << "] (anchor=" << kAnchor << ").";
  // Independent rejection of old-M=1.0 baseline (would give ~15 attempts).
  EXPECT_LT(stats.mean, 6.0) << "uniform-near-pole mean(attempts)=" << stats.mean
                             << " — tight envelope not active? old M=1.0 baseline would give ~15.";
}

// scrum-328.3 Step 4(c) — device (Metal) vs CPU (math.cpp::SampleSphericalPointsSph)
// distribution-shape parity for the new Rayleigh tight-envelope path. AC2 hard
// constraint: cannot be bit-parity with the OLD sampler (new one is intentionally
// more precise); must match the analytic target ∝ exp(-θ²/2σ²)·sin(θ) at the
// distribution level. This test hits the same target with both backends
// (Gaussian σ=5, mean=90°, seed 42, N=65536) and compares distribution moments
// (mean + variance of the crystal-axis colatitude) to be equal within Monte
// Carlo tolerance. Not a bit-parity assertion — RNG streams differ across
// backends — the check is that both samplers describe the same distribution
// shape for the axis latitude drawn from the tight-envelope Rayleigh path.
//
// Extraction of the crystal-axis direction from the per-ray rotation matrix
// stored in root_rot_buf (via ReadbackRootRot): build_crystal_rotation_9
// (pcg_shared.h:402) builds R = Rz(lon-π)·Ry(lat-π/2)·Rz(roll) with row-major
// storage mat9[i*3+j] = R_ij. Applying R to the crystal-local +Z axis (0,0,1)
// gives axis_world = (mat9[2], mat9[5], mat9[8]); the Z component is
// mat9[8] = sin(lat). |mat9[8]| = cos(colatitude_from_pole), so
// colatitude = acos(|mat9[8]|) for both the +90° and −90° mean folds.
TEST(RngObservabilityFacilitySmoke, NearPoleGaussianDirsMatchCpuMoments) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  constexpr size_t kRayN = 65536;
  const AxisDistribution axis_dist = [] {
    AxisDistribution a;
    a.latitude_dist = Distribution{ DistributionType::kGaussian, 90.0f, 5.0f };
    a.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    a.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    return a;
  }();

  // --- Metal side: read gen'd crystal→world rotation matrices (9 floats/ray).
  std::vector<float> metal_rot;
  {
    auto scene = MakeMetalScene(/*max_hits=*/1, /*ms_layers=*/1);
    scene.ms_[0].setting_[0].crystal_.axis_ = axis_dist;

    auto render = MakeRectangularRender();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 42;

    HostRayBatch host;
    host.count = kRayN;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;

    MetalTraceBackend metal;
    metal.BeginSession(spec);
    MetalTraceBackendTestHooks hooks(metal);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    size_t n = hooks.ReadbackRootRot(metal_rot, kRayN);
    metal.EndSession();
    ASSERT_EQ(n, 9u * kRayN);
  }

  auto compute_moments = [](const std::vector<double>& xs) {
    double sum = 0.0;
    double sum_sq = 0.0;
    for (double x : xs) {
      sum += x;
      sum_sq += x * x;
    }
    const double n = static_cast<double>(xs.size());
    const double mean = sum / n;
    const double var = std::max(0.0, sum_sq / n - mean * mean);
    return std::pair<double, double>{ mean, var };
  };

  std::vector<double> metal_colat;
  metal_colat.reserve(kRayN);
  for (size_t r = 0; r < kRayN; r++) {
    float z = metal_rot[r * 9u + 8u];  // mat9[8] = sin(lat) = cos(colatitude_from_pole)
    z = std::max(-1.0f, std::min(1.0f, std::abs(z)));
    metal_colat.push_back(std::acos(static_cast<double>(z)));
  }
  const auto [metal_mean, metal_var] = compute_moments(metal_colat);

  // --- CPU side: same axis_dist, same-N, RandomSampler::SampleSphericalPointsSph.
  RandomNumberGenerator::GetInstance().SetSeed(42);
  std::vector<float> cpu_lon_lat(3u * kRayN);
  RandomSampler::SampleSphericalPointsSph(axis_dist, cpu_lon_lat.data(), kRayN);
  std::vector<double> cpu_colat;
  cpu_colat.reserve(kRayN);
  for (size_t r = 0; r < kRayN; r++) {
    // lon_lat layout: [lambda, phi, roll]. phi is latitude in [-π/2, π/2]; take
    // colatitude = π/2 - |phi| so south-pole folds match the Metal |mat9[8]|
    // fold above.
    const double phi = std::abs(static_cast<double>(cpu_lon_lat[3u * r + 1u]));
    cpu_colat.push_back(0.5 * lumice::math::kPi - phi);
  }
  const auto [cpu_mean, cpu_var] = compute_moments(cpu_colat);

  // For σ=5° = 0.0873 rad targeting ∝ exp(-θ²/2σ²)·sin(θ):
  //   analytic mean ≈ σ·√(π/2) ≈ 0.1094 rad (~6.27°).
  // Metal and CPU use independent PCG streams (different bases/global_idx) so
  // this is a distribution-shape check, not a per-ray comparison. Standard error
  // on the mean ≈ σ/√N ≈ 3.4e-4 rad; ±0.002 rad tolerance (~6σ_MC) leaves
  // margin for cross-backend tail-sampling differences without loosening enough
  // to admit the old sampler's shape.
  EXPECT_NEAR(metal_mean, cpu_mean, 0.002)
      << "Metal vs CPU axis-colatitude mean mismatch: metal=" << metal_mean << " cpu=" << cpu_mean
      << " (rad); tight-envelope samplers should target the same analytic distribution.";
  EXPECT_NEAR(metal_var, cpu_var, 0.0005) << "Metal vs CPU axis-colatitude variance mismatch: metal=" << metal_var
                                          << " cpu=" << cpu_var << " (rad²); distribution-shape parity failed.";
}

// scrum-328.4 Step 6 — Laplacian counterpart to NearPoleGaussianDirsMatchCpuMoments.
// Device (Metal) vs CPU (math.cpp::SampleSphericalPointsSph) distribution-shape parity
// for the new kLatPathLaplacianTightEnvelope path. AC2 hard constraint: cannot be
// bit-parity with the OLD GenericReject Laplacian sampler (which has a known M=cos(5b)
// tail-clamp bias, scrum-328.1 exp1); must match the analytic target
// ∝ exp(-θ/b)·sin(θ). Both backends hit this target with b=5°, mean=90°, seed 42,
// N=65536 and compare distribution moments of the crystal-axis colatitude. Extraction
// convention identical to the Gaussian variant: colatitude = acos(|mat9[8]|).
TEST(RngObservabilityFacilitySmoke, NearPoleLaplacianDirsMatchCpuMoments) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  EnableDeviceGenForStatisticalParity();

  constexpr size_t kRayN = 65536;
  const AxisDistribution axis_dist = [] {
    AxisDistribution a;
    a.latitude_dist = Distribution{ DistributionType::kLaplacian, 90.0f, 5.0f };
    a.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    a.roll_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    return a;
  }();

  std::vector<float> metal_rot;
  {
    auto scene = MakeMetalScene(/*max_hits=*/1, /*ms_layers=*/1);
    scene.ms_[0].setting_[0].crystal_.axis_ = axis_dist;

    auto render = MakeRectangularRender();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 42;

    HostRayBatch host;
    host.count = kRayN;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;

    MetalTraceBackend metal;
    metal.BeginSession(spec);
    MetalTraceBackendTestHooks hooks(metal);
    auto h = metal.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h, nullptr);
    size_t n = hooks.ReadbackRootRot(metal_rot, kRayN);
    metal.EndSession();
    ASSERT_EQ(n, 9u * kRayN);
  }

  auto compute_moments = [](const std::vector<double>& xs) {
    double sum = 0.0;
    double sum_sq = 0.0;
    for (double x : xs) {
      sum += x;
      sum_sq += x * x;
    }
    const double n = static_cast<double>(xs.size());
    const double mean = sum / n;
    const double var = std::max(0.0, sum_sq / n - mean * mean);
    return std::pair<double, double>{ mean, var };
  };

  std::vector<double> metal_colat;
  metal_colat.reserve(kRayN);
  for (size_t r = 0; r < kRayN; r++) {
    float z = metal_rot[r * 9u + 8u];
    z = std::max(-1.0f, std::min(1.0f, std::abs(z)));
    metal_colat.push_back(std::acos(static_cast<double>(z)));
  }
  const auto [metal_mean, metal_var] = compute_moments(metal_colat);

  RandomNumberGenerator::GetInstance().SetSeed(42);
  std::vector<float> cpu_lon_lat(3u * kRayN);
  RandomSampler::SampleSphericalPointsSph(axis_dist, cpu_lon_lat.data(), kRayN);
  std::vector<double> cpu_colat;
  cpu_colat.reserve(kRayN);
  for (size_t r = 0; r < kRayN; r++) {
    const double phi = std::abs(static_cast<double>(cpu_lon_lat[3u * r + 1u]));
    cpu_colat.push_back(0.5 * lumice::math::kPi - phi);
  }
  const auto [cpu_mean, cpu_var] = compute_moments(cpu_colat);

  // For b=5° = 0.0873 rad targeting ∝ exp(-θ/b)·sin(θ): analytic mean ≈ 2b ≈ 0.175 rad
  // (Gamma(2,b) mean 2b, with the sin(θ) reweight pulling slightly larger). Metal and
  // CPU use independent PCG streams, so this is a distribution-shape check, not a
  // per-ray comparison. Standard error on the mean ≈ b/√N ≈ 3.4e-4 rad; ±0.002 rad
  // tolerance mirrors the Gaussian variant above.
  EXPECT_NEAR(metal_mean, cpu_mean, 0.002)
      << "Metal vs CPU axis-colatitude mean mismatch: metal=" << metal_mean << " cpu=" << cpu_mean
      << " (rad); Laplacian tight-envelope samplers should target the same analytic distribution.";
  EXPECT_NEAR(metal_var, cpu_var, 0.0005) << "Metal vs CPU axis-colatitude variance mismatch: metal=" << metal_var
                                          << " cpu=" << cpu_var << " (rad²); distribution-shape parity failed.";
}

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
