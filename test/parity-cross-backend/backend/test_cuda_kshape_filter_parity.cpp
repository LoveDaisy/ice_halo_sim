// CUDA K-shape pool × filter judgment correctness.
//
// Purpose. The pre-K-shape-pool CUDA tests all ran with `filter.type = None`
// (short-circuits DeviceFilterCheck to true), so a regression where the
// K-shape pool corrupts filter judgment — the same class of Metal blindspot
// where `path[]` stored absolute pool-wide indices and downstream
// `ApplyGetFn_dev` narrowed them to 8-bit garbage — would slip past every
// pre-existing CUDA parity test. This test's job is to close that gap for
// CUDA: with an EntryExit filter active on the final layer, does turning
// on the K knob change the filter's admission rate in a way that goes
// beyond Monte-Carlo noise?
//
// Design (mirrors plan §4 Step 5). Two arms compared:
//
//   * CUDA-internal mean-invariance (K=0 vs K=8) — the strongest signal
//     directly targeting the K-shape pool's filter integrity. K only
//     redistributes shape draws within a batch; the per-batch filter-
//     admission-rate expectation is invariant in K. Sample multi-seed
//     means and compare via pooled 3σ. Directly excludes the "K-shape
//     pool subtly biases filter judgment" failure mode without the
//     CPU-vs-CUDA population-mismatch confounds.
//
//   * CPU vs CUDA(K=8) loose ballpark — a cross-backend sanity anchor.
//     CPU fans out reflect + refract as continuations (different ray
//     population from CUDA's refract-priority continuation); the
//     admission rate is directionally comparable but not numerically
//     tight. This arm catches the "K-shape pool + filter → orders-of-
//     magnitude admission collapse" failure mode; tight numeric parity
//     is out of scope by construction (mirrors the pattern in
//     test_cuda_component_mask_parity.cpp:485-496).
//
// Sensitivity guardrail. The filter must not reject everything —
// mean-invariance and CPU-ballpark tests both need non-zero exit_w_sum
// on every arm to have signal. `ASSERT_GT(mean, 0)` in each arm.
// (An "isn't a no-op" check — filter admission strictly < unfiltered —
// is intentionally omitted: it would double the runtime for a signal
// the CPU-vs-CUDA ballpark arm already covers by proxy, since a filter
// that admits every ray on both backends produces near-identical
// exit_w_sum ratios regardless.)
//
// Reverse-verification note (plan §4 Step 5 / §7 risk 1). Mac has no
// CUDA — this TU is `#if defined(LUMICE_CUDA_ENABLED)`-guarded and
// `GTEST_SKIP`s on hosts without a CUDA device. The Test D
// detection-power evidence (temporarily corrupt trace_single_ms_kernel's
// path[] write to add poly_off, rebuild, confirm this test fails, revert,
// rebuild, confirm it passes) must be produced on dev49 as part of the
// Step 7 checklist — writing the test on Mac is code-landing, not
// detection-power evidence.
//
// Build gate: `#if defined(LUMICE_CUDA_ENABLED)` — TU empty on non-CUDA
// hosts.

#include <gtest/gtest.h>

#if defined(LUMICE_CUDA_ENABLED)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "cuda_test_helpers.hpp"

namespace lumice {
namespace {

using cuda_test::MakeRenderConfig;
using cuda_test::MakeStochasticPrismScene;

bool ShouldSkipCudaTests() {
  return !CudaDeviceAvailable();
}

// Single-layer stochastic hex-prism scene with an EntryExit(min_len=3)
// filter on the final layer. min_len=3 selects a middle-of-the-road
// admission fraction — not "everyone" (which would make the test vacuous)
// nor "no one" (which leaves nothing to compare); it exercises the
// FilterSpec path length gate that consumes `path[]` values.
SceneConfig MakeFilteredStochasticScene(size_t max_hits) {
  SceneConfig scene = MakeStochasticPrismScene(max_hits, /*gaussian=*/true);
  auto& setting = scene.ms_[0].setting_[0];
  FilterConfig& fc = setting.filter_;
  fc.id_ = 0;
  fc.symmetry_ = FilterConfig::kSymNone;
  fc.action_ = FilterConfig::kFilterIn;
  EntryExitFilterParam ee;
  ee.entry_ = std::nullopt;
  ee.exit_ = std::nullopt;
  ee.min_len_ = 3;  // ~40-60% admission on hex prism / max_hits=6 (tunable).
  fc.param_ = SimpleFilterParam{ ee };
  return scene;
}

struct Stats {
  double mean = 0.0;
  double sd = 0.0;
  size_t n = 0;
};

Stats ComputeStats(const std::vector<double>& xs) {
  Stats s;
  s.n = xs.size();
  if (s.n == 0) {
    return s;
  }
  double sum = 0.0;
  for (double v : xs) {
    sum += v;
  }
  s.mean = sum / static_cast<double>(s.n);
  double sq = 0.0;
  for (double v : xs) {
    double d = v - s.mean;
    sq += d * d;
  }
  s.sd = std::sqrt(sq / static_cast<double>(s.n));
  return s;
}

// Run a single CUDA session and return (exit_count, exit_w_sum) via
// LayerStats. `k_env` selects the K knob for this run (nullptr → unset).
std::pair<size_t, double> RunCudaOnce(const SceneConfig& scene, const RenderConfig& render, uint32_t seed,
                                      size_t ray_count, const char* k_env) {
  if (k_env != nullptr) {
    ::setenv("LUMICE_GPU_GEOM_CLOCK", k_env, /*overwrite=*/1);
  } else {
    ::unsetenv("LUMICE_GPU_GEOM_CLOCK");
  }
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;
  spec.ray_num = ray_count;

  HostRayBatch host;
  host.count = ray_count;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  CudaTraceBackend backend;
  backend.BeginSession(spec);
  auto h = backend.TraceLayer(RootRaySource::FromHost(host));
  LayerStats st{};
  if (h) {
    st = h->GetLayerStats();
  }
  backend.EndSession();
  return { st.exit_count, static_cast<double>(st.exit_w_sum) };
}

std::pair<size_t, double> RunCpuOnce(const SceneConfig& scene, const RenderConfig& render, uint32_t seed,
                                     size_t ray_count) {
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;

  HostRayBatch host;
  host.count = ray_count;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  CpuTraceBackend backend;
  backend.BeginSession(spec);
  auto h = backend.TraceLayer(RootRaySource::FromHost(host));
  LayerStats st{};
  if (h) {
    st = h->GetLayerStats();
  }
  backend.EndSession();
  return { st.exit_count, static_cast<double>(st.exit_w_sum) };
}

constexpr size_t kNumSeeds = 8;
constexpr size_t kRaysPerBatch = 1024;
constexpr size_t kMaxHits = 6;

// Arm 1 — CUDA K=0 vs K=8 mean-invariance under an active EntryExit filter.
// The failure mode this arm targets: K-shape pool leaks corrupt `path[]`
// values (absolute vs local) into the filter path such that the filter
// admits systematically fewer / more rays at K=8 than at K=0. K itself
// only redistributes per-batch shape variance; the mean over seeds must
// be invariant.
TEST(CudaKShapeFilterParity, KDoesNotBiasEntryExitFilterAdmission_AC1) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  auto scene = MakeFilteredStochasticScene(kMaxHits);
  auto render = MakeRenderConfig();

  std::vector<double> w_k0;
  std::vector<double> w_k8;
  w_k0.reserve(kNumSeeds);
  w_k8.reserve(kNumSeeds);
  for (uint32_t s = 1; s <= kNumSeeds; ++s) {
    auto [_c0, w0] = RunCudaOnce(scene, render, s, kRaysPerBatch, /*k_env=*/nullptr);
    auto [_c8, w8] = RunCudaOnce(scene, render, s, kRaysPerBatch, /*k_env=*/"8");
    w_k0.push_back(w0);
    w_k8.push_back(w8);
  }
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");

  Stats s0 = ComputeStats(w_k0);
  Stats s8 = ComputeStats(w_k8);

  // Sensitivity — both arms must produce non-zero weight (filter isn't
  // silently rejecting everything, and the trace isn't broken).
  ASSERT_GT(s0.mean, 0.0) << "CUDA K=0 filter arm produced zero total weight — filter admits nothing "
                             "(vacuous test) or trace is broken.";
  ASSERT_GT(s8.mean, 0.0) << "CUDA K=8 filter arm produced zero total weight — K-shape pool + filter "
                             "combination admits nothing (regression) or trace is broken.";

  // Pooled 3σ mean-invariance bar (mirrors the pattern from Metal
  // KShapePool_KEnabledReducesCrossSeedVariance_AC2 but focused on filter
  // admission, not variance): |mean(K=8) - mean(K=0)| must fit inside a
  // conservative 3× pooled-SE window. A filter-judgment bias introduced by
  // the K-shape pool would drive this gap far past 3σ (Metal's absolute-
  // path-[] regression collapsed admission by orders of magnitude when
  // filter was active).
  double pooled_se = std::sqrt(s0.sd * s0.sd + s8.sd * s8.sd) / std::sqrt(static_cast<double>(kNumSeeds));
  double mean_gap = std::abs(s0.mean - s8.mean);
  EXPECT_LT(mean_gap, 3.0 * pooled_se) << "AC1 filter-K mean invariance: |mean(K=0) - mean(K=8)| = " << mean_gap
                                       << " exceeds 3× pooled SE = " << (3.0 * pooled_se) << " (mean_k0=" << s0.mean
                                       << " sd_k0=" << s0.sd << ", mean_k8=" << s8.mean << " sd_k8=" << s8.sd
                                       << ") — K-shape pool biases filter admission relative to K=0.";
}

// Arm 2 — CPU vs CUDA(K=8) loose ballpark under the same filter. The two
// backends trace DIFFERENT ray populations (CPU fans out reflect + refract
// as continuations; CUDA continues only refract) so per-ray parity is not
// possible — but the filter-admission ratio should live in the same order
// of magnitude. This arm catches the "K-shape pool + filter → order-of-
// magnitude admission collapse" failure that a same-backend test cannot
// distinguish from "the trace itself broke" (both would show up as low w).
// Tight numeric parity is out of scope (loose 3× admission-ratio band).
TEST(CudaKShapeFilterParity, KEnabledFilterAdmissionMatchesCpuBallpark_AC1) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }
  ::setenv("LUMICE_GPU_GEOM_CLOCK", "8", /*overwrite=*/1);

  auto scene = MakeFilteredStochasticScene(kMaxHits);
  auto render = MakeRenderConfig();

  std::vector<double> w_cuda;
  std::vector<double> w_cpu;
  w_cuda.reserve(kNumSeeds);
  w_cpu.reserve(kNumSeeds);
  for (uint32_t s = 1; s <= kNumSeeds; ++s) {
    auto [_cc, wc] = RunCudaOnce(scene, render, s, kRaysPerBatch, /*k_env=*/"8");
    auto [_pc, wp] = RunCpuOnce(scene, render, s, kRaysPerBatch);
    w_cuda.push_back(wc);
    w_cpu.push_back(wp);
  }
  ::unsetenv("LUMICE_GPU_GEOM_CLOCK");

  Stats sc = ComputeStats(w_cuda);
  Stats sp = ComputeStats(w_cpu);

  ASSERT_GT(sc.mean, 0.0) << "CUDA(K=8) filter arm produced zero weight — trace broke or filter rejects all.";
  ASSERT_GT(sp.mean, 0.0) << "CPU filter arm produced zero weight — trace broke or filter rejects all.";

  // Ratio must live in [1/3, 3]. The two backends' fan-out policies make
  // absolute weights differ, but a K-shape pool bug that made the filter
  // reject an order of magnitude more rays would drop CUDA's mean weight
  // sharply while CPU's stays anchored — the ratio would blow past this
  // band (Metal's regression collapsed admission by ~10x).
  double ratio = sc.mean / sp.mean;
  EXPECT_GT(ratio, 1.0 / 3.0) << "CUDA(K=8) filter admission far below CPU ballpark: " << "cuda_mean=" << sc.mean
                              << " cpu_mean=" << sp.mean << " ratio=" << ratio
                              << " — probable K-shape pool × filter regression.";
  EXPECT_LT(ratio, 3.0) << "CUDA(K=8) filter admission far above CPU ballpark: " << "cuda_mean=" << sc.mean
                        << " cpu_mean=" << sp.mean << " ratio=" << ratio
                        << " — probable K-shape pool × filter regression.";
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
