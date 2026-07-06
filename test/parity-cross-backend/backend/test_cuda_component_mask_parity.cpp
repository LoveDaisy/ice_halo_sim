// CPU-vs-CUDA parity harness for the raypath-color component mask
// (task-331.6, scrum raypath-color-foundation, sub-task 6/6).
//
// CUDA sibling of test_metal_component_mask_parity.cpp. The CUDA emit gate now
// produces the SAME per-ray uint64 component mask the CPU backend produces
// (per-summand Complex match → component bit via the uploaded component table),
// OR-accumulates it across MS layers on device (d_root_component_ /
// d_cont_component_ siblings of the wl_idx carrier), carries it through the
// transit kernel + the continuation shuffle gather + the Recombine handle swap
// (LANDMINE), and the host reads it back via ReadbackComponentCapture.
//
// Why the parity is STRUCTURAL + STATISTICAL, not per-ray byte-exact
//   CudaTraceBackend follows a single refract-priority path per ray while
//   CpuTraceBackend fans out (reflect + refract) per hit and area-weight
//   re-samples continuation entry — the two produce DIFFERENT ray populations.
//   So per-ray mask equality is impossible; we compare weighted distributions +
//   self-consistency invariants (mirrors the Metal harness rationale):
//     - structural: no spurious bits, cross-layer joint bits present (proves the
//       mask survives transit + shuffle + the Recombine handle swap end-to-end);
//     - energy: captured weight > 0;
//     - cross-seed self-consistency (CUDA-only);
//     - per-component marginal ballpark vs CPU (loose — populations diverge);
//     - LANDMINE guard: per-mask-VALUE (joint) histogram is shuffle-invariant
//       under correct carry (a forgotten swap/gather of the mask decorrelates it
//       from its ray and perturbs the joint distribution while leaving marginals
//       intact — exactly the CPU T3/T4 bug, reproduced on device).
//
// Build gate: `#if defined(LUMICE_CUDA_ENABLED)` — the TU is empty on non-CUDA
// hosts. At runtime, each test SKIPs cleanly if no CUDA device is enumerated
// (dev49 host-only runs), matching test_cuda_rich_exit.cpp.

#include <gtest/gtest.h>

#if defined(LUMICE_CUDA_ENABLED)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "config/component_table.hpp"
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/def.hpp"
#include "core/exit_seam.hpp"
#include "cuda_test_helpers.hpp"

namespace lumice {
namespace {

using cuda_test::MakeFullViewRender;

// A 2-layer prism scene whose per-layer filter is a Complex OR of two
// length-gated EntryExit summands (both entry/exit wildcard → length-only
// match). Different exit-path lengths match different summands, so the produced
// component masks vary; the cross-layer OR then joins L0's bits {0,1} with L1's
// bits {2,3}. Random axis so the exit population is rich (and device-gen draws
// differ per seed). Layer 0 continues (prob 0.7) into layer 1 (final, prob 0).
SceneConfig MakeComplexMaskScene(size_t max_hits) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  for (int layer = 0; layer < 2; ++layer) {
    MsInfo ms;
    ms.prob_ = (layer == 0) ? 0.7f : 0.0f;  // final layer keeps 0.0
    ScatteringSetting s;
    s.filter_ = FilterConfig{};
    s.crystal_.id_ = 0;
    PrismCrystalParam prism;
    prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    for (auto& d : prism.d_) {
      d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    }
    s.crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    s.crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    s.crystal_.param_ = prism;
    s.crystal_proportion_ = 1.0f;

    FilterConfig& fc = s.filter_;
    fc.id_ = 0;
    fc.symmetry_ = FilterConfig::kSymNone;
    fc.action_ = FilterConfig::kFilterIn;
    ComplexFilterParam cx;
    // summand 0: path length >= 2 (almost every exit); summand 1: length >= 4.
    EntryExitFilterParam ee0;
    ee0.entry_ = std::nullopt;
    ee0.exit_ = std::nullopt;
    ee0.min_len_ = 2;
    EntryExitFilterParam ee1;
    ee1.entry_ = std::nullopt;
    ee1.exit_ = std::nullopt;
    ee1.min_len_ = 4;
    cx.filters_.push_back({ { 0, SimpleFilterParam{ ee0 } } });
    cx.filters_.push_back({ { 0, SimpleFilterParam{ ee1 } } });
    fc.param_ = cx;

    ms.setting_.push_back(std::move(s));
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

size_t ComponentBitCount(const SceneConfig& scene) {
  ComponentTable t = BuildComponentTable(scene);
  size_t n = 0;
  for (const auto& e : t.entries_) {
    if (e.bit_ != ComponentTable::kNoBit) {
      n++;
    }
  }
  return n;
}

struct Capture {
  std::vector<uint64_t> masks;
  std::vector<float> weights;
  double total_w = 0.0;
};

// Per-component (bit) weighted fraction of the total captured weight.
std::vector<double> ComponentFractions(const Capture& c, size_t n_bits) {
  std::vector<double> frac(n_bits, 0.0);
  if (c.total_w <= 0.0) {
    return frac;
  }
  for (size_t i = 0; i < c.masks.size(); i++) {
    for (size_t b = 0; b < n_bits; b++) {
      if ((c.masks[i] & (1ull << b)) != 0ull) {
        frac[b] += static_cast<double>(c.weights[i]);
      }
    }
  }
  for (double& f : frac) {
    f /= c.total_w;
  }
  return frac;
}

// Per-mask-VALUE weighted fraction (the JOINT distribution — sensitive to the
// cross-layer decorrelation landmine that marginals miss).
std::map<uint64_t, double> MaskValueFractions(const Capture& c) {
  std::map<uint64_t, double> h;
  if (c.total_w <= 0.0) {
    return h;
  }
  for (size_t i = 0; i < c.masks.size(); i++) {
    h[c.masks[i]] += static_cast<double>(c.weights[i]);
  }
  for (auto& kv : h) {
    kv.second /= c.total_w;
  }
  return h;
}

// L1 distance between two mask-value distributions (sum of |a-b| over the union
// of keys). 0 = identical; 2 = disjoint support.
double MaskValueL1(const std::map<uint64_t, double>& a, const std::map<uint64_t, double>& b) {
  double d = 0.0;
  for (const auto& kv : a) {
    auto it = b.find(kv.first);
    d += std::abs(kv.second - (it == b.end() ? 0.0 : it->second));
  }
  for (const auto& kv : b) {
    if (a.find(kv.first) == a.end()) {
      d += std::abs(kv.second);
    }
  }
  return d;
}

Capture RunCuda(const SceneConfig& scene, const RenderConfig& render, uint32_t seed, size_t ray_count, bool shuffle) {
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;

  HostRayBatch host;
  host.count = ray_count;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  Capture cap;
  CudaTraceBackend cuda;
  cuda.SetCaptureComponent(true);
  cuda.BeginSession(spec);
  RootRaySource roots = RootRaySource::FromHost(host);
  for (size_t mi = 0; mi < scene.ms_.size(); mi++) {
    LayerHandlePtr h = cuda.TraceLayer(roots);
    bool last = (mi + 1 == scene.ms_.size());
    if (last) {
      break;
    }
    RecombineSpec rspec;
    rspec.shuffle = shuffle;
    roots = cuda.Recombine(std::move(h), rspec);
  }
  cuda.ReadbackComponentCapture(cap.masks, cap.weights);
  cuda.EndSession();
  for (float w : cap.weights) {
    cap.total_w += static_cast<double>(w);
  }
  return cap;
}

Capture RunCpu(const SceneConfig& scene, const RenderConfig& render, uint32_t seed, size_t ray_count) {
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;

  HostRayBatch host;
  host.count = ray_count;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  Capture cap;
  CpuTraceBackend cpu;
  cpu.BeginSession(spec);
  RootRaySource roots = RootRaySource::FromHost(host);
  for (size_t mi = 0; mi < scene.ms_.size(); mi++) {
    LayerHandlePtr h = cpu.TraceLayer(roots);
    std::vector<ExitRayRecord> le;
    cpu.DrainExits(le);
    for (const auto& r : le) {
      cap.masks.push_back(r.component_mask);
      cap.weights.push_back(r.weight);
    }
    bool last = (mi + 1 == scene.ms_.size());
    if (last) {
      break;
    }
    RecombineSpec rspec;
    rspec.shuffle = true;
    roots = cpu.Recombine(std::move(h), rspec);
  }
  cpu.EndSession();
  for (float w : cap.weights) {
    cap.total_w += static_cast<double>(w);
  }
  return cap;
}

// True iff any captured mask carries a bit from BOTH the low (layer-0) set and
// the high (layer-1) set — the cross-layer OR joint (mask survived the carry).
bool HasCrossLayerJoint(const Capture& c, uint64_t low_bits, uint64_t high_bits) {
  for (uint64_t m : c.masks) {
    if ((m & low_bits) != 0ull && (m & high_bits) != 0ull) {
      return true;
    }
  }
  return false;
}

bool ShouldSkipCudaTests() {
  return !CudaDeviceAvailable();
}

constexpr size_t kRayCount = 8192;
constexpr size_t kMaxHits = 6;

TEST(CudaComponentMaskParity, StructuralAndCrossLayerCarry) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  size_t n_bits = ComponentBitCount(scene);
  ASSERT_EQ(n_bits, 4u) << "expected 2 summands × 2 layers = 4 component bits";
  const uint64_t low_bits = 0b0011ull;   // layer-0 summands
  const uint64_t high_bits = 0b1100ull;  // layer-1 summands

  Capture cuda = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true);

  // Non-degenerate: rays were emitted and carry non-zero masks.
  ASSERT_FALSE(cuda.masks.empty()) << "CUDA captured no emitted rays";
  EXPECT_GT(cuda.total_w, 0.0);
  size_t nonzero = 0;
  uint64_t all_bits_or = 0ull;
  for (uint64_t m : cuda.masks) {
    if (m != 0ull) {
      nonzero++;
    }
    all_bits_or |= m;
    // No spurious bits above the component budget.
    EXPECT_EQ(m >> n_bits, 0ull) << "captured mask has a bit outside the component table";
  }
  EXPECT_GT(nonzero, 0u) << "every captured mask was 0 — gate produced no component bits";

  // Cross-layer carry survived transit + shuffle + Recombine handle swap.
  EXPECT_TRUE(HasCrossLayerJoint(cuda, low_bits, high_bits))
      << "no captured mask has both a layer-0 and a layer-1 bit — cross-layer "
         "carry is broken (mask not propagated across the MS boundary)";

  std::vector<double> mf = ComponentFractions(cuda, n_bits);
  fprintf(stderr, "[component-mask] CUDA captured=%zu totalW=%.3f bitsOR=0x%llx\n", cuda.masks.size(), cuda.total_w,
          static_cast<unsigned long long>(all_bits_or));
  fprintf(stderr, "[component-mask] CUDA per-component frac: b0=%.4f b1=%.4f b2=%.4f b3=%.4f\n", mf[0], mf[1], mf[2],
          mf[3]);
}

TEST(CudaComponentMaskParity, CrossSeedSelfConsistency) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  size_t n_bits = ComponentBitCount(scene);

  const uint32_t seeds[3] = { 7, 101, 9001 };
  std::vector<std::vector<double>> fracs;
  for (uint32_t s : seeds) {
    Capture c = RunCuda(scene, render, s, kRayCount, /*shuffle=*/true);
    ASSERT_FALSE(c.masks.empty());
    fracs.push_back(ComponentFractions(c, n_bits));
  }
  double max_drift = 0.0;
  for (size_t b = 0; b < n_bits; b++) {
    double lo = fracs[0][b], hi = fracs[0][b];
    for (const auto& f : fracs) {
      lo = std::min(lo, f[b]);
      hi = std::max(hi, f[b]);
    }
    max_drift = std::max(max_drift, hi - lo);
    fprintf(stderr, "[component-mask] cross-seed b%zu: %.4f %.4f %.4f\n", b, fracs[0][b], fracs[1][b], fracs[2][b]);
  }
  fprintf(stderr, "[component-mask] cross-seed max per-component drift = %.4f\n", max_drift);
  // Per-component fractions are Monte-Carlo estimates of the same underlying
  // distribution — stable across seeds (loose bound for the 8192-ray sample).
  EXPECT_LT(max_drift, 0.10) << "per-component fraction drifts too much across seeds";
}

TEST(CudaComponentMaskParity, ShuffleInvariantJointDistribution_LandmineGuard) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();

  // Under CORRECT carry the mask travels with its ray through the shuffle gather
  // AND the Recombine handle swap, so shuffle on/off give statistically-equal
  // JOINT (per-mask-value) distributions. A forgotten swap/gather of the mask
  // decorrelates it from its ray only when shuffle is on → the joint histogram
  // shifts while marginals stay put (the CPU T3/T4 bug, on device).
  Capture on = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true);
  Capture off = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/false);
  ASSERT_FALSE(on.masks.empty());
  ASSERT_FALSE(off.masks.empty());

  auto h_on = MaskValueFractions(on);
  auto h_off = MaskValueFractions(off);
  double l1 = MaskValueL1(h_on, h_off);
  fprintf(stderr, "[component-mask] shuffle on/off joint mask-value distributions:\n");
  for (const auto& kv : h_on) {
    auto it = h_off.find(kv.first);
    fprintf(stderr, "   mask 0x%llx : on=%.4f off=%.4f\n", static_cast<unsigned long long>(kv.first), kv.second,
            it == h_off.end() ? 0.0 : it->second);
  }
  fprintf(stderr, "[component-mask] shuffle on/off joint L1 distance = %.4f\n", l1);
  // Statistical equality (both correctly pair mask with ray; orientation is iid
  // either way). Loose bound absorbs Monte-Carlo noise; a decorrelation bug
  // moves the joint far more than this (Metal saw 55x separation when the swap
  // was removed).
  EXPECT_LT(l1, 0.20) << "joint mask-value distribution changed with shuffle — "
                         "cross-layer mask decorrelation (LANDMINE: mask not carried "
                         "through the shuffle gather / Recombine swap)";
}

TEST(CudaComponentMaskParity, CpuMarginalBallpark) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  size_t n_bits = ComponentBitCount(scene);

  Capture cuda = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true);
  Capture cpu = RunCpu(scene, render, /*seed=*/7, kRayCount);
  ASSERT_FALSE(cuda.masks.empty());
  ASSERT_FALSE(cpu.masks.empty());

  std::vector<double> mf = ComponentFractions(cuda, n_bits);
  std::vector<double> cf = ComponentFractions(cpu, n_bits);
  fprintf(stderr, "[component-mask] CPU captured=%zu totalW=%.3f\n", cpu.masks.size(), cpu.total_w);
  double max_abs = 0.0;
  for (size_t b = 0; b < n_bits; b++) {
    double d = std::abs(mf[b] - cf[b]);
    max_abs = std::max(max_abs, d);
    fprintf(stderr, "[component-mask] per-component b%zu: cuda=%.4f cpu=%.4f |d|=%.4f\n", b, mf[b], cf[b], d);
  }
  fprintf(stderr, "[component-mask] CPU-vs-CUDA max |per-component frac| = %.4f\n", max_abs);
  // Same filter-match + component-table logic on both backends → the per-
  // component marginals live in the same ballpark. LOOSE bound: the two
  // backends trace DIFFERENT ray populations (single-path vs fan-out), so this
  // is a sanity anchor, not a tight numeric parity. Final human-eye/analytic
  // parity is the orchestrator's step.
  EXPECT_LT(max_abs, 0.35) << "CPU and CUDA per-component marginals diverge beyond the ballpark bound";
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
