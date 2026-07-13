// CPU-vs-CUDA parity harness for the raypath-color per-ray mask
// (task-331.6 foundation → task-358.2 Design-2 color pass → task-358.3
// Fork-C retirement + landmine tests migrated to the color path).
//
// CUDA sibling of test_metal_component_mask_parity.cpp. The CUDA emit gate
// produces a per-ray uint64 mask whose bits are the Design-2 raypath_color
// bits (`ColorGateTable` / `raypath_color`), OR-accumulates it across MS
// layers on device (d_root_component_ / d_cont_component_ siblings of the
// wl_idx carrier), carries it through the transit kernel + the continuation
// shuffle gather + the Recombine handle swap (LANDMINE), and the host reads
// it back via ReadbackRayMask. The pre-Design-2 Fork-C `ComponentTable`
// produce branch has been retired (task-358.3).
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
//     - color-configured cross-seed self-consistency (CUDA-only);
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

#include "config/color_class_table.hpp"  // task-358.2 Design-2 color-class + ColorClassCombine
#include "config/color_gate_table.hpp"   // task-358.2 Design-2 color-bit source
#include "config/component_table.hpp"
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"  // task-358.2 raypath_color session config
#include "config/render_config.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/def.hpp"
#include "core/exit_seam.hpp"
#include "cuda_test_helpers.hpp"
#include "util/bit_utils.hpp"  // task-358.2 PopCount for popcount whitebox test

namespace lumice {
namespace {

using cuda_test::MakeFullViewRender;

// task-358.3: `MakeComplexMaskScene` + `ComponentBitCount` (Fork-C physical
// bit-map scaffolds) retired alongside the Fork-C GPU produce path. All tests
// below source their bits from the Design-2 `MakeColorConfiguredScene` +
// `MakeColorConfiguredColorConfig` + `ColorGateBitCount` fixtures further down.

// task-358.2 (cuda-color-parity) fixture — bit-for-bit STRUCTURAL mirror of
// Metal `MakeColorConfiguredScene` (test_metal_component_mask_parity.cpp).
// Physical filter set to `NoneFilterParam` on both layers — after task-358.3
// this means every mask bit comes exclusively from the Design-2 color pass,
// letting per-color-bit marginals be directly compared to CPU.
// Critical detail: `crystal_.id_ = static_cast<IdType>(mi)` so BuildColorGate
// Table can key each layer's placement by a distinct crystal_id (the 4-ref
// color config below binds bit0/bit1 to crystal=0 and bit2/bit3 to crystal=1).
SceneConfig MakeColorConfiguredScene(size_t max_hits) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  for (int layer = 0; layer < 2; ++layer) {
    MsInfo ms;
    // Layer 0 continues (prob 0.7) into layer 1 (final, prob 0.0). Mirrors Metal.
    ms.prob_ = (layer == 0) ? 0.7f : 0.0f;
    ScatteringSetting s;
    s.filter_ = FilterConfig{};
    s.crystal_.id_ = static_cast<IdType>(layer);  // distinct id per layer, mirroring MakeMetalScene
    PrismCrystalParam prism;
    prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    for (auto& d : prism.d_) {
      d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    }
    s.crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    s.crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
    s.crystal_.param_ = prism;
    s.crystal_proportion_ = 1.0f;

    // task-358.2: physical filter = None. Fork-C ComponentTable will therefore
    // build 0 bits, so the captured mask carries only Design-2 color bits.
    FilterConfig& fc = s.filter_;
    fc.id_ = 0;
    fc.symmetry_ = FilterConfig::kSymNone;
    fc.action_ = FilterConfig::kFilterIn;
    fc.param_ = SimpleFilterParam{ NoneFilterParam{} };

    ms.setting_.push_back(std::move(s));
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

// task-358.2 fixture — bit-for-bit STRUCTURAL mirror of Metal
// `MakeColorConfiguredColorConfig` (test_metal_component_mask_parity.cpp:202-
// 258). Four color refs across two layers, each with `symmetry_ = kSymNone`
// so BuildColorGateTable assigns bits in insertion order:
//   bit 0: layer 0, crystal 0, NoneFilterParam (whole-crystal — AC5 signal)
//   bit 1: layer 0, crystal 0, EntryExit(min_len=2)
//   bit 2: layer 1, crystal 1, NoneFilterParam (whole-crystal — AC5 signal)
//   bit 3: layer 1, crystal 1, EntryExit(min_len=3)
// One class combine="any" so its member_bits_ = 0b1111 → predicate over
// this_mask fires whenever ANY of the 4 bits are set (matches the metal
// mirror). This layout is asserted by NoneWholeCrystalBits_AC5 and the
// popcount whitebox test.
std::shared_ptr<RaypathColorConfig> MakeColorConfiguredColorConfig() {
  auto cfg = std::make_shared<RaypathColorConfig>();
  cfg->mode_ = "dominant";
  ColorClassConfig cls;
  cls.color_[0] = 1.0f;
  cls.color_[1] = 1.0f;
  cls.color_[2] = 1.0f;
  cls.combine_ = "any";
  cls.visible_ = true;
  cls.solo_ = false;
  // bit 0
  {
    RaypathColorRef r;
    r.layer_ = 0;
    r.crystal_ = 0;
    r.predicate_ = NoneFilterParam{};
    r.symmetry_ = FilterConfig::kSymNone;
    cls.match_.push_back(r);
  }
  // bit 1
  {
    RaypathColorRef r;
    r.layer_ = 0;
    r.crystal_ = 0;
    EntryExitFilterParam ee;
    ee.entry_ = std::nullopt;
    ee.exit_ = std::nullopt;
    ee.min_len_ = 2;
    r.predicate_ = ee;
    r.symmetry_ = FilterConfig::kSymNone;
    cls.match_.push_back(r);
  }
  // bit 2
  {
    RaypathColorRef r;
    r.layer_ = 1;
    r.crystal_ = 1;
    r.predicate_ = NoneFilterParam{};
    r.symmetry_ = FilterConfig::kSymNone;
    cls.match_.push_back(r);
  }
  // bit 3
  {
    RaypathColorRef r;
    r.layer_ = 1;
    r.crystal_ = 1;
    EntryExitFilterParam ee;
    ee.entry_ = std::nullopt;
    ee.exit_ = std::nullopt;
    ee.min_len_ = 3;
    r.predicate_ = ee;
    r.symmetry_ = FilterConfig::kSymNone;
    cls.match_.push_back(r);
  }
  cfg->classes_.push_back(std::move(cls));
  return cfg;
}

// Number of color-gate bits for a (scene, color_cfg) pair. Mirrors
// ComponentBitCount but sources bits from ColorGateTable (Design-2) rather
// than Fork-C's ComponentTable. Used by the AC5 / popcount tests to pin the
// bit layout before relying on it (fail loudly if BuildColorGateTable's
// insertion order changes; mirrors Metal test:263-272).
size_t ColorGateBitCount(const SceneConfig& scene, const RaypathColorConfig& color_cfg) {
  ColorGateTable t = BuildColorGateTable(color_cfg, scene);
  size_t n = 0;
  for (const auto& e : t.entries_) {
    if (e.bit_ != ColorGateEntry{}.bit_ && e.bit_ != ComponentTable::kNoBit) {
      n = std::max(n, static_cast<size_t>(e.bit_) + 1u);
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

Capture RunCuda(const SceneConfig& scene, const RenderConfig& render, uint32_t seed, size_t ray_count, bool shuffle,
                std::shared_ptr<const RaypathColorConfig> color_cfg = nullptr) {
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;
  spec.raypath_color = color_cfg;

  HostRayBatch host;
  host.count = ray_count;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  Capture cap;
  CudaTraceBackend cuda;
  cuda.SetCaptureRayMask(true);
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
  cuda.ReadbackRayMask(cap.masks, cap.weights);
  cuda.EndSession();
  for (float w : cap.weights) {
    cap.total_w += static_cast<double>(w);
  }
  return cap;
}

Capture RunCpu(const SceneConfig& scene, const RenderConfig& render, uint32_t seed, size_t ray_count,
               std::shared_ptr<const RaypathColorConfig> color_cfg = nullptr) {
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;
  spec.raypath_color = color_cfg;

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

  // task-358.3: migrated to the Design-2 color-configured scene. The bit
  // layout coincidentally matches the retired Fork-C scene (2 refs per layer
  // × 2 layers = 4 bits with `low_bits=0b0011`/`high_bits=0b1100`).
  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  size_t n_bits = ColorGateBitCount(scene, *color_cfg);
  ASSERT_EQ(n_bits, 4u) << "expected 2 refs × 2 layers = 4 color bits";
  const uint64_t low_bits = 0b0011ull;   // layer-0 refs
  const uint64_t high_bits = 0b1100ull;  // layer-1 refs

  Capture cuda = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);

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

// task-358.3: retired `CrossSeedSelfConsistency` (Fork-C physical-bit scene
// version). Cross-seed self-consistency coverage on the color path is provided
// by `ColorConfiguredCrossSeedSelfConsistency` (task-358.1/358.2 landing).

TEST(CudaComponentMaskParity, ShuffleInvariantJointDistribution_LandmineGuard) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  // task-358.3: migrated to the Design-2 color-configured scene.
  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  auto color_cfg = MakeColorConfiguredColorConfig();

  // Under CORRECT carry the mask travels with its ray through the shuffle gather
  // AND the Recombine handle swap, so shuffle on/off give statistically-equal
  // JOINT (per-mask-value) distributions. A forgotten swap/gather of the mask
  // decorrelates it from its ray only when shuffle is on → the joint histogram
  // shifts while marginals stay put (the CPU T3/T4 bug, on device).
  Capture on = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  Capture off = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/false, color_cfg);
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
  // task-358.2 Step 5 (AC1): un-skipped from the Design-2-redirect skip
  // (scrum-engine-redirect-design2). CudaTraceBackend now runs the Design-2
  // color pass (Step 1 host upload + Step 2 device emit gate) alongside the
  // CPU path, so per-color-bit marginals are directly comparable. Uses
  // MakeColorConfiguredScene (physical filter = None → after task-358.3 all
  // mask bits come from raypath_color). Mirrors Metal CpuMarginalBallpark.
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  size_t n_bits = ColorGateBitCount(scene, *color_cfg);
  ASSERT_EQ(n_bits, 4u) << "expected 4 color bits (2 refs × 2 layers)";

  Capture cuda = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  Capture cpu = RunCpu(scene, render, /*seed=*/7, kRayCount, color_cfg);
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
  // Same predicate-match + component-table logic on both backends → the per-
  // component marginals live in the same ballpark. LOOSE bound: the two
  // backends trace DIFFERENT ray populations (single-path vs fan-out), so this
  // is a sanity anchor, not a tight numeric parity. Whole-crystal (None) bits
  // 0 and 2 are the tightest signal — they should be near 1.0 on both backends
  // (every emitted ray hit layer 0 / layer 1 respectively), while length-gated
  // bits 1 / 3 vary with the path-length distribution.
  //
  // AC2 environmental note: no single machine has both Metal + CUDA, so the
  // "Metal与CUDA互 parity" AC is satisfied indirectly (transitively via CPU) —
  // this test drives CUDA's per-bit marginals to the same CPU baseline that
  // 358.1's Metal CpuMarginalBallpark drives Metal to, within the same 0.35
  // loose band. This is an equivalence-through-CPU, not a same-process
  // Metal-vs-CUDA diff (see plan §2 default assumption).
  EXPECT_LT(max_abs, 0.35) << "CPU and CUDA per-component marginals diverge beyond the ballpark bound";
}

// task-358.2 Step 5 (AC5): whole-crystal None predicate ref must yield a bit
// set on every ray that traversed the referenced (layer, crystal). Combined
// with Step 2's cross-layer OR carry, every final-layer emit should carry
// BOTH bit 0 (layer 0 whole-crystal) AND bit 2 (layer 1 whole-crystal) — a
// device-side sanity check that None routes through the color pass with the
// integer-crystal contract preserved (plan §3 decision D3 + Step 3).
// Mirrors Metal NoneWholeCrystalBits_AC5 (test_metal:527-586).
TEST(CudaComponentMaskParity, NoneWholeCrystalBits_AC5) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  // Pin the bit layout this test hardcodes (bit0/bit2 = whole-crystal refs)
  // before relying on it — if BuildColorGateTable's insertion order ever
  // changes, fail loudly here instead of silently asserting on the wrong bits.
  ASSERT_EQ(ColorGateBitCount(scene, *color_cfg), 4u)
      << "expected 4 color bits (2 refs × 2 layers); bit0/bit2 whole-crystal assumption below depends on this "
         "exact layout";

  Capture cuda = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  ASSERT_FALSE(cuda.masks.empty());

  // Capture ring interleaves mid-layer emits (from layer 0's do_continue=false
  // path → carry only layer-0 bits) with final-layer emits (from layer 1,
  // carrying layer-0 bits OR'd into layer-1 bits). Separate them by presence
  // of any layer-1 bit (bits 2 / 3):
  const uint64_t layer1_bits = 0b1100ull;
  const uint64_t bit0 = 1ull << 0;  // layer-0 whole-crystal
  const uint64_t bit2 = 1ull << 2;  // layer-1 whole-crystal
  size_t final_layer = 0;
  size_t final_layer_with_bit0_and_bit2 = 0;
  size_t mid_layer = 0;
  size_t mid_layer_with_bit0 = 0;
  for (uint64_t m : cuda.masks) {
    if ((m & layer1_bits) != 0ull) {
      final_layer++;
      if ((m & bit0) != 0ull && (m & bit2) != 0ull) {
        final_layer_with_bit0_and_bit2++;
      }
    } else {
      mid_layer++;
      if ((m & bit0) != 0ull) {
        mid_layer_with_bit0++;
      }
    }
  }
  fprintf(stderr,
          "[component-mask AC5] final-layer emits=%zu (with bit0+bit2=%zu), "
          "mid-layer emits=%zu (with bit0=%zu)\n",
          final_layer, final_layer_with_bit0_and_bit2, mid_layer, mid_layer_with_bit0);

  ASSERT_GT(mid_layer, 0u) << "no mid-layer emits captured — scene mis-configured";
  ASSERT_GT(final_layer, 0u) << "no final-layer emits captured — cross-layer path broken";
  EXPECT_EQ(final_layer_with_bit0_and_bit2, final_layer)
      << "final-layer emit missing whole-crystal bit0 (layer 0) and/or bit2 (layer 1) — "
         "None-predicate color pass or cross-layer carry broken (AC5)";
  EXPECT_EQ(mid_layer_with_bit0, mid_layer)
      << "mid-layer emit missing whole-crystal bit0 (layer 0) — layer-0 None-predicate "
         "color pass broken (AC5)";
}

// task-358.2 Step 5 (AC2): cross-seed self-consistency for the color-configured
// scene. Mirrors CrossSeedSelfConsistency but with color_cfg fed to CUDA.
TEST(CudaComponentMaskParity, ColorConfiguredCrossSeedSelfConsistency) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  size_t n_bits = ColorGateBitCount(scene, *color_cfg);

  const uint32_t seeds[3] = { 7, 101, 9001 };
  std::vector<std::vector<double>> fracs;
  for (uint32_t s : seeds) {
    Capture c = RunCuda(scene, render, s, kRayCount, /*shuffle=*/true, color_cfg);
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
    fprintf(stderr, "[color-configured cross-seed] b%zu: %.4f %.4f %.4f\n", b, fracs[0][b], fracs[1][b], fracs[2][b]);
  }
  fprintf(stderr, "[color-configured cross-seed] max per-bit drift = %.4f\n", max_drift);
  EXPECT_LT(max_drift, 0.10) << "per-bit fraction drifts too much across seeds for color-configured scene";
}

// task-358.2 Step 5 (AC2): white-box popcount check — every captured mask must
// have popcount within the expected range for its layer (mid-layer emits: 1-2
// bits; final-layer emits: 2-4 bits after cross-layer OR). Guards against
// spurious bits or dropped bits that a marginal check would smooth over.
// Mirrors Metal ColorConfiguredMaskPopcountWhitebox (test_metal:627-677).
TEST(CudaComponentMaskParity, ColorConfiguredMaskPopcountWhitebox) {
  if (ShouldSkipCudaTests()) {
    GTEST_SKIP() << "no CUDA device enumerated";
  }

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeFullViewRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  // Pin the bit layout this test hardcodes (4-bit budget, layer1_bits mask
  // below) before relying on it — same rationale as NoneWholeCrystalBits_AC5.
  ASSERT_EQ(ColorGateBitCount(scene, *color_cfg), 4u)
      << "expected 4 color bits (2 refs × 2 layers); layer1_bits/popcount-range assumptions below depend on this "
         "exact layout";

  Capture cuda = RunCuda(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  ASSERT_FALSE(cuda.masks.empty());

  const uint64_t layer1_bits = 0b1100ull;
  size_t bad_mid = 0;
  size_t bad_final = 0;
  // Popcount = number of set bits. Uses lumice::PopCount (SWAR) for cross-
  // compiler portability — __builtin_popcountll is a GCC/Clang extension that
  // MSVC rejects (no-msvc-unsafe-builtin policy).
  for (uint64_t m : cuda.masks) {
    int pc = PopCount(m);
    if ((m >> 4) != 0ull) {
      bad_mid++;
      bad_final++;
      continue;
    }
    if ((m & layer1_bits) != 0ull) {
      // Final-layer emit: bit0 + bit2 mandatory (AC5), bit1 / bit3 optional.
      if (pc < 2 || pc > 4) {
        bad_final++;
      }
    } else {
      // Mid-layer emit: bit0 mandatory (AC5), bit1 optional. bits 2/3 absent.
      if (pc < 1 || pc > 2) {
        bad_mid++;
      }
    }
  }
  fprintf(stderr, "[color-configured popcount] bad_mid=%zu bad_final=%zu (of %zu total)\n", bad_mid, bad_final,
          cuda.masks.size());
  EXPECT_EQ(bad_mid, 0u) << "mid-layer emit popcount out of range — color pass produced spurious/missing bits";
  EXPECT_EQ(bad_final, 0u) << "final-layer emit popcount out of range — cross-layer OR carry or color pass broken";
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
