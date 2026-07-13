// CPU-vs-Metal parity harness for the raypath-color component mask
// (task-331.5, scrum raypath-color-foundation, sub-task 5/6).
//
// What this validates
//   The Metal emit gate now produces the SAME per-ray uint64 component mask the
//   CPU backend produces (per-summand Complex match → component bit via the
//   uploaded component table), OR-accumulates it across MS layers on device
//   (root_component / cont_component siblings of the wl_idx carrier), and the
//   host reads it back via ReadbackComponentCapture.
//
// Why the parity is STRUCTURAL + STATISTICAL, not per-ray byte-exact
//   MetalTraceBackend follows a single refract-priority path per ray while
//   CpuTraceBackend fans out (reflect + refract) per hit and area-weight
//   re-samples continuation entry — the two produce DIFFERENT ray populations
//   (see test_metal_trace_parity.cpp header). So per-ray mask equality is
//   impossible; we compare weighted distributions + self-consistency invariants:
//     - structural: no spurious bits, cross-layer joint bits present (proves the
//       mask survives transit + shuffle + the Recombine handle swap end-to-end);
//     - energy: captured weight > 0 and stable;
//     - cross-seed self-consistency (Metal-only);
//     - per-component marginal ballpark vs CPU (loose — populations diverge);
//     - LANDMINE guard: per-mask-VALUE (joint) histogram is shuffle-invariant
//       under correct carry (a forgotten swap decorrelates the mask from its ray
//       and perturbs the joint distribution while leaving marginals intact —
//       exactly the CPU T3/T4 bug, reproduced on device).
//
// In LUMICE_SKIP_METAL_TESTS=1 environments each test SKIPs cleanly.

#include <gtest/gtest.h>

#if defined(__APPLE__)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "config/color_gate_table.hpp"
#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "config/render_config.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/metal_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/def.hpp"
#include "core/exit_seam.hpp"
#include "metal_test_helpers.hpp"
#include "util/bit_utils.hpp"

namespace lumice {
namespace {

using metal_test::ForceHostGenForByteIdentity;
using metal_test::MakeMetalScene;
using metal_test::MakeRectangularRender;
using metal_test::ShouldSkipMetalTests;

// A 2-layer scene whose per-layer filter is a Complex OR of two length-gated
// EntryExit summands (both entry/exit wildcard → length-only match). Different
// exit-path lengths match different summands, so the produced component masks
// vary; the cross-layer OR then joins L0's bits {0,1} with L1's bits {2,3}.
SceneConfig MakeComplexMaskScene(size_t max_hits) {
  SceneConfig scene = MakeMetalScene(max_hits, /*ms_layers=*/2);
  for (auto& ms : scene.ms_) {
    // Non-final continuation prob so rays reach layer 1 (final layer keeps 0.0).
    if (&ms != &scene.ms_.back()) {
      ms.prob_ = 0.7f;
    }
    FilterConfig& fc = ms.setting_[0].filter_;
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

// task-358.1 (Step 5 fixture): a 2-layer scene whose physical filter is a bare
// NoneFilterParam (Fork-C `BuildComponentTable` produces bits, but on the
// device side kDeviceFilterTypeNone routes through the top-level `type=None`
// branch in `DeviceFilterSummandMask` which fixed-returns mask=0 — the
// physical Fork-C contribution to `this_mask` is thus 0 for every ray). All
// mask bits come from the raypath_color pass alone, so CPU (Design-2 color
// pass) and Metal (task-358.1 Step 2 color pass) can be compared bit-for-bit.
//
// Layout of color-gate bits (bit ordering follows BuildColorGateTable insertion
// order, which walks color_cfg.classes_[].match_[] in declaration order):
//   bit 0 : layer 0 crystal 0, predicate = NoneFilterParam (whole-crystal)
//   bit 1 : layer 0 crystal 0, predicate = EntryExit min_len=2
//   bit 2 : layer 1 crystal 1, predicate = NoneFilterParam (whole-crystal, AC5)
//   bit 3 : layer 1 crystal 1, predicate = EntryExit min_len=3
//
// AC5 (whole-crystal None ref) is verified via bit 0 (always set on any ray
// that traversed layer 0 = every mid-exit + cross-layer emit) and bit 2 (always
// set on any ray emitted from layer 1). Bits 1 / 3 vary with path length.
//
// Cross-layer joint is exercised because final-layer emits carry
// `carried_component` from layer 0 (bit 0 always set, bit 1 iff len≥2 at
// layer 0), OR'd with layer-1 color bits.
SceneConfig MakeColorConfiguredScene(size_t max_hits) {
  SceneConfig scene = MakeMetalScene(max_hits, /*ms_layers=*/2);
  for (auto& ms : scene.ms_) {
    if (&ms != &scene.ms_.back()) {
      ms.prob_ = 0.7f;
    }
    FilterConfig& fc = ms.setting_[0].filter_;
    fc.id_ = 0;
    fc.symmetry_ = FilterConfig::kSymNone;
    fc.action_ = FilterConfig::kFilterIn;
    fc.param_ = SimpleFilterParam{ NoneFilterParam{} };
  }
  return scene;
}

// Build a raypath_color config matching MakeColorConfiguredScene's layout.
// One class (combine=any, single lane) with four refs — see the bit ordering
// comment on MakeColorConfiguredScene.
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
  // bit 0: layer 0 crystal 0, whole-crystal (AC5).
  {
    RaypathColorRef r;
    r.layer_ = 0;
    r.crystal_ = 0;
    r.predicate_ = NoneFilterParam{};
    r.symmetry_ = FilterConfig::kSymNone;
    cls.match_.push_back(r);
  }
  // bit 1: layer 0 crystal 0, length>=2.
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
  // bit 2: layer 1 crystal 1, whole-crystal (AC5).
  {
    RaypathColorRef r;
    r.layer_ = 1;
    r.crystal_ = 1;
    r.predicate_ = NoneFilterParam{};
    r.symmetry_ = FilterConfig::kSymNone;
    cls.match_.push_back(r);
  }
  // bit 3: layer 1 crystal 1, length>=3.
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
// than Fork-C's ComponentTable.
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

Capture RunMetal(const SceneConfig& scene, const RenderConfig& render, uint32_t seed, size_t ray_count, bool shuffle,
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
  MetalTraceBackend metal;
  metal.SetCaptureComponent(true);
  metal.BeginSession(spec);
  RootRaySource roots = RootRaySource::FromHost(host);
  for (size_t mi = 0; mi < scene.ms_.size(); mi++) {
    LayerHandlePtr h = metal.TraceLayer(roots);
    bool last = (mi + 1 == scene.ms_.size());
    if (last) {
      break;
    }
    RecombineSpec rspec;
    rspec.shuffle = shuffle;
    roots = metal.Recombine(std::move(h), rspec);
  }
  metal.ReadbackComponentCapture(cap.masks, cap.weights);
  metal.EndSession();
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

constexpr size_t kRayCount = 8192;
constexpr size_t kMaxHits = 6;

TEST(MetalComponentMaskParity, StructuralAndCrossLayerCarry) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();
  size_t n_bits = ComponentBitCount(scene);
  ASSERT_EQ(n_bits, 4u) << "expected 2 summands × 2 layers = 4 component bits";
  const uint64_t low_bits = 0b0011ull;   // layer-0 summands
  const uint64_t high_bits = 0b1100ull;  // layer-1 summands

  Capture metal = RunMetal(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true);

  // Non-degenerate: rays were emitted and carry non-zero masks.
  ASSERT_FALSE(metal.masks.empty()) << "Metal captured no emitted rays";
  EXPECT_GT(metal.total_w, 0.0);
  size_t nonzero = 0;
  uint64_t all_bits_or = 0ull;
  for (uint64_t m : metal.masks) {
    if (m != 0ull) {
      nonzero++;
    }
    all_bits_or |= m;
    // No spurious bits above the component budget.
    EXPECT_EQ(m >> n_bits, 0ull) << "captured mask has a bit outside the component table";
  }
  EXPECT_GT(nonzero, 0u) << "every captured mask was 0 — gate produced no component bits";

  // Cross-layer carry survived transit + shuffle + Recombine handle swap.
  EXPECT_TRUE(HasCrossLayerJoint(metal, low_bits, high_bits))
      << "no captured mask has both a layer-0 and a layer-1 bit — cross-layer "
         "carry is broken (mask not propagated across the MS boundary)";

  std::vector<double> mf = ComponentFractions(metal, n_bits);
  fprintf(stderr, "[component-mask] Metal captured=%zu totalW=%.3f bitsOR=0x%llx\n", metal.masks.size(), metal.total_w,
          static_cast<unsigned long long>(all_bits_or));
  fprintf(stderr, "[component-mask] Metal per-component frac: b0=%.4f b1=%.4f b2=%.4f b3=%.4f\n", mf[0], mf[1], mf[2],
          mf[3]);
}

TEST(MetalComponentMaskParity, CrossSeedSelfConsistency) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();
  size_t n_bits = ComponentBitCount(scene);

  const uint32_t seeds[3] = { 7, 101, 9001 };
  std::vector<std::vector<double>> fracs;
  for (uint32_t s : seeds) {
    Capture c = RunMetal(scene, render, s, kRayCount, /*shuffle=*/true);
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

TEST(MetalComponentMaskParity, ShuffleInvariantJointDistribution_LandmineGuard) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeComplexMaskScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();

  // Under CORRECT carry the mask travels with its ray through the shuffle gather
  // AND the Recombine handle swap, so shuffle on/off give statistically-equal
  // JOINT (per-mask-value) distributions. A forgotten swap/gather of the mask
  // decorrelates it from its ray only when shuffle is on → the joint histogram
  // shifts while marginals stay put (the CPU T3/T4 bug, on device).
  Capture on = RunMetal(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true);
  Capture off = RunMetal(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/false);
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
  // moves the joint far more than this.
  EXPECT_LT(l1, 0.20) << "joint mask-value distribution changed with shuffle — "
                         "cross-layer mask decorrelation (LANDMINE: mask not carried "
                         "through the shuffle gather / Recombine swap)";
}

TEST(MetalComponentMaskParity, CpuMarginalBallpark) {
  // task-358.1 Step 5 (AC1): un-skipped from the Design-2-redirect skip
  // (scrum-engine-redirect-design2). MetalTraceBackend now runs the Design-2
  // color pass (Step 1 host upload + Step 2 MSL emit gate) alongside the CPU
  // path, so per-color-bit marginals are directly comparable. Uses
  // MakeColorConfiguredScene (physical filter = None → Fork-C contribution =
  // 0 on device; all mask bits come from raypath_color).
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  size_t n_bits = ColorGateBitCount(scene, *color_cfg);
  ASSERT_EQ(n_bits, 4u) << "expected 4 color bits (2 refs × 2 layers)";

  Capture metal = RunMetal(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  Capture cpu = RunCpu(scene, render, /*seed=*/7, kRayCount, color_cfg);
  ASSERT_FALSE(metal.masks.empty());
  ASSERT_FALSE(cpu.masks.empty());

  std::vector<double> mf = ComponentFractions(metal, n_bits);
  std::vector<double> cf = ComponentFractions(cpu, n_bits);
  fprintf(stderr, "[component-mask] CPU captured=%zu totalW=%.3f\n", cpu.masks.size(), cpu.total_w);
  double max_abs = 0.0;
  for (size_t b = 0; b < n_bits; b++) {
    double d = std::abs(mf[b] - cf[b]);
    max_abs = std::max(max_abs, d);
    fprintf(stderr, "[component-mask] per-component b%zu: metal=%.4f cpu=%.4f |d|=%.4f\n", b, mf[b], cf[b], d);
  }
  fprintf(stderr, "[component-mask] CPU-vs-Metal max |per-component frac| = %.4f\n", max_abs);
  // Same predicate-match + component-table logic on both backends → the per-
  // component marginals live in the same ballpark. LOOSE bound: the two
  // backends trace DIFFERENT ray populations (single-path vs fan-out), so this
  // is a sanity anchor, not a tight numeric parity. Whole-crystal (None) bits
  // 0 and 2 are the tightest signal — they should be near 1.0 on both backends
  // (every emitted ray hit layer 0 / layer 1 respectively), while length-gated
  // bits 1 / 3 vary with the path-length distribution.
  EXPECT_LT(max_abs, 0.35) << "CPU and Metal per-component marginals diverge beyond the ballpark bound";
}

// task-358.1 Step 3 (AC5): whole-crystal None predicate ref must yield a bit
// set on every ray that traversed the referenced (layer, crystal). Combined
// with Step 2's cross-layer OR carry, every final-layer emit should carry
// BOTH bit 0 (layer 0 whole-crystal) AND bit 2 (layer 1 whole-crystal) — a
// device-side sanity check that None routes through the color pass with the
// integer-crystal contract preserved (plan §3 key design point + §4 Step 3).
TEST(MetalComponentMaskParity, NoneWholeCrystalBits_AC5) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();
  auto color_cfg = MakeColorConfiguredColorConfig();

  Capture metal = RunMetal(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  ASSERT_FALSE(metal.masks.empty());

  // The capture ring interleaves mid-layer emits (from layer 0's do_continue=
  // false path → carry only layer-0 bits) with final-layer emits (from layer
  // 1, carrying layer-0 bits OR'd into layer-1 bits). Separate them by
  // presence of any layer-1 bit (bits 2 / 3):
  const uint64_t layer1_bits = 0b1100ull;
  const uint64_t bit0 = 1ull << 0;  // layer-0 whole-crystal
  const uint64_t bit2 = 1ull << 2;  // layer-1 whole-crystal
  size_t final_layer = 0;
  size_t final_layer_with_bit0_and_bit2 = 0;
  size_t mid_layer = 0;
  size_t mid_layer_with_bit0 = 0;
  for (uint64_t m : metal.masks) {
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
  // Whole-crystal (None) predicate MUST set the bit on every ray that traversed
  // the placement. Any counter-example means the color pass mis-handled None.
  EXPECT_EQ(final_layer_with_bit0_and_bit2, final_layer)
      << "final-layer emit missing whole-crystal bit0 (layer 0) and/or bit2 (layer 1) — "
         "None-predicate color pass or cross-layer carry broken (AC5)";
  EXPECT_EQ(mid_layer_with_bit0, mid_layer)
      << "mid-layer emit missing whole-crystal bit0 (layer 0) — layer-0 None-predicate "
         "color pass broken (AC5)";
}

// task-358.1 Step 5 (AC2): cross-seed self-consistency for the color-configured
// scene. Mirrors CrossSeedSelfConsistency but with color_cfg fed to both CPU
// and Metal.
TEST(MetalComponentMaskParity, ColorConfiguredCrossSeedSelfConsistency) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();
  auto color_cfg = MakeColorConfiguredColorConfig();
  size_t n_bits = ColorGateBitCount(scene, *color_cfg);

  const uint32_t seeds[3] = { 7, 101, 9001 };
  std::vector<std::vector<double>> fracs;
  for (uint32_t s : seeds) {
    Capture c = RunMetal(scene, render, s, kRayCount, /*shuffle=*/true, color_cfg);
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

// task-358.1 Step 5 (AC2): white-box popcount check — every captured mask must
// have popcount within the expected range for its layer (mid-layer emits: 1-2
// bits; final-layer emits: 2-4 bits after cross-layer OR). Guards against
// spurious bits or dropped bits that a marginal check would smooth over.
TEST(MetalComponentMaskParity, ColorConfiguredMaskPopcountWhitebox) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  ForceHostGenForByteIdentity();

  SceneConfig scene = MakeColorConfiguredScene(kMaxHits);
  RenderConfig render = MakeRectangularRender();
  auto color_cfg = MakeColorConfiguredColorConfig();

  Capture metal = RunMetal(scene, render, /*seed=*/7, kRayCount, /*shuffle=*/true, color_cfg);
  ASSERT_FALSE(metal.masks.empty());

  const uint64_t layer1_bits = 0b1100ull;
  size_t bad_mid = 0;
  size_t bad_final = 0;
  // Popcount = number of set bits in the mask. Uses lumice::PopCount (SWAR)
  // for cross-compiler portability — __builtin_popcountll is a GCC/Clang
  // extension that MSVC rejects with C3861 (see no-msvc-unsafe-builtin policy).
  for (uint64_t m : metal.masks) {
    int pc = PopCount(m);
    // No spurious high bits above the 4-bit color budget.
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
      // Mid-layer emit: bit0 mandatory (AC5), bit1 optional. bits 2/3 must
      // be absent (else it'd have been counted as final-layer above).
      if (pc < 1 || pc > 2) {
        bad_mid++;
      }
    }
  }
  fprintf(stderr, "[color-configured popcount] bad_mid=%zu bad_final=%zu (of %zu total)\n", bad_mid, bad_final,
          metal.masks.size());
  EXPECT_EQ(bad_mid, 0u) << "mid-layer emit popcount out of range — color pass produced spurious/missing bits";
  EXPECT_EQ(bad_final, 0u) << "final-layer emit popcount out of range — cross-layer OR carry or color pass broken";
}

}  // namespace
}  // namespace lumice

#endif  // defined(__APPLE__)
