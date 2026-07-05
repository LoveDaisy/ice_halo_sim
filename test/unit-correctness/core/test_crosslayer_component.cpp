// Whitebox cross-layer verification for the raypath-color foundation
// (scrum-raypath-color-foundation task-331.3 crosslayer-accumulate-deliver +
// task-331.4 cpu-whitebox-verify).
//
// This is the CPU-first correctness gate for cross-MS-layer component-mask
// accumulation. T1 wired the per-ray uint64 mask transport; T2 made the emit
// gate OR each layer's component bits into the carried mask. This file nails the
// one cross-layer risk NOT covered by T1/T2's single-CollectData / RayBuffer
// unit tests: that a ray's OR-accumulated mask stays paired with the ray through
// the continuation-pool decorrelation shuffle and the layer-entry hand-off, so
// the final emitted mask is exactly the union of the components matched along
// that ray's real multi-layer path (with distinct per-layer bits).
//
// Discipline (per project memory): per-captured-ray assertions (not statistical
// contrast), positive + negative controls, real machinery (real CollectData /
// InitRay*/ BuildComponentTable / FilterSpec / the production shuffle primitive),
// no synthetic bypass, and no patching a test to hide a mechanism defect.
//
// ⭐Mechanism defect this file pins down (task-331.3 finding): both CPU paths'
// continuation shuffle used `std::swap(buf[i], buf[j])`, which swaps only the
// rays_ array (operator[] returns RaySeg&) and leaves the parallel components_
// array behind — decorrelating each ray's cross-layer mask. The
// `ContinuationNaiveSwapDecorrelatesComponentMask` test demonstrates the bug on
// the exact operation the production code performed; the fix is
// `RayBuffer::SwapRay` (swaps rays_ + components_ together), verified by
// `ContinuationShuffleWithSwapRayPreservesComponentMask`.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/simulator.hpp"
#include "core/trace_ops.hpp"

namespace lumice {
namespace {

// ---------------------------------------------------------------------------
// Shared builders (mirror test_component_gate.cpp so both files use the same
// real FilterSpec / raypath shapes).
// ---------------------------------------------------------------------------

RaypathRecorder ToRecorder(const std::vector<IdType>& rp) {
  RaypathRecorder out;
  out.Clear();
  for (auto fn : rp) {
    out << fn;
  }
  return out;
}

AxisDistribution MakeAxis() {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.std = 360.0f;
  d.azimuth_dist.mean = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.mean = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.mean = 0.0f;
  d.roll_dist.std = 0.0f;
  return d;
}

RaySeg MakeRay() {
  RaySeg r{};
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;
  r.w_ = 1.0f;
  r.crystal_idx_ = 0;
  r.crystal_config_id_ = 0;
  r.d_[0] = 0.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 1.0f;
  r.p_[0] = 0.0f;
  r.p_[1] = 0.0f;
  r.p_[2] = 0.0f;
  r.crystal_rot_ = Rotation{};
  return r;
}

RaySeg MakeOutgoingCandidate() {
  RaySeg r = MakeRay();
  r.w_ = 0.5f;
  return r;
}

SimpleFilterParam Raypath(const std::vector<IdType>& rp) {
  RaypathFilterParam p{};
  p.raypath_ = rp;
  return SimpleFilterParam{ p };
}

std::unique_ptr<FilterSpec> MakeComplexFromSummands(const Crystal& crystal,
                                                    const std::vector<SimpleFilterParam>& summands) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  ComplexFilterParam cp{};
  for (const auto& s : summands) {
    std::vector<std::pair<IdType, SimpleFilterParam>> and_clause;
    and_clause.emplace_back(IdType{ 0 }, s);
    cp.filters_.push_back(std::move(and_clause));
  }
  cfg.param_ = cp;
  return FilterSpec::Create(cfg, crystal, MakeAxis());
}

// A real 2-layer scene: layer 0 and layer 1 each carry a single crystal with a
// 2-summand Complex filter. BuildComponentTable counts summands only, so this
// yields bits {0,1} for (layer0,crystal0) and {2,3} for (layer1,crystal0) — the
// layer-key separation we assert end-to-end below. The summand CONTENTS are
// irrelevant to the table; the CollectData tests build their own FilterSpec with
// the raypaths they need.
SceneConfig MakeTwoLayerScene() {
  auto make_complex2 = []() {
    ComplexFilterParam cp{};
    cp.filters_.resize(2);
    cp.filters_[0].emplace_back(IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
    cp.filters_[1].emplace_back(IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 1 } });
    return FilterParam{ cp };
  };
  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  for (int layer = 0; layer < 2; layer++) {
    MsInfo ms{};
    ms.prob_ = 0.5f;
    ScatteringSetting s{};
    s.filter_.id_ = 0;
    s.filter_.symmetry_ = FilterConfig::kSymNone;
    s.filter_.action_ = FilterConfig::kFilterIn;
    s.filter_.param_ = make_complex2();
    s.crystal_ = CrystalConfig{};
    s.crystal_proportion_ = 1.0f;
    ms.setting_.push_back(std::move(s));
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

// Populate a continuation pool with N tagged rays: ray i carries identity i
// (encoded in p_[0]) and a unique component mask kMaskBase+i. Built by direct
// slot writes (not EmplaceBack) so the shuffle tests can inspect any slot
// without tripping N4 EmplaceBack asserts. Returns the mask base.
constexpr uint64_t kMaskBase = 1000;

void FillTaggedPool(RayBuffer& buf, size_t n) {
  buf.Reset(n + 4);
  buf.size_ = n;
  for (size_t i = 0; i < n; i++) {
    RaySeg r = MakeRay();
    r.p_[0] = static_cast<float>(i);  // identity tag
    buf.rays_[i] = r;
    buf.SetComponent(i, kMaskBase + i);
  }
}

// The expected mask for the ray now resting in slot i, derived from its identity
// tag (p_[0]) — the ground truth against which the shuffled mask is checked.
uint64_t ExpectedMaskForSlot(const RayBuffer& buf, size_t i) {
  return kMaskBase + static_cast<uint64_t>(buf.rays_[i].p_[0]);
}

}  // namespace

// ===========================================================================
// A. Continuation-shuffle correlation — the core cross-layer mechanism.
// ===========================================================================

// Primitive unit: SwapRay swaps rays_ AND components_ (and deliberately NOT
// recorders_, which are reset at layer entry).
TEST(CrossLayerSwapRay, SwapsRayAndComponentButNotRecorder) {
  RayBuffer buf;
  buf.Reset(4);
  buf.size_ = 2;

  RaySeg a = MakeRay();
  a.p_[0] = 11.0f;
  RaySeg b = MakeRay();
  b.p_[0] = 22.0f;
  buf.rays_[0] = a;
  buf.rays_[1] = b;
  buf.SetComponent(0, 0xAAAAull);
  buf.SetComponent(1, 0xBBBBull);
  // Distinct recorders so we can assert they are NOT swapped.
  buf.RecorderAppend(0, 3);
  buf.RecorderAppend(1, 7);

  buf.SwapRay(0, 1);

  EXPECT_FLOAT_EQ(buf.rays_[0].p_[0], 22.0f) << "RaySeg not swapped";
  EXPECT_FLOAT_EQ(buf.rays_[1].p_[0], 11.0f);
  EXPECT_EQ(buf.ComponentAt(0), 0xBBBBull) << "component mask must swap with its ray";
  EXPECT_EQ(buf.ComponentAt(1), 0xAAAAull);
  // Recorders intentionally untouched (cleared at layer entry before read).
  EXPECT_EQ(buf.RecorderAt(0).size_, 1u);
  EXPECT_EQ(buf.RecorderDataPtr(0)[0], 3) << "recorder deliberately not swapped";
  EXPECT_EQ(buf.RecorderDataPtr(1)[0], 7);

  buf.SwapRay(1, 1);  // self-swap is a no-op
  EXPECT_EQ(buf.ComponentAt(1), 0xAAAAull);
}

// ⭐Defect demonstration: the production shuffle performed
// `std::swap(buf[i], buf[j])` == `std::swap(buf.rays_[i], buf.rays_[j])`, which
// swaps ONLY the RaySeg. Running that exact operation over a Fisher-Yates loop
// decorrelates each ray from its component mask. This is the bug T1/T2's unit
// tests could not see and that task-331.3 fixes with SwapRay.
TEST(CrossLayerShuffle, ContinuationNaiveSwapDecorrelatesComponentMask) {
  constexpr size_t kN = 16;
  RayBuffer buf;
  FillTaggedPool(buf, kN);

  RandomNumberGenerator rng(42);
  bool any_moved = false;
  for (size_t i = 0; i < buf.size_; i++) {
    size_t j = static_cast<size_t>(rng.GetUniform() * (buf.size_ - i)) + i;
    // EXACT operation the pre-331.3 production shuffle performed.
    std::swap(buf.rays_[i], buf.rays_[j]);
    if (i != j) {
      any_moved = true;
    }
  }
  ASSERT_TRUE(any_moved) << "seed must yield a non-identity permutation for this to be meaningful";

  size_t mismatches = 0;
  for (size_t i = 0; i < buf.size_; i++) {
    if (buf.ComponentAt(i) != ExpectedMaskForSlot(buf, i)) {
      mismatches++;
    }
  }
  EXPECT_GT(mismatches, 0u) << "naive RaySeg-only swap must decorrelate the mask from its ray "
                               "(this is the defect SwapRay fixes)";
}

// ⭐Fix verification: the same Fisher-Yates permutation via SwapRay keeps every
// ray paired with its component mask.
TEST(CrossLayerShuffle, ContinuationShuffleWithSwapRayPreservesComponentMask) {
  constexpr size_t kN = 16;
  RayBuffer buf;
  FillTaggedPool(buf, kN);

  RandomNumberGenerator rng(42);
  bool any_moved = false;
  for (size_t i = 0; i < buf.size_; i++) {
    size_t j = static_cast<size_t>(rng.GetUniform() * (buf.size_ - i)) + i;
    buf.SwapRay(i, j);  // production shuffle after task-331.3
    if (i != j) {
      any_moved = true;
    }
  }
  ASSERT_TRUE(any_moved) << "permutation must be non-trivial";

  for (size_t i = 0; i < buf.size_; i++) {
    EXPECT_EQ(buf.ComponentAt(i), ExpectedMaskForSlot(buf, i))
        << "slot " << i << " mask decorrelated from its ray after SwapRay shuffle";
  }
}

// ===========================================================================
// B. Cross-layer hand-off: InitRayOtherMs carries the mask across the layer
//    boundary and does NOT reset it (contrast: InitRayFirstMs zeroes).
// ===========================================================================

TEST(CrossLayerHandoff, InitRayOtherMsCarriesComponentMaskNoReset) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(7);
  AxisDistribution axis = MakeAxis();

  // Source continuation pool (init_data[0]) with three known masks.
  RayBuffer init_data[2];
  init_data[0].Reset(16);
  init_data[1].Reset(16);
  const uint64_t masks[3] = { (1ull << 5), (1ull << 9) | (1ull << 2), (1ull << 40) };
  for (size_t i = 0; i < 3; i++) {
    RaySeg r = MakeRay();  // continuation ray: to_face_ == kInvalidId, w_ > 0
    init_data[0].EmplaceBack(r, RaypathRecorder{});
    init_data[0].SetComponent(i, masks[i]);
  }

  RayBuffer buffer_data[2];
  buffer_data[0].Reset(16);
  buffer_data[1].Reset(16);
  RayBuffer all_data;
  all_data.Reset(64);
  // Pre-poison destination component slots so the "carried, not fresh-zero"
  // assertion is load-bearing.
  for (size_t i = 0; i < 8; i++) {
    buffer_data[0].SetComponent(i, 0xFFFFFFFFFFFFFFFFull);
  }

  size_t init_ray_offset = 0;
  InitRayOtherMs(rng, init_data, /*curr_ray_num=*/3, crystal, /*curr_crystal_id=*/0, axis, buffer_data, all_data,
                 init_ray_offset);

  ASSERT_EQ(buffer_data[0].size_, 3u);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(buffer_data[0].ComponentAt(i), masks[i])
        << "InitRayOtherMs must carry the prior-layer mask verbatim (no cross-layer reset), slot " << i;
  }
  EXPECT_EQ(init_ray_offset, 3u);
}

// Negative control for the reset invariant: a ray that arrives at a fresh first
// MS layer carrying stale bits has its mask cleared (root reset).
TEST(CrossLayerHandoff, InitRayFirstMsClearsCarriedCrossLayerBits) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(3);
  SunParam sun{ 90.0f, 0.0f, 0.5f };
  WlParam wl{ 550.0f, 1.0f };
  AxisDistribution axis = MakeAxis();

  RayBuffer buffer_data[2];
  buffer_data[0].Reset(8);
  buffer_data[1].Reset(8);
  RayBuffer all_data;
  all_data.Reset(16);

  for (size_t i = 0; i < 4; i++) {
    buffer_data[0].SetComponent(i, (1ull << 33) | (1ull << 1));  // stale cross-layer bits
  }

  InitRayFirstMs(rng, sun, wl, /*curr_ray_num=*/4, crystal, /*curr_crystal_id=*/0, axis, buffer_data, all_data);

  ASSERT_EQ(buffer_data[0].size_, 4u);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(buffer_data[0].ComponentAt(i), 0ull) << "first-MS entry must clear any carried mask, slot " << i;
  }
}

// ===========================================================================
// C. End-to-end cross-layer accumulation through two real CollectData passes,
//    with real layer-keyed bits from BuildComponentTable.
//    Claims (per captured ray): union accumulation, distinct layer-key bits,
//    cross-layer AND reconstruction, and no spurious bits (negative control).
// ===========================================================================

TEST(CrossLayerAccumulation, OrsBothLayerBitsWithDistinctLayerKeys) {
  auto scene = MakeTwoLayerScene();
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 4u);

  // Real layer-keyed bit maps: layer 0 -> {0,1}, layer 1 -> {2,3}.
  auto bits_l0 = ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/0);
  auto bits_l1 = ComponentBitsFor(table, /*layer=*/1, /*crystal_id=*/0);
  ASSERT_EQ(bits_l0.size(), 2u);
  ASSERT_EQ(bits_l1.size(), 2u);
  const uint64_t l0_set = (1ull << bits_l0[0]) | (1ull << bits_l0[1]);
  const uint64_t l1_set = (1ull << bits_l1[0]) | (1ull << bits_l1[1]);
  ASSERT_EQ(l0_set & l1_set, 0ull) << "layer-key: layer0 and layer1 bit sets must be disjoint";

  Crystal prism = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(42);

  // ---- Layer 1 (first MS): ray matches summand 0 of {Raypath{3,5},Raypath{5,7}}
  //      -> produces bit bits_l0[0]. prob_=1 -> continue -> carried into init.
  auto spec_l0 = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }), Raypath({ 5, 7 }) });
  RayBuffer bd0[2];
  RayBuffer id0[2];
  for (auto* b : { &bd0[0], &bd0[1], &id0[0], &id0[1] }) {
    b->Reset(8);
  }
  bd0[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));
  MsInfo ms_cont;
  ms_cont.prob_ = 1.0f;
  CollectData(rng, ms_cont, spec_l0.get(), bd0, id0, &bits_l0);
  ASSERT_TRUE(bd0[1].rays_[0].IsContinue());
  ASSERT_EQ(id0[1].size_, 1u);
  const uint64_t carried = id0[1].ComponentAt(0);
  EXPECT_EQ(carried, (1ull << bits_l0[0])) << "layer-1 continuation carries exactly its matched summand's bit";

  // ---- Layer 2 (next MS): the SAME ray (carried mask) now matches summand 0 of
  //      layer 1's filter {Raypath{2,4},Raypath{4,6}} -> bit bits_l1[0]. prob_=0
  //      -> emit. The carried layer-1 bit must survive and OR with the new bit.
  auto spec_l1 = MakeComplexFromSummands(prism, { Raypath({ 2, 4 }), Raypath({ 4, 6 }) });
  RayBuffer bd1[2];
  RayBuffer id1[2];
  for (auto* b : { &bd1[0], &bd1[1], &id1[0], &id1[1] }) {
    b->Reset(8);
  }
  bd1[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 2, 4 }));
  bd1[1].SetComponent(0, carried);  // cross-layer hand-off of the accumulated mask
  MsInfo ms_emit;
  ms_emit.prob_ = 0.0f;
  CollectData(rng, ms_emit, spec_l1.get(), bd1, id1, &bits_l1);

  ASSERT_TRUE(bd1[1].rays_[0].IsOutgoing());
  const uint64_t final_mask = bd1[1].ComponentAt(0);

  // Union accumulation: both layers' bits present.
  EXPECT_NE(final_mask & (1ull << bits_l0[0]), 0ull) << "layer-1 matched bit missing from final mask";
  EXPECT_NE(final_mask & (1ull << bits_l1[0]), 0ull) << "layer-2 matched bit missing from final mask";
  // Distinct layer keys end-to-end (not just at table level).
  EXPECT_NE(bits_l0[0], bits_l1[0]);
  // Cross-layer AND reconstruction (§4.2 hard-commit ①).
  EXPECT_TRUE((final_mask & l0_set) != 0 && (final_mask & l1_set) != 0)
      << "must be able to reconstruct 'matched X@L1 AND Y@L2' from the accumulated mask";
  // Negative control: exactly the two matched summands' bits, no spurious bits.
  EXPECT_EQ(final_mask & (1ull << bits_l0[1]), 0ull) << "unmatched layer-1 summand bit must NOT be set";
  EXPECT_EQ(final_mask & (1ull << bits_l1[1]), 0ull) << "unmatched layer-2 summand bit must NOT be set";
  EXPECT_EQ(final_mask, (1ull << bits_l0[0]) | (1ull << bits_l1[0]))
      << "final mask must be exactly the union of the matched per-layer bits";
}

// Negative control: a ray that does NOT match any layer-2 summand carries no
// layer-2 bit — it only keeps the layer-1 bit it arrived with. (Filter action
// In -> non-match terminates the ray, so it is not emitted, and no L2 bit is
// produced; the carried L1 mask is untouched.)
TEST(CrossLayerAccumulation, NonMatchingSecondLayerAddsNoBit) {
  auto scene = MakeTwoLayerScene();
  auto table = BuildComponentTable(scene);
  auto bits_l0 = ComponentBitsFor(table, 0, 0);
  auto bits_l1 = ComponentBitsFor(table, 1, 0);
  ASSERT_EQ(bits_l0.size(), 2u);
  ASSERT_EQ(bits_l1.size(), 2u);

  Crystal prism = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(42);

  const uint64_t carried = (1ull << bits_l0[0]);  // arrived from layer 1

  auto spec_l1 = MakeComplexFromSummands(prism, { Raypath({ 2, 4 }), Raypath({ 4, 6 }) });
  RayBuffer bd[2];
  RayBuffer id[2];
  for (auto* b : { &bd[0], &bd[1], &id[0], &id[1] }) {
    b->Reset(8);
  }
  // Raypath {8,9} matches NEITHER layer-2 summand.
  bd[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 8, 9 }));
  bd[1].SetComponent(0, carried);
  MsInfo ms_emit;
  ms_emit.prob_ = 0.0f;
  CollectData(rng, ms_emit, spec_l1.get(), bd, id, &bits_l1);

  const uint64_t mask = bd[1].ComponentAt(0);
  EXPECT_LT(bd[1].rays_[0].w_, 0.0f) << "non-matching filter-In ray terminates";
  EXPECT_EQ(mask & (1ull << bits_l1[0]), 0ull) << "no layer-2 summand matched -> no layer-2 bit";
  EXPECT_EQ(mask & (1ull << bits_l1[1]), 0ull);
  EXPECT_EQ(mask, carried) << "carried layer-1 mask must be preserved unchanged";
}

}  // namespace lumice
