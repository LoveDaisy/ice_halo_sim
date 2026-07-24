// Tests for the Design-2 CPU emit gate (task-engine-redirect-design2):
//
// Coverage (whitebox / captured rays, not statistical contrast):
//   - FilterSpec::MatchSummandMask / CheckSummandMask per-summand semantics
//     (disjoint OR-terms -> one bit; overlapping summands -> multiple bits;
//      None -> zero bits; simple filter -> single bit; action-out mask).
//     [Unchanged from task-331.2 — this is the shared per-predicate mechanism
//      Design 2 reuses for the color pass.]
//   - ComponentBitsFor + BuildComponentTable layer-key correctness — GPU
//     (Fork-C) code path only, preserved by A4.
//   - CollectData end-to-end (Design 2): a surviving ray matching color
//     summand k gets `color_bits[k]` OR'd into its carried mask (emit +
//     continue); physical-filter-fail terminates the ray BEFORE the color
//     pass runs (AC4); a null color spec is a zero-cost no-op.
//   - AC4 anchor: `color_spec` never touches `w_` — a ray whose physical
//     filter accepts and whose color spec accepts nothing still emits.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
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

namespace lumice {
namespace {

RaypathRecorder ToRecorder(const std::vector<IdType>& rp) {
  RaypathRecorder out;
  out.Clear();
  for (auto fn : rp) {
    out << fn;
  }
  return out;
}

// symmetry-none axis so RaypathSpec/EntryExitSpec match the exact seed path.
AxisDistribution MakeAxis() {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.spread = 360.0f;
  d.azimuth_dist.center = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.center = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.center = 0.0f;
  d.roll_dist.spread = 0.0f;
  return d;
}

RaySeg MakeRay() {
  RaySeg r{};
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;
  r.w_ = 1.0f;
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

// Outgoing candidate for the CollectData gate: positive weight, to_face_ =
// kInvalidId (exit), identity rotation (already "world space").
RaySeg MakeOutgoingCandidate() {
  RaySeg r = MakeRay();
  r.w_ = 0.5f;
  return r;
}

// Build a ComplexSpec whose OR-summands are the given simple filters (each a
// single-term AND-chain). symmetry = none so raypath/EE match exact seeds.
std::unique_ptr<FilterSpec> MakeComplexFromSummands(const Crystal& crystal,
                                                    const std::vector<SimpleFilterParam>& summands,
                                                    FilterConfig::Action action = FilterConfig::kFilterIn) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = action;
  ComplexFilterParam cp{};
  for (const auto& s : summands) {
    std::vector<std::pair<IdType, SimpleFilterParam>> and_clause;
    and_clause.emplace_back(IdType{ 0 }, s);
    cp.filters_.push_back(std::move(and_clause));
  }
  cfg.param_ = cp;
  return FilterSpec::Create(cfg, crystal, MakeAxis());
}

SimpleFilterParam Raypath(const std::vector<IdType>& rp) {
  RaypathFilterParam p{};
  p.raypath_ = rp;
  return SimpleFilterParam{ p };
}

SimpleFilterParam EntryExit(std::optional<IdType> entry, std::optional<IdType> exit) {
  EntryExitFilterParam p{};
  p.entry_ = entry;
  p.exit_ = exit;
  p.min_len_ = 1;
  return SimpleFilterParam{ p };
}

uint64_t SummandMask(const FilterSpec* spec, const std::vector<IdType>& rp, bool* matched = nullptr) {
  bool m = false;
  uint64_t mask = spec->MatchSummandMask(MakeRay(), ToRecorder(rp), nullptr, &m);
  if (matched != nullptr) {
    *matched = m;
  }
  return mask;
}

// ======================= MatchSummandMask semantics =======================

TEST(ComponentGateMatchSummand, DisjointOrTermsSetExactlyOneBit) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }), Raypath({ 5, 7 }) });
  ASSERT_NE(spec, nullptr);

  bool m0 = false;
  EXPECT_EQ(SummandMask(spec.get(), { 3, 5 }, &m0), 0b01ull) << "path {3,5} matches only summand 0";
  EXPECT_TRUE(m0);

  bool m1 = false;
  EXPECT_EQ(SummandMask(spec.get(), { 5, 7 }, &m1), 0b10ull) << "path {5,7} matches only summand 1";
  EXPECT_TRUE(m1);

  bool m2 = true;
  EXPECT_EQ(SummandMask(spec.get(), { 4, 6 }, &m2), 0ull) << "path {4,6} matches neither summand";
  EXPECT_FALSE(m2) << "no summand matched -> collapse boolean must be false";
}

TEST(ComponentGateMatchSummand, OverlappingSummandsSetMultipleBits) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  // summand 0 = EE(entry=3, exit=5) (broad); summand 1 = raypath {3,5} (narrow).
  auto spec = MakeComplexFromSummands(prism, { EntryExit(IdType{ 3 }, IdType{ 5 }), Raypath({ 3, 5 }) });
  ASSERT_NE(spec, nullptr);

  // {3,5} satisfies BOTH the EE ends and the exact raypath -> two bits.
  EXPECT_EQ(SummandMask(spec.get(), { 3, 5 }), 0b11ull);
  // {3,4,5} satisfies EE (entry 3, exit 5) but not the exact raypath -> one bit.
  EXPECT_EQ(SummandMask(spec.get(), { 3, 4, 5 }), 0b01ull);
}

TEST(ComponentGateMatchSummand, NoneFilterProducesWholeCrystalBit) {
  // task-339.1: None now exposes a single whole-crystal summand (mask=0b1)
  // whose per-ray value ignores the raypath contents — every ray traversing a
  // None-filter crystal picks up that crystal's bit.
  Crystal prism = Crystal::CreatePrism(1.0f);
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  cfg.param_ = SimpleFilterParam{ NoneFilterParam{} };
  auto spec = FilterSpec::Create(cfg, prism, MakeAxis());
  ASSERT_NE(spec, nullptr);

  bool m = false;
  EXPECT_EQ(SummandMask(spec.get(), { 3, 5 }, &m), 0b1ull) << "None matches every ray -> summand 0 set";
  EXPECT_TRUE(m);
  // Different raypath contents produce the same whole-crystal mask.
  EXPECT_EQ(SummandMask(spec.get(), { 1, 2, 3, 4 }, &m), 0b1ull) << "None ignores raypath contents";
  EXPECT_TRUE(m);
  // Empty raypath still matches (None does not consult the recorder at all).
  EXPECT_EQ(SummandMask(spec.get(), {}, &m), 0b1ull);
  EXPECT_TRUE(m);
}

TEST(ComponentGateMatchSummand, SimpleFilterExposesSingleSummand) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  cfg.param_ = Raypath({ 3, 5 });
  auto spec = FilterSpec::Create(cfg, prism, MakeAxis());
  ASSERT_NE(spec, nullptr);

  EXPECT_EQ(SummandMask(spec.get(), { 3, 5 }), 0b1ull) << "simple non-None filter = one summand (bit 0)";
  EXPECT_EQ(SummandMask(spec.get(), { 4, 6 }), 0ull);
}

TEST(ComponentGateMatchSummand, CheckSummandMaskAppliesActionButMaskIsPreAction) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto spec = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }), Raypath({ 5, 7 }) }, FilterConfig::kFilterOut);
  ASSERT_NE(spec, nullptr);

  uint64_t mask = 0xdead;
  // {3,5} matches summand 0 -> pre-action mask = 0b01; action=Out -> gate FAILS.
  bool pass = spec->CheckSummandMask(MakeRay(), ToRecorder({ 3, 5 }), nullptr, &mask);
  EXPECT_FALSE(pass) << "filter-out: a matching path fails the gate";
  EXPECT_EQ(mask, 0b01ull) << "mask reflects raw pre-action per-summand match";

  // {4,6} matches nothing -> pre-action mask = 0; action=Out -> gate PASSES.
  bool pass2 = spec->CheckSummandMask(MakeRay(), ToRecorder({ 4, 6 }), nullptr, &mask);
  EXPECT_TRUE(pass2);
  EXPECT_EQ(mask, 0ull);
}

// ======================= ComponentBitsFor / layer-key =======================

SceneConfig MakeTwoLayerComplexScene() {
  // Layer 0: one crystal with a 2-summand Complex.
  // Layer 1: one crystal with a 2-summand Complex.
  // Summand contents are irrelevant to BuildComponentTable (only counts).
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

TEST(ComponentGateBitsFor, SameCrystalSummandAtDifferentLayersGetsDifferentBits) {
  auto scene = MakeTwoLayerComplexScene();
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 4u);

  auto l0 = ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/0);
  auto l1 = ComponentBitsFor(table, /*layer=*/1, /*crystal_id=*/0);

  ASSERT_EQ(l0.size(), 2u);
  ASSERT_EQ(l1.size(), 2u);
  // (mi,ci,sk) push order -> bits 0,1 for layer 0; 2,3 for layer 1.
  EXPECT_EQ(l0[0], 0u);
  EXPECT_EQ(l0[1], 1u);
  EXPECT_EQ(l1[0], 2u);
  EXPECT_EQ(l1[1], 3u);
  // Layer-key correctness: identical (crystal-slot, summand) at different
  // layers maps to DIFFERENT global bits.
  EXPECT_NE(l0[0], l1[0]) << "same crystal-slot/summand at different layers must differ";
  EXPECT_NE(l0[1], l1[1]);
}

TEST(ComponentGateBitsFor, MissingKeyAndNoneCrystalYieldEmpty) {
  auto scene = MakeTwoLayerComplexScene();
  auto table = BuildComponentTable(scene);
  EXPECT_TRUE(ComponentBitsFor(table, /*layer=*/5, /*crystal_id=*/0).empty()) << "absent layer -> empty";
  EXPECT_TRUE(ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/9).empty()) << "absent crystal-slot -> empty";
}

// ======================= CollectData end-to-end =======================

namespace {
struct GateFixture {
  RayBuffer buffer_data[2];
  RayBuffer init_data[2];
  GateFixture() {
    buffer_data[0].Reset(8);
    buffer_data[1].Reset(8);
    init_data[0].Reset(8);
    init_data[1].Reset(8);
  }
};
}  // namespace

// Design 2 CollectData tests share this helper: a match-all physical filter
// (None) so we can vary the color spec independently.
namespace {
std::unique_ptr<FilterSpec> MakeNoneSpec(const Crystal& crystal) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  cfg.param_ = SimpleFilterParam{ NoneFilterParam{} };
  return FilterSpec::Create(cfg, crystal, MakeAxis());
}
}  // namespace

TEST(ComponentGateCollectData, EmitRayGetsMappedBitForMatchedColorSummand) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);
  auto color = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }), Raypath({ 5, 7 }) });
  // summand 0 -> bit 7, summand 1 -> bit 3.
  std::vector<uint8_t> color_bits{ 7, 3 };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;  // filter-pass + prob-fail -> emit

  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, color.get(), &color_bits);

  ASSERT_EQ(fx.buffer_data[1].size_, 1u);
  EXPECT_TRUE(fx.buffer_data[1].rays_[0].IsOutgoing());
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), (1ull << 7)) << "matched color summand 0 -> mapped bit 7";
}

TEST(ComponentGateCollectData, ContinueRayCarriesMappedBitIntoNextLayerInit) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);
  auto color = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }), Raypath({ 5, 7 }) });
  std::vector<uint8_t> color_bits{ 7, 3 };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 1.0f;  // filter-pass + prob-pass -> continue

  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 5, 7 }));

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, color.get(), &color_bits);

  ASSERT_TRUE(fx.buffer_data[1].rays_[0].IsContinue());
  ASSERT_EQ(fx.init_data[1].size_, 1u);
  EXPECT_EQ(fx.init_data[1].ComponentAt(0), (1ull << 3))
      << "matched color summand 1 -> mapped bit 3, carried forward across layers";
}

TEST(ComponentGateCollectData, ProducedBitsOrIntoCarriedMask) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);
  auto color = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }), Raypath({ 5, 7 }) });
  std::vector<uint8_t> color_bits{ 7, 3 };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));
  // Pre-existing (prior-layer) bit that must survive the OR.
  constexpr uint64_t kPrior = (1ull << 20);
  fx.buffer_data[1].SetComponent(0, kPrior);

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, color.get(), &color_bits);

  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), kPrior | (1ull << 7)) << "new bit OR-accumulates onto carried mask";
}

// AC4 anchor #1 (physical-fail short-circuits the color pass — no cheating
// tag can outlive a rejected ray).
TEST(ComponentGateCollectData, PhysicalFailTerminatesRayAndProducesNoBits) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  // Physical filter: filter_in matching {3,5} only.
  auto phys = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }) });
  // Color spec that WOULD tag the same ray, but must not be evaluated.
  auto color = MakeComplexFromSummands(prism, { Raypath({ 4, 6 }) });
  std::vector<uint8_t> color_bits{ 9 };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  GateFixture fx;
  // {4,6} fails the physical filter (raypath not in set) — ray terminates.
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 4, 6 }));

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, color.get(), &color_bits);

  EXPECT_LT(fx.buffer_data[1].rays_[0].w_, 0.0f) << "physical-fail terminates the ray";
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), 0ull) << "physical-fail short-circuits the color pass";
}

// AC4 anchor #2 (decoupled predicates): physical filter accepts a ray whose
// color spec matches nothing → ray still emits, mask stays 0. Color pass
// cannot influence physical survival.
TEST(ComponentGateCollectData, ColorMissDoesNotAffectPhysicalSurvival) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);  // always passes
  auto color = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }) });
  std::vector<uint8_t> color_bits{ 4 };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  GateFixture fx;
  // {4,6} does NOT match the color predicate {3,5} — but physical filter
  // (None) accepts everything → ray must still emit, without a color bit.
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 4, 6 }));

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, color.get(), &color_bits);

  EXPECT_TRUE(fx.buffer_data[1].rays_[0].IsOutgoing()) << "physical accepts → ray emits regardless of color match";
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), 0ull) << "color miss leaves the mask clean";
}

TEST(ComponentGateCollectData, MatchAllColorSpecTagsEveryRayRegardlessOfPath) {
  // Design 2's whole-crystal color = match-all NoneFilterParam predicate.
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);
  auto color = MakeNoneSpec(prism);  // color = None (match-all)
  std::vector<uint8_t> color_bits{ 11 };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 4, 6, 8 }));
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({}));

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, color.get(), &color_bits);

  ASSERT_EQ(fx.buffer_data[1].size_, 3u);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_TRUE(fx.buffer_data[1].rays_[i].IsOutgoing()) << "ray " << i << " must survive";
    EXPECT_EQ(fx.buffer_data[1].ComponentAt(i), (1ull << 11))
        << "ray " << i << " must carry the whole-crystal bit regardless of raypath";
  }
}

TEST(ComponentGateCollectData, NullColorSpecLeavesMaskUntouched) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;

  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));
  constexpr uint64_t kPrior = (1ull << 20);
  fx.buffer_data[1].SetComponent(0, kPrior);

  // Design 2 zero-cost path: no color spec → mask untouched, ray still emits.
  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, /*color_spec=*/nullptr,
              /*color_bits=*/nullptr);

  EXPECT_TRUE(fx.buffer_data[1].rays_[0].IsOutgoing());
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), kPrior) << "nullptr color spec: no production, mask carried verbatim";
}

}  // namespace
}  // namespace lumice
