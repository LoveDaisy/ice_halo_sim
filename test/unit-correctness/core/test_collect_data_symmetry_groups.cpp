// Tests for src/core/simulator.cpp::CollectData multi-group overload —
// scrum-color-predicate-symmetry Step 5.
//
// Covers:
//   - Two color-spec groups (per symmetry value) each contribute their own
//     mapped bits to a surviving ray's carried mask via OR — additive across
//     groups (AC2 anchor for multi-symmetry placement).
//   - Physical filter check + MS prob_ roll (rng.GetUniform() < prob_) happen
//     exactly once per ray, independent of the number of color groups. Guards
//     against a refactor that accidentally moves the prob roll inside the
//     per-group loop, which would silently perturb continue/emit routing as a
//     function of color config (raypath_color's symmetry breakdown).

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

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

RaySeg MakeOutgoingCandidate() {
  RaySeg r{};
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;
  r.w_ = 0.5f;
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

std::unique_ptr<FilterSpec> MakeNoneSpec(const Crystal& crystal) {
  FilterConfig cfg{};
  cfg.id_ = 1;
  cfg.symmetry_ = FilterConfig::kSymNone;
  cfg.action_ = FilterConfig::kFilterIn;
  cfg.param_ = SimpleFilterParam{ NoneFilterParam{} };
  return FilterSpec::Create(cfg, crystal, MakeAxis());
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

TEST(CollectDataColorGroups, MultipleGroupsEachContributeTheirOwnBits) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);
  // Group A: one summand — path {3,5}. Bit map: local summand 0 -> global bit 7.
  auto color_a = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }) });
  std::vector<uint8_t> bits_a{ 7 };
  // Group B: one summand — path {3,5}. Bit map: local summand 0 -> global bit 3.
  auto color_b = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }) });
  std::vector<uint8_t> bits_b{ 3 };

  std::vector<ColorSpecGroup> groups{ { color_a.get(), &bits_a }, { color_b.get(), &bits_b } };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;  // emit
  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));

  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, &groups);

  ASSERT_EQ(fx.buffer_data[1].size_, 1u);
  EXPECT_TRUE(fx.buffer_data[1].rays_[0].IsOutgoing());
  // Both groups matched -> both bits OR'd into the ray's carried mask.
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), (1ull << 7) | (1ull << 3));
}

TEST(CollectDataColorGroups, NonMatchingGroupContributesNothing) {
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);
  auto color_a = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }) });  // matches
  std::vector<uint8_t> bits_a{ 7 };
  auto color_b = MakeComplexFromSummands(prism, { Raypath({ 9, 9 }) });  // does NOT match
  std::vector<uint8_t> bits_b{ 3 };
  std::vector<ColorSpecGroup> groups{ { color_a.get(), &bits_a }, { color_b.get(), &bits_b } };

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;
  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));
  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, &groups);
  ASSERT_EQ(fx.buffer_data[1].size_, 1u);
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), (1ull << 7));
}

TEST(CollectDataColorGroups, PhysicalFilterAndProbRollOnlyEvaluatedOnce) {
  // Compare RNG state after CollectData across three color configurations:
  //   (a) null color_groups
  //   (b) one-group color_groups
  //   (c) three-group color_groups
  // All three must consume the exact same RNG amount (the physical-filter/prob
  // roll runs once per ray, unaffected by color-spec count).
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);

  auto color = MakeComplexFromSummands(prism, { Raypath({ 3, 5 }) });
  std::vector<uint8_t> bits{ 0 };

  auto run_and_peek_rng = [&](const std::vector<ColorSpecGroup>* groups) {
    RandomNumberGenerator rng(1337);
    MsInfo ms_info;
    ms_info.prob_ = 0.5f;  // may continue OR emit — but roll runs exactly once
    GateFixture fx;
    fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));
    CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, groups);
    // Sample RNG state via the next draw. The value tells us how much of the
    // stream CollectData consumed — if the prob roll accidentally ran per-group
    // instead of once, subsequent draws would differ across configurations.
    return rng.GetUniform();
  };

  std::vector<ColorSpecGroup> one_group{ { color.get(), &bits } };
  std::vector<ColorSpecGroup> three_groups{
    { color.get(), &bits },
    { color.get(), &bits },
    { color.get(), &bits },
  };

  float rng_null = run_and_peek_rng(nullptr);
  float rng_one = run_and_peek_rng(&one_group);
  float rng_three = run_and_peek_rng(&three_groups);

  EXPECT_FLOAT_EQ(rng_null, rng_one)
      << "adding a color group must not perturb the RNG stream (roll stays outside per-group loop)";
  EXPECT_FLOAT_EQ(rng_null, rng_three)
      << "adding three color groups must not perturb the RNG stream (roll stays outside per-group loop)";
}

TEST(CollectDataColorGroups, EmptyGroupsVectorBehavesLikeNull) {
  // AC3 zero-cost path: an empty groups vector must be equivalent to passing
  // nullptr — the emit gate should skip the color pass entirely.
  Crystal prism = Crystal::CreatePrism(1.0f);
  auto phys = MakeNoneSpec(prism);

  RandomNumberGenerator rng(42);
  MsInfo ms_info;
  ms_info.prob_ = 0.0f;
  GateFixture fx;
  fx.buffer_data[1].EmplaceBack(MakeOutgoingCandidate(), ToRecorder({ 3, 5 }));

  std::vector<ColorSpecGroup> empty;
  CollectData(rng, ms_info, phys.get(), fx.buffer_data, fx.init_data, &empty);

  ASSERT_EQ(fx.buffer_data[1].size_, 1u);
  EXPECT_TRUE(fx.buffer_data[1].rays_[0].IsOutgoing());
  EXPECT_EQ(fx.buffer_data[1].ComponentAt(0), 0u);
}

}  // namespace lumice
