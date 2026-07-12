// Tests for src/core/filter_spec.hpp::BuildColorSpecGroups —
// scrum-color-predicate-symmetry Step 4.
//
// Covers:
//   - Empty placement returns empty groups (AC3 zero-cost path).
//   - All-kSymNone placement produces exactly ONE group carrying every
//     predicate in placement order (AC3 逐位相等 anchor — matches the pre-
//     refactor "single synthesized ComplexFilterParam with kSymNone" path).
//   - Mixed symmetry produces one group per distinct value; within a group,
//     predicate order matches the placement's original order (AC2 anchor).
//   - Interleaved symmetries preserve placement-original ordering inside each
//     group (防 "分组变成排序再分组" off-by-one).

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "config/color_gate_table.hpp"
#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/filter_spec.hpp"
#include "core/math.hpp"

namespace lumice {
namespace {

// Az-uniform, latitude fixed (mimics test_filter_spec.cpp::MakeAxis) —
// AxisDistribution shape matters for FilterSpec::Create's IsDApplicable /
// ComputeSigmaA derivations.
AxisDistribution MakeAxis(float roll_mean_deg = 0.0f) {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.std = 360.0f;
  d.azimuth_dist.mean = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.mean = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.mean = roll_mean_deg;
  d.roll_dist.std = 0.0f;
  return d;
}

// Build a ColorGatePlacement directly (bypassing BuildColorGateTable) — the
// helper only reads predicates_/bits_/symmetries_ so the raw arrays suffice
// and we avoid coupling this unit's tests to the gate-table builder shape.
ColorGatePlacement MakePlacement(const std::vector<SimpleFilterParam>& predicates, const std::vector<uint8_t>& bits,
                                 const std::vector<uint8_t>& symmetries) {
  ColorGatePlacement p;
  p.predicates_ = predicates;
  p.bits_ = bits;
  p.symmetries_ = symmetries;
  return p;
}

SimpleFilterParam PredNone() {
  return SimpleFilterParam{ NoneFilterParam{} };
}

SimpleFilterParam PredRaypath(std::vector<IdType> rp) {
  RaypathFilterParam r{};
  r.raypath_ = std::move(rp);
  return SimpleFilterParam{ r };
}

}  // namespace

// ---- Empty placement (AC3 zero-cost) ----

TEST(BuildColorSpecGroups, EmptyPlacementYieldsEmptyGroups) {
  ColorGatePlacement placement{};
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();
  auto groups = BuildColorSpecGroups(placement, crystal, axis);
  EXPECT_TRUE(groups.empty());
}

// ---- All-kSymNone → single group carrying every predicate in order (AC3) ----

TEST(BuildColorSpecGroups, AllSymNoneProducesSingleGroupWithAllPredicatesInOrder) {
  auto placement = MakePlacement({ PredRaypath({ 3, 5, 1 }), PredNone(), PredRaypath({ 4, 6, 2 }) }, { 0, 1, 2 },
                                 { FilterConfig::kSymNone, FilterConfig::kSymNone, FilterConfig::kSymNone });
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();
  auto groups = BuildColorSpecGroups(placement, crystal, axis);
  ASSERT_EQ(groups.size(), 1u);
  // bits array preserves placement order across the single group.
  ASSERT_EQ(groups[0].bits.size(), 3u);
  EXPECT_EQ(groups[0].bits[0], 0u);
  EXPECT_EQ(groups[0].bits[1], 1u);
  EXPECT_EQ(groups[0].bits[2], 2u);
  // The synthesized FilterSpec is non-null and usable.
  ASSERT_NE(groups[0].spec, nullptr);
}

// ---- Mixed symmetry → one group per distinct value (AC2) ----

TEST(BuildColorSpecGroups, MixedSymmetryProducesOneGroupPerDistinctValue) {
  // 3 predicates: symmetries [kSymNone, kSymP, kSymP]. Expect 2 groups —
  // first is the singleton kSymNone (bit 0), second is the 2-predicate kSymP
  // (bits 1 and 2). Group order follows first-occurrence of the symmetry value.
  auto placement = MakePlacement({ PredRaypath({ 3, 5, 1 }), PredNone(), PredRaypath({ 4, 6, 2 }) }, { 0, 1, 2 },
                                 { FilterConfig::kSymNone, FilterConfig::kSymP, FilterConfig::kSymP });
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();
  auto groups = BuildColorSpecGroups(placement, crystal, axis);
  ASSERT_EQ(groups.size(), 2u);
  // Group 0: kSymNone with 1 predicate (bit 0).
  ASSERT_EQ(groups[0].bits.size(), 1u);
  EXPECT_EQ(groups[0].bits[0], 0u);
  // Group 1: kSymP with 2 predicates (bits 1, 2).
  ASSERT_EQ(groups[1].bits.size(), 2u);
  EXPECT_EQ(groups[1].bits[0], 1u);
  EXPECT_EQ(groups[1].bits[1], 2u);
}

// ---- Interleaved symmetries preserve placement-original order within group ----

TEST(BuildColorSpecGroups, GroupBitsPreserveOriginalOrderNotSymmetryOrder) {
  // Placement: symmetries interleaved [P, B, P, B]. If BuildColorSpecGroups
  // accidentally sorts by symmetry value first, the bits array for group P
  // would become [1, 2] instead of the correct placement-original [0, 2].
  auto placement =
      MakePlacement({ PredRaypath({ 3, 5, 1 }), PredNone(), PredRaypath({ 4, 6, 2 }), PredNone() }, { 0, 1, 2, 3 },
                    { FilterConfig::kSymP, FilterConfig::kSymB, FilterConfig::kSymP, FilterConfig::kSymB });
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();
  auto groups = BuildColorSpecGroups(placement, crystal, axis);
  ASSERT_EQ(groups.size(), 2u);
  // Group 0 (first-occurrence: P): bits = [0, 2] — placement indices 0 and 2.
  ASSERT_EQ(groups[0].bits.size(), 2u);
  EXPECT_EQ(groups[0].bits[0], 0u);
  EXPECT_EQ(groups[0].bits[1], 2u);
  // Group 1 (B): bits = [1, 3].
  ASSERT_EQ(groups[1].bits.size(), 2u);
  EXPECT_EQ(groups[1].bits[0], 1u);
  EXPECT_EQ(groups[1].bits[1], 3u);
}

// ---- 3 distinct symmetries → 3 groups; group order = first-occurrence ----

TEST(BuildColorSpecGroups, ThreeDistinctSymmetriesProduceThreeGroupsInOccurrenceOrder) {
  auto placement = MakePlacement({ PredNone(), PredNone(), PredNone() }, { 0, 1, 2 },
                                 { FilterConfig::kSymB, FilterConfig::kSymP, FilterConfig::kSymD });
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();
  auto groups = BuildColorSpecGroups(placement, crystal, axis);
  ASSERT_EQ(groups.size(), 3u);
  // Each group has exactly one member; order reflects first-occurrence
  // (B → P → D).
  ASSERT_EQ(groups[0].bits.size(), 1u);
  ASSERT_EQ(groups[1].bits.size(), 1u);
  ASSERT_EQ(groups[2].bits.size(), 1u);
  EXPECT_EQ(groups[0].bits[0], 0u);
  EXPECT_EQ(groups[1].bits[0], 1u);
  EXPECT_EQ(groups[2].bits[0], 2u);
}

}  // namespace lumice
