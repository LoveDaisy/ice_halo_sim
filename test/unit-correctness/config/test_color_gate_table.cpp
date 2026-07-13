// Tests for src/config/color_gate_table.hpp — Design 2 (2026-07-08,
// doc/gui-custom-spectrum-and-raypath-color.md §4.0) CPU-only,
// placement-scoped color-predicate → component-bit source.
//
// These tests lock down: bit allocation, structural dedup, layer-key
// distinctness, ambiguity throws, kNoBit overflow, empty-config no-op,
// and pure-function idempotence (crucial invariant — server.cpp and the
// emit gate each rebuild the table from the same config and MUST get the
// same bit numbering).

#include <gtest/gtest.h>

#include <vector>

#include "config/color_gate_table.hpp"
#include "config/component_table.hpp"
#include "config/config_compare.hpp"  // operator== for SimpleFilterParam alts
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "core/def.hpp"

namespace {

using lumice::ColorClassConfig;
using lumice::ColorGatePlacement;
using lumice::ComponentTable;
using lumice::CrystalConfig;
using lumice::EntryExitFilterParam;
using lumice::FilterConfig;
using lumice::MsInfo;
using lumice::NoneFilterParam;
using lumice::RaypathColorConfig;
using lumice::RaypathColorRef;
using lumice::RaypathFilterParam;
using lumice::ScatteringSetting;
using lumice::SceneConfig;
using lumice::SimpleFilterParam;

// Build one ScatteringSetting whose crystal_.id_ is caller-visible (the key
// Design 2 refs use); filter is a placeholder (BuildColorGateTable does NOT
// read filter_ — it only checks crystal_id ambiguity).
ScatteringSetting MakeSetting(lumice::IdType crystal_id) {
  ScatteringSetting s{};
  s.filter_.id_ = 0;
  s.filter_.symmetry_ = FilterConfig::kSymNone;
  s.filter_.action_ = FilterConfig::kFilterIn;
  s.filter_.param_ = SimpleFilterParam{ NoneFilterParam{} };
  s.crystal_ = CrystalConfig{};
  s.crystal_.id_ = crystal_id;
  s.crystal_proportion_ = 1.0f;
  return s;
}

SceneConfig MakeScene(const std::vector<std::vector<lumice::IdType>>& layers_crystal_ids) {
  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  for (const auto& ids : layers_crystal_ids) {
    MsInfo ms{};
    ms.prob_ = 0.5f;
    for (auto id : ids) {
      ms.setting_.push_back(MakeSetting(id));
    }
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

RaypathColorRef MakeRef(lumice::IdType layer, lumice::IdType crystal_id,
                        SimpleFilterParam predicate = NoneFilterParam{}, uint8_t symmetry = FilterConfig::kSymNone) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal_id;
  r.predicate_ = std::move(predicate);
  r.symmetry_ = symmetry;
  return r;
}

ColorClassConfig MakeClass(std::vector<RaypathColorRef> match) {
  ColorClassConfig c{};
  c.color_[0] = 1.0f;
  c.match_ = std::move(match);
  return c;
}

}  // namespace

// ---- bit allocation ----

TEST(BuildColorGateTable, EmptyConfigYieldsEmptyTable) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  auto table = lumice::BuildColorGateTable(cfg, scene);
  EXPECT_TRUE(table.entries_.empty());  // AC3: no `raypath_color` → zero bits.
}

TEST(BuildColorGateTable, SinglePredicateGetsOneBit) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 1u);
  EXPECT_EQ(table.entries_[0].layer_, 0);
  EXPECT_EQ(table.entries_[0].crystal_id_, 1);
  EXPECT_EQ(table.entries_[0].bit_, 0u);
}

TEST(BuildColorGateTable, SamePlacementDifferentPredicatesGetDifferentBits) {
  SceneConfig scene = MakeScene({ { 1 } });
  EntryExitFilterParam ee2{};
  ee2.min_len_ = 2;
  ee2.max_len_ = 2;
  EntryExitFilterParam ee3{};
  ee3.min_len_ = 3;
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, SimpleFilterParam{ ee2 }) }));
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, SimpleFilterParam{ ee3 }) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 2u);
  EXPECT_EQ(table.entries_[0].bit_, 0u);
  EXPECT_EQ(table.entries_[1].bit_, 1u);
}

TEST(BuildColorGateTable, StructurallyEqualPredicatesCollapseToOneBit) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  // Same placement (layer 0, crystal 1) + same predicate (default match-all):
  // second ref must reuse the first's bit.
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1) }));
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 1u);
  EXPECT_EQ(table.entries_[0].bit_, 0u);
}

// AC2: two crystals with SAME predicate but DIFFERENT (layer, crystal_id) keep
// distinct bits — layer key must actually distinguish placements.
TEST(BuildColorGateTable, LayerKeyDistinguishesPlacementsEvenForEqualPredicates) {
  SceneConfig scene = MakeScene({ { 1 }, { 1 } });
  RaypathColorConfig cfg;
  RaypathFilterParam rp{ /*raypath=*/{ 3, 5, 1 } };
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, SimpleFilterParam{ rp }),  //
                                     MakeRef(1, 1, SimpleFilterParam{ rp }) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 2u);
  EXPECT_NE(table.entries_[0].bit_, table.entries_[1].bit_);
  EXPECT_EQ(table.entries_[0].layer_, 0);
  EXPECT_EQ(table.entries_[1].layer_, 1);
}

TEST(BuildColorGateTable, DifferentCrystalIdsKeepDistinctBits) {
  SceneConfig scene = MakeScene({ { 1, 2 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1), MakeRef(0, 2) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 2u);
  EXPECT_NE(table.entries_[0].bit_, table.entries_[1].bit_);
}

// scrum-color-predicate-symmetry: dedup key must include symmetry so two refs
// sharing (layer, crystal_id, predicate) but differing on symmetry_ each get
// their own bit (AC2 anchor — otherwise the two color classes would resolve
// to a shared bit and cross-contaminate hits).
TEST(BuildColorGateTable, SamePredicateDifferentSymmetryGetsDistinctBits) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, NoneFilterParam{}, FilterConfig::kSymNone) }));
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, NoneFilterParam{}, FilterConfig::kSymP) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 2u);
  EXPECT_NE(table.entries_[0].bit_, table.entries_[1].bit_);
  EXPECT_EQ(table.entries_[0].symmetry_, FilterConfig::kSymNone);
  EXPECT_EQ(table.entries_[1].symmetry_, FilterConfig::kSymP);
}

// Regression anchor: dedup did not become stricter than necessary — two refs
// with identical (predicate, symmetry) still collapse to a single bit.
TEST(BuildColorGateTable, SamePredicateSameSymmetryStillCollapses) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, NoneFilterParam{}, FilterConfig::kSymP) }));
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, NoneFilterParam{}, FilterConfig::kSymP) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), 1u);
  EXPECT_EQ(table.entries_[0].bit_, 0u);
  EXPECT_EQ(table.entries_[0].symmetry_, FilterConfig::kSymP);
}

// ---- Design 2 §3.2 decision 2(b): throw-on-ambiguity ----

TEST(BuildColorGateTable, ThrowsWhenCrystalIdNotFoundOnLayer) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 99) }));
  EXPECT_THROW(lumice::BuildColorGateTable(cfg, scene), std::invalid_argument);
}

TEST(BuildColorGateTable, ThrowsWhenLayerOutOfRange) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(5, 1) }));
  EXPECT_THROW(lumice::BuildColorGateTable(cfg, scene), std::invalid_argument);
}

TEST(BuildColorGateTable, ThrowsWhenCrystalIdMatchesMultipleSettingsOnLayer) {
  // Two scattering settings on the same layer sharing crystal_id=1 — this is
  // the ambiguity Design 2 rules out (§4.0). Config authors must use distinct
  // ids (geometry may still be copied).
  SceneConfig scene = MakeScene({ { 1, 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1) }));
  EXPECT_THROW(lumice::BuildColorGateTable(cfg, scene), std::invalid_argument);
}

// ---- kNoBit overflow (soft cap = ComponentTable::kMaxBits = 64) ----

TEST(BuildColorGateTable, PredicatesBeyondBudgetGetKNoBit) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  ColorClassConfig cls;
  cls.color_[0] = 1.0f;
  // 65 unique predicates on the same placement — first 64 get bits 0..63,
  // 65th gets kNoBit (sentinel, no crash).
  for (size_t k = 0; k < ComponentTable::kMaxBits + 1; ++k) {
    EntryExitFilterParam ee{};
    ee.min_len_ = static_cast<size_t>(k + 1);  // structurally unique per k
    cls.match_.push_back(MakeRef(0, 1, SimpleFilterParam{ ee }));
  }
  cfg.classes_.push_back(std::move(cls));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(table.entries_.size(), ComponentTable::kMaxBits + 1);
  EXPECT_EQ(table.entries_[ComponentTable::kMaxBits].bit_, ComponentTable::kNoBit);
}

// ---- pure-function idempotence ----
// Server.cpp CommitConfig and the CPU emit gate (Simulator::Run) each call
// BuildColorGateTable independently on the same (config, scene) input.
// Both callers MUST get bit-identical output — otherwise `ColorClassTable`
// consumed by the consumer would resolve to different bits than the emit
// gate produces, and no user-visible test would catch it.

TEST(BuildColorGateTable, TwoIndependentCallsProduceEqualTables) {
  SceneConfig scene = MakeScene({ { 1, 2 } });
  RaypathColorConfig cfg;
  EntryExitFilterParam ee{};
  ee.min_len_ = 2;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, SimpleFilterParam{ ee }), MakeRef(0, 2) }));
  auto a = lumice::BuildColorGateTable(cfg, scene);
  auto b = lumice::BuildColorGateTable(cfg, scene);
  ASSERT_EQ(a.entries_.size(), b.entries_.size());
  for (size_t i = 0; i < a.entries_.size(); ++i) {
    EXPECT_EQ(a.entries_[i].layer_, b.entries_[i].layer_);
    EXPECT_EQ(a.entries_[i].crystal_id_, b.entries_[i].crystal_id_);
    EXPECT_EQ(a.entries_[i].bit_, b.entries_[i].bit_);
    EXPECT_TRUE(a.entries_[i].predicate_ == b.entries_[i].predicate_);
  }
}

// ---- ColorGatePlacementFor ----

TEST(ColorGatePlacementFor, ReturnsInsertionOrderPredicatesAndBits) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  EntryExitFilterParam ee2{};
  ee2.min_len_ = 2;
  EntryExitFilterParam ee3{};
  ee3.min_len_ = 3;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1, SimpleFilterParam{ ee2 }),  //
                                     MakeRef(0, 1, SimpleFilterParam{ ee3 }) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ColorGatePlacement p = lumice::ColorGatePlacementFor(table, 0, 1);
  ASSERT_EQ(p.predicates_.size(), 2u);
  ASSERT_EQ(p.bits_.size(), 2u);
  ASSERT_EQ(p.symmetries_.size(), 2u);
  EXPECT_EQ(p.bits_[0], 0u);
  EXPECT_EQ(p.bits_[1], 1u);
  EXPECT_EQ(p.symmetries_[0], FilterConfig::kSymNone);
  EXPECT_EQ(p.symmetries_[1], FilterConfig::kSymNone);
}

// scrum-color-predicate-symmetry: placement view carries per-ref symmetry_
// parallel to predicates_/bits_. Preserves insertion order across the third
// parallel array.
TEST(ColorGatePlacementFor, ReturnsSymmetriesParallelToBits) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  EntryExitFilterParam ee2{};
  ee2.min_len_ = 2;
  EntryExitFilterParam ee3{};
  ee3.min_len_ = 3;
  cfg.classes_.push_back(MakeClass({
      MakeRef(0, 1, SimpleFilterParam{ ee2 }, FilterConfig::kSymP),
      MakeRef(0, 1, SimpleFilterParam{ ee3 }, FilterConfig::kSymB),
  }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ColorGatePlacement p = lumice::ColorGatePlacementFor(table, 0, 1);
  ASSERT_EQ(p.symmetries_.size(), 2u);
  EXPECT_EQ(p.symmetries_[0], FilterConfig::kSymP);
  EXPECT_EQ(p.symmetries_[1], FilterConfig::kSymB);
}

TEST(ColorGatePlacementFor, MissingPlacementReturnsEmpty) {
  SceneConfig scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(MakeClass({ MakeRef(0, 1) }));
  auto table = lumice::BuildColorGateTable(cfg, scene);
  ColorGatePlacement p = lumice::ColorGatePlacementFor(table, /*layer=*/5, /*crystal_id=*/99);
  EXPECT_TRUE(p.predicates_.empty());
  EXPECT_TRUE(p.bits_.empty());
}

// ---- GroupPlacementBySymmetry ----
//
// task-358.1 code-review round 2: this is the single shared authority both
// CPU (BuildColorSpecGroups, filter_spec.cpp) and Metal
// (MetalTraceBackend::Impl::EnsureFilterBuffers) call for the "group
// predicates by first-occurrence symmetry" step — see test_color_spec_groups.cpp
// for the CPU-side FilterSpec-building test that now exercises this same
// function transitively. These tests lock the grouping contract itself
// (order, group_of_ indices) directly against ColorGatePlacement inputs.

TEST(GroupPlacementBySymmetry, EmptyPlacementYieldsNoGroups) {
  ColorGatePlacement p;
  auto g = lumice::GroupPlacementBySymmetry(p);
  EXPECT_TRUE(g.group_symmetry_.empty());
  EXPECT_TRUE(g.group_of_.empty());
}

TEST(GroupPlacementBySymmetry, AllSameSymmetryCollapsesToOneGroup) {
  ColorGatePlacement p;
  p.predicates_ = { NoneFilterParam{}, NoneFilterParam{}, NoneFilterParam{} };
  p.bits_ = { 0, 1, 2 };
  p.symmetries_ = { FilterConfig::kSymP, FilterConfig::kSymP, FilterConfig::kSymP };
  auto g = lumice::GroupPlacementBySymmetry(p);
  ASSERT_EQ(g.group_symmetry_.size(), 1u);
  EXPECT_EQ(g.group_symmetry_[0], FilterConfig::kSymP);
  ASSERT_EQ(g.group_of_.size(), 3u);
  EXPECT_EQ(g.group_of_[0], 0u);
  EXPECT_EQ(g.group_of_[1], 0u);
  EXPECT_EQ(g.group_of_[2], 0u);
}

// First-occurrence order (not sorted, not stable-by-value): the group list
// must list each distinct symmetry the first time it's seen while walking
// predicates_ in placement order, and group_of_ must route every predicate
// with a repeated symmetry back to that same earlier group index — this is
// the exact ordering discipline that used to be duplicated (and only
// comment-enforced) between CPU and Metal.
TEST(GroupPlacementBySymmetry, MixedSymmetriesGroupByFirstOccurrenceOrder) {
  ColorGatePlacement p;
  p.predicates_ = { NoneFilterParam{}, NoneFilterParam{}, NoneFilterParam{}, NoneFilterParam{} };
  p.bits_ = { 0, 1, 2, 3 };
  // Order: B, P, B, D — expect groups in first-occurrence order [B, P, D].
  p.symmetries_ = { FilterConfig::kSymB, FilterConfig::kSymP, FilterConfig::kSymB, FilterConfig::kSymD };
  auto g = lumice::GroupPlacementBySymmetry(p);
  ASSERT_EQ(g.group_symmetry_.size(), 3u);
  EXPECT_EQ(g.group_symmetry_[0], FilterConfig::kSymB);
  EXPECT_EQ(g.group_symmetry_[1], FilterConfig::kSymP);
  EXPECT_EQ(g.group_symmetry_[2], FilterConfig::kSymD);
  ASSERT_EQ(g.group_of_.size(), 4u);
  EXPECT_EQ(g.group_of_[0], 0u);  // B -> group 0
  EXPECT_EQ(g.group_of_[1], 1u);  // P -> group 1
  EXPECT_EQ(g.group_of_[2], 0u);  // B (repeat) -> group 0
  EXPECT_EQ(g.group_of_[3], 2u);  // D -> group 2
}

// Regression anchor for the code-review round-2 Major: a placement whose
// distinct-symmetry count exceeds Metal's kColorMaxGroupsPerSlot budget (4)
// must still be grouped correctly by this shared function — the overflow
// guard lives in MetalTraceBackend::Impl::EnsureFilterBuffers (clamped +
// logged there), not here; this function itself has no group-count cap.
TEST(GroupPlacementBySymmetry, MoreThanFourDistinctSymmetriesGroupCorrectly) {
  ColorGatePlacement p;
  // 5 distinct symmetry bitmask values (kSymNone=0 plus 4 more bit patterns),
  // one predicate each.
  const std::vector<uint8_t> kSyms = { 0, FilterConfig::kSymP, FilterConfig::kSymB, FilterConfig::kSymD,
                                       static_cast<uint8_t>(FilterConfig::kSymP | FilterConfig::kSymB) };
  for (size_t k = 0; k < kSyms.size(); ++k) {
    p.predicates_.emplace_back(NoneFilterParam{});
    p.bits_.push_back(static_cast<uint8_t>(k));
    p.symmetries_.push_back(kSyms[k]);
  }
  auto g = lumice::GroupPlacementBySymmetry(p);
  ASSERT_EQ(g.group_symmetry_.size(), 5u);
  for (size_t k = 0; k < kSyms.size(); ++k) {
    EXPECT_EQ(g.group_symmetry_[k], kSyms[k]);
    EXPECT_EQ(g.group_of_[k], k);  // every predicate is its own group (all distinct)
  }
}
