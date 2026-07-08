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
                        SimpleFilterParam predicate = NoneFilterParam{}) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal_id;
  r.predicate_ = std::move(predicate);
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
  EXPECT_EQ(p.bits_[0], 0u);
  EXPECT_EQ(p.bits_[1], 1u);
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
