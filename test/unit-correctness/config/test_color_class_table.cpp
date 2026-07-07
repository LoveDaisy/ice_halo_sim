// Tests for src/config/color_class_table.hpp — Design 2 (2026-07-08,
// doc/gui-custom-spectrum-and-raypath-color.md §4.0) BuildColorClassTable
// contract on the join of DTO × SceneConfig × ColorGateTable.
//
// After Design 2, ambiguity / OOB errors have moved to BuildColorGateTable
// (test_color_gate_table.cpp covers them). BuildColorClassTable now consumes
// a pre-built gate table and translates each Design-2 predicate ref into its
// assigned bit. This test suite covers:
//   - single / multi-ref any → OR union
//   - cross-layer combine=all → distinct bits
//   - match-all whole-crystal (AC5: none-filter equivalence via default
//     NoneFilterParam predicate)
//   - combine defaults / unknown combine → throw
//   - empty raypath_color → empty table (AC3 anchor)
//   - kNoBit overflow → skipped, class retained with 0 bits, no throw
//   - explicit empty match → class retained with 0 bits

#include <gtest/gtest.h>

#include <bitset>
#include <stdexcept>
#include <utility>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/color_gate_table.hpp"
#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "core/def.hpp"

namespace {

using lumice::BuildColorClassTable;
using lumice::BuildColorGateTable;
using lumice::ColorClassCombine;
using lumice::ColorClassConfig;
using lumice::ColorClassTable;
using lumice::ComponentTable;
using lumice::CrystalConfig;
using lumice::EntryExitFilterParam;
using lumice::FilterConfig;
using lumice::IdType;
using lumice::MsInfo;
using lumice::NoneFilterParam;
using lumice::RaypathColorConfig;
using lumice::RaypathColorRef;
using lumice::ScatteringSetting;
using lumice::SceneConfig;
using lumice::SimpleFilterParam;

ScatteringSetting MakeSetting(IdType crystal_id) {
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

SceneConfig MakeScene(const std::vector<std::vector<IdType>>& layers_crystal_ids) {
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

RaypathColorRef Ref(uint16_t layer, uint16_t crystal, SimpleFilterParam predicate = NoneFilterParam{}) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.predicate_ = std::move(predicate);
  return r;
}

ColorClassConfig Class(float r, float g, float b, std::vector<RaypathColorRef> match) {
  ColorClassConfig c{};
  c.color_[0] = r;
  c.color_[1] = g;
  c.color_[2] = b;
  c.match_ = std::move(match);
  return c;
}

}  // namespace

// ---- single ref → single bit, combine=any (default) ----

TEST(BuildColorClassTable, SingleRefResolvesToSingleBit) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1) }));  // whole-crystal match-all
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAny);
  EXPECT_EQ(ct.classes_[0].member_bits_, static_cast<uint64_t>(1) << 0);
  EXPECT_EQ(ct.referenced_mask_, static_cast<uint64_t>(1) << 0);
}

// ---- multi-ref any → OR-union ----

TEST(BuildColorClassTable, MultiRefAnyIsOrUnion) {
  auto scene = MakeScene({ { 1 } });
  EntryExitFilterParam ee2{};
  ee2.min_len_ = 2;
  ee2.max_len_ = 2;
  EntryExitFilterParam ee3{};
  ee3.min_len_ = 3;
  RaypathColorConfig cfg;
  cfg.classes_.push_back(
      Class(0.5f, 0.5f, 0.5f, { Ref(0, 1, SimpleFilterParam{ ee2 }), Ref(0, 1, SimpleFilterParam{ ee3 }) }));
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAny);
  EXPECT_EQ(ct.classes_[0].member_bits_, (static_cast<uint64_t>(1) << 0) | (static_cast<uint64_t>(1) << 1));
}

// ---- multi-layer combine=all → cross-layer AND, distinct bits (AC2 anchor) ----

TEST(BuildColorClassTable, CrossLayerAllBitsAreDistinct) {
  auto scene = MakeScene({ { 1 }, { 2 } });
  ColorClassConfig cls = Class(1.0f, 1.0f, 1.0f, { Ref(0, 1), Ref(1, 2) });
  cls.combine_ = "all";
  RaypathColorConfig cfg;
  cfg.classes_.push_back(cls);
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAll);
  uint64_t bits = ct.classes_[0].member_bits_;
  EXPECT_EQ(std::bitset<64>(bits).count(), 2u);
  EXPECT_EQ(bits, (static_cast<uint64_t>(1) << 0) | (static_cast<uint64_t>(1) << 1));
}

// AC5: match-all whole-crystal ref (default NoneFilterParam predicate) is
// how Design 2 expresses "染整颗晶体" — replaces the 339.1 none-filter special
// case. Should resolve to a real bit, be non-zero, and drive predicate-fires.

TEST(BuildColorClassTable, MatchAllWholeCrystalRefResolvesToBit) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1) }));
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].member_bits_, static_cast<uint64_t>(1) << 0);
}

// ---- combine default = "any" ----

TEST(BuildColorClassTable, CombineDefaultIsAny) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1) }));
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAny);
}

// ---- unknown combine → throw ----

TEST(BuildColorClassTable, UnknownCombineThrows) {
  auto scene = MakeScene({ { 1 } });
  ColorClassConfig cls = Class(1.0f, 0.0f, 0.0f, { Ref(0, 1) });
  cls.combine_ = "xor";
  RaypathColorConfig cfg;
  cfg.classes_.push_back(cls);
  auto gate = BuildColorGateTable(cfg, scene);
  EXPECT_THROW(BuildColorClassTable(cfg, scene, gate), std::invalid_argument);
}

// ---- empty raypath_color → empty table (AC3 anchor) ----

TEST(BuildColorClassTable, EmptyConfigYieldsEmptyTable) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig empty;
  auto gate = BuildColorGateTable(empty, scene);
  auto ct = BuildColorClassTable(empty, scene, gate);
  EXPECT_TRUE(ct.classes_.empty());
  EXPECT_EQ(ct.referenced_mask_, 0u);
}

// ---- kNoBit overflow → skipped, class retained with 0 bits, no throw ----

TEST(BuildColorClassTable, KNoBitOverflowClassRetainedWithZeroBits) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  // 65 unique predicates on the same placement — 65th overflows (kNoBit).
  ColorClassConfig overflow_cls;
  overflow_cls.color_[0] = 1.0f;
  for (size_t k = 0; k < ComponentTable::kMaxBits + 1; ++k) {
    EntryExitFilterParam ee{};
    ee.min_len_ = static_cast<size_t>(k + 1);
    overflow_cls.match_.push_back(Ref(0, 1, SimpleFilterParam{ ee }));
  }
  cfg.classes_.push_back(std::move(overflow_cls));
  auto gate = BuildColorGateTable(cfg, scene);
  ColorClassTable ct;
  EXPECT_NO_THROW(ct = BuildColorClassTable(cfg, scene, gate));
  ASSERT_EQ(ct.classes_.size(), 1u);
  // 64 real bits + 1 skipped kNoBit — member_bits_ has 64 bits set.
  EXPECT_EQ(std::bitset<64>(ct.classes_[0].member_bits_).count(), ComponentTable::kMaxBits);
}

// ---- explicit empty match → class retained with 0 bits ----

TEST(BuildColorClassTable, ExplicitEmptyMatchClassKeptWithZeroBits) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, {}));             // empty match
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1) }));  // valid bit 0
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 2u);
  EXPECT_EQ(ct.classes_[0].member_bits_, 0u);
  EXPECT_EQ(ct.classes_[1].member_bits_, static_cast<uint64_t>(1) << 0);
  EXPECT_EQ(ct.referenced_mask_, static_cast<uint64_t>(1) << 0);
}
