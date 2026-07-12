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
#include "util/bit_utils.hpp"

namespace {

using lumice::BuildColorClassTable;
using lumice::BuildColorGateTable;
using lumice::ColorClass;
using lumice::ColorClassCombine;
using lumice::ColorClassConfig;
using lumice::ColorClassTable;
using lumice::ComponentTable;
using lumice::CrystalConfig;
using lumice::EntryExitFilterParam;
using lumice::FilterConfig;
using lumice::IdType;
using lumice::MsInfo;
using lumice::NeedsRebuild;
using lumice::NoneFilterParam;
using lumice::PopCount;
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

RaypathColorRef Ref(uint16_t layer, uint16_t crystal, SimpleFilterParam predicate = NoneFilterParam{},
                    uint8_t symmetry = FilterConfig::kSymNone) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.predicate_ = std::move(predicate);
  r.symmetry_ = symmetry;
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
  EXPECT_EQ(static_cast<size_t>(PopCount(bits)), 2u);
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
  EXPECT_EQ(static_cast<size_t>(PopCount(ct.classes_[0].member_bits_)), ComponentTable::kMaxBits);
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

// scrum-color-predicate-symmetry AC2: two refs sharing predicate but differing
// symmetry_ must each resolve to their own bit — the class-table resolver's
// lookup key includes symmetry (else the second ref would silently inherit
// the first's bit, cross-contaminating two semantically distinct color
// classes).
TEST(BuildColorClassTable, SamePredicateDifferentSymmetryResolvesToOwnBit) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, NoneFilterParam{}, FilterConfig::kSymNone) }));
  cfg.classes_.push_back(Class(0.0f, 0.0f, 1.0f, { Ref(0, 1, NoneFilterParam{}, FilterConfig::kSymP) }));
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 2u);
  // Each class references a single bit; the two bits are disjoint.
  EXPECT_EQ(static_cast<size_t>(PopCount(ct.classes_[0].member_bits_)), 1u);
  EXPECT_EQ(static_cast<size_t>(PopCount(ct.classes_[1].member_bits_)), 1u);
  EXPECT_EQ(ct.classes_[0].member_bits_ & ct.classes_[1].member_bits_, 0u);
  // referenced_mask is the union.
  EXPECT_EQ(ct.referenced_mask_, ct.classes_[0].member_bits_ | ct.classes_[1].member_bits_);
}

// ---- z_order (task-342.2) ----

// BuildColorClassTable seeds each class's z_order_ to its list position, so a config with no
// explicit z-order draws in list order — identical to pre-342.2 behavior.
TEST(BuildColorClassTable, AssignsSequentialZOrder) {
  auto scene = MakeScene({ { 1 } });
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1) }));
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1) }));
  cfg.classes_.push_back(Class(0.0f, 0.0f, 1.0f, { Ref(0, 1) }));
  auto gate = BuildColorGateTable(cfg, scene);
  auto ct = BuildColorClassTable(cfg, scene, gate);
  ASSERT_EQ(ct.classes_.size(), 3u);
  EXPECT_EQ(ct.classes_[0].z_order_, 0);
  EXPECT_EQ(ct.classes_[1].z_order_, 1);
  EXPECT_EQ(ct.classes_[2].z_order_, 2);
}

// NeedsRebuild must ignore z_order_ (it is a display-time field like color_/visible_/solo_).
// This is the direct guarantee that SetRaypathColors' z-order change never triggers a
// consumer/lane rebuild.
TEST(NeedsRebuild, IgnoresZOrder) {
  ColorClassTable a;
  ColorClass c0{};
  c0.combine_ = ColorClassCombine::kAny;
  c0.member_bits_ = 0b01;
  c0.z_order_ = 0;
  ColorClass c1{};
  c1.combine_ = ColorClassCombine::kAny;
  c1.member_bits_ = 0b10;
  c1.z_order_ = 1;
  a.classes_ = { c0, c1 };

  // Same combine_/member_bits_, only z_order_ (and color/visible/solo) differ.
  ColorClassTable b = a;
  b.classes_[0].z_order_ = 5;
  b.classes_[1].z_order_ = 3;
  b.classes_[0].color_[0] = 0.9f;
  b.classes_[0].visible_ = false;
  b.classes_[1].solo_ = true;
  EXPECT_FALSE(NeedsRebuild(a, b));

  // Sanity: a real member_bits_ change DOES require a rebuild.
  ColorClassTable c = a;
  c.classes_[0].member_bits_ = 0b100;
  EXPECT_TRUE(NeedsRebuild(a, c));
}
