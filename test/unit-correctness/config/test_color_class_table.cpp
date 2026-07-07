// Tests for src/config/color_class_table.hpp — BuildColorClassTable's
// contract on the join of the task-339.2 color-class DTO ×
// SceneConfig (id → ci resolution) × ComponentTable. Covers the §6 test
// matrix (single ref / multi-ref any / cross-layer all / none-filter ref /
// combine defaults + errors / degenerate scenes / kNoBit overflow / omit-
// summand union / interim legacy adapter equivalence / empty-union class
// retention).

#include <gtest/gtest.h>

#include <stdexcept>
#include <utility>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/component_color_map.hpp"
#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "core/def.hpp"
#include "server/component_compositor.hpp"

namespace {

using lumice::BuildColorClassTable;
using lumice::BuildComponentTable;
using lumice::ColorClassCombine;
using lumice::ColorClassConfig;
using lumice::ColorClassTable;
using lumice::ComplexFilterParam;
using lumice::CompositeMode;
using lumice::CrystalConfig;
using lumice::CrystalFilterParam;
using lumice::FilterConfig;
using lumice::FilterParam;
using lumice::IdType;
using lumice::MsInfo;
using lumice::NoneFilterParam;
using lumice::RaypathColorConfig;
using lumice::RaypathColorRef;
using lumice::RaypathFilterParam;
using lumice::ScatteringSetting;
using lumice::SceneConfig;
using lumice::SimpleFilterParam;
using lumice::ToLegacyColorMap;
using lumice::ToLegacyCompositeOptions;

ScatteringSetting MakeSetting(IdType crystal_id, IdType filter_id, FilterParam param) {
  ScatteringSetting s{};
  s.filter_.id_ = filter_id;
  s.filter_.symmetry_ = FilterConfig::kSymNone;
  s.filter_.action_ = FilterConfig::kFilterIn;
  s.filter_.param_ = std::move(param);
  s.crystal_ = CrystalConfig{};
  s.crystal_.id_ = crystal_id;
  s.crystal_proportion_ = 1.0f;
  return s;
}

ScatteringSetting MakeNoneSetting(IdType crystal_id) {
  return MakeSetting(crystal_id, lumice::kInvalidId, SimpleFilterParam{ NoneFilterParam{} });
}

// The three-arcs scene (fixture-parallel): one MS layer with two settings —
// (crystal=1, filter=1) simple raypath (1 summand) at ci=0, then
// (crystal=1, filter=4) complex with 2 OR-summands at ci=1.
SceneConfig MakeThreeArcsScene() {
  RaypathFilterParam raypath;
  raypath.raypath_ = { 3, 5 };

  ComplexFilterParam complex;
  complex.filters_.resize(2);
  complex.filters_[0].emplace_back(IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  complex.filters_[1].emplace_back(IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 1 } });

  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  MsInfo ms{};
  ms.prob_ = 0.5f;
  ms.setting_.push_back(MakeSetting(1, 1, SimpleFilterParam{ raypath }));
  ms.setting_.push_back(MakeSetting(1, 4, FilterParam{ complex }));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

RaypathColorRef Ref(uint16_t layer, uint16_t crystal, bool has_filter, uint16_t filter, bool has_summand,
                    uint16_t summand) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.has_filter_ = has_filter;
  r.filter_ = filter;
  r.has_summand_ = has_summand;
  r.summand_ = summand;
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

// ---- §6 case 1: single ref → single bit, combine=any (default) ----

TEST(BuildColorClassTable, SingleRefResolvesToSingleBit) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 3u);

  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 4, true, 0) }));

  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAny);
  // ci=1, summand=0 → bit 1 (bit 0 belongs to ci=0's single summand).
  EXPECT_EQ(ct.classes_[0].member_bits_, static_cast<uint64_t>(1) << 1);
  EXPECT_EQ(ct.referenced_mask_, static_cast<uint64_t>(1) << 1);
}

// ---- §6 case 2: multi-ref any → OR-union ----

TEST(BuildColorClassTable, MultiRefAnyIsOrUnion) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);

  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(0.5f, 0.5f, 0.5f, { Ref(0, 1, true, 4, true, 0), Ref(0, 1, true, 4, true, 1) }));

  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAny);
  // ci=1 → bits 1 and 2.
  EXPECT_EQ(ct.classes_[0].member_bits_, (static_cast<uint64_t>(1) << 1) | (static_cast<uint64_t>(1) << 2));
}

// ---- §6 case 3: multi-layer combine=all → cross-layer AND, distinct bits ----

TEST(BuildColorClassTable, CrossLayerAllBitsAreDistinct) {
  // Layer 0: crystal=1 filter=10 simple raypath (ci=0, bit 0)
  // Layer 1: crystal=2 filter=20 simple raypath (ci=0, bit 1)
  RaypathFilterParam raypath;
  raypath.raypath_ = { 3, 5 };

  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  MsInfo ms0{};
  ms0.prob_ = 0.5f;
  ms0.setting_.push_back(MakeSetting(1, 10, SimpleFilterParam{ raypath }));
  scene.ms_.push_back(std::move(ms0));
  MsInfo ms1{};
  ms1.prob_ = 0.5f;
  ms1.setting_.push_back(MakeSetting(2, 20, SimpleFilterParam{ raypath }));
  scene.ms_.push_back(std::move(ms1));

  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 2u);

  ColorClassConfig cls = Class(1.0f, 1.0f, 1.0f, { Ref(0, 1, true, 10, false, 0), Ref(1, 2, true, 20, false, 0) });
  cls.combine_ = "all";
  RaypathColorConfig cfg;
  cfg.classes_.push_back(cls);

  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAll);
  const uint64_t bits = ct.classes_[0].member_bits_;
  EXPECT_EQ(__builtin_popcountll(bits), 2);
  EXPECT_EQ(bits, (static_cast<uint64_t>(1) << 0) | (static_cast<uint64_t>(1) << 1));
}

// ---- §6 case 4: none-filter {layer, crystal} ref ----

TEST(BuildColorClassTable, NoneFilterRefResolvesToWholeCrystalBit) {
  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  MsInfo ms{};
  ms.prob_ = 0.5f;
  ms.setting_.push_back(MakeNoneSetting(1));  // whole-crystal, one summand
  scene.ms_.push_back(std::move(ms));

  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 1u);

  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1, false, 0, false, 0) }));

  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].member_bits_, static_cast<uint64_t>(1) << 0);
}

// ---- §6 case 5: combine default = "any" ----

TEST(BuildColorClassTable, CombineDefaultIsAny) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 1, false, 0) }));
  // combine_ left default "any" in ColorClassConfig.
  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 1u);
  EXPECT_EQ(ct.classes_[0].combine_, ColorClassCombine::kAny);
}

// ---- §6 case 6: unknown combine → throw ----

TEST(BuildColorClassTable, UnknownCombineThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  ColorClassConfig cls = Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 1, false, 0) });
  cls.combine_ = "xor";
  RaypathColorConfig cfg;
  cfg.classes_.push_back(cls);
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

// ---- §6 case 7: degenerate duplicate (crystal,filter) → throw ----

TEST(BuildColorClassTable, DegenerateDuplicateCrystalFilterPairThrows) {
  RaypathFilterParam raypath;
  raypath.raypath_ = { 3, 5 };
  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  MsInfo ms{};
  ms.prob_ = 0.5f;
  ms.setting_.push_back(MakeSetting(1, 7, SimpleFilterParam{ raypath }));
  ms.setting_.push_back(MakeSetting(1, 7, SimpleFilterParam{ raypath }));  // duplicate pair
  scene.ms_.push_back(std::move(ms));

  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 7, false, 0) }));
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

// ---- §6 case 8: OOB / not-found refs → throw ----

TEST(BuildColorClassTable, LayerOutOfRangeThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(9, 1, true, 1, false, 0) }));
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

TEST(BuildColorClassTable, UnknownCrystalIdThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 99, true, 1, false, 0) }));
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

TEST(BuildColorClassTable, UnknownFilterIdThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 99, false, 0) }));
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

TEST(BuildColorClassTable, ExplicitSummandOutOfRangeThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  // filter=1 is simple → only 1 summand; summand=1 is out of range.
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 1, true, 1) }));
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

// ---- §6 case 9: empty raypath_color → empty table ----

TEST(BuildColorClassTable, EmptyConfigYieldsEmptyTable) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig empty;
  auto ct = BuildColorClassTable(empty, scene, table);
  EXPECT_TRUE(ct.classes_.empty());
  EXPECT_EQ(ct.referenced_mask_, 0u);
}

// ---- §6 case 10: omit-summand on multi-summand filter → OR-union ----

TEST(BuildColorClassTable, OmitSummandOnComplexFilterUnionsAllBits) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 1.0f, 0.0f, { Ref(0, 1, true, 4, false, 0) }));
  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 1u);
  // ci=1 → bits 1 and 2, unioned.
  EXPECT_EQ(ct.classes_[0].member_bits_, (static_cast<uint64_t>(1) << 1) | (static_cast<uint64_t>(1) << 2));
}

// ---- §6 case 11: combine=all + omit-summand producing >1 bit → throw (ban) ----

TEST(BuildColorClassTable, CombineAllWithOmitSummandMultiBitRefThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  ColorClassConfig cls = Class(1.0f, 1.0f, 0.0f, { Ref(0, 1, true, 4, false, 0) });
  cls.combine_ = "all";
  RaypathColorConfig cfg;
  cfg.classes_.push_back(cls);
  EXPECT_THROW(BuildColorClassTable(cfg, scene, table), std::invalid_argument);
}

// ---- §6 case 12: kNoBit overflow → warn+skip, no throw ----

TEST(BuildColorClassTable, KNoBitOverflowIsSkipped) {
  constexpr size_t kTotalSummands = 70;
  ComplexFilterParam complex;
  complex.filters_.resize(kTotalSummands);
  for (size_t i = 0; i < kTotalSummands; i++) {
    complex.filters_[i].emplace_back(IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  }
  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  MsInfo ms{};
  ms.prob_ = 0.5f;
  ms.setting_.push_back(MakeSetting(1, 1, FilterParam{ complex }));
  scene.ms_.push_back(std::move(ms));

  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), kTotalSummands);

  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 1, true, 65) }));  // overflow bit
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1, true, 1, true, 3) }));   // in-cap bit

  ColorClassTable ct;
  EXPECT_NO_THROW(ct = BuildColorClassTable(cfg, scene, table));
  ASSERT_EQ(ct.classes_.size(), 2u);
  EXPECT_EQ(ct.classes_[0].member_bits_, 0u);  // overflow class kept, empty
  EXPECT_EQ(ct.classes_[1].member_bits_, static_cast<uint64_t>(1) << 3);
  EXPECT_EQ(ct.referenced_mask_, static_cast<uint64_t>(1) << 3);
}

// ---- §6 case 13: legacy adapter equivalence (single-member classes) ----

TEST(ToLegacyColorMap, SingleMemberClassEquivalentToPerBit) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 1, false, 0) }));  // bit 0
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1, true, 4, true, 0) }));   // bit 1
  cfg.classes_.push_back(Class(0.0f, 0.0f, 1.0f, { Ref(0, 1, true, 4, true, 1) }));   // bit 2

  auto ct = BuildColorClassTable(cfg, scene, table);
  auto map = ToLegacyColorMap(ct);
  EXPECT_EQ(map.colored_mask_, 0b111u);
  EXPECT_FLOAT_EQ(map.colors_[0][0], 1.0f);
  EXPECT_FLOAT_EQ(map.colors_[1][1], 1.0f);
  EXPECT_FLOAT_EQ(map.colors_[2][2], 1.0f);
}

TEST(ToLegacyCompositeOptions, VisibleSoloFoldIntoMasks) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  ColorClassConfig cls_hidden = Class(1.0f, 0.0f, 0.0f, { Ref(0, 1, true, 1, false, 0) });
  cls_hidden.visible_ = false;
  ColorClassConfig cls_solo = Class(0.0f, 1.0f, 0.0f, { Ref(0, 1, true, 4, true, 0) });
  cls_solo.solo_ = true;
  RaypathColorConfig cfg;
  cfg.classes_.push_back(cls_hidden);
  cfg.classes_.push_back(cls_solo);

  auto ct = BuildColorClassTable(cfg, scene, table);
  auto opts = ToLegacyCompositeOptions(ct, "additive");
  EXPECT_EQ(opts.mode_, CompositeMode::kAdditive);
  EXPECT_EQ(opts.hidden_mask_, static_cast<uint64_t>(1) << 0);
  EXPECT_EQ(opts.solo_mask_, static_cast<uint64_t>(1) << 1);
}

TEST(ToLegacyCompositeOptions, UnknownModeFallsBackToDominant) {
  ColorClassTable empty;
  auto opts = ToLegacyCompositeOptions(empty, "greebles");
  EXPECT_EQ(opts.mode_, CompositeMode::kDominant);
}

// ---- §6 case 14: empty-union class retained + does not contribute ----

TEST(BuildColorClassTable, ExplicitEmptyMatchClassKeptWithZeroBits) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.classes_.push_back(Class(1.0f, 0.0f, 0.0f, {}));                                // empty match
  cfg.classes_.push_back(Class(0.0f, 1.0f, 0.0f, { Ref(0, 1, true, 1, false, 0) }));  // valid bit 0

  auto ct = BuildColorClassTable(cfg, scene, table);
  ASSERT_EQ(ct.classes_.size(), 2u);
  EXPECT_EQ(ct.classes_[0].member_bits_, 0u);
  EXPECT_EQ(ct.classes_[1].member_bits_, static_cast<uint64_t>(1) << 0);
  EXPECT_EQ(ct.referenced_mask_, static_cast<uint64_t>(1) << 0);
}
