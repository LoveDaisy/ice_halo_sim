// Tests for src/config/component_color_map.hpp — BuildComponentColorMap's
// contract on the join of RaypathColorConfig × ComponentTable. Covers plan
// §4 Step 3 five branches:
//   1) three-arcs mapping (plan §3.2 anchor)
//   2) empty color config → zero mask, no throw
//   3) out-of-range triple → std::invalid_argument (fail fast)
//   4) soft-cap overflow (kNoBit) → warn + skip, other bits unaffected
//   5) duplicate triple → last-write wins

#include <gtest/gtest.h>

#include <stdexcept>
#include <utility>
#include <vector>

#include "config/component_color_map.hpp"
#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "core/def.hpp"

namespace {

using lumice::BuildComponentColorMap;
using lumice::BuildComponentTable;
using lumice::ComplexFilterParam;
using lumice::ComponentColorMap;
using lumice::ComponentTable;
using lumice::CrystalConfig;
using lumice::CrystalFilterParam;
using lumice::FilterConfig;
using lumice::MsInfo;
using lumice::RaypathColorConfig;
using lumice::RaypathColorEntry;
using lumice::RaypathFilterParam;
using lumice::ScatteringSetting;
using lumice::SceneConfig;
using lumice::SimpleFilterParam;

// Mirror the helper from test_component_table.cpp — a filterparam-only scene
// wrapper (BuildComponentTable never inspects crystal_ / crystal_proportion_).
ScatteringSetting MakeSetting(lumice::FilterParam param) {
  ScatteringSetting s{};
  s.filter_.id_ = 0;
  s.filter_.symmetry_ = FilterConfig::kSymNone;
  s.filter_.action_ = FilterConfig::kFilterIn;
  s.filter_.param_ = std::move(param);
  s.crystal_ = CrystalConfig{};
  s.crystal_proportion_ = 1.0f;
  return s;
}

SceneConfig MakeSceneWithLayers(std::vector<std::vector<lumice::FilterParam>> layers) {
  SceneConfig scene{};
  scene.ray_num_ = 1;
  scene.max_hits_ = 1;
  for (auto& layer_params : layers) {
    MsInfo ms{};
    ms.prob_ = 0.5f;
    for (auto& p : layer_params) {
      ms.setting_.push_back(MakeSetting(std::move(p)));
    }
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

RaypathColorEntry MakeEntry(uint16_t layer, uint16_t crystal_slot, uint16_t summand, float r, float g, float b) {
  RaypathColorEntry e{};
  e.layer_ = layer;
  e.crystal_id_ = crystal_slot;
  e.summand_idx_ = summand;
  e.color_[0] = r;
  e.color_[1] = g;
  e.color_[2] = b;
  return e;
}

// The plan §3.2 anchor: single MS layer, two entries. Slot 0 = Simple(raypath)
// → 1 summand. Slot 1 = Complex with 2 OR-summands.
SceneConfig MakeThreeArcsScene() {
  RaypathFilterParam raypath;
  raypath.raypath_ = { 3, 5 };

  ComplexFilterParam complex;
  complex.filters_.resize(2);
  // AND-chain contents are irrelevant to BuildComponentTable — only the outer
  // filters_.size() matters (see test_component_table.cpp for the same trick).
  complex.filters_[0].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  complex.filters_[1].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 1 } });

  return MakeSceneWithLayers({ { SimpleFilterParam{ raypath }, lumice::FilterParam{ complex } } });
}

}  // namespace

// ---- (1) three-arcs mapping ----

TEST(BuildComponentColorMap, ThreeArcsMapsExpectedBits) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  // Expect 3 bits: (0,0,0), (0,1,0), (0,1,1) — see plan §3.2 table.
  ASSERT_EQ(table.entries_.size(), 3u);

  RaypathColorConfig cfg;
  cfg.entries_.push_back(MakeEntry(0, 0, 0, 1.0f, 0.0f, 0.0f));  // 晶体1·3-5 = 红
  cfg.entries_.push_back(MakeEntry(0, 1, 0, 0.0f, 1.0f, 0.0f));  // 晶体2·3   = 绿
  cfg.entries_.push_back(MakeEntry(0, 1, 1, 0.0f, 0.0f, 1.0f));  // 晶体2·3-5 = 蓝

  auto map = BuildComponentColorMap(cfg, table);
  EXPECT_EQ(map.colored_mask_, 0b111u);
  EXPECT_FLOAT_EQ(map.colors_[0][0], 1.0f);
  EXPECT_FLOAT_EQ(map.colors_[0][1], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[0][2], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[1][0], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[1][1], 1.0f);
  EXPECT_FLOAT_EQ(map.colors_[1][2], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[2][0], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[2][1], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[2][2], 1.0f);
}

// ---- (2) empty color config ----

TEST(BuildComponentColorMap, EmptyColorConfigYieldsZeroMask) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig empty;
  auto map = BuildComponentColorMap(empty, table);
  EXPECT_EQ(map.colored_mask_, 0u);
}

TEST(BuildComponentColorMap, EmptyTableAndEmptyConfigDoesNotThrow) {
  ComponentTable empty_table;
  RaypathColorConfig empty_cfg;
  EXPECT_NO_THROW({
    auto map = BuildComponentColorMap(empty_cfg, empty_table);
    EXPECT_EQ(map.colored_mask_, 0u);
  });
}

// ---- (3) out-of-range triple → std::invalid_argument ----

TEST(BuildComponentColorMap, LayerOutOfRangeThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.entries_.push_back(MakeEntry(9, 0, 0, 1.0f, 0.0f, 0.0f));
  EXPECT_THROW(BuildComponentColorMap(cfg, table), std::invalid_argument);
}

TEST(BuildComponentColorMap, CrystalSlotOutOfRangeThrows) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.entries_.push_back(MakeEntry(0, 5, 0, 1.0f, 0.0f, 0.0f));
  EXPECT_THROW(BuildComponentColorMap(cfg, table), std::invalid_argument);
}

TEST(BuildComponentColorMap, SummandOutOfRangeThrows) {
  // Slot 0 in three-arcs has exactly 1 summand — summand=1 must throw
  // (distinct from kNoBit which would be summand within-range-but-overflowed).
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.entries_.push_back(MakeEntry(0, 0, 1, 1.0f, 0.0f, 0.0f));
  EXPECT_THROW(BuildComponentColorMap(cfg, table), std::invalid_argument);
}

// ---- (4) soft-cap overflow (kNoBit) → warn + skip, others unaffected ----

TEST(BuildComponentColorMap, KNoBitOverflowIsSkippedNotThrown) {
  // 70 summands > 64-bit cap — indices [64, 70) get kNoBit but still appear
  // in table.entries_ (see BuildComponentTable, OverflowPast64SummandsGets
  // NoBitSentinel).
  constexpr size_t kTotalSummands = 70;
  ComplexFilterParam complex;
  complex.filters_.resize(kTotalSummands);
  for (size_t i = 0; i < kTotalSummands; i++) {
    complex.filters_[i].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  }
  auto scene = MakeSceneWithLayers({ { lumice::FilterParam{ complex } } });
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), kTotalSummands);

  RaypathColorConfig cfg;
  // Overflowed summand — must skip silently (well, LOG_WARNING) and NOT throw.
  cfg.entries_.push_back(MakeEntry(0, 0, 65, 1.0f, 0.0f, 0.0f));
  // Also within-cap summand — must land at its bit unaffected.
  cfg.entries_.push_back(MakeEntry(0, 0, 3, 0.0f, 1.0f, 0.0f));

  ComponentColorMap map;
  EXPECT_NO_THROW(map = BuildComponentColorMap(cfg, table));

  // Overflowed entry contributes nothing to the mask.
  EXPECT_EQ(map.colored_mask_, static_cast<uint64_t>(1) << 3);
  EXPECT_FLOAT_EQ(map.colors_[3][1], 1.0f);
}

// ---- (5) duplicate triple → last-write wins ----

TEST(BuildComponentColorMap, DuplicateTripleLastWriteWins) {
  auto scene = MakeThreeArcsScene();
  auto table = BuildComponentTable(scene);
  RaypathColorConfig cfg;
  cfg.entries_.push_back(MakeEntry(0, 0, 0, 1.0f, 0.0f, 0.0f));  // red
  cfg.entries_.push_back(MakeEntry(0, 0, 0, 0.0f, 1.0f, 0.0f));  // green, same triple

  auto map = BuildComponentColorMap(cfg, table);
  EXPECT_EQ(map.colored_mask_, 0b1u);
  EXPECT_FLOAT_EQ(map.colors_[0][0], 0.0f);
  EXPECT_FLOAT_EQ(map.colors_[0][1], 1.0f);
  EXPECT_FLOAT_EQ(map.colors_[0][2], 0.0f);
}
