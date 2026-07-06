// Tests for src/config/component_table.hpp — BuildComponentTable's summand
// counting rule and (mi, ci, sk)-ordered bit allocation.
//
// task-331.1 (raypath-color foundation): this table is a pure config-time
// function with no runtime consumer yet (T1 only proves it can be built from
// a SceneConfig; T2 wires the actual per-summand lookup into CollectData).
// These tests lock down BuildComponentTable's own contract in isolation.

#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <tuple>
#include <vector>

#include "config/component_table.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "core/def.hpp"

namespace {

using lumice::ComplexFilterParam;
using lumice::ComponentTable;
using lumice::CrystalConfig;
using lumice::CrystalFilterParam;
using lumice::DirectionFilterParam;
using lumice::FilterConfig;
using lumice::MsInfo;
using lumice::NoneFilterParam;
using lumice::RaypathFilterParam;
using lumice::ScatteringSetting;
using lumice::SceneConfig;
using lumice::SimpleFilterParam;

// Build one ScatteringSetting with the given filter param and an otherwise
// default (deterministic prism) crystal — BuildComponentTable never reads
// crystal_/crystal_proportion_, so these are irrelevant placeholders.
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

}  // namespace

// ---- Summand counting rule ----

TEST(BuildComponentTable, NoneFilterParamContributesZeroSummands) {
  auto scene = MakeSceneWithLayers({ { SimpleFilterParam{ NoneFilterParam{} } } });
  auto table = lumice::BuildComponentTable(scene);
  EXPECT_TRUE(table.entries_.empty()) << "NoneFilterParam must not contribute any component-table entry";
}

TEST(BuildComponentTable, BareSimpleFilterParamContributesOneSummand) {
  RaypathFilterParam raypath;
  raypath.raypath_ = { 1, 2, 3 };
  auto scene = MakeSceneWithLayers({ { SimpleFilterParam{ raypath } } });
  auto table = lumice::BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 1u) << "a non-None SimpleFilterParam is exactly one summand";
  EXPECT_EQ(table.entries_[0].bit_, 0u);
  EXPECT_EQ(table.entries_[0].layer_, 0);
  EXPECT_EQ(table.entries_[0].crystal_id_, 0);
  EXPECT_EQ(table.entries_[0].summand_idx_, 0);
}

TEST(BuildComponentTable, ComplexFilterParamContributesFiltersSizeSummands) {
  ComplexFilterParam complex;
  // 3 OR-summands, each a 1-term AND-chain. The AND-chain contents don't
  // matter to BuildComponentTable (only the outer filters_.size()).
  complex.filters_.resize(3);
  complex.filters_[0].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  complex.filters_[1].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 1 } });
  complex.filters_[2].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ DirectionFilterParam{ 0.0f, 0.0f, 1.0f } });

  auto scene = MakeSceneWithLayers({ { lumice::FilterParam{ complex } } });
  auto table = lumice::BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 3u) << "ComplexFilterParam summand count must equal filters_.size()";
  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(table.entries_[i].summand_idx_, static_cast<lumice::IdType>(i));
    EXPECT_EQ(table.entries_[i].bit_, static_cast<uint8_t>(i));
  }
}

// ---- Bit allocation ordered by (mi, ci, sk), unique across the whole scene ----

TEST(BuildComponentTable, BitsAssignedInLayerCrystalSummandOrderAndAreUnique) {
  // Layer 0: crystal 0 -> None (0 summands), crystal 1 -> Simple (1 summand).
  // Layer 1: crystal 0 -> Complex with 2 summands, crystal 1 -> Simple (1 summand).
  ComplexFilterParam complex2;
  complex2.filters_.resize(2);
  complex2.filters_[0].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  complex2.filters_[1].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 1 } });

  RaypathFilterParam raypath;
  raypath.raypath_ = { 5 };

  auto scene = MakeSceneWithLayers({
      { SimpleFilterParam{ NoneFilterParam{} }, SimpleFilterParam{ raypath } },
      { lumice::FilterParam{ complex2 }, SimpleFilterParam{ raypath } },
  });

  auto table = lumice::BuildComponentTable(scene);
  // Expected entries in push order: (mi=0,ci=1,sk=0), (mi=1,ci=0,sk=0),
  // (mi=1,ci=0,sk=1), (mi=1,ci=1,sk=0) — 4 total (the mi=0/ci=0 None
  // contributes nothing).
  ASSERT_EQ(table.entries_.size(), 4u);

  std::vector<std::tuple<lumice::IdType, lumice::IdType, lumice::IdType>> keys;
  std::set<uint8_t> bits;
  for (const auto& e : table.entries_) {
    keys.emplace_back(e.layer_, e.crystal_id_, e.summand_idx_);
    bits.insert(e.bit_);
  }
  EXPECT_TRUE(std::is_sorted(keys.begin(), keys.end())) << "entries must be in (mi, ci, sk) lexicographic order";
  EXPECT_EQ(bits.size(), 4u) << "bit_ values must be unique across the whole scene";

  // Bits themselves must be exactly {0,1,2,3} assigned in entries_ order.
  for (size_t i = 0; i < table.entries_.size(); i++) {
    EXPECT_EQ(table.entries_[i].bit_, static_cast<uint8_t>(i))
        << "bit " << i << " not assigned in strict (mi,ci,sk) push order";
  }

  EXPECT_EQ(std::get<0>(keys[0]), 0);
  EXPECT_EQ(std::get<1>(keys[0]), 1);
  EXPECT_EQ(std::get<0>(keys[1]), 1);
  EXPECT_EQ(std::get<1>(keys[1]), 0);
  EXPECT_EQ(std::get<2>(keys[1]), 0);
  EXPECT_EQ(std::get<2>(keys[2]), 1);
  EXPECT_EQ(std::get<1>(keys[3]), 1);
}

// ---- Soft-cap overflow: >64 summands must not crash, overflow entries get kNoBit ----

TEST(BuildComponentTable, OverflowPast64SummandsGetsNoBitSentinelAndDoesNotCrash) {
  constexpr size_t kTotalSummands = 70;  // > ComponentTable::kMaxBits (64)
  ComplexFilterParam complex;
  complex.filters_.resize(kTotalSummands);
  for (size_t i = 0; i < kTotalSummands; i++) {
    complex.filters_[i].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  }

  auto scene = MakeSceneWithLayers({ { lumice::FilterParam{ complex } } });
  auto table = lumice::BuildComponentTable(scene);

  ASSERT_EQ(table.entries_.size(), kTotalSummands) << "overflow summands must still be recorded in entries_";
  for (size_t i = 0; i < ComponentTable::kMaxBits; i++) {
    EXPECT_NE(table.entries_[i].bit_, ComponentTable::kNoBit) << "summand " << i << " should still fit under the cap";
    EXPECT_EQ(table.entries_[i].bit_, static_cast<uint8_t>(i));
  }
  for (size_t i = ComponentTable::kMaxBits; i < kTotalSummands; i++) {
    EXPECT_EQ(table.entries_[i].bit_, ComponentTable::kNoBit)
        << "summand " << i << " exceeds the 64-bit budget and must be the sentinel";
  }
}
