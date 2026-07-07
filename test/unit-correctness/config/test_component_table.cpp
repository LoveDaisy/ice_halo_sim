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

TEST(BuildComponentTable, NoneFilterParamContributesOneWholeCrystalSummand) {
  // task-339.1: NoneFilterParam contributes exactly one whole-crystal virtual
  // summand (summand_idx = 0) so the raypath-color engine's {layer, crystal}
  // ref (color-class schema, task-339.2) can resolve to a component bit.
  auto scene = MakeSceneWithLayers({ { SimpleFilterParam{ NoneFilterParam{} } } });
  auto table = lumice::BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 1u) << "NoneFilterParam must contribute exactly one whole-crystal entry";
  EXPECT_EQ(table.entries_[0].bit_, 0u);
  EXPECT_EQ(table.entries_[0].layer_, 0);
  EXPECT_EQ(table.entries_[0].crystal_id_, 0);
  EXPECT_EQ(table.entries_[0].summand_idx_, 0);
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
  // Layer 0: crystal 0 -> None (1 whole-crystal summand after task-339.1),
  //          crystal 1 -> Simple (1 summand).
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
  // Expected entries in push order (task-339.1 — None-filter now contributes
  // its whole-crystal summand, interleaved in the same (mi, ci, sk) traversal):
  //   (mi=0,ci=0,sk=0)  None            -> bit 0
  //   (mi=0,ci=1,sk=0)  raypath         -> bit 1
  //   (mi=1,ci=0,sk=0)  complex2 sum 0  -> bit 2
  //   (mi=1,ci=0,sk=1)  complex2 sum 1  -> bit 3
  //   (mi=1,ci=1,sk=0)  raypath         -> bit 4
  ASSERT_EQ(table.entries_.size(), 5u);

  std::vector<std::tuple<lumice::IdType, lumice::IdType, lumice::IdType>> keys;
  std::set<uint8_t> bits;
  for (const auto& e : table.entries_) {
    keys.emplace_back(e.layer_, e.crystal_id_, e.summand_idx_);
    bits.insert(e.bit_);
  }
  EXPECT_TRUE(std::is_sorted(keys.begin(), keys.end())) << "entries must be in (mi, ci, sk) lexicographic order";
  EXPECT_EQ(bits.size(), 5u) << "bit_ values must be unique across the whole scene";

  // Bits themselves must be exactly {0,1,2,3,4} assigned in entries_ order.
  for (size_t i = 0; i < table.entries_.size(); i++) {
    EXPECT_EQ(table.entries_[i].bit_, static_cast<uint8_t>(i))
        << "bit " << i << " not assigned in strict (mi,ci,sk) push order";
  }

  // Reverse-lookup via ComponentBitsFor to avoid coupling every assertion to
  // hard-coded numeric bit constants (plan §4 Step 1 test-point guidance).
  auto l0c0 = lumice::ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/0);
  auto l0c1 = lumice::ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/1);
  auto l1c0 = lumice::ComponentBitsFor(table, /*layer=*/1, /*crystal_id=*/0);
  auto l1c1 = lumice::ComponentBitsFor(table, /*layer=*/1, /*crystal_id=*/1);
  ASSERT_EQ(l0c0.size(), 1u) << "None-filter crystal must expose exactly one whole-crystal bit";
  ASSERT_EQ(l0c1.size(), 1u);
  ASSERT_EQ(l1c0.size(), 2u);
  ASSERT_EQ(l1c1.size(), 1u);
  // Every reverse-lookup bit is distinct — the union covers the whole table.
  std::set<uint8_t> covered{ l0c0[0], l0c1[0], l1c0[0], l1c0[1], l1c1[0] };
  EXPECT_EQ(covered.size(), 5u) << "reverse lookups collectively cover every distinct bit";
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

TEST(BuildComponentTable, NoneFilterCountsTowardKMaxBitsOverflowBudget) {
  // task-339.1 verifies that None's whole-crystal bit consumes a slot in the
  // 64-bit budget just like any other summand. Construct a scene where the
  // total summand count crosses `kMaxBits` (64) precisely because None crystals
  // are contributing: 63 simple-filter summands + 2 None-filter crystals = 65
  // total. The 65-th summand (whichever it happens to be in traversal order)
  // must land on `kNoBit`, and both None entries must appear in `entries_`.
  constexpr size_t kSimpleSummands = 63;
  ComplexFilterParam big_complex;
  big_complex.filters_.resize(kSimpleSummands);
  for (size_t i = 0; i < kSimpleSummands; i++) {
    big_complex.filters_[i].emplace_back(lumice::IdType{ 0 }, SimpleFilterParam{ CrystalFilterParam{ 0 } });
  }

  // Layout: [None, Complex(63), None] on layer 0 — traversal order puts the
  // first None at bits[0], the 63 complex summands at bits[1..63], and the
  // trailing None at bit index 64 which overflows to kNoBit.
  auto scene = MakeSceneWithLayers({ {
      SimpleFilterParam{ NoneFilterParam{} },
      lumice::FilterParam{ big_complex },
      SimpleFilterParam{ NoneFilterParam{} },
  } });
  auto table = lumice::BuildComponentTable(scene);

  ASSERT_EQ(table.entries_.size(), kSimpleSummands + 2u)
      << "None entries (including overflow) must still be recorded in entries_";

  size_t assigned = 0;
  size_t overflow = 0;
  for (const auto& e : table.entries_) {
    if (e.bit_ == ComponentTable::kNoBit) {
      overflow++;
    } else {
      assigned++;
      EXPECT_LT(e.bit_, ComponentTable::kMaxBits);
    }
  }
  EXPECT_EQ(assigned, ComponentTable::kMaxBits) << "budget must be filled to exactly kMaxBits (64)";
  EXPECT_EQ(overflow, 1u) << "exactly one summand must overflow to kNoBit";

  // Confirm both None crystals still show up in reverse lookup (with the
  // trailing one now carrying the sentinel).
  auto first_none = lumice::ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/0);
  auto trailing_none = lumice::ComponentBitsFor(table, /*layer=*/0, /*crystal_id=*/2);
  ASSERT_EQ(first_none.size(), 1u);
  ASSERT_EQ(trailing_none.size(), 1u);
  EXPECT_EQ(first_none[0], 0u) << "first None-crystal grabs bit 0 in traversal order";
  EXPECT_EQ(trailing_none[0], ComponentTable::kNoBit) << "trailing None-crystal overflows the 64-bit budget";
}
