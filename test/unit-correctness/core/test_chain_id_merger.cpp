// ChainIdInterningTable::Segments() and ChainIdMerger in isolation, plus the
// one Simulator fact the merger's producer key rests on.
//
//   A. Segments() is the structured twin of Format(): same walk, same order,
//      root -> leaf, empty for the root. Checked on chains of depth 1/2/3.
//   B. ChainIdMerger keeps producers' id spaces apart and converges the same
//      chain reported by two producers onto one merged id, with the two
//      producers' deltas arriving interleaved the way a consumer queue would
//      deliver them. Resolve() has a sentinel, not a silent 0, for a pair it
//      never saw; the root maps to itself.
//   C. Absorb() reports the two contract breaks it can witness — an orphaned
//      entry (parent never delivered) and a non-ascending local id — and
//      Clear() forgets everything.
//   D. Simulator::GetEffectiveSeed(): a fixed non-zero seed is returned
//      verbatim (two Simulators with the same seed collide), seed 0 derives a
//      distinct value per instance. Not new behaviour — pinned because the
//      merger's "distinct key per worker" premise is served by ServerImpl's
//      fixed-seed -> single-worker rule, not by the seed derivation itself.
//   E. The table's bound: a table of capacity K answers the (K+1)-th distinct
//      key with kOverflowChainId, keeps every earlier chain intact, counts the
//      arrivals it turned away (again when the same key comes back — it is not
//      remembered — and not the layers walked past the loss), and hands the
//      count over once per FlushDelta() cycle. The
//      capacity is the ruler, not a fixed assertion: two tables of different
//      capacity under the same stream overflow at different points, which is
//      what shows the bound is in force rather than the test being true of
//      any table. The merger maps the sentinel to itself for every producer,
//      so two workers' overflow meet on one merged sentinel.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/chain_id_table.hpp"
#include "core/def.hpp"
#include "core/simulator.hpp"
#include "util/queue.hpp"

namespace lumice {
namespace {

using Seg = std::vector<IdType>;

// A: Segments() vs Format() on a depth-3 chain and its two ancestors.
TEST(ChainIdSegments, RootToLeafOrderMatchesFormatAndRootIsEmpty) {
  ChainIdInterningTable table;
  const uint32_t l1 = table.Intern(ChainIdInterningTable::kRootChainId, 1, Seg{ 1, 3, 5 });
  const uint32_t l2 = table.Intern(l1, 2, Seg{ 3, 2 });
  const uint32_t l3 = table.Intern(l2, 1, Seg{ 4 });

  EXPECT_TRUE(table.Segments(ChainIdInterningTable::kRootChainId).empty());
  EXPECT_EQ(table.Format(ChainIdInterningTable::kRootChainId), "");

  auto s1 = table.Segments(l1);
  ASSERT_EQ(s1.size(), 1u);
  EXPECT_EQ(s1[0].id, l1);
  EXPECT_EQ(s1[0].crystal_id, 1u);
  EXPECT_EQ(s1[0].segment, (Seg{ 1, 3, 5 }));

  auto s2 = table.Segments(l2);
  ASSERT_EQ(s2.size(), 2u);
  EXPECT_EQ(s2[0].id, l1);
  EXPECT_EQ(s2[1].id, l2);
  EXPECT_EQ(s2[1].crystal_id, 2u);
  EXPECT_EQ(s2[1].segment, (Seg{ 3, 2 }));

  auto s3 = table.Segments(l3);
  ASSERT_EQ(s3.size(), 3u);
  // result[i] is layer i+1 — the order Format() prints. Rebuild Format()'s
  // string from the segments to pin that the two walks agree.
  std::string rebuilt;
  for (const auto& e : s3) {
    if (!rebuilt.empty()) {
      rebuilt += '-';
    }
    rebuilt += "crystal" + std::to_string(e.crystal_id) + "(";
    for (size_t i = 0; i < e.segment.size(); i++) {
      rebuilt += (i ? "-" : "") + std::to_string(e.segment[i]);
    }
    rebuilt += ")";
  }
  EXPECT_EQ(rebuilt, table.Format(l3));
  EXPECT_EQ(table.Format(l3), "crystal1(1-3-5)-crystal2(3-2)-crystal1(4)");
}

// Build a delta the way FlushDelta() would: ascending ids, parents first.
ChainIdTableEntry Entry(uint32_t id, uint32_t parent, IdType crystal, Seg seg) {
  ChainIdTableEntry e;
  e.id = id;
  e.parent_id = parent;
  e.crystal_id = crystal;
  e.segment = std::move(seg);
  return e;
}

// B: two producers, interleaved deltas, colliding local ids.
TEST(ChainIdMerger, SameChainFromTwoProducersConvergesAndDifferentChainsStayApart) {
  // Producer 10 interns X=(c1,{1,3}) as 1 then Y=(c1,{2,4}) as 2.
  // Producer 20 interns Y as 1 then X as 2 — the same local ids, swapped.
  // Both then add a second-layer child under their own id for X.
  ChainIdMerger merger;
  auto r = merger.Absorb(10, { Entry(1, 0, 1, { 1, 3 }) });
  EXPECT_EQ(r.orphaned, 0u);
  EXPECT_EQ(r.non_monotonic, 0u);
  merger.Absorb(20, { Entry(1, 0, 1, { 2, 4 }) });
  merger.Absorb(10, { Entry(2, 0, 1, { 2, 4 }), Entry(3, 1, 2, { 5 }) });
  merger.Absorb(20, { Entry(2, 0, 1, { 1, 3 }), Entry(3, 2, 2, { 5 }) });

  const uint32_t x10 = merger.Resolve(10, 1);
  const uint32_t x20 = merger.Resolve(20, 2);
  const uint32_t y10 = merger.Resolve(10, 2);
  const uint32_t y20 = merger.Resolve(20, 1);
  EXPECT_EQ(x10, x20) << "one chain, two producers: one merged id";
  EXPECT_EQ(y10, y20);
  EXPECT_NE(x10, y10) << "different chains must not fuse";
  // Local id 1 means X on producer 10 and Y on producer 20 — the merge must
  // not have trusted the number.
  EXPECT_NE(merger.Resolve(10, 1), merger.Resolve(20, 1));

  const uint32_t xc10 = merger.Resolve(10, 3);
  const uint32_t xc20 = merger.Resolve(20, 3);
  EXPECT_EQ(xc10, xc20) << "child under X converges through the remapped parent";
  EXPECT_EQ(merger.Table().Format(xc10), "crystal1(1-3)-crystal2(5)");
  EXPECT_EQ(merger.Table().Size(), 3u) << "X, Y, X's child — nothing duplicated";

  EXPECT_EQ(merger.Resolve(10, ChainIdInterningTable::kRootChainId), ChainIdInterningTable::kRootChainId);
  EXPECT_EQ(merger.Resolve(10, 99), ChainIdMerger::kUnresolved);
  EXPECT_EQ(merger.Resolve(30, 1), ChainIdMerger::kUnresolved) << "unknown producer";
}

// C: contract-break witnesses and Clear().
TEST(ChainIdMerger, AbsorbReportsOrphansAndNonMonotonicIdsAndClearForgets) {
  ChainIdMerger merger;
  // Child names parent 7, never delivered by producer 1.
  auto r = merger.Absorb(1, { Entry(1, 0, 1, { 1 }), Entry(2, 7, 2, { 3 }) });
  EXPECT_EQ(r.orphaned, 1u);
  EXPECT_EQ(merger.Resolve(1, 1), 1u);
  EXPECT_EQ(merger.Resolve(1, 2), ChainIdMerger::kUnresolved) << "an orphan stays unresolved";
  EXPECT_EQ(merger.Table().Size(), 1u);

  // Producer 1 delivers id 1 again — either a second Simulator under the same
  // key or a restart the consumer was not Reset() for.
  r = merger.Absorb(1, { Entry(1, 0, 1, { 9 }) });
  EXPECT_EQ(r.non_monotonic, 1u);
  EXPECT_EQ(r.orphaned, 0u);
  EXPECT_EQ(merger.Resolve(1, 1), 2u) << "still absorbed; the report is the signal";

  // A fresh producer's ascending ids are not non-monotonic.
  r = merger.Absorb(2, { Entry(1, 0, 1, { 1 }), Entry(2, 1, 1, { 2 }) });
  EXPECT_EQ(r.non_monotonic, 0u);

  merger.Clear();
  EXPECT_EQ(merger.Table().Size(), 0u);
  EXPECT_EQ(merger.Resolve(1, 1), ChainIdMerger::kUnresolved);
  EXPECT_EQ(merger.Resolve(2, 1), ChainIdMerger::kUnresolved);
  // Ids restart from 1 after Clear(), and the old producer state is gone, so
  // the first delta of a producer is monotonic again.
  r = merger.Absorb(1, { Entry(1, 0, 3, { 1 }) });
  EXPECT_EQ(r.non_monotonic, 0u);
  EXPECT_EQ(merger.Resolve(1, 1), 1u);
}

// D: the effective-seed facts the producer key rests on.
TEST(ChainIdMerger, EffectiveSeedIsVerbatimWhenFixedAndDistinctWhenZero) {
  auto cq = std::make_shared<Queue<SimBatch>>();
  auto dq = std::make_shared<Queue<SimData>>();
  Simulator fixed_a(cq, dq, 4242);
  Simulator fixed_b(cq, dq, 4242);
  EXPECT_EQ(fixed_a.GetEffectiveSeed(), 4242u);
  EXPECT_EQ(fixed_a.GetEffectiveSeed(), fixed_b.GetEffectiveSeed())
      << "a fixed seed is returned verbatim: two workers under one fixed seed WOULD share a producer key, "
         "which is why ServerImpl collapses a fixed seed to one worker";

  Simulator auto_a(cq, dq, 0);
  Simulator auto_b(cq, dq, 0);
  EXPECT_NE(auto_a.GetEffectiveSeed(), 0u);
  EXPECT_NE(auto_b.GetEffectiveSeed(), 0u);
  EXPECT_NE(auto_a.GetEffectiveSeed(), auto_b.GetEffectiveSeed()) << "seed 0 derives a distinct value per instance";
}

// E: the bound, its ruler, and the merger's view of the sentinel.
TEST(ChainIdTableBound, CapacityIsTheRulerNewKeysPastItGetTheSentinelAndEarlierChainsStay) {
  // `static` so the capture-less lambda below can name them on MSVC too (C3493: a block-scope
  // constexpr still counts as a capture there; GCC/Clang let a non-odr-use through).
  static constexpr uint32_t kOverflow = ChainIdInterningTable::kOverflowChainId;
  static constexpr uint32_t kRoot = ChainIdInterningTable::kRootChainId;
  // Red state first: the same stream of 6 distinct keys against capacity 3
  // and capacity 5 must overflow at the 4th and the 6th key respectively. If
  // the bound were not in force both would intern all six.
  auto overflow_index = [](size_t capacity) -> size_t {
    ChainIdInterningTable t(capacity);
    for (size_t i = 0; i < 6; i++) {
      if (t.Intern(kRoot, 1, Seg{ static_cast<IdType>(i + 1) }) == kOverflow) {
        return i;
      }
    }
    return 6;
  };
  EXPECT_EQ(overflow_index(3), 3u);
  EXPECT_EQ(overflow_index(5), 5u);
  EXPECT_EQ(overflow_index(ChainIdInterningTable::kUnboundedCapacity), 6u) << "unbounded never overflows";

  ChainIdInterningTable t(3);
  EXPECT_EQ(t.Capacity(), 3u);
  const uint32_t a = t.Intern(kRoot, 1, Seg{ 1 });
  const uint32_t b = t.Intern(kRoot, 1, Seg{ 2 });
  const uint32_t c = t.Intern(a, 2, Seg{ 3 });
  EXPECT_EQ(t.Size(), 3u);
  EXPECT_EQ(t.ConsumeOverflowCount(), 0u) << "nothing turned away while there was room";

  const uint32_t d = t.Intern(kRoot, 1, Seg{ 4 });
  EXPECT_EQ(d, kOverflow);
  EXPECT_EQ(t.Size(), 3u) << "the sentinel is not an entry";
  // Earlier chains are untouched: same ids on re-query, same walks.
  EXPECT_EQ(t.Intern(kRoot, 1, Seg{ 1 }), a);
  EXPECT_EQ(t.Intern(kRoot, 1, Seg{ 2 }), b);
  EXPECT_EQ(t.Intern(a, 2, Seg{ 3 }), c);
  EXPECT_EQ(t.Format(c), "crystal1(1)-crystal2(3)");
  // A known key under a known parent is still answered, even at capacity:
  // the bound is on distinct chains, not on lookups.
  EXPECT_EQ(t.Intern(a, 2, Seg{ 3 }), c);

  // Sticky down the chain, and not counted twice: the child of an overflowed
  // chain is the sentinel again, but the chain was lost once.
  EXPECT_EQ(t.Intern(d, 2, Seg{ 9 }), kOverflow);
  EXPECT_EQ(t.Intern(kOverflow, 1, Seg{ 1 }), kOverflow);
  EXPECT_EQ(t.Intern(kRoot, 1, Seg{ 5 }), kOverflow);
  EXPECT_EQ(t.ConsumeOverflowCount(), 2u) << "keys {4} and {5} were turned away; the children were not new keys";
  EXPECT_EQ(t.ConsumeOverflowCount(), 0u) << "the count is handed over once";
  // A turned-away key is not remembered: the same chain arriving again is
  // turned away again and counted again (arrivals, not distinct chains).
  EXPECT_EQ(t.Intern(kRoot, 1, Seg{ 4 }), kOverflow);
  EXPECT_EQ(t.ConsumeOverflowCount(), 1u);

  // The sentinel walks as nothing: no path, no crash.
  EXPECT_TRUE(t.Segments(kOverflow).empty());
  EXPECT_EQ(t.Format(kOverflow), "");

  // FlushDelta() carries only the real entries; the count travels beside it.
  auto delta = t.FlushDelta();
  ASSERT_EQ(delta.size(), 3u);
  for (const auto& e : delta) {
    EXPECT_NE(e.id, kOverflow);
    EXPECT_NE(e.parent_id, kOverflow);
  }
  t.Intern(kRoot, 1, Seg{ 6 });
  EXPECT_EQ(t.ConsumeOverflowCount(), 1u);
  EXPECT_TRUE(t.FlushDelta().empty());

  // Clear() rewinds the bound with the ids: room again, count zeroed.
  t.Intern(kRoot, 1, Seg{ 7 });
  t.Clear();
  EXPECT_EQ(t.ConsumeOverflowCount(), 0u);
  EXPECT_EQ(t.Intern(kRoot, 1, Seg{ 4 }), 1u) << "ids restart from 1 and the 4th key now fits";
}

TEST(ChainIdTableBound, MergerMapsTheSentinelToItselfForEveryProducer) {
  constexpr uint32_t kOverflow = ChainIdInterningTable::kOverflowChainId;
  ChainIdMerger merger;
  merger.Absorb(10, { Entry(1, 0, 1, { 1, 3 }) });
  merger.Absorb(20, { Entry(1, 0, 1, { 2, 4 }) });
  // Both producers overflowed: their rays carry the same constant, which the
  // merger resolves to the same constant without either producer having
  // delivered an entry for it — there is none to deliver.
  EXPECT_EQ(merger.Resolve(10, kOverflow), kOverflow);
  EXPECT_EQ(merger.Resolve(20, kOverflow), kOverflow);
  EXPECT_EQ(merger.Resolve(30, kOverflow), kOverflow) << "even a producer never seen: the sentinel needs no remap";
  EXPECT_EQ(merger.Table().Size(), 2u) << "no entry was interned for the sentinel";
  EXPECT_EQ(merger.Resolve(10, 99), ChainIdMerger::kUnresolved) << "the two sentinels stay distinct";
  EXPECT_NE(kOverflow, ChainIdMerger::kUnresolved);
}

}  // namespace
}  // namespace lumice
