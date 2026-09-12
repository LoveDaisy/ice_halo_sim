// Whitebox verification of the raypath-analysis foundation: the per-ray chain
// id carried across MS layers and the interning table that turns it back into
// a printable chain (doc/raypath-analysis-panel.md §3.1–§3.2).
//
// Four things are pinned here, each on its own mechanism rather than on a
// statistic:
//   A. ChainIdInterningTable in isolation — dense ids, idempotent interning,
//      incremental FlushDelta, parent-pointer walk.
//   B. RayBuffer's chain-id column — allocated only on request (the zero-cost
//      contract), and carried through EVERY reorder point components_ is
//      carried through: fan-out, batch EmplaceBack, SwapRay, copy/move. The
//      naive-swap negative control shows what a missed point looks like.
//   C. The hand-off primitives (InitRayFirstMs / InitRayOtherMs / CollectData)
//      and the single fold rule InternRayChainId, including the literal chain
//      string the design doc uses as its example.
//   D. End to end through Simulator::Run(): with analysis on, every delivered
//      chain id is checked against an oracle rebuilt from the raw segment
//      buffer (SetAllDataObserverForTest) — depth, crystal, and the reduced
//      segment recomputed by calling Crystal::ReduceRaypath ourselves; for the
//      second MS layer the parent segment is recovered by matching the carried
//      weight back to the layer-1 continuation segment, which is a linkage the
//      chain id plays no part in. Then: analysis off leaves the outputs empty
//      AND bit-identical to a run that never had the feature (the RNG stream is
//      untouched), deltas compose incrementally across batches, and two workers'
//      deltas merge into one trie without collision or loss.
//
// Discipline (per project memory): per-captured-ray assertions, positive and
// negative controls, real machinery, no synthetic bypass.

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/chain_id_table.hpp"
#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/simulator.hpp"
#include "core/trace_ops.hpp"
#include "util/queue.hpp"

namespace lumice {
namespace {

constexpr uint8_t kSymAll = FilterConfig::kSymP | FilterConfig::kSymB | FilterConfig::kSymD;

// Plate-like axis: azimuth uniform over 360°, latitude fixed at 90°, roll
// fixed at 0° — D symmetry applicable (full-360 uniform azimuth + roll on a
// multiple of 30°), so every one of P/B/D has something to reduce.
AxisDistribution MakeAxis() {
  AxisDistribution d{};
  d.azimuth_dist.type = DistributionType::kUniform;
  d.azimuth_dist.spread = 360.0f;
  d.azimuth_dist.center = 0.0f;
  d.latitude_dist.type = DistributionType::kNoRandom;
  d.latitude_dist.center = 90.0f;
  d.roll_dist.type = DistributionType::kNoRandom;
  d.roll_dist.center = 0.0f;
  d.roll_dist.spread = 0.0f;
  return d;
}

RaySeg MakeRay() {
  RaySeg r{};
  r.from_face_ = kInvalidId;
  r.to_face_ = kInvalidId;
  r.w_ = 1.0f;
  r.crystal_idx_ = 0;
  r.crystal_config_id_ = 0;
  r.d_[0] = 0.0f;
  r.d_[1] = 0.0f;
  r.d_[2] = 1.0f;
  r.p_[0] = 0.0f;
  r.p_[1] = 0.0f;
  r.p_[2] = 0.0f;
  r.crystal_rot_ = Rotation{};
  return r;
}

void AppendRecorder(RayBuffer& buf, size_t idx, const std::vector<IdType>& rp) {
  buf.RecorderClear(idx);
  for (auto fn : rp) {
    buf.RecorderAppend(idx, fn);
  }
}

std::vector<IdType> RecorderToVec(const RayBuffer& buf, size_t idx) {
  const auto* data = buf.RecorderDataPtr(idx);
  return std::vector<IdType>(data, data + buf.RecorderAt(idx).size_);
}

// The oracle's reduction: derived by hand from the same public pieces a filter
// on this crystal would use, NOT via MakeChainIdLayerContext.
std::vector<IdType> OracleReduce(const Crystal& crystal, const AxisDistribution& axis, uint8_t symmetry,
                                 const std::vector<IdType>& rp) {
  bool d_applicable = detail::IsDApplicable(axis);
  int sigma_a = d_applicable ? detail::ComputeSigmaA(axis.roll_dist.center) : 0;
  return crystal.ReduceRaypath(rp, symmetry, sigma_a, d_applicable);
}

std::string SegmentString(const std::vector<IdType>& seg) {
  std::string s;
  for (size_t i = 0; i < seg.size(); i++) {
    if (i > 0) {
      s += '-';
    }
    s += std::to_string(seg[i]);
  }
  return s;
}

}  // namespace

// ===========================================================================
// A. ChainIdInterningTable in isolation.
// ===========================================================================

TEST(ChainIdTable, InternIsIdempotentAndIdsAreDense) {
  ChainIdInterningTable table;
  EXPECT_EQ(table.Size(), 0u);

  uint32_t a = table.Intern(ChainIdInterningTable::kRootChainId, 1, { 3, 5 });
  uint32_t b = table.Intern(ChainIdInterningTable::kRootChainId, 1, { 3, 5 });
  uint32_t c = table.Intern(ChainIdInterningTable::kRootChainId, 1, { 3, 6 });
  uint32_t d = table.Intern(ChainIdInterningTable::kRootChainId, 2, { 3, 5 });  // same segment, other crystal
  uint32_t e = table.Intern(a, 1, { 3, 5 });                                    // same segment, other parent

  EXPECT_EQ(a, 1u) << "first interned key gets id 1 (0 is the root)";
  EXPECT_EQ(b, a) << "re-interning the same key returns the same id";
  EXPECT_EQ(c, 2u);
  EXPECT_EQ(d, 3u);
  EXPECT_EQ(e, 4u);
  EXPECT_EQ(table.Size(), 4u);
  for (uint32_t id : { a, c, d, e }) {
    EXPECT_NE(id, ChainIdInterningTable::kRootChainId) << "the root id is never handed out";
    EXPECT_EQ(table.EntryAt(id).id, id);
  }
  EXPECT_EQ(table.EntryAt(e).parent_id, a);
  EXPECT_EQ(table.EntryAt(d).crystal_id, 2u);
  EXPECT_EQ(table.EntryAt(c).segment, (std::vector<IdType>{ 3, 6 }));
}

TEST(ChainIdTable, FlushDeltaReturnsOnlyEntriesSinceLastFlush) {
  ChainIdInterningTable table;
  EXPECT_TRUE(table.FlushDelta().empty()) << "nothing interned yet";

  table.Intern(0, 1, { 3, 5 });
  table.Intern(0, 1, { 4, 6 });
  auto first = table.FlushDelta();
  ASSERT_EQ(first.size(), 2u);
  EXPECT_EQ(first[0].id, 1u);
  EXPECT_EQ(first[1].id, 2u);

  EXPECT_TRUE(table.FlushDelta().empty()) << "a second flush with nothing new returns nothing";

  table.Intern(0, 1, { 3, 5 });  // already known: no new entry
  table.Intern(2, 2, { 1, 2 });  // new
  auto second = table.FlushDelta();
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(second[0].id, 3u);
  EXPECT_EQ(second[0].parent_id, 2u);
  EXPECT_EQ(second[0].crystal_id, 2u);
}

TEST(ChainIdTable, FormatWalksParentPointersRootFirst) {
  ChainIdInterningTable table;
  uint32_t l1 = table.Intern(0, 1, { 1, 3, 5 });
  uint32_t l2 = table.Intern(l1, 2, { 3, 2 });
  uint32_t l3 = table.Intern(l2, 1, { 8 });

  EXPECT_EQ(table.Format(ChainIdInterningTable::kRootChainId), "");
  EXPECT_EQ(table.Format(l1), "crystal1(1-3-5)");
  EXPECT_EQ(table.Format(l2), "crystal1(1-3-5)-crystal2(3-2)");
  EXPECT_EQ(table.Format(l3), "crystal1(1-3-5)-crystal2(3-2)-crystal1(8)");
}

TEST(ChainIdTable, ClearRestartsIdsAndFlushCursor) {
  ChainIdInterningTable table;
  table.Intern(0, 1, { 3, 5 });
  table.Intern(0, 1, { 4, 6 });
  (void)table.FlushDelta();
  table.Clear();
  EXPECT_EQ(table.Size(), 0u);
  EXPECT_EQ(table.Intern(0, 1, { 4, 6 }), 1u) << "ids restart from 1 after Clear";
  auto delta = table.FlushDelta();
  ASSERT_EQ(delta.size(), 1u);
  EXPECT_EQ(delta[0].id, 1u);
}

// ===========================================================================
// B. RayBuffer chain-id column: zero-cost allocation contract + reorder points.
// ===========================================================================

TEST(ChainIdColumn, NotAllocatedUnlessRequested) {
  RayBuffer ctor(16);
  EXPECT_FALSE(ctor.HasChainIds()) << "the capacity constructor never allocates the column";

  RayBuffer buf;
  buf.Reset(16);
  EXPECT_FALSE(buf.HasChainIds()) << "default Reset must not allocate the column (AC5)";
  buf.Reset(32, false);
  EXPECT_FALSE(buf.HasChainIds()) << "growth without the flag must not allocate it either";

  // The default-argument path is what every production call outside the
  // legacy CPU path takes, so ResetHitLoopBuffers must not allocate through
  // its own default either.
  RayBuffer pair[2];
  ResetHitLoopBuffers(pair, 8);
  EXPECT_FALSE(pair[0].HasChainIds());
  EXPECT_FALSE(pair[1].HasChainIds());
  ResetHitLoopBuffers(pair, 8, true);
  EXPECT_TRUE(pair[0].HasChainIds());
  EXPECT_TRUE(pair[1].HasChainIds());
}

// The edge the components_ column never meets (it is never turned off): a
// recycled buffer whose capacity does NOT grow must still gain / lose the
// column when the flag changes between two Resets.
TEST(ChainIdColumn, ResetFollowsTheFlagEvenWhenCapacityDoesNotGrow) {
  RayBuffer buf;
  buf.Reset(16, false);
  ASSERT_FALSE(buf.HasChainIds());

  buf.Reset(16, true);  // same capacity: the grow branch is NOT taken
  ASSERT_TRUE(buf.HasChainIds()) << "flag flip at equal capacity must allocate";
  EXPECT_EQ(buf.ChainIdAt(15), 0u) << "fresh column is zero-filled";
  buf.SetChainId(3, 77u);

  buf.Reset(16, true);  // stays: grow-never-shrink keeps the allocation
  ASSERT_TRUE(buf.HasChainIds());
  EXPECT_EQ(buf.ChainIdAt(3), 77u) << "a retained column keeps its slots (like components_; the layer "
                                      "entry, not Reset, is the reset point)";

  buf.Reset(64, true);  // growth: reallocated at the new capacity
  ASSERT_TRUE(buf.HasChainIds());
  EXPECT_EQ(buf.ChainIdAt(63), 0u);

  buf.Reset(64, false);  // flag off at equal capacity: released
  EXPECT_FALSE(buf.HasChainIds()) << "flag off must release, or the disabled path would keep paying";
}

TEST(ChainIdColumn, SwapRayCarriesChainIdWithItsRay) {
  RayBuffer buf;
  buf.Reset(4, true);
  buf.size_ = 2;
  RaySeg a = MakeRay();
  a.p_[0] = 11.0f;
  RaySeg b = MakeRay();
  b.p_[0] = 22.0f;
  buf.rays_[0] = a;
  buf.rays_[1] = b;
  buf.SetComponent(0, 0xAAAAull);
  buf.SetComponent(1, 0xBBBBull);
  buf.SetChainId(0, 101u);
  buf.SetChainId(1, 202u);

  buf.SwapRay(0, 1);

  EXPECT_FLOAT_EQ(buf.rays_[0].p_[0], 22.0f);
  EXPECT_EQ(buf.ComponentAt(0), 0xBBBBull);
  EXPECT_EQ(buf.ChainIdAt(0), 202u) << "chain id must swap with its ray";
  EXPECT_EQ(buf.ChainIdAt(1), 101u);
  buf.SwapRay(1, 1);
  EXPECT_EQ(buf.ChainIdAt(1), 101u);

  // Without the column SwapRay is still a valid operation (disabled path).
  RayBuffer plain;
  plain.Reset(4);
  plain.size_ = 2;
  plain.rays_[0] = a;
  plain.rays_[1] = b;
  plain.SwapRay(0, 1);
  EXPECT_FLOAT_EQ(plain.rays_[0].p_[0], 22.0f);
  EXPECT_FALSE(plain.HasChainIds());
}

namespace {

constexpr uint32_t kChainBase = 5000;

// Pool of n tagged rays: ray i carries identity i in p_[0] and chain id
// kChainBase+i. Direct slot writes so any slot can be inspected.
void FillTaggedPool(RayBuffer& buf, size_t n) {
  buf.Reset(n + 4, true);
  buf.size_ = n;
  for (size_t i = 0; i < n; i++) {
    RaySeg r = MakeRay();
    r.p_[0] = static_cast<float>(i);
    buf.rays_[i] = r;
    buf.SetChainId(i, kChainBase + static_cast<uint32_t>(i));
  }
}

uint32_t ExpectedChainForSlot(const RayBuffer& buf, size_t i) {
  return kChainBase + static_cast<uint32_t>(buf.rays_[i].p_[0]);
}

}  // namespace

// Negative control: the shape of a missed reorder point. Swapping only the
// RaySeg (what the continuation shuffle did before SwapRay existed) leaves the
// chain id behind and decorrelates it from its ray.
TEST(ChainIdShuffle, NaiveRaySegSwapDecorrelatesChainId) {
  constexpr size_t kN = 16;
  RayBuffer buf;
  FillTaggedPool(buf, kN);
  RandomNumberGenerator rng(42);
  bool any_moved = false;
  for (size_t i = 0; i < buf.size_; i++) {
    size_t j = rng.GetUniformIndex(buf.size_ - i) + i;
    std::swap(buf.rays_[i], buf.rays_[j]);
    any_moved = any_moved || (i != j);
  }
  ASSERT_TRUE(any_moved);
  size_t mismatches = 0;
  for (size_t i = 0; i < buf.size_; i++) {
    if (buf.ChainIdAt(i) != ExpectedChainForSlot(buf, i)) {
      mismatches++;
    }
  }
  EXPECT_GT(mismatches, 0u) << "a RaySeg-only swap must decorrelate the chain id (what a missed point does)";
}

// Positive: the production shuffle primitive keeps the pairing.
TEST(ChainIdShuffle, SwapRayShufflePreservesChainIdPairing) {
  constexpr size_t kN = 16;
  RayBuffer buf;
  FillTaggedPool(buf, kN);
  RandomNumberGenerator rng(42);
  bool any_moved = false;
  for (size_t i = 0; i < buf.size_; i++) {
    size_t j = rng.GetUniformIndex(buf.size_ - i) + i;
    buf.SwapRay(i, j);
    any_moved = any_moved || (i != j);
  }
  ASSERT_TRUE(any_moved);
  for (size_t i = 0; i < buf.size_; i++) {
    EXPECT_EQ(buf.ChainIdAt(i), ExpectedChainForSlot(buf, i)) << "slot " << i << " decorrelated";
  }
}

TEST(ChainIdColumn, BatchEmplaceBackCarriesChainIds) {
  RayBuffer src;
  FillTaggedPool(src, 5);
  RayBuffer dst;
  dst.Reset(16, true);
  for (size_t i = 0; i < 16; i++) {
    dst.SetChainId(i, 0xDEADu);  // poison: "carried, not stale" must be load-bearing
  }
  dst.EmplaceBack(src, 1, 3);  // slots 1,2,3 of src
  ASSERT_EQ(dst.size_, 3u);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(dst.ChainIdAt(i), kChainBase + static_cast<uint32_t>(i + 1)) << "slot " << i;
    EXPECT_FLOAT_EQ(dst.rays_[i].p_[0], static_cast<float>(i + 1));
  }

  // Source without the column → destination slot gets the root id, not the
  // poison left in the recycled slot.
  RayBuffer plain;
  plain.Reset(4);
  plain.size_ = 1;
  plain.rays_[0] = MakeRay();
  dst.EmplaceBack(plain);
  ASSERT_EQ(dst.size_, 4u);
  EXPECT_EQ(dst.ChainIdAt(3), ChainIdInterningTable::kRootChainId);

  // Destination without the column: nothing to carry, nothing to crash on.
  RayBuffer nocol;
  nocol.Reset(16);
  nocol.EmplaceBack(src);
  EXPECT_EQ(nocol.size_, 5u);
  EXPECT_FALSE(nocol.HasChainIds());
}

TEST(ChainIdColumn, FanOutCopiesParentChainIdToBothChildren) {
  RayBuffer src;
  FillTaggedPool(src, 3);
  RayBuffer dst;
  dst.Reset(8, true);
  dst.ChainIdFanOut(src, 2, 4, 5);
  EXPECT_EQ(dst.ChainIdAt(4), kChainBase + 2u);
  EXPECT_EQ(dst.ChainIdAt(5), kChainBase + 2u);

  RayBuffer nocol;
  nocol.Reset(8);
  nocol.ChainIdFanOut(src, 2, 4, 5);  // no-op, must not touch memory
  EXPECT_FALSE(nocol.HasChainIds());
}

TEST(ChainIdColumn, CopyAndMovePreserveTheColumn) {
  RayBuffer src;
  FillTaggedPool(src, 4);

  RayBuffer copy(src);
  ASSERT_TRUE(copy.HasChainIds());
  EXPECT_EQ(copy.ChainIdAt(3), kChainBase + 3u);
  copy.SetChainId(3, 1u);
  EXPECT_EQ(src.ChainIdAt(3), kChainBase + 3u) << "copy must be deep";

  RayBuffer assigned;
  assigned.Reset(2);
  assigned = src;
  ASSERT_TRUE(assigned.HasChainIds());
  EXPECT_EQ(assigned.ChainIdAt(2), kChainBase + 2u);

  RayBuffer moved(std::move(copy));
  ASSERT_TRUE(moved.HasChainIds());
  EXPECT_EQ(moved.ChainIdAt(0), kChainBase + 0u);
  EXPECT_FALSE(copy.HasChainIds()) << "moved-from must not keep the column";  // NOLINT(bugprone-use-after-move)

  RayBuffer move_assigned;
  move_assigned = std::move(assigned);
  ASSERT_TRUE(move_assigned.HasChainIds());
  EXPECT_EQ(move_assigned.ChainIdAt(1), kChainBase + 1u);

  // Copying a buffer WITHOUT the column yields one without it.
  RayBuffer plain;
  plain.Reset(4);
  RayBuffer plain_copy(plain);
  EXPECT_FALSE(plain_copy.HasChainIds());
}

// ===========================================================================
// C. Hand-off primitives and the fold rule.
// ===========================================================================

TEST(ChainIdHandoff, InitRayFirstMsResetsToRoot) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(3);
  SunParam sun{ 90.0f, 0.0f, 0.5f };
  WlParam wl{ 550.0f, 1.0f };
  AxisDistribution axis = MakeAxis();
  RayBuffer buffer_data[2];
  ResetHitLoopBuffers(buffer_data, 4, true);
  RayBuffer all_data;
  all_data.Reset(16);
  for (size_t i = 0; i < 4; i++) {
    buffer_data[0].SetChainId(i, 999u);  // stale id from a recycled slot
  }
  InitRayFirstMs(rng, sun, wl, 4, crystal, 0, axis, buffer_data, all_data);
  ASSERT_EQ(buffer_data[0].size_, 4u);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_EQ(buffer_data[0].ChainIdAt(i), ChainIdInterningTable::kRootChainId) << "slot " << i;
  }

  // Disabled path: same call without the column must simply not touch it.
  RayBuffer plain[2];
  ResetHitLoopBuffers(plain, 4);
  InitRayFirstMs(rng, sun, wl, 4, crystal, 0, axis, plain, all_data);
  EXPECT_FALSE(plain[0].HasChainIds());
}

TEST(ChainIdHandoff, InitRayOtherMsCarriesChainIdWithoutReset) {
  Crystal crystal = Crystal::CreatePrism(1.0f);
  RandomNumberGenerator rng(7);
  AxisDistribution axis = MakeAxis();
  RayBuffer init_data[2];
  init_data[0].Reset(16, true);
  init_data[1].Reset(16, true);
  const uint32_t ids[3] = { 5u, 9u, 40u };
  for (size_t i = 0; i < 3; i++) {
    init_data[0].EmplaceBack(MakeRay(), RaypathRecorder{});
    init_data[0].SetChainId(i, ids[i]);
  }
  RayBuffer buffer_data[2];
  ResetHitLoopBuffers(buffer_data, 8, true);
  for (size_t i = 0; i < 8; i++) {
    buffer_data[0].SetChainId(i, 0xFFFFu);  // poison
  }
  RayBuffer all_data;
  all_data.Reset(64);
  size_t offset = 0;
  InitRayOtherMs(rng, init_data, 3, crystal, 0, axis, buffer_data, all_data, offset);
  ASSERT_EQ(buffer_data[0].size_, 3u);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(buffer_data[0].ChainIdAt(i), ids[i]) << "layer entry must carry the parent id verbatim, slot " << i;
  }
}

TEST(ChainIdHandoff, CollectDataHandsChainIdToBothDestinations) {
  RayBuffer bd[2];
  RayBuffer id[2];
  for (auto* b : { &bd[0], &bd[1], &id[0], &id[1] }) {
    b->Reset(8, true);
  }
  // Slot 0: a normal (in-crystal) ray → goes to bd[0]. Slot 1: an outgoing
  // candidate with prob_=1 → continues into id[1].
  RaySeg normal = MakeRay();
  normal.to_face_ = 2;
  bd[1].EmplaceBack(normal, RaypathRecorder{});
  bd[1].SetChainId(0, 31u);
  RaySeg cont = MakeRay();
  cont.w_ = 0.5f;
  bd[1].EmplaceBack(cont, RaypathRecorder{});
  bd[1].SetChainId(1, 32u);
  for (size_t i = 0; i < 8; i++) {
    bd[0].SetChainId(i, 0xEEEEu);
    id[1].SetChainId(i, 0xEEEEu);
  }
  MsInfo ms;
  ms.prob_ = 1.0f;
  RandomNumberGenerator rng(1);
  CollectData(rng, ms, nullptr, bd, id);

  ASSERT_EQ(bd[0].size_, 1u);
  EXPECT_EQ(bd[0].ChainIdAt(0), 31u) << "in-layer hand-off must carry the id";
  ASSERT_EQ(id[1].size_, 1u);
  ASSERT_TRUE(bd[1].rays_[1].IsContinue());
  EXPECT_EQ(id[1].ChainIdAt(0), 32u) << "continuation hand-off carries the PARENT id (the caller interns)";

  // Disabled path: same routing with no column anywhere.
  RayBuffer pbd[2];
  RayBuffer pid[2];
  for (auto* b : { &pbd[0], &pbd[1], &pid[0], &pid[1] }) {
    b->Reset(8);
  }
  pbd[1].EmplaceBack(normal, RaypathRecorder{});
  pbd[1].EmplaceBack(cont, RaypathRecorder{});
  CollectData(rng, ms, nullptr, pbd, pid);
  EXPECT_EQ(pbd[0].size_, 1u);
  EXPECT_EQ(pid[1].size_, 1u);
  EXPECT_FALSE(pid[1].HasChainIds());
}

TEST(ChainIdFold, LayerContextDerivesSigmaAndDLikeFilterSpecCreate) {
  ChainIdInterningTable table;
  Crystal crystal = Crystal::CreatePrism(1.0f);

  AxisDistribution d_axis = MakeAxis();
  auto ctx = MakeChainIdLayerContext(table, crystal, 7, d_axis, kSymAll);
  EXPECT_EQ(ctx.table, &table);
  EXPECT_EQ(ctx.crystal, &crystal);
  EXPECT_EQ(ctx.crystal_id, 7u);
  EXPECT_EQ(ctx.symmetry, kSymAll);
  EXPECT_EQ(ctx.d_applicable, detail::IsDApplicable(d_axis));
  EXPECT_TRUE(ctx.d_applicable) << "this axis is D-applicable by construction";
  EXPECT_EQ(ctx.sigma_a, detail::ComputeSigmaA(d_axis.roll_dist.center));

  AxisDistribution no_d = MakeAxis();
  no_d.roll_dist.center = 17.0f;  // not a multiple of 30° → D not applicable
  auto ctx2 = MakeChainIdLayerContext(table, crystal, 7, no_d, kSymAll);
  EXPECT_FALSE(ctx2.d_applicable);
  EXPECT_EQ(ctx2.sigma_a, 0);
}

// The design doc's own example, built through the fold rule with the real
// recorder → reduce → intern path. kSymNone pins the literal face numbers.
TEST(ChainIdFold, InternRayChainIdBuildsTheDocExampleChain) {
  ChainIdInterningTable table;
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();

  // Layer 1: crystal 1, path 1-3-5, entered with the root id.
  RayBuffer l1;
  l1.Reset(4, true);
  l1.EmplaceBack(MakeRay(), RaypathRecorder{});
  AppendRecorder(l1, 0, { 1, 3, 5 });
  l1.SetChainId(0, ChainIdInterningTable::kRootChainId);
  auto ctx1 = MakeChainIdLayerContext(table, crystal, 1, axis, FilterConfig::kSymNone);
  uint32_t id1 = InternRayChainId(ctx1, l1, 0);
  EXPECT_EQ(table.Format(id1), "crystal1(1-3-5)");

  // Layer 2: crystal 2, path 3-2, entered carrying id1.
  RayBuffer l2;
  l2.Reset(4, true);
  l2.EmplaceBack(MakeRay(), RaypathRecorder{});
  AppendRecorder(l2, 0, { 3, 2 });
  l2.SetChainId(0, id1);
  auto ctx2 = MakeChainIdLayerContext(table, crystal, 2, axis, FilterConfig::kSymNone);
  uint32_t id2 = InternRayChainId(ctx2, l2, 0);
  EXPECT_EQ(table.Format(id2), "crystal1(1-3-5)-crystal2(3-2)");

  // A second ray with the same two segments lands on the same ids.
  EXPECT_EQ(InternRayChainId(ctx1, l1, 0), id1);
  EXPECT_EQ(InternRayChainId(ctx2, l2, 0), id2);
  EXPECT_EQ(table.Size(), 2u);
}

// With symmetry on, the interned segment is the canonical form — the same one
// Crystal::ReduceRaypath returns when the test calls it directly — and two
// symmetric variants of one path collapse onto one id.
TEST(ChainIdFold, SymmetryReducesSegmentToTheFilterCanonicalForm) {
  ChainIdInterningTable table;
  Crystal crystal = Crystal::CreatePrism(1.0f);
  AxisDistribution axis = MakeAxis();
  auto ctx = MakeChainIdLayerContext(table, crystal, 1, axis, kSymAll);

  const std::vector<IdType> path_a = { 4, 6 };  // one rotation of the 22° path 3-5
  const std::vector<IdType> path_b = { 5, 7 };  // another
  auto expected = OracleReduce(crystal, axis, kSymAll, path_a);
  ASSERT_EQ(expected, OracleReduce(crystal, axis, kSymAll, path_b)) << "test premise: both are the same orbit";
  ASSERT_NE(expected, path_a) << "test premise: reduction must change at least one of them, else it proves nothing";

  RayBuffer buf;
  buf.Reset(4, true);
  buf.EmplaceBack(MakeRay(), RaypathRecorder{});
  buf.EmplaceBack(MakeRay(), RaypathRecorder{});
  AppendRecorder(buf, 0, path_a);
  AppendRecorder(buf, 1, path_b);
  buf.SetChainId(0, 0);
  buf.SetChainId(1, 0);

  uint32_t a = InternRayChainId(ctx, buf, 0);
  uint32_t b = InternRayChainId(ctx, buf, 1);
  EXPECT_EQ(a, b) << "symmetric variants must intern to one chain";
  EXPECT_EQ(table.EntryAt(a).segment, expected);

  // Control: kSymNone keeps them apart.
  ChainIdInterningTable raw_table;
  auto raw_ctx = MakeChainIdLayerContext(raw_table, crystal, 1, axis, FilterConfig::kSymNone);
  EXPECT_NE(InternRayChainId(raw_ctx, buf, 0), InternRayChainId(raw_ctx, buf, 1));
  EXPECT_EQ(raw_table.EntryAt(1).segment, path_a);
}

// ===========================================================================
// D. End to end through Simulator::Run().
// ===========================================================================

namespace {

// Two-layer scene: crystal id 1 on layer 0 (half the exits continue), crystal
// id 2 on layer 1 (everything exits). Both are unit hexagonal prisms under the
// plate axis. `layers == 1` drops the second layer and sets prob_ to 0.
SceneConfig MakeScene(size_t layers) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = 6;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };
  for (size_t layer = 0; layer < layers; layer++) {
    MsInfo ms;
    ms.prob_ = (layer + 1 < layers) ? 0.5f : 0.0f;
    ScatteringSetting s{};
    // No physical filter: an explicit match-all In filter, not a default-
    // constructed FilterConfig whose action_ is indeterminate.
    s.filter_.id_ = 0;
    s.filter_.symmetry_ = FilterConfig::kSymNone;
    s.filter_.action_ = FilterConfig::kFilterIn;
    s.filter_.param_ = SimpleFilterParam{ NoneFilterParam{} };
    s.crystal_.id_ = static_cast<IdType>(layer + 1);
    PrismCrystalParam prism;
    prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    for (auto& d : prism.d_) {
      d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
    }
    s.crystal_.param_ = prism;
    s.crystal_.axis_ = MakeAxis();
    s.crystal_proportion_ = 1.0f;
    ms.setting_.push_back(std::move(s));
    scene.ms_.push_back(std::move(ms));
  }
  return scene;
}

struct RunOutput {
  std::vector<SimData> batches;
  std::vector<RayBuffer> all_data;  // one snapshot per batch, same order
};

void SnapshotAllData(void* ctx, const RayBuffer& all_data) {
  static_cast<std::vector<RayBuffer>*>(ctx)->emplace_back(all_data);
}

struct AnalysisSetting {
  bool enabled = false;
  uint8_t symmetry = kSymAll;
};

// Drive Run() over `batches` batches of `ray_num` rays on ONE Simulator and
// return everything it produced.
RunOutput RunScene(const SceneConfig& scene, size_t ray_num, size_t batches, uint32_t seed,
                   const AnalysisSetting& analysis) {
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, seed);
  sim.SetAnalysisChainId(analysis.enabled, analysis.symmetry);
  RunOutput out;
  sim.SetAllDataObserverForTest(&SnapshotAllData, &out.all_data);

  auto shared_scene = std::make_shared<const SceneConfig>(scene);
  for (size_t b = 0; b < batches; ++b) {
    SimBatch batch;
    batch.ray_num_ = ray_num;
    batch.scene_ = shared_scene;
    batch.generation_ = 1;
    config_queue->Emplace(std::move(batch));
  }
  config_queue->Emplace(SimBatch{});  // ray_num_ == 0 → Run() exits

  std::thread runner([&] { sim.Run(); });
  runner.join();
  // Drain by emptiness, NOT by shutting the queue down first: Queue::Shutdown
  // discards whatever is still queued.
  while (!data_queue->Empty()) {
    out.batches.push_back(data_queue->Get());
  }
  return out;
}

// The consumer-side merge is production code now (ChainIdMerger, exercised
// by test_chain_id_merger.cpp in isolation); the end-to-end tests below drive
// it with real deltas. A delta naming a parent it never delivered is a
// contract break, so an orphan count is a failure here too.
void AbsorbOrFail(ChainIdMerger& merger, uint32_t producer, const std::vector<ChainIdTableEntry>& delta) {
  auto report = merger.Absorb(producer, delta);
  EXPECT_EQ(report.orphaned, 0u) << "producer " << producer << ": delta names a parent before delivering it";
  EXPECT_EQ(report.non_monotonic, 0u) << "producer " << producer << ": delta ids not strictly ascending";
}

// Segments of one batch's all_data that are outgoing, in buffer order — the
// SAME order SimulateOneWavelength pushes outgoing_d_/w_/chain_id_ in (both
// loops walk buffer_data[1] at the same point of the same hit iteration).
std::vector<size_t> OutgoingSegmentIndices(const RayBuffer& all_data) {
  std::vector<size_t> idx;
  for (size_t i = 0; i < all_data.size_; i++) {
    if (all_data[i].IsOutgoing()) {
      idx.push_back(i);
    }
  }
  return idx;
}

}  // namespace

TEST(ChainIdEndToEnd, DisabledLeavesOutputsEmptyAndRayStreamBitIdentical) {
  auto scene = MakeScene(2);
  auto off = RunScene(scene, 256, 2, 4242, AnalysisSetting{ false, kSymAll });
  auto on = RunScene(scene, 256, 2, 4242, AnalysisSetting{ true, kSymAll });
  ASSERT_EQ(off.batches.size(), 2u);
  ASSERT_EQ(on.batches.size(), 2u);
  for (size_t b = 0; b < 2; b++) {
    EXPECT_TRUE(off.batches[b].outgoing_chain_id_.empty()) << "analysis off must deliver no chain ids";
    EXPECT_TRUE(off.batches[b].chain_id_table_delta_.empty());
    if (off.batches[b].outgoing_w_.empty()) {
      ADD_FAILURE() << "batch " << b << ": probe measured nothing";
      continue;
    }
    // The feature draws nothing from the RNG and touches no ray field, so the
    // physical output must be the same bits whether it is on or off.
    EXPECT_EQ(on.batches[b].outgoing_w_, off.batches[b].outgoing_w_) << "batch " << b;
    EXPECT_EQ(on.batches[b].outgoing_d_, off.batches[b].outgoing_d_) << "batch " << b;
    EXPECT_EQ(on.batches[b].outgoing_component_, off.batches[b].outgoing_component_);
    EXPECT_EQ(on.batches[b].outgoing_chain_id_.size(), on.batches[b].outgoing_w_.size())
        << "analysis on: one chain id per outgoing ray";
  }
}

TEST(ChainIdEndToEnd, SingleLayerChainsHaveDepthOneAndTheCanonicalSegment) {
  auto scene = MakeScene(1);
  auto out = RunScene(scene, 512, 1, 77, AnalysisSetting{ true, kSymAll });
  ASSERT_EQ(out.batches.size(), 1u);
  ASSERT_EQ(out.all_data.size(), 1u);
  const auto& sd = out.batches[0];
  const auto& all = out.all_data[0];
  auto outgoing = OutgoingSegmentIndices(all);
  ASSERT_EQ(outgoing.size(), sd.outgoing_w_.size()) << "oracle/delivery order premise broken";
  ASSERT_EQ(sd.outgoing_chain_id_.size(), sd.outgoing_w_.size());
  ASSERT_GT(outgoing.size(), 50u);

  ChainIdMerger trie;
  AbsorbOrFail(trie, 0, sd.chain_id_table_delta_);
  size_t reduced_differs = 0;
  for (size_t k = 0; k < outgoing.size(); k++) {
    const size_t si = outgoing[k];
    const auto& seg = all[si];
    EXPECT_FLOAT_EQ(seg.w_, sd.outgoing_w_[k]) << "order premise: k-th outgoing segment is k-th delivery";
    uint32_t id = trie.Resolve(0, sd.outgoing_chain_id_[k]);
    if (id == ChainIdMerger::kUnresolved) {
      ADD_FAILURE() << "delivered id " << sd.outgoing_chain_id_[k] << " missing from the delta";
      continue;
    }
    const auto& e = trie.Table().EntryAt(id);
    EXPECT_EQ(e.parent_id, ChainIdInterningTable::kRootChainId) << "MS=1: chain depth must be exactly 1";
    EXPECT_EQ(e.crystal_id, 1u);
    auto raw = RecorderToVec(all, si);
    auto expected =
        OracleReduce(sd.crystals_[seg.crystal_idx_], sd.crystal_axis_dists_[seg.crystal_idx_], kSymAll, raw);
    EXPECT_EQ(e.segment, expected) << "ray " << k << ": segment must be the canonical form of " << SegmentString(raw);
    if (expected != raw) {
      reduced_differs++;
    }
  }
  EXPECT_GT(reduced_differs, 0u) << "the symmetry setting must actually have reduced something, else the "
                                    "check above is vacuous";
}

TEST(ChainIdEndToEnd, SymmetrySettingFlowsIntoTheSegments) {
  auto scene = MakeScene(1);
  auto out = RunScene(scene, 256, 1, 77, AnalysisSetting{ true, FilterConfig::kSymNone });
  ASSERT_EQ(out.batches.size(), 1u);
  const auto& sd = out.batches[0];
  const auto& all = out.all_data[0];
  auto outgoing = OutgoingSegmentIndices(all);
  ASSERT_EQ(outgoing.size(), sd.outgoing_chain_id_.size());
  ChainIdMerger trie;
  AbsorbOrFail(trie, 0, sd.chain_id_table_delta_);
  for (size_t k = 0; k < outgoing.size(); k++) {
    uint32_t id = trie.Resolve(0, sd.outgoing_chain_id_[k]);
    if (id == ChainIdMerger::kUnresolved) {
      ADD_FAILURE() << "delivered id " << sd.outgoing_chain_id_[k] << " missing from the delta";
      continue;
    }
    const auto& e = trie.Table().EntryAt(id);
    EXPECT_EQ(e.segment, RecorderToVec(all, outgoing[k])) << "kSymNone: segment must be the raw face sequence";
  }
}

// The AC3 multi-layer claim, per delivered ray: a ray that left from layer L
// carries a depth-L chain whose k-th node is the reduced segment of THE
// layer-k traversal this very ray made. The chain id plays no part in the
// oracle: a layer-k entry segment (reached through root_ray_idx_) still
// carries the weight the ray crossed the k-1 → k boundary with, and that
// weight identifies the layer-(k-1) continuation segment, whose recorder is
// the layer-(k-1) path and whose own root_ray_idx_ leads one layer further
// back. Walked until the root; a ray whose crossing weight is shared by two
// continuations at some layer is skipped rather than guessed.
namespace {

void VerifyChainsAgainstTheRaysOwnSegments(const SimData& sd, const RayBuffer& all, size_t layers, uint8_t symmetry) {
  auto outgoing = OutgoingSegmentIndices(all);
  ASSERT_EQ(outgoing.size(), sd.outgoing_chain_id_.size());
  // Which layer a segment belongs to is read off crystal_idx_: every layer
  // carries a deterministic shape, so the batch materialises exactly one
  // Crystal per layer, in layer order. (RaySeg::crystal_config_id_ is NOT
  // usable for this — no host path ever assigns Crystal::config_id_, so it
  // reads kInvalidId on every legacy-path segment.)
  ASSERT_EQ(sd.crystals_.size(), layers) << "premise: one crystal instance per layer";
  auto layer_of = [](const RaySeg& seg) { return static_cast<size_t>(seg.crystal_idx_); };
  auto reduce_at = [&](size_t si) {
    const auto& seg = all[si];
    return OracleReduce(sd.crystals_[seg.crystal_idx_], sd.crystal_axis_dists_[seg.crystal_idx_], symmetry,
                        RecorderToVec(all, si));
  };

  // Per layer: continuation segments keyed by the weight they crossed with.
  std::vector<std::map<float, size_t>> continuation_by_w(layers);
  std::vector<std::set<float>> ambiguous_w(layers);
  for (size_t i = 0; i < all.size_; i++) {
    const auto& seg = all[i];
    if (seg.IsContinue()) {
      size_t l = layer_of(seg);
      if (!continuation_by_w[l].emplace(seg.w_, i).second) {
        ambiguous_w[l].insert(seg.w_);
      }
    }
  }
  for (size_t l = 0; l + 1 < layers; l++) {
    if (continuation_by_w[l].size() <= 20u) {
      ADD_FAILURE() << "layer " << l << ": not enough continuations for the claim to have teeth";
      return;
    }
  }

  ChainIdMerger trie;
  AbsorbOrFail(trie, 0, sd.chain_id_table_delta_);
  std::vector<size_t> verified_at_depth(layers + 1, 0);
  size_t overflowed = 0;
  for (size_t k = 0; k < outgoing.size(); k++) {
    const size_t leaf_si = outgoing[k];
    uint32_t id = trie.Resolve(0, sd.outgoing_chain_id_[k]);
    if (id == ChainIdMerger::kUnresolved) {
      ADD_FAILURE() << "ray " << k << ": delivered id " << sd.outgoing_chain_id_[k] << " missing from the delta";
      continue;
    }
    if (id == ChainIdInterningTable::kOverflowChainId) {
      // The producer's table is bounded (ChainIdInterningTable::kDefaultCapacity)
      // and a three-layer scene exceeds it: such a ray has no chain to verify.
      // What is checked instead is below — that the batch says so.
      overflowed++;
      continue;
    }
    const size_t leaf_layer = layer_of(all[leaf_si]);
    // Walk the chain and the ray's own history in lockstep, leaf to root.
    uint32_t node_id = id;
    size_t si = leaf_si;
    size_t layer = leaf_layer;
    bool skipped = false;
    bool broken = false;
    while (true) {
      const auto& node = trie.Table().EntryAt(node_id);
      // Scene: layer L carries CrystalConfig::id_ == L+1.
      if (node.crystal_id != layer + 1 || node.segment != reduce_at(si)) {
        ADD_FAILURE() << "ray " << k << " (left from layer " << leaf_layer + 1 << "): chain " << trie.Table().Format(id)
                      << " disagrees at layer " << layer + 1 << " with the ray's own path "
                      << SegmentString(reduce_at(si)) << " on crystal " << layer + 1;
        broken = true;
        break;
      }
      if (layer == 0) {
        if (node.parent_id != ChainIdInterningTable::kRootChainId) {
          ADD_FAILURE() << "ray " << k << ": chain " << trie.Table().Format(id) << " is deeper than the ray's path";
          broken = true;
        }
        break;
      }
      if (node.parent_id == ChainIdInterningTable::kRootChainId) {
        ADD_FAILURE() << "ray " << k << ": chain " << trie.Table().Format(id) << " ends at layer " << layer + 1
                      << " but the ray came through layer " << layer;
        broken = true;
        break;
      }
      // One layer back: this layer's entry segment carries the crossing weight.
      const auto& entry_seg = all[all[si].root_ray_idx_];
      if (layer_of(entry_seg) != layer || entry_seg.root_ray_idx_ != all[si].root_ray_idx_) {
        ADD_FAILURE() << "ray " << k << ": root_ray_idx_ does not point at this ray's layer-" << layer + 1 << " entry";
        broken = true;
        break;
      }
      const float crossing_w = entry_seg.w_;
      if (ambiguous_w[layer - 1].count(crossing_w) != 0) {
        skipped = true;
        break;
      }
      auto it = continuation_by_w[layer - 1].find(crossing_w);
      if (it == continuation_by_w[layer - 1].end()) {
        ADD_FAILURE() << "ray " << k << ": no layer-" << layer << " continuation crossed with w=" << crossing_w;
        broken = true;
        break;
      }
      node_id = node.parent_id;
      si = it->second;
      layer--;
    }
    if (!skipped && !broken) {
      verified_at_depth[leaf_layer + 1]++;
    }
  }
  for (size_t d = 1; d <= layers; d++) {
    EXPECT_GT(verified_at_depth[d], 20u) << "too few depth-" << d << " chains were independently verified";
  }
  // The sentinel and the count travel together: a ray carries it iff the
  // batch reports at least one chain turned away, and never the other way.
  EXPECT_EQ(overflowed > 0, sd.chain_id_overflow_count_ > 0)
      << overflowed << " rays carry the overflow sentinel, the batch reports " << sd.chain_id_overflow_count_
      << " chains turned away";
}

}  // namespace

TEST(ChainIdEndToEnd, TwoLayerChainsMatchTheRaysOwnPerLayerSegments) {
  auto scene = MakeScene(2);
  auto out = RunScene(scene, 1024, 1, 2024, AnalysisSetting{ true, kSymAll });
  ASSERT_EQ(out.batches.size(), 1u);
  VerifyChainsAgainstTheRaysOwnSegments(out.batches[0], out.all_data[0], 2, kSymAll);
}

// Three layers is where the continuation hand-off carries a NON-root parent
// for the first time (layer 2 → 3): with two layers every continuation leaves
// layer 1 with the root id, so an omitted hand-off is invisible there.
TEST(ChainIdEndToEnd, ThreeLayerChainsMatchTheRaysOwnPerLayerSegments) {
  auto scene = MakeScene(3);
  auto out = RunScene(scene, 4096, 1, 3033, AnalysisSetting{ true, kSymAll });
  ASSERT_EQ(out.batches.size(), 1u);
  VerifyChainsAgainstTheRaysOwnSegments(out.batches[0], out.all_data[0], 3, kSymAll);
}

TEST(ChainIdEndToEnd, DeltasComposeIncrementallyAcrossBatches) {
  auto scene = MakeScene(2);
  auto out = RunScene(scene, 256, 3, 99, AnalysisSetting{ true, kSymAll });
  ASSERT_EQ(out.batches.size(), 3u);

  uint32_t next_expected = 1;
  std::set<uint32_t> delivered;
  size_t total_delta = 0;
  for (size_t b = 0; b < 3; b++) {
    const auto& sd = out.batches[b];
    for (const auto& e : sd.chain_id_table_delta_) {
      EXPECT_EQ(e.id, next_expected) << "batch " << b << ": ids must be dense and delivered in order";
      next_expected++;
      EXPECT_TRUE(e.parent_id == ChainIdInterningTable::kRootChainId || delivered.count(e.parent_id) != 0)
          << "batch " << b << ": parent " << e.parent_id << " of " << e.id << " was never delivered";
      delivered.insert(e.id);
      total_delta++;
    }
    for (uint32_t id : sd.outgoing_chain_id_) {
      EXPECT_TRUE(delivered.count(id) != 0) << "batch " << b << ": delivered id " << id << " has no entry yet";
    }
  }
  EXPECT_GT(total_delta, 0u);
  EXPECT_LT(out.batches[2].chain_id_table_delta_.size(), out.batches[0].chain_id_table_delta_.size())
      << "later batches mostly re-hit known chains, so their deltas shrink — if not, FlushDelta is re-sending";
}

// Each Run() is one session: the table restarts, so a second Run() on the same
// Simulator (recycled workspace) delivers ids from 1 again, and a Run() with
// analysis off in between neither delivers nor breaks anything.
TEST(ChainIdEndToEnd, SessionRestartsIdsAndToggleOffInBetweenIsClean) {
  auto scene = MakeScene(2);
  auto config_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(config_queue, data_queue, 5);
  auto shared_scene = std::make_shared<const SceneConfig>(scene);
  auto run_once = [&](bool enabled) {
    sim.SetAnalysisChainId(enabled, kSymAll);
    SimBatch batch;
    batch.ray_num_ = 256;
    batch.scene_ = shared_scene;
    batch.generation_ = 1;
    config_queue->Emplace(std::move(batch));
    config_queue->Emplace(SimBatch{});
    std::thread runner([&] { sim.Run(); });
    runner.join();
    std::vector<SimData> got;
    while (!data_queue->Empty()) {
      got.push_back(data_queue->Get());
    }
    return got;
  };

  auto first = run_once(true);
  ASSERT_EQ(first.size(), 1u);
  ASSERT_FALSE(first[0].chain_id_table_delta_.empty());
  EXPECT_EQ(first[0].chain_id_table_delta_[0].id, 1u);

  auto off = run_once(false);
  ASSERT_EQ(off.size(), 1u);
  EXPECT_TRUE(off[0].outgoing_chain_id_.empty());
  EXPECT_TRUE(off[0].chain_id_table_delta_.empty());
  EXPECT_FALSE(off[0].outgoing_w_.empty());

  auto second = run_once(true);
  ASSERT_EQ(second.size(), 1u);
  ASSERT_FALSE(second[0].chain_id_table_delta_.empty());
  EXPECT_EQ(second[0].chain_id_table_delta_[0].id, 1u) << "a new Run() is a new session: ids restart";
  EXPECT_EQ(second[0].outgoing_chain_id_.size(), second[0].outgoing_w_.size());
}

// AC6: two workers = two Simulators with their own tables. Their deltas merge
// into one trie by canonical key; the merged chain set is exactly the union of
// the two workers' chain sets, every delivered ray resolves, and no two
// different chains were fused (nor one chain split) by the merge.
TEST(ChainIdEndToEnd, TwoWorkerDeltasMergeByCanonicalKeyWithoutCollisionOrLoss) {
  auto scene = MakeScene(2);
  auto w0 = RunScene(scene, 512, 2, 1001, AnalysisSetting{ true, kSymAll });
  auto w1 = RunScene(scene, 512, 2, 2002, AnalysisSetting{ true, kSymAll });
  ASSERT_EQ(w0.batches.size(), 2u);
  ASSERT_EQ(w1.batches.size(), 2u);

  // Interleave the two workers' batches the way a consumer queue would.
  ChainIdMerger merged;
  for (size_t b = 0; b < 2; b++) {
    AbsorbOrFail(merged, 0, w0.batches[b].chain_id_table_delta_);
    AbsorbOrFail(merged, 1, w1.batches[b].chain_id_table_delta_);
  }

  // Per-worker chain strings, rebuilt from each worker's OWN deltas alone.
  auto own_strings = [](const RunOutput& w) {
    ChainIdMerger solo;
    std::set<std::string> strings;
    for (const auto& sd : w.batches) {
      AbsorbOrFail(solo, 0, sd.chain_id_table_delta_);
    }
    for (const auto& sd : w.batches) {
      for (uint32_t id : sd.outgoing_chain_id_) {
        strings.insert(solo.Table().Format(solo.Resolve(0, id)));
      }
    }
    return strings;
  };
  auto s0 = own_strings(w0);
  auto s1 = own_strings(w1);
  ASSERT_GT(s0.size(), 5u);
  ASSERT_GT(s1.size(), 5u);
  std::set<std::string> expected_union(s0);
  expected_union.insert(s1.begin(), s1.end());
  // The workers must actually disagree on ids for the merge to be tested:
  // some chain both saw must carry different local ids.
  bool any_id_disagreement = false;
  for (const auto& sd0 : w0.batches) {
    for (const auto& e0 : sd0.chain_id_table_delta_) {
      for (const auto& sd1 : w1.batches) {
        for (const auto& e1 : sd1.chain_id_table_delta_) {
          if (e1.id == e0.id && (e1.segment != e0.segment || e1.crystal_id != e0.crystal_id)) {
            any_id_disagreement = true;
          }
        }
      }
    }
  }
  EXPECT_TRUE(any_id_disagreement) << "premise: worker-local ids differ, so a merge that trusted them would be wrong";

  std::set<std::string> merged_strings;
  std::map<std::string, uint32_t> merged_id_of;
  for (size_t src = 0; src < 2; src++) {
    const auto& w = src == 0 ? w0 : w1;
    for (const auto& sd : w.batches) {
      for (uint32_t id : sd.outgoing_chain_id_) {
        uint32_t mid = merged.Resolve(static_cast<uint32_t>(src), id);
        if (mid == ChainIdMerger::kUnresolved) {
          ADD_FAILURE() << "worker " << src << " delivered an id the merge never saw";
          continue;
        }
        std::string s = merged.Table().Format(mid);
        merged_strings.insert(s);
        auto it = merged_id_of.find(s);
        if (it == merged_id_of.end()) {
          merged_id_of.emplace(s, mid);
        } else {
          EXPECT_EQ(it->second, mid) << "one chain, two merged ids: " << s;
        }
      }
    }
  }
  EXPECT_EQ(merged_strings, expected_union) << "merged chain set must be exactly the union of the workers' sets";
  // No chain fused: the merged table has one entry per distinct chain string
  // over BOTH leaves and interior nodes, so count distinct strings over all ids.
  std::set<std::string> all_node_strings;
  for (uint32_t id = 1; id <= merged.Table().Size(); id++) {
    all_node_strings.insert(merged.Table().Format(id));
  }
  EXPECT_EQ(all_node_strings.size(), merged.Table().Size()) << "two merged ids format to the same chain";
}

}  // namespace lumice
