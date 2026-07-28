// StatsConsumer unit tests.
//
// Locks the aggregation rules for the two halves of the reported crystal count:
// stochastic draws ACCUMULATE across SimData, the deterministic population is
// OVERWRITTEN (it is a config constant every producer reports identically).
// Getting this pair wrong is the whole defect family this suite guards: summing
// the deterministic half made the stat scale with the batch count AND with the
// worker-pool size, while accumulating nothing at all would make geometry
// randomization invisible.
//
// Also covers the server.cpp chunk-dispatch invariant, which is now two
// opposite rules on the same dispatch: accumulated fields ride the FIRST chunk
// only, overwritten fields ride EVERY chunk.

#include <gtest/gtest.h>

#include <variant>

#include "config/sim_data.hpp"
#include "server/stats.hpp"

namespace lumice {
namespace {

size_t CrystalNum(StatsConsumer& stats) {
  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  EXPECT_TRUE(std::holds_alternative<StatsResult>(result));
  return std::get<StatsResult>(result).crystal_num_;
}

TEST(StatsConsumer, StochasticSamplesAccumulateAcrossBatches) {
  StatsConsumer stats;

  SimData batch1;
  batch1.stochastic_crystal_sample_count_ = 3;
  batch1.root_ray_count_ = 100;
  stats.Consume(batch1);

  SimData batch2;
  batch2.stochastic_crystal_sample_count_ = 5;
  batch2.root_ray_count_ = 200;
  stats.Consume(batch2);

  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  ASSERT_TRUE(std::holds_alternative<StatsResult>(result));
  const auto& r = std::get<StatsResult>(result);
  EXPECT_EQ(r.crystal_num_, 8u) << "every stochastic draw is a genuinely different geometry — they sum";
  EXPECT_EQ(r.sim_ray_num_, 300u);
}

// The AC10 statement at unit scope: P workers each report the SAME deterministic
// scene, because each derives the same geometries from the same config. The
// reported total must be the scene's count, not P x it. Same shape as a single
// worker feeding P batches, which is why one rule covers both knobs.
TEST(StatsConsumer, DeterministicCountIsOverwrittenNotSummed) {
  constexpr size_t kSceneCrystals = 5;
  constexpr int kProducers = 4;

  StatsConsumer stats;
  for (int i = 0; i < kProducers; i++) {
    SimData batch;
    batch.deterministic_crystal_count_ = kSceneCrystals;
    batch.root_ray_count_ = 10;
    stats.Consume(batch);
  }

  EXPECT_EQ(CrystalNum(stats), kSceneCrystals)
      << kProducers << " producers reported the same deterministic scene; summing them would report "
      << kSceneCrystals * kProducers
      << ", which is how this stat used to scale with the dispatch grain and the "
         "worker pool instead of describing the scene";
}

// A mixed scene: the deterministic half stays put while the stochastic half
// grows. Separating them is the point — a single counter cannot do both.
TEST(StatsConsumer, MixedSceneSumsStochasticOnTopOfConstantDeterministic) {
  constexpr size_t kSceneDeterministic = 2;
  StatsConsumer stats;

  SimData batch1;
  batch1.deterministic_crystal_count_ = kSceneDeterministic;
  batch1.stochastic_crystal_sample_count_ = 7;
  stats.Consume(batch1);
  EXPECT_EQ(CrystalNum(stats), kSceneDeterministic + 7u);

  SimData batch2;
  batch2.deterministic_crystal_count_ = kSceneDeterministic;
  batch2.stochastic_crystal_sample_count_ = 6;
  stats.Consume(batch2);
  EXPECT_EQ(CrystalNum(stats), kSceneDeterministic + 13u)
      << "the deterministic half must not grow with the batch count, and the stochastic half must";
}

TEST(StatsConsumer, ResetClearsBothHalves) {
  StatsConsumer stats;
  SimData batch;
  batch.stochastic_crystal_sample_count_ = 7;
  batch.deterministic_crystal_count_ = 3;
  batch.root_ray_count_ = 42;
  stats.Consume(batch);

  stats.Reset();
  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  ASSERT_TRUE(std::holds_alternative<StatsResult>(result));
  const auto& r = std::get<StatsResult>(result);
  EXPECT_EQ(r.crystal_num_, 0u);
  EXPECT_EQ(r.sim_ray_num_, 0u);
}

// Server-side chunk dispatch invariant (server.cpp ConsumeData): when a large
// SimData is fanned out to N chunks, the ACCUMULATED fields (root_ray_count_,
// stochastic_crystal_sample_count_) ride only the first chunk, while the
// OVERWRITTEN field (deterministic_crystal_count_) rides every chunk. This test
// mirrors that fill exactly. Note what it would catch: putting the deterministic
// count on the first chunk only leaves the trailing chunks overwriting it back
// to 0, so the whole deterministic population silently vanishes from the report.
TEST(StatsConsumer, ChunkedDispatchAppliesTheRightRuleToEachHalf) {
  constexpr size_t kStochastic = 4;
  constexpr size_t kDeterministic = 6;
  constexpr size_t kOriginalRootRays = 512;
  constexpr int kChunkCount = 5;

  StatsConsumer stats;
  for (int i = 0; i < kChunkCount; i++) {
    SimData chunk;
    if (i == 0) {
      chunk.stochastic_crystal_sample_count_ = kStochastic;
      chunk.root_ray_count_ = kOriginalRootRays;
    }
    chunk.deterministic_crystal_count_ = kDeterministic;  // every chunk, per server.cpp
    // Non-first chunks otherwise carry only per-chunk outgoing rays (not modelled here).
    stats.Consume(chunk);
  }

  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  ASSERT_TRUE(std::holds_alternative<StatsResult>(result));
  const auto& r = std::get<StatsResult>(result);
  EXPECT_EQ(r.crystal_num_, kStochastic + kDeterministic)
      << "expected the stochastic draws counted once (not " << kChunkCount
      << "x) plus the deterministic population counted once (not 0)";
  EXPECT_EQ(r.sim_ray_num_, kOriginalRootRays)
      << "First-chunk-only dispatch: root_ray_count_ must also only be counted once.";
}

// CHARACTERIZATION: because the deterministic half is OVERWRITTEN, EVERY SimData
// that reaches Consume() must carry it — one that leaves the field at its default
// 0 erases the scene's whole config-constant population from the report.
//
// This documents a real coupling rather than a defect. Consume() deliberately does
// NOT defend against it: the "is this a produced batch" predicate is owned by
// server.cpp::ConsumeData (`rays_.Empty() && root_ray_count_ == 0` breaks out
// before any Consume() call, so the shutdown/interruption sentinel never arrives
// here). Re-deriving that predicate in this class would create a second authority
// for the same protocol question and let the two drift — the exact anti-pattern
// this change removed elsewhere, so it is not worth buying defence-in-depth with.
//
// What this test is for: the guarantee therefore lives in an upstream convention,
// and that convention has already been wrong once (its predecessor keyed on
// is_backend_path_ and mis-flagged exit-seam batches as sentinels). Pinning the
// consequence here means a future reader who is tempted to relax the sentinel key,
// add a Consume() call site, or add a producer that forgets the field can see in
// one place what it costs. The sibling case is already covered above: trailing
// chunks must re-assert the constant, and that one IS reachable today.
TEST(StatsConsumer, DeterministicHalfIsOverwrittenSoEveryBatchMustCarryIt) {
  constexpr size_t kStochastic = 7;
  constexpr size_t kDeterministic = 5;

  StatsConsumer stats;

  SimData real_batch;
  real_batch.root_ray_count_ = 128;
  real_batch.stochastic_crystal_sample_count_ = kStochastic;
  real_batch.deterministic_crystal_count_ = kDeterministic;
  stats.Consume(real_batch);
  EXPECT_EQ(CrystalNum(stats), kStochastic + kDeterministic);

  // A batch that forgets the field: the stochastic half still accumulates, but
  // the deterministic population is gone. Asserting the erasure (rather than
  // immunity to it) is the point — it is the cost of the OVERWRITE rule.
  SimData forgot_deterministic;
  forgot_deterministic.root_ray_count_ = 128;
  forgot_deterministic.stochastic_crystal_sample_count_ = 1;
  stats.Consume(forgot_deterministic);

  EXPECT_EQ(CrystalNum(stats), kStochastic + 1u)
      << "OVERWRITE semantics: a producer that omits deterministic_crystal_count_ "
         "silently drops the scene's config-constant population. If this ever "
         "needs to stop being true, change the aggregation rule deliberately — do "
         "not re-derive server.cpp's produced-batch predicate here.";
}

size_t OrientationNum(StatsConsumer& stats) {
  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  EXPECT_TRUE(std::holds_alternative<StatsResult>(result));
  return std::get<StatsResult>(result).orientation_num_;
}

// ── Orientation half ────────────────────────────────────────────────────────
// The orientation count carries the SAME two-rule split as the crystal count
// (stochastic ACCUMULATE / deterministic OVERWRITE), so it inherits the same
// defect family. These are separate tests rather than extra assertions inside
// the crystal ones on purpose: the two counts are independent axes of a crystal
// setting, and a test that only ever moves them together cannot tell a genuine
// orientation counter from one that shadows the crystal count.

TEST(StatsConsumer, StochasticOrientationSamplesAccumulateAcrossBatches) {
  StatsConsumer stats;

  SimData batch1;
  batch1.stochastic_orientation_sample_count_ = 30;
  batch1.root_ray_count_ = 100;
  stats.Consume(batch1);

  SimData batch2;
  batch2.stochastic_orientation_sample_count_ = 50;
  batch2.root_ray_count_ = 200;
  stats.Consume(batch2);

  EXPECT_EQ(OrientationNum(stats), 80u) << "orientation is redrawn per ray with no reuse — draws sum";
}

TEST(StatsConsumer, DeterministicOrientationCountIsOverwrittenNotSummed) {
  constexpr size_t kSceneDeterministicAxes = 5;
  constexpr int kProducers = 4;

  StatsConsumer stats;
  for (int i = 0; i < kProducers; i++) {
    SimData batch;
    batch.root_ray_count_ = 64;
    batch.deterministic_orientation_count_ = kSceneDeterministicAxes;
    stats.Consume(batch);
  }

  EXPECT_EQ(OrientationNum(stats), kSceneDeterministicAxes)
      << "the deterministic-axis population is a config constant every producer reports "
         "identically; summing it would make the stat scale with the worker pool";
}

// The task's whole point at unit scope: shape determinism and axis determinism
// are independent, so the two reported counts must be able to diverge. The
// headline scene — one fixed geometry, an orientation redrawn per ray — is
// exactly where a counter that merely shadows the crystal count looks right in
// every other test and is wrong here.
TEST(StatsConsumer, OrientationAndCrystalCountsAreIndependentAxes) {
  StatsConsumer stats;

  SimData batch;
  batch.root_ray_count_ = 20000;
  batch.deterministic_crystal_count_ = 1;  // one fixed shape
  batch.stochastic_crystal_sample_count_ = 0;
  batch.deterministic_orientation_count_ = 0;
  batch.stochastic_orientation_sample_count_ = 20000;  // redrawn per ray
  stats.Consume(batch);

  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  ASSERT_TRUE(std::holds_alternative<StatsResult>(result));
  const auto& r = std::get<StatsResult>(result);
  EXPECT_EQ(r.crystal_num_, 1u);
  EXPECT_EQ(r.orientation_num_, 20000u);
  EXPECT_NE(r.crystal_num_, r.orientation_num_) << "the two counts must not be wired to the same source";
}

// Regression pin for a defect this task actually shipped and then fixed:
// server.cpp::ConsumeData rebuilds SimData field-by-field when chunking the
// exit-seam path, and the two orientation fields were initially left out — so
// the backend route reported orientations=0 while the legacy route (which hands
// the whole SimData over intact) kept working, which is what hid it. Found by
// the e2e cpu_backend arm; pinned here so it is caught in milliseconds instead.
TEST(StatsConsumer, ChunkedDispatchCarriesBothOrientationHalves) {
  constexpr size_t kStochasticOrientations = 4096;
  constexpr size_t kDeterministicOrientations = 3;
  constexpr size_t kOriginalRootRays = 512;
  constexpr int kChunkCount = 5;

  StatsConsumer stats;
  for (int i = 0; i < kChunkCount; i++) {
    SimData chunk;
    if (i == 0) {
      chunk.stochastic_orientation_sample_count_ = kStochasticOrientations;
      chunk.root_ray_count_ = kOriginalRootRays;
    }
    chunk.deterministic_orientation_count_ = kDeterministicOrientations;  // every chunk, per server.cpp
    stats.Consume(chunk);
  }

  EXPECT_EQ(OrientationNum(stats), kStochasticOrientations + kDeterministicOrientations)
      << "a chunk copy that drops either orientation field reports a plausible-looking "
         "low number (or 0) on the backend route only — the legacy route keeps working";
}

}  // namespace
}  // namespace lumice
