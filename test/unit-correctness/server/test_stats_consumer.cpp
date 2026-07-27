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

}  // namespace
}  // namespace lumice
