// StatsConsumer unit tests (task-exit-seam-crystal-count).
//
// Locks the "crystals_ accumulates SimData.crystal_count_ (not
// crystals_.size())" contract switched in stats.cpp, and covers the
// server.cpp:834 "only-first-chunk carries stats fields" invariant that
// chunked SimData dispatch relies on: with the new field, feeding N chunks
// where only the first has crystal_count_ set must yield exactly that value
// (not N× that value, not 0).

#include <gtest/gtest.h>

#include <variant>

#include "config/sim_data.hpp"
#include "server/stats.hpp"

namespace lumice {
namespace {

TEST(StatsConsumer, CrystalCountAccumulatesAcrossBatches) {
  StatsConsumer stats;

  SimData batch1;
  batch1.crystal_count_ = 3;
  batch1.root_ray_count_ = 100;
  stats.Consume(batch1);

  SimData batch2;
  batch2.crystal_count_ = 5;
  batch2.root_ray_count_ = 200;
  stats.Consume(batch2);

  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  ASSERT_TRUE(std::holds_alternative<StatsResult>(result));
  const auto& r = std::get<StatsResult>(result);
  EXPECT_EQ(r.crystal_num_, 8u);
  EXPECT_EQ(r.sim_ray_num_, 300u);
}

TEST(StatsConsumer, ResetClearsCrystalCount) {
  StatsConsumer stats;
  SimData batch;
  batch.crystal_count_ = 7;
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

// Server-side chunk dispatch invariant (server.cpp:834): when a large SimData
// is fanned out to N chunks, ONLY the first chunk carries the stats fields
// (root_ray_count_, crystal_count_). This test simulates that dispatch and
// verifies that accumulating N chunks yields exactly the original crystal
// count once — no N× duplication, no zero.
TEST(StatsConsumer, ChunkedDispatchAccumulatesFirstChunkOnlyOnce) {
  constexpr size_t kOriginalCrystals = 4;
  constexpr size_t kOriginalRootRays = 512;
  constexpr int kChunkCount = 5;

  StatsConsumer stats;
  for (int i = 0; i < kChunkCount; i++) {
    SimData chunk;
    if (i == 0) {
      chunk.crystal_count_ = kOriginalCrystals;
      chunk.root_ray_count_ = kOriginalRootRays;
    }
    // Non-first chunks carry only per-chunk outgoing rays (not modelled here).
    stats.Consume(chunk);
  }

  stats.PrepareSnapshot();
  auto result = stats.GetResult();
  ASSERT_TRUE(std::holds_alternative<StatsResult>(result));
  const auto& r = std::get<StatsResult>(result);
  EXPECT_EQ(r.crystal_num_, kOriginalCrystals) << "First-chunk-only dispatch: expected the original crystal_count_, "
                                                  "not "
                                               << kChunkCount << "x or 0.";
  EXPECT_EQ(r.sim_ray_num_, kOriginalRootRays)
      << "First-chunk-only dispatch: root_ray_count_ must also only be counted once.";
}

}  // namespace
}  // namespace lumice
