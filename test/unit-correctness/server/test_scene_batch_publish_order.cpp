// Pins the account-before-publish order in AccountThenPublishBatch
// (server/scene_batch_publish.hpp). Single-threaded and deterministic — it does not
// reproduce the original race (stall-injection under a real scheduler does that); it pins
// the ordering property that makes the race impossible on this path: a spy "queue" records
// the counter's value at the instant Emplace is called, so reverting the two statements
// inside AccountThenPublishBatch turns this test red on every run, with no timing
// dependency.

#include <gtest/gtest.h>

#include <atomic>
#include <functional>
#include <vector>

#include "server/scene_batch_publish.hpp"

namespace {

using lumice::AccountThenPublishBatch;

// Minimal double matching Queue<T>::Emplace's variadic-forwarding signature. Records the
// shared counter's value at the moment Emplace is called — that is the property under
// test, not what gets stored.
struct SpyQueue {
  std::function<void()> on_emplace;
  int emplace_count = 0;

  template <class... Args>
  void Emplace(Args&&... /*args*/) {
    ++emplace_count;
    if (on_emplace) {
      on_emplace();
    }
  }
};

TEST(SceneBatchPublishOrder, CreditIsVisibleBeforeEmplace) {
  std::atomic_int cnt{ 0 };
  SpyQueue queue;
  int observed_at_emplace = -1;
  queue.on_emplace = [&] { observed_at_emplace = cnt.load(); };

  AccountThenPublishBatch(cnt, /*credit=*/5, queue, 42);

  EXPECT_EQ(observed_at_emplace, 5)
      << "sim_scene_cnt_ must already reflect this batch's credit by the time the batch "
         "reaches the queue — a consumer racing to drain it must never see the pre-credit count";
  EXPECT_EQ(cnt.load(), 5);
  EXPECT_EQ(queue.emplace_count, 1);
}

TEST(SceneBatchPublishOrder, AccumulatesAcrossMultipleBatches) {
  // Mirrors GenerateScene's loop: each call adds its own credit on top of what is already
  // outstanding, and every Emplace must observe the running total.
  std::atomic_int cnt{ 0 };
  SpyQueue queue;
  std::vector<int> observed;
  queue.on_emplace = [&] { observed.push_back(cnt.load()); };

  AccountThenPublishBatch(cnt, 3, queue, 1);
  AccountThenPublishBatch(cnt, 4, queue, 2);
  AccountThenPublishBatch(cnt, 2, queue, 3);

  ASSERT_EQ(observed.size(), 3u);
  EXPECT_EQ(observed[0], 3);
  EXPECT_EQ(observed[1], 7);
  EXPECT_EQ(observed[2], 9);
}

}  // namespace
