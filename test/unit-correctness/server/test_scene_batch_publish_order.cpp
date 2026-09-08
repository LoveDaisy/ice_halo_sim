// Pins the account-before-publish order in AccountThenPublishBatch
// (server/scene_batch_publish.hpp). Single-threaded and deterministic — it does not
// reproduce the original race (stall-injection under a real scheduler does that); it pins
// the ordering property that makes the race impossible on this path: a spy "queue" records
// the counter's value at the instant Emplace is called, so reverting the two statements
// inside AccountThenPublishBatch turns this test red on every run, with no timing
// dependency.

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
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

// ---------------------------------------------------------------------------
// The inverse: DiscardQueuedBatchesThenRefund
// ---------------------------------------------------------------------------
// Used when the GPU backend is dropped mid-run: the batches already queued were sized
// for the GPU dispatch grain, and feeding them to the legacy CPU path (one host
// wavelength per batch) is the defect. They are dropped and re-emitted at the smaller
// grain, which makes this function responsible for two ledgers at once — the in-flight
// SimData counter, and the producer's own ray budget.

// Minimal double for Queue<SimBatch>: DrainAll plus the one field the refund reads.
struct FakeBatch {
  size_t ray_num_ = 0;
};

struct DrainableQueue {
  std::vector<FakeBatch> items;
  std::function<void()> on_drain;

  std::vector<FakeBatch> DrainAll() {
    if (on_drain) {
      on_drain();
    }
    std::vector<FakeBatch> out;
    out.swap(items);
    return out;
  }
};

TEST(SceneBatchDiscardRefund, RefundsBothLedgersExactly) {
  std::atomic_int cnt{ 4 * 3 };  // four queued batches, three SimData each
  DrainableQueue queue{ { { 262144 }, { 262144 }, { 262144 }, { 100 } }, {} };

  size_t dropped = 0;
  const size_t rays = lumice::DiscardQueuedBatchesThenRefund(cnt, /*credit_per_batch=*/3, queue, &dropped);

  EXPECT_EQ(dropped, 4u);
  EXPECT_EQ(rays, 262144u * 3 + 100)
      << "the producer owes this budget back to committed_num — a finite ray_num run that "
         "does not get it back traces exactly this many rays fewer than its config asked for";
  EXPECT_EQ(cnt.load(), 0) << "every discarded batch's SimData credit must come back off the in-flight counter";
}

TEST(SceneBatchDiscardRefund, LeavesOtherBatchesCreditUntouched) {
  // Batches already dequeued by a consumer are not in the queue and keep their credit —
  // ConsumeData still owes a decrement for each one.
  std::atomic_int cnt{ 10 };
  DrainableQueue queue{ { { 128 }, { 128 } }, {} };

  const size_t rays = lumice::DiscardQueuedBatchesThenRefund(cnt, /*credit_per_batch=*/1, queue);

  EXPECT_EQ(rays, 256u);
  EXPECT_EQ(cnt.load(), 8) << "only the two dropped batches' credit is refunded";
}

TEST(SceneBatchDiscardRefund, EmptyQueueIsANoOp) {
  std::atomic_int cnt{ 7 };
  DrainableQueue queue{};
  size_t dropped = 99;

  EXPECT_EQ(lumice::DiscardQueuedBatchesThenRefund(cnt, 5, queue, &dropped), 0u);
  EXPECT_EQ(dropped, 0u);
  EXPECT_EQ(cnt.load(), 7);
}

TEST(SceneBatchDiscardRefund, DiscountIsVisibleOnlyAfterTheDrain) {
  // The mirror of CreditIsVisibleBeforeEmplace, and for the mirror reason: while the
  // batches are still dequeuable the counter must not yet have given their credit back,
  // or a consumer reading it in that window can call the epoch drained with work left.
  std::atomic_int cnt{ 6 };
  int observed_at_drain = -1;
  DrainableQueue queue{ { { 1 }, { 2 } }, {} };
  queue.on_drain = [&] { observed_at_drain = cnt.load(); };

  lumice::DiscardQueuedBatchesThenRefund(cnt, /*credit_per_batch=*/3, queue);

  EXPECT_EQ(observed_at_drain, 6) << "the refund must not land before the batches leave the queue";
  EXPECT_EQ(cnt.load(), 0);
}

}  // namespace
