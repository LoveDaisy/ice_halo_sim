// Tests for src/util/queue.hpp — Queue<T> concurrency contract + DISABLED_ perf micro-benchmarks.
//
// Source facts verified against queue.hpp (see plan.md 2.4):
//   - Queue::Shutdown() swap-clears q_ and notify_all. (queue.hpp:49-52)
//   - Queue::Start() only flips shutdown_=false + notify_all; does NOT reset q_ and does NOT
//     wake consumers that are not yet in cv_.wait. (queue.hpp:55-59)
//   - Queue::Get() predicate is `!q_.empty() || shutdown_`; on shutdown returns default T(). (queue.hpp:15-30)
//   - Queue::Emplace() silently drops when shutdown_ is true. (queue.hpp:34-41)
//
// Key inference (plan 2.4 corollary A): the sequence "Shutdown → Start → Get()" with an empty q_
// deadlocks forever (wait predicate never flips). All test cases below avoid that sequence.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "util/queue.hpp"

namespace {

using lumice::Queue;

// ---------------------------------------------------------------------------
// Correctness tests
// ---------------------------------------------------------------------------

TEST(QueueTest, FifoOrderSPSC) {
  Queue<int> q;
  constexpr int kN = 100;
  for (int i = 0; i < kN; ++i) {
    q.Emplace(i);
  }
  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(q.Get(), i);
  }
}

TEST(QueueTest, EmplaceForwardsArgs) {
  // Validate variadic Emplace forwards constructor arguments (in-place construction).
  Queue<std::pair<int, std::string>> q;
  q.Emplace(42, std::string("hello"));
  q.Emplace(7, std::string("world"));

  auto a = q.Get();
  EXPECT_EQ(a.first, 42);
  EXPECT_EQ(a.second, "hello");

  auto b = q.Get();
  EXPECT_EQ(b.first, 7);
  EXPECT_EQ(b.second, "world");
}

TEST(QueueTest, GetBlocksUntilEmplace) {
  // Consumer thread calls Get() first; main thread Emplaces; consumer wakes.
  // Synchronization point is t.join(): if consumer never unblocks, join hangs the test.
  Queue<int> q;
  int received = -1;
  std::thread t([&] { received = q.Get(); });
  // Emplace happens after thread start; thanks to q_mutex_ + notify_one, the consumer either
  // enters wait first (then notify wakes it) or sees q_ non-empty directly — both correct.
  q.Emplace(123);
  t.join();
  EXPECT_EQ(received, 123);
}

TEST(QueueTest, GetReturnsDefaultOnShutdownWhenEmpty) {
  // Consumer blocks on empty q_; Shutdown wakes it; Get returns default-constructed T().
  // If consumer hasn't entered wait yet when Shutdown fires, Get's top-level `if (shutdown_)`
  // check still returns T{} immediately — the test remains correct either way.
  Queue<int> q;
  int received = 999;
  std::thread t([&] { received = q.Get(); });
  // Give the consumer a chance to reach the wait; even if it hasn't, the top-of-Get shutdown
  // check covers us.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  q.Shutdown();
  t.join();
  EXPECT_EQ(received, 0);  // default int()
}

TEST(QueueTest, EmplaceAfterShutdownIsDropped) {
  // Sentinel-value strategy (plan Step 1 #5). This single test proves BOTH contracts:
  //   (a) Queue::Shutdown() swap-clears existing elements  — 999 must be gone.
  //   (b) Queue::Emplace() after Shutdown is dropped       — 777/555 must be gone.
  //   (c) Queue::Start() re-enables Emplace                — sentinel 42 must arrive.
  // If any of (a)(b)(c) is broken, the consumer would receive a value other than 42 first
  // (FIFO ordering inside Queue<T> is preserved for successful Emplaces).
  Queue<int> q;
  q.Emplace(999);  // normal enqueue
  q.Shutdown();    // swap-clears q_; 999 gone. shutdown_=true.
  q.Emplace(777);  // dropped (shutdown_=true)
  q.Emplace(555);  // dropped
  q.Start();       // shutdown_=false; q_ still empty
  q.Emplace(42);   // sentinel: enqueued normally
  int v = q.Get();
  EXPECT_EQ(v, 42);
}

TEST(QueueTest, ShutdownIdempotent) {
  Queue<int> q;
  q.Emplace(1);
  q.Emplace(2);
  q.Shutdown();
  q.Shutdown();  // Must not deadlock.
  // Post-condition: Get on a shutdown queue returns default T().
  EXPECT_EQ(q.Get(), 0);
}

TEST(QueueTest, StartAfterShutdownResumesNormal) {
  // Race-free by lock sequencing (see plan 3.2 #7 and review v2 Minor 2):
  //   - Queue::Emplace holds q_mutex_, push into q_, release lock, then notify_one.
  //   - Queue::Get takes q_mutex_ and either sees q_ non-empty (skips wait) OR enters wait
  //     and will be woken by notify.
  // Consumer-first ordering guarantees at least one of these two paths succeeds.
  // This test tolerates weak semantics: even if the consumer has not yet entered cv_.wait
  // when main Emplaces, the post-Emplace q_ non-empty state makes Get succeed immediately.
  Queue<int> q;
  q.Shutdown();
  q.Start();

  int received = 0;
  std::thread t([&] { received = q.Get(); });
  // Give the consumer a small opportunity to reach wait. Not a correctness requirement.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  q.Emplace(7);
  t.join();
  EXPECT_EQ(received, 7);
}

TEST(QueueTest, MpmcAllItemsDelivered) {
  // MPMC contract: all produced items are consumed, ignoring order (Queue is not strictly
  // FIFO across multiple producers — one producer's items remain FIFO amongst themselves,
  // but cross-producer order is arbitrary).
  //
  // Sequence (critical — see review v1 Critical 1):
  //   1. Start 4 consumer threads (each loops Get until it receives T{}).
  //   2. Start 4 producer threads (each Emplaces 1000 items); wait producers join.
  //   3. Busy-wait (with timeout fuse) until consumed_count reaches produced_count.
  //   4. Only THEN Shutdown the queue, which wakes blocked consumers and makes them exit.
  //   5. Join consumers.
  //
  // Strict equality on Queue.consumed_count is an invariant, not a timing guess (plan 3.3).
  constexpr int kProducers = 4;
  constexpr int kConsumers = 4;
  constexpr int kPerProducer = 1000;
  constexpr int kTotal = kProducers * kPerProducer;

  Queue<int> q;
  std::mutex received_mu;
  std::vector<int> received;
  received.reserve(kTotal);
  std::atomic<int> consumed_count{ 0 };
  std::atomic<bool> producers_done{ false };

  // Consumers
  std::vector<std::thread> consumers;
  for (int c = 0; c < kConsumers; ++c) {
    consumers.emplace_back([&] {
      while (true) {
        int v = q.Get();
        if (v == 0 && producers_done.load()) {
          // After producers are done and Shutdown was called, Get returns 0 (default int)
          // on shutdown. Also accept 0 sentinel only when producers_done flag is set, to
          // avoid losing a legitimate 0 value from a producer (we use payload > 0 below).
          return;
        }
        if (v > 0) {
          {
            std::scoped_lock lk(received_mu);
            received.push_back(v);
          }
          consumed_count.fetch_add(1);
        }
      }
    });
  }

  // Producers — encode payload > 0 as (producer_id + 1) * 100000 + offset so 0 is never a
  // legitimate value. Range: 100000..400999.
  std::vector<std::thread> producers;
  for (int p = 0; p < kProducers; ++p) {
    producers.emplace_back([&, p] {
      for (int i = 0; i < kPerProducer; ++i) {
        q.Emplace((p + 1) * 100000 + i);
      }
    });
  }
  for (auto& t : producers) {
    t.join();
  }

  // Busy-wait until all items are consumed (5s fuse).
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (consumed_count.load() < kTotal) {
    if (std::chrono::steady_clock::now() > deadline) {
      FAIL() << "MPMC timeout: consumed_count=" << consumed_count.load() << " expected=" << kTotal;
      producers_done.store(true);
      q.Shutdown();
      for (auto& t : consumers) {
        t.join();
      }
      return;
    }
    std::this_thread::yield();
  }

  // All items in hand; now let consumers exit via Shutdown.
  producers_done.store(true);
  q.Shutdown();
  for (auto& t : consumers) {
    t.join();
  }

  EXPECT_EQ(consumed_count.load(), kTotal);

  // Verify set equality (no duplicates, no losses).
  std::set<int> expected;
  for (int p = 0; p < kProducers; ++p) {
    for (int i = 0; i < kPerProducer; ++i) {
      expected.insert((p + 1) * 100000 + i);
    }
  }
  std::set<int> actual(received.begin(), received.end());
  EXPECT_EQ(actual, expected);
  EXPECT_EQ(static_cast<int>(received.size()), kTotal);  // no duplicates
}

TEST(QueueTest, ConcurrentShutdownWakesBlockedConsumer) {
  // Weak-semantics test (see review v2 Minor 4): this test tolerates the case where the
  // consumers have not yet entered cv_.wait when Shutdown fires — in that case Get's
  // top-level `if (shutdown_) return T();` returns immediately. The test still catches
  // notify_all regression via the blocking path: if notify_all were removed AND consumers
  // really reached cv_.wait, the join() calls below would hang forever (with a test
  // timeout as the backstop).
  Queue<int> q;
  std::atomic<int> returned{ 0 };
  std::thread t1([&] {
    int v = q.Get();
    EXPECT_EQ(v, 0);
    returned.fetch_add(1);
  });
  std::thread t2([&] {
    int v = q.Get();
    EXPECT_EQ(v, 0);
    returned.fetch_add(1);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  q.Shutdown();
  t1.join();
  t2.join();
  EXPECT_EQ(returned.load(), 2);
}

// ---------------------------------------------------------------------------
// Performance micro-benchmarks (DISABLED_ by default; manual invocation only)
//
// Run manually:
//   ./build/cmake_install/unit_test --gtest_also_run_disabled_tests --gtest_filter='*Perf*'
//
// Verified: scripts/build.sh:21 invokes `ctest` without --gtest_also_run_disabled_tests,
// so these tests never run in CI.
// ---------------------------------------------------------------------------

TEST(QueuePerfTest, DISABLED_PerfSpscPushPop) {
  // Single producer, single consumer, 1M push/pop cycles. Report average ns/op.
  constexpr int kN = 1'000'000;
  Queue<int> q;

  // Warm-up: 10k iterations to avoid first-touch / allocator effects.
  for (int i = 0; i < 10'000; ++i) {
    q.Emplace(i);
  }
  for (int i = 0; i < 10'000; ++i) {
    (void)q.Get();
  }

  std::atomic<bool> producer_ready{ false };
  auto start = std::chrono::steady_clock::now();
  std::thread producer([&] {
    producer_ready.store(true);
    for (int i = 0; i < kN; ++i) {
      q.Emplace(i + 1);  // non-zero payload
    }
  });
  while (!producer_ready.load()) {
    std::this_thread::yield();
  }
  for (int i = 0; i < kN; ++i) {
    (void)q.Get();
  }
  producer.join();
  auto end = std::chrono::steady_clock::now();
  auto total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  double ns_per_op = static_cast<double>(total_ns) / (kN * 2);  // push + pop = 2 ops
  std::cout << "[PERF] queue_spsc_push_pop_ns_per_op: " << ns_per_op << " ns" << std::endl;
}

TEST(QueuePerfTest, DISABLED_PerfMpscPushPop) {
  // 4 producers x 250k items each, 1 consumer. Report Mops/s aggregate throughput.
  constexpr int kProducers = 4;
  constexpr int kPerProducer = 250'000;
  constexpr int kTotal = kProducers * kPerProducer;
  Queue<int> q;

  // Warm-up
  for (int i = 0; i < 10'000; ++i) {
    q.Emplace(i);
  }
  for (int i = 0; i < 10'000; ++i) {
    (void)q.Get();
  }

  auto start = std::chrono::steady_clock::now();
  std::vector<std::thread> producers;
  for (int p = 0; p < kProducers; ++p) {
    producers.emplace_back([&, p] {
      for (int i = 0; i < kPerProducer; ++i) {
        q.Emplace((p + 1) * 1'000'000 + i);
      }
    });
  }
  int consumed = 0;
  while (consumed < kTotal) {
    (void)q.Get();
    ++consumed;
  }
  for (auto& t : producers) {
    t.join();
  }
  auto end = std::chrono::steady_clock::now();
  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  double mops = (static_cast<double>(kTotal) * 2 /* push + pop */) / (total_ms * 1000.0);
  std::cout << "[PERF] queue_mpsc_throughput_mops_per_s: " << mops << " Mops/s" << " (total_ms=" << total_ms << ")"
            << std::endl;
}

}  // namespace
