// Tests for src/util/threading_pool.hpp/cpp — ThreadingPool concurrency contract
// + DISABLED_ perf micro-benchmarks.
//
// Source facts verified against threading_pool.{hpp,cpp} (see plan.md 2.4):
//   F1.  CreatePool(size<=0) returns nullptr                     — threading_pool.cpp:24-29
//   F2.  StartPool blocks until running_workers_ >= n, THEN
//        sets state_=kIdle (not a transient kStarting observable) — threading_pool.cpp:32-43
//   F3.  kCommittable=1u; kIdle=3(011b); kRunning=5(101b) both
//        satisfy (state & kCommittable) != 0. kStarting=2,
//        kStopping=4, kTerminated=6(110b) all satisfy (state&1)==0 — threading_pool.hpp:35-42
//   F4.  Shutdown completion leaves state_ == kTerminated          — threading_pool.cpp:87
//   F5.  CommitSingleJob returns false when !(state_ & kCommittable) — threading_pool.cpp:150-170
//   F6.  CommitSingleJob / WaitFinish / Shutdown access
//        job_future_list_ WITHOUT a lock — ALL test code must call
//        Commit*/WaitFinish/Shutdown from a SINGLE thread.
//   F7.  CommitRangeStepJobs / CommitRangeSliceJobs return false both
//        on start>=end and when CommitSingleJob returns false (&&
//        short-circuit propagation)                                — threading_pool.cpp:101-139
//   F8.  ~ThreadingPool() calls Shutdown()                          — threading_pool.cpp:14-16
//   F(TOCTOU). CommitSingleJob's lock-free state check then in-lock
//        stop_flag_ check (cpp:151, 159-161) is correct: if Shutdown
//        races in after the state check, the in-lock branch leaves
//        res=false so the return is still false — the
//        CommitAfterShutdownReturnsFalse test relies on this.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "util/threading_pool.hpp"

namespace {

using lumice::ThreadingPool;

// ---------------------------------------------------------------------------
// Correctness tests — 16 cases
//
// All tests call Commit*/WaitFinish/Shutdown from the main test thread only.
// This is required by F6 (job_future_list_ has no lock).
// ---------------------------------------------------------------------------

TEST(ThreadingPoolTest, CreateWithZeroOrNegativeSizeReturnsNullptr) {
  EXPECT_EQ(ThreadingPool::CreatePool(0), nullptr);
  EXPECT_EQ(ThreadingPool::CreatePool(-1), nullptr);
}

TEST(ThreadingPoolTest, CreateWithSpecificSizeHasMatchingPoolSize) {
  auto pool = ThreadingPool::CreatePool(4);
  ASSERT_NE(pool, nullptr);
  EXPECT_EQ(pool->GetPoolSize(), 4u);
}

TEST(ThreadingPoolTest, DefaultCreateHasNonZeroPoolSize) {
  auto pool = ThreadingPool::CreatePool();
  ASSERT_NE(pool, nullptr);
  EXPECT_GT(pool->GetPoolSize(), 0u);
}

TEST(ThreadingPoolTest, StateAfterCreateIsCommittable) {
  // Per F2, CreatePool returns only after StartPool has set state_=kIdle. We assert:
  //   (a) byte-position bit test using kCommittable (the source-intended way to
  //       query committability per F3), and
  //   (b) the concrete kIdle value as a secondary sanity check tied to F2.
  // We do NOT externally assert kStarting/kStopping/kRunning — they are transient.
  auto pool = ThreadingPool::CreatePool(2);
  ASSERT_NE(pool, nullptr);
  EXPECT_NE(pool->GetState() & ThreadingPool::kCommittable, 0u);
  EXPECT_EQ(pool->GetState(), ThreadingPool::kIdle);
}

TEST(ThreadingPoolTest, StateAfterShutdownIsTerminated) {
  auto pool = ThreadingPool::CreatePool(2);
  ASSERT_NE(pool, nullptr);
  pool->Shutdown();
  EXPECT_EQ(pool->GetState(), ThreadingPool::kTerminated);
}

TEST(ThreadingPoolTest, CommitSingleJobRunsJob) {
  auto pool = ThreadingPool::CreatePool(2);
  std::atomic<int> counter{ 0 };
  EXPECT_TRUE(pool->CommitSingleJob([&](int) { counter.fetch_add(1); }));
  pool->WaitFinish();
  EXPECT_EQ(counter.load(), 1);
}

TEST(ThreadingPoolTest, CommitManyJobsAllRun) {
  // Note: all Commit and WaitFinish calls happen on the main test thread — this
  // serializes access to job_future_list_ per F6.
  auto pool = ThreadingPool::CreatePool(4);
  constexpr int kN = 200;
  std::atomic<int> counter{ 0 };
  for (int i = 0; i < kN; ++i) {
    EXPECT_TRUE(pool->CommitSingleJob([&](int) { counter.fetch_add(1); }));
  }
  pool->WaitFinish();
  EXPECT_EQ(counter.load(), kN);
}

TEST(ThreadingPoolTest, WaitFinishOnEmptyIsNoop) {
  auto pool = ThreadingPool::CreatePool(2);
  // Execute WaitFinish on a separate thread with a 1-second timeout backstop.
  // That thread only calls WaitFinish (never any Commit*) to avoid racing with the
  // main thread's Commit state — F6 requires serial access to job_future_list_.
  auto fut = std::async(std::launch::async, [&] { pool->WaitFinish(); });
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready)
      << "WaitFinish on idle pool should return immediately";
}

TEST(ThreadingPoolTest, CommitAfterShutdownReturnsFalse) {
  // Note: kStarting (2u), kStopping (4u), kTerminated (6u) all satisfy
  // (state & kCommittable) == 0 per F3. Only kTerminated is externally observable
  // after Shutdown completes; the other two are transient. This test represents
  // the general "non-committable" contract via the kTerminated case.
  auto pool = ThreadingPool::CreatePool(2);
  pool->Shutdown();
  std::atomic<bool> ran{ false };
  EXPECT_FALSE(pool->CommitSingleJob([&](int) { ran.store(true); }));
  EXPECT_FALSE(ran.load());
}

TEST(ThreadingPoolTest, RangeCommitAfterShutdownReturnsFalse) {
  // Per F7, both Range Commit variants chain-propagate CommitSingleJob's false via
  // &&-short-circuit when state is kTerminated. We use a legal range (start<end)
  // so the false cannot come from the start>=end early-return branch.
  // Sanity: pre-Shutdown these calls succeed.
  auto pool = ThreadingPool::CreatePool(2);
  std::atomic<int> pre_shutdown_count{ 0 };
  EXPECT_TRUE(pool->CommitRangeStepJobsAndWait(0, 10, [&](int, int) { pre_shutdown_count.fetch_add(1); }));
  EXPECT_EQ(pre_shutdown_count.load(), 10);

  pool->Shutdown();
  std::atomic<int> post_shutdown_count{ 0 };
  EXPECT_FALSE(pool->CommitRangeStepJobs(0, 100, [&](int, int) { post_shutdown_count.fetch_add(1); }));
  EXPECT_FALSE(pool->CommitRangeSliceJobs(0, 100, [&](int, int, int) { post_shutdown_count.fetch_add(1); }));
  EXPECT_EQ(post_shutdown_count.load(), 0);
}

TEST(ThreadingPoolTest, ShutdownIsIdempotent) {
  auto pool = ThreadingPool::CreatePool(2);
  const size_t kExpectedSize = pool->GetPoolSize();
  pool->Shutdown();
  pool->Shutdown();  // must not deadlock
  // Post-condition: state and pool_size unchanged.
  EXPECT_EQ(pool->GetState(), ThreadingPool::kTerminated);
  EXPECT_EQ(pool->GetPoolSize(), kExpectedSize);
}

TEST(ThreadingPoolTest, DestructorDoesNotHang) {
  // Only asserts: pool destruction (which implicitly calls Shutdown per F8) does
  // NOT hang when a job is currently executing, as long as the job itself exits on
  // an external flag. Test does NOT assert that Shutdown was called by a specific
  // mechanism — it just verifies "destruction completes within timeout".
  //
  // Three-step sequence per review v2 Minor 1:
  //   (1) CommitSingleJob(spin_until_flag)
  //   (2) set flag=true to let the spin job return
  //   (3) async pool.reset(); wait_for(2s) ASSERT != timeout
  std::atomic<bool> release_job{ false };
  auto pool = ThreadingPool::CreatePool(2);
  ASSERT_TRUE(pool->CommitSingleJob([&](int) {
    while (!release_job.load()) {
      std::this_thread::yield();
    }
  }));

  // Step 2: let the spin job exit on its own. Without this, the worker stays in
  // curr_job(idx) forever and Shutdown's worker_cv_.wait(running_workers_<=0) hangs.
  release_job.store(true);

  // Step 3: destroy the pool asynchronously and wait up to 2 seconds.
  auto fut = std::async(std::launch::async, [&] { pool.reset(); });
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready)
      << "pool destructor hung; worker likely stuck in spin job or Shutdown sequence";
}

TEST(ThreadingPoolTest, RangeStepJobsCoverAllIndices) {
  constexpr int kN = 100;
  auto pool = ThreadingPool::CreatePool(4);
  std::vector<std::atomic<int>> visits(kN);
  for (auto& v : visits) {
    v.store(0);
  }
  EXPECT_TRUE(pool->CommitRangeStepJobsAndWait(0, kN, [&](int, int j) { visits[j].fetch_add(1); }));
  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(visits[i].load(), 1) << "index " << i;
  }
}

TEST(ThreadingPoolTest, RangeSliceJobsCoverAllIndices) {
  constexpr int kN = 100;
  auto pool = ThreadingPool::CreatePool(4);
  std::vector<std::atomic<int>> visits(kN);
  for (auto& v : visits) {
    v.store(0);
  }
  EXPECT_TRUE(pool->CommitRangeSliceJobsAndWait(0, kN, [&](int, int start, int end) {
    for (int j = start; j < end; ++j) {
      visits[j].fetch_add(1);
    }
  }));
  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(visits[i].load(), 1) << "index " << i;
  }
}

TEST(ThreadingPoolTest, RangeJobsEmptyRangeReturnsFalse) {
  auto pool = ThreadingPool::CreatePool(2);
  EXPECT_FALSE(pool->CommitRangeStepJobs(5, 5, [](int, int) {}));
  EXPECT_FALSE(pool->CommitRangeSliceJobs(5, 5, [](int, int, int) {}));
}

TEST(ThreadingPoolTest, AllWorkersParticipate) {
  // Barrier pattern: each job enters a cv.wait until counter == pool_size, forcing
  // every worker to be in-flight simultaneously. This strictly tests "all workers
  // are eligible to pick up work" — weaker to-the-metal than "at least 2" assertions.
  //
  // A 5-second wait_until backstop converts any anomaly into a FAIL instead of a hang.
  constexpr int kPoolSize = 4;
  auto pool = ThreadingPool::CreatePool(kPoolSize);
  ASSERT_EQ(pool->GetPoolSize(), static_cast<size_t>(kPoolSize));

  std::atomic<int> arrived{ 0 };
  std::mutex gate_mu;
  std::condition_variable gate_cv;
  std::mutex id_mu;
  std::unordered_set<std::thread::id> ids;

  for (int i = 0; i < kPoolSize; ++i) {
    pool->CommitSingleJob([&](int) {
      {
        std::scoped_lock lk(id_mu);
        ids.insert(std::this_thread::get_id());
      }
      arrived.fetch_add(1);
      gate_cv.notify_all();
      std::unique_lock<std::mutex> lk(gate_mu);
      const bool kReached = gate_cv.wait_until(lk, std::chrono::steady_clock::now() + std::chrono::seconds(5),
                                               [&] { return arrived.load() >= kPoolSize; });
      EXPECT_TRUE(kReached) << "barrier timeout: arrived=" << arrived.load();
    });
  }
  pool->WaitFinish();
  EXPECT_EQ(ids.size(), static_cast<size_t>(kPoolSize));
}

// ---------------------------------------------------------------------------
// Performance micro-benchmarks (DISABLED_ by default; manual invocation only)
//
// Run manually:
//   ./build/Release/bin/unit_test --gtest_also_run_disabled_tests \
//       --gtest_filter='*Perf*' examples/config_example.json /tmp
//
// Verified: scripts/build.sh:21 invokes `ctest` without
// --gtest_also_run_disabled_tests, so these tests never run in CI.
// ---------------------------------------------------------------------------

TEST(ThreadingPoolPerfTest, DISABLED_PerfEmptyJobScheduling) {
  // Submit 100k empty jobs; report mean per-job scheduling overhead (median of 3).
  constexpr int kJobs = 100'000;
  auto pool = ThreadingPool::CreatePool();
  std::vector<double> samples;
  samples.reserve(3);
  for (int round = 0; round < 3; ++round) {
    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kJobs; ++i) {
      pool->CommitSingleJob([](int) {});
    }
    pool->WaitFinish();
    auto end = std::chrono::steady_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    samples.push_back(static_cast<double>(us) / kJobs);
  }
  std::sort(samples.begin(), samples.end());
  std::cout << "[PERF] threading_pool_empty_job_us_per_job (median of 3): " << samples[1] << " us" << std::endl;
}

TEST(ThreadingPoolPerfTest, DISABLED_PerfRangeStepThroughput) {
  constexpr int kN = 1'000'000;
  auto pool = ThreadingPool::CreatePool();
  auto start = std::chrono::steady_clock::now();
  pool->CommitRangeStepJobsAndWait(0, kN, [](int, int) {});
  auto end = std::chrono::steady_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  double ns_per_item = static_cast<double>(ns) / kN;
  std::cout << "[PERF] threading_pool_range_step_ns_per_item: " << ns_per_item << " ns" << std::endl;
}

TEST(ThreadingPoolPerfTest, DISABLED_PerfScalingCurve) {
  // Adaptive workload (see plan Step 3 #3):
  //   - Warm-up on a single-thread pool with initial item count.
  //   - Double/halve item count up to 3 times until single-thread wall_ms is in
  //     [500, 5000]. If still out of range, GTEST_SKIP.
  //   - Then run scaling at thread_count ∈ {1, 2, 4, 8, hw_conc}, printing
  //     wall_ms and speedup per configuration.
  // Also: clamp hardware_concurrency() to at least 1 (per review v2 Minor 3 —
  // the standard allows it to return 0 if it cannot detect).
  auto workload_fn = [](int iterations) {
    return [iterations](int, int) {
      volatile double acc = 1.0;
      for (int k = 0; k < iterations; ++k) {
        acc = acc * 1.0000001 + 0.5;
      }
      (void)acc;
    };
  };
  constexpr int kInnerIters = 1000;

  int items = 10'000;
  long long warmup_ms = 0;
  for (int attempt = 0; attempt < 4; ++attempt) {
    auto warmup_pool = ThreadingPool::CreatePool(1);
    auto t0 = std::chrono::steady_clock::now();
    warmup_pool->CommitRangeStepJobsAndWait(0, items, workload_fn(kInnerIters));
    auto t1 = std::chrono::steady_clock::now();
    warmup_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    if (warmup_ms >= 500 && warmup_ms <= 5000) {
      break;
    }
    if (warmup_ms < 500) {
      items *= 2;
    } else {
      items /= 2;
    }
    if (items <= 0) {
      items = 1;
    }
  }
  std::cout << "[PERF] scaling_warmup_items=" << items << " single_thread_ms=" << warmup_ms << std::endl;
  if (warmup_ms > 5000) {
    GTEST_SKIP() << "single-thread warm-up exceeds 5s, scaling test skipped";
  }

  unsigned hw_conc = std::thread::hardware_concurrency();
  if (hw_conc == 0) {
    hw_conc = 1;  // per review v2 Minor 3
  }
  const std::vector<int> kThreadCounts = { 1, 2, 4, 8, static_cast<int>(hw_conc) };
  long long single_ms = 0;
  for (int tc : kThreadCounts) {
    if (tc <= 0) {
      continue;
    }
    auto p = ThreadingPool::CreatePool(tc);
    auto t0 = std::chrono::steady_clock::now();
    p->CommitRangeStepJobsAndWait(0, items, workload_fn(kInnerIters));
    auto t1 = std::chrono::steady_clock::now();
    auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    if (tc == 1) {
      single_ms = wall_ms;
    }
    double speedup =
        (single_ms > 0 && wall_ms > 0) ? static_cast<double>(single_ms) / static_cast<double>(wall_ms) : 0.0;
    std::cout << "[PERF] scaling_threads=" << tc << " wall_ms=" << wall_ms << " speedup=" << speedup << std::endl;
  }
}

}  // namespace
