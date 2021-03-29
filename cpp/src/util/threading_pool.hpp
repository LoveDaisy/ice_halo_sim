#ifndef UTIL_THREADING_POOL_H_
#define UTIL_THREADING_POOL_H_

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

namespace icehalo {

class ThreadingPool;
using ThreadingPoolPtr = std::shared_ptr<ThreadingPool>;
using ThreadingPoolPtrU = std::unique_ptr<ThreadingPool>;

class ThreadingPool {
 public:
  static const size_t kDefaultPoolSize;

  /**
   * @brief State of threading pool.
   *
   * States of a threading pool are:
   *     These are kCommittable ==>        kIdle -->  kRunning
   *                                        ^ ^--------|  |
   *     ...................................|.............|...........
   *                                        |             v
   *     These are not kCommittable ==>  kStarting    kStopping --> kTerminated
   *
   * `kIdle` and `kRunning` are both `kCommittable`, i.e. `kIdle & kCommittable == true`
   * and `kRunning & kCommittable == true`.
   */
  enum State : uint8_t {
    kCommittable = 1u,
    kStarting = 2u,
    kIdle = 3u,
    kStopping = 4u,
    kRunning = 5u,
    kTerminated = 6u,
  };

  static ThreadingPoolPtrU CreatePool();

  static ThreadingPoolPtrU CreatePool(int size);

  ~ThreadingPool();

  /**
   * @brief Adds a job to the pool.
   * @param job A functor has signature of `void function(int index)`. The argument `index` is the
   *        index of current worker.
   * @return true if the job is successfully committed to the queue; false otherwise.
   */
  bool CommitSingleJob(std::function<void(int)> job);

  /**
   * @brief Waits for all jobs finishing. This function will block until all jobs finished.
   */
  void WaitFinish();

  /**
   * @brief Tells the pool to stop all jobs as soon as possible.
   *
   * Tells the pool to stop all worker threads as soon as possible, and to drop all jobs in the queue
   * that have not started yet. This function is blocked, until all worker threads are stopped.
   */
  void Shutdown();

  uint8_t GetState() const;

  size_t GetPoolSize() const;

  /**
   * @brief Adds range-step jobs to the pool.
   *
   * This function adds range-step jobs to the pool.
   * These jobs will handle data from `start` inclusive, to data `end` exclusive.
   * It is a convenient pack-up of normal for-loop and ThreadingPool::CommitSingleJob().
   * It is equivalent to:
   * ~~~{.cpp}
   * for (int i = 0; i < POOL_SIZE; i++) {
   *   CommitSingleJob([=](int thread_id) {
   *     for (int j = start + i; j < end; j += POOL_SIZE) {
   *       range_step_func(thread_id, j);
   *     }
   *   });
   * }
   * ~~~
   *
   * NOTE: Jobs may NOT finish when this function returns. You need to call
   * ThreadingPool::WaitFinish() to wait all jobs finishing.
   *
   * @warning If the pool is shutdown, this function will not do anything and return false.
   * @param start The start index for data, inclusive.
   * @param end Then end index for data, exclusive.
   * @param range_step_func A functor has signature of `void function(int thread_idx, int i)`.
   *        The arguments are automatically determined by the pool.
   * @return true if all jobs successfully added. false if any job is failed to commit (for example by a
   *         ThreadingPool::Shutdown() call).
   */
  bool CommitRangeStepJobs(int start, int end, const std::function<void(int, int)>& range_step_func);

  /**
   * @brief It is equivalent to call ThreadingPool::CommitRangeStepJobs() and then ThreadingPool::WaitFinish().
   *
   * See ThreadingPool::CommitRangeStepJobs()
   *
   * @warning If the pool is shutdown, this function will not do anything and return false.
   * @param start The start index for data, inclusive.
   * @param end The end index for data, exclusive.
   * @param range_step_func A functor has signature of `void function(int thread_idx, int i)`.
   *        The arguments are automatically determined by the pool.
   * @return true if all jobs finished successfully. false if any job is cancelled (for example by
   *         a ThreadingPool::Shutdown() call).
   */
  bool CommitRangeStepJobsAndWait(int start, int end, const std::function<void(int, int)>& range_step_func);

  /**
   * @brief Adds range-slice jobs to the pool.
   *
   * This function adds range-slice jobs to the pool.
   * These jobs will handle data from `start` inclusive, to data `end` exclusive.
   * It is a convenient pack-up of normal for-loop and ThreadingPool::CommitSingleJob().
   * It is equivalent to:
   * ~~~{.cpp}
   * for (int i = 0; i < POOL_SIZE; i++) {
   *   int idx1 = (end - start) * i / POOL_SIZE;
   *   int idx2 = (end - start) * i / POOL_SIZE;
   *   CommitSingleJob([=](int thread_id) {
   *     range_slice_func(thread_id, idx1, idx2);
   *   });
   * }
   * ~~~
   *
   * NOTE: Jobs may NOT finish when this function returns. You need to call
   * ThreadingPool::WaitFinish() to wait all jobs finishing.
   *
   * @warning If the pool is shutdown, this function will do nothing and return false.
   * @param start The start index for data, inclusive.
   * @param end The end index for data, exclusive.
   * @param range_slice_func A functor has signature of `void function(int thread_idx, int start, int end)`.
   *        The arguments are automatically determined by the pool.
   * @return true if all jobs finished successfully. false if any job is cancelled (for example by
   *         a ThreadingPool::Shutdown() call).
   */
  bool CommitRangeSliceJobs(int start, int end, const std::function<void(int, int, int)>& range_slice_func);

  /**
   * @brief It is equivalent to call ThreadingPool::CommitRangeSliceJobs() and then ThreadingPool::WaitFinish().
   *
   * See ThreadingPool::CommitRangeSliceJobs()
   *
   * @warning If the pool is shutdown, this function will not do anything and return false.
   * @param start The start index for data, inclusive.
   * @param end The end index for data, exclusive.
   * @param range_slice_func A functor has signature of `void function(int thread_idx, int start, int end)`.
   *        The arguments are automatically determined by the pool.
   * @return true if all jobs finished successfully. false if any job is cancelled (for example by a
   *         ThreadingPool::Shutdown() call).
   */
  bool CommitRangeSliceJobsAndWait(int start, int end, const std::function<void(int, int, int)>& range_slice_func);

 private:
  explicit ThreadingPool(size_t size);

  void WorkingFunction(int idx);

  /**
   * @brief Starts the pool with given size.
   * @note Private use.
   * @param size The pool size.
   */
  void StartPool(size_t size);

  std::queue<std::function<void(int)>> job_queue_;
  std::vector<std::future<void>> job_future_list_;
  std::atomic_int running_jobs_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  std::vector<std::thread> pool_;
  std::atomic_uint8_t state_;
  std::atomic_int running_workers_;
  std::atomic_bool stop_flag_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
};

}  // namespace icehalo

#endif  // UTIL_THREADING_POOL_H_
