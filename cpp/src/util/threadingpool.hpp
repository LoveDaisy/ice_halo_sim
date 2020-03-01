#ifndef SRC_UTIL_THREADINGPOOL_H_
#define SRC_UTIL_THREADINGPOOL_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>


namespace icehalo {

class ThreadingPool {
 public:
  ~ThreadingPool() = default;

  void Start();
  void AddJob(const std::function<void()>& job);
  void AddStepJobs(size_t num, const std::function<void(size_t idx)>& job);
  void AddRangeJobs(size_t num, const std::function<void(size_t start_idx, size_t end_idx)>& job);
  void AddPoolIndexedStepJobs(size_t num, const std::function<void(size_t i, size_t pool_idx)>& job);
  void WaitFinish();
  bool IsTaskRunning();
  size_t GetPoolSize() const;

  static ThreadingPool* GetInstance();

 private:
  explicit ThreadingPool(size_t num = 1);

  size_t thread_num_;
  std::vector<std::thread> pool_;
  std::atomic<bool> alive_;

  std::queue<std::function<void(size_t)>> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;

  std::atomic<int> running_jobs_;
  std::atomic<int> alive_threads_;
  std::mutex task_mutex_;
  std::condition_variable task_condition_;

  void WorkingFunction(size_t pool_idx);

  static const unsigned int kHardwareConcurrency;
};

}  // namespace icehalo


#endif  // SRC_UTIL_THREADINGPOOL_H_
