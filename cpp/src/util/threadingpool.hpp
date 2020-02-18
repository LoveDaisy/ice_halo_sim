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
  void AddJob(std::function<void()> job);
  void AddRangeBasedJobs(size_t size, const std::function<void(size_t start_idx, size_t end_idx)>& job);
  void WaitFinish();
  bool IsTaskRunning();

  static ThreadingPool* GetInstance();

 private:
  explicit ThreadingPool(size_t num = 1);

  size_t thread_num_;
  std::vector<std::thread> pool_;
  std::atomic<bool> alive_;

  std::queue<std::function<void()>> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;

  std::atomic<int> running_jobs_;
  std::atomic<int> alive_threads_;
  std::mutex task_mutex_;
  std::condition_variable task_condition_;

  void WorkingFunction();

  static const unsigned int kHardwareConcurrency;
};

}  // namespace icehalo


#endif  // SRC_UTIL_THREADINGPOOL_H_
