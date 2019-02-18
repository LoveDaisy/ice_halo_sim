#ifndef SRC_THREADINGPOOL_H_
#define SRC_THREADINGPOOL_H_

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>


namespace IceHalo {

class Pool {
public:
  ~Pool() = default;

  void Start();
  void AddJob(std::function<void()> job);
  void WaitFinish();
  bool TaskRunning();

  static Pool* GetInstance();

private:
  Pool();

  size_t thread_num_;
  std::vector<std::thread> pool_;
  std::atomic<bool> alive_;

  std::queue<std::function<void()> > queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;

  std::atomic<int> running_jobs_;
  std::atomic<int> alive_threads_;
  std::mutex task_mutex_;
  std::condition_variable task_condition_;

  void WorkingFunction();

  static Pool* instance_;
  static std::mutex instance_mutex_;
};


}   // namespace IceHalo


#endif  // SRC_THREADINGPOOL_H_
