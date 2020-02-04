#include "threadingpool.h"

namespace icehalo {


const unsigned int ThreadingPool::kHardwareConcurrency = std::thread::hardware_concurrency();

ThreadingPool* ThreadingPool::GetInstance() {
#ifdef MULTI_THREAD
  static auto* instance = new ThreadingPool(kHardwareConcurrency);
#else
  static auto* instance = new ThreadingPool();  // Default use single thread.
#endif
  return instance;
}


ThreadingPool::ThreadingPool(size_t num) : thread_num_(num), alive_(false), running_jobs_(0), alive_threads_(0) {
  Start();
}


void ThreadingPool::Start() {
  if (alive_ || running_jobs_ > 0 || alive_threads_ > 0) {
    return;
  }

  pool_.clear();
  alive_ = true;
  alive_threads_ = 0;
  printf("Threading pool size: %zu\n", thread_num_);
  for (decltype(thread_num_) ii = 0; ii < thread_num_; ii++) {
    pool_.emplace_back(&ThreadingPool::WorkingFunction, this);
    alive_threads_ += 1;
  }
}


void ThreadingPool::AddJob(std::function<void()> job) {
  if (!alive_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.emplace(std::move(job));
  }
  queue_condition_.notify_one();
}


void ThreadingPool::AddRangeBasedJobs(size_t num, const std::function<void(size_t, size_t)>& job) {
  auto step = std::max(num / 100, static_cast<size_t>(10));
  for (size_t i = 0; i < num; i += step) {
    auto current_num = std::min(num - i, step);
    size_t start_idx = i;
    size_t end_idx = i + current_num;
    AddJob([=] { job(start_idx, end_idx); });
  }
}


void ThreadingPool::WaitFinish() {
  std::unique_lock<std::mutex> lock(task_mutex_);
  task_condition_.wait(lock, [=] { return !IsTaskRunning(); });
}


bool ThreadingPool::IsTaskRunning() {
  return running_jobs_ > 0 || !queue_.empty();
}


void ThreadingPool::WorkingFunction() {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  while (true) {
    if (!queue_.empty()) {
      std::function<void()> job = queue_.front();
      queue_.pop();
      lock.unlock();
      {
        std::lock_guard<std::mutex> lk(task_mutex_);
        running_jobs_ += 1;
      }
      job();
      {
        std::lock_guard<std::mutex> lk(task_mutex_);
        running_jobs_ -= 1;
      }
      lock.lock();
      task_condition_.notify_one();
    } else if (!alive_) {
      alive_threads_ -= 1;
      task_condition_.notify_one();
      queue_condition_.notify_one();
      break;
    } else {
      task_condition_.notify_one();
      queue_condition_.wait(lock, [=] { return !this->queue_.empty() || !this->alive_; });
    }
  }
}


}  // namespace icehalo
