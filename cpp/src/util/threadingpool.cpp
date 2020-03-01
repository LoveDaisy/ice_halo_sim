#include "util/threadingpool.hpp"

#include "util/log.hpp"

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
  LOG_DEBUG("Threading pool size: %zu", thread_num_);
  for (size_t ii = 0; ii < thread_num_; ii++) {
    pool_.emplace_back(&ThreadingPool::WorkingFunction, this, ii);
    alive_threads_ += 1;
  }
}


void ThreadingPool::AddJob(const std::function<void()>& job) {
  if (!alive_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.emplace([=](size_t /* pool_idx */) { job(); });
  }
  queue_condition_.notify_one();
}


void ThreadingPool::AddStepJobs(size_t num, const std::function<void(size_t)>& job) {
  if (!alive_) {
    return;
  }

  auto step = GetPoolSize();
  for (size_t i = 0; i < step; i++) {
    size_t start_idx = i;
    size_t end_idx = num;
    AddJob([=] {
      for (size_t i = start_idx; i < end_idx; i += step) {
        job(i);
      }
    });
  }
}


void ThreadingPool::AddRangeJobs(size_t num, const std::function<void(size_t, size_t)>& job) {
  if (!alive_) {
    return;
  }

  auto step = std::max(num / 97, static_cast<size_t>(10));
  for (size_t i = 0; i < num; i += step) {
    auto current_num = std::min(num - i, step);
    size_t start_idx = i;
    size_t end_idx = i + current_num;
    AddJob([=] { job(start_idx, end_idx); });
  }
}


void ThreadingPool::AddPoolIndexedStepJobs(size_t num, const std::function<void(size_t, size_t)>& job) {
  if (!alive_) {
    return;
  }

  auto step = GetPoolSize();
  for (size_t i = 0; i < step; i++) {
    size_t start_idx = i;
    size_t end_idx = num;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.emplace([=](size_t pool_idx) {
      for (size_t i = start_idx; i < end_idx; i += step) {
        job(i, pool_idx);
      }
    });
    queue_condition_.notify_one();
  }
}


void ThreadingPool::WaitFinish() {
  std::unique_lock<std::mutex> lock(task_mutex_);
  task_condition_.wait(lock, [=] { return !IsTaskRunning(); });
}


bool ThreadingPool::IsTaskRunning() {
  return running_jobs_ > 0 || !queue_.empty();
}


size_t ThreadingPool::GetPoolSize() const {
  return pool_.size();
}


void ThreadingPool::WorkingFunction(size_t pool_idx) {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  while (true) {
    if (!queue_.empty()) {
      auto job = queue_.front();
      queue_.pop();
      lock.unlock();
      {
        std::lock_guard<std::mutex> lk(task_mutex_);
        running_jobs_ += 1;
      }
      job(pool_idx);
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
