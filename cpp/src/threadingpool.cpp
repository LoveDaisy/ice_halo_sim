#include "threadingpool.h"

namespace IceHalo {


const int ThreadingPool::kHardwareConcurrency = std::thread::hardware_concurrency();
ThreadingPool* ThreadingPool::instance_ = nullptr;
std::mutex ThreadingPool::instance_mutex_;


ThreadingPool* ThreadingPool::GetInstance() {
  if (instance_ == nullptr) {
    {
      std::unique_lock<std::mutex> lock(instance_mutex_);
      if (instance_ == nullptr) {
#ifdef MULTI_THREAD
        instance_ = new ThreadingPool(kHardwareConcurrency);
#else
        instance_ = new ThreadingPool();  // Default use single thread.
#endif
      }
    }
  }
  return instance_;
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
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_.emplace(job);
  }
  queue_condition_.notify_one();
}


void ThreadingPool::WaitFinish() {
  std::unique_lock<std::mutex> lock(task_mutex_);
  task_condition_.wait(lock, [=] { return !TaskRunning(); });
}


bool ThreadingPool::TaskRunning() {
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
        std::unique_lock<std::mutex> lk(task_mutex_);
        running_jobs_ += 1;
      }
      job();
      {
        std::unique_lock<std::mutex> lk(task_mutex_);
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


}  // namespace IceHalo
