#include "threadingpool.h"

namespace IceHalo {


Pool * Pool::instance_ = nullptr;
std::mutex Pool::instance_mutex_;


Pool * Pool::GetInstance() {
  if (instance_ == nullptr) {
    {
      std::unique_lock<std::mutex> lock(instance_mutex_);
      if (instance_ == nullptr) {
        instance_ = new Pool();
      }
    }
  }
  return instance_;
}


Pool::Pool()
    : thread_num_(std::thread::hardware_concurrency()), alive_(false),
      running_jobs_(0), alive_threads_(0) {
  Start();
}


void Pool::Start() {
  if (alive_ || running_jobs_ > 0 || alive_threads_ > 0) {
    return;
  }

  pool_.clear();
  alive_ = true;
  alive_threads_ = 0;
  printf("Threading pool size: %u\n", thread_num_);
  for (decltype(thread_num_) ii = 0; ii < thread_num_; ii++) {
    pool_.emplace_back(&Pool::WorkingFunction, this);
    alive_threads_ += 1;
  }
}



void Pool::AddJob(std::function<void()> job) {
  if (!alive_) {
    return;
  }

  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_.emplace(job);
  }
  queue_condition_.notify_one();
}


void Pool::WaitFinish() {
  std::unique_lock<std::mutex> lock(task_mutex_);
  task_condition_.wait(lock, [this]{ return !TaskRunning(); });
}


bool Pool::TaskRunning() {
  return running_jobs_ > 0 || !queue_.empty();
}


void Pool::WorkingFunction() {
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
      queue_condition_.wait(lock, [this]{ return !this->queue_.empty() || !this->alive_; });
    }
  }
}


}  // namespace IceHalo
