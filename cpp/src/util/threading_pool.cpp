#include "util/threading_pool.hpp"

namespace icehalo {

#ifdef MULTI_THREAD
const size_t ThreadingPool::kDefaultPoolSize = std::thread::hardware_concurrency();
#else
const size_t ThreadingPool::kDefaultPoolSize = 1;
#endif


ThreadingPool::ThreadingPool(size_t size)
    : running_jobs_(0), pool_{}, state_(kStarting), running_workers_(0), stop_flag_(false) {
  StartPool(size);
}


ThreadingPool::~ThreadingPool() {
  Shutdown();
}


ThreadingPoolPtrU ThreadingPool::CreatePool() {
  return CreatePool(kDefaultPoolSize);
}


ThreadingPoolPtrU ThreadingPool::CreatePool(int size) {
  if (size <= 0) {
    return nullptr;
  }
  return ThreadingPoolPtrU{ new ThreadingPool(size) };
}


void ThreadingPool::StartPool(size_t size) {
  int n = static_cast<int>(size);
  state_ = kStarting;
  for (int i = 0; i < n; i++) {
    pool_.emplace_back(&ThreadingPool::WorkingFunction, this, i);
  }
  {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_cv_.wait(lock, [=] { return running_workers_ >= n; });
  }
  state_ = kIdle;
}


void ThreadingPool::WaitFinish() {
  if (stop_flag_ || state_ == kStopping || state_ == kTerminated) {
    return;
  }
  for (auto& f : job_future_list_) {
    f.wait();
  }
  job_future_list_.clear();
}


void ThreadingPool::Shutdown() {
  if (stop_flag_ || state_ == kStopping || state_ == kTerminated) {
    return;
  }

  // 1. Set stop flag
  state_ = kStopping;
  stop_flag_ = true;
  // 2. Clear job_queue_
  {
    std::unique_lock<std::mutex> head_lock(queue_mutex_);
    std::queue<std::function<void(int)>> p;
    job_queue_.swap(p);
  }

  // Notify
  queue_cv_.notify_all();

  {
    // Wait until all running workers finished.
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_cv_.wait(lock, [=] { return running_workers_ <= 0; });
  }
  job_future_list_.clear();

  for (auto& t : pool_) {
    if (t.joinable()) {
      t.join();
    }
  }
  state_ = kTerminated;
}


uint8_t ThreadingPool::GetState() const {
  return state_;
}


size_t ThreadingPool::GetPoolSize() const {
  return pool_.size();
}


bool ThreadingPool::CommitRangeStepJobs(int start, int end, const std::function<void(int, int)>& range_step_func) {
  if (start >= end) {
    return false;
  }
  auto step = static_cast<int>(pool_.size());
  bool res = true;
  for (int i = 0; i < step; i++) {
    res = res && CommitSingleJob([=](int ti) {
            for (int j = start + i; j < end; j += step) {
              range_step_func(ti, j);
            }
          });
  }
  return res;
}


bool ThreadingPool::CommitRangeStepJobsAndWait(int start, int end,
                                               const std::function<void(int, int)>& range_step_func) {
  auto res = CommitRangeStepJobs(start, end, range_step_func);
  WaitFinish();
  return res;
}


bool ThreadingPool::CommitRangeSliceJobs(int start, int end,
                                         const std::function<void(int, int, int)>& range_slice_func) {
  if (start >= end) {
    return false;
  }
  auto num = static_cast<int>(pool_.size());
  bool res = true;
  for (int i = 0; i < num; i++) {
    auto idx1 = (end - start) * i / num;
    auto idx2 = (end - start) * (i + 1) / num;
    res = res && CommitSingleJob([=](int ti) { range_slice_func(ti, idx1, idx2); });
  }
  return res;
}


bool ThreadingPool::CommitRangeSliceJobsAndWait(int start, int end,
                                                const std::function<void(int, int, int)>& range_slice_func) {
  auto res = CommitRangeSliceJobs(start, end, range_slice_func);
  WaitFinish();
  return res;
}


bool ThreadingPool::CommitSingleJob(std::function<void(int)> job) {
  if (!(state_ & kCommittable)) {
    // The pool is not committable, either it is on starting or on stopping.
    return false;
  }

  bool res = false;
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (stop_flag_) {
      state_ = kStopping;
    } else {
      auto task = std::make_shared<std::packaged_task<void(int)>>(std::move(job));
      job_future_list_.emplace_back(task->get_future());
      job_queue_.emplace([=](int idx) { (*task)(idx); });
      res = true;
    }
  }
  queue_cv_.notify_one();
  return res;
}


void ThreadingPool::WorkingFunction(int idx) {
  {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    running_workers_++;
  }
  worker_cv_.notify_one();
  while (true) {
    // Fetch a job and run
    auto pred = [=] { return !job_queue_.empty() || stop_flag_; };
    std::function<void(int)> curr_job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, pred);
      if (!stop_flag_) {
        curr_job = job_queue_.front();
        job_queue_.pop();
      }
    }

    if (stop_flag_) {
      state_ = kStopping;
      break;
    }

    if (curr_job) {
      running_jobs_++;
      state_ = kRunning;
      curr_job(idx);
      running_jobs_--;
    }

    if (job_queue_.empty() && running_jobs_ <= 0) {
      state_ = kIdle;
    }
  }
  state_ = kStopping;
  {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    running_workers_--;
  }
  worker_cv_.notify_one();
}

}  // namespace kve
