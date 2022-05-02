#ifndef UTIL_QUEUE_H_
#define UTIL_QUEUE_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

namespace icehalo {
namespace v3 {

template <class T>
class Queue {
 public:
  T Get() {
    std::unique_lock lock(q_mutex_);
    if (shutdown_) {
      return T();
    }

    if (q_.empty()) {
      q_cv_.wait(lock, [=]() { return !q_.empty() || shutdown_; });  // Block
    }
    if (shutdown_) {
      return T();
    }

    T e = std::move(q_.front());
    q_.pop();
    return e;
  }

  template <class... Args>
  void Emplace(Args&&... args) {
    std::unique_lock lock(q_mutex_);
    if (shutdown_) {
      return;
    }
    q_.emplace(std::forward<Args>(args)...);
    q_cv_.notify_one();
  }

  void Shutdown() {
    std::unique_lock<std::mutex> lock(q_mutex_);
    if (shutdown_) {
      return;
    }

    std::queue<T> tmp_q;
    q_.swap(tmp_q);
    shutdown_ = true;
    q_cv_.notify_all();
  }

  void Start() {
    std::unique_lock<std::mutex> lock(q_mutex_);
    shutdown_ = false;
    q_cv_.notify_all();
  }

 private:
  std::queue<T> q_;  // Just simple example
  std::mutex q_mutex_;
  std::condition_variable q_cv_;
  bool shutdown_ = false;
};

}  // namespace v3
}  // namespace icehalo

#endif  // UTIL_QUEUE_H_
