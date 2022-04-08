#ifndef SRC_UTIL_QUEUE_HPP_
#define SRC_UTIL_QUEUE_HPP_

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
    if (q_.empty()) {
      q_cv_.wait(lock, [=]() { return !q_.empty(); });  // Block
    }
    T e = std::move(q_.front());
    q_.pop();
    return e;
  }

  template <class... Args>
  void Emplace(Args&&... args) {
    std::unique_lock lock(q_mutex_);
    q_.emplace(std::forward<Args>(args)...);
  }

 private:
  std::queue<T> q_;  // Just simple example
  std::mutex q_mutex_;
  std::condition_variable q_cv_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // SRC_UTIL_QUEUE_HPP_