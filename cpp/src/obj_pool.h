#ifndef SRC_OBJ_POOL_H_
#define SRC_OBJ_POOL_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

namespace icehalo {

template <typename T>
class ObjectPool {
 public:
  ~ObjectPool();

  ObjectPool(const ObjectPool&) = delete;
  void operator=(const ObjectPool&) = delete;

  template <class... Arg>
  T* GetObject(Arg&&... args) {
    auto id = RefreshChunkIndex();
    T* obj = segments_[current_chunk_id_] + id;
    return new (obj) T(std::forward<Arg>(args)...);
  }

  void Clear();

  static ObjectPool<T>* GetInstance();

 private:
  ObjectPool();

  uint32_t RefreshChunkIndex();

  static constexpr uint32_t kChunkSize = 1024 * 1024;

  std::vector<T*> segments_;
  size_t current_chunk_id_;
  std::atomic<uint32_t> next_unused_id_;
  std::mutex id_mutex_;
};

struct RaySegment;
using RaySegmentPool = ObjectPool<RaySegment>;

struct RayInfo;
using RayInfoPool = ObjectPool<RayInfo>;


}  // namespace icehalo


#endif  // SRC_OBJ_POOL_H_
