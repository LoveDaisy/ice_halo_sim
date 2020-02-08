#ifndef SRC_OBJ_POOL_H_
#define SRC_OBJ_POOL_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

namespace icehalo {

constexpr uint32_t kInvalidIndex = 0xffffffff;


template <typename T>
class ObjectPool {
 public:
  ~ObjectPool();

  ObjectPool(const ObjectPool&) = delete;
  void operator=(const ObjectPool&) = delete;

  template <class... Arg>
  T* GetObject(Arg&&... args) {
    auto id = RefreshChunkIndex();
    T* obj = objects_[current_chunk_id_] + id;
    return new (obj) T(std::forward<Arg>(args)...);
  }

  void Clear();

  T* GetSerializedPointer(uint32_t chunk_id, uint32_t obj_id);
  std::tuple<uint32_t, uint32_t> GetObjectSerializeIndex(T* obj);

  void Map(std::function<void(T&)>);

  static ObjectPool<T>* GetInstance();

 private:
  ObjectPool();

  uint32_t RefreshChunkIndex();

  static constexpr size_t kChunkSize = 1024 * 1024;

  std::vector<T*> objects_;
  uint32_t current_chunk_id_;
  std::atomic<uint32_t> next_unused_id_;
  std::mutex id_mutex_;
  size_t deserialized_chunk_size_;
};

struct RaySegment;
using RaySegmentPool = ObjectPool<RaySegment>;

struct RayInfo;
using RayInfoPool = ObjectPool<RayInfo>;


}  // namespace icehalo


#endif  // SRC_OBJ_POOL_H_
