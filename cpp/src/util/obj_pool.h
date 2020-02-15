#ifndef SRC_UTIL_OBJ_POOL_H_
#define SRC_UTIL_OBJ_POOL_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

#include "io/serialize.h"

namespace icehalo {

template <typename T>
class ObjectPool : public ISerializable {
 public:
  ~ObjectPool() override;

  ObjectPool(const ObjectPool&) = delete;
  void operator=(const ObjectPool&) = delete;

  template <class... Arg>
  T* GetObject(Arg&&... args) {
    auto id = RefreshChunkIndex();
    T* obj = objects_[current_chunk_id_] + id;
    return new (obj) T(std::forward<Arg>(args)...);
  }

  void Clear();
  void Map(std::function<void(T&)>);

  T* GetPointerFromSerializeData(T* dummy_ptr);
  T* GetPointerFromSerializeData(uint32_t chunk_id, uint32_t obj_id);
  std::tuple<uint32_t, uint32_t> GetObjectSerializeIndex(T* obj);

  /**
   * @brief Serialize self to a file.
   *
   * This class is a template class. It has only 2 instantiations, RaySegmentPool and RayInfoPool, which
   * all implement interface ISerializable. In fact, this method will call objects'
   * ISerializable::Serialize(File&, bool) to serialize themselves.
   *
   * If the object contains pointers, it is necessary to call ObjectPool<T>::GetPointerFromSerializeData(T*)
   * to get the real pointer. This could be done by calling ObjectPool<T>::Map(std::function<void(T&)>)
   * to apply the action on every element in this pool.
   *
   * The file layout is:
   * uint64,            // the number of object
   * uint64,            // chunk size
   * obj * N,           // the object serialization data
   *
   * @param file
   * @param with_boi
   */
  void Serialize(File& file, bool with_boi) const override;
  void Deserialize(File& file, endian::Endianness endianness) override;

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


#endif  // SRC_UTIL_OBJ_POOL_H_
