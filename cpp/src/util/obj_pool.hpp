#ifndef SRC_UTIL_OBJ_POOL_H_
#define SRC_UTIL_OBJ_POOL_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

#include "core/def.hpp"
#include "io/serialize.hpp"

namespace icehalo {

template <typename T>
class ObjectPool {
 public:
  ~ObjectPool();

  ObjectPool(const ObjectPool&) = delete;
  void operator=(const ObjectPool&) = delete;

  template <class... Arg>
  T* GetObject(Arg&&... args) {
    T* obj = RefreshChunkIndex(1);
    return new (obj) T(std::forward<Arg>(args)...);
  }

  T* AllocateObjectArray(size_t n) {
    if (n > 0) {
      return RefreshChunkIndex(n);
    } else {
      return nullptr;
    }
  }

  void Clear();
  void Map(std::function<void(T&)>);

  T* GetPointerFromSerializeData(T* dummy_ptr);
  T* GetPointerFromSerializeData(uint32_t chunk_id, uint32_t obj_id);
  std::tuple<uint32_t, uint32_t> GetObjectSerializeIndex(T* obj);

  static ObjectPool<T>* GetInstance();

 protected:
  ObjectPool();

  T* RefreshChunkIndex(uint32_t n);

  static constexpr size_t kChunkSize = 1024 * 1024;
  static constexpr size_t kUnusedIdMask = 0xffffffff;
  static constexpr size_t kChunkIdMask = 0xffffffff00000000;
  static constexpr unsigned kIdOffset = 32;

  std::vector<T*> objects_;
  std::atomic_uint64_t id_;  //!< chunk_id << 32 | next_unused_id
  std::mutex id_mutex_;
  size_t deserialized_chunk_size_;
};


template <typename T>
class SerializableObjectPool : public ObjectPool<T>, public ISerializable {
 public:
  SerializableObjectPool() = default;
  SerializableObjectPool(const SerializableObjectPool& other) = delete;
  ~SerializableObjectPool() override = default;

  SerializableObjectPool& operator=(const SerializableObjectPool& other) = delete;

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

  static SerializableObjectPool<T>* GetInstance();
};


using RaySegmentPool = SerializableObjectPool<RaySegment>;
using RayInfoPool = SerializableObjectPool<RayInfo>;
using IdPool = ObjectPool<ShortIdType>;

}  // namespace icehalo


#endif  // SRC_UTIL_OBJ_POOL_H_
