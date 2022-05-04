#include "util/obj_pool.hpp"

#include "core/optics.hpp"
#include "include/log.hpp"

namespace icehalo {

constexpr uint32_t kInvalidIndex = 0xffffffff;


template <typename T>
ObjectPool<T>::~ObjectPool() {
  for (auto seg : objects_) {
    delete[] seg;
  }
  objects_.clear();
}


template <typename T>
void ObjectPool<T>::Clear() {
  id_ = 0;
  deserialized_chunk_size_ = 0;
}


template <typename T>
T* ObjectPool<T>::GetPointerFromSerializeData(T* dummy_ptr) {
  auto combined_id = reinterpret_cast<uintptr_t>(dummy_ptr);
  uint32_t chunk_id = (combined_id & 0xffffffff00000000) >> 32;
  uint32_t obj_id = (combined_id & 0x00000000ffffffff);

  if (deserialized_chunk_size_ == 0) {
    return nullptr;
  }
  if (chunk_id == kInvalidIndex || obj_id == kInvalidIndex) {
    return nullptr;
  }

  size_t id = chunk_id * deserialized_chunk_size_ + obj_id;
  uint32_t this_chunk_id = id / kChunkSize;
  uint32_t this_obj_id = id % kChunkSize;
  if (this_chunk_id >= objects_.size()) {
    return nullptr;
  }

  return objects_[this_chunk_id] + this_obj_id;
}


template <typename T>
T* ObjectPool<T>::GetPointerFromSerializeData(uint32_t chunk_id, uint32_t obj_id) {
  return GetPointerFromSerializeData(reinterpret_cast<T*>(CombineU32AsPointer(chunk_id, obj_id)));
}


template <typename T>
std::tuple<uint32_t, uint32_t> ObjectPool<T>::GetObjectSerializeIndex(T* obj) {
  if (!obj) {
    return { kInvalidIndex, kInvalidIndex };
  }

  uint32_t chunk_id = 0;
  T* last_chunk = nullptr;
  for (const auto& chunk : objects_) {
    if (last_chunk && obj < chunk) {
      return { chunk_id - 1, static_cast<uint32_t>(obj - last_chunk) };
    }
    chunk_id++;
    last_chunk = chunk;
  }
  if (last_chunk && obj < objects_.back() + kChunkSize) {
    return { chunk_id - 1, static_cast<uint32_t>(obj - last_chunk) };
  } else {
    return { kInvalidIndex, kInvalidIndex };
  }
}


template <typename T>
void ObjectPool<T>::Map(std::function<void(T&)> f) {
  const std::lock_guard<std::mutex> lock(id_mutex_);
  for (const auto& chunk : objects_) {
    size_t chunk_size = (chunk == objects_.back() ? (id_.load() & kUnusedIdMask) : kChunkSize);
    for (size_t j = 0; j < chunk_size; j++) {
      f(chunk[j]);
    }
  }
}


template <typename T>
ObjectPool<T>* ObjectPool<T>::GetInstance() {
  static auto instance = new ObjectPool<T>();
  return instance;
}


template <typename T>
ObjectPool<T>::ObjectPool() : id_(0), deserialized_chunk_size_(0) {
  auto* pool = new T[kChunkSize];
  objects_.emplace_back(pool);
}


template <typename T>
T* ObjectPool<T>::RefreshChunkIndex(uint32_t n) {
  auto curr_id = id_.fetch_add(n);
  auto id = curr_id & kUnusedIdMask;
  auto c_id = (curr_id & kChunkIdMask) >> kIdOffset;
  if (id + n > kChunkSize) {
    const std::lock_guard<std::mutex> lock(id_mutex_);
    curr_id = id_.fetch_add(n);
    id = curr_id & kUnusedIdMask;
    c_id = (curr_id & kChunkIdMask) >> kIdOffset;
    if (id > kChunkSize) {
      auto seg_size = objects_.size();
      if (c_id + 1 >= seg_size) {
        auto* curr_pool = new T[kChunkSize];
        objects_.emplace_back(curr_pool);
        c_id = seg_size;
      } else {
        c_id++;
      }
      id = 0;
      id_ = (c_id << kIdOffset) | (id + n);
    }
  }
  return objects_[c_id] + id;
}


template <typename T>
void SerializableObjectPool<T>::Serialize(File& file, bool with_boi) const {
  if (with_boi) {
    file.Write(ISerializable::kDefaultBoi);
  }

  auto next_unused_id = (ObjectPool<T>::id_ & ObjectPool<T>::kUnusedIdMask);
  size_t total_num = ObjectPool<T>::kChunkSize * (ObjectPool<T>::objects_.size() - 1) + next_unused_id;
  file.Write(total_num);
  file.Write(ObjectPool<T>::kChunkSize);

  for (const auto& chunk : ObjectPool<T>::objects_) {
    size_t num = (chunk == ObjectPool<T>::objects_.back() ? next_unused_id : ObjectPool<T>::kChunkSize);
    for (size_t i = 0; i < num; i++) {
      chunk[i].Serialize(file, false);
    }
  }
}


template <typename T>
void SerializableObjectPool<T>::Deserialize(File& file, endian::Endianness endianness) {
  const std::lock_guard<std::mutex> lock(ObjectPool<T>::id_mutex_);

  endianness = CheckEndianness(file, endianness);
  bool need_swap = (endianness != endian::kCompileEndian);

  size_t total_num = 0;
  file.Read(&total_num);
  if (need_swap) {
    endian::ByteSwap::Swap(&total_num);
  }

  size_t chunk_size = 0;
  file.Read(&chunk_size);
  if (need_swap) {
    endian::ByteSwap::Swap(&chunk_size);
  }
  if (chunk_size == 0) {
    throw std::invalid_argument("Chunk size is invalid!");
  }

  ObjectPool<T>::Clear();
  ObjectPool<T>::deserialized_chunk_size_ = chunk_size;
  size_t chunks = total_num / ObjectPool<T>::kChunkSize + (total_num % ObjectPool<T>::kChunkSize ? 1 : 0);
  for (size_t i = 0; i < chunks; i++) {
    if (i >= ObjectPool<T>::objects_.size()) {
      ObjectPool<T>::objects_.emplace_back(new T[ObjectPool<T>::kChunkSize]);
    }
    auto* chunk = ObjectPool<T>::objects_[i];
    size_t curr_num = ObjectPool<T>::kChunkSize;
    if (i + 1 == chunks) {
      curr_num = total_num % ObjectPool<T>::kChunkSize;
    }
    for (size_t j = 0; j < curr_num; j++) {
      chunk[j].Deserialize(file, endianness);
    }
    ObjectPool<T>::id_ = (chunks << ObjectPool<T>::kIdOffset) | curr_num;
  }
}


template <typename T>
SerializableObjectPool<T>* SerializableObjectPool<T>::GetInstance() {
  static auto instance = new SerializableObjectPool<T>();
  return instance;
}


template class SerializableObjectPool<RaySegment>;
template class SerializableObjectPool<RayInfo>;
template class ObjectPool<RaySegment>;
template class ObjectPool<RayInfo>;
template class ObjectPool<ShortIdType>;

}  // namespace icehalo