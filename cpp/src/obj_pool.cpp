#include "obj_pool.h"

#include "optics.h"

namespace icehalo {

template <typename T>
ObjectPool<T>::~ObjectPool() {
  for (auto seg : segments_) {
    delete[] seg;
  }
  segments_.clear();
}


template <typename T>
void ObjectPool<T>::Clear() {
  next_unused_id_ = 0;
  current_chunk_id_ = 0;
}


template <typename T>
ObjectPool<T>* ObjectPool<T>::GetInstance() {
  static auto instance = new ObjectPool<T>();
  return instance;
}


template <typename T>
ObjectPool<T>::ObjectPool() : current_chunk_id_(0), next_unused_id_(0) {
  auto* pool = new T[kChunkSize];
  segments_.emplace_back(pool);
}


template <typename T>
uint32_t ObjectPool<T>::RefreshChunkIndex() {
  auto id = next_unused_id_.fetch_add(1);
  if (id >= kChunkSize) {
    const std::lock_guard<std::mutex> lock(id_mutex_);
    id = next_unused_id_;
    if (id > kChunkSize) {
      auto seg_size = segments_.size();
      if (current_chunk_id_ + 1 >= seg_size) {
        auto* curr_pool = new T[kChunkSize];
        segments_.emplace_back(curr_pool);
        current_chunk_id_ = seg_size;
      } else {
        current_chunk_id_++;
      }
      id = 0;
      next_unused_id_ = 0;
    }
  }
  return id;
}

template class ObjectPool<RaySegment>;
template class ObjectPool<RayInfo>;

}  // namespace icehalo