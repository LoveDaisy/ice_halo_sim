#ifndef CORE_BUFFER_H_
#define CORE_BUFFER_H_

#include <cstddef>
#include <cstdint>

namespace icehalo {
namespace v3 {

template <class T>
struct BufferWrapper {
  BufferWrapper() : data_(nullptr), step_(0) {}
  BufferWrapper(T* data, int step) : data_(data), step_(step) {}
  BufferWrapper(T* data) : data_(data), step_(sizeof(T)) {}

  T* data_;
  int step_;  // bytes to next object

  T& operator[](size_t idx) const { return *reinterpret_cast<T*>(reinterpret_cast<uint8_t*>(data_) + idx * step_); }
  T* Ptr(size_t idx) const { return reinterpret_cast<T*>(reinterpret_cast<uint8_t*>(data_) + idx * step_); }
};

using float_bf_t = BufferWrapper<float>;
using int_bf_t = BufferWrapper<int>;

}  // namespace v3
}  // namespace icehalo

#endif  // CORE_BUFFER_H_
