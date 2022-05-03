#include "config/sim_data.hpp"

#include <climits>
#include <cstddef>
#include <cstring>

#include "core/def.hpp"

namespace icehalo {
namespace v3 {

RayBuffer::RayBuffer() : capacity_(0), size_(0) {}

RayBuffer::RayBuffer(size_t capacity) : capacity_(capacity), size_(0), rays_(new RaySeg[capacity]{}) {}

RaySeg& RayBuffer::operator[](size_t idx) const {
  return rays_[idx];
}

void RayBuffer::Reset(size_t capacity) {
  if (capacity > capacity_) {
    rays_.reset(new RaySeg[capacity]{});
    capacity_ = capacity;
  }
  size_ = 0;
}

bool RayBuffer::Empty() const {
  return size_ == 0;
}

void RayBuffer::EmplaceBack(RaySeg r) {
  if (size_ + 1 < capacity_) {
    rays_[size_++] = std::move(r);
  }
}

void RayBuffer::EmplaceBack(const RayBuffer& buffer, size_t start, size_t len) {
  size_t end = std::min({ start + len, capacity_, buffer.size_ });
  for (size_t i = start; i < end; i++) {
    rays_[size_++] = buffer.rays_[i];
  }
}

RaySeg* RayBuffer::rays() const {
  return rays_.get();
}

RaySeg* RayBuffer::begin() const {
  return rays_.get();
}

RaySeg* RayBuffer::end() const {
  return rays_.get() + size_;
}


SimData::SimData() : curr_wl_(0.0f) {}

SimData::SimData(size_t capacity) : curr_wl_(0.0f), rays_(capacity) {}

SimData::SimData(const SimData& other)
    : curr_wl_(other.curr_wl_), total_intensity_(0), rays_(other.rays_.capacity_), crystals_(other.crystals_) {
  rays_.size_ = other.rays_.size_;
  std::memcpy(rays_.rays_.get(), other.rays_.rays_.get(), sizeof(RaySeg) * other.rays_.capacity_);
}

SimData::SimData(SimData&& other)
    : curr_wl_(other.curr_wl_), total_intensity_(other.total_intensity_), crystals_(std::move(other.crystals_)) {
  rays_.size_ = other.rays_.size_;
  rays_.capacity_ = other.rays_.capacity_;
  rays_.rays_ = std::move(other.rays_.rays_);

  other.rays_.size_ = 0;
  other.rays_.capacity_ = 0;
  other.rays_.rays_ = nullptr;
}

SimData& SimData::operator=(const SimData& other) {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  total_intensity_ = other.total_intensity_;
  rays_.size_ = other.rays_.size_;
  rays_.capacity_ = other.rays_.capacity_;
  rays_.rays_.reset(new RaySeg[rays_.capacity_]);
  std::memcpy(rays_.rays_.get(), other.rays_.rays_.get(), sizeof(RaySeg) * rays_.capacity_);
  crystals_ = other.crystals_;
  return *this;
}

SimData& SimData::operator=(SimData&& other) {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  total_intensity_ = other.total_intensity_;
  rays_.size_ = other.rays_.size_;
  rays_.capacity_ = other.rays_.capacity_;
  rays_.rays_ = std::move(other.rays_.rays_);
  crystals_ = std::move(other.crystals_);

  other.rays_.capacity_ = 0;
  other.rays_.size_ = 0;
  other.rays_.rays_ = nullptr;
  return *this;
}

}  // namespace v3
}  // namespace icehalo