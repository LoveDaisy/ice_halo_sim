#include "protocol/sim_data.hpp"

#include <cstddef>
#include <cstring>

namespace icehalo {
namespace v3 {

RaypathHashHelper& RaypathHashHelper::operator<<(IdType c) {
  hash_ = hash_ * kMagic + c;
  return *this;
}

size_t RaypathHashHelper::GetHash() const {
  return hash_;
}


size_t RaypathHash::operator()(const std::vector<IdType>& rp) {
  RaypathHashHelper h;
  for (auto x : rp) {
    h << x;
  }
  return h.GetHash();
}


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

SimData::SimData(size_t capacity) : curr_wl_(0.0f), ray_buffer_(capacity) {}

SimData::SimData(const SimData& other)
    : curr_wl_(other.curr_wl_), ray_buffer_(other.ray_buffer_.capacity_), crystals_(other.crystals_) {
  ray_buffer_.size_ = other.ray_buffer_.size_;
  std::memcpy(ray_buffer_.rays_.get(), other.ray_buffer_.rays_.get(), sizeof(RaySeg) * other.ray_buffer_.capacity_);
}

SimData::SimData(SimData&& other) : curr_wl_(other.curr_wl_), crystals_(std::move(other.crystals_)) {
  ray_buffer_.size_ = other.ray_buffer_.size_;
  ray_buffer_.capacity_ = other.ray_buffer_.capacity_;
  ray_buffer_.rays_ = std::move(other.ray_buffer_.rays_);

  other.ray_buffer_.size_ = 0;
  other.ray_buffer_.capacity_ = 0;
  other.ray_buffer_.rays_ = nullptr;
}

SimData& SimData::operator=(const SimData& other) {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  ray_buffer_.size_ = other.ray_buffer_.size_;
  ray_buffer_.capacity_ = other.ray_buffer_.capacity_;
  ray_buffer_.rays_.reset(new RaySeg[ray_buffer_.capacity_]);
  std::memcpy(ray_buffer_.rays_.get(), other.ray_buffer_.rays_.get(), sizeof(RaySeg) * ray_buffer_.capacity_);
  crystals_ = other.crystals_;
  return *this;
}

SimData& SimData::operator=(SimData&& other) {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  ray_buffer_.size_ = other.ray_buffer_.size_;
  ray_buffer_.capacity_ = other.ray_buffer_.capacity_;
  ray_buffer_.rays_ = std::move(other.ray_buffer_.rays_);
  crystals_ = std::move(other.crystals_);

  other.ray_buffer_.capacity_ = 0;
  other.ray_buffer_.size_ = 0;
  other.ray_buffer_.rays_ = nullptr;
  return *this;
}

}  // namespace v3
}  // namespace icehalo