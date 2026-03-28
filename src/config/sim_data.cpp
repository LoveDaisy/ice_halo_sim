#include "config/sim_data.hpp"

#include <cstddef>
#include <cstring>

namespace lumice {

// Guard: if SimData gains new fields, sizeof changes and this fires,
// reminding you to update the 4 special member functions below.
static_assert(sizeof(SimData) == 144, "SimData size changed — update copy/move ctors and operators");

RayBuffer::RayBuffer() : capacity_(0), size_(0) {}

RayBuffer::RayBuffer(size_t capacity) : capacity_(capacity), size_(0), rays_(std::make_unique<RaySeg[]>(capacity)) {}

RaySeg& RayBuffer::operator[](size_t idx) const {
  return rays_[idx];
}

void RayBuffer::Reset(size_t capacity) {
  if (capacity > capacity_) {
    rays_ = std::make_unique<RaySeg[]>(capacity);
    capacity_ = capacity;
  }
  size_ = 0;
}

bool RayBuffer::Empty() const {
  return size_ == 0;
}

void RayBuffer::EmplaceBack(RaySeg r) {
  if (size_ + 1 < capacity_) {
    rays_[size_++] = r;
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
    : curr_wl_(other.curr_wl_), total_intensity_(other.total_intensity_), generation_(other.generation_),
      rays_(other.rays_.capacity_), crystals_(other.crystals_), outgoing_indices_(other.outgoing_indices_),
      outgoing_d_(other.outgoing_d_), outgoing_w_(other.outgoing_w_), root_ray_count_(other.root_ray_count_) {
  rays_.size_ = other.rays_.size_;
  std::memcpy(rays_.rays_.get(), other.rays_.rays_.get(), sizeof(RaySeg) * other.rays_.capacity_);
}

SimData::SimData(SimData&& other) noexcept
    : curr_wl_(other.curr_wl_), total_intensity_(other.total_intensity_), generation_(other.generation_),
      crystals_(std::move(other.crystals_)), outgoing_indices_(std::move(other.outgoing_indices_)),
      outgoing_d_(std::move(other.outgoing_d_)), outgoing_w_(std::move(other.outgoing_w_)),
      root_ray_count_(other.root_ray_count_) {
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
  generation_ = other.generation_;
  rays_.size_ = other.rays_.size_;
  rays_.capacity_ = other.rays_.capacity_;
  rays_.rays_ = std::make_unique<RaySeg[]>(rays_.capacity_);
  std::memcpy(rays_.rays_.get(), other.rays_.rays_.get(), sizeof(RaySeg) * rays_.capacity_);
  crystals_ = other.crystals_;
  outgoing_indices_ = other.outgoing_indices_;
  outgoing_d_ = other.outgoing_d_;
  outgoing_w_ = other.outgoing_w_;
  root_ray_count_ = other.root_ray_count_;
  return *this;
}

SimData& SimData::operator=(SimData&& other) noexcept {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  total_intensity_ = other.total_intensity_;
  generation_ = other.generation_;
  rays_.size_ = other.rays_.size_;
  rays_.capacity_ = other.rays_.capacity_;
  rays_.rays_ = std::move(other.rays_.rays_);
  crystals_ = std::move(other.crystals_);
  outgoing_indices_ = std::move(other.outgoing_indices_);
  outgoing_d_ = std::move(other.outgoing_d_);
  outgoing_w_ = std::move(other.outgoing_w_);
  root_ray_count_ = other.root_ray_count_;

  other.rays_.capacity_ = 0;
  other.rays_.size_ = 0;
  other.rays_.rays_ = nullptr;
  return *this;
}

}  // namespace lumice