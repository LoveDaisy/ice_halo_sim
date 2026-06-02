#include "config/sim_data.hpp"

#include <cassert>
#include <cstddef>
#include <cstring>

namespace lumice {

// Guard: if SimData gains new fields, sizeof changes and this fires,
// reminding you to update the 4 special member functions below.
static_assert(sizeof(SimData) == 176, "SimData size changed — update copy/move ctors and operators");

RayBuffer::RayBuffer() : capacity_(0), size_(0) {}

RayBuffer::RayBuffer(size_t capacity)
    : capacity_(capacity), size_(0), rays_(std::make_unique<RaySeg[]>(capacity)),
      recorders_(std::make_unique<RaypathRecorder[]>(capacity)) {}

RaySeg& RayBuffer::operator[](size_t idx) const {
  return rays_[idx];
}

RaypathRecorder& RayBuffer::RecorderAt(size_t idx) {
  assert(idx < capacity_);
  return recorders_[idx];
}

const RaypathRecorder& RayBuffer::RecorderAt(size_t idx) const {
  assert(idx < capacity_);
  return recorders_[idx];
}

void RayBuffer::Reset(size_t capacity) {
  if (capacity > capacity_) {
    rays_ = std::make_unique<RaySeg[]>(capacity);
    recorders_ = std::make_unique<RaypathRecorder[]>(capacity);
    capacity_ = capacity;
  }
  size_ = 0;
}

bool RayBuffer::Empty() const {
  return size_ == 0;
}

void RayBuffer::EmplaceBack(RaySeg r, const RaypathRecorder& rec) {
  // N4 construction-time invariant gate. Debug only; noop in Release (NDEBUG).
  assert(r.IsValidComplete());
  if (size_ + 1 < capacity_) {
    rays_[size_] = r;
    recorders_[size_] = rec;
    size_++;
  }
}

void RayBuffer::EmplaceBack(const RayBuffer& buffer, size_t start, size_t len) {
  // Note: batch condition is `size_ < capacity_` (can fill the last slot);
  // single-ray version uses `size_ + 1 < capacity_` (always leaves one empty).
  // The asymmetry is intentional (contract-locked by test_sim_data tests).
  size_t src_end = std::min(start + len, buffer.size_);
  for (size_t i = start; i < src_end && size_ < capacity_; i++) {
    rays_[size_] = buffer.rays_[i];
    recorders_[size_] = buffer.recorders_[i];
    size_++;
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

// Copy/move semantics over the two parallel owning arrays. Copy uses capacity_
// (not size_) as the copy extent, matching the SimData::operator= legacy and
// MakePopulatedSimData test contract that size_ < capacity_ trailing slots are
// preserved. When #247.3 introduces a third parallel owning array (hot/cold
// split), it should sustain the same copy-by-capacity convention so RayBuffer's
// internal arrays remain symmetric.
RayBuffer::RayBuffer(const RayBuffer& other)
    : capacity_(other.capacity_), size_(other.size_),
      rays_(other.capacity_ > 0 ? std::make_unique<RaySeg[]>(other.capacity_) : nullptr),
      recorders_(other.capacity_ > 0 ? std::make_unique<RaypathRecorder[]>(other.capacity_) : nullptr) {
  if (other.capacity_ > 0) {
    std::memcpy(rays_.get(), other.rays_.get(), sizeof(RaySeg) * other.capacity_);
    std::memcpy(recorders_.get(), other.recorders_.get(), sizeof(RaypathRecorder) * other.capacity_);
  }
}

RayBuffer::RayBuffer(RayBuffer&& other) noexcept
    : capacity_(other.capacity_), size_(other.size_), rays_(std::move(other.rays_)),
      recorders_(std::move(other.recorders_)) {
  other.capacity_ = 0;
  other.size_ = 0;
}

RayBuffer& RayBuffer::operator=(const RayBuffer& other) {
  if (&other == this) {
    return *this;
  }
  capacity_ = other.capacity_;
  size_ = other.size_;
  rays_ = other.capacity_ > 0 ? std::make_unique<RaySeg[]>(other.capacity_) : nullptr;
  recorders_ = other.capacity_ > 0 ? std::make_unique<RaypathRecorder[]>(other.capacity_) : nullptr;
  if (other.capacity_ > 0) {
    std::memcpy(rays_.get(), other.rays_.get(), sizeof(RaySeg) * other.capacity_);
    std::memcpy(recorders_.get(), other.recorders_.get(), sizeof(RaypathRecorder) * other.capacity_);
  }
  return *this;
}

RayBuffer& RayBuffer::operator=(RayBuffer&& other) noexcept {
  if (&other == this) {
    return *this;
  }
  capacity_ = other.capacity_;
  size_ = other.size_;
  rays_ = std::move(other.rays_);
  recorders_ = std::move(other.recorders_);
  other.capacity_ = 0;
  other.size_ = 0;
  return *this;
}


SimData::SimData() : curr_wl_(0.0f) {}

SimData::SimData(size_t capacity) : curr_wl_(0.0f), rays_(capacity) {}

SimData::SimData(const SimData& other)
    : curr_wl_(other.curr_wl_), generation_(other.generation_), rays_(other.rays_), crystals_(other.crystals_),
      crystal_axis_dists_(other.crystal_axis_dists_), outgoing_indices_(other.outgoing_indices_),
      outgoing_d_(other.outgoing_d_), outgoing_w_(other.outgoing_w_), root_ray_count_(other.root_ray_count_) {}

SimData::SimData(SimData&& other) noexcept
    : curr_wl_(other.curr_wl_), generation_(other.generation_), rays_(std::move(other.rays_)),
      crystals_(std::move(other.crystals_)), crystal_axis_dists_(std::move(other.crystal_axis_dists_)),
      outgoing_indices_(std::move(other.outgoing_indices_)), outgoing_d_(std::move(other.outgoing_d_)),
      outgoing_w_(std::move(other.outgoing_w_)), root_ray_count_(other.root_ray_count_) {}

SimData& SimData::operator=(const SimData& other) {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  generation_ = other.generation_;
  rays_ = other.rays_;
  crystals_ = other.crystals_;
  crystal_axis_dists_ = other.crystal_axis_dists_;
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
  generation_ = other.generation_;
  rays_ = std::move(other.rays_);
  crystals_ = std::move(other.crystals_);
  crystal_axis_dists_ = std::move(other.crystal_axis_dists_);
  outgoing_indices_ = std::move(other.outgoing_indices_);
  outgoing_d_ = std::move(other.outgoing_d_);
  outgoing_w_ = std::move(other.outgoing_w_);
  root_ray_count_ = other.root_ray_count_;
  return *this;
}

}  // namespace lumice