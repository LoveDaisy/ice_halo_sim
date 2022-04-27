#include "protocol/sim_data.hpp"

#include <climits>
#include <cstddef>
#include <cstring>

#include "core/def.hpp"

namespace icehalo {
namespace v3 {

constexpr int kRp01Bits = CHAR_BIT - kRpIdBits;
constexpr int kRp12Bits = 2 * CHAR_BIT - 2 * kRpIdBits;
constexpr int kRp23Bits = kRpIdBits;

RaypathRecorder& RaypathRecorder::operator<<(IdType fn) {
  if (size_ >= kMaxRpLen) {
    return *this;
  }

  switch (size_ % 4) {
    case 0:
      recorder_[size_ / 4] |= static_cast<std::byte>(fn & kRpIdMask);
      break;
    case 1:
      recorder_[size_ / 4] |= static_cast<std::byte>((fn & 0x0003) << (CHAR_BIT - kRp01Bits));
      recorder_[size_ / 4 + 1] |= static_cast<std::byte>(fn & 0x003c);
      break;
    case 2:
      recorder_[size_ / 4 + 1] |= static_cast<std::byte>((fn & 0x000f) << (CHAR_BIT - kRp12Bits));
      recorder_[size_ / 4 + 2] |= static_cast<std::byte>(fn & 0x0030);
      break;
    case 3:
      recorder_[size_ / 4 + 2] |= static_cast<std::byte>((fn & kRpIdMask) << (CHAR_BIT - kRp23Bits));
      break;
  }

  size_++;
  return *this;
}

constexpr std::byte kRp00Mask{ kRpIdMask };
constexpr std::byte kRp01Mask{ 0xc0 };
constexpr std::byte kRp11Mask{ 0x0f };
constexpr std::byte kRp12Mask{ 0xf0 };
constexpr std::byte kRp22Mask{ 0x30 };
constexpr std::byte kRp23Mask{ 0xfc };

IdType RaypathRecorder::operator[](size_t idx) {
  if (idx >= size_) {
    return kInvalidId;
  }

  IdType fn = 0;
  switch (idx % 4) {
    case 0:
      fn = static_cast<IdType>(recorder_[idx / 4] & kRp00Mask);
      break;
    case 1:
      fn = static_cast<IdType>((recorder_[idx / 4] & kRp01Mask) >> (CHAR_BIT - kRp01Bits));
      fn |= static_cast<IdType>((recorder_[idx / 4 + 1] & kRp11Mask) << kRp01Bits);
      break;
    case 2:
      fn = static_cast<IdType>((recorder_[idx / 4 + 1] & kRp12Mask) >> (CHAR_BIT - kRp12Bits));
      fn |= static_cast<IdType>((recorder_[idx / 4 + 2] & kRp22Mask) << kRp12Bits);
      break;
    case 3:
      fn = static_cast<IdType>((recorder_[idx / 4 + 2] & kRp23Mask) >> (CHAR_BIT - kRp23Bits));
      break;
  }
  return fn;
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