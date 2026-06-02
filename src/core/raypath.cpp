#include "core/raypath.hpp"

#include <cassert>

namespace lumice {

RaypathRecorder& RaypathRecorder::operator<<(IdType fn) {
  // Inline-only append. Recorders in RayBuffer must go through
  // RayBuffer::RecorderAppend so overflow is routed to the arena.
  assert(size_ < kInlineCap);
  if (size_ < kInlineCap) {
    data_[size_++] = static_cast<uint8_t>(fn & 0xff);
  }
  return *this;
}

IdType RaypathRecorder::operator[](size_t idx) const {
  // Inline-only access (used by EntryExitSpec::Match on the inline `ee`
  // scratch recorder and by overflow-free production recorders). For
  // overflow-capable reads, use RayBuffer::RecorderDataPtr(idx)[idx].
  if (idx >= size_ || idx >= kInlineCap || data_[idx] == 0xff) {
    return kInvalidId;
  }
  return static_cast<IdType>(data_[idx]);
}


// =============== RaypathHashHelper ===============
class RaypathHashHelper {
  // Use SDBM algorithm. See http://www.cse.yorku.ca/~oz/hash.html for detail.
 public:
  RaypathHashHelper& operator<<(IdType c) {
    hash_ = hash_ * kMagic + c;
    return *this;
  }
  size_t GetHash() const { return hash_; }

 private:
  static constexpr unsigned kMagic = 65599u;
  size_t hash_ = 0;
};


// =============== RaypathHash ===============
size_t RaypathHash::operator()(const std::vector<IdType>& rp) {
  RaypathHashHelper h;
  for (auto x : rp) {
    h << x;
  }
  return h.GetHash();
}

size_t RaypathHash::operator()(const RaypathRecorder& rp) {
  // Hash only inline data; overflow recorders are not expected here (canonical_
  // raypaths in filter_spec are always ≤ kInlineCap).
  assert(!rp.HasOverflow());
  RaypathHashHelper h;
  for (auto x : rp) {
    h << x;
  }
  return h.GetHash();
}

}  // namespace lumice
