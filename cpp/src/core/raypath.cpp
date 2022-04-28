#include "core/raypath.hpp"

namespace icehalo {
namespace v3 {

RaypathRecorder& RaypathRecorder::operator<<(IdType fn) {
  if (size_ >= kMaxHits) {
    return *this;
  }
  recorder_[size_++] = static_cast<uint8_t>(fn & 0xff);
  return *this;
}

IdType RaypathRecorder::operator[](size_t idx) const {
  if (idx >= size_ || recorder_[idx] == 0xff) {
    return kInvalidId;
  }

  return static_cast<IdType>(recorder_[idx]);
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
  RaypathHashHelper h;
  for (auto x : rp) {
    h << x;
  }
  return h.GetHash();
}

}  // namespace v3
}  // namespace icehalo
