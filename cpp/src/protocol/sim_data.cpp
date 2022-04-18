#include "protocol/sim_data.hpp"

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


SimBasicData::SimBasicData() : size_(0), capacity_(0) {}

SimBasicData::SimBasicData(size_t capacity) : size_(0), capacity_(capacity), rays_(new RaySeg[capacity]{}) {}

void SimBasicData::Reset(size_t capacity) {
  rays_.reset(new RaySeg[capacity]{});
  size_ = 0;
  capacity_ = capacity;
}

bool SimBasicData::Empty() const {
  return size_ == 0;
}

}  // namespace v3
}  // namespace icehalo