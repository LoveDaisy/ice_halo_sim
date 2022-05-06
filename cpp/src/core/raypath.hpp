#ifndef CORE_RAYPATH_H_
#define CORE_RAYPATH_H_

#include <algorithm>
#include <iterator>
#include <vector>

#include "core/def.hpp"
#include "core/geo3d.hpp"

namespace icehalo {
namespace v3 {

struct RaypathRecorder {
  RaypathRecorder& operator<<(IdType fn);
  IdType operator[](size_t idx) const;

  void Clear() {
    size_ = 0;
    std::fill(std::begin(recorder_), std::end(recorder_), 0);
  }

  const uint8_t* begin() const { return recorder_ + 0; }
  const uint8_t* end() const { return recorder_ + size_; }

  size_t size_ = 0;
  uint8_t recorder_[kMaxHits]{};
};


struct RaypathHash {
  size_t operator()(const std::vector<IdType>& rp);
  size_t operator()(const RaypathRecorder& rp);
};


struct RaySeg {
  enum State {
    kNormal,
    kOutgoing,
    kContinue,  // continue to next crystal, on multi-scattering situation
    kStopped,   // exceed max_hits, or filtered out
  };

  // NOTE: if state == kNormal, d and p are in crystal-local coordinates; otherwise
  // d and p are in world coordinates.
  float d_[3];
  float p_[3];  // Generally it is for end point, **NOT** for start point.
  float w_;
  int fid_;

  size_t prev_ray_idx_;
  size_t root_ray_idx_;

  IdType crystal_idx_;
  IdType crystal_config_id_;
  Rotation crystal_rot_;

  State state_;
  RaypathRecorder rp_;  // Raypath in **CURRENT** crystal.
};

}  // namespace v3
}  // namespace icehalo

#endif  // CORE_RAYPATH_H_
