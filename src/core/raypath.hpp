#ifndef CORE_RAYPATH_H_
#define CORE_RAYPATH_H_

#include <algorithm>
#include <iterator>
#include <vector>

#include "core/def.hpp"
#include "core/geo3d.hpp"

namespace lumice {

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
  // NOTE: if IsNormal(), d and p are in crystal-local coordinates; otherwise
  // d and p are in world coordinates.
  float d_[3];
  float p_[3];  // Generally it is for end point, **NOT** for start point.
  float w_;
  // Polygon-face index this segment originated from (i.e. the parent segment's
  // to_face_). Records the face the ray just exited; used for show_rays logging
  // and as a SoA-ready bookkeeping field. The actual source-face guard passed to
  // Propagate is the *current* segment's to_face_ via BufferWrapper — not this
  // field directly. kInvalidId means no source face (first segment of a chain).
  IdType from_face_;
  // Polygon-face index this segment hit / currently rests on. Updated by
  // Propagate at exit; consumed by HitSurface (Fresnel normal lookup) and the
  // raypath recorder. kInvalidId marks "no hit": either an outgoing candidate
  // (no further face intersected) or a TIR-stopped segment.
  IdType to_face_;

  size_t prev_ray_idx_;
  size_t root_ray_idx_;

  IdType crystal_idx_;
  IdType crystal_config_id_;
  Rotation crystal_rot_;

  // 1-bit run-time decision: whether this segment continues into the next
  // crystal (multi-scattering). Other segment kinds (Normal / Outgoing / Tir)
  // are pure derivations of to_face_ and w_; see helpers below.
  bool is_continue_ = false;
  RaypathRecorder rp_;  // Raypath in **CURRENT** crystal.

  // Derived segment-kind helpers. Invariants (mutually exclusive, exhaustive):
  //   IsTir()      <=> w_ < 0
  //   IsNormal()   <=> to_face_ != kInvalidId && w_ >= 0
  //   IsOutgoing() <=> to_face_ == kInvalidId && w_ >= 0 && !is_continue_
  //   IsContinue() <=> is_continue_ (only set when to_face_ == kInvalidId && w_ >= 0)
  bool IsTir() const { return w_ < 0; }
  bool IsNormal() const { return to_face_ != kInvalidId && w_ >= 0; }
  bool IsContinue() const { return is_continue_; }
  bool IsOutgoing() const { return to_face_ == kInvalidId && w_ >= 0 && !is_continue_; }
};

}  // namespace lumice

#endif  // CORE_RAYPATH_H_
