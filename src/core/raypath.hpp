#ifndef CORE_RAYPATH_H_
#define CORE_RAYPATH_H_

#include <algorithm>
#include <cmath>
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

  // --- Construction-time invariant validation (for assert only) ---
  // N4 invariants checked at RayBuffer::EmplaceBack(RaySeg) entry. Debug only;
  // Release builds compile this to noop via assert(). NOT for business-logic
  // queries — use IsNormal()/IsTir()/etc. for state inspection.
  //
  // N4-1 (to_face_ < poly_count) is intentionally omitted: it requires
  // external crystal context not carried by RaySeg. If needed in the future,
  // expose as a separate IsValidWithCrystal(const Crystal&) overload.
  //
  // Per-field helpers below are column-friendly: an SoA migration can reuse
  // each predicate against a single column without re-deriving the formula.
  static bool IsValidW(float w) {
    // N4-2: weight is non-negative or the TIR sentinel exactly.
    return w >= 0.0f || w == -1.0f;
  }
  static bool IsValidContinueFace(bool is_continue, IdType to_face) {
    // N4-3: is_continue_ implies no outgoing face (continue rays do not hit).
    return !is_continue || to_face == kInvalidId;
  }
  static bool IsValidCrystalIdx(IdType crystal_idx) {
    // N4-4: crystal_idx_ in [0, kMaxCrystalNum) or the init sentinel.
    return crystal_idx == kInvalidId || crystal_idx < static_cast<IdType>(kMaxCrystalNum);
  }
  static bool IsValidVec3Finite(const float v[3]) {
    // N4-5: vector components must be finite (no NaN / Inf).
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }

  bool IsValidComplete() const {
    return IsValidW(w_) &&                                 //
           IsValidContinueFace(is_continue_, to_face_) &&  //
           IsValidCrystalIdx(crystal_idx_) &&              //
           IsValidVec3Finite(d_) && IsValidVec3Finite(p_);
  }
};

}  // namespace lumice

#endif  // CORE_RAYPATH_H_
