#ifndef CORE_RAYPATH_H_
#define CORE_RAYPATH_H_

#include <algorithm>
#include <cmath>
#include <cstring>
#include <type_traits>
#include <vector>

#include "core/def.hpp"
#include "core/geo3d.hpp"

namespace lumice {

// Round 2 layout (#247.4): pure POD, trivially copyable. Overflow data lives in
// RayBuffer::overflow_arena_; this struct only stores an arena slot index, so
// fan-out (simulator.cpp hot path) reduces to a trivial 18B memcpy.
//
//   offset 0:  uint8_t  size_                   (1B, ≤ kMaxHits)
//   offset 1:  uint8_t  data_[kInlineCap=15]    (15B inline hits)
//   offset 16: uint16_t overflow_idx_           (2B, kNoOverflow → inline-only)
//   sizeof == 18, align == 2.
//
// Read path: use RayBuffer::RecorderDataPtr(idx) which inspects overflow_idx_
// and returns either data_ or overflow_arena_ + overflow_idx_*kMaxHits.
// Append path: use RayBuffer::RecorderAppend(idx, fn) — operator<< on a bare
// recorder is inline-only (asserts size_ < kInlineCap) and is reserved for
// canonical_ / EntryExitSpec scratch recorders.
struct RaypathRecorder {
  static constexpr uint8_t kInlineCap = 15;
  static constexpr uint16_t kNoOverflow = 0xFFFFu;

  // Inline-only append. PRECONDITION: size_ < kInlineCap. Hot recorders living
  // in RayBuffer::recorders_ MUST go through RayBuffer::RecorderAppend instead
  // so overflow into the arena is handled correctly.
  RaypathRecorder& operator<<(IdType fn);
  IdType operator[](size_t idx) const;

  void Clear() {
    size_ = 0;
    overflow_idx_ = kNoOverflow;
    std::fill(data_, data_ + kInlineCap, static_cast<uint8_t>(0));
  }

  bool HasOverflow() const { return overflow_idx_ != kNoOverflow; }

  // Inline-only iteration (size_ <= kInlineCap). For overflow-capable recorders
  // iterate the arena buffer directly via RayBuffer::RecorderDataPtr.
  const uint8_t* begin() const { return data_; }
  const uint8_t* end() const { return data_ + std::min<size_t>(size_, kInlineCap); }

  // Inline-only equality (size_ <= kInlineCap). Used in unit tests and
  // non-filter comparison paths. Overflow recorders are not supported; assert
  // guards the contract in debug builds, consistent with RaypathHash::operator().
  bool operator==(const RaypathRecorder& other) const {
    assert(!HasOverflow() && !other.HasOverflow());
    if (size_ != other.size_) {
      return false;
    }
    return std::memcmp(data_, other.data_, std::min<size_t>(size_, kInlineCap)) == 0;
  }
  bool operator!=(const RaypathRecorder& other) const { return !(*this == other); }

  uint8_t size_ = 0;
  uint8_t data_[kInlineCap]{};
  uint16_t overflow_idx_ = kNoOverflow;
};

// Round 2 hard guards: regression here means the fan-out hot path lost trivial
// memcpy (#247.4 Round 1 root cause: unique_ptr made the struct non-trivially
// copyable, +31% instructions). Both asserts must hold.
static_assert(std::is_trivially_copyable_v<RaypathRecorder>,
              "RaypathRecorder must stay trivially copyable so fan-out is memcpy "
              "(see #247.4 Round 1 perf regression)");
static_assert(sizeof(RaypathRecorder) == 18,
              "RaypathRecorder layout changed — re-check #247.4 SBO invariants and SimData size");


// Inline-only face-sequence for the exit seam (scrum-258.2). Mirrors the inline
// half of RaypathRecorder but drops overflow_idx_: all real configs have
// max_hits <= kInlineCap (7-8 in practice), so overflow never fires here. The
// device-side kernel populates this directly into shared-memory buffers (no
// device-side dynamic allocation), and the host-side {dir, weight, path,
// crystal_id, ms_layer_idx} record stays trivially copyable + defined-layout.
struct ExitFaceSeq {
  static constexpr uint8_t kCap = 15;
  static_assert(kCap == RaypathRecorder::kInlineCap,
                "ExitFaceSeq::kCap must stay in sync with RaypathRecorder::kInlineCap");
  uint8_t size_ = 0;
  uint8_t data_[kCap]{};
};
static_assert(sizeof(ExitFaceSeq) == 16u, "ExitFaceSeq layout changed — re-check exit-seam wire format");
static_assert(std::is_trivially_copyable_v<ExitFaceSeq>,
              "ExitFaceSeq must stay trivially copyable for memcpy out of device buffers");


struct RaypathHash {
  size_t operator()(const std::vector<IdType>& rp);
  // Precondition: !rp.HasOverflow(). Hash consumers (filter_spec canonical_)
  // never feed overflow recorders; callers must guard externally.
  size_t operator()(const RaypathRecorder& rp);
};


// See doc/raypath-rayseg-architecture.md §1 for field invariants and state-machine semantics.
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
  // NOTE: RaypathRecorder lives in RayBuffer::recorders_ as a parallel array;
  // access via RayBuffer::RecorderAt(idx). Removed from RaySeg to keep the hot
  // BufferWrapper<float>(step=sizeof(RaySeg)) cache density high (see
  // scratchpad/scrum-cpu-soa-refactor for the SoA-prize analysis).

  // Derived segment-kind helpers. Invariants (mutually exclusive):
  //   IsTir()      <=> w_ < 0  (also absorbs filter-terminated outgoing candidates,
  //                             which reuse the w_<0 sentinel — see doc/filter-architecture.md §7)
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

// SoA-prize guard: RaypathRecorder was moved out to RayBuffer::recorders_.
// If RaySeg grows again, the hot BufferWrapper<float>(step=sizeof(RaySeg))
// cache density regresses — re-evaluate before bumping this value.
static_assert(sizeof(RaySeg) == 96, "RaySeg size changed — re-check hot-loop cache density");

}  // namespace lumice

#endif  // CORE_RAYPATH_H_
