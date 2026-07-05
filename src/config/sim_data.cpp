#include "config/sim_data.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace lumice {

// Guard: if SimData gains new fields, sizeof changes and this fires,
// reminding you to update the 4 special member functions below.
//
// Round 2 (#247.4) bumped RayBuffer from 32B to 48B (+16B for overflow_arena_
// unique_ptr + cap_/used_ + alignment padding), so SimData grew 176 → 192.
// Task 252.3 added backend_xyz_ (24B) + backend_total_intensity_ (4B) +
// is_backend_path_ (1B + 7B align padding) bumping 192 → 232 for the
// image-seam path. scrum-258.1 Step 4 removes that path entirely (exit
// seam is the canonical out path), shrinking back to 192. scrum-258.2 adds
// exit_records_ (vector<ExitRayRecord>, 24B) for rich exit metadata,
// bumping 192 → 216. scrum-268.8 (DR-3) adds outgoing_wl_ (vector<float>, 24B)
// for per-ray wavelength CMF, bumping 216 → 240. chore-292 (A2) removes the
// vestigial outgoing_indices_ (vector<size_t>, 24B) — its content was never
// read; the consumer's outgoing-ray count is outgoing_w_.size() — shrinking 240 → 216.
// S1 device-fused: adds xyz_pixel_data_ (vector<float>, 24B) + xyz_landed_weight_
// (float, 4B) + 4B padding = 32B total, bumping 216 → 248. task-exit-seam-
// crystal-count adds crystal_count_ (size_t, 8B) for exit-seam stats, bumping
// 248 → 256. scrum-312 adds sim_scene_credit_ (size_t, 8B) for third-clock drain
// counter balance, bumping 256 → 264. task-331.1 (raypath-color foundation)
// adds outgoing_component_ (vector<uint64_t>, 24B) for per-outgoing-ray
// component mask, bumping 264 → 288. task-331.1 also grows RayBuffer by 8B
// (components_ unique_ptr) so the rays_ member's contribution to sizeof(SimData)
// bumps by 8B → total 264 + 24 (outgoing_component_) + 8 (RayBuffer.components_)
// = 296.
static_assert(sizeof(SimData) == 296, "SimData size changed — update copy/move ctors and operators");

namespace {

// Initial slot count for the overflow arena. Bench (max_hits=8) never spills;
// the arena only matters for #231 long-path rays which are rare.
constexpr uint16_t kInitialOverflowCap = 256;

// Sanity: recorders_ array must memcpy-able for the fan-out hot path. Mirror
// the static_assert in raypath.hpp at the buffer boundary too — if anyone
// resurrects a non-trivial member on RaypathRecorder this fires here as well.
static_assert(std::is_trivially_copyable_v<RaypathRecorder>,
              "RayBuffer fan-out memcpy invariant: RaypathRecorder must stay POD");

}  // namespace

RayBuffer::RayBuffer() : capacity_(0), size_(0), overflow_cap_(0), overflow_used_(0) {}

RayBuffer::RayBuffer(size_t capacity)
    : capacity_(capacity), size_(0), rays_(std::make_unique<RaySeg[]>(capacity)),
      recorders_(std::make_unique<RaypathRecorder[]>(capacity)),
      // task-331.1: value-initialised uint64 array (all zeros); mirrors rays_/recorders_
      // allocation gating so capacity==0 leaves components_ as nullptr.
      components_(std::make_unique<uint64_t[]>(capacity)), overflow_cap_(0), overflow_used_(0) {}

RaySeg& RayBuffer::operator[](size_t idx) const {
  return rays_[idx];
}

const RaypathRecorder& RayBuffer::RecorderAt(size_t idx) const {
  assert(idx < capacity_);
  return recorders_[idx];
}

uint8_t* RayBuffer::RecorderDataPtr(size_t idx) {
  assert(idx < capacity_);
  auto& rec = recorders_[idx];
  if (rec.HasOverflow()) {
    return overflow_arena_.get() + static_cast<size_t>(rec.overflow_idx_) * kMaxHits;
  }
  return rec.data_;
}

const uint8_t* RayBuffer::RecorderDataPtr(size_t idx) const {
  assert(idx < capacity_);
  const auto& rec = recorders_[idx];
  if (rec.HasOverflow()) {
    return overflow_arena_.get() + static_cast<size_t>(rec.overflow_idx_) * kMaxHits;
  }
  return rec.data_;
}

uint16_t RayBuffer::AllocOverflowSlot() {
  // Guard against silently overwriting slot 0xFFFE+ since kNoOverflow=0xFFFF
  // would clash with a valid index. In practice overflow is rare and 0xFFFE
  // distinct rays per buffer is far beyond bench / production scenarios.
  assert(overflow_used_ < 0xFFFEu);
  if (overflow_used_ >= overflow_cap_) {
    uint16_t new_cap = overflow_cap_ == 0 ? kInitialOverflowCap : static_cast<uint16_t>(overflow_cap_ * 2);
    if (new_cap <= overflow_cap_) {
      new_cap = 0xFFFEu;  // saturation if doubling overflows uint16_t
    }
    auto new_arena = std::make_unique<uint8_t[]>(static_cast<size_t>(new_cap) * kMaxHits);
    if (overflow_arena_ && overflow_used_ > 0) {
      std::memcpy(new_arena.get(), overflow_arena_.get(), static_cast<size_t>(overflow_used_) * kMaxHits);
    }
    overflow_arena_ = std::move(new_arena);
    overflow_cap_ = new_cap;
  }
  return overflow_used_++;
}

void RayBuffer::RecorderAppend(size_t idx, IdType fn) {
  assert(idx < capacity_);
  auto& rec = recorders_[idx];
  if (rec.size_ >= kMaxHits) {
    return;
  }
  auto byte = static_cast<uint8_t>(fn & 0xff);
  if (rec.size_ < RaypathRecorder::kInlineCap) {
    rec.data_[rec.size_++] = byte;
    return;
  }
  // size_ == kInlineCap: first time we need overflow storage for this recorder.
  // size_ > kInlineCap: arena slot already allocated, just write into it.
  if (!rec.HasOverflow()) {
    uint16_t slot = AllocOverflowSlot();
    uint8_t* slot_ptr = overflow_arena_.get() + static_cast<size_t>(slot) * kMaxHits;
    std::memcpy(slot_ptr, rec.data_, RaypathRecorder::kInlineCap);
    rec.overflow_idx_ = slot;
  }
  uint8_t* slot_ptr = overflow_arena_.get() + static_cast<size_t>(rec.overflow_idx_) * kMaxHits;
  slot_ptr[rec.size_++] = byte;
}

void RayBuffer::RecorderClear(size_t idx) {
  assert(idx < capacity_);
  auto& rec = recorders_[idx];
  rec.size_ = 0;
  rec.overflow_idx_ = RaypathRecorder::kNoOverflow;
}

void RayBuffer::DupOverflowSlot(const RayBuffer& src, size_t dst_idx) {
  auto& rec = recorders_[dst_idx];
  if (!rec.HasOverflow()) {
    return;
  }
  // rec.overflow_idx_ currently still points into src's arena (just memcpy'd in).
  uint16_t src_slot = rec.overflow_idx_;
  uint16_t new_slot = AllocOverflowSlot();
  std::memcpy(overflow_arena_.get() + static_cast<size_t>(new_slot) * kMaxHits,
              src.overflow_arena_.get() + static_cast<size_t>(src_slot) * kMaxHits, kMaxHits);
  rec.overflow_idx_ = new_slot;
}

void RayBuffer::RecorderFanOut(const RayBuffer& src, size_t src_idx, size_t dst0, size_t dst1) {
  assert(&src != this);  // src and dst must be different buffers
  assert(dst0 != dst1);
  assert(src_idx < src.capacity_);
  assert(dst0 < capacity_);
  assert(dst1 < capacity_);
  // Trivial POD memcpy on the hot path (each assignment compiles to ~18B
  // memcpy). The HasOverflow() branch is cold for bench (max_hits=8) but is
  // needed for #231 long-path rays.
  recorders_[dst0] = src.recorders_[src_idx];
  recorders_[dst1] = src.recorders_[src_idx];
  if (recorders_[dst0].HasOverflow()) {
    DupOverflowSlot(src, dst0);
    DupOverflowSlot(src, dst1);
  }
}

// task-331.1: per-ray component mask parallel-array accessors + fan-out.
// Kept adjacent to the RaypathRecorder helpers so future contributors treat
// components_ as the same style of hot-loop parallel column.
uint64_t RayBuffer::ComponentAt(size_t idx) const {
  assert(idx < capacity_);
  return components_[idx];
}

void RayBuffer::SetComponent(size_t idx, uint64_t v) {
  assert(idx < capacity_);
  components_[idx] = v;
}

void RayBuffer::ComponentFanOut(const RayBuffer& src, size_t src_idx, size_t dst0, size_t dst1) {
  assert(&src != this);
  assert(dst0 != dst1);
  assert(src_idx < src.capacity_);
  assert(dst0 < capacity_);
  assert(dst1 < capacity_);
  uint64_t v = src.components_[src_idx];
  components_[dst0] = v;
  components_[dst1] = v;
}

void RayBuffer::SwapRay(size_t i, size_t j) {
  assert(i < capacity_);
  assert(j < capacity_);
  if (i == j) {
    return;
  }
  // Keep the RaySeg and its parallel component mask together — see the header
  // note. recorders_ intentionally excluded (cleared at layer entry).
  std::swap(rays_[i], rays_[j]);
  std::swap(components_[i], components_[j]);
}

void RayBuffer::Reset(size_t capacity) {
  if (capacity > capacity_) {
    rays_ = std::make_unique<RaySeg[]>(capacity);
    recorders_ = std::make_unique<RaypathRecorder[]>(capacity);
    // task-331.1: grow the components_ parallel array in lockstep with rays_.
    // Grow-never-shrink mirrors rays_/recorders_ so pool reuse across MS
    // layers pays no re-allocation cost. Reset does NOT zero existing slots
    // (only newly-allocated capacity is value-initialised); the mask reset for
    // MS-layer entry is done explicitly in InitRayFirstMs — see
    // doc/raypath-rayseg-architecture.md §3 "Reset Points Summary".
    components_ = std::make_unique<uint64_t[]>(capacity);
    capacity_ = capacity;
  }
  size_ = 0;
  // Bump-allocator semantics: hand all arena slots back at once. The buffer
  // itself is retained so we don't pay for re-allocation across MS layers.
  overflow_used_ = 0;
}

bool RayBuffer::Empty() const {
  return size_ == 0;
}

void RayBuffer::EmplaceBack(RaySeg r, const RaypathRecorder& rec) {
  // N4 construction-time invariant gate. Debug only; noop in Release (NDEBUG).
  assert(r.IsValidComplete());
  // Single-rec EmplaceBack callers only feed inline recorders (test fixtures
  // and ray initialisation paths). Overflow recorders would need an arena
  // source to copy from; route through the buffer-to-buffer overload instead.
  assert(!rec.HasOverflow());
  if (size_ + 1 < capacity_) {
    rays_[size_] = r;
    recorders_[size_] = rec;
    size_++;
  }
}

void RayBuffer::EmplaceBack(RaySeg r, const RaypathRecorder& rec, const RayBuffer& arena_src) {
  // N4 invariant gate (Debug only).
  assert(r.IsValidComplete());
  // PRECONDITION: when rec.HasOverflow(), rec.overflow_idx_ must index into
  // arena_src.overflow_arena_. DupOverflowSlot will then clone that slot into
  // *this*'s arena and re-route the dst recorder's overflow_idx_. For inline
  // recorders the arena_src parameter is ignored (DupOverflowSlot no-ops).
  if (size_ + 1 < capacity_) {
    rays_[size_] = r;
    recorders_[size_] = rec;
    DupOverflowSlot(arena_src, size_);
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
    // task-331.1: carry the per-ray component mask alongside the ray+recorder
    // pair so MS-layer batch moves (InitRayOtherMs → EmplaceBack(init_data[0])
    // and the hit-loop all_data.EmplaceBack(buffer_data[1])) don't drop the
    // mask. Single-ray EmplaceBack overloads deliberately do NOT touch
    // components_ — their callers (CollectData) SetComponent explicitly to
    // avoid changing the two-arg / three-arg signatures.
    components_[size_] = buffer.components_[i];
    DupOverflowSlot(buffer, size_);
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

// Copy/move semantics over the parallel owning arrays plus the overflow
// arena. Copy uses capacity_ (not size_) as the recorder/rays copy extent —
// matches the SimData::operator= legacy and MakePopulatedSimData test
// contract that size_ < capacity_ trailing slots are preserved. The arena is
// copied by overflow_used_ * kMaxHits bytes (only the populated prefix);
// recorder overflow_idx_ values stay valid because slot indices are preserved.
RayBuffer::RayBuffer(const RayBuffer& other)
    : capacity_(other.capacity_), size_(other.size_),
      rays_(other.capacity_ > 0 ? std::make_unique<RaySeg[]>(other.capacity_) : nullptr),
      recorders_(other.capacity_ > 0 ? std::make_unique<RaypathRecorder[]>(other.capacity_) : nullptr),
      components_(other.capacity_ > 0 ? std::make_unique<uint64_t[]>(other.capacity_) : nullptr),
      overflow_cap_(other.overflow_cap_), overflow_used_(other.overflow_used_) {
  if (other.capacity_ > 0) {
    std::memcpy(rays_.get(), other.rays_.get(), sizeof(RaySeg) * other.capacity_);
    // Round 2: RaypathRecorder is POD again — trivial memcpy across the
    // full capacity_ array (no per-element deep copy needed).
    std::memcpy(recorders_.get(), other.recorders_.get(), sizeof(RaypathRecorder) * other.capacity_);
    // task-331.1: components_ is a POD uint64 parallel array — memcpy the full
    // capacity_ slice to mirror the rays_/recorders_ deep-copy semantics
    // (SimData::operator= convention: trailing slots preserved).
    std::memcpy(components_.get(), other.components_.get(), sizeof(uint64_t) * other.capacity_);
  }
  if (other.overflow_cap_ > 0) {
    overflow_arena_ = std::make_unique<uint8_t[]>(static_cast<size_t>(other.overflow_cap_) * kMaxHits);
    if (other.overflow_used_ > 0) {
      std::memcpy(overflow_arena_.get(), other.overflow_arena_.get(),
                  static_cast<size_t>(other.overflow_used_) * kMaxHits);
    }
  }
}

RayBuffer::RayBuffer(RayBuffer&& other) noexcept
    : capacity_(other.capacity_), size_(other.size_), rays_(std::move(other.rays_)),
      recorders_(std::move(other.recorders_)), components_(std::move(other.components_)),
      overflow_arena_(std::move(other.overflow_arena_)), overflow_cap_(other.overflow_cap_),
      overflow_used_(other.overflow_used_) {
  other.capacity_ = 0;
  other.size_ = 0;
  other.overflow_cap_ = 0;
  other.overflow_used_ = 0;
}

RayBuffer& RayBuffer::operator=(const RayBuffer& other) {
  if (&other == this) {
    return *this;
  }
  capacity_ = other.capacity_;
  size_ = other.size_;
  rays_ = other.capacity_ > 0 ? std::make_unique<RaySeg[]>(other.capacity_) : nullptr;
  recorders_ = other.capacity_ > 0 ? std::make_unique<RaypathRecorder[]>(other.capacity_) : nullptr;
  components_ = other.capacity_ > 0 ? std::make_unique<uint64_t[]>(other.capacity_) : nullptr;
  if (other.capacity_ > 0) {
    std::memcpy(rays_.get(), other.rays_.get(), sizeof(RaySeg) * other.capacity_);
    std::memcpy(recorders_.get(), other.recorders_.get(), sizeof(RaypathRecorder) * other.capacity_);
    // task-331.1: mirror rays_/recorders_ deep-copy semantics for components_.
    std::memcpy(components_.get(), other.components_.get(), sizeof(uint64_t) * other.capacity_);
  }
  overflow_cap_ = other.overflow_cap_;
  overflow_used_ = other.overflow_used_;
  if (other.overflow_cap_ > 0) {
    overflow_arena_ = std::make_unique<uint8_t[]>(static_cast<size_t>(other.overflow_cap_) * kMaxHits);
    if (other.overflow_used_ > 0) {
      std::memcpy(overflow_arena_.get(), other.overflow_arena_.get(),
                  static_cast<size_t>(other.overflow_used_) * kMaxHits);
    }
  } else {
    overflow_arena_.reset();
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
  // task-331.1: transfer components_ ownership alongside rays_/recorders_.
  // Omitting this on move-assign would leak the mask exactly like the
  // scrum-268.8 outgoing_wl_ move-assign miss — see the note below on SimData.
  components_ = std::move(other.components_);
  overflow_arena_ = std::move(other.overflow_arena_);
  overflow_cap_ = other.overflow_cap_;
  overflow_used_ = other.overflow_used_;
  other.capacity_ = 0;
  other.size_ = 0;
  other.overflow_cap_ = 0;
  other.overflow_used_ = 0;
  return *this;
}


SimData::SimData() : curr_wl_(0.0f) {}

SimData::SimData(size_t capacity) : curr_wl_(0.0f), rays_(capacity) {}

SimData::SimData(const SimData& other)
    : curr_wl_(other.curr_wl_), generation_(other.generation_), rays_(other.rays_), crystals_(other.crystals_),
      crystal_axis_dists_(other.crystal_axis_dists_), outgoing_d_(other.outgoing_d_), outgoing_w_(other.outgoing_w_),
      outgoing_wl_(other.outgoing_wl_), outgoing_component_(other.outgoing_component_),
      exit_records_(other.exit_records_), xyz_pixel_data_(other.xyz_pixel_data_),
      xyz_landed_weight_(other.xyz_landed_weight_), root_ray_count_(other.root_ray_count_),
      crystal_count_(other.crystal_count_), sim_scene_credit_(other.sim_scene_credit_) {}

SimData::SimData(SimData&& other) noexcept
    : curr_wl_(other.curr_wl_), generation_(other.generation_), rays_(std::move(other.rays_)),
      crystals_(std::move(other.crystals_)), crystal_axis_dists_(std::move(other.crystal_axis_dists_)),
      outgoing_d_(std::move(other.outgoing_d_)), outgoing_w_(std::move(other.outgoing_w_)),
      outgoing_wl_(std::move(other.outgoing_wl_)), outgoing_component_(std::move(other.outgoing_component_)),
      exit_records_(std::move(other.exit_records_)), xyz_pixel_data_(std::move(other.xyz_pixel_data_)),
      xyz_landed_weight_(other.xyz_landed_weight_), root_ray_count_(other.root_ray_count_),
      crystal_count_(other.crystal_count_), sim_scene_credit_(other.sim_scene_credit_) {}

SimData& SimData::operator=(const SimData& other) {
  if (&other == this) {
    return *this;
  }

  curr_wl_ = other.curr_wl_;
  generation_ = other.generation_;
  rays_ = other.rays_;
  crystals_ = other.crystals_;
  crystal_axis_dists_ = other.crystal_axis_dists_;
  outgoing_d_ = other.outgoing_d_;
  outgoing_w_ = other.outgoing_w_;
  outgoing_wl_ = other.outgoing_wl_;
  outgoing_component_ = other.outgoing_component_;  // task-331.1
  exit_records_ = other.exit_records_;
  xyz_pixel_data_ = other.xyz_pixel_data_;
  xyz_landed_weight_ = other.xyz_landed_weight_;
  root_ray_count_ = other.root_ray_count_;
  crystal_count_ = other.crystal_count_;
  sim_scene_credit_ = other.sim_scene_credit_;
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
  outgoing_d_ = std::move(other.outgoing_d_);
  outgoing_w_ = std::move(other.outgoing_w_);
  outgoing_wl_ = std::move(other.outgoing_wl_);                // scrum-268.8 (DR-3): was missing
                                                               // here (present in copy/move ctor
                                                               // + copy assign) — the omission
                                                               // silently dropped per-ray wl on
                                                               // the move-assign path used by the
                                                               // consumer queue, collapsing the
                                                               // CMF onto per-batch curr_wl_.
  outgoing_component_ = std::move(other.outgoing_component_);  // task-331.1: same
                                                               // move-assign trap as
                                                               // outgoing_wl_ above.
  exit_records_ = std::move(other.exit_records_);
  xyz_pixel_data_ = std::move(other.xyz_pixel_data_);
  xyz_landed_weight_ = other.xyz_landed_weight_;
  root_ray_count_ = other.root_ray_count_;
  crystal_count_ = other.crystal_count_;
  sim_scene_credit_ = other.sim_scene_credit_;
  return *this;
}

}  // namespace lumice
