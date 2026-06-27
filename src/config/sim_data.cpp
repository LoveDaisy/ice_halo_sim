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
// (float, 4B) + 4B padding = 32B total, bumping 216 → 248.
static_assert(sizeof(SimData) == 248, "SimData size changed — update copy/move ctors and operators");

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
      recorders_(std::make_unique<RaypathRecorder[]>(capacity)), overflow_cap_(0), overflow_used_(0) {}

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

void RayBuffer::Reset(size_t capacity) {
  if (capacity > capacity_) {
    rays_ = std::make_unique<RaySeg[]>(capacity);
    recorders_ = std::make_unique<RaypathRecorder[]>(capacity);
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
      overflow_cap_(other.overflow_cap_), overflow_used_(other.overflow_used_) {
  if (other.capacity_ > 0) {
    std::memcpy(rays_.get(), other.rays_.get(), sizeof(RaySeg) * other.capacity_);
    // Round 2: RaypathRecorder is POD again — trivial memcpy across the
    // full capacity_ array (no per-element deep copy needed).
    std::memcpy(recorders_.get(), other.recorders_.get(), sizeof(RaypathRecorder) * other.capacity_);
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
      recorders_(std::move(other.recorders_)), overflow_arena_(std::move(other.overflow_arena_)),
      overflow_cap_(other.overflow_cap_), overflow_used_(other.overflow_used_) {
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
  if (other.capacity_ > 0) {
    std::memcpy(rays_.get(), other.rays_.get(), sizeof(RaySeg) * other.capacity_);
    std::memcpy(recorders_.get(), other.recorders_.get(), sizeof(RaypathRecorder) * other.capacity_);
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
      outgoing_wl_(other.outgoing_wl_), exit_records_(other.exit_records_), xyz_pixel_data_(other.xyz_pixel_data_),
      xyz_landed_weight_(other.xyz_landed_weight_), root_ray_count_(other.root_ray_count_) {}

SimData::SimData(SimData&& other) noexcept
    : curr_wl_(other.curr_wl_), generation_(other.generation_), rays_(std::move(other.rays_)),
      crystals_(std::move(other.crystals_)), crystal_axis_dists_(std::move(other.crystal_axis_dists_)),
      outgoing_d_(std::move(other.outgoing_d_)), outgoing_w_(std::move(other.outgoing_w_)),
      outgoing_wl_(std::move(other.outgoing_wl_)), exit_records_(std::move(other.exit_records_)),
      xyz_pixel_data_(std::move(other.xyz_pixel_data_)), xyz_landed_weight_(other.xyz_landed_weight_),
      root_ray_count_(other.root_ray_count_) {}

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
  exit_records_ = other.exit_records_;
  xyz_pixel_data_ = other.xyz_pixel_data_;
  xyz_landed_weight_ = other.xyz_landed_weight_;
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
  outgoing_d_ = std::move(other.outgoing_d_);
  outgoing_w_ = std::move(other.outgoing_w_);
  outgoing_wl_ = std::move(other.outgoing_wl_);  // scrum-268.8 (DR-3): was missing
                                                 // here (present in copy/move ctor
                                                 // + copy assign) — the omission
                                                 // silently dropped per-ray wl on
                                                 // the move-assign path used by the
                                                 // consumer queue, collapsing the
                                                 // CMF onto per-batch curr_wl_.
  exit_records_ = std::move(other.exit_records_);
  xyz_pixel_data_ = std::move(other.xyz_pixel_data_);
  xyz_landed_weight_ = other.xyz_landed_weight_;
  root_ray_count_ = other.root_ray_count_;
  return *this;
}

}  // namespace lumice
