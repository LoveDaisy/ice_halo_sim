#ifndef CONFIG_SIM_DATA_H_
#define CONFIG_SIM_DATA_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"

namespace lumice {

struct RayBuffer {
  RayBuffer();
  explicit RayBuffer(size_t capacity);

  // Value semantics over the parallel owning arrays plus the overflow arena.
  // Copy is a deep dup of rays_/recorders_ sized by capacity_ (matches the
  // legacy SimData::operator= convention so trailing slots are preserved) and
  // copies overflow_used_*kMaxHits bytes of the arena. Move transfers
  // ownership and zeroes the source. SimData's four special members delegate
  // to these.
  RayBuffer(const RayBuffer& other);
  RayBuffer(RayBuffer&& other) noexcept;
  RayBuffer& operator=(const RayBuffer& other);
  RayBuffer& operator=(RayBuffer&& other) noexcept;
  ~RayBuffer() = default;

  RaySeg& operator[](size_t idx) const;

  // Parallel-array access to the per-ray RaypathRecorder (moved out of RaySeg
  // to keep the hot float/id BufferWrapper cache density high). Named method
  // rather than a second operator[] overload to avoid `buf[i]` ambiguity
  // between RaySeg and RaypathRecorder arrays.
  const RaypathRecorder& RecorderAt(size_t idx) const;

  // Round 2 (#247.4): recorders are POD with an overflow_idx_ that indexes
  // into overflow_arena_. The buffer-level helpers below own the arena state
  // machine so the recorders themselves stay trivially copyable.

  // Resolve the active hit buffer for recorders_[idx] — either the inline
  // data_ or the arena slot. Length is recorders_[idx].size_.
  uint8_t* RecorderDataPtr(size_t idx);
  const uint8_t* RecorderDataPtr(size_t idx) const;

  // Append one face id to recorders_[idx], routing into the arena once size
  // exceeds kInlineCap. No-op once size hits kMaxHits.
  void RecorderAppend(size_t idx, IdType fn);

  // Clear recorders_[idx] back to inline-empty. Does NOT reclaim the arena
  // slot (arena is bump-allocated; reset happens via Reset()).
  void RecorderClear(size_t idx);

  // Hot fan-out used by simulator.cpp: copy src.recorders_[src_idx] into
  // recorders_[dst0] and recorders_[dst1]. Inline path = two 18B memcpys;
  // overflow path also duplicates the source arena slot into this buffer's
  // arena so dst recorders remain self-contained. Bench (max_hits=8) never
  // hits the overflow branch.
  void RecorderFanOut(const RayBuffer& src, size_t src_idx, size_t dst0, size_t dst1);

  // Read-only pointer to the overflow arena base for downstream consumers
  // (e.g. FilterSpec::Match needs it to resolve recorder data). Returns
  // nullptr when no overflow slot has been allocated yet.
  const uint8_t* OverflowArena() const { return overflow_arena_.get(); }
  // Read-only accessors for arena bookkeeping (tests / diagnostics only).
  uint16_t OverflowUsed() const { return overflow_used_; }
  uint16_t OverflowCap() const { return overflow_cap_; }

  void Reset(size_t capacity);
  bool Empty() const;
  // Single-RaySeg entry point. Asserts RaySeg::IsValidComplete() at entry
  // to gate the N4 construction-time invariants (Debug only; noop in Release).
  // Buffer-to-buffer overload below is internal data movement and skips this
  // gate — its inputs were already validated when first emplaced.
  void EmplaceBack(RaySeg r, const RaypathRecorder& rec);
  void EmplaceBack(const RayBuffer& buffer, size_t start = 0, size_t len = kInfSize);

  RaySeg* rays() const;
  RaySeg* begin() const;
  RaySeg* end() const;

  size_t capacity_;
  size_t size_;
  std::unique_ptr<RaySeg[]> rays_;
  std::unique_ptr<RaypathRecorder[]> recorders_;

 private:
  // Bump-allocated arena holding overflow hit buffers (each slot is kMaxHits
  // bytes). Lazily grown — overflow_arena_ stays nullptr until the first
  // recorder spills past kInlineCap. Reset() rewinds overflow_used_ to 0;
  // copy/move propagate the populated prefix.
  std::unique_ptr<uint8_t[]> overflow_arena_;
  uint16_t overflow_cap_;
  uint16_t overflow_used_;

  // Allocate (or grow) the arena and return the index of a fresh kMaxHits-byte
  // slot. Doubles capacity on growth.
  uint16_t AllocOverflowSlot();

  // Re-routes recorders_[dst_idx].overflow_idx_ to a fresh slot in *this*
  // buffer's arena and copies the 64B payload from src. No-op when the
  // recorder is inline.
  // PRECONDITION: recorders_[dst_idx] must have been trivially memcpy'd from
  // src immediately before this call (i.e. recorders_[dst_idx].overflow_idx_
  // currently indexes into src.overflow_arena_). This invariant is maintained
  // by RecorderFanOut and the batch EmplaceBack overload.
  void DupOverflowSlot(const RayBuffer& src, size_t dst_idx);
};


struct SimData {
  SimData();
  explicit SimData(size_t capacity);
  SimData(const SimData& other);
  SimData(SimData&& other) noexcept;
  ~SimData() = default;

  SimData& operator=(const SimData& other);
  SimData& operator=(SimData&& other) noexcept;

  // ----- Data -----
  float curr_wl_;
  uint64_t generation_ = 0;
  RayBuffer rays_;
  std::vector<Crystal> crystals_;
  std::vector<AxisDistribution> crystal_axis_dists_;  // parallel to crystals_
  std::vector<size_t> outgoing_indices_;              // Indices of kOutgoing rays in rays_ (filled by Simulator)

  // Pre-packed outgoing ray data for cache-friendly access in Consume().
  // Tightly packed by outgoing order: outgoing_d_[k*3..k*3+2] and outgoing_w_[k]
  // correspond to outgoing_indices_[k]. Filled by Simulator alongside outgoing_indices_.
  std::vector<float> outgoing_d_;  // direction (3 floats per outgoing ray)
  std::vector<float> outgoing_w_;  // weight (1 float per outgoing ray)

  size_t root_ray_count_ = 0;  // Count of root rays (prev_ray_idx_ == kInfSize)

  // Backend pre-accumulated XYZ path (TraceBackend seam integration, task 252.3).
  // When is_backend_path_ is true: the simulator ran via a TraceBackend (e.g.
  // MetalTraceBackend) that performed projection + XYZ accumulation on-device.
  // RenderConsumer::Consume detects this and routes through an early-return XYZ
  // ingest branch instead of the raw-ray projection path. Layout of backend_xyz_:
  // width * height * 3 floats, RenderConfig pixel order. The legacy CPU path
  // leaves these fields at their defaults and behavior is unchanged.
  bool is_backend_path_ = false;          // True iff simulator used a TraceBackend
  std::vector<float> backend_xyz_;        // Non-empty when is_backend_path_ is true
  float backend_total_intensity_ = 0.0f;  // Total landed weight (sum of w_ over landed rays)
};

using SimDataPtrS = std::shared_ptr<SimData>;
using SimDataPtrU = std::unique_ptr<SimData>;

}  // namespace lumice

#endif  // CONFIG_SIM_DATA_H_
