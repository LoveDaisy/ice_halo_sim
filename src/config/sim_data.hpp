#ifndef CONFIG_SIM_DATA_H_
#define CONFIG_SIM_DATA_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/crystal.hpp"
#include "core/def.hpp"
#include "core/exit_seam.hpp"
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

  // ---- Per-ray component mask (raypath-color foundation, task-331.1) ----
  // Parallel array carrying a uint64 per-ray mask across MS layers. Kept out of
  // RaySeg (which has no free padding for an 8B field — see task-331.1 plan
  // §3.1) so `sizeof(RaySeg) == 96` and the hot BufferWrapper cache density
  // survive. Values are OR-accumulated across layers by the phase-2 producer
  // (T2/T3); T1 only wires the transport path and leaves the mask at 0.
  uint64_t ComponentAt(size_t idx) const;
  void SetComponent(size_t idx, uint64_t v);
  // Copy src.components_[src_idx] into components_[dst0] and components_[dst1].
  // Mirror of RecorderFanOut for the hit-loop child pair in TraceRayBasicInfo.
  void ComponentFanOut(const RayBuffer& src, size_t src_idx, size_t dst0, size_t dst1);

  // Swap the full per-ray hot state of slots i and j: the RaySeg AND its
  // parallel component mask, so a ray and its OR-accumulated raypath-color mask
  // stay paired under the continuation-pool decorrelation shuffle
  // (simulator.cpp + cpu_trace_backend.cpp). Before task-331.3 the shuffle used
  // `std::swap(buf[i], buf[j])`, which swaps ONLY rays_ (operator[] returns
  // RaySeg&) and left components_ behind — decorrelating the accumulated mask
  // from its ray across MS layers (the mask is the only cross-layer-surviving
  // per-ray state; recorders are RecorderClear'd at the next layer's entry).
  // recorders_ is DELIBERATELY not swapped here: it is reset at layer entry
  // before being read, so swapping it would only churn the overflow arena for
  // no observable effect — keeping the shuffle's recorder behaviour bit-identical
  // to pre-331.3 (rendering unchanged).
  void SwapRay(size_t i, size_t j);

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
  //
  // The 2-arg overload accepts ONLY inline recorders (asserts !rec.HasOverflow()).
  // Overflow-capable recorders MUST use the 3-arg overload below, which carries
  // the arena source so DupOverflowSlot can clone the slot into *this*. The
  // unguarded 2-arg call on an overflow recorder used to silently copy
  // overflow_idx_ pointing into the foreign arena, then null-deref on the next
  // RecorderFanOut/DupOverflowSlot (root cause of the max_hits>15 crash).
  void EmplaceBack(RaySeg r, const RaypathRecorder& rec);
  void EmplaceBack(RaySeg r, const RaypathRecorder& rec, const RayBuffer& arena_src);
  void EmplaceBack(const RayBuffer& buffer, size_t start = 0, size_t len = kInfSize);

  RaySeg* rays() const;
  RaySeg* begin() const;
  RaySeg* end() const;

  size_t capacity_;
  size_t size_;
  std::unique_ptr<RaySeg[]> rays_;
  std::unique_ptr<RaypathRecorder[]> recorders_;
  // Per-ray component mask (raypath-color foundation, task-331.1). Zero-initialised
  // on allocation; propagated by ComponentFanOut and buffer-batch EmplaceBack.
  std::unique_ptr<uint64_t[]> components_;

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

  // Pre-packed outgoing ray data for cache-friendly access in Consume().
  // Tightly packed by outgoing order: outgoing_d_[k*3..k*3+2] and outgoing_w_[k]
  // are parallel — one entry per outgoing ray. Filled by Simulator. The outgoing
  // ray count is outgoing_w_.size() (the consumer's single source of truth).
  std::vector<float> outgoing_d_;  // direction (3 floats per outgoing ray)
  std::vector<float> outgoing_w_;  // weight (1 float per outgoing ray)
  // scrum-268.8 (DR-3): per-outgoing-ray wavelength (nm). When non-empty the
  // consumer (server/render.cpp) uses per-ray CMF (SpectrumToXyzPerRay) instead
  // of the single curr_wl_ — this is the Metal/per-ray-pool seam. Empty for
  // CPU path so SpectrumToXyz(curr_wl_) stays in force.
  std::vector<float> outgoing_wl_;
  // task-331.1 (raypath-color foundation): per-outgoing-ray component mask
  // (uint64). CPU paths (legacy + CpuTraceBackend) populate this parallel to
  // outgoing_w_; phase-1 leaves the values at 0 because no producer wires bits
  // yet. Metal/CUDA leave it empty for now — see scrum-331 T5/T6 for the
  // device-side delivery path.
  std::vector<uint64_t> outgoing_component_;

  // Rich exit records (scrum-258.2+) parallel to outgoing_d_/w_. Produced by
  // the trace backend via ReadbackExitRays; consumed by 258.3 (filter +
  // symmetry fold). 258.2 only POPULATES these; outgoing_d_/w_ stay as the
  // consumer's projection input until 258.3 unifies the two.
  std::vector<ExitRayRecord> exit_records_;

  // S1 device-fused: XYZ pixel accumulation from Metal kernel (SupportsDeviceXyzAccum path).
  // Non-empty only when the backend accumulates on-device; CPU path leaves these empty.
  // xyz_pixel_data_: W * H * 3 floats (row-major, XYZ channels).
  // xyz_landed_weight_: total weight of in-bounds primary-pixel writes this batch.
  std::vector<float> xyz_pixel_data_;
  float xyz_landed_weight_ = 0.0f;
  // task-358.1 Step 4 (AC3 device-side per-color-class Y-lane accumulation):
  // per-class flattened Y accumulator produced by GPU backends that also fuse
  // rule-lane accumulation on-device. Layout:
  //     lane_pixel_data_[c * W*H + (py*W+px)] = Y for class c at (px, py)
  // Populated by Simulator::DrainDeviceXyz via TraceBackend::ReadbackClassLanes
  // and folded into RenderConsumer::lane_y_ by ConsumeDeviceFused. Empty when
  // the session has no raypath_color config OR the backend does not accumulate
  // on device (CPU path stays on per-ray outgoing_component_).
  // `lane_class_count_` is a redundant witness of `lane_pixel_data_.size() /
  // (W*H)` — carried so the consumer can validate the shape without knowing W/H.
  std::vector<float> lane_pixel_data_;
  size_t lane_class_count_ = 0;

  // --- Consumer-side bookkeeping (NOT physical render payload) ---
  // The fields below are counters the server/consumer use for stats + queue
  // accounting; they do not affect the rendered image. Keep them distinct from
  // the physical-result fields above (rays_/xyz_pixel_data_/exit_records_).
  size_t root_ray_count_ = 0;  // Count of root rays (prev_ray_idx_ == kInfSize)

  // Crystal count for stats reporting. Populated by legacy path from
  // all_crystals.size() and by exit-seam path from
  // TraceBackend::GetLastBatchCrystalCount() (== final-layer setting count).
  // See scrum-cleanup-cuda-ci-misc/task-exit-seam-crystal-count.
  size_t crystal_count_ = 0;

  // scrum-312 (third-clock drain): how many sim_scene_cnt_ units this SimData
  // accounts for on the consumer side. Normally 1 (one SimData per
  // SimulateOneWavelength* call, paired 1:1 with GenerateScene's per-wavelength
  // increment). The CUDA third-clock path drains ONE SimData for a WINDOW of N
  // accumulated (batch × wavelength) calls, so it sets this to N — ConsumeData
  // decrements sim_scene_cnt_ by this credit to keep the increment/decrement
  // invariant balanced (else the counter never reaches 0 and the server hangs /
  // never idles). See simulator.cpp DrainDeviceXyz + server.cpp ConsumeData.
  size_t sim_scene_credit_ = 1;
};

using SimDataPtrS = std::shared_ptr<SimData>;
using SimDataPtrU = std::unique_ptr<SimData>;

}  // namespace lumice

#endif  // CONFIG_SIM_DATA_H_
