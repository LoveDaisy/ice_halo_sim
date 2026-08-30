#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/render_config.hpp"
#include "server/consumer.hpp"
#include "util/logger.hpp"

namespace lumice {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

/**
 * @brief Recycles the fixed-size snapshot buffers a RenderConsumer hands over to each
 *        published ResultFrame.
 * @details Since a frame OWNS the buffer it was built from (see ResultFrame in
 *          server.hpp), a consumer can no longer write its next snapshot into the same
 *          fixed member — it borrows a fresh buffer per snapshot instead. Without
 *          recycling that would mean one W*H*3 allocation per snapshot per consumer;
 *          the pool makes the steady state "hand the just-released buffer straight
 *          back", since the number of simultaneously live frames is small (published +
 *          in-flight + whatever a reader holds).
 *
 *          A buffer is returned by the deleter of the shared_ptr handed out, so it
 *          comes back at the exact moment its last holder drops it — on whichever
 *          thread that happens, hence the mutex. The deleter keeps a shared_ptr to the
 *          pool itself: a frame may outlive the RenderConsumer that produced it (a
 *          CommitConfig rebuild drops consumers while a poller still holds a frame), and
 *          a deleter reaching into a destroyed pool would be exactly the kind of
 *          lifetime defect this task exists to remove.
 *
 *          A pool serves ONE element count (a consumer's resolution is fixed for its
 *          lifetime — ResetWith is gated on NeedsRebuild); a request for a different
 *          count drops the free list rather than growing a size-indexed structure.
 */
template <typename T>
class FrameBufferPool : public std::enable_shared_from_this<FrameBufferPool<T>> {
 public:
  // Hands out `count` value-initialized elements (fresh allocations are zeroed;
  // recycled ones carry the previous frame's bytes and MUST be fully overwritten by
  // the caller, which is what PrepareSnapshot/PostSnapshot do).
  std::shared_ptr<T[]> Acquire(size_t count) {
    std::shared_ptr<T[]> buf;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (element_count_ != count) {
        free_list_.clear();
        element_count_ = count;
      } else if (!free_list_.empty()) {
        buf = std::move(free_list_.back());
        free_list_.pop_back();
      }
    }
    if (!buf) {
      buf = std::shared_ptr<T[]>(std::make_unique<T[]>(count));
    }
    T* raw = buf.get();
    auto self = this->shared_from_this();
    return std::shared_ptr<T[]>(
        raw, [self, count, inner = std::move(buf)](T*) mutable { self->Recycle(std::move(inner), count); });
  }

  // Free-list depth, for the pool's own white-box test.
  size_t FreeCountForTest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return free_list_.size();
  }

 private:
  // Bounds the idle memory a burst of concurrently-held frames can leave behind;
  // beyond it a returned buffer is simply freed.
  static constexpr size_t kMaxFreeBuffers = 8;

  void Recycle(std::shared_ptr<T[]> buf, size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count != element_count_ || free_list_.size() >= kMaxFreeBuffers) {
      return;  // wrong size or pool full — let it die with this shared_ptr
    }
    free_list_.push_back(std::move(buf));
  }

  mutable std::mutex mutex_;
  std::vector<std::shared_ptr<T[]>> free_list_;
  size_t element_count_ = 0;
};

// See doc/accumulator-consumer-architecture.md §3 (state machine), §4 (thread safety), §6 (invariants).
class RenderConsumer : public IConsume {
 public:
  // task-339.3: `class_table` drives per-color-class rule-lane allocation
  // (see doc/gui-custom-spectrum-and-raypath-color.md §4.7). Each class in the
  // table gets one W*H Y-lane; Consume() runs each class's `any`/`all` predicate
  // over every ray's component mask and accumulates Y into the matching lane.
  // Default (empty table) = pre-336 behavior bit-for-bit (no lane state).
  explicit RenderConsumer(RenderConfig config, ColorClassTable class_table = ColorClassTable{});

  void Consume(const SimData& data) override;
  // S1 device-fused (scrum-302): fold a backend-accumulated XYZ pixel buffer
  // into internal_xyz_ via Neumaier compensation. Split out of Consume() to
  // keep that hot projection path within the cognitive-complexity budget.
  void ConsumeDeviceFused(const SimData& data);
  void PrepareSnapshot() override;
  void CountEffectivePixels();
  void PostSnapshot() override;
  Result GetResult() const override;
  RawXyzResult GetRawXyzResult() const;

  // Lifetime share of the buffers the two getters above return VIEWS into. The
  // assembling ResultFrame anchors these next to the views, which is what makes a
  // published frame outlive every later snapshot (see ResultFrame in server.hpp).
  // Both are re-pointed at a fresh pool buffer by PrepareSnapshot / PostSnapshot, so a
  // caller must take the storage from the same snapshot pass as the view it pairs it
  // with — DoSnapshot does exactly that, under the serialization its Phase 1..2 holds.
  std::shared_ptr<const float[]> SnapshotXyzStorage() const { return snapshot_xyz_; }
  std::shared_ptr<const uint8_t[]> SnapshotImageStorage() const { return snapshot_image_buffer_; }
  void Reset() override;
  void ResetWith(const RenderConfig& new_config);
  void LogConsumeProfile() const;  // Dump accumulated profiling stats

  // task-339.3 read-only accessors. Both read snapshot state, mirroring
  // GetRawXyzResult()'s tearing-free contract.
  //
  // ColoredMask(): union of all class member-bits. Zero when no raypath_color
  // is configured; used by DoSnapshot to gate compositor invocation.
  uint64_t ColoredMask() const { return class_table_.referenced_mask_; }
  // Per-color-class lane accessor (task-339.4's compositor consumes this
  // directly, one lane per class in list order = z-order). Returns pointer to
  // a W*H float array of accumulated Y for the class at `class_idx`, or
  // nullptr when the index is out of range.
  const float* GetColorClassLaneY(size_t class_idx) const;
  // task-342.3 (AC4 empty-arc feedback): whether the given class has any
  // non-zero pixel in its snapshot Y-lane. O(W*H) scan; only called from the
  // GUI empty-warning path (once per commit-debounce tick, not per render
  // frame). Reads snapshot state under the same tearing-free contract as
  // GetColorClassLaneY. Returns false for out-of-range indices.
  bool HasColorClassSignal(size_t class_idx) const;
  // Image dimensions (config resolution). Exposed so the compositor can size
  // its output without reaching into the private config.
  int ImageWidth() const { return config_.resolution_[0]; }
  int ImageHeight() const { return config_.resolution_[1]; }

  // task-336.3: the SINGLE mono-image exposure scale, the sole source of truth
  // for both PostSnapshot() and the component compositor (plan §1.1). Returns
  // config_.intensity_factor_ * kNormScale * total_pix / snapshot_intensity_,
  // or 0 when total_pix<=0 or snapshot_intensity_<=0. Reads the frozen snapshot
  // (snapshot_intensity_), so it is tearing-free once PrepareSnapshot() has run.
  float ExposureScale() const;

  // task-347 (Fix B): server-side self-anchored exposure scale for the
  // composite (raypath_color) path. Given the participating-P99 of the current
  // frame's active class union (raw, unexposed Y), returns the scalar that
  // brings that P99 up to the sRGB target_white=135 linear equivalent — so
  // hiding a bright class instantly re-brightens the remaining dim classes,
  // in the same DoSnapshot call, without any GUI auto-EV round-trip.
  //   = config_.intensity_factor_ * target_linear / participating_p99_y
  // (see doc/ev-pipeline-architecture.md §6.6). Note: the snapshot_intensity
  // factor that appears in ComputeEvAuto's numerator does NOT appear here —
  // that factor cancels against the mono shader's `intensity_scale =
  // intensity_factor / snapshot_intensity` post-processing step. Composite
  // applies `s` directly to lane[p], so the effective per-pixel multiplier
  // must be reproduced without the cancelling factor. Returns 0 when
  // participating_p99_y<=0 or snapshot_intensity_<=0 (guard against pre-first
  // snapshot). MIRROR: the target_white constant and sRGB reverse transform
  // below are duplicated from gui_ev_auto.hpp::ComputeEvAuto (server/ and gui/
  // layers cannot share a header without dragging one into the other — same
  // precedent as the ComputeParticipatingP99Y / ComputeP99Y pair; keep in sync).
  float ParticipatingExposureScale(float participating_p99_y) const;

  // White-box handle on the per-pixel render-domain mask (core/lens_proj_build.hpp's
  // BuildVisibleMask), for the tests that pin two things: that it is built once at
  // construction rather than per snapshot, and that PostSnapshot consumes THAT buffer.
  // Non-const on purpose — a test corrupts the stored mask and asserts the next
  // PostSnapshot honours the corruption, which is what separates "reads the cached mask"
  // from "recomputes an identical one"; a call counter could not tell those apart.
  // Follows FrameBufferPool::FreeCountForTest above: an accessor onto state the class
  // holds anyway, not extra per-instance state carried for the tests' benefit.
  std::vector<uint8_t>& VisibleMaskForTest() { return visible_mask_; }

  // Read-only handle on the same per-pixel render-domain mask, for the production consumer
  // that has to reproduce PostSnapshot's background masking outside this class: the composite
  // (raypath-colour) image is baked from the per-class lanes by the compositor, which never
  // goes through PostSnapshot, so it needs THIS buffer to decide which pixels its own
  // background may touch. Sharing the buffer rather than the predicate is what makes the two
  // paths agree pixel-for-pixel; see visible_mask_'s own comment for what it is and why it
  // stays valid for the consumer's whole life.
  const std::vector<uint8_t>& VisibleMask() const { return visible_mask_; }

 private:
  // task-339.3: per-class lane accumulation, split out of Consume() to keep its
  // cognitive complexity bounded. For each ray it evaluates the class predicate
  // (any / all) against the ray's component mask and adds the ray's Y into that
  // class's lane if the predicate matches.
  void AccumulateColorClassLanes(bool per_ray_wl, const float* wl_buf, float curr_wl, const float* w_buf,
                                 const uint64_t* comp_buf, const int* xy_buf, size_t num);

  // Whether this consumer has any color classes to accumulate into. The gate
  // is checked in four places (constructor, Consume() buffer growth, Consume()
  // has_lanes flag, ConsumeDeviceFused warning) so it lives as a helper.
  bool HasColorClasses() const { return !class_table_.classes_.empty(); }

  RenderConfig config_;
  Rotation rot_;  // camera pose rotation
  float short_pix_ = 0;
  // Row-major W*H, 1 where the lens images a visible piece of sky. Built once in the
  // constructor: it is a function of resolution / lens / view / visible / overlap only, and
  // NeedsRebuild already forces a new consumer when any of those change (background is on
  // the appearance-only side of that split), so it stays valid for this consumer's whole
  // life — ResetWith included. Read by PostSnapshot to decide which pixels the background
  // is added to.
  std::vector<uint8_t> visible_mask_;
  float total_intensity_ = 0;
  float snapshot_intensity_ = 0;
  int effective_pix_ = 0;  // Non-zero pixel count from last PrepareSnapshot
  std::unique_ptr<float[]> internal_xyz_;
  std::unique_ptr<float[]> comp_xyz_;  // Neumaier compensation buffer (S1 device-fused)
  // Borrowed fresh from the pools below on every snapshot, then handed to the frame
  // being assembled — NOT rewritten in place, which is what used to tear a reader's
  // data under it. shared_ptr, not unique_ptr, because the frame co-owns them.
  std::shared_ptr<float[]> snapshot_xyz_;
  std::shared_ptr<uint8_t[]> snapshot_image_buffer_;  // produced by PostSnapshot()
  std::shared_ptr<FrameBufferPool<float>> xyz_pool_ = std::make_shared<FrameBufferPool<float>>();
  std::shared_ptr<FrameBufferPool<uint8_t>> image_pool_ = std::make_shared<FrameBufferPool<uint8_t>>();

  // Pre-allocated Consume() buffers (grow-only)
  std::unique_ptr<float[]> d_buf_;
  std::unique_ptr<float[]> w_buf_;
  std::unique_ptr<int[]> xy_buf_;
  std::unique_ptr<float[]> overlap_w_buf_;  // weight copy for overlap dual-write pass
  // scrum-268.8 (DR-3): per-ray wavelength (nm) buffer mirrored from
  // SimData.outgoing_wl_; compacted in lock-step with w_buf_ during pass 1
  // and read by SpectrumToXyzPerRay. Stays empty when the producer uses the
  // legacy per-batch path (curr_wl_). overlap_wl_buf_ mirrors overlap_w_buf_'s
  // role: pre-compaction snapshot consumed by pass 2 overlap projection.
  std::unique_ptr<float[]> wl_buf_;
  std::unique_ptr<float[]> overlap_wl_buf_;
  size_t buf_capacity_ = 0;

  // task-339.3: per-color-class Y-lane state. `class_table_` is the runtime
  // color-class table (list of classes, each with color + combine + member_bits
  // + visible/solo). Lane arrays are compact: index i = class at index i in
  // class_table_.classes_ (also z-order). Only allocated when the table has
  // any classes → zero-config path stays at pre-336 zero heap allocations.
  // Snapshot lanes shadow lane_y_ under the two-phase snapshot protocol
  // (PrepareSnapshot memcpy).
  ColorClassTable class_table_;
  std::vector<std::unique_ptr<float[]>> lane_y_;
  std::vector<std::unique_ptr<float[]>> snapshot_lane_y_;
  size_t lane_pixel_count_ = 0;  // W * H
  // Per-ray component mask side-cars (parallel to w_buf_ / overlap_w_buf_).
  // Grown in Consume() only when the class table is non-empty (Minor #1 from
  // 336.2 review: zero-config = zero extra allocation).
  std::unique_ptr<uint64_t[]> comp_buf_;
  std::unique_ptr<uint64_t[]> overlap_comp_buf_;
  // One-shot warning latches for backends that do not yet populate
  // SimData.outgoing_component_ (Metal/CUDA per §2 out-of-scope for 336.2, but
  // must not silently drop lane data if a config-writer mixes GPU + colors).
  bool logged_missing_component_ = false;

  // Profiling counters (accumulated across Consume calls, for benchmark analysis)
  size_t consume_count_ = 0;
  double consume_proj_us_ = 0;   // memcpy outgoing buffers + lens projection
  double consume_accum_us_ = 0;  // SpectrumToXyz scatter writes

  Logger logger_{ "Render" };
};

}  // namespace lumice

#endif  // CONSUMER_RENDER_H_
