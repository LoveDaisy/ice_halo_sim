#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/annotation_overlay.hpp"  // annotation::CanvasPoint (zenith_point_ / nadir_point_)
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
  // `sun` is the scene's light source, needed for one reason only: the angular-distance
  // annotations in config.angular_dist_grid_ are circles AROUND THE SUN, so their geometry cannot
  // be derived from the RenderConfig alone. It is the first piece of non-render config this class
  // has ever taken, and it stays a separate parameter rather than moving into RenderConfig
  // because the sun belongs to the scene, not to any one renderer — three renderers in one scene
  // share it.
  // `sun` comes LAST, after the older `class_table`, so that every existing two-argument
  // construction keeps compiling — it is the sun a renderer with no angular_dist_grid_ entries
  // never consults.
  explicit RenderConsumer(RenderConfig config, ColorClassTable class_table = ColorClassTable{},
                          SunParam sun = SunParam{ 0.0f, 0.0f, 0.5f });

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
  // The reuse path. `new_sun` is passed for the same reason the constructor takes one, and it is
  // NOT merely stored: neither the sun nor angular_dist_grid_ takes part in NeedsRebuild (the sun
  // is not even a RenderConfig field), so this is the ONLY path by which either can change without
  // a fresh consumer, and the annotation masks are rebuilt here when they do.
  void ResetWith(const RenderConfig& new_config, const SunParam& new_sun);
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
  // for both PostSnapshot() and the component compositor (plan §1.1). Reads the
  // frozen snapshot, so it is tearing-free once PrepareSnapshot() has run, and
  // returns 0 whenever the resolution is degenerate (total_pix<=0).
  //
  // WHICH scale it is, is `config_.ev_mode_`'s decision — the two modes anchor to
  // different things and are not two tunings of one formula:
  //
  //   kAbsolute: intensity_factor_ * kNormScale * total_pix / snapshot_emitted_energy_
  //     (0 if snapshot_emitted_energy_<=0). The denominator is the energy EMITTED, not the
  //     energy that landed. That is what makes this scale absolute: it is fixed by the light
  //     source and the ray budget alone, so it does not move when a filter, a low scene pass
  //     rate, or lens clipping removes rays. Under the old landed-weight denominator a filtered
  //     scene was silently re-brightened by exactly the factor it had been dimmed by, which made
  //     two scenes at the same EV incomparable.
  //
  //   kRelative (the default): intensity_factor_ * TargetWhiteToLinear(kAnchorTargetWhite)
  //     / ComputeP99Y(snapshot, kMonoAnchorDownsampleFactor)   (0 if that P99 is 0).
  //     This is the frame anchoring to ITSELF — algebraically the same per-pixel multiplier the
  //     GUI's ComputeEvAuto + shader pair has always applied, so a CLI render matches what the
  //     GUI shows. Being self-anchored, it carries no energy term: the image keeps its look as
  //     ray_num grows, and correspondingly the config alone does NOT determine output
  //     brightness. See the derivation at the definition in render.cpp.
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
  // snapshot). The sRGB reverse transform this shares with ComputeEvAuto lives
  // in core/ev_anchor.hpp::TargetWhiteToLinear — shared at that level and at
  // the P99 level, but deliberately NOT at the level of the final expression,
  // for the cancellation reason spelled out above.
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

  // White-box handle on the horizon-annotation mask, for the tests that pin its shape against
  // the projection it is derived from. Same rationale as VisibleMaskForTest above.
  const std::vector<uint8_t>& HorizonMaskForTest() const { return horizon_mask_; }
  const std::vector<std::vector<uint8_t>>& AngularDistMasksForTest() const { return angular_dist_masks_; }
  const std::vector<std::vector<uint8_t>>& ElevationMasksForTest() const { return elevation_masks_; }
  const std::vector<std::vector<uint8_t>>& LongitudeMasksForTest() const { return longitude_masks_; }
  // The marker positions, for the tests that pin WHERE a ring lands and WHETHER one is drawn at
  // all. Same rationale as the mask handles above: the alternative is inferring the point back out
  // of the image, which is the thing under test.
  const annotation::CanvasPoint& ZenithPointForTest() const { return zenith_point_; }
  const annotation::CanvasPoint& NadirPointForTest() const { return nadir_point_; }
  // The cached label anchors, per family. Same rationale as the mask handles above, plus one of
  // its own: these say whether the label GEOMETRY was computed, which is the half of the *_label_
  // contract that is independent of the family's line switch. That half cannot be read off the
  // image — a label whose family is fully transparent is computed and then composites to nothing,
  // which is exactly the case worth being able to tell apart from "never computed".
  const std::vector<annotation::Label>& HorizonLabelsForTest() const { return horizon_labels_; }
  const std::vector<annotation::Label>& ElevationLabelsForTest() const { return elevation_labels_; }
  const std::vector<annotation::Label>& LongitudeLabelsForTest() const { return longitude_labels_; }
  const std::vector<annotation::Label>& AngularDistLabelsForTest() const { return angular_dist_labels_; }

  // The composite path's anchor, chosen by `config_.ev_mode_`. This exists so the compositor has
  // ONE call to make and the mode decision has ONE owner — the compositor keeps its single-scalar
  // structure (doc §4.3) and never learns that two anchors exist.
  //   kRelative: ParticipatingExposureScale(participating_p99_y) — unchanged, the participating
  //              pixels anchor themselves, so hiding a bright class re-brightens the rest.
  //   kAbsolute: ExposureScale() — the SAME scalar the mono path uses, argument ignored.
  //              Sharing the mono scale is the point rather than an economy: composite lanes are
  //              copies of the same accumulated Y that feeds mono, so any separately-derived
  //              "absolute composite formula" would differ by some coefficient and break exactly
  //              the property absolute mode exists for — that two renders at one EV are
  //              comparable, mono against composite included.
  float CompositeAnchorScale(float participating_p99_y) const;

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

  // (Re)build angular_dist_masks_ from config_.angular_dist_grid_ and sun_, skipping the work when
  // neither has changed since the last build. See the member's declaration for why this is not a
  // constructor-only job.
  void RebuildAngularDistMasks();

  // The same job for the two sun-INDEPENDENT line families: parallels (config_.elevation_grid_)
  // and meridians (config_.longitude_grid_). One helper serves both because the two differ in
  // exactly two expressions — which Request list the angle goes into and which Overlay mask comes
  // back — while the change detection, the ViewSnapshot fill and the one-call-per-line rule are
  // identical. `family` selects those two expressions.
  enum class LineFamily { kElevation, kLongitude };
  void RebuildLineFamilyMasks(LineFamily family);
  // Both families at once, for the two call sites (constructor, ResetWith) that always want both.
  void RebuildGridMasks();

  // The horizon's mask AND its label anchors, out of ONE annotation::ComputeOverlay call — the
  // same shape the three families above have. It used to be labels only, with the line coming from
  // a second, independent path (BuildHorizonMask, since deleted): two computations of one curve,
  // which is what this task removed. Unlike the three families it takes no angle list, so there is
  // one call rather than one per line.
  void RebuildHorizonAnnotation();

  // Draw the cached label anchors' text into snapshot_image_buffer_. Called at the END of
  // PostSnapshot, after the fused per-pixel loop has written its final sRGB bytes — deliberately
  // NOT inside that loop. The loop is a per-pixel, register-only, byte-exact chain
  // (test_render_consumer_post_snapshot_fusion.cpp pins it); glyph coverage is sparse and
  // two-dimensional, and threading it through there would buy a full-canvas coverage buffer and
  // put a second concern inside the one function that must stay a straight element-wise map.
  //
  // Blends in LINEAR RGB, like every other annotation layer: the bytes it reads are decoded back
  // out of sRGB, mixed, and re-encoded. Only the pixels a glyph actually covers are touched.
  void PaintLabels();


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
  // Row-major W*H, 1 where the celestial horizon (altitude = 0) annotation is drawn. Same
  // lifetime argument as visible_mask_ above, and built the same way — unconditionally, NOT
  // only when config_.horizon_ is set. The flag is on the appearance-only side of
  // NeedsRebuild, so a config that turns it on mid-run reaches this consumer through
  // ResetWith() with no rebuild; a mask built only for the flag's value at construction would
  // then be empty exactly when the user has just asked for the line. Gating happens at the
  // point of use in PostSnapshot instead.
  std::vector<uint8_t> horizon_mask_;
  // Whether horizon_mask_ has been filled at all. Not a change detector like the *_masks_built_
  // flags below: nothing this mask depends on can move under a reused consumer (the view is a
  // NeedsRebuild field), so this only stops ResetWith from redoing the sweep.
  bool horizon_mask_built_ = false;
  // One W*H mask per entry of config_.angular_dist_grid_, index-aligned with it. Per LINE, not
  // per category, because each entry carries its own opacity_ / color_ and a category-wide union
  // could not tell one line's pixels from another's. Building them is a W*H inverse-projection
  // sweep each, which is why they are built once here and not per snapshot.
  //
  // Unlike visible_mask_ / horizon_mask_ these CANNOT be built once and left alone: their shape
  // depends on the requested angles and on the sun, and neither is a NeedsRebuild field, so both
  // can change under a reused consumer. RebuildAngularDistMasks() is therefore called from the
  // constructor AND from ResetWith, and skips the sweep when the inputs it last built from are
  // unchanged.
  std::vector<std::vector<uint8_t>> angular_dist_masks_;
  // What angular_dist_masks_ was last built from — the change detector for the paragraph above.
  std::vector<float> angular_dist_mask_angles_;
  float angular_dist_mask_sun_[3]{ 0.0f, 0.0f, 0.0f };
  bool angular_dist_masks_built_ = false;
  // Parallels and meridians, index-aligned with config_.elevation_grid_ / config_.longitude_grid_.
  // Same per-LINE rationale and same rebuild lifetime as angular_dist_masks_ above, with one
  // difference: these geometries are fixed in the celestial frame, so the sun is NOT an input and
  // the change detector compares the angle list alone.
  std::vector<std::vector<uint8_t>> elevation_masks_;
  std::vector<float> elevation_mask_angles_;
  bool elevation_masks_built_ = false;
  std::vector<std::vector<uint8_t>> longitude_masks_;
  std::vector<float> longitude_mask_angles_;
  bool longitude_masks_built_ = false;
  // The label anchors, one vector per family, harvested from the same ComputeOverlay calls that
  // built the masks beside them (the horizon's from RebuildHorizonAnnotation's single call).
  // Each Label's `index` is REWRITTEN on the way in to name the config line it annotates: core
  // answers per request, and every request here carries exactly one angle, so the index it returns
  // is always 0 and would not survive the merge into one per-family list.
  //
  // Rebuilt whenever the masks are, plus whenever the family's own *_label_ switch flips — that
  // switch is an appearance field, so it can change under a reused consumer with the angle list
  // untouched, and a change detector that watched only the angles would keep an empty list exactly
  // when the user has just asked for the text.
  std::vector<annotation::Label> horizon_labels_;
  std::vector<annotation::Label> elevation_labels_;
  std::vector<annotation::Label> longitude_labels_;
  std::vector<annotation::Label> angular_dist_labels_;
  // What each label list was last built for, alongside the angle lists above. Only the switch
  // needs recording: everything else these lists depend on is already in the mask detectors.
  bool horizon_labels_built_for_ = false;
  bool elevation_labels_built_for_ = false;
  bool longitude_labels_built_for_ = false;
  bool angular_dist_labels_built_for_ = false;
  // Where the zenith and the nadir land on the canvas, each with its own `valid`. Points, not
  // masks: the marker is a ring of a radius the config names, so a whole W*H mask would encode
  // the appearance too and would have to be rebuilt whenever the radius changed. PostSnapshot
  // tests the two distances directly instead, which is O(1) per pixel.
  //
  // Built ONCE in the constructor, like visible_mask_ / horizon_mask_ and unlike the three mask
  // families above: this geometry depends only on the layout (lens / fov / view / resolution /
  // visible / overlap), every one of which NeedsRebuild pins for the consumer's whole life. The
  // appearance fields it does NOT depend on are exactly the ones ResetWith can change.
  annotation::CanvasPoint zenith_point_;
  annotation::CanvasPoint nadir_point_;
  SunParam sun_;
  float total_intensity_ = 0;
  float snapshot_intensity_ = 0;
  // Σ SimData::emitted_energy_ over every batch consumed since the last Reset(),
  // and its snapshot freeze — the absolute-scale denominator (see
  // ExposureScale). Parallel to total_intensity_/snapshot_intensity_ above in
  // every respect except what they measure: this one counts what the source
  // emitted, that one what reached a pixel. Both accumulate on BOTH consume
  // paths (Consume + ConsumeDeviceFused); charging only one of them was the
  // shape of two historical GUI brightness bugs.
  float total_emitted_energy_ = 0;
  float snapshot_emitted_energy_ = 0;
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
