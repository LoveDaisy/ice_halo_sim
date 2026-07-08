#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/render_config.hpp"
#include "server/consumer.hpp"
#include "util/logger.hpp"

namespace lumice {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

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
  float total_intensity_ = 0;
  float snapshot_intensity_ = 0;
  int effective_pix_ = 0;  // Non-zero pixel count from last PrepareSnapshot
  std::unique_ptr<float[]> internal_xyz_;
  std::unique_ptr<float[]> comp_xyz_;  // Neumaier compensation buffer (S1 device-fused)
  std::unique_ptr<float[]> snapshot_xyz_;
  std::unique_ptr<float[]> snapshot_work_;            // PostSnapshot work buffer (preserves snapshot_xyz_)
  std::unique_ptr<uint8_t[]> snapshot_image_buffer_;  // produced by PostSnapshot()

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
