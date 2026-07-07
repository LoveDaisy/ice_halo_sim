#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "config/component_table.hpp"
#include "config/render_config.hpp"
#include "server/consumer.hpp"
#include "util/logger.hpp"

namespace lumice {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

// See doc/accumulator-consumer-architecture.md §3 (state machine), §4 (thread safety), §6 (invariants).
class RenderConsumer : public IConsume {
 public:
  // task-336.2: `colored_mask` selects which uint64 component bits the
  // consumer should keep as separate Y-lanes (bit i set → allocate lane i).
  // Defaults to 0 → no lane state, pre-336 behavior bit-for-bit.
  explicit RenderConsumer(RenderConfig config, uint64_t colored_mask = 0);

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

  // task-336.2 read-only accessors (consumed by 336.3 compositing + white-box
  // tests). Both read snapshot state, mirroring GetRawXyzResult()'s tearing-free
  // contract.
  uint64_t ColoredMask() const { return colored_mask_; }
  // Returns pointer to a W*H float array holding the accumulated Y for the
  // component bit at `bit`, or nullptr when `bit` is out of range or is not a
  // participating bit (`colored_mask_ >> bit) & 1 == 0`).
  const float* GetComponentLaneY(uint8_t bit) const;

 private:
  // task-336.2: split out lane accumulation so Consume()'s cognitive complexity
  // does not grow with per-lane bookkeeping. Reads compacted arrays parallel to
  // the SpectrumToXyz(PerRay) call that immediately precedes it.
  void AccumulateComponentLanes(bool per_ray_wl, const float* wl_buf, float curr_wl, const float* w_buf,
                                const uint64_t* comp_buf, const int* xy_buf, size_t num);

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

  // task-336.2: per-component Y-lane state. `colored_mask_` is bit-i-set for
  // every component bit that carries a user-configured RGB (see
  // ComponentColorMap::colored_mask_). Only participating bits allocate a
  // W*H `float` lane; non-participating bits leave `lane_y_[bit]` as nullptr
  // (unique_ptr default) so the zero-config path costs 64 stale pointers.
  // Snapshot lanes shadow lane_y_ under the two-phase snapshot protocol
  // (PrepareSnapshot memcpy).
  uint64_t colored_mask_ = 0;
  std::vector<uint8_t> participating_bits_;  // bits set in colored_mask_, sorted asc
  std::array<std::unique_ptr<float[]>, ComponentTable::kMaxBits> lane_y_{};
  std::array<std::unique_ptr<float[]>, ComponentTable::kMaxBits> snapshot_lane_y_{};
  size_t lane_pixel_count_ = 0;  // W * H
  // Per-ray component mask side-cars (parallel to w_buf_ / overlap_w_buf_).
  // Grown in Consume() only when colored_mask_ != 0 (Minor #1 from review:
  // zero-config = zero extra allocation).
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
