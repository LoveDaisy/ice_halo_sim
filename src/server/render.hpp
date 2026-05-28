#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <memory>

#include "config/render_config.hpp"
#include "server/consumer.hpp"
#include "util/logger.hpp"

namespace lumice {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

// See doc/accumulator-consumer-architecture.md §3 (state machine), §4 (thread safety), §6 (invariants).
class RenderConsumer : public IConsume {
 public:
  explicit RenderConsumer(RenderConfig config);

  void Consume(const SimData& data) override;
  void PrepareSnapshot() override;
  void CountEffectivePixels();
  void PostSnapshot() override;
  Result GetResult() const override;
  RawXyzResult GetRawXyzResult() const;
  void Reset() override;
  void ResetWith(const RenderConfig& new_config);
  void LogConsumeProfile() const;  // Dump accumulated profiling stats

 private:
  RenderConfig config_;
  Rotation rot_;  // camera pose rotation
  float short_pix_ = 0;
  float total_intensity_ = 0;
  float snapshot_intensity_ = 0;
  int effective_pix_ = 0;  // Non-zero pixel count from last PrepareSnapshot
  std::unique_ptr<float[]> internal_xyz_;
  std::unique_ptr<float[]> snapshot_xyz_;
  std::unique_ptr<float[]> snapshot_work_;            // PostSnapshot work buffer (preserves snapshot_xyz_)
  std::unique_ptr<uint8_t[]> snapshot_image_buffer_;  // produced by PostSnapshot()

  // Anchor lane: lazily populated when a filter spec is active (SimData::anchor_d_/anchor_w_
  // is non-empty). Accumulates filter-fail outgoing rays; combined with internal_xyz_ at
  // snapshot time yields filter-independent total emission, so anchor_p995_y_ stays stable
  // across filter toggles. When no filter is configured the anchor buffers remain unallocated
  // / zero and SyncFromPoller's degenerate branch falls back to filter-self snapshot.
  // anchor_total_intensity_ tracks the anchor lane weight (filter-fail outgoing rays only);
  // the filter-independent total intensity at snapshot time is
  // anchor_snapshot_intensity_ = snapshot_intensity_ (filter-pass) + anchor_total_intensity_ (filter-fail).
  // See doc/filter-architecture.md §7.
  float anchor_total_intensity_ = 0;
  float anchor_snapshot_intensity_ = 0;
  float anchor_p995_y_ = 0;
  std::unique_ptr<float[]> anchor_internal_xyz_;
  std::unique_ptr<float[]> anchor_snapshot_xyz_;

  // Pre-allocated Consume() buffers (grow-only)
  std::unique_ptr<float[]> d_buf_;
  std::unique_ptr<float[]> w_buf_;
  std::unique_ptr<int[]> xy_buf_;
  std::unique_ptr<float[]> overlap_w_buf_;  // weight copy for overlap dual-write pass
  size_t buf_capacity_ = 0;

  // Profiling counters (accumulated across Consume calls, for benchmark analysis)
  size_t consume_count_ = 0;
  double consume_proj_us_ = 0;   // memcpy outgoing buffers + lens projection
  double consume_accum_us_ = 0;  // SpectrumToXyz scatter writes

  Logger logger_{ "Render" };
};

}  // namespace lumice

#endif  // CONSUMER_RENDER_H_
