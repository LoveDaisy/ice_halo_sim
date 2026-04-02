#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <memory>

#include "config/render_config.hpp"
#include "core/filter.hpp"
#include "server/consumer.hpp"
#include "util/logger.hpp"

namespace lumice {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

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
  std::vector<FilterPtrU> filters_;
  Rotation rot_;  // camera pose rotation
  float diag_pix_ = 0;
  float total_intensity_ = 0;
  float snapshot_intensity_ = 0;
  int effective_pix_ = 0;  // Non-zero pixel count from last PrepareSnapshot
  std::unique_ptr<float[]> internal_xyz_;
  std::unique_ptr<float[]> snapshot_xyz_;
  std::unique_ptr<float[]> snapshot_work_;            // PostSnapshot work buffer (preserves snapshot_xyz_)
  std::unique_ptr<uint8_t[]> snapshot_image_buffer_;  // produced by PostSnapshot()

  // Pre-allocated Consume() buffers (grow-only)
  std::unique_ptr<float[]> d_buf_;
  std::unique_ptr<float[]> w_buf_;
  std::unique_ptr<int[]> xy_buf_;
  size_t buf_capacity_ = 0;

  // Profiling counters (accumulated across Consume calls, for benchmark analysis)
  size_t consume_count_ = 0;
  double consume_filter_us_ = 0;  // FilterRay + copy to buffers
  double consume_proj_us_ = 0;    // lens projection
  double consume_accum_us_ = 0;   // SpectrumToXyz scatter writes

  Logger logger_{ "Render" };
};

}  // namespace lumice

#endif  // CONSUMER_RENDER_H_
