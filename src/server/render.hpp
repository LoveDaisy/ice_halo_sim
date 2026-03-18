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
  void ResetAccumulation() override;
  void PrepareSnapshot() override;
  void PostSnapshot() override;
  Result GetResult() const override;
  RawXyzResult GetRawXyzResult() const;

 private:
  RenderConfig config_;
  std::vector<FilterPtrU> filters_;
  Rotation rot_;  // camera pose rotation
  float diag_pix_ = 0;
  float total_intensity_ = 0;
  float snapshot_intensity_ = 0;
  std::unique_ptr<float[]> internal_xyz_;
  std::unique_ptr<float[]> snapshot_xyz_;
  std::unique_ptr<uint8_t[]> snapshot_image_buffer_;  // produced by PostSnapshot()

  // Pre-allocated Consume() buffers (grow-only)
  std::unique_ptr<float[]> d_buf_;
  std::unique_ptr<float[]> w_buf_;
  std::unique_ptr<int[]> xy_buf_;
  size_t buf_capacity_ = 0;

  Logger logger_{ "Render" };
};

}  // namespace lumice

#endif  // CONSUMER_RENDER_H_
