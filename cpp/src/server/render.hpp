#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <map>
#include <memory>

#include "config/render_config.hpp"
#include "core/filter.hpp"
#include "server/consumer.hpp"

namespace icehalo {
namespace v3 {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

class RenderConsumer : public IConsume {
 public:
  RenderConsumer(RenderConfig config);

  void Consume(const SimData& data) override;
  Result GetResult() const override;

 private:
  RenderConfig config_;
  std::vector<FilterPtrU> filters_;
  Rotation rot_;  // camera pose rotation
  float diag_pix_ = 0;
  float total_intensity_ = 0;
  std::unique_ptr<float[]> internal_xyz_;
  std::unique_ptr<uint8_t[]> image_buffer_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_RENDER_H_
