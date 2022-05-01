#ifndef CONSUMER_RENDER_H_
#define CONSUMER_RENDER_H_

#include <map>
#include <memory>

#include "config/render_config.hpp"
#include "consumer/consumer.hpp"

namespace icehalo {
namespace v3 {

constexpr int kMinWavelength = 360;
constexpr int kMaxWavelength = 830;

class Renderer : public IConsume {
 public:
  Renderer(RenderConfig config);

  void Consume(const SimData& data) override;
  const uint8_t* RenderToImage(float factor);

 private:
  RenderConfig config_;
  Rotation rot_;  // from world to camera
  float diag_pix_ = 0;
  float total_intensity_ = 0;
  std::map<int, std::unique_ptr<float[]>> internal_data_;  // (wl, data)

  std::unique_ptr<uint8_t[]> image_buffer_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CONSUMER_RENDER_H_
