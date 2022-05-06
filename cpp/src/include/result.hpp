#ifndef INCLUDE_RESULT_H_
#define INCLUDE_RESULT_H_

#include <cstddef>
#include <variant>

namespace icehalo {
namespace v3 {

struct NoneResult {};

struct RenderResult {
  int renderer_id_;
  int img_width_;
  int img_height_;
  uint8_t* img_buffer_;
};

struct StatsResult {
  size_t ray_seg_num_;
  size_t sim_ray_num_;
  size_t crystal_num_;
};

using Result = std::variant<NoneResult, RenderResult, StatsResult>;

}  // namespace v3
}  // namespace icehalo

#endif  // INCLUDE_RESULT_H_
