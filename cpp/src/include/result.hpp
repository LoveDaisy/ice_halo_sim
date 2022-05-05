#ifndef INCLUDE_RESULT_H_
#define INCLUDE_RESULT_H_

#include <cstddef>
#include <variant>

namespace icehalo {
namespace v3 {

struct NoneResult {};

struct RenderResult {
  int img_width_;
  int img_height_;
  uint8_t* img_buffer_;
};

struct StatsResult {
  size_t total_ray_num_;
  size_t total_crystal_num_;
};

using Result = std::variant<NoneResult, RenderResult, StatsResult>;

}  // namespace v3
}  // namespace icehalo

#endif  // INCLUDE_RESULT_H_
