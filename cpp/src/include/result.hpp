#ifndef INCLUDE_RESULT_H_
#define INCLUDE_RESULT_H_

#include <variant>

namespace icehalo {
namespace v3 {

struct NoneResult {};

struct RenderResult {
  int img_width_;
  int img_height_;
  const uint8_t* img_buffer_;
};

using Result = std::variant<NoneResult, RenderResult>;

}  // namespace v3
}  // namespace icehalo

#endif  // INCLUDE_RESULT_H_
