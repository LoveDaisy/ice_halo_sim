#ifndef PROTOCOL_RENDER_CONFIG_H_
#define PROTOCOL_RENDER_CONFIG_H_

#include <vector>

#include "core/core_def.hpp"

namespace icehalo {
namespace v3 {

struct RgbColor {
  float r_;
  float g_;
  float b_;
};

struct ViewParam {
  float az_;  // Azimuth
  float el_;  // Elevation
  float ro_;  // Roll
  float d_;   // Distance
};

struct GridLineParam {
  float value_;
  float with_;
  float opacity_;
  RgbColor color_;
};

struct RenderConfig {
  enum LensType {
    kLinear,
    kFisheyeEqualArea,
    kFisheyeEquidistant,
    kFisheyeStereographic,
    kDualFisheyeEqualArea,
    kDualFisheyeEquidistant,
    kDualFisheyeStereographic,
    kRectangular,
  };

  enum VisibleRange {
    kUpper,
    kLower,
    kFull,
  };

  IdType id_;
  LensType lens_;
  int width_;
  int height_;
  ViewParam view_;
  VisibleRange visible_;

  RgbColor background_;
  RgbColor ray_color_;
  float opacity_;

  std::vector<GridLineParam> central_grid_;
  std::vector<GridLineParam> elevation_grid_;
  bool celestial_outline_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // PROTOCOL_RENDER_CONFIG_H_
