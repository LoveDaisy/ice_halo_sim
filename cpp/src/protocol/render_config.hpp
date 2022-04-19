#ifndef PROTOCOL_RENDER_CONFIG_H_
#define PROTOCOL_RENDER_CONFIG_H_

#include <vector>

#include "core/core_def.hpp"
#include "protocol/filter_config.hpp"

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

struct ResolutionParam {
  int width_;
  int height_;
};

struct LensParam {
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

  LensType type_;
  float fov_;
};

struct RenderConfig {
  enum VisibleRange {
    kUpper,
    kLower,
    kFull,
  };

  IdType id_;
  LensParam lens_;
  ResolutionParam resolution_;
  ViewParam view_;
  VisibleRange visible_;

  RgbColor background_;
  RgbColor ray_color_;
  float opacity_;

  std::vector<GridLineParam> central_grid_;
  std::vector<GridLineParam> elevation_grid_;
  bool celestial_outline_;

  FilterConfig filter_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // PROTOCOL_RENDER_CONFIG_H_
