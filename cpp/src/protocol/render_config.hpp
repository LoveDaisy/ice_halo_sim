#ifndef PROTOCOL_RENDER_CONFIG_H_
#define PROTOCOL_RENDER_CONFIG_H_

#include <vector>

#include "core/def.hpp"
#include "json.hpp"
#include "protocol/filter_config.hpp"

namespace icehalo {
namespace v3 {

struct ViewParam {
  float az_;  // Azimuth
  float el_;  // Elevation
  float ro_;  // Roll
  float d_;   // Distance
};

void to_json(nlohmann::json& j, const ViewParam& v);
void from_json(const nlohmann::json& j, ViewParam& v);

struct GridLineParam {
  float value_;
  float width_;
  float opacity_;
  float color_[3];  // r, g, b
};

void to_json(nlohmann::json& j, const GridLineParam& l);
void from_json(const nlohmann::json& j, GridLineParam& l);

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

NLOHMANN_JSON_SERIALIZE_ENUM(  // declear
    LensParam::LensType,       // type
    {
        { LensParam::kLinear, "linear" },
        { LensParam::kFisheyeEqualArea, "fisheye_equal_area" },
        { LensParam::kFisheyeEquidistant, "fisheye_equidistant" },
        { LensParam::kFisheyeStereographic, "fisheye_stereographic" },
        { LensParam::kDualFisheyeEqualArea, "dual_fisheye_equal_area" },
        { LensParam::kDualFisheyeEquidistant, "dual_fisheye_equidistant" },
        { LensParam::kDualFisheyeStereographic, "dual_fisheye_stereographic" },
        { LensParam::kRectangular, "rectangular" },
    })

void to_json(nlohmann::json& j, const LensParam& l);
void from_json(const nlohmann::json& j, LensParam& l);

struct RenderConfig {
  enum VisibleRange {
    kUpper,
    kLower,
    kFull,
  };

  IdType id_;
  LensParam lens_;
  int lens_shift_[2];  // dx, dy
  int resolution_[2];  // width, height
  ViewParam view_;
  VisibleRange visible_;

  float background_[3];  // r, g, b
  float ray_color_[3];   // r, g, b
  float opacity_;

  std::vector<GridLineParam> central_grid_;
  std::vector<GridLineParam> elevation_grid_;
  bool celestial_outline_;

  FilterConfig filter_;
};

NLOHMANN_JSON_SERIALIZE_ENUM(    // declear
    RenderConfig::VisibleRange,  // type
    {
        { RenderConfig::kUpper, "upper" },
        { RenderConfig::kLower, "lower" },
        { RenderConfig::kFull, "full" },
    })

void to_json(nlohmann::json& j, const RenderConfig& r);

}  // namespace v3
}  // namespace icehalo

#endif  // PROTOCOL_RENDER_CONFIG_H_
