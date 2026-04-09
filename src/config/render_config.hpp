#ifndef CONFIG_RENDER_CONFIG_H_
#define CONFIG_RENDER_CONFIG_H_

#include <nlohmann/json.hpp>
#include <vector>

#include "config/filter_config.hpp"
#include "core/def.hpp"

namespace lumice {

struct ViewParam {
  float az_{};  // Azimuth
  float el_{};  // Elevation
  float ro_{};  // Roll
};

void to_json(nlohmann::json& j, const ViewParam& v);
void from_json(const nlohmann::json& j, ViewParam& v);

struct GridLineParam {
  float value_{};
  float width_{ 1.0f };
  float opacity_{ 1.0f };
  float color_[3]{ 1.0f, 1.0f, 1.0f };
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

// Returns the maximum valid FOV (in degrees) for the given lens type.
float MaxFov(LensParam::LensType type);

struct RenderConfig {
  enum VisibleRange {
    kUpper,
    kLower,
    kFull,
  };

  IdType id_{};
  LensParam lens_{ LensParam::kLinear, 90.0f };
  int lens_shift_[2]{};  // dx, dy
  int resolution_[2]{};  // width, height
  ViewParam view_{};
  VisibleRange visible_ = kUpper;

  float background_[3]{};                      // r, g, b
  float ray_color_[3]{ -1.0f, -1.0f, -1.0f };  // r, g, b
  float opacity_ = 1.0f;
  // Brightness scaling for CLI output (PostSnapshot). GUI uses exposure_offset (EV stops) in
  // gui_state.hpp directly; the two are related by intensity_factor = 2^exposure_offset but serve
  // different paths and may differ at runtime (GUI updates EV without re-committing config).
  float intensity_factor_ = 1.0f;
  int norm_mode_ = 0;     // 0=absolute (W*H), 1=adaptive (non-zero pixel count). GUI defaults to 1.
  float overlap_ = 0.0f;  // Dual fisheye overlap zone |sky.z| threshold (sin value). 0 = no overlap.

  std::vector<GridLineParam> central_grid_;
  std::vector<GridLineParam> elevation_grid_;
  bool celestial_outline_ = true;

  std::vector<FilterConfig> ms_filter_;  // for multi-scattering
};

NLOHMANN_JSON_SERIALIZE_ENUM(    // declear
    RenderConfig::VisibleRange,  // type
    {
        { RenderConfig::kUpper, "upper" },
        { RenderConfig::kLower, "lower" },
        { RenderConfig::kFull, "full" },
    })

void to_json(nlohmann::json& j, const RenderConfig& r);

// Returns true if layout-affecting fields differ (resolution, lens, view, visible, overlap, filter).
// Appearance-only changes (background, ray_color, opacity, intensity_factor, norm_mode, grids) return false.
bool NeedsRebuild(const RenderConfig& old_cfg, const RenderConfig& new_cfg);

}  // namespace lumice

#endif  // CONFIG_RENDER_CONFIG_H_
