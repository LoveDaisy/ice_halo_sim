#ifndef CONFIG_RENDER_CONFIG_H_
#define CONFIG_RENDER_CONFIG_H_

#include <nlohmann/json.hpp>
#include <vector>

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
    kFisheyeOrthographic,
    kDualFisheyeOrthographic,
    kGlobe,
  };

  LensType type_;
  float fov_;
};

NLOHMANN_JSON_SERIALIZE_ENUM(  // declare
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
        { LensParam::kFisheyeOrthographic, "fisheye_orthographic" },
        { LensParam::kDualFisheyeOrthographic, "dual_fisheye_orthographic" },
        { LensParam::kGlobe, "globe" },
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

  // Which anchor the mono/composite exposure scale is measured against. See
  // doc/ev-pipeline-architecture.md and RenderConsumer::ExposureScale().
  //   kRelative — anchor to the frame's own P99 (what the GUI has always displayed): the image
  //               keeps its look as ray_num grows, but the config alone does NOT determine the
  //               output brightness (ray_num co-determines it). This is the default.
  //   kAbsolute — anchor to the EMITTED energy: the scale is fixed by light source + ray budget,
  //               so two simulations at the same EV are directly comparable.
  enum EvMode {
    kRelative,
    kAbsolute,
  };

  IdType id_{};
  LensParam lens_{ LensParam::kLinear, 90.0f };
  int lens_shift_[2]{};  // dx, dy
  int resolution_[2]{};  // width, height
  ViewParam view_{};
  VisibleRange visible_ = kUpper;

  // Linear RGB. The JSON "background" key is sRGB; to_json / ParseRenderConfig convert.
  float background_[3]{};
  float ray_color_[3]{ -1.0f, -1.0f, -1.0f };  // r, g, b
  // Brightness scaling for CLI output (PostSnapshot). GUI uses exposure_offset (EV stops) in
  // gui_state.hpp directly; the two are related by intensity_factor = 2^exposure_offset but serve
  // different paths and may differ at runtime (GUI updates EV without re-committing config).
  float intensity_factor_ = 1.0f;
  float overlap_ = 0.0f;  // Dual fisheye overlap zone |sky.z| threshold (sin value). 0 = no overlap.
  // Appearance field (like intensity_factor_): it selects WHICH exposure formula PostSnapshot()
  // and the compositor use, never the accumulation layout, so a change needs no consumer rebuild.
  EvMode ev_mode_ = kRelative;

  std::vector<GridLineParam> angular_dist_grid_;
  std::vector<GridLineParam> elevation_grid_;
  // Opt-in, not on by default. It was `true` for the four years the field parsed and drew nothing,
  // which cost nothing; now that it draws, `true` would put a horizon line into every existing
  // config that never asked for one (13 of the 14 reference renders in test/e2e-correctness/ set
  // no `grid.outline` key at all). Turning an annotation on for every render is a product decision
  // nobody has made, so the default states the one thing that is certain: draw it when asked.
  bool horizon_ = false;
};

NLOHMANN_JSON_SERIALIZE_ENUM(    // declare
    RenderConfig::VisibleRange,  // type
    {
        { RenderConfig::kUpper, "upper" },
        { RenderConfig::kLower, "lower" },
        { RenderConfig::kFull, "full" },
    })

// kRelative FIRST on purpose: NLOHMANN_JSON_SERIALIZE_ENUM maps any unrecognized string to the
// first table entry, so a misspelled value lands on the same mode a MISSING key does.
NLOHMANN_JSON_SERIALIZE_ENUM(  // declare
    RenderConfig::EvMode,      // type
    {
        { RenderConfig::kRelative, "relative" },
        { RenderConfig::kAbsolute, "absolute" },
    })

void to_json(nlohmann::json& j, const RenderConfig& r);

// Returns true if layout-affecting fields differ (resolution, lens, view, visible, overlap, filter).
// Appearance-only changes (background, ray_color, intensity_factor, grids) return false.
bool NeedsRebuild(const RenderConfig& old_cfg, const RenderConfig& new_cfg);

}  // namespace lumice

#endif  // CONFIG_RENDER_CONFIG_H_
