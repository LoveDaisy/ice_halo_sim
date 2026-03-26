#include "config/render_config.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <variant>

#include "config/config_compare.hpp"
#include "config/filter_config.hpp"
#include "core/math.hpp"

namespace lumice {

// ========== ViewParam ==========
void to_json(nlohmann::json& j, const ViewParam& v) {
  j["azimuth"] = v.az_;
  j["elevation"] = v.el_;
  j["roll"] = v.ro_;
}

void from_json(const nlohmann::json& j, ViewParam& v) {
  if (j.contains("azimuth")) {
    j.at("azimuth").get_to(v.az_);
  }
  if (j.contains("elevation")) {
    j.at("elevation").get_to(v.el_);
  }
  if (j.contains("roll")) {
    j.at("roll").get_to(v.ro_);
  }
}


// ========== GridLineParam ==========
void to_json(nlohmann::json& j, const GridLineParam& l) {
  j["value"] = l.value_;
  j["color"] = l.color_;
  j["opacity"] = l.opacity_;
  j["width"] = l.width_;
}

void from_json(const nlohmann::json& j, GridLineParam& l) {
  j.at("value").get_to(l.value_);

  if (j.contains("color")) {
    j.at("color").get_to(l.color_);
  }
  if (j.contains("opacity")) {
    j.at("opacity").get_to(l.opacity_);
  }
  if (j.contains("width")) {
    j.at("width").get_to(l.width_);
  }
}


// ========== LensParam ==========
void to_json(nlohmann::json& j, const LensParam& l) {
  j["type"] = l.type_;
  j["fov"] = l.fov_;
}

void from_json(const nlohmann::json& j, LensParam& l) {
  constexpr int kErrCodeMissingKey = 403;
  constexpr int kErrCodeInvalidValue = 404;
  constexpr float kHalfDiagLen = 21.63f;  // half diagonal of 35mm film (43.27mm / 2)

  j.at("type").get_to(l.type_);
  if (j.contains("fov")) {
    j.at("fov").get_to(l.fov_);
  } else if (j.contains("f")) {
    float f = j.at("f").get<float>();
    float d = kHalfDiagLen;
    // NOTE: f→fov formula must match the scale formula in render.cpp for each projection model.
    switch (l.type_) {
      case LensParam::kLinear:
        l.fov_ = std::atan2(d, f) * 2 * math::kRadToDegree;
        break;
      case LensParam::kFisheyeEqualArea:
      case LensParam::kDualFisheyeEqualArea:
        if (d / (2 * f) > 1.0f) {
          throw nlohmann::detail::out_of_range::create(
              kErrCodeInvalidValue, "focal length too short for equal area fisheye (f >= 10.815mm required)", j);
        }
        l.fov_ = std::asin(d / (2 * f)) * 4 * math::kRadToDegree;
        break;
      case LensParam::kFisheyeEquidistant:
      case LensParam::kDualFisheyeEquidistant:
        l.fov_ = (d / f) * math::kRadToDegree;
        break;
      case LensParam::kFisheyeStereographic:
      case LensParam::kDualFisheyeStereographic:
        l.fov_ = std::atan(d / (2 * f)) * 4 * math::kRadToDegree;
        break;
      case LensParam::kRectangular:
        l.fov_ = 0;  // Rectangular is always full-sky; fov is ignored
        break;
    }
  } else {
    throw nlohmann::detail::out_of_range::create(kErrCodeMissingKey, "missing key [fov] or [f]", j);
  }

  // Validate fov range (skip Rectangular which uses fov=0 for full-sky)
  if (l.type_ != LensParam::kRectangular && (l.fov_ <= 0 || l.fov_ > 360)) {
    throw nlohmann::detail::out_of_range::create(kErrCodeInvalidValue, "fov must be in (0, 360] degrees", j);
  }
}


// ========== RenderConfig ==========
void to_json(nlohmann::json& j, const RenderConfig& r) {
  j["id"] = r.id_;
  j["resolution"] = r.resolution_;
  j["lens"] = r.lens_;
  j["lens_shift"] = r.lens_shift_;
  j["view"] = r.view_;
  j["visible"] = r.visible_;
  j["background"] = r.background_;
  j["ray_color"] = r.ray_color_;
  j["opacity"] = r.opacity_;
  j["intensity_factor"] = r.intensity_factor_;
  j["norm_mode"] = r.norm_mode_;

  j["grid"].emplace("central", r.central_grid_);
  j["grid"].emplace("elevation", r.elevation_grid_);
  j["grid"].emplace("outline", r.celestial_outline_);

  for (const auto& f : r.ms_filter_) {
    if (f.id_ == kInvalidId || (std::holds_alternative<SimpleFilterParam>(f.param_) &&
                                std::holds_alternative<NoneFilterParam>(std::get<SimpleFilterParam>(f.param_)))) {
      j["filter"].emplace_back(-1);
    } else {
      j["filter"].emplace_back(f.id_);
    }
  }
}


bool NeedsRebuild(const RenderConfig& a, const RenderConfig& b) {
  // Bump this when adding fields to RenderConfig — then classify as layout or appearance.
  static_assert(sizeof(RenderConfig) == 160, "Update NeedsRebuild when RenderConfig fields change");
  // Compare layout-affecting fields only. Appearance fields (background, ray_color, opacity,
  // intensity_factor, norm_mode, grids) are handled by ResetWith() without rebuild.
  // id_ is excluded: map key matching guarantees id agreement on the reuse path.
  return !std::equal(std::begin(a.resolution_), std::end(a.resolution_), std::begin(b.resolution_)) ||
         !(a.lens_ == b.lens_) ||
         !std::equal(std::begin(a.lens_shift_), std::end(a.lens_shift_), std::begin(b.lens_shift_)) ||
         !(a.view_ == b.view_) || a.visible_ != b.visible_ || a.ms_filter_ != b.ms_filter_;
}

}  // namespace lumice