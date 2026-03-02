#include "config/render_config.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <variant>

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
  constexpr float kHalfDiagLen = 21.63f;

  j.at("type").get_to(l.type_);
  if (j.contains("fov")) {
    j.at("fov").get_to(l.fov_);
  } else if (j.contains("f")) {
    float f = j.at("f").get<float>();
    l.fov_ = std::atan2(kHalfDiagLen, f) * 2 * math::kRadToDegree;  // atan2(y, x)
  } else {
    throw nlohmann::detail::out_of_range::create(kErrCodeMissingKey, "missing key [fov] or [f]", j);
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

}  // namespace lumice