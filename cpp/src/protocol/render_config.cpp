#include "protocol/render_config.hpp"

#include <cmath>
#include <variant>

#include "core/math.hpp"
#include "io/json_util.hpp"
#include "json.hpp"
#include "protocol/filter_config.hpp"

namespace icehalo {
namespace v3 {

// ========== ViewParam ==========
void to_json(nlohmann::json& j, const ViewParam& v) {
  j["azimuth"] = v.az_;
  j["elevation"] = v.el_;
  j["roll"] = v.ro_;
}

void from_json(const nlohmann::json& j, ViewParam& v) {
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "azimuth", v.az_)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "elevation", v.el_)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "roll", v.ro_)
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

  l.color_[0] = 1.0f;
  l.color_[1] = 1.0f;
  l.color_[2] = 1.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "color", l.color_)

  l.opacity_ = 1.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "opacity", l.opacity_)

  l.width_ = 1.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j, "width", l.width_)
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
  j["ray"] = r.ray_color_;
  j["opacity"] = r.opacity_;

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

}  // namespace v3
}  // namespace icehalo