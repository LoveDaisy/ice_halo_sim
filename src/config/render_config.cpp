#include "config/render_config.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>

#include "config/config_compare.hpp"
#include "core/math.hpp"
#include "util/color_space.hpp"

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


// ========== ZenithNadirParam ==========
void to_json(nlohmann::json& j, const ZenithNadirParam& z) {
  j["enabled"] = z.enabled_;
  j["radius_px"] = z.radius_px_;
  j["opacity"] = z.opacity_;
  j["color"] = z.color_;
}

// Every key optional, including "enabled": a present-but-partial object keeps the member defaults
// for whatever it leaves out, which is the same rule GridLineParam::from_json follows for
// everything but its mandatory `value`. There is no mandatory key here — the struct's own defaults
// describe a complete marker on their own.
void from_json(const nlohmann::json& j, ZenithNadirParam& z) {
  if (j.contains("enabled")) {
    j.at("enabled").get_to(z.enabled_);
  }
  if (j.contains("radius_px")) {
    j.at("radius_px").get_to(z.radius_px_);
  }
  if (j.contains("opacity")) {
    j.at("opacity").get_to(z.opacity_);
  }
  if (j.contains("color")) {
    j.at("color").get_to(z.color_);
  }
}


// ========== MarkerStyleParam ==========
namespace {

// The id spellings, indexed by MarkerRefId. One table for both directions, so the writer and the
// reader cannot drift into two vocabularies.
//
// The words are the annotation layer's own (annotation::MarkerId), lowercased: a persisted schema
// that renamed them would give one concept two names across the C API boundary, which is the defect
// `longitude` vs `azimuth` was already fixed for once.
constexpr const char* kMarkerIdNames[] = {
  "zenith", "nadir", "sun", "subsun", "anthelion", "antisolar",
};
constexpr int kMarkerIdNameCount = static_cast<int>(sizeof(kMarkerIdNames) / sizeof(kMarkerIdNames[0]));

}  // namespace

void to_json(nlohmann::json& j, const MarkerStyleParam& m) {
  const auto idx = static_cast<int>(m.id_);
  // Defensive, not expected: every MarkerRefId value has a name. An id outside the table would
  // otherwise index out of bounds while serializing a struct the caller built in C++ without going
  // through any validation.
  j["id"] = (idx >= 0 && idx < kMarkerIdNameCount) ? kMarkerIdNames[idx] : kMarkerIdNames[0];
  j["enabled"] = m.enabled_;
  j["color"] = m.color_;
}

// "id" is the one MANDATORY key — unlike ZenithNadirParam, where every key is optional because the
// struct's defaults already describe a complete marker. Here they do not: an entry with no id names
// no direction, and defaulting it to the zenith would turn a malformed list into a plausible one.
// `enabled` and `color` stay optional and keep the member defaults, as everywhere else in this file.
void from_json(const nlohmann::json& j, MarkerStyleParam& m) {
  constexpr int kErrCodeMissingKey = 403;
  constexpr int kErrCodeInvalidValue = 404;

  if (!j.contains("id")) {
    throw nlohmann::detail::out_of_range::create(kErrCodeMissingKey, "grid.markers entry is missing key [id]", j);
  }
  const auto name = j.at("id").get<std::string>();
  const auto* found = std::find(std::begin(kMarkerIdNames), std::end(kMarkerIdNames), name);
  // Reject rather than fall back. NLOHMANN_JSON_SERIALIZE_ENUM, which every other enum in this file
  // uses, would silently map an unrecognized string to the first entry — here that is the zenith,
  // so a typo would draw a ring at a direction the author never named and report nothing.
  if (found == std::end(kMarkerIdNames)) {
    throw nlohmann::detail::out_of_range::create(kErrCodeInvalidValue, "unknown marker id [" + name + "]", j);
  }
  m.id_ = static_cast<MarkerRefId>(std::distance(std::begin(kMarkerIdNames), found));

  if (j.contains("enabled")) {
    j.at("enabled").get_to(m.enabled_);
  }
  if (j.contains("color")) {
    j.at("color").get_to(m.color_);
  }
}

bool HasDuplicateMarkerId(const std::vector<MarkerStyleParam>& markers, MarkerRefId* dup) {
  // O(n^2) over a list whose validated length cannot exceed the six ids that exist. A set would
  // cost an allocation to save fifteen integer comparisons.
  for (size_t i = 0; i < markers.size(); ++i) {
    for (size_t k = i + 1; k < markers.size(); ++k) {
      if (markers[i].id_ == markers[k].id_) {
        if (dup != nullptr) {
          *dup = markers[i].id_;
        }
        return true;
      }
    }
  }
  return false;
}


// ========== LensParam ==========
void to_json(nlohmann::json& j, const LensParam& l) {
  j["type"] = l.type_;
  j["fov"] = l.fov_;
}

void from_json(const nlohmann::json& j, LensParam& l) {
  constexpr int kErrCodeMissingKey = 403;
  constexpr int kErrCodeInvalidValue = 404;
  constexpr float kHalfShortEdge = 12.0f;  // half short edge of 35mm film (24mm / 2)

  j.at("type").get_to(l.type_);
  if (j.contains("fov")) {
    j.at("fov").get_to(l.fov_);
  } else if (j.contains("f")) {
    float f = j.at("f").get<float>();
    float d = kHalfShortEdge;
    // NOTE: f→fov formula must match the scale formula in render.cpp for each projection model.
    switch (l.type_) {
      case LensParam::kLinear:
        l.fov_ = std::atan2(d, f) * 2 * math::kRadToDegree;
        break;
      case LensParam::kFisheyeEqualArea:
      case LensParam::kDualFisheyeEqualArea:
        if (d / (2 * f) > 1.0f) {
          throw nlohmann::detail::out_of_range::create(
              kErrCodeInvalidValue, "focal length too short for equal area fisheye (f >= 6mm required)", j);
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
      case LensParam::kFisheyeOrthographic:
      case LensParam::kDualFisheyeOrthographic:
        // r = f * sin(theta); boundary r_max = d = kHalfShortEdge. fov = 2 * asin(d / f).
        // Unlike EA's r = 2f * sin(theta/2) (denominator 2f), orthographic uses f directly.
        if (d / f > 1.0f) {
          throw nlohmann::detail::out_of_range::create(
              kErrCodeInvalidValue, "focal length too short for orthographic fisheye (f >= 12mm required for fov=180)",
              j);
        }
        l.fov_ = std::asin(d / f) * 2 * math::kRadToDegree;
        break;
      case LensParam::kGlobe:
        // Globe's on-image scale uses focal = img_radius/tan(fov/2), identical
        // to the linear model (see ComputeLensScale / GUI globeInverse), so the
        // f→fov mapping mirrors linear.
        l.fov_ = std::atan2(d, f) * 2 * math::kRadToDegree;
        break;
    }
  } else {
    throw nlohmann::detail::out_of_range::create(kErrCodeMissingKey, "missing key [fov] or [f]", j);
  }

  // Validate fov range (skip Rectangular which uses fov=0 for full-sky)
  if (l.type_ != LensParam::kRectangular && (l.fov_ <= 0 || l.fov_ > MaxFov(l.type_))) {
    throw nlohmann::detail::out_of_range::create(
        kErrCodeInvalidValue,
        "fov must be in (0, " + std::to_string(static_cast<int>(MaxFov(l.type_))) + "] degrees for this lens type", j);
  }
}


float MaxFov(LensParam::LensType type) {
  switch (type) {
    case LensParam::kLinear:
      return 179.0f;  // tan(fov/2) singular at 180
    case LensParam::kFisheyeStereographic:
      return 359.0f;  // tan(fov/4) singular at 360
    case LensParam::kFisheyeOrthographic:
    case LensParam::kDualFisheyeOrthographic:
      return 180.0f;  // sin(theta) aliases past pi/2; theta > pi/2 rejected
    case LensParam::kGlobe:
      return 90.0f;  // beyond 90° wide-angle distortion grows without enlarging the sphere on screen
    default:
      return 360.0f;  // equal area, equidistant, dual fisheye, rectangular
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
  j["front"] = r.front_;
  // background_ is linear; the JSON key is sRGB. Twin of the decode-side conversion in
  // config_manager.cpp::ParseRenderConfig.
  j["background"] = { LinearToSrgb(r.background_[0]), LinearToSrgb(r.background_[1]), LinearToSrgb(r.background_[2]) };
  j["ray_color"] = r.ray_color_;
  j["intensity_factor"] = r.intensity_factor_;
  j["overlap"] = r.overlap_;
  j["ev_mode"] = r.ev_mode_;

  j["grid"].emplace("angular_dist", r.angular_dist_grid_);
  j["grid"].emplace("elevation", r.elevation_grid_);
  j["grid"].emplace("longitude", r.longitude_grid_);
  j["grid"].emplace("horizon", r.horizon_);
  j["grid"].emplace("horizon_label", r.horizon_label_);
  j["grid"].emplace("label", r.grid_label_);
  j["grid"].emplace("angular_dist_label", r.angular_dist_label_);
  j["grid"].emplace("zenith_nadir", r.zenith_nadir_);
  // The marker family: the list, then its two family-wide appearance fields as SIBLING keys rather
  // than as members of a wrapper object. Same shape as "horizon" / "horizon_label" and the rest of
  // this block, i.e. every appearance knob the grid has is one key under "grid".
  j["grid"].emplace("markers", r.markers_);
  j["grid"].emplace("markers_opacity", r.markers_opacity_);
  j["grid"].emplace("markers_radius_px", r.markers_radius_px_);
}


// See doc/accumulator-consumer-architecture.md §5.1 (layout vs. appearance classification),
// §5.2 (sizeof sentinel).
bool NeedsRebuild(const RenderConfig& a, const RenderConfig& b) {
  // Bump this when adding fields to RenderConfig — then classify as layout or appearance.
  // Still 192 after the three text-label switches were added: they landed in the tail padding
  // `horizon_` already carried ahead of ZenithNadirParam's 4-byte alignment. The number is a
  // tripwire for "a field was added", not a size budget, so an unchanged one is only safe to leave
  // once the new fields have actually been classified — and these three are APPEARANCE, not
  // layout. They change what is painted on top of the accumulated image and nothing about the
  // buffer it accumulates into, so a config that flips one reaches an existing consumer through
  // ResetWith() with no rebuild (which is what RebuildHorizonLabels and the two mask rebuilds are
  // re-run from there for).
  // 192 -> 224 for the marker family: std::vector markers_ (24) + two floats (8), landing on
  // exactly 32 with no new padding. All three are APPEARANCE, like zenith_nadir_ beside them: they
  // change what is painted on top of the accumulated image and nothing about the buffer it
  // accumulates into, so a config that edits them reaches an existing consumer through ResetWith()
  // with no rebuild — which is why RebuildMarkerPoints() is called from there as well as from the
  // constructor.
  static_assert(sizeof(RenderConfig) == 224, "Update NeedsRebuild when RenderConfig fields change");
  // Compare layout-affecting fields only. Appearance fields (background, ray_color,
  // intensity_factor, ev_mode, grids) are handled by ResetWith() without rebuild.

  // id_ is excluded: map key matching guarantees id agreement on the reuse path.
  return !std::equal(std::begin(a.resolution_), std::end(a.resolution_), std::begin(b.resolution_)) ||
         !(a.lens_ == b.lens_) ||
         !std::equal(std::begin(a.lens_shift_), std::end(a.lens_shift_), std::begin(b.lens_shift_)) ||
         !(a.view_ == b.view_) || a.visible_ != b.visible_ || a.front_ != b.front_ || a.overlap_ != b.overlap_;
}

}  // namespace lumice