#ifndef CONFIG_RAYPATH_COLOR_CONFIG_H_
#define CONFIG_RAYPATH_COLOR_CONFIG_H_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "core/def.hpp"

namespace lumice {

// User-visible schema for per-raypath color display (task-339.2 color-class
// schema; supersedes 336.1's flat positional form). The top-level
// `raypath_color` is an ordered list of color classes; each class binds one
// RGB color to a boolean predicate over `component` bits (see
// doc/gui-custom-spectrum-and-raypath-color.md §4.7 定案 2). List order is
// z-order (used by 339.4's per-class compositor).
//
// A `RaypathColorRef` picks one or more component bits by id (NOT by internal
// setting_[] slot index). The `crystal`/`filter` fields refer to
// `CrystalConfig::id_` / `FilterConfig::id_`, matching the ids used elsewhere
// in scene config. Resolution to ci (setting_[] slot) and to bits happens in
// the builder (BuildColorClassTable). `has_filter_ == false` means "the
// none-filter scattering setting for that crystal" (whole-crystal virtual
// summand emitted by task-339.1). `has_summand_ == false` means "any summand
// of the referenced filter" (OR-union over all summand bits of that filter).

struct RaypathColorRef {
  IdType layer_ = 0;
  IdType crystal_ = kInvalidId;
  IdType filter_ = kInvalidId;
  IdType summand_ = 0;
  bool has_filter_ = false;
  bool has_summand_ = false;
};

// One color class = an RGB color, a boolean combine over the members, a set of
// member refs, and per-class display-time visibility. `combine_` is the raw
// string ("any" | "all"); unknown values are rejected by the builder (NOT the
// DTO — DTO reads original text, semantics stays in builder).
struct ColorClassConfig {
  float color_[3]{};
  std::string combine_ = "any";
  bool visible_ = true;
  bool solo_ = false;
  std::vector<RaypathColorRef> match_;
};

struct RaypathColorConfig {
  std::vector<ColorClassConfig> classes_;
  // Display-time composite mode ("dominant" | "additive" | "painter"). Unknown
  // strings degrade to dominant + warn (in the compositor adapter/builder); the
  // DTO layer stores the string verbatim.
  std::string mode_ = "dominant";
};

// JSON representation:
//   - bare-array of color classes when `mode_ == "dominant"` (preserves the
//     minimal wire format for the default);
//   - object `{ "mode": ..., "classes": [ ... ] }` when mode is non-default.
// A missing top-level "raypath_color" key in the ConfigManager JSON is treated
// as an empty config (zero regression against configs without color) —
// handled by ConfigManager's from_json, not here. `to_json` omits default
// per-class fields (combine=any, visible=true, solo=false) and per-ref
// optional fields (filter/summand when not present) to keep configs small.
void to_json(nlohmann::json& j, const RaypathColorRef& r);
void from_json(const nlohmann::json& j, RaypathColorRef& r);
void to_json(nlohmann::json& j, const ColorClassConfig& c);
void from_json(const nlohmann::json& j, ColorClassConfig& c);
void to_json(nlohmann::json& j, const RaypathColorConfig& c);
void from_json(const nlohmann::json& j, RaypathColorConfig& c);

}  // namespace lumice

#endif  // CONFIG_RAYPATH_COLOR_CONFIG_H_
