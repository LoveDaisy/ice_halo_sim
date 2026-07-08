#ifndef CONFIG_RAYPATH_COLOR_CONFIG_H_
#define CONFIG_RAYPATH_COLOR_CONFIG_H_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "config/filter_config.hpp"
#include "core/def.hpp"

namespace lumice {

// User-visible schema for per-raypath color display.
//
// Design 2 (2026-07-08, doc/gui-custom-spectrum-and-raypath-color.md §4.0
// SUPERSEDES §4.1 Fork C + §4.7): a color class is decoupled from the physical
// filter. `match[]` is a list of placement-scoped color PREDICATES — each
// atom `{layer, crystal, <predicate>}` is one component bit, evaluated as a
// non-destructive pass on the CPU gate (physical filter still decides
// survival; the color predicate decides which surviving rays get tagged).
//
// `layer` / `crystal` reference the same ids used elsewhere in scene config
// (`SceneConfig::ms_[layer]` and `CrystalConfig::id_`). Predicate types reuse
// `SimpleFilterParam` verbatim (raypath / entry_exit / direction / crystal /
// none) — see filter_config.hpp. Default `predicate_` = `NoneFilterParam{}`
// is the wire-form "no `type` field", meaning match-all (whole-crystal color).

struct RaypathColorRef {
  IdType layer_ = 0;
  IdType crystal_ = kInvalidId;
  SimpleFilterParam predicate_ = NoneFilterParam{};
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
// per-class fields (combine=any, visible=true, solo=false) and omits the
// predicate `type` field when it is `NoneFilterParam` (match-all whole-crystal).
void to_json(nlohmann::json& j, const RaypathColorRef& r);
void from_json(const nlohmann::json& j, RaypathColorRef& r);
void to_json(nlohmann::json& j, const ColorClassConfig& c);
void from_json(const nlohmann::json& j, ColorClassConfig& c);
void to_json(nlohmann::json& j, const RaypathColorConfig& c);
void from_json(const nlohmann::json& j, RaypathColorConfig& c);

}  // namespace lumice

#endif  // CONFIG_RAYPATH_COLOR_CONFIG_H_
