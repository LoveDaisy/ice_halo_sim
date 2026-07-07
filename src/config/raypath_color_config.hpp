#ifndef CONFIG_RAYPATH_COLOR_CONFIG_H_
#define CONFIG_RAYPATH_COLOR_CONFIG_H_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "core/def.hpp"

namespace lumice {

// User-visible schema for per-raypath color display (task-336.1).
//
// Each entry pins one triple (layer, crystal_slot, summand) to an RGB color.
// The triple keys directly into `ComponentTable::entries_`:
//
// - layer_          : index into `SceneConfig::ms_[]`
// - crystal_id_     : index into `SceneConfig::ms_[layer].setting_[]` — the
//                     STATIC config slot. JSON key is `crystal_slot` (not
//                     `crystal`) to avoid collision with the top-level
//                     `"crystal"` key which refers to `CrystalConfig::id_`.
//                     The C++ field name mirrors `ComponentTableEntry::crystal_id_`
//                     for consistency inside the codebase (see
//                     `component_table.hpp` — same naming pitfall documented
//                     there).
// - summand_idx_    : index into the OR-of-AND summand list emitted by
//                     `BuildComponentTable` (component_table.cpp:52-65). This
//                     is an internal implementation detail of the current
//                     filter-expansion algorithm; changes to the summand
//                     enumeration order will silently invalidate existing
//                     `summand` indices in user configs. Any refactor of the
//                     filter-expansion pipeline must revisit this coupling.
// - color_          : linear RGB in [0, 1], same convention as
//                     `RenderConfig::background_` / `RenderConfig::ray_color_`.
//                     No range validation (trust input, match existing
//                     RenderConfig color-field behavior).
struct RaypathColorEntry {
  IdType layer_ = 0;
  IdType crystal_id_ = 0;
  IdType summand_idx_ = 0;
  float color_[3]{};
  // task-336.3: visibility axis, orthogonal to the composite mode. Absent in
  // JSON → visible (default true) and not solo (default false) = the 336.1
  // behavior (every colored bit participates). `solo_` (when any entry sets it)
  // restricts the visible set to solo'd bits, overriding hide.
  bool visible_ = true;
  bool solo_ = false;
};

struct RaypathColorConfig {
  std::vector<RaypathColorEntry> entries_;
  // task-336.3: display-time composite mode. "dominant" (default) | "additive" |
  // "painter". Parsed into CompositeMode by BuildCompositeOptions; an unknown
  // string falls back to dominant with a one-shot warning. Absent in JSON →
  // "dominant" = the 336.1 behavior.
  std::string mode_ = "dominant";
};

// JSON representation is a bare array. Missing top-level "raypath_color" key
// in an owning ConfigManager JSON is treated as an empty list (zero
// regression against configs written before this feature existed) — handled
// by ConfigManager's from_json, not here.
void to_json(nlohmann::json& j, const RaypathColorEntry& e);
void from_json(const nlohmann::json& j, RaypathColorEntry& e);
void to_json(nlohmann::json& j, const RaypathColorConfig& c);
void from_json(const nlohmann::json& j, RaypathColorConfig& c);

}  // namespace lumice

#endif  // CONFIG_RAYPATH_COLOR_CONFIG_H_
