#include "config/raypath_color_config.hpp"

#include <nlohmann/json.hpp>

namespace lumice {

void to_json(nlohmann::json& j, const RaypathColorEntry& e) {
  j["layer"] = e.layer_;
  j["crystal_slot"] = e.crystal_id_;
  j["summand"] = e.summand_idx_;
  j["color"] = { e.color_[0], e.color_[1], e.color_[2] };
  // task-336.3: emit visibility only when non-default so pre-336 configs (and
  // configs that never touch visibility) stay byte-compatible.
  if (!e.visible_) {
    j["visible"] = e.visible_;
  }
  if (e.solo_) {
    j["solo"] = e.solo_;
  }
}

void from_json(const nlohmann::json& j, RaypathColorEntry& e) {
  j.at("layer").get_to(e.layer_);
  j.at("crystal_slot").get_to(e.crystal_id_);
  j.at("summand").get_to(e.summand_idx_);
  const auto& jc = j.at("color");
  e.color_[0] = jc.at(0).get<float>();
  e.color_[1] = jc.at(1).get<float>();
  e.color_[2] = jc.at(2).get<float>();
  // task-336.3: absent → defaults (visible, not solo) = 336.1 behavior.
  e.visible_ = j.value("visible", true);
  e.solo_ = j.value("solo", false);
}

// JSON form is a bare array of entries when `mode_` is the default "dominant"
// (preserves the 336.1 wire format byte-for-byte); it becomes an object
// { "mode": ..., "entries": [...] } only once a non-default mode is set.
// from_json accepts both forms so any config written by either version parses.
void to_json(nlohmann::json& j, const RaypathColorConfig& c) {
  if (c.mode_ == "dominant") {
    j = nlohmann::json::array();
    for (const auto& e : c.entries_) {
      j.emplace_back(e);
    }
    return;
  }
  j = nlohmann::json::object();
  j["mode"] = c.mode_;
  j["entries"] = nlohmann::json::array();
  for (const auto& e : c.entries_) {
    j["entries"].emplace_back(e);
  }
}

void from_json(const nlohmann::json& j, RaypathColorConfig& c) {
  c.entries_.clear();
  c.mode_ = "dominant";
  if (j.is_object()) {
    c.mode_ = j.value("mode", std::string("dominant"));
    if (j.contains("entries")) {
      for (const auto& je : j.at("entries")) {
        c.entries_.emplace_back(je.get<RaypathColorEntry>());
      }
    }
    return;
  }
  // Bare-array (336.1) form: entries only, default mode.
  for (const auto& je : j) {
    c.entries_.emplace_back(je.get<RaypathColorEntry>());
  }
}

}  // namespace lumice
