#include "config/raypath_color_config.hpp"

#include <nlohmann/json.hpp>

namespace lumice {

void to_json(nlohmann::json& j, const RaypathColorEntry& e) {
  j["layer"] = e.layer_;
  j["crystal_slot"] = e.crystal_id_;
  j["summand"] = e.summand_idx_;
  j["color"] = { e.color_[0], e.color_[1], e.color_[2] };
}

void from_json(const nlohmann::json& j, RaypathColorEntry& e) {
  j.at("layer").get_to(e.layer_);
  j.at("crystal_slot").get_to(e.crystal_id_);
  j.at("summand").get_to(e.summand_idx_);
  const auto& jc = j.at("color");
  e.color_[0] = jc.at(0).get<float>();
  e.color_[1] = jc.at(1).get<float>();
  e.color_[2] = jc.at(2).get<float>();
}

void to_json(nlohmann::json& j, const RaypathColorConfig& c) {
  j = nlohmann::json::array();
  for (const auto& e : c.entries_) {
    j.emplace_back(e);
  }
}

void from_json(const nlohmann::json& j, RaypathColorConfig& c) {
  c.entries_.clear();
  for (const auto& je : j) {
    c.entries_.emplace_back(je.get<RaypathColorEntry>());
  }
}

}  // namespace lumice
