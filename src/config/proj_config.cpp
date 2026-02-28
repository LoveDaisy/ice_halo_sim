#include "config/proj_config.hpp"

#include <nlohmann/json.hpp>

namespace lumice {

void to_json(nlohmann::json& j, const SceneConfig& s) {
  j["light_source"] = s.light_source_;
  j["ray_num"] = s.ray_num_;
  j["max_hits"] = s.max_hits_;
  for (const auto& m : s.ms_) {
    nlohmann::json j_m;
    j_m["prob"] = m.prob_;
    nlohmann::json entries = nlohmann::json::array();
    for (const auto& e : m.setting_) {
      nlohmann::json j_entry;
      j_entry["crystal"] = e.crystal_.id_;
      j_entry["proportion"] = e.crystal_proportion_;
      if (e.filter_.id_ != kInvalidId) {
        j_entry["filter"] = e.filter_.id_;
      }
      entries.emplace_back(std::move(j_entry));
    }
    j_m["entries"] = std::move(entries);
    j["scattering"].emplace_back(j_m);
  }
}


void to_json(nlohmann::json& j, const ProjConfig& p) {
  for (const auto& r : p.renderers_) {
    j["render"].emplace_back(r.id_);
  }
}

}  // namespace lumice