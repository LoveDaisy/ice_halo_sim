#include "protocol/proj_config.hpp"

#include "core/def.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

void to_json(nlohmann::json& j, const SceneConfig& s) {
  j["id"] = s.id_;
  j["light_source"] = s.light_source_.id_;
  j["ray_num"] = s.ray_num_;
  j["max_hits"] = s.max_hits_;
  for (const auto& m : s.ms_) {
    nlohmann::json j_m;
    j_m["prob"] = m.prob_;
    for (const auto& s : m.setting_) {
      j_m["crystal"].emplace_back(s.crystal_.id_);
      j_m["proportion"].emplace_back(s.crystal_proportion_);
      j_m["filter"].emplace_back(s.filter_.id_);
    }
    j["scattering"].emplace_back(j_m);
  }
}


void to_json(nlohmann::json& j, const ProjConfig& p) {
  j["id"] = p.id_;

  for (const auto& r : p.renderers_) {
    j["render"].emplace_back(r.id_);
  }

  j["scene"] = p.scene_.id_;
}

}  // namespace v3
}  // namespace icehalo