#include "protocol/config_manager.hpp"

#include <cstddef>
#include <utility>

#include "core/core_def.hpp"
#include "io/json_util.hpp"
#include "json.hpp"
#include "protocol/filter_config.hpp"
#include "protocol/light_config.hpp"
#include "protocol/render_config.hpp"
#include "util/log.hpp"

namespace icehalo {
namespace v3 {

RenderConfig ParseRenderConfig(const nlohmann::json& j_render, const ConfigManager& m) {
  RenderConfig render{};

  // id
  j_render.at("id").get_to(render.id_);

  // TODO:

  return render;
}

SceneConfig ParseSceneConfig(const nlohmann::json& j_scene, const ConfigManager& m) {
  SceneConfig scene{};
  // id
  j_scene.at("id").get_to(scene.id_);

  // ray_num
  if (int n = j_scene.at("ray_num").get<int>(); n < 0) {
    scene.ray_num_ = kInfSize;
  } else {
    scene.ray_num_ = static_cast<size_t>(n);
  }

  // max_hits
  j_scene.at("max_hits").get_to(scene.max_hits_);

  // light_source
  if (IdType id = j_scene.at("light_source").get<IdType>(); m.lights_.count(id)) {
    scene.light_source_ = m.lights_.at(id);
  } else {
    LOG_ERROR("Light source ID(%u) cannot be found!", id);
    scene.light_source_.id = kInvalidId;
  }

  // scattering
  FilterConfig default_none_filter{ kInvalidId, FilterConfig::kSymNone, FilterConfig::kFilterIn, NoneFilterParam{} };
  for (const auto& j_s : j_scene.at("scattering")) {
    MsInfo ms{};
    if (j_s.contains("prob")) {
      j_s.at("prob").get_to(ms.prob_);
    }
    for (const auto& j_c : j_s.at("crystal")) {
      IdType id = j_c.get<IdType>();
      ScatteringSetting s{ default_none_filter, m.crystals_.at(id), 100.0 };
      ms.setting_.emplace_back(s);
    }
    if (j_s.contains("proportion")) {
      size_t i = 0;
      for (const auto& j_p : j_s.at("proportion")) {
        if (i >= ms.setting_.size()) {
          break;
        }
        j_p.get_to(ms.setting_[i].crystal_proportion_);
        i++;
      }
    }
    if (j_s.contains("filter")) {
      size_t i = 0;
      for (const auto& j_f : j_s.at("filter")) {
        if (i >= ms.setting_.size()) {
          break;
        }
        IdType id = j_f.get<IdType>();
        ms.setting_[i].filter_ = m.filters_.at(id);
        i++;
      }
    }
    scene.ms_.emplace_back(ms);
  }

  return scene;
}


void to_json(nlohmann::json& j, const ConfigManager& m) {
  ;
}

void from_json(const nlohmann::json& j, ConfigManager& m) {
  // Light source
  for (const auto& j_light : j.at("light_source")) {
    IdType id = j_light.at("id").get<IdType>();
    m.lights_.emplace(std::make_pair(id, j_light.get<LightSourceConfig>()));
  }

  // Crystals
  for (const auto& j_crystal : j.at("crystal")) {
    IdType id = j_crystal.at("id").get<IdType>();
    m.crystals_.emplace(std::make_pair(id, j_crystal.get<CrystalConfig>()));
  }

  // Filters
  for (const auto& j_filter : j.at("filter")) {
    IdType id = j_filter.at("id").get<IdType>();
    m.filters_.emplace(std::make_pair(id, j_filter.get<FilterConfig>()));
  }

  // Renderers
  for (const auto& j_render : j.at("render")) {
    auto renderer = ParseRenderConfig(j_render, m);
    m.renderers_.emplace(std::make_pair(renderer.id_, renderer));
  }

  // Scenes
  for (const auto& j_scene : j.at("scene")) {
    auto scene = ParseSceneConfig(j_scene, m);
    m.scenes_.emplace(std::make_pair(scene.id_, scene));
  }

  // Projects
  for (const auto& j_proj : j.at("project")) {
    ProjConfig proj{};

    // id
    j_proj.at("id").get_to(proj.id_);

    // scene
    IdType scene_id = j_proj.at("scene").get<IdType>();
    proj.scene_ = m.scenes_.at(scene_id);

    // renderer
    if (j_proj.contains("render")) {
      for (const auto& j_render : j_proj.at("render")) {
        IdType render_id = j_render.get<IdType>();
        proj.renderers_.emplace_back(m.renderers_.at(render_id));
      }
    }
  }
}

}  // namespace v3
}  // namespace icehalo
