#include "config/config_manager.hpp"

#include <algorithm>
#include <cstddef>
#include <variant>

#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/def.hpp"
#include "include/log.hpp"
#include "io/json_util.hpp"
#include "json.hpp"

namespace icehalo {
namespace v3 {

void to_json(nlohmann::json& j, const ConfigManager& m) {
  // Light sources
  for (const auto& [_, v] : m.lights_) {
    j["light_source"].emplace_back(v);
  }

  // Crystals
  for (const auto& [_, v] : m.crystals_) {
    j["crystal"].emplace_back(v);
  }

  // Filters
  for (const auto& [_, v] : m.filters_) {
    j["filter"].emplace_back(v);
  }

  // Renderers
  for (const auto& [_, v] : m.renderers_) {
    j["render"].emplace_back(v);
  }

  // Scenes
  for (const auto& [_, v] : m.scenes_) {
    j["scene"].emplace_back(v);
  }

  // Projects
  for (const auto& [_, v] : m.projects_) {
    j["project"].emplace_back(v);
  }
}

RenderConfig ParseRenderConfig(const nlohmann::json& j_render, const ConfigManager& m) {
  RenderConfig render{};

  j_render.at("id").get_to(render.id_);
  j_render.at("resolution").get_to(render.resolution_);

  render.lens_.type_ = LensParam::kLinear;
  render.lens_.fov_ = 90.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "lens", render.lens_)              // default {kLinear, 90.0}
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "lens_shift", render.lens_shift_)  // default [0, 0]

  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "view", render.view_)  // default {0.0, 0.0, 0.0}

  render.visible_ = RenderConfig::kUpper;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "visible", render.visible_)  // default kUpper

  std::fill(std::begin(render.background_), std::end(render.background_), 0.0f);
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "background", render.background_)  // default [0, 0, 0]

  std::fill(std::begin(render.ray_color_), std::end(render.ray_color_), -1.0f);
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "ray", render.ray_color_)  // default [-1, -1, -1]

  render.opacity_ = 1.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "opacity", render.opacity_)  // default 1

  render.intensity_factor_ = 1.0f;
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(j_render, "intensity_factor", render.intensity_factor_)  // default 1

  render.celestial_outline_ = true;
  if (j_render.contains("grid")) {
    const auto& j_grid = j_render.at("grid");
    if (j_grid.contains("central")) {
      j_grid.at("central").get_to(render.central_grid_);
    }
    if (j_grid.contains("elevation")) {
      j_grid.at("elevation").get_to(render.elevation_grid_);
    }
    if (j_grid.contains("outline")) {
      j_grid.at("outline").get_to(render.celestial_outline_);
    }
  }

  if (j_render.contains("filter")) {
    for (const auto& j_filter : j_render.at("filter")) {
      if (auto id = j_filter.get<IdType>() > 0) {
        render.ms_filter_.emplace_back(m.filters_.at(id));
      } else {
        FilterConfig none_filter;
        none_filter.id_ = kInvalidId;
        none_filter.param_ = NoneFilterParam{};
        render.ms_filter_.emplace_back(none_filter);
      }
    }
  }

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
    scene.light_source_.id_ = kInvalidId;
  }

  // scattering
  FilterConfig default_none_filter{ kInvalidId, FilterConfig::kSymNone, FilterConfig::kFilterIn, NoneFilterParam{} };
  for (const auto& j_s : j_scene.at("scattering")) {
    MsInfo ms{};
    if (j_s.contains("prob")) {
      j_s.at("prob").get_to(ms.prob_);  // default 0.0f (ms is zero-initialized)
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
        j_p.get_to(ms.setting_[i].crystal_proportion_);  // default 100.0. see above
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


void from_json(const nlohmann::json& j, ConfigManager& m) {
  // Light sources
  for (const auto& j_light : j.at("light_source")) {
    IdType id = j_light.at("id").get<IdType>();
    m.lights_.emplace(id, j_light.get<LightSourceConfig>());
  }

  // Crystals
  for (const auto& j_crystal : j.at("crystal")) {
    IdType id = j_crystal.at("id").get<IdType>();
    m.crystals_.emplace(id, j_crystal.get<CrystalConfig>());
  }

  // Filters
  for (const auto& j_filter : j.at("filter")) {
    if (j_filter.at("type") == "complex") {
      continue;
    }
    IdType id = j_filter.at("id").get<IdType>();
    m.filters_.emplace(id, j_filter.get<FilterConfig>());
  }
  for (const auto& j_filter : j.at("filter")) {
    if (j_filter.at("type") != "complex") {
      continue;
    }
    IdType id = j_filter.at("id").get<IdType>();
    auto complex_filter = j_filter.get<FilterConfig>();  // incompleted

    const auto& cmp = j_filter.at("composition");
    ComplexFilterParam p;
    for (const auto& c : cmp) {
      std::vector<std::pair<IdType, SimpleFilterParam>> f;
      for (const auto& cc : c) {
        IdType fid = cc.get<IdType>();
        const auto& p = m.filters_.at(fid).param_;
        f.emplace_back(fid, std::get<SimpleFilterParam>(p));
      }
      p.filters_.emplace_back(f);
    }
    complex_filter.param_ = p;
    m.filters_.emplace(id, complex_filter);
  }

  // Renderers
  for (const auto& j_render : j.at("render")) {
    auto renderer = ParseRenderConfig(j_render, m);
    m.renderers_.emplace(renderer.id_, renderer);
  }

  // Scenes
  for (const auto& j_scene : j.at("scene")) {
    auto scene = ParseSceneConfig(j_scene, m);
    m.scenes_.emplace(scene.id_, scene);
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

    m.projects_.emplace(proj.id_, proj);
  }
}

}  // namespace v3
}  // namespace icehalo
