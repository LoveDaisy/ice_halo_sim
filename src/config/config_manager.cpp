#include "config/config_manager.hpp"

#include <cstddef>
#include <nlohmann/json.hpp>

#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/def.hpp"
#include "util/logger.hpp"

namespace lumice {

void to_json(nlohmann::json& j, const ConfigManager& m) {
  // Crystals
  for (const auto& [_, v] : m.crystals_) {
    j["crystal"].emplace_back(v);
  }

  // Filters
  for (const auto& [_, v] : m.filters_) {
    j["filter"].emplace_back(v);
  }

  // Scene (single object, light_source inlined)
  j["scene"] = m.scene_;

  // Renderers
  for (const auto& [_, v] : m.renderers_) {
    j["render"].emplace_back(v);
  }
}

RenderConfig ParseRenderConfig(const nlohmann::json& j_render, const ConfigManager& m) {
  RenderConfig render{};

  j_render.at("id").get_to(render.id_);
  j_render.at("resolution").get_to(render.resolution_);

  if (j_render.contains("lens")) {
    j_render.at("lens").get_to(render.lens_);
  }
  if (j_render.contains("lens_shift")) {
    j_render.at("lens_shift").get_to(render.lens_shift_);
  }
  if (j_render.contains("view")) {
    j_render.at("view").get_to(render.view_);
  }
  if (j_render.contains("visible")) {
    j_render.at("visible").get_to(render.visible_);
  }
  if (j_render.contains("background")) {
    j_render.at("background").get_to(render.background_);
  }
  if (j_render.contains("ray_color")) {
    j_render.at("ray_color").get_to(render.ray_color_);
  }
  if (j_render.contains("opacity")) {
    j_render.at("opacity").get_to(render.opacity_);
  }
  if (j_render.contains("intensity_factor")) {
    j_render.at("intensity_factor").get_to(render.intensity_factor_);
  }

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
      if (auto id = j_filter.get<int>(); id >= 0) {
        render.ms_filter_.emplace_back(m.filters_.at(static_cast<IdType>(id)));
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

static MsInfo ParseScatteringInfo(const nlohmann::json& j_s, const ConfigManager& m) {
  static const FilterConfig kDefaultNoneFilter{ kInvalidId, FilterConfig::kSymNone, FilterConfig::kFilterIn,
                                                NoneFilterParam{} };
  MsInfo ms{};
  if (j_s.contains("prob")) {
    j_s.at("prob").get_to(ms.prob_);
  }

  for (const auto& j_entry : j_s.at("entries")) {
    IdType crystal_id = j_entry.at("crystal").get<IdType>();
    ScatteringSetting setting{ kDefaultNoneFilter, m.crystals_.at(crystal_id), 100.0f };

    if (j_entry.contains("proportion")) {
      j_entry.at("proportion").get_to(setting.crystal_proportion_);
    }
    if (j_entry.contains("filter")) {
      IdType filter_id = j_entry.at("filter").get<IdType>();
      setting.filter_ = m.filters_.at(filter_id);
    }

    ms.setting_.emplace_back(std::move(setting));
  }

  return ms;
}

SceneConfig ParseSceneConfig(const nlohmann::json& j_scene, const ConfigManager& m) {
  SceneConfig scene{};

  const auto& j_ray_num = j_scene.at("ray_num");
  if (j_ray_num.is_string() && j_ray_num.get<std::string>() == "infinite") {
    scene.ray_num_ = kInfSize;
  } else {
    scene.ray_num_ = j_ray_num.get<size_t>();
  }

  j_scene.at("max_hits").get_to(scene.max_hits_);

  scene.light_source_ = j_scene.at("light_source").get<LightSourceConfig>();

  for (const auto& j_s : j_scene.at("scattering")) {
    scene.ms_.emplace_back(ParseScatteringInfo(j_s, m));
  }

  return scene;
}


void from_json(const nlohmann::json& j, ConfigManager& m) {
  // Crystals
  for (const auto& j_crystal : j.at("crystal")) {
    IdType id = j_crystal.at("id").get<IdType>();
    m.crystals_.emplace(id, j_crystal.get<CrystalConfig>());
  }

  // Filters (two passes: simple first, then complex)
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
    auto complex_filter = j_filter.get<FilterConfig>();

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

  // Scene (single object, light_source inlined)
  m.scene_ = ParseSceneConfig(j.at("scene"), m);
}

}  // namespace lumice
