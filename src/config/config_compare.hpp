#ifndef CONFIG_CONFIG_COMPARE_H_
#define CONFIG_CONFIG_COMPARE_H_

#include <algorithm>

#include "config/config_manager.hpp"

namespace lumice {

// ---- Core math types ----

inline bool operator==(const Distribution& a, const Distribution& b) {
  static_assert(sizeof(Distribution) == 12, "Update operator== when Distribution fields change");
  return a.type == b.type && a.mean == b.mean && a.std == b.std;
}

inline bool operator==(const AxisDistribution& a, const AxisDistribution& b) {
  static_assert(sizeof(AxisDistribution) == 36, "Update operator== when AxisDistribution fields change");
  return a.azimuth_dist == b.azimuth_dist && a.latitude_dist == b.latitude_dist && a.roll_dist == b.roll_dist;
}

// ---- Crystal config ----

inline bool operator==(const PrismCrystalParam& a, const PrismCrystalParam& b) {
  return a.h_ == b.h_ && std::equal(std::begin(a.d_), std::end(a.d_), std::begin(b.d_));
}

inline bool operator==(const PyramidCrystalParam& a, const PyramidCrystalParam& b) {
  return a.h_prs_ == b.h_prs_ && a.h_pyr_u_ == b.h_pyr_u_ && a.h_pyr_l_ == b.h_pyr_l_ &&
         std::equal(std::begin(a.d_), std::end(a.d_), std::begin(b.d_)) && a.wedge_angle_u_ == b.wedge_angle_u_ &&
         a.wedge_angle_l_ == b.wedge_angle_l_;
}

inline bool operator==(const CrystalConfig& a, const CrystalConfig& b) {
  return a.id_ == b.id_ && a.param_ == b.param_ && a.axis_ == b.axis_;
}

// ---- Filter config ----

inline bool operator==(const NoneFilterParam&, const NoneFilterParam&) {
  return true;
}

inline bool operator==(const RaypathFilterParam& a, const RaypathFilterParam& b) {
  return a.raypath_ == b.raypath_;
}

inline bool operator==(const EntryExitFilterParam& a, const EntryExitFilterParam& b) {
  return a.entry_ == b.entry_ && a.exit_ == b.exit_;
}

inline bool operator==(const DirectionFilterParam& a, const DirectionFilterParam& b) {
  return a.lon_ == b.lon_ && a.lat_ == b.lat_ && a.radii_ == b.radii_;
}

inline bool operator==(const CrystalFilterParam& a, const CrystalFilterParam& b) {
  return a.crystal_id_ == b.crystal_id_;
}

inline bool operator==(const ComplexFilterParam& a, const ComplexFilterParam& b) {
  return a.filters_ == b.filters_;
}

inline bool operator==(const FilterConfig& a, const FilterConfig& b) {
  return a.id_ == b.id_ && a.symmetry_ == b.symmetry_ && a.action_ == b.action_ && a.param_ == b.param_;
}

// ---- Render config ----

inline bool operator==(const ViewParam& a, const ViewParam& b) {
  return a.az_ == b.az_ && a.el_ == b.el_ && a.ro_ == b.ro_;
}

inline bool operator==(const GridLineParam& a, const GridLineParam& b) {
  return a.value_ == b.value_ && a.width_ == b.width_ && a.opacity_ == b.opacity_ &&
         std::equal(std::begin(a.color_), std::end(a.color_), std::begin(b.color_));
}

inline bool operator==(const LensParam& a, const LensParam& b) {
  return a.type_ == b.type_ && a.fov_ == b.fov_;
}

inline bool operator==(const RenderConfig& a, const RenderConfig& b) {
  return a.id_ == b.id_ && a.lens_ == b.lens_ &&
         std::equal(std::begin(a.lens_shift_), std::end(a.lens_shift_), std::begin(b.lens_shift_)) &&
         std::equal(std::begin(a.resolution_), std::end(a.resolution_), std::begin(b.resolution_)) &&
         a.view_ == b.view_ && a.visible_ == b.visible_ &&
         std::equal(std::begin(a.background_), std::end(a.background_), std::begin(b.background_)) &&
         std::equal(std::begin(a.ray_color_), std::end(a.ray_color_), std::begin(b.ray_color_)) &&
         a.opacity_ == b.opacity_ && a.intensity_factor_ == b.intensity_factor_ && a.central_grid_ == b.central_grid_ &&
         a.elevation_grid_ == b.elevation_grid_ && a.celestial_outline_ == b.celestial_outline_ &&
         a.ms_filter_ == b.ms_filter_;
}

// ---- Light config ----

inline bool operator==(const SunParam& a, const SunParam& b) {
  return a.altitude_ == b.altitude_ && a.azimuth_ == b.azimuth_ && a.diameter_ == b.diameter_;
}

inline bool operator==(const WlParam& a, const WlParam& b) {
  return a.wl_ == b.wl_ && a.weight_ == b.weight_;
}

inline bool operator==(const LightSourceConfig& a, const LightSourceConfig& b) {
  return a.param_ == b.param_ && a.spectrum_ == b.spectrum_;
}

// ---- Scene config ----

inline bool operator==(const ScatteringSetting& a, const ScatteringSetting& b) {
  return a.filter_ == b.filter_ && a.crystal_ == b.crystal_ && a.crystal_proportion_ == b.crystal_proportion_;
}

inline bool operator==(const MsInfo& a, const MsInfo& b) {
  return a.prob_ == b.prob_ && a.setting_ == b.setting_;
}

inline bool operator==(const SceneConfig& a, const SceneConfig& b) {
  return a.ray_num_ == b.ray_num_ && a.max_hits_ == b.max_hits_ && a.light_source_ == b.light_source_ && a.ms_ == b.ms_;
}

// ---- ConfigManager ----

inline bool operator==(const ConfigManager& a, const ConfigManager& b) {
  return a.crystals_ == b.crystals_ && a.filters_ == b.filters_ && a.renderers_ == b.renderers_ && a.scene_ == b.scene_;
}

}  // namespace lumice

#endif  // CONFIG_CONFIG_COMPARE_H_
