#ifndef LUMICE_GUI_AXIS_PRESETS_HPP
#define LUMICE_GUI_AXIS_PRESETS_HPP

#include <cmath>

#include "gui/gui_state.hpp"

namespace lumice::gui {

enum class AxisPreset { kColumn, kPlate, kParry, kLowitz, kRandom, kCustom };

inline const char* AxisPresetLabel(AxisPreset p) {
  switch (p) {
    case AxisPreset::kColumn:
      return "Column";
    case AxisPreset::kPlate:
      return "Plate";
    case AxisPreset::kParry:
      return "Parry";
    case AxisPreset::kLowitz:
      return "Lowitz";
    case AxisPreset::kRandom:
      return "Random";
    case AxisPreset::kCustom:
      return "Custom";
  }
  return "Custom";
}

namespace axis_preset_detail {

// Angular tolerance (degrees) for equality on mean / range=360 comparisons.
constexpr float kEpsilon = 1.0f;

inline bool FloatNear(float a, float b) {
  return std::fabs(a - b) <= kEpsilon;
}

inline bool IsGaussType(AxisDistType t) {
  return t == AxisDistType::kGauss || t == AxisDistType::kGaussLegacy;
}

// Gauss-like: accepted zenith types for column/plate/parry (issue §识别规则).
// Gauss + gauss_legacy + laplacian + uniform.
inline bool IsGaussLike(AxisDistType t) {
  return t == AxisDistType::kGauss || t == AxisDistType::kGaussLegacy || t == AxisDistType::kLaplacian ||
         t == AxisDistType::kUniform;
}

// Lowitz zenith types: gauss-like set plus zigzag (issue default is [zigzag]).
inline bool IsLowitzZenithType(AxisDistType t) {
  return IsGaussLike(t) || t == AxisDistType::kZigzag;
}

// Full-uniform 360°: type=kUniform, mean≈0, std≈360 (±kEpsilon). Issue §识别规则
// requires mean=0 for the Random preset; encoding confirmed from src/gui/panels.cpp:144-148.
inline bool IsFullUniform360(const AxisDist& d) {
  return d.type == AxisDistType::kUniform && FloatNear(d.mean, 0.0f) && FloatNear(d.std, 360.0f);
}

// Roll locked: narrow-distribution roll centered at 0 (mean≈0, std<10).
// Accepts gauss / gauss_legacy / laplacian / uniform per issue §识别规则.
inline bool IsRollLocked(const AxisDist& d) {
  bool ok_type = d.type == AxisDistType::kGauss || d.type == AxisDistType::kGaussLegacy ||
                 d.type == AxisDistType::kLaplacian || d.type == AxisDistType::kUniform;
  // strict: std < 10.0f  (issue §识别规则 requires strict inequality)
  return ok_type && FloatNear(d.mean, 0.0f) && d.std < 10.0f;
}

}  // namespace axis_preset_detail

// Classify an AxisDist triple (zenith, azimuth, roll) into the best-matching
// preset. Order of checks is strict → permissive: Lowitz → Parry → Plate →
// Column → Random → Custom.
inline AxisPreset ClassifyAxisPreset(const AxisDist& zenith, const AxisDist& azimuth, const AxisDist& roll) {
  using axis_preset_detail::FloatNear;
  using axis_preset_detail::IsFullUniform360;
  using axis_preset_detail::IsGaussLike;
  using axis_preset_detail::IsRollLocked;

  bool az_full = IsFullUniform360(azimuth);
  bool roll_locked = IsRollLocked(roll);

  // Lowitz: zenith is any of {zigzag, uniform, gauss, gauss_legacy, laplacian} at
  // mean=0, std>15; roll locked; azimuth full-uniform.
  // strict: std > 15.0f
  using axis_preset_detail::IsLowitzZenithType;
  if (IsLowitzZenithType(zenith.type) && FloatNear(zenith.mean, 0.0f) && zenith.std > 15.0f && roll_locked && az_full) {
    return AxisPreset::kLowitz;
  }

  // Parry: zenith gauss-like at mean=90, std<10; roll locked; azimuth full-uniform.
  // strict: std < 10.0f
  if (IsGaussLike(zenith.type) && FloatNear(zenith.mean, 90.0f) && zenith.std < 10.0f && roll_locked && az_full) {
    return AxisPreset::kParry;
  }

  // Plate: zenith gauss-like at mean=0, std<10; azimuth full-uniform.
  // strict: std < 10.0f
  if (IsGaussLike(zenith.type) && FloatNear(zenith.mean, 0.0f) && zenith.std < 10.0f && az_full) {
    return AxisPreset::kPlate;
  }

  // Column: zenith gauss-like at mean=90, std<10; azimuth full-uniform.
  // strict: std < 10.0f
  if (IsGaussLike(zenith.type) && FloatNear(zenith.mean, 90.0f) && zenith.std < 10.0f && az_full) {
    return AxisPreset::kColumn;
  }

  // Random: all three axes uniform with full 360 range.
  if (IsFullUniform360(zenith) && az_full && IsFullUniform360(roll)) {
    return AxisPreset::kRandom;
  }

  return AxisPreset::kCustom;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_AXIS_PRESETS_HPP
