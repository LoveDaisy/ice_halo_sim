#ifndef LUMICE_GUI_AXIS_PRESETS_HPP
#define LUMICE_GUI_AXIS_PRESETS_HPP

#include <cassert>
#include <cmath>
#include <cstring>

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

// Single source of truth for the default preview-camera rotation (4x4 column-major,
// same convention as CrystalRenderer::Render's rotation parameter). Both the modal
// crystal preview's Reset View and the entry-card thumbnail derive from this.
inline void DefaultPreviewRotation(AxisPreset preset, float rotation[16]) {
  constexpr float kPi = 3.14159265358979323846f;
  std::memset(rotation, 0, 16 * sizeof(float));
  rotation[0] = 1.0f;
  rotation[5] = 1.0f;
  rotation[10] = 1.0f;
  rotation[15] = 1.0f;

  auto set_rx = [&](float angle_deg) {
    float rad = angle_deg * kPi / 180.0f;
    float c = std::cos(rad);
    float s = std::sin(rad);
    rotation[5] = c;
    rotation[6] = s;
    rotation[9] = -s;
    rotation[10] = c;
  };

  switch (preset) {
    case AxisPreset::kColumn:
    case AxisPreset::kParry:
      // Side view: tilt +80 deg around X (c-axis nearly horizontal).
      set_rx(80.0f);
      return;
    case AxisPreset::kPlate:
      // Top-down view with a slight tilt to reveal hexagonal outline.
      set_rx(-10.0f);
      return;
    case AxisPreset::kLowitz:
      // Tilted side view at +60 deg around X.
      set_rx(60.0f);
      return;
    case AxisPreset::kRandom:
    case AxisPreset::kCustom:
      break;
    default:
      // -Wswitch-enum guards new enum values at compile time; this assert
      // surfaces unhandled cases at runtime if someone bypasses the warning.
      assert(false && "DefaultPreviewRotation: unhandled AxisPreset");
      break;
  }

  // Random / Custom / fallback: isometric view (Ry(+25 deg) * Rx(+35 deg)).
  constexpr float kAngleX = 35.0f * kPi / 180.0f;
  constexpr float kAngleY = 25.0f * kPi / 180.0f;
  float cx = std::cos(kAngleX);
  float sx = std::sin(kAngleX);
  float cy = std::cos(kAngleY);
  float sy = std::sin(kAngleY);
  rotation[0] = cy;
  rotation[1] = 0.0f;
  rotation[2] = -sy;
  rotation[4] = sy * sx;
  rotation[5] = cx;
  rotation[6] = cy * sx;
  rotation[8] = sy * cx;
  rotation[9] = -sx;
  rotation[10] = cy * cx;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_AXIS_PRESETS_HPP
