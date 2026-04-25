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

// Internal helper: only called by DefaultPreviewRotation. If reused elsewhere,
// relocate to src/util/. Builds the simulator-equivalent chain rotation
//   R = Rz(az_deg - 180°) · Ry(-zenith_deg) · Rz(roll_deg)
// in column-major 4x4. Mirrors src/core/simulator.cpp::BuildCrystalRotation;
// kept as a separate GUI implementation to avoid GUI → core reverse dependency
// (contract test in test_axis_presets.cpp guards equivalence).
inline void ChainRotationToMatrix(float az_deg, float zenith_deg, float roll_deg, float out[16]) {
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kDeg2Rad = kPi / 180.0f;
  float az = az_deg * kDeg2Rad - kPi;  // azimuth - 180°
  float zen = -zenith_deg * kDeg2Rad;  // -zenith
  float roll = roll_deg * kDeg2Rad;

  auto fill_rz = [](float angle, float m[16]) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    std::memset(m, 0, 16 * sizeof(float));
    m[0] = c;
    m[1] = s;
    m[4] = -s;
    m[5] = c;
    m[10] = 1.0f;
    m[15] = 1.0f;
  };
  auto fill_ry = [](float angle, float m[16]) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    std::memset(m, 0, 16 * sizeof(float));
    m[0] = c;
    m[2] = -s;
    m[5] = 1.0f;
    m[8] = s;
    m[10] = c;
    m[15] = 1.0f;
  };
  auto mul4 = [](const float a[16], const float b[16], float r[16]) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        float s = 0.0f;
        for (int k = 0; k < 4; ++k) {
          s += a[i + k * 4] * b[k + j * 4];
        }
        r[i + j * 4] = s;
      }
    }
  };

  float m_az[16];
  float m_zen[16];
  float m_roll[16];
  float tmp[16];
  fill_rz(az, m_az);
  fill_ry(zen, m_zen);
  fill_rz(roll, m_roll);
  mul4(m_zen, m_roll, tmp);  // tmp = Ry(-zenith) · Rz(roll)
  mul4(m_az, tmp, out);      // out = Rz(az-π) · tmp
}

// Per-preset typical chain parameters used to derive the modal preview's
// default orientation. kColumn and kParry intentionally share (zenith=90°,
// az=0°, roll=0°): their default views are visually identical here, and
// classification (ClassifyAxisPreset) distinguishes them via AxisDist.type
// (roll free uniform vs. locked gauss), not via the mean values stored here.
struct PresetTypicalChain {
  float az_deg;
  float zenith_deg;
  float roll_deg;
  bool use_sentinel;  // true → fall back to isometric Ry·Rx (Random / Custom-with-null-params)
};

inline constexpr PresetTypicalChain kPresetTypicalChain[6] = {
  { 0.0f, 90.0f, 0.0f, false },  // kColumn
  { 0.0f, 0.0f, 0.0f, false },   // kPlate
  { 0.0f, 90.0f, 0.0f, false },  // kParry — same as kColumn (intentional, see comment above)
  { 0.0f, 60.0f, 0.0f, false },  // kLowitz
  { 0.0f, 0.0f, 0.0f, true },    // kRandom — isometric sentinel
  { 0.0f, 0.0f, 0.0f, true },    // kCustom — sentinel when params == nullptr
};

// Single source of truth for the default preview-camera rotation (4x4 column-major,
// same convention as CrystalRenderer::Render's model_rotation argument). Both the
// modal crystal preview's Reset View / preset-button and the entry-card thumbnail
// derive from this. For kCustom with non-null params, the rotation is derived from
// the user's current axis-distribution mean values (zenith / azimuth / roll). All
// other cases fall back to a per-preset typical chain or an isometric sentinel.
//
// params layout (must match g_axis_buf in edit_modals.cpp):
//   params[0] = zenith dist
//   params[1] = azimuth dist
//   params[2] = roll dist
// params may be nullptr — in that case kCustom degrades to the isometric sentinel.
inline void DefaultPreviewRotation(AxisPreset preset, const AxisDist params[3], float out[16]) {
  // kCustom with live params → derive from mean values via chain formula.
  if (preset == AxisPreset::kCustom && params != nullptr) {
    ChainRotationToMatrix(params[1].mean, params[0].mean, params[2].mean, out);
    return;
  }

  int idx = static_cast<int>(preset);
  if (idx < 0 || idx >= 6) {
    assert(false && "DefaultPreviewRotation: unhandled AxisPreset");
    // Fall through to sentinel below.
  } else if (!kPresetTypicalChain[idx].use_sentinel) {
    const auto& t = kPresetTypicalChain[idx];
    ChainRotationToMatrix(t.az_deg, t.zenith_deg, t.roll_deg, out);
    return;
  }

  // Random / Custom-with-null-params / fallback: isometric view (Ry(+25°) · Rx(+35°)).
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kAngleX = 35.0f * kPi / 180.0f;
  constexpr float kAngleY = 25.0f * kPi / 180.0f;
  float cx = std::cos(kAngleX);
  float sx = std::sin(kAngleX);
  float cy = std::cos(kAngleY);
  float sy = std::sin(kAngleY);
  std::memset(out, 0, 16 * sizeof(float));
  out[0] = cy;
  out[1] = 0.0f;
  out[2] = -sy;
  out[4] = sy * sx;
  out[5] = cx;
  out[6] = cy * sx;
  out[8] = sy * cx;
  out[9] = -sx;
  out[10] = cy * cx;
  out[15] = 1.0f;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_AXIS_PRESETS_HPP
