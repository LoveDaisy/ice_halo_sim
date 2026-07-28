#ifndef LUMICE_GUI_AXIS_PRESETS_HPP
#define LUMICE_GUI_AXIS_PRESETS_HPP

#include <cassert>
#include <cmath>
#include <cstddef>
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

// Zenith-std bounds of the classifier's tolerance domain. Shared by ClassifyAxisPreset below
// and by the user-defaults preset-override clamp (src/gui/user_defaults.cpp), so the "what
// counts as this preset" threshold and the "what may a user store" threshold cannot drift
// apart as two independent literals:
//   Column / Plate / Parry require zenith.std < kColumnPlateParryZenithStdUpperBound
//   Lowitz                 requires zenith.std > kLowitzZenithStdLowerBound
// These are ZENITH bounds only. IsRollLocked's own `std < 10` below is a separate predicate
// about the ROLL axis that merely shares the number; folding them together would couple two
// unrelated thresholds.
inline constexpr float kColumnPlateParryZenithStdUpperBound = 10.0f;
inline constexpr float kLowitzZenithStdLowerBound = 15.0f;

// AxisDistType names, in enum order, as an ImGui zero-separated combo item string. Shared by the
// live editor (RenderAxisDist, panels.cpp) and the preset library's read-only type cells so the
// two cannot come to spell a distribution differently. The static_assert is the reason this is a
// named constant and not a literal at each call site: a sixth AxisDistType has to break the build
// somewhere, and "somewhere" should be next to the string that would otherwise silently omit it.
inline constexpr const char* kAxisDistTypeComboItems = "Gauss\0Uniform\0Zigzag\0Laplacian\0Gauss (legacy)\0";
static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update kAxisDistTypeComboItems when adding AxisDistType");

// --------------------------------------------------------------------------------------------------
// The built-in preset table.
//
// Lives in the header (rather than in edit_modals.cpp, where it started) because three consumers
// now read it: the axis modal's preset buttons, the preset-library panel (defaults_panel.cpp) and
// the override store (user_defaults.cpp). It is a table of `constexpr` constants with no lifetime
// of its own, so the exposure costs nothing; an accessor function would have to name
// AxisPresetEntry in its signature anyway, exposing the same type through one more indirection.
// GUI-internal: nothing here belongs in the C API.
// --------------------------------------------------------------------------------------------------

struct AxisPresetEntry {
  const char* label;
  AxisPreset id;
  AxisDist zenith;
  AxisDist azimuth;
  AxisDist roll;
  // Whether a user may retune this preset's zenith std and have it persist. THE single source for
  // that question — the write guard (user_defaults.cpp), the "Save as <preset>" gesture and the
  // panel's editable-cell test all read this field rather than each restating the same four names.
  //
  // False for Random (defined as three uniform-360 axes: there is no narrow-distribution field to
  // widen, so an input box for it would be one that does nothing) and for Custom (not a built-in
  // identity at all — it is the classifier's "none of the above").
  bool has_adjustable_zenith_std;
  // Key this preset occupies in the override file under presets.axis.<name>. Lowercase, matching
  // the other enum name tables (kLensTypeJsonNames et al.). nullptr for the presets that store
  // nothing — kept in lockstep with has_adjustable_zenith_std by the static_assert below, so the
  // two cannot drift into disagreeing about which presets are storable.
  const char* override_json_name;
};

inline constexpr AxisDist kAzFullUniform{ AxisDistType::kUniform, 0.0f, 360.0f };
inline constexpr AxisDist kRollFreeUniform{ AxisDistType::kUniform, 0.0f, 360.0f };
inline constexpr AxisDist kRollLockedGauss{ AxisDistType::kGauss, 0.0f, 1.0f };

inline constexpr AxisPresetEntry kAxisPresets[] = {
  { "Column",
    AxisPreset::kColumn,
    { AxisDistType::kGauss, 90.0f, 1.0f },
    kAzFullUniform,
    kRollFreeUniform,
    true,
    "column" },
  { "Plate",
    AxisPreset::kPlate,
    { AxisDistType::kGauss, 0.0f, 1.0f },
    kAzFullUniform,
    kRollFreeUniform,
    true,
    "plate" },
  { "Parry",
    AxisPreset::kParry,
    { AxisDistType::kGauss, 90.0f, 1.0f },
    kAzFullUniform,
    kRollLockedGauss,
    true,
    "parry" },
  // Lowitz default zenith uses Gauss (v11 内测反馈：gauss 更符合物理直觉; classifier 仍接受 zigzag).
  { "Lowitz",
    AxisPreset::kLowitz,
    { AxisDistType::kGauss, 0.0f, 40.0f },
    kAzFullUniform,
    kRollLockedGauss,
    true,
    "lowitz" },
  { "Random", AxisPreset::kRandom, kAzFullUniform, kAzFullUniform, kRollFreeUniform, false, nullptr },
  { "Custom",
    AxisPreset::kCustom,
    { AxisDistType::kGauss, 90.0f, 20.0f },
    kAzFullUniform,
    { AxisDistType::kGauss, 0.0f, 20.0f },
    false,
    nullptr },
};

namespace axis_preset_detail {

// has_adjustable_zenith_std and override_json_name answer two different questions ("may the user
// retune it" vs "what is it called on disk"), but their populations are the same set by
// construction: a preset with nothing to adjust has nothing to store. Binding them at compile time
// means a future preset added with only one of the two filled in is a build error rather than a
// preset that is editable in the UI and silently discarded on save (or vice versa).
constexpr bool AxisPresetTableIsConsistent() {
  for (const auto& entry : kAxisPresets) {
    if (entry.has_adjustable_zenith_std != (entry.override_json_name != nullptr)) {
      return false;
    }
  }
  return true;
}

// The override store and the panel both reach an entry by casting an AxisPreset to an index, so
// position and identity have to agree. Checked here rather than left to the reader: a reordered
// enum or a table row inserted in the middle would otherwise retarget every lookup silently.
constexpr bool AxisPresetTableIsInEnumOrder() {
  for (std::size_t i = 0; i < sizeof(kAxisPresets) / sizeof(kAxisPresets[0]); ++i) {
    if (static_cast<std::size_t>(kAxisPresets[i].id) != i) {
      return false;
    }
  }
  return true;
}

}  // namespace axis_preset_detail

static_assert(axis_preset_detail::AxisPresetTableIsConsistent(),
              "kAxisPresets: has_adjustable_zenith_std must be true exactly for the entries that carry an "
              "override_json_name — the UI's editable set and the override file's key set are one population");

static_assert(sizeof(kAxisPresets) / sizeof(kAxisPresets[0]) == static_cast<std::size_t>(AxisPreset::kCustom) + 1,
              "kAxisPresets must carry one entry per AxisPreset enumerator");

static_assert(axis_preset_detail::AxisPresetTableIsInEnumOrder(),
              "kAxisPresets must be ordered so that kAxisPresets[static_cast<size_t>(p)].id == p");

// Entry for a preset, by identity rather than by a hand-written index. Total over the enum (the
// static_asserts above guarantee the cast lands on the matching row).
inline constexpr const AxisPresetEntry& AxisPresetEntryFor(AxisPreset preset) {
  return kAxisPresets[static_cast<std::size_t>(preset)];
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
  // strict: std > kLowitzZenithStdLowerBound
  using axis_preset_detail::IsLowitzZenithType;
  if (IsLowitzZenithType(zenith.type) && FloatNear(zenith.mean, 0.0f) && zenith.std > kLowitzZenithStdLowerBound &&
      roll_locked && az_full) {
    return AxisPreset::kLowitz;
  }

  // Parry: zenith gauss-like at mean=90, std<10; roll locked; azimuth full-uniform.
  // strict: std < kColumnPlateParryZenithStdUpperBound
  if (IsGaussLike(zenith.type) && FloatNear(zenith.mean, 90.0f) && zenith.std < kColumnPlateParryZenithStdUpperBound &&
      roll_locked && az_full) {
    return AxisPreset::kParry;
  }

  // Plate: zenith gauss-like at mean=0, std<10; azimuth full-uniform.
  // strict: std < kColumnPlateParryZenithStdUpperBound
  if (IsGaussLike(zenith.type) && FloatNear(zenith.mean, 0.0f) && zenith.std < kColumnPlateParryZenithStdUpperBound &&
      az_full) {
    return AxisPreset::kPlate;
  }

  // Column: zenith gauss-like at mean=90, std<10; azimuth full-uniform.
  // strict: std < kColumnPlateParryZenithStdUpperBound
  if (IsGaussLike(zenith.type) && FloatNear(zenith.mean, 90.0f) && zenith.std < kColumnPlateParryZenithStdUpperBound &&
      az_full) {
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

// Lock the enum-to-index mapping that DefaultPreviewRotation relies on. If any
// AxisPreset value is renumbered or reordered, these asserts fail at compile
// time — preventing a silent typical-chain mismatch that the runtime
// `idx < 0 || idx >= 6` guard cannot catch.
static_assert(static_cast<int>(AxisPreset::kColumn) == 0, "kPresetTypicalChain index mismatch (kColumn)");
static_assert(static_cast<int>(AxisPreset::kPlate) == 1, "kPresetTypicalChain index mismatch (kPlate)");
static_assert(static_cast<int>(AxisPreset::kParry) == 2, "kPresetTypicalChain index mismatch (kParry)");
static_assert(static_cast<int>(AxisPreset::kLowitz) == 3, "kPresetTypicalChain index mismatch (kLowitz)");
static_assert(static_cast<int>(AxisPreset::kRandom) == 4, "kPresetTypicalChain index mismatch (kRandom)");
static_assert(static_cast<int>(AxisPreset::kCustom) == 5, "kPresetTypicalChain index mismatch (kCustom)");

// Convert a core-frame 3x3 rotation (column-major 4x4 with translation=0) into
// the GUI mesh frame, accounting for the Y-Z swap performed by
// crystal_preview.cpp::BuildCrystalMeshData (core +z → mesh +y, core +y →
// mesh -z). Applies the conjugation `out = S · in · S^T` where
//   S = | 1  0  0 |   S^T = | 1  0  0 |
//       | 0  0  1 |         | 0  0 -1 |
//       | 0 -1  0 |         | 0  1  0 |
// Only the upper-left 3x3 is meaningful; translation column / row are zeroed.
inline void ApplyMeshSwapWrap(const float in[16], float out[16]) {
  std::memset(out, 0, 16 * sizeof(float));
  // Column 0 (j=0): out[i][0] derived from S · in · S^T at j=0.
  out[0] = in[0];   // (0,0)
  out[1] = in[2];   // (1,0)
  out[2] = -in[1];  // (2,0)
  // Column 1 (j=1).
  out[4] = in[8];   // (0,1)
  out[5] = in[10];  // (1,1)
  out[6] = -in[9];  // (2,1)
  // Column 2 (j=2).
  out[8] = -in[4];  // (0,2)
  out[9] = -in[6];  // (1,2)
  out[10] = in[5];  // (2,2)
  out[15] = 1.0f;
}

// Single source of truth for the default preview-camera rotation (4x4 column-major,
// same convention as CrystalRenderer::Render's model_rotation argument).
//
// **Output frame:** the rotation is meant to be applied directly to mesh-frame
// vertices, so chain-formula outputs (which `ChainRotationToMatrix` produces in
// the core/world frame) are post-processed by `ApplyMeshSwapWrap` to convert
// them into the mesh frame. The isometric sentinel was originally designed for
// the OLD pipeline (which had no view rotation), so it's already mesh-friendly
// and is written directly without the wrap.
//
// params layout (must match g_axis_buf in edit_modals.cpp):
//   params[0] = zenith dist
//   params[1] = azimuth dist
//   params[2] = roll dist
// params may be nullptr — in that case kCustom degrades to the isometric sentinel.
inline void DefaultPreviewRotation(AxisPreset preset, const AxisDist params[3], float out[16]) {
  // kCustom with live params → derive from mean values via chain formula
  // (post-wrapped to mesh frame).
  if (preset == AxisPreset::kCustom && params != nullptr) {
    float core[16];
    ChainRotationToMatrix(params[1].mean, params[0].mean, params[2].mean, core);
    ApplyMeshSwapWrap(core, out);
    return;
  }

  int idx = static_cast<int>(preset);
  if (idx < 0 || idx >= 6) {
    assert(false && "DefaultPreviewRotation: unhandled AxisPreset");
    // Fall through to sentinel below.
  } else if (!kPresetTypicalChain[idx].use_sentinel) {
    const auto& t = kPresetTypicalChain[idx];
    float core[16];
    ChainRotationToMatrix(t.az_deg, t.zenith_deg, t.roll_deg, core);
    ApplyMeshSwapWrap(core, out);
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
