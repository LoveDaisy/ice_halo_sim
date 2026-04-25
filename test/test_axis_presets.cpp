#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "core/simulator.hpp"
#include "gui/axis_presets.hpp"

namespace {

using lumice::gui::AxisDist;
using lumice::gui::AxisDistType;
using lumice::gui::AxisPreset;
using lumice::gui::AxisPresetLabel;
using lumice::gui::ChainRotationToMatrix;
using lumice::gui::ClassifyAxisPreset;
using lumice::gui::DefaultPreviewRotation;

constexpr AxisDist kAzFull{ AxisDistType::kUniform, 0.0f, 360.0f };
constexpr AxisDist kRollFull{ AxisDistType::kUniform, 0.0f, 360.0f };
constexpr AxisDist kRollLockedGauss{ AxisDistType::kGauss, 0.0f, 1.0f };
constexpr AxisDist kRollLockedUniform{ AxisDistType::kUniform, 0.0f, 9.0f };

// --- Column ---
TEST(AxisPresetTest, ColumnStrictDefault) {
  AxisDist z{ AxisDistType::kGauss, 90.0f, 1.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kColumn);
}
TEST(AxisPresetTest, ColumnPermissiveLaplacian) {
  AxisDist z{ AxisDistType::kLaplacian, 90.0f, 9.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kColumn);
}

// --- Plate ---
TEST(AxisPresetTest, PlateStrictDefault) {
  AxisDist z{ AxisDistType::kGauss, 0.0f, 1.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kPlate);
}
TEST(AxisPresetTest, PlatePermissiveUniform) {
  AxisDist z{ AxisDistType::kUniform, 0.0f, 9.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kPlate);
}

// --- Parry ---
TEST(AxisPresetTest, ParryStrictDefault) {
  AxisDist z{ AxisDistType::kGauss, 90.0f, 1.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollLockedGauss), AxisPreset::kParry);
}
TEST(AxisPresetTest, ParryPermissiveLaplacianRollUniform) {
  AxisDist z{ AxisDistType::kLaplacian, 90.0f, 9.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollLockedUniform), AxisPreset::kParry);
}

// --- Lowitz ---
// Preset button default: Lowitz 默认 zenith 从 v11 起改为 kGauss（0°, 40°），
// classifier 仍接受 kZigzag 以兼容老 .lmc 文件（见 LowitzStrictDefault）。
TEST(AxisPresetTest, LowitzDefaultGauss) {
  AxisDist z{ AxisDistType::kGauss, 0.0f, 40.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollLockedGauss), AxisPreset::kLowitz);
}
TEST(AxisPresetTest, LowitzStrictDefault) {
  AxisDist z{ AxisDistType::kZigzag, 0.0f, 40.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollLockedGauss), AxisPreset::kLowitz);
}
TEST(AxisPresetTest, LowitzPermissiveGaussRollUniform) {
  AxisDist z{ AxisDistType::kGauss, 0.0f, 20.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollLockedUniform), AxisPreset::kLowitz);
}

// --- Random ---
TEST(AxisPresetTest, RandomAllFullUniform) {
  AxisDist z{ AxisDistType::kUniform, 0.0f, 360.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kRandom);
}
TEST(AxisPresetTest, RandomOffsetBecomesCustom) {
  AxisDist z{ AxisDistType::kUniform, 30.0f, 360.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kCustom);
}

// --- Custom ---
TEST(AxisPresetTest, CustomButtonDefault) {
  AxisDist z{ AxisDistType::kGauss, 90.0f, 20.0f };
  AxisDist r{ AxisDistType::kGauss, 0.0f, 20.0f };
  // zenith mean=90 + std=20 (>=10) fails Column strict, no preset matches.
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, r), AxisPreset::kCustom);
}

// --- Boundary cases (strict inequality) ---
TEST(AxisPresetTest, BoundaryPlateStdEq10IsCustom) {
  // Plate requires std < 10.0f; std==10 must NOT match.
  AxisDist z{ AxisDistType::kGauss, 0.0f, 10.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kCustom);
}
TEST(AxisPresetTest, BoundaryLowitzStdEq15IsCustom) {
  // Lowitz requires std > 15.0f; std==15 must NOT match.
  AxisDist z{ AxisDistType::kGauss, 0.0f, 15.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollLockedGauss), AxisPreset::kCustom);
}
TEST(AxisPresetTest, BoundaryColumnStdEq10IsCustom) {
  AxisDist z{ AxisDistType::kGauss, 90.0f, 10.0f };
  EXPECT_EQ(ClassifyAxisPreset(z, kAzFull, kRollFull), AxisPreset::kCustom);
}

// --- Gauss / GaussLegacy equivalence ---
TEST(AxisPresetTest, GaussLegacyEquivalentToGauss) {
  AxisDist z_gauss{ AxisDistType::kGauss, 0.0f, 1.0f };
  AxisDist z_legacy{ AxisDistType::kGaussLegacy, 0.0f, 1.0f };
  EXPECT_EQ(ClassifyAxisPreset(z_gauss, kAzFull, kRollFull), AxisPreset::kPlate);
  EXPECT_EQ(ClassifyAxisPreset(z_legacy, kAzFull, kRollFull), AxisPreset::kPlate);
}

// --- Label mapping sanity ---
TEST(AxisPresetTest, LabelsAreCapitalized) {
  EXPECT_STREQ(AxisPresetLabel(AxisPreset::kColumn), "Column");
  EXPECT_STREQ(AxisPresetLabel(AxisPreset::kPlate), "Plate");
  EXPECT_STREQ(AxisPresetLabel(AxisPreset::kParry), "Parry");
  EXPECT_STREQ(AxisPresetLabel(AxisPreset::kLowitz), "Lowitz");
  EXPECT_STREQ(AxisPresetLabel(AxisPreset::kRandom), "Random");
  EXPECT_STREQ(AxisPresetLabel(AxisPreset::kCustom), "Custom");
}

// --- DefaultPreviewRotation contract ---
// Helpers below operate on a 4x4 column-major float matrix:
//   m[col*4 + row]  →  row r, column c is rotation[c*4 + r].

void ExpectHomogeneousAffine(const float r[16]) {
  for (int i : { 3, 7, 11, 12, 13, 14 }) {
    EXPECT_FLOAT_EQ(r[i], 0.0f) << "index " << i;
  }
  EXPECT_FLOAT_EQ(r[15], 1.0f);
}

float Det3(const float r[16]) {
  // 3x3 upper-left in column-major layout.
  float m00 = r[0];
  float m10 = r[1];
  float m20 = r[2];
  float m01 = r[4];
  float m11 = r[5];
  float m21 = r[6];
  float m02 = r[8];
  float m12 = r[9];
  float m22 = r[10];
  return m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
}

void ExpectOrthonormal(const float r[16]) {
  // R^T R = I for the upper-left 3x3 (column-major: column k = (r[k*4+0..2])).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float dot = 0.0f;
      for (int k = 0; k < 3; ++k) {
        dot += r[i * 4 + k] * r[j * 4 + k];
      }
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(dot, expected, 1e-5f) << "dot(col[" << i << "], col[" << j << "])";
    }
  }
}

// Multiply column-major 4x4 R by a column 3-vector (homogeneous w=1 truncated).
std::array<float, 3> Mul3(const float r[16], float x, float y, float z) {
  return {
    r[0] * x + r[4] * y + r[8] * z,
    r[1] * x + r[5] * y + r[9] * z,
    r[2] * x + r[6] * y + r[10] * z,
  };
}

TEST(DefaultPreviewRotation, AllPresetsAreOrthonormalAffine) {
  for (auto p : { AxisPreset::kColumn, AxisPreset::kPlate, AxisPreset::kParry, AxisPreset::kLowitz, AxisPreset::kRandom,
                  AxisPreset::kCustom }) {
    float r[16] = { 0 };
    DefaultPreviewRotation(p, nullptr, r);
    ExpectHomogeneousAffine(r);
    EXPECT_NEAR(Det3(r), 1.0f, 1e-5f) << "preset=" << static_cast<int>(p);
    ExpectOrthonormal(r);
  }
}

constexpr float kPi = 3.14159265358979323846f;

// --- ChainRotationToMatrix unit tests ---
// Formula: R = Rz(az_deg - 180°) · Ry(-zenith_deg) · Rz(roll_deg).

TEST(ChainRotationToMatrix, IdentityWhenAzMinus180AndZenithZero) {
  // (az=180°, zenith=0°, roll=0°) → Rz(0)·Ry(0)·Rz(0) = identity.
  float r[16] = { 0 };
  ChainRotationToMatrix(180.0f, 0.0f, 0.0f, r);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(r[j * 4 + i], expected, 1e-5f) << "row=" << i << " col=" << j;
    }
  }
}

TEST(ChainRotationToMatrix, ZenithOnlyMapsLocalZ) {
  // (az=180°, zenith=90°, roll=0°): R = Ry(-π/2). Local +z (0,0,1) → (-1, 0, 0).
  float r[16] = { 0 };
  ChainRotationToMatrix(180.0f, 90.0f, 0.0f, r);
  auto v = Mul3(r, 0.0f, 0.0f, 1.0f);
  EXPECT_NEAR(v[0], -1.0f, 1e-5f);
  EXPECT_NEAR(v[1], 0.0f, 1e-5f);
  EXPECT_NEAR(v[2], 0.0f, 1e-5f);
}

TEST(ChainRotationToMatrix, RollOnlyRotatesAroundLocalZ) {
  // (az=180°, zenith=0°, roll=90°): R = Rz(π/2). Local +x (1,0,0) → (0, 1, 0).
  float r[16] = { 0 };
  ChainRotationToMatrix(180.0f, 0.0f, 90.0f, r);
  auto v = Mul3(r, 1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(v[0], 0.0f, 1e-5f);
  EXPECT_NEAR(v[1], 1.0f, 1e-5f);
  EXPECT_NEAR(v[2], 0.0f, 1e-5f);
}

TEST(ChainRotationToMatrix, AzimuthOnlyAddsExtraSpinAroundWorldZ) {
  // (az=270°, zenith=0°, roll=0°): R = Rz(π/2). Local +x (1,0,0) → (0, 1, 0).
  float r[16] = { 0 };
  ChainRotationToMatrix(270.0f, 0.0f, 0.0f, r);
  auto v = Mul3(r, 1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(v[0], 0.0f, 1e-5f);
  EXPECT_NEAR(v[1], 1.0f, 1e-5f);
  EXPECT_NEAR(v[2], 0.0f, 1e-5f);
}

TEST(ChainRotationToMatrix, ProducesOrthonormalAffineForCompositeInputs) {
  // Composite (az=45°, zenith=30°, roll=60°).
  float r[16] = { 0 };
  ChainRotationToMatrix(45.0f, 30.0f, 60.0f, r);
  ExpectHomogeneousAffine(r);
  EXPECT_NEAR(Det3(r), 1.0f, 1e-5f);
  ExpectOrthonormal(r);
}

// Contract test: GUI ChainRotationToMatrix must agree with core BuildCrystalRotation
// for the same (azimuth, latitude, roll) inputs. Note: simulator uses latitude
// (= π/2 − zenith), and works in radians; GUI takes degrees + zenith.
TEST(ChainRotationToMatrix, MatchesCoreBuildCrystalRotation) {
  struct Case {
    float az_deg, zenith_deg, roll_deg;
  };
  Case cases_arr[] = {
    { 0.0f, 90.0f, 0.0f },      // Column-typical
    { 0.0f, 0.0f, 0.0f },       // Plate-typical
    { 0.0f, 60.0f, 0.0f },      // Lowitz-typical
    { 45.0f, 30.0f, 60.0f },    // Composite
    { 270.0f, 75.0f, 180.0f },  // Composite
  };
  constexpr float kDeg2Rad = kPi / 180.0f;
  for (const auto& c : cases_arr) {
    float gui_r[16] = { 0 };
    ChainRotationToMatrix(c.az_deg, c.zenith_deg, c.roll_deg, gui_r);

    float az_rad = c.az_deg * kDeg2Rad;
    float lat_rad = kPi / 2.0f - c.zenith_deg * kDeg2Rad;
    float roll_rad = c.roll_deg * kDeg2Rad;
    auto core_rot = lumice::BuildCrystalRotation(az_rad, lat_rad, roll_rad);
    // Apply core rotation to the three basis vectors and compare with GUI columns.
    for (int basis = 0; basis < 3; ++basis) {
      float p[3] = { 0.0f, 0.0f, 0.0f };
      p[basis] = 1.0f;
      core_rot.Apply(p);
      EXPECT_NEAR(gui_r[basis * 4 + 0], p[0], 1e-4f)
          << "az=" << c.az_deg << " zen=" << c.zenith_deg << " roll=" << c.roll_deg << " basis=" << basis << " axis=x";
      EXPECT_NEAR(gui_r[basis * 4 + 1], p[1], 1e-4f)
          << "az=" << c.az_deg << " zen=" << c.zenith_deg << " roll=" << c.roll_deg << " basis=" << basis << " axis=y";
      EXPECT_NEAR(gui_r[basis * 4 + 2], p[2], 1e-4f)
          << "az=" << c.az_deg << " zen=" << c.zenith_deg << " roll=" << c.roll_deg << " basis=" << basis << " axis=z";
    }
  }
}

// --- DefaultPreviewRotation chain-based contracts ---

TEST(DefaultPreviewRotation, ColumnEqualsChainAt90) {
  // kColumn typical: (az=0°, zenith=90°, roll=0°) → R = Rz(-π) · Ry(-π/2).
  // Local +z (c-axis) should be horizontal (z-component ≈ 0).
  float r[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kColumn, nullptr, r);
  auto v = Mul3(r, 0.0f, 0.0f, 1.0f);
  EXPECT_NEAR(v[2], 0.0f, 1e-5f) << "Column c-axis must lie in horizontal plane";
}

TEST(DefaultPreviewRotation, PlateKeepsCAxisVertical) {
  // kPlate typical: (az=0°, zenith=0°, roll=0°) → R = Rz(-π).
  // Local +z stays at world +z (vertical).
  float r[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kPlate, nullptr, r);
  auto v = Mul3(r, 0.0f, 0.0f, 1.0f);
  EXPECT_NEAR(v[2], 1.0f, 1e-5f) << "Plate c-axis must remain vertical";
}

TEST(DefaultPreviewRotation, ParrySharesColumnDefault) {
  // kColumn and kParry intentionally share the same typical chain parameters
  // (zenith=90°, az=0°, roll=0°) — see kPresetTypicalChain comment.
  float r_col[16] = { 0 };
  float r_par[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kColumn, nullptr, r_col);
  DefaultPreviewRotation(AxisPreset::kParry, nullptr, r_par);
  for (int i = 0; i < 16; ++i) {
    EXPECT_FLOAT_EQ(r_col[i], r_par[i]) << "index " << i;
  }
}

TEST(DefaultPreviewRotation, LowitzCAxisAt60Degrees) {
  // kLowitz typical: (az=0°, zenith=60°, roll=0°). Local +z → tilted by 60° from vertical.
  // After R = Rz(-π) · Ry(-60°): (0,0,1) → (sin(60°), 0, cos(60°)).
  float r[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kLowitz, nullptr, r);
  auto v = Mul3(r, 0.0f, 0.0f, 1.0f);
  float rad = 60.0f * kPi / 180.0f;
  EXPECT_NEAR(v[0], std::sin(rad), 1e-5f);
  EXPECT_NEAR(v[1], 0.0f, 1e-5f);
  EXPECT_NEAR(v[2], std::cos(rad), 1e-5f);
}

// kRandom + nullptr-params and kCustom + nullptr-params both fall back to the
// isometric sentinel — verify they produce identical output (single source).
TEST(DefaultPreviewRotation, RandomEqualsCustomIsometricForNullParams) {
  float r_rand[16] = { 0 };
  float r_cust[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kRandom, nullptr, r_rand);
  DefaultPreviewRotation(AxisPreset::kCustom, nullptr, r_cust);
  for (int i = 0; i < 16; ++i) {
    EXPECT_FLOAT_EQ(r_rand[i], r_cust[i]) << "index " << i;
  }
}

TEST(DefaultPreviewRotation, IsometricSentinelMapsZUnitWithXComponent) {
  // Sentinel: Ry(+25°) · Rx(+35°) applied to (0,0,1) =
  //   (sin(25°)·cos(35°), -sin(35°), cos(25°)·cos(35°)).
  float r[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kRandom, nullptr, r);
  auto v = Mul3(r, 0.0f, 0.0f, 1.0f);
  float rad_x = 35.0f * kPi / 180.0f;
  float rad_y = 25.0f * kPi / 180.0f;
  EXPECT_NEAR(v[0], std::sin(rad_y) * std::cos(rad_x), 1e-5f);
  EXPECT_NEAR(v[1], -std::sin(rad_x), 1e-5f);
  EXPECT_NEAR(v[2], std::cos(rad_y) * std::cos(rad_x), 1e-5f);
  EXPECT_GT(std::fabs(v[0]), 0.1f);
}

// --- kCustom with live params: must match chain-derived output ---

TEST(DefaultPreviewRotation, CustomWithMeansEquivalentToChain) {
  // Construct AxisDist with mean=(zenith=90°, azimuth=0°, roll=0°) — the kColumn
  // typical. Output must match kColumn (chain-based).
  AxisDist params[3] = {
    { AxisDistType::kGauss, 90.0f, 1.0f },     // zenith
    { AxisDistType::kUniform, 0.0f, 360.0f },  // azimuth
    { AxisDistType::kUniform, 0.0f, 360.0f },  // roll
  };
  float r_custom[16] = { 0 };
  float r_column[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kCustom, params, r_custom);
  DefaultPreviewRotation(AxisPreset::kColumn, nullptr, r_column);
  for (int i = 0; i < 16; ++i) {
    EXPECT_FLOAT_EQ(r_custom[i], r_column[i]) << "index " << i;
  }
}

TEST(DefaultPreviewRotation, CustomWithCompositeMeansMatchesChainFormula) {
  // mean=(zenith=30°, azimuth=45°, roll=60°). Output must equal ChainRotationToMatrix(45,30,60,*).
  AxisDist params[3] = {
    { AxisDistType::kGauss, 30.0f, 1.0f },
    { AxisDistType::kUniform, 45.0f, 1.0f },
    { AxisDistType::kGauss, 60.0f, 1.0f },
  };
  float r_custom[16] = { 0 };
  float r_chain[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kCustom, params, r_custom);
  ChainRotationToMatrix(45.0f, 30.0f, 60.0f, r_chain);
  for (int i = 0; i < 16; ++i) {
    EXPECT_NEAR(r_custom[i], r_chain[i], 1e-5f) << "index " << i;
  }
}

// Modal/thumbnail same-source contract: both code paths build the same params[3]
// from the same (zenith, azimuth, roll) AxisDist trio and call the same function.
// This test simulates the thumbnail_cache.cpp construction order
// (params[0]=zenith, [1]=azimuth, [2]=roll) and verifies it matches the chain
// formula directly — covering the field-order correctness of the construction
// site (per review-03 Suggestion 2).
TEST(DefaultPreviewRotation, ThumbnailFieldOrderConstructionContract) {
  AxisDist zenith_dist{ AxisDistType::kGauss, 25.0f, 1.0f };
  AxisDist az_dist{ AxisDistType::kUniform, 100.0f, 1.0f };
  AxisDist roll_dist{ AxisDistType::kGauss, 50.0f, 1.0f };
  // Mirror thumbnail_cache.cpp construction: params[0]=zenith, [1]=azimuth, [2]=roll.
  AxisDist params[3] = { zenith_dist, az_dist, roll_dist };
  float r_via_params[16] = { 0 };
  float r_via_chain[16] = { 0 };
  DefaultPreviewRotation(AxisPreset::kCustom, params, r_via_params);
  ChainRotationToMatrix(az_dist.mean, zenith_dist.mean, roll_dist.mean, r_via_chain);
  for (int i = 0; i < 16; ++i) {
    EXPECT_NEAR(r_via_params[i], r_via_chain[i], 1e-5f) << "index " << i;
  }
}

}  // namespace
