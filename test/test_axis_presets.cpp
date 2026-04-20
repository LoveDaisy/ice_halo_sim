#include <gtest/gtest.h>

#include "gui/axis_presets.hpp"

namespace {

using lumice::gui::AxisDist;
using lumice::gui::AxisDistType;
using lumice::gui::AxisPreset;
using lumice::gui::AxisPresetLabel;
using lumice::gui::ClassifyAxisPreset;

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

}  // namespace
