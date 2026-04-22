#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "gui/slider_mapping.hpp"

namespace {

using lumice::gui::slider_mapping::kLogLinearTSwitch;
using lumice::gui::slider_mapping::kLogLinearX0;
using lumice::gui::slider_mapping::LogLinearNormToValue;
using lumice::gui::slider_mapping::LogLinearValueToNorm;
using lumice::gui::slider_mapping::LogNormToValue;
using lumice::gui::slider_mapping::LogValueToNorm;

// ============================================================
// Convention 1 — Prism Height: [0.01, 100] kLog
// ============================================================

TEST(SliderMappingLog, PrismHeightBoundaryLow) {
  EXPECT_NEAR(LogValueToNorm(0.01f, 0.01f, 100.0f), 0.0f, 1e-6f);
  EXPECT_NEAR(LogNormToValue(0.0f, 0.01f, 100.0f), 0.01f, 1e-6f);
}

TEST(SliderMappingLog, PrismHeightBoundaryHigh) {
  EXPECT_NEAR(LogValueToNorm(100.0f, 0.01f, 100.0f), 1.0f, 1e-6f);
  EXPECT_NEAR(LogNormToValue(1.0f, 0.01f, 100.0f), 100.0f, 1e-4f);
}

TEST(SliderMappingLog, PrismHeightMidpointLogScale) {
  // Geometric midpoint of [0.01, 100] is 1.0 — should land at norm=0.5
  EXPECT_NEAR(LogValueToNorm(1.0f, 0.01f, 100.0f), 0.5f, 1e-6f);
  EXPECT_NEAR(LogNormToValue(0.5f, 0.01f, 100.0f), 1.0f, 1e-4f);
}

TEST(SliderMappingLog, PrismHeightRoundTrip) {
  // Sample 17 geometrically spaced points; round-trip relative error < 1e-4.
  constexpr int kSamples = 17;
  constexpr float kMinVal = 0.01f;
  constexpr float kMaxVal = 100.0f;
  for (int i = 0; i <= kSamples; ++i) {
    float t = static_cast<float>(i) / kSamples;
    float value = kMinVal * std::exp(t * std::log(kMaxVal / kMinVal));
    float norm = LogValueToNorm(value, kMinVal, kMaxVal);
    float round_trip = LogNormToValue(norm, kMinVal, kMaxVal);
    EXPECT_NEAR(round_trip, value, std::max(value, 1e-4f) * 1e-4f) << "i=" << i << " value=" << value;
  }
}

TEST(SliderMappingLog, PrismHeightMonotonic) {
  // Monotonicity: larger norm → larger value.
  constexpr float kMinVal = 0.01f;
  constexpr float kMaxVal = 100.0f;
  float prev = LogNormToValue(0.0f, kMinVal, kMaxVal);
  for (int i = 1; i <= 20; ++i) {
    float norm = static_cast<float>(i) / 20.0f;
    float value = LogNormToValue(norm, kMinVal, kMaxVal);
    EXPECT_GT(value, prev) << "i=" << i;
    prev = value;
  }
}

// ============================================================
// Convention 2 — Pyramid prism_h: [0, 100] kLogLinear
// ============================================================

TEST(SliderMappingLogLinear, PyramidPrismHZero) {
  EXPECT_FLOAT_EQ(LogLinearValueToNorm(0.0f, 100.0f), 0.0f);
  EXPECT_FLOAT_EQ(LogLinearNormToValue(0.0f, 100.0f), 0.0f);
}

TEST(SliderMappingLogLinear, PyramidPrismHMax) {
  EXPECT_NEAR(LogLinearValueToNorm(100.0f, 100.0f), 1.0f, 1e-6f);
  EXPECT_NEAR(LogLinearNormToValue(1.0f, 100.0f), 100.0f, 1e-3f);
}

TEST(SliderMappingLogLinear, PyramidPrismHC0ContinuousAtX0) {
  // C0 continuity at the linear/log switch point x0.
  float norm_at_x0 = LogLinearValueToNorm(kLogLinearX0, 100.0f);
  EXPECT_NEAR(norm_at_x0, kLogLinearTSwitch, 1e-6f);
  float value_at_switch = LogLinearNormToValue(kLogLinearTSwitch, 100.0f);
  EXPECT_NEAR(value_at_switch, kLogLinearX0, 1e-6f);
}

TEST(SliderMappingLogLinear, PyramidPrismHRoundTrip) {
  // Targeted samples including below-x0 linear region, at-x0 switch, above-x0 log region.
  constexpr float kSamples[] = { 0.0f, 0.005f, 0.01f, 0.05f, 0.2f, 1.0f, 10.0f, 50.0f, 100.0f };
  for (float v : kSamples) {
    float norm = LogLinearValueToNorm(v, 100.0f);
    float round_trip = LogLinearNormToValue(norm, 100.0f);
    EXPECT_NEAR(round_trip, v, std::max(v, 1e-3f) * 1e-3f) << "value=" << v;
  }
}

TEST(SliderMappingLogLinear, PyramidPrismHMonotonic) {
  float prev = LogLinearNormToValue(0.0f, 100.0f);
  for (int i = 1; i <= 40; ++i) {
    float norm = static_cast<float>(i) / 40.0f;
    float value = LogLinearNormToValue(norm, 100.0f);
    EXPECT_GE(value, prev) << "i=" << i;  // non-decreasing; at boundary may tie
    prev = value;
  }
}

// ============================================================
// Convention 3 — Pyramid upper_h / lower_h: [0, 1] kLinear
// ============================================================
//
// These tests validate the `std::clamp([0, 1])` contract that kLinear [0, 1]
// depends on; they do NOT call any function from slider_mapping.hpp (kLinear
// has no helper — SliderWithInput passes the raw value to ImGui::SliderFloat).
// Treat this block as an executable anchor for the convention table.
//
// The actual call-site configuration is guarded by the GUI tests
// pyramid_h_allows_zero_via_modal / pyramid_h_clamp_upper_via_modal /
// pyramid_h_linear_readback_via_modal in test/gui/test_gui_interaction.cpp.

TEST(SliderMappingLinear, UpperLowerHZero) {
  float v = std::clamp(0.0f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(v, 0.0f);
}

TEST(SliderMappingLinear, UpperLowerHOne) {
  float v = std::clamp(1.0f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(v, 1.0f);
}

TEST(SliderMappingLinear, UpperLowerHClampLow) {
  float v = std::clamp(-0.1f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(v, 0.0f);
}

TEST(SliderMappingLinear, UpperLowerHClampHigh) {
  float v = std::clamp(1.5f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(v, 1.0f);
}

TEST(SliderMappingLinear, UpperLowerHMidpointIdentity) {
  // Linear mapping: midpoint value survives round-trip unchanged.
  float v = std::clamp(0.5f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(v, 0.5f);
}

}  // namespace
