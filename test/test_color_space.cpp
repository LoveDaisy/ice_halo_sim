#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "util/color_data.hpp"
#include "util/color_space.hpp"

namespace lumice {
namespace {

// ---- GamutClipXyz ----

TEST(ColorSpace, GamutClipXyzPassthrough) {
  // A color already within gamut should pass through mostly unchanged.
  // D65 white at Y=0.5 is well within gamut.
  float xyz[3] = { 0.95047f * 0.5f, 0.5f, 1.08883f * 0.5f };
  float clipped[3];
  GamutClipXyz(xyz, clipped);
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(clipped[j], xyz[j], 1e-5f);
  }
}

TEST(ColorSpace, GamutClipXyzOutOfGamut) {
  // Highly saturated color — gamut clipping should bring it closer to gray.
  float xyz[3] = { 1.5f, 0.3f, 0.0f };
  float clipped[3];
  GamutClipXyz(xyz, clipped);
  // After clipping, XyzToLinearRgb should produce values in [0,1].
  float rgb[3];
  XyzToLinearRgb(clipped, rgb);
  for (int j = 0; j < 3; j++) {
    EXPECT_GE(rgb[j], 0.0f);
    EXPECT_LE(rgb[j], 1.0f);
  }
}

TEST(ColorSpace, GamutClipXyzZero) {
  float xyz[3] = { 0.0f, 0.0f, 0.0f };
  float clipped[3];
  GamutClipXyz(xyz, clipped);
  for (int j = 0; j < 3; j++) {
    EXPECT_FLOAT_EQ(clipped[j], 0.0f);
  }
}

// ---- XyzToLinearRgb ----

TEST(ColorSpace, XyzToLinearRgbD65White) {
  // D65 white point should map to approximately (1, 1, 1) in linear RGB.
  float xyz[3] = { 0.95047f, 1.00000f, 1.08883f };
  float rgb[3];
  XyzToLinearRgb(xyz, rgb);
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(rgb[j], 1.0f, 0.01f);
  }
}

TEST(ColorSpace, XyzToLinearRgbZero) {
  float xyz[3] = { 0.0f, 0.0f, 0.0f };
  float rgb[3];
  XyzToLinearRgb(xyz, rgb);
  for (int j = 0; j < 3; j++) {
    EXPECT_FLOAT_EQ(rgb[j], 0.0f);
  }
}

TEST(ColorSpace, XyzToLinearRgbClamp) {
  // Very large XYZ should clamp to [0,1].
  float xyz[3] = { 5.0f, 5.0f, 5.0f };
  float rgb[3];
  XyzToLinearRgb(xyz, rgb);
  for (int j = 0; j < 3; j++) {
    EXPECT_GE(rgb[j], 0.0f);
    EXPECT_LE(rgb[j], 1.0f);
  }
}

// ---- LinearToSrgb ----

TEST(ColorSpace, LinearToSrgbThreshold) {
  // Below threshold: linear scaling
  float below = 0.001f;
  EXPECT_NEAR(LinearToSrgb(below), below * 12.92f, 1e-6f);

  // Above threshold: power curve
  float above = 0.5f;
  float expected = 1.055f * std::pow(above, 1.0f / 2.4f) - 0.055f;
  EXPECT_NEAR(LinearToSrgb(above), expected, 1e-6f);
}

TEST(ColorSpace, LinearToSrgbZero) {
  EXPECT_FLOAT_EQ(LinearToSrgb(0.0f), 0.0f);
}

TEST(ColorSpace, LinearToSrgbOne) {
  EXPECT_NEAR(LinearToSrgb(1.0f), 1.0f, 1e-6f);
}

TEST(ColorSpace, LinearToSrgbBatch) {
  float data[6] = { 0.0f, 0.001f, 0.1f, 0.5f, 0.8f, 1.0f };
  float expected[6];
  for (int i = 0; i < 6; i++) {
    expected[i] = LinearToSrgb(data[i]);
  }
  LinearToSrgbBatch(data, 6);
  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(data[i], expected[i], 1e-6f);
  }
}

// ---- XyzToSrgb (full pipeline) ----

TEST(ColorSpace, XyzToSrgbD65White) {
  float xyz[3] = { 0.95047f, 1.00000f, 1.08883f };
  float rgb[3];
  XyzToSrgb(xyz, rgb);
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(rgb[j], 1.0f, 0.01f);
  }
}

TEST(ColorSpace, XyzToSrgbZero) {
  float xyz[3] = { 0.0f, 0.0f, 0.0f };
  float rgb[3];
  XyzToSrgb(xyz, rgb);
  for (int j = 0; j < 3; j++) {
    EXPECT_FLOAT_EQ(rgb[j], 0.0f);
  }
}

// ---- XyzToSrgbUint8 ----

TEST(ColorSpace, XyzToSrgbUint8Basic) {
  float xyz[3] = { 0.95047f, 1.00000f, 1.08883f };  // D65 white
  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1);
  for (int j = 0; j < 3; j++) {
    EXPECT_GE(out[j], 250);  // Near 255
    EXPECT_LE(out[j], 255);
  }
}

TEST(ColorSpace, XyzToSrgbUint8Zero) {
  float xyz[3] = { 0.0f, 0.0f, 0.0f };
  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1);
  for (int j = 0; j < 3; j++) {
    EXPECT_EQ(out[j], 0);
  }
}

TEST(ColorSpace, XyzToSrgbUint8Batch) {
  // Two pixels: black + white
  float xyz[6] = { 0.0f, 0.0f, 0.0f, 0.95047f, 1.00000f, 1.08883f };
  unsigned char out[6];
  XyzToSrgbUint8(xyz, out, 2);
  // First pixel: black
  for (int j = 0; j < 3; j++) {
    EXPECT_EQ(out[j], 0);
  }
  // Second pixel: near white
  for (int j = 3; j < 6; j++) {
    EXPECT_GE(out[j], 250);
  }
}

TEST(ColorSpace, XyzToSrgbUint8ConsistentWithFloat) {
  float xyz[3] = { 0.4f, 0.3f, 0.2f };
  float rgb[3];
  XyzToSrgb(xyz, rgb);
  unsigned char expected[3];
  for (int j = 0; j < 3; j++) {
    expected[j] = static_cast<unsigned char>(std::clamp(rgb[j], 0.0f, 1.0f) * 255.0f);
  }
  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1);
  for (int j = 0; j < 3; j++) {
    EXPECT_EQ(out[j], expected[j]);
  }
}

// ---- XyzToSrgbUint8 with intensity_scale ----

TEST(ColorSpace, XyzToSrgbUint8WithScale) {
  float xyz[3] = { 0.5f, 0.5f, 0.5f };
  float scale = 2.0f;

  // Manual: scale then convert
  float scaled[3] = { xyz[0] * scale, xyz[1] * scale, xyz[2] * scale };
  unsigned char expected[3];
  XyzToSrgbUint8(scaled, expected, 1);

  // API: convert with scale parameter
  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1, scale);

  for (int j = 0; j < 3; j++) {
    EXPECT_EQ(out[j], expected[j]);
  }
}

TEST(ColorSpace, XyzToSrgbUint8ZeroScale) {
  float xyz[3] = { 0.5f, 0.5f, 0.5f };
  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1, 0.0f);
  for (int j = 0; j < 3; j++) {
    EXPECT_EQ(out[j], 0);
  }
}

}  // namespace
}  // namespace lumice
