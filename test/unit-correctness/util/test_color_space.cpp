#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

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

// ---- SrgbToLinear ----

TEST(ColorSpace, SrgbToLinearThreshold) {
  // Below threshold: linear scaling
  float below = 0.02f;
  EXPECT_NEAR(SrgbToLinear(below), below / 12.92f, 1e-6f);

  // Above threshold: power curve
  float above = 0.5f;
  float expected = std::pow((above + 0.055f) / 1.055f, 2.4f);
  EXPECT_NEAR(SrgbToLinear(above), expected, 1e-6f);
}

TEST(ColorSpace, SrgbToLinearZero) {
  EXPECT_FLOAT_EQ(SrgbToLinear(0.0f), 0.0f);
}

TEST(ColorSpace, SrgbToLinearOne) {
  EXPECT_NEAR(SrgbToLinear(1.0f), 1.0f, 1e-6f);
}

// The two functions split their domain at the SAME point on the curve, expressed once in each
// space (0.0031308 linear ↔ 0.04045 sRGB). If either branch were mismatched the piecewise
// definition would step at the seam, and the round trip below would only hide it away from the
// seam — so the seam itself is asserted directly.
TEST(ColorSpace, GammaCurvesAreContinuousAtTheirThresholds) {
  // Evaluated as the two BRANCH formulas at the seam rather than as samples either side of it:
  // the linear segment has slope 12.92, so an epsilon step in linear is a 12.92x larger step in
  // sRGB and a sampled comparison would measure the slope, not a discontinuity.
  EXPECT_NEAR(0.0031308f * 12.92f, 1.055f * std::pow(0.0031308f, 1.0f / 2.4f) - 0.055f, 1e-5f);
  EXPECT_NEAR(0.04045f / 12.92f, std::pow((0.04045f + 0.055f) / 1.055f, 2.4f), 1e-5f);
  // ... and the seam is the same point seen from both spaces.
  EXPECT_NEAR(LinearToSrgb(0.0031308f), 0.04045f, 1e-5f);
  EXPECT_NEAR(SrgbToLinear(0.04045f), 0.0031308f, 1e-5f);
}

// ---- The bit-exact round trip ----
//
// This is the property the JSON boundary actually rests on, and the one the EXPECT_NEAR test
// below could not see. A config's "background" is decoded to linear on the way in and re-encoded
// on the way out, so a save/reload applies decode -> encode -> decode where a single decode is the
// truth; RenderConfig::operator== then compares the two linear floats EXACTLY. A tolerance of
// 1e-6f passes a one-ULP drift, which is how a real defect sat under a green test until two new
// corpus configs happened to carry background 0.05 and CI went red on Ubuntu and Windows while
// staying green on macOS. So this asserts bit patterns, not proximity.
//
// Deliberately NOT the whole float32 domain: that is 1.07e9 points and belongs in the one-off
// exhaustive sweep that qualified the fix (0 failures either way, on AppleClang and glibc). What
// lives here is the dense-but-cheap version — a stride across the range, the seam neighbourhood
// where the two branches meet, and the literal values that were red.
TEST(ColorSpace, GammaRoundTripIsBitExact) {
  auto bits = [](float f) {
    std::uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(u));
    return u;
  };

  std::vector<float> samples;
  for (int i = 0; i <= 20000; i++) {
    samples.push_back(static_cast<float>(i) / 20000.0f);
  }
  // The seam: 0.0031308 * 12.92 is 0.040449936, NOT the 0.04045 the old comment claimed, so the
  // interval between them is where a mismatched pair of branch predicates shows up and nowhere
  // else. Walk it one ULP at a time from either side.
  float seam = 0.0404f;
  for (int i = 0; i < 4000; i++) {
    samples.push_back(seam);
    seam = std::nextafter(seam, 1.0f);
  }
  // The reverse direction's seam is somewhere else entirely, and the vector feeds both loops
  // below, so it needs walking too: LinearToSrgb branches on kSrgbCurveCutoffLinear = 0.0031308 in
  // LINEAR space, an order of magnitude away from the 0.0404 sRGB-space seam above. Without this
  // walk the encode/decode/encode direction meets its own branch boundary only at the 5e-5 stride
  // of the sweep — coarse enough to step straight over a mismatched pair of predicates.
  float linear_seam = std::nextafter(kSrgbCurveCutoffLinear, 0.0f);
  for (int i = 0; i < 2000; i++) {
    samples.push_back(linear_seam);
    linear_seam = std::nextafter(linear_seam, 0.0f);
  }
  linear_seam = kSrgbCurveCutoffLinear;
  for (int i = 0; i < 2000; i++) {
    samples.push_back(linear_seam);
    linear_seam = std::nextafter(linear_seam, 1.0f);
  }
  // The two values that were actually red on CI (test/e2e/configs/reference_point_markers.json and
  // zenith_nadir_marker_compat.json both carry background [0.02, 0.02, 0.05]).
  samples.push_back(0.02f);
  samples.push_back(0.05f);

  for (float srgb : samples) {
    // sRGB -> linear -> sRGB -> linear must land on the same linear float as the first decode.
    const float linear = SrgbToLinear(srgb);
    EXPECT_EQ(bits(SrgbToLinear(LinearToSrgb(linear))), bits(linear))
        << "decode/encode/decode drifted at sRGB " << srgb;
  }
  for (float linear : samples) {
    // ... and the same in the other direction, for the encode-side callers.
    const float srgb = LinearToSrgb(linear);
    EXPECT_EQ(bits(LinearToSrgb(SrgbToLinear(srgb))), bits(srgb))
        << "encode/decode/encode drifted at linear " << linear;
  }
}

// The property the JSON boundary rests on: a value authored in sRGB, decoded to linear on the way
// in and re-encoded on the way out, comes back unchanged. Both directions are checked because the
// two parsers use one each (decode side / encode side).
TEST(ColorSpace, SrgbToLinearInverseOfLinearToSrgb) {
  const float kValues[] = {
    0.0f, 0.001f, 0.02f, 0.04f, 0.041f, 0.05f, 0.1f, 0.15f, 0.2f, 0.35f, 0.5f, 0.6f, 0.8f, 1.0f
  };
  for (float v : kValues) {
    EXPECT_NEAR(SrgbToLinear(LinearToSrgb(v)), v, 1e-6f) << "linear→sRGB→linear at " << v;
    EXPECT_NEAR(LinearToSrgb(SrgbToLinear(v)), v, 1e-6f) << "sRGB→linear→sRGB at " << v;
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

// ---- XyzToSrgbUint8 with an additive background ----
//
// The overload the GUI's .lmc bake path uses. Its whole reason to exist is WHERE the background
// enters the chain (linear RGB, before clamp and gamma), so these cases pin the composition, not
// just the arithmetic of one call.

TEST(ColorSpace, XyzToSrgbUint8BackgroundZeroMatchesScaleOverload) {
  // Regression anchor: a zero background must be a bit-exact no-op against the pre-existing
  // overload. Anything else means the new code path is not the same pipeline with one term added.
  const float xyz[9] = { 0.4f, 0.3f, 0.2f, 0.0f, 0.0f, 0.0f, 1.5f, 0.3f, 0.0f };
  const float background[3] = { 0.0f, 0.0f, 0.0f };

  unsigned char expected[9];
  XyzToSrgbUint8(xyz, expected, 3, 1.7f);
  unsigned char out[9];
  XyzToSrgbUint8(xyz, out, 3, 1.7f, background);

  for (int j = 0; j < 9; j++) {
    EXPECT_EQ(out[j], expected[j]) << "channel " << j;
  }
}

TEST(ColorSpace, XyzToSrgbUint8ZeroEnergyPixelReturnsTheAuthoredSrgbTriple) {
  // The identity the whole colour-space contract exists for, on the CPU side: a pixel carrying no
  // halo energy comes back as the sRGB triple the user picked, because
  // LinearToSrgb(SrgbToLinear(x)) == x. The e2e sibling
  // (test/e2e-correctness/test_background_color_contract.py) asserts the same property through the
  // CLI's own render path.
  //
  // The uint8 write is a TRUNCATION (`static_cast<unsigned char>(v * 255.0f)`), so a channel whose
  // float32 round trip lands a hair below its authored value loses a whole byte — 0.2 comes back
  // as 0.19999999, and 0.19999999 * 255 truncates to 50 where round(0.2 * 255) is 51. That is the
  // pipeline's pre-existing narrowing behaviour, not something this overload introduces, so the
  // tolerance is the same 1 LSB the e2e sibling allows.
  const float srgb[3] = { 0.2f, 0.35f, 0.6f };
  float background[3];
  SrgbToLinearRgb(srgb, background);

  const float xyz[3] = { 0.0f, 0.0f, 0.0f };
  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1, 1.0f, background);

  for (int j = 0; j < 3; j++) {
    const int expected = static_cast<int>(std::lround(srgb[j] * 255.0f));
    EXPECT_LE(std::abs(static_cast<int>(out[j]) - expected), 1)
        << "channel " << j << ": got " << static_cast<int>(out[j]) << ", authored byte " << expected
        << ". A value near " << static_cast<int>(std::lround(LinearToSrgb(srgb[j]) * 255.0f))
        << " would mean the picker value was taken as linear and gamma-encoded a second time.";
  }
}

TEST(ColorSpace, XyzToSrgbUint8BackgroundIsAddedInLinearRgbBeforeGamma) {
  // Independent re-derivation of the composition, spelled out step by step rather than by calling
  // the function under test differently: gamut clip -> matrix -> add -> clamp -> gamma. A version
  // that added the background AFTER the gamma curve, or before the gamut clip, disagrees here.
  const float xyz[3] = { 0.4f, 0.3f, 0.2f };
  const float scale = 1.3f;
  const float background[3] = { 0.05f, 0.1f, 0.25f };

  float scaled[3];
  for (int j = 0; j < 3; j++) {
    scaled[j] = xyz[j] * scale;
  }
  float clipped[3];
  GamutClipXyz(scaled, clipped);
  float rgb[3];
  XyzToLinearRgb(clipped, rgb);
  unsigned char expected[3];
  for (int j = 0; j < 3; j++) {
    expected[j] = static_cast<unsigned char>(LinearToSrgb(std::clamp(rgb[j] + background[j], 0.0f, 1.0f)) * 255.0f);
  }

  unsigned char out[3];
  XyzToSrgbUint8(xyz, out, 1, scale, background);
  for (int j = 0; j < 3; j++) {
    EXPECT_EQ(out[j], expected[j]) << "channel " << j;
  }
}

TEST(ColorSpace, XyzToSrgbUint8BackgroundNeverDarkensAPixel) {
  // Additive blending, so the output is monotone in the background: a background can only lift a
  // pixel, never sink the halo beneath it. Stated as a test because the opposite reading ("the
  // halo disappears into a bright sky") is the intuition a reader arrives with.
  const float xyz[3] = { 0.4f, 0.3f, 0.2f };
  const float none[3] = { 0.0f, 0.0f, 0.0f };
  const float sky[3] = { 0.03f, 0.1f, 0.32f };

  unsigned char without[3];
  unsigned char with[3];
  XyzToSrgbUint8(xyz, without, 1, 1.0f, none);
  XyzToSrgbUint8(xyz, with, 1, 1.0f, sky);
  for (int j = 0; j < 3; j++) {
    EXPECT_GE(with[j], without[j]) << "channel " << j;
  }
}

// ---- SrgbToLinearRgb ----

TEST(ColorSpace, SrgbToLinearRgbMatchesTheScalarPerChannel) {
  const float srgb[3] = { 0.0f, 0.04f, 0.75f };  // below, at and above the 0.04045 curve knee
  float linear[3];
  SrgbToLinearRgb(srgb, linear);
  for (int j = 0; j < 3; j++) {
    EXPECT_FLOAT_EQ(linear[j], SrgbToLinear(srgb[j])) << "channel " << j;
  }
}

}  // namespace
}  // namespace lumice
