#include "util/illuminant.hpp"

#include <cmath>

namespace lumice {
namespace {

// ============================================================================
// D-series daylight SPD reconstruction
// ============================================================================

// Compute chromaticity coordinate x_D from CCT (Kelvin).
// CIE 015:2018 formulas.
float DaylightChromaticityX(float cct) {
  float t_inv = 1.0f / cct;
  float t_inv2 = t_inv * t_inv;
  float t_inv3 = t_inv2 * t_inv;

  if (cct <= 7000.0f) {
    return 0.244063f + 0.09911e3f * t_inv + 2.9678e6f * t_inv2 - 4.6070e9f * t_inv3;
  }
  return 0.237040f + 0.24748e3f * t_inv + 1.9018e6f * t_inv2 - 2.0064e9f * t_inv3;
}

// Compute chromaticity coordinate y_D from x_D.
float DaylightChromaticityY(float x_d) {
  return -3.000f * x_d * x_d + 2.870f * x_d - 0.275f;
}

// Compute D-series mixing coefficients M1, M2.
struct DaylightCoeffs {
  float m1;
  float m2;
};

DaylightCoeffs DaylightMixingCoeffs(float x_d, float y_d) {
  float denom = 0.0241f + 0.2562f * x_d - 0.7341f * y_d;
  float m1 = (-1.3515f - 1.7703f * x_d + 5.9114f * y_d) / denom;
  float m2 = (0.0300f - 31.4424f * x_d + 30.0717f * y_d) / denom;
  return { m1, m2 };
}

// Get CCT for a D-series illuminant type. Returns 0 for non-D types.
float GetDaylightCct(IlluminantType type) {
  switch (type) {
    case IlluminantType::kD50:
      return kCctD50;
    case IlluminantType::kD55:
      return kCctD55;
    case IlluminantType::kD65:
      return kCctD65;
    case IlluminantType::kD75:
      return kCctD75;
    default:
      return 0.0f;
  }
}

// Reconstruct D-series SPD at a given wavelength using basis vectors.
// Linear interpolation between 5 nm grid points.
float GetDaylightSpd(float cct, float wavelength) {
  if (wavelength < kDaylightLambdaMin || wavelength > kDaylightLambdaMax) {
    return 0.0f;
  }

  float x_d = DaylightChromaticityX(cct);
  float y_d = DaylightChromaticityY(x_d);
  auto [m1, m2] = DaylightMixingCoeffs(x_d, y_d);

  // Fractional index into the 5 nm grid
  float fi = (wavelength - kDaylightLambdaMin) / static_cast<float>(kDaylightLambdaStep);
  int i0 = static_cast<int>(fi);
  float frac = fi - static_cast<float>(i0);

  // Clamp to valid range
  if (i0 >= kDaylightNumPoints - 1) {
    i0 = kDaylightNumPoints - 1;
    frac = 0.0f;
  }

  auto lerp = [](float a, float b, float t) { return a + t * (b - a); };

  float s0 = lerp(kDaylightS0[i0], kDaylightS0[i0 + (i0 < kDaylightNumPoints - 1 ? 1 : 0)], frac);
  float s1 = lerp(kDaylightS1[i0], kDaylightS1[i0 + (i0 < kDaylightNumPoints - 1 ? 1 : 0)], frac);
  float s2 = lerp(kDaylightS2[i0], kDaylightS2[i0 + (i0 < kDaylightNumPoints - 1 ? 1 : 0)], frac);

  return s0 + m1 * s1 + m2 * s2;
}


// ============================================================================
// Illuminant A: Planck blackbody at 2856 K
// ============================================================================

float GetIlluminantASpd(float wavelength) {
  if (wavelength <= 0.0f) {
    return 0.0f;
  }
  float ratio = kIlluminantARefWl / wavelength;
  float ratio5 = ratio * ratio * ratio * ratio * ratio;
  float exp_ref = std::exp(kIlluminantAC2 / (kIlluminantATemp * kIlluminantARefWl));
  float exp_lam = std::exp(kIlluminantAC2 / (kIlluminantATemp * wavelength));
  return 100.0f * ratio5 * (exp_ref - 1.0f) / (exp_lam - 1.0f);
}

}  // namespace


// ============================================================================
// Public API
// ============================================================================

float GetIlluminantSpd(IlluminantType type, float wavelength) {
  switch (type) {
    case IlluminantType::kD50:
    case IlluminantType::kD55:
    case IlluminantType::kD65:
    case IlluminantType::kD75: {
      float cct = GetDaylightCct(type);
      return GetDaylightSpd(cct, wavelength);
    }
    case IlluminantType::kA:
      if (wavelength < kDaylightLambdaMin || wavelength > kDaylightLambdaMax) {
        return 0.0f;
      }
      return GetIlluminantASpd(wavelength);
    case IlluminantType::kE:
      if (wavelength < kDaylightLambdaMin || wavelength > kDaylightLambdaMax) {
        return 0.0f;
      }
      return 1.0f;
  }
  return 0.0f;
}

}  // namespace lumice
