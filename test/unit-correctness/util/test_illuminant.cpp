#include <cmath>

#include "gtest/gtest.h"
#include "util/illuminant.hpp"

namespace lumice {
namespace {

// MeanIlluminantWeight is the emitted-energy denominator's illuminant term: the
// expected SPD weight of a batch whose wavelength is drawn uniformly over
// [380, 780] nm. It has to be a deterministic constant, not the weight actually
// sampled, or the same config at a different seed would render at a different
// brightness.

TEST(IlluminantTest, MeanWeightD65MatchesQuadrature) {
  // Independently measured over the shipped GetIlluminantSpd: a dense uniform
  // average of the D65 SPD across the sampled band is 87.9532682. The tolerance
  // is far tighter than any plausible grid-density difference, so this pins the
  // value rather than merely the method.
  EXPECT_NEAR(MeanIlluminantWeight(IlluminantType::kD65), 87.9532682f, 0.01f);
}

TEST(IlluminantTest, MeanWeightEqualEnergyIsExactlyOne) {
  // Illuminant E is constant 1.0 across the whole band, so its band average is
  // 1.0 by construction — an analytic anchor that catches a grid that walks off
  // the band (a point outside [300, 830] returns 0 and would drag the mean
  // below 1).
  EXPECT_NEAR(MeanIlluminantWeight(IlluminantType::kE), 1.0f, 1e-5f);
}

TEST(IlluminantTest, MeanWeightFiniteAndPositiveForEveryType) {
  const IlluminantType kAll[] = { IlluminantType::kD50, IlluminantType::kD55, IlluminantType::kD65,
                                  IlluminantType::kD75, IlluminantType::kA,   IlluminantType::kE };
  for (auto type : kAll) {
    float mean = MeanIlluminantWeight(type);
    EXPECT_TRUE(std::isfinite(mean)) << "type index " << static_cast<int>(type);
    EXPECT_GT(mean, 0.0f) << "type index " << static_cast<int>(type);
  }
}

TEST(IlluminantTest, MeanWeightIsMemoizedBitIdentical) {
  // Two calls must return the identical value — the second one reads the
  // memoized static rather than re-running the quadrature.
  for (auto type : { IlluminantType::kD50, IlluminantType::kA }) {
    EXPECT_EQ(MeanIlluminantWeight(type), MeanIlluminantWeight(type));
  }
}

TEST(IlluminantTest, MeanWeightIsBoundedByTheSpdOverTheBand) {
  // A band average must sit between the band's min and max. This is the guard
  // against the mean and the sampler drifting onto different intervals: if
  // MeanIlluminantWeight averaged a wider band than [380, 780], the
  // out-of-range zeros would push it under the minimum measured here.
  for (auto type : { IlluminantType::kD65, IlluminantType::kA }) {
    float lo = 1e30f;
    float hi = -1e30f;
    for (int i = 0; i <= 4000; ++i) {
      float wl = 380.0f + 400.0f * (static_cast<float>(i) / 4000.0f);
      float w = GetIlluminantSpd(type, wl);
      lo = std::min(lo, w);
      hi = std::max(hi, w);
    }
    float mean = MeanIlluminantWeight(type);
    EXPECT_GE(mean, lo);
    EXPECT_LE(mean, hi);
  }
}

}  // namespace
}  // namespace lumice
