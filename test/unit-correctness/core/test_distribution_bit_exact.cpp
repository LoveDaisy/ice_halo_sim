#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "core/math.hpp"

// Bit-exact golden values for RandomNumberGenerator::Get() over every DistributionType.
//
// Why raw bit patterns and not EXPECT_FLOAT_EQ: this file exists to prove that a *representation*
// change to lumice::Distribution leaves the sampled numbers untouched. A near-equality assertion
// would tolerate exactly the kind of drift it is supposed to catch, so the comparison is on the
// IEEE-754 bits.
//
// Why a locally constructed generator and not RandomNumberGenerator::GetInstance(): SetSeed()
// reseeds the Mersenne twister but does NOT reset gauss_dist_, whose std::normal_distribution
// caches a second Box-Muller value. On the shared instance the Gaussian rows would therefore
// depend on whatever ran before in the same binary. A fresh local generator is order-independent.
//
// The distributions are built with positional aggregate initialization
// (`Distribution{ type, anchor, spread }`) on purpose: the slot order is stable across the
// representation change, so this file compiles and runs unmodified against both the old and the
// new member names — which is what makes a literal before/after comparison possible.

namespace {

using lumice::Distribution;
using lumice::DistributionType;

constexpr uint32_t kSeed = 20260724u;
constexpr int kDraws = 8;

struct Case {
  const char* name;
  DistributionType type;
  float anchor;  // `center`: value / midpoint / mean / tilt / location
  float spread;  // `spread`: unused / full range / sigma / amplitude / scale
};

// Parameters are chosen so no two rows share a formula path and none of them is degenerate
// (a zero spread would make several rows trivially equal and hide a slot swap).
constexpr Case kCases[] = {
  { "kNoRandom", DistributionType::kNoRandom, 12.5f, 3.0f },
  { "kUniform", DistributionType::kUniform, 45.0f, 90.0f },
  { "kGaussian", DistributionType::kGaussian, 30.0f, 5.0f },
  { "kGaussianLegacy", DistributionType::kGaussianLegacy, 30.0f, 5.0f },
  { "kZigzag", DistributionType::kZigzag, 10.0f, 25.0f },
  { "kLaplacian", DistributionType::kLaplacian, 45.0f, 3.0f },
};

constexpr int kCaseCnt = static_cast<int>(sizeof(kCases) / sizeof(kCases[0]));

uint32_t FloatBits(float v) {
  uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  return bits;
}

}  // namespace

TEST(DistributionBitExact, GetMatchesGoldenBitsForEveryType) {
  // Row order matches kCases. Captured from the implementation and cross-checked by running this
  // same file against the pre-rename revision — see the task record for the comparison.
  static const uint32_t kGolden[kCaseCnt][kDraws] = {
    // kNoRandom
    { 0x41480000u, 0x41480000u, 0x41480000u, 0x41480000u, 0x41480000u, 0x41480000u, 0x41480000u, 0x41480000u },
    // kUniform
    { 0x4290ba4fu, 0x41045c79u, 0x422222a1u, 0x4135f49au, 0x42ad8230u, 0x41611610u, 0x4141947bu, 0x422c9f50u },
    // kGaussian
    { 0x41ea66b7u, 0x41c5d833u, 0x41c36a10u, 0x41ed8025u, 0x41c54908u, 0x42082fc7u, 0x41b12e8fu, 0x41e70f03u },
    // kGaussianLegacy
    { 0x41ea66b7u, 0x41c5d833u, 0x41c36a10u, 0x41ed8025u, 0x41c54908u, 0x42082fc7u, 0x41b12e8fu, 0x41e70f03u },
    // kZigzag
    { 0x415928fau, 0x41bd312au, 0x418d5aa9u, 0x41de9f72u, 0x408c4580u, 0x41f655f0u, 0x41e58c4cu, 0x41535d1bu },
    // kLaplacian
    { 0x423f3d96u, 0x421face7u, 0x4232bee6u, 0x42237e7fu, 0x42538d47u, 0x42260bfau, 0x42243cbeu, 0x42337f6fu },
  };

  for (int ci = 0; ci < kCaseCnt; ci++) {
    const Case& c = kCases[ci];
    lumice::RandomNumberGenerator rng{ kSeed };
    Distribution dist{ c.type, c.anchor, c.spread };
    for (int i = 0; i < kDraws; i++) {
      const float v = rng.Get(dist);
      EXPECT_EQ(FloatBits(v), kGolden[ci][i])
          << c.name << " draw " << i << ": got " << v << " (0x" << std::hex << FloatBits(v) << std::dec << ")";
    }
  }
}

// The two Gaussian variants differ only in the Jacobian handling applied by the *caller*
// (SampleSphericalPointsSph); RandomNumberGenerator::Get() itself shares one branch. Pinning that
// keeps the shared branch from being split without a deliberate decision.
TEST(DistributionBitExact, GaussianAndLegacyShareTheSameGetBranch) {
  lumice::RandomNumberGenerator rng_a{ kSeed };
  lumice::RandomNumberGenerator rng_b{ kSeed };
  Distribution gauss{ DistributionType::kGaussian, 30.0f, 5.0f };
  Distribution legacy{ DistributionType::kGaussianLegacy, 30.0f, 5.0f };
  for (int i = 0; i < kDraws; i++) {
    EXPECT_EQ(FloatBits(rng_a.Get(gauss)), FloatBits(rng_b.Get(legacy))) << "draw " << i;
  }
}

// A slot swap (reading the spread where the anchor is meant, or vice versa) is the one error class
// the named accessors are there to prevent and the one the compiler cannot catch — both slots are
// float. These assertions fail loudly on a swap even if the golden table above were ever
// regenerated from swapped code.
TEST(DistributionBitExact, AnchorAndSpreadSlotsAreNotSwapped) {
  // kNoRandom ignores the spread entirely and returns the anchor.
  lumice::RandomNumberGenerator rng{ kSeed };
  Distribution no_random{ DistributionType::kNoRandom, 7.25f, 999.0f };
  EXPECT_FLOAT_EQ(rng.Get(no_random), 7.25f);

  // kUniform draws from anchor ± spread/2, so with spread=2 every draw is within 1 of the anchor.
  Distribution uniform{ DistributionType::kUniform, 100.0f, 2.0f };
  for (int i = 0; i < 64; i++) {
    const float v = rng.Get(uniform);
    EXPECT_GE(v, 99.0f);
    EXPECT_LE(v, 101.0f);
  }

  // kZigzag is |amplitude·sin(2πU) + tilt|; with tilt=1000 and amplitude=1 every draw stays near
  // the tilt. A swap would instead centre the draws near 1.
  Distribution zigzag{ DistributionType::kZigzag, 1000.0f, 1.0f };
  for (int i = 0; i < 64; i++) {
    const float v = rng.Get(zigzag);
    EXPECT_GE(v, 999.0f);
    EXPECT_LE(v, 1001.0f);
  }

  // kLaplacian is location - scale·sign·ln(...); with scale=0 every draw collapses to the
  // location. A swap would collapse them to 0 instead.
  Distribution laplacian{ DistributionType::kLaplacian, 55.0f, 0.0f };
  for (int i = 0; i < 64; i++) {
    EXPECT_FLOAT_EQ(rng.Get(laplacian), 55.0f);
  }

  // kGaussian with sigma=0 collapses to the mean; a swap would collapse to 0.
  Distribution gauss{ DistributionType::kGaussian, 33.0f, 0.0f };
  for (int i = 0; i < 64; i++) {
    EXPECT_FLOAT_EQ(rng.Get(gauss), 33.0f);
  }
}
