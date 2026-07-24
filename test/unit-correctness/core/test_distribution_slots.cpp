#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "core/math.hpp"

// Per-type slot semantics of lumice::Distribution: which of the two float slots each
// DistributionType reads as its anchor and which as its spread, and that
// RandomNumberGenerator::Get() feeds them into the documented formula.
//
// What is being protected: a swap of the two slots. Both are float, so a swap compiles cleanly
// and shows up only in the numbers — the compiler cannot help here, which is why the named
// accessors on Distribution exist and why this file exists to check them.
//
// Two things this file deliberately does NOT do, both because they were tried and measured:
//
//   * No hardcoded golden output. A first revision pinned raw IEEE-754 bit patterns; running the
//     identical file on Linux showed the Gaussian rows disagreeing with macOS, because libstdc++
//     and libc++ return the two Box-Muller values of std::normal_distribution in the opposite
//     order, so the sequences come out pairwise transposed. A constant table pins the standard
//     library's choices, not this project's code.
//   * No bit-equality against a reference computed here. That was the second revision, and on
//     gcc/Linux it disagreed with Get() by 1 ULP on three kUniform draws and one kZigzag draw:
//     `a * b + c` gets contracted into an FMA in one translation unit and not the other, so the
//     same arithmetic on the same inputs legitimately lands on adjacent floats. macOS/clang
//     happened to agree, which is exactly how a portability trap hides. EXPECT_FLOAT_EQ (4 ULP)
//     is loose enough to absorb contraction and still orders of magnitude tighter than any slot
//     swap, which moves results by whole units.
//
// Bit-exactness of the mean/std → center/spread rename itself was verified separately, by running
// a golden-constant build of this file against both the pre-rename and post-rename revisions on a
// single toolchain, where bit-equality is a meaningful claim. That experiment belongs to the
// refactor, not to the permanent test suite.
//
// Why a locally constructed generator and not RandomNumberGenerator::GetInstance(): SetSeed()
// reseeds the Mersenne twister but does NOT reset gauss_dist_, whose std::normal_distribution
// caches a second Box-Muller value. On the shared instance the Gaussian rows would therefore
// depend on whatever ran before in the same binary. A fresh local generator is order-independent.
//
// The distributions are built with positional aggregate initialization
// (`Distribution{ type, anchor, spread }`) on purpose: the slot order is stable across the
// representation change, so this file compiles and runs unmodified against both the old and the
// new member names.

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

// The documented formula for each type, written against the slot roles rather than against the
// member names: `anchor` is what Distribution calls `center`, `spread` what it calls `spread`.
// `ref` must be an independently seeded generator that has consumed exactly as many primitive
// draws as the generator under test. Deliberately mirrors RandomNumberGenerator::Get() — that is
// the point: the mirror is written from the per-type table, so an implementation that reads the
// wrong slot diverges from it immediately.
float ExpectedDraw(const Case& c, lumice::RandomNumberGenerator& ref) {
  switch (c.type) {
    case DistributionType::kNoRandom:
      return c.anchor;
    case DistributionType::kUniform:
      return (ref.GetUniform() - 0.5f) * c.spread + c.anchor;
    case DistributionType::kGaussian:
    case DistributionType::kGaussianLegacy:
      return ref.GetGaussian() * c.spread + c.anchor;
    case DistributionType::kZigzag:
      return std::abs(c.spread * std::sin(ref.GetUniform() * 2.0f * lumice::math::kPi) + c.anchor);
    case DistributionType::kLaplacian: {
      const float u = ref.GetUniform();
      const float sign = (u < 0.5f) ? -1.0f : 1.0f;
      float arg = 1.0f - 2.0f * std::abs(u - 0.5f);
      arg = std::max(arg, std::numeric_limits<float>::min());
      return c.anchor - c.spread * sign * std::log(arg);
    }
  }
  ADD_FAILURE() << "unhandled DistributionType in ExpectedDraw: " << c.name;
  return 0.0f;
}

}  // namespace

TEST(DistributionSlots, GetMatchesTheDocumentedFormulaForEveryType) {
  for (int ci = 0; ci < kCaseCnt; ci++) {
    const Case& c = kCases[ci];
    lumice::RandomNumberGenerator rng{ kSeed };
    lumice::RandomNumberGenerator ref{ kSeed };
    Distribution dist{ c.type, c.anchor, c.spread };
    for (int i = 0; i < kDraws; i++) {
      const float got = rng.Get(dist);
      const float want = ExpectedDraw(c, ref);
      EXPECT_FLOAT_EQ(got, want) << c.name << " draw " << i << ": got 0x" << std::hex << FloatBits(got) << ", want 0x"
                                 << FloatBits(want) << std::dec;
    }
  }
}

// The two Gaussian variants differ only in the Jacobian handling applied by the *caller*
// (SampleSphericalPointsSph); RandomNumberGenerator::Get() itself shares one branch. Pinning that
// keeps the shared branch from being split without a deliberate decision.
TEST(DistributionSlots, GaussianAndLegacyShareTheSameGetBranch) {
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
// float. Unlike the formula test above, these assertions do not reference the implementation's
// arithmetic at all: each case picks parameters whose *range* separates a correct read from a
// swapped one, so they still fail if the mirror above were ever edited to match a broken Get().
TEST(DistributionSlots, AnchorAndSpreadSlotsAreNotSwapped) {
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
