#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "support/portable_random.hpp"

// Self-verification for the portable sampling helpers.
//
// Scope note, because it is easy to over-claim here: every proposition below is checked on ONE
// standard library — whichever one built this binary. That is deliberate and it is the whole
// contract these helpers can be held to locally. What makes them portable is not a test, it is
// that they read `std::mt19937::operator()` and nothing else, and that operator's output sequence
// is fixed by the standard for a given seed. So these cases pin the two things a test CAN pin:
// the arithmetic on top of that stream behaves as documented, and the number of engine draws each
// call consumes is exactly what the header promises — the second being what would silently break
// if anyone "optimised" the Gaussian by caching its discarded partner, which is the very state
// that makes std::normal_distribution non-portable.

namespace lumice {
namespace {

using test::PortableCanonicalFloat;
using test::PortableGaussianFloat;
using test::PortableUniformFloat;
using test::PortableUniformInt;

constexpr uint32_t kSeed = 20260908u;

TEST(PortableRandom, SameSeedReproducesTheSameSequence) {
  std::mt19937 a(kSeed);
  std::mt19937 b(kSeed);
  int mismatches = 0;
  for (int i = 0; i < 2000; ++i) {
    // Interleave all four helpers so a divergence in engine consumption by any one of them
    // desynchronises everything after it, rather than staying local to its own draws.
    mismatches += (PortableCanonicalFloat(a) != PortableCanonicalFloat(b)) ? 1 : 0;
    mismatches += (PortableUniformFloat(a, -3.5f, 7.25f) != PortableUniformFloat(b, -3.5f, 7.25f)) ? 1 : 0;
    mismatches += (PortableUniformInt(a, -7, 42) != PortableUniformInt(b, -7, 42)) ? 1 : 0;
    mismatches += (PortableGaussianFloat(a, 1.0f, 0.5f) != PortableGaussianFloat(b, 1.0f, 0.5f)) ? 1 : 0;
  }
  EXPECT_EQ(mismatches, 0) << "two engines seeded alike diverged";
}

TEST(PortableRandom, CanonicalFloatStaysInsideTheHalfOpenUnitInterval) {
  std::mt19937 rng(kSeed);
  int out_of_range = 0;
  float observed_max = 0.0f;
  float observed_min = 1.0f;
  for (int i = 0; i < 500000; ++i) {
    const float u = PortableCanonicalFloat(rng);
    if (!(u >= 0.0f && u < 1.0f)) {
      ++out_of_range;
    }
    observed_max = std::fmax(observed_max, u);
    observed_min = std::fmin(observed_min, u);
  }
  EXPECT_EQ(out_of_range, 0) << "canonical draw escaped [0, 1)";
  // Both ends of the interval are reachable. The out_of_range count above already rules out 1.0f
  // itself — the MSVC STL behaviour recorded for uniform_real_distribution<float> in
  // test_geo3d.cpp — so what is left to show is the complementary direction: the range is not
  // silently narrower than declared.
  //
  // Note the shape of these two. The construction's true ceiling is the single value 1 - 2^-24,
  // and an earlier revision of this test asserted the sample reached exactly that. It does not:
  // 500000 draws out of 2^24 representable steps hit the top step about 3% of the time, so that
  // assertion depended on which samples came out — the very defect this file's helpers exist to
  // remove, reintroduced one layer up in the test that checks them. Bands instead: P(max <
  // 1 - 1e-4) = (1 - 1e-4)^500000 ~ e^-50, and P(min > 2^-14) ~ e^-30. Both are unreachable by
  // sampling luck and stay true under any resampling.
  EXPECT_GT(observed_max, 1.0f - 1e-4f) << "the top of the interval is never approached";
  EXPECT_LT(observed_min, 1.0f / 16384.0f) << "the low end of the interval is never reached";
}

TEST(PortableRandom, UniformFloatStaysWithinTheRequestedRange) {
  std::mt19937 rng(kSeed);
  constexpr float kLo = -12.5f;
  constexpr float kHi = 3.75f;
  int out_of_range = 0;
  for (int i = 0; i < 200000; ++i) {
    const float x = PortableUniformFloat(rng, kLo, kHi);
    if (!(x >= kLo && x < kHi)) {
      ++out_of_range;
    }
  }
  EXPECT_EQ(out_of_range, 0);
}

TEST(PortableRandom, UniformIntCoversItsClosedRangeAndNothingElse) {
  std::mt19937 rng(kSeed);
  constexpr int kLo = -3;
  constexpr int kHi = 9;
  constexpr int kSpan = kHi - kLo + 1;
  std::vector<int> hits(kSpan, 0);
  int out_of_range = 0;
  for (int i = 0; i < 200000; ++i) {
    const int v = PortableUniformInt(rng, kLo, kHi);
    if (v < kLo || v > kHi) {
      ++out_of_range;
      continue;
    }
    hits[static_cast<size_t>(v - kLo)]++;
  }
  EXPECT_EQ(out_of_range, 0) << "draw escaped [" << kLo << ", " << kHi << "]";
  // Both endpoints inclusive: an off-by-one at either end shows up as an empty bucket, which is
  // the failure a caller swapping away from std::uniform_int_distribution would actually hit.
  int empty_buckets = 0;
  for (int i = 0; i < kSpan; ++i) {
    if (hits[static_cast<size_t>(i)] == 0) {
      ++empty_buckets;
    }
  }
  EXPECT_EQ(empty_buckets, 0) << "some value in the closed range was never drawn";
  // Uniformity, loosely: 200000 draws over 13 buckets is ~15385 each, sigma ~119, so a +-8% band
  // is roughly 10 sigma. This is a smoke check for a gross mapping error, not a distribution test.
  const int expected = 200000 / kSpan;
  int skewed_buckets = 0;
  for (int i = 0; i < kSpan; ++i) {
    if (std::abs(hits[static_cast<size_t>(i)] - expected) > expected / 12) {
      ++skewed_buckets;
    }
  }
  EXPECT_EQ(skewed_buckets, 0) << "a bucket landed more than 8% off uniform";
}

TEST(PortableRandom, GaussianMatchesItsNominalMoments) {
  std::mt19937 rng(kSeed);
  constexpr float kMean = 2.5f;
  constexpr float kStd = 0.75f;
  constexpr int kN = 500000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < kN; ++i) {
    const double x = PortableGaussianFloat(rng, kMean, kStd);
    sum += x;
    sum_sq += x * x;
  }
  const double mean = sum / kN;
  const double var = sum_sq / kN - mean * mean;
  // Standard error of the mean is kStd/sqrt(N) = 1.06e-3; +-0.02 is ~19 sigma. The band is wide on
  // purpose — this asks "is it the right distribution", not "is it this exact sample".
  EXPECT_NEAR(mean, kMean, 0.02);
  EXPECT_NEAR(std::sqrt(var), kStd, 0.02);
}

TEST(PortableRandom, EachHelperConsumesTheDeclaredNumberOfEngineDraws) {
  // The header promises: one engine draw for the canonical/uniform helpers, exactly two for the
  // Gaussian with no value carried across calls. Both halves matter. A cached Box-Muller partner
  // would make the Gaussian average one draw instead of two, and would also make its output depend
  // on call history — which is exactly why a reseed does not reset std::normal_distribution, and
  // why the shared RandomNumberGenerator instance cannot be used for order-independent tests.
  struct Case {
    const char* name;
    int draws;
    void (*advance)(std::mt19937&);
  };
  const Case cases[] = {
    { "PortableCanonicalFloat", 1, [](std::mt19937& g) { (void)PortableCanonicalFloat(g); } },
    { "PortableUniformFloat", 1, [](std::mt19937& g) { (void)PortableUniformFloat(g, 0.0f, 1.0f); } },
    { "PortableUniformInt", 1, [](std::mt19937& g) { (void)PortableUniformInt(g, 0, 99); } },
    { "PortableGaussianFloat", 2, [](std::mt19937& g) { (void)PortableGaussianFloat(g, 0.0f, 1.0f); } },
  };
  for (const Case& c : cases) {
    std::mt19937 used(kSeed);
    std::mt19937 reference(kSeed);
    constexpr int kCalls = 64;
    for (int i = 0; i < kCalls; ++i) {
      c.advance(used);
    }
    reference.discard(static_cast<unsigned long long>(c.draws) * kCalls);
    EXPECT_EQ(used(), reference()) << c.name << " did not consume " << c.draws << " draw(s) per call";
  }
}

TEST(PortableRandom, GaussianIsStatelessAcrossReseeding) {
  // Reseeding an engine must fully reset the sampler's behaviour. This is the property
  // std::normal_distribution does NOT have — it caches its second Box-Muller value in the
  // distribution object, which outlives any reseed of the engine. Nothing here asserts anything
  // about std::normal_distribution itself (that would be pinning the standard library's choices,
  // which is the mistake this whole family of tests exists to avoid); it asserts only that these
  // helpers carry no such state.
  std::mt19937 rng(kSeed);
  std::vector<float> first;
  first.reserve(16);
  for (int i = 0; i < 16; ++i) {
    first.push_back(PortableGaussianFloat(rng, 0.0f, 1.0f));
  }
  rng.seed(kSeed);
  int mismatches = 0;
  for (int i = 0; i < 16; ++i) {
    if (PortableGaussianFloat(rng, 0.0f, 1.0f) != first[static_cast<size_t>(i)]) {
      ++mismatches;
    }
  }
  EXPECT_EQ(mismatches, 0) << "the sampler carried state across a reseed";
}

}  // namespace
}  // namespace lumice
