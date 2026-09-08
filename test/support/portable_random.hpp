#ifndef LUMICE_TEST_SUPPORT_PORTABLE_RANDOM_HPP_
#define LUMICE_TEST_SUPPORT_PORTABLE_RANDOM_HPP_

// Sampling helpers that draw from std::mt19937 directly, for test code that needs random inputs but
// must draw the SAME ones on every standard library.
//
// Why this exists. `std::mt19937` is a named specialization of `mersenne_twister_engine` whose
// recurrence, seeding and output sequence the standard pins bit for bit — it even fixes the
// 10000th output value as a conformance check. The distribution adaptors layered on top of it are
// pinned only in *law*: [rand.dist.uni.real] and [rand.dist.norm.normal] give the pdf and stop
// there, leaving both the algorithm and the number of engine draws each sample consumes to the
// implementation. So `std::mt19937 rng(42); std::normal_distribution<float> d(0, 1); d(rng);`
// produces one value under libc++ and a different one under libstdc++ or the MSVC STL, from the
// same seed. Three such divergences are already recorded in this tree, each from a real incident:
// libstdc++ and libc++ return the two Box-Muller partners in opposite order
// (test_distribution_slots.cpp), the MSVC STL's `uniform_real_distribution<float>` can return its
// upper bound (test_geo3d.cpp), and `normal_distribution`'s polar method caches a second value
// across calls so a reseed does not reset it (test_distribution_slots.cpp).
//
// That is why an assertion whose outcome depends on WHICH samples were drawn is not portable, and
// it is how the closed-form prism/pyramid golden tests once turned CI red: random samples fed into
// `EXPECT_EQ(mismatch, 0)`. Drawing through these functions removes the adaptor layer, so a fixed
// seed reconstructs one sample sequence everywhere and that whole failure mode is closed.
//
// What this does NOT buy you — worth being blunt about, because assuming otherwise is how the same
// bug comes back one layer down. These functions pin the *sample sequence*, not the *arithmetic
// performed on it*. `PortableGaussianFloat` calls std::log / std::sqrt / std::cos, whose last-place
// accuracy libm does not guarantee across platforms (this tree has a measured 1-ULP case from FMA
// contraction, in test_distribution_slots.cpp). A portable sample source and a tolerant assertion
// are two complementary halves: any new assertion built on these should still be a band
// (`EXPECT_LT(ratio, ceiling)`), never an exact equality on a computed float.
//
// Interface shape — free functions taking `std::mt19937&`, deliberately unlike the production
// `RandomNumberGenerator` (src/core/math.hpp), which owns `gauss_dist_` / `uniform_dist_` members.
// The difference is the point: the whole purpose here is to avoid the adaptor members that class
// holds. This is not an inconsistency waiting to be unified — production's own portability question
// is a separate one (its symptom is "the same config renders different pixels on different
// platforms", not "a test goes red"), with a different contract and a different fix.

#include <cmath>
#include <cstdint>
#include <random>

namespace lumice {
namespace test {

// Uniform float in [0, 1). Takes the top 24 bits of one engine draw — exactly the mantissa width of
// a float, so every representable step is reachable and no rounding can land on 1.0f. This is the
// spelling already proven in test_crystal.cpp's pyramid move/assign fuzz; it is repeated here as a
// single owner rather than copied a third time.
inline float PortableCanonicalFloat(std::mt19937& rng) {
  return static_cast<float>(rng() >> 8) * (1.0f / 16777216.0f);
}

// Uniform float in [lo, hi). Consumes one engine draw.
inline float PortableUniformFloat(std::mt19937& rng, float lo, float hi) {
  return lo + (hi - lo) * PortableCanonicalFloat(rng);
}

// Uniform integer in [lo, hi] — CLOSED at both ends, matching std::uniform_int_distribution's
// convention so a call site can be swapped over without an off-by-one. Consumes one engine draw.
//
// Uses Lemire's multiply-shift (draw * span >> 32) rather than `draw % span`: both are biased by at
// most one extra slot in 2^32 per value, but multiply-shift keeps the mapping monotone in the
// engine output, which makes a divergence between two runs readable when debugging. The bias is
// span / 2^32 in relative terms — for the index selection this is meant for (spans of tens, at
// most thousands) it is under 1e-6 and irrelevant. It is NOT a uniform generator for spans near
// 2^32, and nothing here should be pressed into that use.
inline int PortableUniformInt(std::mt19937& rng, int lo, int hi) {
  const auto span = static_cast<uint64_t>(hi - lo) + 1u;
  const auto offset = static_cast<uint64_t>(rng()) * span >> 32;
  return lo + static_cast<int>(offset);
}

// Normal deviate with the given mean and standard deviation, by Box-Muller.
//
// Consumes exactly two engine draws per call and returns exactly one value, discarding the sine
// partner. That waste is deliberate: caching the partner across calls is precisely what makes
// std::normal_distribution stateful, and that state is what survives a reseed and what libstdc++
// and libc++ hand back in opposite order. A stateless function costs one extra draw and removes
// both hazards, so a caller can reason about engine consumption by reading the call site.
inline float PortableGaussianFloat(std::mt19937& rng, float mean, float stddev) {
  // 1 - u maps [0, 1) onto (0, 1], keeping std::log's argument strictly positive. Drawing u in
  // [0, 1) and testing for zero would branch on a value that occurs once in 2^24 draws — i.e. a
  // path that a fixed seed almost never takes and no test would ever cover.
  const float u1 = 1.0f - PortableCanonicalFloat(rng);
  const float u2 = PortableCanonicalFloat(rng);
  constexpr float kTwoPi = 6.28318530717958647692f;
  const float radius = std::sqrt(-2.0f * std::log(u1));
  return mean + stddev * radius * std::cos(kTwoPi * u2);
}

}  // namespace test
}  // namespace lumice

#endif  // LUMICE_TEST_SUPPORT_PORTABLE_RANDOM_HPP_
