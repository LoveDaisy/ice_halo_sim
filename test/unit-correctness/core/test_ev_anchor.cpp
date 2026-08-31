// Unit tests for the auto-EV anchor algorithm (core/ev_anchor.hpp), the single owner shared by
// the GUI mono path (via LUMICE_ComputeP99Y / LUMICE_ComputeEvAuto) and the server composite path
// (via NthElementP99 / TargetWhiteToLinear).

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/ev_anchor.hpp"
#include "gtest/gtest.h"

namespace lumice {
namespace {

// Helper: build a packed XYZ buffer (stride 3) of size w*h, with Y filled from a row-major
// vector of length w*h and X/Z left at 0 (ComputeP99Y looks only at channel 1).
std::vector<float> MakeXyz(int w, int h, const std::vector<float>& y_values) {
  std::vector<float> data(static_cast<size_t>(w) * static_cast<size_t>(h) * 3, 0.0f);
  for (size_t i = 0; i < y_values.size(); ++i) {
    data[i * 3 + 1] = y_values[i];
  }
  return data;
}

// (The former T1 pinned the f=1 fine path against a no-sizes overload of ComputeP99Y. That
// overload is gone: the function now takes a borrowed `const float*`, which carries no length to
// fall back on, so img_width/img_height became required. With only one call shape left the case
// reduced to asserting a value equals itself, so it was deleted rather than kept as a tautology.
// T2-T4 below still cover the box-sum exact values, the all-zero fallback and the wc=0 guard.)

// T2 — f=2 correctness on a 4x4 image: hand-verify the box-sum + P99 + /f^2.
//
// Layout (Y channel, row-major 4x4):
//   1  2 |  3  4
//   5  6 |  7  8
//   ----+-----
//   9 10 | 11 12
//  13 14 | 15 16
//
// Four 2x2 coarse bins (f=2), each sum:
//   top-left:     1+2+5+6   = 14
//   top-right:    3+4+7+8   = 22
//   bottom-left:  9+10+13+14= 46
//   bottom-right: 11+12+15+16=54
//
// P99 over the 4 nonzero coarse Y values:
//   sorted = {14, 22, 46, 54}; idx = floor(4 * 0.99) = 3 -> y_vals[3] = 54.
// Returned value = 54 / (f^2 = 4) = 13.5.
TEST(EvAnchor, ComputeP99YBoxSumsCoarselyAndFallsBackWhenItCannot) {
  std::vector<float> y = { 1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,
                           9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f };
  std::vector<float> xyz = MakeXyz(4, 4, y);
  EXPECT_FLOAT_EQ(ComputeP99Y(xyz.data(), 4, 4, 2), 13.5f);

  // Sanity: the helper itself returns the raw coarse sums.
  std::vector<float> coarse = DownsampleBoxSumY(xyz.data(), 4, 4, 2);
  ASSERT_EQ(coarse.size(), 4u);
  EXPECT_FLOAT_EQ(coarse[0], 14.0f);  // top-left
  EXPECT_FLOAT_EQ(coarse[1], 22.0f);  // top-right
  EXPECT_FLOAT_EQ(coarse[2], 46.0f);  // bottom-left
  EXPECT_FLOAT_EQ(coarse[3], 54.0f);  // bottom-right

  // T3 — all-zero input on the coarse path returns 0.0f (matches the fine-path empty case).
  std::vector<float> zeros = MakeXyz(8, 8, std::vector<float>(8 * 8, 0.0f));
  EXPECT_FLOAT_EQ(ComputeP99Y(zeros.data(), 8, 8, 8), 0.0f);

  // T4 — wc=0 guard: f=8 on a 1x1 image gives wc = hc = 0, so DownsampleBoxSumY collapses to {} and
  // ComputeP99Y falls back to the fine path (which on a single positive Y returns that value).
  std::vector<float> tiny = MakeXyz(1, 1, { 7.5f });
  EXPECT_FLOAT_EQ(ComputeP99Y(tiny.data(), 1, 1, 8), 7.5f);
  EXPECT_TRUE(DownsampleBoxSumY(tiny.data(), 1, 1, 8).empty());
}

// Mechanism-layer check on the production formula: recompute
// clamp(log2(target_linear / (p99 / snapshot_intensity)), -6, 6) independently, including the
// sRGB reverse transform, and compare. A drift in either shared sub-piece (TargetWhiteToLinear
// or the clamp bounds) shows up here rather than only as a pixel difference downstream.
TEST(EvAnchor, ComputeEvAutoIndependentFormulaCrossCheck) {
  struct Case {
    float p99;
    float snapshot_intensity;
    float target_white;
  };
  Case cases[] = {
    { 2.0e-3f, 1.0f, 135.0f },  // typical mono-path magnitudes
    { 5.0e-2f, 4.0f, 135.0f },  //
    { 1.0e-6f, 1.0f, 135.0f },  // dim enough to hit the +6 clamp
    { 1.0e3f, 1.0f, 135.0f },   // bright enough to hit the -6 clamp
    { 0.5f, 2.0f, 10.0f },      // target_white below 0.04045*255 -> linear branch of sRGB
  };
  for (const Case& c : cases) {
    float t = c.target_white / 255.0f;
    float target_linear = t <= 0.04045f ? t / 12.92f : std::pow((t + 0.055f) / 1.055f, 2.4f);
    float expected = std::clamp(std::log2f(target_linear / (c.p99 / c.snapshot_intensity)), -6.0f, 6.0f);
    EXPECT_FLOAT_EQ(ComputeEvAuto(c.p99, c.snapshot_intensity, c.target_white), expected)
        << "p99=" << c.p99 << " si=" << c.snapshot_intensity << " tw=" << c.target_white;
    // TargetWhiteToLinear is the shared sub-piece the composite path calls directly; pin it too.
    EXPECT_FLOAT_EQ(TargetWhiteToLinear(c.target_white), target_linear);
  }
}

// Both guards return a hard 0 rather than an infinity/NaN — the callers treat 0 as "no anchor
// yet" (pre-first-snapshot), so this is a contract, not just defensive coding.
TEST(EvAnchor, ComputeEvAutoGuardsNonPositiveInputs) {
  EXPECT_FLOAT_EQ(ComputeEvAuto(1.0e-3f, 0.0f, 135.0f), 0.0f);
  EXPECT_FLOAT_EQ(ComputeEvAuto(1.0e-3f, -1.0f, 135.0f), 0.0f);
  EXPECT_FLOAT_EQ(ComputeEvAuto(0.0f, 1.0f, 135.0f), 0.0f);
  EXPECT_FLOAT_EQ(ComputeEvAuto(-1.0e-3f, 1.0f, 135.0f), 0.0f);
}

// NthElementP99 is the piece three call sites used to inline; the index rule (floor(n*0.99),
// clamped to n-1) is what makes the composite anchor and the mono anchor comparable.
TEST(EvAnchor, NthElementP99IndexRuleAndEmptyInput) {
  std::vector<float> empty;
  EXPECT_FLOAT_EQ(NthElementP99(empty), 0.0f);

  // n=4 -> idx = floor(3.96) = 3 -> the max.
  std::vector<float> four = { 22.0f, 54.0f, 14.0f, 46.0f };
  EXPECT_FLOAT_EQ(NthElementP99(four), 54.0f);

  // n=1 -> idx = 0.
  std::vector<float> one = { 7.5f };
  EXPECT_FLOAT_EQ(NthElementP99(one), 7.5f);

  // n=100 -> idx = floor(99.0) = 99, i.e. the max; the clamp only engages if float rounding
  // pushes the product to n. Values 1..100 shuffled so nth_element has real work to do.
  std::vector<float> hundred;
  hundred.reserve(100);
  for (int i = 100; i >= 1; --i) {
    hundred.push_back(static_cast<float>(i));
  }
  EXPECT_FLOAT_EQ(NthElementP99(hundred), 100.0f);

  // n=200 -> idx = floor(198.0) = 198, the second largest.
  std::vector<float> two_hundred;
  two_hundred.reserve(200);
  for (int i = 200; i >= 1; --i) {
    two_hundred.push_back(static_cast<float>(i));
  }
  EXPECT_FLOAT_EQ(NthElementP99(two_hundred), 199.0f);
}

}  // namespace
}  // namespace lumice
