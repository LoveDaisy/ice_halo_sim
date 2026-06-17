#include <gtest/gtest.h>

#include <vector>

#include "gui/gui_ev_auto.hpp"

namespace lumice::gui {
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

// T1 — f=1 regression: passing downsample_factor=1 with sizes must match the
// no-sizes overload (the legacy fine path).
TEST(EvAuto, ComputeP99YFineRegressionWithDefaultFactor) {
  // 3x3 image with varied positive Y and one zero — small enough to enumerate.
  std::vector<float> y = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
  std::vector<float> xyz = MakeXyz(3, 3, y);

  float legacy = ComputeP99Y(xyz);
  float with_factor_one = ComputeP99Y(xyz, 3, 3, 1);
  EXPECT_FLOAT_EQ(legacy, with_factor_one);
}

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
TEST(EvAuto, ComputeP99YBoxSumExact4x4Factor2) {
  std::vector<float> y = { 1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,
                           9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f };
  std::vector<float> xyz = MakeXyz(4, 4, y);

  float p99 = ComputeP99Y(xyz, 4, 4, 2);
  EXPECT_FLOAT_EQ(p99, 13.5f);

  // Sanity: the helper itself returns the raw coarse sums.
  std::vector<float> coarse = DownsampleBoxSumY(xyz, 4, 4, 2);
  ASSERT_EQ(coarse.size(), 4u);
  EXPECT_FLOAT_EQ(coarse[0], 14.0f);  // top-left
  EXPECT_FLOAT_EQ(coarse[1], 22.0f);  // top-right
  EXPECT_FLOAT_EQ(coarse[2], 46.0f);  // bottom-left
  EXPECT_FLOAT_EQ(coarse[3], 54.0f);  // bottom-right
}

// T3 — all-zero input on the coarse path returns 0.0f (matches fine-path empty case).
TEST(EvAuto, ComputeP99YAllZeroCoarseReturnsZero) {
  std::vector<float> y(8 * 8, 0.0f);
  std::vector<float> xyz = MakeXyz(8, 8, y);

  EXPECT_FLOAT_EQ(ComputeP99Y(xyz, 8, 8, 8), 0.0f);
}

// T4 — wc=0 guard: when f exceeds dimensions, DownsampleBoxSumY collapses to
// {} and ComputeP99Y falls back to the fine path without crashing.
TEST(EvAuto, ComputeP99YWcZeroGuardFallsBackToFine) {
  std::vector<float> xyz = MakeXyz(1, 1, { 7.5f });

  // f=8 with 1x1 image -> wc = hc = 0 -> empty coarse -> fine fallback.
  float p99 = ComputeP99Y(xyz, 1, 1, 8);
  // Fine path on a single positive Y returns that value.
  EXPECT_FLOAT_EQ(p99, 7.5f);

  // Helper must report empty so the fallback branch in ComputeP99Y is exercised.
  EXPECT_TRUE(DownsampleBoxSumY(xyz, 1, 1, 8).empty());
}

}  // namespace
}  // namespace lumice::gui
