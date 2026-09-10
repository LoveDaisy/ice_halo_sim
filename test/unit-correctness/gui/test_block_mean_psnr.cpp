// The stochastic comparison ruler (test/support/block_mean_psnr.hpp), pinned at the two edges a
// calibration depends on: that the block mean is a true mean (a constant pair reads infinity, a
// known step reads the hand-computed dB), and that a canvas whose height does not divide by the
// block — the parity fixture's 512x683 portrait — is neither cropped nor zero-padded, i.e. a
// difference confined to the last three rows is still counted, at the weight of the pixels it
// actually covers.
//
// Header-only and free of GL/ImGui, hence unit_correctness_test rather than gui_test.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "support/block_mean_psnr.hpp"

namespace {

std::vector<unsigned char> Flat(int w, int h, int channels, unsigned char v) {
  return std::vector<unsigned char>(static_cast<size_t>(w) * h * channels, v);
}

double PsnrFromMse(double mse) {
  return 10.0 * std::log10(255.0 * 255.0 / mse);
}

}  // namespace

TEST(BlockMeanPsnr, IdenticalImagesReadInfinity) {
  const auto a = Flat(8, 8, 3, 90);
  EXPECT_TRUE(std::isinf(lumice::test::ComputeBlockMeanPsnr(a.data(), a.data(), 8, 8, 3)));
}

// One 4x4 block of 16 differs by 10 in every channel: the block-mean MSE is 100 / 16 and the
// number is the hand-computed one — no quantisation back to 8 bits in between.
TEST(BlockMeanPsnr, OneWholeBlockStepReadsTheHandComputedValue) {
  const int w = 16;
  const int h = 16;
  const auto a = Flat(w, h, 3, 100);
  auto b = a;
  for (int y = 4; y < 8; ++y) {
    for (int x = 8; x < 12; ++x) {
      for (int c = 0; c < 3; ++c) {
        b[(static_cast<size_t>(y) * w + x) * 3 + c] = 110;
      }
    }
  }
  const double expected = PsnrFromMse(100.0 / 16.0);
  EXPECT_NEAR(lumice::test::ComputeBlockMeanPsnr(a.data(), b.data(), w, h, 3), expected, 1e-9);
}

// A per-pixel checkerboard of +-d inside a block averages to zero: the ruler is BLIND to it by
// design (that is the noise term it exists to suppress), and the case states so explicitly rather
// than leaving it to be discovered when a real noise pattern reads as agreement.
TEST(BlockMeanPsnr, ZeroMeanNoiseInsideABlockAveragesAway) {
  const int w = 8;
  const int h = 8;
  const auto a = Flat(w, h, 1, 100);
  auto b = a;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      b[static_cast<size_t>(y) * w + x] = static_cast<unsigned char>(((x + y) % 2) ? 100 + 20 : 100 - 20);
    }
  }
  EXPECT_TRUE(std::isinf(lumice::test::ComputeBlockMeanPsnr(a.data(), b.data(), w, h, 1)));
}

// 512x683: 683 = 4 * 170 + 3, so the last row of blocks is 3 px tall. A step of 40 confined to
// those 3 rows must be counted, and counted as a mean over 12 pixels, not 16 (zero padding would
// read 30) and not dropped (cropping would read infinity). 128 blocks of the 171 * 128 carry it.
TEST(BlockMeanPsnr, LastPartialRowIsAveragedOverThePixelsItCovers) {
  const int w = 512;
  const int h = 683;
  const auto a = Flat(w, h, 3, 50);
  auto b = a;
  for (int y = 680; y < 683; ++y) {
    for (int x = 0; x < w; ++x) {
      for (int c = 0; c < 3; ++c) {
        b[(static_cast<size_t>(y) * w + x) * 3 + c] = 90;
      }
    }
  }
  const double n_blocks = 171.0 * 128.0;
  const double expected = PsnrFromMse(40.0 * 40.0 * 128.0 / n_blocks);
  const double got = lumice::test::ComputeBlockMeanPsnr(a.data(), b.data(), w, h, 3);
  EXPECT_NEAR(got, expected, 1e-9);
  EXPECT_FALSE(std::isinf(got));
  // And the two wrong treatments read differently, so the assertion above is not vacuous.
  EXPECT_GT(std::fabs(got - PsnrFromMse(30.0 * 30.0 * 128.0 / n_blocks)), 0.1);
}

// Same on the width, with a block that divides neither axis, and both partial edges at once.
TEST(BlockMeanPsnr, PartialCornerBlockUsesItsOwnPixelCount) {
  const int w = 7;
  const int h = 5;  // 4x4 blocks -> 2x2 grid; the corner block is 3x1
  const auto a = Flat(w, h, 1, 0);
  auto b = a;
  b[static_cast<size_t>(4) * w + 6] = 30;  // one pixel of the 3-px corner block
  const double expected = PsnrFromMse((30.0 / 3.0) * (30.0 / 3.0) / 4.0);
  EXPECT_NEAR(lumice::test::ComputeBlockMeanPsnr(a.data(), b.data(), w, h, 1), expected, 1e-9);
}

TEST(BlockMeanPsnr, RejectsBadInput) {
  const auto a = Flat(8, 8, 3, 0);
  EXPECT_EQ(lumice::test::ComputeBlockMeanPsnr(nullptr, a.data(), 8, 8, 3), -1.0);
  EXPECT_EQ(lumice::test::ComputeBlockMeanPsnr(a.data(), a.data(), 8, 8, 3, /*block_w=*/0), -1.0);
  EXPECT_EQ(lumice::test::ComputeBlockMeanPsnr(a.data(), a.data(), 8, 8, 3, 4, 9), -1.0);
}
