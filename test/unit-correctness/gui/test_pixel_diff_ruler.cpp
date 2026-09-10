// The deterministic screenshot ruler (test/support/pixel_diff_metrics.hpp), pinned on the two
// drift shapes the old ruler let through.
//
// The proposition this file exists for is a red/green pair on ONE input: a frame and the same
// frame with a small coherent block moved in it. The old ruler — full-frame PSNR against a 40 dB
// floor — reads that pair as green; the new one reads it as red. Both verdicts are asserted here,
// side by side, so the file says not just "the new ruler fires" but "the new ruler fires where the
// old one did not", which is the whole reason it was introduced. The shapes are the minimal
// synthetic forms of two real drifts this suite missed: a 4x23-px scrollbar thumb that moved (90
// differing pixels, 53.45 dB) and a text row that changed (127 px, 42.6 dB). The frames are
// synthesised rather than committed: 760x584, the settings panel's capture size, a flat background
// and one block of |delta| = 50 — small, single-blob, high-PSNR, i.e. exactly the regime PSNR
// averages away.
//
// The remaining cases pin the ruler's own semantics at the edges a calibration depends on: that
// tau is a strict "greater than", that the components are 8-connected (a diagonal touch joins),
// that alpha never counts, and that a whole-frame fill one level apart — the llvmpipe-vs-Metal
// noise the tau exists to remove — dissolves at tau >= 1 while a real block survives it.
//
// Header-only and free of GL/ImGui, hence unit_correctness_test rather than gui_test.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "support/pixel_diff_metrics.hpp"

namespace {

constexpr int kW = 760;
constexpr int kH = 584;
constexpr int kCh = 3;

// The retired rule, restated locally as the control arm: full-frame PSNR over every channel,
// 8-bit MSE, exactly as test/gui/test_screenshot.cpp::ComputePsnr computes it. The number this
// ruler used to be held to was a 40 dB floor.
constexpr double kOldRulerFloorDb = 40.0;

double OldRulerPsnrDb(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
  double mse = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
    mse += d * d;
  }
  mse /= static_cast<double>(a.size());
  return mse == 0.0 ? INFINITY : 10.0 * std::log10(255.0 * 255.0 / mse);
}

std::vector<unsigned char> FlatFrame(unsigned char v, int channels = kCh) {
  return std::vector<unsigned char>(static_cast<size_t>(kW) * kH * channels, v);
}

// Adds `delta` to every channel of the w x h block whose top-left is (x0, y0).
void PaintBlock(std::vector<unsigned char>& img, int x0, int y0, int w, int h, int delta, int channels = kCh) {
  ASSERT_TRUE(x0 + w <= kW && y0 + h <= kH) << "block leaves the frame";
  for (int y = y0; y < y0 + h; ++y) {
    for (int x = x0; x < x0 + w; ++x) {
      for (int c = 0; c < channels; ++c) {
        img[(static_cast<size_t>(y) * kW + x) * channels + c] =
            static_cast<unsigned char>(img[(static_cast<size_t>(y) * kW + x) * channels + c] + delta);
      }
    }
  }
}

}  // namespace

// The dcfa5f9e shape: a 4x23 scrollbar thumb one row over. n_diff = maxcc = 92 (the real pair
// measured 90 because two rows of the moved thumb overlapped its old position).
TEST(PixelDiffRuler, ScrollbarThumbBlockIsRedForNewRulerGreenForOld) {
  const auto a = FlatFrame(60);
  auto b = a;
  PaintBlock(b, 744, 356, 4, 23, 50);

  const auto r = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, /*tau=*/0);
  EXPECT_EQ(r.n_diff, 92);
  EXPECT_EQ(r.max_cc, 92);
  EXPECT_EQ(r.dmax, 50);

  // Old ruler: green. 92 px of |delta| = 50 in a 443,840-px frame is ~51 dB, well over the floor.
  const double psnr = OldRulerPsnrDb(a, b);
  EXPECT_GT(psnr, kOldRulerFloorDb);
  EXPECT_LT(psnr, 60.0);  // and not so high that the assertion above is vacuous

  // New ruler: red under every K this tree ships (the loosest is 70).
  constexpr lumice::test::MaxCcRuler kLoosestShipped{ 16, 70 };
  const auto r16 = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, kLoosestShipped.tau);
  EXPECT_GT(r16.max_cc, kLoosestShipped.max_cc_threshold);
}

// The 0a191bea shape: a text row changed, 127 differing pixels in one blob. Text is not a solid
// rectangle, so the blob is a 13x10 bounding box with 3 pixels missing — still one component.
TEST(PixelDiffRuler, TextRowBlockIsRedForNewRulerGreenForOld) {
  const auto a = FlatFrame(60);
  auto b = a;
  PaintBlock(b, 52, 418, 13, 10, 50);
  // Punch three holes so the blob is 127 px rather than 130; they must not split it.
  for (int x : { 55, 58, 61 }) {
    for (int c = 0; c < kCh; ++c) {
      b[(static_cast<size_t>(422) * kW + x) * kCh + c] = 60;
    }
  }

  const auto r = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, /*tau=*/0);
  EXPECT_EQ(r.n_diff, 127);
  EXPECT_EQ(r.max_cc, 127);
  EXPECT_GT(OldRulerPsnrDb(a, b), kOldRulerFloorDb);
  EXPECT_GT(lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, 16).max_cc, 70);
}

TEST(PixelDiffRuler, IdenticalFramesMeasureZero) {
  const auto a = FlatFrame(60);
  const auto r = lumice::test::ComputePixelDiff(a.data(), a.data(), kW, kH, kCh, 0);
  EXPECT_EQ(r.n_diff, 0);
  EXPECT_EQ(r.max_cc, 0);
  EXPECT_EQ(r.dmax, 0);
}

// tau is strict: a pixel differs when max-channel |delta| > tau, so |delta| == tau does not count.
TEST(PixelDiffRuler, TauIsStrictGreaterThan) {
  const auto a = FlatFrame(60);
  auto b = a;
  PaintBlock(b, 10, 10, 5, 5, 16);
  EXPECT_EQ(lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, 15).n_diff, 25);
  EXPECT_EQ(lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, 16).n_diff, 0);
}

// The llvmpipe noise shape: a whole header bar one quantisation level apart joins into one huge
// blob at tau = 0 (the measured 8288–9380-px case) and vanishes at tau = 1, while a real |delta| =
// 50 block in the same frame is unaffected by tau in that range.
TEST(PixelDiffRuler, OneLevelFillDissolvesAtTauOneWhileRealBlockSurvives) {
  const auto a = FlatFrame(60);
  auto b = a;
  PaintBlock(b, 300, 173, 400, 21, 1);  // a full-width header row, delta = 1
  PaintBlock(b, 744, 356, 4, 23, 50);   // the scrollbar thumb, delta = 50

  const auto r0 = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, 0);
  EXPECT_EQ(r0.n_diff, 400 * 21 + 92);
  EXPECT_EQ(r0.max_cc, 400 * 21);
  const auto r1 = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, 1);
  EXPECT_EQ(r1.n_diff, 92);
  EXPECT_EQ(r1.max_cc, 92);
}

// Two blobs that touch only at a corner are one 8-connected component; separated by one clear
// pixel, they are two, and max_cc reports the larger.
TEST(PixelDiffRuler, ComponentsAreEightConnected) {
  const auto a = FlatFrame(60);
  auto b = a;
  PaintBlock(b, 100, 100, 3, 3, 50);
  PaintBlock(b, 103, 103, 2, 2, 50);  // corner-touching: (102,102) and (103,103) are diagonal
  auto r = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, 0);
  EXPECT_EQ(r.n_diff, 13);
  EXPECT_EQ(r.max_cc, 13);

  auto c = a;
  PaintBlock(c, 100, 100, 3, 3, 50);
  PaintBlock(c, 104, 104, 2, 2, 50);  // one clear pixel between: two components
  r = lumice::test::ComputePixelDiff(a.data(), c.data(), kW, kH, kCh, 0);
  EXPECT_EQ(r.n_diff, 13);
  EXPECT_EQ(r.max_cc, 9);
}

// Only the first three channels are compared, so an RGBA capture whose alpha differs from the
// reference's measures identical — the comparison is of what is on screen.
TEST(PixelDiffRuler, AlphaChannelIsIgnored) {
  const auto a = FlatFrame(60, 4);
  auto b = a;
  for (size_t p = 0; p < static_cast<size_t>(kW) * kH; ++p) {
    b[p * 4 + 3] = 0;
  }
  const auto r = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, 4, 0);
  EXPECT_EQ(r.n_diff, 0);
  EXPECT_EQ(r.max_cc, 0);
}

TEST(PixelDiffRuler, RejectsNullAndEmptyInput) {
  const auto a = FlatFrame(60);
  EXPECT_EQ(lumice::test::ComputePixelDiff(nullptr, a.data(), kW, kH, kCh, 0).n_diff, -1);
  EXPECT_EQ(lumice::test::ComputePixelDiff(a.data(), a.data(), 0, kH, kCh, 0).max_cc, -1);
}

// The mask entry point shares the flood fill with the tau entry point rather than carrying a
// second one. Pinned by the one input on which the two must agree exactly: a mask that IS the
// tau = 0 difference mask of a pair. Two blobs of different sizes and a diagonal join, so a
// second implementation that got connectivity or ranking subtly wrong would read differently.
TEST(PixelDiffRuler, MaskEntryAgreesWithTauEntryOnTheTauZeroMask) {
  const auto a = FlatFrame(60);
  auto b = a;
  PaintBlock(b, 100, 100, 3, 3, 50);
  PaintBlock(b, 103, 103, 2, 2, 50);  // diagonal touch: joins the 3x3 into one 13-px blob
  PaintBlock(b, 300, 300, 5, 4, 7);   // a separate 20-px blob, |delta| well under the other's
  PaintBlock(b, 700, 500, 1, 1, 1);   // a lone pixel

  std::vector<unsigned char> mask(static_cast<size_t>(kW) * kH, 0);
  for (size_t p = 0; p < mask.size(); ++p) {
    for (int c = 0; c < kCh; ++c) {
      if (a[p * kCh + c] != b[p * kCh + c]) {
        mask[p] = 1;
      }
    }
  }
  const auto from_tau = lumice::test::ComputePixelDiff(a.data(), b.data(), kW, kH, kCh, /*tau=*/0);
  const auto from_mask = lumice::test::ComputePixelDiffFromMask(mask, kW, kH);
  EXPECT_EQ(from_tau.n_diff, 34);
  EXPECT_EQ(from_tau.max_cc, 20);
  EXPECT_EQ(from_mask.n_diff, from_tau.n_diff);
  EXPECT_EQ(from_mask.max_cc, from_tau.max_cc);
  EXPECT_EQ(from_mask.dmax, 0);  // a mask carries no per-channel delta
  // Any non-zero byte is a set pixel, not just 1.
  for (auto& m : mask) {
    m = m ? 255 : 0;
  }
  EXPECT_EQ(lumice::test::ComputePixelDiffFromMask(mask, kW, kH).max_cc, 20);
}

TEST(PixelDiffRuler, MaskEntryRejectsAMaskOfTheWrongSize) {
  const std::vector<unsigned char> mask(static_cast<size_t>(kW) * kH - 1, 0);
  EXPECT_EQ(lumice::test::ComputePixelDiffFromMask(mask, kW, kH).n_diff, -1);
  const std::vector<unsigned char> empty;
  EXPECT_EQ(lumice::test::ComputePixelDiffFromMask(empty, 0, 0).max_cc, -1);
}
