#ifndef LUMICE_TEST_SUPPORT_BLOCK_MEAN_PSNR_HPP_
#define LUMICE_TEST_SUPPORT_BLOCK_MEAN_PSNR_HPP_

// The ruler for a STOCHASTIC comparison between two independent Monte-Carlo renders of one scene:
// PSNR taken after both images are averaged down over block_w x block_h pixel blocks.
//
// Why not full-frame PSNR. When the two arms are independent simulations, the difference energy a
// full-frame PSNR is computed from is mostly the two arms' OWN noise, and a break that lowers one
// arm's noise can lower that energy too. Measured on the CLI<->GUI export parity fixture: darkening
// the CLI arm by 0.25 stop moved the full-frame PSNR UP by 0.18 dB — a coherent, whole-frame error
// read as an improvement — while a 4x4 block mean read the same break as -1.8 dB. Averaging over a
// block suppresses the per-pixel noise term roughly with the block's pixel count, leaving the
// coherent difference to set the number, so the ruler moves in the direction the break did.
//
// Why block-mean and NOT a deterministic frame's ruler: a block mean would hide exactly the small
// coherent blob support/pixel_diff_metrics.hpp exists to see. Two rulers for two kinds of frame.
//
// Single authority in C++. test/e2e/_parity_metrics.py::_block_mean is the offline twin used for
// calibration sweeps; it asserts that the dimensions divide by the block, which the parity
// fixture's 512x683 portrait canvas does not. This one takes the honest mean of whatever pixels
// the last row/column of blocks actually covers, dividing by THAT count — neither cropping (which
// drops real pixels) nor zero-padding (which pulls the mean toward black).

#include <cmath>
#include <limits>
#include <vector>

namespace lumice::test {

// PSNR in dB between two same-size interleaved 8-bit images after a block_w x block_h block mean
// over every channel. The mean is taken in double and never re-quantised. Returns +infinity when
// the two block-mean images are identical and -1.0 on a null pointer, a non-positive dimension, a
// non-positive block size, or a block larger than the image.
inline double ComputeBlockMeanPsnr(const unsigned char* a, const unsigned char* b, int w, int h, int channels,
                                   int block_w = 4, int block_h = 4) {
  if (!a || !b || w <= 0 || h <= 0 || channels <= 0 || block_w <= 0 || block_h <= 0 || block_w > w || block_h > h) {
    return -1.0;
  }
  const int bw = (w + block_w - 1) / block_w;
  const int bh = (h + block_h - 1) / block_h;
  double sq = 0.0;
  size_t n = 0;
  for (int by = 0; by < bh; ++by) {
    const int y0 = by * block_h;
    const int y1 = y0 + block_h < h ? y0 + block_h : h;
    for (int bx = 0; bx < bw; ++bx) {
      const int x0 = bx * block_w;
      const int x1 = x0 + block_w < w ? x0 + block_w : w;
      const double count = static_cast<double>(y1 - y0) * static_cast<double>(x1 - x0);
      for (int c = 0; c < channels; ++c) {
        double sa = 0.0;
        double sb = 0.0;
        for (int y = y0; y < y1; ++y) {
          for (int x = x0; x < x1; ++x) {
            const size_t i = (static_cast<size_t>(y) * w + x) * channels + c;
            sa += a[i];
            sb += b[i];
          }
        }
        const double d = (sa - sb) / count;
        sq += d * d;
        ++n;
      }
    }
  }
  const double mse = sq / static_cast<double>(n);
  if (mse == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return 10.0 * std::log10(255.0 * 255.0 / mse);
}

}  // namespace lumice::test

#endif  // LUMICE_TEST_SUPPORT_BLOCK_MEAN_PSNR_HPP_
