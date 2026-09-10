#ifndef LUMICE_TEST_SUPPORT_PIXEL_DIFF_METRICS_HPP_
#define LUMICE_TEST_SUPPORT_PIXEL_DIFF_METRICS_HPP_

// The ruler for DETERMINISTIC screenshot comparisons: differing-pixel count plus the largest
// 8-connected blob of differing pixels.
//
// Why not PSNR. A deterministic frame (edit modal, settings panel, crystal preview, harness smoke
// capture) carries no noise, so any spatially coherent difference is a semantic change — a moved
// scrollbar thumb, a re-flowed text row, a widget one pixel over. PSNR is an energy average over the
// whole frame and is structurally blind to a coherent block that covers 0.1% of it: two real drifts
// in this tree passed the 40 dB floor at 42.6 dB (a 127-px text row) and 53.45 dB (a 4x23 px
// scrollbar thumb, 90 px), and both were only found by eye. Counting pixels, and measuring the
// largest blob they form, sees exactly what PSNR averages away.
//
// Why a threshold `tau` at all, given "no noise". The reference machine renders every one of these
// frames byte-identically run to run (n_diff = 0), but CI runs them under Mesa's llvmpipe against
// references shot on Metal, and that pair differs in two ways that are not semantic: flat fills
// one quantisation level apart (|delta| <= 2, but spanning whole header bars and scrollbar tracks,
// so at tau = 0 they join into blobs of 245-9380 px), and anti-aliasing coverage at glyph and line
// edges (|delta| up to ~98, but in small disconnected specks). `tau` removes the first kind
// outright and shrinks the second; the per-group `max_cc_threshold` then sits between what the
// second kind leaves and the smallest real drift on record. The values in use, the measurements
// behind them and their margins are in doc/testing-architecture.md §4.6.
//
// Single authority. This header is the only implementation of the ruler in C++: the gui_test
// comparison (test/gui/test_screenshot.cpp) and the unit case that pins the ruler's own behaviour
// (test/unit-correctness/gui/test_pixel_diff_ruler.cpp) both include it. It is header-only, pure
// C++17 and free of GL/ImGui/stb so the unit target can link it without a window. An offline Python
// twin exists for calibration sweeps and was checked against this one sample-by-sample before the
// thresholds were chosen; keep the two in step if either changes.

#include <cstdlib>
#include <utility>
#include <vector>

namespace lumice::test {

// Which ruler a deterministic comparison is held to: pixels whose max-channel |delta| exceeds
// `tau` are "different"; the comparison fails when the largest 8-connected blob of them has more
// than `max_cc_threshold` pixels. tau = 0, max_cc_threshold = 0 demands byte-identity.
struct MaxCcRuler {
  int tau;
  int max_cc_threshold;
};

struct PixelDiffResult {
  int n_diff;  // pixels whose max-channel |delta| > tau
  int max_cc;  // size of the largest 8-connected component of those pixels (0 when n_diff == 0)
  int dmax;    // largest max-channel |delta| anywhere in the frame (0 when identical)
};

// Compares two same-size interleaved 8-bit images. `channels` may be 3 or 4; when it is 4 the
// alpha channel is ignored, so an RGBA capture and an RGB reference measure the same. Returns
// {-1, -1, -1} on a null pointer or a non-positive dimension.
inline PixelDiffResult ComputePixelDiff(const unsigned char* a, const unsigned char* b, int w, int h, int channels,
                                        int tau) {
  if (!a || !b || w <= 0 || h <= 0 || channels <= 0) {
    return { -1, -1, -1 };
  }
  const int cmp_ch = channels < 3 ? channels : 3;
  const size_t n_px = static_cast<size_t>(w) * static_cast<size_t>(h);

  std::vector<unsigned char> mask(n_px, 0);
  PixelDiffResult r{ 0, 0, 0 };
  for (size_t p = 0; p < n_px; ++p) {
    int d = 0;
    for (int c = 0; c < cmp_ch; ++c) {
      const int v = std::abs(static_cast<int>(a[p * channels + c]) - static_cast<int>(b[p * channels + c]));
      if (v > d) {
        d = v;
      }
    }
    if (d > r.dmax) {
      r.dmax = d;
    }
    if (d > tau) {
      mask[p] = 1;
      ++r.n_diff;
    }
  }
  if (r.n_diff == 0) {
    return r;
  }

  // 8-connected flood fill over the mask; `mask` doubles as the visited set (cleared on visit).
  std::vector<std::pair<int, int>> stack;
  for (int y0 = 0; y0 < h; ++y0) {
    for (int x0 = 0; x0 < w; ++x0) {
      if (!mask[static_cast<size_t>(y0) * w + x0]) {
        continue;
      }
      int size = 0;
      mask[static_cast<size_t>(y0) * w + x0] = 0;
      stack.emplace_back(x0, y0);
      while (!stack.empty()) {
        const auto [x, y] = stack.back();
        stack.pop_back();
        ++size;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int xx = x + dx;
            const int yy = y + dy;
            if (xx < 0 || yy < 0 || xx >= w || yy >= h) {
              continue;
            }
            unsigned char& m = mask[static_cast<size_t>(yy) * w + xx];
            if (m) {
              m = 0;
              stack.emplace_back(xx, yy);
            }
          }
        }
      }
      if (size > r.max_cc) {
        r.max_cc = size;
      }
    }
  }
  return r;
}

}  // namespace lumice::test

#endif  // LUMICE_TEST_SUPPORT_PIXEL_DIFF_METRICS_HPP_
