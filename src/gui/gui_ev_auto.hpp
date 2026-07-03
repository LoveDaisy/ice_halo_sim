#ifndef LUMICE_GUI_EV_AUTO_HPP
#define LUMICE_GUI_EV_AUTO_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lumice::gui {

// Auto-EV downsample factor for the box-sum coarse-bin metric.
// Rationale (scrum-auto-ev-77halo-followup, explore-infinite-accum-drift-truth):
// coarse bins have f^2 larger expected hit count than fine pixels in sparse
// scenes, so the P99-over-lit anchor stabilises earlier and 77halo previews
// brighten faster.  Math equivalence (E5/F7):
//   ev = log2(target_linear * snapshot_fine / (P99_coarse / f^2))
// Display path remains fine-res.  Final f=8 confirmed by e6 gold harness
// (22/25 in-band, only ms05_prob0.5_EV0.5/EV1.5 dropped vs 23/25 fine).
constexpr int kEvAutoDownsampleFactor = 8;

// Box-sum downsample of the Y channel from a packed XYZ buffer.
// Returns a flat coarse buffer of size (img_width/f) * (img_height/f); the
// trailing rows/cols that don't divide evenly are dropped (same rule as the
// Python reference `e6_gold_downsample.py`).  Returns an empty vector if the
// coarse dimensions collapse to zero so callers can fall back to the fine
// path.  Channel index 1 (Y), stride 3 (XYZ).
inline std::vector<float> DownsampleBoxSumY(const std::vector<float>& xyz_data, int img_width, int img_height, int f) {
  if (f <= 0 || img_width <= 0 || img_height <= 0) {
    return {};
  }
  int wc = img_width / f;
  int hc = img_height / f;
  if (wc <= 0 || hc <= 0) {
    return {};
  }
  std::vector<float> coarse(static_cast<size_t>(wc) * static_cast<size_t>(hc), 0.0f);
  for (int rc = 0; rc < hc; ++rc) {
    for (int cc = 0; cc < wc; ++cc) {
      float sum = 0.0f;
      int r0 = rc * f;
      int c0 = cc * f;
      for (int dr = 0; dr < f; ++dr) {
        size_t row_base = (static_cast<size_t>(r0 + dr) * static_cast<size_t>(img_width)) * 3;
        for (int dc = 0; dc < f; ++dc) {
          size_t idx = row_base + (static_cast<size_t>(c0 + dc) * 3) + 1;  // Y channel
          sum += xyz_data[idx];
        }
      }
      coarse[static_cast<size_t>(rc) * static_cast<size_t>(wc) + static_cast<size_t>(cc)] = sum;
    }
  }
  return coarse;
}

// See doc/ev-pipeline-architecture.md §2.2 (zero-skip semantics), §2.5 (GUI usage)
// Compute the P99 of the non-zero Y values in a packed XYZ buffer.
//
// When `downsample_factor > 1`, the Y channel is first box-summed onto a
// coarse grid (see DownsampleBoxSumY) and the P99 is taken over non-zero
// coarse bins, then divided by `downsample_factor^2` so the returned value
// is a **fine-equivalent P99** — feeding it straight into ComputeEvAuto with
// the fine `snapshot_intensity` reproduces the math equivalence:
//   ev = log2(target_linear * snapshot_fine / (P99_coarse / f^2))
// Therefore `TexturePayload.p99_y` no longer represents the true per-pixel Y
// statistic when downsample is active; downstream consumers must treat it
// only as the EV anchor and not as a raw Y measurement.
//
// Fallback order (must match for downstream invariants):
//   1) If `downsample_factor <= 1`           -> use the fine Y path.
//   2) Else if DownsampleBoxSumY returns {}  -> use the fine Y path.
//   3) Otherwise                             -> coarse Y path; if the coarse
//      buffer has no non-zero element, return 0.0f (same convention as the
//      fine path's empty case).
//
// Returns 0 if no positive Y entries exist (fine or coarse, by path).
inline float ComputeP99Y(const std::vector<float>& xyz_data, int img_width = 0, int img_height = 0,
                         int downsample_factor = 1) {
  if (downsample_factor > 1) {
    std::vector<float> coarse = DownsampleBoxSumY(xyz_data, img_width, img_height, downsample_factor);
    if (!coarse.empty()) {
      std::vector<float> y_vals;
      y_vals.reserve(coarse.size());
      for (float v : coarse) {
        if (v > 0.0f) {
          y_vals.push_back(v);
        }
      }
      if (y_vals.empty()) {
        return 0.0f;
      }
      auto idx = static_cast<size_t>(static_cast<float>(y_vals.size()) * 0.99f);
      if (idx >= y_vals.size()) {
        idx = y_vals.size() - 1;
      }
      std::nth_element(y_vals.begin(), y_vals.begin() + static_cast<ptrdiff_t>(idx), y_vals.end());
      float p99_coarse = y_vals[idx];
      float f2 = static_cast<float>(downsample_factor) * static_cast<float>(downsample_factor);
      return p99_coarse / f2;
    }
    // Fall through to fine path on empty coarse buffer.
  }
  std::vector<float> y_vals;
  y_vals.reserve(xyz_data.size() / 3);
  for (size_t i = 1; i < xyz_data.size(); i += 3) {
    if (xyz_data[i] > 0.0f) {
      y_vals.push_back(xyz_data[i]);
    }
  }
  if (y_vals.empty()) {
    return 0.0f;
  }
  auto idx = static_cast<size_t>(static_cast<float>(y_vals.size()) * 0.99f);
  if (idx >= y_vals.size()) {
    idx = y_vals.size() - 1;
  }
  std::nth_element(y_vals.begin(), y_vals.begin() + static_cast<ptrdiff_t>(idx), y_vals.end());
  return y_vals[idx];
}

// Compute the P99-anchored auto-EV (in stops) such that the P99 normalised Y
// maps to target_white on the 0-255 sRGB scale.  Clamps to [-6, 6].
// Returns 0 if snapshot_intensity or p99_raw_y is non-positive.
inline float ComputeEvAuto(float p99_raw_y, float snapshot_intensity, float target_white) {
  if (snapshot_intensity <= 0.0f || p99_raw_y <= 0.0f) {
    return 0.0f;
  }
  float p99_norm = p99_raw_y / snapshot_intensity;
  float t = target_white / 255.0f;
  float target_linear = t <= 0.04045f ? t / 12.92f : std::pow((t + 0.055f) / 1.055f, 2.4f);
  if (target_linear <= 0.0f || p99_norm <= 0.0f) {
    return 0.0f;
  }
  float ev = std::log2f(target_linear / p99_norm);
  return std::clamp(ev, -6.0f, 6.0f);
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EV_AUTO_HPP
