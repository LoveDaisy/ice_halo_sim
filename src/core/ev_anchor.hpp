#ifndef CORE_EV_ANCHOR_H_
#define CORE_EV_ANCHOR_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lumice {

// Single owner of the P99 partial-sort step (idx = floor(size * 0.99), nth_element).
// This exact code used to be inlined in three places: the coarse and fine paths of ComputeP99Y
// (which lived in the GUI layer) and component_compositor.cpp's ComputeParticipatingP99Y.
// Callers own the "which values participate" question — this function only answers "what is the
// P99 of the values you handed me".
//
// `values` is reordered in place (nth_element is a partial sort). Returns 0 for an empty range,
// the same convention every caller already used for "no positive samples".
inline float NthElementP99(std::vector<float>& values) {
  if (values.empty()) {
    return 0.0f;
  }
  auto idx = static_cast<size_t>(static_cast<float>(values.size()) * 0.99f);
  if (idx >= values.size()) {
    idx = values.size() - 1;
  }
  std::nth_element(values.begin(), values.begin() + static_cast<ptrdiff_t>(idx), values.end());
  return values[idx];
}

// Single owner of the sRGB reverse transform used to turn a target_white on the 0-255 scale into
// its linear equivalent. Shared by ComputeEvAuto (below) and
// RenderConsumer::ParticipatingExposureScale (server/render.cpp) — the two consumers of the
// anchor share this level and the P99 level, and NOTHING beyond that: their final expressions
// differ (the composite one carries no snapshot_intensity in its numerator), see render.cpp.
inline float TargetWhiteToLinear(float target_white) {
  float t = target_white / 255.0f;
  return t <= 0.04045f ? t / 12.92f : std::pow((t + 0.055f) / 1.055f, 2.4f);
}

// Box-sum downsample of the Y channel from a packed XYZ buffer.
// Returns a flat coarse buffer of size (img_width/f) * (img_height/f); the
// trailing rows/cols that don't divide evenly are dropped (same rule as the
// Python reference `e6_gold_downsample.py`).  Returns an empty vector if the
// coarse dimensions collapse to zero so callers can fall back to the fine
// path.
//
// `channel_stride` / `y_offset` name where Y sits: the default (3, 1) is a packed XYZ
// image, which is what every render buffer is. The exposure anchor plane passes (1, 0)
// because it stores Y ALONE — it is a measurement, not an image, and nothing downstream
// of it ever reads an X or a Z. Two parameters rather than one derived from the other
// because "stride 1 implies offset 0" is a coincidence of today's two callers, not a rule.
//
// PRECONDITION: `xyz_data` is a borrowed view of at least
// img_width*img_height*channel_stride floats. A raw pointer carries no length, so the
// caller's dimensions are the only bound this function has — exactly as before, since the
// vector overload this replaced never consulted `.size()` either.
inline std::vector<float> DownsampleBoxSumY(const float* xyz_data, int img_width, int img_height, int f,
                                            int channel_stride = 3, int y_offset = 1) {
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
        size_t row_base =
            (static_cast<size_t>(r0 + dr) * static_cast<size_t>(img_width)) * static_cast<size_t>(channel_stride);
        for (int dc = 0; dc < f; ++dc) {
          size_t idx = row_base + (static_cast<size_t>(c0 + dc) * static_cast<size_t>(channel_stride)) +
                       static_cast<size_t>(y_offset);
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
// The coarse and the fine path are NOT two precisions of one statistic — measured on the 77halo
// 1e7 fixture they differ by 64x (coarse 3.28e-4 vs fine 2.10e-2) and respond to N with different
// slopes (+0.329 vs +0.034). Collapsing them into one path would silently shift sparse-scene
// auto-EV by ~6 stops. The mono path picks coarse (f=8), the composite path picks fine; both
// choices are the caller's to make.
//
// Fallback order (must match for downstream invariants):
//   1) If `downsample_factor <= 1`           -> use the fine Y path.
//   2) Else if DownsampleBoxSumY returns {}  -> use the fine Y path.
//   3) Otherwise                             -> coarse Y path; if the coarse
//      buffer has no non-zero element, return 0.0f (same convention as the
//      fine path's empty case).
//
// Returns 0 if no positive Y entries exist (fine or coarse, by path).
//
// PRECONDITION (same as DownsampleBoxSumY): `xyz_data` views at least
// img_width*img_height*channel_stride floats. img_width/img_height are required rather than
// defaulted — a raw pointer carries no length, so there is no buffer to fall back to measuring.
// `channel_stride` / `y_offset` default to a packed XYZ image; see DownsampleBoxSumY for the
// one caller that does not have one.
inline float ComputeP99Y(const float* xyz_data, int img_width, int img_height, int downsample_factor = 1,
                         int channel_stride = 3, int y_offset = 1) {
  if (downsample_factor > 1) {
    std::vector<float> coarse =
        DownsampleBoxSumY(xyz_data, img_width, img_height, downsample_factor, channel_stride, y_offset);
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
      float p99_coarse = NthElementP99(y_vals);
      float f2 = static_cast<float>(downsample_factor) * static_cast<float>(downsample_factor);
      return p99_coarse / f2;
    }
    // Fall through to fine path on empty coarse buffer.
  }
  // Non-positive dims yield an empty range, matching what the retired `xyz_data.size()` bound did
  // for an empty buffer. The explicit test is not decoration: unlike a vector's size(), a negative
  // int silently becomes an enormous size_t, and this loop has no other bound.
  const size_t float_count =
      (img_width > 0 && img_height > 0) ?
          static_cast<size_t>(img_width) * static_cast<size_t>(img_height) * static_cast<size_t>(channel_stride) :
          0;
  std::vector<float> y_vals;
  y_vals.reserve(float_count / static_cast<size_t>(channel_stride));
  for (size_t i = static_cast<size_t>(y_offset); i < float_count; i += static_cast<size_t>(channel_stride)) {
    if (xyz_data[i] > 0.0f) {
      y_vals.push_back(xyz_data[i]);
    }
  }
  if (y_vals.empty()) {
    return 0.0f;
  }
  return NthElementP99(y_vals);
}

// The two numbers the P99 self-anchor is parameterised by.
//
// WHO READS WHICH, as of the two-sided anchor adoption. `kAnchorTargetWhite` still has two
// consumers on this side (ExposureScale's kRelative branch and ParticipatingExposureScale), and
// the GUI mirrors it in GuiState::target_white. `kMonoAnchorDownsampleFactor` now has exactly
// ONE: `anchor_buffer.hpp::kAnchorDownsampleFactor`, which aliases it for the full-sky anchor
// plane. The call site that used to be its second consumer — a P99 over the renderer's own
// output buffer, inside ExposureScale — is gone with the per-view anchor it belonged to. The
// constant is not folded into anchor_buffer.hpp all the same: that header pairs it with the
// anchor's RESOLUTION and says why the two only move together, which is an argument that needs
// the box-sum factor to be a name rather than an 8.
//
// The GUI mirrors both, not shares them: `src/gui/` may not include `core/`, so
// gui_constants.hpp::kEvAutoDownsampleFactor and GuiState::target_white hold the same two values
// independently. That mirroring predates this owner and is not removed by it — the API boundary
// is what forbids sharing. Note that the GUI's copy of the downsample factor is now unread by
// the mono exposure path for the same reason core's second consumer went: the GUI reads the
// server's anchor instead of computing one.
//
// f=8 is the MONO path's choice specifically. Coarse and fine are not two precisions of one
// statistic (see ComputeP99Y's comment: 64x apart on the 77halo fixture), so the composite path
// deliberately keeps f=1 and must not be "unified" onto this constant.
constexpr int kMonoAnchorDownsampleFactor = 8;
constexpr float kAnchorTargetWhite = 135.0f;

// Compute the P99-anchored auto-EV (in stops) such that the P99 normalised Y
// maps to target_white on the 0-255 sRGB scale.  Clamps to [-6, 6].
// Returns 0 if snapshot_intensity or p99_raw_y is non-positive.
inline float ComputeEvAuto(float p99_raw_y, float snapshot_intensity, float target_white) {
  if (snapshot_intensity <= 0.0f || p99_raw_y <= 0.0f) {
    return 0.0f;
  }
  float p99_norm = p99_raw_y / snapshot_intensity;
  float target_linear = TargetWhiteToLinear(target_white);
  if (target_linear <= 0.0f || p99_norm <= 0.0f) {
    return 0.0f;
  }
  float ev = std::log2f(target_linear / p99_norm);
  return std::clamp(ev, -6.0f, 6.0f);
}

}  // namespace lumice

#endif  // CORE_EV_ANCHOR_H_
