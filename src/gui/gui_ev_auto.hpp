#ifndef LUMICE_GUI_EV_AUTO_HPP
#define LUMICE_GUI_EV_AUTO_HPP

#include <algorithm>
#include <cmath>
#include <vector>

namespace lumice::gui {

// Extract non-zero Y-channel values from a packed XYZ float array and return
// their P99.5 value.  Returns 0 if the array has no positive Y entries.
inline float ComputeP995Y(const std::vector<float>& xyz_data) {
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
  auto idx = static_cast<size_t>(static_cast<float>(y_vals.size()) * 0.995f);
  if (idx >= y_vals.size()) {
    idx = y_vals.size() - 1;
  }
  std::nth_element(y_vals.begin(), y_vals.begin() + static_cast<ptrdiff_t>(idx), y_vals.end());
  return y_vals[idx];
}

// Compute the P99.5-anchored auto-EV (in stops) such that the P99.5 normalised Y
// maps to target_white on the 0-255 sRGB scale.  Clamps to [-6, 6].
// Returns 0 if snapshot_intensity or p995_raw_y is non-positive.
inline float ComputeEvAuto(float p995_raw_y, float snapshot_intensity, float target_white) {
  if (snapshot_intensity <= 0.0f || p995_raw_y <= 0.0f) {
    return 0.0f;
  }
  float p99_norm = p995_raw_y / snapshot_intensity;
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
