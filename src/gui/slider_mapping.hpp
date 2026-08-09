#ifndef LUMICE_GUI_SLIDER_MAPPING_HPP
#define LUMICE_GUI_SLIDER_MAPPING_HPP

// Pure-function slider mapping helpers, shared by panels.cpp (UI) and test code.
//
// This file owns the MAPPING (how a value becomes a [0,1] slider position). It does NOT own the
// DOMAIN each shape scalar is mapped over — the table that used to be written out here in prose,
// with a "call sites in edit_modals.cpp MUST match one of these rows" instruction that nothing
// could check, is now data in gui/shape_scalar_domain.hpp and is read by the row helper itself.
// Read that file for which slot gets which range / format / scale.
//
// kLinear rows use ImGui::SliderFloat directly; no mapping helper is defined here.
// The Pyramid prism_h kLogLinear mapping was introduced in tasks.md #145.3; keep
// kLogLinearX0 / kLogLinearTSwitch in sync with that tuning if either is adjusted.

#include <algorithm>
#include <cmath>

namespace lumice::gui::slider_mapping {

// LogLinear hybrid mapping constants. Tuned for prism_h [0, 100] range;
// re-validate before reusing for other ranges. REQUIRES: min_val == 0,
// max_val > kLogLinearX0 at call site.
constexpr float kLogLinearX0 = 0.01f;       // Value threshold: linear below, log above
constexpr float kLogLinearTSwitch = 0.15f;  // Slider position threshold (fraction of [0,1])

// Log-scale: compute normalized [0,1] position from value in [min_val, max_val].
inline float LogValueToNorm(float value, float min_val, float max_val) {
  value = std::max(value, min_val);
  float log_ratio = std::log(max_val / min_val);
  float norm = std::log(value / min_val) / log_ratio;
  return std::clamp(norm, 0.0f, 1.0f);
}

// Log-scale: compute value from normalized [0,1] position.
inline float LogNormToValue(float norm, float min_val, float max_val) {
  float log_ratio = std::log(max_val / min_val);
  return min_val * std::exp(norm * log_ratio);
}

// LogLinear hybrid: compute normalized [0,1] position from value in [0, max_val].
// Linear in [0, x0], log in [x0, max_val], C0 continuous at x0.
// NOTE: This pair (LogLinearValueToNorm / LogLinearNormToValue) is purpose-built
// for the min_val == 0 case; it is NOT a general LogLinear primitive with a
// configurable min. Do not pattern-match LogValueToNorm's (value, min, max) signature.
inline float LogLinearValueToNorm(float value, float max_val) {
  value = std::clamp(value, 0.0f, max_val);
  float log_ratio = std::log(max_val / kLogLinearX0);
  float norm = 0.0f;
  if (value <= kLogLinearX0) {
    norm = kLogLinearTSwitch * value / kLogLinearX0;
  } else {
    norm = kLogLinearTSwitch + (1.0f - kLogLinearTSwitch) * std::log(value / kLogLinearX0) / log_ratio;
  }
  return std::clamp(norm, 0.0f, 1.0f);
}

// LogLinear hybrid: compute value from normalized [0,1] position.
inline float LogLinearNormToValue(float norm, float max_val) {
  float log_ratio = std::log(max_val / kLogLinearX0);
  if (norm <= kLogLinearTSwitch) {
    return kLogLinearX0 * norm / kLogLinearTSwitch;
  }
  float t_log = (norm - kLogLinearTSwitch) / (1.0f - kLogLinearTSwitch);
  return kLogLinearX0 * std::exp(t_log * log_ratio);
}

}  // namespace lumice::gui::slider_mapping

#endif  // LUMICE_GUI_SLIDER_MAPPING_HPP
