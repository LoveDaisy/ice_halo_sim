#ifndef LUMICE_GUI_SUN_CIRCLE_RULES_HPP
#define LUMICE_GUI_SUN_CIRCLE_RULES_HPP

// The angular-distance ("sun circle") overlay's edit rules: when a preset may be added, when the
// list is full, and what an angle typed by hand is clamped to.
//
// These lived inside the Edit-Angles popup's draw loop, where the duplicate test in particular
// was a nested for-loop over the current list — reachable only by opening the popup in a live
// frame and clicking a preset. Extracted as pure functions of their arguments so the tolerance
// and the cap can be checked directly at their boundaries.
//
// Extracted verbatim: each body is the expression that stood at its call site, including the
// 0.01 tolerance and the [0.1, 180] clamp band.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "gui/gui_constants.hpp"  // kMaxSunCircles

namespace lumice::gui {

// Angles closer together than this read as the same circle, so offering to add one again would
// produce a visually identical duplicate.
constexpr float kSunCircleDuplicateEpsilonDeg = 0.01f;

// The list already holds `candidate` (within kSunCircleDuplicateEpsilonDeg).
inline bool SunCircleAlreadyPresent(const std::vector<float>& angles, float candidate) {
  for (float a : angles) {
    if (std::abs(a - candidate) < kSunCircleDuplicateEpsilonDeg) {
      return true;
    }
  }
  return false;
}

// The overlay's fixed-size upload buffer is full (preview_renderer.hpp sizes its array by
// kMaxSunCircles), so no further angle can be accepted.
inline bool SunCirclesAtLimit(std::size_t count) {
  return static_cast<int>(count) >= kMaxSunCircles;
}

// What the custom-angle input accepts. 0 is excluded (a zero-radius circle is not drawable) and
// 180 is the far side of the sky, so the band is closed on both ends.
inline float ClampSunCircleAngle(float degrees) {
  return std::max(0.1f, std::min(180.0f, degrees));
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SUN_CIRCLE_RULES_HPP
