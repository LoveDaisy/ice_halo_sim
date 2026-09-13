#ifndef LUMICE_GUI_ANGULAR_DIST_RULES_HPP
#define LUMICE_GUI_ANGULAR_DIST_RULES_HPP

// The angular-distance overlays' edit rules: when a preset may be added, when the list is full,
// and what an angle typed by hand is clamped to.
//
// One rule set, two callers: the sun-centred family (`sun_circle_angles`) and the axis-centred one
// (`view_dist_angles`) are the same list of ring radii with a different centre, so each family's
// popup calls these same three functions on its own vector. The file was born as the sun circles'
// and carried that name while it was their only caller; it is named for the shared semantics now
// that the panel groups both families under one "Angular Distance" section.
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

#include "gui/gui_constants.hpp"  // kMaxAnnotationCircles

namespace lumice::gui {

// Angles closer together than this read as the same circle, so offering to add one again would
// produce a visually identical duplicate.
constexpr float kAngularDistCircleDuplicateEpsilonDeg = 0.01f;

// The list already holds `candidate` (within kAngularDistCircleDuplicateEpsilonDeg).
inline bool AngularDistCircleAlreadyPresent(const std::vector<float>& angles, float candidate) {
  for (float a : angles) {
    if (std::abs(a - candidate) < kAngularDistCircleDuplicateEpsilonDeg) {
      return true;
    }
  }
  return false;
}

// The overlay's fixed-size upload buffer is full (preview_renderer.hpp sizes its array by
// kMaxAnnotationCircles), so no further angle can be accepted.
inline bool AngularDistCirclesAtLimit(std::size_t count) {
  return static_cast<int>(count) >= kMaxAnnotationCircles;
}

// What the custom-angle input accepts. 0 is excluded (a zero-radius circle is not drawable) and
// 180 is the far side of the sky, so the band is closed on both ends.
inline float ClampAngularDistCircleAngle(float degrees) {
  return std::max(0.1f, std::min(180.0f, degrees));
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_ANGULAR_DIST_RULES_HPP
