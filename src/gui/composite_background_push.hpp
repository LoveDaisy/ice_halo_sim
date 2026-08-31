#ifndef LUMICE_GUI_COMPOSITE_BACKGROUND_PUSH_HPP
#define LUMICE_GUI_COMPOSITE_BACKGROUND_PUSH_HPP

// Pure-function guard for RenderPreviewPanel's per-frame display-time background push, the sibling
// of composite_exposure_push.hpp. Extracted out of app_panels.cpp for the same reason: the guard's
// branches (off->off / off->on / on->on changed / on->on unchanged) are then independently
// unit-testable rather than only reachable through a full ImGui frame.

#include <cmath>

namespace lumice::gui {

// Decides whether RenderPreviewPanel should call LUMICE_SetCompositeBackground this frame.
// `background_linear` and `last_pushed` are 3-component linear RGB. Two independent conditions
// OR together, exactly as in ShouldPushCompositeExposure:
//   (1) value guard  — any component moved by more than `epsilon` since the last push (or no push
//                      has happened yet, i.e. last_pushed holds NaN).
//   (2) off->on edge — composite just became active this frame (composite_active &&
//                      !last_composite_active). This must fire even when the value is numerically
//                      unchanged. The server's stored background is only ever written while
//                      composite is active, so a colour edit made while it was off never reached
//                      it: without the edge, the first bake after re-enabling would composite the
//                      stale colour while the mono path already shows the new one — the exact
//                      cross-path mismatch this whole feature exists to remove.
// Returns false whenever composite_active is false — no push while the mono-only / CLI-like path
// is active.
inline bool ShouldPushCompositeBackground(bool composite_active, bool last_composite_active,
                                          const float background_linear[3], const float last_pushed[3], float epsilon) {
  if (!composite_active) {
    return false;
  }
  const bool edge_on = !last_composite_active;
  bool value_changed = false;
  for (int j = 0; j < 3; ++j) {
    if (std::isnan(last_pushed[j]) || std::fabs(background_linear[j] - last_pushed[j]) > epsilon) {
      value_changed = true;
      break;
    }
  }
  return edge_on || value_changed;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_COMPOSITE_BACKGROUND_PUSH_HPP
