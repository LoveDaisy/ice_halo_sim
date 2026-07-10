#ifndef LUMICE_GUI_COMPOSITE_EXPOSURE_PUSH_HPP
#define LUMICE_GUI_COMPOSITE_EXPOSURE_PUSH_HPP

// Pure-function guard for RenderPreviewPanel's per-frame display-time EV push
// (task-345.3, code-review round 1 Major #1 + Minor #1). Extracted out of
// app_panels.cpp so the guard's four branches (off->off / off->on / on->on
// changed / on->on unchanged) are independently unit-testable, rather than
// only reachable through a full ImGui frame.

#include <cmath>

namespace lumice::gui {

// Decides whether RenderPreviewPanel should call LUMICE_SetCompositeExposure
// this frame. Two independent conditions OR together:
//   (1) value guard    — ev_total changed by more than `epsilon` since the
//                         last push (or no push has happened yet, i.e.
//                         last_pushed_ev is NaN).
//   (2) off->on edge    — composite just became active this frame
//                         (composite_active && !last_composite_active).
//                         Must fire unconditionally on this edge even if
//                         ev_total numerically equals last_pushed_ev, or the
//                         first composite bake after enabling would use the
//                         server's stale display_ev_total_ (plan-review
//                         Minor #2).
// Returns false whenever composite_active is false — no push while the
// mono-only / CLI-like path is active.
inline bool ShouldPushCompositeExposure(bool composite_active, bool last_composite_active, float ev_total,
                                        float last_pushed_ev, float epsilon) {
  if (!composite_active) {
    return false;
  }
  const bool edge_on = !last_composite_active;
  const bool value_changed = std::isnan(last_pushed_ev) || std::fabs(ev_total - last_pushed_ev) > epsilon;
  return edge_on || value_changed;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_COMPOSITE_EXPOSURE_PUSH_HPP
