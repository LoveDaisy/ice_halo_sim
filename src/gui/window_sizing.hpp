#ifndef LUMICE_GUI_WINDOW_SIZING_HPP
#define LUMICE_GUI_WINDOW_SIZING_HPP

#include <algorithm>
#include <utility>

#include "gui/gui_constants.hpp"

struct GLFWwindow;

namespace lumice::gui {

// Pure function: clamp desired window size to the usable work area.
// workarea already excludes OS bars (menubar/Dock/taskbar) per GLFW docs;
// kWindowDecorationMargin covers the remaining title-bar + border cost.
// kMinWindowWidth/Height serve as a hard floor against pathological workarea.
inline std::pair<int, int> ClampWindowSizeToWorkarea(int desired_w, int desired_h, int work_w, int work_h) {
  int max_w = std::max(kMinWindowWidth, work_w - kWindowDecorationMargin);
  int max_h = std::max(kMinWindowHeight, work_h - kWindowDecorationMargin);
  return { std::min(desired_w, max_w), std::min(desired_h, max_h) };
}

// POD describing a monitor's workarea in virtual screen coordinates.
struct MonitorRect {
  int x;
  int y;
  int w;
  int h;
};

// Pure function: return the index of the monitor whose workarea contains the
// point (cx, cy); -1 if none. Edge convention: left/top inclusive, right/bottom
// exclusive, so adjacent monitors do not both claim the shared seam.
// Used by ApplyAspectRatio to route multi-monitor window sizing — see
// scratchpad/scrum-gui-polish-v11/task-fix-multi-monitor-aspect for rationale.
inline int SelectMonitorIndexByCenter(int cx, int cy, const MonitorRect* rects, int count) {
  for (int i = 0; i < count; i++) {
    const MonitorRect& r = rects[i];
    if (cx >= r.x && cx < r.x + r.w && cy >= r.y && cy < r.y + r.h) {
      return i;
    }
  }
  return -1;
}

// Return the workarea of the monitor containing the given window's center.
// Returns false when win is nullptr, GLFW cannot enumerate monitors, or the
// window center falls outside every known monitor — in all failure cases the
// caller should fall back to an unbounded constraint (e.g. FLT_MAX) rather
// than silently defaulting to a primary-monitor workarea, which would
// reintroduce the multi-monitor "primary bias" anti-pattern documented in
// scrum-gui-polish-v11/task-fix-multi-monitor-aspect.
//
// Mirrors the inline pattern used by ApplyAspectRatio in app.cpp:147-168.
// Both sites must evolve together; prefer routing future monitor-lookup
// needs through this helper to collapse the duplicate pattern over time.
bool GetCurrentMonitorWorkArea(GLFWwindow* win, MonitorRect* out);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_WINDOW_SIZING_HPP
