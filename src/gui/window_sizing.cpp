#include "gui/window_sizing.hpp"

#include <GLFW/glfw3.h>

#include <vector>

namespace lumice::gui {

bool GetCurrentMonitorWorkArea(GLFWwindow* win, MonitorRect* out) {
  if (win == nullptr || out == nullptr) {
    return false;
  }
  int mon_count = 0;
  GLFWmonitor** mons = glfwGetMonitors(&mon_count);
  if (mon_count <= 0 || mons == nullptr) {
    return false;
  }
  std::vector<MonitorRect> rects;
  rects.reserve(static_cast<size_t>(mon_count));
  for (int i = 0; i < mon_count; i++) {
    MonitorRect r{};
    // Defend against an unexpectedly null entry (e.g. race during monitor
    // hotplug). GLFW does not guarantee every pointer is non-null even when
    // the top-level array is; skip invalid entries rather than dereferencing.
    if (mons[i] == nullptr) {
      rects.push_back(r);
      continue;
    }
    glfwGetMonitorWorkarea(mons[i], &r.x, &r.y, &r.w, &r.h);
    rects.push_back(r);
  }
  int pos_x = 0;
  int pos_y = 0;
  int win_w = 0;
  int win_h = 0;
  glfwGetWindowPos(win, &pos_x, &pos_y);
  glfwGetWindowSize(win, &win_w, &win_h);
  int cx = pos_x + win_w / 2;
  int cy = pos_y + win_h / 2;
  int idx = SelectMonitorIndexByCenter(cx, cy, rects.data(), static_cast<int>(rects.size()));
  if (idx < 0) {
    return false;
  }
  *out = rects[idx];
  return true;
}

}  // namespace lumice::gui
