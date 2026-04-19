#ifndef LUMICE_GUI_WINDOW_SIZING_HPP
#define LUMICE_GUI_WINDOW_SIZING_HPP

#include <algorithm>
#include <utility>

#include "gui/gui_constants.hpp"

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

}  // namespace lumice::gui

#endif  // LUMICE_GUI_WINDOW_SIZING_HPP
