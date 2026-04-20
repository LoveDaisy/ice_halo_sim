#ifndef LUMICE_GUI_OVERLAY_LABELS_HPP
#define LUMICE_GUI_OVERLAY_LABELS_HPP

#include <string>
#include <vector>

#include "imgui.h"

namespace lumice::gui {

struct OverlayLabel {
  float screen_x, screen_y;  // ImGui logical screen coordinates
  std::string text;
  ImU32 color;
  bool has_bg = false;  // draw semi-transparent black background behind text
  int group = 0;        // collision avoidance only within same group (0=grid, 1=sun circles)
};

struct OverlayLabelInput {
  int lens_type;
  float fov, elevation, azimuth, roll;
  int visible;  // 0=upper, 1=lower, 2=full
  bool show_horizon, show_grid, show_sun_circles;
  float sun_dir[3];
  int sun_circle_count;
  const float* sun_circle_angles;
  float horizon_color[3], grid_color[3], sun_circles_color[3];
  float grid_alpha, sun_circles_alpha;
};

// Compute labels at viewport edges where overlay lines cross.
// vp_screen_* are in ImGui logical (window) coordinates.
void ComputeOverlayLabels(const OverlayLabelInput& input, float vp_screen_x, float vp_screen_y, float vp_screen_w,
                          float vp_screen_h, std::vector<OverlayLabel>& out);

// Draw labels using ImGui foreground draw list, with collision avoidance.
void DrawOverlayLabels(const std::vector<OverlayLabel>& labels);

// Append overlay labels to an arbitrary ImDrawList (with collision avoidance).
// Used by DrawOverlayLabels for the foreground list and by export_fbo_renderer for
// a self-owned list targeting an off-screen FBO.
void AppendOverlayToDrawList(ImDrawList* dl, const std::vector<OverlayLabel>& labels);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_OVERLAY_LABELS_HPP
