#ifndef LUMICE_GUI_OVERLAY_LABELS_HPP
#define LUMICE_GUI_OVERLAY_LABELS_HPP

#include <string>
#include <vector>

#include "imgui.h"

namespace lumice::gui {

struct OverlayLabel {
  // Draw list screen coordinates. With ImGuiConfigFlags_ViewportsEnable enabled
  // (gui-polish-v15), these are absolute OS screen coordinates; the caller of
  // ComputeOverlayLabels owns the conversion from window-local to screen space.
  // For self-owned ImDrawList targets (e.g. export_fbo_renderer rendering to an
  // off-screen FBO), the caller passes a (0, 0) origin and these stay in FBO space.
  float screen_x, screen_y;
  std::string text;
  ImU32 color;
  bool has_bg = false;  // draw semi-transparent black background behind text
  int group = 0;        // collision avoidance only within same group (0=grid, 1=sun circles)
};

// The show_* fields here control **label** rendering only (label sampling along
// viewport edges). They are sourced from GuiState::show_<x>_label. The companion
// fields GuiState::show_<x>_line are consumed by OverlayDecoration (in
// preview_renderer.hpp), not this struct.
struct OverlayLabelInput {
  int lens_type;
  float fov, elevation, azimuth, roll;
  int visible;  // 0=upper, 1=lower, 2=full
  bool show_horizon, show_grid, show_sun_circles;
  float sun_dir[3];
  int sun_circle_count;
  const float* sun_circle_angles;
  float horizon_color[3], grid_color[3], sun_circles_color[3];
  float horizon_alpha, grid_alpha, sun_circles_alpha;
};

// Compute labels at viewport edges where overlay lines cross.
// vp_screen_* are in the same coordinate space as the target ImDrawList:
//   - For ImGui::GetWindowDrawList() under ImGuiConfigFlags_ViewportsEnable,
//     this is absolute OS screen space (caller must add vp->Pos offset).
//   - For self-owned ImDrawList targets (off-screen FBO), this is FBO-local
//     space starting at (0, 0).
// Output OverlayLabel.screen_x/y inherit this same coordinate space.
void ComputeOverlayLabels(const OverlayLabelInput& input, float vp_screen_x, float vp_screen_y, float vp_screen_w,
                          float vp_screen_h, std::vector<OverlayLabel>& out);

// Draw labels using the current ImGui window's draw list (so modals/popups correctly
// occlude the labels), with collision avoidance. Caller must invoke this inside an
// active ImGui::Begin/End pair.
//
// `vp_screen_*` is the same viewport rect the caller passed to ComputeOverlayLabels
// (same coordinate space — see ComputeOverlayLabels comment above). Each label's
// rendered text bounding box is clamped at least 2 px inside each viewport edge
// (`kViewportInsetPx` in detail::ClampLabelPosToViewport) so labels never straddle
// the viewport edge.
//
// Coverage asymmetry: this viewport clamp is unconditional (all lens types,
// all visible modes). The companion hemisphere-boundary inset (~3° push toward
// the visible side) is applied at compute time inside ComputeOverlayLabels and
// is gated to lens 0–3 + visible=upper/lower/front (see overlay_labels.cpp).
void DrawOverlayLabels(const std::vector<OverlayLabel>& labels, float vp_screen_x, float vp_screen_y, float vp_screen_w,
                       float vp_screen_h);

// Append overlay labels to an arbitrary ImDrawList (with collision avoidance).
// Used by DrawOverlayLabels for the preview window's draw list and by
// export_fbo_renderer for a self-owned list targeting an off-screen FBO.
// `vp_screen_*` semantics match DrawOverlayLabels.
void AppendOverlayToDrawList(ImDrawList* dl, const std::vector<OverlayLabel>& labels, float vp_screen_x,
                             float vp_screen_y, float vp_screen_w, float vp_screen_h);

namespace detail {

// Pure-function inverse projection used by ComputeOverlayLabels. Exposed
// here so unit tests can pin the per-lens-type dispatch (especially the
// orthographic branches added in task-orthographic-followup) without the
// edge-sampling layer in between.
//
// Inputs:
//   px, py            pixel offset from viewport center (shader convention)
//   res_x, res_y      viewport width / height in pixels
//   lens_type         LensType enum value (0..9)
//   fov               horizontal field-of-view in degrees
//   view_matrix       column-major 3x3 from BuildViewMatrix (preview_renderer.hpp);
//                     ignored by full-sky lens branches that don't view-transform.
//
// Outputs:
//   out_x/out_y/out_z   world-space unit direction (only set when *out_valid is true)
//   out_valid           false if the pixel falls outside the projection domain
//                       (asin guard, |lat| > π/2, etc.); the xyz outputs are
//                       left untouched in that case.
void PixelToWorldDirForTesting(float px, float py, float res_x, float res_y, int lens_type, float fov,
                               const float view_matrix[9], float* out_x, float* out_y, float* out_z, bool* out_valid);

// Clamp a label's anchor position so the rendered text bounding box stays
// inside the viewport rect with at least 2 px (`kViewportInsetPx`) margin
// on each side.
// `pos` is the desktop-relative top-left of the text glyph rect (caller has
// already subtracted half of `text_size` from the label's center). The returned
// position keeps `[pos.x, pos.x + text_size.x] ⊂ [vp_x + inset, vp_x + vp_w − inset]`
// (and similarly for y), unless the viewport is too narrow to fit the text +
// 2×inset — in that case the original pos is returned unchanged (rendering
// degrades to "centered on label anchor", matching legacy behaviour).
//
// Pure function — exposed in detail:: for unit testing
// (`overlay_labels/clamp_label_pos_*` tests).
ImVec2 ClampLabelPosToViewport(ImVec2 pos, ImVec2 text_size, float vp_x, float vp_y, float vp_w, float vp_h);

}  // namespace detail

}  // namespace lumice::gui

#endif  // LUMICE_GUI_OVERLAY_LABELS_HPP
