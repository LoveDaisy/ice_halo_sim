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
  int group = 0;        // collision avoidance only within same group; see kGroup* below
};

// Collision-avoidance groups. The pass compares a label only against others in its own group, so
// these say which labels are allowed to sit close together: a 22 deg circle marker and a 30 deg
// parallel marker mean different things and may crowd, two circle markers may not. Public because
// the sets built in app_panels.cpp choose between them by name rather than by literal.
constexpr int kGroupGrid = 0;
constexpr int kGroupSunCircles = 1;

// The show_* fields here control **label** rendering only (label sampling along
// viewport edges). They are sourced from GuiState::show_<x>_label. The companion
// fields GuiState::show_<x>_line are consumed by OverlayDecoration (in
// preview_renderer.hpp), not this struct.
struct OverlayLabelInput {
  int lens_type;
  float fov, elevation, azimuth, roll;
  int visible;  // 0=upper, 1=lower, 2=full (base hemisphere)
  bool front = false;
  // Sun circles are NOT here. Their anchors come from core (AnnotationOverlayCache), so the six
  // fields that used to describe them — the switch, the sun direction, the angle list and its
  // count, the colour and the alpha — moved out with the walk that consumed them. Everything left
  // in this struct is an annotation the GUI still places itself.
  // show_grid is gone with the walk it drove: the coordinate grid's anchors come from core now,
  // through the same CurveLabelSet path the circles use. What is left in this struct is the
  // horizon, the one annotation the GUI still places itself.
  bool show_horizon;
  float horizon_color[3], grid_color[3];
  float horizon_alpha, grid_alpha;

  // Coordinate grid step in degrees. NOT dead despite the grid's walk being gone: the horizon's
  // label text is formatted to one decimal or none depending on it, which is a property of how
  // fine the grid around it is. Default 10 deg keeps existing callers' behaviour unchanged; the
  // live preview path overrides via ComputeGridStep(fov).
  float grid_step = 10.0f;
};

// Turn core's label anchors into OverlayLabels the two draw paths already understand. Kept here
// rather than in either caller because both of them need it and the conversion — canvas pixels to
// the target draw list's space, plus the colour and group the collision pass reads — is the same
// both times.
//
// NOT specific to one annotation family: the angular-distance circles and the coordinate grid both
// arrive as core anchors and are both drawn this way, differing only in the appearance and the
// collision group the caller attaches. That is why the type is named for the shape of the data
// rather than for its first consumer.
//
// `anchors` are in canvas pixel space with a top-left origin, which is what
// AnnotationOverlayCache returns and what the viewport rect below is offset by. `px`/`py` and
// `text` are read; the label's kind is not, because a set carries one family's anchors.
struct CurveLabelAnchor {
  float px = 0.0f;
  float py = 0.0f;
  std::string text;
};
// The anchors plus the appearance the drawer gives them. Bundled because both the live preview and
// the off-screen export need to carry all three together, and an appearance that travels separately
// from its anchors is one more thing that can arrive without them.
struct CurveLabelSet {
  std::vector<CurveLabelAnchor> anchors;
  float color[3] = { 1.0f, 1.0f, 1.0f };
  float alpha = 1.0f;
  // Which collision group these labels join. Defaulted to the sun circles because they were the
  // first family through here; every caller sets it explicitly.
  int group = kGroupSunCircles;
  // Semi-transparent plate behind the text. On for the circles, off for the grid — which is not a
  // taste call but the appearance each family already had when it was walked here, kept so moving
  // the walk into core changes where the numbers come from and nothing about how they look.
  bool has_bg = true;
};
void AppendCurveLabels(const CurveLabelSet& set, float vp_screen_x, float vp_screen_y, std::vector<OverlayLabel>& out);

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

// Test-only thin wrapper exposing the anonymous-namespace WorldDirToPixel so
// unit tests can pin per-lens dispatch (especially the full-sky lens forward
// projectors added in task-label-placement-impl). Mirror of
// PixelToWorldDirForTesting.
//
// Inputs:
//   wx, wy, wz         world-space unit direction
//   res_x, res_y       viewport width / height in pixels
//   lens_type          LensType enum value (0..10)
//   fov                horizontal field-of-view in degrees
//   view_matrix        column-major 3x3 (BuildViewMatrix output); ignored by
//                      full-sky lens branches.
//
// Outputs:
//   out_px, out_py     pixel offset from viewport center (only set when valid)
//   out_valid          false if direction is behind camera, outside the
//                      projection domain, or otherwise unmapped.
void WorldDirToPixelForTesting(float wx, float wy, float wz, float res_x, float res_y, int lens_type, float fov,
                               const float view_matrix[9], float* out_px, float* out_py, bool* out_valid);

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
