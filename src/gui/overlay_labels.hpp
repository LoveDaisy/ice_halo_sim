#ifndef LUMICE_GUI_OVERLAY_LABELS_HPP
#define LUMICE_GUI_OVERLAY_LABELS_HPP

#include <string>
#include <vector>

#include "imgui.h"

namespace lumice::gui {

struct OverlayLabel {
  // Draw list coordinates, in logical points. Both drawing consumers are self-owned ImDrawLists
  // targeting an off-screen FBO (export_fbo_renderer.cpp), so the caller of AppendCurveLabels
  // passes a (0, 0) origin and these stay in FBO-local space.
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
// The sky reference points' names. Its own group, not the circles': a marker's name sits beside a
// single point rather than along a curve, and two DIFFERENT reference points that happen to project
// close together must both stay named — suppressing one would say the sky has fewer named
// directions than it does, which is the opposite of what this family is for.
constexpr int kGroupMarkers = 2;

// Turn core's label anchors into OverlayLabels the two draw paths already understand. Kept here
// rather than in either caller because both of them need it and the conversion — canvas pixels to
// the target draw list's space, plus the colour and group the collision pass reads — is the same
// both times.
//
// NOT specific to one annotation family: all three — the horizon, the angular-distance circles and
// the coordinate grid — arrive as core anchors and are drawn this way, differing only in the
// appearance and the collision group the caller attaches. That is why the type is named for the
// shape of the data rather than for its first consumer, and it is now the ONLY way a label reaches
// this file: the GUI's own curve walk (ComputeOverlayLabels) is gone with the last family that
// needed it.
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

// Append overlay labels to an ImDrawList, with collision avoidance. THE one drawing path: both the
// on-screen preview and the PNG export reach it through export_fbo_renderer.cpp, which owns a
// self-allocated list targeting the preview/export FBO. There is no second path that draws onto an
// ImGui window's draw list any more — that is what makes "the screenshot shows what the screen
// shows" a property of the structure rather than of two implementations kept in step.
//
// `vp_screen_*` is the viewport rect the anchors live in: FBO-local, starting at (0, 0), in LOGICAL
// POINTS (the space a draw list works in; the DPI is applied once by the backend, see
// RenderOverlayToFbo). Each label's rendered text bounding box is clamped at least 2 px inside each
// viewport edge (`kViewportInsetPx` in detail::ClampLabelPosToViewport) so labels never straddle the
// viewport edge.
//
// This clamp is unconditional (all lens types, all visible modes) and is the GUI's alone: the CLI
// renderer draws core's raw anchors with no clamp and no collision pass, so a label near the rim
// can sit a few pixels differently in the two. Where the two agree is WHICH labels appear and
// where the curve puts them, which is what moving the walk into core bought.
void AppendOverlayToDrawList(ImDrawList* dl, const std::vector<OverlayLabel>& labels, float vp_screen_x,
                             float vp_screen_y, float vp_screen_w, float vp_screen_h);

namespace detail {

// Pure-function inverse projection: the C++ mirror of the preview fragment shader's own inverse
// lens math (preview_renderer.cpp), kept in step with it by hand because GLSL cannot include a C++
// header.
//
// ITS CONSUMER IS THE TEST LAYER, and deliberately so — read this before concluding it is dead.
// It had one production caller, the GUI's curve walk, and that walk is gone (its labels come from
// core now). What is left is its role as the GUI-SIDE ORACLE of the cross-implementation parity
// gates: test_visible_mask_gui_parity.cpp, test_horizon_gui_parity.cpp,
// test_annotation_overlay_gui_parity.cpp, test_lens_border_geometry.cpp,
// test_render_handedness_guard.cpp and test_preview_projection_chain.cpp all compare core's
// projection against THIS function, because the only other copy of the shader's math is in GLSL
// and cannot be called from a test. Deleting it would not remove a duplicate; it would remove the
// second implementation those six gates exist to compare against.
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

// The forward direction of the same mirror, and the same standing: no production caller since the
// curve walk was deleted, and the same parity gates as its inverse above are what it is kept for.
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
