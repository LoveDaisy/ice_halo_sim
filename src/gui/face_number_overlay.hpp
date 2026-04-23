#ifndef LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP
#define LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP

#include "gui/crystal_renderer.hpp"
#include "imgui.h"

namespace lumice::gui {

// Upper bound on aggregated face labels drawn in one frame.
// Derivation: basal 2 + prism 6 + upper-pyramidal 6 + lower-pyramidal 6 = 20.
// 32 leaves margin for future crystal types without a larger allocation.
constexpr int kMaxFaceLabels = 32;

struct FaceLabel {
  float display_center[3];  // AABB-normalized, Y-Z swapped (GUI display space)
  float display_normal[3];
  // Per-face AABB in the same display space as display_center. Step 2 of the
  // face-number style refinement projects the 8 corners through the same MVP
  // used by ProjectLabelToScreen to estimate a screen-space bbox for the
  // viewport-ratio size filter.
  float display_aabb_min[3];
  float display_aabb_max[3];
  int face_number;
};

// Aggregate per-triangle data into one label per distinct face_number.
// Skips face_number <= 0 (0 = uninitialized slot, -1 = kInvalidId).
// Returns the number of labels written into out_labels (bounded by max_labels).
int AggregateFaceLabels(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                        const int* face_numbers, FaceLabel* out_labels, int max_labels);

// Per-CrystalStyle face-number rendering policy.
// `visible_*` colors apply to front-facing labels; `hidden_*` apply to back-
// facing labels and are only consulted when `draw_hidden == true` (otherwise
// the fields are dead and unconstrained).
struct FaceLabelStyle {
  ImU32 visible_fill;
  ImU32 visible_outline;
  ImU32 hidden_fill;
  ImU32 hidden_outline;
  bool draw_hidden;
  bool apply_size_filter;
};

// Minimum viewport-bbox-ratio threshold for a face label to remain visible
// when `apply_size_filter` is in effect (Hidden Line / Shaded modes). A face
// whose projected screen-space AABB width or height is below this fraction of
// the viewport extent is skipped to avoid sliver clutter. Single-source-of-
// truth for visual calibration.
constexpr float kFaceLabelMinViewportRatio = 0.10f;

namespace detail {

// Resolve the per-mode FaceLabelStyle. Exposed in detail:: for unit testing
// (`unit/face_number_resolve_style`) and DrawFaceNumberOverlay; not part of
// the stable public surface — only callable from face_number_overlay.cpp and
// face_number_overlay tests.
FaceLabelStyle ResolveFaceLabelStyle(CrystalStyle style);

// Project the 8 corners of `label->display_aabb_*` through `mvp` and return
// the screen-space AABB extent as fractions of `image_size`. Returns false
// when any corner has clip.w <= 0 (camera-behind degeneracy that would mix
// sign-flipped NDC into the bbox); callers must treat false as "undecidable
// → do not filter" to align with §7 risk 3 (prefer keeping labels over wrong
// filtering). Exposed in detail:: for unit testing
// (`unit/face_number_bbox_ratio_basic`).
bool ComputeLabelScreenBboxRatio(const FaceLabel* label, const float mvp[16], ImVec2 image_size, float* out_w_ratio,
                                 float* out_h_ratio);

}  // namespace detail

// Project a label center to 2D image-space coordinates and evaluate a front-face
// test in eye space (see crystal_renderer.cpp:357-367 for the reference algorithm).
// Returns false only when the projection is degenerate (clip.w == 0); otherwise
// writes into out_screen_x / out_screen_y / out_front_facing.
//
// On false return, the out parameters are left uninitialized. Callers must
// check the return value before reading them.
bool ProjectLabelToScreen(const FaceLabel* label, const float rotation[16], const float mvp[16], float image_pos_x,
                          float image_pos_y, float image_width, float image_height, float* out_screen_x,
                          float* out_screen_y, bool* out_front_facing);

// Draw face-number text labels over the crystal preview image.
//
// Caller protocol: `mvp` must be produced by CrystalRenderer::ComputeMvp with the
// same `rotation`, `zoom`, and image dimensions used for the current render pass;
// otherwise overlay coordinates will misalign with GL-rendered pixels.
//
// `face_numbers` and `vertices` / `triangles` must come from the same
// LUMICE_CrystalMesh in display space (Y-Z swapped + AABB normalized).
//
// `style` selects the per-mode policy from ResolveFaceLabelStyle: it controls
// whether hidden (back-facing) faces get drawn (Wireframe / X-Ray do; Hidden
// Line / Shaded skip them) and whether visible faces are filtered out when
// their projected screen-space AABB drops below kFaceLabelMinViewportRatio.
void DrawFaceNumberOverlay(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                           const int* face_numbers, const float rotation[16], const float mvp[16], ImVec2 image_pos,
                           ImVec2 image_size, ImDrawList* draw_list, CrystalStyle style);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP
