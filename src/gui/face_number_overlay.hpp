#ifndef LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP
#define LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP

#include "gui/crystal_renderer.hpp"
#include "imgui.h"

namespace lumice::gui {

// Upper bound on aggregated face labels drawn in one frame.
// Derivation: basal 2 + prism 6 + upper-pyramidal 6 + lower-pyramidal 6 = 20.
// 32 leaves margin for future crystal types without a larger allocation.
constexpr int kMaxFaceLabels = 32;

// Upper bound on unique vertices stored per face polygon. hex crystal faces
// max out at 6; 8 leaves headroom for future custom-crystal face shapes
// without changing the FaceLabel memory footprint. Sizing derivation:
//   FaceLabel ≈ 2 * float[3] + float[8*3] + 2 * int ≈ 116-128 bytes
//   total ≈ kMaxFaceLabels (32) * ~128 ≈ 4 KB stack per aggregate call.
constexpr int kMaxFacePolygonVerts = 8;

struct FaceLabel {
  float display_center[3];  // AABB-normalized, Y-Z swapped (GUI display space)
  float display_normal[3];
  // Unique face vertices in display-space (object-space 3D coords). Populated
  // by AggregateFaceLabels via per-triangle index dedup. Consumed by
  // detail::ComputeLabelMinWidthRatio (projects through MVP each frame).
  // Naming aligned with display_center / display_normal to signal object-space.
  float display_polygon_verts[kMaxFacePolygonVerts * 3];
  int display_polygon_vertex_count;
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

// Minimum viewport-ratio threshold for a face label to remain visible when
// `apply_size_filter` is in effect (Hidden Line / Shaded modes). The metric
// is now the projected-polygon **min-width** (smallest thickness across any
// edge-normal direction) divided by min(viewport.w, viewport.h). A face
// whose projected min-width drops below this fraction is skipped to avoid
// sliver clutter. Single-source-of-truth for visual calibration.
//
// Calibration history: v12 started at 0.10 (AABB w/h AND). v14 rewrites the
// metric to min-width (see ComputeLabelMinWidthRatio) and lowers to 0.05
// to expose more small faces.
constexpr float kFaceLabelMinViewportRatio = 0.05f;

namespace detail {

// Resolve the per-mode FaceLabelStyle. Exposed in detail:: for unit testing
// (`unit/face_number_resolve_style`) and DrawFaceNumberOverlay; not part of
// the stable public surface — only callable from face_number_overlay.cpp and
// face_number_overlay tests.
FaceLabelStyle ResolveFaceLabelStyle(CrystalStyle style);

// Project `label->display_polygon_verts[0..display_polygon_vertex_count-1]`
// through `mvp`, perform the perspective divide, convert to pixel space, and
// return the polygon's **min-width ratio** — the smallest thickness across any
// edge-normal direction divided by min(image_size.x, image_size.y).
//
// Contract ([[nodiscard]]; failure always clears `*out_ratio`):
//   * requires `image_size.x > 0 && image_size.y > 0` (caller-enforced).
//   * returns false + `*out_ratio = 0.0f` on degeneracy:
//       - polygon vertex count < 3 (too few to form a polygon);
//       - any vertex has clip.w <= 0 (camera-behind);
//       - all polygon edges fall below `kDegenerateEdgePx` (pixel-space) so no
//         stable edge normal is available.
//   * callers must treat false as "undecidable → do not filter" (prefer
//     keeping labels over wrong-filtering; §7 risk 3).
//
// Convexity: assumes projected polygon is convex. Standard ice-crystal faces
// (prism/column/pyramid) are convex; custom-crystal non-convex faces yield a
// conservative *underestimate* (label tends to be kept rather than
// wrong-filtered — safe direction).
//
// Exposed in detail:: for unit testing
// (`unit/face_number_min_width_ratio_basic`, `*_polygon_ordering`).
[[nodiscard]] bool ComputeLabelMinWidthRatio(const FaceLabel* label, const float mvp[16], ImVec2 image_size,
                                             float* out_ratio);

}  // namespace detail

// Project a label center to 2D image-space coordinates and evaluate a
// front-face test in eye space using the same `dot(n_eye, p_eye) < 0` rule
// as crystal_renderer.cpp:358-385. `mvp` and `zoom` serve *different*
// purposes and are **not** redundant:
//   * `mvp` produces NDC / screen coordinates (4x4 includes projection);
//   * `zoom` reconstructs the eye-space translation `(0, 0, -ComputeDist(zoom))`
//     needed for the culling dot product (p_eye = rotation_3x3 * center
//     + (0, 0, -dist)). Do not attempt to reduce to a single parameter.
//
// Returns false only when the projection is degenerate (clip.w == 0); otherwise
// writes into out_screen_x / out_screen_y / out_front_facing.
//
// On false return, the out parameters are left uninitialized. Callers must
// check the return value before reading them.
bool ProjectLabelToScreen(const FaceLabel* label, const float rotation[16], const float mvp[16], float zoom,
                          float image_pos_x, float image_pos_y, float image_width, float image_height,
                          float* out_screen_x, float* out_screen_y, bool* out_front_facing);

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
// their projected-polygon min-width ratio (see ComputeLabelMinWidthRatio)
// drops below kFaceLabelMinViewportRatio.
void DrawFaceNumberOverlay(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                           const int* face_numbers, const float rotation[16], const float mvp[16], float zoom,
                           ImVec2 image_pos, ImVec2 image_size, ImDrawList* draw_list, CrystalStyle style);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP
