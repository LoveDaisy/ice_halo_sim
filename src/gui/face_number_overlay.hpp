#ifndef LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP
#define LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP

#include "imgui.h"

namespace lumice::gui {

// Upper bound on aggregated face labels drawn in one frame.
// Derivation: basal 2 + prism 6 + upper-pyramidal 6 + lower-pyramidal 6 = 20.
// 32 leaves margin for future crystal types without a larger allocation.
constexpr int kMaxFaceLabels = 32;

struct FaceLabel {
  float display_center[3];  // AABB-normalized, Y-Z swapped (GUI display space)
  float display_normal[3];
  int face_number;
};

// Aggregate per-triangle data into one label per distinct face_number.
// Skips face_number <= 0 (0 = uninitialized slot, -1 = kInvalidId).
// Returns the number of labels written into out_labels (bounded by max_labels).
int AggregateFaceLabels(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                        const int* face_numbers, FaceLabel* out_labels, int max_labels);

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
void DrawFaceNumberOverlay(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                           const int* face_numbers, const float rotation[16], const float mvp[16], ImVec2 image_pos,
                           ImVec2 image_size, ImDrawList* draw_list);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FACE_NUMBER_OVERLAY_HPP
