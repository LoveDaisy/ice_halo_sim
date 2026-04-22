#include "gui/face_number_overlay.hpp"

#include <cmath>
#include <cstdio>

namespace lumice::gui {

int AggregateFaceLabels(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                        const int* face_numbers, FaceLabel* out_labels, int max_labels) {
  int label_count = 0;
  // tri_count[k] accumulates triangles folded into out_labels[k]; used to divide
  // the summed centroid into a mean before returning.
  int tri_count[kMaxFaceLabels] = {};

  for (int t = 0; t < triangle_count; ++t) {
    int fn = face_numbers[t];
    if (fn <= 0) {
      continue;  // 0 = uninitialized, -1 = kInvalidId
    }

    int v0 = triangles[t * 3 + 0];
    int v1 = triangles[t * 3 + 1];
    int v2 = triangles[t * 3 + 2];
    if (v0 < 0 || v0 >= vertex_count || v1 < 0 || v1 >= vertex_count || v2 < 0 || v2 >= vertex_count) {
      continue;
    }

    const float* p0 = vertices + v0 * 3;
    const float* p1 = vertices + v1 * 3;
    const float* p2 = vertices + v2 * 3;
    float cx = (p0[0] + p1[0] + p2[0]) / 3.0f;
    float cy = (p0[1] + p1[1] + p2[1]) / 3.0f;
    float cz = (p0[2] + p1[2] + p2[2]) / 3.0f;

    // Triangle normal = normalize(cross(p1 - p0, p2 - p0))
    float e1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
    float e2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
    float nx = e1[1] * e2[2] - e1[2] * e2[1];
    float ny = e1[2] * e2[0] - e1[0] * e2[2];
    float nz = e1[0] * e2[1] - e1[1] * e2[0];
    float len = nx * nx + ny * ny + nz * nz;
    if (len > 1e-12f) {
      float inv = 1.0f / std::sqrt(len);
      nx *= inv;
      ny *= inv;
      nz *= inv;
    }

    // Find existing label bucket.
    int slot = -1;
    for (int k = 0; k < label_count; ++k) {
      if (out_labels[k].face_number == fn) {
        slot = k;
        break;
      }
    }
    if (slot == -1) {
      if (label_count >= max_labels) {
        continue;  // bucket overflow; drop extra faces silently
      }
      slot = label_count++;
      out_labels[slot].face_number = fn;
      out_labels[slot].display_center[0] = 0.0f;
      out_labels[slot].display_center[1] = 0.0f;
      out_labels[slot].display_center[2] = 0.0f;
      out_labels[slot].display_normal[0] = nx;
      out_labels[slot].display_normal[1] = ny;
      out_labels[slot].display_normal[2] = nz;
      tri_count[slot] = 0;
    }

    out_labels[slot].display_center[0] += cx;
    out_labels[slot].display_center[1] += cy;
    out_labels[slot].display_center[2] += cz;
    tri_count[slot]++;
  }

  // Finalize means.
  for (int k = 0; k < label_count; ++k) {
    int n = tri_count[k];
    if (n > 1) {
      float inv = 1.0f / static_cast<float>(n);
      out_labels[k].display_center[0] *= inv;
      out_labels[k].display_center[1] *= inv;
      out_labels[k].display_center[2] *= inv;
    }
  }

  return label_count;
}

bool ProjectLabelToScreen(const FaceLabel& label, const float rotation[16], const float mvp[16], float image_pos_x,
                          float image_pos_y, float image_width, float image_height, float* out_screen_x,
                          float* out_screen_y, bool* out_front_facing) {
  const float* p = label.display_center;

  // clip = mvp * (p, 1), column-major.
  float cx = mvp[0] * p[0] + mvp[4] * p[1] + mvp[8] * p[2] + mvp[12];
  float cy = mvp[1] * p[0] + mvp[5] * p[1] + mvp[9] * p[2] + mvp[13];
  float cw = mvp[3] * p[0] + mvp[7] * p[1] + mvp[11] * p[2] + mvp[15];
  if (cw == 0.0f) {
    return false;
  }

  float ndc_x = cx / cw;
  float ndc_y = cy / cw;

  *out_screen_x = image_pos_x + (ndc_x + 1.0f) * 0.5f * image_width;
  // NDC.y is Y-up; screen is Y-down.
  *out_screen_y = image_pos_y + (1.0f - (ndc_y + 1.0f) * 0.5f) * image_height;

  // Front-face test: rotate normal to eye space (rotation is orthonormal, F12).
  // For display-space meshes in a unit cube with camera at +Z dist along the
  // view direction, a face is front-facing iff the rotated normal's z
  // component is positive (points back toward the camera). This is equivalent
  // to crystal_renderer.cpp:357-367's dot(n_eye, p_eye) < 0 test for near-
  // centered points and is what drives the GL back-face classification.
  const float* n = label.display_normal;
  float rn_z = rotation[2] * n[0] + rotation[6] * n[1] + rotation[10] * n[2];
  *out_front_facing = (rn_z > 0.0f);

  return true;
}

void DrawFaceNumberOverlay(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                           const int* face_numbers, const float rotation[16], const float mvp[16], ImVec2 image_pos,
                           ImVec2 image_size, ImDrawList* draw_list) {
  if (triangle_count <= 0 || draw_list == nullptr) {
    return;
  }

  FaceLabel labels[kMaxFaceLabels];
  int n = AggregateFaceLabels(vertices, vertex_count, triangles, triangle_count, face_numbers, labels, kMaxFaceLabels);
  if (n <= 0) {
    return;
  }

  ImVec2 clip_max = ImVec2(image_pos.x + image_size.x, image_pos.y + image_size.y);
  draw_list->PushClipRect(image_pos, clip_max, true);

  constexpr ImU32 kWhite = IM_COL32(255, 255, 255, 255);
  constexpr ImU32 kBlack = IM_COL32(0, 0, 0, 255);

  for (int i = 0; i < n; ++i) {
    float sx = 0.0f;
    float sy = 0.0f;
    bool front = false;
    if (!ProjectLabelToScreen(labels[i], rotation, mvp, image_pos.x, image_pos.y, image_size.x, image_size.y, &sx, &sy,
                              &front)) {
      continue;
    }
    if (!front) {
      continue;
    }

    char text[8];
    std::snprintf(text, sizeof(text), "%d", labels[i].face_number);

    ImVec2 text_size = ImGui::CalcTextSize(text);
    ImVec2 pos(sx - text_size.x * 0.5f, sy - text_size.y * 0.5f);

    // 4-way offset outline + centered white text.
    draw_list->AddText(ImVec2(pos.x - 1, pos.y), kBlack, text);
    draw_list->AddText(ImVec2(pos.x + 1, pos.y), kBlack, text);
    draw_list->AddText(ImVec2(pos.x, pos.y - 1), kBlack, text);
    draw_list->AddText(ImVec2(pos.x, pos.y + 1), kBlack, text);
    draw_list->AddText(pos, kWhite, text);
  }

  draw_list->PopClipRect();
}

}  // namespace lumice::gui
