#include "gui/face_number_overlay.hpp"

#include <cfloat>
#include <cmath>
#include <cstdio>

#include "gui/gui_logger.hpp"

namespace lumice::gui {

namespace {

void AccumulateAabb(FaceLabel& label, const float* p0, const float* p1, const float* p2) {
  const float* tri_pts[3] = { p0, p1, p2 };
  for (const float* p : tri_pts) {
    label.display_aabb_min[0] = std::fmin(label.display_aabb_min[0], p[0]);
    label.display_aabb_min[1] = std::fmin(label.display_aabb_min[1], p[1]);
    label.display_aabb_min[2] = std::fmin(label.display_aabb_min[2], p[2]);
    label.display_aabb_max[0] = std::fmax(label.display_aabb_max[0], p[0]);
    label.display_aabb_max[1] = std::fmax(label.display_aabb_max[1], p[1]);
    label.display_aabb_max[2] = std::fmax(label.display_aabb_max[2], p[2]);
  }
}

}  // namespace

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

    // Triangle normal = normalize(cross(p1 - p0, p2 - p0)).
    // Note: we recompute from vertices rather than reusing
    // LUMICE_CrystalMesh.edge_face_normals because the latter indexes by edge,
    // not by triangle; there is no direct triangle-to-edge mapping exposed.
    // Cost is negligible (<=128 triangles).
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
        // Bucket overflow: drop extra faces. hex crystals max out at ~20 labels,
        // so this only triggers on a new crystal kind exceeding the compile-time
        // bound — log once so it's discoverable.
        GUI_LOG_WARNING("face_number_overlay: label bucket overflow (max={})", max_labels);
        continue;
      }
      slot = label_count++;
      out_labels[slot].face_number = fn;
      out_labels[slot].display_center[0] = 0.0f;
      out_labels[slot].display_center[1] = 0.0f;
      out_labels[slot].display_center[2] = 0.0f;
      out_labels[slot].display_normal[0] = nx;
      out_labels[slot].display_normal[1] = ny;
      out_labels[slot].display_normal[2] = nz;
      out_labels[slot].display_aabb_min[0] = FLT_MAX;
      out_labels[slot].display_aabb_min[1] = FLT_MAX;
      out_labels[slot].display_aabb_min[2] = FLT_MAX;
      out_labels[slot].display_aabb_max[0] = -FLT_MAX;
      out_labels[slot].display_aabb_max[1] = -FLT_MAX;
      out_labels[slot].display_aabb_max[2] = -FLT_MAX;
      tri_count[slot] = 0;
    }

    out_labels[slot].display_center[0] += cx;
    out_labels[slot].display_center[1] += cy;
    out_labels[slot].display_center[2] += cz;
    AccumulateAabb(out_labels[slot], p0, p1, p2);
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

namespace detail {

FaceLabelStyle ResolveFaceLabelStyle(CrystalStyle style) {
  // Sentinel: if a new CrystalStyle value is added, this assert forces an
  // explicit revisit of every branch below.
  static_assert(kCrystalStyleCount == 4, "update ResolveFaceLabelStyle when adding CrystalStyle");

  constexpr ImU32 kVisibleFill = IM_COL32(255, 255, 255, 255);
  constexpr ImU32 kVisibleOutline = IM_COL32(0, 0, 0, 255);
  // Translucent grey for X-Ray hidden faces: distinct from the bright visible
  // labels but still readable. Calibrated alpha=140 (~55%) sits inside the
  // [80, 200] band asserted by unit/face_number_resolve_style.
  constexpr ImU32 kXrayHiddenFill = IM_COL32(180, 180, 180, 140);
  constexpr ImU32 kXrayHiddenOutline = IM_COL32(0, 0, 0, 140);

  FaceLabelStyle s{};
  s.visible_fill = kVisibleFill;
  s.visible_outline = kVisibleOutline;
  s.hidden_fill = 0;
  s.hidden_outline = 0;
  s.draw_hidden = false;
  s.apply_size_filter = false;

  switch (style) {
    case CrystalStyle::kWireframe:
      s.draw_hidden = true;
      // Hidden labels in wireframe re-use the visible style (no occlusion to
      // disambiguate; matching style avoids visual hierarchy where there is
      // no semantic one).
      s.hidden_fill = kVisibleFill;
      s.hidden_outline = kVisibleOutline;
      break;
    case CrystalStyle::kHiddenLine:
      s.apply_size_filter = true;
      break;
    case CrystalStyle::kXRay:
      s.draw_hidden = true;
      s.hidden_fill = kXrayHiddenFill;
      s.hidden_outline = kXrayHiddenOutline;
      break;
    case CrystalStyle::kShaded:
      s.apply_size_filter = true;
      break;
  }
  return s;
}

bool ComputeLabelScreenBboxRatio(const FaceLabel* label, const float mvp[16], ImVec2 image_size, float* out_w_ratio,
                                 float* out_h_ratio) {
  const float* mn = label->display_aabb_min;
  const float* mx = label->display_aabb_max;

  float screen_min_x = FLT_MAX;
  float screen_min_y = FLT_MAX;
  float screen_max_x = -FLT_MAX;
  float screen_max_y = -FLT_MAX;

  for (int corner = 0; corner < 8; ++corner) {
    float p[3] = {
      (corner & 1) ? mx[0] : mn[0],
      (corner & 2) ? mx[1] : mn[1],
      (corner & 4) ? mx[2] : mn[2],
    };
    // clip = mvp * (p, 1), column-major (matches ProjectLabelToScreen).
    float cx = mvp[0] * p[0] + mvp[4] * p[1] + mvp[8] * p[2] + mvp[12];
    float cy = mvp[1] * p[0] + mvp[5] * p[1] + mvp[9] * p[2] + mvp[13];
    float cw = mvp[3] * p[0] + mvp[7] * p[1] + mvp[11] * p[2] + mvp[15];
    // clip.w <= 0 means the corner is at or behind the near plane; mixing
    // sign-flipped ndc into the bbox would inflate it spuriously. Treat as
    // undecidable → caller must not filter (keep the label).
    if (cw <= 0.0f) {
      return false;
    }
    float ndc_x = cx / cw;
    float ndc_y = cy / cw;
    float screen_x = (ndc_x + 1.0f) * 0.5f * image_size.x;
    // NDC.y is Y-up; screen is Y-down. Flip so screen_y matches the texture
    // coordinate produced by ProjectLabelToScreen.
    float screen_y = (1.0f - (ndc_y + 1.0f) * 0.5f) * image_size.y;
    screen_min_x = std::fmin(screen_min_x, screen_x);
    screen_min_y = std::fmin(screen_min_y, screen_y);
    screen_max_x = std::fmax(screen_max_x, screen_x);
    screen_max_y = std::fmax(screen_max_y, screen_y);
  }

  *out_w_ratio = (image_size.x > 0.0f) ? (screen_max_x - screen_min_x) / image_size.x : 0.0f;
  *out_h_ratio = (image_size.y > 0.0f) ? (screen_max_y - screen_min_y) / image_size.y : 0.0f;
  return true;
}

}  // namespace detail

bool ProjectLabelToScreen(const FaceLabel* label, const float rotation[16], const float mvp[16], float image_pos_x,
                          float image_pos_y, float image_width, float image_height, float* out_screen_x,
                          float* out_screen_y, bool* out_front_facing) {
  const float* p = label->display_center;

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
  const float* n = label->display_normal;
  float rn_z = rotation[2] * n[0] + rotation[6] * n[1] + rotation[10] * n[2];
  *out_front_facing = (rn_z > 0.0f);

  return true;
}

void DrawFaceNumberOverlay(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                           const int* face_numbers, const float rotation[16], const float mvp[16], ImVec2 image_pos,
                           ImVec2 image_size, ImDrawList* draw_list, CrystalStyle style) {
  if (triangle_count <= 0 || draw_list == nullptr) {
    return;
  }

  FaceLabel labels[kMaxFaceLabels];
  int n = AggregateFaceLabels(vertices, vertex_count, triangles, triangle_count, face_numbers, labels, kMaxFaceLabels);
  if (n <= 0) {
    return;
  }

  FaceLabelStyle resolved = detail::ResolveFaceLabelStyle(style);

  ImVec2 clip_max = ImVec2(image_pos.x + image_size.x, image_pos.y + image_size.y);
  draw_list->PushClipRect(image_pos, clip_max, true);

  for (int i = 0; i < n; ++i) {
    float sx = 0.0f;
    float sy = 0.0f;
    bool front = false;
    if (!ProjectLabelToScreen(&labels[i], rotation, mvp, image_pos.x, image_pos.y, image_size.x, image_size.y, &sx, &sy,
                              &front)) {
      continue;
    }
    // Hidden-face skip: only suppress when the resolved policy declines hidden
    // faces (kHiddenLine / kShaded). kWireframe / kXRay keep them.
    if (!front && !resolved.draw_hidden) {
      continue;
    }
    // Size filter: only applied to visible faces under kHiddenLine / kShaded.
    // ComputeLabelScreenBboxRatio returns false on near-plane degeneracy; in
    // that case we fall through to drawing (prefer keeping the label over
    // wrong-filtering, matching plan §7 risk 3 / 7).
    if (front && resolved.apply_size_filter) {
      float w_ratio = 0.0f;
      float h_ratio = 0.0f;
      if (detail::ComputeLabelScreenBboxRatio(&labels[i], mvp, image_size, &w_ratio, &h_ratio)) {
        if (w_ratio < kFaceLabelMinViewportRatio || h_ratio < kFaceLabelMinViewportRatio) {
          continue;
        }
      }
    }

    ImU32 fill = front ? resolved.visible_fill : resolved.hidden_fill;
    ImU32 outline = front ? resolved.visible_outline : resolved.hidden_outline;

    char text[8];
    std::snprintf(text, sizeof(text), "%d", labels[i].face_number);

    ImVec2 text_size = ImGui::CalcTextSize(text);
    ImVec2 pos(sx - text_size.x * 0.5f, sy - text_size.y * 0.5f);

    // 4-way offset outline + centered fill text.
    draw_list->AddText(ImVec2(pos.x - 1, pos.y), outline, text);
    draw_list->AddText(ImVec2(pos.x + 1, pos.y), outline, text);
    draw_list->AddText(ImVec2(pos.x, pos.y - 1), outline, text);
    draw_list->AddText(ImVec2(pos.x, pos.y + 1), outline, text);
    draw_list->AddText(pos, fill, text);
  }

  draw_list->PopClipRect();
}

}  // namespace lumice::gui
