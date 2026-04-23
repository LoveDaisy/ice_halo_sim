#include "gui/face_number_overlay.hpp"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdio>

#include "gui/gui_logger.hpp"

namespace lumice::gui {

namespace {

// Edge-degeneracy threshold in pixel space. At preview size 320px, 1px is
// ~0.003 viewport-ratio; half a pixel is well below the 0.05 filter threshold
// so normal thin faces are unaffected, while true degeneracies (coincident
// projected vertices) are stably caught before `normalize` yields NaN.
constexpr float kDegenerateEdgePx = 0.5f;

// Linear dedup: returns index in polygon_verts (0..count-1) if `v` is already
// present (within a tiny tolerance to absorb float-equal jitter), else -1.
int FindPolygonVertex(const float* polygon_verts, int count, const float* v) {
  for (int i = 0; i < count; ++i) {
    const float* p = polygon_verts + i * 3;
    // Strict equality is fine here: vertices come from the same mesh buffer
    // via identical indices, so shared verts are bit-identical.
    if (p[0] == v[0] && p[1] == v[1] && p[2] == v[2]) {
      return i;
    }
  }
  return -1;
}

void AppendPolygonVertex(FaceLabel& label, const float* v) {
  if (FindPolygonVertex(label.display_polygon_verts, label.display_polygon_vertex_count, v) >= 0) {
    return;
  }
  if (label.display_polygon_vertex_count >= kMaxFacePolygonVerts) {
    // Bucket overflow: custom-crystal face with >kMaxFacePolygonVerts unique
    // vertices. Warn once per process (overlay runs every frame; without this
    // gate a spammy crystal would produce ~60 lines/second). Min-width will
    // be computed on the first kMaxFacePolygonVerts vertices (still a
    // conservative result → label tends to be kept).
    static bool warned = false;
    if (!warned) {
      GUI_LOG_WARNING("face_number_overlay: polygon vertex overflow (max={})", kMaxFacePolygonVerts);
      warned = true;
    }
    return;
  }
  float* slot = label.display_polygon_verts + label.display_polygon_vertex_count * 3;
  slot[0] = v[0];
  slot[1] = v[1];
  slot[2] = v[2];
  ++label.display_polygon_vertex_count;
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
      out_labels[slot].display_polygon_vertex_count = 0;
      tri_count[slot] = 0;
    }

    out_labels[slot].display_center[0] += cx;
    out_labels[slot].display_center[1] += cy;
    out_labels[slot].display_center[2] += cz;

    // Dedup-append each triangle vertex into the per-face polygon bucket.
    AppendPolygonVertex(out_labels[slot], p0);
    AppendPolygonVertex(out_labels[slot], p1);
    AppendPolygonVertex(out_labels[slot], p2);

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

bool ComputeLabelMinWidthRatio(const FaceLabel* label, const float mvp[16], ImVec2 image_size, float* out_ratio) {
  // Contract clear: [[nodiscard]] + failure clears *out_ratio. Callers must
  // honor the bool; hide-the-label fallback lives in DrawFaceNumberOverlay.
  *out_ratio = 0.0f;

  // Caller guarantees positive image_size (preview pane is non-zero at build
  // time). Belt-and-suspenders: assert for dev builds.
  assert(image_size.x > 0.0f && image_size.y > 0.0f);

  int count = label->display_polygon_vertex_count;
  if (count < 3) {
    return false;  // Degenerate polygon; cannot define a min-width direction.
  }
  // Invariant: AppendPolygonVertex caps count at kMaxFacePolygonVerts. A larger
  // value here signals caller-side memory corruption — assert loudly rather
  // than silently truncating (which would mask the bug).
  assert(count <= kMaxFacePolygonVerts);

  // Project each polygon vertex through MVP + perspective divide + NDC→pixel.
  // Pixel space is consistent with ProjectLabelToScreen's screen coords so
  // kDegenerateEdgePx reads as "half a screen pixel".
  float px_x[kMaxFacePolygonVerts];
  float px_y[kMaxFacePolygonVerts];
  for (int i = 0; i < count; ++i) {
    const float* v = label->display_polygon_verts + i * 3;
    float cx = mvp[0] * v[0] + mvp[4] * v[1] + mvp[8] * v[2] + mvp[12];
    float cy = mvp[1] * v[0] + mvp[5] * v[1] + mvp[9] * v[2] + mvp[13];
    float cw = mvp[3] * v[0] + mvp[7] * v[1] + mvp[11] * v[2] + mvp[15];
    if (cw <= 0.0f) {
      // Camera-behind degeneracy: NDC would sign-flip and inflate the polygon.
      return false;
    }
    float ndc_x = cx / cw;
    float ndc_y = cy / cw;
    px_x[i] = (ndc_x + 1.0f) * 0.5f * image_size.x;
    // NDC.y up → screen y down, matching ProjectLabelToScreen.
    px_y[i] = (1.0f - (ndc_y + 1.0f) * 0.5f) * image_size.y;
  }

  // Sort the projected polygon vertices by angle around their centroid to
  // recover a convex-polygon traversal order. AggregateFaceLabels provides
  // vertices in arbitrary triangle-discovery order, so this sort is required
  // even for genuinely convex faces.
  float cx = 0.0f;
  float cy = 0.0f;
  for (int i = 0; i < count; ++i) {
    cx += px_x[i];
    cy += px_y[i];
  }
  cx /= static_cast<float>(count);
  cy /= static_cast<float>(count);

  int order[kMaxFacePolygonVerts];
  for (int i = 0; i < count; ++i) {
    order[i] = i;
  }
  std::sort(order, order + count, [&](int a, int b) {
    return std::atan2(px_y[a] - cy, px_x[a] - cx) < std::atan2(px_y[b] - cy, px_x[b] - cx);
  });

  // For each ordered edge, take its outward normal; project all polygon
  // vertices onto the normal; the min-width is the smallest of these spans
  // (classic convex-polygon rotating-calipers result). Assumes convex; on
  // non-convex input the result is a conservative underestimate → label kept.
  float min_span = FLT_MAX;
  int valid_edges = 0;
  for (int i = 0; i < count; ++i) {
    int a = order[i];
    int b = order[(i + 1) % count];
    float dx = px_x[b] - px_x[a];
    float dy = px_y[b] - px_y[a];
    float edge_len = std::sqrt(dx * dx + dy * dy);
    if (edge_len < kDegenerateEdgePx) {
      continue;  // Coincident screen points → unstable normal, skip direction.
    }
    float nx = -dy / edge_len;
    float ny = dx / edge_len;
    float min_proj = FLT_MAX;
    float max_proj = -FLT_MAX;
    for (int k = 0; k < count; ++k) {
      float p = nx * px_x[k] + ny * px_y[k];
      min_proj = std::fmin(min_proj, p);
      max_proj = std::fmax(max_proj, p);
    }
    float span = max_proj - min_proj;
    min_span = std::fmin(min_span, span);
    ++valid_edges;
  }

  if (valid_edges == 0) {
    return false;  // Every edge degenerate → polygon collapsed on screen.
  }

  float viewport_ref = std::fmin(image_size.x, image_size.y);
  *out_ratio = min_span / viewport_ref;
  return true;
}

}  // namespace detail

bool ProjectLabelToScreen(const FaceLabel* label, const float rotation[16], const float mvp[16], float zoom,
                          float image_pos_x, float image_pos_y, float image_width, float image_height,
                          float* out_screen_x, float* out_screen_y, bool* out_front_facing) {
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

  // Front-face test mirrors crystal_renderer.cpp:358-385 (dot(n_eye,p_eye)<0).
  // `view_rot` = rotation(3x3) + translate (0,0,-dist); only the translation
  // contributes beyond the rotated center.
  //
  // DO NOT modify this formula independently of crystal_renderer.cpp:358-385 —
  // change crystal_renderer first, then sync here. The reason not to share a
  // helper: crystal_renderer operates on edge midpoint + dual edge-face-normal,
  // while face_number uses face center + single face normal. Correspondence
  // is mechanically guarded by test
  // `unit/face_number_cull_matches_crystal_renderer_formula`.
  const float* n = label->display_normal;
  float dist = CrystalRenderer::ComputeDist(zoom);
  float px = rotation[0] * p[0] + rotation[4] * p[1] + rotation[8] * p[2];
  float py = rotation[1] * p[0] + rotation[5] * p[1] + rotation[9] * p[2];
  float pz = rotation[2] * p[0] + rotation[6] * p[1] + rotation[10] * p[2] - dist;
  float nxe = rotation[0] * n[0] + rotation[4] * n[1] + rotation[8] * n[2];
  float nye = rotation[1] * n[0] + rotation[5] * n[1] + rotation[9] * n[2];
  float nze = rotation[2] * n[0] + rotation[6] * n[1] + rotation[10] * n[2];
  *out_front_facing = (nxe * px + nye * py + nze * pz) < 0.0f;

  return true;
}

void DrawFaceNumberOverlay(const float* vertices, int vertex_count, const int* triangles, int triangle_count,
                           const int* face_numbers, const float rotation[16], const float mvp[16], float zoom,
                           ImVec2 image_pos, ImVec2 image_size, ImDrawList* draw_list, CrystalStyle style) {
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
    if (!ProjectLabelToScreen(&labels[i], rotation, mvp, zoom, image_pos.x, image_pos.y, image_size.x, image_size.y,
                              &sx, &sy, &front)) {
      continue;
    }
    // Hidden-face skip: only suppress when the resolved policy declines hidden
    // faces (kHiddenLine / kShaded). kWireframe / kXRay keep them.
    if (!front && !resolved.draw_hidden) {
      continue;
    }
    // Size filter: only applied to visible faces under kHiddenLine / kShaded.
    // ComputeLabelMinWidthRatio returns false on degeneracy; in that case we
    // fall through to drawing (prefer keeping the label over wrong-filtering).
    if (front && resolved.apply_size_filter) {
      float ratio = 0.0f;
      if (detail::ComputeLabelMinWidthRatio(&labels[i], mvp, image_size, &ratio)) {
        if (ratio < kFaceLabelMinViewportRatio) {
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
