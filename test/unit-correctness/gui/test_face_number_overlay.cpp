// Unit tests for face_number_overlay's pure functions: AggregateFaceLabels,
// ProjectLabelToScreen, ComputeLabelMinWidthRatio, ResolveFaceLabelStyle.
//
// The original file split these from its rendering cases by ImGui test category ("unit" vs
// "screenshot") — the author's own migration marker. The twelve "unit" ones need no ImGui
// context at all, so they run here in the windowless target; the three that draw with the font
// atlas or walk BuildCrystalMeshData stay in test/gui/functional/test_gui_face_number_overlay.cpp.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>

#include "gui/crystal_preview.hpp"
#include "gui/crystal_renderer.hpp"
#include "gui/face_number_overlay.hpp"
#include "gui/gui_constants.hpp"

namespace {

using lumice::gui::AggregateFaceLabels;
using lumice::gui::AggregateFaceLabelsFromTopology;
using lumice::gui::CrystalStyle;
using lumice::gui::FaceLabel;
using lumice::gui::kFaceLabelMinViewportRatio;
using lumice::gui::kMaxFaceLabels;
using lumice::gui::kMaxFacePolygonVerts;
using lumice::gui::ProjectLabelToScreen;
using lumice::gui::detail::ComputeLabelMinWidthRatio;
using lumice::gui::detail::ResolveFaceLabelStyle;


void Identity4x4(float m[16]) {
  std::memset(m, 0, sizeof(float) * 16);
  m[0] = m[5] = m[10] = m[15] = 1.0f;
}

// Rotation around X axis by 180°, column-major 4x4.
void RotX180(float m[16]) {
  std::memset(m, 0, sizeof(float) * 16);
  m[0] = 1.0f;
  m[5] = -1.0f;
  m[10] = -1.0f;
  m[15] = 1.0f;
}

// Reference front-face test — INDEPENDENTLY re-derives the same eye-space
// front/back formula the renderer uses, so this test mechanically guards
// cross-file drift instead of being a tautology on the production helper.
//
// Formula: m_eye = Rx(+kCameraTiltDeg) · rotation; p_eye = m_eye · center +
// (0, 0, -dist); n_eye = m_eye · normal; front = dot(n_eye, p_eye) < 0.
//
// The Rx(+kCameraTiltDeg) 3×3 components are hand-written here (NOT via
// CrystalRenderer::ComputeEyeRotation) so a wrong tilt sign / missing V_rot /
// swapped composition order in the production side is caught by
// face_number_cull_matches_crystal_renderer_formula rather than silently
// mirrored.
bool ReferenceFrontFacing(const float rotation[16], float zoom, const float center[3], const float normal[3]) {
  const float angle = lumice::gui::kCameraTiltDeg * lumice::gui::CrystalRenderer::kDeg2Rad;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  // V_rot = Rx(+kCameraTiltDeg), column-major 3×3 (padded to 4×4 semantics).
  // Row 0: (1, 0, 0), Row 1: (0, c, -s), Row 2: (0, s, c).
  const float v_rot[9] = {
    1.0f, 0.0f, 0.0f,  // col 0
    0.0f, c,    s,     // col 1
    0.0f, -s,   c,     // col 2
  };
  // m_eye_3x3 = v_rot * rotation_3x3, column-major.
  float m_eye[9] = {};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < 3; ++k) {
        // rotation is column-major 4×4 so column k lives at [k*4 + row].
        sum += v_rot[i + k * 3] * rotation[k + j * 4];
      }
      m_eye[i + j * 3] = sum;
    }
  }
  float dist = lumice::gui::CrystalRenderer::ComputeDist(zoom);
  float px = m_eye[0] * center[0] + m_eye[3] * center[1] + m_eye[6] * center[2];
  float py = m_eye[1] * center[0] + m_eye[4] * center[1] + m_eye[7] * center[2];
  float pz = m_eye[2] * center[0] + m_eye[5] * center[1] + m_eye[8] * center[2] - dist;
  float nx = m_eye[0] * normal[0] + m_eye[3] * normal[1] + m_eye[6] * normal[2];
  float ny = m_eye[1] * normal[0] + m_eye[4] * normal[1] + m_eye[7] * normal[2];
  float nz = m_eye[2] * normal[0] + m_eye[5] * normal[1] + m_eye[8] * normal[2];
  return (nx * px + ny * py + nz * pz) < 0.0f;
}

}  // namespace

// Aggregate: 4 triangles, face_numbers = [3, 3, 4, 4] → 2 labels, centers = group means.
TEST(FaceNumberOverlay, face_number_aggregate) {
  lumice::gui::ResetLastCrystalMesh();
  // 6 vertices defining two pairs of adjacent triangles.
  float verts[] = {
    0.0f, 0.0f, 0.0f,  // v0
    1.0f, 0.0f, 0.0f,  // v1
    0.0f, 1.0f, 0.0f,  // v2
    2.0f, 0.0f, 0.0f,  // v3
    3.0f, 0.0f, 0.0f,  // v4
    2.0f, 1.0f, 0.0f,  // v5
  };
  int tris[] = {
    0, 1, 2,  // face 3
    0, 2, 1,  // face 3 (duplicate coverage)
    3, 4, 5,  // face 4
    3, 5, 4,  // face 4
  };
  int fn[] = { 3, 3, 4, 4 };

  FaceLabel labels[kMaxFaceLabels] = {};
  int n = AggregateFaceLabels(verts, 6, tris, 4, fn, labels, kMaxFaceLabels);
  EXPECT_EQ(n, 2);

  // Centers are triangle-centroid averages within each group.
  for (int k = 0; k < 2; ++k) {
    if (labels[k].face_number == 3) {
      EXPECT_TRUE(std::abs(labels[k].display_center[0] - 1.0f / 3.0f) < 1e-5f);
      EXPECT_TRUE(std::abs(labels[k].display_center[1] - 1.0f / 3.0f) < 1e-5f);
    } else if (labels[k].face_number == 4) {
      EXPECT_TRUE(std::abs(labels[k].display_center[0] - 7.0f / 3.0f) < 1e-5f);
      EXPECT_TRUE(std::abs(labels[k].display_center[1] - 1.0f / 3.0f) < 1e-5f);
    } else {
      EXPECT_TRUE(false);  // unexpected face_number
    }
  }
}

// Aggregate polygon: unique vertex dedup across triangles sharing the same
// face_number. Replaces the v14-retired face_number_aggregate_bbox test.
TEST(FaceNumberOverlay, face_number_aggregate_polygon) {
  lumice::gui::ResetLastCrystalMesh();
  // Quad made of 2 triangles sharing an edge: 4 unique vertices, both
  // triangles share face_number=7.
  float verts[] = {
    0.0f, 0.0f, 0.0f,  // v0
    1.0f, 0.0f, 0.0f,  // v1
    1.0f, 1.0f, 0.0f,  // v2
    0.0f, 1.0f, 0.0f,  // v3
  };
  int tris[] = {
    0, 1, 2,  // face 7
    0, 2, 3,  // face 7 (shares v0, v2 with the first)
  };
  int fn[] = { 7, 7 };

  FaceLabel labels[kMaxFaceLabels] = {};
  int n = AggregateFaceLabels(verts, 4, tris, 2, fn, labels, kMaxFaceLabels);
  EXPECT_EQ(n, 1);
  EXPECT_EQ(labels[0].face_number, 7);
  EXPECT_EQ(labels[0].display_polygon_vertex_count, 4);

  // All 4 unique vertices must appear in the polygon bucket (order is
  // triangle-discovery, not geometric).
  bool seen[4] = { false, false, false, false };
  for (int i = 0; i < labels[0].display_polygon_vertex_count; ++i) {
    const float* p = labels[0].display_polygon_verts + i * 3;
    for (int v = 0; v < 4; ++v) {
      const float* w = verts + v * 3;
      if (std::abs(p[0] - w[0]) < 1e-5f && std::abs(p[1] - w[1]) < 1e-5f && std::abs(p[2] - w[2]) < 1e-5f) {
        seen[v] = true;
      }
    }
  }
  for (int v = 0; v < 4; ++v) {
    EXPECT_TRUE(seen[v]);
  }
}

// Aggregate: face_number <= 0 triangles are skipped (0 uninitialized, -1 kInvalidId).
TEST(FaceNumberOverlay, face_number_aggregate_skip_nonpositive) {
  lumice::gui::ResetLastCrystalMesh();
  float verts[] = {
    0.0f, 0.0f, 0.0f,  //
    1.0f, 0.0f, 0.0f,  //
    0.0f, 1.0f, 0.0f,  //
  };
  int tris[] = { 0, 1, 2, 0, 1, 2, 0, 1, 2 };
  int fn[] = { -1, 0, 5 };

  FaceLabel labels[kMaxFaceLabels] = {};
  int n = AggregateFaceLabels(verts, 3, tris, 3, fn, labels, kMaxFaceLabels);
  EXPECT_EQ(n, 1);
  EXPECT_EQ(labels[0].face_number, 5);
}

// Project: front-face test — normal pointing +Z with identity rotation +
// center at origin makes n_eye=(0,0,1), p_eye=(0,0,-dist), dot<0 → front.
TEST(FaceNumberOverlay, face_number_project_front_facing) {
  lumice::gui::ResetLastCrystalMesh();
  FaceLabel label{};
  label.display_center[0] = 0.0f;
  label.display_center[1] = 0.0f;
  label.display_center[2] = 0.0f;
  label.display_normal[2] = 1.0f;
  label.face_number = 1;

  float rot[16];
  Identity4x4(rot);
  float mvp[16];
  lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

  float sx = 0.0f;
  float sy = 0.0f;
  bool front = false;
  EXPECT_TRUE(ProjectLabelToScreen(&label, rot, mvp, /*zoom=*/2.0f, 10.0f, 20.0f, 320.0f, 320.0f, &sx, &sy, &front));
  EXPECT_TRUE(front);
}

// Project: rotation around X by 180° flips +Z normal and center stays at
// origin → n_eye=(0,0,-1), p_eye=(0,0,-dist), dot>0 → back.
TEST(FaceNumberOverlay, face_number_project_back_facing_after_180) {
  lumice::gui::ResetLastCrystalMesh();
  FaceLabel label{};
  label.display_normal[2] = 1.0f;
  label.face_number = 1;

  float rot[16];
  RotX180(rot);
  float mvp[16];
  lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

  float sx = 0.0f;
  float sy = 0.0f;
  bool front = false;
  EXPECT_TRUE(ProjectLabelToScreen(&label, rot, mvp, /*zoom=*/2.0f, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front));
  EXPECT_TRUE(!front);
}

// Project: off-axis center + tilted normal where the old `rn_z > 0` rule
// and the new `dot(n_eye, p_eye) < 0` rule diverge. Pre-computed values
// (identity rotation, zoom=0.5, center=(0.8,0,0), normal=(3,0,1)):
//   dist = 0.5 / tan(15°) ≈ 1.866
//   n_eye = (3, 0, 1); n_eye.z = 1 > 0 → old rule says FRONT
//   p_eye = (0.8, 0, 0 - dist) = (0.8, 0, -1.866)
//   dot   = 3*0.8 + 0 + 1*(-1.866) = 2.4 - 1.866 = 0.534 > 0 → new rule says BACK
// This case is the raison d'être of the v14 culling fix: center-offset +
// tilted normal would wrongly display a hidden label under the old rule.
TEST(FaceNumberOverlay, face_number_project_off_axis_back_facing) {
  lumice::gui::ResetLastCrystalMesh();
  FaceLabel label{};
  label.display_center[0] = 0.8f;
  label.display_center[1] = 0.0f;
  label.display_center[2] = 0.0f;
  label.display_normal[0] = 3.0f;  // unnormalized, sign-only matters for dot test
  label.display_normal[1] = 0.0f;
  label.display_normal[2] = 1.0f;
  label.face_number = 1;

  float rot[16];
  Identity4x4(rot);
  float mvp[16];
  constexpr float kZoom = 0.5f;
  lumice::gui::CrystalRenderer::ComputeMvp(rot, kZoom, 320, 320, mvp);

  float sx = 0.0f;
  float sy = 0.0f;
  bool front = false;
  EXPECT_TRUE(ProjectLabelToScreen(&label, rot, mvp, kZoom, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front));
  // Old rule (retired): rn_z = normal.z = 1 > 0 → would say FRONT.
  // New rule: dot ≈ 0.534 > 0 → says BACK. Test asserts the new
  // behavior — the fix.
  EXPECT_TRUE(!front);
}

// ComputeLabelMinWidthRatio: big face passes, tiny fails, sliver fails
// (min-width picks up the narrow dimension even though bbox-width is large
// — this is the v14 core semantic change). Plus degenerate paths:
//   * vertex_count < 3 → false + ratio=0
//   * ε-boundary: one near-degenerate edge coexists with valid edges → still
//     returns a ratio from the valid directions
TEST(FaceNumberOverlay, face_number_min_width_ratio_basic) {
  lumice::gui::ResetLastCrystalMesh();

  float rot[16];
  Identity4x4(rot);
  float mvp[16];
  lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

  // Big face: 4-vertex square at ±0.4 in XY, z=0. Projects to ~20%
  // viewport width each side → ratio well above threshold.
  {
    FaceLabel big{};
    big.face_number = 1;
    big.display_polygon_vertex_count = 4;
    float big_verts[4][3] = {
      { -0.4f, -0.4f, 0.0f },
      { 0.4f, -0.4f, 0.0f },
      { 0.4f, 0.4f, 0.0f },
      { -0.4f, 0.4f, 0.0f },
    };
    for (int i = 0; i < 4; ++i) {
      std::memcpy(big.display_polygon_verts + i * 3, big_verts[i], sizeof(float) * 3);
    }
    float ratio = 0.0f;
    EXPECT_TRUE(ComputeLabelMinWidthRatio(&big, mvp, ImVec2(320.0f, 320.0f), &ratio));
    EXPECT_GT(ratio, kFaceLabelMinViewportRatio);
  }

  // Tiny face: ±0.005 square; ratio well below threshold.
  {
    FaceLabel tiny{};
    tiny.face_number = 2;
    tiny.display_polygon_vertex_count = 4;
    float tiny_verts[4][3] = {
      { -0.005f, -0.005f, 0.0f },
      { 0.005f, -0.005f, 0.0f },
      { 0.005f, 0.005f, 0.0f },
      { -0.005f, 0.005f, 0.0f },
    };
    for (int i = 0; i < 4; ++i) {
      std::memcpy(tiny.display_polygon_verts + i * 3, tiny_verts[i], sizeof(float) * 3);
    }
    float ratio = 0.0f;
    EXPECT_TRUE(ComputeLabelMinWidthRatio(&tiny, mvp, ImVec2(320.0f, 320.0f), &ratio));
    EXPECT_LT(ratio, kFaceLabelMinViewportRatio);
  }

  // Sliver: 1.8 × 0.02 rectangle. AABB-width ratio is ~45% (would pass an
  // AND/OR AABB filter) but min-width (the narrow dimension) is ~0.5% →
  // must fail the v14 filter. Key differential vs v13 semantics.
  {
    FaceLabel sliver{};
    sliver.face_number = 3;
    sliver.display_polygon_vertex_count = 4;
    float sliver_verts[4][3] = {
      { -0.9f, -0.01f, 0.0f },
      { 0.9f, -0.01f, 0.0f },
      { 0.9f, 0.01f, 0.0f },
      { -0.9f, 0.01f, 0.0f },
    };
    for (int i = 0; i < 4; ++i) {
      std::memcpy(sliver.display_polygon_verts + i * 3, sliver_verts[i], sizeof(float) * 3);
    }
    float ratio = 0.0f;
    EXPECT_TRUE(ComputeLabelMinWidthRatio(&sliver, mvp, ImVec2(320.0f, 320.0f), &ratio));
    EXPECT_LT(ratio, kFaceLabelMinViewportRatio);
  }

  // Degenerate: vertex_count=2 → false + ratio=0 (cannot form polygon).
  // Covers the < 3 guard without a separate test registration.
  {
    FaceLabel two{};
    two.face_number = 4;
    two.display_polygon_vertex_count = 2;
    two.display_polygon_verts[0] = 0.0f;
    two.display_polygon_verts[1] = 0.0f;
    two.display_polygon_verts[2] = 0.0f;
    two.display_polygon_verts[3] = 0.3f;
    two.display_polygon_verts[4] = 0.3f;
    two.display_polygon_verts[5] = 0.0f;
    float ratio = 999.0f;  // sentinel: must be cleared to 0 on false return
    EXPECT_TRUE(!ComputeLabelMinWidthRatio(&two, mvp, ImVec2(320.0f, 320.0f), &ratio));
    EXPECT_EQ(ratio, 0.0f);
  }

  // ε-boundary: triangle with one edge shorter than kDegenerateEdgePx but
  // other edges valid. The filter should still return a usable ratio
  // (drawn from the valid edges), not silently return false.
  {
    FaceLabel near_edge{};
    near_edge.face_number = 5;
    near_edge.display_polygon_vertex_count = 3;
    // v0-v1 distance in world ~0.003 → ~0.24 px in 320-viewport, below
    // 0.5 px threshold. v0-v2 and v1-v2 are full-size.
    float near_verts[3][3] = {
      { 0.0f, 0.0f, 0.0f },
      { 0.003f, 0.0f, 0.0f },
      { 0.0015f, 0.4f, 0.0f },
    };
    for (int i = 0; i < 3; ++i) {
      std::memcpy(near_edge.display_polygon_verts + i * 3, near_verts[i], sizeof(float) * 3);
    }
    float ratio = 0.0f;
    // Implementation must still produce a valid ratio from the non-
    // degenerate edges; the triangle is extremely thin → ratio very
    // small but not zero / false.
    bool ok = ComputeLabelMinWidthRatio(&near_edge, mvp, ImVec2(320.0f, 320.0f), &ratio);
    EXPECT_TRUE(ok);
    EXPECT_GT(ratio, 0.0f);
  }
}

// ComputeLabelMinWidthRatio: order-independence — same vertex set in random
// order yields the same ratio (atan2 centroid sort is the invariant).
TEST(FaceNumberOverlay, face_number_min_width_ratio_polygon_ordering) {
  lumice::gui::ResetLastCrystalMesh();

  float rot[16];
  Identity4x4(rot);
  float mvp[16];
  lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

  const float square[4][3] = {
    { -0.3f, -0.3f, 0.0f },
    { 0.3f, -0.3f, 0.0f },
    { 0.3f, 0.3f, 0.0f },
    { -0.3f, 0.3f, 0.0f },
  };
  // Two permutations: sorted CCW and shuffled.
  int orders[2][4] = {
    { 0, 1, 2, 3 },  // CCW
    { 2, 0, 3, 1 },  // interleaved
  };
  float ratios[2] = { 0.0f, 0.0f };
  for (int p = 0; p < 2; ++p) {
    FaceLabel label{};
    label.face_number = p + 1;
    label.display_polygon_vertex_count = 4;
    for (int i = 0; i < 4; ++i) {
      std::memcpy(label.display_polygon_verts + i * 3, square[orders[p][i]], sizeof(float) * 3);
    }
    EXPECT_TRUE(ComputeLabelMinWidthRatio(&label, mvp, ImVec2(320.0f, 320.0f), &ratios[p]));
  }
  EXPECT_TRUE(std::abs(ratios[0] - ratios[1]) < 1e-4f);
}

// Cross-file formula parity: ProjectLabelToScreen's front-face test must
// match the reference formula composed from CrystalRenderer::ComputeEyeRotation
// (see ReferenceFrontFacing above — independently re-derived, not a direct
// call, so the test guards drift rather than mirroring it). Four fixed
// cases spanning identity / off-axis / 180°-flipped / tilt-sensitive
// configurations; **no randomness** to avoid CI flakiness. Each case also
// carries an explicit `expected_front` so a bug that drops V_rot on BOTH
// sides is caught — pure ref==prod comparison alone would be a tautology.
TEST(FaceNumberOverlay, face_number_cull_matches_crystal_renderer_formula) {
  lumice::gui::ResetLastCrystalMesh();

  struct Case {
    float rot[16];
    float zoom;
    float center[3];
    float normal[3];
    bool expected_front;
  };
  std::array<Case, 4> cases{};
  // Case 1: identity, center at origin, +Z normal → n_eye rotated by +15°
  // pitches slightly downward but +Z dominance survives; still front.
  Identity4x4(cases[0].rot);
  cases[0].zoom = 2.0f;
  cases[0].center[2] = 0.0f;
  cases[0].normal[2] = 1.0f;
  cases[0].expected_front = true;
  // Case 2: identity, off-axis center + tilted normal → back
  // (matches the off_axis_back_facing test setup; V_rot doesn't flip it
  // because the +Y tilt of both p and n is small relative to the X terms).
  Identity4x4(cases[1].rot);
  cases[1].zoom = 0.5f;
  cases[1].center[0] = 0.8f;
  cases[1].normal[0] = 3.0f;
  cases[1].normal[2] = 1.0f;
  cases[1].expected_front = false;
  // Case 3: X-180°, +Z normal at origin → back.
  RotX180(cases[2].rot);
  cases[2].zoom = 2.0f;
  cases[2].normal[2] = 1.0f;
  cases[2].expected_front = false;
  // Case 4 (tilt-sensitive): identity rotation, normal at 80° above the
  // "back" hemisphere (0, sin80°, -cos80°) ≈ (0, 0.985, -0.174). Without
  // V_rot, n_eye = normal, p_eye = (0,0,-dist), dot = +0.174·dist > 0 → back.
  // With V_rot (Rx(+15°)), the normal tilts to θ=80°+15°=95° from +Z, giving
  // a slightly forward-facing z-component, so dot flips to < 0 → front.
  // A production regression that drops V_rot flips only prod's output;
  // the reference (still composing V_rot) stays FRONT; EXPECT_EQ fires.
  Identity4x4(cases[3].rot);
  cases[3].zoom = 2.0f;
  constexpr float kTiltCase4 = 80.0f * lumice::gui::CrystalRenderer::kDeg2Rad;
  cases[3].normal[1] = std::sin(kTiltCase4);
  cases[3].normal[2] = -std::cos(kTiltCase4);
  cases[3].expected_front = true;

  for (const auto& c : cases) {
    FaceLabel label{};
    std::memcpy(label.display_center, c.center, sizeof(float) * 3);
    std::memcpy(label.display_normal, c.normal, sizeof(float) * 3);
    label.face_number = 1;
    float mvp[16];
    lumice::gui::CrystalRenderer::ComputeMvp(c.rot, c.zoom, 320, 320, mvp);
    float sx = 0.0f;
    float sy = 0.0f;
    bool front_under_test = false;
    EXPECT_TRUE(
        ProjectLabelToScreen(&label, c.rot, mvp, c.zoom, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front_under_test));
    bool front_reference = ReferenceFrontFacing(c.rot, c.zoom, c.center, c.normal);
    EXPECT_EQ(front_under_test, front_reference);
    EXPECT_EQ(front_under_test, c.expected_front);
  }
}

// ResolveFaceLabelStyle: per-mode draw_hidden / apply_size_filter table +
// X-Ray hidden_fill semantics.
TEST(FaceNumberOverlay, face_number_resolve_style) {
  lumice::gui::ResetLastCrystalMesh();

  auto wireframe = ResolveFaceLabelStyle(CrystalStyle::kWireframe);
  EXPECT_TRUE(wireframe.draw_hidden);
  EXPECT_TRUE(!wireframe.apply_size_filter);

  auto hidden_line = ResolveFaceLabelStyle(CrystalStyle::kHiddenLine);
  EXPECT_TRUE(!hidden_line.draw_hidden);
  EXPECT_TRUE(hidden_line.apply_size_filter);

  auto xray = ResolveFaceLabelStyle(CrystalStyle::kXRay);
  EXPECT_TRUE(xray.draw_hidden);
  EXPECT_TRUE(!xray.apply_size_filter);
  EXPECT_TRUE(xray.hidden_fill != xray.visible_fill);
  int hidden_alpha = static_cast<int>((xray.hidden_fill >> IM_COL32_A_SHIFT) & 0xFFu);
  EXPECT_GE(hidden_alpha, 80);
  EXPECT_LE(hidden_alpha, 200);

  auto shaded = ResolveFaceLabelStyle(CrystalStyle::kShaded);
  EXPECT_TRUE(!shaded.draw_hidden);
  EXPECT_TRUE(shaded.apply_size_filter);

  for (auto s : { wireframe, hidden_line, xray, shaded }) {
    int visible_alpha = static_cast<int>((s.visible_fill >> IM_COL32_A_SHIFT) & 0xFFu);
    EXPECT_GE(visible_alpha, 200);
  }
}

// Project: Y-up NDC maps to Y-down screen. Center at origin lands at image center.
TEST(FaceNumberOverlay, face_number_project_y_flip_center) {
  lumice::gui::ResetLastCrystalMesh();
  FaceLabel label{};
  label.display_normal[2] = 1.0f;
  label.face_number = 1;

  float rot[16];
  Identity4x4(rot);
  float mvp[16];
  lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

  float sx = 0.0f;
  float sy = 0.0f;
  bool front = false;
  EXPECT_TRUE(ProjectLabelToScreen(&label, rot, mvp, /*zoom=*/2.0f, /*image_pos*/ 100.0f, 200.0f,
                                   /*image_size*/ 320.0f, 320.0f, &sx, &sy, &front));
  EXPECT_TRUE(std::abs(sx - (100.0f + 160.0f)) < 1e-3f);
  EXPECT_TRUE(std::abs(sy - (200.0f + 160.0f)) < 1e-3f);
}

// face_normals must be Y-Z swapped
// in the same coordinate frame as vertices before being handed to the overlay.
// Regression that ran without this test: BuildCrystalMeshData swapped
// vertices + edge_face_normals into GL frame but left face_normals in core
// frame, so ProjectLabelToScreen's front/back test compared a GL-frame center
// against a core-frame normal → systematic 90° error.
//
// Uses a view-independent geometric invariant instead of hard-coded face_number
// expectations (which are fragile against mesh topology changes): for a convex
// solid, every face normal must point away from the mesh centroid, i.e.
// dot(normal, center - centroid) > 0. This holds in ANY consistent frame; it
// fails only if the two vectors sit in different frames (the exact bug).
TEST(FaceNumberOverlay, face_number_normal_matches_center_frame) {
  lumice::gui::ResetLastCrystalMesh();

  // Two convex crystals of different topologies, exercising both prism
  // faces and pyramid slanted faces so a partial swap (e.g. side faces only)
  // would still trigger.
  struct Case {
    const char* label;
    lumice::gui::CrystalConfig cfg;
  };
  lumice::gui::CrystalConfig prism_cfg;
  prism_cfg.type = lumice::gui::CrystalType::kPrism;
  prism_cfg.height = 1.0f;
  lumice::gui::CrystalConfig pyramid_cfg;
  pyramid_cfg.type = lumice::gui::CrystalType::kPyramid;
  pyramid_cfg.prism_h = 1.0f;
  pyramid_cfg.upper_h = 0.5f;
  pyramid_cfg.lower_h = 0.5f;
  pyramid_cfg.upper_alpha = 60.0f;
  pyramid_cfg.lower_alpha = 60.0f;
  const Case cases[] = { { "prism", prism_cfg }, { "pyramid", pyramid_cfg } };

  for (const auto& c : cases) {
    LUMICE_CrystalMesh mesh{};
    EXPECT_TRUE(lumice::gui::BuildCrystalMeshData(c.cfg, lumice::gui::kPreviewFixedSampleSeed, &mesh));
    EXPECT_GT(mesh.face_count, 0);

    // Centroid over the GL-frame vertex buffer (already swapped + AABB-
    // normalized inside BuildCrystalMeshData).
    float centroid[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < mesh.vertex_count; ++i) {
      centroid[0] += mesh.vertices[i * 3 + 0];
      centroid[1] += mesh.vertices[i * 3 + 1];
      centroid[2] += mesh.vertices[i * 3 + 2];
    }
    float inv_v = 1.0f / static_cast<float>(mesh.vertex_count);
    centroid[0] *= inv_v;
    centroid[1] *= inv_v;
    centroid[2] *= inv_v;

    FaceLabel labels[lumice::gui::kMaxFaceLabels] = {};
    int n = AggregateFaceLabelsFromTopology(mesh.vertices, mesh.vertex_count, mesh.face_count,
                                            mesh.face_numbers_by_face, mesh.face_vtx_offsets, mesh.face_vtx_counts,
                                            mesh.face_vtx_pool, mesh.face_normals, labels, lumice::gui::kMaxFaceLabels);
    EXPECT_GT(n, 0);

    for (int i = 0; i < n; ++i) {
      const float* ctr = labels[i].display_center;
      const float* nrm = labels[i].display_normal;
      float dx = ctr[0] - centroid[0];
      float dy = ctr[1] - centroid[1];
      float dz = ctr[2] - centroid[2];
      float dot = nrm[0] * dx + nrm[1] * dy + nrm[2] * dz;
      EXPECT_GT(dot, 0.0f) << "crystal=" << c.label << " face=" << labels[i].face_number
                           << " dot(normal, center-centroid) must be > 0; frame mismatch";
    }
  }
}
