// Which face numbers the crystal preview writes on the crystal, and where.
//
// Four pure functions decide that, and each one owns a way the overlay can be wrong in a way the
// user reads as a drawing bug rather than a maths bug: AggregateFaceLabels decides how many labels
// there are and where their anchors sit, ProjectLabelToScreen decides where each lands and whether
// the face is even pointing at the camera, ComputeLabelMinWidthRatio decides whether a face is big
// enough on screen to hold a number, and ResolveFaceLabelStyle decides what each display mode shows.
//
// The three cases that need the font atlas or a rendered frame stay in
// test/gui/functional/test_gui_face_number_overlay.cpp.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <vector>

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
using lumice::gui::detail::ComputeLabelMinWidthRatio;
using lumice::gui::detail::ResolveFaceLabelStyle;

constexpr float kViewport = 320.0f;
using Vec3 = std::array<float, 3>;

void Identity4x4(float m[16]) {
  std::memset(m, 0, sizeof(float) * 16);
  m[0] = m[5] = m[10] = m[15] = 1.0f;
}

// Rotation around X by 180 degrees, column-major.
void RotX180(float m[16]) {
  std::memset(m, 0, sizeof(float) * 16);
  m[0] = 1.0f;
  m[5] = -1.0f;
  m[10] = -1.0f;
  m[15] = 1.0f;
}

// The camera the preview actually uses, packaged so a case can state its rotation and zoom and stop
// there. Every projection assertion below needs both the rotation (the cull test consumes it
// directly) and the MVP built from it, and building one without the other is how they drift.
struct View {
  float rot[16] = {};
  float mvp[16] = {};

  static View Identity(float zoom) {
    View v;
    Identity4x4(v.rot);
    lumice::gui::CrystalRenderer::ComputeMvp(v.rot, zoom, 320, 320, v.mvp);
    return v;
  }
  static View FlippedX(float zoom) {
    View v;
    RotX180(v.rot);
    lumice::gui::CrystalRenderer::ComputeMvp(v.rot, zoom, 320, 320, v.mvp);
    return v;
  }
};

FaceLabel MakeLabel(const Vec3& center, const Vec3& normal) {
  FaceLabel label{};
  std::memcpy(label.display_center, center.data(), sizeof(float) * 3);
  std::memcpy(label.display_normal, normal.data(), sizeof(float) * 3);
  label.face_number = 1;
  return label;
}

FaceLabel MakePolygonLabel(std::initializer_list<Vec3> verts) {
  FaceLabel label{};
  label.face_number = 1;
  label.display_polygon_vertex_count = static_cast<int>(verts.size());
  int i = 0;
  for (const Vec3& v : verts) {
    std::memcpy(label.display_polygon_verts + i * 3, v.data(), sizeof(float) * 3);
    ++i;
  }
  return label;
}

float MinWidthRatioOf(const FaceLabel& label, const View& view) {
  float ratio = 0.0f;
  FaceLabel copy = label;
  EXPECT_TRUE(ComputeLabelMinWidthRatio(&copy, view.mvp, ImVec2(kViewport, kViewport), &ratio));
  return ratio;
}

// An INDEPENDENT re-derivation of the eye-space front/back formula the renderer uses, so the parity
// case below guards cross-file drift instead of being a tautology on the production helper. The
// Rx(+kCameraTiltDeg) components are hand-written rather than obtained from
// CrystalRenderer::ComputeEyeRotation, which is the whole point: a wrong tilt sign, a missing V_rot
// or a swapped composition order on the production side is caught rather than silently mirrored.
//
//   m_eye = Rx(+kCameraTiltDeg) . rotation;  p_eye = m_eye . center + (0, 0, -dist)
//   n_eye = m_eye . normal;                  front = dot(n_eye, p_eye) < 0
bool ReferenceFrontFacing(const float rotation[16], float zoom, const Vec3& center, const Vec3& normal) {
  const float angle = lumice::gui::kCameraTiltDeg * lumice::gui::CrystalRenderer::kDeg2Rad;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  // V_rot = Rx(+kCameraTiltDeg), column-major 3x3. Row 0: (1, 0, 0); row 1: (0, c, -s); row 2: (0, s, c).
  const float v_rot[9] = {
    1.0f, 0.0f, 0.0f,  // col 0
    0.0f, c,    s,     // col 1
    0.0f, -s,   c,     // col 2
  };
  float m_eye[9] = {};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float sum = 0.0f;
      for (int k = 0; k < 3; ++k) {
        // rotation is column-major 4x4, so column k lives at [k*4 + row].
        sum += v_rot[i + k * 3] * rotation[k + j * 4];
      }
      m_eye[i + j * 3] = sum;
    }
  }
  const float dist = lumice::gui::CrystalRenderer::ComputeDist(zoom);
  const float px = m_eye[0] * center[0] + m_eye[3] * center[1] + m_eye[6] * center[2];
  const float py = m_eye[1] * center[0] + m_eye[4] * center[1] + m_eye[7] * center[2];
  const float pz = m_eye[2] * center[0] + m_eye[5] * center[1] + m_eye[8] * center[2] - dist;
  const float nx = m_eye[0] * normal[0] + m_eye[3] * normal[1] + m_eye[6] * normal[2];
  const float ny = m_eye[1] * normal[0] + m_eye[4] * normal[1] + m_eye[7] * normal[2];
  const float nz = m_eye[2] * normal[0] + m_eye[5] * normal[1] + m_eye[8] * normal[2];
  return (nx * px + ny * py + nz * pz) < 0.0f;
}

}  // namespace

// ---- One label per face, anchored at the face's own centre ----
//
// The mesh arrives as triangles; a face is a group of them sharing a face number, and the label has
// to end up at the group's centre with the face's full outline behind it — the outline is what the
// size filter below measures, so a polygon that kept duplicate shared vertices would measure the
// wrong shape. Triangles with a non-positive face number are the unassigned ones and must not
// produce a label at all, since a "0" floating on a crystal is worse than no number.
TEST(FaceNumberOverlay, TrianglesSharingAFaceNumberBecomeOneLabelAtTheirSharedCentre) {
  lumice::gui::ResetLastCrystalMesh();
  // Two separated triangle pairs. Each pair covers the same triangle twice, so the group centre is
  // the triangle centroid and duplicate coverage does not shift it.
  const float verts[] = {
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // face 3
    2.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 2.0f, 1.0f, 0.0f,  // face 4
  };
  const int tris[] = { 0, 1, 2, 0, 2, 1, 3, 4, 5, 3, 5, 4 };
  const int fn[] = { 3, 3, 4, 4 };

  FaceLabel labels[kMaxFaceLabels] = {};
  ASSERT_EQ(AggregateFaceLabels(verts, 6, tris, 4, fn, labels, kMaxFaceLabels), 2);
  for (int k = 0; k < 2; ++k) {
    const float expected_x = labels[k].face_number == 3 ? 1.0f / 3.0f : 7.0f / 3.0f;
    EXPECT_TRUE(labels[k].face_number == 3 || labels[k].face_number == 4) << labels[k].face_number;
    EXPECT_NEAR(labels[k].display_center[0], expected_x, 1e-5f);
    EXPECT_NEAR(labels[k].display_center[1], 1.0f / 3.0f, 1e-5f);
  }
}

TEST(FaceNumberOverlay, AFacesOutlineKeepsEachSharedVertexExactlyOnce) {
  lumice::gui::ResetLastCrystalMesh();
  // A quad as two triangles sharing an edge: four unique vertices, six triangle corners.
  const float verts[] = {
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
  };
  const int tris[] = { 0, 1, 2, 0, 2, 3 };
  const int fn[] = { 7, 7 };

  FaceLabel labels[kMaxFaceLabels] = {};
  ASSERT_EQ(AggregateFaceLabels(verts, 4, tris, 2, fn, labels, kMaxFaceLabels), 1);
  EXPECT_EQ(labels[0].face_number, 7);
  ASSERT_EQ(labels[0].display_polygon_vertex_count, 4);

  // Every unique vertex appears, in triangle-discovery order rather than a geometric one.
  bool seen[4] = {};
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
    EXPECT_TRUE(seen[v]) << "vertex " << v;
  }
}

TEST(FaceNumberOverlay, TrianglesWithNoFaceAssignedProduceNoLabel) {
  lumice::gui::ResetLastCrystalMesh();
  const float verts[] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
  const int tris[] = { 0, 1, 2, 0, 1, 2, 0, 1, 2 };
  const int fn[] = { -1, 0, 5 };  // kInvalidId, uninitialized, and one real face

  FaceLabel labels[kMaxFaceLabels] = {};
  ASSERT_EQ(AggregateFaceLabels(verts, 3, tris, 3, fn, labels, kMaxFaceLabels), 1);
  EXPECT_EQ(labels[0].face_number, 5);
}

// ---- A label on a face turned away from the camera must not be drawn, and lands where it should ----
//
// The retired rule was "the normal's eye-space z points at us", which is right only for a face whose
// centre is on the view axis. The rule that replaced it asks whether the face turns away from the
// point it actually sits at: dot(n_eye, p_eye) < 0. The off-axis row is the whole reason for the
// change — under the old rule its label was drawn on the far side of the crystal, floating over
// geometry that hides it.
//
// Every row is cross-checked against the independently re-derived formula above AND carries its own
// expected verdict, because a pure reference-vs-production comparison is a tautology against a bug
// that drops V_rot on BOTH sides — the last row exists precisely to be sensitive to that term, being
// a normal whose verdict the camera tilt alone decides. Four fixed configurations, no randomness: a
// flaky geometry test teaches people to re-run rather than to look.
//
// The rows also answer where the label lands, which is the same call's other output and needs no
// staging of its own: NDC is Y-up and the screen is Y-down, so a face at the origin must reach the
// middle of the image rectangle wherever in the window that rectangle sits.
TEST(FaceNumberOverlay, AFaceIsVisibleByItsOwnPositionAndItsLabelLandsWhereItProjects) {
  lumice::gui::ResetLastCrystalMesh();
  constexpr float kTilted = 80.0f * lumice::gui::CrystalRenderer::kDeg2Rad;
  constexpr float kImageX = 100.0f;
  constexpr float kImageY = 200.0f;

  struct CullCase {
    const char* name;
    View view;
    float zoom;
    Vec3 center;
    Vec3 normal;
    bool expect_front;
    bool at_image_centre;  // true for a face sitting on the origin, whatever way it faces
  };
  const CullCase kCases[] = {
    { "facing the camera at the view axis", View::Identity(2.0f), 2.0f, { 0, 0, 0 }, { 0, 0, 1 }, true, true },
    // Rotating the crystal by half a turn puts the same face on the back.
    { "the same face after a half turn", View::FlippedX(2.0f), 2.0f, { 0, 0, 0 }, { 0, 0, 1 }, false, true },
    // n_eye.z is +1 here, so the retired rule called this front. p_eye is (0.8, 0, -1.866) and the
    // dot is +0.534, so the face is genuinely turned away.
    { "off the view axis with a tilted normal", View::Identity(0.5f), 0.5f, { 0.8f, 0, 0 }, { 3, 0, 1 }, false, false },
    // 80 degrees into the back hemisphere. Without the camera tilt the dot is +0.174*dist and this
    // reads as back; with it the normal reaches 95 degrees from +Z and the dot flips. A production
    // regression that drops V_rot moves only the left-hand side, and the row goes red.
    { "tilt-sensitive: 80 degrees into the back hemisphere",
      View::Identity(2.0f),
      2.0f,
      { 0, 0, 0 },
      { 0.0f, std::sin(kTilted), -std::cos(kTilted) },
      true,
      true },
  };

  for (const CullCase& c : kCases) {
    FaceLabel label = MakeLabel(c.center, c.normal);
    float sx = 0.0f;
    float sy = 0.0f;
    bool front = false;
    EXPECT_TRUE(ProjectLabelToScreen(&label, c.view.rot, c.view.mvp, c.zoom, kImageX, kImageY, kViewport, kViewport,
                                     &sx, &sy, &front))
        << c.name;
    EXPECT_EQ(front, ReferenceFrontFacing(c.view.rot, c.zoom, c.center, c.normal)) << c.name;
    EXPECT_EQ(front, c.expect_front) << c.name;
    if (c.at_image_centre) {
      EXPECT_NEAR(sx, kImageX + kViewport / 2.0f, 1e-3f) << c.name;
      EXPECT_NEAR(sy, kImageY + kViewport / 2.0f, 1e-3f) << c.name;
    }
  }
}

// ---- A face gets a number only if there is room to read it ----
//
// The measure is the polygon's NARROWEST on-screen width, not its bounding box. The sliver row is
// what separates the two: a long thin face fills nearly half the viewport by bounding box while
// being two pixels tall, so a bounding-box filter puts a digit on a face that cannot hold one.
TEST(FaceNumberOverlay, TheSizeFilterMeasuresAFacesNarrowestWidthNotItsBoundingBox) {
  lumice::gui::ResetLastCrystalMesh();
  const View view = View::Identity(2.0f);

  struct SizeCase {
    const char* name;
    FaceLabel label;
    bool expect_above_threshold;
  };
  const SizeCase kCases[] = {
    { "a face filling a fifth of the viewport",
      MakePolygonLabel({ { -0.4f, -0.4f, 0 }, { 0.4f, -0.4f, 0 }, { 0.4f, 0.4f, 0 }, { -0.4f, 0.4f, 0 } }), true },
    { "a face a few pixels across",
      MakePolygonLabel(
          { { -0.005f, -0.005f, 0 }, { 0.005f, -0.005f, 0 }, { 0.005f, 0.005f, 0 }, { -0.005f, 0.005f, 0 } }),
      false },
    // 1.8 x 0.02: about 45% of the viewport wide, about 0.5% narrow.
    { "a sliver that is wide but unreadably thin",
      MakePolygonLabel({ { -0.9f, -0.01f, 0 }, { 0.9f, -0.01f, 0 }, { 0.9f, 0.01f, 0 }, { -0.9f, 0.01f, 0 } }), false },
  };

  for (const SizeCase& c : kCases) {
    const float ratio = MinWidthRatioOf(c.label, view);
    if (c.expect_above_threshold) {
      EXPECT_GT(ratio, kFaceLabelMinViewportRatio) << c.name;
    } else {
      EXPECT_LT(ratio, kFaceLabelMinViewportRatio) << c.name;
    }
  }

  // Fewer than three vertices is not a polygon: the filter reports failure AND clears the output, so
  // a caller that ignores the return value still cannot act on a stale ratio.
  FaceLabel two = MakePolygonLabel({ { 0, 0, 0 }, { 0.3f, 0.3f, 0 } });
  float ratio = 999.0f;  // sentinel: must be cleared on the false return
  EXPECT_FALSE(ComputeLabelMinWidthRatio(&two, view.mvp, ImVec2(kViewport, kViewport), &ratio));
  EXPECT_EQ(ratio, 0.0f);

  // One sub-pixel edge among valid ones must not make the whole face unmeasurable — the ratio comes
  // from the directions that are still meaningful, so an extremely thin face reads as thin rather
  // than as an error.
  FaceLabel near_degenerate = MakePolygonLabel({ { 0, 0, 0 }, { 0.003f, 0, 0 }, { 0.0015f, 0.4f, 0 } });
  EXPECT_GT(MinWidthRatioOf(near_degenerate, view), 0.0f);

  // The same vertex set in a different order is the same face. The filter sorts by angle around the
  // centroid before measuring, so its answer cannot depend on the order the triangles happened to be
  // discovered in — which is not stable across mesh builds.
  const Vec3 s[4] = { { -0.3f, -0.3f, 0 }, { 0.3f, -0.3f, 0 }, { 0.3f, 0.3f, 0 }, { -0.3f, 0.3f, 0 } };
  const float ccw = MinWidthRatioOf(MakePolygonLabel({ s[0], s[1], s[2], s[3] }), view);
  const float interleaved = MinWidthRatioOf(MakePolygonLabel({ s[2], s[0], s[3], s[1] }), view);
  EXPECT_NEAR(ccw, interleaved, 1e-4f);
}

// ---- What each display mode shows ----
//
// The two flags are not independent knobs: a mode that draws the crystal's hidden edges must also
// label the hidden faces (otherwise the numbers disagree with the wireframe the user is looking
// through), and a mode that occludes them must apply the size filter (a solid crystal's faces
// foreshorten to nothing at grazing angles). X-Ray is the one mode that shows hidden labels
// DIFFERENTLY rather than identically, and its dimming has to stay in a band: too faint and the
// feature is invisible, too strong and it competes with the front faces.
TEST(FaceNumberOverlay, EachDisplayModeLabelsHiddenFacesExactlyWhenItDrawsThrough) {
  lumice::gui::ResetLastCrystalMesh();

  struct StyleCase {
    const char* name;
    CrystalStyle style;
    bool draws_through;
  };
  const StyleCase kCases[] = {
    { "wireframe", CrystalStyle::kWireframe, true },
    { "x-ray", CrystalStyle::kXRay, true },
    { "hidden-line", CrystalStyle::kHiddenLine, false },
    { "shaded", CrystalStyle::kShaded, false },
  };

  for (const StyleCase& c : kCases) {
    const auto style = ResolveFaceLabelStyle(c.style);
    EXPECT_EQ(style.draw_hidden, c.draws_through) << c.name;
    EXPECT_EQ(style.apply_size_filter, !c.draws_through) << c.name;
    // A visible label is a visible label in every mode.
    EXPECT_GE(static_cast<int>((style.visible_fill >> IM_COL32_A_SHIFT) & 0xFFu), 200) << c.name;
  }

  const auto xray = ResolveFaceLabelStyle(CrystalStyle::kXRay);
  EXPECT_NE(xray.hidden_fill, xray.visible_fill);
  const int hidden_alpha = static_cast<int>((xray.hidden_fill >> IM_COL32_A_SHIFT) & 0xFFu);
  EXPECT_GE(hidden_alpha, 80);
  EXPECT_LE(hidden_alpha, 200);
}

// ---- A face's normal and its centre must be expressed in the same frame ----
//
// The regression: BuildCrystalMeshData swapped vertices and edge normals into the GL frame but left
// face_normals in the core frame, so the cull test above compared a GL-frame centre against a
// core-frame normal and got a systematic 90-degree error — every label on the wrong half of the
// crystal.
//
// Asserted through a frame-independent invariant rather than hard-coded face numbers, which would be
// fragile against any topology change: on a convex solid every face normal points away from the
// centroid, so dot(normal, centre - centroid) > 0 in ANY consistent frame. It fails only when the
// two vectors are in different ones, which is exactly the bug. Both crystal families are exercised
// because a partial swap — side faces only — would still pass on one of them.
TEST(FaceNumberOverlay, EveryFaceNormalPointsAwayFromTheCrystalCentre) {
  lumice::gui::ResetLastCrystalMesh();

  lumice::gui::CrystalConfig prism;
  prism.type = lumice::gui::CrystalType::kPrism;
  prism.height = 1.0f;
  lumice::gui::CrystalConfig pyramid;
  pyramid.type = lumice::gui::CrystalType::kPyramid;
  pyramid.prism_h = 1.0f;
  pyramid.upper_h = 0.5f;
  pyramid.lower_h = 0.5f;
  pyramid.upper_alpha = 60.0f;
  pyramid.lower_alpha = 60.0f;

  struct CrystalCase {
    const char* name;
    lumice::gui::CrystalConfig cfg;
  };
  const CrystalCase kCases[] = { { "prism", prism }, { "pyramid", pyramid } };

  for (const CrystalCase& c : kCases) {
    LUMICE_CrystalMesh mesh{};
    if (!lumice::gui::BuildCrystalMeshData(c.cfg, lumice::gui::kPreviewFixedSampleSeed, &mesh)) {
      ADD_FAILURE() << c.name << ": mesh build failed";
      continue;  // no mesh to inspect for this crystal; the rest still get checked
    }
    if (mesh.face_count <= 0) {
      ADD_FAILURE() << c.name << ": mesh has no faces";
      continue;
    }

    // Centroid over the GL-frame vertex buffer, already swapped and AABB-normalized by the builder.
    float centroid[3] = {};
    for (int i = 0; i < mesh.vertex_count; ++i) {
      for (int k = 0; k < 3; ++k) {
        centroid[k] += mesh.vertices[i * 3 + k];
      }
    }
    for (float& v : centroid) {
      v /= static_cast<float>(mesh.vertex_count);
    }

    FaceLabel labels[kMaxFaceLabels] = {};
    const int n = AggregateFaceLabelsFromTopology(
        mesh.vertices, mesh.vertex_count, mesh.face_count, mesh.face_numbers_by_face, mesh.face_vtx_offsets,
        mesh.face_vtx_counts, mesh.face_vtx_pool, mesh.face_normals, labels, kMaxFaceLabels);
    if (n <= 0) {
      ADD_FAILURE() << c.name << ": no face labels aggregated";
      continue;
    }

    for (int i = 0; i < n; ++i) {
      float dot = 0.0f;
      for (int k = 0; k < 3; ++k) {
        dot += labels[i].display_normal[k] * (labels[i].display_center[k] - centroid[k]);
      }
      EXPECT_GT(dot, 0.0f) << c.name << " face " << labels[i].face_number
                           << ": the normal and the centre are in different frames";
    }
  }
}
