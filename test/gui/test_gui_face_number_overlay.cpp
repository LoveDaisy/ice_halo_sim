// Unit tests for face_number_overlay pure functions:
// AggregateFaceLabels, ProjectLabelToScreen, ComputeLabelMinWidthRatio.
// Most tests registered under the "unit" group (no ImGui context required);
// face_number_draw_per_style needs ImGui font atlas and is in "screenshot".

#include <array>
#include <cmath>
#include <cstring>

#include "gui/crystal_preview.hpp"
#include "gui/crystal_renderer.hpp"
#include "gui/face_number_overlay.hpp"
#include "test_gui_shared.hpp"

namespace {

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

// Reference front-face test mirroring crystal_renderer.cpp:358-385 verbatim.
// Adapted from the edge-midpoint + dual edge-face-normal path to the
// face-center + single face-normal case (see F7 notes in plan.md). Formula is
// structurally identical: p_eye = rotation_3x3 * center + (0, 0, -dist);
// n_eye = rotation_3x3 * normal; front = dot(n_eye, p_eye) < 0.
//
// DO NOT modify this reference formula independently of
// crystal_renderer.cpp:358-385 — change crystal_renderer first, then sync
// here. The test face_number_cull_matches_crystal_renderer_formula uses this
// to mechanically guard cross-file formula drift.
bool ReferenceFrontFacing(const float rotation[16], float zoom, const float center[3], const float normal[3]) {
  float dist = lumice::gui::CrystalRenderer::ComputeDist(zoom);
  float px = rotation[0] * center[0] + rotation[4] * center[1] + rotation[8] * center[2];
  float py = rotation[1] * center[0] + rotation[5] * center[1] + rotation[9] * center[2];
  float pz = rotation[2] * center[0] + rotation[6] * center[1] + rotation[10] * center[2] - dist;
  float nx = rotation[0] * normal[0] + rotation[4] * normal[1] + rotation[8] * normal[2];
  float ny = rotation[1] * normal[0] + rotation[5] * normal[1] + rotation[9] * normal[2];
  float nz = rotation[2] * normal[0] + rotation[6] * normal[1] + rotation[10] * normal[2];
  return (nx * px + ny * py + nz * pz) < 0.0f;
}

}  // namespace

void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine) {
  using lumice::gui::AggregateFaceLabels;
  using lumice::gui::CrystalStyle;
  using lumice::gui::FaceLabel;
  using lumice::gui::kFaceLabelMinViewportRatio;
  using lumice::gui::kMaxFaceLabels;
  using lumice::gui::kMaxFacePolygonVerts;
  using lumice::gui::ProjectLabelToScreen;
  using lumice::gui::detail::ComputeLabelMinWidthRatio;
  using lumice::gui::detail::ResolveFaceLabelStyle;

  // Aggregate: 4 triangles, face_numbers = [3, 3, 4, 4] → 2 labels, centers = group means.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_aggregate");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK_EQ(n, 2);

      // Centers are triangle-centroid averages within each group.
      for (int k = 0; k < 2; ++k) {
        if (labels[k].face_number == 3) {
          IM_CHECK(std::abs(labels[k].display_center[0] - 1.0f / 3.0f) < 1e-5f);
          IM_CHECK(std::abs(labels[k].display_center[1] - 1.0f / 3.0f) < 1e-5f);
        } else if (labels[k].face_number == 4) {
          IM_CHECK(std::abs(labels[k].display_center[0] - 7.0f / 3.0f) < 1e-5f);
          IM_CHECK(std::abs(labels[k].display_center[1] - 1.0f / 3.0f) < 1e-5f);
        } else {
          IM_CHECK(false);  // unexpected face_number
        }
      }
    };
  }

  // Aggregate polygon: unique vertex dedup across triangles sharing the same
  // face_number. Replaces the v14-retired face_number_aggregate_bbox test.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_aggregate_polygon");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK_EQ(n, 1);
      IM_CHECK_EQ(labels[0].face_number, 7);
      IM_CHECK_EQ(labels[0].display_polygon_vertex_count, 4);

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
        IM_CHECK(seen[v]);
      }
    };
  }

  // Aggregate: face_number <= 0 triangles are skipped (0 uninitialized, -1 kInvalidId).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_aggregate_skip_nonpositive");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK_EQ(n, 1);
      IM_CHECK_EQ(labels[0].face_number, 5);
    };
  }

  // Project: front-face test — normal pointing +Z with identity rotation +
  // center at origin makes n_eye=(0,0,1), p_eye=(0,0,-dist), dot<0 → front.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_project_front_facing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, /*zoom=*/2.0f, 10.0f, 20.0f, 320.0f, 320.0f, &sx, &sy, &front));
      IM_CHECK(front);
    };
  }

  // Project: rotation around X by 180° flips +Z normal and center stays at
  // origin → n_eye=(0,0,-1), p_eye=(0,0,-dist), dot>0 → back.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_project_back_facing_after_180");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, /*zoom=*/2.0f, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front));
      IM_CHECK(!front);
    };
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
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_project_off_axis_back_facing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, kZoom, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front));
      // Old rule (retired): rn_z = normal.z = 1 > 0 → would say FRONT.
      // New rule: dot ≈ 0.534 > 0 → says BACK. Test asserts the new
      // behavior — the fix.
      IM_CHECK(!front);
    };
  }

  // End-to-end smoke: open Crystal modal, verify GetLastCrystalMesh populates
  // with prism config + at least one labelable face (>0).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "screenshot", "face_number_overlay_smoke");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      lumice::gui::ResetLastCrystalMesh();
      ctx->Yield(2);
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      const auto* mesh = lumice::gui::GetLastCrystalMesh();
      IM_CHECK(mesh != nullptr);
      IM_CHECK_GT(mesh->triangle_count, 0);

      bool any_labelable = false;
      for (int i = 0; i < mesh->triangle_count; ++i) {
        if (mesh->face_numbers[i] > 0) {
          any_labelable = true;
          break;
        }
      }
      IM_CHECK(any_labelable);

      ctx->ItemClick("**/Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // ComputeLabelMinWidthRatio: big face passes, tiny fails, sliver fails
  // (min-width picks up the narrow dimension even though bbox-width is large
  // — this is the v14 core semantic change). Plus degenerate paths:
  //   * vertex_count < 3 → false + ratio=0
  //   * ε-boundary: one near-degenerate edge coexists with valid edges → still
  //     returns a ratio from the valid directions
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_min_width_ratio_basic");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
        IM_CHECK(ComputeLabelMinWidthRatio(&big, mvp, ImVec2(320.0f, 320.0f), &ratio));
        IM_CHECK_GT(ratio, kFaceLabelMinViewportRatio);
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
        IM_CHECK(ComputeLabelMinWidthRatio(&tiny, mvp, ImVec2(320.0f, 320.0f), &ratio));
        IM_CHECK_LT(ratio, kFaceLabelMinViewportRatio);
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
        IM_CHECK(ComputeLabelMinWidthRatio(&sliver, mvp, ImVec2(320.0f, 320.0f), &ratio));
        IM_CHECK_LT(ratio, kFaceLabelMinViewportRatio);
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
        IM_CHECK(!ComputeLabelMinWidthRatio(&two, mvp, ImVec2(320.0f, 320.0f), &ratio));
        IM_CHECK_EQ(ratio, 0.0f);
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
        IM_CHECK(ok);
        IM_CHECK_GT(ratio, 0.0f);
      }
    };
  }

  // ComputeLabelMinWidthRatio: order-independence — same vertex set in random
  // order yields the same ratio (atan2 centroid sort is the invariant).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_min_width_ratio_polygon_ordering");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
        IM_CHECK(ComputeLabelMinWidthRatio(&label, mvp, ImVec2(320.0f, 320.0f), &ratios[p]));
      }
      IM_CHECK(std::abs(ratios[0] - ratios[1]) < 1e-4f);
    };
  }

  // Cross-file formula parity: ProjectLabelToScreen's front-face test must
  // match the reference formula lifted from crystal_renderer.cpp:358-385
  // (see ReferenceFrontFacing above). Three fixed input vectors spanning
  // identity / off-axis / 180°-flipped configurations; **no randomness** to
  // avoid CI flakiness.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_cull_matches_crystal_renderer_formula");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      lumice::gui::ResetLastCrystalMesh();

      struct Case {
        float rot[16];
        float zoom;
        float center[3];
        float normal[3];
      };
      std::array<Case, 3> cases{};
      // Case 1: identity, center at origin, +Z normal → both say front.
      Identity4x4(cases[0].rot);
      cases[0].zoom = 2.0f;
      cases[0].center[0] = 0.0f;
      cases[0].center[1] = 0.0f;
      cases[0].center[2] = 0.0f;
      cases[0].normal[0] = 0.0f;
      cases[0].normal[1] = 0.0f;
      cases[0].normal[2] = 1.0f;
      // Case 2: identity, off-axis center + tilted normal → both say back
      // (matches the off_axis_back_facing test setup).
      Identity4x4(cases[1].rot);
      cases[1].zoom = 0.5f;
      cases[1].center[0] = 0.8f;
      cases[1].center[1] = 0.0f;
      cases[1].center[2] = 0.0f;
      cases[1].normal[0] = 3.0f;
      cases[1].normal[1] = 0.0f;
      cases[1].normal[2] = 1.0f;
      // Case 3: X-180°, +Z normal → both say back.
      RotX180(cases[2].rot);
      cases[2].zoom = 2.0f;
      cases[2].center[0] = 0.0f;
      cases[2].center[1] = 0.0f;
      cases[2].center[2] = 0.0f;
      cases[2].normal[0] = 0.0f;
      cases[2].normal[1] = 0.0f;
      cases[2].normal[2] = 1.0f;

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
        IM_CHECK(
            ProjectLabelToScreen(&label, c.rot, mvp, c.zoom, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front_under_test));
        bool front_reference = ReferenceFrontFacing(c.rot, c.zoom, c.center, c.normal);
        IM_CHECK_EQ(front_under_test, front_reference);
      }
    };
  }

  // ResolveFaceLabelStyle: per-mode draw_hidden / apply_size_filter table +
  // X-Ray hidden_fill semantics.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_resolve_style");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      lumice::gui::ResetLastCrystalMesh();

      auto wireframe = ResolveFaceLabelStyle(CrystalStyle::kWireframe);
      IM_CHECK(wireframe.draw_hidden);
      IM_CHECK(!wireframe.apply_size_filter);

      auto hidden_line = ResolveFaceLabelStyle(CrystalStyle::kHiddenLine);
      IM_CHECK(!hidden_line.draw_hidden);
      IM_CHECK(hidden_line.apply_size_filter);

      auto xray = ResolveFaceLabelStyle(CrystalStyle::kXRay);
      IM_CHECK(xray.draw_hidden);
      IM_CHECK(!xray.apply_size_filter);
      IM_CHECK(xray.hidden_fill != xray.visible_fill);
      int hidden_alpha = static_cast<int>((xray.hidden_fill >> IM_COL32_A_SHIFT) & 0xFFu);
      IM_CHECK_GE(hidden_alpha, 80);
      IM_CHECK_LE(hidden_alpha, 200);

      auto shaded = ResolveFaceLabelStyle(CrystalStyle::kShaded);
      IM_CHECK(!shaded.draw_hidden);
      IM_CHECK(shaded.apply_size_filter);

      for (auto s : { wireframe, hidden_line, xray, shaded }) {
        int visible_alpha = static_cast<int>((s.visible_fill >> IM_COL32_A_SHIFT) & 0xFFu);
        IM_CHECK_GE(visible_alpha, 200);
      }
    };
  }

  // DrawFaceNumberOverlay: per-mode strict-inequality vertex-count test.
  //   - kWireframe / kXRay must draw hidden labels;
  //   - kHiddenLine / kShaded must still draw the visible label (size filter
  //     passes for a face large enough to clear the threshold).
  //
  // Pre-flight in v14 uses ComputeLabelMinWidthRatio (replaces v13 bbox
  // ratio) to verify the visible face clears threshold before the strict-
  // inequality assertions run.
  {
    static struct DrawPerStyleCapture {
      bool arrange_ok = false;
      int hidden_count = 0;
      int visible_count = 0;
      bool visible_passes_size_filter = false;
      std::array<int, 4> vertex_delta = { 0, 0, 0, 0 };  // indexed by CrystalStyle
      bool done = false;
    } g_capture;

    ImGuiTest* t = IM_REGISTER_TEST(engine, "screenshot", "face_number_draw_per_style");
    t->GuiFunc = [](ImGuiTestContext*) {
      if (g_capture.done) {
        return;
      }

      // Two triangles, two face_numbers, opposing normals.
      static float verts[] = {
        -0.45f, -0.45f, 0.0f,  // v0
        0.45f,  -0.45f, 0.0f,  // v1
        0.0f,   0.45f,  0.0f,  // v2
        -0.45f, -0.45f, 0.0f,  // v3
        0.0f,   0.45f,  0.0f,  // v4
        0.45f,  -0.45f, 0.0f,  // v5
      };
      static int tris[] = {
        0, 1, 2,  // face 1 → cross(v1-v0, v2-v0).z > 0 → +z normal → front
        3, 4, 5,  // face 2 → reversed winding → -z normal → back
      };
      static int face_numbers[] = { 1, 2 };

      float rot[16];
      Identity4x4(rot);
      float mvp[16];
      constexpr float kZoom = 2.0f;
      lumice::gui::CrystalRenderer::ComputeMvp(rot, kZoom, 320, 320, mvp);

      lumice::gui::FaceLabel labels[lumice::gui::kMaxFaceLabels] = {};
      int n = AggregateFaceLabels(verts, 6, tris, 2, face_numbers, labels, lumice::gui::kMaxFaceLabels);
      g_capture.hidden_count = 0;
      g_capture.visible_count = 0;
      g_capture.visible_passes_size_filter = false;
      for (int i = 0; i < n; ++i) {
        float sx = 0.0f;
        float sy = 0.0f;
        bool front = false;
        if (!ProjectLabelToScreen(&labels[i], rot, mvp, kZoom, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front)) {
          continue;
        }
        if (front) {
          ++g_capture.visible_count;
          float ratio = 0.0f;
          if (ComputeLabelMinWidthRatio(&labels[i], mvp, ImVec2(320.0f, 320.0f), &ratio)) {
            if (ratio >= kFaceLabelMinViewportRatio) {
              g_capture.visible_passes_size_filter = true;
            }
          }
        } else {
          ++g_capture.hidden_count;
        }
      }
      g_capture.arrange_ok =
          (g_capture.hidden_count > 0) && (g_capture.visible_count > 0) && g_capture.visible_passes_size_filter;

      CrystalStyle styles[4] = {
        CrystalStyle::kWireframe,
        CrystalStyle::kHiddenLine,
        CrystalStyle::kXRay,
        CrystalStyle::kShaded,
      };
      for (int s = 0; s < 4; ++s) {
        ImGui::SetNextWindowSize(ImVec2(320, 320), ImGuiCond_Always);
        char title[64];
        std::snprintf(title, sizeof(title), "##FaceNumberPerStyleHost_%d", s);
        if (ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse)) {
          ImDrawList* draw_list = ImGui::GetWindowDrawList();
          int before = draw_list->VtxBuffer.Size;
          lumice::gui::DrawFaceNumberOverlay(verts, 6, tris, 2, face_numbers, rot, mvp, kZoom, ImVec2(0.0f, 0.0f),
                                             ImVec2(320.0f, 320.0f), draw_list, styles[s]);
          int after = draw_list->VtxBuffer.Size;
          g_capture.vertex_delta[s] = after - before;
        }
        ImGui::End();
      }

      g_capture.done = true;
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_capture = {};
      ctx->Yield(3);
      IM_CHECK(g_capture.done);
      IM_CHECK_GT(g_capture.hidden_count, 0);
      IM_CHECK_GT(g_capture.visible_count, 0);
      IM_CHECK(g_capture.visible_passes_size_filter);
      IM_CHECK(g_capture.arrange_ok);

      int wire = g_capture.vertex_delta[0];
      int hidden_line = g_capture.vertex_delta[1];
      int xray = g_capture.vertex_delta[2];
      int shaded = g_capture.vertex_delta[3];

      IM_CHECK_GT(wire, 0);
      IM_CHECK_GT(hidden_line, 0);
      IM_CHECK_GT(xray, 0);
      IM_CHECK_GT(shaded, 0);

      IM_CHECK_GT(wire, hidden_line);
      IM_CHECK_GT(xray, shaded);
    };
  }

  // Project: Y-up NDC maps to Y-down screen. Center at origin lands at image center.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_project_y_flip_center");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, /*zoom=*/2.0f, /*image_pos*/ 100.0f, 200.0f,
                                    /*image_size*/ 320.0f, 320.0f, &sx, &sy, &front));
      IM_CHECK(std::abs(sx - (100.0f + 160.0f)) < 1e-3f);
      IM_CHECK(std::abs(sy - (200.0f + 160.0f)) < 1e-3f);
    };
  }
}
