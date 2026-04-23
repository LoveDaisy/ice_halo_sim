// Unit tests for face_number_overlay pure functions:
// AggregateFaceLabels and ProjectLabelToScreen.
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

}  // namespace

void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine) {
  using lumice::gui::AggregateFaceLabels;
  using lumice::gui::CrystalStyle;
  using lumice::gui::FaceLabel;
  using lumice::gui::kFaceLabelMinViewportRatio;
  using lumice::gui::kMaxFaceLabels;
  using lumice::gui::ProjectLabelToScreen;
  using lumice::gui::detail::ComputeLabelScreenBboxRatio;
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
      // Group 3: both triangles share centroid ((0+1+0)/3, (0+0+1)/3, 0) = (1/3, 1/3, 0)
      // Group 4: both triangles share centroid ((2+3+2)/3, (0+0+1)/3, 0) = (7/3, 1/3, 0)
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

  // Aggregate: AABB tracks component-wise min/max across all triangle vertices
  // sharing the same face_number.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_aggregate_bbox");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      lumice::gui::ResetLastCrystalMesh();
      // Two triangles sharing face_number=7: vertices span x∈[-1,3], y∈[0,2], z∈[-0.5,0.5].
      float verts[] = {
        -1.0f, 0.0f, 0.0f,   // v0
        3.0f,  2.0f, 0.5f,   // v1
        0.0f,  1.0f, -0.5f,  // v2
        2.0f,  0.0f, 0.0f,   // v3
        1.0f,  2.0f, 0.0f,   // v4
        0.0f,  1.0f, 0.5f,   // v5
      };
      int tris[] = {
        0, 1, 2,  // face 7
        3, 4, 5,  // face 7
      };
      int fn[] = { 7, 7 };

      FaceLabel labels[kMaxFaceLabels] = {};
      int n = AggregateFaceLabels(verts, 6, tris, 2, fn, labels, kMaxFaceLabels);
      IM_CHECK_EQ(n, 1);
      IM_CHECK_EQ(labels[0].face_number, 7);
      // Min: per-axis component-wise minimum across all 6 vertices.
      IM_CHECK(std::abs(labels[0].display_aabb_min[0] - (-1.0f)) < 1e-5f);
      IM_CHECK(std::abs(labels[0].display_aabb_min[1] - 0.0f) < 1e-5f);
      IM_CHECK(std::abs(labels[0].display_aabb_min[2] - (-0.5f)) < 1e-5f);
      // Max: per-axis component-wise maximum.
      IM_CHECK(std::abs(labels[0].display_aabb_max[0] - 3.0f) < 1e-5f);
      IM_CHECK(std::abs(labels[0].display_aabb_max[1] - 2.0f) < 1e-5f);
      IM_CHECK(std::abs(labels[0].display_aabb_max[2] - 0.5f) < 1e-5f);
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

  // Project: front-face test — normal pointing +Z with identity rotation
  // makes rn_z > 0, so face is front-facing.
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, 10.0f, 20.0f, 320.0f, 320.0f, &sx, &sy, &front));
      IM_CHECK(front);
    };
  }

  // Project: rotation around X by 180° flips +Z normal to -Z → back-facing.
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front));
      IM_CHECK(!front);
    };
  }

  // End-to-end smoke: open Crystal modal, verify GetLastCrystalMesh populates
  // with prism config + at least one labelable face (>0). This is the degraded
  // path for the 3-style PSNR regression (plan §7 Risk 4 — ImGui text anti-
  // aliasing is non-deterministic across MSAA state, so strict PSNR would
  // produce flaky failures; smoke coverage instead verifies the overlay
  // pipeline is wired and the mesh view exposes real face numbers).
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

  // ComputeLabelScreenBboxRatio: a large face spanning the central frustum
  // produces a screen bbox that comfortably exceeds the threshold; a tiny
  // face produces a bbox below it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_bbox_ratio_basic");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      lumice::gui::ResetLastCrystalMesh();

      float rot[16];
      Identity4x4(rot);
      float mvp[16];
      lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

      // Large face: AABB centered at origin spanning ±0.4 in display space —
      // well within the view frustum, projects to a sizable screen bbox.
      FaceLabel big{};
      big.face_number = 1;
      big.display_aabb_min[0] = -0.4f;
      big.display_aabb_min[1] = -0.4f;
      big.display_aabb_min[2] = -0.4f;
      big.display_aabb_max[0] = 0.4f;
      big.display_aabb_max[1] = 0.4f;
      big.display_aabb_max[2] = 0.4f;

      float w = 0.0f;
      float h = 0.0f;
      IM_CHECK(ComputeLabelScreenBboxRatio(&big, mvp, ImVec2(320.0f, 320.0f), &w, &h));
      IM_CHECK_GT(w, kFaceLabelMinViewportRatio);
      IM_CHECK_GT(h, kFaceLabelMinViewportRatio);

      // Tiny face: AABB centered at origin spanning ±0.005, well below the
      // 10% threshold at this zoom/viewport.
      FaceLabel tiny{};
      tiny.face_number = 2;
      tiny.display_aabb_min[0] = -0.005f;
      tiny.display_aabb_min[1] = -0.005f;
      tiny.display_aabb_min[2] = -0.005f;
      tiny.display_aabb_max[0] = 0.005f;
      tiny.display_aabb_max[1] = 0.005f;
      tiny.display_aabb_max[2] = 0.005f;

      IM_CHECK(ComputeLabelScreenBboxRatio(&tiny, mvp, ImVec2(320.0f, 320.0f), &w, &h));
      IM_CHECK_LT(w, kFaceLabelMinViewportRatio);
      IM_CHECK_LT(h, kFaceLabelMinViewportRatio);
    };
  }

  // ResolveFaceLabelStyle: per-mode draw_hidden / apply_size_filter table +
  // X-Ray hidden_fill semantics. Asserts only on fields that the consumer
  // (DrawFaceNumberOverlay) actually reads — hidden_* fields are dead under
  // draw_hidden=false (kHiddenLine / kShaded) and intentionally untested.
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
      // Hidden-face X-Ray fill must be visually distinct from the visible fill
      // and within the calibrated alpha band [80, 200] (~30%-80% opaque).
      IM_CHECK(xray.hidden_fill != xray.visible_fill);
      int hidden_alpha = static_cast<int>((xray.hidden_fill >> IM_COL32_A_SHIFT) & 0xFFu);
      IM_CHECK_GE(hidden_alpha, 80);
      IM_CHECK_LE(hidden_alpha, 200);

      auto shaded = ResolveFaceLabelStyle(CrystalStyle::kShaded);
      IM_CHECK(!shaded.draw_hidden);
      IM_CHECK(shaded.apply_size_filter);

      // visible_fill across all modes must be near-fully opaque so labels stay
      // readable on top of the GL preview (asserts the only universally
      // semantically meaningful color field).
      for (auto s : { wireframe, hidden_line, xray, shaded }) {
        int visible_alpha = static_cast<int>((s.visible_fill >> IM_COL32_A_SHIFT) & 0xFFu);
        IM_CHECK_GE(visible_alpha, 200);
      }
    };
  }

  // DrawFaceNumberOverlay: per-mode strict-inequality vertex-count test.
  // Defends against the regressions enumerated in plan §3 / §7:
  //   - kWireframe / kXRay must draw hidden labels (more vertices than
  //     kHiddenLine / kShaded which skip them);
  //   - kHiddenLine / kShaded must still draw the *visible* label (size
  //     filter passes for a face large enough to clear the threshold).
  //
  // Arrange constructs two triangles in the same display-space layout that
  // ProjectLabelToScreen evaluates under identity rotation:
  //   - face 1 (face_number=1): front-facing (+z normal), AABB ratio well
  //     above kFaceLabelMinViewportRatio;
  //   - face 2 (face_number=2): back-facing (-z normal).
  // Pre-flight asserts hidden_count > 0 and visible_count > 0 + size_filter
  // passes for the visible face — failing either means the test would
  // degenerate into a tautology, so the test fails loudly instead of
  // silently passing.
  //
  // GuiFunc / TestFunc thread split mirrors test_gui_overlay_labels.cpp
  // (modal_does_not_leak_to_foreground): ImGui calls only on the GUI
  // thread; results captured in a file-static struct.
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
        -0.45f, -0.45f, 0.0f,  // v3 (reuse layout, +z offset to keep distinct centroid)
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
      lumice::gui::CrystalRenderer::ComputeMvp(rot, /*zoom=*/2.0f, 320, 320, mvp);

      // Arrange-phase pre-flight: aggregate, classify front/back, verify
      // size_filter pre-condition for the visible face.
      lumice::gui::FaceLabel labels[lumice::gui::kMaxFaceLabels] = {};
      int n = AggregateFaceLabels(verts, 6, tris, 2, face_numbers, labels, lumice::gui::kMaxFaceLabels);
      g_capture.hidden_count = 0;
      g_capture.visible_count = 0;
      g_capture.visible_passes_size_filter = false;
      for (int i = 0; i < n; ++i) {
        float sx = 0.0f;
        float sy = 0.0f;
        bool front = false;
        if (!ProjectLabelToScreen(&labels[i], rot, mvp, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front)) {
          continue;
        }
        if (front) {
          ++g_capture.visible_count;
          float w = 0.0f;
          float h = 0.0f;
          if (ComputeLabelScreenBboxRatio(&labels[i], mvp, ImVec2(320.0f, 320.0f), &w, &h)) {
            if (w >= kFaceLabelMinViewportRatio && h >= kFaceLabelMinViewportRatio) {
              g_capture.visible_passes_size_filter = true;
            }
          }
        } else {
          ++g_capture.hidden_count;
        }
      }
      g_capture.arrange_ok =
          (g_capture.hidden_count > 0) && (g_capture.visible_count > 0) && g_capture.visible_passes_size_filter;

      // Act: for each CrystalStyle, host a fresh window and measure DrawList
      // vertex growth across one DrawFaceNumberOverlay call.
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
          lumice::gui::DrawFaceNumberOverlay(verts, 6, tris, 2, face_numbers, rot, mvp, ImVec2(0.0f, 0.0f),
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
      // Pre-flight: arrange must satisfy the prerequisites that make the
      // strict-inequality assertions non-vacuous.
      IM_CHECK_GT(g_capture.hidden_count, 0);
      IM_CHECK_GT(g_capture.visible_count, 0);
      IM_CHECK(g_capture.visible_passes_size_filter);
      IM_CHECK(g_capture.arrange_ok);

      int wire = g_capture.vertex_delta[0];
      int hidden_line = g_capture.vertex_delta[1];
      int xray = g_capture.vertex_delta[2];
      int shaded = g_capture.vertex_delta[3];

      // Visible face renders under every mode (size_filter passes for the
      // arranged big face in HiddenLine/Shaded), so all deltas are > 0.
      IM_CHECK_GT(wire, 0);
      IM_CHECK_GT(hidden_line, 0);
      IM_CHECK_GT(xray, 0);
      IM_CHECK_GT(shaded, 0);

      // Strict inequalities — the differential test: WF/XR draw the hidden
      // face on top of the visible one, HL/SH skip it.
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
      IM_CHECK(ProjectLabelToScreen(&label, rot, mvp, /*image_pos*/ 100.0f, 200.0f,
                                    /*image_size*/ 320.0f, 320.0f, &sx, &sy, &front));
      // Origin → NDC (0, 0) → screen center of image rect.
      IM_CHECK(std::abs(sx - (100.0f + 160.0f)) < 1e-3f);
      IM_CHECK(std::abs(sy - (200.0f + 160.0f)) < 1e-3f);
    };
  }
}
