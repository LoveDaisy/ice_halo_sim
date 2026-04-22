// Unit tests for face_number_overlay pure functions:
// AggregateFaceLabels and ProjectLabelToScreen.
// Registered under the "unit" test group; no ImGui context required.

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
  using lumice::gui::FaceLabel;
  using lumice::gui::kMaxFaceLabels;
  using lumice::gui::ProjectLabelToScreen;

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
      IM_CHECK(ProjectLabelToScreen(label, rot, mvp, 10.0f, 20.0f, 320.0f, 320.0f, &sx, &sy, &front));
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
      IM_CHECK(ProjectLabelToScreen(label, rot, mvp, 0.0f, 0.0f, 320.0f, 320.0f, &sx, &sy, &front));
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
      IM_CHECK(ProjectLabelToScreen(label, rot, mvp, /*image_pos*/ 100.0f, 200.0f,
                                    /*image_size*/ 320.0f, 320.0f, &sx, &sy, &front));
      // Origin → NDC (0, 0) → screen center of image rect.
      IM_CHECK(std::abs(sx - (100.0f + 160.0f)) < 1e-3f);
      IM_CHECK(std::abs(sy - (200.0f + 160.0f)) < 1e-3f);
    };
  }
}
