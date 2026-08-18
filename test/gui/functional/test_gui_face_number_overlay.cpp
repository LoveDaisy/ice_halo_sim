// The face_number_overlay cases that need ImGui or a live mesh: the two "screenshot" ones (font
// atlas / real draw) and the topology-parity one (walks BuildCrystalMeshData).
//
// The twelve pure-math "unit" ones moved to test/unit-correctness/gui/test_face_number_overlay.cpp.

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

}  // namespace

void RegisterFaceNumberOverlayTests(ImGuiTestEngine* engine) {
  using lumice::gui::AggregateFaceLabels;
  using lumice::gui::AggregateFaceLabelsFromTopology;
  using lumice::gui::CrystalStyle;
  using lumice::gui::FaceLabel;
  using lumice::gui::kFaceLabelMinViewportRatio;
  using lumice::gui::kMaxFaceLabels;
  using lumice::gui::ProjectLabelToScreen;
  using lumice::gui::detail::ComputeLabelMinWidthRatio;

  // End-to-end smoke: put the inspector's crystal page on an entry, verify GetLastCrystalMesh
  // populates with prism config + at least one labelable face (>0).
  //
  // What drives the mesh is the preview pane, and what decides the preview pane is on screen is the
  // tree's selection — the card's "Edit" button that used to raise a modal here is gone. There is
  // no dismissal at the end for the same reason: the page is simply always up.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "screenshot", "face_number_overlay_smoke");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      lumice::gui::ResetLastCrystalMesh();
      ctx->Yield(2);
      OpenCrystalTab(ctx);
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
      // Build a minimal LUMICE_CrystalMesh (face_count=0 → fallback path)
      LUMICE_CrystalMesh mesh{};
      mesh.vertex_count = 6;
      std::memcpy(mesh.vertices, verts, sizeof(verts));
      mesh.triangle_count = 2;
      std::memcpy(mesh.triangles, tris, sizeof(tris));
      std::memcpy(mesh.face_numbers, face_numbers, sizeof(face_numbers));
      mesh.face_count = 0;

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
          lumice::gui::DrawFaceNumberOverlay(&mesh, rot, mvp, kZoom, ImVec2(0.0f, 0.0f), ImVec2(320.0f, 320.0f),
                                             draw_list, styles[s]);
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

  // AggregateFaceLabelsFromTopology vs AggregateFaceLabels: same face_number set,
  // same vertex count per face. Uses a real prism mesh from LUMICE_GetCrystalMesh.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "face_number_aggregate_from_topology_parity");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      lumice::gui::ResetLastCrystalMesh();

      LUMICE_CrystalMesh mesh{};
      LUMICE_CrystalParam prism_param{};
      prism_param.type = 0;  // prism
      prism_param.height = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
      for (auto& fd : prism_param.face_distance) {
        fd = LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, 1.0f, 0.0f };
      }
      IM_CHECK_EQ(LUMICE_GetCrystalMesh(&prism_param, lumice::gui::kPreviewFixedSampleSeed, &mesh), LUMICE_OK);
      IM_CHECK_GT(mesh.face_count, 0);

      FaceLabel labels_old[lumice::gui::kMaxFaceLabels] = {};
      FaceLabel labels_new[lumice::gui::kMaxFaceLabels] = {};

      int n_old = AggregateFaceLabels(mesh.vertices, mesh.vertex_count, mesh.triangles, mesh.triangle_count,
                                      mesh.face_numbers, labels_old, lumice::gui::kMaxFaceLabels);
      int n_new = lumice::gui::AggregateFaceLabelsFromTopology(
          mesh.vertices, mesh.vertex_count, mesh.face_count, mesh.face_numbers_by_face, mesh.face_vtx_offsets,
          mesh.face_vtx_counts, mesh.face_vtx_pool, mesh.face_normals, labels_new, lumice::gui::kMaxFaceLabels);

      IM_CHECK_EQ(n_old, n_new);
      IM_CHECK_EQ(n_old, mesh.face_count);

      // Both must produce the same set of face_numbers. Reported non-fatally: this compares data
      // already fully computed above (no ctx-> driving in this loop at all, see IM_UNUSED(ctx)), so
      // a fatal assert here would only hide every other mismatched face behind the first one found
      // (see scripts/check_loop_fatal_asserts.py).
      for (int i = 0; i < n_new; ++i) {
        bool found = false;
        for (int j = 0; j < n_old; ++j) {
          if (labels_new[i].face_number == labels_old[j].face_number) {
            found = true;
            // vertex count per face should match
            if (labels_new[i].display_polygon_vertex_count != labels_old[j].display_polygon_vertex_count) {
              IM_ERRORF("face_number %d: vertex count mismatch new=%d old=%d", labels_new[i].face_number,
                        labels_new[i].display_polygon_vertex_count, labels_old[j].display_polygon_vertex_count);
            }
            break;
          }
        }
        if (!found) {
          IM_ERRORF(
              "face_number %d (from AggregateFaceLabelsFromTopology) not found in "
              "AggregateFaceLabels' set",
              labels_new[i].face_number);
        }
      }
    };
  }
}
