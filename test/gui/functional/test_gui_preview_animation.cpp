// Functional tests for the edit-modal crystal preview animation (task gui-preview-animation,
// scrum gui-shape-randomization T5).
//
// Coverage map (issue AC → test):
//   AC1  randomization ON animates / OFF is frame-static  → preview_animation_ticks_when_random_enabled
//                                                            preview_animation_static_when_no_random
//   AC3  thumbnail seed pinned deterministic               → preview_thumbnail_seed_pinned_deterministic
//   AC4  ticker resets across preview sessions (epoch-keyed) → preview_animation_epoch_resets_across_sessions
//   AC5  animation coexists with trackball drag            → preview_animation_coexists_with_trackball_drag
//
// All cases run in the fast --fixed-dt pool: ImGuiTestEngine overrides IO.DeltaTime to a constant
// 1/60s, so ceil(kCrystalPreviewAnimIntervalMs / (1000/60)) = 18 Yield frames deterministically
// crosses one animation tick — no real-timing isolation pool needed.

#include <cstring>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/crystal_preview.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// Frames to Yield to guarantee crossing at least one animation tick under --fixed-dt (18 frames =
// 300ms at 1/60s per frame; 30 leaves comfortable margin).
constexpr int kFramesPastOneTick = 30;

// Snapshot the vertex data of the most-recently built modal preview mesh. Copies out of the shared
// g_last_mesh singleton (GetLastCrystalMesh returns a pointer into it) so later rebuilds don't
// mutate the captured value.
std::vector<float> SnapshotPreviewVertices() {
  const auto* m = lumice::gui::GetLastCrystalMesh();
  IM_ASSERT(m != nullptr);
  return std::vector<float>(m->vertices, m->vertices + m->vertex_count * 3);
}

// Install a modest uniform randomization on face_distance[0] of entry 0's crystal so the preview
// mesh varies with sample_seed (spread 0.3 stays well clear of degenerate shapes).
void RandomizeEntry0FaceDistance() {
  auto& cr = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
  cr.face_distance[0] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.0f, 0.3f };
}

}  // namespace

void RegisterPreviewAnimationTests(ImGuiTestEngine* engine) {
  // AC1 positive: with an active shape distribution, the preview mesh re-samples over time.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_animation_ticks_when_random_enabled");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      lumice::gui::ResetLastCrystalMesh();
      RandomizeEntry0FaceDistance();
      IM_CHECK(lumice::gui::HasActiveShapeRandomization(
          gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id]));
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      const std::vector<float> before = SnapshotPreviewVertices();

      ctx->Yield(kFramesPastOneTick);
      const std::vector<float> after = SnapshotPreviewVertices();

      // A new sample_seed after the tick must produce a different draw.
      IM_CHECK(before != after);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // AC1 negative / static contract: no randomization ⇒ preview is byte-identical frame to frame,
  // across multiple tick intervals.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_animation_static_when_no_random");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      lumice::gui::ResetLastCrystalMesh();
      // Default crystal from DoNew() has all shape fields kNoRandom.
      IM_CHECK(!lumice::gui::HasActiveShapeRandomization(
          gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id]));
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      const std::vector<float> baseline = SnapshotPreviewVertices();

      // Sample across ~3 tick intervals; every read must equal the baseline.
      for (int i = 0; i < 3; ++i) {
        ctx->Yield(kFramesPastOneTick);
        IM_CHECK(SnapshotPreviewVertices() == baseline);
      }

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // AC3: the thumbnail / fixed-seed draw is deterministic — same (config, kPreviewFixedSampleSeed)
  // yields a bit-identical mesh. Pins the "thumbnail cache always passes the literal fixed seed"
  // contract so a future edit can't silently make thumbnails animate.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "preview_thumbnail_seed_pinned_deterministic");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      gui::CrystalConfig cr;
      cr.face_distance[0] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.0f, 0.3f };

      LUMICE_CrystalMesh mesh1{};
      LUMICE_CrystalMesh mesh2{};
      IM_CHECK(lumice::gui::BuildCrystalMeshData(cr, lumice::gui::kPreviewFixedSampleSeed, &mesh1));
      IM_CHECK(lumice::gui::BuildCrystalMeshData(cr, lumice::gui::kPreviewFixedSampleSeed, &mesh2));

      IM_CHECK_EQ(mesh1.vertex_count, mesh2.vertex_count);
      IM_CHECK_EQ(mesh1.edge_count, mesh2.edge_count);
      IM_CHECK_EQ(mesh1.triangle_count, mesh2.triangle_count);
      IM_CHECK(std::memcmp(mesh1.vertices, mesh2.vertices, sizeof(float) * mesh1.vertex_count * 3) == 0);
      IM_CHECK(std::memcmp(mesh1.edges, mesh2.edges, sizeof(int) * mesh1.edge_count * 2) == 0);
      IM_CHECK(std::memcmp(mesh1.triangles, mesh2.triangles, sizeof(int) * mesh1.triangle_count * 3) == 0);
    };
  }

  // AC4: the ticker is epoch-keyed — reopening a modal starts a fresh session whose first frame
  // builds with kPreviewFixedSampleSeed, regardless of how far the prior session had advanced.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_animation_epoch_resets_across_sessions");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      lumice::gui::ResetLastCrystalMesh();
      RandomizeEntry0FaceDistance();
      ctx->Yield(2);

      // First session: advance well past two ticks so the internal seed has moved off the fixed seed.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ctx->Yield(kFramesPastOneTick * 2);
      const std::vector<float> advanced = SnapshotPreviewVertices();

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);

      // Reopen the same entry: the first built frame must match the fixed-seed reference exactly,
      // proving the ticker reset rather than resuming the prior session's seed.
      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      const std::vector<float> reopened = SnapshotPreviewVertices();

      const auto& cr = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
      LUMICE_CrystalMesh ref{};
      IM_CHECK(lumice::gui::BuildCrystalMeshData(cr, lumice::gui::kPreviewFixedSampleSeed, &ref));
      const std::vector<float> ref_verts(ref.vertices, ref.vertices + ref.vertex_count * 3);

      IM_CHECK(reopened == ref_verts);
      // Sanity: the reset genuinely rewound — the reopened frame differs from the advanced one.
      IM_CHECK(reopened != advanced);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }

  // AC5: the animation ticker and the trackball state machine are independent — animation ticks
  // never perturb g_crystal_rotation, and a drag survives subsequent ticks. (Proves logical
  // non-interference; the --fixed-dt pool cannot observe real-frame jitter — that is left to the
  // hand-verification step.)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "p1_edit_modal", "preview_animation_coexists_with_trackball_drag");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      lumice::gui::ResetLastCrystalMesh();
      RandomizeEntry0FaceDistance();
      ctx->Yield(2);

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);

      float rot_before[16];
      std::memcpy(rot_before, gui::g_crystal_rotation, sizeof(rot_before));

      // Let the animation tick; the rotation must be untouched.
      ctx->Yield(kFramesPastOneTick);
      IM_CHECK(std::memcmp(gui::g_crystal_rotation, rot_before, sizeof(rot_before)) == 0);

      // Drag the trackball; the rotation must change.
      gui::ApplyTrackballRotation(60.0f, 0.0f);
      float rot_dragged[16];
      std::memcpy(rot_dragged, gui::g_crystal_rotation, sizeof(rot_dragged));
      IM_CHECK(std::memcmp(rot_dragged, rot_before, sizeof(rot_before)) != 0);

      // Continue animating; the dragged pose must survive further ticks unchanged.
      ctx->Yield(kFramesPastOneTick);
      IM_CHECK(std::memcmp(gui::g_crystal_rotation, rot_dragged, sizeof(rot_dragged)) == 0);

      ctx->ItemClick("**/" ICON_FA_XMARK " Cancel##edit_modal");
      ctx->Yield(2);
    };
  }
}
