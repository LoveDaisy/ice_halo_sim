// Unit tests for CrystalRenderer::ComputeMvp's geometric contracts (camera
// view-rotation split, world-coord interaction). Pure math — no GL context
// required, registered under the "unit" group like test_gui_face_number_overlay.
//
// These tests guard:
//   - V_rot = Rx(-kCameraTiltDeg) sign convention (camera elevated above origin
//     looks DOWN at the crystal — world +z visible above screen center)
//   - Y-Z swap equivalence: under the GUI mesh swap, mesh +y represents world
//     +z, so the kColumn chain rotation (which maps c-axis to world +x in core
//     conventions) — when applied to a c-axis-aligned mesh vertex — produces a
//     screen-y near zero (c-axis horizontal).
//
// Cross-module guard: if BuildCrystalMeshData ever changes the swap direction,
// the kColumn sub-test fails immediately because the mesh-frame chain output
// no longer aligns with the V_rot convention.

#include <array>
#include <cmath>
#include <cstring>

#include "gui/axis_presets.hpp"
#include "gui/crystal_renderer.hpp"
#include "test_gui_shared.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Identity 4x4 column-major.
void Identity4x4(float m[16]) {
  std::memset(m, 0, 16 * sizeof(float));
  m[0] = 1.0f;
  m[5] = 1.0f;
  m[10] = 1.0f;
  m[15] = 1.0f;
}

// Multiply column-major 4x4 MVP by a homogeneous (x, y, z, 1) point. Returns
// {clip_x, clip_y, clip_z, clip_w}; perspective divide is the caller's job.
std::array<float, 4> Mvp4(const float mvp[16], float x, float y, float z) {
  return {
    mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12],
    mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13],
    mvp[2] * x + mvp[6] * y + mvp[10] * z + mvp[14],
    mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15],
  };
}

}  // namespace

void RegisterCrystalRendererTests(ImGuiTestEngine* engine) {
  // unit/crystal_renderer_camera_tilt_world_z_above_center — With identity
  // model rotation and zoom=1, a vertex at world +z (= mesh +y under the GUI
  // Y-Z swap) must project ABOVE the screen center (clip-space y > 0). This
  // pins the V_rot = Rx(-kCameraTiltDeg) sign: a NEGATIVE-angle Rx tilts the
  // camera DOWN so that points along world +z (which the swap aligns with mesh
  // +y) appear on the upper half of the screen.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "crystal_renderer_camera_tilt_world_z_above_center");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      float model[16];
      Identity4x4(model);
      float mvp[16];
      lumice::gui::CrystalRenderer::ComputeMvp(model, /*zoom=*/1.0f, /*width=*/512, /*height=*/512, mvp);

      // World +z is mesh +y (post Y-Z swap). With camera elevated by
      // +kCameraTiltDeg looking down, this vertex must land above center.
      auto top = Mvp4(mvp, 0.0f, 1.0f, 0.0f);
      IM_CHECK(top[3] > 0.0f);
      float ndc_y_top = top[1] / top[3];
      IM_CHECK(ndc_y_top > 0.0f);

      // Symmetric sanity: world -z (mesh -y) must land below center.
      auto bot = Mvp4(mvp, 0.0f, -1.0f, 0.0f);
      IM_CHECK(bot[3] > 0.0f);
      float ndc_y_bot = bot[1] / bot[3];
      IM_CHECK(ndc_y_bot < 0.0f);
    };
  }

  // unit/crystal_renderer_kcolumn_caxis_horizontal — With the kColumn default
  // (chain + swap-wrapped to mesh frame) as the model matrix and a vertex at
  // the c-axis (mesh +y, which represents world +z under the swap), the rotation
  // maps mesh +y to mesh +x (horizontal). With V_rot only adding a small camera
  // elevation, the resulting screen y stays close to zero — the c-axis appears
  // horizontal. This guards the joint correctness of (a) ChainRotationToMatrix,
  // (b) the swap wrap inside DefaultPreviewRotation, and (c) the V_rot sign.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "unit", "crystal_renderer_kcolumn_caxis_horizontal");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      // Use DefaultPreviewRotation — the same path used by the runtime, which
      // applies the mesh swap wrap so the rotation is geometrically valid for
      // mesh-space vertices.
      float model[16];
      lumice::gui::DefaultPreviewRotation(lumice::gui::AxisPreset::kColumn, nullptr, model);

      float mvp[16];
      lumice::gui::CrystalRenderer::ComputeMvp(model, /*zoom=*/1.0f, /*width=*/512, /*height=*/512, mvp);

      // Apply MVP to mesh +y (the c-axis). After the wrapped chain the c-axis
      // lies along mesh +x (horizontal); kCameraTiltDeg adds a tiny screen-y
      // offset that stays well within tolerance.
      auto p = Mvp4(mvp, 0.0f, 1.0f, 0.0f);
      IM_CHECK(p[3] > 0.0f);
      float ndc_y = p[1] / p[3];
      IM_CHECK(std::fabs(ndc_y) < 0.1f);
    };
  }
}
