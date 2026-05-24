#include <array>
#include <cmath>

#include "gui/gui_constants.hpp"
#include "gui/preview_renderer.hpp"
#include "test_gui_shared.hpp"

namespace {

constexpr float kSentinel = -9000.f;  // any helper output ≤ this is treated as the sentinel
constexpr float kEps = 1.0f;          // ± 1 px tolerance vs. analytic expectation

bool IsSentinel(const std::array<float, 2>& p) {
  return p[0] <= kSentinel && p[1] <= kSentinel;
}

lumice::gui::ViewProjection MakeVp(int lens_type, float fov, float elev, float az, float roll) {
  lumice::gui::ViewProjection vp;
  vp.lens_type = lens_type;
  vp.fov = fov;
  vp.elevation = elev;
  vp.azimuth = az;
  vp.roll = roll;
  vp.visible = lumice::gui::kVisibleFull;
  return vp;
}

}  // namespace

// Pure-CPU unit tests for ProjectWorldDirToScreen. Validate analytic expectations
// against the shader's inverse projection formulas — see plan.md Step 3 / F14.
void RegisterProjectWorldDirTests(ImGuiTestEngine* engine) {
  // Case 1: Linear pinhole. Front direction (-1,0,0) at elevation=0, azimuth=0
  // lands at viewport center; zenith (0,0,-1) is at exactly view_dir.z=0 — perspective
  // singularity — and must return sentinel.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "linear");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto vp = MakeVp(lumice::gui::kLensTypeLinear, 90.0f, 0.0f, 0.0f, 0.0f);
      const float front[3] = { -1.0f, 0.0f, 0.0f };
      auto p = lumice::gui::ProjectWorldDirToScreen(vp, front, 800, 600);
      IM_CHECK(!IsSentinel(p));
      IM_CHECK(std::abs(p[0]) <= kEps);
      IM_CHECK(std::abs(p[1]) <= kEps);

      const float zenith[3] = { 0.0f, 0.0f, -1.0f };
      auto pz = lumice::gui::ProjectWorldDirToScreen(vp, zenith, 800, 600);
      IM_CHECK(IsSentinel(pz));  // view_dir.z = 0 ⇒ behind-or-on-camera-plane sentinel
    };
  }

  // Case 2: Fisheye equal area, camera tilted to look at zenith (elevation=90°).
  // img_radius = min(800,800)/2 = 400.
  // Zenith (0,0,-1) ⇒ view_dir = (0,0,-1) ⇒ theta=0 ⇒ (0,0).
  // Front world (-1,0,0) maps to view_dir = (0,-1,0) (col0=(0,-1,0), col1=(1,0,0)
  // when elevation=90°,az=0,roll=0; transpose·world_dir gives view_dir). Then
  // theta=π/2, r_norm = sin(π/4)/sin(π/4) = 1, r = 400, phi = atan2(-1, 0) = -π/2,
  // ⇒ pixel (0, -400) (image bottom-center).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "fisheye_ea");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto vp = MakeVp(lumice::gui::kLensTypeFisheyeEqualArea, 180.0f, 90.0f, 0.0f, 0.0f);
      const float zenith[3] = { 0.0f, 0.0f, -1.0f };
      auto pz = lumice::gui::ProjectWorldDirToScreen(vp, zenith, 800, 800);
      IM_CHECK(!IsSentinel(pz));
      IM_CHECK(std::abs(pz[0]) <= kEps);
      IM_CHECK(std::abs(pz[1]) <= kEps);

      const float front[3] = { -1.0f, 0.0f, 0.0f };
      auto pf = lumice::gui::ProjectWorldDirToScreen(vp, front, 800, 800);
      IM_CHECK(!IsSentinel(pf));
      IM_CHECK(std::abs(pf[0]) <= kEps);
      IM_CHECK(std::abs(pf[1] - (-400.0f)) <= kEps);
    };
  }

  // Case 3: Rectangular (equirectangular). vp = 800×400 ⇒
  // short_res_dual = min(400, 400) = 400, scale = 400/π.
  // Zenith (0,0,-1) ⇒ lat = π/2, py = -(π/2) * 400/π = -200.
  // Nadir (0,0,+1)  ⇒ py = +200.
  // Equator front (-1,0,0) ⇒ lat=0, lon=0 ⇒ (0,0).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "rectangular");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto vp = MakeVp(lumice::gui::kLensTypeRectangular, 180.0f, 0.0f, 0.0f, 0.0f);
      const float zenith[3] = { 0.0f, 0.0f, -1.0f };
      auto pz = lumice::gui::ProjectWorldDirToScreen(vp, zenith, 800, 400);
      IM_CHECK(!IsSentinel(pz));
      IM_CHECK(std::abs(pz[0]) <= kEps);
      IM_CHECK(std::abs(pz[1] - (-200.0f)) <= kEps);

      const float nadir[3] = { 0.0f, 0.0f, 1.0f };
      auto pn = lumice::gui::ProjectWorldDirToScreen(vp, nadir, 800, 400);
      IM_CHECK(!IsSentinel(pn));
      IM_CHECK(std::abs(pn[0]) <= kEps);
      IM_CHECK(std::abs(pn[1] - 200.0f) <= kEps);

      const float front[3] = { -1.0f, 0.0f, 0.0f };
      auto pf = lumice::gui::ProjectWorldDirToScreen(vp, front, 800, 400);
      IM_CHECK(!IsSentinel(pf));
      IM_CHECK(std::abs(pf[0]) <= kEps);
      IM_CHECK(std::abs(pf[1]) <= kEps);
    };
  }

  // Case 4: Dual fisheye equal area. vp = 800×400 ⇒
  // short_res_dual = min(400, 400) = 400, circle_radius = 200.
  // Zenith (0,0,-1) lands at left-circle center  (-200, 0).
  // Nadir  (0,0,+1) lands at right-circle center (+200, 0).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "dual_fisheye_ea");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto vp = MakeVp(lumice::gui::kLensTypeDualFisheyeEqualArea, 180.0f, 0.0f, 0.0f, 0.0f);
      const float zenith[3] = { 0.0f, 0.0f, -1.0f };
      auto pz = lumice::gui::ProjectWorldDirToScreen(vp, zenith, 800, 400);
      IM_CHECK(!IsSentinel(pz));
      IM_CHECK(std::abs(pz[0] - (-200.0f)) <= kEps);
      IM_CHECK(std::abs(pz[1]) <= kEps);

      const float nadir[3] = { 0.0f, 0.0f, 1.0f };
      auto pn = lumice::gui::ProjectWorldDirToScreen(vp, nadir, 800, 400);
      IM_CHECK(!IsSentinel(pn));
      IM_CHECK(std::abs(pn[0] - 200.0f) <= kEps);
      IM_CHECK(std::abs(pn[1]) <= kEps);
    };
  }

  // Case 5: Globe lens is out-of-scope — must return sentinel.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "globe_sentinel");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto vp = MakeVp(lumice::gui::kLensTypeGlobe, 30.0f, 0.0f, 0.0f, 0.0f);
      const float zenith[3] = { 0.0f, 0.0f, -1.0f };
      auto p = lumice::gui::ProjectWorldDirToScreen(vp, zenith, 800, 800);
      IM_CHECK(IsSentinel(p));
    };
  }
}
