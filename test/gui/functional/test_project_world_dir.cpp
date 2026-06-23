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

  // Case 5: Globe lens projection. Camera at O=(0,0,D), D=kGlobeCameraD=4.
  // World convention: z-down, so zenith=(0,0,-1) and nadir=(0,0,+1) — matches
  // the other cases (Case 2, Case 3, Case 4) and overlay_labels.cpp.
  // elevation=-90° tilts the camera to look upward; with az=roll=0 the
  // WorldToView column for zenith yields eye_dir=(0,0,+1) (in front of camera,
  // dz>1/D). FOV=30° ⇒ half_fov=15°, img_radius=400, focal=400/tan(15°)≈1492.8.
  // denom = D - 1 = 3, px = 0/3 * focal = 0, py = 0 ⇒ viewport center.
  // Nadir gives eye_dir=(0,0,-1) ⇒ dz=-1 ≤ 1/D ⇒ sentinel (behind sphere).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "globe_projection");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      auto vp = MakeVp(lumice::gui::kLensTypeGlobe, 30.0f, -90.0f, 0.0f, 0.0f);
      const float zenith[3] = { 0.0f, 0.0f, -1.0f };
      auto pz = lumice::gui::ProjectWorldDirToScreen(vp, zenith, 800, 800);
      IM_CHECK(!IsSentinel(pz));
      IM_CHECK(std::abs(pz[0]) <= kEps);
      IM_CHECK(std::abs(pz[1]) <= kEps);

      const float nadir[3] = { 0.0f, 0.0f, 1.0f };
      auto pn = lumice::gui::ProjectWorldDirToScreen(vp, nadir, 800, 800);
      IM_CHECK(IsSentinel(pn));  // back of sphere — front-hemisphere visibility filter
    };
  }

  // Case 6: Fisheye circular viewport clip. FOV<180° in a non-square viewport
  // creates black-bar regions where r_norm>1 falls inside the rectangular
  // viewport bounds — directions there must return sentinel.
  // Equidistant fisheye, FOV=90° ⇒ half_fov=π/4, vp=1920×1080,
  // img_radius=min(1920,1080)/2=540.
  // A direction 60° off the optical axis (theta=π/3) maps to
  // r_norm = (π/3)/(π/4) = 4/3 ≈ 1.333 > 1 ⇒ sentinel.
  // Same direction at FOV=180° (half_fov=π/2) maps to r_norm = (π/3)/(π/2) = 2/3,
  // which falls inside the imaging circle ⇒ not sentinel.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "project_world_dir", "fisheye_circular_clip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      // At elevation=az=roll=0, WorldToView maps world_dir w to
      //   view_dir = (-w.y, -w.z, w.x).
      // We want view_dir = (sin θ, 0, -cos θ) with θ = 60°, i.e. (√3/2, 0, -1/2)
      // — 60° off the -z optical axis. Solving: w.y = -√3/2, w.z = 0, w.x = -1/2.
      // This is a unit horizontal direction 60° off the camera-front axis.
      const float dir_60deg[3] = { -0.5f, -0.86602540378f, 0.0f };

      auto vp_90 = MakeVp(lumice::gui::kLensTypeFisheyeEquidist, 90.0f, 0.0f, 0.0f, 0.0f);
      auto p_90 = lumice::gui::ProjectWorldDirToScreen(vp_90, dir_60deg, 1920, 1080);
      IM_CHECK(IsSentinel(p_90));  // outside imaging circle (r_norm ≈ 1.33)

      auto vp_180 = MakeVp(lumice::gui::kLensTypeFisheyeEquidist, 180.0f, 0.0f, 0.0f, 0.0f);
      auto p_180 = lumice::gui::ProjectWorldDirToScreen(vp_180, dir_60deg, 1920, 1080);
      IM_CHECK(!IsSentinel(p_180));  // inside imaging circle (r_norm ≈ 0.67)
    };
  }
}
