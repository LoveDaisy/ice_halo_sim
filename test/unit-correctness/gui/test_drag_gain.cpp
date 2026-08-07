// Pure-CPU unit tests for ComputeDragGainDegPerPixel — the preview drag sensitivity law.
// No GL context and no ImGui, which is why they live in gui_unit_test rather than gui_test.
//
// The contract under test is not "the closed form matches the closed form" (that would be a
// tautology): it is the end-to-end statement the drag interaction actually owes the user —
// feed the gain through the SAME azimuth/elevation update rule app_panels.cpp uses, then let
// the UNMODIFIED production projection (ProjectWorldDirToScreen / BuildViewMatrix) say where
// the reference sky point landed. If the hand-derived derivative in preview_renderer.cpp is
// off by a factor, a sign, or disagrees with the projection's img_radius / theta conventions,
// the round trip misses and no amount of "the formula looks right" hides it.
//
// A note on step size, which is the honest boundary of this design: the gain is a DERIVATIVE
// (exact at the frame center only, to first order). At the top of a lens's FOV range the
// angular resolution at the center is enormous — linear at fov=179° is ~21.9 deg per pixel —
// so a "20 px drag" there is a 438° rotation and no linearization survives it. The probe tests
// therefore normalize the step to a fixed small ANGLE and assert the ratio is 1; the
// realistic-drag tests use whole-pixel deltas only where the second-order term is genuinely
// negligible. Widening either one's tolerance to make an extreme-FOV whole-pixel case pass
// would be measuring the linearization error, not the gain.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "gui/gui_constants.hpp"
#include "gui/preview_renderer.hpp"

namespace {

using lumice::gui::ComputeDragGainDegPerPixel;
using lumice::gui::ProjectWorldDirToScreen;
using lumice::gui::ViewProjection;

constexpr float kSentinelLimit = -9000.f;  // any helper output <= this is the sentinel
constexpr float kPi = 3.14159265358979323846f;

bool IsSentinel(const std::array<float, 2>& p) {
  return p[0] <= kSentinelLimit && p[1] <= kSentinelLimit;
}

ViewProjection MakeVp(int lens_type, float fov, float elev, float az) {
  ViewProjection vp;
  vp.lens_type = lens_type;
  vp.fov = fov;
  vp.elevation = elev;
  vp.azimuth = az;
  vp.roll = 0.0f;
  vp.visible = lumice::gui::kVisibleFull;
  return vp;
}

// The world point that sits exactly at the frame center when azimuth = elevation = roll = 0.
//
// BuildViewMatrix documents the camera as looking at world (-1,0,0) in that pose, and that is
// the reference used by the five inside-out lenses. Globe is the exception: ProjectGlobe
// reinterprets the WorldToView output as a POSITION in eye space rather than a view direction,
// so (-1,0,0) becomes eye_dir.z = -1, which fails its eye_dir.z > 1/kGlobeCameraD visibility
// test and returns the sentinel; (+1,0,0) is the one that lands at the center. GlobeReference
// (below) is a standing check on that asymmetry, so the choice here is measured, not assumed.
const float* ReferenceWorldDir(int lens_type) {
  static const float kFront[3] = { -1.0f, 0.0f, 0.0f };
  static const float kBack[3] = { 1.0f, 0.0f, 0.0f };
  return lens_type == lumice::gui::kLensTypeGlobe ? kBack : kFront;
}

// Verbatim mirror of the update rule in app_panels.cpp's drag branch — including the globe
// sign flip, which exists so a right/down drag moves the sphere right/down like the inside-out
// lenses. Copied rather than re-derived: a test that invents its own sign convention would
// pass while the app drags backwards.
//
//   if (rc.lens_type == kLensTypeGlobe) { rc.azimuth += delta.x * g; rc.elevation -= delta.y * g; }
//   else                                { rc.azimuth -= delta.x * g; rc.elevation += delta.y * g; }
//
// The azimuth wrap and the elevation clamp are carried over too, so the mirror is the whole
// branch rather than the convenient half of it. Neither fires for the step sizes used here —
// every case stays well inside ±180° / ±89° — but leaving them out would make the helper a
// selective copy, and the next person to grow the step sizes would not find out.
ViewProjection ApplyDrag(int lens_type, float fov, float gain_deg_per_px, float delta_x, float delta_y) {
  float az = 0.0f;
  float el = 0.0f;
  if (lens_type == lumice::gui::kLensTypeGlobe) {
    az += delta_x * gain_deg_per_px;
    el -= delta_y * gain_deg_per_px;
  } else {
    az -= delta_x * gain_deg_per_px;
    el += delta_y * gain_deg_per_px;
  }
  if (az > 180.0f) {
    az -= 360.0f;
  } else if (az < -180.0f) {
    az += 360.0f;
  }
  float el_lim = (lens_type == lumice::gui::kLensTypeGlobe) ? 89.0f : 90.0f;
  el = std::max(-el_lim, std::min(el_lim, el));
  return MakeVp(lens_type, fov, el, az);
}

struct LensCase {
  int lens_type;
  const char* name;
  float max_fov;  // LUMICE_MaxFov for this lens, per src/config/render_config.cpp MaxFov()
  float min_fov;  // lowest FOV at which ProjectWorldDirToScreen is usable as an oracle — see below
};

// The six lens types that currently enable dragging, i.e. exactly !LensIsFullSky.
// AC2's "all lens types with drag enabled" is this list.
//
// min_fov is a property of the ORACLE, not of the gain. ProjectFisheye recovers the view angle
// with acos(-view_dir.z), and 1 - cos(theta) ~ theta^2/2 falls under the float ulp at 1.0
// (5.96e-8) once theta drops below ~3.5e-4 rad — cos returns exactly 1.0f and the angle is
// gone. Measured: at fisheye fov=1° a 10 px probe is theta = 2.18e-4 rad and the oracle reports
// theta = 0, i.e. 100% of the angle lost. ProjectLinear and ProjectGlobe divide instead of
// taking an acos, so they hold their precision all the way down and keep min_fov = 1.
// The fisheye band below 8° is instead covered by SmallFovLawsConvergeToEquidistant, which
// checks the four fisheye laws against an independent analytic limit rather than a projection.
const std::vector<LensCase>& DraggableLenses() {
  static const std::vector<LensCase> kCases = {
    { lumice::gui::kLensTypeLinear, "linear", 179.0f, 1.0f },
    { lumice::gui::kLensTypeFisheyeEqualArea, "fisheye_equal_area", 360.0f, 8.0f },
    { lumice::gui::kLensTypeFisheyeEquidist, "fisheye_equidistant", 360.0f, 8.0f },
    { lumice::gui::kLensTypeFisheyeStereographic, "fisheye_stereographic", 359.0f, 8.0f },
    { lumice::gui::kLensTypeFisheyeOrthographic, "fisheye_orthographic", 180.0f, 8.0f },
    { lumice::gui::kLensTypeGlobe, "globe", 90.0f, 1.0f },
  };
  return kCases;
}

struct Viewport {
  int w;
  int h;
  const char* name;
};

// Square, landscape, portrait, and a 2x-DPI landscape (AC3: the gain must be expressed in
// framebuffer pixels, so the 2x viewport is the same physical feel at twice the pixel delta).
const std::vector<Viewport>& Viewports() {
  static const std::vector<Viewport> kVps = {
    { 800, 800, "800x800" },
    { 1280, 720, "1280x720" },
    { 600, 900, "600x900" },
    { 2560, 1440, "2560x1440_hidpi" },
  };
  return kVps;
}

// The probe is sized by ANGLE, not by pixels, because both error sources that bound it are
// angular: the linearization error above (worst case tan(theta)/theta - 1 = theta^2/3, so
// 1.3e-4 at 0.02 rad) and the oracle's float floor below (see DraggableLenses). 0.02 rad sits
// between them with roughly an order of magnitude of headroom on each side.
//
// The pixel cap is the one place the frame gets a say: at narrow FOV, 0.02 rad is a large
// fraction of the whole field, so the probe is also held to 40% of img_radius, keeping the
// projected point comfortably inside the viewport (and inside the fisheye imaging circle)
// rather than clipping to the sentinel.
constexpr float kProbeAngleRad = 0.02f;
constexpr float kProbeFrameFraction = 0.4f;

// 2e-3 is ~15x the worst linearization term in play and ~25x the worst oracle float noise,
// while any real defect in a gain branch (a missing factor of 2, a sin/tan swap, a wrong
// img_radius) is a several-percent-to-several-hundred-percent effect. It separates the two.
constexpr float kRoundTripTol = 2e-3f;

float ProbePixels(float gain_deg_per_px, int vp_w, int vp_h) {
  float gain_rad_per_px = gain_deg_per_px * kPi / 180.0f;
  float img_radius = std::min(static_cast<float>(vp_w), static_cast<float>(vp_h)) * 0.5f;
  return std::min(kProbeFrameFraction * img_radius, kProbeAngleRad / gain_rad_per_px);
}

std::string Label(const LensCase& lc, float fov, const Viewport& vp, float dx, float dy) {
  return std::string(lc.name) + " fov=" + std::to_string(fov) + " vp=" + vp.name + " delta=(" + std::to_string(dx) +
         "," + std::to_string(dy) + ")";
}

// One round trip: drag by (dx, dy) screen pixels, reproject the reference sky point, and
// report where it landed. Screen coords are the projection's own y-up pixel frame, so a
// mouse delta of +dy (ImGui y is DOWN-positive) must move the content to py = -dy.
std::array<float, 2> RoundTrip(const LensCase& lc, float fov, const Viewport& vp, float dx, float dy) {
  float gain = ComputeDragGainDegPerPixel(lc.lens_type, fov, vp.w, vp.h);
  ViewProjection dragged = ApplyDrag(lc.lens_type, fov, gain, dx, dy);
  return ProjectWorldDirToScreen(dragged, ReferenceWorldDir(lc.lens_type), vp.w, vp.h);
}

// FOV ladder for a lens: its oracle-usable floor, a narrow value, mid-range, and two rungs up
// against its own MaxFov. The near-max rungs are the ones that matter — that is where a
// uniform fov/height approximation would have been wrong by ~70x for linear and stereographic,
// which is the whole reason this task took the per-lens closed form.
std::vector<float> FovLadder(const LensCase& lc) {
  std::vector<float> ladder = { lc.min_fov, 15.0f, lc.max_fov * 0.5f, lc.max_fov * 0.9f, lc.max_fov - 0.5f };
  ladder.erase(
      std::remove_if(ladder.begin(), ladder.end(), [&lc](float f) { return f < lc.min_fov || f > lc.max_fov; }),
      ladder.end());
  return ladder;
}

}  // namespace

// ---------------------------------------------------------------------------------------
// Precondition for every round trip below: the reference world point really is the frame
// center at the identity pose, and globe really is the one that needs (+1,0,0). If this
// flips, the round-trip assertions are measuring the wrong thing.
// ---------------------------------------------------------------------------------------
TEST(DragGain, ReferencePointIsFrameCenter) {
  for (const auto& lc : DraggableLenses()) {
    auto vp = MakeVp(lc.lens_type, 60.0f, 0.0f, 0.0f);
    auto p = ProjectWorldDirToScreen(vp, ReferenceWorldDir(lc.lens_type), 800, 600);
    ASSERT_FALSE(IsSentinel(p)) << lc.name;
    EXPECT_NEAR(p[0], 0.0f, 1e-3f) << lc.name;
    EXPECT_NEAR(p[1], 0.0f, 1e-3f) << lc.name;
  }
}

TEST(DragGain, GlobeReferenceIsMirroredVersusInsideOutLenses) {
  const float front[3] = { -1.0f, 0.0f, 0.0f };
  const float back[3] = { 1.0f, 0.0f, 0.0f };

  // Globe: the inside-out lenses' front direction is NOT visible (eye_dir.z = -1).
  auto globe = MakeVp(lumice::gui::kLensTypeGlobe, 60.0f, 0.0f, 0.0f);
  EXPECT_TRUE(IsSentinel(ProjectWorldDirToScreen(globe, front, 800, 600)));
  EXPECT_FALSE(IsSentinel(ProjectWorldDirToScreen(globe, back, 800, 600)));

  // Linear: the mirror image of the above.
  auto linear = MakeVp(lumice::gui::kLensTypeLinear, 60.0f, 0.0f, 0.0f);
  EXPECT_FALSE(IsSentinel(ProjectWorldDirToScreen(linear, front, 800, 600)));
  EXPECT_TRUE(IsSentinel(ProjectWorldDirToScreen(linear, back, 800, 600)));
}

// ---------------------------------------------------------------------------------------
// AC1 / AC2 / AC3 — the main event. For every draggable lens, across its own full FOV range,
// on square / landscape / portrait / HiDPI viewports, in both directions on both axes: one
// pixel of drag moves the content one pixel. Constant across the FOV range is the whole
// point — the same assertion with the same tolerance holds at fov=1° and at fov=MaxFov.
// ---------------------------------------------------------------------------------------
TEST(DragGain, HorizontalDragMovesContentOnePixelPerPixel) {
  for (const auto& lc : DraggableLenses()) {
    for (float fov : FovLadder(lc)) {
      for (const auto& vp : Viewports()) {
        float gain = ComputeDragGainDegPerPixel(lc.lens_type, fov, vp.w, vp.h);
        ASSERT_GT(gain, 0.0f) << Label(lc, fov, vp, 0, 0);
        float step = ProbePixels(gain, vp.w, vp.h);
        for (float dx : { step, -step }) {
          auto p = RoundTrip(lc, fov, vp, dx, 0.0f);
          ASSERT_FALSE(IsSentinel(p)) << Label(lc, fov, vp, dx, 0.0f);
          // Content follows the cursor: +dx of mouse motion => +dx of screen-x motion.
          EXPECT_NEAR(p[0] / dx, 1.0f, kRoundTripTol) << Label(lc, fov, vp, dx, 0.0f);
          EXPECT_NEAR(p[1], 0.0f, std::abs(dx) * kRoundTripTol) << Label(lc, fov, vp, dx, 0.0f);
        }
      }
    }
  }
}

TEST(DragGain, VerticalDragMovesContentOnePixelPerPixel) {
  for (const auto& lc : DraggableLenses()) {
    for (float fov : FovLadder(lc)) {
      for (const auto& vp : Viewports()) {
        float gain = ComputeDragGainDegPerPixel(lc.lens_type, fov, vp.w, vp.h);
        ASSERT_GT(gain, 0.0f) << Label(lc, fov, vp, 0, 0);
        float step = ProbePixels(gain, vp.w, vp.h);
        for (float dy : { step, -step }) {
          auto p = RoundTrip(lc, fov, vp, 0.0f, dy);
          ASSERT_FALSE(IsSentinel(p)) << Label(lc, fov, vp, 0.0f, dy);
          // ImGui's mouse y is DOWN-positive, the projection's py is UP-positive, so
          // content following the cursor means py = -dy. This is also where a globe sign
          // regression would surface: its elevation update is -= where the others are +=.
          EXPECT_NEAR(p[1] / dy, -1.0f, kRoundTripTol) << Label(lc, fov, vp, 0.0f, dy);
          EXPECT_NEAR(p[0], 0.0f, std::abs(dy) * kRoundTripTol) << Label(lc, fov, vp, 0.0f, dy);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------------------
// The same contract stated in whole mouse pixels, for the FOV band where a real drag is
// small enough that the first-order gain is a good description of a 20 px motion. This is
// the reading of AC1 closest to the user's hand; the probe tests above are what cover the
// extreme ends of the FOV range, where no whole-pixel step can be linear.
// ---------------------------------------------------------------------------------------
TEST(DragGain, RealisticDragStepsAtModerateFov) {
  struct Combo {
    int lens_type;
    const char* name;
    float fov;
  };
  const Combo kCombos[] = {
    { lumice::gui::kLensTypeLinear, "linear", 30.0f },
    { lumice::gui::kLensTypeLinear, "linear", 90.0f },
    { lumice::gui::kLensTypeLinear, "linear", 150.0f },
    { lumice::gui::kLensTypeFisheyeEqualArea, "fisheye_equal_area", 120.0f },
    { lumice::gui::kLensTypeFisheyeEqualArea, "fisheye_equal_area", 360.0f },
    { lumice::gui::kLensTypeFisheyeEquidist, "fisheye_equidistant", 180.0f },
    { lumice::gui::kLensTypeFisheyeEquidist, "fisheye_equidistant", 360.0f },
    { lumice::gui::kLensTypeFisheyeStereographic, "fisheye_stereographic", 120.0f },
    { lumice::gui::kLensTypeFisheyeOrthographic, "fisheye_orthographic", 180.0f },
    { lumice::gui::kLensTypeGlobe, "globe", 30.0f },
    { lumice::gui::kLensTypeGlobe, "globe", 90.0f },
  };
  const Viewport vp = { 1280, 720, "1280x720" };
  // 3% absorbs the second-order term of the widest combo here (linear at fov=150°, where a
  // 20 px step is 0.25 rad and tan(theta)/theta = 1.021). Every other combo lands well
  // inside it; this is a linearization budget, not a fudge factor.
  constexpr float kRelTol = 0.03f;

  for (const auto& c : kCombos) {
    LensCase lc = { c.lens_type, c.name, 0.0f };
    for (float d : { 5.0f, 20.0f, -5.0f, -20.0f }) {
      auto px = RoundTrip(lc, c.fov, vp, d, 0.0f);
      ASSERT_FALSE(IsSentinel(px)) << Label(lc, c.fov, vp, d, 0.0f);
      EXPECT_NEAR(px[0] / d, 1.0f, kRelTol) << Label(lc, c.fov, vp, d, 0.0f);

      auto py = RoundTrip(lc, c.fov, vp, 0.0f, d);
      ASSERT_FALSE(IsSentinel(py)) << Label(lc, c.fov, vp, 0.0f, d);
      EXPECT_NEAR(py[1] / d, -1.0f, kRelTol) << Label(lc, c.fov, vp, 0.0f, d);
    }
  }
}

// ---------------------------------------------------------------------------------------
// AC3, stated structurally rather than through the round trip: the gain is inversely
// proportional to framebuffer pixels. Doubling the DPI doubles the pixel delta of the same
// physical hand motion and halves the gain, so the rotation — the feel — is unchanged.
// ---------------------------------------------------------------------------------------
TEST(DragGain, ScalesInverselyWithFramebufferPixels) {
  for (const auto& lc : DraggableLenses()) {
    for (float fov : FovLadder(lc)) {
      float g1 = ComputeDragGainDegPerPixel(lc.lens_type, fov, 1280, 720);
      float g2 = ComputeDragGainDegPerPixel(lc.lens_type, fov, 2560, 1440);
      ASSERT_GT(g1, 0.0f) << lc.name;
      EXPECT_NEAR(g2 * 2.0f / g1, 1.0f, 1e-5f) << lc.name << " fov=" << fov;
    }
  }
}

// img_radius is min(w, h)/2 — the same definition ProjectWorldDirToScreen uses — so a
// viewport that is wide but the same height gives the same gain, and the short side is what
// governs. Pinning this keeps the two from drifting apart into a non-square mismatch.
TEST(DragGain, GovernedByShorterViewportSide) {
  for (const auto& lc : DraggableLenses()) {
    float wide = ComputeDragGainDegPerPixel(lc.lens_type, 60.0f, 2000, 720);
    float square = ComputeDragGainDegPerPixel(lc.lens_type, 60.0f, 720, 720);
    float tall = ComputeDragGainDegPerPixel(lc.lens_type, 60.0f, 720, 2000);
    EXPECT_FLOAT_EQ(wide, square) << lc.name;
    EXPECT_FLOAT_EQ(tall, square) << lc.name;
  }
}

// ---------------------------------------------------------------------------------------
// Closed-form anchors. These separate "the derivative is wrong" from "the projection is
// consuming it wrongly" when a round trip above goes red: pick combinations whose value is
// exact by hand, so a failure here indicts the formula alone.
// ---------------------------------------------------------------------------------------
TEST(DragGain, ClosedFormAnchors) {
  constexpr float kRad2Deg = 180.0f / kPi;

  // linear, fov=90 => half_fov=45°, tan(45°)=1 => 1/img_radius rad/px. img_radius = 400.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 90.0f, 800, 800), kRad2Deg / 400.0f, 1e-5f);

  // fisheye equidistant, fov=180 => half_fov=pi/2 => (pi/2)/img_radius rad/px, i.e. the
  // whole 90° half-field spans exactly img_radius pixels. img_radius = 300.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeEquidist, 180.0f, 800, 600), 90.0f / 300.0f,
              1e-5f);

  // fisheye orthographic, fov=180 => sin(90°)=1 => 1/img_radius rad/px. img_radius = 300.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeOrthographic, 180.0f, 800, 600),
              kRad2Deg / 300.0f, 1e-5f);

  // fisheye equal area, fov=180 => 2*sin(45°)=sqrt(2) => sqrt(2)/img_radius rad/px.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeEqualArea, 180.0f, 800, 600),
              std::sqrt(2.0f) * kRad2Deg / 300.0f, 1e-5f);

  // fisheye stereographic, fov=180 => 2*tan(45°)=2 => 2/img_radius rad/px.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeStereographic, 180.0f, 800, 600),
              2.0f * kRad2Deg / 300.0f, 1e-5f);

  // globe, fov=90 => (D-1)*tan(45°) = 3 => 3/img_radius rad/px. img_radius = 300.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeGlobe, 90.0f, 800, 600), 3.0f * kRad2Deg / 300.0f,
              1e-4f);
}

// ---------------------------------------------------------------------------------------
// Narrow-FOV coverage for the fisheye family, which the round trip above cannot reach (the
// oracle's acos loses the angle — see DraggableLenses).
//
// The check is against an independent physical fact rather than against the implementation:
// every projection is locally equidistant near its optical axis, so as fov -> 0 all five
// inside-out laws must converge to the SAME angular resolution, half_fov / img_radius. This
// is not a restatement of any branch — it is the property that pins each branch's constant
// factor. Drop the 2 from the equal-area law, or write sin where tan belongs, and the ratio
// leaves 1 immediately, at exactly the FOV band nothing else covers.
//
// Globe is the meaningful exception and is asserted at its own factor: its camera sits at
// D = kGlobeCameraD sphere radii away, so a given rotation of the sphere sweeps (D-1) = 3x
// the screen distance a pinhole at the same fov would give. Pinning 3 here is what would
// catch the camera distance and the gain law drifting apart.
// ---------------------------------------------------------------------------------------
TEST(DragGain, SmallFovLawsConvergeToEquidistant) {
  constexpr int kW = 800;
  constexpr int kH = 600;  // img_radius = 300
  for (float fov : { 1.0f, 2.0f, 5.0f }) {
    float reference = ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeEquidist, fov, kW, kH);
    ASSERT_GT(reference, 0.0f);
    // The equidistant law IS half_fov / img_radius, exactly and at every FOV — it is the
    // limit itself, so it is the natural yardstick for the other five.
    EXPECT_NEAR(reference, (fov * 0.5f) / 300.0f, 1e-6f) << "fov=" << fov;

    // Every law's departure from the shared limit is second order in half_fov, with 1/3 the
    // largest coefficient among the six (tan(hf)/hf - 1 = hf^2/3, hit by both linear and
    // globe), so hf^2 bounds them all with 3x to spare. Compared as a RELATIVE deviation:
    // globe's expected ratio is 3, and an absolute tolerance would silently give it three
    // times the budget the others get — which is exactly how the first version of this
    // assertion went red at fov=5° while claiming a 1e-3 bound.
    float half_fov_rad = fov * 0.5f * kPi / 180.0f;
    float tol = std::max(1e-4f, half_fov_rad * half_fov_rad);
    for (const auto& lc : DraggableLenses()) {
      float expected_ratio = (lc.lens_type == lumice::gui::kLensTypeGlobe) ? 3.0f : 1.0f;
      float g = ComputeDragGainDegPerPixel(lc.lens_type, fov, kW, kH);
      EXPECT_NEAR(g / reference / expected_ratio, 1.0f, tol) << lc.name << " fov=" << fov;
    }
  }
}

// The 0.3 deg/px the drag used before this became FOV-aware, quantified: at the default
// 30° linear preview it was ~2.3x too slow, and at fov=1° it was ~180x too fast. That
// spread is the defect AC1 names, recorded here so a regression to a constant is visible.
TEST(DragGain, FovAwareGainSpansTheRangeTheConstantMissed) {
  float narrow = ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 1.0f, 800, 600);
  float wide = ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 179.0f, 800, 600);
  EXPECT_LT(narrow, 0.3f / 100.0f);
  EXPECT_GT(wide, 0.3f * 50.0f);
}

// ---------------------------------------------------------------------------------------
// Degenerate inputs.
// ---------------------------------------------------------------------------------------
TEST(DragGain, DegenerateViewportProducesNoRotation) {
  EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 60.0f, 0, 600), 0.0f);
  EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 60.0f, 800, 0), 0.0f);
  EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 60.0f, -1, -1), 0.0f);
}

// Full-sky lenses never reach the drag branch (app_panels.cpp guards on !LensIsFullSky), so
// this is the defensive fallback, not a live path: it must stay finite and match the legacy
// constant rather than return 0 or NaN.
TEST(DragGain, FullSkyLensesFallBackToLegacyConstant) {
  for (int lt : lumice::gui::kFullSkyLensTypes) {
    EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lt, 180.0f, 800, 600), 0.3f) << "lens_type " << lt;
  }
}
