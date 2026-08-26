// Where the preview puts a sky direction on screen, and how far the sky moves when the user drags.
//
// The two halves of this file are one loop: ProjectWorldDirToScreen decides where a direction
// lands, and the drag law decides how much the view has to turn per pixel of mouse travel to move
// that landing point by a fixed amount. The drag cases use the projection as their oracle, so the
// projection's own anchors are asserted here too rather than in a file that could drift from it.
//
// Pure CPU: no GL context and no ImGui, which is why this lives in gui_unit_test rather than
// gui_test.
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
// angular resolution at the center is enormous — linear at fov=179° is ~54.7 deg per pixel —
// so a "20 px drag" there is a full revolution and no linearization survives it. The probe
// tests therefore normalize the step to a fixed small ANGLE; the realistic-drag test uses
// whole-pixel deltas and states its tolerance as the analytic second-order bound for the
// angle each case actually sweeps, so the budget follows the sensitivity instead of being a
// hand-tuned number that silently goes stale when the sensitivity changes.

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

// Screen pixels the content must travel per dragged pixel — the owner-calibrated sensitivity
// (AC1's revised anchor). Restated here as a literal rather than imported from the production
// side on purpose: this value is a product decision, so the test's job is to hold it, not to
// agree with whatever the implementation currently says. Changing kDragSensitivity in
// preview_renderer.cpp without changing this turns every round trip below red, which is the
// intended signal. The FOV-independence of the ratio — the actual defect this task fixes — is
// asserted separately and does not depend on the value.
constexpr float kExpectedSensitivity = 2.5f;

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
// fraction of the whole field, so the probe is also held to a drag whose RESULTING content
// displacement is 40% of img_radius, keeping the projected point comfortably inside the
// viewport (and inside the fisheye imaging circle) rather than clipping to the sentinel.
// The displacement is kExpectedSensitivity times the drag, hence the division.
constexpr float kProbeAngleRad = 0.02f;
constexpr float kProbeFrameFraction = 0.4f;

// 2e-3 is ~15x the worst linearization term in play and ~25x the worst oracle float noise,
// while any real defect in a gain branch (a missing factor of 2, a sin/tan swap, a wrong
// img_radius) is a several-percent-to-several-hundred-percent effect. It separates the two.
constexpr float kRoundTripTol = 2e-3f;

float ProbePixels(float gain_deg_per_px, int vp_w, int vp_h) {
  float gain_rad_per_px = gain_deg_per_px * kPi / 180.0f;
  float img_radius = std::min(static_cast<float>(vp_w), static_cast<float>(vp_h)) * 0.5f;
  return std::min(kProbeFrameFraction * img_radius / kExpectedSensitivity, kProbeAngleRad / gain_rad_per_px);
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
TEST(DragGain, ReferencePointIsFrameCenterAndGlobesIsMirrored) {
  for (const auto& lc : DraggableLenses()) {
    auto vp = MakeVp(lc.lens_type, 60.0f, 0.0f, 0.0f);
    auto p = ProjectWorldDirToScreen(vp, ReferenceWorldDir(lc.lens_type), 800, 600);
    if (IsSentinel(p)) {
      ADD_FAILURE() << lc.name << ": the reference direction projected to the sentinel";
      continue;  // no pixel to compare for this lens; the rest still get checked
    }
    EXPECT_NEAR(p[0], 0.0f, 1e-3f) << lc.name;
    EXPECT_NEAR(p[1], 0.0f, 1e-3f) << lc.name;
  }

  // ...and the asymmetry that choice rests on, both ways round: globe cannot show the inside-out
  // lenses' front direction (eye_dir.z = -1), and linear cannot show globe's.
  const float front[3] = { -1.0f, 0.0f, 0.0f };
  const float back[3] = { 1.0f, 0.0f, 0.0f };
  auto globe = MakeVp(lumice::gui::kLensTypeGlobe, 60.0f, 0.0f, 0.0f);
  EXPECT_TRUE(IsSentinel(ProjectWorldDirToScreen(globe, front, 800, 600)));
  EXPECT_FALSE(IsSentinel(ProjectWorldDirToScreen(globe, back, 800, 600)));
  auto linear = MakeVp(lumice::gui::kLensTypeLinear, 60.0f, 0.0f, 0.0f);
  EXPECT_FALSE(IsSentinel(ProjectWorldDirToScreen(linear, front, 800, 600)));
  EXPECT_TRUE(IsSentinel(ProjectWorldDirToScreen(linear, back, 800, 600)));
}

// ---------------------------------------------------------------------------------------
// AC1 / AC2 / AC3 — the main event. For every draggable lens, across its own full FOV range,
// on square / landscape / portrait / HiDPI viewports, in both directions on both axes: one
// pixel of drag moves the content kExpectedSensitivity pixels. Constant across the FOV range
// is the whole point — the same assertion with the same tolerance holds at fov=1° and at
// fov=MaxFov.
// ---------------------------------------------------------------------------------------
TEST(DragGain, DraggingMovesContentAtAFixedPixelRatioOnBothAxes) {
  for (const auto& lc : DraggableLenses()) {
    for (float fov : FovLadder(lc)) {
      for (const auto& vp : Viewports()) {
        float gain = ComputeDragGainDegPerPixel(lc.lens_type, fov, vp.w, vp.h);
        if (!(gain > 0.0f)) {
          ADD_FAILURE() << Label(lc, fov, vp, 0, 0) << ": non-positive drag gain";
          continue;  // no gain to probe with for this combination; the rest still get checked
        }
        const float step = ProbePixels(gain, vp.w, vp.h);
        for (const bool horizontal : { true, false }) {
          for (float d : { step, -step }) {
            const float dx = horizontal ? d : 0.0f;
            const float dy = horizontal ? 0.0f : d;
            auto p = RoundTrip(lc, fov, vp, dx, dy);
            if (IsSentinel(p)) {
              ADD_FAILURE() << Label(lc, fov, vp, dx, dy) << ": round trip landed on the sentinel";
              continue;
            }
            // Content follows the cursor: +dx of mouse motion => +k*dx of screen-x motion.
            // ImGui's mouse y is DOWN-positive and the projection's py is UP-positive, so the
            // vertical arm is py = -k*dy. That arm is also where a globe sign regression would
            // surface: its elevation update is -= where the others are +=.
            //
            // Written as a ratio against the expectation so kRoundTripTol stays a RELATIVE
            // budget; comparing p/d against k directly would silently hand it k times the slack.
            const float along = horizontal ? p[0] : p[1];
            const float across = horizontal ? p[1] : p[0];
            EXPECT_NEAR(along / (d * kExpectedSensitivity), horizontal ? 1.0f : -1.0f, kRoundTripTol)
                << Label(lc, fov, vp, dx, dy);
            EXPECT_NEAR(across, 0.0f, std::abs(d) * kExpectedSensitivity * kRoundTripTol) << Label(lc, fov, vp, dx, dy);
          }
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

  // The tolerance is DERIVED per case, not tuned: the gain is a first derivative, so the
  // round trip is off by each law's second-order term in the swept angle theta. Over the six
  // laws those terms are theta^2/3 (linear, globe), theta^2/12 (stereographic), theta^2/6
  // (orthographic), theta^2/24 (equal-area) and exactly 0 (equidistant), so theta^2/3 bounds
  // them all; 1.5x covers the theta^4 remainder, and a 0.5% floor keeps the near-zero cases
  // from being pinned tighter than float noise.
  //
  // A single hand-tuned constant was what this test used to have, and raising the sensitivity
  // to k=2.5 multiplied every swept angle by 2.5 and quadrupled these terms — linear at
  // fov=150° went from 1.5% to 10.3%. A fixed budget would have had to be widened by hand,
  // which reads identically to widening it to hide a defect. Deriving it removes the choice.
  auto tolerance_for = [](float gain_deg_per_px, float pixels) {
    float theta = std::abs(pixels) * gain_deg_per_px * kPi / 180.0f;
    return std::max(0.005f, 1.5f * theta * theta / 3.0f);
  };

  for (const auto& c : kCombos) {
    LensCase lc = { c.lens_type, c.name, 0.0f };
    float gain = ComputeDragGainDegPerPixel(c.lens_type, c.fov, vp.w, vp.h);
    if (!(gain > 0.0f)) {
      ADD_FAILURE() << c.name << ": non-positive drag gain";
      continue;  // no gain to probe with for this combo; the rest still get checked
    }
    for (float d : { 5.0f, 20.0f, -5.0f, -20.0f }) {
      float tol = tolerance_for(gain, d);
      // The budget must stay well under the effects it has to separate: the smallest defect
      // this suite is built to catch is a missing factor (>=2x, i.e. 100%).
      if (!(tol < 0.2f)) {
        ADD_FAILURE() << Label(lc, c.fov, vp, d, 0.0f) << ": derived tolerance " << tol << " exceeds the budget";
        continue;  // the tolerance itself is unusable for this step; the rest still get checked
      }

      auto px = RoundTrip(lc, c.fov, vp, d, 0.0f);
      if (IsSentinel(px)) {
        ADD_FAILURE() << Label(lc, c.fov, vp, d, 0.0f) << ": horizontal round trip landed on the sentinel";
        continue;
      }
      EXPECT_NEAR(px[0] / (d * kExpectedSensitivity), 1.0f, tol) << Label(lc, c.fov, vp, d, 0.0f);

      auto py = RoundTrip(lc, c.fov, vp, 0.0f, d);
      if (IsSentinel(py)) {
        ADD_FAILURE() << Label(lc, c.fov, vp, 0.0f, d) << ": vertical round trip landed on the sentinel";
        continue;
      }
      EXPECT_NEAR(py[1] / (d * kExpectedSensitivity), -1.0f, tol) << Label(lc, c.fov, vp, 0.0f, d);
    }
  }
}

// ---------------------------------------------------------------------------------------
// AC3, stated structurally rather than through the round trip: the gain is inversely
// proportional to framebuffer pixels. Doubling the DPI doubles the pixel delta of the same
// physical hand motion and halves the gain, so the rotation — the feel — is unchanged.
// ---------------------------------------------------------------------------------------
TEST(DragGain, TheGainIsGovernedByTheShorterSideAndScalesInverselyWithIt) {
  for (const auto& lc : DraggableLenses()) {
    for (float fov : FovLadder(lc)) {
      float g1 = ComputeDragGainDegPerPixel(lc.lens_type, fov, 1280, 720);
      float g2 = ComputeDragGainDegPerPixel(lc.lens_type, fov, 2560, 1440);
      if (!(g1 > 0.0f)) {
        ADD_FAILURE() << lc.name << " fov=" << fov << ": non-positive drag gain";
        continue;  // no gain to form a ratio with for this fov; the rest still get checked
      }
      EXPECT_NEAR(g2 * 2.0f / g1, 1.0f, 1e-5f) << lc.name << " fov=" << fov;
    }
    // img_radius is min(w, h)/2 — the same definition ProjectWorldDirToScreen uses — so a viewport
    // that is wide but the same height gives the same gain, and the short side is what governs.
    // Pinning this keeps the two from drifting apart into a non-square mismatch.
    float square = ComputeDragGainDegPerPixel(lc.lens_type, 60.0f, 720, 720);
    EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lc.lens_type, 60.0f, 2000, 720), square) << lc.name;
    EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lc.lens_type, 60.0f, 720, 2000), square) << lc.name;
  }
}

// ---------------------------------------------------------------------------------------
// Closed-form anchors. These separate "the derivative is wrong" from "the projection is
// consuming it wrongly" when a round trip above goes red: pick combinations whose value is
// exact by hand, so a failure here indicts the formula alone.
// ---------------------------------------------------------------------------------------
TEST(DragGain, ClosedFormAnchors) {
  constexpr float kRad2Deg = 180.0f / kPi;
  // Every anchor is the 1:1 closed form times the sensitivity, so a k that stopped being
  // applied — or got applied to the legacy fallback too — shows up here as well.
  constexpr float kK = kExpectedSensitivity;

  // linear, fov=90 => half_fov=45°, tan(45°)=1 => 1/img_radius rad/px. img_radius = 400.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 90.0f, 800, 800), kK * kRad2Deg / 400.0f, 1e-5f);

  // fisheye equidistant, fov=180 => half_fov=pi/2 => (pi/2)/img_radius rad/px, i.e. the
  // whole 90° half-field spans exactly img_radius pixels. img_radius = 300.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeEquidist, 180.0f, 800, 600), kK * 90.0f / 300.0f,
              1e-5f);

  // fisheye orthographic, fov=180 => sin(90°)=1 => 1/img_radius rad/px. img_radius = 300.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeOrthographic, 180.0f, 800, 600),
              kK * kRad2Deg / 300.0f, 1e-5f);

  // fisheye equal area, fov=180 => 2*sin(45°)=sqrt(2) => sqrt(2)/img_radius rad/px.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeEqualArea, 180.0f, 800, 600),
              kK * std::sqrt(2.0f) * kRad2Deg / 300.0f, 1e-5f);

  // fisheye stereographic, fov=180 => 2*tan(45°)=2 => 2/img_radius rad/px.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeFisheyeStereographic, 180.0f, 800, 600),
              kK * 2.0f * kRad2Deg / 300.0f, 1e-5f);

  // globe, fov=90 => (D-1)*tan(45°) = 3 => 3/img_radius rad/px. img_radius = 300.
  EXPECT_NEAR(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeGlobe, 90.0f, 800, 600), kK * 3.0f * kRad2Deg / 300.0f,
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
    if (!(reference > 0.0f)) {
      ADD_FAILURE() << "fov=" << fov << ": non-positive reference drag gain";
      continue;  // no reference to form a ratio with for this fov; the rest still get checked
    }
    // The equidistant law IS half_fov / img_radius, exactly and at every FOV — it is the
    // limit itself, so it is the natural yardstick for the other five. (Scaled by the
    // sensitivity, like every branch; the five ratios below are unaffected by that scale,
    // which is why this one line is where k enters this test.)
    EXPECT_NEAR(reference, kExpectedSensitivity * (fov * 0.5f) / 300.0f, 1e-6f) << "fov=" << fov;

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

// The 0.3 deg/px the drag used before this became FOV-aware, quantified on this 800x600
// viewport: at fov=1° the constant was ~72x too fast (0.3 vs 0.0042 deg/px) and at fov=179°
// ~180x too slow (0.3 vs 54.7). The sensitivity calibration deliberately keeps the two laws
// close in the mid band — 2.3x apart at the 30° default here — so the SPREAD is what this
// pins, and it is the defect AC1 names: a regression to any single constant fails one end.
TEST(DragGain, TheEdgesOfTheGainLawSpanTheRangeDegradeSafelyAndKeepTheLegacyFallback) {
  float narrow = ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 1.0f, 800, 600);
  float wide = ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 179.0f, 800, 600);
  EXPECT_LT(narrow, 0.3f / 50.0f);
  EXPECT_GT(wide, 0.3f * 100.0f);

  // Degenerate viewports happen during window setup and on a collapsed panel: answering with a
  // sensible zero rather than dividing by it is what keeps the first frame after a resize from
  // teleporting the view.
  EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 60.0f, 0, 600), 0.0f);
  EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 60.0f, 800, 0), 0.0f);
  EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lumice::gui::kLensTypeLinear, 60.0f, -1, -1), 0.0f);

  // Full-sky lenses never reach the drag branch (app_panels.cpp guards on !LensIsFullSky), so this
  // is the defensive fallback, not a live path: it must stay finite and match the legacy constant
  // rather than return 0 or NaN.
  //
  // Exactly 0.3, NOT 0.3 * kExpectedSensitivity: the fallback IS the historical feel value, and the
  // sensitivity constant exists to bring the closed forms up to it, not to be stacked on top of it.
  // Applying k to this branch too would be a silent 2.5x on the one path that is supposed to change
  // nothing, so the exact compare here is the guard against that.
  for (int lt : lumice::gui::kFullSkyLensTypes) {
    EXPECT_FLOAT_EQ(ComputeDragGainDegPerPixel(lt, 180.0f, 800, 600), 0.3f) << "lens_type " << lt;
  }
}

// ---------------------------------------------------------------------------------------
// ProjectWorldDirToScreen, the forward projection the drag round trips above use as their
// oracle — and which the overlay's label placement uses for real. Every row below is an
// anchor whose pixel is derivable in closed form from the shader's own projection law, so a
// changed formula turns red rather than agreeing with itself.
//
// The world frame at elevation = azimuth = roll = 0 puts the camera's forward at (-1, 0, 0),
// the zenith at (0, 0, -1) and the nadir at (0, 0, +1) — z-down, matching overlay_labels.cpp.
// Screen coordinates are the projection's own y-up pixel frame, centred on the viewport.
//
// The sentinel rows are as load-bearing as the positional ones: a direction the lens cannot
// show must come back as the sentinel and not as a plausible pixel, or the overlay draws a
// label for a point that is not on screen.
// ---------------------------------------------------------------------------------------
TEST(PreviewRenderer, EveryProjectionPutsItsAnchorDirectionsWhereItsGeometryDemands) {
  constexpr float kPxEps = 1.0f;  // +-1 px against the analytic expectation

  struct ProjectionCase {
    const char* name;
    int lens_type;
    float fov;
    float elevation;
    int vp_w;
    int vp_h;
    std::array<float, 3> world_dir;
    bool expect_sentinel;
    float px;  // ignored when expect_sentinel
    float py;
  };
  // Directions reused across rows.
  constexpr std::array<float, 3> kFront = { -1.0f, 0.0f, 0.0f };
  constexpr std::array<float, 3> kZenith = { 0.0f, 0.0f, -1.0f };
  constexpr std::array<float, 3> kNadir = { 0.0f, 0.0f, 1.0f };

  const ProjectionCase kCases[] = {
    // Linear pinhole: forward lands dead centre; the zenith sits exactly on the camera plane
    // (view_dir.z == 0), which is the perspective singularity rather than a wide angle.
    { "linear, forward", lumice::gui::kLensTypeLinear, 90.0f, 0.0f, 800, 600, kFront, false, 0.0f, 0.0f },
    { "linear, the zenith is on the camera plane", lumice::gui::kLensTypeLinear, 90.0f, 0.0f, 800, 600, kZenith, true,
      0.0f, 0.0f },

    // Equal-area fisheye looking straight up: the zenith is now the optical axis, and the
    // horizon lands on the imaging circle. img_radius = min(800, 800) / 2 = 400, and the
    // world forward maps to view (0, -1, 0), i.e. theta = pi/2 at phi = -pi/2 — the bottom of
    // the image, where r_norm = 1 makes r exactly img_radius.
    { "fisheye equal-area at the zenith, looking up", lumice::gui::kLensTypeFisheyeEqualArea, 180.0f, 90.0f, 800, 800,
      kZenith, false, 0.0f, 0.0f },
    { "fisheye equal-area, the horizon on the circle", lumice::gui::kLensTypeFisheyeEqualArea, 180.0f, 90.0f, 800, 800,
      kFront, false, 0.0f, -400.0f },

    // Equirectangular: latitude maps linearly to py. 800x400 gives scale = 400/pi, so the
    // poles land at +-(pi/2)(400/pi) = +-200 and the equator at 0.
    { "rectangular, the zenith", lumice::gui::kLensTypeRectangular, 180.0f, 0.0f, 800, 400, kZenith, false, 0.0f,
      -200.0f },
    { "rectangular, the nadir", lumice::gui::kLensTypeRectangular, 180.0f, 0.0f, 800, 400, kNadir, false, 0.0f,
      200.0f },
    { "rectangular, the equator ahead", lumice::gui::kLensTypeRectangular, 180.0f, 0.0f, 800, 400, kFront, false, 0.0f,
      0.0f },

    // Dual fisheye: one disc per hemisphere, so the two poles are the two disc centres.
    // 800x400 gives circle_radius = 200, hence centres at -+200.
    { "dual fisheye, the zenith is the left disc centre", lumice::gui::kLensTypeDualFisheyeEqualArea, 180.0f, 0.0f, 800,
      400, kZenith, false, -200.0f, 0.0f },
    { "dual fisheye, the nadir is the right disc centre", lumice::gui::kLensTypeDualFisheyeEqualArea, 180.0f, 0.0f, 800,
      400, kNadir, false, 200.0f, 0.0f },

    // Globe, viewed from outside at distance kGlobeCameraD. Tilting the camera to look upward
    // brings the zenith to the near pole, dead centre; the nadir is then around the back of the
    // sphere and must be culled rather than drawn over the front of it.
    { "globe, the near pole", lumice::gui::kLensTypeGlobe, 30.0f, -90.0f, 800, 800, kZenith, false, 0.0f, 0.0f },
    { "globe, the far pole is behind the sphere", lumice::gui::kLensTypeGlobe, 30.0f, -90.0f, 800, 800, kNadir, true,
      0.0f, 0.0f },
  };

  for (const ProjectionCase& c : kCases) {
    auto vp = MakeVp(c.lens_type, c.fov, c.elevation, 0.0f);
    auto p = ProjectWorldDirToScreen(vp, c.world_dir.data(), c.vp_w, c.vp_h);
    EXPECT_EQ(IsSentinel(p), c.expect_sentinel) << c.name;
    if (c.expect_sentinel) {
      continue;
    }
    EXPECT_NEAR(p[0], c.px, kPxEps) << c.name;
    EXPECT_NEAR(p[1], c.py, kPxEps) << c.name;
  }
}

// A fisheye narrower than 180 degrees in a non-square viewport leaves black bars: pixels that
// are inside the rectangle but outside the imaging circle. A direction that lands there is not
// visible, and the projection has to say so rather than return a pixel in the bar.
//
// At elevation = azimuth = roll = 0 the view matrix maps world w to view (-w.y, -w.z, w.x), so
// a direction 60 degrees off the optical axis is w = (-1/2, -sqrt(3)/2, 0). At fov=90 its
// r_norm is (pi/3)/(pi/4) = 4/3, outside the circle; at fov=180 it is (pi/3)/(pi/2) = 2/3,
// comfortably inside. Same direction, same viewport: only the imaging circle moved.
TEST(PreviewRenderer, ADirectionOutsideTheImagingCircleIsNotGivenAPixelInTheBlackBars) {
  const float dir_60deg[3] = { -0.5f, -0.86602540378f, 0.0f };

  auto narrow = MakeVp(lumice::gui::kLensTypeFisheyeEquidist, 90.0f, 0.0f, 0.0f);
  EXPECT_TRUE(IsSentinel(ProjectWorldDirToScreen(narrow, dir_60deg, 1920, 1080)));

  auto wide = MakeVp(lumice::gui::kLensTypeFisheyeEquidist, 180.0f, 0.0f, 0.0f);
  EXPECT_FALSE(IsSentinel(ProjectWorldDirToScreen(wide, dir_60deg, 1920, 1080)));
}

// ---------------------------------------------------------------------------------------
// The background overlay's UV transform, and the modifier key that arms its canvas gestures.
//
// Both are pure and live in preview_renderer.hpp for the same reason the drag gain does: a GL
// context is not needed to state what they owe, so they are asserted here rather than in a
// suite that must open a window.
// ---------------------------------------------------------------------------------------

// The identity case is the whole backward-compatibility argument, so it is asserted bit for bit
// rather than within a tolerance. An .lmc written before bg_offset_x/bg_offset_y/bg_scale existed
// deserializes to (0, 0, 1); if that does not reproduce the historical hard-coded centered fit
// EXACTLY, every such document renders slightly differently after this change and there is no
// compat branch anywhere to catch it.
TEST(BgTransform, TheIdentityTransformReproducesTheHistoricalCenteredFitBitForBit) {
  struct Row {
    int vp_w;
    int vp_h;
    float bg_aspect;
  };
  // Square, both letterbox directions, and a wide viewport against a wide image.
  const Row kRows[] = {
    { 800, 800, 1.0f },
    { 1920, 1080, 1.0f },
    { 800, 800, 16.0f / 9.0f },
    { 1920, 1080, 4.0f / 3.0f },
  };

  for (const Row& row : kRows) {
    // The expression the production code carried inline before ComputeBgUvTransform existed.
    const float vp_aspect = static_cast<float>(row.vp_w) / static_cast<float>(row.vp_h);
    float sx = 1.0f;
    float sy = 1.0f;
    if (vp_aspect > row.bg_aspect) {
      sx = row.bg_aspect / vp_aspect;
    } else {
      sy = vp_aspect / row.bg_aspect;
    }

    const auto t = lumice::gui::ComputeBgUvTransform(row.vp_w, row.vp_h, row.bg_aspect, 0.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(t.scale_x, 0.5f / sx) << "vp " << row.vp_w << "x" << row.vp_h;
    EXPECT_FLOAT_EQ(t.scale_y, -0.5f / sy) << "vp " << row.vp_w << "x" << row.vp_h;
    EXPECT_FLOAT_EQ(t.offset_x, 0.5f) << "vp " << row.vp_w << "x" << row.vp_h;
    EXPECT_FLOAT_EQ(t.offset_y, 0.5f) << "vp " << row.vp_w << "x" << row.vp_h;
  }
}

// Zoom and pan are independent knobs: zoom moves only the scale, pan moves only the offset. If
// zoom leaked into the offset the image would slide sideways while being scaled (the classic
// "zoom is not about the center" bug); if pan leaked into the scale the photo would breathe while
// being dragged. Both are the kind of coupling that looks fine in a still screenshot.
TEST(BgTransform, ZoomScalesOnlyTheScaleAndPanShiftsOnlyTheOffset) {
  const auto base = lumice::gui::ComputeBgUvTransform(1920, 1080, 4.0f / 3.0f, 0.0f, 0.0f, 1.0f);

  for (float zoom : { 0.25f, 0.5f, 2.0f, 4.0f }) {
    const auto z = lumice::gui::ComputeBgUvTransform(1920, 1080, 4.0f / 3.0f, 0.0f, 0.0f, zoom);
    EXPECT_FLOAT_EQ(z.scale_x, base.scale_x / zoom) << "zoom " << zoom;
    EXPECT_FLOAT_EQ(z.scale_y, base.scale_y / zoom) << "zoom " << zoom;
    EXPECT_FLOAT_EQ(z.offset_x, base.offset_x) << "zoom " << zoom;
    EXPECT_FLOAT_EQ(z.offset_y, base.offset_y) << "zoom " << zoom;
  }

  for (float pan : { -0.75f, -0.1f, 0.1f, 0.75f }) {
    const auto p = lumice::gui::ComputeBgUvTransform(1920, 1080, 4.0f / 3.0f, pan, -pan, 1.0f);
    EXPECT_FLOAT_EQ(p.scale_x, base.scale_x) << "pan " << pan;
    EXPECT_FLOAT_EQ(p.scale_y, base.scale_y) << "pan " << pan;
    EXPECT_FLOAT_EQ(p.offset_x, base.offset_x + pan) << "pan " << pan;
    EXPECT_FLOAT_EQ(p.offset_y, base.offset_y - pan) << "pan " << pan;
  }
}

// A unit of pan is a full texture width whatever the zoom, so the two knobs stay usable together:
// the slider's ±2 range means the same thing zoomed in as zoomed out. Stated as the round trip
// that actually matters — the NDC point that samples a given texel — because that is the property
// the canvas drag inverts to turn a mouse delta into a pan delta.
TEST(BgTransform, PanIsMeasuredInTextureWidthsIndependentlyOfZoom) {
  for (float zoom : { 0.5f, 1.0f, 3.0f }) {
    const auto t = lumice::gui::ComputeBgUvTransform(1024, 1024, 1.0f, 0.0f, 0.0f, zoom);
    const auto panned = lumice::gui::ComputeBgUvTransform(1024, 1024, 1.0f, 0.25f, 0.0f, zoom);

    // The NDC coordinate that samples u = 0.5 shifts by exactly pan / scale_x, i.e. a quarter of
    // the texture width expressed in the CURRENT zoom's NDC units.
    const float ndc_before = (0.5f - t.offset_x) / t.scale_x;
    const float ndc_after = (0.5f - panned.offset_x) / panned.scale_x;
    EXPECT_FLOAT_EQ(ndc_after - ndc_before, -0.25f / t.scale_x) << "zoom " << zoom;
  }
}

// The letterbox branch is picked by the aspect comparison alone; pan and zoom must not be able to
// flip which side gets the bars. A viewport wider than the image squeezes x (sx < 1) and leaves y
// alone, and the other way round — the same split the pixel-level letterbox case in
// test_background_overlay.cpp checks end to end.
TEST(BgTransform, TheLetterboxBranchIsChosenByAspectAloneAndSurvivesPanAndZoom) {
  // Viewport 2:1 against a 1:1 image — bars left and right, so x is the squeezed axis.
  for (float zoom : { 0.5f, 1.0f, 2.0f }) {
    const auto t = lumice::gui::ComputeBgUvTransform(1600, 800, 1.0f, 0.3f, -0.2f, zoom);
    // sx = 1/2, sy = 1  =>  |scale_x| = 2 * |scale_y|
    EXPECT_FLOAT_EQ(t.scale_x, 1.0f / zoom);
    EXPECT_FLOAT_EQ(t.scale_y, -0.5f / zoom);
  }

  // Viewport 1:1 against a 2:1 image — bars top and bottom, so y is the squeezed axis.
  for (float zoom : { 0.5f, 1.0f, 2.0f }) {
    const auto t = lumice::gui::ComputeBgUvTransform(800, 800, 2.0f, 0.3f, -0.2f, zoom);
    EXPECT_FLOAT_EQ(t.scale_x, 0.5f / zoom);
    EXPECT_FLOAT_EQ(t.scale_y, -1.0f / zoom);
  }
}

// Both platform branches of the pan/zoom modifier, on every platform that runs this suite.
//
// The point of the `is_apple` parameter is exactly this test: spelled as an `#if` inside the call
// site, the macOS branch would not exist in the Ubuntu CI binary at all, so "CI is green" would
// say nothing about it. What remains outside any automated reach is the platform FACT that a real
// mac reports Cmd as io.KeySuper — that is one layer below this function and is tracked as a
// human-verification item, not as something these assertions claim.
TEST(BgTransform, BothPlatformBranchesOfTheModifierKeyArePinned) {
  // Non-Apple: Alt arms it, Cmd/Super does not.
  EXPECT_TRUE(lumice::gui::BgTransformModifierDown(/*key_alt=*/true, /*key_super=*/false, /*is_apple=*/false));
  EXPECT_FALSE(lumice::gui::BgTransformModifierDown(/*key_alt=*/false, /*key_super=*/true, /*is_apple=*/false));

  // Apple: Cmd arms it, Alt/Option does not.
  EXPECT_TRUE(lumice::gui::BgTransformModifierDown(/*key_alt=*/false, /*key_super=*/true, /*is_apple=*/true));
  EXPECT_FALSE(lumice::gui::BgTransformModifierDown(/*key_alt=*/true, /*key_super=*/false, /*is_apple=*/true));

  // Neither key down is never armed; both down is armed on either platform, so a user holding an
  // extra modifier does not silently lose the gesture.
  EXPECT_FALSE(lumice::gui::BgTransformModifierDown(false, false, false));
  EXPECT_FALSE(lumice::gui::BgTransformModifierDown(false, false, true));
  EXPECT_TRUE(lumice::gui::BgTransformModifierDown(true, true, false));
  EXPECT_TRUE(lumice::gui::BgTransformModifierDown(true, true, true));

  // The build's own platform constant must agree with the compiler's view of it.
#if defined(__APPLE__)
  EXPECT_TRUE(lumice::gui::kBgModifierIsApple);
  EXPECT_STREQ(lumice::gui::kBgModifierName, "Cmd");
#else
  EXPECT_FALSE(lumice::gui::kBgModifierIsApple);
  EXPECT_STREQ(lumice::gui::kBgModifierName, "Alt");
#endif
}
