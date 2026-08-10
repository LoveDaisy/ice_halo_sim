// Composition chain: the preview's lens model, shared by everything drawn on top of it.
//
// Units in the chain: app_panels × preview_renderer × overlay_labels × gui_state.
//
// overlay_labels is a full member here, not fixture scaffolding: it carries the second, independent
// implementation of the same lens model (pixel -> direction), and the strongest case below is that
// the two implementations must invert each other. Neither unit can make that check alone.
//
// What the collaboration produces that is observable: where an overlay marker lands on the preview,
// and how far the image moves when the user drags it. Both are answers to the same question — what
// angle does this pixel correspond to — and both are computed from the renderer sub-state through
// the same lens. When they stop agreeing, nothing errors: the zenith marker simply sits somewhere
// that is not the zenith, which reads as a simulation that got the sky wrong, and dragging under a
// wide-angle lens becomes unusably fast, which reads as a broken mouse.
//
// Derived from the src call graph: app_panels.cpp -> preview_renderer is 11 call sites, and app.cpp
// -> preview_renderer is 16 — between them the projection helpers, the background aspect and the
// upload entry points.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

#include "gui/gui_state.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {
namespace {

constexpr float kSentinel = -9999.0f;

bool IsSentinel(const std::array<float, 2>& p) {
  return p[0] == kSentinel && p[1] == kSentinel;
}

RenderConfig LensAt(int lens_type, float fov_deg, float elevation_deg) {
  RenderConfig rc;
  rc.lens_type = lens_type;
  rc.fov = fov_deg;
  rc.elevation = elevation_deg;
  rc.azimuth = 0.0f;
  rc.roll = 0.0f;
  return rc;
}

// ---------------------------------------------------------------------------------------------
// E13 — the overlay's inverse and the marker projection's forward are inverses of each other.
//
// Two units answer the same question from opposite ends. overlay_labels maps a pixel back to the
// direction it shows (that is how a label knows what it is labelling); preview_renderer maps a
// direction forward to the pixel a marker is drawn at. They are separate implementations of one
// lens model, and nothing forces them to agree — when they drift, the zenith marker is simply drawn
// somewhere that is not the zenith, which reads as a simulation that got the sky wrong.
//
// Round-tripping through both is the check neither unit can make on its own, which is what puts it
// in this layer rather than in either unit's own tests.

TEST(PreviewProjectionChain, PixelToDirectionAndDirectionToPixelAreInverses) {
  constexpr float kResX = 800.0f;
  constexpr float kResY = 600.0f;

  struct LensCase {
    const char* name;
    int lens_type;
    float fov;
    float elevation;
  };
  const LensCase kLenses[] = {
    { "linear 60", LUMICE_LENS_TYPE_LINEAR, 60.0f, 0.0f },
    { "linear 20", LUMICE_LENS_TYPE_LINEAR, 20.0f, 0.0f },
    { "equal-area 120", LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, 120.0f, 0.0f },
    { "equal-area 120, looking up", LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, 120.0f, 45.0f },
    { "equidistant 180", LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT, 180.0f, 0.0f },
  };
  // Offsets from the viewport centre, in pixels. The centre itself is the anchor every other
  // marker is placed relative to, so it leads.
  const float kOffsets[][2] = { { 0.0f, 0.0f }, { 40.0f, 0.0f }, { 0.0f, 40.0f }, { -60.0f, 30.0f } };

  for (const LensCase& lens : kLenses) {
    RenderConfig rc = LensAt(lens.lens_type, lens.fov, lens.elevation);
    const ViewProjection vp = BuildPreviewViewProjFromRenderer(rc);
    float view_matrix[9] = {};
    BuildViewMatrix(vp.elevation, vp.azimuth, vp.roll, view_matrix);

    for (const auto& offset : kOffsets) {
      float wx = 0.0f;
      float wy = 0.0f;
      float wz = 0.0f;
      bool valid = false;
      detail::PixelToWorldDirForTesting(offset[0], offset[1], kResX, kResY, lens.lens_type, lens.fov, view_matrix, &wx,
                                        &wy, &wz, &valid);
      if (!valid) {
        continue;  // the pixel is outside this lens's domain; nothing is drawn there either
      }

      const float world_dir[3] = { wx, wy, wz };
      const std::array<float, 2> screen =
          ProjectWorldDirToScreen(vp, world_dir, static_cast<int>(kResX), static_cast<int>(kResY));
      if (IsSentinel(screen)) {
        ADD_FAILURE() << lens.name << " at (" << offset[0] << "," << offset[1]
                      << "): the inverse produced a direction the forward projection cannot place";
        continue;  // this combination has no coordinates to compare; the rest still get checked
      }
      EXPECT_NEAR(screen[0], offset[0], 1.0f) << lens.name << " x";
      EXPECT_NEAR(screen[1], offset[1], 1.0f) << lens.name << " y";
    }
  }
}

// The round trip above would also pass if BOTH directions were the identity on some degenerate
// path, so this pins that the projection actually separates directions: two different pixels must
// come back as two different pixels.
TEST(PreviewProjectionChain, DistinctPixelsStayDistinctThroughTheRoundTrip) {
  const RenderConfig rc = LensAt(LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, 120.0f, 0.0f);
  const ViewProjection vp = BuildPreviewViewProjFromRenderer(rc);
  float view_matrix[9] = {};
  BuildViewMatrix(vp.elevation, vp.azimuth, vp.roll, view_matrix);

  auto round_trip = [&](float px, float py) {
    float wx = 0.0f;
    float wy = 0.0f;
    float wz = 0.0f;
    bool valid = false;
    detail::PixelToWorldDirForTesting(px, py, 800.0f, 600.0f, vp.lens_type, vp.fov, view_matrix, &wx, &wy, &wz, &valid);
    EXPECT_TRUE(valid);
    const float dir[3] = { wx, wy, wz };
    return ProjectWorldDirToScreen(vp, dir, 800, 600);
  };

  const std::array<float, 2> a = round_trip(0.0f, 0.0f);
  const std::array<float, 2> b = round_trip(80.0f, 0.0f);
  ASSERT_FALSE(IsSentinel(a));
  ASSERT_FALSE(IsSentinel(b));
  EXPECT_GT(std::abs(b[0] - a[0]), 10.0f) << "two pixels 80 apart collapse to the same place";
}

// A direction behind the camera has no screen position, and the answer for that is a sentinel
// rather than a number. Clamping instead would pin an off-screen marker to the viewport edge,
// where it reads as a real feature sitting at the horizon.
TEST(PreviewProjectionChain, ADirectionBehindTheCameraHasNoScreenPosition) {
  const RenderConfig rc = LensAt(LUMICE_LENS_TYPE_LINEAR, 60.0f, 0.0f);
  const ViewProjection vp = BuildPreviewViewProjFromRenderer(rc);
  float view_matrix[9] = {};
  BuildViewMatrix(vp.elevation, vp.azimuth, vp.roll, view_matrix);

  float wx = 0.0f;
  float wy = 0.0f;
  float wz = 0.0f;
  bool valid = false;
  detail::PixelToWorldDirForTesting(0.0f, 0.0f, 800.0f, 600.0f, vp.lens_type, vp.fov, view_matrix, &wx, &wy, &wz,
                                    &valid);
  ASSERT_TRUE(valid);

  const float behind[3] = { -wx, -wy, -wz };
  EXPECT_TRUE(IsSentinel(ProjectWorldDirToScreen(vp, behind, 800, 600)));
}

// ---------------------------------------------------------------------------------------------
// E13 (drag half) — the drag law's whole purpose is that the ratio is CONSTANT.
//
// The value of the constant is calibration and may be retuned. What may not change is that content
// at the centre of the frame moves the same number of screen pixels per dragged pixel whatever the
// lens, the field of view or the viewport size. A gain that scales with FOV is what makes a
// wide-angle preview impossible to aim.

TEST(PreviewProjectionChain, DragMovesTheSameScreenDistancePerPixelAcrossFovAndViewport) {
  constexpr int kVpW = 800;
  constexpr int kVpH = 600;
  const int kLenses[] = { LUMICE_LENS_TYPE_LINEAR, LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA,
                          LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT };
  const float kFovs[] = { 20.0f, 60.0f, 120.0f };

  for (int lens : kLenses) {
    for (float fov : kFovs) {
      const float gain = ComputeDragGainDegPerPixel(lens, fov, kVpW, kVpH);
      if (!(gain > 0.0f)) {
        ADD_FAILURE() << "lens " << lens << " fov " << fov << ": no drag gain";
        continue;
      }

      // Turn the angular gain back into a screen distance: take the direction at the centre of the
      // frame, rotate it by one drag-pixel's worth of angle, and ask where that lands. That round
      // trip is the actual invariant — an angular gain can be "constant" while the projection it
      // feeds is not.
      const RenderConfig rc = LensAt(lens, fov, 0.0f);
      const ViewProjection vp = BuildPreviewViewProjFromRenderer(rc);
      float view_matrix[9] = {};
      BuildViewMatrix(vp.elevation, vp.azimuth, vp.roll, view_matrix);

      float cx = 0.0f;
      float cy = 0.0f;
      float cz = 0.0f;
      bool valid = false;
      detail::PixelToWorldDirForTesting(0.0f, 0.0f, static_cast<float>(kVpW), static_cast<float>(kVpH), lens, fov,
                                        view_matrix, &cx, &cy, &cz, &valid);
      if (!valid) {
        ADD_FAILURE() << "lens " << lens << " fov " << fov << ": the frame centre has no direction";
        continue;
      }

      // Rotate the centre direction by `gain` degrees about an axis perpendicular to it. Any
      // perpendicular will do: the drag law is isotropic by construction.
      // Not `M_PI`: that one is a POSIX extension, and MSVC only defines it when
      // `_USE_MATH_DEFINES` is set before <cmath> — so using it here builds on the
      // two Unix legs and breaks the Windows one, which is exactly how it got in.
      constexpr float kPiF = 3.14159265358979323846f;
      const float theta = gain * kPiF / 180.0f;
      float ax = -cy;
      float ay = cx;
      float az = 0.0f;
      const float axis_len = std::sqrt(ax * ax + ay * ay + az * az);
      if (!(axis_len > 1e-6f)) {
        ADD_FAILURE() << "lens " << lens << " fov " << fov << ": no perpendicular to rotate about";
        continue;
      }
      ax /= axis_len;
      ay /= axis_len;
      az /= axis_len;
      const float ct = std::cos(theta);
      const float st = std::sin(theta);
      const float dot = ax * cx + ay * cy + az * cz;
      const float offset_dir[3] = {
        cx * ct + (ay * cz - az * cy) * st + ax * dot * (1.0f - ct),
        cy * ct + (az * cx - ax * cz) * st + ay * dot * (1.0f - ct),
        cz * ct + (ax * cy - ay * cx) * st + az * dot * (1.0f - ct),
      };
      const std::array<float, 2> moved = ProjectWorldDirToScreen(vp, offset_dir, kVpW, kVpH);
      if (IsSentinel(moved)) {
        ADD_FAILURE() << "lens " << lens << " fov " << fov << ": one drag-pixel of rotation leaves the frame";
        continue;
      }

      const float pixels = std::sqrt(moved[0] * moved[0] + moved[1] * moved[1]);
      // The tolerance is wide on purpose: this asserts the invariant (a constant of order one
      // pixel), not the calibrated value, which the drag law's own header says may be retuned.
      EXPECT_GT(pixels, 0.5f) << "lens " << lens << " fov " << fov << ": dragging barely moves anything";
      EXPECT_LT(pixels, 20.0f) << "lens " << lens << " fov " << fov << ": one dragged pixel throws the view";
    }
  }
}

// The gain law's own edge cases — a degenerate viewport, and a full-sky lens that has no drag
// interaction and answers with the historical constant rather than with zero — are statements about
// ComputeDragGainDegPerPixel alone, and are asserted one layer down in
// test/unit-correctness/gui/test_preview_renderer.cpp (DragGain.*). What is above is the part that
// needs the projection as well: that the angular gain, fed back through the projection, actually
// moves the picture by a constant number of pixels.

// ---------------------------------------------------------------------------------------------
// E13 (roll half) — the lens types that ignore roll must see zero roll, at every consumer.
//
// BuildPreviewViewProjFromRenderer is where that normalisation happens once, so the overlay and the
// shader cannot disagree about whether the sky is tilted. A consumer that read rc.roll directly
// would draw its markers rotated against an image that is not.

TEST(PreviewProjectionChain, LensTypesThatIgnoreRollSeeZeroRollInTheProjection) {
  int zeroed = 0;
  for (int lens : { LUMICE_LENS_TYPE_LINEAR, LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT,
                    LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA, LUMICE_LENS_TYPE_RECTANGULAR, LUMICE_LENS_TYPE_GLOBE }) {
    RenderConfig rc = LensAt(lens, 90.0f, 0.0f);
    rc.roll = 30.0f;
    const float effective = EffectiveRollForLens(lens, 30.0f);
    EXPECT_FLOAT_EQ(BuildPreviewViewProjFromRenderer(rc).roll, effective)
        << "lens " << lens << ": the projection's roll is not the one the lens rule produced";
    zeroed += (effective == 0.0f) ? 1 : 0;
  }
  // And the rule itself has to actually zero SOMETHING, or the normalisation is a no-op that every
  // consumer could have skipped and the loop above is vacuous.
  EXPECT_GT(zeroed, 0) << "no lens discards roll";
}

}  // namespace
}  // namespace lumice::gui
