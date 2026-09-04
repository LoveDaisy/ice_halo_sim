// The Look At presets' arithmetic: a named sky direction in, the camera angles that point at it out.
//
// THE ORACLE IS WRITTEN OUT LONGHAND HERE, ON PURPOSE. Every expected direction below is derived
// from the reflection rules stated in prose (annotation_overlay.hpp, and the header of the code
// under test), and every forward vector is rebuilt from three trigonometric lines typed into this
// file rather than obtained by calling BuildViewMatrix. An "equivalence" check that called the
// production formula on both sides would pass just as happily with the azimuth sign inverted
// everywhere — the exact failure mode task 487.1 recorded, where a shared implementation makes an
// oracle structurally blind to the thing it exists to catch.
//
// The judgement is therefore always an ANALYTIC RELATION — "the antisolar preset points 180 degrees
// away from the sun", "the sun-side horizon preset is level and shares the sun's bearing" — not a
// reproduction of what the code does.

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "gui/field_editor_registry.hpp"  // ConstraintFor — the gate and bounds the menu borrows
#include "gui/gui_state.hpp"              // kMarkerDisplayNames — the Overlay list this menu must agree with
#include "gui/view_look_at.hpp"

namespace {

using lumice::gui::LookAtId;

constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;

// The camera's forward vector at (az, el), typed out from the convention BuildViewMatrix documents
// rather than obtained from it. This is the whole oracle: "looking at D" means forward == D.
std::array<float, 3> ForwardAt(float az_deg, float el_deg) {
  const float a = az_deg * kDeg2Rad;
  const float e = el_deg * kDeg2Rad;
  return { -std::cos(e) * std::cos(a), -std::cos(e) * std::sin(a), -std::sin(e) };
}

// The sun as the GUI means it: azimuth is not exposed, so the sun always sits in the y = 0 plane,
// and altitude = asin(-z) puts a sun ABOVE the horizon at negative z.
std::array<float, 3> SunDirAt(float altitude_deg) {
  const float s = altitude_deg * kDeg2Rad;
  return { -std::cos(s), 0.0f, -std::sin(s) };
}

float Dot(const std::array<float, 3>& a, const std::array<float, 3>& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// The forward vector the preset produces, or a failure the caller can assert on.
std::array<float, 3> ForwardOfPreset(LookAtId id, float sun_altitude_deg, bool* ok) {
  float az = 12345.0f;
  float el = 12345.0f;
  *ok = lumice::gui::ResolveLookAtAzEl(id, sun_altitude_deg, &az, &el);
  if (!*ok) {
    return { 0.0f, 0.0f, 0.0f };
  }
  return ForwardAt(az, el);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// WorldDirToAzEl — the pose half, pinned at hand-computed points.
// ---------------------------------------------------------------------------------------------

TEST(ViewLookAtMath, InvertsTheCameraForwardAtHandComputedPoints) {
  // Each row is read straight off forward = (-cos(el)cos(az), -cos(el)sin(az), -sin(el)):
  // world -x is the direction the camera faces at (0, 0), so it must come back as (0, 0).
  const auto check = [](float dx, float dy, float dz, float want_az, float want_el, const char* what) {
    const float dir[3] = { dx, dy, dz };
    float az = 0.0f;
    float el = 0.0f;
    lumice::gui::WorldDirToAzEl(dir, &az, &el);
    EXPECT_NEAR(az, want_az, 1e-3f) << what << " azimuth";
    EXPECT_NEAR(el, want_el, 1e-3f) << what << " elevation";
  };
  check(-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, "the equirect centre");
  check(0.0f, 1.0f, 0.0f, -90.0f, 0.0f, "world +y");
  check(0.0f, -1.0f, 0.0f, 90.0f, 0.0f, "world -y");
  // World +x is a half turn from the centre. Asserted on the magnitude because +180 and -180 are
  // the same azimuth — both inside the slider's range, giving cameras no caller can tell apart —
  // and which one comes back is decided by the sign of the zero atan2 is handed.
  {
    const float back[3] = { 1.0f, 0.0f, 0.0f };
    float az = 0.0f;
    float el = 0.0f;
    lumice::gui::WorldDirToAzEl(back, &az, &el);
    EXPECT_NEAR(std::abs(az), 180.0f, 1e-3f);
    EXPECT_NEAR(el, 0.0f, 1e-3f);
  }
  // Altitude = asin(-z), so the zenith is z = -1. This sign is the one thing in this coordinate
  // family that is worth a dedicated row: the same English word means the opposite z in
  // doc/coordinate-convention.md.
  // The poles have no bearing at all, and this is the branch that says so: left to atan2's
  // signed-zero rule they would come back at -180, half a turn of camera spin on the way to a
  // direction that does not depend on the azimuth.
  check(0.0f, 0.0f, -1.0f, 0.0f, 90.0f, "the zenith");
  check(0.0f, 0.0f, 1.0f, 0.0f, -90.0f, "the nadir");
}

TEST(ViewLookAtMath, IgnoresTheMagnitudeOfTheDirectionItIsGiven) {
  const float unit[3] = { -0.6f, 0.48f, -0.64f };
  const float scaled[3] = { unit[0] * 25.0f, unit[1] * 25.0f, unit[2] * 25.0f };
  float az_u = 0.0f, el_u = 0.0f, az_s = 0.0f, el_s = 0.0f;
  lumice::gui::WorldDirToAzEl(unit, &az_u, &el_u);
  lumice::gui::WorldDirToAzEl(scaled, &az_s, &el_s);
  EXPECT_NEAR(az_s, az_u, 1e-4f);
  EXPECT_NEAR(el_s, el_u, 1e-4f);
}

TEST(ViewLookAtMath, AnswersFiniteAnglesAtThePolesRatherThanNaN) {
  // atan2(0, 0) is 0, not a NaN — the azimuth at a pole is undefined and 0 is one of the infinitely
  // many correct answers. What must never happen is a NaN reaching a camera angle: it would poison
  // the view matrix and blank the preview with no error anywhere.
  const auto check_finite = [](float dx, float dy, float dz) {
    const float dir[3] = { dx, dy, dz };
    float az = 0.0f;
    float el = 0.0f;
    lumice::gui::WorldDirToAzEl(dir, &az, &el);
    EXPECT_TRUE(std::isfinite(az)) << dx << "," << dy << "," << dz;
    EXPECT_TRUE(std::isfinite(el)) << dx << "," << dy << "," << dz;
  };
  check_finite(0.0f, 0.0f, -1.0f);
  check_finite(0.0f, 0.0f, 1.0f);
  check_finite(0.0f, 0.0f, 0.0f);         // no direction at all
  check_finite(0.0f, 0.0f, -1.0000002f);  // |z| past 1 by rounding, asin's domain edge
}

// ---------------------------------------------------------------------------------------------
// ResolveLookAtAzEl — the seven presets, each judged by the relation that defines it.
// ---------------------------------------------------------------------------------------------

TEST(ViewLookAtPresets, EachDirectionSatisfiesItsDefiningRelation) {
  // Three altitudes, one of them below the horizon: a sun that has set still has an anthelion and
  // a subsun, and the camera can still be pointed at them. A single altitude would let a rule that
  // confuses "negate the altitude" with "negate the bearing" pass by accident.
  const auto check_altitude = [](float alt) {
    const std::array<float, 3> sun = SunDirAt(alt);
    bool ok = false;

    const std::array<float, 3> f_sun = ForwardOfPreset(LookAtId::kSun, alt, &ok);
    ASSERT_TRUE(ok) << "sun at altitude " << alt;
    EXPECT_NEAR(Dot(f_sun, sun), 1.0f, 1e-4f) << "Sun must look straight at the sun, altitude " << alt;

    const std::array<float, 3> f_anti = ForwardOfPreset(LookAtId::kAntisolar, alt, &ok);
    ASSERT_TRUE(ok);
    EXPECT_NEAR(Dot(f_anti, sun), -1.0f, 1e-4f) << "Antisolar must look 180 degrees from the sun, altitude " << alt;

    // Subsun: the sun reflected in a horizontal surface — same bearing, negated altitude.
    const std::array<float, 3> subsun = { sun[0], sun[1], -sun[2] };
    const std::array<float, 3> f_subsun = ForwardOfPreset(LookAtId::kSubsun, alt, &ok);
    ASSERT_TRUE(ok);
    EXPECT_NEAR(Dot(f_subsun, subsun), 1.0f, 1e-4f) << "altitude " << alt;

    // Anthelion: opposite bearing, SAME altitude.
    const std::array<float, 3> anthelion = { -sun[0], -sun[1], sun[2] };
    const std::array<float, 3> f_anth = ForwardOfPreset(LookAtId::kAnthelion, alt, &ok);
    ASSERT_TRUE(ok);
    EXPECT_NEAR(Dot(f_anth, anthelion), 1.0f, 1e-4f) << "altitude " << alt;

    // The poles do not depend on the sun at all, which is half of what makes them poles.
    const std::array<float, 3> f_zen = ForwardOfPreset(LookAtId::kZenith, alt, &ok);
    ASSERT_TRUE(ok);
    EXPECT_NEAR(Dot(f_zen, { 0.0f, 0.0f, -1.0f }), 1.0f, 1e-4f) << "altitude " << alt;
    const std::array<float, 3> f_nad = ForwardOfPreset(LookAtId::kNadir, alt, &ok);
    ASSERT_TRUE(ok);
    EXPECT_NEAR(Dot(f_nad, { 0.0f, 0.0f, 1.0f }), 1.0f, 1e-4f) << "altitude " << alt;

    // Sun-side horizon: level, and on the sun's side rather than the opposite one. The second half
    // is what a sign slip would break, and a "parallel to the sun's bearing" test alone would not
    // notice it.
    float az_h = 0.0f;
    float el_h = 0.0f;
    ASSERT_TRUE(lumice::gui::ResolveLookAtAzEl(LookAtId::kSunHorizon, alt, &az_h, &el_h));
    EXPECT_NEAR(el_h, 0.0f, 1e-4f) << "Sun-side horizon must be level, altitude " << alt;
    const std::array<float, 3> f_horiz = ForwardAt(az_h, el_h);
    const float sun_h_len = std::sqrt(sun[0] * sun[0] + sun[1] * sun[1]);
    ASSERT_GT(sun_h_len, 1e-3f) << "test setup: altitude " << alt << " has no bearing to compare";
    EXPECT_NEAR(f_horiz[0] * sun[0] / sun_h_len + f_horiz[1] * sun[1] / sun_h_len, 1.0f, 1e-4f)
        << "Sun-side horizon must share the sun's bearing, not oppose it; altitude " << alt;
  };
  check_altitude(25.0f);
  check_altitude(0.0f);
  check_altitude(-15.0f);  // sun below the horizon
}

TEST(ViewLookAtPresets, TheSevenDirectionsAreDistinctForAGenericSun) {
  // Without this, every relation above could be satisfied by an implementation that returned one
  // direction for several ids — each assertion only looks at its own row.
  const float alt = 33.0f;
  std::array<std::array<float, 3>, static_cast<int>(LookAtId::kCount)> forwards{};
  for (int i = 0; i < static_cast<int>(LookAtId::kCount); ++i) {
    bool ok = false;
    forwards[i] = ForwardOfPreset(static_cast<LookAtId>(i), alt, &ok);
    EXPECT_TRUE(ok) << "id " << i;
  }
  for (int i = 0; i < static_cast<int>(LookAtId::kCount); ++i) {
    for (int j = i + 1; j < static_cast<int>(LookAtId::kCount); ++j) {
      EXPECT_LT(Dot(forwards[i], forwards[j]), 0.999f) << "ids " << i << " and " << j << " point the same way";
    }
  }
}

TEST(ViewLookAtPresets, SunSideHorizonStaysFiniteThroughTheDegenerateBand) {
  // AC5: as the sun approaches a pole its bearing stops existing, and core falls back to a FIXED
  // direction (world +x, azimuth 0) rather than a nearest-neighbour one. What this case pins is the
  // consequence for a camera angle: finite, level, and — at the two poles — the SAME answer, which
  // a nearest-neighbour rule would not give, since the float residue of cos() changes sign across
  // +/-90 and would send the two poles 180 degrees apart.
  const auto check_finite = [](float alt) {
    float az = 0.0f;
    float el = 0.0f;
    ASSERT_TRUE(lumice::gui::ResolveLookAtAzEl(LookAtId::kSunHorizon, alt, &az, &el)) << "altitude " << alt;
    EXPECT_TRUE(std::isfinite(az)) << "altitude " << alt;
    EXPECT_TRUE(std::isfinite(el)) << "altitude " << alt;
    EXPECT_NEAR(el, 0.0f, 1e-4f) << "altitude " << alt;
  };
  check_finite(89.0f);
  check_finite(90.0f - 1e-7f);
  check_finite(90.0f);
  check_finite(-(90.0f - 1e-7f));
  check_finite(-90.0f);

  float az_up = 0.0f, el_up = 0.0f, az_down = 0.0f, el_down = 0.0f;
  ASSERT_TRUE(lumice::gui::ResolveLookAtAzEl(LookAtId::kSunHorizon, 90.0f, &az_up, &el_up));
  ASSERT_TRUE(lumice::gui::ResolveLookAtAzEl(LookAtId::kSunHorizon, -90.0f, &az_down, &el_down));
  EXPECT_FLOAT_EQ(az_up, az_down) << "the fixed fallback must not depend on which pole the sun is at";
  // The fallback is world +x, a half turn from the equirect centre. Magnitude again, for the same
  // signed-zero reason as above; what this pins is that the fallback is that direction and not
  // some other one, so a change to it shows up here rather than passing silently.
  EXPECT_NEAR(std::abs(az_up), 180.0f, 1e-3f);
}

TEST(ViewLookAtPresets, RejectsAnOutOfRangeIdWithoutWritingAnything) {
  float az = 4242.0f;
  float el = 4242.0f;
  EXPECT_FALSE(lumice::gui::ResolveLookAtAzEl(static_cast<LookAtId>(-1), 20.0f, &az, &el));
  EXPECT_FALSE(
      lumice::gui::ResolveLookAtAzEl(static_cast<LookAtId>(static_cast<int>(LookAtId::kCount)), 20.0f, &az, &el));
  // A rejected preset must leave the camera where it was rather than point it somewhere arbitrary.
  EXPECT_EQ(az, 4242.0f);
  EXPECT_EQ(el, 4242.0f);
}

// ---------------------------------------------------------------------------------------------
// AC6 — the menu and the Overlay list must show ONE name per direction.
// ---------------------------------------------------------------------------------------------

TEST(ViewLookAtNames, MarkerEntriesReadTheirNameFromTheOverlayListsOwnTable) {
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    // Pointer equality, not string equality: it says the menu READS the table rather than agreeing
    // with it today, so a rename in one place cannot leave the two showing different words.
    EXPECT_EQ(lumice::gui::LookAtDisplayName(static_cast<LookAtId>(i)), lumice::gui::kMarkerDisplayNames[i])
        << "marker id " << i;
  }
  EXPECT_STREQ(lumice::gui::LookAtDisplayName(LookAtId::kSunHorizon), "Sun-side horizon");
  EXPECT_EQ(lumice::gui::LookAtDisplayName(static_cast<LookAtId>(static_cast<int>(LookAtId::kCount))), nullptr);
}

// ---------------------------------------------------------------------------------------------
// The gate and the bounds the menu borrows from the Az/El sliders. What the panel does with them
// is a frame-level claim and lives in test/gui/functional/test_view_display_controls.cpp; what is
// asserted here is that the registry actually HAS what the panel reads — in particular a reason
// string to put in the disabled tooltip, which no gui_test case can see (SetTooltip draws through
// TextUnformatted, id == 0, invisible to the test engine's item registry).
// ---------------------------------------------------------------------------------------------

TEST(ViewLookAtGate, TheElevationConstraintCarriesEverythingTheMenuNeedsToReadOffIt) {
  lumice::gui::GuiState state;

  const auto check_lens = [&state](int lens, bool want_enabled, float want_limit) {
    state.renderer.lens_type = lens;
    const lumice::gui::FieldEditorConstraint el = lumice::gui::ConstraintFor("renderer.elevation", state);
    EXPECT_EQ(el.enabled, want_enabled) << "lens " << lens;
    if (want_enabled) {
      // The bounds the menu clamps with. Globe stopping one degree short of the pole is the case
      // that makes clamping load-bearing rather than decorative.
      EXPECT_TRUE(el.has_numeric_domain) << "lens " << lens;
      EXPECT_DOUBLE_EQ(el.max_value, want_limit) << "lens " << lens;
      EXPECT_DOUBLE_EQ(el.min_value, -want_limit) << "lens " << lens;
    } else {
      // A disabled entry with nothing to say is worse than no entry: "Look At is grey and I do not
      // know why" is the confusion this string exists to prevent.
      EXPECT_NE(el.disabled_reason, nullptr) << "lens " << lens << " is gated but offers no reason";
    }
  };
  check_lens(lumice::gui::kLensTypeLinear, true, 90.0f);
  check_lens(lumice::gui::kLensTypeGlobe, true, 89.0f);
  check_lens(lumice::gui::kLensTypeDualFisheyeEqualArea, false, 0.0f);
}
