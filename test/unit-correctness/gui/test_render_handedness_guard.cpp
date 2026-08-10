// Render handedness regression guard.
//
// Purpose: pin the RIGHT = +az screen-handedness convention across the THREE
// independent forward-projection implementations, so a future sign flip on any
// single one is caught immediately. The three implementations are:
//   1. backend  — core/shared/projection_shared.h  lm_proj::ProjectExitToPixel
//                 (the CLI/render + Metal/CUDA single source; carries the
//                 `xy.x = -xy.x` flip at projection_shared.h:237).
//   2. gui_shader — src/gui/preview_renderer.cpp  ProjectWorldDirToScreen
//                 (CPU mirror of the marker-overlay fragment shader).
//   3. gui_label  — src/gui/overlay_labels.cpp  WorldDirToPixel
//                 (exposed via detail::WorldDirToPixelForTesting; the forward
//                 used for interaction and label placement).
// Plus an interaction read-back (detail::PixelToWorldDirForTesting) fed the
// cross-validated gui_shader pixel to pin the GUI inverse convention.
//
// The single-lens and globe cases below are the original scope. Surface 2b adds the
// dual-fisheye family, which has no backend/label counterpart to cross-check against here
// and is therefore pinned against its own closed form instead — see the kDualLens comment.
//
// These are genuinely independent implementations (different rotation builders —
// MakeCameraRotation vs BuildViewMatrix — different per-lens forward math, and
// the backend's explicit `xy.x = -xy.x` flip has NO counterpart in either GUI
// path). That independence is what gives the cross-check detection power: a
// flip on any one surface breaks exactly that surface's case and no other.
//
// Why ABSOLUTE screen-side (px > / < center) rather than a forward/inverse
// round-trip: round-trip closes by construction under ANY self-consistent
// convention and cannot detect a handedness flip. We assert the absolute side
// and pin it to the owner-decided value (single-lens family = RIGHT for +az;
// globe = LEFT, i.e. opposite, because globe is an outside-in view).
//
// View / world-dir params: single lens view (az=180, el=0, ro=0), globe view
// (az=0, el=0, ro=0); feature world dir az=±15°, el=+3° (the +3° elevation
// avoids globe's el=0 culling); W=640, H=320.
//
// This TU is a CROSS-LAYER oracle: it deliberately #includes both core/config
// and GUI headers, because comparing the backend forward against the two GUI
// forwards is the whole point. gui_unit_test is the only target in the repo
// that links lumice_gui_obj and lumice_obj together, so it is the only place
// this comparison can be made at all. check_policies.py's GUI API-boundary gate
// scans src/gui/ only, so a test TU is not in its scope either way.

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "core/shared/projection_shared.h"
#include "gui/gui_constants.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace {

// Cross-validated geometry (see the header comment).
constexpr int kW = 640;
constexpr int kH = 320;
constexpr float kElOffDeg = 3.0f;   // avoids globe el=0 culling
constexpr float kAzOffDeg = 15.0f;  // non-zero, non-symmetric anchor
constexpr float kSentinel = -9000.f;

// A world direction at azimuth az_deg, elevation kElOffDeg (world convention:
// wx=cos(el)cos(az), wy=cos(el)sin(az), wz=sin(el)). The SIGN of wy tracks the
// sign of az (wx stays positive for |az| < 90°), so wy is the az-axis probe used
// by the interaction read-back.
struct WorldDir {
  float x, y, z;
};
WorldDir MakeWorldDir(float az_deg) {
  const float el = kElOffDeg * lumice::math::kDegreeToRad;
  const float az = az_deg * lumice::math::kDegreeToRad;
  const float ce = std::cos(el);
  return { ce * std::cos(az), ce * std::sin(az), std::sin(el) };
}

// Minimal RenderConfig builder (mirror of the golden test's MakeRC — only the
// fields BuildProjParams / MakeCameraRotation read).
lumice::RenderConfig MakeRc(lumice::LensParam::LensType t, float fov_deg, float view_az_deg) {
  lumice::RenderConfig cfg;
  cfg.lens_.type_ = t;
  cfg.lens_.fov_ = fov_deg;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.lens_shift_[0] = 0;
  cfg.lens_shift_[1] = 0;
  cfg.visible_ = lumice::RenderConfig::kFull;
  cfg.overlap_ = 0.0f;
  cfg.view_.az_ = view_az_deg;
  cfg.view_.el_ = 0.0f;
  cfg.view_.ro_ = 0.0f;
  return cfg;
}

// The three forwards share one signature so the convention can be asserted over {surface} x {lens}
// as a rule rather than as one near-identical case per surface. `lens_type` is the GUI integer,
// which equals the LensParam::LensType enum value (render_config.hpp / gui_constants.hpp share the
// 0..10 ordering), so one number drives both sides.
using SurfacePxFn = bool (*)(int lens_type, float fov_deg, float view_az_deg, float world_az_deg, float* px_out);

// Backend forward: returns horizontal pixel offset from image center (>0 RIGHT).
bool BackendPx(int lens_type, float fov_deg, float view_az_deg, float world_az_deg, float* off_out) {
  auto cfg = MakeRc(static_cast<lumice::LensParam::LensType>(lens_type), fov_deg, view_az_deg);
  lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
  auto pp = lumice::BuildProjParams(cfg, rot, static_cast<float>(std::min(kW, kH)));
  WorldDir w = MakeWorldDir(world_az_deg);
  auto res = lm_proj::ProjectExitToPixel(pp, w.x, w.y, w.z);
  if (res.count < 1) {
    return false;
  }
  *off_out = static_cast<float>(res.hits[0].px) - static_cast<float>(kW) / 2.0f;
  return true;
}

// GUI shader-mirror forward: horizontal pixel offset from viewport center
// (ProjectWorldDirToScreen already returns center-origin, x-right pixels). Reports py as well as
// px, because the interaction read-back below has to be fed a full cross-validated pixel.
bool GuiShaderPxPy(int lens_type, float fov_deg, float view_az_deg, float world_az_deg, float* px_out, float* py_out) {
  lumice::gui::ViewProjection vp;
  vp.lens_type = lens_type;
  vp.fov = fov_deg;
  vp.elevation = 0.0f;
  vp.azimuth = view_az_deg;
  vp.roll = 0.0f;
  vp.visible = lumice::gui::kVisibleFull;
  vp.front = false;
  WorldDir w = MakeWorldDir(world_az_deg);
  const float dir[3] = { w.x, w.y, w.z };
  auto p = lumice::gui::ProjectWorldDirToScreen(vp, dir, kW, kH);
  if (p[0] <= kSentinel && p[1] <= kSentinel) {
    return false;
  }
  *px_out = p[0];
  *py_out = p[1];
  return true;
}

bool GuiShaderPx(int lens_type, float fov_deg, float view_az_deg, float world_az_deg, float* px_out) {
  float py = 0.0f;
  return GuiShaderPxPy(lens_type, fov_deg, view_az_deg, world_az_deg, px_out, &py);
}

// GUI interaction/label forward: horizontal pixel offset from viewport center.
bool GuiLabelPx(int lens_type, float fov_deg, float view_az_deg, float world_az_deg, float* px_out) {
  float vm[9];
  lumice::gui::BuildViewMatrix(/*elevation*/ 0.0f, /*azimuth*/ view_az_deg, /*roll*/ 0.0f, vm);
  WorldDir w = MakeWorldDir(world_az_deg);
  float out_px = 0.0f;
  float out_py = 0.0f;
  bool valid = false;
  lumice::gui::detail::WorldDirToPixelForTesting(w.x, w.y, w.z, static_cast<float>(kW), static_cast<float>(kH),
                                                 lens_type, fov_deg, vm, &out_px, &out_py, &valid);
  if (!valid) {
    return false;
  }
  *px_out = out_px;
  return true;
}

// GUI interaction read-back: pixel (center-origin, shader convention) → world
// dir. Returns the recovered az-axis component (world y).
bool GuiReadbackWorldY(int lens_type, float fov_deg, float view_az_deg, float px, float py, float* wy_out) {
  float vm[9];
  lumice::gui::BuildViewMatrix(/*elevation*/ 0.0f, /*azimuth*/ view_az_deg, /*roll*/ 0.0f, vm);
  float ox = 0.0f;
  float oy = 0.0f;
  float oz = 0.0f;
  bool valid = false;
  lumice::gui::detail::PixelToWorldDirForTesting(px, py, static_cast<float>(kW), static_cast<float>(kH), lens_type,
                                                 fov_deg, vm, &ox, &oy, &oz, &valid);
  if (!valid) {
    return false;
  }
  *wy_out = oy;
  return true;
}

// The three surfaces the convention is asserted over, named so a red says which one flipped.
struct Surface {
  const char* name;
  SurfacePxFn px;
};
constexpr Surface kSurfaces[] = {
  { "backend", BackendPx },
  { "gui_shader", GuiShaderPx },
  { "gui_label", GuiLabelPx },
};

// Single-lens family (5 types).
struct LensCase {
  lumice::LensParam::LensType t;
  float fov;
};
constexpr LensCase kSingleLens[] = {
  { lumice::LensParam::kLinear, 90.0f },
  { lumice::LensParam::kFisheyeEqualArea, 180.0f },
  { lumice::LensParam::kFisheyeEquidistant, 180.0f },
  { lumice::LensParam::kFisheyeStereographic, 180.0f },
  { lumice::LensParam::kFisheyeOrthographic, 180.0f },
};

constexpr float kSingleLensViewAz = 180.0f;  // camera faces world +x sun
constexpr float kGlobeViewAz = 0.0f;         // globe DOC-TEST view
constexpr float kGlobeFov = 60.0f;

// --- Dual-fisheye family (4 types) -------------------------------------------------
//
// These lenses are full-sky, so none of the single-lens framing above transfers: no view
// transform is applied, the frame carries TWO discs (upper hemisphere left, lower right),
// and each disc spans a fixed 90° from its own pole regardless of fov. "Which side of image
// center" is therefore meaningless here — the load-bearing statements are (a) which side of
// ITS OWN disc center a probe lands on, (b) how far from that center it lands, and (c) that
// the two discs mirror each other. (c) is a real convention, not an artifact: the shader's
// dualFisheyeInverse solves phi_pixel = PI/2 + az on the left disc and PI/2 - az on the
// right (preview_renderer.cpp), and the CPU forward mirrors it by flipping the sign of
// world y between hemispheres. Delete that flip and both discs become copies of one another
// — which only case (c) below can see.
//
// r_norm is written as a literal per type rather than computed by calling the same
// expression the implementation uses. That expression IS the type dispatch, so re-calling it
// would make the test agree with any future edit to it; a literal makes the edit fail here
// and have to be re-derived deliberately. Each row records its derivation.
struct DualLensCase {
  int lens_type;
  float r_norm;  // DualFisheyeRNorm(kDualProbeThetaDeg, type), normalized to circle_radius
};

// Probe polar angle, measured from the disc's pole (zenith on the left disc, nadir on the
// right). 45° sits mid-disc: far enough from the center that the four types separate
// (r_norm spans 0.41..0.71 here), far enough from the rim that no clipping guard is in play.
constexpr float kDualProbeThetaDeg = 45.0f;

constexpr DualLensCase kDualLens[] = {
  // equal area:    sin(θ/2) / sin(45°)  = sin(22.5°) / sin(45°)
  { lumice::gui::kLensTypeDualFisheyeEqualArea, 0.5411961f },
  // equidistant:   θ / 90°              = 0.5
  { lumice::gui::kLensTypeDualFisheyeEquidist, 0.5f },
  // stereographic: tan(θ/2) / tan(45°)  = tan(22.5°)
  { lumice::gui::kLensTypeDualFisheyeStereographic, 0.4142136f },
  // orthographic:  sin(θ)               = sin(45°)
  { lumice::gui::kLensTypeDualFisheyeOrthographic, 0.7071068f },
};

// Geometry of the dual-fisheye frame at kW x kH, mirroring the shader:
// short_res = min(W/2, H) = min(320, 320) = 320, circle_radius = short_res/2.
constexpr float kDualCircleRadius = 160.0f;

// Project a probe onto one dual-fisheye disc and report its offset from THAT disc's center.
// hemi_up selects the hemisphere (true = upper, which the projection puts on the left disc);
// y_positive selects the sign of world y, the az-axis probe used throughout this file.
// The probe has no x component, so its whole in-disc displacement is horizontal and the
// expected offset is exactly +/- r_norm * circle_radius.
bool DualDiscOffset(int lens_type, bool hemi_up, bool y_positive, float* dx_out, float* py_out) {
  const float theta = kDualProbeThetaDeg * lumice::math::kDegreeToRad;
  const float st = std::sin(theta);
  const float ct = std::cos(theta);
  // World convention: zenith is -z (see overlay_labels.cpp / the shader's -world_dir.z
  // altitude), so the upper hemisphere is the one with negative z.
  const float dir[3] = { 0.0f, y_positive ? st : -st, hemi_up ? -ct : ct };

  lumice::gui::ViewProjection vp;
  vp.lens_type = lens_type;
  vp.fov = 180.0f;  // ignored by the dual family (fixed 90° per hemisphere)
  vp.elevation = 0.0f;
  vp.azimuth = 0.0f;
  vp.roll = 0.0f;
  vp.visible = lumice::gui::kVisibleFull;
  vp.front = false;
  auto p = lumice::gui::ProjectWorldDirToScreen(vp, dir, kW, kH);
  if (p[0] <= kSentinel && p[1] <= kSentinel) {
    return false;
  }
  const float disc_center_x = hemi_up ? -kDualCircleRadius : kDualCircleRadius;
  *dx_out = p[0] - disc_center_x;
  *py_out = p[1];
  return true;
}

}  // namespace

// Surfaces 1-3 over the single-lens family: backend lm_proj::ProjectExitToPixel (which carries the
// `xy.x = -xy.x` flip at projection_shared.h:237), the GUI shader mirror ProjectWorldDirToScreen,
// and the GUI interaction/label forward WorldDirToPixel. Quantified over {surface} x {lens} rather
// than written out per surface: the claim is one convention holding everywhere, and a flip on any
// single surface fails exactly its own rows. The backend arm is redundant with the golden
// absolute-column pin and kept anyway, so the GUI-binary cross-check has an in-suite anchor.
TEST(RenderHandednessGuard, EverySurfacePutsPositiveAzimuthRightAcrossTheSingleLensFamily) {
  for (const Surface& s : kSurfaces) {
    for (const auto& c : kSingleLens) {
      SCOPED_TRACE(testing::Message() << s.name << ", lens " << static_cast<int>(c.t));
      float px = 0.0f;
      if (!s.px(static_cast<int>(c.t), c.fov, kSingleLensViewAz, kAzOffDeg, &px)) {
        ADD_FAILURE() << s.name << " lens " << static_cast<int>(c.t) << ": positive-az projection failed";
        continue;  // this surface/lens has no pixel to compare; the rest still get checked
      }
      EXPECT_GT(px, 0.0f);  // +az → RIGHT (owner-decided: right = +az)
      if (!s.px(static_cast<int>(c.t), c.fov, kSingleLensViewAz, -kAzOffDeg, &px)) {
        ADD_FAILURE() << s.name << " lens " << static_cast<int>(c.t) << ": negative-az projection failed";
        continue;
      }
      EXPECT_LT(px, 0.0f);  // -az → LEFT
    }
  }
}

// Surface 2b: GUI shader mirror ProjectWorldDirToScreen, dual-fisheye family. Until this
// case existed, three of the four types (equidistant / stereographic / orthographic) had no
// test on this surface at all: ProjectDualFisheye and DualFisheyeRNorm were reachable only
// through the equal-area path that the lens_proj pixel references happen to render.
TEST(RenderHandednessGuard, dual_lens_gui_shader) {
  // Upper (left) disc: +y lands RIGHT of the disc center, -y lands LEFT, both at the type's own
  // radius. A wrong r_norm formula moves the magnitude; a flipped az convention moves the side; a
  // flipped hemisphere test moves the probe to the other disc entirely (an offset of
  // +/-2*circle_radius, far outside the tolerance). The lower (right) disc is the MIRROR of it —
  // same radius, opposite side — and that pair is what pins the hemisphere sign split: equalizing
  // the two discs passes both upper rows and fails both lower ones.
  struct Probe {
    bool hemi_up;
    bool y_positive;
    float side;  // expected sign of the in-disc offset
  };
  constexpr Probe kProbes[] = {
    { true, true, +1.0f },
    { true, false, -1.0f },
    { false, true, -1.0f },
    { false, false, +1.0f },
  };

  for (const auto& c : kDualLens) {
    for (const Probe& p : kProbes) {
      SCOPED_TRACE(testing::Message() << "lens " << c.lens_type << ", hemi_up=" << p.hemi_up
                                      << ", y_positive=" << p.y_positive);
      float dx = 0.0f;
      float py = 0.0f;
      if (!DualDiscOffset(c.lens_type, p.hemi_up, p.y_positive, &dx, &py)) {
        ADD_FAILURE() << "lens " << c.lens_type << ", hemi_up=" << p.hemi_up << ", y_positive=" << p.y_positive
                      << ": projection failed";
        continue;  // this probe has no offset to compare; the rest still get checked
      }
      EXPECT_NEAR(dx, p.side * c.r_norm * kDualCircleRadius, 1e-3f);
      EXPECT_NEAR(py, 0.0f, 1e-3f);
    }
  }
}

// --- Globe: intentionally OPPOSITE side (LEFT for +az) because globe is an
// outside-in view (the GUI lens math was aligned to the CLI; backend `-cx`). Asserted per surface,
// same as above, so equalizing globe with the single-lens family cannot slip through silently.
// gui_label globe uses WorldDirToPixel's globe branch.
TEST(RenderHandednessGuard, EverySurfacePutsPositiveAzimuthLeftOnTheGlobe) {
  for (const Surface& s : kSurfaces) {
    SCOPED_TRACE(s.name);
    float px = 0.0f;
    if (!s.px(lumice::gui::kLensTypeGlobe, kGlobeFov, kGlobeViewAz, kAzOffDeg, &px)) {
      ADD_FAILURE() << s.name << ": positive-az globe projection failed";
      continue;  // this surface has no pixel to compare; the rest still get checked
    }
    EXPECT_LT(px, 0.0f);  // globe +az → LEFT (opposite of single-lens)
    if (!s.px(lumice::gui::kLensTypeGlobe, kGlobeFov, kGlobeViewAz, -kAzOffDeg, &px)) {
      ADD_FAILURE() << s.name << ": negative-az globe projection failed";
      continue;
    }
    EXPECT_GT(px, 0.0f);  // globe -az → RIGHT
  }
}

// --- Interaction read-back: pins the GUI inverse (PixelToWorldDir) az-axis
// convention. Per plan Step 3 / Round 2 review, the pixel fed to the inverse
// is the CROSS-VALIDATED gui_shader pixel (external absolute reference), NOT a
// self-computed forward∘inverse round-trip (which would close under any
// convention and have zero detection power). Recovered world-y sign must match
// the input az sign. Detection power is proven by the GUI-shader negative
// control 4c (which flips the input pixel), not the backend controls.
TEST(RenderHandednessGuard, interaction_readback) {
  for (const auto& c : kSingleLens) {
    const auto lt = static_cast<int>(c.t);
    for (const float az : { kAzOffDeg, -kAzOffDeg }) {
      SCOPED_TRACE(testing::Message() << "lens " << lt << ", az " << az);
      float px = 0.0f;
      float py = 0.0f;
      if (!GuiShaderPxPy(lt, c.fov, kSingleLensViewAz, az, &px, &py)) {
        ADD_FAILURE() << "lens " << lt << ", az " << az << ": gui_shader forward projection failed";
        continue;  // no pixel to feed the read-back; the rest still get checked
      }
      float wy = 0.0f;
      if (!GuiReadbackWorldY(lt, c.fov, kSingleLensViewAz, px, py, &wy)) {
        ADD_FAILURE() << "lens " << lt << ", az " << az << ": interaction read-back failed";
        continue;
      }
      // The sign of the recovered world y must follow the sign of the input azimuth.
      EXPECT_GT(wy * az, 0.0f);
    }
  }
}
