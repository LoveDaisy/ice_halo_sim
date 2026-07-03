// Render handedness regression guard (scrum-321.4).
//
// Purpose: pin the RIGHT = +az screen-handedness convention across the THREE
// independent forward-projection implementations, so a future sign flip on any
// single one is caught immediately. The three implementations are:
//   1. backend  — core/shared/projection_shared.h  lm_proj::ProjectExitToPixel
//                 (the CLI/render + Metal/CUDA single source; carries the 321.2
//                 `xy.x = -xy.x` flip at projection_shared.h:237).
//   2. gui_shader — src/gui/preview_renderer.cpp  ProjectWorldDirToScreen
//                 (CPU mirror of the marker-overlay fragment shader).
//   3. gui_label  — src/gui/overlay_labels.cpp  WorldDirToPixel
//                 (exposed via detail::WorldDirToPixelForTesting; the
//                 interaction/label-placement forward named in issue.md).
// Plus an interaction read-back (detail::PixelToWorldDirForTesting) fed the
// cross-validated gui_shader pixel to pin the GUI inverse convention.
//
// These are genuinely independent implementations (different rotation builders —
// MakeCameraRotation vs BuildViewMatrix — different per-lens forward math, and
// the backend's explicit `xy.x = -xy.x` flip has NO counterpart in either GUI
// path). That independence is what gives the cross-check detection power: see the
// per-surface negative controls 4a/4b/4c in the task plan.
//
// Why ABSOLUTE screen-side (px > / < center) rather than a forward/inverse
// round-trip: round-trip closes by construction under ANY self-consistent
// convention and cannot detect a handedness flip (scrum-321.1 audit;
// code-quality/assertion-and-coverage-traps). We assert the absolute side and
// pin it to the owner-decided value (single-lens family = RIGHT for +az; globe =
// LEFT, i.e. opposite, because globe is an outside-in view — scrum-320
// gui-lens-math-cli-alignment).
//
// View / world-dir params reuse the scrum-321.1-validated combination: single
// lens view (az=180, el=0, ro=0), globe view (az=0, el=0, ro=0); feature world
// dir az=±15°, el=+3° (the +3° elevation avoids globe's el=0 culling); W=640,
// H=320. This TU deliberately #includes both core and GUI headers — permitted
// because it lives under test/gui/ (check_policies.py's GUI API-boundary gate
// only scans src/gui/, not test/gui/).

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
#include "test_gui_shared.hpp"

namespace {

// scrum-321.1-validated geometry.
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

// Backend forward: returns horizontal pixel offset from image center (>0 RIGHT).
bool BackendPx(lumice::LensParam::LensType t, float fov_deg, float view_az_deg, float world_az_deg, float* off_out) {
  auto cfg = MakeRc(t, fov_deg, view_az_deg);
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
// (ProjectWorldDirToScreen already returns center-origin, x-right pixels).
bool GuiShaderPx(int lens_type, float fov_deg, float view_az_deg, float world_az_deg, float* px_out, float* py_out) {
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

// Single-lens family (5 types). The GUI lens_type integers equal the
// LensParam::LensType enum values (render_config.hpp / gui_constants.hpp share
// the 0..10 ordering), so one enum value drives both sides.
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

}  // namespace

void RegisterHandednessGuardTests(ImGuiTestEngine* engine) {
  // --- Single-lens family: three independent forwards must all land RIGHT for
  // +az and LEFT for -az. One test PER SURFACE (backend / gui_shader / gui_label)
  // so a single-surface handedness drift is uniquely identifiable in the pass/
  // fail set (this is what the per-surface negative controls 4a/4c key off).

  // Surface 1: backend lm_proj::ProjectExitToPixel. Pins the 321.2 flip
  // (projection_shared.h:237). Redundant with the golden absolute-column pin,
  // kept here so the GUI-binary cross-check has an in-suite backend anchor.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "single_lens_backend");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      for (const auto& c : kSingleLens) {
        float pos = 0.0f;
        IM_CHECK(BackendPx(c.t, c.fov, kSingleLensViewAz, kAzOffDeg, &pos));
        IM_CHECK(pos > 0.0f);  // +az → RIGHT (owner: right=+az, scrum-321.2)
        float neg = 0.0f;
        IM_CHECK(BackendPx(c.t, c.fov, kSingleLensViewAz, -kAzOffDeg, &neg));
        IM_CHECK(neg < 0.0f);  // -az → LEFT
      }
    };
  }

  // Surface 2: GUI shader mirror ProjectWorldDirToScreen.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "single_lens_gui_shader");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      for (const auto& c : kSingleLens) {
        auto lt = static_cast<int>(c.t);
        float px = 0.0f;
        float py = 0.0f;
        IM_CHECK(GuiShaderPx(lt, c.fov, kSingleLensViewAz, kAzOffDeg, &px, &py));
        IM_CHECK(px > 0.0f);  // +az → RIGHT (GUI is the reference side, scrum-321.1)
        float pxn = 0.0f;
        float pyn = 0.0f;
        IM_CHECK(GuiShaderPx(lt, c.fov, kSingleLensViewAz, -kAzOffDeg, &pxn, &pyn));
        IM_CHECK(pxn < 0.0f);  // -az → LEFT
      }
    };
  }

  // Surface 3: GUI interaction/label forward WorldDirToPixel (issue.md-named).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "single_lens_gui_label");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      for (const auto& c : kSingleLens) {
        auto lt = static_cast<int>(c.t);
        float px = 0.0f;
        IM_CHECK(GuiLabelPx(lt, c.fov, kSingleLensViewAz, kAzOffDeg, &px));
        IM_CHECK(px > 0.0f);  // +az → RIGHT
        float pxn = 0.0f;
        IM_CHECK(GuiLabelPx(lt, c.fov, kSingleLensViewAz, -kAzOffDeg, &pxn));
        IM_CHECK(pxn < 0.0f);  // -az → LEFT
      }
    };
  }

  // --- Globe: intentionally OPPOSITE side (LEFT for +az) because globe is an
  // outside-in view (scrum-320 gui-lens-math-cli-alignment; backend `-cx`). Pin
  // per surface so equalizing globe with the single-lens family cannot slip
  // through silently. gui_label globe uses WorldDirToPixel's globe branch.

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "globe_backend");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      float pos = 0.0f;
      IM_CHECK(BackendPx(lumice::LensParam::kGlobe, kGlobeFov, kGlobeViewAz, kAzOffDeg, &pos));
      IM_CHECK(pos < 0.0f);  // globe +az → LEFT (opposite of single-lens)
      float neg = 0.0f;
      IM_CHECK(BackendPx(lumice::LensParam::kGlobe, kGlobeFov, kGlobeViewAz, -kAzOffDeg, &neg));
      IM_CHECK(neg > 0.0f);  // globe -az → RIGHT
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "globe_gui_shader");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      float px = 0.0f;
      float py = 0.0f;
      IM_CHECK(GuiShaderPx(lumice::gui::kLensTypeGlobe, kGlobeFov, kGlobeViewAz, kAzOffDeg, &px, &py));
      IM_CHECK(px < 0.0f);  // globe +az → LEFT
      float pxn = 0.0f;
      float pyn = 0.0f;
      IM_CHECK(GuiShaderPx(lumice::gui::kLensTypeGlobe, kGlobeFov, kGlobeViewAz, -kAzOffDeg, &pxn, &pyn));
      IM_CHECK(pxn > 0.0f);  // globe -az → RIGHT
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "globe_gui_label");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      float px = 0.0f;
      IM_CHECK(GuiLabelPx(lumice::gui::kLensTypeGlobe, kGlobeFov, kGlobeViewAz, kAzOffDeg, &px));
      IM_CHECK(px < 0.0f);  // globe +az → LEFT
      float pxn = 0.0f;
      IM_CHECK(GuiLabelPx(lumice::gui::kLensTypeGlobe, kGlobeFov, kGlobeViewAz, -kAzOffDeg, &pxn));
      IM_CHECK(pxn > 0.0f);  // globe -az → RIGHT
    };
  }

  // --- Interaction read-back: pins the GUI inverse (PixelToWorldDir) az-axis
  // convention. Per plan Step 3 / Round 2 review, the pixel fed to the inverse
  // is the CROSS-VALIDATED gui_shader pixel (external absolute reference), NOT a
  // self-computed forward∘inverse round-trip (which would close under any
  // convention and have zero detection power). Recovered world-y sign must match
  // the input az sign. Detection power is proven by the GUI-shader negative
  // control 4c (which flips the input pixel), not the backend controls.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "handedness_guard", "interaction_readback");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      for (const auto& c : kSingleLens) {
        auto lt = static_cast<int>(c.t);
        // +az: input world y > 0 ⇒ recovered world y must be > 0.
        float px = 0.0f;
        float py = 0.0f;
        IM_CHECK(GuiShaderPx(lt, c.fov, kSingleLensViewAz, kAzOffDeg, &px, &py));
        float wy = 0.0f;
        IM_CHECK(GuiReadbackWorldY(lt, c.fov, kSingleLensViewAz, px, py, &wy));
        IM_CHECK(wy > 0.0f);
        // -az: input world y < 0 ⇒ recovered world y must be < 0.
        float pxn = 0.0f;
        float pyn = 0.0f;
        IM_CHECK(GuiShaderPx(lt, c.fov, kSingleLensViewAz, -kAzOffDeg, &pxn, &pyn));
        float wyn = 0.0f;
        IM_CHECK(GuiReadbackWorldY(lt, c.fov, kSingleLensViewAz, pxn, pyn, &wyn));
        IM_CHECK(wyn < 0.0f);
      }
    };
  }
}
