// Cross-implementation gate for the annotation overlay: core's forward projection and label
// anchors against the GUI's own, over the same view, direction by direction and label by label.
//
// WHY THE COMPARISON EXISTS. The annotation layer is a NET REDUCTION or it is nothing: core grows
// a curve walk so the GUI can delete one, and the CLI can draw the same lines for the first time.
// Deleting the GUI's copy is only safe if the replacement is an equivalent, not a second
// approximation, so equivalence has to be demonstrated before the deletion — that is this file.
//
// THE TWO SIDES.
//   core: lm_proj::ProjectExitToPixel, the forward all three trace backends run. It is the
//         function that decides where a photon actually lands, so an annotation built on it is by
//         construction drawn where the render puts the sky.
//   GUI:  overlay_labels.cpp's WorldDirToPixel, the CPU mirror of the preview fragment shader's
//         re-projection. src/gui/ may not include core/ and GLSL cannot include C++, so this
//         second copy of the lens math exists on purpose (projection_shared.h's header comment
//         says as much).
// The plan for this task chose core's forward over porting the GUI's, on the grounds that the
// former is the product-level answer and the latter is a mirror of a mirror. That choice is only
// sound if the two agree inside the render domain, which is what ProjectionAgrees* below measures.
//
// KNOWN DIVERGENCES, pinned rather than hidden — none of them new here:
//   1. Single-lens fisheye past the equator. Core's domain ends at theta = 90 deg (the `cz <= 0`
//      cull ProjectExitToPixel applies); the GUI's ends at its asin guard, theta = 180 deg. Same
//      difference test_visible_mask_gui_parity.cpp pins for the mask. Every annotation this task
//      serves (parallels / meridians / angular-distance circles / horizon / zenith-nadir) works
//      inside the render domain, so the tests below sample there.
//   2. Rectangular carries the camera azimuth. Core folds it into ProjParams::az0
//      (ComputeScaleAz0); the GUI treats rectangular as full-sky and applies no view transform at
//      all. The two therefore agree only where az0 is zero. Pinned by
//      ProjectionRectangularDivergesUnderCameraAzimuth.
//   3. Dual fisheye with a non-zero overlap ring. core's r_scale changes the mapping; the GUI's
//      display dual-fisheye is fixed at 180 deg per hemisphere and ignores it. Held at overlap = 0
//      throughout, for the same reason test_visible_mask_gui_parity.cpp holds it there.
//
// PIXEL CONVENTIONS. Core answers in image space (x right, y down, origin top-left) and bins with
// floor(v + 0.5), i.e. it rounds the continuous coordinate. The GUI answers as a continuous offset
// from the viewport centre, y up. ToImageSpace() below converts, and the tolerance is
// kPixelTol = 1.0, which covers core's half-pixel rounding with room for float noise; it is NOT a
// slack that would hide a projection difference, since every divergence in the list above is
// hemisphere-scale, not sub-pixel.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/annotation_overlay.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "gui/gui_constants.hpp"
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace {

using lumice::LensParam;
using lumice::RenderConfig;
namespace ann = lumice::annotation;

constexpr float kPixelTol = 1.0f;
constexpr float kDeg2Rad = lumice::math::kDegreeToRad;

ann::ViewSnapshot MakeView(LensParam::LensType type, float fov, int w, int h, RenderConfig::VisibleRange vis, float el,
                           float az = 0.0f, float ro = 0.0f) {
  ann::ViewSnapshot v;
  v.lens_type = type;
  v.fov_deg = fov;
  v.width = w;
  v.height = h;
  v.visible = vis;
  v.el_deg = el;
  v.az_deg = az;
  v.roll_deg = ro;
  return v;
}

// Core's forward for a view, with the hemisphere cull neutralised the same way ComputeOverlay
// neutralises it (the annotation layer owns that policy, not the lens branch).
lm_proj::ProjParams CoreForwardParams(const ann::ViewSnapshot& view) {
  const RenderConfig cfg = ann::ToRenderConfig(view);
  const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(view.width, view.height));
  lm_proj::ProjParams p = lumice::BuildProjParams(cfg, rot, short_pix);
  p.visible_range = static_cast<int>(RenderConfig::kFull);
  return p;
}

struct GuiPoint {
  float px = 0.0f;  // image space, x right
  float py = 0.0f;  // image space, y down
  bool valid = false;
};

// The GUI's forward, converted out of its centred y-up convention into the image space core
// answers in.
GuiPoint GuiForward(const ann::ViewSnapshot& view, const float view_matrix[9], float wx, float wy, float wz) {
  float px = 0.0f;
  float py = 0.0f;
  bool valid = false;
  lumice::gui::detail::WorldDirToPixelForTesting(wx, wy, wz, static_cast<float>(view.width),
                                                 static_cast<float>(view.height), static_cast<int>(view.lens_type),
                                                 view.fov_deg, view_matrix, &px, &py, &valid);
  if (!valid) {
    return {};
  }
  return { px + static_cast<float>(view.width) / 2.0f, static_cast<float>(view.height) / 2.0f - py, true };
}

// A world direction from (altitude, azimuth) in the convention every annotation curve uses:
// altitude = asin(-z), azimuth = atan2(-y, -x).
void DirFromAltAz(float alt_deg, float az_deg, float out[3]) {
  const float ca = std::cos(alt_deg * kDeg2Rad);
  out[0] = -ca * std::cos(az_deg * kDeg2Rad);
  out[1] = -ca * std::sin(az_deg * kDeg2Rad);
  out[2] = -std::sin(alt_deg * kDeg2Rad);
}

struct ProjDiff {
  int compared = 0;    // both sides put the direction on the canvas
  int mismatched = 0;  // both drew it, at different pixels
  int core_only = 0;   // core drew it, the GUI did not
  int gui_only = 0;    // the GUI drew it, core did not
  int border = 0;      // landed in the undecidable half-pixel band at the canvas edge
  float worst = 0.0f;  // largest disagreement among `compared`
  std::string worst_at;
  std::string first_gap;
};

// THE ORACLE IS THE CANVAS, not the raw forward. "Where does this direction land" is only a
// question about the same thing on both sides once the answer is a pixel of THIS image: a
// rectilinear lens maps a near-grazing direction to a coordinate in the billions, and comparing
// two such coordinates measures float cancellation, not projection agreement. A direction is
// therefore DRAWN when the forward accepts it and the pixel falls inside [0, w) x [0, h) — which
// is exactly the test ComputeOverlay's curve walk applies before it will anchor a label.
struct CanvasHit {
  float px = 0.0f;
  float py = 0.0f;
  bool drawn = false;
};

// Equirectangular is a CYLINDER: core folds the longitude back into [0, w)
// (ProjectExitToPixel's `((raw_x % img_w) + img_w) % img_w`), while the GUI lets it run past the
// edge and relies on its viewport clip. Column 0 and column w name the SAME meridian, so the
// GUI's coordinate is folded the same way before either the canvas test or the distance below —
// otherwise the seam reads as a whole-canvas disagreement about one azimuth that both sides in
// fact place identically.
float FoldHorizontal(const ann::ViewSnapshot& view, float px) {
  if (view.lens_type != LensParam::kRectangular) {
    return px;
  }
  const float w = static_cast<float>(view.width);
  float folded = std::fmod(px, w);
  if (folded < 0.0f) {
    folded += w;
  }
  return folded;
}

CanvasHit ToCanvasHit(const ann::ViewSnapshot& view, float px, float py, bool valid) {
  if (!valid) {
    return {};
  }
  const float x = FoldHorizontal(view, px);
  if (x < 0.0f || x >= static_cast<float>(view.width) || py < 0.0f || py >= static_cast<float>(view.height)) {
    return {};
  }
  return { x, py, true };
}

// The half-pixel band along the canvas edge where "inside or outside" is not decidable at this
// resolution. Core bins with floor(v + 0.5) about res/2 — half a pixel off the symmetric centre
// convention the GUI uses (the asymmetry is stated in lens_proj_build.hpp's own header comment),
// so a direction landing exactly on the image-circle rim or a frame corner can round in on one
// side and out on the other. That is a binning artefact, not a projection difference, and a
// comparison whose unit is one pixel cannot resolve it. Samples in the band are counted and
// reported rather than silently dropped, and the caller asserts the band stays a rim.
bool NearCanvasBorder(const ann::ViewSnapshot& view, float px, float py) {
  const float w = static_cast<float>(view.width);
  const float h = static_cast<float>(view.height);
  return px < kPixelTol || px > w - kPixelTol || py < kPixelTol || py > h - kPixelTol;
}

float HorizontalDistance(const ann::ViewSnapshot& view, float a, float b) {
  const float raw = std::fabs(a - b);
  if (view.lens_type != LensParam::kRectangular) {
    return raw;
  }
  return std::min(raw, static_cast<float>(view.width) - raw);
}

// Sweep a lattice of directions and compare the two forwards. `alt_lo/alt_hi` bound the altitude
// band so a caller can stay inside a lens's shared domain.
ProjDiff SweepDirections(const ann::ViewSnapshot& view, float alt_lo, float alt_hi) {
  float view_matrix[9];
  lumice::gui::BuildViewMatrix(view.el_deg, view.az_deg, view.roll_deg, view_matrix);
  const lm_proj::ProjParams p = CoreForwardParams(view);

  ProjDiff d;
  for (int ai = 0; ai <= 24; ++ai) {
    const float alt = alt_lo + (alt_hi - alt_lo) * static_cast<float>(ai) / 24.0f;
    for (int zi = 0; zi < 36; ++zi) {
      const float az = -180.0f + 10.0f * static_cast<float>(zi);
      float w[3];
      DirFromAltAz(alt, az, w);
      const ann::CanvasPoint c = ann::ProjectWorldDir(p, w[0], w[1], w[2]);
      const GuiPoint g = GuiForward(view, view_matrix, w[0], w[1], w[2]);
      const CanvasHit ch = ToCanvasHit(view, c.px, c.py, c.valid);
      const CanvasHit gh = ToCanvasHit(view, g.px, g.py, g.valid);
      std::ostringstream where;
      where << "alt=" << alt << " az=" << az;
      if (ch.drawn != gh.drawn) {
        const bool near_edge = (c.valid && NearCanvasBorder(view, FoldHorizontal(view, c.px), c.py)) ||
                               (g.valid && NearCanvasBorder(view, FoldHorizontal(view, g.px), g.py));
        if (near_edge) {
          ++d.border;
          continue;
        }
        if (ch.drawn) {
          if (d.core_only++ == 0) {
            d.first_gap = where.str() + " (core only)";
          }
        } else {
          if (d.gui_only++ == 0) {
            d.first_gap = where.str() + " (gui only)";
          }
        }
        continue;
      }
      if (!ch.drawn) {
        continue;
      }
      ++d.compared;
      const float dist = std::max(HorizontalDistance(view, ch.px, gh.px), std::fabs(ch.py - gh.py));
      if (dist > d.worst) {
        d.worst = dist;
        d.worst_at = where.str();
      }
      if (dist > kPixelTol) {
        ++d.mismatched;
      }
    }
  }
  return d;
}

// Full agreement: same drawn/not-drawn verdict everywhere outside the half-pixel edge band, same
// pixel everywhere inside it.
void ExpectForwardAgrees(const ann::ViewSnapshot& view, float alt_lo, float alt_hi, const char* label) {
  const ProjDiff d = SweepDirections(view, alt_lo, alt_hi);
  EXPECT_GT(d.compared, 0) << label << ": nothing was compared, the assertion is vacuous";
  EXPECT_EQ(d.core_only, 0) << label << ": core draws " << d.core_only
                            << " direction(s) the GUI leaves off the canvas; first " << d.first_gap;
  EXPECT_EQ(d.gui_only, 0) << label << ": the GUI draws " << d.gui_only
                           << " direction(s) core leaves off the canvas; first " << d.first_gap;
  EXPECT_EQ(d.mismatched, 0) << label << ": " << d.mismatched << " of " << d.compared << " directions land more than "
                             << kPixelTol << " px apart; worst " << d.worst << " px at " << d.worst_at;
  // The excluded band has to stay a RIM. If it grows into a region, the exclusion has stopped
  // describing a rounding boundary and started hiding a real difference.
  EXPECT_LT(d.border, d.compared / 5) << label << ": " << d.border << " of " << (d.border + d.compared)
                                      << " samples fell in the undecidable edge band — too many for a rim";
}

constexpr RenderConfig::VisibleRange kAllRanges[] = { RenderConfig::kUpper, RenderConfig::kLower, RenderConfig::kFull };

}  // namespace

// =================================================================================================
// Step 0 of this task: does core's product-level forward answer the same question as the GUI's
// shader mirror, inside the render domain?
// =================================================================================================

TEST(AnnotationOverlayGuiParity, ProjectionAgreesForLinear) {
  ExpectForwardAgrees(MakeView(LensParam::kLinear, 90.0f, 96, 72, RenderConfig::kFull, /*el=*/20.0f), -80.0f, 80.0f,
                      "linear");
  ExpectForwardAgrees(MakeView(LensParam::kLinear, 60.0f, 128, 96, RenderConfig::kFull, /*el=*/-15.0f, /*az=*/40.0f,
                               /*ro=*/12.0f),
                      -80.0f, 80.0f, "linear rolled");
}

TEST(AnnotationOverlayGuiParity, ProjectionAgreesForSingleFisheyeInsideTheRenderDomain) {
  // Camera at the zenith, so theta = 90 - alt and the altitude band below keeps every sample
  // inside core's theta <= 90 deg domain (the `cz <= 0` cull). Orthographic is excluded here and
  // pinned separately: it is the one member of the family whose two forwards disagree about the
  // region past the image circle, see the next test.
  const LensParam::LensType kTypes[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                         LensParam::kFisheyeStereographic };
  for (LensParam::LensType t : kTypes) {
    ExpectForwardAgrees(MakeView(t, 120.0f, 96, 96, RenderConfig::kFull, /*el=*/90.0f), 5.0f, 85.0f,
                        "fisheye at zenith");
    ExpectForwardAgrees(MakeView(t, 180.0f, 128, 128, RenderConfig::kFull, /*el=*/90.0f, /*az=*/33.0f, /*ro=*/7.0f),
                        5.0f, 85.0f, "fisheye all-sky, rolled");
  }
}

// KNOWN DIVERGENCE (pre-existing, not introduced here): the two orthographic forwards disagree
// about directions past the image circle but still inside the canvas RECTANGLE — the corners of a
// square frame reach sqrt(2) x the image radius, so there is such a region on every square canvas.
//   core: FisheyeOrthographicForward rejects only dz < 0, i.e. theta > 90 deg. Past the fov it
//         returns r_norm = sin(theta) / sin(fov/2) > 1 and the pixel lands outside the circle —
//         which is what the RENDER does too, so the annotation agrees with the image core draws.
//   GUI:  WorldDirToPixel rejects theta > half_fov outright, matching what its own inverse
//         (fisheyeInverse's `s > 1` asin guard) can recover.
// Neither side is wrong; they answer for two different pictures. Annotation curves are asked for
// inside the fov, so this band is out of the way in practice, and the shape is pinned rather than
// tolerated in the general assertion above.
TEST(AnnotationOverlayGuiParity, OrthographicDivergesOnlyPastTheImageCircle) {
  const ann::ViewSnapshot view =
      MakeView(LensParam::kFisheyeOrthographic, 120.0f, 96, 96, RenderConfig::kFull, /*el=*/90.0f);

  // Inside the fov (theta <= 60 deg at a zenith camera, i.e. alt >= 30 deg) the two agree exactly.
  ExpectForwardAgrees(view, 32.0f, 88.0f, "orthographic inside the fov");

  // Past it, every disagreement is core-drawing-more, never the reverse, and never a different
  // pixel for a direction both sides draw.
  const ProjDiff d = SweepDirections(view, 5.0f, 28.0f);
  EXPECT_GT(d.core_only, 0) << "the divergence this test pins has disappeared; re-derive it before relaxing";
  EXPECT_EQ(d.gui_only, 0) << "the GUI drew " << d.gui_only << " direction(s) core does not; first " << d.first_gap;
  EXPECT_EQ(d.mismatched, 0) << "a direction both sides draw landed " << d.worst << " px apart at " << d.worst_at;
}

TEST(AnnotationOverlayGuiParity, ProjectionAgreesForDualFisheye) {
  const LensParam::LensType kTypes[] = { LensParam::kDualFisheyeEqualArea, LensParam::kDualFisheyeEquidistant,
                                         LensParam::kDualFisheyeStereographic, LensParam::kDualFisheyeOrthographic };
  for (LensParam::LensType t : kTypes) {
    ExpectForwardAgrees(MakeView(t, 180.0f, 128, 64, RenderConfig::kFull, /*el=*/0.0f), -85.0f, 85.0f, "dual fisheye");
  }
}

TEST(AnnotationOverlayGuiParity, ProjectionAgreesForGlobe) {
  ExpectForwardAgrees(MakeView(LensParam::kGlobe, 60.0f, 96, 96, RenderConfig::kFull, /*el=*/30.0f, /*az=*/25.0f),
                      -85.0f, 85.0f, "globe");
}

TEST(AnnotationOverlayGuiParity, ProjectionAgreesForRectangularWithoutCameraAzimuth) {
  ExpectForwardAgrees(MakeView(LensParam::kRectangular, 180.0f, 128, 64, RenderConfig::kFull, /*el=*/0.0f), -85.0f,
                      85.0f, "rectangular");
}

// KNOWN DIVERGENCE (pre-existing): equirectangular is the one lens where core honours the camera
// azimuth and the GUI does not. ComputeScaleAz0 folds it into ProjParams::az0, so core's
// equirectangular canvas re-centres as the view turns; the GUI classifies rectangular as full-sky
// (LensIsFullSky) and applies no view transform at all, so its canvas is pinned to world azimuth.
// The two therefore differ by a horizontal SHIFT of az0 * scale pixels, which at a whole-turn
// canvas is most of the frame. Pinned as a shift rather than asserted away: nothing in this task
// changes it, and a future reconciliation should start by failing this test on purpose.
TEST(AnnotationOverlayGuiParity, RectangularDivergesUnderCameraAzimuth) {
  const ann::ViewSnapshot view =
      MakeView(LensParam::kRectangular, 180.0f, 128, 64, RenderConfig::kFull, /*el=*/0.0f, /*az=*/60.0f);
  const ProjDiff d = SweepDirections(view, -85.0f, 85.0f);
  EXPECT_GT(d.compared, 0);
  EXPECT_GT(d.mismatched, 0) << "core and the GUI now agree about rectangular under a camera azimuth; "
                                "if that is deliberate, delete this test and fold the case into "
                                "ProjectionAgreesForRectangularWithoutCameraAzimuth";
  // The disagreement is HORIZONTAL only: latitude is unaffected by az0, so every sample must keep
  // its row. A vertical component would mean something other than the az0 shift is loose.
  float view_matrix[9];
  lumice::gui::BuildViewMatrix(view.el_deg, view.az_deg, view.roll_deg, view_matrix);
  const lm_proj::ProjParams p = CoreForwardParams(view);
  int rows_compared = 0;
  for (int ai = 0; ai <= 24; ++ai) {
    const float alt = -85.0f + 170.0f * static_cast<float>(ai) / 24.0f;
    for (int zi = 0; zi < 36; ++zi) {
      float w[3];
      DirFromAltAz(alt, -180.0f + 10.0f * static_cast<float>(zi), w);
      const ann::CanvasPoint c = ann::ProjectWorldDir(p, w[0], w[1], w[2]);
      const GuiPoint g = GuiForward(view, view_matrix, w[0], w[1], w[2]);
      const CanvasHit ch = ToCanvasHit(view, c.px, c.py, c.valid);
      const CanvasHit gh = ToCanvasHit(view, g.px, g.py, g.valid);
      if (!ch.drawn || !gh.drawn || NearCanvasBorder(view, ch.px, ch.py)) {
        continue;
      }
      ++rows_compared;
      EXPECT_NEAR(ch.py, gh.py, kPixelTol) << "the rectangular divergence acquired a vertical component";
    }
  }
  EXPECT_GT(rows_compared, 0);
}

TEST(AnnotationOverlayGuiParity, VisibleRangeIsNotAppliedByTheAnnotationForward) {
  // The annotation layer applies the hemisphere policy itself (uniformly, for all eleven lens
  // types); the forward must therefore stay blind to it, or the five single-lens types would be
  // culled twice and the other six once. Same lattice, three visible settings, identical results.
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    ExpectForwardAgrees(MakeView(LensParam::kLinear, 90.0f, 96, 72, vis, /*el=*/20.0f), -80.0f, 80.0f,
                        "linear under a visible restriction");
  }
}
