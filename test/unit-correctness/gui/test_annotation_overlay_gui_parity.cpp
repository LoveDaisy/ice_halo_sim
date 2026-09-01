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
#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/annotation_overlay.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "gui/annotation_overlay_cache.hpp"
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

// =================================================================================================
// Label anchors: core's migrated curve walk against the GUI's own.
//
// This is the assertion the migration was FOR: run the two walks over the same view and compare
// them label by label, which is the evidence required before the GUI's copy is deleted.
//
// WHAT IS LEFT TO COMPARE. The parallels and the meridians have since been deleted from the GUI —
// the preview and the export read core's anchors now — so those two families have one
// implementation and nothing to compare against; see the retirement note further down for where
// their propositions went. The horizon still has a GUI-side walk (process_altitude_curve, at
// altitude 0) and will until the rest of the annotation layer moves over, so the cases below run
// on it, over the same six projections they always did.
//
// The two walks sample the same curve at the same density and apply the same boundary/interior
// dispatch; what differs underneath is the forward projection, which the tests above have already
// shown to agree on the canvas. So a disagreement here is a disagreement about the ALGORITHM, not
// about the lens.
// =================================================================================================

namespace {

struct LabelCase {
  ann::ViewSnapshot view;
  // Not a curve to walk any more, but still an input: the horizon's label text is formatted to one
  // decimal or none depending on how fine the grid around it is, on both sides.
  float grid_step = 10.0f;
  bool horizon = true;
};

std::vector<lumice::gui::OverlayLabel> GuiLabels(const LabelCase& c) {
  lumice::gui::OverlayLabelInput in{};
  in.lens_type = static_cast<int>(c.view.lens_type);
  in.fov = c.view.fov_deg;
  in.elevation = c.view.el_deg;
  in.azimuth = c.view.az_deg;
  in.roll = c.view.roll_deg;
  in.visible = static_cast<int>(c.view.visible);
  in.front = c.view.front;
  in.show_horizon = c.horizon;
  in.horizon_alpha = 1.0f;
  in.grid_step = c.grid_step;
  std::vector<lumice::gui::OverlayLabel> out;
  // A (0, 0) origin with the canvas as the viewport makes the GUI's screen coordinates the same
  // image-space pixels core reports, so no second convention enters the comparison.
  lumice::gui::ComputeOverlayLabels(in, 0.0f, 0.0f, static_cast<float>(c.view.width), static_cast<float>(c.view.height),
                                    out);
  return out;
}

std::vector<ann::Label> CoreLabels(const LabelCase& c) {
  ann::Request req;
  req.view = c.view;
  req.horizon = c.horizon;
  return ann::ComputeOverlay(req).labels;
}

// One sample step of a walked curve is 1 degree, and both sides emit at a SAMPLE, so a single
// disagreement about whether the sample at the very edge of the canvas is inside costs one step.
// Near the rim of a wide-angle projection that is several pixels, which is why the anchor
// tolerance is stated as a fraction of the canvas rather than in absolute pixels: the quantity
// being bounded is "the same sample, or its neighbour", not "the same pixel".
float AnchorTolerance(const ann::ViewSnapshot& view) {
  return 0.03f * static_cast<float>(std::max(view.width, view.height));
}

void ExpectLabelsAgree(const LabelCase& c, const char* label) {
  const std::vector<lumice::gui::OverlayLabel> gui = GuiLabels(c);
  const std::vector<ann::Label> core = CoreLabels(c);
  ASSERT_FALSE(gui.empty()) << label << ": the GUI emitted no labels, the comparison is vacuous";
  ASSERT_EQ(core.size(), gui.size()) << label << ": core emitted " << core.size() << " label(s), the GUI " << gui.size()
                                     << ". The two walks visit the same curves in the same order, so a count "
                                        "difference is a difference in the boundary/interior dispatch, not in "
                                        "placement.";
  const float tol = AnchorTolerance(c.view);
  int off = 0;
  float worst = 0.0f;
  size_t worst_i = 0;
  // Non-fatal per label, and the worst offender reported once: a systematic shift would otherwise
  // print hundreds of identical lines, and a fatal assert would hide whether it is one label or
  // all of them.
  for (size_t i = 0; i < core.size(); ++i) {
    EXPECT_EQ(core[i].text, gui[i].text) << label << ": label " << i << " text";
    const float d = std::max(std::fabs(core[i].px - gui[i].screen_x), std::fabs(core[i].py - gui[i].screen_y));
    if (d > worst) {
      worst = d;
      worst_i = i;
    }
    if (d > tol) {
      ++off;
    }
  }
  EXPECT_EQ(off, 0) << label << ": " << off << " of " << core.size() << " anchors are more than " << tol
                    << " px apart; worst " << worst << " px at label " << worst_i << " (\"" << core[worst_i].text
                    << "\")";
}

}  // namespace

TEST(AnnotationOverlayGuiParity, LabelAnchorsAgreeOnAnAllSkyDualFisheye) {
  LabelCase c;
  c.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128, RenderConfig::kFull, /*el=*/0.0f);
  ExpectLabelsAgree(c, "dual fisheye all-sky");
}

TEST(AnnotationOverlayGuiParity, LabelAnchorsAgreeUnderAHemisphereRestriction) {
  for (RenderConfig::VisibleRange vis : { RenderConfig::kUpper, RenderConfig::kLower }) {
    LabelCase c;
    c.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128, vis, /*el=*/0.0f);
    ExpectLabelsAgree(c, "dual fisheye, half sky");
  }
}

// RETIRED with the grid half (see the note below LabelAnchorsAgreeOnALinearLens): two cases stood
// here, LabelAnchorsAgreeOnASingleFisheyeWhoseDomainEdgeIsOffCanvas and
// SingleFisheyeLabelsDivergeOnlyPastTheEquatorRadius. Both were about the SINGLE-LENS DOMAIN EDGE
// — core's fisheye branch stops at the equator (theta = 90 deg) where the GUI's runs to 180 —
// and both could only see it through the parallels and meridians, the curves that actually cross
// that edge. The first chose a view that keeps the edge off the canvas so the comparison is exact;
// the second chose one that brings it on, and pinned the resulting disagreement to labels lying
// OUTSIDE the equator radius. With the grid's GUI walk deleted the horizon is the only curve left
// to compare, and it never reaches that edge, so neither case has an input any more: the first
// went vacuous (its view puts the horizon off-canvas, so the GUI emits nothing at all) and the
// second read zero disagreements, which is its own "the divergence has disappeared" failure rather
// than evidence of anything.
//
// The domain divergence itself is NOT retired: OrthographicDivergesOnlyPastTheImageCircle above
// pins it directly on the projection, which is where it lives, and
// test_visible_mask_gui_parity.cpp pins the same edge for the mask.

TEST(AnnotationOverlayGuiParity, LabelAnchorsAgreeOnALinearLens) {
  LabelCase c;
  c.view = MakeView(LensParam::kLinear, 60.0f, 192, 144, RenderConfig::kFull, /*el=*/20.0f, /*az=*/-40.0f,
                    /*ro=*/8.0f);
  c.grid_step = 5.0f;
  ExpectLabelsAgree(c, "linear, 5 deg grid");
}

// RETIRED, not lost — twice over, and for the same reason both times: a parity test needs two
// implementations, and each of these families is now down to one.
//
// (1) There used to be a LabelAnchorsAgreeForAngularDistanceCircles here, comparing core's circle
// anchors against the GUI's own walk of the same rings. Its proposition — "an anchor for the
// N-degree circle actually lies N degrees from the sun" — is asserted directly against the
// geometry in test/unit-correctness/core/test_annotation_overlay.cpp
// (AngularDistLabelsSitOnTheCircleTheyName), which is a stronger statement than agreement with a
// walk that could have been wrong in the same way.
//
// (2) The cases below used to compare the PARALLELS and MERIDIANS too, not just the horizon; that
// is what the `grid` field on LabelCase selected. The GUI's walk of those two families has now been
// deleted — which is precisely the deletion this file was written to license, so its comparison
// ending here is the migration completing, not coverage quietly going missing. Their propositions
// stayed where they always were, in test/unit-correctness/gui/test_overlay_labels.cpp, which asks
// the same questions about which numbers appear and where; those cases were re-pointed at core's
// walk through the production chain (AnnotationOverlayCache -> BuildGridLabelSet ->
// AppendCurveLabels) rather than deleted along with the producer they used to call.
//
// The horizon keeps its cases below: it still has a GUI-side walk, and will until the rest of the
// annotation layer moves over.

TEST(AnnotationOverlayGuiParity, LabelAnchorsAgreeUnderTheFrontHemisphereClip) {
  LabelCase c;
  c.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128, RenderConfig::kFull, /*el=*/15.0f,
                    /*az=*/70.0f);
  c.view.front = true;
  ExpectLabelsAgree(c, "dual fisheye, front clip");
}

// =================================================================================================
// The zenith / nadir markers: the placement the GUI used to compute against the one it now reads.
//
// These two are not curves and carry no text, so they are the one annotation that never went
// through WorldDirToPixel above. Their GUI-side implementation was ProjectWorldDirToScreen — a
// THIRD copy of the forward projection, this one mirroring the fragment shader's inverse rather
// than the label walk — called on the two fixed world directions and handed to the shader as
// uniforms. app_panels.cpp now reads core's answer instead, so the same "the replacement is an
// equivalent, not a second approximation" argument this file exists to make has to be made for it.
//
// The comparison is made in the SHADER's space (centre origin, y up), because that is what both
// paths ultimately produce and therefore the only place a sign error in the canvas -> shader
// conversion is observable. Reading it in core's canvas space would test the projections while
// leaving the conversion — the part that is new — unexamined.
//
// Two conventions differ by construction and are covered by kPixelTol rather than corrected for:
// core answers an integer pixel INDEX (ProjectExitToPixel's binning) while the GUI answers a
// continuous offset, so the two sit up to about half a pixel apart even in perfect agreement.
//
// Every case holds `visible` at kFull with no front clip, deliberately: core's marker sampler
// applies the hemisphere policy (VisibleForLabel) and ProjectWorldDirToScreen does not — the
// shader discards those pixels in a later stage instead. Under a restriction the two would
// disagree about a marker in the excluded half for a reason that has nothing to do with the
// projection, which is the question these cases ask.
// =================================================================================================

namespace {

struct MarkerPos {
  float x = 0.0f;  // shader space: centre origin, x right, y UP
  float y = 0.0f;
  bool drawn = false;
};

MarkerPos FromShaderPair(const float p[2]) {
  if (p[0] == lumice::gui::kOverlaySentinel || p[1] == lumice::gui::kOverlaySentinel) {
    return {};
  }
  return { p[0], p[1], true };
}

// The path this task removed, kept here as an independent oracle: it is still compiled, still
// tested elsewhere, and shares no code with the replacement.
MarkerPos GuiMarker(const lumice::gui::ViewProjection& vp, const float dir[3], int w, int h) {
  const std::array<float, 2> p = lumice::gui::ProjectWorldDirToScreen(vp, dir, w, h);
  const float pair[2] = { p[0], p[1] };
  return FromShaderPair(pair);
}

// The path production takes now: core's overlay call, then the one canvas -> shader conversion.
void CoreMarkers(const lumice::gui::ViewProjection& vp, int w, int h, MarkerPos* zenith, MarkerPos* nadir) {
  lumice::gui::AnnotationViewInput in;
  in.lens_type = vp.lens_type;
  in.fov = vp.fov;
  in.azimuth = vp.azimuth;
  in.elevation = vp.elevation;
  in.roll = vp.roll;
  in.visible = vp.visible;
  in.front = vp.front;
  in.overlap = 0.0f;  // held at zero for the reason divergence 3 in this file's header gives
  in.zenith_nadir = true;
  lumice::gui::AnnotationOverlayCache cache;
  cache.Refresh(lumice::gui::MakeAnnotationViewKey(in, w, h));
  ASSERT_TRUE(cache.HasResult()) << "core produced no overlay for this view";
  float zp[2];
  float np[2];
  lumice::gui::CanvasPointToShaderScreenPos(cache.ZenithPoint(), w, h, zp);
  lumice::gui::CanvasPointToShaderScreenPos(cache.NadirPoint(), w, h, np);
  *zenith = FromShaderPair(zp);
  *nadir = FromShaderPair(np);
}

void ExpectMarkerAgrees(const MarkerPos& gui, const MarkerPos& core, const char* label) {
  ASSERT_EQ(gui.drawn, core.drawn) << label << ": the two paths disagree about whether the marker is on screen at all"
                                   << " (gui " << gui.drawn << ", core " << core.drawn << ")";
  if (!gui.drawn) {
    return;
  }
  EXPECT_NEAR(gui.x, core.x, kPixelTol) << label << ": x";
  EXPECT_NEAR(gui.y, core.y, kPixelTol) << label << ": y";
}

constexpr float kZenithWorldDir[3] = { 0.0f, 0.0f, -1.0f };
constexpr float kNadirWorldDir[3] = { 0.0f, 0.0f, 1.0f };

// Both markers of one view, both paths. `expect_zenith`/`expect_nadir` state which of the two this
// view images, so a case cannot pass by having both paths agree that nothing is drawn.
void ExpectMarkersAgree(const lumice::gui::ViewProjection& vp, int w, int h, bool expect_zenith, bool expect_nadir,
                        const char* label) {
  MarkerPos core_z;
  MarkerPos core_n;
  ASSERT_NO_FATAL_FAILURE(CoreMarkers(vp, w, h, &core_z, &core_n));
  EXPECT_EQ(core_z.drawn, expect_zenith) << label << ": the fixture does not image the zenith it claims to";
  EXPECT_EQ(core_n.drawn, expect_nadir) << label << ": the fixture does not image the nadir it claims to";
  ExpectMarkerAgrees(GuiMarker(vp, kZenithWorldDir, w, h), core_z, (std::string(label) + " zenith").c_str());
  ExpectMarkerAgrees(GuiMarker(vp, kNadirWorldDir, w, h), core_n, (std::string(label) + " nadir").c_str());
}

}  // namespace

TEST(AnnotationOverlayGuiParity, MarkerPlacementAgreesOnALinearLens) {
  lumice::gui::ViewProjection vp;
  vp.lens_type = lumice::gui::kLensTypeLinear;
  vp.fov = 90.0f;
  vp.elevation = 60.0f;
  ExpectMarkersAgree(vp, 96, 96, /*expect_zenith=*/true, /*expect_nadir=*/false, "linear at el 60");

  // Off-axis and rolled: the case that separates a correct conversion from one that happens to be
  // right on a centred, upright marker. A y-flip error is invisible at the canvas centre.
  vp.fov = 60.0f;
  vp.elevation = 70.0f;
  vp.azimuth = 30.0f;
  vp.roll = 15.0f;
  ExpectMarkersAgree(vp, 128, 96, /*expect_zenith=*/true, /*expect_nadir=*/false, "linear rolled at el 70");
}

TEST(AnnotationOverlayGuiParity, MarkerPlacementAgreesOnASingleFisheye) {
  lumice::gui::ViewProjection vp;
  vp.lens_type = lumice::gui::kLensTypeFisheyeEqualArea;
  vp.fov = 120.0f;
  vp.elevation = 45.0f;
  ExpectMarkersAgree(vp, 96, 96, /*expect_zenith=*/true, /*expect_nadir=*/false, "equal-area 120 at el 45");

  vp.lens_type = lumice::gui::kLensTypeFisheyeEquidist;
  vp.fov = 160.0f;
  vp.elevation = 30.0f;
  vp.roll = 20.0f;
  ExpectMarkersAgree(vp, 128, 128, /*expect_zenith=*/true, /*expect_nadir=*/false, "equidistant 160 rolled");
}

TEST(AnnotationOverlayGuiParity, MarkerPlacementAgreesOnAnAllSkyDualFisheye) {
  // The only lens family that images BOTH markers at once, so it is the only case where a
  // conversion that swapped the two would still pass everything above.
  lumice::gui::ViewProjection vp;
  vp.lens_type = lumice::gui::kLensTypeDualFisheyeEqualArea;
  vp.fov = 180.0f;
  ExpectMarkersAgree(vp, 192, 96, /*expect_zenith=*/true, /*expect_nadir=*/true, "dual fisheye all-sky");
}
