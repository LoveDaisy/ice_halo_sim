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
//   2. Rectangular follows the camera pose on core's side and deliberately does not on the GUI's
//      (owner decision 2026-09-02): core consumes the full rotation like every other lens, the GUI
//      treats rectangular as full-sky and applies no view transform at all. The two therefore agree
//      exactly at a zero pose — which is the only pose the GUI ever renders this lens at, since
//      RenderPreviewPanel pins it there every frame. Stated by
//      RectangularFollowsTheCameraPoseAndTheGuiDeliberatelyDoesNot; this is a division of labour,
//      not an open question.
//   3. Dual fisheye with a non-zero overlap ring. core's r_scale changes the mapping; the GUI's
//      display dual-fisheye is fixed at 180 deg per hemisphere and ignores it. Held at overlap = 0
//      throughout, for the same reason test_visible_mask_gui_parity.cpp holds it there.
//
// PIXEL CONVENTIONS. Core answers a PIXEL INDEX in image space (x right, y down, origin top-left),
// binning with floor(v) about res/2; the GUI answers a CONTINUOUS offset from the viewport centre,
// y up. The two are not the same kind of number, so the comparison converts core's index to the
// continuous coordinate it stands for — the centre of that pixel, index + 0.5 — before measuring
// any distance. Comparing the bare index against a continuous coordinate would measure the
// truncation, not the projections: the residual would run to a full pixel with a systematic sign,
// and it did until 2026-09-02, when forward binning still carried an extra + 0.5 that made the
// index a ROUND of the continuous coordinate and hid the category error behind a symmetric +-0.5.
// With the centres compared, what is left is one quantisation step, and kPixelTol = 1.0 covers it
// with room for float noise; it is NOT a slack that would hide a projection difference, since
// every divergence in the list above is hemisphere-scale, not sub-pixel.

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
// resolution. Core answers a whole pixel while the GUI answers a continuous coordinate, so a
// direction landing exactly on the image-circle rim or a frame corner can quantise in on one side
// and out on the other. That is a binning artefact, not a projection difference, and a comparison
// whose unit is one pixel cannot resolve it. Samples in the band are counted and reported rather
// than silently dropped, and the caller asserts the band stays a rim.
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
      // + 0.5: core's answer is a pixel INDEX, and the continuous coordinate it stands for is that
      // pixel's centre. See the PIXEL CONVENTIONS note at the top of this file.
      const CanvasHit ch = ToCanvasHit(view, c.px + 0.5f, c.py + 0.5f, c.valid);
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

// The label layer's half of the deliberate split described at the top of this file. core's
// equirectangular canvas is oriented by the camera — all three angles, not just the azimuth it used
// to fold into a scalar — while the GUI classifies rectangular as full-sky (LensIsFullSky) and pins
// its canvas to world coordinates. Both are right for what they do: the GUI resamples a fixed
// all-sky texture at display time, and the poses where the two answers differ are poses the GUI
// never renders this lens at (RenderPreviewPanel zeroes them every frame).
//
// This test used to say the opposite in its closing sentence — "a future reconciliation should start
// by failing this test on purpose". That reconciliation happened, and it went the other way: core
// was widened to follow the whole pose rather than narrowed to match the GUI. What is asserted now
// is the SHAPE of the difference, which is what tells the two apart:
//   - azimuth only  → a horizontal shift, latitude untouched (a rotation about world z);
//   - elevation/roll → a difference with a vertical component, which the azimuth-only projection
//     was structurally incapable of producing. That is the half that would have been silently green
//     before 2026-09-02.
TEST(AnnotationOverlayGuiParity, RectangularFollowsTheCameraPoseAndTheGuiDeliberatelyDoesNot) {
  // Rows kept, as a function of the pose: does the disagreement stay on one line of latitude?
  struct RowVerdict {
    int compared = 0;
    int moved_rows = 0;
  };
  auto verdict = [](const ann::ViewSnapshot& view) {
    RowVerdict out;
    float view_matrix[9];
    lumice::gui::BuildViewMatrix(view.el_deg, view.az_deg, view.roll_deg, view_matrix);
    const lm_proj::ProjParams p = CoreForwardParams(view);
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
        ++out.compared;
        if (std::fabs(ch.py - gh.py) > kPixelTol) {
          ++out.moved_rows;
        }
      }
    }
    return out;
  };

  // Azimuth only: the two disagree, and the disagreement is purely horizontal.
  {
    const ann::ViewSnapshot view =
        MakeView(LensParam::kRectangular, 180.0f, 128, 64, RenderConfig::kFull, /*el=*/0.0f, /*az=*/60.0f);
    const ProjDiff d = SweepDirections(view, -85.0f, 85.0f);
    EXPECT_GT(d.compared, 0);
    EXPECT_GT(d.mismatched, 0) << "core and the GUI now agree about rectangular under a camera azimuth; if the GUI "
                                  "started applying the view transform, this file's premise changed";
    const RowVerdict rows = verdict(view);
    EXPECT_GT(rows.compared, 0);
    EXPECT_EQ(rows.moved_rows, 0) << "an azimuth-only pose turns the sky about world z, so every sample must keep "
                                     "its row; a vertical component means something other than the pose is loose";
  }

  // Elevation and roll: the disagreement acquires a vertical component. Asserted as a COUNT rather
  // than as "at least one sample moved", so that a projection which follows only part of the pose
  // (the pre-2026-09-02 shape being the extreme case, following none of it) cannot pass on a
  // handful of borderline rows.
  const struct {
    float el;
    float ro;
    const char* what;
  } kTilts[] = { { 30.0f, 0.0f, "elevation" }, { 0.0f, 45.0f, "roll" }, { 30.0f, 45.0f, "elevation and roll" } };
  for (const auto& t : kTilts) {
    const ann::ViewSnapshot view =
        MakeView(LensParam::kRectangular, 180.0f, 128, 64, RenderConfig::kFull, t.el, /*az=*/0.0f, t.ro);
    const RowVerdict rows = verdict(view);
    if (rows.compared == 0) {
      ADD_FAILURE() << t.what << ": no sample was drawn by both sides, so this row proves nothing";
      continue;
    }
    EXPECT_GT(rows.moved_rows, rows.compared / 4)
        << t.what << " (el=" << t.el << " ro=" << t.ro
        << "): core must lift the equirectangular map out of the horizon plane, so most samples change row ("
        << rows.moved_rows << " of " << rows.compared << " did)";
  }
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
// Label anchors: RETIRED IN FULL, and the retirement is the migration completing.
//
// This section held the assertion the migration was FOR — run core's walk and the GUI's own over
// the same view and compare them label by label, which is the evidence required before the GUI's
// copy is deleted. It retired in two steps, each one licensed by the comparison it had already
// made:
//
//   (1) The PARALLELS and MERIDIANS went first, when the preview and the export started reading
//       core's anchors for them. Two cases about the single-lens domain edge
//       (LabelAnchorsAgreeOnASingleFisheyeWhoseDomainEdgeIsOffCanvas and
//       SingleFisheyeLabelsDivergeOnlyPastTheEquatorRadius) went with them: both could only see
//       that edge through curves that cross it, and the horizon never does. The domain divergence
//       itself is NOT retired — OrthographicDivergesOnlyPastTheImageCircle above pins it directly
//       on the projection, where it lives, and test_visible_mask_gui_parity.cpp pins the same edge
//       for the mask.
//   (2) The HORIZON followed, and with it the last GUI-side curve walk (ComputeOverlayLabels).
//       Five cases stood here — an all-sky dual fisheye, the two hemisphere restrictions, a linear
//       lens with a 5 deg grid, and the front-hemisphere clip — and every one of them compared two
//       implementations of the same walk. There is one implementation now, so there is nothing
//       left for them to compare: a parity test whose second side is gone does not become a
//       weaker test, it becomes a test of core against itself.
//
// WHERE THE PROPOSITIONS WENT, so this is a move and not a deletion:
//   - "which numbers appear for this view, and where" — test/unit-correctness/gui/
//     test_overlay_labels.cpp, whose cases now drive the production chain
//     (AnnotationOverlayCache -> BuildHorizonLabelSet / BuildGridLabelSet -> AppendCurveLabels).
//     That includes the four placement-gap regression anchors.
//   - "an anchor for the N-degree circle really lies N degrees from the sun" —
//     test/unit-correctness/core/test_annotation_overlay.cpp
//     (AngularDistLabelsSitOnTheCircleTheyName), which is a stronger statement than agreement with
//     a walk that could have been wrong in the same way.
//   - "the two projections agree at all" — the mask and projection cases ABOVE in this file, which
//     never went through the label walk and are untouched. Those are what keep the GUI's shader
//     mirror and core's forward projection on top of each other now.
//
// =================================================================================================

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
