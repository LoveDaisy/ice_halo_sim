// Cross-implementation gate: the core render-domain mask vs the GUI's own projection-domain
// judgement, over the same config, pixel by pixel.
//
// The two sides are genuinely separate implementations of "what does this lens image?", and
// have to be: GLSL cannot include a C++ header and src/gui/ may not include core/ (the API
// boundary gate), so the preview shader carries its own inverse lens math. Nothing has ever
// compared them. This file is that comparison — for the mask half of it, which is the half that
// decides where the CLI paints its background and therefore where a CLI render and a GUI preview
// of the same config visibly agree or diverge.
//
// GUI SIDE ORACLE: detail::PixelToWorldDirForTesting (overlay_labels.cpp). That function is the
// CPU mirror of the fragment shader's inverse — each of its branches carries a "Synced with
// shader <fn> (preview_renderer.cpp line ~N-M)" reference and the identical domain guards — and
// it is production code (label placement and mouse interaction run through it), not a copy made
// for this test. Using it rather than a fresh transcription of the GLSL trades one risk for
// another, deliberately: a transcription cannot drift from the GLSL only as long as someone
// re-reads it, whereas the mirror is exercised by the running GUI and by the handedness guard in
// this same target. What it cannot catch is a divergence between the shader and its own mirror;
// that is what test_render_handedness_guard.cpp exists for.
//
// The `visible` half of the GUI judgement lives in the shader's main() rather than in the
// inverse, so it is restated here (VisibleInGuiTerms) — three lines, and pinned by
// VisibleRangeMatchesTheGuiRule below against the same rule spelled out from lat = asin(-wz).
//
// ============================ THE SINGLE-FISHEYE DOMAIN ========================================
// This file used to pin a DIVERGENCE here: for the single-lens fisheye types core's domain ended
// at the EQUATOR (theta = 90 deg) while the GUI's ended at its asin guard (theta = 180 deg), so
// core's mask was a strict subset of the GUI's and the difference was the annulus between the two
// boundaries. That gap is gone. `ProjectExitToPixel`'s `cz <= 0` cull is now taken per lens type
// rather than for the fisheye family as a whole, and `projection.cpp`'s `Fisheye*Inverse` domain
// guards were widened to match it, so core RENDERS the past-equator region the GUI was already
// re-projecting. The tests below therefore assert set EQUALITY for every lens type in the file.
//
// Two members of the family are equal for reasons worth stating, because "equal" does not mean
// the same thing for both:
//
//   ORTHOGRAPHIC was never divergent. r = sin(theta) peaks at the equator and aliases past it, so
//   its cull stays at `cz <= 0` and the GUI's own asin guard rejects at exactly the same circle.
//
//   STEREOGRAPHIC is equal in practice rather than by construction. The GUI has no guard at all
//   (r = tan(theta/2) is monotone, so its inverse accepts every pixel), while core caps theta at
//   179.5 deg — a NUMERICAL safety bound, not a visibility judgement, aligned with the fov ceiling
//   `render_config.cpp::MaxFov` already imposes on this lens. That cap sits at r = tan(89.75 deg)
//   = 229.18 image radii, so no frame at any usable resolution reaches it and the two masks are
//   identical over any real canvas. The fixture below asserts that off-frame-ness explicitly
//   rather than leaving it implied.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/projection.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "gui/overlay_labels.hpp"
#include "gui/preview_renderer.hpp"

namespace {

using lumice::LensParam;
using lumice::RenderConfig;

RenderConfig MakeCfg(LensParam::LensType type, float fov, int w, int h, RenderConfig::VisibleRange vis, float el,
                     float az = 0.0f, float ro = 0.0f) {
  RenderConfig cfg;
  cfg.lens_.type_ = type;
  cfg.lens_.fov_ = fov;
  cfg.resolution_[0] = w;
  cfg.resolution_[1] = h;
  cfg.view_.az_ = az;
  cfg.view_.el_ = el;
  cfg.view_.ro_ = ro;
  cfg.visible_ = vis;
  cfg.overlap_ = 0.0f;  // see the dual-fisheye note in GuiMask()
  return cfg;
}

std::vector<uint8_t> CoreMask(const RenderConfig& cfg) {
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return lumice::BuildVisibleMask(cfg, lumice::MakeCameraRotation(cfg), short_pix);
}

// The shader's own visibility rule, from preview_renderer.cpp's main():
//   lat = asin(clamp(-world_dir.z, -1, 1));
//   u_visible == 0 (upper) && lat < 0 -> not visible
//   u_visible == 1 (lower) && lat > 0 -> not visible
// The shader's front-hemisphere rule, from the same main():
//   u_front == 1 && dot(world_dir, u_view_matrix[2]) > 0 -> not visible
// u_view_matrix[2] is the matrix's third COLUMN, which BuildViewMatrix fills with MINUS the
// camera forward (out[6..8], and its comment says so) — hence "> 0 discards", where core's
// mask_detail::FrontVisible writes the same cut as ">= 0 keeps" against the forward itself. The
// two spellings agree on the seam as well as off it: `> 0` discards and `>= 0` keeps both hand
// dot == 0 to the visible side.
bool FrontInGuiTerms(bool front, const float view_matrix[9], float wx, float wy, float wz) {
  if (!front) {
    return true;
  }
  return !(wx * view_matrix[6] + wy * view_matrix[7] + wz * view_matrix[8] > 0.0f);
}

bool VisibleInGuiTerms(RenderConfig::VisibleRange vis, float wz) {
  const float lat = std::asin(std::clamp(-wz, -1.0f, 1.0f));
  if (vis == RenderConfig::kUpper && lat < 0.0f) {
    return false;
  }
  if (vis == RenderConfig::kLower && lat > 0.0f) {
    return false;
  }
  return true;
}

// The GUI's verdict for every pixel of the same frame, in the same row-major layout the core
// mask uses.
//
// `overlap_` is held at 0 throughout this file: the GUI's dualFisheyeInverse does not read
// u_r_scale at all — that uniform describes the SOURCE texture's layout, while the display
// dual-fisheye lens is fixed at 180 deg per hemisphere. With an overlap ring configured, core's
// inverse (which does use r_scale) recovers past-equator directions near the rim that the GUI
// does not, so the visibility half would differ there for a reason that has nothing to do with
// the mask. Comparing at overlap = 0 keeps this gate on the question it is asking.
std::vector<uint8_t> GuiMask(const RenderConfig& cfg) {
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  float view_matrix[9];
  lumice::gui::BuildViewMatrix(cfg.view_.el_, cfg.view_.az_, cfg.view_.ro_, view_matrix);

  std::vector<uint8_t> mask(static_cast<size_t>(w) * static_cast<size_t>(h), 0);
  for (int py = 0; py < h; ++py) {
    for (int px = 0; px < w; ++px) {
      // Shader convention: pixel offset from the viewport centre, y-UP.
      const float sx = static_cast<float>(px) + 0.5f - static_cast<float>(w) / 2.0f;
      const float sy = -(static_cast<float>(py) + 0.5f - static_cast<float>(h) / 2.0f);
      float dx = 0.0f;
      float dy = 0.0f;
      float dz = 0.0f;
      bool valid = false;
      lumice::gui::detail::PixelToWorldDirForTesting(sx, sy, static_cast<float>(w), static_cast<float>(h),
                                                     static_cast<int>(cfg.lens_.type_), cfg.lens_.fov_, view_matrix,
                                                     &dx, &dy, &dz, &valid);
      mask[static_cast<size_t>(py) * static_cast<size_t>(w) + static_cast<size_t>(px)] =
          (valid && VisibleInGuiTerms(cfg.visible_, dz) && FrontInGuiTerms(cfg.front_, view_matrix, dx, dy, dz)) ? 1 :
                                                                                                                   0;
    }
  }
  return mask;
}

size_t CountOn(const std::vector<uint8_t>& m) {
  return static_cast<size_t>(std::count(m.begin(), m.end(), uint8_t{ 1 }));
}

std::string PixelLabel(const RenderConfig& cfg, size_t i) {
  std::ostringstream os;
  os << "pixel (" << i % static_cast<size_t>(cfg.resolution_[0]) << "," << i / static_cast<size_t>(cfg.resolution_[0])
     << ")";
  return os.str();
}

// A whole-frame comparison must not stop at the first disagreement: the shape of the difference
// (a rim, an annulus, a hemisphere) is the diagnostic, and one row's fatal assert would hide it.
struct Diff {
  size_t core_only = 0;
  size_t gui_only = 0;
  std::string first_core_only;
  std::string first_gui_only;
};

Diff Compare(const RenderConfig& cfg) {
  const auto core = CoreMask(cfg);
  const auto gui = GuiMask(cfg);
  Diff d;
  for (size_t i = 0; i < core.size(); ++i) {
    if (core[i] != 0 && gui[i] == 0 && d.core_only++ == 0) {
      d.first_core_only = PixelLabel(cfg, i);
    }
    if (core[i] == 0 && gui[i] != 0 && d.gui_only++ == 0) {
      d.first_gui_only = PixelLabel(cfg, i);
    }
  }
  return d;
}

void ExpectIdentical(const RenderConfig& cfg, const char* label) {
  const Diff d = Compare(cfg);
  EXPECT_EQ(d.core_only, 0u) << label << ": core paints " << d.core_only
                             << " pixel(s) the GUI treats as outside its domain; first " << d.first_core_only;
  EXPECT_EQ(d.gui_only, 0u) << label << ": the GUI images " << d.gui_only << " pixel(s) the core mask excludes; first "
                            << d.first_gui_only;
  EXPECT_GT(CountOn(CoreMask(cfg)), 0u) << label << ": both masks empty makes the comparison vacuous";
}

constexpr RenderConfig::VisibleRange kAllRanges[] = { RenderConfig::kUpper, RenderConfig::kLower, RenderConfig::kFull };

// =================================================================================================
// Lens types where the two sides must agree exactly
// =================================================================================================

TEST(VisibleMaskGuiParity, LinearAgreesExactly) {
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    ExpectIdentical(MakeCfg(LensParam::kLinear, 90.0f, 96, 72, vis, /*el=*/20.0f), "linear");
  }
}

TEST(VisibleMaskGuiParity, RectangularAgreesExactlyIncludingThePolarCaps) {
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    // 2:1 canvas — no caps — and a square one, where both sides must carve out the same caps.
    ExpectIdentical(MakeCfg(LensParam::kRectangular, 180.0f, 128, 64, vis, 0.0f), "rectangular 2:1");
    ExpectIdentical(MakeCfg(LensParam::kRectangular, 180.0f, 96, 96, vis, 0.0f), "rectangular square");
  }
}

TEST(VisibleMaskGuiParity, DualFisheyeAgreesExactlyForAllFourVariants) {
  const LensParam::LensType types[] = { LensParam::kDualFisheyeEqualArea, LensParam::kDualFisheyeEquidistant,
                                        LensParam::kDualFisheyeStereographic, LensParam::kDualFisheyeOrthographic };
  for (LensParam::LensType t : types) {
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      ExpectIdentical(MakeCfg(t, 180.0f, 192, 96, vis, 0.0f), "dual fisheye");
    }
  }
}

TEST(VisibleMaskGuiParity, GlobeAgreesExactly) {
  // Globe is the lens this task had to add a C++ inverse for, and the one with a real invalid
  // region that is not a fisheye circle — so it is also the one where a fresh implementation
  // could most easily have disagreed with the shader and nobody would have noticed.
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    ExpectIdentical(MakeCfg(LensParam::kGlobe, 30.0f, 128, 96, vis, /*el=*/25.0f, /*az=*/40.0f), "globe");
  }
}

// =================================================================================================
// The single-fisheye family: the two domains must now coincide
// =================================================================================================

// Core's normalized-radius domain bound per single-fisheye type, i.e. the largest r that
// `projection.cpp`'s `Fisheye*Inverse` accepts at r_scale = 1. Each is the value of that type's
// radius formula at the largest theta core will render:
//   equal-area     sqrt(2) sin(theta/2)  at theta = 180 deg -> sqrt(2)
//   equidistant    theta / (pi/2)        at theta = 180 deg -> 2
//   stereographic  tan(theta/2)          at theta = 179.5 deg -> tan(89.75 deg)  (numerical cap)
//   orthographic   sin(theta)            at theta = 90 deg  -> 1  (aliases past the equator)
float CoreRadiusBound(LensParam::LensType t) {
  if (t == LensParam::kFisheyeEqualArea) {
    return std::sqrt(2.0f);
  }
  if (t == LensParam::kFisheyeEquidistant) {
    return 2.0f;
  }
  if (t == LensParam::kFisheyeStereographic) {
    return std::tan(0.5f * 179.5f * lumice::math::kDegreeToRad);
  }
  return 1.0f;  // kFisheyeOrthographic
}

// Radius, in pixels from the frame centre, where each side's domain ends. Core's is
// CoreRadiusBound(t) in its normalized coordinates, i.e. that many `scale` pixels. The GUI's is
// read off the guard in fisheyeInverse (img_radius * r_boundary), and is infinite for
// stereographic, which has no guard at all.
struct Boundaries {
  float core_px;
  float gui_px;  // infinity when the GUI never rejects
};

Boundaries BoundariesFor(LensParam::LensType t, float fov_deg, float short_pix) {
  const float fov = fov_deg * lumice::math::kDegreeToRad;
  const float half_fov = fov / 2.0f;
  const float img_radius = short_pix / 2.0f;
  const float core = lumice::ComputeLensScale(t, fov, short_pix, 0, 0) * CoreRadiusBound(t);
  float gui = std::numeric_limits<float>::infinity();
  if (t == LensParam::kFisheyeEqualArea) {
    gui = img_radius / std::sin(half_fov / 2.0f);
  } else if (t == LensParam::kFisheyeEquidistant) {
    gui = img_radius * lumice::math::kPi / half_fov;
  } else if (t == LensParam::kFisheyeOrthographic) {
    gui = img_radius / std::sin(half_fov);
  }
  return { core, gui };
}

TEST(VisibleMaskGuiParity, SingleFisheyeCoreDomainMatchesGuiDomain) {
  // fov = 180 is the regime that used to expose the divergence: core's old equator boundary fell
  // inside the frame while the GUI's ran past it. It is kept as the fixture for exactly that
  // reason — if the widening were reverted or applied to the wrong lens, this is where it shows.
  const LensParam::LensType types[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                        LensParam::kFisheyeStereographic };
  const int w = 160;
  const int h = 120;
  const float short_pix = 120.0f;
  // Frame half-diagonal for 160x120: the radius past which a boundary cannot be observed at all.
  const float half_diag = std::sqrt(80.0f * 80.0f + 60.0f * 60.0f);
  for (LensParam::LensType t : types) {
    const Boundaries b = BoundariesFor(t, 180.0f, short_pix);
    if (t == LensParam::kFisheyeStereographic) {
      // No construction-level equality to assert here: the GUI has no guard and core's is a
      // numerical cap. What makes the two masks equal is that the cap is unreachable — assert
      // that, rather than the tautology core_px < gui_px = infinity.
      EXPECT_GT(b.core_px, half_diag) << "stereographic: core's numerical cap must stay off-frame, or this "
                                         "fixture is comparing a bound the GUI does not have";
    } else {
      EXPECT_NEAR(b.core_px, b.gui_px, 1e-3f)
          << "type " << static_cast<int>(t) << ": the two domains are supposed to end on the same circle (core "
          << b.core_px << ", gui " << b.gui_px << ")";
    }
    // el = 90 (zenith) with `full`: every recovered direction passes the visibility rule, so the
    // comparison is about the DOMAIN and nothing else.
    ExpectIdentical(MakeCfg(t, 180.0f, w, h, RenderConfig::kFull, /*el=*/90.0f), "single fisheye fov=180");
  }

  // Non-vacuity: equal-area's shared boundary (sqrt(2) image radii = 84.85 px here) falls between
  // the frame's inscribed circle and its corners, so the frame really does contain pixels BOTH
  // sides reject. Without this, "identical" could be satisfied by two masks that are both
  // everywhere-on.
  const auto ea = CoreMask(MakeCfg(LensParam::kFisheyeEqualArea, 180.0f, w, h, RenderConfig::kFull, 90.0f));
  EXPECT_LT(CountOn(ea), ea.size()) << "equal-area at fov=180 must still reject the frame corners";
}

TEST(VisibleMaskGuiParity, SingleFisheyeAgreesOnTheWidenedRadialMapping) {
  // Domain equality alone would still pass if one side mapped radius to theta differently past
  // the old equator. Tilting the camera makes the `visible` cut a curve whose position depends on
  // theta(r), so comparing `upper` / `lower` over the WIDENED region tests the mapping itself.
  const LensParam::LensType types[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                        LensParam::kFisheyeStereographic };
  for (LensParam::LensType t : types) {
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      ExpectIdentical(MakeCfg(t, 180.0f, 160, 120, vis, /*el=*/20.0f), "single fisheye fov=180, tilted");
    }
  }
}

TEST(VisibleMaskGuiParity, OrthographicIsTheSingleFisheyeWhoseTwoBoundariesCoincide) {
  // Orthographic is the one member of the family whose two boundaries coincided even before the
  // widening, and the one that must NOT be widened. r = sin(theta) is not injective past the
  // equator (sin 120 deg == sin 60 deg), so the shader's `s = r * sin(half_fov) > 1` guard rejects
  // at exactly theta = 90 deg — the same place core's r <= 1 domain ends and the same place its
  // `cz <= 0` cull still stands. This test is what fails if a future change widens the family as
  // a whole instead of per type.
  const float short_pix = 120.0f;
  const Boundaries b = BoundariesFor(LensParam::kFisheyeOrthographic, 180.0f, short_pix);
  EXPECT_NEAR(b.core_px, b.gui_px, 1e-3f) << "orthographic's two boundaries are supposed to be the same circle";
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    // el = 0 for the same reason as the narrow-fov case below: pointed at the zenith, `lower`
    // would empty the mask and the comparison would be between two empty sets.
    ExpectIdentical(MakeCfg(LensParam::kFisheyeOrthographic, 180.0f, 160, 120, vis, /*el=*/0.0f),
                    "orthographic fov=180");
  }
}

TEST(VisibleMaskGuiParity, SingleFisheyeAgreesExactlyWhenBothBoundariesAreOffFrame) {
  // The divergence above is not a permanent state of disagreement: it needs the equator circle
  // to fall inside the frame. At a normal field of view neither boundary is reachable and the
  // two sides agree pixel for pixel, `visible` included.
  const LensParam::LensType types[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                        LensParam::kFisheyeStereographic, LensParam::kFisheyeOrthographic };
  for (LensParam::LensType t : types) {
    const Boundaries b = BoundariesFor(t, 60.0f, 120.0f);
    // Frame half-diagonal for 160x120.
    const float half_diag = std::sqrt(80.0f * 80.0f + 60.0f * 60.0f);
    if (!(b.core_px > half_diag)) {
      ADD_FAILURE() << "type " << static_cast<int>(t) << ": fixture must keep both boundaries off-frame (core "
                    << b.core_px << ", half-diagonal " << half_diag << ")";
      continue;
    }
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      // el = 20: high enough that the horizon crosses the frame as a CURVE, low enough that
      // `lower` still keeps a crescent. Both halves matter — see the note on
      // VisibleRangeMatchesTheGuiRule for why el = 0 would make this blind to the radial
      // mapping, and why an empty `lower` would make it vacuous.
      ExpectIdentical(MakeCfg(t, 60.0f, 160, 120, vis, /*el=*/20.0f), "narrow-fov single fisheye");
    }
  }
}

// =================================================================================================
// The visibility rule itself
// =================================================================================================

TEST(VisibleMaskGuiParity, VisibleRangeMatchesTheGuiRule) {
  // `upper` and `lower` must cut the same frame in the same place on both sides. Equal-area at a
  // narrow fov: the whole frame is inside BOTH domains, so any difference that shows up here is
  // the visibility rule and nothing else.
  //
  // el = 20 rather than 0, and that is load-bearing. Pointed exactly at the horizon the cut is
  // the frame's horizontal centre line whatever theta(r) each side computes — the sign of a
  // pixel's elevation then depends only on which half of the frame it is in, so the comparison
  // would pass even if one side's radial mapping were completely wrong. Tilted up, the horizon
  // is a curve whose position depends on theta(r), and the comparison becomes sensitive to it.
  const float kEl = 20.0f;
  const size_t full_on =
      CountOn(CoreMask(MakeCfg(LensParam::kFisheyeEqualArea, 60.0f, 160, 120, RenderConfig::kFull, kEl)));
  for (RenderConfig::VisibleRange vis : { RenderConfig::kUpper, RenderConfig::kLower }) {
    const RenderConfig cfg = MakeCfg(LensParam::kFisheyeEqualArea, 60.0f, 160, 120, vis, kEl);
    ExpectIdentical(cfg, "horizon-crossing fisheye");
    const size_t on = CountOn(CoreMask(cfg));
    EXPECT_GT(on, 0u);
    EXPECT_LT(on, full_on) << "the horizon must actually cut this frame, or the comparison is vacuous";
  }
}

// =================================================================================================
// The front-hemisphere clip
// =================================================================================================

TEST(VisibleMaskGuiParity, FrontClipMatchesTheGuiRule) {
  // The clip has to land on the same pixels on both sides, and the two sides compute it from
  // DIFFERENT objects: core rotates (0,0,1) through its own Rotation chain and negates, the GUI
  // reads column 2 of a matrix assembled from az/el/roll in closed form. That the two are the same
  // vector is exactly what is unproven until compared.
  //
  // Only the lens types whose recovered DIRECTIONS already agree (see THE KNOWN DIVERGENCE at the
  // top, and RectangularRecentresOnTheViewAzimuthAndTheGuiDoesNot below): on a single-lens fisheye
  // the domains differ by an annulus, and on rectangular the two inverses disagree about azimuth.
  // Either would swamp this comparison with a disagreement that is not about `front`. The globe
  // agrees but is empty under the clip, so it gets its own test below rather than a vacuous row
  // here.
  const LensParam::LensType types[] = { LensParam::kLinear, LensParam::kDualFisheyeEqualArea };
  // Off-axis on all three angles: a clip derived from the wrong column, or from a matrix that
  // ignores roll, survives an axis-aligned view and dies here.
  for (LensParam::LensType t : types) {
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      RenderConfig cfg = MakeCfg(t, t == LensParam::kLinear ? 90.0f : 180.0f, 128, 96, vis, /*el=*/25.0f,
                                 /*az=*/40.0f, /*ro=*/15.0f);
      cfg.front_ = true;
      ExpectIdentical(cfg, "front clip");

      // ...and the clip must actually be cutting something on the type that can see behind the
      // camera, or the agreement above is vacuous. Linear at 90 deg fov sees only forward, so its
      // no-op is the point of the check above rather than a gap here.
      if (t != LensParam::kLinear) {
        RenderConfig unclipped = cfg;
        unclipped.front_ = false;
        EXPECT_LT(CountOn(CoreMask(cfg)), CountOn(CoreMask(unclipped)))
            << "type " << static_cast<int>(t) << ": the front clip removed nothing";
      }
    }
  }
}

// The globe is the one lens for which `front` is not a crop but a blackout, and BOTH sides do it:
// globeInverse returns the direction from the sphere's centre to the point on the near side of the
// silhouette, which by construction points back TOWARD the camera. Every imaged pixel therefore
// fails a "keep what the camera faces" test. That is a consequence of what the globe lens is — a
// view of the sky from outside — not a bug in either implementation, and it predates the clip
// gaining a core field: the preview shader has applied u_front to the globe branch all along.
// Pinned rather than left implicit so that a future change which makes the globe survive the clip
// has to say so out loud.
TEST(VisibleMaskGuiParity, FrontClipEmptiesTheGlobeOnBothSides) {
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    RenderConfig cfg = MakeCfg(LensParam::kGlobe, 30.0f, 128, 96, vis, /*el=*/25.0f, /*az=*/40.0f);
    if (CountOn(CoreMask(cfg)) == 0u || CountOn(GuiMask(cfg)) == 0u) {
      // Non-fatal: one vacuous visible range must not hide the other two.
      ADD_FAILURE() << "visible " << static_cast<int>(vis)
                    << ": the unclipped globe images nothing, so this row proves nothing";
      continue;
    }
    cfg.front_ = true;
    EXPECT_EQ(CountOn(CoreMask(cfg)), 0u);
    EXPECT_EQ(CountOn(GuiMask(cfg)), 0u);
  }
}

// A DELIBERATE division of labour between the two sides, stated here so nobody reads it as a
// divergence waiting to be fixed. Owner decision, 2026-09-02:
//
//   core keeps maximum flexibility — the equirectangular map follows the camera's FULL pose
//   (azimuth AND elevation AND roll), the same rotation every other lens consumes. GUI keeps a
//   FIXED all-sky texture, because every pose transform on that side is the front end's job (the
//   preview shader resamples the texture; `needs_view_transform = false` for this lens, and
//   RenderPreviewPanel pins the pose of every full-sky lens to zero every frame — asserted in
//   test/gui/functional/test_view_display_controls.cpp).
//
// Which is why they can differ without either being wrong: the GUI never renders this lens at a
// non-zero pose at all, so the configurations where the two answers differ are configurations the
// GUI does not produce. What makes that safe rather than lucky is the FIRST block below — at a zero
// pose the two agree exactly, so on every document the GUI can actually build, core's answer IS the
// GUI's answer.
//
// This test used to read the other way round: it recorded "core recentres on the view azimuth and
// the GUI does not" as an unsettled difference nobody had adjudicated, and said so ("which side is
// right is a product question... this test states the divergence exactly"). The product question
// has been answered; what is left is to state the answer and to check that core really does follow
// all three angles, not just the one it used to fold into a scalar.
//
// Historical note worth keeping: nothing caught the azimuth half before the front clip arrived,
// because every comparison in this file read only wz — and a rotation about z leaves wz untouched.
// Elevation and roll do NOT leave it untouched, which is what the third block leans on.
TEST(VisibleMaskGuiParity, RectangularFollowsTheFullCameraPoseAndTheGuiIsAFixedTexture) {
  const int w = 128;
  const int h = 96;
  const int px = 8;
  const int py = 25;

  auto core_dir = [&](float az, float el, float ro) {
    const RenderConfig cfg = MakeCfg(LensParam::kRectangular, 180.0f, w, h, RenderConfig::kFull, el, az, ro);
    const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
    const lm_proj::ProjParams p = lumice::BuildProjParams(cfg, rot, static_cast<float>(std::min(w, h)));
    return lumice::mask_detail::PixelToWorld(cfg, p, rot, px, py);
  };
  auto gui_dir = [&](float az, float el, float ro) {
    float view_matrix[9];
    lumice::gui::BuildViewMatrix(el, az, ro, view_matrix);
    const float sx = static_cast<float>(px) + 0.5f - static_cast<float>(w) / 2.0f;
    const float sy = -(static_cast<float>(py) + 0.5f - static_cast<float>(h) / 2.0f);
    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;
    bool valid = false;
    lumice::gui::detail::PixelToWorldDirForTesting(sx, sy, static_cast<float>(w), static_cast<float>(h),
                                                   static_cast<int>(LensParam::kRectangular), 180.0f, view_matrix, &dx,
                                                   &dy, &dz, &valid);
    return std::array<float, 4>{ dx, dy, dz, valid ? 1.0f : 0.0f };
  };
  auto azimuth_deg = [](float x, float y) { return std::atan2(y, x) * 180.0f / 3.14159265358979323846f; };

  // 1. At the pose the GUI pins this lens to, the two agree completely. This is the load-bearing
  //    block: it is what makes the divergences below harmless rather than a bug the GUI hides.
  {
    const auto c = core_dir(0.0f, 0.0f, 0.0f);
    const auto g = gui_dir(0.0f, 0.0f, 0.0f);
    ASSERT_TRUE(c.valid);
    ASSERT_EQ(g[3], 1.0f);
    EXPECT_NEAR(c.x, g[0], 1e-5f);
    EXPECT_NEAR(c.y, g[1], 1e-5f);
    EXPECT_NEAR(c.z, g[2], 1e-5f);
  }

  // 2. Panning the camera turns core's sky and leaves the GUI's where it was, by exactly az. Still
  //    a pure rotation about world z, so wz agrees — the reason this went unnoticed for so long.
  for (float az : { 40.0f, -40.0f, 90.0f }) {
    const auto c = core_dir(az, 0.0f, 0.0f);
    const auto g = gui_dir(az, 0.0f, 0.0f);
    if (!c.valid || g[3] != 1.0f) {
      // Non-fatal: a pixel that stops being imaged at one azimuth must not hide the other two.
      ADD_FAILURE() << "az " << az << ": the probe pixel is no longer imaged by both sides";
      continue;
    }
    EXPECT_NEAR(c.z, g[2], 1e-5f) << "az " << az << ": an azimuth-only pose turns the sky about z, so wz must agree";
    float delta = azimuth_deg(c.x, c.y) - azimuth_deg(g[0], g[1]);
    while (delta > 180.0f) {
      delta -= 360.0f;
    }
    while (delta < -180.0f) {
      delta += 360.0f;
    }
    EXPECT_NEAR(delta, az, 1e-2f) << "az " << az << ": core is expected to lead the GUI by exactly the view azimuth";
  }

  // 3. Elevation and roll move core's sky too — this is the half that did not exist before
  //    2026-09-02, when core reduced the camera rotation to an azimuth scalar and a tilted or
  //    rolled camera produced a bit-identical frame. The GUI's texture stays put under all of them.
  const auto gui_at_zero = gui_dir(0.0f, 0.0f, 0.0f);
  ASSERT_EQ(gui_at_zero[3], 1.0f);
  const struct {
    float el;
    float ro;
    const char* what;
  } kTilts[] = { { 25.0f, 0.0f, "elevation" },
                 { -40.0f, 0.0f, "elevation" },
                 { 0.0f, 35.0f, "roll" },
                 { 0.0f, -70.0f, "roll" },
                 { 25.0f, 35.0f, "elevation and roll" } };
  for (const auto& t : kTilts) {
    const auto c = core_dir(0.0f, t.el, t.ro);
    const auto g = gui_dir(0.0f, t.el, t.ro);
    if (!c.valid || g[3] != 1.0f) {
      ADD_FAILURE() << t.what << " (el=" << t.el << " ro=" << t.ro << "): the probe pixel is no longer imaged";
      continue;
    }
    // The GUI is the fixed texture: same pixel, same world direction, whatever the pose says.
    EXPECT_NEAR(g[0], gui_at_zero[0], 1e-6f) << t.what << ": the GUI's all-sky texture must not move";
    EXPECT_NEAR(g[1], gui_at_zero[1], 1e-6f) << t.what << ": the GUI's all-sky texture must not move";
    EXPECT_NEAR(g[2], gui_at_zero[2], 1e-6f) << t.what << ": the GUI's all-sky texture must not move";
    // Core moved, and not by a rotation about z: wz itself changes, which an azimuth-only
    // projection could never produce.
    EXPECT_GT(std::fabs(c.z - g[2]), 1e-3f)
        << t.what << " (el=" << t.el << " ro=" << t.ro << "): core did not follow the camera out of the horizon plane";
  }
}

}  // namespace
