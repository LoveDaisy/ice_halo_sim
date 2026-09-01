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
          (valid && VisibleInGuiTerms(cfg.visible_, dz)) ? 1 : 0;
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
  const float core = lumice::ComputeScaleAz0(t, fov, short_pix, 0, 0, lumice::Rotation{}).scale * CoreRadiusBound(t);
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

}  // namespace
