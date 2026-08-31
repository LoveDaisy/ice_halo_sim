// Analytic coverage for the celestial-horizon annotation mask
// (core/lens_proj_build.hpp::BuildCelestialOutlineMask).
//
// The mask answers "does pixel (px, py) sit on the line at altitude 0?". Its two halves fail
// differently and are pinned separately here:
//
//   position — WHERE the line lands. Checked against closed-form geometry stated in this file,
//              never against the mask's own arithmetic. The load-bearing fact is that a
//              rotationally symmetric lens whose optical axis lies IN the horizon plane images
//              that plane as the straight line through the principal point: every horizon
//              direction is at azimuthal angle 0 or pi around the axis, so its image is the
//              diameter, whatever the radial profile r(theta) is. That prediction is therefore
//              identical for `linear` and for all four single fisheyes, and it does not depend on
//              this repo's sign or handedness conventions — which is what makes it an oracle
//              rather than a restatement.
//   width    — HOW THICK the line is. The rule is the preview shader's `clamp(fwidth(alt), 1e-4,
//              2) * 1.5`, i.e. a band measured in LOCAL degrees-per-pixel, so the line stays a
//              couple of pixels wide across the whole lens/FOV space. A fixed angular half-width
//              would pass every position test in this file and still be wrong; the FOV sweep is
//              what separates them.
//
// The `visible` interaction gets its own case, and it is the sharpest one in the file. The
// horizon IS the boundary of the visible hemisphere, so under `visible: upper` every pixel of the
// line has its downward neighbour excluded from the sky. An implementation that measured the
// local gradient across the DRAWABLE pixels rather than the IMAGED ones would read that
// difference as zero, clamp the width to 1e-4 deg, and lose the line entirely — precisely in the
// configuration that asks for it. The upper/lower pair below fails under that defect and passes
// under the shipped one.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/lens_proj_build.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation

namespace lumice {
namespace {

RenderConfig MakeCfg(LensParam::LensType type, float fov, int w, int h,
                     RenderConfig::VisibleRange vis = RenderConfig::kFull, float el = 0.0f, float ro = 0.0f) {
  RenderConfig cfg;
  cfg.lens_.type_ = type;
  cfg.lens_.fov_ = fov;
  cfg.resolution_[0] = w;
  cfg.resolution_[1] = h;
  cfg.view_.el_ = el;
  cfg.view_.ro_ = ro;
  cfg.visible_ = vis;
  return cfg;
}

std::vector<uint8_t> Outline(const RenderConfig& cfg) {
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return BuildCelestialOutlineMask(cfg, MakeCameraRotation(cfg), short_pix);
}

bool At(const std::vector<uint8_t>& mask, const RenderConfig& cfg, int px, int py) {
  return mask[static_cast<size_t>(py) * static_cast<size_t>(cfg.resolution_[0]) + static_cast<size_t>(px)] != 0;
}

size_t CountOn(const std::vector<uint8_t>& mask) {
  return static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
}

// A whole-frame sweep compares thousands of pixels, so a fatal assert inside the loop would
// report the first row and hide the SHAPE of the failure (and every later case in the same test).
// Disagreements are accumulated and reported once, with the first one spelled out.
struct Mismatches {
  size_t count = 0;
  std::string first;

  void Note(const std::string& what) {
    if (count++ == 0) {
      first = what;
    }
  }
};

// Marked pixels in one column.
size_t ColumnCount(const std::vector<uint8_t>& mask, const RenderConfig& cfg, int px) {
  size_t n = 0;
  for (int py = 0; py < cfg.resolution_[1]; py++) {
    if (At(mask, cfg, px, py)) {
      n++;
    }
  }
  return n;
}

// The two rows straddling the frame centre. Pixel py samples at py + 0.5 - h/2, so for an even
// height the horizon at altitude 0 falls exactly between rows h/2 - 1 and h/2, each half a pixel
// from it; both are inside any half-width the shader's rule can produce (its minimum is 1.5x the
// local degrees-per-pixel, i.e. 1.5 pixels), so both are asserted ON rather than one of them.
constexpr int kW = 128;
constexpr int kH = 96;
constexpr int kRowAbove = kH / 2 - 1;  // 47
constexpr int kRowBelow = kH / 2;      // 48

// Columns far enough from the frame edge that every lens type in the single-lens family still
// images them at the centre row (the fisheye image circle is inscribed in the short side).
constexpr int kColLo = kW / 2 - 20;
constexpr int kColHi = kW / 2 + 20;

const LensParam::LensType kSingleLensTypes[]{ LensParam::kLinear, LensParam::kFisheyeEqualArea,
                                              LensParam::kFisheyeEquidistant, LensParam::kFisheyeStereographic,
                                              LensParam::kFisheyeOrthographic };

const char* TypeName(LensParam::LensType t) {
  switch (t) {
    case LensParam::kLinear:
      return "linear";
    case LensParam::kFisheyeEqualArea:
      return "fisheye_equal_area";
    case LensParam::kFisheyeEquidistant:
      return "fisheye_equidistant";
    case LensParam::kFisheyeStereographic:
      return "fisheye_stereographic";
    case LensParam::kFisheyeOrthographic:
      return "fisheye_orthographic";
    case LensParam::kDualFisheyeEqualArea:
      return "dual_fisheye_equal_area";
    case LensParam::kDualFisheyeEquidistant:
      return "dual_fisheye_equidistant";
    case LensParam::kDualFisheyeStereographic:
      return "dual_fisheye_stereographic";
    case LensParam::kDualFisheyeOrthographic:
      return "dual_fisheye_orthographic";
    case LensParam::kRectangular:
      return "rectangular";
    case LensParam::kGlobe:
      return "globe";
  }
  return "?";
}

// ==================================================================================================
// Position — the axis-on-the-horizon prediction, which every rotationally symmetric lens shares.
// ==================================================================================================

TEST(CelestialOutlineMask, AxisOnTheHorizonImagesTheCentralRow) {
  for (LensParam::LensType type : kSingleLensTypes) {
    const RenderConfig cfg = MakeCfg(type, 90.0f, kW, kH);
    const std::vector<uint8_t> mask = Outline(cfg);
    if (mask.size() != static_cast<size_t>(kW) * static_cast<size_t>(kH)) {
      ADD_FAILURE() << TypeName(type) << ": mask must be sized to the frame";
      continue;
    }

    Mismatches missing;
    for (int px = kColLo; px <= kColHi; px++) {
      if (!At(mask, cfg, px, kRowAbove) || !At(mask, cfg, px, kRowBelow)) {
        missing.Note("column " + std::to_string(px));
      }
    }
    EXPECT_EQ(missing.count, 0u) << TypeName(type) << ": the two rows straddling the horizon must both be on; first "
                                 << missing.first;

    // ...and the line must not be a band. Nothing more than two rows away from the centre.
    Mismatches stray;
    for (int py = 0; py < kH; py++) {
      if (py >= kRowAbove - 1 && py <= kRowBelow + 1) {
        continue;
      }
      for (int px = 0; px < kW; px++) {
        if (At(mask, cfg, px, py)) {
          stray.Note("(" + std::to_string(px) + "," + std::to_string(py) + ")");
        }
      }
    }
    EXPECT_EQ(stray.count, 0u) << TypeName(type) << ": no pixel further than one row from the centre pair may be on; "
                               << "first " << stray.first;
  }
}

TEST(CelestialOutlineMask, RollTurnsTheHorizonVertical) {
  // Rolling the camera 90 deg about its axis rotates the image, and with it the line. The
  // prediction is the same diameter, now the vertical one — a mask that had hardcoded "rows near
  // the centre" instead of deriving the direction per pixel passes the previous case and fails
  // this one.
  const RenderConfig cfg = MakeCfg(LensParam::kLinear, 90.0f, kW, kH, RenderConfig::kFull, 0.0f, 90.0f);
  const std::vector<uint8_t> mask = Outline(cfg);
  const int col_left = kW / 2 - 1;
  const int col_right = kW / 2;

  Mismatches missing;
  for (int py = kH / 2 - 20; py <= kH / 2 + 20; py++) {
    if (!At(mask, cfg, col_left, py) || !At(mask, cfg, col_right, py)) {
      missing.Note("row " + std::to_string(py));
    }
  }
  EXPECT_EQ(missing.count, 0u) << "the two columns straddling the horizon must both be on; first " << missing.first;

  Mismatches stray;
  for (int px = 0; px < kW; px++) {
    if (px >= col_left - 1 && px <= col_right + 1) {
      continue;
    }
    for (int py = 0; py < kH; py++) {
      if (At(mask, cfg, px, py)) {
        stray.Note("(" + std::to_string(px) + "," + std::to_string(py) + ")");
      }
    }
  }
  EXPECT_EQ(stray.count, 0u) << "no pixel further than one column from the centre pair may be on; first "
                             << stray.first;
}

TEST(CelestialOutlineMask, HorizonOutsideTheFrameDrawsNothing) {
  // A 40 deg frame pointed 70 deg up cannot contain altitude 0. Anything drawn here is the line
  // appearing where it does not belong, which is the failure mode a too-wide fixed threshold has.
  const RenderConfig cfg = MakeCfg(LensParam::kLinear, 40.0f, kW, kH, RenderConfig::kFull, 70.0f);
  EXPECT_EQ(CountOn(Outline(cfg)), 0u);
}

TEST(CelestialOutlineMask, EquirectangularKeepsTheHorizonOnTheCentreRow) {
  // The equirectangular frame maps latitude linearly onto rows and is horizon-centred by
  // construction (RectangularPixelToWorld does not consult the camera rotation at all), so the
  // line is the centre row across the FULL width, elevation notwithstanding. That last clause is
  // asserted rather than assumed: it is the one lens whose annotation does not move with the view.
  constexpr int kRw = 256;
  constexpr int kRh = 128;
  for (float el : { 0.0f, 30.0f, -45.0f }) {
    const RenderConfig cfg = MakeCfg(LensParam::kRectangular, 90.0f, kRw, kRh, RenderConfig::kFull, el);
    const std::vector<uint8_t> mask = Outline(cfg);

    Mismatches missing;
    for (int px = 0; px < kRw; px++) {
      if (!At(mask, cfg, px, kRh / 2 - 1) || !At(mask, cfg, px, kRh / 2)) {
        missing.Note("column " + std::to_string(px));
      }
    }
    EXPECT_EQ(missing.count, 0u) << "el=" << el << ": the centre row pair must span the whole width; first "
                                 << missing.first;

    Mismatches stray;
    for (int py = 0; py < kRh; py++) {
      if (py >= kRh / 2 - 2 && py <= kRh / 2 + 1) {
        continue;
      }
      for (int px = 0; px < kRw; px++) {
        if (At(mask, cfg, px, py)) {
          stray.Note("(" + std::to_string(px) + "," + std::to_string(py) + ")");
        }
      }
    }
    EXPECT_EQ(stray.count, 0u) << "el=" << el << ": the line must stay within two rows of the centre; first "
                               << stray.first;
  }
}

// ==================================================================================================
// Width — the property a fixed angular threshold cannot have.
// ==================================================================================================

TEST(CelestialOutlineMask, LineWidthTracksDegreesPerPixelRatherThanAFixedAngle) {
  // Across this FOV span the degrees-per-pixel at the frame centre changes by more than an order
  // of magnitude. A fixed angular half-width would make the line a hairline at one end and a band
  // at the other; the shader's rule keeps it a couple of pixels at both.
  for (float fov : { 10.0f, 45.0f, 90.0f, 120.0f }) {
    const RenderConfig cfg = MakeCfg(LensParam::kLinear, fov, kW, kH);
    const std::vector<uint8_t> mask = Outline(cfg);
    const size_t n = ColumnCount(mask, cfg, kW / 2);
    EXPECT_GE(n, 1u) << "fov=" << fov << ": the horizon must be drawn at all";
    EXPECT_LE(n, 4u) << "fov=" << fov << ": the horizon must stay a line, not a band (" << n << " rows)";
  }
}

// ==================================================================================================
// The visible hemisphere — the case that fails if the width gradient is measured over the
// DRAWABLE pixels instead of the IMAGED ones.
// ==================================================================================================

TEST(CelestialOutlineMask, VisibleUpperKeepsTheLineAndClipsItsLowerHalf) {
  const RenderConfig cfg = MakeCfg(LensParam::kLinear, 90.0f, kW, kH, RenderConfig::kUpper);
  const std::vector<uint8_t> mask = Outline(cfg);
  ASSERT_GT(CountOn(mask), 0u) << "the horizon bounds the upper hemisphere; clipping to it must not erase the line";

  Mismatches wrong;
  for (int px = kColLo; px <= kColHi; px++) {
    if (!At(mask, cfg, px, kRowAbove)) {
      wrong.Note("column " + std::to_string(px) + " lost its sky-side pixel");
    }
    if (At(mask, cfg, px, kRowBelow)) {
      wrong.Note("column " + std::to_string(px) + " drew below the horizon");
    }
  }
  EXPECT_EQ(wrong.count, 0u) << "first " << wrong.first;
}

TEST(CelestialOutlineMask, VisibleLowerKeepsTheLineAndClipsItsUpperHalf) {
  const RenderConfig cfg = MakeCfg(LensParam::kLinear, 90.0f, kW, kH, RenderConfig::kLower);
  const std::vector<uint8_t> mask = Outline(cfg);
  ASSERT_GT(CountOn(mask), 0u);

  Mismatches wrong;
  for (int px = kColLo; px <= kColHi; px++) {
    if (!At(mask, cfg, px, kRowBelow)) {
      wrong.Note("column " + std::to_string(px) + " lost its sky-side pixel");
    }
    if (At(mask, cfg, px, kRowAbove)) {
      wrong.Note("column " + std::to_string(px) + " drew above the horizon");
    }
  }
  EXPECT_EQ(wrong.count, 0u) << "first " << wrong.first;
}

// ==================================================================================================
// The remaining lens families, and the degenerate input.
// ==================================================================================================

TEST(CelestialOutlineMask, DualFisheyeAndGlobeDrawAHorizonToo) {
  // No closed-form row prediction for these two (the dual layout splits the sphere across two
  // circles, and the globe images a sphere from OUTSIDE, so its horizon is the silhouette-bounded
  // equator, a curve). What is asserted is that the branch produces a LINE: non-empty, and small
  // compared with the frame — a mask that fell back to "everything imaged" or to "nothing" is the
  // failure this catches.
  struct Case {
    LensParam::LensType type;
    int w;
    int h;
    float fov;
  };
  const Case cases[]{ { LensParam::kDualFisheyeEqualArea, 256, 128, 180.0f },
                      { LensParam::kDualFisheyeEquidistant, 256, 128, 180.0f },
                      { LensParam::kGlobe, 128, 128, 60.0f } };
  for (const Case& c : cases) {
    const RenderConfig cfg = MakeCfg(c.type, c.fov, c.w, c.h);
    const std::vector<uint8_t> mask = Outline(cfg);
    const size_t on = CountOn(mask);
    EXPECT_GT(on, 0u) << TypeName(c.type) << ": a view containing the horizon must draw it";
    EXPECT_LT(on, static_cast<size_t>(8 * std::max(c.w, c.h)))
        << TypeName(c.type) << ": " << on << " pixels is a region, not a line";
  }
}

TEST(CelestialOutlineMask, DegenerateResolutionYieldsAnEmptyMask) {
  RenderConfig cfg = MakeCfg(LensParam::kLinear, 90.0f, kW, kH);
  cfg.resolution_[0] = 0;
  EXPECT_TRUE(Outline(cfg).empty());
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = 0;
  EXPECT_TRUE(Outline(cfg).empty());
}

}  // namespace
}  // namespace lumice
