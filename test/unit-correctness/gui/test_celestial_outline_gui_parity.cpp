// Cross-implementation gate: where core draws the celestial horizon (RenderConfig's
// `grid.outline`) versus where the GUI preview draws its own horizon overlay, over the same
// config, pixel by pixel.
//
// Sibling of test_visible_mask_gui_parity.cpp and built on the same footing: the two sides are
// genuinely separate implementations of the inverse lens math (GLSL cannot include a C++ header
// and src/gui/ may not include core/), and the GUI side is read out of production code —
// detail::PixelToWorldDirForTesting (overlay_labels.cpp), the CPU mirror of the fragment shader's
// inverse — rather than transcribed from the GLSL for this file.
//
// WHAT THIS COMPARES, AND WHAT IT DELIBERATELY DOES NOT
//
// The horizon annotation is two things composed: WHERE each pixel looks (the inverse projection,
// one implementation per side) and HOW WIDE the line is around altitude = 0 (the shader's
// `clamp(fwidth(altitude_deg), 1e-4, 2.0) * 1.5` band). Only the first of those is duplicated in
// this repo. `fwidth` is a rasterizer primitive with no C++ counterpart, so BOTH sides of this
// test necessarily run the same CPU restatement of the width rule —
// mask_detail::HorizonLineFromAltitudeField, which core's BuildCelestialOutlineMask itself calls.
// Feeding both altitude fields through the one shared rule is what makes this test about the
// question that HAS two answers. What it therefore does not cover is a divergence between the
// shader's own fwidth and this CPU forward difference; that is a line-thickness question, and AC6
// asks about position.
//
// The divergence this file inherits: for the four SINGLE-lens fisheye types core's domain ends at
// the equator and the GUI's at its asin guard (see the sibling file's header for why neither is a
// bug). Wherever that annulus contains horizon pixels, the GUI has line and core does not — so
// those types are asserted as a SUBSET relation, and the two-way assertion is reserved for the
// lens types the sibling file already established have identical domains.
//
// WHY THE COMPARISON IS NOT PIXEL-SET EQUALITY, AND WHAT REPLACES IT
//
// It cannot be, and the reason is measured rather than assumed. The two inverses agree on altitude
// to ~2e-6 degrees (float noise; the raw z components differ by ~4e-8). The line's edge is a
// threshold on that same quantity, so a pixel whose |altitude| lands within 2e-6 deg of its own
// half-width is a coin flip — decided independently on each side, by noise far below anything
// either side means. Measured instance: equal-area at fov 60 on a 128x96 frame puts pixel (63,46)
// at |alt| = 0.9268317 against a half-width of 0.9268... , and the flip costs exactly one pixel per
// line end. Demanding set equality would make this file fail on arithmetic, not on behaviour.
//
// So the comparison is stated in two parts, neither of which has a knife edge:
//   (1) the ALTITUDE FIELDS agree to a tolerance far tighter than a pixel — this is AC6's actual
//       content ("the two put the horizon in the same place"), and it holds continuously rather
//       than only after thresholding;
//   (2) the two thresholded LINES COINCIDE WITHIN ONE PIXEL — every pixel one side marks is marked
//       by the other or 4-adjacent to one it marks, and the pixels needing the adjacency clause
//       stay a small fraction of the line. A line shifted by a row, or drawn from a different
//       predicate, fails (2); a coin flip at the line's edge does not.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
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
  cfg.overlap_ = 0.0f;  // same reason as the sibling file: the GUI's inverse ignores r_scale
  cfg.celestial_outline_ = true;
  return cfg;
}

std::vector<uint8_t> CoreOutline(const RenderConfig& cfg) {
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return lumice::BuildCelestialOutlineMask(cfg, lumice::MakeCameraRotation(cfg), short_pix);
}

// The shader's visibility rule, restated exactly as the sibling file does (preview_renderer.cpp
// main(): lat = asin(clamp(-world_dir.z, -1, 1)), then the two half-sky rejections). The overlay
// is drawn under `result.w >= 0.5 && pixel_visible`, so the annotation inherits this gate.
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

// The GUI's horizon verdict for every pixel of the same frame, in the same row-major layout:
// its own inverse for the direction, then the SHARED width rule (see the header).
std::vector<uint8_t> GuiOutline(const RenderConfig& cfg) {
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  float view_matrix[9];
  lumice::gui::BuildViewMatrix(cfg.view_.el_, cfg.view_.az_, cfg.view_.ro_, view_matrix);

  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  std::vector<float> alt_deg(n, 0.0f);
  std::vector<uint8_t> imaged(n, 0);
  std::vector<uint8_t> drawable(n, 0);
  for (int py = 0; py < h; ++py) {
    for (int px = 0; px < w; ++px) {
      // Shader convention: pixel offset from the viewport centre, y-UP.
      const float sx = static_cast<float>(px) + 0.5f - static_cast<float>(w) / 2.0f;
      const float sy = -(static_cast<float>(py) + 0.5f - static_cast<float>(h) / 2.0f);
      float dx = 0.0f;
      float dy = 0.0f;
      float dz = 0.0f;
      bool ok = false;
      lumice::gui::detail::PixelToWorldDirForTesting(sx, sy, static_cast<float>(w), static_cast<float>(h),
                                                     static_cast<int>(cfg.lens_.type_), cfg.lens_.fov_, view_matrix,
                                                     &dx, &dy, &dz, &ok);
      if (!ok) {
        continue;
      }
      const size_t i = static_cast<size_t>(py) * static_cast<size_t>(w) + static_cast<size_t>(px);
      alt_deg[i] = lumice::mask_detail::AltitudeDeg({ dx, dy, dz, true });
      imaged[i] = 1;
      drawable[i] = VisibleInGuiTerms(cfg.visible_, dz) ? 1 : 0;
    }
  }
  return lumice::mask_detail::HorizonLineFromAltitudeField(alt_deg, imaged, drawable, w, h);
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

// Part (1) of the comparison (see the header): the two altitude fields, wherever both sides image
// the pixel at all. Tolerance is 1e-3 deg — three orders above the ~2e-6 deg noise measured between
// the two inverses, and orders BELOW the coarsest degrees-per-pixel any fixture here produces, so
// it cannot absorb a positional disagreement of even a hundredth of a pixel.
constexpr float kAltToleranceDeg = 1e-3f;

void ExpectAltitudeFieldsAgree(const RenderConfig& cfg, const char* label) {
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const float short_pix = static_cast<float>(std::min(w, h));
  const auto rot = lumice::MakeCameraRotation(cfg);
  const auto pp = lumice::BuildProjParams(cfg, rot, short_pix);
  float view_matrix[9];
  lumice::gui::BuildViewMatrix(cfg.view_.el_, cfg.view_.az_, cfg.view_.ro_, view_matrix);

  float worst = 0.0f;
  size_t worst_i = 0;
  size_t compared = 0;
  for (int py = 0; py < h; ++py) {
    for (int px = 0; px < w; ++px) {
      const auto d = lumice::mask_detail::PixelToWorld(cfg, pp, rot, px, py);
      const float sx = static_cast<float>(px) + 0.5f - static_cast<float>(w) / 2.0f;
      const float sy = -(static_cast<float>(py) + 0.5f - static_cast<float>(h) / 2.0f);
      float gx = 0.0f;
      float gy = 0.0f;
      float gz = 0.0f;
      bool ok = false;
      lumice::gui::detail::PixelToWorldDirForTesting(sx, sy, static_cast<float>(w), static_cast<float>(h),
                                                     static_cast<int>(cfg.lens_.type_), cfg.lens_.fov_, view_matrix,
                                                     &gx, &gy, &gz, &ok);
      if (!d.valid || !ok) {
        continue;
      }
      ++compared;
      const float diff =
          std::fabs(lumice::mask_detail::AltitudeDeg(d) - lumice::mask_detail::AltitudeDeg({ gx, gy, gz, true }));
      if (diff > worst) {
        worst = diff;
        worst_i = static_cast<size_t>(py) * static_cast<size_t>(w) + static_cast<size_t>(px);
      }
    }
  }
  EXPECT_GT(compared, 0u) << label << ": no pixel imaged by both sides — the comparison is vacuous";
  EXPECT_LT(worst, kAltToleranceDeg) << label << ": the two inverses disagree about altitude by " << worst << " deg at "
                                     << PixelLabel(cfg, worst_i);
}

// Part (2): the two lines coincide within one pixel. `strict` names the direction that must hold
// even where the domains differ — for the single-fisheye family only core-must-be-covered is
// meaningful, since the GUI images an annulus core never reaches.
struct Coincidence {
  size_t core_uncovered = 0;  // core marks it, the GUI marks nothing within 4-adjacency
  size_t gui_uncovered = 0;
  size_t core_needing_adjacency = 0;  // marked by core, not by the GUI, but adjacent to a GUI mark
  std::string first_core_uncovered;
  std::string first_gui_uncovered;
};

bool MarkedNear(const std::vector<uint8_t>& m, int w, int h, int px, int py) {
  const int dx[5] = { 0, 1, -1, 0, 0 };
  const int dy[5] = { 0, 0, 0, 1, -1 };
  for (int k = 0; k < 5; ++k) {
    const int x = px + dx[k];
    const int y = py + dy[k];
    if (x >= 0 && x < w && y >= 0 && y < h &&
        m[static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)] != 0) {
      return true;
    }
  }
  return false;
}

Coincidence CompareLines(const std::vector<uint8_t>& core, const std::vector<uint8_t>& gui, const RenderConfig& cfg) {
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  Coincidence c;
  for (int py = 0; py < h; ++py) {
    for (int px = 0; px < w; ++px) {
      const size_t i = static_cast<size_t>(py) * static_cast<size_t>(w) + static_cast<size_t>(px);
      if (core[i] != 0 && gui[i] == 0) {
        if (MarkedNear(gui, w, h, px, py)) {
          ++c.core_needing_adjacency;
        } else if (c.core_uncovered++ == 0) {
          c.first_core_uncovered = PixelLabel(cfg, i);
        }
      }
      if (gui[i] != 0 && core[i] == 0 && !MarkedNear(core, w, h, px, py) && c.gui_uncovered++ == 0) {
        c.first_gui_uncovered = PixelLabel(cfg, i);
      }
    }
  }
  return c;
}

// Both directions: for the lens types whose domains the sibling file established are identical.
void ExpectLinesCoincide(const RenderConfig& cfg, const char* label) {
  const auto core = CoreOutline(cfg);
  const auto gui = GuiOutline(cfg);
  const size_t marked = CountOn(core);
  ASSERT_GT(marked, 0u) << label << ": no horizon in frame makes the comparison vacuous";
  const Coincidence c = CompareLines(core, gui, cfg);
  EXPECT_EQ(c.core_uncovered, 0u) << label << ": core draws horizon on " << c.core_uncovered
                                  << " pixel(s) with no GUI horizon anywhere adjacent; first "
                                  << c.first_core_uncovered;
  EXPECT_EQ(c.gui_uncovered, 0u) << label << ": the GUI draws horizon on " << c.gui_uncovered
                                 << " pixel(s) with no core horizon anywhere adjacent; first " << c.first_gui_uncovered;
  // The adjacency clause exists for the threshold coin flip at the line's two ends, which costs a
  // handful of pixels. If it ever starts carrying a large share of the line, the two sides have
  // drifted by about a pixel and this file would otherwise keep passing.
  EXPECT_LE(c.core_needing_adjacency * 4u, marked)
      << label << ": " << c.core_needing_adjacency << " of " << marked
      << " core horizon pixels need the one-pixel adjacency clause — that is a drift, not a coin flip";
  ExpectAltitudeFieldsAgree(cfg, label);
}

constexpr RenderConfig::VisibleRange kAllRanges[] = { RenderConfig::kUpper, RenderConfig::kLower, RenderConfig::kFull };

// =================================================================================================
// Lens types whose domains the sibling file established are identical: the horizon must match too
// =================================================================================================

TEST(CelestialOutlineGuiParity, LinearAgreesExactly) {
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    // el = 0 puts the horizon across the middle of the frame for every `visible` setting.
    ExpectLinesCoincide(MakeCfg(LensParam::kLinear, 90.0f, 96, 72, vis, /*el=*/0.0f), "linear");
    ExpectLinesCoincide(MakeCfg(LensParam::kLinear, 40.0f, 96, 72, vis, /*el=*/10.0f), "linear narrow fov");
  }
}

TEST(CelestialOutlineGuiParity, RectangularAgreesExactly) {
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    ExpectLinesCoincide(MakeCfg(LensParam::kRectangular, 180.0f, 128, 64, vis, 0.0f), "rectangular 2:1");
    ExpectLinesCoincide(MakeCfg(LensParam::kRectangular, 180.0f, 96, 96, vis, 0.0f), "rectangular square");
  }
}

TEST(CelestialOutlineGuiParity, DualFisheyeAgreesExactlyForAllFourVariants) {
  const LensParam::LensType types[] = { LensParam::kDualFisheyeEqualArea, LensParam::kDualFisheyeEquidistant,
                                        LensParam::kDualFisheyeStereographic, LensParam::kDualFisheyeOrthographic };
  for (LensParam::LensType t : types) {
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      ExpectLinesCoincide(MakeCfg(t, 180.0f, 192, 96, vis, 0.0f), "dual fisheye");
    }
  }
}

TEST(CelestialOutlineGuiParity, GlobeAgreesExactly) {
  for (RenderConfig::VisibleRange vis : kAllRanges) {
    ExpectLinesCoincide(MakeCfg(LensParam::kGlobe, 30.0f, 128, 96, vis, /*el=*/5.0f, /*az=*/40.0f), "globe");
  }
}

// =================================================================================================
// The single-fisheye family: the inherited domain divergence, in horizon terms
// =================================================================================================

TEST(CelestialOutlineGuiParity, SingleFisheyeCoreIsASubsetOfTheGuiHorizon) {
  // At fov = 180 core's domain stops at the equator and the GUI's does not, so the GUI can carry
  // horizon pixels core never reaches. What must NOT happen is the other direction: a pixel core
  // annotates and the GUI does not would mean the two disagree about where altitude 0 IS, which is
  // a defect rather than the known domain gap.
  const LensParam::LensType types[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                        LensParam::kFisheyeStereographic, LensParam::kFisheyeOrthographic };
  for (LensParam::LensType t : types) {
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      const RenderConfig cfg = MakeCfg(t, 180.0f, 160, 120, vis, /*el=*/0.0f);
      const auto core = CoreOutline(cfg);
      const auto gui = GuiOutline(cfg);
      if (CountOn(core) == 0u) {
        ADD_FAILURE() << "type " << static_cast<int>(t) << ": no horizon in frame — fixture is vacuous";
        continue;
      }
      const Coincidence c = CompareLines(core, gui, cfg);
      EXPECT_EQ(c.core_uncovered, 0u) << "type " << static_cast<int>(t) << " visible " << static_cast<int>(vis)
                                      << ": core annotates " << c.core_uncovered
                                      << " pixel(s) with no GUI horizon adjacent; first " << c.first_core_uncovered;
      ExpectAltitudeFieldsAgree(cfg, "single fisheye fov=180");
    }
  }
}

TEST(CelestialOutlineGuiParity, SingleFisheyeAgreesExactlyWhenBothBoundariesAreOffFrame) {
  // Narrow FOV keeps both domain boundaries outside the frame, which removes the annulus and with
  // it the only reason the two sides may differ. Asserting equality here is what separates "the
  // subset above is the domain gap" from "the subset above hides a position disagreement".
  const LensParam::LensType types[] = { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant,
                                        LensParam::kFisheyeStereographic, LensParam::kFisheyeOrthographic };
  for (LensParam::LensType t : types) {
    for (RenderConfig::VisibleRange vis : kAllRanges) {
      ExpectLinesCoincide(MakeCfg(t, 60.0f, 128, 96, vis, /*el=*/0.0f), "single fisheye narrow fov");
    }
  }
}

}  // namespace
