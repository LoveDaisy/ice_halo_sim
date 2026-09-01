// Analytic coverage for the render-domain mask (core/lens_proj_build.hpp::BuildVisibleMask)
// and for the GlobeInverse it rests on.
//
// The mask answers "does pixel (px, py) image a visible piece of sky?", which is the question
// lm_proj::ProjectExitToPixel — a forward projection — structurally cannot answer. Its two
// halves are tested separately here because they fail differently:
//
//   domain     — is the pixel inside what the lens images at all? Checked against closed-form
//                boundaries computed in this file from the projection definitions, not from
//                BuildVisibleMask's own arithmetic.
//   visibility — is the sky it images in the hemisphere `visible` admits? Checked by flipping
//                `visible` on ONE config and asserting the partition swaps.
//
// Every lens type is covered, including the two whose domain half is trivially true — linear
// always, and rectangular on a canvas 2:1 or wider. That those are all-ones is a conclusion
// (their inverses reject nothing reachable), so it is asserted rather than left untested; a
// future change that started rejecting there would otherwise pass silently.

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
#include "core/projection.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "core/shared/projection_shared.h"

namespace lumice {
namespace {

RenderConfig MakeCfg(LensParam::LensType type, float fov, int w, int h,
                     RenderConfig::VisibleRange vis = RenderConfig::kFull, float el = 90.0f) {
  RenderConfig cfg;
  cfg.lens_.type_ = type;
  cfg.lens_.fov_ = fov;
  cfg.resolution_[0] = w;
  cfg.resolution_[1] = h;
  cfg.view_.el_ = el;
  cfg.visible_ = vis;
  return cfg;
}

std::vector<uint8_t> Mask(const RenderConfig& cfg) {
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return BuildVisibleMask(cfg, MakeCameraRotation(cfg), short_pix);
}

bool At(const std::vector<uint8_t>& mask, const RenderConfig& cfg, int px, int py) {
  return mask[static_cast<size_t>(py) * static_cast<size_t>(cfg.resolution_[0]) + static_cast<size_t>(px)] != 0;
}

// A whole-frame sweep compares tens of thousands of pixels, so a fatal assert inside the loop
// would report row 0 and hide the shape of the failure (and, worse, hide every later case in the
// same test). Disagreements are accumulated instead and reported once, after the sweep, with the
// first one spelled out.
struct Mismatches {
  size_t count = 0;
  std::string first;

  void Note(const std::string& what) {
    if (count++ == 0) {
      first = what;
    }
  }
};

size_t CountOn(const std::vector<uint8_t>& mask) {
  return static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
}

// Distance of a pixel centre from the frame centre, in the same units BuildVisibleMask uses.
float RadiusPx(const RenderConfig& cfg, int px, int py) {
  const float dx = static_cast<float>(px) + 0.5f - static_cast<float>(cfg.resolution_[0]) / 2.0f;
  const float dy = static_cast<float>(py) + 0.5f - static_cast<float>(cfg.resolution_[1]) / 2.0f;
  return std::sqrt(dx * dx + dy * dy);
}

// ==================================================================================================
// GlobeInverse — the one inverse this task had to add, and the only lens whose domain is a
// ray-sphere discriminant rather than a radius bound.
// ==================================================================================================

// The sphere's silhouette: the ray from (0,0,-D) is tangent to the unit sphere when
// sin(half-angle) = 1/D, i.e. at screen radius focal * tan(asin(1/D)) = focal / sqrt(D^2 - 1).
float GlobeSilhouettePx(float focal) {
  const float d = lm_proj::kGlobeCameraD;
  return focal / std::sqrt(d * d - 1.0f);
}

TEST(GlobeInverse, InsideTheSilhouetteIsValidAndUnitLength) {
  constexpr float kFocal = 300.0f;
  const float edge = GlobeSilhouettePx(kFocal);
  for (float frac : { 0.0f, 0.25f, 0.5f, 0.9f, 0.99f }) {
    const projection::Dir3 c = projection::GlobeInverse(edge * frac, 0.0f, kFocal);
    if (!c.valid) {
      ADD_FAILURE() << "screen radius " << frac << " x silhouette must hit the sphere";
      continue;
    }
    EXPECT_NEAR(std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z), 1.0f, 1e-5f);
    // Camera sits at c.z = -D looking toward +z, so the near surface has c.z <= -1/D... the
    // visible cap is bounded by the tangency plane c.z = -1/D.
    EXPECT_LE(c.z, -1.0f / lm_proj::kGlobeCameraD + 1e-5f);
  }
}

TEST(GlobeInverse, OutsideTheSilhouetteMissesTheSphere) {
  constexpr float kFocal = 300.0f;
  const float edge = GlobeSilhouettePx(kFocal);
  for (float frac : { 1.01f, 1.5f, 4.0f }) {
    EXPECT_FALSE(projection::GlobeInverse(edge * frac, 0.0f, kFocal).valid)
        << "screen radius " << frac << " x silhouette must miss the sphere";
    EXPECT_FALSE(projection::GlobeInverse(0.0f, edge * frac, kFocal).valid);
  }
}

TEST(GlobeInverse, DegenerateFocalImagesNothing) {
  // fov -> 180 deg drives ComputeScaleAz0's globe branch (focal = r / tan(fov/2)) to zero.
  EXPECT_FALSE(projection::GlobeInverse(1.0f, 1.0f, 0.0f).valid);
  EXPECT_FALSE(projection::GlobeInverse(0.0f, 0.0f, -1.0f).valid);
}

TEST(GlobeInverse, RoundTripsAgainstTheForwardGlobeBranch) {
  // The strongest available statement that the new inverse is the inverse of the projection it
  // claims to invert: take a pixel, recover its world direction through the mask's own path,
  // push that direction back through ProjectExitToPixel, and require the original pixel back.
  RenderConfig cfg = MakeCfg(LensParam::kGlobe, 40.0f, 320, 240, RenderConfig::kFull, 30.0f);
  cfg.view_.az_ = 25.0f;
  cfg.view_.ro_ = 10.0f;
  const Rotation rot = MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  const lm_proj::ProjParams p = BuildProjParams(cfg, rot, short_pix);

  size_t checked = 0;
  Mismatches mm;
  for (int py = 0; py < cfg.resolution_[1]; py += 7) {
    for (int px = 0; px < cfg.resolution_[0]; px += 7) {
      const float x = static_cast<float>(px) + 0.5f - static_cast<float>(cfg.resolution_[0]) / 2.0f;
      const float y = static_cast<float>(py) + 0.5f - static_cast<float>(cfg.resolution_[1]) / 2.0f;
      const projection::Dir3 c = projection::GlobeInverse(x, y, p.scale);
      if (!c.valid) {
        continue;
      }
      float d[3]{ c.x, c.y, c.z };
      rot.Apply(d);
      const lm_proj::ProjResult r = lm_proj::ProjectExitToPixel(p, -d[0], -d[1], -d[2]);
      ++checked;
      std::ostringstream at;
      at << "pixel (" << px << "," << py << ")";
      if (r.count != 1) {
        mm.Note(at.str() + " recovered a direction the forward culls");
        continue;
      }
      // Exact pixel equality is not demandable: the forward bins on floor(v + 0.5) about res/2
      // while the mask samples pixel centres at (px + 0.5 - res/2), half a pixel apart, so a
      // round trip lands in the sampled pixel or its immediate neighbour.
      if (std::abs(r.hits[0].px - px) > 1 || std::abs(r.hits[0].py - py) > 1) {
        std::ostringstream got;
        got << at.str() << " round-tripped to (" << r.hits[0].px << "," << r.hits[0].py << ")";
        mm.Note(got.str());
      }
    }
  }
  EXPECT_EQ(mm.count, 0u) << mm.count << " round trips diverged; first: " << mm.first;
  EXPECT_GT(checked, 100u) << "too few in-domain samples for this to mean anything";
}

// ==================================================================================================
// Domain half, per lens family
// ==================================================================================================

TEST(VisibleMask, LinearImagesEveryPixel) {
  // LinearInverse rejects nothing: every screen point is some direction in front of the camera,
  // however wide the frame. This is a property of the projection, not of the chosen resolution,
  // so it is checked on a deliberately extreme aspect ratio too.
  for (const RenderConfig& cfg :
       { MakeCfg(LensParam::kLinear, 90.0f, 64, 48), MakeCfg(LensParam::kLinear, 170.0f, 200, 11) }) {
    const auto mask = Mask(cfg);
    EXPECT_EQ(CountOn(mask), mask.size())
        << "linear has no bounded image circle; every pixel must be in domain (fov " << cfg.lens_.fov_ << ")";
  }
}

TEST(VisibleMask, RectangularImagesEveryPixelOnATwoToOneCanvas) {
  // scale = min(w/2, h) / pi. When h <= w/2 the height is the binding term and lat spans exactly
  // [-pi/2, pi/2] over the full canvas: no polar caps.
  const RenderConfig cfg = MakeCfg(LensParam::kRectangular, 180.0f, 128, 64);
  const auto mask = Mask(cfg);
  EXPECT_EQ(CountOn(mask), mask.size());
}

TEST(VisibleMask, RectangularLeavesPolarCapsOnATallerCanvas) {
  // h > w/2: scale is set by the WIDTH, so the latitude range covers only h/2 +/- (pi/2)*scale
  // rows and the rest of the canvas is above the pole / below the antipole. That region is
  // unreachable by the forward projection too, so it must be masked off.
  const int w = 128;
  const int h = 128;
  const RenderConfig cfg = MakeCfg(LensParam::kRectangular, 180.0f, w, h);
  const auto mask = Mask(cfg);
  const float scale = static_cast<float>(std::min(w / 2, h)) / math::kPi;
  const float half_span = math::kPi_2 * scale;  // rows from the centre that latitudes reach
  ASSERT_LT(half_span, static_cast<float>(h) / 2.0f) << "fixture no longer has caps";

  size_t caps = 0;
  Mismatches mm;
  for (int py = 0; py < h; ++py) {
    const float dy = std::abs(static_cast<float>(py) + 0.5f - static_cast<float>(h) / 2.0f);
    const bool expect_on = dy <= half_span;
    for (int px = 0; px < w; ++px) {
      if (At(mask, cfg, px, py) != expect_on) {
        std::ostringstream at;
        at << "row " << py << " col " << px << " (|dy| = " << dy << ", span " << half_span << ") expected "
           << expect_on;
        mm.Note(at.str());
      }
    }
    if (!expect_on) {
      ++caps;
    }
  }
  EXPECT_EQ(mm.count, 0u) << mm.count << " pixels disagreed; first: " << mm.first;
  EXPECT_GT(caps, 0u);
}

TEST(VisibleMask, SingleFisheyeMasksOutsideTheImageCircle) {
  // Each single-lens fisheye's mask edge is its RIM — the radius its inverse stops accepting,
  // which is that type's radius formula at the largest theta the forward's per-type cull leaves
  // renderable (the three kFisheye*MinCz constants in projection_shared.h). Before
  // 474.1 all four shared one edge, the equator at r = 1; three of them now run further, and only
  // orthographic still stops there because sin(theta) aliases past it.
  //
  // `fov` is chosen per type so the rim lands INSIDE the frame — otherwise the case would assert
  // nothing (`off` would be 0). It is the rim that moved, so the fov that exposes it moved too:
  // at fov = 180 equidistant's rim is at 2 * scale = 150 px, past this frame's 125 px half
  // diagonal, and stereographic's is three orders of magnitude further out.
  struct Case {
    LensParam::LensType type;
    float fov;
    float rim_r;  // rim radius in normalized (r_scale = 1) coordinates
    const char* name;
  };
  const Case cases[] = {
    { LensParam::kFisheyeEqualArea, 180.0f, std::sqrt(2.0f), "equal_area" },
    { LensParam::kFisheyeEquidistant, 240.0f, 2.0f, "equidistant" },
    { LensParam::kFisheyeStereographic, 359.0f, std::tan(0.5f * std::acos(lm_proj::kFisheyeStereographicMinCz)),
      "stereographic" },
    { LensParam::kFisheyeOrthographic, 180.0f, 1.0f, "orthographic" },
  };
  for (const Case& c : cases) {
    const RenderConfig cfg = MakeCfg(c.type, c.fov, 200, 150);
    const float short_pix = 150.0f;
    const float fov_rad = c.fov * math::kDegreeToRad;
    const float scale = ComputeScaleAz0(c.type, fov_rad, short_pix, 200, 150, MakeCameraRotation(cfg)).scale;
    const float edge_px = scale * c.rim_r;
    const auto mask = Mask(cfg);

    size_t off = 0;
    Mismatches mm;
    for (int py = 0; py < cfg.resolution_[1]; ++py) {
      for (int px = 0; px < cfg.resolution_[0]; ++px) {
        const float r = RadiusPx(cfg, px, py);
        // Skip the one-pixel annulus straddling the boundary — a pixel centre landing within a
        // float ulp of r = 1 is a coin flip and pinning it would test rounding, not the mask.
        if (std::abs(r - edge_px) < 1.0f) {
          continue;
        }
        if (At(mask, cfg, px, py) != (r < edge_px)) {
          std::ostringstream at;
          at << c.name << " pixel (" << px << "," << py << ") at r = " << r << " vs edge " << edge_px;
          mm.Note(at.str());
        }
        if (r > edge_px) {
          ++off;
        }
      }
    }
    EXPECT_EQ(mm.count, 0u) << c.name << ": " << mm.count << " pixels disagreed; first: " << mm.first;
    EXPECT_GT(off, 0u) << c.name << ": no pixel outside the circle — this case covers nothing";
  }
}

TEST(VisibleMask, DualFisheyeMasksOutsideBothCircles) {
  // The two circles are the whole story for all four dual variants: PixelToDualFisheye already
  // returns r <= 1 coordinates, and no per-type inverse rejects inside that (orthographic
  // rejects beyond r_scale, and BuildProjParams never gives dual-orthographic an r_scale other
  // than 1). r_scale therefore moves directions, never the domain.
  const LensParam::LensType types[] = { LensParam::kDualFisheyeEqualArea, LensParam::kDualFisheyeEquidistant,
                                        LensParam::kDualFisheyeStereographic, LensParam::kDualFisheyeOrthographic };
  for (LensParam::LensType t : types) {
    const int w = 256;
    const int h = 128;
    const RenderConfig cfg = MakeCfg(t, 180.0f, w, h);
    const auto mask = Mask(cfg);
    const float r = static_cast<float>(std::min(w / 2, h)) / 2.0f;
    const float cy = static_cast<float>(h) / 2.0f;
    const float cx_left = static_cast<float>(w) / 2.0f - r;
    const float cx_right = static_cast<float>(w) / 2.0f + r;

    size_t off = 0;
    Mismatches mm;
    for (int py = 0; py < h; ++py) {
      for (int px = 0; px < w; ++px) {
        const float fx = static_cast<float>(px) + 0.5f;
        const float fy = static_cast<float>(py) + 0.5f;
        const float dl = std::sqrt((fx - cx_left) * (fx - cx_left) + (fy - cy) * (fy - cy));
        const float dr = std::sqrt((fx - cx_right) * (fx - cx_right) + (fy - cy) * (fy - cy));
        const float nearest = std::min(dl, dr);
        if (std::abs(nearest - r) < 1.0f) {
          continue;  // boundary annulus, see the single-fisheye case
        }
        if (At(mask, cfg, px, py) != (nearest < r)) {
          std::ostringstream at;
          at << "type " << static_cast<int>(t) << " pixel (" << px << "," << py << ") at d = " << nearest;
          mm.Note(at.str());
        }
        if (nearest > r) {
          ++off;
        }
      }
    }
    EXPECT_EQ(mm.count, 0u) << mm.count << " pixels disagreed; first: " << mm.first;
    EXPECT_GT(off, 0u) << "type " << static_cast<int>(t) << ": nothing outside both circles";
  }
}

TEST(VisibleMask, GlobeMasksOutsideTheSphereSilhouette) {
  // Globe is the only lens with a real invalid region that is not a fisheye circle, and the
  // region is the whole frame outside the sphere — not an edge case.
  const RenderConfig cfg = MakeCfg(LensParam::kGlobe, 30.0f, 200, 150);
  const float short_pix = 150.0f;
  const float scale =
      ComputeScaleAz0(LensParam::kGlobe, 30.0f * math::kDegreeToRad, short_pix, 200, 150, MakeCameraRotation(cfg))
          .scale;
  const float edge_px = GlobeSilhouettePx(scale);
  ASSERT_LT(edge_px, short_pix / 2.0f) << "fixture must leave frame outside the sphere";

  const auto mask = Mask(cfg);
  size_t off = 0;
  Mismatches mm;
  for (int py = 0; py < cfg.resolution_[1]; ++py) {
    for (int px = 0; px < cfg.resolution_[0]; ++px) {
      const float r = RadiusPx(cfg, px, py);
      if (std::abs(r - edge_px) < 1.0f) {
        continue;
      }
      if (At(mask, cfg, px, py) != (r < edge_px)) {
        std::ostringstream at;
        at << "globe pixel (" << px << "," << py << ") at r = " << r << " vs silhouette " << edge_px;
        mm.Note(at.str());
      }
      if (r > edge_px) {
        ++off;
      }
    }
  }
  EXPECT_EQ(mm.count, 0u) << mm.count << " pixels disagreed; first: " << mm.first;
  EXPECT_GT(off, 0u);
}

// ==================================================================================================
// Visibility half
// ==================================================================================================

TEST(VisibleMask, VisibleRangePartitionsTheFrameAndFullIsTheUnion) {
  // A 180 deg equal-area fisheye pointed at the horizon: the image circle straddles the horizon,
  // so upper and lower each take a real share of it. The three settings must partition the SAME
  // in-domain pixel set — that is what makes `visible` a visibility filter rather than a second
  // domain rule.
  const int w = 160;
  const int h = 160;
  auto cfg_for = [&](RenderConfig::VisibleRange vis) {
    return MakeCfg(LensParam::kFisheyeEqualArea, 180.0f, w, h, vis, /*el=*/0.0f);
  };
  const auto upper = Mask(cfg_for(RenderConfig::kUpper));
  const auto lower = Mask(cfg_for(RenderConfig::kLower));
  const auto full = Mask(cfg_for(RenderConfig::kFull));

  EXPECT_GT(CountOn(upper), 0u);
  EXPECT_GT(CountOn(lower), 0u);
  EXPECT_LT(CountOn(upper), CountOn(full)) << "pointing at the horizon must leave sky below it out of `upper`";
  EXPECT_LT(CountOn(lower), CountOn(full));

  size_t both = 0;
  Mismatches mm;
  for (size_t i = 0; i < full.size(); ++i) {
    // Union == full: every in-domain pixel is on one side of the horizon or the other.
    if (((upper[i] != 0) || (lower[i] != 0)) != (full[i] != 0)) {
      std::ostringstream at;
      at << "pixel " << i << ": upper=" << (upper[i] != 0) << " lower=" << (lower[i] != 0)
         << " full=" << (full[i] != 0);
      mm.Note(at.str());
    }
    if (upper[i] != 0 && lower[i] != 0) {
      ++both;
    }
  }
  EXPECT_EQ(mm.count, 0u) << mm.count << " pixels broke the partition; first: " << mm.first;
  // The only overlap allowed is the horizon itself (wz == 0 passes both tests).
  EXPECT_LT(both, full.size() / 20) << "upper and lower overlap far beyond the horizon line";
}

TEST(VisibleMask, VisibleRangeAppliesToRectangularAndDualFisheyeToo) {
  // ProjectExitToPixel's own `visible_range` cull lives inside the single-lens branch only, so
  // for these types the mask is the FIRST place the setting takes effect. It follows the GUI
  // preview shader, which tests `u_visible` after computing the world direction, independently
  // of the lens branch. (Ray energy is still not culled for them — a separate, pre-existing gap
  // in the shared backend header.)
  for (LensParam::LensType t : { LensParam::kRectangular, LensParam::kDualFisheyeEqualArea }) {
    const auto full = Mask(MakeCfg(t, 180.0f, 256, 128, RenderConfig::kFull));
    const auto upper = Mask(MakeCfg(t, 180.0f, 256, 128, RenderConfig::kUpper));
    const auto lower = Mask(MakeCfg(t, 180.0f, 256, 128, RenderConfig::kLower));
    EXPECT_LT(CountOn(upper), CountOn(full)) << "type " << static_cast<int>(t) << ": `upper` masked nothing";
    EXPECT_LT(CountOn(lower), CountOn(full)) << "type " << static_cast<int>(t) << ": `lower` masked nothing";
    EXPECT_NEAR(static_cast<double>(CountOn(upper) + CountOn(lower)), static_cast<double>(CountOn(full)),
                static_cast<double>(full.size()) * 0.05)
        << "type " << static_cast<int>(t) << ": the two halves should tile the full set";
  }
}

// ==================================================================================================
// Front-hemisphere half (v4.20). A SECOND clip, ANDed with `visible`, so the oracles below are all
// statements that hold whatever the world-axis sign convention turns out to be: what the camera
// faces, where it stops, and that it intersects rather than replaces.
// ==================================================================================================

TEST(VisibleMask, FrontIsANoOpOnlyWhereTheDomainStopsAtTheEquator) {
  // This used to assert the no-op for the WHOLE single-lens family, on the premise its own comment
  // stated: "every single-lens inverse rejects r > 1, and r = 1 is theta = 90 deg". 474.1 removed
  // that premise for three of the five — equal-area, equidistant and stereographic now invert out
  // to theta = 180 deg — so the clip has real work to do there and the old assertion became false.
  // It is narrowed rather than relaxed, and the half it gave up is asserted positively below.
  //
  // The two that remain are not leftovers; each stops at the front hemisphere for its own reason:
  //   * kLinear maps radius through theta = atan(rho / focal), strictly below 90 deg at every
  //     finite radius. There is no image circle to invert past.
  //   * kFisheyeOrthographic is capped at 90 deg on purpose (474.1): r = sin(theta) peaks at the
  //     equator and folds back, so widening it is not invertible.
  for (LensParam::LensType t : { LensParam::kLinear, LensParam::kFisheyeOrthographic }) {
    for (float el : { 90.0f, 30.0f, 0.0f }) {
      RenderConfig off = MakeCfg(t, 170.0f, 128, 128, RenderConfig::kFull, el);
      RenderConfig on = off;
      on.front_ = true;
      const auto m_off = Mask(off);
      const auto m_on = Mask(on);
      if (CountOn(m_off) == 0u) {
        // Non-fatal: a vacuous row must not take the remaining lens types and elevations with it.
        ADD_FAILURE() << "type " << static_cast<int>(t) << " el " << el << ": the unclipped mask is empty";
        continue;
      }
      EXPECT_EQ(m_on, m_off) << "type " << static_cast<int>(t) << " el " << el
                             << ": the front clip removed pixels a single lens never imaged behind";
    }
  }
}

TEST(VisibleMask, FrontClipsTheWidenedSingleLensTypes) {
  // The positive half of the split above, and the reason narrowing it is not a loss of coverage:
  // for the three types 474.1 widened past the equator, the front clip MUST remove something, or
  // the widening and the clip are not both reaching the mask. A clip written against the wrong
  // axis would show up here as "removed nothing" just as surely as it would have shown up in the
  // old no-op assertion as "removed something".
  //
  // Elevation 0 only: the camera on the horizon is where a 170 deg frame reaches furthest behind
  // it. At the zenith the widened region is the sky near the nadir, which the clip also removes,
  // but the horizon case is the one that fails first if the clip's axis is wrong.
  for (LensParam::LensType t :
       { LensParam::kFisheyeEqualArea, LensParam::kFisheyeEquidistant, LensParam::kFisheyeStereographic }) {
    RenderConfig off = MakeCfg(t, 170.0f, 128, 128, RenderConfig::kFull, 0.0f);
    RenderConfig on = off;
    on.front_ = true;
    const auto m_off = Mask(off);
    const auto m_on = Mask(on);
    if (CountOn(m_off) == 0u) {
      // Non-fatal for the same reason the no-op test above is: one empty row must not take the
      // remaining lens types with it.
      ADD_FAILURE() << "type " << static_cast<int>(t) << ": the unclipped mask is empty";
      continue;
    }
    EXPECT_LT(CountOn(m_on), CountOn(m_off))
        << "type " << static_cast<int>(t)
        << ": the front clip removed nothing from a lens whose domain now reaches past the equator";
    // And it must not remove everything either — that is the flipped-comparison failure.
    EXPECT_GT(CountOn(m_on), 0u) << "type " << static_cast<int>(t) << ": the front clip emptied the mask";
  }
}

TEST(VisibleMask, FrontAtTheZenithIsExactlyTheUpperHemisphere) {
  // The proposition is geometric, not conventional: a camera pointed straight up faces the upper
  // half of the sky, so ITS front hemisphere and THE upper hemisphere are the same set. Pointed
  // straight down, the lower one. Checked on a full-sky lens, where the two clips can disagree —
  // on kLinear and kFisheyeOrthographic both are no-ops and the test would prove nothing. (The
  // other three single lenses reach past the equator since 474.1, so the clip does bite there;
  // the full-sky lens is still the clearer probe because its two clips can differ, not merely
  // both be non-empty.)
  const int w = 200;
  const int h = 100;
  {
    RenderConfig front_up = MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, RenderConfig::kFull, 90.0f);
    front_up.front_ = true;
    const auto upper = Mask(MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, RenderConfig::kUpper, 90.0f));
    ASSERT_GT(CountOn(upper), 0u);
    EXPECT_EQ(Mask(front_up), upper);
  }
  {
    RenderConfig front_down = MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, RenderConfig::kFull, -90.0f);
    front_down.front_ = true;
    const auto lower = Mask(MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, RenderConfig::kLower, -90.0f));
    ASSERT_GT(CountOn(lower), 0u);
    EXPECT_EQ(Mask(front_down), lower);
  }
}

TEST(VisibleMask, FrontHalvesAFullSkyLensAndAndsWithVisible) {
  // Camera on the horizon: now the two clips cut along PERPENDICULAR planes, which is the only
  // arrangement in which "AND with base" is distinguishable from "replaces base". front alone must
  // take half the sky, visible alone the other half, and the two together a quarter — a clip that
  // overwrote `visible` instead of intersecting it would land on a half.
  const int w = 200;
  const int h = 100;
  auto cfg = [&](RenderConfig::VisibleRange vis, bool front) {
    RenderConfig c = MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, vis, /*el=*/0.0f);
    c.front_ = front;
    return c;
  };
  const auto full = Mask(cfg(RenderConfig::kFull, false));
  const auto front_only = Mask(cfg(RenderConfig::kFull, true));
  const auto upper_only = Mask(cfg(RenderConfig::kUpper, false));
  const auto both = Mask(cfg(RenderConfig::kUpper, true));

  const double n = static_cast<double>(CountOn(full));
  ASSERT_GT(n, 0.0);
  EXPECT_NEAR(static_cast<double>(CountOn(front_only)), n / 2.0, n * 0.02);
  EXPECT_NEAR(static_cast<double>(CountOn(upper_only)), n / 2.0, n * 0.02);
  EXPECT_NEAR(static_cast<double>(CountOn(both)), n / 4.0, n * 0.02);

  // Pixel-exact: the combined mask is the intersection, not an approximation of one.
  Mismatches mm;
  for (size_t i = 0; i < full.size(); ++i) {
    const bool expected = (front_only[i] != 0) && (upper_only[i] != 0);
    if ((both[i] != 0) != expected) {
      std::ostringstream at;
      at << "pixel " << i << ": front=" << (front_only[i] != 0) << " upper=" << (upper_only[i] != 0)
         << " both=" << (both[i] != 0);
      mm.Note(at.str());
    }
  }
  EXPECT_EQ(mm.count, 0u) << mm.count << " pixels broke the AND; first: " << mm.first;
}

TEST(VisibleMask, FrontFollowsTheAzimuthRatherThanAFixedWorldAxis) {
  // Two cameras on the horizon looking opposite ways partition the same sky: whatever one keeps,
  // the other drops. This is what pins the clip to the CAMERA's forward — a clip hard-wired to a
  // world axis would return the same set for both.
  const int w = 200;
  const int h = 100;
  auto looking = [&](float az) {
    RenderConfig c = MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, RenderConfig::kFull, /*el=*/0.0f);
    c.view_.az_ = az;
    c.front_ = true;
    return Mask(c);
  };
  const auto north = looking(0.0f);
  const auto south = looking(180.0f);
  const auto full = Mask(MakeCfg(LensParam::kDualFisheyeEqualArea, 180.0f, w, h, RenderConfig::kFull, /*el=*/0.0f));

  size_t overlap = 0;
  Mismatches mm;
  for (size_t i = 0; i < full.size(); ++i) {
    // The two half-skies are complementary WITHIN the imaged domain; the lens domain itself is
    // identical for the two, since azimuth only spins the sky the mask reads.
    if (((north[i] != 0) || (south[i] != 0)) != (full[i] != 0)) {
      std::ostringstream at;
      at << "pixel " << i << ": north=" << (north[i] != 0) << " south=" << (south[i] != 0)
         << " full=" << (full[i] != 0);
      mm.Note(at.str());
    }
    if (north[i] != 0 && south[i] != 0) {
      ++overlap;
    }
  }
  EXPECT_EQ(mm.count, 0u) << mm.count << " pixels fell outside both half-skies; first: " << mm.first;
  // Only the seam plane (dot == 0, kept by both) may overlap.
  EXPECT_LT(overlap, full.size() / 20) << "the two opposite views overlap far beyond their shared seam";
}

TEST(VisibleMask, DegenerateResolutionYieldsAnEmptyMask) {
  RenderConfig cfg = MakeCfg(LensParam::kLinear, 90.0f, 0, 0);
  EXPECT_TRUE(Mask(cfg).empty());
}

}  // namespace
}  // namespace lumice
