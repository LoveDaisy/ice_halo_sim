// RenderConsumer's side of the angular-distance circles (grid.angular_dist): that the pixels the
// masks mark are the pixels that come out tinted, that EACH LINE keeps its own colour and opacity,
// that the circles are centred on the sun rather than its antipode, and that both of the inputs
// NeedsRebuild does not cover — the line list and the sun — reach the image through the reuse path.
//
// Sibling of test_render_consumer_horizon.cpp and asks its question the same way: difference two
// snapshots of the SAME consumer config, one with the annotation and one without, so the ray
// energy cancels and only the annotation is left. What is different here, and is the reason this
// is a separate file rather than more cases in that one:
//
//   - the geometry depends on the SUN, which no other RenderConfig-derived mask does. An inverted
//     sun direction still produces a perfectly round circle, just around the wrong point, so
//     "a ring appeared" is not evidence of correctness and CircleIsCentredOnTheSun asks for the
//     radius against the sun's own projected pixel instead;
//   - each line carries its own opacity_/color_, which is why the consumer holds one mask PER LINE
//     rather than the single per-category union annotation::ComputeOverlay returns for a
//     batched request. TwoLinesKeepTheirOwnColours is the case a batched implementation fails;
//   - neither the line list nor the sun takes part in NeedsRebuild, so ResetWith is the only path
//     by which either can change under a live consumer. Both halves are covered below.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/annotation_overlay.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "server/render.hpp"
#include "support/render_anchor.hpp"

namespace lumice {
namespace {

constexpr int kW = 96;
constexpr int kH = 96;
constexpr int kTotalPix = kW * kH;

// The sun 45 deg up, due 0 azimuth. The view below is centred on it, so a 22 deg circle around it
// sits well inside the frame with room on every side — which is what lets the radius assertion
// distinguish "centred on the sun" from "centred on anything else in view".
constexpr float kSunAltitude = 45.0f;
SunParam MakeSun(float altitude_deg = kSunAltitude) {
  return SunParam{ altitude_deg, 0.0f, 0.5f };
}

// A 120 deg linear frame centred 45 deg up: altitudes [-15, 105]. Holds the sun, a 22 deg circle
// around it, and the zenith where the one ray lands. Background stays black so a tinted pixel is
// unambiguous.
RenderConfig MakeConfig(const std::vector<GridLineParam>& lines) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kLinear;
  cfg.lens_.fov_ = 120.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = kSunAltitude;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.angular_dist_grid_ = lines;
  return cfg;
}

GridLineParam Line(float value_deg, float opacity, float r, float g, float b) {
  GridLineParam p;
  p.value_ = value_deg;
  p.opacity_ = opacity;
  p.color_[0] = r;
  p.color_[1] = g;
  p.color_[2] = b;
  return p;
}

// One ray straight up: only rays that LAND reach total_intensity_, and PostSnapshot takes a
// zero-intensity early-out that would make every assertion below vacuous. Which pixel it lights
// does not matter — it is the same pixel in both arms and the difference removes it.
SimData MakeOneRayBatch() {
  SimData data;
  data.curr_wl_ = 550.0f;
  data.outgoing_d_ = { 0.0f, 0.0f, -1.0f };
  data.outgoing_w_ = { 1.0f };
  return data;
}

std::vector<uint8_t> SnapshotOnce(RenderConsumer* rc) {
  auto data = MakeOneRayBatch();
  rc->Consume(data);
  lumice::test::TakeSnapshotAtFormerSelfAnchor(rc);
  auto result = rc->GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  if (rr == nullptr || rr->img_buffer_ == nullptr) {
    return {};
  }
  return std::vector<uint8_t>(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
}

bool IsBlack(const std::vector<uint8_t>& img, int i) {
  return img[static_cast<size_t>(i) * 3] == 0 && img[static_cast<size_t>(i) * 3 + 1] == 0 &&
         img[static_cast<size_t>(i) * 3 + 2] == 0;
}

// "Unmistakably this hue", not an exact triple: the blend runs in linear RGB through the sRGB
// transfer curve, so pinning the byte would pin the whole colour pipeline in a file about geometry.
bool LooksRed(const std::vector<uint8_t>& img, int i) {
  const int r = img[static_cast<size_t>(i) * 3];
  const int g = img[static_cast<size_t>(i) * 3 + 1];
  const int b = img[static_cast<size_t>(i) * 3 + 2];
  return r >= 120 && r > g + 60 && r > b + 60;
}

bool LooksBlue(const std::vector<uint8_t>& img, int i) {
  const int r = img[static_cast<size_t>(i) * 3];
  const int g = img[static_cast<size_t>(i) * 3 + 1];
  const int b = img[static_cast<size_t>(i) * 3 + 2];
  return b >= 120 && b > r + 60 && b > g + 60;
}

// Where the sun itself lands on this canvas, derived through the projection the renderer uses
// rather than from the mask being tested.
annotation::CanvasPoint SunPixel(const RenderConfig& cfg, const SunParam& sun) {
  float dir[3];
  annotation::SunWorldDir(sun, dir);
  const Rotation rot = MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  lm_proj::ProjParams p = BuildProjParams(cfg, rot, short_pix);
  return annotation::ProjectWorldDir(p, dir[0], dir[1], dir[2]);
}

TEST(RenderConsumerAngularDist, PaintedPixelsAreExactlyTheMaskedOnes) {
  RenderConsumer off(MakeConfig({}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  RenderConsumer on(MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) }), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const auto& masks = on.AngularDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));
  const size_t marked = static_cast<size_t>(std::count(masks[0].begin(), masks[0].end(), uint8_t{ 1 }));
  ASSERT_GT(marked, 0u) << "an empty mask would make every assertion below vacuous";
  ASSERT_LT(marked, static_cast<size_t>(kTotalPix) / 4) << "the mask must be a line, not a region";

  size_t checked = 0;
  size_t missing = 0;
  size_t stray = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (!IsBlack(img_off, i)) {
      continue;  // already lit by the ray; carries no verdict either way
    }
    if (masks[0][static_cast<size_t>(i)] != 0) {
      ++checked;
      if (!LooksRed(img_on, i)) {
        ++missing;
      }
    } else if (!IsBlack(img_on, i)) {
      ++stray;
    }
  }
  EXPECT_GT(checked, 0u) << "the mask and the ray-lit pixels must not be the same set";
  EXPECT_EQ(missing, 0u) << "every masked pixel over black background must come out tinted";
  EXPECT_EQ(stray, 0u) << "no pixel outside the masks may change colour when the annotation is switched on";
}

TEST(RenderConsumerAngularDist, CircleIsCentredOnTheSun) {
  // The assertion an inverted sun direction fails: a 22 deg circle around -sun is a 158 deg circle
  // around the sun, which on this 120 deg frame would not be a ring around the sun's pixel at all.
  // The expected radius is computed from the projection, not measured from the mask.
  const RenderConfig cfg = MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) });
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  const auto& masks = rc.AngularDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));

  const annotation::CanvasPoint sun_px = SunPixel(cfg, MakeSun());
  ASSERT_TRUE(sun_px.valid) << "the fixture must put the sun on the canvas";

  // Where a point 22 deg from the sun ALONG THE MERIDIAN lands, i.e. the ring's radius in pixels,
  // derived independently of the mask.
  const annotation::CanvasPoint edge_px = SunPixel(cfg, MakeSun(kSunAltitude + 22.0f));
  ASSERT_TRUE(edge_px.valid);
  const float dx = edge_px.px - sun_px.px;
  const float dy = edge_px.py - sun_px.py;
  const float expect_r = std::sqrt(dx * dx + dy * dy);
  ASSERT_GT(expect_r, 4.0f) << "a ring this small would not separate centre from antipode";

  size_t marked = 0;
  float min_r = 1e9f;
  float max_r = 0.0f;
  for (int i = 0; i < kTotalPix; ++i) {
    if (masks[0][static_cast<size_t>(i)] == 0) {
      continue;
    }
    ++marked;
    const float px = static_cast<float>(i % kW) + 0.5f;
    const float py = static_cast<float>(i / kW) + 0.5f;
    const float r = std::sqrt((px - sun_px.px) * (px - sun_px.px) + (py - sun_px.py) * (py - sun_px.py));
    min_r = std::min(min_r, r);
    max_r = std::max(max_r, r);
  }
  ASSERT_GT(marked, 0u);
  // Every lit pixel sits within a couple of pixels of the expected radius — a ring, at the right
  // distance, all the way round. A band 2 px wide is what the mask generator's own adaptive
  // half-width produces here.
  EXPECT_NEAR(min_r, expect_r, 3.0f) << "inner edge of the ring";
  EXPECT_NEAR(max_r, expect_r, 3.0f) << "outer edge of the ring";
}

TEST(RenderConsumerAngularDist, TwoLinesKeepTheirOwnColours) {
  // The proposition a single per-category union mask cannot satisfy: the 22 deg ring comes out red
  // and the 46 deg ring blue. Batching both angles into one ComputeOverlay call would return one
  // mask covering both rings, and whichever colour the code picked would paint both.
  const RenderConfig cfg = MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f), Line(46.0f, 1.0f, 0.0f, 0.0f, 1.0f) });
  RenderConsumer off(MakeConfig({}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const auto& masks = on.AngularDistMasksForTest();
  ASSERT_EQ(masks.size(), 2u) << "one mask per line, not one per category";
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));
  ASSERT_EQ(masks[1].size(), static_cast<size_t>(kTotalPix));

  size_t red_on_inner = 0;
  size_t wrong_on_inner = 0;
  size_t blue_on_outer = 0;
  size_t wrong_on_outer = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (!IsBlack(img_off, i)) {
      continue;
    }
    const bool inner = masks[0][static_cast<size_t>(i)] != 0;
    const bool outer = masks[1][static_cast<size_t>(i)] != 0;
    if (inner && !outer) {
      LooksRed(img_on, i) ? ++red_on_inner : ++wrong_on_inner;
    } else if (outer && !inner) {
      LooksBlue(img_on, i) ? ++blue_on_outer : ++wrong_on_outer;
    }
  }
  EXPECT_GT(red_on_inner, 0u) << "the 22 deg ring must exist and be red";
  EXPECT_GT(blue_on_outer, 0u) << "the 46 deg ring must exist and be blue";
  EXPECT_EQ(wrong_on_inner, 0u) << "no pixel of the 22 deg ring may take the 46 deg line's colour";
  EXPECT_EQ(wrong_on_outer, 0u) << "no pixel of the 46 deg ring may take the 22 deg line's colour";
}

TEST(RenderConsumerAngularDist, ResetWithPicksUpANewLineList) {
  // angular_dist_grid_ is on the appearance-only side of NeedsRebuild, so a config that adds a
  // circle mid-run reaches a REUSED consumer. Masks that were only ever built in the constructor
  // would leave that consumer drawing nothing.
  RenderConsumer rc(MakeConfig({}), ColorClassTable{}, MakeSun());
  ASSERT_TRUE(rc.AngularDistMasksForTest().empty());
  const std::vector<uint8_t> img_off = SnapshotOnce(&rc);

  rc.ResetWith(MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) }), MakeSun());
  ASSERT_EQ(rc.AngularDistMasksForTest().size(), 1u);
  const std::vector<uint8_t> img_on = SnapshotOnce(&rc);

  size_t tinted = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (IsBlack(img_off, i) && LooksRed(img_on, i)) {
      ++tinted;
    }
  }
  EXPECT_GT(tinted, 0u) << "adding a circle through ResetWith must reach the image without a rebuild";
}

TEST(RenderConsumerAngularDist, ResetWithPicksUpANewSun) {
  // The other half of the same gap, and the one nothing else in the tree covers: the sun is not a
  // RenderConfig field at all, so NOTHING forces a new consumer when it moves. A consumer that
  // cached its masks against the construction-time sun would keep drawing the circle around where
  // the sun used to be — a wrong picture that still looks like a correct one.
  const std::vector<GridLineParam> lines = { Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) };
  RenderConsumer rc(MakeConfig(lines), ColorClassTable{}, MakeSun(kSunAltitude));
  ASSERT_EQ(rc.AngularDistMasksForTest().size(), 1u);
  const std::vector<uint8_t> before = rc.AngularDistMasksForTest()[0];
  ASSERT_GT(static_cast<size_t>(std::count(before.begin(), before.end(), uint8_t{ 1 })), 0u);

  rc.ResetWith(MakeConfig(lines), MakeSun(kSunAltitude - 20.0f));
  ASSERT_EQ(rc.AngularDistMasksForTest().size(), 1u);
  const std::vector<uint8_t>& after = rc.AngularDistMasksForTest()[0];
  ASSERT_EQ(after.size(), before.size());
  EXPECT_NE(before, after) << "moving the sun must move the circle";
}

// ---------------------------------------------------------------------------------------------
// The globe lens. Two cases, because the way this circle "does not draw" under globe has a
// specific mechanism that is easy to re-derive as a bug: `view` under globe is the observer's
// ORBIT position (doc/configuration.md, "globe lens"), so the cap the frame shows is centred on
// the direction OPPOSITE to (azimuth, elevation) — (azimuth + 180, -elevation). A config that
// points `view` at the sun the way an inside-out lens would puts the sun 180 deg from the cap's
// centre, and a 22 deg circle around it lies wholly on the far side of the sphere. The parallels,
// the meridians and the horizon are global curves that cross the cap from any orbit position,
// which is why they still appear in the same frame and make the circle look singled out.
//
// The mask path itself (RebuildAngularDistMasks -> annotation::ComputeOverlay -> PixelToWorld's
// kGlobe branch -> LevelSetMaskFromField) has no lens-type branch of its own; the first case
// shows it producing the right circle when the geometry can, and the second pins the orbit
// semantics that make the "wrong" view empty, so the next reader does not spend a day proving
// the circle code is fine.

// The globe's default frame: fov 30 fills ~96% of the short edge with the sphere. `az`/`el` are
// the ORBIT position; see above.
RenderConfig MakeGlobeConfig(float view_az, float view_el, const std::vector<GridLineParam>& lines) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kGlobe;
  cfg.lens_.fov_ = 30.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.az_ = view_az;
  cfg.view_.el_ = view_el;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.angular_dist_grid_ = lines;
  return cfg;
}

size_t CountMarked(const std::vector<uint8_t>& mask) {
  return static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
}

TEST(RenderConsumerAngularDist, GlobeCircleAppearsAtTheGlobeRadius) {
  // The orbit position opposite the sun, so the sun sits on the optical axis and the 22 deg circle
  // around it is, by symmetry, a circle on the canvas too (off-axis it would be an ellipse).
  const RenderConfig cfg = MakeGlobeConfig(180.0f, -kSunAltitude, { Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) });
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  const auto& masks = rc.AngularDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));

  const annotation::CanvasPoint sun_px = SunPixel(cfg, MakeSun());
  ASSERT_TRUE(sun_px.valid) << "the orbit position opposite the sun must image the sun";
  EXPECT_NEAR(sun_px.px, kW / 2.0f, 1.0f) << "and image it on the optical axis";
  EXPECT_NEAR(sun_px.py, kH / 2.0f, 1.0f);

  // The radius, twice over and independently of the mask: once through the forward projection
  // the rays themselves land by (a point 22 deg up the meridian from the sun), and once from the
  // globe's own closed form, rho = focal * sin(psi) / (D - cos(psi)) with focal = (short/2) /
  // tan(fov/2) — the formula projection_shared.h's globe branch evaluates.
  const annotation::CanvasPoint edge_px = SunPixel(cfg, MakeSun(kSunAltitude + 22.0f));
  ASSERT_TRUE(edge_px.valid);
  const float expect_r = std::hypot(edge_px.px - sun_px.px, edge_px.py - sun_px.py);
  const float focal = (static_cast<float>(std::min(kW, kH)) / 2.0f) / std::tan(15.0f * math::kDegreeToRad);
  const float psi = 22.0f * math::kDegreeToRad;
  const float closed_form_r = focal * std::sin(psi) / (lm_proj::kGlobeCameraD - std::cos(psi));
  EXPECT_NEAR(expect_r, closed_form_r, 1.0f) << "the forward projection and the closed form must agree";
  ASSERT_GT(expect_r, 4.0f);

  size_t marked = 0;
  float min_r = 1e9f;
  float max_r = 0.0f;
  for (int i = 0; i < kTotalPix; ++i) {
    if (masks[0][static_cast<size_t>(i)] == 0) {
      continue;
    }
    ++marked;
    const float px = static_cast<float>(i % kW) + 0.5f;
    const float py = static_cast<float>(i / kW) + 0.5f;
    const float r = std::hypot(px - sun_px.px, py - sun_px.py);
    min_r = std::min(min_r, r);
    max_r = std::max(max_r, r);
  }
  ASSERT_GT(marked, 0u) << "globe must draw the 22 deg circle when the sun is on the visible cap";
  EXPECT_NEAR(min_r, expect_r, 3.0f) << "inner edge of the ring";
  EXPECT_NEAR(max_r, expect_r, 3.0f) << "outer edge of the ring";
}

TEST(RenderConsumerAngularDist, GlobeImagesTheCapOppositeTheViewDirection) {
  // The shape of the report that motivated the case above: `view` written to face the sun, as it
  // would be for a linear lens, under globe shows the cap centred 180 deg away. The sun is then
  // not on the canvas at all and its circle has nowhere to be drawn — while the same list in the
  // opposite orbit position draws hundreds of pixels. Pinned so that this reads as the lens's
  // documented orbit semantics rather than as a mask defect.
  const std::vector<GridLineParam> lines = { Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) };
  const RenderConfig facing = MakeGlobeConfig(0.0f, kSunAltitude, lines);
  const RenderConfig opposite = MakeGlobeConfig(180.0f, -kSunAltitude, lines);

  EXPECT_FALSE(SunPixel(facing, MakeSun()).valid) << "an orbit position AT the sun looks at the far side";
  EXPECT_TRUE(SunPixel(opposite, MakeSun()).valid);

  RenderConsumer rc_facing(facing, ColorClassTable{}, MakeSun());
  RenderConsumer rc_opposite(opposite, ColorClassTable{}, MakeSun());
  ASSERT_EQ(rc_facing.AngularDistMasksForTest().size(), 1u);
  ASSERT_EQ(rc_opposite.AngularDistMasksForTest().size(), 1u);
  EXPECT_EQ(CountMarked(rc_facing.AngularDistMasksForTest()[0]), 0u)
      << "a 22 deg circle around a point 180 deg from the cap's centre lies wholly outside the cap";
  EXPECT_GT(CountMarked(rc_opposite.AngularDistMasksForTest()[0]), 0u);

  // The cap's angular radius is acos(1/D) ~ 75.5 deg, set by the camera distance alone (never by
  // fov, which only scales the sphere on the canvas). A circle of that radius or more around the
  // on-axis sun cannot be imaged, and one well inside it is — the two bounds on what "visible" can
  // mean for this family under globe.
  RenderConsumer inside(MakeGlobeConfig(180.0f, -kSunAltitude, { Line(46.0f, 1.0f, 1.0f, 0.0f, 0.0f) }),
                        ColorClassTable{}, MakeSun());
  RenderConsumer beyond(MakeGlobeConfig(180.0f, -kSunAltitude, { Line(80.0f, 1.0f, 1.0f, 0.0f, 0.0f) }),
                        ColorClassTable{}, MakeSun());
  EXPECT_GT(CountMarked(inside.AngularDistMasksForTest()[0]), 0u);
  EXPECT_EQ(CountMarked(beyond.AngularDistMasksForTest()[0]), 0u);
}

}  // namespace
}  // namespace lumice
