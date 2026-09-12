// RenderConsumer's side of the view-distance circles (grid.view_dist): circles of constant angular
// distance from the camera's OPTICAL AXIS, the axis-referenced twin of grid.angular_dist. Same
// masks-per-line compositing, same reuse path, and the same way of asking (difference two
// snapshots of one consumer config, with and without the annotation, so the ray energy cancels).
// What is pinned here is exactly what differs from the twin:
//
//   - the centre is the camera forward, NOT the sun and NOT the canvas centre. Two of those three
//     coincide on any fixture that points the camera at the sun with no lens shift, so the fixtures
//     below deliberately put the sun off-axis and one of them shifts the lens;
//   - the radius follows the lens's own projection of a direction the ring's angle away from the
//     axis — derived through BuildProjParams / ProjectWorldDir, never from the mask — for the two
//     lenses whose radius formulas differ (linear: f*tan; equal-area fisheye: 2f*sin(theta/2));
//   - the sun is NOT an input: a ResetWith that moves only the sun must leave the mask untouched,
//     the opposite of the proposition test_render_consumer_angular_dist.cpp pins for its family.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
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

// The sun well away from the optical axis: 20 deg up and 90 deg round from the camera azimuth. A
// fixture that pointed the camera at the sun could not tell "centred on the axis" from "centred on
// the sun", which is the one confusion this family invites.
SunParam MakeSun(float altitude_deg = 20.0f) {
  return SunParam{ altitude_deg, 90.0f, 0.5f };
}

constexpr float kCameraElevation = 45.0f;

// A frame centred 45 deg up. Background black so a tinted pixel is unambiguous; `visible` upper
// so the fixture stays on the render domain the angular_dist sibling uses.
RenderConfig MakeConfig(const std::vector<GridLineParam>& lines, LensParam::LensType lens = LensParam::kLinear,
                        float fov_deg = 120.0f) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = lens;
  cfg.lens_.fov_ = fov_deg;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = kCameraElevation;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.view_dist_grid_ = lines;
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

// One ray straight up, for the reason the angular_dist sibling gives: PostSnapshot early-outs on
// zero intensity, and the difference removes the pixel it lights.
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

lm_proj::ProjParams ProjOf(const RenderConfig& cfg) {
  const Rotation rot = MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return BuildProjParams(cfg, rot, short_pix);
}

// The optical axis in world space, from the renderer's own camera rotation — the same authority
// RenderConsumer's Omega_axis and annotation_overlay.cpp's SetUp read.
std::array<float, 3> AxisOf(const RenderConfig& cfg) {
  const Rotation rot = MakeCameraRotation(cfg);
  std::array<float, 3> f{};
  mask_detail::CameraForward(rot, f.data());
  return f;
}

// A world direction `theta_deg` away from the axis, tilted towards the zenith: cos(t)*f + sin(t)*u
// with u the unit vector perpendicular to f in the plane of f and +z. Built from the axis itself
// rather than through any sun/altitude helper, so the expected radius does not lean on a second
// convention agreeing with the first.
std::array<float, 3> OffAxisTowardsZenith(const std::array<float, 3>& f, float theta_deg) {
  std::array<float, 3> u{ -f[0] * f[2], -f[1] * f[2], 1.0f - f[2] * f[2] };
  const float len = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  for (float& c : u) {
    c /= len;
  }
  const float ct = std::cos(theta_deg * math::kDegreeToRad);
  const float st = std::sin(theta_deg * math::kDegreeToRad);
  return { ct * f[0] + st * u[0], ct * f[1] + st * u[1], ct * f[2] + st * u[2] };
}

annotation::CanvasPoint Project(const lm_proj::ProjParams& p, const std::array<float, 3>& d) {
  return annotation::ProjectWorldDir(p, d[0], d[1], d[2]);
}

// The radial extent of a mask about a centre: every lit pixel's distance, min and max.
struct RingExtent {
  size_t marked = 0;
  float min_r = 1e9f;
  float max_r = 0.0f;
};
RingExtent ExtentAbout(const std::vector<uint8_t>& mask, float cx, float cy) {
  RingExtent e;
  for (int i = 0; i < kTotalPix; ++i) {
    if (mask[static_cast<size_t>(i)] == 0) {
      continue;
    }
    ++e.marked;
    const float px = static_cast<float>(i % kW) + 0.5f;
    const float py = static_cast<float>(i / kW) + 0.5f;
    const float r = std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
    e.min_r = std::min(e.min_r, r);
    e.max_r = std::max(e.max_r, r);
  }
  return e;
}

TEST(RenderConsumerViewDist, PaintedPixelsAreExactlyTheMaskedOnes) {
  RenderConsumer off(MakeConfig({}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  RenderConsumer on(MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) }), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const auto& masks = on.ViewDistMasksForTest();
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
      continue;
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
  EXPECT_GT(checked, 0u);
  EXPECT_EQ(missing, 0u) << "every masked pixel over black background must come out tinted";
  EXPECT_EQ(stray, 0u) << "no pixel outside the masks may change colour when the annotation is switched on";
}

// AC1, linear: the ring's radius is f*tan(22 deg) about the axis pixel — obtained by projecting a
// direction 22 deg off-axis, not by transcribing the formula. The sun is 90 deg round from the
// axis, so a ring centred on it would not be a ring about the axis pixel at any radius.
TEST(RenderConsumerViewDist, LinearCircleIsCentredOnTheOpticalAxisAtTheProjectedRadius) {
  const RenderConfig cfg = MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) });
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  const auto& masks = rc.ViewDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));

  const lm_proj::ProjParams p = ProjOf(cfg);
  const std::array<float, 3> axis = AxisOf(cfg);
  const annotation::CanvasPoint axis_px = Project(p, axis);
  ASSERT_TRUE(axis_px.valid);
  // Premise, not proposition: with no lens shift the axis images at the canvas centre.
  ASSERT_NEAR(axis_px.px, kW / 2.0f, 0.51f);
  ASSERT_NEAR(axis_px.py, kH / 2.0f, 0.51f);

  const annotation::CanvasPoint edge_px = Project(p, OffAxisTowardsZenith(axis, 22.0f));
  ASSERT_TRUE(edge_px.valid);
  const float expect_r = std::hypot(edge_px.px - axis_px.px, edge_px.py - axis_px.py);
  ASSERT_GT(expect_r, 4.0f);

  const RingExtent e = ExtentAbout(masks[0], axis_px.px, axis_px.py);
  ASSERT_GT(e.marked, 0u);
  EXPECT_NEAR(e.min_r, expect_r, 3.0f) << "inner edge of the ring";
  EXPECT_NEAR(e.max_r, expect_r, 3.0f) << "outer edge of the ring";
}

// AC1, equal-area fisheye: same proposition under the other radius law (2f*sin(theta/2)). A ring
// at 60 deg is where the two laws differ by far more than the band width, so a consumer that
// projected through the wrong lens here would not pass on the linear case's tolerance.
TEST(RenderConsumerViewDist, FisheyeCircleIsCentredOnTheOpticalAxisAtTheProjectedRadius) {
  const RenderConfig cfg = MakeConfig({ Line(60.0f, 1.0f, 1.0f, 0.0f, 0.0f) }, LensParam::kFisheyeEqualArea, 180.0f);
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  const auto& masks = rc.ViewDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);

  const lm_proj::ProjParams p = ProjOf(cfg);
  const std::array<float, 3> axis = AxisOf(cfg);
  const annotation::CanvasPoint axis_px = Project(p, axis);
  ASSERT_TRUE(axis_px.valid);
  ASSERT_NEAR(axis_px.px, kW / 2.0f, 0.51f);
  ASSERT_NEAR(axis_px.py, kH / 2.0f, 0.51f);

  const annotation::CanvasPoint edge_px = Project(p, OffAxisTowardsZenith(axis, 60.0f));
  ASSERT_TRUE(edge_px.valid);
  const float expect_r = std::hypot(edge_px.px - axis_px.px, edge_px.py - axis_px.py);
  // The equal-area radius is 2f*sin(30 deg) = f; a linear projection of the same angle would put it
  // at f*tan(60 deg) = 1.73f, off the canvas at this fov. Guard that the two laws really separate.
  const float linear_r =
      expect_r * std::tan(60.0f * math::kDegreeToRad) / (2.0f * std::sin(30.0f * math::kDegreeToRad));
  ASSERT_GT(std::abs(linear_r - expect_r), 10.0f);

  const RingExtent e = ExtentAbout(masks[0], axis_px.px, axis_px.py);
  ASSERT_GT(e.marked, 0u);
  EXPECT_NEAR(e.min_r, expect_r, 3.0f) << "inner edge of the ring";
  EXPECT_NEAR(e.max_r, expect_r, 3.0f) << "outer edge of the ring";
}

// The fov is measured on the SHORT side. On a W > H linear canvas at fov F, the ring at F/2 is
// therefore the circle inscribed in the short side: it touches the top and bottom rows and stops
// short of the left and right columns. A renderer that measured F on the long side or the diagonal
// would draw a smaller ring that touches nothing. This is the second, independent witness to that
// convention beside the projection's own tests; a square canvas could not tell the three apart.
TEST(RenderConsumerViewDist, HalfFovCircleIsInscribedInTheShortSideOfAWideCanvas) {
  constexpr int kWide = 160;
  constexpr float kFov = 90.0f;
  RenderConfig cfg = MakeConfig({ Line(kFov / 2.0f, 1.0f, 1.0f, 0.0f, 0.0f) }, LensParam::kLinear, kFov);
  cfg.resolution_[0] = kWide;
  cfg.resolution_[1] = kH;
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  const auto& masks = rc.ViewDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kWide) * static_cast<size_t>(kH));

  int min_x = kWide;
  int max_x = -1;
  int min_y = kH;
  int max_y = -1;
  size_t marked = 0;
  for (int py = 0; py < kH; ++py) {
    for (int px = 0; px < kWide; ++px) {
      if (masks[0][static_cast<size_t>(py) * static_cast<size_t>(kWide) + static_cast<size_t>(px)] == 0) {
        continue;
      }
      ++marked;
      min_x = std::min(min_x, px);
      max_x = std::max(max_x, px);
      min_y = std::min(min_y, py);
      max_y = std::max(max_y, py);
    }
  }
  ASSERT_GT(marked, 0u);
  // Touches the short side: the ring's vertical extent is the whole canvas height (within the
  // band's own half-width — the level set at exactly the edge lands in the first/last row).
  EXPECT_LE(min_y, 1) << "the ring does not reach the top row — the fov is not measured on the short side";
  EXPECT_GE(max_y, kH - 2) << "the ring does not reach the bottom row";
  // ...and is round, so its horizontal extent equals its vertical one and sits well inside the
  // long side. The diameter of the inscribed circle is kH; measured to the band's outer edge.
  const int width_px = max_x - min_x + 1;
  const int height_px = max_y - min_y + 1;
  EXPECT_NEAR(width_px, height_px, 3);
  EXPECT_GT(min_x, (kWide - kH) / 2 - 3) << "the ring spills past the inscribed circle on the left";
  EXPECT_LT(max_x, kWide - (kWide - kH) / 2 + 3) << "the ring spills past the inscribed circle on the right";
}

// AC2: a shifted lens images the axis off the canvas centre; the ring follows the axis's pixel.
// The same config with and without the shift, compared on where the ring is centred.
TEST(RenderConsumerViewDist, CircleFollowsTheAxisPixelNotTheCanvasCentreUnderLensShift) {
  RenderConfig shifted = MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) });
  shifted.lens_shift_[0] = 18;
  shifted.lens_shift_[1] = -9;
  RenderConsumer rc(shifted, ColorClassTable{}, MakeSun());
  const auto& masks = rc.ViewDistMasksForTest();
  ASSERT_EQ(masks.size(), 1u);

  const lm_proj::ProjParams p = ProjOf(shifted);
  const std::array<float, 3> axis = AxisOf(shifted);
  const annotation::CanvasPoint axis_px = Project(p, axis);
  ASSERT_TRUE(axis_px.valid);
  // Premise: the shift moved the axis pixel far enough from the centre that the two are distinct
  // hypotheses at this ring's band width.
  const float off_centre = std::hypot(axis_px.px - kW / 2.0f, axis_px.py - kH / 2.0f);
  ASSERT_GT(off_centre, 8.0f) << "lens_shift left the axis at the canvas centre; the case cannot tell the two apart";

  const annotation::CanvasPoint edge_px = Project(p, OffAxisTowardsZenith(axis, 22.0f));
  ASSERT_TRUE(edge_px.valid);
  const float expect_r = std::hypot(edge_px.px - axis_px.px, edge_px.py - axis_px.py);

  const RingExtent about_axis = ExtentAbout(masks[0], axis_px.px, axis_px.py);
  ASSERT_GT(about_axis.marked, 0u);
  EXPECT_NEAR(about_axis.min_r, expect_r, 3.0f) << "the ring is a circle about the axis pixel";
  EXPECT_NEAR(about_axis.max_r, expect_r, 3.0f) << "the ring is a circle about the axis pixel";

  // And it is NOT a circle about the canvas centre: seen from there its radius spreads by at least
  // the amount the axis moved.
  const RingExtent about_centre = ExtentAbout(masks[0], kW / 2.0f, kH / 2.0f);
  EXPECT_GT(about_centre.max_r - about_centre.min_r, off_centre)
      << "the ring reads as centred on the canvas, i.e. it ignored the lens shift";
}

TEST(RenderConsumerViewDist, TwoLinesKeepTheirOwnColours) {
  const RenderConfig cfg = MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f), Line(46.0f, 1.0f, 0.0f, 0.0f, 1.0f) });
  RenderConsumer off(MakeConfig({}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const auto& masks = on.ViewDistMasksForTest();
  ASSERT_EQ(masks.size(), 2u) << "one mask per line, not one per category";

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
  EXPECT_GT(red_on_inner, 0u);
  EXPECT_GT(blue_on_outer, 0u);
  EXPECT_EQ(wrong_on_inner, 0u);
  EXPECT_EQ(wrong_on_outer, 0u);
}

TEST(RenderConsumerViewDist, ResetWithPicksUpANewLineListAndTheLabelSwitch) {
  // view_dist_grid_ and view_dist_label_ are appearance fields, so a config that adds a circle or
  // asks for its text mid-run reaches a REUSED consumer.
  RenderConsumer rc(MakeConfig({}), ColorClassTable{}, MakeSun());
  ASSERT_TRUE(rc.ViewDistMasksForTest().empty());
  ASSERT_TRUE(rc.ViewDistLabelsForTest().empty());
  const std::vector<uint8_t> img_off = SnapshotOnce(&rc);

  rc.ResetWith(MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) }), MakeSun());
  ASSERT_EQ(rc.ViewDistMasksForTest().size(), 1u);
  EXPECT_TRUE(rc.ViewDistLabelsForTest().empty()) << "labels are opt-in";
  const std::vector<uint8_t> img_on = SnapshotOnce(&rc);

  size_t tinted = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (IsBlack(img_off, i) && LooksRed(img_on, i)) {
      ++tinted;
    }
  }
  EXPECT_GT(tinted, 0u) << "adding a circle through ResetWith must reach the image without a rebuild";

  RenderConfig with_labels = MakeConfig({ Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) });
  with_labels.view_dist_label_ = true;
  rc.ResetWith(with_labels, MakeSun());
  EXPECT_FALSE(rc.ViewDistLabelsForTest().empty())
      << "flipping the label switch alone must rebuild the anchors — the angle list did not change";
}

// The proposition that separates this family from its twin: the sun is not an input. Where
// test_render_consumer_angular_dist.cpp's ResetWithPicksUpANewSun demands the mask CHANGE, this
// demands it stay byte-identical.
TEST(RenderConsumerViewDist, MovingTheSunLeavesTheCircleWhereItWas) {
  const std::vector<GridLineParam> lines = { Line(22.0f, 1.0f, 1.0f, 0.0f, 0.0f) };
  RenderConsumer rc(MakeConfig(lines), ColorClassTable{}, MakeSun(20.0f));
  ASSERT_EQ(rc.ViewDistMasksForTest().size(), 1u);
  const std::vector<uint8_t> before = rc.ViewDistMasksForTest()[0];
  ASSERT_GT(static_cast<size_t>(std::count(before.begin(), before.end(), uint8_t{ 1 })), 0u);

  rc.ResetWith(MakeConfig(lines), MakeSun(60.0f));
  ASSERT_EQ(rc.ViewDistMasksForTest().size(), 1u);
  EXPECT_EQ(before, rc.ViewDistMasksForTest()[0]) << "the axis did not move, so neither may the circle";
}

}  // namespace
}  // namespace lumice
