// RenderConsumer's side of the zenith / nadir ring markers (grid.zenith_nadir): that a ring lands
// where core says the direction lands, that it is the radius and colour the config names, and —
// the case this file exists for — that a marker whose direction is NOT on the canvas draws
// nothing at all.
//
// Sibling of test_render_consumer_grid.cpp and asks its questions the same way: difference two
// snapshots of the SAME consumer config, one with the annotation and one without, so the ray
// energy cancels and only the annotation is left. What differs from that file is the shape of the
// annotation, and it is why this is a separate one:
//
//   - a marker is a POINT plus a radius, not a level set, so there is no mask to compare the
//     painted pixels against. The oracle is the geometry itself: every painted pixel must sit one
//     ring-radius away from the point core reported;
//   - zenith and nadir are opposite world directions, so a view that is not full-sky has one of
//     them off the canvas. A default-constructed CanvasPoint sits at (0, 0), so a `valid` bit that
//     is checked once for the PAIR rather than once per point draws a ghost ring in the corner.
//     SingleSidedViewDrawsExactlyOneRing is the case that fails on.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"
#include "support/render_anchor.hpp"

namespace lumice {
namespace {

constexpr int kW = 128;
constexpr int kH = 64;
constexpr int kTotalPix = kW * kH;

constexpr float kSunAltitude = 45.0f;
SunParam MakeSun() {
  return SunParam{ kSunAltitude, 0.0f, 0.5f };
}

// The half-width the renderer draws the ring at (server/render.cpp kMarkerHalfWidthPx). Restated
// rather than shared because the assertions below are about the ring a viewer sees, and a change
// to the renderer's thickness should have to be acknowledged here.
constexpr float kRingHalfWidthPx = 1.5f;

// A dual equal-area fisheye over the full sphere: the two poles land at the CENTRE of one disc
// each, half a canvas apart. The single-lens family cannot serve this case at all — it culls
// everything past theta = 90 deg from the view axis whatever the FOV says, so one pole is always
// off the canvas, which is what the single-sided fixture below is for. Background stays black so a
// tinted pixel is unambiguous.
RenderConfig MakeBothVisibleConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kDualFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.visible_ = RenderConfig::kFull;
  return cfg;
}

// The ordinary single-lens case: a 180 deg fisheye pointed straight up over the upper hemisphere.
// The zenith is at the centre of the frame; the nadir is both outside the lens's domain and in the
// hemisphere `visible` excludes, so core reports it as a miss.
RenderConfig MakeSingleSidedConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = 90.0f;
  cfg.visible_ = RenderConfig::kUpper;
  return cfg;
}

RenderConfig WithMarker(RenderConfig cfg, float radius_px, float opacity, float r, float g, float b) {
  cfg.zenith_nadir_.enabled_ = true;
  cfg.zenith_nadir_.radius_px_ = radius_px;
  cfg.zenith_nadir_.opacity_ = opacity;
  cfg.zenith_nadir_.color_[0] = r;
  cfg.zenith_nadir_.color_[1] = g;
  cfg.zenith_nadir_.color_[2] = b;
  return cfg;
}

// One ray straight up: only rays that LAND reach total_intensity_, and PostSnapshot takes a
// zero-intensity early-out that would make every assertion below vacuous.
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

float DistanceTo(const annotation::CanvasPoint& p, int i) {
  const auto px = static_cast<float>(i % kW);
  const auto py = static_cast<float>(i / kW);
  return std::hypot(px - p.px, py - p.py);
}

// Every pixel the annotation turned red, over what was black background before it.
std::vector<int> RedPixels(const std::vector<uint8_t>& img_off, const std::vector<uint8_t>& img_on) {
  std::vector<int> out;
  for (int i = 0; i < kTotalPix; ++i) {
    if (IsBlack(img_off, i) && LooksRed(img_on, i)) {
      out.push_back(i);
    }
  }
  return out;
}

TEST(RenderConsumerZenithNadir, DisabledByDefaultChangesNoPixel) {
  // The default is off, and off must be byte-identical to a build that had never heard of the
  // field — the same property test_render_consumer_post_snapshot_fusion.cpp holds the whole loop
  // to. A marker blended at alpha 0 would pass a "looks the same" check and fail this one.
  RenderConsumer a(MakeBothVisibleConfig(), ColorClassTable{}, MakeSun());
  RenderConsumer b(MakeBothVisibleConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_a = SnapshotOnce(&a);
  const std::vector<uint8_t> img_b = SnapshotOnce(&b);
  ASSERT_EQ(img_a.size(), static_cast<size_t>(kTotalPix) * 3);
  EXPECT_EQ(img_a, img_b);
  EXPECT_FALSE(MakeBothVisibleConfig().zenith_nadir_.enabled_) << "the field must be opt-in";
}

TEST(RenderConsumerZenithNadir, BothVisiblePaintsTwoRingsOnTheReportedPoints) {
  RenderConsumer off(MakeBothVisibleConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  constexpr float kRadius = 12.0f;
  RenderConsumer on(WithMarker(MakeBothVisibleConfig(), kRadius, 1.0f, 1.0f, 0.0f, 0.0f), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const annotation::CanvasPoint& zp = on.ZenithPointForTest();
  const annotation::CanvasPoint& np = on.NadirPointForTest();
  ASSERT_TRUE(zp.valid) << "the fixture must actually image the zenith";
  ASSERT_TRUE(np.valid) << "the fixture must actually image the nadir";
  // Opposite sides of the frame, so "two rings" is a statement about two places and not about one
  // point counted twice.
  ASSERT_GT(std::hypot(zp.px - np.px, zp.py - np.py), 4.0f * kRadius);

  const std::vector<int> red = RedPixels(img_off, img_on);
  ASSERT_FALSE(red.empty()) << "the markers must reach the image";

  size_t near_zenith = 0;
  size_t near_nadir = 0;
  size_t stray = 0;
  for (const int i : red) {
    const float dz = std::fabs(DistanceTo(zp, i) - kRadius);
    const float dn = std::fabs(DistanceTo(np, i) - kRadius);
    if (dz < kRingHalfWidthPx) {
      ++near_zenith;
    } else if (dn < kRingHalfWidthPx) {
      ++near_nadir;
    } else {
      ++stray;
    }
  }
  EXPECT_EQ(stray, 0u) << "every painted pixel must lie one ring-radius from one of the two points";
  // A circle of radius 12 at a 3 px thickness holds ~2*pi*12*3 pixels; the bound only has to
  // separate "a ring" from "a dot" and from "nothing".
  EXPECT_GT(near_zenith, 40u) << "the zenith ring must be a ring, not a handful of pixels";
  EXPECT_GT(near_nadir, 40u) << "the nadir ring must be a ring, not a handful of pixels";
}

TEST(RenderConsumerZenithNadir, SingleSidedViewDrawsExactlyOneRing) {
  // The defect this file exists for: zenith and nadir are opposite directions, so a single-lens
  // view images one of them and misses the other. Testing `valid` once for the PAIR — or not at
  // all — puts a ring at the invalid point's default coordinates, (0, 0), which is a corner of
  // the canvas and therefore a QUARTER ring rather than an obviously wrong full one.
  RenderConsumer off(MakeSingleSidedConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  constexpr float kRadius = 10.0f;
  RenderConsumer on(WithMarker(MakeSingleSidedConfig(), kRadius, 1.0f, 1.0f, 0.0f, 0.0f), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);

  const annotation::CanvasPoint& zp = on.ZenithPointForTest();
  const annotation::CanvasPoint& np = on.NadirPointForTest();
  ASSERT_TRUE(zp.valid) << "the fixture must image the zenith";
  ASSERT_FALSE(np.valid) << "the fixture must NOT image the nadir — otherwise this case tests nothing";

  const std::vector<int> red = RedPixels(img_off, img_on);
  ASSERT_FALSE(red.empty()) << "the visible marker must still be drawn";

  size_t on_zenith_ring = 0;
  size_t elsewhere = 0;
  for (const int i : red) {
    if (std::fabs(DistanceTo(zp, i) - kRadius) < kRingHalfWidthPx) {
      ++on_zenith_ring;
    } else {
      ++elsewhere;
    }
  }
  EXPECT_GT(on_zenith_ring, 30u) << "the zenith ring must be drawn";
  EXPECT_EQ(elsewhere, 0u) << "the invisible nadir must not draw a ghost ring anywhere on the canvas";
}

TEST(RenderConsumerZenithNadir, FrontClipRemovesTheMarkerBehindTheCamera) {
  // The wiring check for RenderConfig::front_ -> annotation::Request::view.front
  // (render.cpp MakeMaskRequest). It is worth its own case because a miss here is INVISIBLE to the
  // mask tests: BuildVisibleMask would clip the background correctly while every annotation kept
  // drawing over the clipped-away half, i.e. rings floating on black. Zenith and nadir are the
  // clearest probe because they are opposite directions, so the clip has to keep exactly one.
  for (const float el : { 45.0f, -45.0f }) {
    RenderConfig cfg = MakeBothVisibleConfig();
    cfg.view_.el_ = el;

    RenderConsumer unclipped(WithMarker(cfg, 10.0f, 1.0f, 1.0f, 0.0f, 0.0f), ColorClassTable{}, MakeSun());
    if (!unclipped.ZenithPointForTest().valid || !unclipped.NadirPointForTest().valid) {
      // Non-fatal: the up-looking row must not swallow the down-looking one.
      ADD_FAILURE() << "el " << el << ": the fixture must image both poles, or the clip proves nothing";
      continue;
    }

    cfg.front_ = true;
    RenderConsumer clipped(WithMarker(cfg, 10.0f, 1.0f, 1.0f, 0.0f, 0.0f), ColorClassTable{}, MakeSun());
    // Looking up keeps the zenith and drops the nadir; looking down, the other way round. Stated
    // as "the pole the camera faces survives" rather than as a fixed pair, so the case cannot pass
    // by clipping the wrong one.
    const bool looking_up = el > 0.0f;
    EXPECT_EQ(clipped.ZenithPointForTest().valid, looking_up) << "el " << el << ": wrong verdict for the zenith";
    EXPECT_EQ(clipped.NadirPointForTest().valid, !looking_up) << "el " << el << ": wrong verdict for the nadir";
  }
}

TEST(RenderConsumerZenithNadir, RadiusIsTheConfiguredOne) {
  // Two consumers differing only in radius must paint two different circles. A hardcoded radius
  // reads as correct on any single-radius fixture.
  RenderConsumer off(MakeBothVisibleConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);

  for (const float radius : { 6.0f, 20.0f }) {
    RenderConsumer on(WithMarker(MakeBothVisibleConfig(), radius, 1.0f, 1.0f, 0.0f, 0.0f), ColorClassTable{},
                      MakeSun());
    const std::vector<uint8_t> img_on = SnapshotOnce(&on);
    const annotation::CanvasPoint& zp = on.ZenithPointForTest();
    const std::vector<int> red = RedPixels(img_off, img_on);
    // Non-fatal: one radius failing must not take the other radius's verdict with it, and WHICH
    // radius broke is half of what the failure says.
    if (red.empty()) {
      ADD_FAILURE() << "radius " << radius << ": the markers must reach the image";
      continue;
    }

    size_t matching = 0;
    for (const int i : red) {
      if (std::fabs(DistanceTo(zp, i) - radius) < kRingHalfWidthPx ||
          std::fabs(DistanceTo(on.NadirPointForTest(), i) - radius) < kRingHalfWidthPx) {
        ++matching;
      }
    }
    EXPECT_EQ(matching, red.size()) << "radius " << radius << ": a painted pixel sits at a different radius";
  }
}

TEST(RenderConsumerZenithNadir, ZeroOpacityDrawsNothing) {
  // The appearance is honoured on the way out too: the points are still computed (they are a
  // layout-derived quantity), but a fully transparent marker must not reach a pixel.
  RenderConsumer off(MakeBothVisibleConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  RenderConsumer on(WithMarker(MakeBothVisibleConfig(), 12.0f, 0.0f, 1.0f, 0.0f, 0.0f), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), img_off.size());
  EXPECT_EQ(img_on, img_off);
}

TEST(RenderConsumerZenithNadir, ResetWithPicksUpTheSwitch) {
  // Neither the switch nor the radius takes part in NeedsRebuild, so a config that turns the
  // markers on mid-run reaches a REUSED consumer. Points computed only when the switch happened to
  // be on at construction would leave that consumer drawing nothing.
  RenderConsumer rc(MakeBothVisibleConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&rc);
  ASSERT_TRUE(rc.ZenithPointForTest().valid) << "the points are layout-derived and must exist before the switch does";

  rc.ResetWith(WithMarker(MakeBothVisibleConfig(), 12.0f, 1.0f, 1.0f, 0.0f, 0.0f), MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&rc);
  EXPECT_FALSE(RedPixels(img_off, img_on).empty())
      << "turning the markers on through ResetWith must reach the image without a rebuild";

  rc.ResetWith(MakeBothVisibleConfig(), MakeSun());
  const std::vector<uint8_t> img_off_again = SnapshotOnce(&rc);
  EXPECT_EQ(img_off_again, img_off) << "turning them back off must restore the unannotated image";
}

}  // namespace
}  // namespace lumice
