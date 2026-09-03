// RenderConsumer's side of the coordinate grid (grid.elevation / grid.longitude): that the pixels
// the masks mark are the pixels that come out tinted, that each line keeps its own colour and
// opacity, that a new line list reaches a REUSED consumer, and that the grid is composited UNDER
// the angular-distance circles — the layer order the preview shader also uses.
//
// Sibling of test_render_consumer_angular_dist.cpp and asks its questions the same way: difference
// two snapshots of the SAME consumer config, one with the annotation and one without, so the ray
// energy cancels and only the annotation is left. Two things differ from that file, and they are
// why this is a separate one:
//
//   - these two families are fixed in the celestial frame, so unlike the circles they do NOT move
//     with the sun. There is deliberately no "ResetWith picks up a new sun" case here; the mask
//     cache is keyed on the angle list alone, and a sun term in that key would be dead weight;
//   - a parallel and a meridian CROSS, which the two circles of that file never do. The layer
//     order between the grid and the circles is therefore observable, and LayerOrderPutsTheGrid-
//     UnderTheCircles is the case a wrongly ordered composite fails.

#include <gtest/gtest.h>

#include <algorithm>
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

constexpr int kW = 96;
constexpr int kH = 96;
constexpr int kTotalPix = kW * kH;

constexpr float kSunAltitude = 45.0f;
SunParam MakeSun() {
  return SunParam{ kSunAltitude, 0.0f, 0.5f };
}

// A 120 deg linear frame centred 45 deg up and due 0 azimuth: it holds the 30 deg and 60 deg
// parallels and the 0 deg meridian, which cross inside the frame. Background stays black so a
// tinted pixel is unambiguous.
RenderConfig MakeConfig(const std::vector<GridLineParam>& elevation, const std::vector<GridLineParam>& longitude,
                        const std::vector<GridLineParam>& angular_dist = {}) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kLinear;
  cfg.lens_.fov_ = 120.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = kSunAltitude;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.elevation_grid_ = elevation;
  cfg.longitude_grid_ = longitude;
  cfg.angular_dist_grid_ = angular_dist;
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

bool LooksBlue(const std::vector<uint8_t>& img, int i) {
  const int r = img[static_cast<size_t>(i) * 3];
  const int g = img[static_cast<size_t>(i) * 3 + 1];
  const int b = img[static_cast<size_t>(i) * 3 + 2];
  return b >= 120 && b > r + 60 && b > g + 60;
}

size_t CountMarked(const std::vector<uint8_t>& mask) {
  return static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
}

// The shared body of the two "painted pixels are exactly the masked ones" cases below — one per
// family. Written once because the proposition is identical and only the config field differs;
// `masks` is the family's own mask list, read back off the consumer under test.
void ExpectMaskDrivesTheImage(const RenderConfig& cfg_on, const std::vector<std::vector<uint8_t>>& masks,
                              const std::vector<uint8_t>& img_off, const std::vector<uint8_t>& img_on) {
  ASSERT_EQ(masks.size(), 1u);
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));
  const size_t marked = CountMarked(masks[0]);
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
  (void)cfg_on;
}

TEST(RenderConsumerGrid, ElevationPaintedPixelsAreExactlyTheMaskedOnes) {
  RenderConsumer off(MakeConfig({}, {}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  const RenderConfig cfg = MakeConfig({ Line(30.0f, 1.0f, 1.0f, 0.0f, 0.0f) }, {});
  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  ExpectMaskDrivesTheImage(cfg, on.ElevationMasksForTest(), img_off, img_on);
  EXPECT_TRUE(on.LongitudeMasksForTest().empty()) << "an empty meridian list must build no masks";
}

TEST(RenderConsumerGrid, LongitudePaintedPixelsAreExactlyTheMaskedOnes) {
  RenderConsumer off(MakeConfig({}, {}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  const RenderConfig cfg = MakeConfig({}, { Line(0.0f, 1.0f, 1.0f, 0.0f, 0.0f) });
  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  ExpectMaskDrivesTheImage(cfg, on.LongitudeMasksForTest(), img_off, img_on);
  EXPECT_TRUE(on.ElevationMasksForTest().empty()) << "an empty parallel list must build no masks";
}

TEST(RenderConsumerGrid, ThePairOfFamiliesProduceDifferentCurves) {
  // The mistake a copy-paste between the two branches makes: filling Request::elevation_deg for
  // both families, or reading Overlay::elevation for both, yields the SAME mask from the same
  // angle. 30 deg elevation and 30 deg azimuth are different curves on any frame.
  RenderConsumer rc(MakeConfig({ Line(30.0f, 1.0f, 1.0f, 0.0f, 0.0f) }, { Line(30.0f, 1.0f, 0.0f, 0.0f, 1.0f) }),
                    ColorClassTable{}, MakeSun());
  ASSERT_EQ(rc.ElevationMasksForTest().size(), 1u);
  ASSERT_EQ(rc.LongitudeMasksForTest().size(), 1u);
  ASSERT_GT(CountMarked(rc.ElevationMasksForTest()[0]), 0u);
  ASSERT_GT(CountMarked(rc.LongitudeMasksForTest()[0]), 0u);
  EXPECT_NE(rc.ElevationMasksForTest()[0], rc.LongitudeMasksForTest()[0]);
}

TEST(RenderConsumerGrid, TwoLinesKeepTheirOwnColours) {
  // The proposition a single per-category union mask cannot satisfy: the 30 deg parallel comes out
  // red and the 60 deg one blue. Batching both angles into one ComputeOverlay call would return
  // one mask covering both curves, and whichever colour the code picked would paint both.
  const RenderConfig cfg = MakeConfig({ Line(30.0f, 1.0f, 1.0f, 0.0f, 0.0f), Line(60.0f, 1.0f, 0.0f, 0.0f, 1.0f) }, {});
  RenderConsumer off(MakeConfig({}, {}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const auto& masks = on.ElevationMasksForTest();
  ASSERT_EQ(masks.size(), 2u) << "one mask per line, not one per category";
  ASSERT_EQ(masks[0].size(), static_cast<size_t>(kTotalPix));
  ASSERT_EQ(masks[1].size(), static_cast<size_t>(kTotalPix));

  size_t red_on_lower = 0;
  size_t wrong_on_lower = 0;
  size_t blue_on_upper = 0;
  size_t wrong_on_upper = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (!IsBlack(img_off, i)) {
      continue;
    }
    const bool lower = masks[0][static_cast<size_t>(i)] != 0;
    const bool upper = masks[1][static_cast<size_t>(i)] != 0;
    if (lower && !upper) {
      LooksRed(img_on, i) ? ++red_on_lower : ++wrong_on_lower;
    } else if (upper && !lower) {
      LooksBlue(img_on, i) ? ++blue_on_upper : ++wrong_on_upper;
    }
  }
  EXPECT_GT(red_on_lower, 0u) << "the 30 deg parallel must exist and be red";
  EXPECT_GT(blue_on_upper, 0u) << "the 60 deg parallel must exist and be blue";
  EXPECT_EQ(wrong_on_lower, 0u) << "no pixel of the 30 deg parallel may take the 60 deg line's colour";
  EXPECT_EQ(wrong_on_upper, 0u) << "no pixel of the 60 deg parallel may take the 30 deg line's colour";
}

TEST(RenderConsumerGrid, ZeroOpacityLineDrawsNothing) {
  // The per-line appearance is honoured on the way OUT of the mask stage too: a mask is still
  // built (the cache key is the angle list), but a fully transparent line must not reach a pixel.
  RenderConsumer off(MakeConfig({}, {}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  RenderConsumer on(MakeConfig({ Line(30.0f, 0.0f, 1.0f, 0.0f, 0.0f) }, {}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), img_off.size());
  EXPECT_EQ(img_on, img_off);
}

TEST(RenderConsumerGrid, ResetWithPicksUpANewLineList) {
  // Neither list takes part in NeedsRebuild, so a config that adds grid lines mid-run reaches a
  // REUSED consumer. Masks that were only ever built in the constructor would leave that consumer
  // drawing nothing.
  RenderConsumer rc(MakeConfig({}, {}), ColorClassTable{}, MakeSun());
  ASSERT_TRUE(rc.ElevationMasksForTest().empty());
  ASSERT_TRUE(rc.LongitudeMasksForTest().empty());
  const std::vector<uint8_t> img_off = SnapshotOnce(&rc);

  rc.ResetWith(MakeConfig({ Line(30.0f, 1.0f, 1.0f, 0.0f, 0.0f) }, { Line(0.0f, 1.0f, 1.0f, 0.0f, 0.0f) }), MakeSun());
  ASSERT_EQ(rc.ElevationMasksForTest().size(), 1u);
  ASSERT_EQ(rc.LongitudeMasksForTest().size(), 1u);
  const std::vector<uint8_t> img_on = SnapshotOnce(&rc);

  size_t tinted = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (IsBlack(img_off, i) && LooksRed(img_on, i)) {
      ++tinted;
    }
  }
  EXPECT_GT(tinted, 0u) << "adding grid lines through ResetWith must reach the image without a rebuild";

  // And the reverse: clearing the lists must clear the masks, not leave the old ones cached.
  rc.ResetWith(MakeConfig({}, {}), MakeSun());
  EXPECT_TRUE(rc.ElevationMasksForTest().empty());
  EXPECT_TRUE(rc.LongitudeMasksForTest().empty());
}

TEST(RenderConsumerGrid, LayerOrderPutsTheGridUnderTheCircles) {
  // Where a grid line and an angular-distance circle cross, the circle wins — the preview shader's
  // own order (grid -> sun circles -> horizon), which the two paths have to agree on. Both lines
  // are fully opaque, so the pixel takes exactly one of the two colours and the order is readable
  // straight off the byte.
  const RenderConfig cfg =
      MakeConfig({ Line(30.0f, 1.0f, 1.0f, 0.0f, 0.0f) }, {}, { Line(15.0f, 1.0f, 0.0f, 0.0f, 1.0f) });
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img = SnapshotOnce(&rc);
  ASSERT_EQ(img.size(), static_cast<size_t>(kTotalPix) * 3);

  ASSERT_EQ(rc.ElevationMasksForTest().size(), 1u);
  ASSERT_EQ(rc.AngularDistMasksForTest().size(), 1u);
  const auto& grid = rc.ElevationMasksForTest()[0];
  const auto& circle = rc.AngularDistMasksForTest()[0];
  ASSERT_EQ(grid.size(), static_cast<size_t>(kTotalPix));
  ASSERT_EQ(circle.size(), static_cast<size_t>(kTotalPix));

  // The sun sits at altitude 45 and the parallel at 30, so the 15 deg circle is tangent to it —
  // the two curves are guaranteed to share pixels on this fixture.
  size_t crossings = 0;
  size_t grid_won = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (grid[static_cast<size_t>(i)] == 0 || circle[static_cast<size_t>(i)] == 0) {
      continue;
    }
    ++crossings;
    if (!LooksBlue(img, i)) {
      ++grid_won;
    }
  }
  ASSERT_GT(crossings, 0u) << "the fixture must actually cross the two curves";
  EXPECT_EQ(grid_won, 0u) << "the angular-distance circle is composited over the grid, not under it";
}

}  // namespace
}  // namespace lumice
