// RenderConsumer's side of the celestial-horizon annotation: that the pixels the mask marks are
// the pixels that come out tinted, that `grid.outline` gates the PAINTING rather than the mask,
// and that turning the flag on mid-run actually reaches the image.
//
// The mask itself is pinned analytically elsewhere (golden-analytic/core/
// test_celestial_outline_mask.cpp); what cannot be seen from there is whether the consumer ever
// reads it. That is this file's question, and it is asked by differencing two snapshots of the
// SAME consumer config — one with the flag off, one on — so that the ray energy in the frame
// cancels out and the only thing left in the difference is the annotation.
//
// The fixture is the configuration the annotation is most likely to be lost in: `visible: upper`,
// where the horizon is the very edge of the sky being drawn. A blend applied before the visible
// mask, or a line whose width was derived from the drawable pixels rather than the imaged ones,
// shows up here as an empty difference.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"

namespace lumice {
namespace {

constexpr int kW = 64;
constexpr int kH = 48;
constexpr int kTotalPix = kW * kH;

// A frame wide enough to hold BOTH the horizon and the zenith away from its edges: a 120 deg
// vertical FOV centred 45 deg up spans altitudes [-15, 105]. The horizon is what the annotation
// draws on; the zenith is where the one ray below lands, and it has to land somewhere inside the
// frame or PostSnapshot takes its zero-intensity early-out and every pixel comes back black —
// which would make this whole file pass vacuously in the reference arm and fail in the other.
// Background stays black, so a tinted pixel is unambiguous.
RenderConfig MakeOutlineConfig(bool outline) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kLinear;
  cfg.lens_.fov_ = 120.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = 45.0f;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.celestial_outline_ = outline;
  return cfg;
}

// One ray straight up. Only rays that LAND in the frame reach total_intensity_, so this is what
// gets PostSnapshot past its zero-intensity early-out; the fixture's view is chosen to contain the
// zenith for that reason. Which pixel it lights does not matter — the same pixel lights in both
// arms and the difference removes it.
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
  rc->PrepareSnapshot();
  rc->PostSnapshot();
  auto result = rc->GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  if (rr == nullptr || rr->img_buffer_ == nullptr) {
    return {};
  }
  return std::vector<uint8_t>(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
}

// The annotation's colour is a red at 60% over whatever is under it (render.cpp's kOutlineSrgb /
// kOutlineAlpha). Over the black background of this fixture that is a strongly red pixel; the
// test asks only for "unmistakably red", not for the exact triple, so that a future change to the
// constant does not have to come here.
bool LooksLikeTheOutline(const std::vector<uint8_t>& img, int i) {
  const int r = img[static_cast<size_t>(i) * 3];
  const int g = img[static_cast<size_t>(i) * 3 + 1];
  const int b = img[static_cast<size_t>(i) * 3 + 2];
  return r >= 120 && r > g + 60 && r > b + 60;
}

bool IsBlack(const std::vector<uint8_t>& img, int i) {
  return img[static_cast<size_t>(i) * 3] == 0 && img[static_cast<size_t>(i) * 3 + 1] == 0 &&
         img[static_cast<size_t>(i) * 3 + 2] == 0;
}

TEST(RenderConsumerCelestialOutline, PaintedPixelsAreExactlyTheMaskedOnes) {
  RenderConsumer off(MakeOutlineConfig(false), ColorClassTable{});
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  RenderConsumer on(MakeOutlineConfig(true), ColorClassTable{});
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const std::vector<uint8_t>& mask = on.CelestialOutlineMaskForTest();
  ASSERT_EQ(mask.size(), static_cast<size_t>(kTotalPix));
  const size_t marked = static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
  ASSERT_GT(marked, 0u) << "an empty mask would make every assertion below vacuous — this fixture puts the horizon "
                           "across the middle of the frame";
  ASSERT_LT(marked, static_cast<size_t>(kTotalPix) / 4)
      << "the mask must be a line, not a region, or 'painted exactly there' means little";

  // Only pixels the reference arm left black carry an unambiguous verdict; a pixel already lit by
  // the ray is excluded from both directions of the iff rather than guessed at.
  size_t checked_on = 0;
  size_t missing = 0;
  size_t stray = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (!IsBlack(img_off, i)) {
      continue;
    }
    if (mask[static_cast<size_t>(i)] != 0) {
      ++checked_on;
      if (!LooksLikeTheOutline(img_on, i)) {
        ++missing;
      }
    } else if (!IsBlack(img_on, i)) {
      ++stray;
    }
  }
  EXPECT_GT(checked_on, 0u) << "the mask and the ray-lit pixels must not be the same set";
  EXPECT_EQ(missing, 0u) << "every masked pixel over black background must come out tinted";
  EXPECT_EQ(stray, 0u) << "no pixel outside the mask may change colour when the annotation is switched on";
}

TEST(RenderConsumerCelestialOutline, TheFlagGatesThePaintingNotTheMask) {
  RenderConsumer rc(MakeOutlineConfig(false), ColorClassTable{});

  // The mask is built at construction whatever the flag says. That is deliberate: the flag is on
  // the appearance-only side of NeedsRebuild, so it can be turned on through ResetWith without a
  // new consumer, and a mask built only for the construction-time value would be missing exactly
  // then.
  const std::vector<uint8_t>& mask = rc.CelestialOutlineMaskForTest();
  ASSERT_EQ(mask.size(), static_cast<size_t>(kTotalPix));
  const size_t marked = static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
  EXPECT_GT(marked, 0u) << "the mask must be built even with grid.outline off";

  const std::vector<uint8_t> img_off = SnapshotOnce(&rc);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);
  size_t tinted_off = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (LooksLikeTheOutline(img_off, i)) {
      ++tinted_off;
    }
  }
  EXPECT_EQ(tinted_off, 0u) << "grid.outline is off, so nothing may be drawn";

  // Same consumer, flag flipped through the appearance-only path.
  rc.ResetWith(MakeOutlineConfig(true));
  const std::vector<uint8_t> img_on = SnapshotOnce(&rc);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);
  size_t tinted_on = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (LooksLikeTheOutline(img_on, i)) {
      ++tinted_on;
    }
  }
  EXPECT_GT(tinted_on, 0u) << "turning grid.outline on through ResetWith must reach the image without a rebuild";
}

}  // namespace
}  // namespace lumice
