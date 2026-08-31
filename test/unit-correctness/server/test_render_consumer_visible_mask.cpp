// RenderConsumer's side of the render-domain mask: that it is built ONCE, and that
// PostSnapshot reads that one buffer.
//
// "Built once" matters because the mask is a W*H per-pixel derivation with several trig calls
// per pixel: recomputing it every snapshot would put that cost on every frame, for a value that
// cannot change without a consumer rebuild (NeedsRebuild covers all of its inputs). A call counter would be the obvious
// way to check it, but a counter only says how many times the builder ran; it cannot say that the value PostSnapshot
// consumed is the one built at construction. So the test corrupts the stored mask after construction and asserts the
// next PostSnapshot honours the corruption. That fails under either defect: a per-frame rebuild would overwrite the
// corruption (background reappears), and a PostSnapshot that recomputed its own predicate
// would ignore the stored buffer entirely (background never disappears).
//
// The scene is deliberately empty of rays. `snapshot_intensity_ <= 0` makes PostSnapshot
// take its early-out, so every case below feeds one zero-weight-free batch to get past it;
// the image is then pure background, which is exactly the signal being read.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/lens_proj_build.hpp"  // BuildVisibleMask
#include "core/scatter_accum.hpp"    // MakeCameraRotation
#include "server/render.hpp"

namespace lumice {
namespace {

constexpr int kRes = 24;

RenderConfig MakeMaskRenderConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  // 180 deg equal-area on a square canvas: the image circle is inscribed, so the corners
  // are outside the lens's domain and the mask is not all-ones.
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = kRes;
  cfg.resolution_[1] = kRes;
  cfg.view_.el_ = 90.0f;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.background_[0] = 0.5f;
  cfg.background_[1] = 0.5f;
  cfg.background_[2] = 0.5f;
  return cfg;
}

// One ray straight up, so snapshot_intensity_ > 0 and PostSnapshot runs its real loop.
SimData MakeOneRayBatch() {
  SimData data;
  data.curr_wl_ = 550.0f;
  data.outgoing_d_ = { 0.0f, 0.0f, -1.0f };
  data.outgoing_w_ = { 1.0f };
  return data;
}

const uint8_t* SnapshotOnce(RenderConsumer* rc) {
  auto data = MakeOneRayBatch();
  rc->Consume(data);
  rc->PrepareSnapshot();
  rc->PostSnapshot();
  auto result = rc->GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  return rr != nullptr ? rr->img_buffer_ : nullptr;
}

// Fully black pixels — the ones that got neither background nor ray energy. Counting black
// rather than painted keeps the single lit pixel (which is non-black whatever the mask says)
// from blurring the signal.
size_t CountBlackPixels(const uint8_t* img, int total_pix) {
  size_t black = 0;
  for (int i = 0; i < total_pix; ++i) {
    if (img[i * 3] == 0 && img[i * 3 + 1] == 0 && img[i * 3 + 2] == 0) {
      ++black;
    }
  }
  return black;
}

TEST(RenderConsumerVisibleMask, MaskIsSizedToTheFrameAndExcludesTheCorners) {
  RenderConsumer rc(MakeMaskRenderConfig(), ColorClassTable{});
  const auto& mask = rc.VisibleMaskForTest();
  ASSERT_EQ(mask.size(), static_cast<size_t>(kRes) * kRes);
  const size_t on = static_cast<size_t>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
  EXPECT_GT(on, 0u) << "an all-zero mask would make every assertion below vacuous";
  EXPECT_LT(on, mask.size()) << "the inscribed image circle must leave the corners out, or this fixture stopped "
                                "covering what it claims to";
  EXPECT_EQ(mask[0], 0u) << "the top-left corner is outside a 180 deg fisheye's inscribed image circle";
}

TEST(RenderConsumerVisibleMask, PostSnapshotConsumesTheStoredMaskAndNeverRebuildsIt) {
  RenderConsumer rc(MakeMaskRenderConfig(), ColorClassTable{});
  constexpr int kTotalPix = kRes * kRes;

  const size_t black_before = CountBlackPixels(SnapshotOnce(&rc), kTotalPix);
  EXPECT_GT(black_before, 0u) << "the real mask must withhold the background somewhere (the corners), or this fixture "
                                 "stopped covering what it claims to";
  EXPECT_LT(black_before, static_cast<size_t>(kTotalPix) - 1)
      << "the real mask must paint most of the frame, or the corruption below proves nothing";

  // Corrupt the stored mask: nothing may be painted from here on.
  std::fill(rc.VisibleMaskForTest().begin(), rc.VisibleMaskForTest().end(), uint8_t{ 0 });

  // Three more snapshot cycles. Any per-frame rebuild would restore the real mask on the
  // very first of them.
  for (int round = 0; round < 3; ++round) {
    const size_t black = CountBlackPixels(SnapshotOnce(&rc), kTotalPix);
    // Every pixel but the single lit one must now be black.
    EXPECT_EQ(black, static_cast<size_t>(kTotalPix) - 1)
        << "round " << round
        << ": the mask was zeroed after construction, so only the one ray-lit pixel may be non-black. A smaller count "
           "means PostSnapshot did not read the stored mask — either it rebuilt one per frame (an O(W*H) cost per "
           "snapshot) or it recomputes the predicate inline.";
  }

  // Restore it and confirm the signal comes back — otherwise "everything black" above could
  // just as well mean the consumer stopped producing an image at all.
  rc.VisibleMaskForTest() =
      BuildVisibleMask(MakeMaskRenderConfig(), MakeCameraRotation(MakeMaskRenderConfig()), static_cast<float>(kRes));
  EXPECT_EQ(CountBlackPixels(SnapshotOnce(&rc), kTotalPix), black_before);
}

}  // namespace
}  // namespace lumice
