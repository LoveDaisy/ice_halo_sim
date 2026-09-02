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
    // EVERY pixel must now be black, the ray-lit one included. This count was kTotalPix - 1 until
    // 478.2, when a zero mask stopped meaning only "withhold the background" and started meaning
    // "this pixel is clipped" — the display clip zeroes the ray energy too. The corruption's
    // signature got stronger rather than weaker: a per-frame rebuild would restore both the
    // background AND the lit pixel, so it is still the smaller count that indicts.
    EXPECT_EQ(black, static_cast<size_t>(kTotalPix))
        << "round " << round
        << ": the mask was zeroed after construction, so every pixel is outside the display clip and none may be "
           "non-black. A smaller count means PostSnapshot did not read the stored mask — either it rebuilt one per "
           "frame (an O(W*H) cost per snapshot) or it recomputes the predicate inline.";
  }

  // Restore it and confirm the signal comes back — otherwise "everything black" above could
  // just as well mean the consumer stopped producing an image at all.
  rc.VisibleMaskForTest() =
      BuildVisibleMask(MakeMaskRenderConfig(), MakeCameraRotation(MakeMaskRenderConfig()), static_cast<float>(kRes));
  EXPECT_EQ(CountBlackPixels(SnapshotOnce(&rc), kTotalPix), black_before);
}

// ==================================================================================================
// `visible` is a DISPLAY clip, not an energy cull — the same statement for all four lens families.
//
// Before 478.2 the four families disagreed about what `visible: upper` meant. The single-lens
// branch of ProjectExitToPixel dropped the ray outright (energy never reached the buffer), while
// rectangular / dual-fisheye / globe had no such branch: their rays landed normally and the mask
// merely withheld the background there, leaving the excluded region lit instead of dark. The two
// halves below pin the two sides of the unified semantics:
//
//   * FOUR-FAMILY: the excluded region must end up BACKGROUND-FREE and ENERGY-FREE on screen.
//   * ENERGY-CONSERVING: the raw XYZ buffer must be IDENTICAL under `upper` and `full`, because
//     nothing is culled any more — the clip happens after the buffer, on the way to pixels.
//
// The two are deliberately opposite in sign: one says the clip is applied, the other says it is
// applied nowhere before the display stage. A change that satisfies only the first (e.g. by
// reinstating the ray cull for all four families) fails the second, and vice versa.
//
// Only `visible: upper` is exercised. `kUpper` / `kLower` are the two mirrored arms of one `if`
// in VisibleByRange (lens_proj_build.hpp) — testing both would cover one predicate twice. If a
// future change gives `lower` logic of its own, this needs a second arm.
// ==================================================================================================

// A frame that images BOTH hemispheres, so `visible: upper` has something to exclude whatever the
// lens is: the camera sits on the horizon and every type below is configured to its full domain.
RenderConfig MakeFamilyCfg(LensParam::LensType type, RenderConfig::VisibleRange visible) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = type;
  // Globe is the one type whose domain is not a field of view at all — it images a sphere seen
  // from kGlobeCameraD radii away, and its scale is `short_pix / 2 / tan(fov/2)`, which collapses
  // to zero as fov approaches 180 (MaxFov caps it at 90, config/json.cpp). 60 deg is what the
  // repo's other globe fixtures use.
  cfg.lens_.fov_ = type == LensParam::kGlobe ? 60.0f : 180.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 32;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 0.0f;
  cfg.visible_ = visible;
  cfg.background_[0] = 0.5f;
  cfg.background_[1] = 0.5f;
  cfg.background_[2] = 0.5f;
  return cfg;
}

const char* LensName(LensParam::LensType type) {
  switch (type) {
    case LensParam::kFisheyeEqualArea:
      return "fisheye_equal_area (single-lens family representative)";
    case LensParam::kRectangular:
      return "rectangular";
    case LensParam::kDualFisheyeEqualArea:
      return "dual_fisheye_equal_area";
    case LensParam::kGlobe:
      return "globe";
    default:
      return "<unexpected lens type>";
  }
}

// One pixel that `full` images but `upper` excludes, together with the world direction a ray must
// travel to land on it. Derived from the projection's OWN inverse (PixelToWorld, the function
// BuildVisibleMask itself calls) rather than from a hand-picked direction, so the fixture cannot
// silently stop covering the excluded region when a lens's domain changes.
struct ExcludedPixel {
  int index;
  float dir[3];
  bool found;
};

ExcludedPixel FindPixelExcludedByVisibleRange(LensParam::LensType type) {
  const RenderConfig full_cfg = MakeFamilyCfg(type, RenderConfig::kFull);
  const RenderConfig upper_cfg = MakeFamilyCfg(type, RenderConfig::kUpper);
  const auto short_pix = static_cast<float>(std::min(full_cfg.resolution_[0], full_cfg.resolution_[1]));
  const Rotation rot = MakeCameraRotation(full_cfg);
  const lm_proj::ProjParams params = BuildProjParams(full_cfg, rot, short_pix);

  const auto full_mask = BuildVisibleMask(full_cfg, rot, short_pix);
  const auto upper_mask = BuildVisibleMask(upper_cfg, rot, short_pix);
  const int width = full_cfg.resolution_[0];
  for (size_t i = 0; i < full_mask.size(); ++i) {
    if (full_mask[i] == 0 || upper_mask[i] != 0) {
      continue;
    }
    const auto px = static_cast<int>(i % static_cast<size_t>(width));
    const auto py = static_cast<int>(i / static_cast<size_t>(width));
    const mask_detail::MaskDir dir = mask_detail::PixelToWorld(full_cfg, params, rot, px, py);
    if (!dir.valid) {
      continue;
    }
    // The direction PixelToWorld returns is already the RAY TRAVEL direction (it inverts
    // ProjectExitToPixel's own `c = R^T * (-w)` step), so it can be fed to the consumer as-is.
    return { static_cast<int>(i), { dir.x, dir.y, dir.z }, true };
  }
  return { -1, { 0.0f, 0.0f, 0.0f }, false };
}

SimData MakeOneRayBatchTowards(const float dir[3]) {
  SimData data;
  data.curr_wl_ = 550.0f;
  data.outgoing_d_ = { dir[0], dir[1], dir[2] };
  data.outgoing_w_ = { 1.0f };
  return data;
}

const uint8_t* SnapshotOnceWith(RenderConsumer* rc, const SimData& batch) {
  SimData data = batch;
  rc->Consume(data);
  rc->PrepareSnapshot();
  rc->PostSnapshot();
  auto result = rc->GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  return rr != nullptr ? rr->img_buffer_ : nullptr;
}

class VisibleIsADisplayClip : public ::testing::TestWithParam<LensParam::LensType> {};

INSTANTIATE_TEST_SUITE_P(AllFourLensFamilies, VisibleIsADisplayClip,
                         ::testing::Values(LensParam::kFisheyeEqualArea, LensParam::kRectangular,
                                           LensParam::kDualFisheyeEqualArea, LensParam::kGlobe));

TEST_P(VisibleIsADisplayClip, ExcludedRegionShowsNeitherBackgroundNorRayEnergy) {
  const LensParam::LensType type = GetParam();
  const ExcludedPixel target = FindPixelExcludedByVisibleRange(type);
  ASSERT_TRUE(target.found) << LensName(type)
                            << ": no pixel is imaged under `full` and excluded under `upper`, so this fixture "
                               "cannot say anything about the clip. Fix the fixture, not the assertion.";

  RenderConfig::VisibleRange visible = RenderConfig::kUpper;
  RenderConsumer rc(MakeFamilyCfg(type, visible), ColorClassTable{});
  const uint8_t* img = SnapshotOnceWith(&rc, MakeOneRayBatchTowards(target.dir));
  ASSERT_NE(img, nullptr) << LensName(type) << ": no image produced";

  // WHERE the energy landed is read back from the raw buffer rather than assumed to be
  // `target.index`. The mask samples pixel centres (px + 0.5 - res/2) while the forward binning is
  // floor(v + 0.5) about res/2 — a half-pixel offset that is subtask 478.3's subject, not this
  // one's. Locating the lit pixel empirically keeps this test measuring the display clip and not
  // that offset.
  const RawXyzResult raw = rc.GetRawXyzResult();
  ASSERT_NE(raw.xyz_buffer_, nullptr) << LensName(type) << ": no raw buffer";
  const auto total_pix = static_cast<size_t>(raw.img_width_) * static_cast<size_t>(raw.img_height_);
  size_t lit = total_pix;
  for (size_t i = 0; i < total_pix; ++i) {
    if (raw.xyz_buffer_[i * 3 + 1] > 0.0f) {
      lit = i;
      break;
    }
  }
  ASSERT_LT(lit, total_pix) << LensName(type)
                            << ": the ray deposited no energy at all under `visible: upper`, so the assertion below "
                               "would hold for the wrong reason. Under the unified semantics the energy must land — "
                               "it is the DISPLAY that drops it.";
  const auto& mask = rc.VisibleMaskForTest();
  ASSERT_EQ(mask.size(), total_pix);
  ASSERT_EQ(mask[lit], 0u) << LensName(type) << ": the lit pixel " << lit
                           << " is inside the visible region, so this run says nothing about the excluded one. The "
                              "fixture aims a ray at an excluded pixel; if binning moved it across the boundary, "
                              "widen the excluded region rather than weakening the check.";

  const int r = img[lit * 3];
  const int g = img[lit * 3 + 1];
  const int b = img[lit * 3 + 2];
  EXPECT_EQ(r, 0) << LensName(type) << ": pixel " << lit << " is (" << r << ", " << g << ", " << b
                  << "). `visible: upper` excludes it, so it must carry neither background nor ray energy. A "
                     "non-zero value here is ray energy leaking through the display clip.";
  EXPECT_EQ(g, 0) << LensName(type) << ": green channel of the excluded pixel";
  EXPECT_EQ(b, 0) << LensName(type) << ": blue channel of the excluded pixel";
}

TEST_P(VisibleIsADisplayClip, RawEnergyIsIdenticalUnderUpperAndFull) {
  const LensParam::LensType type = GetParam();
  const ExcludedPixel target = FindPixelExcludedByVisibleRange(type);
  ASSERT_TRUE(target.found) << LensName(type) << ": fixture found no excluded pixel";
  const SimData batch = MakeOneRayBatchTowards(target.dir);

  const auto accumulate = [&](RenderConfig::VisibleRange visible) {
    RenderConsumer rc(MakeFamilyCfg(type, visible), ColorClassTable{});
    SnapshotOnceWith(&rc, batch);
    const RawXyzResult raw = rc.GetRawXyzResult();
    double sum_y = 0.0;
    for (size_t i = 0; i < static_cast<size_t>(raw.img_width_) * static_cast<size_t>(raw.img_height_); ++i) {
      sum_y += raw.xyz_buffer_[i * 3 + 1];
    }
    return sum_y;
  };

  const double upper_y = accumulate(RenderConfig::kUpper);
  const double full_y = accumulate(RenderConfig::kFull);

  EXPECT_GT(full_y, 0.0) << LensName(type)
                         << ": the ray landed nowhere even under `full`, so the comparison below is vacuous";
  EXPECT_DOUBLE_EQ(upper_y, full_y)
      << LensName(type) << ": total landed energy under `visible: upper` (" << upper_y << ") differs from `full` ("
      << full_y
      << "). `visible` is a display clip: it must not decide whether a ray's energy reaches the buffer at all.";
}

}  // namespace
}  // namespace lumice
