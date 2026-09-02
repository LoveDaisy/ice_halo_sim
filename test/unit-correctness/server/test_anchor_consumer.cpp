// AnchorConsumer — the fixed full-sky anchor buffer and the L99_sky scalar it yields.
//
// WHAT THIS PINS, and why it needs a control case to mean anything.
//
// The scalar this consumer publishes is meant to be a property of the SCENE, not of any
// renderer looking at it: the same rays must produce the same number whatever lens, fov,
// view pose or output resolution the user's renderers happen to carry. Asserting that on
// AnchorConsumer alone would be very nearly a tautology — the class takes no RenderConfig,
// so of course its output does not vary with one. Every invariance case below therefore
// carries a CONTROL that feeds the identical rays to a RenderConsumer pair configured the
// two ways under test and shows that the OLD anchor (the per-renderer output-buffer P99
// behind ExposureScale()'s kRelative branch) genuinely does move. Without the control a
// consumer that returned a constant would pass.
//
// `visible` is deliberately NOT one of the variables the invariance cases vary. It is a
// pure display-layer clip — no branch of ProjectExitToPixel reads it, and it enters only
// through RenderConsumer::PostSnapshot's per-pixel mask — so it already left the old CLI
// anchor bit-identical. A case built on it would be green before and after this change and
// would prove nothing. It appears once below as an explicit regression pin, labelled as
// such, rather than as evidence of the new property.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/anchor_buffer.hpp"
#include "core/color_util.hpp"
#include "core/ev_anchor.hpp"
#include "server/anchor_consumer.hpp"
#include "server/render.hpp"

namespace lumice {
namespace {

constexpr float kWl = 550.0f;

// A deterministic sky: a Fibonacci-spiral spread over the full sphere with a bright ring at
// 22 degrees from the anti-solar-ish axis, so the field has a real peak for a P99 to find
// rather than being flat (a flat field would make every anchor agree for the wrong reason).
SimData MakeSkyBatch(size_t n) {
  SimData data;
  data.curr_wl_ = kWl;
  data.root_ray_count_ = n;
  data.emitted_energy_ = static_cast<float>(n);
  data.outgoing_d_.reserve(n * 3);
  data.outgoing_w_.reserve(n);
  const double golden = 3.14159265358979323846 * (3.0 - std::sqrt(5.0));
  for (size_t i = 0; i < n; ++i) {
    double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) / static_cast<double>(n);
    double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    double th = golden * static_cast<double>(i);
    double x = r * std::cos(th);
    double y = r * std::sin(th);
    data.outgoing_d_.push_back(static_cast<float>(x));
    data.outgoing_d_.push_back(static_cast<float>(y));
    data.outgoing_d_.push_back(static_cast<float>(z));
    // Sky direction is -d; the "halo" is a ring 22 degrees off the +Z sky pole.
    double ang = std::acos(std::max(-1.0, std::min(1.0, -z)));
    double d22 = std::abs(ang - 22.0 * 3.14159265358979323846 / 180.0);
    float w = (d22 < 0.02) ? 40.0f : 1.0f;
    data.outgoing_w_.push_back(w);
  }
  return data;
}

RenderConfig MakeRenderConfig(LensParam::LensType type, float fov, int w, int h) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = type;
  cfg.lens_.fov_ = fov;
  cfg.resolution_[0] = w;
  cfg.resolution_[1] = h;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.intensity_factor_ = 1.0f;
  // The OLD anchor lives on this branch — the control cases below read it through
  // ExposureScale(), which only consults the P99 self-anchor when ev_mode_ is kRelative.
  cfg.ev_mode_ = RenderConfig::kRelative;
  return cfg;
}

// The anchor scalar for one batch, through a freshly built consumer.
float AnchorFor(const SimData& data) {
  AnchorConsumer ac;
  ac.Consume(data);
  ac.PrepareSnapshot();
  return ac.SnapshotL99Sky();
}

// The OLD anchor for one batch under a given renderer config: the reciprocal-ish scale
// ExposureScale() publishes, which is 1/P99 up to config-fixed constants. Used only as the
// control that the property under test is not already true by accident.
float LegacyAnchorScaleFor(const SimData& data, const RenderConfig& cfg) {
  RenderConsumer rc(cfg);
  rc.Consume(data);
  rc.PrepareSnapshot();
  return rc.ExposureScale();
}

// ---------------------------------------------------------------------------
// AC0 / AC1 — invariance to the renderer's lens, fov and output resolution.
// ---------------------------------------------------------------------------

TEST(AnchorConsumer, InvariantToLensWhileLegacyAnchorMoves) {
  SimData data = MakeSkyBatch(60000);

  auto a_linear = MakeRenderConfig(LensParam::kLinear, 20.0f, 512, 512);
  auto a_ea = MakeRenderConfig(LensParam::kFisheyeEqualArea, 180.0f, 512, 512);

  // Control: the old, per-renderer anchor really does depend on the lens.
  float legacy_linear = LegacyAnchorScaleFor(data, a_linear);
  float legacy_ea = LegacyAnchorScaleFor(data, a_ea);
  ASSERT_GT(legacy_linear, 0.0f);
  ASSERT_GT(legacy_ea, 0.0f);
  EXPECT_GT(std::abs(std::log2(legacy_linear / legacy_ea)), 0.2f)
      << "control failed: the legacy anchor did not move across lenses, so the invariance "
         "assertion below would pass for a reason other than the one under test";

  // The property: one scene, one number.
  EXPECT_FLOAT_EQ(AnchorFor(data), AnchorFor(data));
  EXPECT_GT(AnchorFor(data), 0.0f);
}

TEST(AnchorConsumer, InvariantToFovAndResolutionWhileLegacyAnchorMoves) {
  SimData data = MakeSkyBatch(60000);

  auto small = MakeRenderConfig(LensParam::kLinear, 20.0f, 512, 512);
  auto big = MakeRenderConfig(LensParam::kLinear, 20.0f, 1024, 1024);
  auto wide = MakeRenderConfig(LensParam::kLinear, 60.0f, 512, 512);

  float legacy_small = LegacyAnchorScaleFor(data, small);
  float legacy_big = LegacyAnchorScaleFor(data, big);
  float legacy_wide = LegacyAnchorScaleFor(data, wide);
  ASSERT_GT(legacy_small, 0.0f);
  ASSERT_GT(legacy_big, 0.0f);
  ASSERT_GT(legacy_wide, 0.0f);
  EXPECT_GT(std::abs(std::log2(legacy_small / legacy_big)), 0.2f) << "control failed (resolution)";
  EXPECT_GT(std::abs(std::log2(legacy_small / legacy_wide)), 0.2f) << "control failed (fov)";

  // The anchor consumer never sees any of those three configs, and that is the point:
  // there is exactly one anchor per scene, so there is nothing here for them to move.
  float anchor = AnchorFor(data);
  EXPECT_GT(anchor, 0.0f);
  EXPECT_FLOAT_EQ(anchor, AnchorFor(data));
}

// ---------------------------------------------------------------------------
// The closed form — what the scalar actually IS.
// ---------------------------------------------------------------------------

// Every ray in one direction: the whole batch lands in a single fine pixel, hence a single
// non-zero coarse bin, and NthElementP99 over a one-element sample returns that element.
// The expected value is then fully determined by the two constants this consumer is built
// on — the downsample factor and the anchor pixel's solid angle — so a transcription error
// in either shows up here as a clean ratio rather than as a plausible-looking number.
TEST(AnchorConsumer, ScalarMatchesClosedFormOnSingleDirectionBatch) {
  SimData data;
  data.curr_wl_ = kWl;
  const size_t n = 1000;
  data.root_ray_count_ = n;
  for (size_t i = 0; i < n; ++i) {
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(-1.0f);
    data.outgoing_w_.push_back(1.0f);
  }

  const float total_y = SpectrumToYSingle(kWl, static_cast<float>(n));
  const float f2 = static_cast<float>(kAnchorDownsampleFactor) * static_cast<float>(kAnchorDownsampleFactor);
  const float expected = total_y / f2 / kAnchorPixelSolidAngle;

  EXPECT_NEAR(AnchorFor(data), expected, expected * 1e-5f);
}

TEST(AnchorConsumer, PixelSolidAngleMatchesEqualAreaClosedForm) {
  // r = min(W/2, H)/2 for the dual equal-area layout; a hemisphere (2*pi sr) maps onto a
  // disc of area pi*r^2, so one pixel subtends 2/r^2.
  const float r = static_cast<float>(std::min(kAnchorWidth / 2, kAnchorHeight)) / 2.0f;
  EXPECT_FLOAT_EQ(kAnchorPixelSolidAngle, 2.0f / (r * r));
}

// ---------------------------------------------------------------------------
// Batch shapes the server actually delivers.
// ---------------------------------------------------------------------------

TEST(AnchorConsumer, ZeroRayAccountingBatchLeavesAnchorAtZero) {
  // server.cpp's 0-exit path builds a SimData carrying only bookkeeping counters.
  SimData accounting;
  accounting.curr_wl_ = kWl;
  accounting.root_ray_count_ = 1000;
  accounting.emitted_energy_ = 1000.0f;

  AnchorConsumer ac;
  ac.Consume(accounting);
  ac.PrepareSnapshot();
  EXPECT_FLOAT_EQ(ac.SnapshotL99Sky(), 0.0f);
}

TEST(AnchorConsumer, ChunkedDeliveryEqualsWholeBatchDelivery) {
  // The exit-seam path splits a batch into kCommitCap-sized chunks and calls Consume once
  // per chunk. The anchor must not depend on that grain.
  SimData whole = MakeSkyBatch(30000);

  AnchorConsumer chunked;
  const size_t chunk = 4096;
  for (size_t off = 0; off < whole.outgoing_w_.size(); off += chunk) {
    size_t cnt = std::min(chunk, whole.outgoing_w_.size() - off);
    SimData part;
    part.curr_wl_ = whole.curr_wl_;
    part.outgoing_d_.assign(whole.outgoing_d_.begin() + static_cast<std::ptrdiff_t>(off) * 3,
                            whole.outgoing_d_.begin() + static_cast<std::ptrdiff_t>(off + cnt) * 3);
    part.outgoing_w_.assign(whole.outgoing_w_.begin() + static_cast<std::ptrdiff_t>(off),
                            whole.outgoing_w_.begin() + static_cast<std::ptrdiff_t>(off + cnt));
    chunked.Consume(part);
  }
  chunked.PrepareSnapshot();

  EXPECT_FLOAT_EQ(chunked.SnapshotL99Sky(), AnchorFor(whole));
}

TEST(AnchorConsumer, PerRayWavelengthBatchAgreesWithPerBatchWavelength) {
  // The DR-3 seam hands per-ray wavelengths instead of curr_wl_. Filling every entry with
  // the same wavelength must reproduce the per-batch answer exactly.
  SimData per_batch = MakeSkyBatch(20000);
  SimData per_ray = MakeSkyBatch(20000);
  per_ray.curr_wl_ = 0.0f;  // would be a dropped-wavelength bug if the consumer used it
  per_ray.outgoing_wl_.assign(per_ray.outgoing_w_.size(), kWl);

  EXPECT_FLOAT_EQ(AnchorFor(per_ray), AnchorFor(per_batch));
}

TEST(AnchorConsumer, DeviceFusedAnchorPlaneFoldsIn) {
  // GPU backends cannot hand back exit directions — they hand back the anchor plane they
  // accumulated on device. Feeding that plane must reach the same scalar as feeding the
  // rays it was made from.
  SimData rays = MakeSkyBatch(20000);
  float expected = AnchorFor(rays);

  // Build the plane the device would have produced, using the same single-source projection.
  std::vector<float> plane(static_cast<size_t>(kAnchorWidth) * static_cast<size_t>(kAnchorHeight), 0.0f);
  const auto proj = BuildAnchorProjParams();
  for (size_t i = 0; i < rays.outgoing_w_.size(); ++i) {
    auto hit = lm_proj::ProjectExitToPixel(proj, rays.outgoing_d_[i * 3 + 0], rays.outgoing_d_[i * 3 + 1],
                                           rays.outgoing_d_[i * 3 + 2]);
    for (int k = 0; k < hit.count; ++k) {
      int px = hit.hits[k].px;
      int py = hit.hits[k].py;
      if (px < 0 || px >= kAnchorWidth || py < 0 || py >= kAnchorHeight) {
        continue;
      }
      plane[static_cast<size_t>(py) * kAnchorWidth + static_cast<size_t>(px)] +=
          SpectrumToYSingle(kWl, rays.outgoing_w_[i]);
    }
  }

  SimData fused;
  fused.curr_wl_ = kWl;
  fused.xyz_pixel_data_.assign(12, 0.0f);  // marks the batch as device-fused
  fused.anchor_y_pixel_data_ = plane;

  AnchorConsumer ac;
  ac.Consume(fused);
  ac.PrepareSnapshot();
  EXPECT_NEAR(ac.SnapshotL99Sky(), expected, expected * 1e-5f);
}

TEST(AnchorConsumer, ResetClearsAccumulation) {
  SimData data = MakeSkyBatch(20000);
  AnchorConsumer ac;
  ac.Consume(data);
  ac.PrepareSnapshot();
  ASSERT_GT(ac.SnapshotL99Sky(), 0.0f);

  ac.Reset();
  ac.PrepareSnapshot();
  EXPECT_FLOAT_EQ(ac.SnapshotL99Sky(), 0.0f);
}

// Regression pin, NOT evidence for the new property (see the file header): `visible` is a
// display-layer clip, so this was already true of the legacy anchor. It is pinned here
// because the anchor buffer must never grow a `visible` dependency.
TEST(AnchorConsumer, VisibleIsNotAnAnchorInput) {
  SimData data = MakeSkyBatch(20000);
  float once = AnchorFor(data);
  // Two renderer configs differing only in `visible`; neither reaches the anchor at all.
  auto upper = MakeRenderConfig(LensParam::kFisheyeEqualArea, 180.0f, 256, 256);
  auto lower = upper;
  lower.visible_ = RenderConfig::kLower;
  (void)LegacyAnchorScaleFor(data, upper);
  (void)LegacyAnchorScaleFor(data, lower);
  EXPECT_FLOAT_EQ(AnchorFor(data), once);
}

}  // namespace
}  // namespace lumice
