// RenderConsumer::ExposureScale() — the mono-image normalization scale.
//
// The scale's denominator is the energy the light source EMITTED, not the energy
// that LANDED on a pixel. That single substitution is what makes the displayed
// brightness absolute: emitted energy is fixed by the source and the ray budget,
// so it does not move when a filter, a low scene pass rate, or lens clipping
// removes rays, and two scenes at the same EV therefore differ in brightness by
// their real physical difference rather than by their pass rates.
//
// The whole change is one factor: the new scale is the old one times the LANDED
// FRACTION, the share of emitted energy that reached a pixel. Everything below is
// that law and its consequences, on synthetic batches with no simulation.
//
// Two things remove energy and so lower that fraction, and only the first is
// usually thought of as "content": a filter rejecting rays, and the lens simply
// not covering the sky the rays went to. A full-sphere view lands ~0.98 of what
// it emits and therefore barely moves -- which is why kNormScale, calibrated
// under the old denominator, is deliberately left alone. A narrow lens lands far
// less and moves by that much. Both are the intended behavior: an image is no
// longer rescaled to fill the tonal range regardless of how little of the scene
// it caught.
//
// The "barely moves" case is stated for a full-sphere landed fraction
// specifically. It is NOT a claim about unfiltered scenes in general, and the
// cases that must move are what keep it from being read that way -- without them
// a scale that ignored its input entirely would satisfy it.

#include <gtest/gtest.h>

#include <cmath>
#include <numeric>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
#include "core/ev_anchor.hpp"
#include "server/render.hpp"

namespace lumice {
namespace {

constexpr float kWl = 550.0f;

RenderConfig MakeConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 16;
  cfg.resolution_[1] = 16;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.intensity_factor_ = 1.0f;
  // Pinned explicitly, not left at the default: every case in this file asserts the ABSOLUTE
  // formula (emitted-energy denominator). RenderConfig's own default is kRelative, so without
  // this line the suite would silently retarget onto the P99 self-anchor and stop testing what
  // its names say it tests.
  cfg.ev_mode_ = RenderConfig::kAbsolute;
  return cfg;
}

// A batch of straight-up rays (all landing on one pixel, so the landed weight is
// exactly the sum of the weights) that declares `emitted` as the energy put in.
// landed_fraction = Σweights / emitted is thus dialled directly by the caller.
SimData MakeBatch(const std::vector<float>& weights, float emitted) {
  SimData data;
  data.curr_wl_ = kWl;
  data.root_ray_count_ = weights.size();
  data.emitted_energy_ = emitted;
  data.outgoing_d_.reserve(weights.size() * 3);
  for (size_t i = 0; i < weights.size(); ++i) {
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(-1.0f);  // sky-up: lands in-frame
  }
  data.outgoing_w_ = weights;
  return data;
}

// What ExposureScale() returned before the denominator changed: the same
// expression over the LANDED weight. Written out here rather than read from the
// consumer, because the point of these cases is to compare the two.
float LegacyLandedScale(const RenderConfig& cfg, float landed_weight) {
  int total_pix = cfg.resolution_[0] * cfg.resolution_[1];
  return cfg.intensity_factor_ * kNormScale * static_cast<float>(total_pix) / landed_weight;
}

float ScaleFor(const RenderConfig& cfg, const std::vector<float>& weights, float emitted) {
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeBatch(weights, emitted);
  rc.Consume(data);
  rc.PrepareSnapshot();
  return rc.ExposureScale();
}

float Stops(float a, float b) {
  return std::abs(std::log2(a / b));
}

const std::vector<float>& Weights() {
  static const std::vector<float> kWeights = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f };
  return kWeights;
}

float LandedWeight() {
  return std::accumulate(Weights().begin(), Weights().end(), 0.0f);
}

// -----------------------------------------------------------------------------

TEST(RenderConsumerExposureScale, FullSphereViewKeepsItsBrightnessWithinAFewHundredthsOfAStop) {
  // A full-sphere field of view with no filter lands about 98% of what it emits,
  // so the two denominators differ by ~2% — under 0.03 stop. This is the case
  // kNormScale was checked against, and why it did not have to be re-derived.
  const auto cfg = MakeConfig();
  const float landed = LandedWeight();
  const float emitted = landed / 0.98f;

  const float absolute = ScaleFor(cfg, Weights(), emitted);
  const float legacy = LegacyLandedScale(cfg, landed);

  ASSERT_GT(absolute, 0.0f);
  EXPECT_LE(Stops(absolute, legacy), 0.05f)
      << "unfiltered scene shifted by " << Stops(absolute, legacy) << " stop; kNormScale would need re-deriving";
}

TEST(RenderConsumerExposureScale, ASceneThatLandsLittleGoesDarkerByExactlyThatFraction) {
  // The other half of the pair, and the general law the case above is one point
  // of: the ratio to the old scale is the landed fraction, exactly. Here 5% —
  // reachable by a filter, by a narrow lens, or by the two together — so the
  // image must come out log2(0.05) darker than the old self-normalizing
  // behavior. That is the physical darkness the old denominator hid by dividing
  // it back out.
  const auto cfg = MakeConfig();
  const float landed = LandedWeight();
  const float emitted = landed / 0.05f;

  const float absolute = ScaleFor(cfg, Weights(), emitted);
  const float legacy = LegacyLandedScale(cfg, landed);

  ASSERT_GT(absolute, 0.0f);
  EXPECT_LT(absolute, legacy) << "a filtered scene must render DARKER, not brighter";
  EXPECT_NEAR(std::log2(absolute / legacy), std::log2(0.05f), 0.01f);
  EXPECT_GT(Stops(absolute, legacy), 0.05f) << "the unfiltered-shift bound must not be trivially satisfiable";
}

TEST(RenderConsumerExposureScale, TheScaleTracksEmittedEnergyAndIgnoresHowMuchLanded) {
  // Same emitted energy, wildly different landed weight: the scale must be
  // identical. This is the property that makes two scenes comparable, stated
  // without reference to any particular pass rate.
  const auto cfg = MakeConfig();
  const float emitted = 1000.0f;

  const float bright = ScaleFor(cfg, { 10.0f, 20.0f, 30.0f }, emitted);
  const float dim = ScaleFor(cfg, { 0.01f }, emitted);

  ASSERT_GT(bright, 0.0f);
  EXPECT_FLOAT_EQ(bright, dim);
}

TEST(RenderConsumerExposureScale, EmittedEnergyAccumulatesAcrossBatchesIncludingFullyFilteredOnes) {
  // A batch whose rays were all filtered away still emitted them. Charging only
  // the batches that produced pixels would make the denominator a function of
  // the pass rate again — the bug the whole change exists to remove — and it is
  // invisible in a single-batch test, so drive two.
  const auto cfg = MakeConfig();
  RenderConsumer rc(cfg, ColorClassTable{});

  auto landed_batch = MakeBatch({ 1.0f, 2.0f }, 100.0f);
  rc.Consume(landed_batch);

  SimData all_filtered;  // no outgoing rays at all, but the energy went in
  all_filtered.curr_wl_ = kWl;
  all_filtered.root_ray_count_ = 5;
  all_filtered.emitted_energy_ = 300.0f;
  rc.Consume(all_filtered);

  rc.PrepareSnapshot();
  const float scale = rc.ExposureScale();

  int total_pix = cfg.resolution_[0] * cfg.resolution_[1];
  const float expected = cfg.intensity_factor_ * kNormScale * static_cast<float>(total_pix) / 400.0f;
  EXPECT_FLOAT_EQ(scale, expected);
}

TEST(RenderConsumerExposureScale, ResetClearsTheDenominator) {
  const auto cfg = MakeConfig();
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeBatch(Weights(), 100.0f);
  rc.Consume(data);
  rc.PrepareSnapshot();
  ASSERT_GT(rc.ExposureScale(), 0.0f);

  rc.Reset();
  rc.PrepareSnapshot();
  // Nothing emitted since the reset → no defined scale, and in particular not a
  // stale one carried over from the previous run.
  EXPECT_EQ(rc.ExposureScale(), 0.0f);
}

TEST(RenderConsumerExposureScale, RawXyzResultCarriesTheRawEmittedTotal) {
  // The C API hands the denominator out raw so a consumer can reproduce the
  // scale; assert that is literally what is published, not a per-pixel figure.
  const auto cfg = MakeConfig();
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeBatch(Weights(), 777.5f);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const auto raw = rc.GetRawXyzResult();
  EXPECT_FLOAT_EQ(raw.snapshot_emitted_energy_, 777.5f);

  int total_pix = cfg.resolution_[0] * cfg.resolution_[1];
  const float reproduced =
      raw.intensity_factor_ * kNormScale * static_cast<float>(total_pix) / raw.snapshot_emitted_energy_;
  EXPECT_FLOAT_EQ(reproduced, rc.ExposureScale());
}

// =============================================================================
// ev_mode = kRelative: the frame anchors to ITSELF.
//
// Every case above pins the absolute branch (MakeConfig sets kAbsolute explicitly). These pin
// the other branch, whose defining property is the opposite one: no energy term at all, so the
// emitted total can move by any factor without moving the scale.
//
// The P99 here is genuinely hand-computable rather than re-derived from ComputeP99Y. MakeBatch
// sends every ray straight up, so on the 16x16 upper-hemisphere fisheye the whole weight lands
// on ONE pixel and the other 255 are exactly zero. Downsampling by kMonoAnchorDownsampleFactor
// (8) gives a 2x2 coarse grid with exactly one non-zero bin, whose box sum is that single
// pixel's Y. One non-zero sample means P99 == that sample, and ComputeP99Y then divides by f^2
// to return a fine-equivalent figure. So:
//     p99 = Y_lit / 64
// with Y_lit read off the published raw buffer (a measurement, not a formula), and therefore
//     scale = intensity_factor * TargetWhiteToLinear(135) * 64 / Y_lit.
// =============================================================================

RenderConfig MakeRelativeConfig() {
  auto cfg = MakeConfig();
  cfg.ev_mode_ = RenderConfig::kRelative;
  return cfg;
}

// Sum of the Y channel over the whole snapshot. With this fixture only one pixel is non-zero,
// so this IS that pixel's Y — obtained by scanning rather than by assuming which pixel it is.
float LitPixelY(const RenderConsumer& rc) {
  const auto raw = rc.GetRawXyzResult();
  float sum = 0.0f;
  size_t count = static_cast<size_t>(raw.img_width_) * static_cast<size_t>(raw.img_height_);
  for (size_t i = 0; i < count; ++i) {
    sum += raw.xyz_buffer_[i * 3 + 1];
  }
  return sum;
}

TEST(RenderConsumerExposureScaleRelative, MatchesTheHandComputedSelfAnchor) {
  const auto cfg = MakeRelativeConfig();
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeBatch(Weights(), 1000.0f);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const float y_lit = LitPixelY(rc);
  ASSERT_GT(y_lit, 0.0f);
  const float f2 = static_cast<float>(kMonoAnchorDownsampleFactor) * static_cast<float>(kMonoAnchorDownsampleFactor);
  const float p99 = y_lit / f2;
  const float expected = cfg.intensity_factor_ * TargetWhiteToLinear(kAnchorTargetWhite) / p99;

  EXPECT_FLOAT_EQ(rc.ExposureScale(), expected);
}

TEST(RenderConsumerExposureScaleRelative, IgnoresTheEmittedTotalTheAbsoluteBranchDividesBy) {
  // The defining difference between the two modes, stated as a pair of measurements on ONE
  // fixture: hold the landed content fixed and move only the emitted total. Absolute must track
  // it exactly (it is the denominator); relative must not see it at all.
  const auto rel = MakeRelativeConfig();
  const auto abs_cfg = MakeConfig();

  const float rel_a = ScaleFor(rel, Weights(), 1000.0f);
  const float rel_b = ScaleFor(rel, Weights(), 4000.0f);
  const float abs_a = ScaleFor(abs_cfg, Weights(), 1000.0f);
  const float abs_b = ScaleFor(abs_cfg, Weights(), 4000.0f);

  ASSERT_GT(rel_a, 0.0f);
  EXPECT_FLOAT_EQ(rel_b, rel_a) << "a self-anchored scale carries no energy term";
  EXPECT_FLOAT_EQ(abs_b, abs_a / 4.0f) << "the absolute scale is exactly 1/emitted";
}

TEST(RenderConsumerExposureScaleRelative, IsADifferentNumberFromTheAbsoluteBranch) {
  // Guards the wiring itself: if ev_mode_ were dropped on the floor somewhere between the config
  // and ExposureScale(), both modes would silently return whichever branch won, and the two
  // cases above would still pass on the relative fixture alone.
  const float rel = ScaleFor(MakeRelativeConfig(), Weights(), 1000.0f);
  const float abs_scale = ScaleFor(MakeConfig(), Weights(), 1000.0f);
  ASSERT_GT(rel, 0.0f);
  ASSERT_GT(abs_scale, 0.0f);
  EXPECT_NE(rel, abs_scale);
}

TEST(RenderConsumerExposureScaleRelative, BlackFrameHasNoAnchorAndScoresZero) {
  // A frame with no lit pixel has no P99 to anchor to. Zero is the same answer the absolute
  // branch gives for its own undefined case (nothing emitted) — not an exception, not a
  // silently huge scale from dividing by an epsilon.
  const auto cfg = MakeRelativeConfig();
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeBatch({ 0.0f, 0.0f }, 1000.0f);
  rc.Consume(data);
  rc.PrepareSnapshot();

  EXPECT_EQ(rc.ExposureScale(), 0.0f);
}

TEST(RenderConsumerExposureScaleRelative, ScalesWithIntensityFactorLikeTheAbsoluteBranchDoes) {
  // intensity_factor_ is the user's EV offset and multiplies BOTH branches; it is not part of
  // what the anchor choice changes.
  auto cfg = MakeRelativeConfig();
  const float base = ScaleFor(cfg, Weights(), 1000.0f);
  cfg.intensity_factor_ = 4.0f;
  const float boosted = ScaleFor(cfg, Weights(), 1000.0f);
  EXPECT_FLOAT_EQ(boosted, base * 4.0f);
}

}  // namespace
}  // namespace lumice
