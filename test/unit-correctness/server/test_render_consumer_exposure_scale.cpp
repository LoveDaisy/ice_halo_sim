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

// The sky anchor every relative-mode case below is exposed against. One number for the whole
// file: the point of the new anchor is that it is a property of the SCENE, so a fixture that
// varied it per case would be varying the scene rather than the thing under test. The value
// itself is arbitrary — only the absolute branch has a physically meaningful denominator, and it
// ignores this one entirely.
constexpr float kSceneAnchorL99 = 4.0e3f;

// The server's own order: measure, push, then ask. Pushing unconditionally (the absolute branch
// does not read it) keeps the two modes' fixtures identical apart from ev_mode_.
float ScaleFor(const RenderConfig& cfg, const std::vector<float>& weights, float emitted,
               float anchor_l99_sky = kSceneAnchorL99) {
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeBatch(weights, emitted);
  rc.Consume(data);
  rc.PrepareSnapshot();
  rc.SetAnchorL99Sky(anchor_l99_sky);
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
// ev_mode = kRelative: the frame anchors to the SKY, through a scalar it is handed.
//
// Every case above pins the absolute branch (MakeConfig sets kAbsolute explicitly). These pin
// the other branch, whose defining properties are two: no energy term at all, so the emitted
// total can move by any factor without moving the scale; and no dependence on this frame's own
// pixels either, since the anchor is measured elsewhere (AnchorConsumer, over a fixed full-sky
// plane) and pushed in by ServerImpl::DoSnapshot before the bake.
//
// The expected value is hand-computable end to end. The scale is
//     intensity_factor * TargetWhiteToLinear(135) / (Omega_axis * L99_sky)
// and Omega_axis is exact on this fixture rather than approximate: an equal-area fisheye at
// fov 180 on a 16x16 frame has ComputeLensScale = 16/2/sqrt(2)/sin(45 deg) = 8 exactly, and an
// equal-area map's on-axis pixel subtends 2/scale^2 (see core/lens_proj_build.hpp), i.e.
//     Omega_axis = 2 / 64 = 0.03125 sr
// written below as a literal so the assertion does not ask the subject what the answer is.
// =============================================================================

// The on-axis pixel solid angle of MakeConfig()'s lens, derived above. A hand-computed constant
// on purpose: reading ComputeAxisSolidAngle here would make the expectation agree with whatever
// that function returns, which is the one thing it must not do.
constexpr float kFixtureAxisSolidAngle = 2.0f / 64.0f;

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

TEST(RenderConsumerExposureScaleRelative, MatchesTheHandComputedSkyAnchor) {
  const auto cfg = MakeRelativeConfig();
  const float expected =
      cfg.intensity_factor_ * TargetWhiteToLinear(kAnchorTargetWhite) / (kFixtureAxisSolidAngle * kSceneAnchorL99);

  EXPECT_FLOAT_EQ(ScaleFor(cfg, Weights(), 1000.0f), expected);
}

TEST(RenderConsumerExposureScaleRelative, IsInverselyProportionalToTheSkyAnchor) {
  // The anchor is the denominator and nothing else stands between it and the scale: doubling the
  // sky's brightness must halve the exposure exactly. Stated separately from the case above
  // because that one would also pass for a formula that happened to hit the same number at one
  // anchor value.
  const auto cfg = MakeRelativeConfig();
  const float at_1x = ScaleFor(cfg, Weights(), 1000.0f, kSceneAnchorL99);
  const float at_2x = ScaleFor(cfg, Weights(), 1000.0f, 2.0f * kSceneAnchorL99);
  ASSERT_GT(at_1x, 0.0f);
  EXPECT_FLOAT_EQ(at_2x, at_1x / 2.0f);
}

TEST(RenderConsumerExposureScaleRelative, IgnoresThisFramesOwnPixels) {
  // The property the two-sided anchor exists for, at this layer: the exposure is a statement
  // about the SCENE, so moving the frame's own content — here by a factor of 4 in landed weight,
  // which is a 2-stop swing in the P99 the retired rule anchored to — must not move it at all.
  const auto cfg = MakeRelativeConfig();
  std::vector<float> heavy;
  heavy.reserve(Weights().size());
  for (float w : Weights()) {
    heavy.push_back(w * 4.0f);
  }

  RenderConsumer light_rc(cfg, ColorClassTable{});
  auto light_batch = MakeBatch(Weights(), 1000.0f);
  light_rc.Consume(light_batch);
  light_rc.PrepareSnapshot();
  RenderConsumer heavy_rc(cfg, ColorClassTable{});
  auto heavy_batch = MakeBatch(heavy, 1000.0f);
  heavy_rc.Consume(heavy_batch);
  heavy_rc.PrepareSnapshot();

  // Control: the frames really do differ, by the factor the retired anchor would have seen.
  const float light_y = LitPixelY(light_rc);
  const float heavy_y = LitPixelY(heavy_rc);
  ASSERT_GT(light_y, 0.0f);
  EXPECT_NEAR(heavy_y / light_y, 4.0f, 1e-3f) << "the two fixtures no longer differ, so the invariance below is "
                                                 "a statement about nothing";

  light_rc.SetAnchorL99Sky(kSceneAnchorL99);
  heavy_rc.SetAnchorL99Sky(kSceneAnchorL99);
  EXPECT_FLOAT_EQ(heavy_rc.ExposureScale(), light_rc.ExposureScale());
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

TEST(RenderConsumerExposureScaleRelative, NoAnchorScoresZeroRatherThanCarryingTheLastOne) {
  // Two ways to arrive with no anchor, one answer. Zero is the same answer the absolute branch
  // gives for its own undefined case (nothing emitted) — not an exception, not a silently huge
  // scale from dividing by an epsilon.
  //
  // The second case is the one ServerImpl::DoSnapshot's per-pass reset exists for. A renderer is
  // reused across snapshots and this member persists, so a pass that failed to push would
  // otherwise expose the new frame with the PREVIOUS scene's anchor — a plausible wrong number
  // rather than a visibly broken one. The reset turns that into this, which is covered.
  const auto cfg = MakeRelativeConfig();

  RenderConsumer never_pushed(cfg, ColorClassTable{});
  auto data = MakeBatch(Weights(), 1000.0f);
  never_pushed.Consume(data);
  never_pushed.PrepareSnapshot();
  EXPECT_EQ(never_pushed.ExposureScale(), 0.0f);

  RenderConsumer cleared(cfg, ColorClassTable{});
  auto second = MakeBatch(Weights(), 1000.0f);
  cleared.Consume(second);
  cleared.PrepareSnapshot();
  cleared.SetAnchorL99Sky(kSceneAnchorL99);
  ASSERT_GT(cleared.ExposureScale(), 0.0f) << "the fixture never had an anchor, so clearing it proves nothing";
  cleared.SetAnchorL99Sky(0.0f);
  EXPECT_EQ(cleared.ExposureScale(), 0.0f);
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
