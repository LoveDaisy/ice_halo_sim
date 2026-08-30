// RenderConsumer::ExposureScale() — the mono-image normalization scale.
//
// The scale's denominator is the energy the light source EMITTED, not the energy
// that LANDED on a pixel. That single substitution is what makes the displayed
// brightness absolute: emitted energy is fixed by the source and the ray budget,
// so it does not move when a filter, a low scene pass rate, or lens clipping
// removes rays, and two scenes at the same EV therefore differ in brightness by
// their real physical difference rather than by their pass rates.
//
// These cases pin the consequences of that substitution directly, with synthetic
// batches and no simulation:
//   - a scene where essentially everything lands is left where it was (the
//     constant kNormScale was calibrated under the old denominator and is
//     deliberately unchanged);
//   - a scene where a filter removes most rays moves, by the amount the filter
//     removed. Without the second case the first would be satisfied by a scale
//     that ignores its input entirely.

#include <gtest/gtest.h>

#include <cmath>
#include <numeric>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
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

TEST(RenderConsumerExposureScale, UnfilteredSceneKeepsItsBrightnessWithinAFewHundredthsOfAStop) {
  // A full-sky field of view lands about 98% of what it emits, so the two
  // denominators differ by ~2% — under 0.03 stop. This is why kNormScale did not
  // have to be re-derived when the denominator changed.
  const auto cfg = MakeConfig();
  const float landed = LandedWeight();
  const float emitted = landed / 0.98f;

  const float absolute = ScaleFor(cfg, Weights(), emitted);
  const float legacy = LegacyLandedScale(cfg, landed);

  ASSERT_GT(absolute, 0.0f);
  EXPECT_LE(Stops(absolute, legacy), 0.05f)
      << "unfiltered scene shifted by " << Stops(absolute, legacy) << " stop; kNormScale would need re-deriving";
}

TEST(RenderConsumerExposureScale, FilteredSceneGoesDarkerByTheAmountTheFilterRemoved) {
  // The other half of the pair. If the scale ignored its denominator the case
  // above would still pass, so a case that MUST move is what gives it teeth.
  // A filter passing 5% of the rays must darken the image by log2(0.05/0.98)
  // ≈ 4.3 stops relative to the old self-normalizing behavior — the physical
  // darkness the old denominator hid by dividing it back out.
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

}  // namespace
}  // namespace lumice
