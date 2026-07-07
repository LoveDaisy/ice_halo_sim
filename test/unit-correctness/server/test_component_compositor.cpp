// Tests for task-336.3 (component compositor): the three display-time composite
// modes (dominant / additive / painter), the orthogonal show/hide/solo
// visibility axis, and — the crux — the SHARED-EXPOSURE invariant: every mode
// multiplies each component's raw-Y lane by the single mono-image exposure scale
// (RenderConsumer::ExposureScale()) and nothing else. There is never a per-lane
// / per-composite renormalization (that was the spike's false-color bug).
//
// Coverage (plan §8 A–E):
//   A — per-pixel mode math (dominant / additive / painter, incl. tie + painter
//       vs dominant divergence).
//   B — shared exposure: ExposureScale() == the mono PostSnapshot scale, and
//       additive-white total == mono exposed Y (no self-normalization).
//   C — visibility orthogonality (hide / solo, across modes).
//   D — dominant × three-arcs real backend: three colors appear, no phantom hue.
//   E — linear→sRGB smoke + zero regression (compositor produces nothing when
//       colored_mask == 0).

#include <gtest/gtest.h>

#include <array>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/component_color_map.hpp"
#include "config/component_table.hpp"
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/raypath_color_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/backend/cpu_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/color_util.hpp"
#include "server/component_compositor.hpp"
#include "server/render.hpp"
#include "util/color_space.hpp"

namespace lumice {
namespace {

constexpr float kWl = 550.0f;

// Small fisheye looking straight up: rays with dir=(0,0,-1) land in the image
// centre. A tiny resolution keeps a single lit pixel's exposed Y below 1 so the
// exact per-pixel additive assertions do not hit the [0,1] clamp (exposed Y at a
// lone lit pixel ≈ kCmfY(550) * kNormScale * total_pix — must stay < 1).
RenderConfig MakeRenderConfig(int res) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = res;
  cfg.resolution_[1] = res;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.intensity_factor_ = 1.0f;
  return cfg;
}

// SimData carrying explicit sky-up rays + per-ray component masks (all rays land
// in one pixel unless masks vary the direction — here they all point up).
SimData MakeBatch(const std::vector<uint64_t>& masks, const std::vector<float>& weights) {
  SimData data;
  data.curr_wl_ = kWl;
  data.outgoing_d_.reserve(masks.size() * 3);
  for (size_t i = 0; i < masks.size(); ++i) {
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(-1.0f);
  }
  data.outgoing_w_ = weights;
  data.outgoing_component_ = masks;
  return data;
}

// task-339.3: turn a legacy uint64_t "colored bits" mask into a ColorClassTable
// with one single-member `kAny` class per set bit. Kept as a test-local helper
// (matching the same-shape helper in test_render_consumer_component_lanes.cpp)
// so this file's existing single-bit assertions carry over unchanged — the
// RenderConsumer's construction API is the only thing that had to change.
ColorClassTable MakeSingletonClassTable(uint64_t mask) {
  ColorClassTable t;
  for (uint8_t bit = 0; bit < ComponentTable::kMaxBits; ++bit) {
    if (((mask >> bit) & 1ULL) == 0) {
      continue;
    }
    ColorClass cls;
    cls.combine_ = ColorClassCombine::kAny;
    cls.member_bits_ = static_cast<uint64_t>(1) << bit;
    t.classes_.push_back(cls);
    t.referenced_mask_ |= cls.member_bits_;
  }
  return t;
}

// Build a color map with the given per-bit RGBs; colored_mask_ covers bits 0..n-1.
ComponentColorMap MakeColorMap(const std::vector<std::array<float, 3>>& colors) {
  ComponentColorMap map;
  for (uint8_t bit = 0; bit < colors.size(); ++bit) {
    map.colors_[bit][0] = colors[bit][0];
    map.colors_[bit][1] = colors[bit][1];
    map.colors_[bit][2] = colors[bit][2];
    map.colored_mask_ |= (static_cast<uint64_t>(1) << bit);
  }
  return map;
}

// Index of the single lit pixel (first with a positive lane-0 value).
int FindLitPixel(const RenderConsumer& rc, int total_pix) {
  const float* lane0 = rc.GetComponentLaneY(0);
  for (int p = 0; p < total_pix; ++p) {
    if (lane0 != nullptr && lane0[p] > 0.0f) {
      return p;
    }
  }
  return -1;
}

const std::array<float, 3> kRed{ 1.0f, 0.0f, 0.0f };
const std::array<float, 3> kGreen{ 0.0f, 1.0f, 0.0f };
const std::array<float, 3> kBlue{ 0.0f, 0.0f, 1.0f };
const std::array<float, 3> kWhite{ 1.0f, 1.0f, 1.0f };

// -----------------------------------------------------------------------------
// A. Per-pixel mode math.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, DominantAdditivePainterPerPixelMath) {
  constexpr int kRes = 3;  // 9 px → single lit pixel exposed Y < 1 (no clamp)
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // bit0 (weight a) is brighter than bit1 (weight b): a > b.
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  ASSERT_GT(s, 0.0f);
  const float ey0 = rc.GetComponentLaneY(0)[p] * s;
  const float ey1 = rc.GetComponentLaneY(1)[p] * s;
  ASSERT_GT(ey0, ey1);  // a > b

  const auto colors = MakeColorMap({ kRed, kGreen });
  std::vector<float> out;

  // dominant: brighter bit0 (red) wins → (ey0, 0, 0).
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, 0, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);

  // additive: red*ey0 + green*ey1 → (ey0, ey1, 0), both < 1 so no clamp.
  ASSERT_LT(ey0 + ey1, 1.0f);
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kAdditive, 0, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);

  // painter (order = bit ascending; lowest bit = top layer): bit0 wins → red,
  // same as dominant here because bit0 is both brighter AND on top.
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kPainter, 0, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
}

TEST(ComponentCompositor, PainterVsDominantDivergeWhenTopBitDimmer) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Now bit1 (weight 0.9) is BRIGHTER than bit0 (weight 0.3): dominant → bit1,
  // painter → bit0 (the top/lowest bit) regardless of strength.
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.3f, 0.9f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey0 = rc.GetComponentLaneY(0)[p] * s;
  const float ey1 = rc.GetComponentLaneY(1)[p] * s;
  ASSERT_GT(ey1, ey0);  // bit1 brighter

  const auto colors = MakeColorMap({ kRed, kGreen });
  std::vector<float> out;

  // dominant picks the brighter bit1 (green).
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, 0, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

  // painter picks the top (lowest-index) bit0 (red), even though it is dimmer.
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kPainter, 0, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
}

TEST(ComponentCompositor, DominantTieTakesMinBit) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Equal weights → equal lanes → strict-`>` ascending scan keeps the min bit.
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.5f, 0.5f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey0 = rc.GetComponentLaneY(0)[p] * s;
  const float ey1 = rc.GetComponentLaneY(1)[p] * s;
  EXPECT_FLOAT_EQ(ey0, ey1);

  const auto colors = MakeColorMap({ kRed, kGreen });
  std::vector<float> out;
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, 0, 0 }, out));
  // Tie → bit0 (red), not bit1.
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
}

// -----------------------------------------------------------------------------
// B. Shared exposure (crux): the compositor uses the mono exposure, unmodified.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, SharedExposureNoSelfNormalization) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  // (1) ExposureScale() must equal the mono PostSnapshot scale expression
  //     intensity_factor * kNormScale * total_pix / snapshot_intensity_.
  //     All rays land in-bounds, so snapshot_intensity_ == Σ weights = 1.0 — an
  //     independent reconstruction of the scale from the known batch inputs.
  const auto raw = rc.GetRawXyzResult();
  const float sum_w = 0.6f + 0.4f;  // both rays land → snapshot_intensity_
  const float mono_scale = cfg.intensity_factor_ * kNormScale * static_cast<float>(total_pix) / sum_w;
  EXPECT_NEAR(rc.ExposureScale(), mono_scale, mono_scale * 1e-5f);
  // Cross-check against RawXyzResult (whose reported intensity is per-pixel):
  // scale == intensity_factor / per_pixel_intensity.
  EXPECT_NEAR(rc.ExposureScale(), raw.intensity_factor_ / raw.snapshot_intensity_, mono_scale * 1e-5f);

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey0 = rc.GetComponentLaneY(0)[p] * s;
  const float ey1 = rc.GetComponentLaneY(1)[p] * s;

  // (2) mono exposed Y at the lit pixel = mainY * s. With single-bit rays,
  //     mainY == lane0 + lane1, so the exposed mono Y equals ey0 + ey1 —
  //     precisely the additive-white total. No per-lane renormalization.
  const double mono_y = static_cast<double>(raw.xyz_buffer_[p * 3 + 1]);
  const double mono_exposed_y = mono_y * s;
  EXPECT_NEAR(mono_exposed_y, static_cast<double>(ey0 + ey1), 1e-4);

  const auto white = MakeColorMap({ kWhite, kWhite });
  std::vector<float> out;
  ASSERT_LT(ey0 + ey1, 1.0f);  // keep below clamp so the equality is exact
  ASSERT_TRUE(CompositeComponentLinear(rc, white, { CompositeMode::kAdditive, 0, 0 }, out));
  // Each channel of additive-white == ey0+ey1 == mono exposed Y (shared exposure).
  EXPECT_NEAR(out[p * 3 + 0], mono_exposed_y, 1e-4);
  EXPECT_NEAR(out[p * 3 + 1], mono_exposed_y, 1e-4);
  EXPECT_NEAR(out[p * 3 + 2], mono_exposed_y, 1e-4);
}

// -----------------------------------------------------------------------------
// C. Visibility orthogonality (hide / solo), independent of mode.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, VisibilityHideAndSoloAcrossModes) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // bit0 is brighter; without visibility overrides dominant would pick it.
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey1 = rc.GetComponentLaneY(1)[p] * s;

  const auto colors = MakeColorMap({ kRed, kGreen });
  std::vector<float> out;

  // Hide bit0 → dominant must fall back to bit1 (green), even though bit0 brighter.
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, /*hidden=*/0b01, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

  // Hide bit0 under additive → only bit1 (green) contributes.
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kAdditive, /*hidden=*/0b01, 0 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

  // Solo bit1 → only bit1 visible regardless of mode.
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, 0, /*solo=*/0b10 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

  // Solo overrides hide: everything hidden but bit1 solo'd → bit1 shows.
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, /*hidden=*/0b11, /*solo=*/0b10 }, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);
}

// -----------------------------------------------------------------------------
// D. dominant × three-arcs real backend → three colors, no phantom hue.
//    (harness mirrors test_render_consumer_component_lanes.cpp §6)
// -----------------------------------------------------------------------------
PrismCrystalParam MakeUnitPrism() {
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  return prism;
}

FilterConfig MakeFilterIn(FilterParam param) {
  FilterConfig f;
  f.id_ = 0;
  f.symmetry_ = FilterConfig::kSymNone;
  f.action_ = FilterConfig::kFilterIn;
  f.param_ = std::move(param);
  return f;
}

SceneConfig MakeTwoCrystalColoredScene(size_t max_hits) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { kWl, 1.0f } };

  MsInfo ms;
  ms.prob_ = 0.0f;
  {
    ScatteringSetting s;
    s.crystal_.id_ = 0;
    s.crystal_.param_ = MakeUnitPrism();
    s.filter_ = MakeFilterIn(SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 1, std::nullopt } });
    s.crystal_proportion_ = 0.5f;
    ms.setting_.push_back(std::move(s));
  }
  {
    ComplexFilterParam cf;
    cf.filters_.resize(2);
    cf.filters_[0].emplace_back(IdType{ 0 },
                                SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 2, 2 } });
    cf.filters_[1].emplace_back(
        IdType{ 0 }, SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 3, std::nullopt } });
    ScatteringSetting s;
    s.crystal_.id_ = 1;
    s.crystal_.param_ = MakeUnitPrism();
    s.filter_ = MakeFilterIn(FilterParam{ cf });
    s.crystal_proportion_ = 0.5f;
    ms.setting_.push_back(std::move(s));
  }
  scene.ms_.push_back(std::move(ms));
  return scene;
}

TEST(ComponentCompositor, DominantThreeArcsNoPhantomHue) {
  constexpr size_t kMaxHits = 8;
  auto scene = MakeTwoCrystalColoredScene(kMaxHits);
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 3u);

  RenderConfig render = MakeRenderConfig(64);
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ kWl, 1.0f };
  spec.seed = 20240707u;

  CpuTraceBackend backend;
  backend.BeginSession(spec);

  constexpr size_t kRayCount = 200000;
  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  auto handle = backend.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(handle, nullptr);

  std::vector<ExitRayRecord> records;
  backend.ReadbackExitRays(records);
  backend.EndSession();
  ASSERT_FALSE(records.empty());

  SimData data;
  data.curr_wl_ = kWl;
  data.outgoing_d_.reserve(records.size() * 3);
  data.outgoing_w_.reserve(records.size());
  data.outgoing_component_.reserve(records.size());
  for (const auto& rec : records) {
    data.outgoing_d_.push_back(rec.dir[0]);
    data.outgoing_d_.push_back(rec.dir[1]);
    data.outgoing_d_.push_back(rec.dir[2]);
    data.outgoing_w_.push_back(rec.weight);
    data.outgoing_component_.push_back(rec.component_mask);
  }

  const uint64_t kColored = 0b111;
  RenderConsumer rc(render, MakeSingletonClassTable(kColored));
  rc.Consume(data);
  rc.PrepareSnapshot();

  const auto colors = MakeColorMap({ kRed, kGreen, kBlue });
  std::vector<float> out;
  ASSERT_TRUE(CompositeComponentLinear(rc, colors, { CompositeMode::kDominant, 0, 0 }, out));

  const int total_pix = render.resolution_[0] * render.resolution_[1];
  int red_px = 0;
  int green_px = 0;
  int blue_px = 0;
  for (int p = 0; p < total_pix; ++p) {
    const float r = out[p * 3 + 0];
    const float g = out[p * 3 + 1];
    const float b = out[p * 3 + 2];
    if (r == 0.0f && g == 0.0f && b == 0.0f) {
      continue;  // background
    }
    // No phantom hue: dominant paints exactly ONE pure component color per pixel,
    // so exactly one channel is non-zero. A mixed pixel would betray per-lane
    // renormalization or additive bleed.
    const int nonzero = (r > 0.0f ? 1 : 0) + (g > 0.0f ? 1 : 0) + (b > 0.0f ? 1 : 0);
    EXPECT_EQ(nonzero, 1) << "phantom hue at pixel " << p << " rgb=(" << r << "," << g << "," << b << ")";
    if (r > 0.0f) {
      ++red_px;
    } else if (g > 0.0f) {
      ++green_px;
    } else {
      ++blue_px;
    }
  }
  // All three arc colors are present.
  EXPECT_GT(red_px, 0);
  EXPECT_GT(green_px, 0);
  EXPECT_GT(blue_px, 0);
}

// -----------------------------------------------------------------------------
// E. linear→sRGB smoke + zero regression.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, LinearToSrgbU8Smoke) {
  std::vector<float> lin = { 0.0f, 1.0f, 0.5f, -0.2f, 1.5f };
  std::vector<uint8_t> out;
  LinearRgbToSrgbU8(lin, out);
  ASSERT_EQ(out.size(), lin.size());
  // Expectations mirror PostSnapshot's exact transfer + truncating cast
  // (sRGB(1.0)*255 truncates to 254 in float — identical to the mono path).
  const uint8_t exp_one = static_cast<uint8_t>(LinearToSrgb(1.0f) * 255.0f);
  const uint8_t exp_half = static_cast<uint8_t>(LinearToSrgb(0.5f) * 255.0f);
  EXPECT_EQ(out[0], 0u);        // 0 → 0
  EXPECT_EQ(out[1], exp_one);   // 1 → sRGB(1)
  EXPECT_EQ(out[2], exp_half);  // 0.5 → sRGB(0.5)
  EXPECT_EQ(out[3], 0u);        // negative clamps to 0
  EXPECT_EQ(out[4], exp_one);   // >1 clamps to 1 → sRGB(1)
}

TEST(ComponentCompositor, ZeroColoredMaskProducesNoComposite) {
  RenderConfig cfg = MakeRenderConfig(8);
  RenderConsumer rc(cfg, ColorClassTable{});  // pre-336 path, no lanes
  rc.Consume(MakeBatch({ 0b00 }, { 0.5f }));
  rc.PrepareSnapshot();

  ComponentColorMap empty_map;         // colored_mask_ == 0
  std::vector<float> out = { 42.0f };  // sentinel — must be left untouched
  EXPECT_FALSE(CompositeComponentLinear(rc, empty_map, { CompositeMode::kDominant, 0, 0 }, out));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0], 42.0f) << "compositor must not touch the output when colored_mask == 0";
}

// -----------------------------------------------------------------------------
// F. Config → runtime join (task-339.2): BuildColorClassTable + ToLegacy*
//    adapters produce the same (mode, hidden_mask, solo_mask) tuple as the
//    old flat BuildCompositeOptions and paint the same colored_mask when
//    driven by single-member classes.
// -----------------------------------------------------------------------------
namespace {

ColorClassConfig MakeCls(std::vector<float> rgb, std::vector<RaypathColorRef> match, bool visible = true,
                         bool solo = false) {
  ColorClassConfig c{};
  c.color_[0] = rgb[0];
  c.color_[1] = rgb[1];
  c.color_[2] = rgb[2];
  c.visible_ = visible;
  c.solo_ = solo;
  c.match_ = std::move(match);
  return c;
}

RaypathColorRef MakeRef(uint16_t layer, uint16_t crystal, bool has_filter, uint16_t filter, bool has_summand,
                        uint16_t summand) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.has_filter_ = has_filter;
  r.filter_ = filter;
  r.has_summand_ = has_summand;
  r.summand_ = summand;
  return r;
}

}  // namespace

TEST(ComponentCompositor, ToLegacyCompositeOptionsFoldsModeVisibilitySolo) {
  auto scene = MakeTwoCrystalColoredScene(8);
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 3u);
  // Refs by id: crystal=0 filter=0 → ci=0 (simple, 1 summand, bit0);
  //             crystal=1 filter=0 → ci=1 (complex, 2 summands, bits 1/2).

  RaypathColorConfig cfg;
  cfg.mode_ = "painter";
  cfg.classes_.push_back(MakeCls({ 1.0f, 0.0f, 0.0f }, { MakeRef(0, 0, true, 0, false, 0) }));  // bit0
  cfg.classes_.push_back(
      MakeCls({ 0.0f, 1.0f, 0.0f }, { MakeRef(0, 1, true, 0, true, 0) }, false, false));  // hidden bit1
  cfg.classes_.push_back(MakeCls({ 0.0f, 0.0f, 1.0f }, { MakeRef(0, 1, true, 0, true, 1) }, true, true));  // solo bit2

  ColorClassTable ct = BuildColorClassTable(cfg, scene, table);
  CompositeOptions opt = ToLegacyCompositeOptions(ct, cfg.mode_);
  EXPECT_EQ(opt.mode_, CompositeMode::kPainter);
  EXPECT_EQ(opt.hidden_mask_, 0b010ULL);
  EXPECT_EQ(opt.solo_mask_, 0b100ULL);

  EXPECT_EQ(ToLegacyCompositeOptions(ct, "additive").mode_, CompositeMode::kAdditive);
  EXPECT_EQ(ToLegacyCompositeOptions(ct, "bogus-typo").mode_, CompositeMode::kDominant);
}

TEST(ComponentCompositor, RaypathColorConfigJsonFormsRoundTrip) {
  // Non-default mode → object form {mode, classes}, preserving visible/solo.
  RaypathColorConfig cfg;
  cfg.mode_ = "additive";
  cfg.classes_.push_back(MakeCls({ 0.2f, 0.4f, 0.6f }, { MakeRef(0, 1, true, 0, true, 0) }, false, true));
  nlohmann::json j = cfg;
  EXPECT_TRUE(j.is_object());
  RaypathColorConfig back = j.get<RaypathColorConfig>();
  EXPECT_EQ(back.mode_, "additive");
  ASSERT_EQ(back.classes_.size(), 1u);
  EXPECT_FALSE(back.classes_[0].visible_);
  EXPECT_TRUE(back.classes_[0].solo_);
  EXPECT_FLOAT_EQ(back.classes_[0].color_[2], 0.6f);
  ASSERT_EQ(back.classes_[0].match_.size(), 1u);
  EXPECT_EQ(back.classes_[0].match_[0].crystal_, 1);
  EXPECT_TRUE(back.classes_[0].match_[0].has_filter_);
  EXPECT_TRUE(back.classes_[0].match_[0].has_summand_);

  // Default mode → bare array form, still round-trips.
  RaypathColorConfig dom;
  dom.classes_.push_back(MakeCls({ 1.0f, 0.0f, 0.0f }, { MakeRef(0, 0, true, 0, false, 0) }));
  nlohmann::json jd = dom;
  EXPECT_TRUE(jd.is_array()) << "default-mode config must serialize as a bare array";
  RaypathColorConfig dback = jd.get<RaypathColorConfig>();
  EXPECT_EQ(dback.mode_, "dominant");
  ASSERT_EQ(dback.classes_.size(), 1u);
  EXPECT_TRUE(dback.classes_[0].visible_);
  EXPECT_FALSE(dback.classes_[0].solo_);
}

}  // namespace
}  // namespace lumice
