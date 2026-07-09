// Tests for task-339.4 (per-color-class compositor): the three display-time
// composite modes (dominant / additive / painter), the orthogonal show/hide/
// solo visibility axis, and — the crux — the SHARED-EXPOSURE invariant: every
// mode multiplies each color-class's raw-Y lane by the single mono-image
// exposure scale (RenderConsumer::ExposureScale()) and nothing else. There is
// never a per-lane / per-composite renormalization (that was the spike's
// false-color bug).
//
// Coverage (plan §4 A–E + AC additions):
//   A — per-pixel mode math (dominant / additive / painter, incl. tie + painter
//       vs dominant divergence).
//   B — shared exposure: ExposureScale() == the mono PostSnapshot scale, and
//       additive-white total == mono exposed Y (no self-normalization).
//   C — visibility orthogonality (hide / solo, across modes) applied
//       per-color-class.
//   D — dominant × three-arcs real backend: three colors appear, no phantom hue.
//   E — linear→sRGB smoke + zero regression (compositor produces nothing when
//       the class table is empty).
//   F — overlap: a ray satisfying multiple color classes → dominant/painter
//       pick per z-order, additive mixes.
//   G — cross-layer AND: combine:"all" class only shows where all members hit.
//   H — ParseCompositeMode string parsing + fallback.
//
// The 336.3-era per-bit adapter tests (ToLegacyCompositeOptions round-trip,
// ComponentColorMap layout) are gone: their functions were deleted with 339.4.
// RaypathColorConfig JSON round-trip stays here (it's a compositor-adjacent
// smoke check; SUMMARY notes we evaluated moving it to
// test_color_class_table.cpp and kept the original file to minimise diff).

#include <gtest/gtest.h>

#include <array>
#include <bitset>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "config/color_class_table.hpp"
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

// Build one color class with the given color, combine and member bits. `visible`
// and `solo` default to the per-class ColorClass defaults (visible=true,
// solo=false). Kept flat here (no ctor overhead / no reliance on config DTO
// path) — the DTO join is exercised in test_color_class_table.cpp.
ColorClass MakeClass(const std::array<float, 3>& color, ColorClassCombine combine, uint64_t member_bits,
                     bool visible = true, bool solo = false) {
  ColorClass cls{};
  cls.color_[0] = color[0];
  cls.color_[1] = color[1];
  cls.color_[2] = color[2];
  cls.combine_ = combine;
  cls.member_bits_ = member_bits;
  cls.visible_ = visible;
  cls.solo_ = solo;
  return cls;
}

// One single-member `kAny` class per set bit; class colors follow `colors[i]`.
// Colors must have as many entries as set bits. Kept as a helper so the mode-
// math tests (dominant/additive/painter) match the 336.3 test skeleton — the
// only change is "per bit" → "per single-member class".
ColorClassTable MakeSingletonClassTable(uint64_t mask, const std::vector<std::array<float, 3>>& colors) {
  assert(colors.size() >= std::bitset<64>(mask).count() && "colors must cover every set bit in mask");
  ColorClassTable t;
  size_t ci = 0;
  for (uint8_t bit = 0; bit < ComponentTable::kMaxBits; ++bit) {
    if (((mask >> bit) & 1ULL) == 0) {
      continue;
    }
    t.classes_.push_back(MakeClass(colors[ci], ColorClassCombine::kAny, static_cast<uint64_t>(1) << bit));
    t.referenced_mask_ |= (static_cast<uint64_t>(1) << bit);
    ++ci;
  }
  return t;
}

// Overload for tests that don't care about lane colors (the constructor also
// wires up the RenderConsumer's lane accumulation).
ColorClassTable MakeSingletonClassTable(uint64_t mask) {
  std::vector<std::array<float, 3>> greys(64, { 1.0f, 1.0f, 1.0f });
  return MakeSingletonClassTable(mask, greys);
}

// Index of the single lit pixel (first with a positive class-0 lane value).
int FindLitPixel(const RenderConsumer& rc, int total_pix) {
  const float* lane0 = rc.GetColorClassLaneY(0);
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

// Config-DTO helpers (shared with RaypathColorConfigJsonFormsRoundTrip below;
// also consumed by DominantThreeArcsNoPhantomHue to build a Design-2
// raypath_color config for the CPU emit gate).
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

RaypathColorRef MakeRef(uint16_t layer, uint16_t crystal, SimpleFilterParam predicate = NoneFilterParam{}) {
  RaypathColorRef r{};
  r.layer_ = layer;
  r.crystal_ = crystal;
  r.predicate_ = std::move(predicate);
  return r;
}

// -----------------------------------------------------------------------------
// A. Per-pixel mode math.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, DominantAdditivePainterPerPixelMath) {
  constexpr int kRes = 3;  // 9 px → single lit pixel exposed Y < 1 (no clamp)
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // class0 (bit0, red, weight a=0.6) is brighter than class1 (bit1, green,
  // weight b=0.4). List order: class0 first (top layer for painter).
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  ASSERT_GT(s, 0.0f);
  const float ey0 = rc.GetColorClassLaneY(0)[p] * s;
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
  ASSERT_GT(ey0, ey1);  // a > b

  std::vector<float> out;

  // dominant: brighter class0 (red) wins → (ey0, 0, 0).
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);

  // additive: red*ey0 + green*ey1 → (ey0, ey1, 0), both < 1 so no clamp.
  ASSERT_LT(ey0 + ey1, 1.0f);
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kAdditive, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);

  // painter (list order = z-order; list-first class is the top layer): class0
  // wins → red, same as dominant here because class0 is both brighter AND on top.
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
}

TEST(ComponentCompositor, PainterVsDominantDivergeWhenTopClassDimmer) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Now class1 (bit1, green, weight 0.9) is BRIGHTER than class0 (bit0, red,
  // weight 0.3): dominant → class1, painter → class0 (the list-first / top).
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.3f, 0.9f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey0 = rc.GetColorClassLaneY(0)[p] * s;
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
  ASSERT_GT(ey1, ey0);  // class1 brighter

  std::vector<float> out;

  // dominant picks the brighter class1 (green).
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

  // painter picks the top (list-first) class0 (red), even though it is dimmer.
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
}

TEST(ComponentCompositor, DominantTieTakesFirstClass) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Equal weights → equal lanes → strict-`>` ascending scan keeps the list-first class.
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.5f, 0.5f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey0 = rc.GetColorClassLaneY(0)[p] * s;
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
  EXPECT_FLOAT_EQ(ey0, ey1);

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, out));
  // Tie → class0 (red), not class1.
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

  auto table = MakeSingletonClassTable(0b11, { kWhite, kWhite });
  RenderConsumer rc(cfg, table);
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
  const float ey0 = rc.GetColorClassLaneY(0)[p] * s;
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;

  // (2) mono exposed Y at the lit pixel = mainY * s. With disjoint single-member
  //     classes, mainY == lane0 + lane1, so the exposed mono Y equals ey0 + ey1
  //     — precisely the additive-white total. No per-lane renormalization.
  const double mono_y = static_cast<double>(raw.xyz_buffer_[p * 3 + 1]);
  const double mono_exposed_y = mono_y * s;
  EXPECT_NEAR(mono_exposed_y, static_cast<double>(ey0 + ey1), 1e-4);

  std::vector<float> out;
  ASSERT_LT(ey0 + ey1, 1.0f);  // keep below clamp so the equality is exact
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kAdditive, out));
  // Each channel of additive-white == ey0+ey1 == mono exposed Y (shared exposure).
  EXPECT_NEAR(out[p * 3 + 0], mono_exposed_y, 1e-4);
  EXPECT_NEAR(out[p * 3 + 1], mono_exposed_y, 1e-4);
  EXPECT_NEAR(out[p * 3 + 2], mono_exposed_y, 1e-4);
}

// -----------------------------------------------------------------------------
// C. Visibility orthogonality (per-color-class hide / solo), independent of mode.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, PerClassVisibilityHideAndSoloAcrossModes) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // class0 (bit0, red) is brighter than class1 (bit1, green); without hiding
  // dominant would pick class0. Lane state is shared; per-class visibility is
  // toggled on a fresh table copy for each mode assertion.
  auto lane_table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, lane_table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;

  std::vector<float> out;

  // Hide class0 → dominant must fall back to class1 (green), even though class0 brighter.
  {
    auto t = lane_table;
    t.classes_[0].visible_ = false;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kDominant, out));
    EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
    EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

    // Same table, additive: only class1 (green) contributes.
    ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kAdditive, out));
    EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
    EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);
  }

  // Solo class1 → only class1 visible regardless of mode.
  {
    auto t = lane_table;
    t.classes_[1].solo_ = true;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kDominant, out));
    EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
    EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);
  }

  // Solo overrides hide: everything hidden but class1 solo'd → class1 shows.
  {
    auto t = lane_table;
    t.classes_[0].visible_ = false;
    t.classes_[1].visible_ = false;
    t.classes_[1].solo_ = true;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kDominant, out));
    EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);
  }
}

// -----------------------------------------------------------------------------
// F. Overlap: one ray satisfies two classes → dominant/painter by z-order,
//    additive mixes. AC hard requirement (issue.md).
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, OverlapDominantPicksBrighterAdditiveMixes) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Two overlapping classes on the same bit set:
  //   classA (red)   = {any, 0b01}  → fires on masks touching bit0.
  //   classB (green) = {any, 0b11}  → fires on masks touching bit0 OR bit1.
  // Weight class-B's bit1 ray brighter so the two lanes diverge. Ray masks:
  //   0b01 (weight wA_only) → contributes to A AND B (both match bit0).
  //   0b10 (weight wB_only) → contributes to B only (bit1, not in A).
  ColorClassTable t;
  t.classes_.push_back(MakeClass(kRed, ColorClassCombine::kAny, 0b01));
  t.classes_.push_back(MakeClass(kGreen, ColorClassCombine::kAny, 0b11));
  t.referenced_mask_ = 0b11;

  RenderConsumer rc(cfg, t);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.3f, 0.9f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float eyA = rc.GetColorClassLaneY(0)[p] * s;  // just the 0.3-ray
  const float eyB = rc.GetColorClassLaneY(1)[p] * s;  // 0.3-ray + 0.9-ray
  ASSERT_GT(eyB, eyA);                                // B accumulates the 0.9-ray too

  std::vector<float> out;

  // dominant: brighter B wins → pure green, not red-tinted, not per-lane
  // normalized.
  ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kDominant, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f) << "dominant must not paint the loser's color";
  EXPECT_FLOAT_EQ(out[p * 3 + 1], eyB);

  // painter: list-first = A (red) wins as long as it has energy > 0, even though
  // B is brighter — mirrors PainterVsDominantDivergeWhenTopClassDimmer but under
  // OVERLAPPING classes (AC "z-order 定色").
  ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], eyA);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);

  // additive: red*eyA + green*eyB; both below clamp so exact per-channel.
  ASSERT_LT(eyA + eyB, 1.0f);
  ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kAdditive, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], eyA);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], eyB);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);
}

// -----------------------------------------------------------------------------
// G. Cross-layer AND: combine:"all" class only shows where all members hit.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, CrossLayerAllOnlyShowsWhereAllMembersHit) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Single class {all, 0b11}: only rays whose masks contain BOTH bit0 and bit1
  // contribute. Ray masks match test_render_consumer_component_lanes' cross-
  // layer AND fixture:
  //   0b01 → only layer A → skipped
  //   0b10 → only layer B → skipped
  //   0b11 → both layers   → contributes
  //   0b00 → neither       → skipped
  ColorClassTable t;
  t.classes_.push_back(MakeClass(kRed, ColorClassCombine::kAll, 0b11));
  t.referenced_mask_ = 0b11;

  RenderConsumer rc(cfg, t);
  rc.Consume(MakeBatch({ 0b01, 0b10, 0b11, 0b00 }, { 0.5f, 0.6f, 0.7f, 0.9f }));
  rc.PrepareSnapshot();

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kDominant, out));

  // Exactly ONE pixel should be lit (the one all sky-up rays land in), and it
  // must be pure red (class0's color × its exposed value).
  int lit_count = 0;
  for (int p = 0; p < total_pix; ++p) {
    const float r = out[p * 3 + 0];
    const float g = out[p * 3 + 1];
    const float b = out[p * 3 + 2];
    if (r == 0.0f && g == 0.0f && b == 0.0f) {
      continue;
    }
    ++lit_count;
    EXPECT_GT(r, 0.0f) << "combine:\"all\" class must paint its color where it fires";
    EXPECT_FLOAT_EQ(g, 0.0f);
    EXPECT_FLOAT_EQ(b, 0.0f);
  }
  EXPECT_EQ(lit_count, 1) << "only the ray with mask=0b11 should contribute to the all-class lane";
}

// -----------------------------------------------------------------------------
// F2. z_order decoupling (task-342.2): changing z_order_ reorders the draw
//     sequence (painter top layer / dominant tie winner) but NEVER re-binds a
//     class to a different physical Y-lane — the compositor must index lanes by
//     the ORIGINAL vector position, not the sorted position. This is the direct
//     regression against the "naively re-sort the classes_ vector" trap (plan
//     §3.2 key design point 1): a wrong impl would paint a lane's accumulated
//     energy with another class's color.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, ZOrderReordersDrawButNotLaneBinding) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // class0 = red on bit0, class1 = green on bit1. Ray masks {0b01, 0b10} so lane0
  // accumulates the 0b01 ray and lane1 the 0b10 ray — a permanent binding built at
  // RenderConsumer construction.
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ExposureScale();
  const float lane0 = rc.GetColorClassLaneY(0)[p];  // class0's physical lane
  const float lane1 = rc.GetColorClassLaneY(1)[p];  // class1's physical lane
  const float ey0 = lane0 * s;
  const float ey1 = lane1 * s;

  std::vector<float> out;

  // Baseline z_order (0,1): painter draws list-first class0 (red) on top.
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);

  // Swap z_order so class1 has the LOWER rank (draws first / top for painter). Only the
  // display-time field changes; the vector order and lane data are untouched.
  {
    auto t = table;
    t.classes_[0].z_order_ = 1;
    t.classes_[1].z_order_ = 0;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kPainter, out));
    // Painter top layer is now class1 (green) — draw order changed...
    EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
    EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1)
        << "green must be painted with class1's OWN exposed lane value (lane1), not lane0's — "
           "z_order must not re-bind lane index to class";
    // ...but the value equals class1's own physical lane (lane1), proving the lane→class
    // binding did NOT follow the reorder. If the impl naively re-sorted the vector, green
    // would incorrectly carry lane0's value (ey0 != ey1 here since weights 0.6 != 0.4).
    ASSERT_NE(ey0, ey1);
  }

  // Dominant tie: give both lanes equal energy and flip z_order to prove the tie winner
  // follows draw order (ascending z_order), independent of lane index.
  {
    RenderConsumer rc_tie(cfg, table);
    rc_tie.Consume(MakeBatch({ 0b01, 0b10 }, { 0.5f, 0.5f }));
    rc_tie.PrepareSnapshot();
    const int pt = FindLitPixel(rc_tie, total_pix);
    ASSERT_GE(pt, 0);
    const float st = rc_tie.ExposureScale();
    const float eyt = rc_tie.GetColorClassLaneY(0)[pt] * st;  // == lane1 too (equal weights)

    // Default z_order (0,1): dominant tie → class0 (red).
    auto t0 = table;
    ASSERT_TRUE(CompositeColorClassesLinear(rc_tie, t0, CompositeMode::kDominant, out));
    EXPECT_FLOAT_EQ(out[pt * 3 + 0], eyt);
    EXPECT_FLOAT_EQ(out[pt * 3 + 1], 0.0f);

    // Flip z_order so class1 is scanned first: dominant tie → class1 (green).
    auto t1 = table;
    t1.classes_[0].z_order_ = 1;
    t1.classes_[1].z_order_ = 0;
    ASSERT_TRUE(CompositeColorClassesLinear(rc_tie, t1, CompositeMode::kDominant, out));
    EXPECT_FLOAT_EQ(out[pt * 3 + 0], 0.0f);
    EXPECT_FLOAT_EQ(out[pt * 3 + 1], eyt);
  }
}

// -----------------------------------------------------------------------------
// H. ParseCompositeMode string handling.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, ParseCompositeModeKnownStrings) {
  EXPECT_EQ(ParseCompositeMode("dominant"), CompositeMode::kDominant);
  EXPECT_EQ(ParseCompositeMode("additive"), CompositeMode::kAdditive);
  EXPECT_EQ(ParseCompositeMode("painter"), CompositeMode::kPainter);
}

TEST(ComponentCompositor, ParseCompositeModeUnknownFallsBackToDominant) {
  EXPECT_EQ(ParseCompositeMode("bogus-typo"), CompositeMode::kDominant);
  EXPECT_EQ(ParseCompositeMode(""), CompositeMode::kDominant);
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

  // Design 2: color bits come from a raypath_color config carried on the
  // SessionSpec (see task-engine-redirect-design2 Step 5). The 3 predicates
  // below reproduce the pre-migration three-arc bit allocation:
  //   bit 0: crystal 0 (whole crystal — its only filter is EE{min=1}, which
  //          is match-all across surviving paths on that placement)
  //   bit 1: crystal 1, EE{min=2, max=2}
  //   bit 2: crystal 1, EE{min=3}
  auto rpc = std::make_shared<RaypathColorConfig>();
  rpc->classes_.push_back(
      MakeCls({ 1.0f, 0.0f, 0.0f }, { MakeRef(0, 0, SimpleFilterParam{ EntryExitFilterParam{} }) }));
  rpc->classes_.push_back(
      MakeCls({ 0.0f, 1.0f, 0.0f },
              { MakeRef(0, 1, SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 2, 2 } }) }));
  rpc->classes_.push_back(MakeCls(
      { 0.0f, 0.0f, 1.0f },
      { MakeRef(0, 1, SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 3, std::nullopt } }) }));

  RenderConfig render = MakeRenderConfig(64);
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.raypath_color = rpc;
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
  auto class_table = MakeSingletonClassTable(kColored, { kRed, kGreen, kBlue });
  RenderConsumer rc(render, class_table);
  rc.Consume(data);
  rc.PrepareSnapshot();

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, class_table, CompositeMode::kDominant, out));

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

TEST(ComponentCompositor, EmptyClassTableProducesNoComposite) {
  RenderConfig cfg = MakeRenderConfig(8);
  RenderConsumer rc(cfg, ColorClassTable{});  // pre-336 path, no lanes
  rc.Consume(MakeBatch({ 0b00 }, { 0.5f }));
  rc.PrepareSnapshot();

  ColorClassTable empty_table;         // referenced_mask_ == 0
  std::vector<float> out = { 42.0f };  // sentinel — must be left untouched
  EXPECT_FALSE(CompositeColorClassesLinear(rc, empty_table, CompositeMode::kDominant, out));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0], 42.0f) << "compositor must not touch the output when the class table is empty";
}

// -----------------------------------------------------------------------------
// Config-side smoke: RaypathColorConfig JSON round-trip (kept here to minimise
// diff churn per plan risk 3; the DTO itself is exercised more fully in
// test_color_class_table.cpp / test_raypath_color_config.cpp).
// (MakeCls / MakeRef were promoted into the top-level anonymous namespace so
// DominantThreeArcsNoPhantomHue can share them.)
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, RaypathColorConfigJsonFormsRoundTrip) {
  // Non-default mode → object form {mode, classes}, preserving visible/solo.
  // Design 2: match-all whole-crystal ref (default NoneFilterParam predicate)
  // is the Design-2 equivalent of the pre-migration `has_filter=true,
  // has_summand=true, summand=0` shape on a single-summand filter.
  RaypathColorConfig cfg;
  cfg.mode_ = "additive";
  cfg.classes_.push_back(MakeCls({ 0.2f, 0.4f, 0.6f }, { MakeRef(0, 1) }, false, true));
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
  EXPECT_TRUE(std::holds_alternative<NoneFilterParam>(back.classes_[0].match_[0].predicate_));

  // Default mode → bare array form, still round-trips.
  RaypathColorConfig dom;
  dom.classes_.push_back(MakeCls({ 1.0f, 0.0f, 0.0f }, { MakeRef(0, 0) }));
  nlohmann::json jd = dom;
  EXPECT_TRUE(jd.is_array()) << "default-mode config must serialize as a bare array";
  RaypathColorConfig dback = jd.get<RaypathColorConfig>();
  EXPECT_EQ(dback.mode_, "dominant");
  ASSERT_EQ(dback.classes_.size(), 1u);
  EXPECT_TRUE(dback.classes_[0].visible_);
  EXPECT_FALSE(dback.classes_[0].solo_);
}

// -----------------------------------------------------------------------------
// task-345.3: display-time EV multiplier + participating-P99 union anchor.
//
// The invariants under test map to the plan's Step 1 / Step 2 core acceptance:
//   1. `display_exposure_scale` scales EVERY mode's output linearly and does
//      NOT introduce any per-class renormalization (extends the shared-exposure
//      invariant to the new axis).
//   2. Additive mode's clamp bites AFTER the display scale is applied — a
//      value that stays sub-clamp at scale 1.0 but reaches clamp at scale 2.0
//      must saturate, not be truncated pre-scale.
//   3. `ComputeParticipatingP99Y` only ever sees participating (visible/solo)
//      classes' UNEXPOSED lane values; hidden classes' bright pixels never
//      leak into the anchor.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, DisplayExposureScalesEveryModeLinearly) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  // Keep exposed values small so the composite output stays sub-clamp at
  // both display scales — the equality below is otherwise saturated.
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.06f, 0.04f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);

  std::vector<float> out_1x;
  std::vector<float> out_2x;
  for (CompositeMode mode : { CompositeMode::kDominant, CompositeMode::kAdditive, CompositeMode::kPainter }) {
    ASSERT_TRUE(CompositeColorClassesLinear(rc, table, mode, 1.0f, out_1x, nullptr));
    ASSERT_TRUE(CompositeColorClassesLinear(rc, table, mode, 2.0f, out_2x, nullptr));
    for (int i = 0; i < 3; ++i) {
      // 2x display scale → every channel doubles (linear pre-clamp region).
      // The dominant/painter branches pick the SAME winner regardless of
      // scale (both lanes scale uniformly, argmax invariant) — no hue shift.
      EXPECT_NEAR(out_2x[p * 3 + i], out_1x[p * 3 + i] * 2.0f, 1e-4)
          << "mode=" << static_cast<int>(mode) << " channel=" << i;
    }
  }
}

TEST(ComponentCompositor, DisplayExposureClampBitesAfterScale) {
  // additive mode: at scale 1.0 the additive sum is below the clamp; scale
  // 8.0 pushes it past 1.0 and the compositor must clamp AFTER scaling.
  // Truncating pre-scale would return a proportionally-scaled sub-1 value.
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float ey_at_1x = rc.GetColorClassLaneY(0)[p] * rc.ExposureScale();
  ASSERT_LT(ey_at_1x, 1.0f);
  ASSERT_GT(ey_at_1x * 8.0f, 1.0f);

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kAdditive, 8.0f, out, nullptr));
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(out[p * 3 + i], 1.0f);  // saturated to clamp ceiling.
  }
}

TEST(ComponentCompositor, ParticipatingP99IgnoresHiddenClass) {
  // Two classes: class0 = dim, class1 = 10x brighter. Hide class1 and confirm
  // the participating P99 comes strictly from class0's lane — bright hidden
  // pixels must not leak into the anchor (the "no full-spectrum dilution"
  // fix motivating task-345.3).
  constexpr int kRes = 4;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kWhite, kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.05f, 0.5f }));
  rc.PrepareSnapshot();

  // Baseline: both classes visible — P99 should be dominated by class1.
  float p99_both = 0.0f;
  {
    std::vector<float> out;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, 1.0f, out, &p99_both));
    EXPECT_GT(p99_both, 0.0f);
  }

  // Hide class1 — the bright lane must vanish from the anchor.
  ColorClassTable hidden_table = table;
  hidden_table.classes_[1].visible_ = false;
  float p99_hidden = 0.0f;
  {
    std::vector<float> out;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, hidden_table, CompositeMode::kDominant, 1.0f, out, &p99_hidden));
  }
  EXPECT_GT(p99_both, p99_hidden * 3.0f)
      << "hiding the 10x brighter class must significantly lower the participating P99";

  // Solo class0 alone — same effect (solo restricts participating set).
  ColorClassTable solo_table = table;
  solo_table.classes_[0].solo_ = true;
  float p99_solo = 0.0f;
  {
    std::vector<float> out;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, solo_table, CompositeMode::kDominant, 1.0f, out, &p99_solo));
  }
  EXPECT_NEAR(p99_solo, p99_hidden, 1e-6f) << "solo of the only remaining class must match hiding the others";
}

TEST(ComponentCompositor, ParticipatingP99IndependentOfDisplayExposureScale) {
  // The anchor is over UNEXPOSED lane values — changing the display EV must
  // NOT move the P99 (otherwise the auto-EV feedback loop compounds).
  constexpr int kRes = 3;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc.PrepareSnapshot();

  float p99_a = 0.0f;
  float p99_b = 0.0f;
  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, 0.25f, out, &p99_a));
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, 4.0f, out, &p99_b));
  EXPECT_FLOAT_EQ(p99_a, p99_b);
}

}  // namespace
}  // namespace lumice
