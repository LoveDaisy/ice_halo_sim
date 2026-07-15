// Tests for task-339.4 (per-color-class compositor): the three display-time
// composite modes (dominant / additive / painter), the orthogonal show/hide/
// solo visibility axis, and — the crux — the SHARED-EXPOSURE invariant: every
// mode multiplies each color-class's raw-Y lane by ONE scalar `s` and nothing
// else. There is never a per-lane / per-composite renormalization (that was
// the spike's false-color bug).
//
// task-347 (Fix B): `s` is now server-side self-anchored on the participating
// -P99 (RenderConsumer::ParticipatingExposureScale(p99)) rather than the mono
// ExposureScale — so hiding a bright class instantly re-anchors the composite
// off the surviving dim class(es). The SHARED-nature of `s` still holds
// (single float, one per participating-class union), but the ANCHOR formula
// changed. Every test below that predicts a pixel value now sources `s` via
// a per-variant p99 probe + rc.ParticipatingExposureScale, not the mono
// ExposureScale (which would decouple silently from the compositor after Fix B).
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

// task-347 (Fix B) helper: run the compositor once purely to retrieve the
// participating-P99 anchor without asserting on the RGB output; used by the
// exposure-anchor tests below to independently recompute the expected `s`
// via rc.ParticipatingExposureScale(p99). Assumes the class table has at
// least one active lit lane (participating union non-empty).
float FetchParticipatingP99(const RenderConsumer& rc, const ColorClassTable& table, CompositeMode mode) {
  std::vector<float> tmp;
  float p99 = 0.0f;
  const bool ok = CompositeColorClassesLinear(rc, table, mode, 1.0f, tmp, &p99);
  (void)ok;
  return p99;
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
  // task-347 (Fix B): the compositor now anchors `s` off the participating-P99,
  // not off the mono ExposureScale. Fetch p99 from a probe compositor call, then
  // independently recompute the expected `s` via rc.ParticipatingExposureScale(p99).
  // This is the same "independent cross-check" style the SharedExposureNoSelfNormalization
  // test uses further down (plan §4 Step 2 test point).
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kDominant);
  ASSERT_GT(p99, 0.0f);
  const float s = rc.ParticipatingExposureScale(p99);
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

  // painter (doc §4.8): Porter-Duff over,
  // list-first = top. Both classes contribute because alpha < 1: top-layer
  // class0 (red) contributes `alpha0 * red`, class1 (green) shows through with
  // transmittance `1 - alpha0` → `(1-alpha0) * alpha1 * green`. With ey_c < 1,
  // alpha_c == ey_c (min-clamp is a no-op).
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], (1.0f - ey0) * ey1);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);
}

TEST(ComponentCompositor, PainterAlphaOverBlendsTopAndBottomDominantPicksBrighter) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Now class1 (bit1, green, weight 0.9) is BRIGHTER than class0 (bit0, red,
  // weight 0.3): dominant picks the brighter class1 (green). Painter (post
  // doc §4.8) blends both via Porter-Duff
  // over — the top-layer class0 partially occludes but does NOT hide class1.
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.3f, 0.9f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  // task-347 (Fix B): recover `s` via the participating-P99 anchor (see
  // DominantAdditivePainterPerPixelMath for rationale). display_exposure_scale
  // defaults to 1.0f in the probe, so `s == A` (the self-anchor used by
  // painter's alpha path — no divergence between old `s` and new `A` here).
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kDominant);
  ASSERT_GT(p99, 0.0f);
  const float s = rc.ParticipatingExposureScale(p99);
  const float ey0 = rc.GetColorClassLaneY(0)[p] * s;
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
  ASSERT_GT(ey1, ey0);  // class1 brighter

  std::vector<float> out;

  // dominant picks the brighter class1 (green).
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1);

  // painter alpha-over: top (list-first) class0 (red) contributes `alpha0 * red`,
  // class1 (green) shows through with transmittance `(1 - alpha0)` — both classes
  // contribute, unlike the pre-§4.8 binary-occluder behaviour that painted only
  // red. alpha_c == ey_c since ey < 1 (min-clamp is a no-op).
  ASSERT_LT(ey0, 1.0f);
  ASSERT_LT(ey1, 1.0f);
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], (1.0f - ey0) * ey1);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);
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
  // task-347 (Fix B): anchor `s` via ParticipatingExposureScale(p99).
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kDominant);
  ASSERT_GT(p99, 0.0f);
  const float s = rc.ParticipatingExposureScale(p99);
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
// A2. doc §4.8 painter-specific coverage:
//     black-hole fix, EV independence of occluder structure, single-class
//     f(ey)·color (no ey² double-count), full-opacity color ceiling.
// -----------------------------------------------------------------------------

// (1) Dim top-layer class must NOT 100%-mask a bright bottom-layer class.
// This is the pre-§4.8 painter's "black hole": a class with a small positive
// ey on top would blank out every class below it — the whole point of the
// redesign. The post-§4.8 recurrence gives `T = 1 - alpha_top` transmittance
// to the layers below, so the bright bottom shines through with (1-alpha_top)
// times its own contribution.
TEST(ComponentCompositor, PainterAlphaOverDimTopDoesNotBlockBrightBottom) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // class0 (bit0, red) is the DIM top layer; class1 (bit1, green) is the BRIGHT
  // bottom. Weights push class1 significantly brighter than class0 so the pre-§4.8
  // "list-first wins" bug would output near-black-red instead of green shining through.
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.05f, 0.9f }));  // top very dim, bottom bright
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kPainter);
  ASSERT_GT(p99, 0.0f);
  const float A = rc.ParticipatingExposureScale(p99);
  const float ey0 = rc.GetColorClassLaneY(0)[p] * A;  // top, dim
  const float ey1 = rc.GetColorClassLaneY(1)[p] * A;  // bottom, bright
  ASSERT_LT(ey0, 1.0f);
  ASSERT_LT(ey1, 1.0f);
  ASSERT_LT(ey0, ey1);  // top strictly dimmer than bottom

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  // Green (bottom) must contribute (1-ey0)*ey1. Because ey0 is small (top is
  // dim), the transmittance (1-ey0) is close to 1, so green stays close to ey1
  // — dominates the red contribution ey0.
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], (1.0f - ey0) * ey1);
  EXPECT_GT(out[p * 3 + 1], out[p * 3 + 0]) << "bright bottom must dominate through dim top (black-hole fix)";
}

// (2) EV pulls only brightness, not occluder structure — the whole point of
// EV decoupling. Doubling display_exposure_scale below the clamp doubles every
// channel exactly; the ratio between channels (i.e. the alpha-over structure)
// is invariant.
TEST(ComponentCompositor, PainterDisplayEvOnlyScalesBrightnessNotOccluderStructure) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  // Two classes both partially opaque, both contribute — use very small values
  // so that after 2x EV boost neither channel touches the clamp.
  auto table = MakeSingletonClassTable(0b11, { kRed, kGreen });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.15f, 0.20f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);

  std::vector<float> out_lo;
  std::vector<float> out_hi;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, 1.0f, out_lo, nullptr));
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, 2.0f, out_hi, nullptr));

  // Neither channel may hit clamp under 2x (assertions on ratio require it).
  ASSERT_LT(out_hi[p * 3 + 0], 1.0f);
  ASSERT_LT(out_hi[p * 3 + 1], 1.0f);
  // Every non-zero channel doubled — brightness scales, structure invariant.
  EXPECT_NEAR(out_hi[p * 3 + 0], 2.0f * out_lo[p * 3 + 0], 1e-5f);
  EXPECT_NEAR(out_hi[p * 3 + 1], 2.0f * out_lo[p * 3 + 1], 1e-5f);
  EXPECT_FLOAT_EQ(out_lo[p * 3 + 2], 0.0f);
  EXPECT_FLOAT_EQ(out_hi[p * 3 + 2], 0.0f);
}

// (3) Single-class output is `f(ey) * color`, NOT `ey * color * ey = ey² * color`.
// Guards against the reviewer-flagged "color slot holds color*ey" anti-pattern
// (§4.8): if the pure-hue rule were broken, this test's assertion would show
// `ey² * red` = a much smaller value than `ey * red`.
TEST(ComponentCompositor, PainterSingleClassNoBrightnessDoubleCount) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  auto table = MakeSingletonClassTable(0b01, { kRed });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kPainter);
  ASSERT_GT(p99, 0.0f);
  const float A = rc.ParticipatingExposureScale(p99);
  const float ey0 = rc.GetColorClassLaneY(0)[p] * A;
  ASSERT_LT(ey0, 1.0f);
  ASSERT_GT(ey0, 0.01f);  // above the noise floor so ey vs ey² is visible

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  // Expected: alpha * color = ey0 * (1, 0, 0). Anti-pattern would give ey0 * ey0.
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);
}

// (4) Full opacity (ey >= 1) caps painter output at the class color — no HDR
// headroom. This is a designed property (§4.8), not a bug: painter is a display
// mode; overexposure is dominant/additive's territory. Verify via a large
// display_exposure_scale that pushes alpha_top to the min-clamp.
TEST(ComponentCompositor, PainterFullOpacityCeilsAtClassColor) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  auto table = MakeSingletonClassTable(0b01, { kRed });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);

  // Painter alpha uses ONLY A (self-anchor), not display_exposure_scale — so
  // a large display EV cannot drive alpha above the min(ey, 1) clamp on its
  // own. To realise alpha == 1, the raw lane * A must reach 1; here we bump
  // intensity_factor so A grows enough that lane * A >= 1. (This mirrors the
  // "class energy fills the pixel" scenario in production, where full opacity
  // is possible but capped.)
  //
  // Use a cranked intensity_factor to guarantee alpha == 1.
  RenderConfig cfg_bright = cfg;
  cfg_bright.intensity_factor_ = 1e6f;  // driven high so lane * A >= 1
  RenderConsumer rc2(cfg_bright, table);
  rc2.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc2.PrepareSnapshot();

  std::vector<float> out;
  // Post-composite EV is also 1.0, so a saturated alpha yields exactly the
  // class color (1, 0, 0) — no HDR headroom.
  ASSERT_TRUE(CompositeColorClassesLinear(rc2, table, CompositeMode::kPainter, 1.0f, out, nullptr));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 1.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);

  // Post-composite EV > 1 keeps the clamp firing at the class color ceiling —
  // no overshoot, no HDR headroom.
  ASSERT_TRUE(CompositeColorClassesLinear(rc2, table, CompositeMode::kPainter, 4.0f, out, nullptr));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 1.0f);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], 0.0f);
}

// -----------------------------------------------------------------------------
// B. Shared exposure (crux): every participating lane sees ONE scalar `s`,
//    never per-lane / per-composite renormalization. task-347 (Fix B): `s` is
//    now sourced from the participating-P99 self-anchor (not the mono
//    ExposureScale). The invariant under test is the SHARED-nature of `s`,
//    not its numeric equality to the mono scale — and it is verified by an
//    independent recomputation of the exposed-Y equality (additive white
//    == sum of the per-class exposed lanes) using the anchor from a probe.
// -----------------------------------------------------------------------------
TEST(ComponentCompositor, SharedExposureNoSelfNormalization) {
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);

  auto table = MakeSingletonClassTable(0b11, { kWhite, kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  // (1) task-347 (Fix B): the compositor's `s` must equal the ParticipatingExposureScale
  //     applied to the current participating-P99 anchor — the sole path by which
  //     hiding a bright class re-brightens the rest inside one DoSnapshot.
  //     Independent reconstruction: fetch p99 from a probe compositor call,
  //     then recompute the expected `s` via a hand-recreated version of the
  //     ParticipatingExposureScale formula. If the formula in render.cpp is
  //     ever tweaked, this test must be updated in lock-step (a04: mechanism-
  //     layer cross-check, not a same-source self-check).
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kAdditive);
  ASSERT_GT(p99, 0.0f);
  // snapshot_intensity_ does NOT appear in the formula — it cancels against the
  // mono shader's downstream `intensity_scale = intensity_factor/snapshot_intensity`
  // step, so the effective per-pixel multiplier reduces to target_linear/p99. See
  // the derivation block above ParticipatingExposureScale in render.cpp.
  constexpr float kTargetWhite = 135.0f;
  constexpr float kTargetSrgb = kTargetWhite / 255.0f;
  const float target_linear =
      kTargetSrgb <= 0.04045f ? kTargetSrgb / 12.92f : std::pow((kTargetSrgb + 0.055f) / 1.055f, 2.4f);
  const float expected_s = cfg.intensity_factor_ * target_linear / p99;
  EXPECT_NEAR(rc.ParticipatingExposureScale(p99), expected_s, expected_s * 1e-5f);

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  const float s = rc.ParticipatingExposureScale(p99);
  const float ey0 = rc.GetColorClassLaneY(0)[p] * s;
  const float ey1 = rc.GetColorClassLaneY(1)[p] * s;

  // (2) SHARED-EXPOSURE INVARIANT: additive-white composite at the lit pixel
  //     must equal ey0 + ey1 — the same shared `s` was applied to both lanes,
  //     no per-lane renormalization. This is the direct falsification of the
  //     scrum-336 spike's false-color bug and remains meaningful under Fix B
  //     (the anchor formula changed but the "one `s` for all lanes" contract
  //     didn't).
  std::vector<float> out;
  ASSERT_LT(ey0 + ey1, 1.0f);  // keep below clamp so the equality is exact
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kAdditive, out));
  const double sum_exposed_y = static_cast<double>(ey0 + ey1);
  EXPECT_NEAR(out[p * 3 + 0], sum_exposed_y, 1e-4);
  EXPECT_NEAR(out[p * 3 + 1], sum_exposed_y, 1e-4);
  EXPECT_NEAR(out[p * 3 + 2], sum_exposed_y, 1e-4);
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

  std::vector<float> out;

  // task-347 (Fix B): each visibility variant below re-anchors on its OWN
  // participating-P99 (that is precisely the behavior under test). Recompute
  // the expected class1 exposed-Y per variant via a per-variant p99 probe —
  // do NOT hoist a single `s` out (it would silently paper over the anchor
  // change that motivates this whole task).

  // Hide class0 → dominant must fall back to class1 (green), even though class0 brighter.
  {
    auto t = lane_table;
    t.classes_[0].visible_ = false;
    const float p99 = FetchParticipatingP99(rc, t, CompositeMode::kDominant);
    ASSERT_GT(p99, 0.0f);
    const float s = rc.ParticipatingExposureScale(p99);
    const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
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
    const float p99 = FetchParticipatingP99(rc, t, CompositeMode::kDominant);
    ASSERT_GT(p99, 0.0f);
    const float s = rc.ParticipatingExposureScale(p99);
    const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
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
    const float p99 = FetchParticipatingP99(rc, t, CompositeMode::kDominant);
    ASSERT_GT(p99, 0.0f);
    const float s = rc.ParticipatingExposureScale(p99);
    const float ey1 = rc.GetColorClassLaneY(1)[p] * s;
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
  // task-347 (Fix B): anchor `s` via ParticipatingExposureScale(p99).
  const float p99 = FetchParticipatingP99(rc, t, CompositeMode::kDominant);
  ASSERT_GT(p99, 0.0f);
  const float s = rc.ParticipatingExposureScale(p99);
  const float eyA = rc.GetColorClassLaneY(0)[p] * s;  // just the 0.3-ray
  const float eyB = rc.GetColorClassLaneY(1)[p] * s;  // 0.3-ray + 0.9-ray
  ASSERT_GT(eyB, eyA);                                // B accumulates the 0.9-ray too

  std::vector<float> out;

  // dominant: brighter B wins → pure green, not red-tinted, not per-lane
  // normalized.
  ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kDominant, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], 0.0f) << "dominant must not paint the loser's color";
  EXPECT_FLOAT_EQ(out[p * 3 + 1], eyB);

  // painter (doc §4.8) alpha-over on
  // OVERLAPPING classes: list-first = A (red) is top, its alpha_A = eyA (< 1)
  // does not fully occlude, so class B (green) still shows through with
  // transmittance (1 - alpha_A). Divergence from dominant survives (dominant
  // paints pure green, painter paints red+attenuated-green), but the pre-§4.8
  // binary occluder was "red only" — now it's blend.
  ASSERT_LT(eyA, 1.0f);
  ASSERT_LT(eyB, 1.0f);
  ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], eyA);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], (1.0f - eyA) * eyB);
  EXPECT_FLOAT_EQ(out[p * 3 + 2], 0.0f);

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
  // task-347 (Fix B): anchor `s` via ParticipatingExposureScale(p99). z_order
  // does NOT change the participating set (all classes visible), so this p99
  // is stable across the baseline and z_order-swap variants.
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kPainter);
  ASSERT_GT(p99, 0.0f);
  const float s = rc.ParticipatingExposureScale(p99);
  const float lane0 = rc.GetColorClassLaneY(0)[p];  // class0's physical lane
  const float lane1 = rc.GetColorClassLaneY(1)[p];  // class1's physical lane
  const float ey0 = lane0 * s;
  const float ey1 = lane1 * s;

  std::vector<float> out;

  // Baseline z_order (0,1): painter alpha-over walks class0 first (top layer).
  // Under §4.8: out = (alpha0*red + (1-alpha0)*alpha1*0, (1-alpha0)*alpha1*green
  //                    + alpha0*0, 0) = (ey0, (1-ey0)*ey1, 0) with alpha_c==ey_c
  // when ey_c < 1.
  ASSERT_LT(ey0, 1.0f);
  ASSERT_LT(ey1, 1.0f);
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kPainter, out));
  EXPECT_FLOAT_EQ(out[p * 3 + 0], ey0);
  EXPECT_FLOAT_EQ(out[p * 3 + 1], (1.0f - ey0) * ey1);

  // Swap z_order so class1 has the LOWER rank (drawn FIRST / on top for painter).
  // Only the display-time field changes; the vector order and lane data are untouched.
  {
    auto t = table;
    t.classes_[0].z_order_ = 1;
    t.classes_[1].z_order_ = 0;
    ASSERT_TRUE(CompositeColorClassesLinear(rc, t, CompositeMode::kPainter, out));
    // Painter walks class1 first now. out = ((1-ey1)*ey0, ey1, 0). Green must
    // be painted with class1's OWN exposed lane value (ey1), not ey0 — z_order
    // reorders DRAW sequence, it does NOT re-bind physical lane → class. If the
    // impl naively re-sorted the vector, green would carry lane0's value (ey0
    // != ey1 here since weights 0.6 != 0.4).
    EXPECT_FLOAT_EQ(out[p * 3 + 0], (1.0f - ey1) * ey0);
    EXPECT_FLOAT_EQ(out[p * 3 + 1], ey1)
        << "green must reflect class1's OWN lane (ey1), not the swapped position's lane";
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
    // task-347 (Fix B): rc_tie has different lane values → different p99 than
    // the outer scope; probe locally.
    const float p99_tie = FetchParticipatingP99(rc_tie, table, CompositeMode::kDominant);
    ASSERT_GT(p99_tie, 0.0f);
    const float st = rc_tie.ParticipatingExposureScale(p99_tie);
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

TEST(ComponentCompositor, ParseCompositeModeUnknownFallsBackToPainter) {
  // doc §4.8: default composite mode is
  // now painter; the unknown-string fallback follows the default.
  EXPECT_EQ(ParseCompositeMode("bogus-typo"), CompositeMode::kPainter);
  EXPECT_EQ(ParseCompositeMode(""), CompositeMode::kPainter);
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

  // task-345.3 AC1 white-box: display_exposure_scale must NEVER shift the hue at
  // any non-zero pixel — dominant is argmax over lane*s with a shared s, so
  // scaling s only scales the pre-argmax product uniformly. Verify by comparing
  // a scale=0.5 render against the scale=1.0 baseline pixel-for-pixel:
  //   (a) argmax (which lane wins) is invariant;
  //   (b) the winning channel's value scales exactly linearly (no re-normalization,
  //       no per-lane bleed).
  // This is the strongest structural proof of "no phantom hue under EV" that
  // can be made without an on-screen owner check (AC5) — it directly encodes
  // the color-space invariant the scrum-336 spike bug violated.
  std::vector<float> out_half;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, class_table, CompositeMode::kDominant, 0.5f, out_half, nullptr));
  ASSERT_EQ(out_half.size(), out.size());
  int compared = 0;
  for (int p = 0; p < total_pix; ++p) {
    const float r1 = out[p * 3 + 0];
    const float g1 = out[p * 3 + 1];
    const float b1 = out[p * 3 + 2];
    const float r_h = out_half[p * 3 + 0];
    const float g_h = out_half[p * 3 + 1];
    const float b_h = out_half[p * 3 + 2];
    if (r1 == 0.0f && g1 == 0.0f && b1 == 0.0f) {
      // Background pixel — must stay background at the alternate scale, too.
      EXPECT_FLOAT_EQ(r_h, 0.0f);
      EXPECT_FLOAT_EQ(g_h, 0.0f);
      EXPECT_FLOAT_EQ(b_h, 0.0f);
      continue;
    }
    // Winning channel invariance: whichever channel is non-zero at scale 1.0 must
    // be the SAME channel at scale 0.5 (dominant argmax is invariant under uniform
    // rescale). And its value must be exactly r1 * 0.5.
    if (r1 > 0.0f) {
      EXPECT_GT(r_h, 0.0f) << "pixel " << p << ": red arc lost dominance under EV=-1";
      EXPECT_FLOAT_EQ(g_h, 0.0f);
      EXPECT_FLOAT_EQ(b_h, 0.0f);
      EXPECT_NEAR(r_h, r1 * 0.5f, 1e-5f);
    } else if (g1 > 0.0f) {
      EXPECT_GT(g_h, 0.0f);
      EXPECT_FLOAT_EQ(r_h, 0.0f);
      EXPECT_FLOAT_EQ(b_h, 0.0f);
      EXPECT_NEAR(g_h, g1 * 0.5f, 1e-5f);
    } else {
      EXPECT_GT(b_h, 0.0f);
      EXPECT_FLOAT_EQ(r_h, 0.0f);
      EXPECT_FLOAT_EQ(g_h, 0.0f);
      EXPECT_NEAR(b_h, b1 * 0.5f, 1e-5f);
    }
    ++compared;
  }
  // Three-arcs fixture is thin arcs (~O(10) lit pixels/arc at 64x64) — set the floor
  // just above the arc-count total so an empty three-arc render can't false-pass.
  EXPECT_GT(compared, red_px + green_px + blue_px - 1)
      << "AC1 invariance check must cover every non-background pixel found in the baseline";
  EXPECT_GE(compared, 3) << "at least one pixel from each of the three arcs expected";
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
  // doc §4.8: default is now painter (kDefaultCompositeMode).
  RaypathColorConfig dom;
  dom.classes_.push_back(MakeCls({ 1.0f, 0.0f, 0.0f }, { MakeRef(0, 0) }));
  nlohmann::json jd = dom;
  EXPECT_TRUE(jd.is_array()) << "default-mode config must serialize as a bare array";
  RaypathColorConfig dback = jd.get<RaypathColorConfig>();
  EXPECT_EQ(dback.mode_, "painter");
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
  // additive mode: at scale 1.0 the additive sum is below the clamp; a larger
  // display_exposure_scale pushes it past 1.0 and the compositor must clamp
  // AFTER scaling. Truncating pre-scale would return a proportionally-scaled
  // sub-1 value.
  constexpr int kRes = 3;
  const int total_pix = kRes * kRes;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc.PrepareSnapshot();

  const int p = FindLitPixel(rc, total_pix);
  ASSERT_GE(p, 0);
  // task-347 (Fix B): baseline exposed-Y comes from the participating-P99
  // self-anchor. Sanity-check that at scale=1x we are safely sub-clamp and
  // pick a scale large enough to blow past 1.0 given the anchor formula.
  const float p99 = FetchParticipatingP99(rc, table, CompositeMode::kAdditive);
  ASSERT_GT(p99, 0.0f);
  const float ey_at_1x = rc.GetColorClassLaneY(0)[p] * rc.ParticipatingExposureScale(p99);
  ASSERT_LT(ey_at_1x, 1.0f);
  // Choose scale so the exposed Y comfortably exceeds 1.0.
  const float scale = 4.0f / ey_at_1x;
  ASSERT_GT(ey_at_1x * scale, 1.0f);

  std::vector<float> out;
  ASSERT_TRUE(CompositeColorClassesLinear(rc, table, CompositeMode::kAdditive, scale, out, nullptr));
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

// -----------------------------------------------------------------------------
// task-347 (Fix B) direct unit coverage: ParticipatingExposureScale contract
// (0-guard branches + numeric equivalence to a hand-recomputed formula +
// intensity_factor linearity) and the s<=0 early-return semantic tightening
// (participating_p99 is now published on the false-return path).
// -----------------------------------------------------------------------------
TEST(ComponentConsumer, ParticipatingExposureScaleGuards) {
  constexpr int kRes = 3;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01 }, { 0.5f }));
  rc.PrepareSnapshot();

  // p99<=0 branch: returns 0 regardless of internal state.
  EXPECT_FLOAT_EQ(rc.ParticipatingExposureScale(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(rc.ParticipatingExposureScale(-1.0f), 0.0f);

  // snapshot_intensity<=0 branch: a fresh consumer without any Consume/Prepare
  // still has snapshot_intensity_==0 → guard fires even with a positive p99.
  RenderConsumer rc_empty(cfg, table);
  EXPECT_FLOAT_EQ(rc_empty.ParticipatingExposureScale(0.5f), 0.0f);
}

TEST(ComponentConsumer, ParticipatingExposureScaleFormulaCrossCheck) {
  // Independent recomputation of the formula (a04: mechanism-layer check, not
  // same-source self-check). Any drift in render.cpp's target_white / sRGB
  // reverse transform / intensity_factor multiplier flips this test red.
  constexpr int kRes = 3;
  RenderConfig cfg = MakeRenderConfig(kRes);
  cfg.intensity_factor_ = 1.0f;
  auto table = MakeSingletonClassTable(0b11, { kWhite });
  RenderConsumer rc(cfg, table);
  rc.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc.PrepareSnapshot();

  // snapshot_intensity_ is intentionally NOT in the expected formula — it cancels
  // against the downstream mono shader's intensity_scale=intensity_factor/snapshot_intensity
  // step. See derivation block above ParticipatingExposureScale in render.cpp.
  constexpr float kTargetWhite = 135.0f;
  constexpr float kTargetSrgb = kTargetWhite / 255.0f;
  const float target_linear =
      kTargetSrgb <= 0.04045f ? kTargetSrgb / 12.92f : std::pow((kTargetSrgb + 0.055f) / 1.055f, 2.4f);

  const float p99 = 0.02f;  // arbitrary positive value in the plausible range
  const float expected = cfg.intensity_factor_ * target_linear / p99;
  EXPECT_NEAR(rc.ParticipatingExposureScale(p99), expected, expected * 1e-5f);

  // snapshot_intensity independence: a second consumer whose Consume batch has
  // a very different total weight must produce the SAME s at the same p99. This
  // is the tight structural check that snapshot_intensity does not leak into
  // the composite formula (it would have if we naively mirrored ComputeEvAuto's
  // numerator, and the AC1 gate would still pass by accident when weights=1).
  RenderConsumer rc_heavy(cfg, table);
  rc_heavy.Consume(MakeBatch({ 0b01, 0b10 }, { 6.0f, 4.0f }));  // 10x the mass
  rc_heavy.PrepareSnapshot();
  EXPECT_NEAR(rc_heavy.ParticipatingExposureScale(p99), expected, expected * 1e-5f);

  // intensity_factor linearity: doubling the config's static exposure knob
  // must double the returned scalar (CLI JSON exposure retains its meaning
  // on the composite path, plan §2 default assumption).
  RenderConfig cfg2 = cfg;
  cfg2.intensity_factor_ = 2.0f;
  RenderConsumer rc2(cfg2, table);
  rc2.Consume(MakeBatch({ 0b01, 0b10 }, { 0.6f, 0.4f }));
  rc2.PrepareSnapshot();
  EXPECT_NEAR(rc2.ParticipatingExposureScale(p99), 2.0f * expected, expected * 1e-5f);
}

TEST(ComponentCompositor, EarlyReturnPublishesParticipatingP99) {
  // task-347 (Fix B) semantic tightening: on the A<=0 early-return path, the
  // participating_p99 out parameter (when non-null) is now written with the
  // actual computed p99 (previously left untouched). No in-tree consumer
  // depends on either behavior — but the tightening is worth pinning so a
  // future regression doesn't silently un-tighten it.
  //
  // doc §4.8: the guard formerly triggered
  // on `s = A * display_exposure_scale <= 0` (via scale=0). Since the split
  // A vs s, `display_exposure_scale = 0` is now a legitimate painter input
  // (post-composite black-out) that keeps the alpha structure — so the early-
  // return only fires on true "no signal" (A <= 0). Reproduce here by
  // consuming no rays: participating_p99 == 0 → A == 0.
  constexpr int kRes = 3;
  RenderConfig cfg = MakeRenderConfig(kRes);
  auto table = MakeSingletonClassTable(0b11, { kWhite });
  RenderConsumer rc(cfg, table);
  // No Consume — snapshot has zero signal, participating-P99 = 0 → A = 0.
  rc.PrepareSnapshot();

  // Sentinel-init the out pointer to a distinct value so a "left untouched"
  // behavior would fail.
  constexpr float kSentinel = -12345.0f;
  float p99_early = kSentinel;
  std::vector<float> out;
  const bool ok = CompositeColorClassesLinear(rc, table, CompositeMode::kDominant, 1.0f, out, &p99_early);
  EXPECT_FALSE(ok);
  EXPECT_NE(p99_early, kSentinel) << "p99 must be written on the A<=0 early-return path (task-347 tightening)";
  EXPECT_FLOAT_EQ(p99_early, 0.0f);
}

}  // namespace
}  // namespace lumice
