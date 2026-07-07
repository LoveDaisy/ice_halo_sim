// Tests for task-339.3 (rule-lane consumer): RenderConsumer per-color-class
// Y-lane accumulation and the class-shape structural reuse-judgment. Driven by
// ColorClassTable, with the "per-bit lane" as the degenerate case of one
// single-member `any` class per bit (produced by the local
// MakeSingletonClassTable helper).
//
// The 336.3-era `GetComponentLaneY(uint8_t bit)` bridge was removed in task-
// 339.4 (the compositor now consumes GetColorClassLaneY directly), so the
// legacy per-bit assertions that used to guard that bridge are gone.
//
// Coverage (plan §4 Step 3 + issue.md acceptance):
//   1. Σlane == mono Y (white-box, shared exposure) — single-member classes:
//      the sum of per-class lanes equals the Y channel of the main image,
//      proving lanes share the main image's exposure/CMF (no per-lane
//      normalization).
//   2. Zero-regression — a consumer built with an empty ColorClassTable
//      produces a main image bit-identical to one built with a non-empty
//      table on the same batch (lane accumulation must not perturb the main
//      render path); accessors return nullptr.
//   3. Snapshot isolation + Reset (single-member class, identical semantics
//      to 336.2 coverage).
//   4. Real backend — two-crystal single-MS scene traced through
//      CpuTraceBackend produces real per-ray component masks; the consumer
//      buckets them (three single-member classes) and Σlane == mono Y holds
//      end-to-end.
//   5. Multi-bit `any` class: a ray carrying multiple bits from the same
//      class must contribute exactly ONCE to that class's lane (issue.md
//      "any 谓词下一条光线的 Y 计一次进该类 lane"; new to 339.3 — 336.2
//      routed the same ray to multiple per-bit lanes).
//   6. Cross-layer `all` class: only rays whose masks contain all class
//      members contribute — a ray that is missing any member is skipped
//      (issue.md "cross-layer AND" / plan decision 3 white-box).
//   7. Overlap: a ray satisfying multiple independent classes contributes
//      to each class's lane (rule-lanes are the source of overlap; the
//      compositor at 339.4 is what merges them into pixels).
//   8. Empty-member class defense (plan decision 3): a class with
//      member_bits_ == 0 must never contribute — the `all` predicate's
//      vacuous-truth trap would otherwise let it swallow every ray.
//   9. Server reuse-judgment:
//      a. ColorMaskChangeForcesFullRebuild — a colored-bit set change forces
//         a full rebuild (existing 336.2 coverage, now via the new
//         ColorClassTable structural comparison).
//      b. [MANDATORY, plan decision 1] ClassShapeChangeWithSameMaskForcesRebuild
//         — two configs with identical referenced_mask_ but a different
//         class shape (one 2-bit any class vs two 1-bit classes) MUST still
//         force a rebuild.

#include <gtest/gtest.h>

#include <bitset>
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
#include "server/render.hpp"
#include "server/server.hpp"

namespace lumice {
namespace {

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

RenderConfig MakeLaneRenderConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 64;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  return cfg;
}

// Turn a legacy uint64_t "colored bits" set into a ColorClassTable containing
// one single-member `kAny` class per set bit, in ascending bit order. This is
// the per-bit → per-class mapping that produces byte-for-byte equivalent lane
// output to the 336.2 API, so tests inherited from 336.2 keep their meaning.
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

ColorClass MakeClass(ColorClassCombine combine, uint64_t member_bits) {
  ColorClass cls;
  cls.combine_ = combine;
  cls.member_bits_ = member_bits;
  return cls;
}

// Build a CPU-path SimData carrying explicit outgoing rays + per-ray component
// masks. Every ray points straight up (single in-bounds pixel).
SimData MakeBatch(const std::vector<uint64_t>& masks, const std::vector<float>& weights, float wl) {
  SimData data;
  data.curr_wl_ = wl;
  data.outgoing_d_.reserve(masks.size() * 3);
  for (size_t i = 0; i < masks.size(); ++i) {
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(-1.0f);  // sky-up
  }
  data.outgoing_w_ = weights;
  data.outgoing_component_ = masks;
  return data;
}

double SumMainY(const RawXyzResult& r) {
  double s = 0.0;
  const size_t n = static_cast<size_t>(r.img_width_) * static_cast<size_t>(r.img_height_);
  for (size_t p = 0; p < n; ++p) {
    s += r.xyz_buffer_[p * 3 + 1];  // Y channel
  }
  return s;
}

double SumLane(const float* lane, int w, int h) {
  if (lane == nullptr) {
    return 0.0;
  }
  double s = 0.0;
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  for (size_t p = 0; p < n; ++p) {
    s += lane[p];
  }
  return s;
}

constexpr float kWl = 550.0f;

// -----------------------------------------------------------------------------
// 1. Σlane == mono Y (single-member classes → shared exposure)
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, SumOfLanesEqualsMainYForSingletonClasses) {
  // Each ray carries exactly one colored bit → no overlap. Two single-member
  // classes cover bits 0 and 1; Σ_c lane_c == main Y.
  const std::vector<uint64_t> masks = { 0b01, 0b10, 0b01, 0b10 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.3f, 0.9f };
  auto data = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  ASSERT_EQ(rc.ColoredMask(), 0b11u);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double lane0 = SumLane(rc.GetColorClassLaneY(0), w, h);
  const double lane1 = SumLane(rc.GetColorClassLaneY(1), w, h);
  const double main_y = SumMainY(rc.GetRawXyzResult());

  const double exp_lane0 = SpectrumToYSingle(kWl, weights[0]) + SpectrumToYSingle(kWl, weights[2]);
  const double exp_lane1 = SpectrumToYSingle(kWl, weights[1]) + SpectrumToYSingle(kWl, weights[3]);

  ASSERT_GT(main_y, 0.0);
  EXPECT_NEAR(lane0, exp_lane0, exp_lane0 * 1e-4 + 1e-6);
  EXPECT_NEAR(lane1, exp_lane1, exp_lane1 * 1e-4 + 1e-6);
  EXPECT_NEAR(lane0 + lane1, main_y, main_y * 1e-4 + 1e-6);

  // Class index past the end → no lane at all.
  EXPECT_EQ(rc.GetColorClassLaneY(2), nullptr);
}

// -----------------------------------------------------------------------------
// 2. Multi-bit `any` class: no double count.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, MultiBitAnyClassCountsRayOnce) {
  // One class = {any, {bit0, bit1}}. Rays:
  //   idx0: bit0        → satisfies (one bit hit)
  //   idx1: bit1        → satisfies (one bit hit)
  //   idx2: bit0|bit1   → satisfies EXACTLY once, not twice
  //   idx3: no colored bits (bit2 only, but bit2 is not a class member)
  const std::vector<uint64_t> masks = { 0b001, 0b010, 0b011, 0b100 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.9f, 0.4f };
  auto data = MakeBatch(masks, weights, kWl);

  ColorClassTable t;
  t.classes_.push_back(MakeClass(ColorClassCombine::kAny, 0b011));
  t.referenced_mask_ = 0b011;

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, t);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double lane = SumLane(rc.GetColorClassLaneY(0), w, h);

  const double y0 = SpectrumToYSingle(kWl, weights[0]);
  const double y1 = SpectrumToYSingle(kWl, weights[1]);
  const double y2 = SpectrumToYSingle(kWl, weights[2]);
  // idx2 contributes exactly y2, NOT 2*y2.
  const double exp_lane = y0 + y1 + y2;
  EXPECT_NEAR(lane, exp_lane, exp_lane * 1e-4 + 1e-6);
}

// -----------------------------------------------------------------------------
// 3. Cross-layer `all`: only rays with ALL member bits contribute.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, CrossLayerAllClassRequiresAllMembers) {
  // Two "layer bits": 0b01 (layer A) and 0b10 (layer B). A class {all, 0b11}
  // fires only when a ray passed through BOTH layers.
  const std::vector<uint64_t> masks = { 0b01, 0b10, 0b11, 0b00 };
  const std::vector<float> weights = { 0.5f, 0.6f, 0.7f, 0.9f };
  auto data = MakeBatch(masks, weights, kWl);

  ColorClassTable t;
  t.classes_.push_back(MakeClass(ColorClassCombine::kAll, 0b11));
  t.referenced_mask_ = 0b11;

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, t);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double lane = SumLane(rc.GetColorClassLaneY(0), w, h);

  const double y2 = SpectrumToYSingle(kWl, weights[2]);
  EXPECT_NEAR(lane, y2, y2 * 1e-4 + 1e-6);
}

// -----------------------------------------------------------------------------
// 4. Overlap: a ray satisfying multiple classes accumulates into each lane.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, OverlappingRayAccumulatesIntoEveryMatchingLane) {
  // Two single-member classes on disjoint bits, plus one ray that carries both
  // bits — that ray must land in BOTH lanes.
  const std::vector<uint64_t> masks = { 0b01, 0b10, 0b11 };
  const std::vector<float> weights = { 0.4f, 0.5f, 0.6f };
  auto data = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b11));
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double lane0 = SumLane(rc.GetColorClassLaneY(0), w, h);
  const double lane1 = SumLane(rc.GetColorClassLaneY(1), w, h);

  const double y0 = SpectrumToYSingle(kWl, weights[0]);
  const double y1 = SpectrumToYSingle(kWl, weights[1]);
  const double y2 = SpectrumToYSingle(kWl, weights[2]);
  EXPECT_NEAR(lane0, y0 + y2, (y0 + y2) * 1e-4 + 1e-6);
  EXPECT_NEAR(lane1, y1 + y2, (y1 + y2) * 1e-4 + 1e-6);
  // Overlap: idx2's Y counted in both lanes, so Σ lanes exceeds the mono Y.
  EXPECT_GT(lane0 + lane1, y0 + y1 + y2 + 1e-6);
}

// -----------------------------------------------------------------------------
// 5. Empty-member class must NOT consume energy (`all` vacuous-truth guard).
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, EmptyMemberBitsClassNeverContributes) {
  // A class with member_bits_ == 0. Without the guard, `all` would evaluate
  // (mask & 0) == 0 == true for every ray and swallow the entire batch's Y
  // into this lane. With the guard, the lane must stay at 0.
  ColorClassTable t;
  t.classes_.push_back(MakeClass(ColorClassCombine::kAll, 0));  // empty-member
  t.classes_.push_back(MakeClass(ColorClassCombine::kAny, 0b01));
  t.referenced_mask_ = 0b01;

  const std::vector<uint64_t> masks = { 0b01, 0b01, 0b10, 0b00 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.9f, 0.4f };
  auto data = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, t);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double empty_lane = SumLane(rc.GetColorClassLaneY(0), w, h);
  const double any_lane = SumLane(rc.GetColorClassLaneY(1), w, h);

  EXPECT_NEAR(empty_lane, 0.0, 1e-9) << "empty-member class must never absorb energy";
  const double exp_any = SpectrumToYSingle(kWl, weights[0]) + SpectrumToYSingle(kWl, weights[1]);
  EXPECT_NEAR(any_lane, exp_any, exp_any * 1e-4 + 1e-6);
}

// -----------------------------------------------------------------------------
// 7. Zero-regression: empty class table vs single-member table produce the
//    same main image on the same batch.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, ColoredMaskDoesNotPerturbMainImage) {
  const std::vector<uint64_t> masks = { 0b01, 0b10, 0b11, 0b00 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.9f, 0.4f };
  auto data_a = MakeBatch(masks, weights, kWl);
  auto data_b = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc_plain(cfg, ColorClassTable{});              // pre-336 path
  RenderConsumer rc_color(cfg, MakeSingletonClassTable(0b11));  // lane path active
  rc_plain.Consume(data_a);
  rc_color.Consume(data_b);
  rc_plain.PrepareSnapshot();
  rc_color.PrepareSnapshot();

  auto ra = rc_plain.GetRawXyzResult();
  auto rb = rc_color.GetRawXyzResult();
  ASSERT_EQ(ra.img_width_, rb.img_width_);
  ASSERT_EQ(ra.img_height_, rb.img_height_);
  const size_t n = static_cast<size_t>(ra.img_width_) * static_cast<size_t>(ra.img_height_) * 3;
  for (size_t i = 0; i < n; ++i) {
    ASSERT_FLOAT_EQ(ra.xyz_buffer_[i], rb.xyz_buffer_[i]) << "main image diverged at i=" << i;
  }
  EXPECT_FLOAT_EQ(ra.snapshot_intensity_, rb.snapshot_intensity_);

  // The plain consumer exposes no lanes at all.
  EXPECT_EQ(rc_plain.ColoredMask(), 0u);
  EXPECT_EQ(rc_plain.GetColorClassLaneY(0), nullptr);
}

// -----------------------------------------------------------------------------
// 8. Snapshot isolation + Reset.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, LaneSnapshotIsolationAndReset) {
  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, MakeSingletonClassTable(0b1));
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];

  auto batch_a = MakeBatch({ 0b1 }, { 0.5f }, kWl);
  rc.Consume(batch_a);
  rc.PrepareSnapshot();
  const double after_a = SumLane(rc.GetColorClassLaneY(0), w, h);
  EXPECT_GT(after_a, 0.0);

  // Accumulate more WITHOUT a new snapshot: the snapshot must stay frozen.
  auto batch_b = MakeBatch({ 0b1 }, { 0.7f }, kWl);
  rc.Consume(batch_b);
  EXPECT_NEAR(SumLane(rc.GetColorClassLaneY(0), w, h), after_a, after_a * 1e-4 + 1e-6)
      << "GetColorClassLaneY must read the frozen snapshot, not live accumulation";

  // Now snapshot: both batches visible.
  rc.PrepareSnapshot();
  const double after_b = SumLane(rc.GetColorClassLaneY(0), w, h);
  EXPECT_GT(after_b, after_a);

  // Reset zeroes the live lanes; next snapshot reflects an empty lane.
  rc.Reset();
  rc.PrepareSnapshot();
  EXPECT_NEAR(SumLane(rc.GetColorClassLaneY(0), w, h), 0.0, 1e-9);
}

// -----------------------------------------------------------------------------
// 9. Real backend: two-crystal single-MS scene → real per-ray masks → Σlane==Y.
// -----------------------------------------------------------------------------
namespace {

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

}  // namespace

TEST(RenderConsumerComponentLanes, RealBackendMasksBucketedShareExposure) {
  constexpr size_t kMaxHits = 8;
  auto scene = MakeTwoCrystalColoredScene(kMaxHits);

  // Design 2: color bits come from a raypath_color config on the SessionSpec.
  // The three predicates below match the three arcs originally produced by the
  // Fork-C summand walk (crystal 0 whole-crystal + crystal 1's two EE clauses).
  auto rpc = std::make_shared<RaypathColorConfig>();
  {
    ColorClassConfig c;
    c.color_[0] = 1.0f;
    RaypathColorRef r;
    r.layer_ = 0;
    r.crystal_ = 0;
    r.predicate_ = SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 1, std::nullopt } };
    c.match_.push_back(r);
    rpc->classes_.push_back(std::move(c));
  }
  {
    ColorClassConfig c;
    c.color_[1] = 1.0f;
    RaypathColorRef r;
    r.layer_ = 0;
    r.crystal_ = 1;
    r.predicate_ = SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 2, 2 } };
    c.match_.push_back(r);
    rpc->classes_.push_back(std::move(c));
  }
  {
    ColorClassConfig c;
    c.color_[2] = 1.0f;
    RaypathColorRef r;
    r.layer_ = 0;
    r.crystal_ = 1;
    r.predicate_ = SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 3, std::nullopt } };
    c.match_.push_back(r);
    rpc->classes_.push_back(std::move(c));
  }

  RenderConfig render = MakeLaneRenderConfig();
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

  const uint64_t kColored = 0b111;
  size_t hits[3] = { 0, 0, 0 };
  for (const auto& rec : records) {
    const uint64_t m = rec.component_mask & kColored;
    ASSERT_LE(std::bitset<64>(m).count(), 1u)
        << "expected each emitted ray to match at most one colored summand, mask=" << rec.component_mask;
    for (int b = 0; b < 3; ++b) {
      if ((m >> b) & 1ULL) {
        ++hits[b];
      }
    }
  }
  for (int b = 0; b < 3; ++b) {
    ASSERT_GT(hits[b], 0u) << "bit " << b << " got zero hits — retune scene (plan risk 5)";
  }

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

  RenderConsumer rc(render, MakeSingletonClassTable(kColored));
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = render.resolution_[0];
  const int h = render.resolution_[1];
  const double lane0 = SumLane(rc.GetColorClassLaneY(0), w, h);
  const double lane1 = SumLane(rc.GetColorClassLaneY(1), w, h);
  const double lane2 = SumLane(rc.GetColorClassLaneY(2), w, h);
  const double main_y = SumMainY(rc.GetRawXyzResult());

  EXPECT_GT(lane0, 0.0);
  EXPECT_GT(lane1, 0.0);
  EXPECT_GT(lane2, 0.0);
  ASSERT_GT(main_y, 0.0);
  EXPECT_NEAR(lane0 + lane1 + lane2, main_y, main_y * 1e-3 + 1e-6);
}

// -----------------------------------------------------------------------------
// 10. Server reuse-judgment (color mask + class shape).
// -----------------------------------------------------------------------------
namespace {

// Minimal committable config. `color` selects which raypath_color block to
// emit; the scene has TWO scattering slots on DISTINCT crystal_ids (1 and 2)
// so each Design-2 `{layer, crystal}` ref resolves unambiguously per §3.2
// decision 2(b). The two crystals share geometry — only the id differs.
enum class ColorForm { kNone, kOneBit, kTwoSingleBitClasses, kOneTwoBitClass };

nlohmann::json MakeReuseConfig(ColorForm color) {
  nlohmann::json root;

  nlohmann::json cr1;
  cr1["id"] = 1;
  cr1["type"] = "prism";
  cr1["shape"]["height"] = 1.5f;
  cr1["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
  cr1["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr1["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  nlohmann::json cr2 = cr1;  // identical geometry, distinct id
  cr2["id"] = 2;
  root["crystal"] = nlohmann::json::array({ cr1, cr2 });

  // Two simple raypath filters (distinct raypaths → distinct physical
  // populations; Design-2 color refs are keyed by {layer, crystal} only).
  nlohmann::json flt1;
  flt1["id"] = 1;
  flt1["type"] = "raypath";
  flt1["raypath"] = { 3 };
  flt1["action"] = "filter_in";
  nlohmann::json flt2;
  flt2["id"] = 2;
  flt2["type"] = "raypath";
  flt2["raypath"] = { 5 };
  flt2["action"] = "filter_in";
  root["filter"] = nlohmann::json::array({ flt1, flt2 });

  nlohmann::json scene;
  scene["light_source"]["type"] = "sun";
  scene["light_source"]["altitude"] = 20.0f;
  scene["light_source"]["azimuth"] = 0.0f;
  scene["light_source"]["diameter"] = 0.5f;
  scene["light_source"]["spectrum"] = "D65";
  scene["ray_num"] = 1000ul;
  scene["max_hits"] = 8;
  nlohmann::json ms;
  ms["prob"] = 0.0f;
  // Two scattering slots on distinct crystals:
  //   ci=0 → (crystal=1, filter=1) → color bit for {layer:0, crystal:1};
  //   ci=1 → (crystal=2, filter=2) → color bit for {layer:0, crystal:2}.
  ms["entries"] = nlohmann::json::array({
      { { "crystal", 1 }, { "filter", 1 }, { "proportion", 0.5f } },
      { { "crystal", 2 }, { "filter", 2 }, { "proportion", 0.5f } },
  });
  scene["scattering"] = nlohmann::json::array({ ms });
  root["scene"] = scene;

  nlohmann::json rn;
  rn["id"] = 1;
  rn["lens"]["type"] = "fisheye_equal_area";
  rn["lens"]["fov"] = 180.0f;
  rn["resolution"] = { 64, 64 };
  rn["view"]["elevation"] = 90.0f;
  rn["view"]["azimuth"] = 0.0f;
  rn["view"]["roll"] = 0.0f;
  rn["visible"] = "upper";
  rn["background"] = { 0.0f, 0.0f, 0.0f };
  rn["opacity"] = 1.0f;
  rn["intensity_factor"] = 1.0f;
  root["render"] = nlohmann::json::array({ rn });

  // Design-2 whole-crystal ref (match-all predicate) — the JSON wire form is
  // just {layer, crystal} (no `type` field means NoneFilterParam).
  auto ref = [](int crystal_id) { return nlohmann::json{ { "layer", 0 }, { "crystal", crystal_id } }; };

  switch (color) {
    case ColorForm::kNone:
      break;
    case ColorForm::kOneBit:
      root["raypath_color"] = nlohmann::json::array({
          { { "color", { 1.0f, 0.0f, 0.0f } }, { "match", { ref(1) } } },
      });
      break;
    case ColorForm::kTwoSingleBitClasses:
      // Two independent single-member classes: referenced_mask_ = 0b11,
      // classes.size() = 2.
      root["raypath_color"] = nlohmann::json::array({
          { { "color", { 1.0f, 0.0f, 0.0f } }, { "match", { ref(1) } } },
          { { "color", { 0.0f, 1.0f, 0.0f } }, { "match", { ref(2) } } },
      });
      break;
    case ColorForm::kOneTwoBitClass:
      // Single class whose two refs union to bits {0, 1}: same referenced_mask_
      // = 0b11 as the split shape, but classes.size() = 1 and member_bits_ =
      // 0b11 — a NeedsRebuild that only compares uint64 masks would falsely
      // reuse a split-shape consumer here.
      root["raypath_color"] = nlohmann::json::array({
          { { "color", { 1.0f, 1.0f, 0.0f } }, { "match", { ref(1), ref(2) } } },
      });
      break;
  }
  return root;
}

}  // namespace

TEST(ServerCommitConfigColorReuse, ColorMaskChangeForcesFullRebuild) {
  Server server(1);
  const auto base = MakeReuseConfig(ColorForm::kNone);
  const auto colored = MakeReuseConfig(ColorForm::kOneBit);

  bool reused = true;

  // First commit ever: consumers are empty → always a rebuild.
  ASSERT_TRUE(server.CommitConfig(base, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "first commit must build consumers from scratch";

  // Re-commit the identical (uncolored) config: baseline reuse works.
  ASSERT_TRUE(server.CommitConfig(base, &reused).IsSuccess());
  EXPECT_TRUE(reused) << "identical config with unchanged class table should reuse consumers";

  // Add raypath_color: class table goes empty → 1 class → rebuild.
  ASSERT_TRUE(server.CommitConfig(colored, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "a class-table change must force a full consumer rebuild";

  // Re-commit the identical colored config: reuse resumes now the shape is stable.
  ASSERT_TRUE(server.CommitConfig(colored, &reused).IsSuccess());
  EXPECT_TRUE(reused) << "stable class table should reuse consumers";

  // Remove the color again: class table goes back to empty → rebuild.
  ASSERT_TRUE(server.CommitConfig(base, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "clearing raypath_color must also force a rebuild";
}

// MANDATORY plan decision 1: two configs with identical referenced_mask_ but
// different class shape must STILL force a full consumer rebuild — a plain
// uint64 mask compare would falsely reuse a stale-shape consumer.
TEST(ServerCommitConfigColorReuse, ClassShapeChangeWithSameMaskForcesRebuild) {
  Server server(1);
  const auto split = MakeReuseConfig(ColorForm::kTwoSingleBitClasses);  // 2 classes, bits {0}/{1}
  const auto merged = MakeReuseConfig(ColorForm::kOneTwoBitClass);      // 1 class,  bits {0,1}

  bool reused = true;
  ASSERT_TRUE(server.CommitConfig(split, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "first commit must build consumers from scratch";

  ASSERT_TRUE(server.CommitConfig(split, &reused).IsSuccess());
  EXPECT_TRUE(reused) << "identical split-class config should reuse consumers";

  // referenced_mask_ is 0b11 in BOTH configs; only the class shape differs.
  // The structural NeedsRebuild must catch this and force a rebuild.
  ASSERT_TRUE(server.CommitConfig(merged, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "same referenced_mask_ but different class shape MUST force a rebuild";

  ASSERT_TRUE(server.CommitConfig(merged, &reused).IsSuccess());
  EXPECT_TRUE(reused) << "identical merged-class config should reuse consumers";

  // Back to the split shape: rebuild again.
  ASSERT_TRUE(server.CommitConfig(split, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "class shape change (merged → split) must force a rebuild";
}

}  // namespace
}  // namespace lumice
