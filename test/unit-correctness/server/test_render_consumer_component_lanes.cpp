// Tests for task-336.2 (component-lane consumer): RenderConsumer per-component
// Y-lane accumulation and the server.cpp reuse-judgment wiring.
//
// Coverage (see plan §4 Step 6 + issue.md acceptance):
//   1. Σlane == mono Y (white-box, shared exposure) — single-bit rays: the sum
//      of the per-bit lanes equals the Y channel of the main image, proving the
//      lanes share the main image's exposure/CMF (no per-lane normalization).
//   2. Per-bit routing — a multi-bit ray contributes to every participating
//      lane it carries; a non-participating bit gets no lane at all.
//   3. Uncolored ray ignored — a ray with no colored bit lands in the main image
//      but in no lane.
//   4. Zero-regression — a consumer built with colored_mask=0 produces a main
//      image bit-identical to one built with a non-zero mask on the same batch
//      (lane accumulation must not perturb the main render path).
//   5. Snapshot isolation — GetComponentLaneY reads the two-phase snapshot, so
//      accumulation after PrepareSnapshot is not visible until the next snapshot;
//      Reset zeroes the live lanes.
//   6. Real backend — a two-crystal single-MS scene traced through
//      CpuTraceBackend produces real per-ray component masks; the consumer
//      buckets them and the Σlane == mono Y invariant holds end-to-end.
//   7. Server reuse-judgment [plan-review Minor #2, MANDATORY] — a colored-bit
//      set change across two CommitConfig calls forces a full consumer rebuild.

#include <gtest/gtest.h>

#include <bitset>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "config/component_table.hpp"
#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
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

// Small fisheye render looking straight up: rays with dir=(0,0,-1) (sky.z=+1)
// land near the image centre, safely in-bounds (mirrors the convention in
// test_cpu_trace_backend.cpp Test D).
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

// Build a CPU-path SimData carrying explicit outgoing rays + per-ray component
// masks. Every ray points straight up (single in-bounds pixel unless varied).
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
// 1. Σlane == mono Y (single-bit rays → shared exposure / energy conservation)
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, SumOfLanesEqualsMainYForSingleBitRays) {
  // Each ray carries exactly one colored bit → no double counting; every ray is
  // colored → nothing escapes into the uncolored bucket. Σ_b lane_b == main Y.
  const std::vector<uint64_t> masks = { 0b01, 0b10, 0b01, 0b10 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.3f, 0.9f };
  auto data = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, 0b11);
  ASSERT_EQ(rc.ColoredMask(), 0b11u);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double lane0 = SumLane(rc.GetComponentLaneY(0), w, h);
  const double lane1 = SumLane(rc.GetComponentLaneY(1), w, h);
  const double main_y = SumMainY(rc.GetRawXyzResult());

  // Hand-computed per-lane expectations (bit0 rays: idx 0,2; bit1 rays: idx 1,3).
  const double exp_lane0 = SpectrumToYSingle(kWl, weights[0]) + SpectrumToYSingle(kWl, weights[2]);
  const double exp_lane1 = SpectrumToYSingle(kWl, weights[1]) + SpectrumToYSingle(kWl, weights[3]);

  ASSERT_GT(main_y, 0.0);
  EXPECT_NEAR(lane0, exp_lane0, exp_lane0 * 1e-4 + 1e-6);
  EXPECT_NEAR(lane1, exp_lane1, exp_lane1 * 1e-4 + 1e-6);
  // Headline invariant: Σ lanes == main Y (shared exposure, no lost energy).
  EXPECT_NEAR(lane0 + lane1, main_y, main_y * 1e-4 + 1e-6);

  // Non-participating bit → no lane allocated.
  EXPECT_EQ(rc.GetComponentLaneY(2), nullptr);
}

// -----------------------------------------------------------------------------
// 2 + 3. Multi-bit ray hits both lanes; uncolored rays land only in main Y.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, MultiBitRoutingAndUncoloredIgnored) {
  //   idx0: bit0        idx1: bit1        idx2: bit0|bit1 (multi)
  //   idx3: mask 0 (uncolored)            idx4: bit2 only (not in colored_mask)
  const std::vector<uint64_t> masks = { 0b001, 0b010, 0b011, 0b000, 0b100 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.9f, 0.3f, 0.4f };
  auto data = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, 0b011);  // only bits 0,1 colored
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const double lane0 = SumLane(rc.GetComponentLaneY(0), w, h);
  const double lane1 = SumLane(rc.GetComponentLaneY(1), w, h);
  const double main_y = SumMainY(rc.GetRawXyzResult());

  const double y0 = SpectrumToYSingle(kWl, weights[0]);
  const double y1 = SpectrumToYSingle(kWl, weights[1]);
  const double y2 = SpectrumToYSingle(kWl, weights[2]);
  const double y3 = SpectrumToYSingle(kWl, weights[3]);
  const double y4 = SpectrumToYSingle(kWl, weights[4]);

  // Multi-bit ray (idx2) contributes to BOTH participating lanes.
  EXPECT_NEAR(lane0, y0 + y2, (y0 + y2) * 1e-4 + 1e-6);
  EXPECT_NEAR(lane1, y1 + y2, (y1 + y2) * 1e-4 + 1e-6);

  // Main Y counts every ray exactly once, including the two uncolored ones
  // (idx3 = mask 0, idx4 = bit2 masked out by colored_mask).
  const double exp_main = y0 + y1 + y2 + y3 + y4;
  EXPECT_NEAR(main_y, exp_main, exp_main * 1e-4 + 1e-6);

  // The uncolored energy (y3 + y4) never reaches a lane: main Y strictly exceeds
  // the union of colored contributions (y0+y1+y2).
  EXPECT_GT(main_y, y0 + y1 + y2 + 1e-6);

  // bit2 is carried by a ray but is not a participating bit → no lane.
  EXPECT_EQ(rc.GetComponentLaneY(2), nullptr);
}

// -----------------------------------------------------------------------------
// 4. Zero-regression: colored_mask=0 vs non-zero produce identical main images.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, ColoredMaskDoesNotPerturbMainImage) {
  const std::vector<uint64_t> masks = { 0b01, 0b10, 0b11, 0b00 };
  const std::vector<float> weights = { 0.5f, 0.7f, 0.9f, 0.4f };
  auto data_a = MakeBatch(masks, weights, kWl);
  auto data_b = MakeBatch(masks, weights, kWl);

  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc_plain(cfg, 0);     // pre-336 path
  RenderConsumer rc_color(cfg, 0b11);  // lane path active
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
  EXPECT_EQ(rc_plain.GetComponentLaneY(0), nullptr);
}

// -----------------------------------------------------------------------------
// 5. Snapshot isolation + Reset.
// -----------------------------------------------------------------------------
TEST(RenderConsumerComponentLanes, LaneSnapshotIsolationAndReset) {
  RenderConfig cfg = MakeLaneRenderConfig();
  RenderConsumer rc(cfg, 0b1);
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];

  auto batch_a = MakeBatch({ 0b1 }, { 0.5f }, kWl);
  rc.Consume(batch_a);
  rc.PrepareSnapshot();
  const double after_a = SumLane(rc.GetComponentLaneY(0), w, h);
  EXPECT_GT(after_a, 0.0);

  // Accumulate more WITHOUT a new snapshot: the snapshot must stay frozen.
  auto batch_b = MakeBatch({ 0b1 }, { 0.7f }, kWl);
  rc.Consume(batch_b);
  EXPECT_NEAR(SumLane(rc.GetComponentLaneY(0), w, h), after_a, after_a * 1e-4 + 1e-6)
      << "GetComponentLaneY must read the frozen snapshot, not live accumulation";

  // Now snapshot: both batches visible.
  rc.PrepareSnapshot();
  const double after_b = SumLane(rc.GetComponentLaneY(0), w, h);
  EXPECT_GT(after_b, after_a);

  // Reset zeroes the live lanes; next snapshot reflects an empty lane.
  rc.Reset();
  rc.PrepareSnapshot();
  EXPECT_NEAR(SumLane(rc.GetComponentLaneY(0), w, h), 0.0, 1e-9);
}

// -----------------------------------------------------------------------------
// 6. Real backend: two-crystal single-MS scene → real per-ray masks → Σlane==Y.
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

// Single MS layer, two crystal slots. This is the three-bit "three-arcs" TOPOLOGY
// of the plan, but realised with entry/exit length-band filters instead of the
// literal [3] / [3,5] raypath sequences: a length-1 raypath is physically near
// zero-hit, whereas length bands guarantee non-zero, disjoint populations (plan
// risk 5 explicitly permits this tuning). Bits:
//   slot0  Simple(len>=1)                    → bit0 (all slot-0 rays)
//   slot1  Complex[ len==2 , len>=3 ]        → bit1 (len==2), bit2 (len>=3)
// Every emitted ray carries exactly one bit → Σ_b lane_b == main Y.
SceneConfig MakeTwoCrystalColoredScene(size_t max_hits) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { kWl, 1.0f } };

  MsInfo ms;
  ms.prob_ = 0.0f;  // single (final) layer: all surviving rays emit.

  // slot0: everything (length >= 1) → one summand → bit0.
  {
    ScatteringSetting s;
    s.crystal_.id_ = 0;
    s.crystal_.param_ = MakeUnitPrism();
    s.filter_ = MakeFilterIn(SimpleFilterParam{ EntryExitFilterParam{ std::nullopt, std::nullopt, 1, std::nullopt } });
    s.crystal_proportion_ = 0.5f;
    ms.setting_.push_back(std::move(s));
  }
  // slot1: complex OR of two disjoint length bands → summands bit1, bit2.
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

  // Sanity: the static table assigns exactly bits 0,1,2.
  auto table = BuildComponentTable(scene);
  ASSERT_EQ(table.entries_.size(), 3u);

  RenderConfig render = MakeLaneRenderConfig();
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
  host.crystal = nullptr;  // backend builds each slot's crystal via MakeCrystal.

  auto handle = backend.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(handle, nullptr);

  std::vector<ExitRayRecord> records;
  backend.ReadbackExitRays(records);
  backend.EndSession();
  ASSERT_FALSE(records.empty());

  // --- Risk-5 precheck: confirm every target bit actually got hits, and that no
  //     emitted ray carries more than one colored bit (single-bit invariant). ---
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

  // --- Feed the real exit rays into the consumer and check the invariant. ---
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

  RenderConsumer rc(render, kColored);
  rc.Consume(data);
  rc.PrepareSnapshot();

  const int w = render.resolution_[0];
  const int h = render.resolution_[1];
  const double lane0 = SumLane(rc.GetComponentLaneY(0), w, h);
  const double lane1 = SumLane(rc.GetComponentLaneY(1), w, h);
  const double lane2 = SumLane(rc.GetComponentLaneY(2), w, h);
  const double main_y = SumMainY(rc.GetRawXyzResult());

  EXPECT_GT(lane0, 0.0);
  EXPECT_GT(lane1, 0.0);
  EXPECT_GT(lane2, 0.0);
  ASSERT_GT(main_y, 0.0);
  // Every emitted ray carries exactly one colored bit → Σ lanes == main Y
  // (shared exposure, end-to-end through the real backend).
  EXPECT_NEAR(lane0 + lane1 + lane2, main_y, main_y * 1e-3 + 1e-6);
}

// -----------------------------------------------------------------------------
// 7. Server reuse-judgment [plan-review Minor #2 — MANDATORY].
// -----------------------------------------------------------------------------
namespace {

// Minimal committable config with one prism crystal, one raypath filter, one
// scattering layer referencing them, and one renderer. `with_color` toggles the
// raypath_color entry so the participating-bit set changes between commits.
nlohmann::json MakeReuseConfig(bool with_color) {
  nlohmann::json root;

  nlohmann::json cr;
  cr["id"] = 1;
  cr["type"] = "prism";
  cr["shape"]["height"] = 1.5f;
  cr["axis"]["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 10.0f } };
  cr["axis"]["azimuth"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  cr["axis"]["roll"] = { { "type", "uniform" }, { "mean", 0.0f }, { "std", 180.0f } };
  root["crystal"] = nlohmann::json::array({ cr });

  nlohmann::json flt;
  flt["id"] = 1;
  flt["type"] = "raypath";
  flt["raypath"] = { 3, 5 };
  flt["action"] = "filter_in";
  root["filter"] = nlohmann::json::array({ flt });

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
  ms["entries"] = nlohmann::json::array({ { { "crystal", 1 }, { "filter", 1 }, { "proportion", 1.0f } } });
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

  if (with_color) {
    root["raypath_color"] = nlohmann::json::array({
        { { "color", { 1.0f, 0.0f, 0.0f } }, { "match", { { { "layer", 0 }, { "crystal", 1 }, { "filter", 1 } } } } },
    });
  }
  return root;
}

}  // namespace

TEST(ServerCommitConfigColorReuse, ColorMaskChangeForcesFullRebuild) {
  Server server(1);
  const auto base = MakeReuseConfig(/*with_color=*/false);
  const auto colored = MakeReuseConfig(/*with_color=*/true);

  bool reused = true;

  // First commit ever: consumers are empty → always a rebuild.
  ASSERT_TRUE(server.CommitConfig(base, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "first commit must build consumers from scratch";

  // Re-commit the identical (uncolored) config: baseline reuse works.
  ASSERT_TRUE(server.CommitConfig(base, &reused).IsSuccess());
  EXPECT_TRUE(reused) << "identical config with unchanged color mask should reuse consumers";

  // Add raypath_color (same renderer layout): the participating-bit set changes
  // from 0 to non-zero → the consumer set MUST be fully rebuilt, not reused.
  ASSERT_TRUE(server.CommitConfig(colored, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "a colored-bit set change must force a full consumer rebuild (plan §2 in-scope)";

  // Re-commit the identical colored config: reuse resumes now the mask is stable.
  ASSERT_TRUE(server.CommitConfig(colored, &reused).IsSuccess());
  EXPECT_TRUE(reused) << "stable colored mask should reuse consumers";

  // Remove the color again: mask returns to 0 → rebuild once more.
  ASSERT_TRUE(server.CommitConfig(base, &reused).IsSuccess());
  EXPECT_FALSE(reused) << "clearing the colored mask must also force a rebuild";
}

}  // namespace
}  // namespace lumice
