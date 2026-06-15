// scrum-258.2 rich-metadata regression: assert ReadbackExitRays returns
// ExitRayRecord with crystal_id / path / ms_layer_idx populated. Covers
// both backends (CPU + Metal) at the seam level, and the simulator-level
// SimData.exit_records_ propagation (Step 5 / review Minor #3).
//
// Test motivation (vs. silent failure modes guarded against):
//   - Step 3/4: a CPU/Metal backend that produces correct dir/weight but
//     forgets to fill crystal_id/path/ms_layer_idx (review Minor #2). Fast
//     e2e + image parity tests would NOT detect this — the simulator's
//     dir/weight extraction path is unaffected.
//   - Step 5: simulator forgets the `sim_data.exit_records_ = std::move(...)`
//     line or moves before extracting dir/weight (review Minor #3). Fast e2e
//     would still produce the correct image, but 258.3's filter consumer
//     would see an empty exit_records_ vector.
//   - KernelParams host/MSL field reorder (review Minor #1): the kernel
//     would write crystal_id into the wrong slot. By ASSERTing the exact
//     crystal_id == 0 value (not a `< kMaxCrystalNum` range), any reorder
//     that puts max_hits/face_seq_cap/ms_layer_idx into the crystal_id slot
//     would surface here (those values are 8 / ≤15 / 0 respectively — none
//     of which equal 0 for our single-crystal scenario where ci=0). Note:
//     ms_layer_idx==0 colides with crystal_id==0 for single-MS; that
//     ambiguity is acceptable in 258.2 (single-MS test) and 258.3 will
//     introduce multi-layer scenarios that disambiguate.

#include <gtest/gtest.h>

#include <vector>

#include "config/crystal_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "core/cpu_trace_backend.hpp"
#include "core/exit_seam.hpp"
#include "core/trace_backend.hpp"

#if defined(__APPLE__)
#include "core/metal_trace_backend.hpp"
#include "metal_test_helpers.hpp"
#endif

namespace lumice {
namespace {

#if defined(__APPLE__)
using metal_test::MakeMetalScene;
using metal_test::MakeMetalSceneWithProb;
using metal_test::MakeRectangularRender;
using metal_test::ShouldSkipMetalTests;
#endif

constexpr size_t kRayCount = 4096;
constexpr size_t kMaxHits = 8;

// ============================== CPU backend ==================================
//
// Single-MS, single-crystal: assert every record carries
//   - crystal_id == 0 (single population; precise value, not a range)
//   - path.size_ ∈ [1, max_hits] (at least one face hit; bounded by max_hits
//     and by ExitFaceSeq::kCap)
//   - ms_layer_idx == 0 (single MS layer)

#if defined(__APPLE__)
TEST(ExitRecordsTest, CpuBackendSingleMsFillsMetadata) {
  auto scene = MakeMetalScene(kMaxHits, /*ms_layers=*/1);
  auto render = MakeRectangularRender();
#else
TEST(ExitRecordsTest, CpuBackendSingleMsFillsMetadata) {
  // Non-Apple platforms: scaffold a minimal scene + render here so this test
  // still runs on Linux CI. Mirror MakeMetalScene / MakeRectangularRender
  // closely enough to drive ReadbackExitRays.
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = kMaxHits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };
  MsInfo ms;
  ms.prob_ = 0.0f;
  ScatteringSetting s;
  s.crystal_.id_ = 0;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.param_ = prism;
  s.filter_ = FilterConfig{};
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));

  RenderConfig render;
  render.id_ = 0;
  render.lens_.type_ = LensParam::kRectangular;
  render.lens_.fov_ = 360.0f;
  render.resolution_[0] = 64;
  render.resolution_[1] = 32;
  render.view_.az_ = 0.0f;
  render.view_.el_ = 90.0f;
  render.view_.ro_ = 0.0f;
  render.visible_ = RenderConfig::kUpper;
#endif

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  CpuTraceBackend cpu;
  cpu.BeginSession(spec);
  auto h = cpu.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h, nullptr);

  std::vector<ExitRayRecord> records;
  size_t count = cpu.ReadbackExitRays(records);
  cpu.EndSession();

  ASSERT_GT(count, 0u) << "CPU backend produced no exit rays";
  ASSERT_EQ(records.size(), count);

  for (const auto& rec : records) {
    EXPECT_EQ(rec.crystal_id, 0u);
    EXPECT_EQ(rec.ms_layer_idx, 0u);
    EXPECT_GE(rec.path.size_, 1u);
    EXPECT_LE(rec.path.size_, kMaxHits);
    EXPECT_LE(rec.path.size_, ExitFaceSeq::kCap);
  }
}

// ============================== Metal backend ================================
//
// Same scene + ray count as CPU; asserts metadata fields populated by the
// kernel via buffer(21/22/23). crystal_id is asserted == 0 (single crystal)
// to catch KernelParams host/MSL field reorder (plan §7 Risk 2, review
// Minor #1: tight value bound > range bound for detection).

#if defined(__APPLE__)
TEST(ExitRecordsTest, MetalBackendSingleMsFillsMetadata) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  auto scene = MakeMetalScene(kMaxHits, /*ms_layers=*/1);
  auto render = MakeRectangularRender();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  MetalTraceBackend metal;
  metal.BeginSession(spec);
  auto h = metal.TraceLayer(RootRaySource::FromHost(host));
  ASSERT_NE(h, nullptr);

  std::vector<ExitRayRecord> records;
  size_t count = metal.ReadbackExitRays(records);
  metal.EndSession();

  ASSERT_GT(count, 0u) << "Metal backend produced no exit rays";
  ASSERT_EQ(records.size(), count);

  for (const auto& rec : records) {
    EXPECT_EQ(rec.crystal_id, 0u);
    EXPECT_EQ(rec.ms_layer_idx, 0u);
    EXPECT_GE(rec.path.size_, 1u);
    EXPECT_LE(rec.path.size_, kMaxHits);
    EXPECT_LE(rec.path.size_, ExitFaceSeq::kCap);
    for (uint8_t k = 0; k < rec.path.size_; k++) {
      // After GetFn remap (258.8): path stores face-numbers (1-8 for prism),
      // not poly-indices (0-7). FillHexFnMap assigns basal=1,2; prism=3-8.
      EXPECT_GE(rec.path.data_[k], 1u);
      EXPECT_LE(rec.path.data_[k], 8u);
    }
  }
}
#endif  // __APPLE__

// ============================== scrum-258.3: filter+prob ====================
//
// Per-layer filter+prob tests. Each test isolates one boundary of the legacy
// simulator.cpp:425 CollectData semantics (filter-fail → drop, filter-pass +
// rng<prob → continue/drop-at-final, filter-pass + rng>=prob → exit):
//
//   SingleMsProb1ZeroOutput        : prob=1.0 final layer → every filter-pass
//                                    exit is "would continue" → 0 returned.
//   MultiMsProb1ZeroOutput          : layer 0 prob=1.0 → all continue (no
//                                    mid-exit); layer 1 prob=1.0 → all
//                                    "would continue" dropped → 0 returned.
//   MultiMsProb0AllMidExits         : layer 0 prob=0.0 → all rays mid-exit on
//                                    layer 0 (ms_layer_idx==0); layer 1 has
//                                    zero input → 0 final-layer exits.
//   MultiMsMidLayerSplit            : default 2-layer scene (layer 0 prob=0.6,
//                                    layer 1 prob=0.0) → BOTH ms_layer_idx==0
//                                    AND ms_layer_idx==1 records present —
//                                    proves rng<prob → continue (mid+final
//                                    split direction is correct).

#if defined(__APPLE__)
namespace {

inline std::vector<ExitRayRecord> RunMetalExitRays(const SceneConfig& scene, const RenderConfig& render,
                                                   uint32_t seed = 42) {
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = seed;

  HostRayBatch host;
  host.count = kRayCount;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  MetalTraceBackend metal;
  metal.BeginSession(spec);
  RootRaySource src = RootRaySource::FromHost(host);
  std::vector<ExitRayRecord> records;
  // task-268.4: the exit buffer is now single-layer-cap and recycled per layer
  // (produce -> DrainExits -> recycle), so multi-MS callers MUST drain after
  // every TraceLayer — a single ReadbackExitRays at the end only sees the last
  // layer (mid-layer ms_layer_idx==0 records would be lost). Mirror the real
  // simulator.cpp per-layer drain. For single-MS this is one drain = equivalent
  // to the old single readback.
  for (size_t layer = 0; layer < scene.ms_.size(); layer++) {
    auto h = metal.TraceLayer(src);
    std::vector<ExitRayRecord> layer_exits;
    metal.DrainExits(layer_exits);
    records.insert(records.end(),
                   std::make_move_iterator(layer_exits.begin()),
                   std::make_move_iterator(layer_exits.end()));
    if (layer + 1 < scene.ms_.size()) {
      src = metal.Recombine(std::move(h), RecombineSpec{ /*shuffle=*/false });
    }
  }
  metal.EndSession();
  return records;
}

}  // namespace

TEST(ExitRecordsTest, MetalBackendSingleMsProb1ZeroOutput) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  auto scene = MakeMetalSceneWithProb(kMaxHits, /*ms_layers=*/1, /*prob=*/1.0f);
  auto render = MakeRectangularRender();
  auto records = RunMetalExitRays(scene, render);
  // prob=1.0 on the (only) final layer: every filter-pass exit gets
  // rng<1.0 → would-continue → dropped (no next layer). Mirrors legacy
  // CollectData behaviour at prob=1.0.
  EXPECT_EQ(records.size(), 0u) << "single-MS prob=1.0 should drop every final-layer exit";
}

TEST(ExitRecordsTest, MetalBackendMultiMsProb1ZeroOutput) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  auto scene = MakeMetalSceneWithProb(kMaxHits, /*ms_layers=*/2, /*prob=*/1.0f);
  auto render = MakeRectangularRender();
  auto records = RunMetalExitRays(scene, render);
  // Layer 0 prob=1.0: all filter-pass → rng<1.0 → continue (no mid-exit).
  // Layer 1 prob=1.0: all filter-pass → rng<1.0 → would-continue dropped.
  EXPECT_EQ(records.size(), 0u) << "multi-MS prob=1.0 on both layers should produce no exits";
}

TEST(ExitRecordsTest, MetalBackendMultiMsProb0AllMidExits) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  auto scene = MakeMetalSceneWithProb(kMaxHits, /*ms_layers=*/2, /*prob=*/0.0f);
  auto render = MakeRectangularRender();
  auto records = RunMetalExitRays(scene, render);
  // Layer 0 prob=0.0: rng<0.0 never true → every filter-pass → mid-exit on
  // layer 0 (ms_layer_idx==0). Layer 1 has zero continuation input → zero
  // final-layer exits.
  ASSERT_GT(records.size(), 0u) << "multi-MS prob=0.0 should emit all rays as layer-0 mid-exits";
  size_t mid_cnt = 0;
  size_t other_cnt = 0;
  for (const auto& rec : records) {
    if (rec.ms_layer_idx == 0u) {
      mid_cnt++;
    } else {
      other_cnt++;
    }
  }
  EXPECT_EQ(other_cnt, 0u) << "no final-layer exits expected when prob=0.0 sinks everything to mid";
  EXPECT_EQ(mid_cnt, records.size());
}

TEST(ExitRecordsTest, MetalBackendMultiMsMidLayerSplit) {
  if (ShouldSkipMetalTests()) {
    GTEST_SKIP() << "LUMICE_SKIP_METAL_TESTS set";
  }
  // MakeMetalScene: layer 0 prob=0.6 → ~60% continue / ~40% mid-exit
  //                                    (ms_layer_idx=0)
  //                 layer 1 prob=0.0 → 100% final-layer exits (ms_layer_idx=1)
  // This is the canonical "rng<prob → continue direction is correct" check:
  // if Step 4/5 swapped the prob comparator (rng>=prob → continue), mid_cnt
  // would still be > 0 by symmetry but final_cnt would collapse to zero
  // (continued rays drop to ~0 instead of ~60%).
  auto scene = MakeMetalScene(kMaxHits, /*ms_layers=*/2);
  auto render = MakeRectangularRender();
  auto records = RunMetalExitRays(scene, render);
  ASSERT_GT(records.size(), 0u);
  size_t mid_cnt = 0;
  size_t final_cnt = 0;
  for (const auto& rec : records) {
    if (rec.ms_layer_idx == 0u) {
      mid_cnt++;
    } else if (rec.ms_layer_idx == 1u) {
      final_cnt++;
    }
  }
  EXPECT_GT(mid_cnt, 0u) << "expected mid-layer exits from layer 0 (prob=0.6)";
  EXPECT_GT(final_cnt, 0u) << "expected final-layer exits from layer 1 (continued rays)";
}
#endif  // __APPLE__

}  // namespace
}  // namespace lumice
