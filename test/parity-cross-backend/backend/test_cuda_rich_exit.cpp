// CUDA rich-exit content tests (task-cuda-rich-exit / scrum-296.3).
//
// Verifies that `CudaTraceBackend::DrainExits` populates `ExitRayRecord.path`
// with a non-zero face-id sequence — the prerequisite that filter (296.5) and
// multi-MS (296.4) consume. The MVP (scrum-295) zero-filled `path` because no
// G1-G6 metric depended on it; this task switches the kernel to record the
// real face sequence and these tests guard the new contract.
//
// Build gate: the entire translation unit is `#if defined(LUMICE_CUDA_ENABLED)`,
// so the binary stays empty on Mac / non-CUDA hosts. The shared `parity_test`
// target picks the file up unconditionally; on hosts without CUDA the file
// contributes zero translation-unit symbols.
//
// Acceptance contract (mirrors plan.md §6 + review feedback Minor 1/3):
//   - path.size_ >= 1 for every emitted record (no MVP zero placeholders left)
//   - >= 10% of records have path.size_ >= 2 (refraction-after-bounce exists;
//     guards against the "only entry external reflect captured" failure mode)
//   - every face id in path is a canonical GetFn face number in [1, poly_cnt]
//     (DrainExits remaps raw poly indices to legacy/Metal face numbers, 296.5;
//     the hex prism numbers faces 1-8, so 0 / 255 / >8 = garbage or truncation)
//   - distinct face ids in path[0] across the test population >= 4 (catches
//     stuck-at-zero / stuck-at-one regressions)

#include <gtest/gtest.h>

#if defined(LUMICE_CUDA_ENABLED)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "config/crystal_config.hpp"
#include "config/filter_config.hpp"
#include "config/light_config.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "core/backend/cuda_trace_backend.hpp"
#include "core/backend/trace_backend.hpp"
#include "core/exit_seam.hpp"
#include "core/raypath.hpp"

namespace lumice {
namespace {

// Deterministic prism scene (matches test_cpu_trace_backend.cpp::MakeSimpleScene
// shape so the geometry — unit prism, 8 polygon faces: 2 basal + 6 sides — is
// stable across the parity layer).
SceneConfig MakePrismScene(size_t max_hits) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  MsInfo ms;
  ms.prob_ = 0.0f;  // single MS final layer
  ScatteringSetting s;
  // Value-initialize the filter (NoneFilterParam pass-through). Without this the
  // POD members FilterConfig::symmetry_/action_/id_ stay INDETERMINATE (a bare
  // `ScatteringSetting s;` default-inits them), and the garbage action_/symmetry_
  // make the device emit gate's DeviceFilterCheck reject every exit → no XYZ
  // accumulation (mirrors test/cpu_test_helpers.hpp which sets this explicitly).
  s.filter_ = FilterConfig{};
  s.crystal_.id_ = 0;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.param_ = prism;
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

RenderConfig MakeRenderConfig() {
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

// Drive CUDA backend on the prism config and return the drained exit records.
// Skips the body (returns empty) if no CUDA device is enumerated at runtime,
// matching how the Python parity layer handles dev49 host-only runs.
std::vector<ExitRayRecord> RunCudaPrism(size_t ray_count, size_t max_hits) {
  if (!CudaDeviceAvailable()) {
    return {};
  }
  auto scene = MakePrismScene(max_hits);
  auto render = MakeRenderConfig();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  CudaTraceBackend backend;
  backend.BeginSession(spec);

  HostRayBatch host;
  host.count = ray_count;
  host.crystal = nullptr;
  host.refractive_index = 0.0f;

  backend.TraceLayer(RootRaySource::FromHost(host));

  std::vector<ExitRayRecord> exits;
  backend.DrainExits(exits);
  backend.EndSession();
  return exits;
}

// =============================================================================
// Rich-exit contract — single-MS prism, host-ingest, no filter.
//
// kPolyCnt is the deterministic polygon face count for a unit prism (2 basal +
// 6 side faces). The kernel widens face ids to uint8_t before writing them
// into ExitFaceSeq::data_, so any value >= kPolyCnt would either be a real
// face-id bug or a stale zero from the MVP placeholder (caught by both the
// `>= 1` and `< kPolyCnt` assertions).
// =============================================================================
constexpr uint8_t kPolyCnt = 8;
constexpr size_t kRayCount = 4096;
constexpr size_t kMaxHits = 8;

TEST(CudaRichExit, NonZeroPathContent) {
  auto exits = RunCudaPrism(kRayCount, kMaxHits);
  if (exits.empty()) {
    GTEST_SKIP() << "No CUDA device available on this host; skipping rich-exit test.";
  }
  ASSERT_GT(exits.size(), 0u) << "CUDA backend produced no exit records — rich-exit "
                                 "verification needs at least one record to inspect.";

  size_t zero_size = 0;
  size_t at_least_two = 0;
  size_t out_of_range = 0;
  std::set<uint8_t> distinct_head_faces;

  for (const auto& rec : exits) {
    const uint8_t sz = rec.path.size_;
    if (sz == 0u) {
      zero_size++;
      continue;
    }
    if (sz >= 2u) {
      at_least_two++;
    }
    distinct_head_faces.insert(rec.path.data_[0]);
    for (uint8_t k = 0u; k < sz; k++) {
      // Canonical GetFn face numbers are 1-indexed (296.5): valid range is
      // [1, kPolyCnt]; 0 / 255 / > kPolyCnt indicate garbage or uint8 truncation.
      if (rec.path.data_[k] < 1u || rec.path.data_[k] > kPolyCnt) {
        out_of_range++;
        break;
      }
    }
  }

  EXPECT_EQ(zero_size, 0u) << "rich-exit gate: " << zero_size << "/" << exits.size()
                           << " exit records still carry path.size_==0 (MVP placeholder leaked).";

  // 10% floor: with prism + max_hits=8 the typical fraction is much higher
  // (every refracted-out ray has at least entry + one bounce in path). The
  // floor guards against a regression where only entry external-reflect (size
  // 1) is captured.
  const size_t at_least_two_floor = exits.size() / 10u;
  EXPECT_GE(at_least_two, at_least_two_floor) << "rich-exit gate: only " << at_least_two << "/" << exits.size()
                                              << " records have path.size_ >= 2 (need >= " << at_least_two_floor
                                              << "). Suspect main-loop append never ran — only entry external-reflect "
                                                 "exits are being recorded.";

  EXPECT_EQ(out_of_range, 0u) << "rich-exit gate: " << out_of_range << "/" << exits.size()
                              << " records have a face id >= poly_cnt (" << static_cast<int>(kPolyCnt)
                              << "). Suspect uint8 truncation or uninitialized path_rec entries.";

  // 4 distinct head faces: with axis-random sampling all 8 prism faces are
  // reachable entry candidates over 4k rays. A floor of 4 catches regressions
  // (stuck-at-zero / stuck-at-one) while tolerating low-probability sampling
  // variance on individual faces.
  EXPECT_GE(distinct_head_faces.size(), 4u) << "rich-exit gate: only " << distinct_head_faces.size()
                                            << " distinct head face ids observed across " << exits.size()
                                            << " records; expected >= 4 across an axis-random prism population. "
                                               "Suspect path[0] stuck at a single face value.";
}

// =============================================================================
// Exit-face fidelity — every record's *last* face id must equal a polygon the
// ray could plausibly exit through. The check is the same upper-bound gate as
// above but isolated to the path tail so a future refactor can't accidentally
// shadow `path[0]` correctness with whatever populates `path[size_-1]`.
// =============================================================================
TEST(CudaRichExit, ExitFaceIsValid) {
  auto exits = RunCudaPrism(kRayCount, kMaxHits);
  if (exits.empty()) {
    GTEST_SKIP() << "No CUDA device available on this host; skipping rich-exit test.";
  }
  ASSERT_GT(exits.size(), 0u);

  for (size_t i = 0; i < exits.size(); i++) {
    const auto& rec = exits[i];
    ASSERT_GE(rec.path.size_, 1u) << "record " << i
                                  << " has empty path; covered by NonZeroPathContent. "
                                     "ExitFaceIsValid expects empty-path records to be filtered already.";
    const uint8_t tail = rec.path.data_[rec.path.size_ - 1u];
    // DrainExits canonicalises raw poly indices to GetFn face numbers (296.5),
    // matching legacy/Metal. The hex prism numbers faces 1..8, so a valid tail
    // is in [1, kPolyCnt]; garbage / truncation lands at 0, 255, or > kPolyCnt.
    EXPECT_GE(tail, 1u) << "record " << i << ": tail face id " << static_cast<int>(tail)
                        << " below canonical 1-indexed face range.";
    EXPECT_LE(tail, kPolyCnt) << "record " << i << ": tail face id " << static_cast<int>(tail)
                              << " is not a valid prism face (1.." << static_cast<int>(kPolyCnt) << ").";
  }
}

// =============================================================================
// task-exit-seam-crystal-count: CUDA backend reports final-layer setting count.
// Semantics: return value is the LAST MS layer's setting count (mirrors
// Impl::final_layer_crystals_ populated during BeginSession), not a cross-layer
// sum. Locks plan §2 default assumption 2.
// =============================================================================
TEST(CudaBackendCrystalCount, ReturnsFinalLayerSettings) {
  if (!CudaDeviceAvailable()) {
    GTEST_SKIP() << "No CUDA device available on this host.";
  }

  // Case 1 — single MS, single crystal setting → count == 1.
  {
    auto scene = MakePrismScene(/*max_hits=*/4);
    auto render = MakeRenderConfig();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 21;

    CudaTraceBackend backend;
    backend.BeginSession(spec);
    EXPECT_EQ(backend.GetLastBatchCrystalCount(), 1u);
    backend.EndSession();
  }

  // Case 2 — multi MS scene with 3 settings on the FINAL layer, 1 on the
  // first. Return value MUST be 3, not 4 (cross-layer sum).
  {
    auto scene = MakePrismScene(/*max_hits=*/4);
    // Convert single-MS scene to 2-MS: original layer becomes the first
    // (with prob>0 to route continuations), append a final layer with 3
    // crystal settings.
    scene.ms_.front().prob_ = 0.6f;

    MsInfo final_ms;
    final_ms.prob_ = 0.0f;
    ScatteringSetting base = scene.ms_.front().setting_.front();
    for (int i = 0; i < 3; i++) {
      ScatteringSetting s = base;
      s.crystal_.id_ = static_cast<IdType>(100 + i);
      s.crystal_proportion_ = 1.0f / 3.0f;
      final_ms.setting_.push_back(std::move(s));
    }
    scene.ms_.push_back(std::move(final_ms));
    ASSERT_EQ(scene.ms_.back().setting_.size(), 3u);
    ASSERT_EQ(scene.ms_.front().setting_.size(), 1u);

    auto render = MakeRenderConfig();
    SessionSpec spec;
    spec.scene = &scene;
    spec.render = &render;
    spec.wl = WlParam{ 550.0f, 1.0f };
    spec.seed = 23;

    CudaTraceBackend backend;
    backend.BeginSession(spec);
    EXPECT_EQ(backend.GetLastBatchCrystalCount(), 3u) << "Final-layer settings count, not cross-layer sum (would be 4)";
    backend.EndSession();
  }
}

// =============================================================================
// task-gpu-rng-ray-index-uint64 white-box injection: assert the CUDA kernels
// actually consume the new hi-epoch fields on real hardware. The in-range parity
// battery (all CUDA parity_*.py) only exercises hi==0, which the fix makes
// bit-exact with the pre-fix stream by construction — so a broken host↔device
// wiring of the hi field (wrong kernel param, unread field, mixing not applied)
// slips through parity silently. The fix touches THREE independent device PCG
// streams — gen (gen_root_kernel), gate (trace_single_ms_kernel emit gate) and
// transit (transit_multi_ms_kernel continuation) — and each is a separate
// transformation chain that must be asserted directly (learnings:
// assertion-and-coverage-traps "跨独立代码路径的 AC 间接覆盖论证不成立"). A single
// combined injection would NOT do: gen alone moving the output could mask a
// broken gate/transit wire. So `SetInitialRayBaseForTest` takes per-stream bases
// and each test below drives EXACTLY ONE stream into a non-zero hi epoch.
//
// dev49-only (skips on Mac / hosts without a CUDA device).
// =============================================================================
namespace hi_wire {

constexpr size_t kEpoch = static_cast<size_t>(1) << 32;
// Slots 0/1 = hi==0 baseline (must be bit-identical); 2 = hi==1; 3 = hi==2.
constexpr size_t kBases[4] = { 0u, 0u, kEpoch, 2u * kEpoch };

// Single-crystal random-axis prism scene. Random orientation makes the gen
// stream observable (different mixed_seed → different rotation). `final_prob`
// controls the final-layer emit-gate keep fraction: it must be > 0 for the gate
// stream to be observable (with prob==0 the gate draw never gates emit, so the
// gate seed cannot move the image). filter_ is value-initialized (pass-through)
// so the device emit gate does not reject every exit (a bare `ScatteringSetting
// s;` leaves FilterConfig POD members indeterminate → garbage action_ rejects
// all → no XYZ accumulation).
SceneConfig MakeRandomAxisScene(size_t max_hits, float final_prob) {
  SceneConfig scene;
  scene.ray_num_ = 0;
  scene.max_hits_ = max_hits;
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.5f };
  scene.light_source_.spectrum_ = std::vector<WlParam>{ { 550.0f, 1.0f } };

  MsInfo ms;
  ms.prob_ = final_prob;
  ScatteringSetting s;
  s.filter_ = FilterConfig{};
  s.crystal_.id_ = 0;
  PrismCrystalParam prism;
  prism.h_ = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  for (auto& d : prism.d_) {
    d = Distribution{ DistributionType::kNoRandom, 1.0f, 0.0f };
  }
  s.crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  s.crystal_.axis_.latitude_dist = Distribution{ DistributionType::kUniform, 0.0f, 360.0f };
  s.crystal_.param_ = prism;
  s.crystal_proportion_ = 1.0f;
  ms.setting_.push_back(std::move(s));
  scene.ms_.push_back(std::move(ms));
  return scene;
}

// Two-MS-layer scene tuned so the transit stream is cleanly OBSERVABLE at
// layer-1's d_dirs_ without the atomic-compaction confound:
//   - Point sun (diameter 0) + FIXED-axis layer 0 → every layer-0 ray is
//     identical → every continuation ray carries the SAME world direction. So
//     layer-1's d_dirs_[tid] = R1(tid)^-1 · const depends only on tid (the
//     transit orientation), independent of which continuation ray landed at
//     that tid → deterministic for a fixed (seed, transit_base).
//   - RANDOM-axis layer 1 → the transit kernel samples a per-tid orientation
//     R1(tid) from the transit PCG stream, so a non-zero transit hi moves it.
// Layer 0 continues (prob 0.6) into layer 1 (final, prob 0).
SceneConfig MakeTwoLayerScene(size_t max_hits) {
  SceneConfig scene = MakeRandomAxisScene(max_hits, /*final_prob=*/0.0f);  // layer-1 template (random axis)
  scene.light_source_.param_ = SunParam{ 30.0f, 0.0f, 0.0f };              // point sun (diameter 0)
  MsInfo layer0 = scene.ms_[0];
  layer0.prob_ = 0.6f;
  layer0.setting_[0].crystal_.axis_.azimuth_dist = Distribution{ DistributionType::kNoRandom, 0.0f, 0.0f };
  layer0.setting_[0].crystal_.axis_.latitude_dist = Distribution{ DistributionType::kNoRandom, 20.0f, 0.0f };
  scene.ms_.insert(scene.ms_.begin(), std::move(layer0));
  return scene;
}

// Full-sphere rectangular render — keeps every exit direction in the field so
// the device-fused XYZ image responds to orientation (a narrow fisheye culls
// most random-orientation exits, collapsing the image toward constant).
RenderConfig MakeFullViewRender() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kRectangular;
  cfg.lens_.fov_ = 360.0f;
  cfg.resolution_[0] = 64;
  cfg.resolution_[1] = 32;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kFull;
  return cfg;
}

double L1RelDiff(const std::vector<float>& a, const std::vector<float>& b) {
  double num = 0.0;
  double denom = 0.0;
  for (size_t i = 0; i < a.size(); i++) {
    num += std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i]));
    denom += 0.5 * (std::abs(static_cast<double>(a[i])) + std::abs(static_cast<double>(b[i])));
  }
  return denom > 0.0 ? num / denom : 0.0;
}

// Baseline (hi==0 vs hi==0) determinism + three-way (0-2, 0-3, 2-3) divergence
// on a set of 4 images. `stream` names the stream under test for failures.
void AssertImageHiDivergence(const std::vector<std::vector<float>>& images, const char* stream) {
  const double baseline = L1RelDiff(images[0], images[1]);
  const double threshold = std::max(baseline * 5.0, 1e-3);
  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    const double d = L1RelDiff(images[pr.first], images[pr.second]);
    EXPECT_GT(d, threshold) << stream << " stream: base_pair=(" << pr.first << "," << pr.second
                            << ") per-pixel L1 rel diff=" << d << " baseline(hi==0 vs hi==0)=" << baseline
                            << " threshold=" << threshold << " — the " << stream
                            << " hi wiring is not reaching the device (hi!=0 image indistinguishable from hi==0).";
  }
}

}  // namespace hi_wire

// gen stream — observed at the MOST direct point: the device-gen'd ray
// directions (d_dirs_, the immediate gen_root_kernel output). Injects ONLY the
// gen base; transit/gate stay hi==0.
TEST(CudaRngHiWiring, GenStreamWireUp) {
  if (!CudaDeviceAvailable()) {
    GTEST_SKIP() << "No CUDA device available on this host; requires dev49.";
  }
  auto scene = hi_wire::MakeRandomAxisScene(/*max_hits=*/8, /*final_prob=*/0.0f);
  auto render = MakeRenderConfig();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  std::vector<std::vector<float>> dirs(4);
  for (int i = 0; i < 4; i++) {
    CudaTraceBackend backend;
    backend.BeginSession(spec);
    backend.SetInitialRayBaseForTest(/*gen=*/hi_wire::kBases[i], /*transit=*/0u, /*gate=*/0u);
    HostRayBatch host;
    host.count = kRayCount;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    backend.TraceLayer(RootRaySource::FromHost(host));
    size_t n = backend.ReadbackGenDirsForTest(dirs[i], kRayCount);
    backend.EndSession();
    ASSERT_EQ(n, 3u * kRayCount) << "base_idx=" << i << " — gen-dir readback returned " << n;
  }

  EXPECT_EQ(dirs[0], dirs[1]) << "hi==0 gen directions non-deterministic — cannot distinguish wiring from noise.";
  auto frac_moved = [](const std::vector<float>& a, const std::vector<float>& b) {
    const size_t n = a.size() / 3u;
    size_t moved = 0u;
    for (size_t r = 0; r < n; r++) {
      const float dx = a[3 * r + 0] - b[3 * r + 0];
      const float dy = a[3 * r + 1] - b[3 * r + 1];
      const float dz = a[3 * r + 2] - b[3 * r + 2];
      if (dx * dx + dy * dy + dz * dz > 1e-10f) {
        moved++;
      }
    }
    return n > 0u ? static_cast<double>(moved) / static_cast<double>(n) : 0.0;
  };
  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    const double moved = frac_moved(dirs[pr.first], dirs[pr.second]);
    EXPECT_GT(moved, 0.9) << "gen stream: base_pair=(" << pr.first << "," << pr.second << ") only " << moved
                          << " of gen directions differ — gen_ray_base_hi not reaching the orientation sample.";
  }
}

// gate FINAL stream (gate_ray_base_final_hi, ms_mode==0) — observed via the
// device-fused XYZ image. A single-MS scene is always ms_mode==0, so its emit
// gate uses gate_f seeded from gate_ray_base_final_hi. Final-layer prob 0.5 makes
// the emit gate's prob draw actually gate emission, so a different gate_final
// mixed_seed emits a different subset → different image. Injects ONLY the gate
// base; gen/transit stay hi==0. NOTE: this covers gate_ray_base_FINAL_hi only —
// the ms_mode==1 per-bounce gate_stream (gate_ray_base_hi) is a distinct kernel
// param + stream, covered separately by GateMsMode1StreamWireUp below.
TEST(CudaRngHiWiring, GateStreamWireUp) {
  if (!CudaDeviceAvailable()) {
    GTEST_SKIP() << "No CUDA device available on this host; requires dev49.";
  }
  auto scene = hi_wire::MakeRandomAxisScene(/*max_hits=*/8, /*final_prob=*/0.5f);
  auto render = hi_wire::MakeFullViewRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  const size_t kImgFloats = static_cast<size_t>(render.resolution_[0]) * render.resolution_[1] * 3u;
  std::vector<std::vector<float>> images(4);
  for (int i = 0; i < 4; i++) {
    images[i].assign(kImgFloats, 0.0f);
    CudaTraceBackend backend;
    backend.BeginSession(spec);
    backend.SetInitialRayBaseForTest(/*gen=*/0u, /*transit=*/0u, /*gate=*/hi_wire::kBases[i]);
    HostRayBatch host;
    host.count = kRayCount;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    backend.TraceLayer(RootRaySource::FromHost(host));
    XyzImageData img{ images[i].data(), render.resolution_[0], render.resolution_[1] };
    float landed_weight = 0.0f;
    backend.ReadbackXyzAccum(img, landed_weight);
    backend.EndSession();
    ASSERT_GT(landed_weight, 0.0f) << "base_idx=" << i << " — no rays accumulated; gate test cannot observe.";
  }
  hi_wire::AssertImageHiDivergence(images, "gate");
}

// gate PER-BOUNCE stream (gate_ray_base_hi, ms_mode==1) — RNG-ONLY observation,
// distinct from GateStreamWireUp's gate_ray_base_final_hi. In a 2-layer scene the
// FIRST TraceLayer (layer 0) is ms_mode==1, so trace_single_ms_kernel's
// gate_stream (seeded from gate_mixed_seed = f(gate_ray_base_hi)) is executed on
// device. Its do-continue draws feed the atomic continuation compaction, so the
// image / continuation set are confounded (same reason TransitStreamWireUp uses
// a probe) — we therefore read the gate_stream's raw pcg_uniform draw directly
// via the RNG probe (written at the top of trace_single_ms_kernel when enabled).
// Injects ONLY the gate base; a non-zero gate hi must move gate_mixed_seed →
// move every draw; hi==0 runs are bit-identical. Enabling the probe BEFORE the
// (single) TraceLayer routes it to trace_single_ms_kernel (vs the transit test
// which enables it between layers to route it to transit_multi_ms_kernel).
TEST(CudaRngHiWiring, GateMsMode1StreamWireUp) {
  if (!CudaDeviceAvailable()) {
    GTEST_SKIP() << "No CUDA device available on this host; requires dev49.";
  }
  auto scene = hi_wire::MakeTwoLayerScene(/*max_hits=*/6);  // layer 0 → ms_mode==1
  auto render = hi_wire::MakeFullViewRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  std::vector<std::vector<float>> draws(4);
  for (int i = 0; i < 4; i++) {
    CudaTraceBackend backend;
    backend.BeginSession(spec);
    backend.SetInitialRayBaseForTest(/*gen=*/0u, /*transit=*/0u, /*gate=*/hi_wire::kBases[i]);
    backend.EnableRngProbeForTest(kRayCount);  // before the first TraceLayer → trace_single_ms_kernel fills it
    HostRayBatch host;
    host.count = kRayCount;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));  // layer 0, ms_mode==1
    ASSERT_NE(h0, nullptr);
    size_t n = backend.ReadbackRngProbeForTest(draws[i], kRayCount);
    backend.EndSession();
    ASSERT_EQ(n, kRayCount) << "base_idx=" << i << " — gate RNG-probe readback returned " << n;
  }

  EXPECT_EQ(draws[0], draws[1]) << "hi==0 gate draws non-deterministic — cannot distinguish wiring from noise.";
  auto frac_moved = [](const std::vector<float>& a, const std::vector<float>& b) {
    size_t moved = 0u;
    for (size_t r = 0; r < a.size(); r++) {
      if (std::abs(a[r] - b[r]) > 1e-7f) {
        moved++;
      }
    }
    return a.empty() ? 0.0 : static_cast<double>(moved) / static_cast<double>(a.size());
  };
  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    const double moved = frac_moved(draws[pr.first], draws[pr.second]);
    EXPECT_GT(moved, 0.9) << "gate ms_mode==1 stream: base_pair=(" << pr.first << "," << pr.second << ") only " << moved
                          << " of gate PCG draws differ — gate_ray_base_hi wiring not reaching the device stream.";
  }
}

// transit stream — RNG-ONLY observation. Every ray-physics channel (the XYZ
// image, or d_dirs_) is confounded here: the atomic continuation compaction
// assigns tids non-deterministically, so each transit orientation is combined
// with a different continuation ray across identical runs (measured d_dirs_ /
// image baseline is NON-zero, masking the transit-hi effect behind compaction
// shuffle). So instead of observing the transit orientation's effect on rays we
// observe the transit PCG STREAM DIRECTLY: the continuation transit kernel
// writes each thread's raw pcg_uniform draw (from its transit_mixed_seed stream)
// into a probe sink. That draw is a pure function of (transit_mixed_seed,
// gp.gen_ray_base + tid) — deterministic per tid, independent of ray physics and
// compaction. Injects ONLY the transit base; a non-zero transit hi must change
// transit_mixed_seed → change the draw at every tid; hi==0 runs are bit-
// identical.
TEST(CudaRngHiWiring, TransitStreamWireUp) {
  if (!CudaDeviceAvailable()) {
    GTEST_SKIP() << "No CUDA device available on this host; requires dev49.";
  }
  auto scene = hi_wire::MakeTwoLayerScene(/*max_hits=*/6);
  auto render = hi_wire::MakeFullViewRender();
  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  std::vector<std::vector<float>> draws(4);
  size_t cont_count = 0;
  for (int i = 0; i < 4; i++) {
    CudaTraceBackend backend;
    backend.BeginSession(spec);
    backend.SetInitialRayBaseForTest(/*gen=*/0u, /*transit=*/hi_wire::kBases[i], /*gate=*/0u);
    HostRayBatch host;
    host.count = kRayCount;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    auto h0 = backend.TraceLayer(RootRaySource::FromHost(host));
    ASSERT_NE(h0, nullptr);
    const size_t cont = h0->ContinuationCount();
    ASSERT_GT(cont, 16u) << "base_idx=" << i << " — too few continuation rays (" << cont
                         << ") to observe transit wiring; raise ray count / layer-0 prob.";
    if (i == 0) {
      cont_count = cont;
    }
    ASSERT_EQ(cont, cont_count) << "base_idx=" << i
                                << " — continuation count varies across runs; gen/gate not isolated.";
    backend.EnableRngProbeForTest(cont_count);  // sink for the next (continuation) layer's transit draws
    RecombineSpec rspec;
    rspec.shuffle = false;
    auto roots1 = backend.Recombine(std::move(h0), rspec);
    auto h1 = backend.TraceLayer(roots1);
    ASSERT_NE(h1, nullptr);
    size_t n = backend.ReadbackRngProbeForTest(draws[i], cont_count);
    backend.EndSession();
    ASSERT_EQ(n, cont_count) << "base_idx=" << i << " — transit RNG-probe readback returned " << n;
  }

  EXPECT_EQ(draws[0], draws[1]) << "hi==0 transit draws non-deterministic — cannot distinguish wiring from noise.";
  auto frac_moved = [](const std::vector<float>& a, const std::vector<float>& b) {
    size_t moved = 0u;
    for (size_t r = 0; r < a.size(); r++) {
      if (std::abs(a[r] - b[r]) > 1e-7f) {
        moved++;
      }
    }
    return a.empty() ? 0.0 : static_cast<double>(moved) / static_cast<double>(a.size());
  };
  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    const double moved = frac_moved(draws[pr.first], draws[pr.second]);
    EXPECT_GT(moved, 0.9) << "transit stream: base_pair=(" << pr.first << "," << pr.second << ") only " << moved
                          << " of transit PCG draws differ — transit hi wiring not reaching the device stream.";
  }
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
