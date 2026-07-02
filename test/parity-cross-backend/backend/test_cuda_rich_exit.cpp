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
// actually consume the new `gen_ray_base_hi` field. Mirrors the Metal-side test
// (test_metal_root_gen.cpp::GenRayBaseHiWireUp) — the in-range parity battery
// (all CUDA parity_*.py) only exercises hi==0, which the plan makes bit-exact
// with the pre-fix stream by construction; a broken host↔device wiring for
// `gen_ray_base_hi` / mixed_seed would slip through silently.
//
// See the Metal-side test comment for the full metric-choice rationale
// (aggregate channel sum is an MC INVARIANT — cannot distinguish "same PCG"
// from "different PCG drawing from the same distribution"; per-pixel L1 responds
// but is scene-dependent, so we self-calibrate against a hi==0-repeat baseline
// rather than fix an absolute threshold).
//
// dev49-only (skips on Mac / hosts without a CUDA device).
// =============================================================================
TEST(CudaRngHiWiring, GenRayBaseHiWireUp) {
  if (!CudaDeviceAvailable()) {
    GTEST_SKIP() << "No CUDA device available on this host; task-gpu-rng-ray-index-uint64 "
                    "device-side wiring test requires dev49.";
  }
  auto scene = MakePrismScene(/*max_hits=*/8);
  auto render = MakeRenderConfig();

  SessionSpec spec;
  spec.scene = &scene;
  spec.render = &render;
  spec.wl = WlParam{ 550.0f, 1.0f };
  spec.seed = 42;

  constexpr size_t kEpoch = static_cast<size_t>(1) << 32;
  // Slots 0/1 = hi==0 baseline (atomic-noise floor); 2 = hi==1; 3 = hi==2.
  const size_t kBases[] = { 0u, 0u, kEpoch, 2u * kEpoch };

  const int kW = render.resolution_[0];
  const int kH = render.resolution_[1];
  const size_t kImgFloats = static_cast<size_t>(kW) * static_cast<size_t>(kH) * 3u;
  std::vector<std::vector<float>> images(4);

  for (int i = 0; i < 4; i++) {
    images[i].assign(kImgFloats, 0.0f);
    CudaTraceBackend backend;
    backend.BeginSession(spec);
    backend.SetInitialRayBaseForTest(kBases[i]);

    HostRayBatch host;
    host.count = kRayCount;
    host.crystal = nullptr;
    host.refractive_index = 0.0f;
    backend.TraceLayer(RootRaySource::FromHost(host));

    XyzImageData img{ images[i].data(), kW, kH };
    float landed_weight = 0.0f;
    backend.ReadbackXyzAccum(img, landed_weight);
    backend.EndSession();

    double total = 0.0;
    for (float v : images[i]) {
      total += static_cast<double>(v);
    }
    EXPECT_GT(total, 0.0) << "base_idx=" << i
                          << " — empty image; CUDA test setup is broken (device-fused accum "
                             "path may not be routing exits into d_xyz_buf).";
  }

  auto l1_rel_diff = [](const std::vector<float>& a, const std::vector<float>& b) {
    double num = 0.0;
    double denom = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
      num += std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i]));
      denom += 0.5 * (std::abs(static_cast<double>(a[i])) + std::abs(static_cast<double>(b[i])));
    }
    return denom > 0.0 ? num / denom : 0.0;
  };

  const double baseline = l1_rel_diff(images[0], images[1]);
  const double threshold = std::max(baseline * 5.0, 1e-3);

  const std::pair<int, int> kPairs[] = { { 0, 2 }, { 0, 3 }, { 2, 3 } };
  for (auto pr : kPairs) {
    double d = l1_rel_diff(images[pr.first], images[pr.second]);
    EXPECT_GT(d, threshold) << "base_pair=(" << pr.first << "," << pr.second << ") per-pixel L1 rel diff=" << d
                            << " baseline (hi==0 vs hi==0)=" << baseline << " threshold=" << threshold
                            << " — CUDA device-side hi wire-up appears broken: hi!=0 stream produced pixels "
                               "no different from same-stream atomic noise.";
  }
}

}  // namespace
}  // namespace lumice

#endif  // defined(LUMICE_CUDA_ENABLED)
