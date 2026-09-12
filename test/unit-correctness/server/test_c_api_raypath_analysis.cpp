// The analysis run's C API surface (lumice.h "Raypath Analysis Run"): request validation, the
// two mutual-exclusion error codes, the frame getters end to end on the 22° halo scene, what a
// render frame and an analysis frame each refuse to carry, the truncation branch of the entry
// copy, and the pixel -> direction inverse against the forward projection it must invert.
//
//   AC1  RenderInProgressIsServerError (both directions, through the C API).
//   AC2  ActiveBackendReadsCpuDuringAnalysis (the readable half; the physics half — the histogram
//        exists only on the CPU path — is ServerAnalysisRunGpu in test_server_analysis_run.cpp).
//   AC5  the v4.29 compile-time guard.
//   AC6  UnprojectPixelInvertsTheForwardProjection: every lens branch, forward(inverse(px)) == px.
//   Plan risk 4: DeepChainIsTruncatedNotOverrun.
//   Issue note: an analysis frame's raw-XYZ / render getters write their sentinel at out[0].

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "core/annotation_overlay.hpp"  // annotation::ToRenderConfig
#include "core/lens_proj_build.hpp"     // BuildProjParams
#include "core/scatter_accum.hpp"       // MakeCameraRotation
#include "include/lumice.h"
#include "server/c_api_internal.hpp"              // ToAnnotationViewSnapshot, WrapResultFrameForTest
#include "server/raypath_histogram_consumer.hpp"  // FormatRaypathChainDisplay (the truncation fixture premise)
#include "server/server.hpp"

static_assert(LUMICE_API_VERSION >= 435, "the analysis run needs the v4.35 header (bounded record fields)");

// The layout the ctypes mirrors in test/e2e/capi_runner.py are written against. Sizes AND
// offsets, so a field inserted in the middle (which keeps the size) is caught as well as
// one appended; the Python side pins the same numbers, so either side moving turns one of
// the two red before the library writes past a Python buffer.
static_assert(sizeof(LUMICE_AnnotationView) == 48, "LUMICE_AnnotationView layout changed; update capi_runner.py");
static_assert(sizeof(LUMICE_RaypathAnalysisRequest) == 88, "LUMICE_RaypathAnalysisRequest layout changed");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, frame_view) == 4, "");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, cone_center) == 52, "");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, infinite) == 72, "");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, ray_num) == 80, "");
static_assert(sizeof(LUMICE_RaypathChainSegment) == 264, "LUMICE_RaypathChainSegment layout changed");
static_assert(sizeof(LUMICE_RaypathHistogramEntry) == 5608, "LUMICE_RaypathHistogramEntry layout changed");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, chain_len) == 2112, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, display) == 2116, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, energy) == 5320, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, count) == 5328, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, ring_energy) == 5336, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, ring_count) == 5592, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, error_bound) == 5600, "");
static_assert(sizeof(LUMICE_RaypathAnalysisInfo) == 64, "LUMICE_RaypathAnalysisInfo layout changed");
static_assert(offsetof(LUMICE_RaypathAnalysisInfo, snapshot_generation) == 24, "");
static_assert(offsetof(LUMICE_RaypathAnalysisInfo, other_energy) == 32, "");
static_assert(offsetof(LUMICE_RaypathAnalysisInfo, other_count) == 40, "");
static_assert(offsetof(LUMICE_RaypathAnalysisInfo, truncated_chain_count) == 48, "");
static_assert(offsetof(LUMICE_RaypathAnalysisInfo, max_row_error) == 56, "");

namespace {

// halo_22.json's crystal and sun; the ray budget is the parameter.
std::string Halo22Json(const char* ray_num) {
  return std::string(R"({
  "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.2},
               "axis": {"zenith": {"type": "uniform", "mean": 90, "std": 360},
                        "azimuth": {"type": "uniform", "mean": 0, "std": 360}}}],
  "filter": [],
  "scene": {"light_source": {"type": "sun", "altitude": 20.0, "spectrum": "D65"},
            "ray_num": )") +
         ray_num + R"(, "max_hits": 7,
            "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 10}]}]},
  "render": [{"ev_mode": "absolute", "id": 1,
              "lens": {"type": "fisheye_equal_area", "fov": 120},
              "resolution": [64, 64], "view": {"elevation": 20}}]
})";
}

LUMICE_ErrorCode CommitJson(LUMICE_Server* server, const std::string& json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

// The analysis's twin of CommitJson (v4.36): the scene is the call's own, built and destroyed
// around it exactly as the commit's is — LUMICE_StartRaypathAnalysis deep-copies what it needs.
LUMICE_ErrorCode StartAnalysis(LUMICE_Server* server, const std::string& json,
                               const LUMICE_RaypathAnalysisRequest* request) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_StartRaypathAnalysis(server, scene, request);
  LUMICE_SceneDestroy(scene);
  return err;
}

// The scene's own budget, as every request here asked for before the field existed (v4.32): a
// zero-initialized `infinite` would be "0 rays", so the sentinel is set explicitly, as the header
// says it must be.
LUMICE_RaypathAnalysisRequest FullSky() {
  LUMICE_RaypathAnalysisRequest req{};
  req.roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  req.infinite = LUMICE_RAYPATH_RAY_BUDGET_SCENE_DEFAULT;
  return req;
}

// The reduction every read here asks for unless it is the subject: P|B|D, the GUI's default and
// what the old record-time reduction always applied.
constexpr int kSymAll = LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;

LUMICE_RaypathAnalysisRequest Cone() {
  LUMICE_RaypathAnalysisRequest req = FullSky();
  req.roi_mode = LUMICE_RAYPATH_ROI_CONE;
  req.cone_center[0] = 0.0f;
  req.cone_center[1] = 0.0f;
  req.cone_center[2] = -1.0f;
  req.cone_radius_rad = 0.1f;
  req.cone_ring_count = 4;
  return req;
}

int Lifecycle(LUMICE_Server* server) {
  LUMICE_SimLifecycleResult lc{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
  return lc.lifecycle;
}

bool WaitForCompletedAndDrained(LUMICE_Server* server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    LUMICE_DrainResult drain{};
    EXPECT_EQ(LUMICE_GetDrainStatus(server, &drain), LUMICE_OK);
    if (Lifecycle(server) == LUMICE_LIFECYCLE_COMPLETED && drain.drained_epoch == drain.current_epoch) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// The live ray count past `above`: true once the run has traced more than that many rays.
bool WaitForRayCountAbove(LUMICE_Server* server, LUMICE_RayCount above, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    LUMICE_RayCount n = 0;
    EXPECT_EQ(LUMICE_GetSimRayCount(server, &n), LUMICE_OK);
    if (n > above) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

bool WaitForFirstRays(LUMICE_Server* server, int timeout_ms) {
  return WaitForRayCountAbove(server, 0, timeout_ms);
}

int ActiveBackend(LUMICE_Server* server) {
  int b = -1;
  EXPECT_EQ(LUMICE_GetActiveBackend(server, &b), LUMICE_OK);
  return b;
}

class CApiRaypathAnalysis : public ::testing::Test {
 protected:
  void SetUp() override {
    LUMICE_ServerConfig cfg{};
    cfg.num_workers = 2;
    server_ = LUMICE_CreateServerEx(&cfg);
    ASSERT_NE(server_, nullptr);
  }
  void TearDown() override {
    LUMICE_StopServer(server_);
    LUMICE_DestroyServer(server_);
  }
  LUMICE_Server* server_ = nullptr;
};

// ---------------------------------------------------------------------------
// Request validation: every rejection the header promises, as a return code.
// ---------------------------------------------------------------------------
TEST_F(CApiRaypathAnalysis, RequestValidation) {
  LUMICE_RaypathAnalysisRequest req = FullSky();
  const std::string halo = Halo22Json("1000");
  LUMICE_Scene* scene = nullptr;
  ASSERT_EQ(LUMICE_SceneFromJson(halo.c_str(), &scene), LUMICE_OK);
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(nullptr, scene, &req), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, nullptr, &req), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, scene, nullptr), LUMICE_ERR_NULL_ARG);
  LUMICE_SceneDestroy(scene);

  req.roi_mode = 7;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);

  // The ray budget's `infinite` has exactly three spellings (v4.32); the rest are rejected
  // before anything else is looked at. (The three legal ones each start a run in the cases
  // below — RequestRayBudgetOverridesTheSceneAndTheSentinelKeepsIt and
  // RequestInfiniteBudgetOutlivesAFiniteSceneUntilStopped — so they are not re-driven here.)
  for (const int illegal : { 2, -2, 0xFF }) {
    req = FullSky();
    req.infinite = illegal;
    EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE) << "infinite = " << illegal;
  }

  req = Cone();
  req.cone_radius_rad = 0.0f;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);
  req = Cone();
  req.cone_center[2] = 0.0f;  // zero vector
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);
  req = Cone();
  req.cone_ring_count = 0;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);
  req = Cone();
  req.cone_ring_count = LUMICE_MAX_RAYPATH_CONE_RINGS + 1;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);

  req = FullSky();
  req.roi_mode = LUMICE_RAYPATH_ROI_IN_FRAME;
  req.frame_view.width = 64;
  req.frame_view.height = 64;
  req.frame_view.lens_type = LUMICE_LENS_TYPE_GLOBE + 1;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);
  req.frame_view.lens_type = LUMICE_LENS_TYPE_LINEAR;
  req.frame_view.visible = 3;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);
  req.frame_view.visible = LUMICE_VISIBLE_FULL;
  req.frame_view.width = 0;
  EXPECT_EQ(StartAnalysis(server_, halo, &req), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_IDLE) << "none of the rejections started anything";
}

// A scene the server cannot use is rejected with the code LUMICE_CommitScene gives it, after
// the request's own validation, and with nothing stopped or replaced. LUMICE_SceneFromJson
// validates what it parses, so the scene is built through the scratch route: a scattering
// entry naming a crystal the scene does not have is a config error the server's parse
// reports, not one the scene builder sees.
TEST_F(CApiRaypathAnalysis, ARejectedSceneIsTheCommitsCodeAndChangesNothing) {
  LUMICE_Scene* dangling = LUMICE_SceneCreate();
  ASSERT_NE(dangling, nullptr);
  ASSERT_EQ(LUMICE_SceneSetLightSource(dangling, 20.0f, 0.0f, 0.5f, "D65"), LUMICE_OK);
  LUMICE_ScatterLayer layer{};
  layer.probability = 0.0f;
  layer.entry_count = 1;
  layer.entries[0].crystal_id = 99;
  layer.entries[0].proportion = 10.0f;
  layer.entries[0].filter_id = -1;
  int layer_id = -1;
  ASSERT_EQ(LUMICE_SceneAddScatterLayer(dangling, &layer, &layer_id), LUMICE_OK);

  // Positive control for the code: the commit says the same.
  EXPECT_EQ(LUMICE_CommitScene(server_, dangling, nullptr), LUMICE_ERR_INVALID_CONFIG);
  LUMICE_RaypathAnalysisRequest req = FullSky();
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, dangling, &req), LUMICE_ERR_INVALID_CONFIG);
  // The request is validated first: a bad request over a bad scene is the request's code.
  req.roi_mode = 7;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, dangling, &req), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_IDLE);

  // Over an analysis in flight: the rejection leaves it in flight, at its own epoch.
  req = FullSky();
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("\"infinite\""), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  LUMICE_SimLifecycleResult before{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &before), LUMICE_OK);
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, dangling, &req), LUMICE_ERR_INVALID_CONFIG);
  LUMICE_SimLifecycleResult after{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &after), LUMICE_OK);
  EXPECT_EQ(after.lifecycle, LUMICE_LIFECYCLE_RUNNING) << "the analysis in flight was not stopped";
  EXPECT_EQ(after.epoch, before.epoch) << "a rejected scene must not mint an epoch";
  LUMICE_SceneDestroy(dangling);
}

// ---------------------------------------------------------------------------
// AC4: the scene is the call's own. No LUMICE_CommitScene before the analysis; the commit
// after it renders; and the render's own mutual exclusion is unchanged by the new argument
// (RenderInProgressIsServerError below keeps that half).
// ---------------------------------------------------------------------------
TEST_F(CApiRaypathAnalysis, AnalysisNeedsNoPriorCommitAndTheCommitAfterItRenders) {
  LUMICE_SimLifecycleResult lc{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  ASSERT_EQ(lc.epoch, 0u) << "positive control: nothing submitted yet";
  const LUMICE_RaypathAnalysisRequest req = FullSky();
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("40000"), &req), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.epoch, 1u) << "the analysis is a submission of its own: it minted the epoch";
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  EXPECT_EQ(ActiveBackend(server_), LUMICE_BACKEND_CPU);
  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  LUMICE_RaypathAnalysisInfo info{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1);
  ASSERT_GE(info.entry_count, 1);
  std::vector<LUMICE_RaypathHistogramEntry> entries(static_cast<size_t>(info.entry_count) + 1);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, entries.data(), info.entry_count + 1), LUMICE_OK);
  EXPECT_STREQ(entries[0].display, "3-5") << "the 22-degree path of the handed scene";
  LUMICE_StatsResult stats{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 40000u) << "the handed scene's budget";
  LUMICE_ReleaseResultFrame(frame);

  // The commit after it is a render, on its own budget, with an image and no histogram.
  ASSERT_EQ(CommitJson(server_, Halo22Json("2000")), LUMICE_OK);
  ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &lc), LUMICE_OK);
  EXPECT_EQ(lc.epoch, 2u);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 0) << "a render frame carries no histogram";
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 2000u);
  LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1]{};
  ASSERT_EQ(LUMICE_FrameGetRender(frame, renders, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_NE(renders[0].img_buffer, nullptr) << "the render's image";
  LUMICE_ReleaseResultFrame(frame);
}

// ---------------------------------------------------------------------------
// AC1 through the C API, both directions, plus AC2's readable half.
// ---------------------------------------------------------------------------
TEST_F(CApiRaypathAnalysis, RenderInProgressIsServerError) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("\"infinite\"")), LUMICE_OK);
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_RUNNING);
  const LUMICE_RaypathAnalysisRequest req = FullSky();
  EXPECT_EQ(StartAnalysis(server_, Halo22Json("\"infinite\""), &req), LUMICE_ERR_SERVER);
  EXPECT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_RUNNING) << "the render was not interrupted";
  EXPECT_EQ(ActiveBackend(server_), LUMICE_BACKEND_CPU);

  LUMICE_StopServer(server_);
  EXPECT_EQ(StartAnalysis(server_, Halo22Json("\"infinite\""), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  EXPECT_EQ(ActiveBackend(server_), LUMICE_BACKEND_CPU);
  int fell_back = 1;
  EXPECT_EQ(LUMICE_GetBackendFallbackFlag(server_, &fell_back), LUMICE_OK);
  EXPECT_EQ(fell_back, 0);

  // The other direction: a commit over the running analysis.
  EXPECT_EQ(CommitJson(server_, Halo22Json("1000")), LUMICE_ERR_SERVER);
  EXPECT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_RUNNING) << "the analysis was not interrupted";
  LUMICE_StopServer(server_);
  EXPECT_EQ(CommitJson(server_, Halo22Json("1000")), LUMICE_OK);
}

// ---------------------------------------------------------------------------
// The frame getters, end to end on the 22° halo: the histogram is there, sorted, its top
// chain is the 22° path through crystal 1, the sentinel lands where the contract says, and
// the render / raw-XYZ getters have nothing to give. Then a render commit: the histogram
// is gone from its frames.
// ---------------------------------------------------------------------------
TEST_F(CApiRaypathAnalysis, FrameGettersEndToEnd) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("40000")), LUMICE_OK);
  LUMICE_StopServer(server_);
  const LUMICE_RaypathAnalysisRequest req = FullSky();
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("40000"), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_NE(frame, nullptr);

  LUMICE_RaypathAnalysisInfo info{};
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(nullptr, kSymAll, &info), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, nullptr), LUMICE_ERR_NULL_ARG);
  // The read's symmetry is a bit set 0..7 and nothing else; a bad one writes nothing.
  std::memset(&info, 0x5A, sizeof(info));
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, 8, &info), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, -1, &info), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(info.present, 0x5A5A5A5A) << "nothing written on a rejected symmetry";
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1);
  EXPECT_EQ(info.roi_mode, LUMICE_RAYPATH_ROI_FULL_SKY);
  ASSERT_GE(info.entry_count, 2);
  EXPECT_EQ(info.cone_ring_count, 0);

  // One more slot than entries: the sentinel lands at [entry_count].
  std::vector<LUMICE_RaypathHistogramEntry> entries(static_cast<size_t>(info.entry_count) + 1);
  std::memset(entries.data(), 0x5A, entries.size() * sizeof(LUMICE_RaypathHistogramEntry));  // not zero
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysis(nullptr, kSymAll, entries.data(), 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, nullptr, 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, 8, entries.data(), 1), LUMICE_ERR_INVALID_VALUE);
  EXPECT_EQ(entries[0].count, 0x5A5A5A5A5A5A5A5Au) << "nothing written on a rejected symmetry";
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, entries.data(), info.entry_count + 1), LUMICE_OK);
  EXPECT_EQ(entries.back().count, 0u) << "sentinel at [entry_count]";
  LUMICE_RayCount total = 0;
  for (int i = 0; i < info.entry_count; i++) {
    const auto& e = entries[static_cast<size_t>(i)];
    EXPECT_GT(e.count, 0u) << "entry " << i;
    EXPECT_GT(e.energy, 0.0) << "entry " << i;
    EXPECT_EQ(e.chain_len, 1) << "MS=1: one layer per chain";
    EXPECT_EQ(e.chain[0].crystal_id, 1);
    EXPECT_GE(e.chain[0].segment_len, 1);
    EXPECT_EQ(e.ring_count, 0);
    EXPECT_NE(std::memchr(e.display, 0, sizeof(e.display)), nullptr) << "NUL-terminated";
    // One crystal on one layer: the bare face sequence, no "C1(" prefix and no parentheses.
    EXPECT_EQ(std::string(e.display).find_first_not_of("0123456789-"), std::string::npos) << e.display;
    if (i > 0) {
      EXPECT_GE(entries[static_cast<size_t>(i - 1)].energy, e.energy) << "energy descending at " << i;
    }
    total += e.count;
  }
  // The 22° path — faces 3 -> 5 of the prism, which the read's P|B|D reduction leaves as the
  // canonical "3-5" (the same literal test_raypath_histogram_consumer.cpp derives through
  // Crystal::ReduceRaypath; pinned as text here because the text IS this API's contract).
  EXPECT_STREQ(entries[0].display, "3-5");
  EXPECT_EQ(entries[0].chain[0].segment_len, 2);
  EXPECT_EQ(entries[0].chain[0].segment[0], 3);
  EXPECT_EQ(entries[0].chain[0].segment[1], 5);

  // The same frame under the other three symmetries a reader can ask for: Σ count is the run's
  // whole output at each, the row count never grows as bits are added, and at the finest the
  // 22° path is spread over its orbit — "3-5" and its one-face rotation "4-6" are both rows —
  // which is the positive control that the read, not the run, does the reducing.
  LUMICE_RayCount pbd_total = total;
  int prev_rows = -1;
  for (const int sym :
       { 0, LUMICE_RAYPATH_SYMMETRY_P, LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B, kSymAll }) {
    LUMICE_RaypathAnalysisInfo isym{};
    if (LUMICE_FrameGetRaypathAnalysisInfo(frame, sym, &isym) != LUMICE_OK) {
      ADD_FAILURE() << "symmetry " << sym << ": the info read failed";
      continue;
    }
    EXPECT_EQ(isym.present, 1);
    EXPECT_EQ(isym.snapshot_generation, info.snapshot_generation) << "the symmetry is not a new result";
    // The bucket is the record's, the same under every symmetry; the max row error is this
    // symmetry's — never below the finest one, since merging adds errors.
    EXPECT_EQ(isym.other_count, info.other_count) << "symmetry " << sym;
    EXPECT_DOUBLE_EQ(isym.other_energy, info.other_energy) << "symmetry " << sym;
    EXPECT_EQ(isym.truncated_chain_count, info.truncated_chain_count) << "symmetry " << sym;
    EXPECT_GE(isym.max_row_error, 0.0) << "symmetry " << sym;
    std::vector<LUMICE_RaypathHistogramEntry> rows(static_cast<size_t>(isym.entry_count) + 1);
    if (LUMICE_FrameGetRaypathAnalysis(frame, sym, rows.data(), isym.entry_count + 1) != LUMICE_OK) {
      ADD_FAILURE() << "symmetry " << sym << ": the entry read failed";
      continue;
    }
    EXPECT_EQ(rows.back().count, 0u) << "entry_count under symmetry " << sym << " sizes the read exactly";
    LUMICE_RayCount t = 0;
    bool saw_35 = false;
    bool saw_46 = false;
    double max_err = 0.0;
    for (int i = 0; i < isym.entry_count; i++) {
      t += rows[static_cast<size_t>(i)].count;
      saw_35 = saw_35 || std::strcmp(rows[static_cast<size_t>(i)].display, "3-5") == 0;
      saw_46 = saw_46 || std::strcmp(rows[static_cast<size_t>(i)].display, "4-6") == 0;
      EXPECT_GE(rows[static_cast<size_t>(i)].error_bound, 0.0);
      EXPECT_LE(rows[static_cast<size_t>(i)].error_bound, rows[static_cast<size_t>(i)].energy)
          << "a row is never uncertain by more than it holds";
      max_err = std::max(max_err, rows[static_cast<size_t>(i)].error_bound);
    }
    EXPECT_EQ(t, pbd_total) << "symmetry " << sym;
    EXPECT_DOUBLE_EQ(isym.max_row_error, max_err) << "symmetry " << sym << ": the info's max is over these rows";
    if (prev_rows >= 0) {
      EXPECT_LE(isym.entry_count, prev_rows) << "symmetry " << sym;
    }
    prev_rows = isym.entry_count;
    EXPECT_TRUE(saw_35) << "symmetry " << sym;
    EXPECT_EQ(saw_46, sym == 0) << "symmetry " << sym << ": the P-image is its own row only unreduced";
  }
  EXPECT_EQ(prev_rows, info.entry_count) << "the loop's last symmetry is the one `info` was read under";

  // A short read fills exactly max_count and, the array being full, writes NO sentinel —
  // the slot past it is the caller's and stays as it was.
  LUMICE_RaypathHistogramEntry two[3];
  std::memset(two, 0x5A, sizeof(two));
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, two, 2), LUMICE_OK);
  EXPECT_STREQ(two[0].display, entries[0].display);
  EXPECT_STREQ(two[1].display, entries[1].display);
  EXPECT_EQ(two[2].count, 0x5A5A5A5A5A5A5A5Au) << "no sentinel written past a full array";
  LUMICE_RaypathHistogramEntry one[2];
  std::memset(one, 0x5A, sizeof(one));

  // The stats rode along, and every counted ray is one the run traced.
  LUMICE_StatsResult stats{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 40000u);
  EXPECT_LE(total + info.other_count, stats.sim_ray_num * 8u)
      << "max_hits = 7 bounds the outgoing rays per input ray, rows and bucket together";

  // The image getters on an analysis frame: sentinel at [0], not a previous render's residue.
  LUMICE_RawXyzResult xyz[2];
  std::memset(xyz, 0x5A, sizeof(xyz));
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz, 1), LUMICE_OK);
  EXPECT_EQ(xyz[0].xyz_buffer, nullptr);
  LUMICE_RenderResult img[2];
  std::memset(img, 0x5A, sizeof(img));
  ASSERT_EQ(LUMICE_FrameGetRender(frame, img, 1), LUMICE_OK);
  EXPECT_EQ(img[0].img_buffer, nullptr);
  LUMICE_ReleaseResultFrame(frame);

  // Back to a render: its frame carries no histogram.
  ASSERT_EQ(CommitJson(server_, Halo22Json("2000")), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 0);
  EXPECT_EQ(info.entry_count, 0);
  EXPECT_EQ(info.snapshot_generation, 0u) << "every field is 0 when present == 0";
  EXPECT_DOUBLE_EQ(info.other_energy, 0.0);
  EXPECT_EQ(info.other_count, 0u);
  EXPECT_EQ(info.truncated_chain_count, 0);
  EXPECT_DOUBLE_EQ(info.max_row_error, 0.0);
  std::memset(one, 0x5A, sizeof(one));
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, one, 1), LUMICE_OK);
  EXPECT_EQ(one[0].count, 0u) << "sentinel at [0] on a render frame";
  std::memset(xyz, 0x5A, sizeof(xyz));
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz, 1), LUMICE_OK);
  EXPECT_NE(xyz[0].xyz_buffer, nullptr) << "and the render's own image is back";
  LUMICE_ReleaseResultFrame(frame);
}

// snapshot_generation (v4.30) is the consumer's "new result?" signal, so pin the two halves of its
// contract: two acquires of one published frame read the same non-zero value (a GUI observing the
// same result over many polls must NOT see it change — that is what would make it clear a
// selection every frame), and the next snapshot the server takes reads a strictly larger one.
TEST_F(CApiRaypathAnalysis, InfoSnapshotGenerationIsStableWithinAFrameAndGrowsAcrossSnapshots) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("40000")), LUMICE_OK);
  LUMICE_StopServer(server_);
  const LUMICE_RaypathAnalysisRequest req = FullSky();
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("40000"), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));

  LUMICE_ResultFrame* a = nullptr;
  LUMICE_ResultFrame* b = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &a), LUMICE_OK);
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &b), LUMICE_OK);
  LUMICE_RaypathAnalysisInfo ia{};
  LUMICE_RaypathAnalysisInfo ib{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(a, kSymAll, &ia), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(b, kSymAll, &ib), LUMICE_OK);
  EXPECT_EQ(ia.present, 1);
  EXPECT_NE(ia.snapshot_generation, 0u);
  EXPECT_EQ(ia.snapshot_generation, ib.snapshot_generation) << "same published frame, same counter";
  LUMICE_ReleaseResultFrame(a);
  LUMICE_ReleaseResultFrame(b);

  // A second analysis session publishes at least one further snapshot; whatever the count, the
  // counter it carries is past the one above.
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("40000"), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &a), LUMICE_OK);
  LUMICE_RaypathAnalysisInfo ic{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(a, kSymAll, &ic), LUMICE_OK);
  EXPECT_EQ(ic.present, 1);
  EXPECT_GT(ic.snapshot_generation, ia.snapshot_generation);
  LUMICE_ReleaseResultFrame(a);
}

// ---------------------------------------------------------------------------
// The request's own ray budget (v4.32). Two ingest paths, one committed scene at 40000 rays:
// a request carrying 10000 traces 10000; a request carrying the sentinel traces the scene's
// 40000, as every request did before the field existed. The counts are EXACT, not
// approximate: the CPU route queues batches of min(cap, remaining) until the budget is met
// (server.cpp GenerateScene), and the D65 illuminant is a single wavelength, so the total
// traced is the total asked for with no rounding — the same equality FrameGettersEndToEnd
// already pins at 40000. So "proportional" is 10000 : 40000 to the ray, not to a tolerance.
// ---------------------------------------------------------------------------
TEST_F(CApiRaypathAnalysis, RequestRayBudgetOverridesTheSceneAndTheSentinelKeepsIt) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("40000")), LUMICE_OK);
  LUMICE_StopServer(server_);

  LUMICE_RaypathAnalysisRequest own = FullSky();
  own.infinite = 0;
  own.ray_num = 10000;
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("40000"), &own), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  LUMICE_StatsResult stats{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 10000u) << "the request's budget, not the scene's";
  LUMICE_RaypathAnalysisInfo info{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1) << "a smaller budget is still a full analysis frame";
  LUMICE_ReleaseResultFrame(frame);

  // Same server, same scene, the sentinel: the scene's own 40000 — so the override did not
  // write itself into the scene, and a session without one does not inherit the last one.
  const LUMICE_RaypathAnalysisRequest scene_default = FullSky();
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("40000"), &scene_default), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 40000u) << "the scene's budget, untouched by the previous request";
  LUMICE_ReleaseResultFrame(frame);

  // And the render that follows traces the document's budget: the analysis edited nothing.
  ASSERT_EQ(CommitJson(server_, Halo22Json("2000")), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 2000u);
  LUMICE_ReleaseResultFrame(frame);
}

// The other direction of the same field: a FINITE scene, a request for an unlimited run. The
// only thing that can end it is LUMICE_StopServer (v4.34: there is no cone stop target), and the
// proof the budget was the request's rather than the scene's is that more rays were traced than
// the scene's 1000 allow — a run on the scene's budget stops at exactly 1000. The same sequence
// is the C API's statement of the v4.34 stop contract: the frame acquired after LUMICE_StopServer
// returns carries the histogram consumed up to the stop, not the empty frame the session opened
// with, and the run then reads as idle (a stop is a reset, not a completion).
TEST_F(CApiRaypathAnalysis, RequestInfiniteBudgetOutlivesAFiniteSceneUntilStopped) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("1000")), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));
  LUMICE_RaypathAnalysisRequest req = Cone();
  const float lat = -43.0f * 3.14159265f / 180.0f;  // on the 22° ring above the sun
  req.cone_center[0] = -std::cos(lat);
  req.cone_center[1] = 0.0f;
  req.cone_center[2] = std::sin(lat);
  req.cone_radius_rad = 2.5f * 3.14159265f / 180.0f;
  req.cone_ring_count = 5;
  req.infinite = 1;
  req.ray_num = 7;  // ignored under infinite, and would be a smaller budget still if it were read
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("1000"), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForRayCountAbove(server_, 1000, 30000)) << "an unlimited run must outlive the scene's 1000";
  LUMICE_StopServer(server_);
  EXPECT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_IDLE) << "a stop is a reset, not a completion";
  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  LUMICE_StatsResult stats{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_GT(stats.sim_ray_num, 1000u) << "past the scene's budget: the run was the request's";
  LUMICE_RaypathAnalysisInfo info{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1) << "the stopped run's frame is still an analysis frame";
  ASSERT_GE(info.entry_count, 1) << "the histogram accumulated up to the stop must be readable";
  std::vector<LUMICE_RaypathHistogramEntry> entries(static_cast<size_t>(info.entry_count) + 1);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, entries.data(), info.entry_count + 1), LUMICE_OK);
  LUMICE_RayCount landed = 0;
  for (int i = 0; i < info.entry_count; i++) {
    landed += entries[static_cast<size_t>(i)].count;
  }
  EXPECT_GT(landed, 0u) << "rays landed in the cone before the stop are in the frame";
  LUMICE_ReleaseResultFrame(frame);
}

// The cone request round-trips its echo fields and ring split through the getters. The scene is
// unlimited and the request carries its own finite budget, so the run ends on that budget alone
// (v4.34: a cone has no stop of its own).
TEST_F(CApiRaypathAnalysis, ConeEchoAndRings) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("\"infinite\"")), LUMICE_OK);
  LUMICE_StopServer(server_);
  LUMICE_RaypathAnalysisRequest req = Cone();
  // Straight above the sun on the 22° ring, as the C++ test does: altitude 43°, azimuth 0 —
  // the direction light travels is the antipode of where the sun sits.
  const float lat = -43.0f * 3.14159265f / 180.0f;
  req.cone_center[0] = -std::cos(lat);
  req.cone_center[1] = 0.0f;
  req.cone_center[2] = std::sin(lat);
  req.cone_radius_rad = 2.5f * 3.14159265f / 180.0f;
  req.cone_ring_count = 5;
  req.infinite = 0;
  req.ray_num = 40000;
  ASSERT_EQ(StartAnalysis(server_, Halo22Json("\"infinite\""), &req), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000)) << "the request's own budget must end the run";

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  LUMICE_RaypathAnalysisInfo info{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, kSymAll, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1);
  EXPECT_EQ(info.roi_mode, LUMICE_RAYPATH_ROI_CONE);
  EXPECT_EQ(info.cone_ring_count, 5);
  EXPECT_FLOAT_EQ(info.cone_radius_rad, req.cone_radius_rad);
  ASSERT_GE(info.entry_count, 1);
  std::vector<LUMICE_RaypathHistogramEntry> entries(static_cast<size_t>(info.entry_count) + 1);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, kSymAll, entries.data(), info.entry_count + 1), LUMICE_OK);
  LUMICE_RayCount total = 0;
  for (int i = 0; i < info.entry_count; i++) {
    const auto& e = entries[static_cast<size_t>(i)];
    if (e.ring_count != 5) {
      ADD_FAILURE() << e.display << ": " << e.ring_count << " rings, expected 5";
      continue;
    }
    double s = 0.0;
    for (int r = 0; r < e.ring_count; r++) {
      s += e.ring_energy[r];
    }
    EXPECT_NEAR(s, e.energy, 1e-12 * e.energy) << e.display;
    total += e.count;
  }
  // A 2.5° cone on the halo's brightest arc gathers a few hundred of 40000 rays (262 measured
  // on the reference run); 100 is the floor, so the ring split above is exercised on real rows.
  EXPECT_GE(total, 100u);
  EXPECT_STREQ(entries[0].display, "3-5") << "the 22° path leads in the cone";
  LUMICE_ReleaseResultFrame(frame);
}

// ---------------------------------------------------------------------------
// Plan risk 4: a chain past the C struct's caps is truncated into a self-consistent prefix,
// never written past the arrays, and the scalar fields survive intact.
// ---------------------------------------------------------------------------
TEST(CApiRaypathAnalysisTruncation, DeepChainIsTruncatedNotOverrun) {
  auto frame = std::make_shared<lumice::ResultFrame>();
  lumice::RaypathHistogramResult r;
  lumice::RaypathHistogramEntry deep;
  for (int l = 0; l < LUMICE_MAX_RAYPATH_CHAIN_LAYERS + 3; l++) {
    lumice::RaypathChainSegment seg;
    seg.crystal_id = static_cast<lumice::IdType>(l + 1);
    for (int f = 0; f < LUMICE_MAX_RAYPATH_SEGMENT_LEN + 5; f++) {
      // Five-digit "faces" (nothing reduces them: no crystal is described, the read is at
      // symmetry 0), so the display text the read formats from this chain also overruns
      // LUMICE_RAYPATH_DISPLAY_MAX — 11 layers of 69 five-digit faces is ~4.6k characters.
      seg.segment.push_back(static_cast<lumice::IdType>(60000 + f + 1));
    }
    deep.chain_.push_back(seg);
  }
  deep.energy_ = 2.5;
  deep.count_ = 7;
  deep.error_bound_ = 0.75;
  r.entries_.push_back(deep);
  r.other_energy_ = 1.25;
  r.other_count_ = 3;
  r.truncated_chain_count_ = 11;
  // Every layer multi-crystal: the "C<id>(" prefix is part of the longest-text derivation.
  r.reduce_ctx_.layer_multi_crystal_.assign(deep.chain_.size(), true);
  frame->raypath_histogram_result_ = r;
  ASSERT_GT(lumice::FormatRaypathChainDisplay(deep.chain_, r.reduce_ctx_.layer_multi_crystal_).size(),
            static_cast<size_t>(LUMICE_RAYPATH_DISPLAY_MAX))
      << "fixture premise: the formatted text must exceed the cap for the truncation to be exercised";

  LUMICE_ResultFrame* handle = WrapResultFrameForTest(frame);
  // Guard bytes around the one entry: a write past it is a test failure, not silent UB.
  struct Guarded {
    unsigned char before[64];
    LUMICE_RaypathHistogramEntry entry;
    unsigned char after[64];
  } g;
  std::memset(&g, 0xA5, sizeof(g));
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(handle, 0, &g.entry, 1), LUMICE_OK);
  LUMICE_ReleaseResultFrame(handle);

  EXPECT_EQ(std::count(std::begin(g.before), std::end(g.before), 0xA5), static_cast<long>(sizeof(g.before)))
      << "written before the entry";
  EXPECT_EQ(std::count(std::begin(g.after), std::end(g.after), 0xA5), static_cast<long>(sizeof(g.after)))
      << "written past the entry";
  EXPECT_EQ(g.entry.chain_len, LUMICE_MAX_RAYPATH_CHAIN_LAYERS);
  for (int l = 0; l < LUMICE_MAX_RAYPATH_CHAIN_LAYERS; l++) {
    EXPECT_EQ(g.entry.chain[l].crystal_id, l + 1);
    EXPECT_EQ(g.entry.chain[l].segment_len, LUMICE_MAX_RAYPATH_SEGMENT_LEN);
    for (int f = 0; f < LUMICE_MAX_RAYPATH_SEGMENT_LEN; f++) {
      EXPECT_EQ(g.entry.chain[l].segment[f], 60000 + f + 1);
    }
  }
  EXPECT_EQ(std::strlen(g.entry.display), static_cast<size_t>(LUMICE_RAYPATH_DISPLAY_MAX - 1));
  EXPECT_EQ(g.entry.display[LUMICE_RAYPATH_DISPLAY_MAX - 1], '\0');
  EXPECT_EQ(std::string(g.entry.display).rfind("C1(60001-60002-", 0), 0u) << "a prefix of the formatted text";
  EXPECT_DOUBLE_EQ(g.entry.energy, 2.5);
  EXPECT_EQ(g.entry.count, 7u);
  EXPECT_EQ(g.entry.ring_count, 0);
  EXPECT_DOUBLE_EQ(g.entry.error_bound, 0.75);
  // And the info's v4.35 fields come off the same hand-built record.
  LUMICE_ResultFrame* again = WrapResultFrameForTest(frame);
  LUMICE_RaypathAnalysisInfo info{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(again, 0, &info), LUMICE_OK);
  LUMICE_ReleaseResultFrame(again);
  EXPECT_EQ(info.present, 1);
  EXPECT_DOUBLE_EQ(info.other_energy, 1.25);
  EXPECT_EQ(info.other_count, 3u);
  EXPECT_EQ(info.truncated_chain_count, 11);
  EXPECT_DOUBLE_EQ(info.max_row_error, 0.75) << "one row, its error is the max";
}

// ---------------------------------------------------------------------------
// AC6: forward(inverse(px, py)) == (px, py) on every lens branch. The forward is
// lm_proj::ProjectExitToPixel through the same ProjParams the inverse is built from (the
// one production forward projection, called directly rather than through the C API). The
// equality is EXACT in integer pixels: the inverse returns the pixel's centre, half a pixel
// from any rounding boundary of the forward's floor, and the round trip's floating-point
// error is orders of magnitude below that — a ±1 slip would be a real disagreement between
// the two, not tolerance.
// ---------------------------------------------------------------------------
struct LensCase {
  int lens_type;
  float fov;
  const char* name;
};

const LensCase kLensCases[] = {
  { LUMICE_LENS_TYPE_LINEAR, 60.0f, "linear" },
  { LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, 150.0f, "fisheye_equal_area" },
  { LUMICE_LENS_TYPE_FISHEYE_EQUIDISTANT, 150.0f, "fisheye_equidistant" },
  { LUMICE_LENS_TYPE_FISHEYE_STEREOGRAPHIC, 150.0f, "fisheye_stereographic" },
  { LUMICE_LENS_TYPE_FISHEYE_ORTHOGRAPHIC, 150.0f, "fisheye_orthographic" },
  { LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA, 180.0f, "dual_fisheye_equal_area" },
  { LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUIDISTANT, 180.0f, "dual_fisheye_equidistant" },
  { LUMICE_LENS_TYPE_DUAL_FISHEYE_STEREOGRAPHIC, 180.0f, "dual_fisheye_stereographic" },
  { LUMICE_LENS_TYPE_DUAL_FISHEYE_ORTHOGRAPHIC, 180.0f, "dual_fisheye_orthographic" },
  { LUMICE_LENS_TYPE_RECTANGULAR, 360.0f, "rectangular" },
  // Globe is a perspective view of the sphere from outside (focal = r / tan(fov/2), like
  // linear), so its fov is a camera fov, not a sky fov: 180 would put the focal at infinity.
  { LUMICE_LENS_TYPE_GLOBE, 60.0f, "globe" },
};

LUMICE_AnnotationView ViewFor(const LensCase& c) {
  LUMICE_AnnotationView v{};
  v.width = 256;
  v.height = 192;
  v.lens_type = c.lens_type;
  v.lens_fov = c.fov;
  v.view_azimuth = 40.0f;
  v.view_elevation = 25.0f;
  v.view_roll = 10.0f;
  v.visible = LUMICE_VISIBLE_FULL;
  v.front = 0;
  return v;
}

TEST(CApiUnprojectPixel, InvertsTheForwardProjectionOnEveryLens) {
  for (const LensCase& c : kLensCases) {
    SCOPED_TRACE(c.name);
    const LUMICE_AnnotationView v = ViewFor(c);
    const lumice::RenderConfig cfg = lumice::annotation::ToRenderConfig(ToAnnotationViewSnapshot(v));
    const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
    const auto params = lumice::BuildProjParams(cfg, rot, static_cast<float>(std::min(v.width, v.height)));

    int valid_pixels = 0;
    int mismatches = 0;
    for (int py = 0; py < v.height; py += 7) {
      for (int px = 0; px < v.width; px += 5) {
        float dir[3] = { 0.0f, 0.0f, 0.0f };
        int valid = -1;
        if (LUMICE_UnprojectPixel(&v, px, py, dir, &valid) != LUMICE_OK) {
          ADD_FAILURE() << "(" << px << "," << py << ") returned an error";
          continue;
        }
        if (valid == 0) {
          continue;
        }
        valid_pixels++;
        const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        EXPECT_NEAR(len, 1.0f, 1e-5f) << "(" << px << "," << py << ") not a unit vector";
        const lm_proj::ProjResult hit = lm_proj::ProjectExitToPixel(params, dir[0], dir[1], dir[2]);
        bool matched = false;
        for (int h = 0; h < hit.count; h++) {
          if (hit.hits[h].px == px && hit.hits[h].py == py) {
            matched = true;
          }
        }
        if (!matched) {
          mismatches++;
          if (mismatches <= 3) {
            ADD_FAILURE() << "(" << px << "," << py << ") -> dir (" << dir[0] << "," << dir[1] << "," << dir[2]
                          << ") -> " << (hit.count > 0 ? hit.hits[0].px : -1) << ","
                          << (hit.count > 0 ? hit.hits[0].py : -1) << " (" << hit.count << " hits)";
          }
        }
      }
    }
    EXPECT_GT(valid_pixels, 100) << "the sample must actually cover the lens";
    EXPECT_EQ(mismatches, 0) << "of " << valid_pixels << " valid pixels";
  }
}

TEST(CApiUnprojectPixel, ArgumentsAndClips) {
  LUMICE_AnnotationView v = ViewFor(kLensCases[1]);
  float dir[3];
  int valid = -1;
  EXPECT_EQ(LUMICE_UnprojectPixel(nullptr, 0, 0, dir, &valid), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, 0, 0, nullptr, &valid), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, 0, 0, dir, nullptr), LUMICE_ERR_NULL_ARG);
  v.lens_type = -1;
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, 0, 0, dir, &valid), LUMICE_ERR_INVALID_VALUE);
  v = ViewFor(kLensCases[1]);
  v.width = 0;
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, 0, 0, dir, &valid), LUMICE_ERR_INVALID_VALUE);

  // Outside the canvas, and outside a fisheye's image circle: valid 0, out_dir untouched.
  v = ViewFor(kLensCases[1]);
  dir[0] = dir[1] = dir[2] = 42.0f;
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, -1, 0, dir, &valid), LUMICE_OK);
  EXPECT_EQ(valid, 0);
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, v.width, 0, dir, &valid), LUMICE_OK);
  EXPECT_EQ(valid, 0);
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, 0, 0, dir, &valid), LUMICE_OK) << "the corner is outside the circle";
  EXPECT_EQ(valid, 0);
  EXPECT_FLOAT_EQ(dir[0], 42.0f);

  // The centre pixel images sky; under `visible: lower` the same direction (above the
  // horizon at elevation 25) is clipped, and under `front` it is not (it is in front).
  const int cx = v.width / 2;
  const int cy = v.height / 2;
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, cx, cy, dir, &valid), LUMICE_OK);
  ASSERT_EQ(valid, 1);
  EXPECT_LT(dir[2], 0.0f) << "looking up at elevation 25: the direction light travels has z < 0";
  v.visible = LUMICE_VISIBLE_LOWER;
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, cx, cy, dir, &valid), LUMICE_OK);
  EXPECT_EQ(valid, 0) << "`lower` keeps only z > 0";
  v.visible = LUMICE_VISIBLE_UPPER;
  v.front = 1;
  EXPECT_EQ(LUMICE_UnprojectPixel(&v, cx, cy, dir, &valid), LUMICE_OK);
  EXPECT_EQ(valid, 1);
}

// ---------------------------------------------------------------------------
// LUMICE_ProjectDirection (v4.31): the forward on a caller's direction. Its geometry is pinned in
// core (test_annotation_overlay.cpp, ProjectDirectionOnView against every named marker); the bridge
// is checked here for its argument contract, for the round trip through LUMICE_UnprojectPixel on
// every lens branch, and for reporting a marker-policy miss as valid 0 with the outputs untouched.
// ---------------------------------------------------------------------------
TEST(CApiProjectDirection, RoundTripsUnprojectPixelOnEveryLens) {
  for (const LensCase& c : kLensCases) {
    SCOPED_TRACE(c.name);
    const LUMICE_AnnotationView v = ViewFor(c);
    int valid_pixels = 0;
    for (int py = 0; py < v.height; py += 7) {
      for (int px = 0; px < v.width; px += 5) {
        float dir[3] = { 0.0f, 0.0f, 0.0f };
        int valid = -1;
        if (LUMICE_UnprojectPixel(&v, px, py, dir, &valid) != LUMICE_OK) {
          ADD_FAILURE() << "(" << px << "," << py << ") inverse returned an error";
          continue;
        }
        if (valid == 0) {
          continue;
        }
        valid_pixels++;
        float fx = -1.0f;
        float fy = -1.0f;
        int fvalid = -1;
        if (LUMICE_ProjectDirection(&v, dir, &fx, &fy, &fvalid) != LUMICE_OK) {
          ADD_FAILURE() << "(" << px << "," << py << ") forward returned an error";
          continue;
        }
        // The marker policy is a superset of the render-domain one (half a degree of slack at the
        // hemisphere edge), so a direction the inverse called sky the forward must place.
        EXPECT_EQ(fvalid, 1) << "(" << px << "," << py << ")";
        // The forward bins with floor(v + 0.5) about the centre and the inverse returns the pixel's
        // centre: the landing point is the pixel index itself, to float noise.
        EXPECT_NEAR(fx, static_cast<float>(px), 1e-3f) << "(" << px << "," << py << ")";
        EXPECT_NEAR(fy, static_cast<float>(py), 1e-3f) << "(" << px << "," << py << ")";
      }
    }
    EXPECT_GT(valid_pixels, 100) << "the sample must actually cover the lens";
  }
}

TEST(CApiProjectDirection, ArgumentsAndPolicy) {
  LUMICE_AnnotationView v = ViewFor(kLensCases[1]);
  const float zenith[3] = { 0.0f, 0.0f, -1.0f };
  float px = 0.0f;
  float py = 0.0f;
  int valid = -1;
  EXPECT_EQ(LUMICE_ProjectDirection(nullptr, zenith, &px, &py, &valid), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ProjectDirection(&v, nullptr, &px, &py, &valid), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, nullptr, &py, &valid), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, nullptr, &valid), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, nullptr), LUMICE_ERR_NULL_ARG);
  v.lens_type = -1;
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, &valid), LUMICE_ERR_INVALID_VALUE);
  v = ViewFor(kLensCases[1]);
  v.visible = 99;
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, &valid), LUMICE_ERR_INVALID_VALUE);
  v = ViewFor(kLensCases[1]);
  v.height = 0;
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, &valid), LUMICE_ERR_INVALID_VALUE);

  // A 150-degree fisheye at elevation 25 images the zenith (65 degrees off axis); `lower` hides
  // it under the marker policy, and the outputs are untouched on the miss.
  v = ViewFor(kLensCases[1]);
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, &valid), LUMICE_OK);
  ASSERT_EQ(valid, 1);
  EXPECT_GE(px, 0.0f);
  EXPECT_LE(px, static_cast<float>(v.width - 1));
  EXPECT_GE(py, 0.0f);
  EXPECT_LE(py, static_cast<float>(v.height - 1));
  v.visible = LUMICE_VISIBLE_LOWER;
  px = py = 42.0f;
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, &valid), LUMICE_OK);
  EXPECT_EQ(valid, 0);
  EXPECT_FLOAT_EQ(px, 42.0f);
  EXPECT_FLOAT_EQ(py, 42.0f);
  // A narrow linear lens on the horizon does not image the zenith at all.
  v = ViewFor(kLensCases[0]);
  v.view_elevation = 0.0f;
  EXPECT_EQ(LUMICE_ProjectDirection(&v, zenith, &px, &py, &valid), LUMICE_OK);
  EXPECT_EQ(valid, 0);
}

}  // namespace
