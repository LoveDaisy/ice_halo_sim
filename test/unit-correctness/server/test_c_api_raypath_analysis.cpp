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
#include "server/c_api_internal.hpp"  // ToAnnotationViewSnapshot, WrapResultFrameForTest
#include "server/server.hpp"

static_assert(LUMICE_API_VERSION >= 429, "the analysis run needs the v4.29 header");

// The layout the ctypes mirrors in test/e2e/capi_runner.py are written against. Sizes AND
// offsets, so a field inserted in the middle (which keeps the size) is caught as well as
// one appended; the Python side pins the same numbers, so either side moving turns one of
// the two red before the library writes past a Python buffer.
static_assert(sizeof(LUMICE_AnnotationView) == 48, "LUMICE_AnnotationView layout changed; update capi_runner.py");
static_assert(sizeof(LUMICE_RaypathAnalysisRequest) == 88, "LUMICE_RaypathAnalysisRequest layout changed");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, frame_view) == 4, "");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, cone_center) == 52, "");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, cone_stop_target) == 72, "");
static_assert(offsetof(LUMICE_RaypathAnalysisRequest, chain_id_symmetry) == 80, "");
static_assert(sizeof(LUMICE_RaypathChainSegment) == 264, "LUMICE_RaypathChainSegment layout changed");
static_assert(sizeof(LUMICE_RaypathHistogramEntry) == 5600, "LUMICE_RaypathHistogramEntry layout changed");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, chain_len) == 2112, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, display) == 2116, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, energy) == 5320, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, count) == 5328, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, ring_energy) == 5336, "");
static_assert(offsetof(LUMICE_RaypathHistogramEntry, ring_count) == 5592, "");
static_assert(sizeof(LUMICE_RaypathAnalysisInfo) == 20, "LUMICE_RaypathAnalysisInfo layout changed");

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

LUMICE_RaypathAnalysisRequest FullSky() {
  LUMICE_RaypathAnalysisRequest req{};
  req.roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  req.chain_id_symmetry = LUMICE_RAYPATH_SYMMETRY_SESSION_DEFAULT;
  return req;
}

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

bool WaitForFirstRays(LUMICE_Server* server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    LUMICE_RayCount n = 0;
    EXPECT_EQ(LUMICE_GetSimRayCount(server, &n), LUMICE_OK);
    if (n > 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
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
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(nullptr, &req), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, nullptr), LUMICE_ERR_NULL_ARG);

  // Nothing committed yet: the request is fine, the server has no scene.
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_CONFIG);

  req.roi_mode = 7;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);

  req = FullSky();
  req.chain_id_symmetry = 8;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
  req.chain_id_symmetry = -1;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);

  req = Cone();
  req.cone_radius_rad = 0.0f;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
  req = Cone();
  req.cone_center[2] = 0.0f;  // zero vector
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
  req = Cone();
  req.cone_ring_count = 0;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
  req = Cone();
  req.cone_ring_count = LUMICE_MAX_RAYPATH_CONE_RINGS + 1;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);

  req = FullSky();
  req.roi_mode = LUMICE_RAYPATH_ROI_IN_FRAME;
  req.frame_view.width = 64;
  req.frame_view.height = 64;
  req.frame_view.lens_type = LUMICE_LENS_TYPE_GLOBE + 1;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
  req.frame_view.lens_type = LUMICE_LENS_TYPE_LINEAR;
  req.frame_view.visible = 3;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
  req.frame_view.visible = LUMICE_VISIBLE_FULL;
  req.frame_view.width = 0;
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_INVALID_VALUE);
}

// ---------------------------------------------------------------------------
// AC1 through the C API, both directions, plus AC2's readable half.
// ---------------------------------------------------------------------------
TEST_F(CApiRaypathAnalysis, RenderInProgressIsServerError) {
  ASSERT_EQ(CommitJson(server_, Halo22Json("\"infinite\"")), LUMICE_OK);
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_RUNNING);
  const LUMICE_RaypathAnalysisRequest req = FullSky();
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_ERR_SERVER);
  EXPECT_EQ(Lifecycle(server_), LUMICE_LIFECYCLE_RUNNING) << "the render was not interrupted";
  EXPECT_EQ(ActiveBackend(server_), LUMICE_BACKEND_CPU);

  LUMICE_StopServer(server_);
  EXPECT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_OK);
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
  ASSERT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000));

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  ASSERT_NE(frame, nullptr);

  LUMICE_RaypathAnalysisInfo info{};
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(nullptr, &info), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, nullptr), LUMICE_ERR_NULL_ARG);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1);
  EXPECT_EQ(info.roi_mode, LUMICE_RAYPATH_ROI_FULL_SKY);
  ASSERT_GE(info.entry_count, 2);
  EXPECT_EQ(info.cone_ring_count, 0);

  // One more slot than entries: the sentinel lands at [entry_count].
  std::vector<LUMICE_RaypathHistogramEntry> entries(static_cast<size_t>(info.entry_count) + 1);
  std::memset(entries.data(), 0x5A, entries.size() * sizeof(LUMICE_RaypathHistogramEntry));  // not zero
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysis(nullptr, entries.data(), 1), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, nullptr, 1), LUMICE_ERR_NULL_ARG);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, entries.data(), info.entry_count + 1), LUMICE_OK);
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
    EXPECT_EQ(std::string(e.display).rfind("crystal1(", 0), 0u) << e.display;
    if (i > 0) {
      EXPECT_GE(entries[static_cast<size_t>(i - 1)].energy, e.energy) << "energy descending at " << i;
    }
    total += e.count;
  }
  // The 22° path — faces 3 -> 5 of the prism, which the session's P|B|D reduction leaves as
  // the canonical "3-5" (the same literal test_raypath_histogram_consumer.cpp derives through
  // Crystal::ReduceRaypath; pinned as text here because the text IS this API's contract).
  EXPECT_STREQ(entries[0].display, "crystal1(3-5)");
  EXPECT_EQ(entries[0].chain[0].segment_len, 2);
  EXPECT_EQ(entries[0].chain[0].segment[0], 3);
  EXPECT_EQ(entries[0].chain[0].segment[1], 5);

  // A short read fills exactly max_count and, the array being full, writes NO sentinel —
  // the slot past it is the caller's and stays as it was.
  LUMICE_RaypathHistogramEntry two[3];
  std::memset(two, 0x5A, sizeof(two));
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, two, 2), LUMICE_OK);
  EXPECT_STREQ(two[0].display, entries[0].display);
  EXPECT_STREQ(two[1].display, entries[1].display);
  EXPECT_EQ(two[2].count, 0x5A5A5A5A5A5A5A5Au) << "no sentinel written past a full array";
  LUMICE_RaypathHistogramEntry one[2];
  std::memset(one, 0x5A, sizeof(one));

  // The stats rode along, and every counted ray is one the run traced.
  LUMICE_StatsResult stats{};
  ASSERT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
  EXPECT_EQ(stats.sim_ray_num, 40000u);
  EXPECT_LE(total, stats.sim_ray_num * 8u) << "max_hits = 7 bounds the outgoing rays per input ray";

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
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 0);
  EXPECT_EQ(info.entry_count, 0);
  std::memset(one, 0x5A, sizeof(one));
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, one, 1), LUMICE_OK);
  EXPECT_EQ(one[0].count, 0u) << "sentinel at [0] on a render frame";
  std::memset(xyz, 0x5A, sizeof(xyz));
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz, 1), LUMICE_OK);
  EXPECT_NE(xyz[0].xyz_buffer, nullptr) << "and the render's own image is back";
  LUMICE_ReleaseResultFrame(frame);
}

// The cone request round-trips its echo fields and ring split through the getters.
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
  req.cone_stop_target = 300;
  ASSERT_EQ(LUMICE_StartRaypathAnalysis(server_, &req), LUMICE_OK);
  ASSERT_TRUE(WaitForCompletedAndDrained(server_, 30000)) << "the stop target must end an infinite run";

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  LUMICE_RaypathAnalysisInfo info{};
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysisInfo(frame, &info), LUMICE_OK);
  EXPECT_EQ(info.present, 1);
  EXPECT_EQ(info.roi_mode, LUMICE_RAYPATH_ROI_CONE);
  EXPECT_EQ(info.cone_ring_count, 5);
  EXPECT_FLOAT_EQ(info.cone_radius_rad, req.cone_radius_rad);
  ASSERT_GE(info.entry_count, 1);
  std::vector<LUMICE_RaypathHistogramEntry> entries(static_cast<size_t>(info.entry_count) + 1);
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(frame, entries.data(), info.entry_count + 1), LUMICE_OK);
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
  EXPECT_GE(total, 300u);
  EXPECT_STREQ(entries[0].display, "crystal1(3-5)") << "the 22° path leads in the cone";
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
      seg.segment.push_back(static_cast<lumice::IdType>(f + 1));
    }
    deep.chain_.push_back(seg);
  }
  deep.display_ = std::string(LUMICE_RAYPATH_DISPLAY_MAX + 100, 'x');
  deep.energy_ = 2.5;
  deep.count_ = 7;
  r.entries_.push_back(deep);
  frame->raypath_histogram_result_ = r;

  LUMICE_ResultFrame* handle = WrapResultFrameForTest(frame);
  // Guard bytes around the one entry: a write past it is a test failure, not silent UB.
  struct Guarded {
    unsigned char before[64];
    LUMICE_RaypathHistogramEntry entry;
    unsigned char after[64];
  } g;
  std::memset(&g, 0xA5, sizeof(g));
  ASSERT_EQ(LUMICE_FrameGetRaypathAnalysis(handle, &g.entry, 1), LUMICE_OK);
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
      EXPECT_EQ(g.entry.chain[l].segment[f], f + 1);
    }
  }
  EXPECT_EQ(std::strlen(g.entry.display), static_cast<size_t>(LUMICE_RAYPATH_DISPLAY_MAX - 1));
  EXPECT_EQ(g.entry.display[LUMICE_RAYPATH_DISPLAY_MAX - 1], '\0');
  EXPECT_DOUBLE_EQ(g.entry.energy, 2.5);
  EXPECT_EQ(g.entry.count, 7u);
  EXPECT_EQ(g.entry.ring_count, 0);
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

}  // namespace
