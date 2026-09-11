// The analysis run as a lifecycle of the live Server (Server::StartRaypathAnalysis):
// mutual exclusion with the render run in both directions, the forced CPU route, the
// cone stop target's early completion, and the result frame's histogram field.
//
// Driven through lumice::Server (the C++ surface the C API wraps 1:1) on the 22° halo
// scene of test/e2e/configs/halo_22.json, with TWO workers on the CPU route — the first
// place in the tree where ChainIdMerger sees two real producers concurrently rather
// than a synthetic pair (the consumer's own tests cover the merge on hand-built
// batches).
//
//   AC1  RenderInProgressRejectsAnalysis / AnalysisInProgressRejectsCommit, plus the
//        two allowed transitions next to them (restart with a new ROI; commit-then-
//        Stop-then-analyse).
//   AC2  AnalysisForcesCpuUnderGpuPreference: GetActiveBackend() reads CPU, and the
//        histogram is non-empty — a GPU-routed run would leave it empty, since only the
//        legacy CPU path carries chain ids.
//   Step 3  ConeStopTargetCompletesThroughNaturalCompletion: an unbounded ray_num run ends
//        on its own once the cone has its rays, and reads kCompleted with a valid frame;
//        the control case beside it shows what Stop() would have made of the same run.
//   Step 4  the frame's raypath_histogram_result_ is set in an analysis session and
//        nullopt again after the next render commit.
//   RenderAfter{ConeStopped,FullSky}AnalysisProducesAFrame: the render that follows an
//        analysis traces rays and carries an image — the cone-stopped case is the one
//        whose early-stop flag is still up at the switch, the full-sky case the control.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "core/math.hpp"
#include "server/server.hpp"

namespace lumice {
namespace {

constexpr float kDegToRad = math::kDegreeToRad;

// halo_22.json's crystal and sun, with the ray budget as a parameter: "infinite" for the
// cases that need a run that never ends by itself, a small finite number otherwise.
nlohmann::json Halo22Config(const nlohmann::json& ray_num) {
  return nlohmann::json{
    { "crystal",
      { { { "id", 1 },
          { "type", "prism" },
          { "shape", { { "height", 1.2 } } },
          { "axis",
            { { "zenith", { { "type", "uniform" }, { "mean", 90 }, { "std", 360 } } },
              { "azimuth", { { "type", "uniform" }, { "mean", 0 }, { "std", 360 } } } } } } } },
    { "filter", nlohmann::json::array() },
    { "scene",
      { { "light_source", { { "type", "sun" }, { "altitude", 20.0 }, { "spectrum", "D65" } } },
        { "ray_num", ray_num },
        { "max_hits", 7 },
        { "scattering", { { { "prob", 0.0 }, { "entries", { { { "crystal", 1 }, { "proportion", 10 } } } } } } } } },
    { "render",
      { { { "ev_mode", "absolute" },
          { "id", 1 },
          { "lens", { { "type", "fisheye_equal_area" }, { "fov", 120 } } },
          { "resolution", { 64, 64 } },
          { "view", { { "elevation", 20 } } } } } },
  };
}

// The direction sunlight travels for a sun at (altitude, azimuth), i.e. the antipode of
// where the sun sits — the convention test_raypath_histogram_consumer.cpp pins against
// the simulator's own undeviated rays.
void SunlightDir(float altitude_deg, float azimuth_deg, float out[3]) {
  const float lon = (azimuth_deg + 180.0f) * kDegToRad;
  const float lat = -altitude_deg * kDegToRad;
  out[0] = std::cos(lat) * std::cos(lon);
  out[1] = std::cos(lat) * std::sin(lon);
  out[2] = std::sin(lat);
}

RaypathAnalysisRequest FullSkyRequest() {
  return RaypathAnalysisRequest{};
}

// A cone on the 22° ring straight above the sun (altitude 20 + 23), radius 2.5°.
RaypathAnalysisRequest ConeOnHaloRequest(size_t stop_target) {
  RaypathAnalysisRequest req;
  req.roi_.mode_ = RaypathRoiMode::kCone;
  SunlightDir(20.0f + 23.0f, 0.0f, req.roi_.cone_center_);
  req.roi_.cone_radius_rad_ = 2.5f * kDegToRad;
  req.roi_.cone_ring_count_ = 5;
  req.roi_.cone_stop_target_ = stop_target;
  return req;
}

// Wait for the run to leave kRunning AND for the consumer to drain it (the two halves of
// "the numbers are final" — see DrainedEpoch's contract). Returns the lifecycle observed.
SimLifecycle WaitForRunToEnd(Server& server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  SimLifecycle lc = SimLifecycle::kRunning;
  while (clock::now() < deadline) {
    lc = server.GetSimLifecycle();
    if (lc != SimLifecycle::kRunning && server.DrainedEpoch() == server.CommittedEpoch()) {
      return lc;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return lc;
}

// Wait until the live ray count shows the run has actually started tracing — the point
// past which "in progress" is not merely "Start() was called".
bool WaitForFirstRays(Server& server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    if (server.GetLiveSimRayCount() > 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

size_t TotalCount(const RaypathHistogramResult& r) {
  size_t n = 0;
  for (const auto& e : r.entries_) {
    n += e.count_;
  }
  return n;
}

class ServerAnalysisRun : public ::testing::Test {
 protected:
  // Two workers, random seed: the multi-producer merge is exercised for real.
  ServerAnalysisRun() : server_(2, 0, BackendKind::kCpu) {}
  ~ServerAnalysisRun() override { server_.Stop(); }
  Server server_;
};

// ---------------------------------------------------------------------------
// AC1, direction 1: a render in progress refuses the analysis; Stop() unblocks it.
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, RenderInProgressRejectsAnalysis) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);

  const Error err = server_.StartRaypathAnalysis(FullSkyRequest());
  EXPECT_EQ(err.code, ErrorCode::kServerError) << err.message;
  // The refusal changed nothing: the render is still the run in progress, and its frame is
  // still a render frame.
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);
  EXPECT_FALSE(server_.AcquireResultFrame()->raypath_histogram_result_.has_value());

  server_.Stop();
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kIdle);
  const Error ok = server_.StartRaypathAnalysis(FullSkyRequest());
  EXPECT_FALSE(ok) << ok.message;
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  EXPECT_EQ(server_.GetActiveBackend(), BackendKind::kCpu);
}

// ---------------------------------------------------------------------------
// AC1, direction 2: an analysis in progress refuses a render commit, and the rejected
// commit leaves the committed config (epoch) and the running session untouched. After
// Stop() the commit goes through, and its frame no longer carries a histogram (Step 4).
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, AnalysisInProgressRejectsCommit) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
  server_.Stop();
  const uint64_t epoch_before = server_.CommittedEpoch();
  ASSERT_FALSE(server_.StartRaypathAnalysis(FullSkyRequest()));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);

  const Error err = server_.CommitConfig(Halo22Config(1000));
  EXPECT_EQ(err.code, ErrorCode::kServerError) << err.message;
  EXPECT_EQ(server_.CommittedEpoch(), epoch_before) << "a rejected commit must not mint an epoch";
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning) << "the analysis keeps running";
  EXPECT_EQ(server_.GetActiveBackend(), BackendKind::kCpu);
  {
    auto frame = server_.AcquireResultFrame();
    ASSERT_TRUE(frame->raypath_histogram_result_.has_value()) << "still an analysis frame";
    EXPECT_EQ(frame->raypath_histogram_result_->roi_mode_, RaypathRoiMode::kFullSky);
  }

  server_.Stop();
  ASSERT_FALSE(server_.CommitConfig(Halo22Config(1000)));
  EXPECT_EQ(server_.CommittedEpoch(), epoch_before + 1);
  ASSERT_EQ(WaitForRunToEnd(server_, 20000), SimLifecycle::kCompleted);
  auto frame = server_.AcquireResultFrame();
  EXPECT_TRUE(frame->has_valid_data_);
  EXPECT_FALSE(frame->raypath_histogram_result_.has_value()) << "a render frame carries no histogram";
  EXPECT_EQ(frame->render_results_.size(), 1u);
}

// ---------------------------------------------------------------------------
// The allowed transition beside the two refusals: an analysis in progress may be
// restarted with a new ROI, and the next frame reflects the new one.
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, AnalysisInProgressRestartsWithNewRoi) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
  server_.Stop();
  ASSERT_FALSE(server_.StartRaypathAnalysis(FullSkyRequest()));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);

  const Error ok = server_.StartRaypathAnalysis(ConeOnHaloRequest(100));
  EXPECT_FALSE(ok) << ok.message;
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  auto frame = server_.AcquireResultFrame();
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value());
  EXPECT_EQ(frame->raypath_histogram_result_->roi_mode_, RaypathRoiMode::kCone);
  EXPECT_EQ(frame->raypath_histogram_result_->cone_ring_count_, 5);
}

// ---------------------------------------------------------------------------
// The other allowed transition (plan risk 2): CommitConfig always starts a render, so
// "commit, then analyse" is commit → Stop → StartRaypathAnalysis, and nothing about the
// commit having just run must make the guard read it as "render in progress".
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, CommitThenStopThenAnalysisIsAccepted) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config(20000)));
  server_.Stop();
  const Error ok = server_.StartRaypathAnalysis(FullSkyRequest());
  EXPECT_FALSE(ok) << ok.message;
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  auto frame = server_.AcquireResultFrame();
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value());
  EXPECT_GT(TotalCount(*frame->raypath_histogram_result_), 0u);
  // Not a render frame: no image, and no anchor was measured.
  EXPECT_TRUE(frame->render_results_.empty());
  EXPECT_TRUE(frame->xyz_results_.empty());
  // The stats consumer rode along: the live count matches the run's budget.
  ASSERT_TRUE(frame->stats_result_.has_value());
  EXPECT_EQ(frame->stats_result_->sim_ray_num_, 20000u);
}

// Neither session's first frame carries the other's results — not even flagged stale.
// Read IMMEDIATELY after each switch: whether the frame is the empty one published at the
// switch or already the new session's first snapshot, the other session's payload is gone.
TEST_F(ServerAnalysisRun, FrameRightAfterASessionSwitchCarriesNoResidue) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config(20000)));
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  ASSERT_EQ(server_.AcquireResultFrame()->render_results_.size(), 1u) << "positive control: the render's image";
  const uint64_t generation = server_.AcquireResultFrame()->snapshot_generation_;

  server_.Stop();
  ASSERT_FALSE(server_.StartRaypathAnalysis(FullSkyRequest()));
  {
    auto frame = server_.AcquireResultFrame();
    EXPECT_TRUE(frame->render_results_.empty()) << "the render's image is gone from the analysis session";
    EXPECT_TRUE(frame->xyz_results_.empty());
    EXPECT_TRUE(frame->composite_results_.empty());
    EXPECT_GE(frame->snapshot_generation_, generation) << "the generation never goes backwards";
  }
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  ASSERT_TRUE(server_.AcquireResultFrame()->raypath_histogram_result_.has_value()) << "positive control";

  ASSERT_FALSE(server_.CommitConfig(Halo22Config(20000)));
  {
    auto frame = server_.AcquireResultFrame();
    EXPECT_FALSE(frame->raypath_histogram_result_.has_value()) << "the histogram is gone from the render session";
  }
}

// Nothing committed yet: there is no scene to analyse.
TEST_F(ServerAnalysisRun, AnalysisWithoutCommitIsInvalidConfig) {
  const Error err = server_.StartRaypathAnalysis(FullSkyRequest());
  EXPECT_EQ(err.code, ErrorCode::kInvalidConfig) << err.message;
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kIdle);
}

// ---------------------------------------------------------------------------
// Step 3: the cone stop target ends an UNBOUNDED run by itself, and the run reads
// kCompleted with a valid frame whose cone has at least the target's rays. The control
// case shows the same run ended by Stop() instead: kIdle, no data — which is why the
// mechanism must not be Stop().
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, ConeStopTargetCompletesThroughNaturalCompletion) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
  server_.Stop();
  constexpr size_t kTarget = 200;
  ASSERT_FALSE(server_.StartRaypathAnalysis(ConeOnHaloRequest(kTarget)));

  // An infinite run with no stop target would sit in kRunning until the timeout, so
  // reaching the end at all is the assertion; 30 s is far above the measured time.
  const auto t0 = std::chrono::steady_clock::now();
  const SimLifecycle lc = WaitForRunToEnd(server_, 30000);
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_EQ(lc, SimLifecycle::kCompleted) << "ended in " << secs << " s as lifecycle " << static_cast<int>(lc);

  auto frame = server_.AcquireResultFrame();
  EXPECT_TRUE(frame->has_valid_data_);
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value());
  const auto& r = *frame->raypath_histogram_result_;
  EXPECT_EQ(r.roi_mode_, RaypathRoiMode::kCone);
  EXPECT_GE(TotalCount(r), kTarget);
  // The producer stopped: the live count is finite and stays put.
  const size_t rays_at_end = server_.GetLiveSimRayCount();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(server_.GetLiveSimRayCount(), rays_at_end);
  // And the histogram is the 22° halo's: its top chain is one layer through crystal 1.
  ASSERT_FALSE(r.entries_.empty());
  ASSERT_EQ(r.entries_[0].chain_.size(), 1u);
  EXPECT_EQ(r.entries_[0].chain_[0].crystal_id, 1u);
  EXPECT_EQ(r.entries_[0].ring_energy_.size(), 5u);
}

// ---------------------------------------------------------------------------
// The render AFTER an analysis — the owner's own sequence: Run, Analyze, Run again. The
// second Run is a plain CommitConfig (an Exclude in between is a GUI-side document edit
// that reaches the server only as part of that commit), so the proposition is the
// server's alone: a render session that follows an analysis session traces rays, and
// its frame carries the image.
//
// Two cases, one predicate each, and the pair is the diagnosis: the cone-stopped
// analysis is the one whose early-stop flag (analysis_roi_target_reached_) is up when the
// session ends, and the whole-sky analysis is the one that can never raise it
// (RaypathHistogramConsumer::RoiTargetReached is kCone-only). A render that traces
// nothing after the first and everything after the second puts the first divergence
// on that flag, not on the session switch as such.
//
// What "traces nothing" reads as at this layer: GenerateScene's loop never runs, so no
// batch is ever queued or consumed, and the lifecycle settles as kIdle — the producer-
// side predicates are all quiet and has_ever_consumed_ never turns true — with a frame
// that has no valid data. (The GUI reads that as "Simulating forever": its sim_state
// leaves kSimulating on COMPLETED only, and nothing ever completes.)
// ---------------------------------------------------------------------------
void RenderAfterAnalysisProducesAFrame(Server& server, const RaypathAnalysisRequest& request) {
  ASSERT_FALSE(server.CommitConfig(Halo22Config("infinite")));
  server.Stop();
  ASSERT_FALSE(server.StartRaypathAnalysis(request));
  if (request.roi_.mode_ == RaypathRoiMode::kCone) {
    // The cone target ends the unbounded run by itself, with the early-stop flag up.
    ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  } else {
    // A whole-sky analysis of an unbounded run has no end of its own: Stop() it once it
    // has data, which is the other way an analysis session can be left behind.
    ASSERT_TRUE(WaitForFirstRays(server, 5000));
    server.Stop();
  }

  // The second Run.
  ASSERT_FALSE(server.CommitConfig(Halo22Config(20000)));
  EXPECT_TRUE(WaitForFirstRays(server, 5000)) << "the render session after the analysis traced no rays at all";
  const SimLifecycle lc = WaitForRunToEnd(server, 30000);
  ASSERT_EQ(lc, SimLifecycle::kCompleted)
      << "settled as lifecycle " << static_cast<int>(lc) << " (kIdle = no batch was ever produced or consumed)";
  auto frame = server.AcquireResultFrame();
  EXPECT_TRUE(frame->has_valid_data_);
  EXPECT_FALSE(frame->raypath_histogram_result_.has_value()) << "the histogram is the analysis session's";
  ASSERT_EQ(frame->render_results_.size(), 1u) << "the render's image";
  ASSERT_TRUE(frame->stats_result_.has_value());
  EXPECT_EQ(frame->stats_result_->sim_ray_num_, 20000u) << "the whole budget was traced";
}

TEST_F(ServerAnalysisRun, RenderAfterConeStoppedAnalysisProducesAFrame) {
  RenderAfterAnalysisProducesAFrame(server_, ConeOnHaloRequest(200));
}

// The control: the same sequence through the ROI mode that cannot raise the flag.
TEST_F(ServerAnalysisRun, RenderAfterFullSkyAnalysisProducesAFrame) {
  RenderAfterAnalysisProducesAFrame(server_, FullSkyRequest());
}

TEST_F(ServerAnalysisRun, ControlStopReadsAsIdleWithNoData) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
  server_.Stop();
  ASSERT_FALSE(server_.StartRaypathAnalysis(FullSkyRequest()));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  server_.Stop();
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kIdle) << "Stop() is a reset, not a completion";
  EXPECT_FALSE(server_.AcquireResultFrame()->has_valid_data_);
}

// ---------------------------------------------------------------------------
// AC2: under a GPU preference the analysis still runs on the CPU. Two observations, one
// direct (GetActiveBackend) and one that cannot be faked by a flag: a histogram with
// entries, which only the legacy CPU path produces (the trace-backend routes leave
// SimData::outgoing_chain_id_ empty). The render half of the case first shows the
// preference was honoured for a render, so the CPU reading is the session's doing.
// ---------------------------------------------------------------------------
TEST(ServerAnalysisRunGpu, AnalysisForcesCpuUnderGpuPreference) {
#if defined(__APPLE__)
  const BackendKind kGpu = BackendKind::kMetal;
#elif defined(LUMICE_CUDA_ENABLED)
  const BackendKind kGpu = BackendKind::kCuda;
#else
  GTEST_SKIP() << "no GPU backend in this build; the CPU-route half is covered by the fixture above";
#endif
  Logger probe{ "ServerAnalysisRunGpu" };
  if (!ResolveGpuRoute(kGpu, probe)) {
    GTEST_SKIP() << "GPU route unavailable on this machine (or overridden by LUMICE_TRACE_BACKEND)";
  }
  Server server(1, 0, kGpu);
  ASSERT_FALSE(server.CommitConfig(Halo22Config(20000)));
  ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  EXPECT_EQ(server.GetActiveBackend(), kGpu) << "the render honoured the preference";
  EXPECT_FALSE(server.BackendFellBack());

  server.Stop();
  ASSERT_FALSE(server.StartRaypathAnalysis(FullSkyRequest()));
  EXPECT_EQ(server.GetActiveBackend(), BackendKind::kCpu) << "the analysis session forces CPU";
  EXPECT_FALSE(server.BackendFellBack()) << "a forced route is not a fallback";
  ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  EXPECT_EQ(server.GetActiveBackend(), BackendKind::kCpu);
  auto frame = server.AcquireResultFrame();
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value());
  EXPECT_GT(TotalCount(*frame->raypath_histogram_result_), 0u) << "chain ids exist on the CPU path only";

  // And the preference survives the analysis: the next render is GPU again.
  ASSERT_FALSE(server.CommitConfig(Halo22Config(20000)));
  ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  EXPECT_EQ(server.GetActiveBackend(), kGpu) << "the preference was not overwritten";
  EXPECT_FALSE(server.AcquireResultFrame()->raypath_histogram_result_.has_value());
  server.Stop();
}

}  // namespace
}  // namespace lumice
