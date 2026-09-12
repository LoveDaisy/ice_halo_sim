// The analysis run as a lifecycle of the live Server (Server::StartRaypathAnalysis):
// mutual exclusion with the render run in both directions, the forced CPU route, the
// manual stop that keeps the accumulated result, the result frame's histogram field, and
// the scene being the call's own — an analysis needs no CommitConfig before it, traces the
// document it is handed rather than the last render's, and a document it rejects changes
// nothing.
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
//   Step 3  AnalysisManualStopPreservesAccumulatedResults: an unbounded run has no end of
//        its own (there is no cone stop target); Stop() ends it, the run reads kIdle, and
//        the frame published by that Stop() carries the histogram consumed up to it.
//   Step 4  the frame's raypath_histogram_result_ is set in an analysis session and
//        nullopt again after the next render commit.
//   RenderAfterStoppedAnalysisProducesAFrame: the render that follows a stopped analysis
//        traces rays and carries an image — the session switch itself leaves nothing
//        behind that could keep the next render from producing.

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
RaypathAnalysisRequest ConeOnHaloRequest() {
  RaypathAnalysisRequest req;
  req.roi_.mode_ = RaypathRoiMode::kCone;
  SunlightDir(20.0f + 23.0f, 0.0f, req.roi_.cone_center_);
  req.roi_.cone_radius_rad_ = 2.5f * kDegToRad;
  req.roi_.cone_ring_count_ = 5;
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

  const Error err = server_.StartRaypathAnalysis(Halo22Config("infinite"), FullSkyRequest());
  EXPECT_EQ(err.code, ErrorCode::kServerError) << err.message;
  // The refusal changed nothing: the render is still the run in progress, and its frame is
  // still a render frame.
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);
  EXPECT_FALSE(server_.AcquireResultFrame()->raypath_histogram_result_.has_value());

  server_.Stop();
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kIdle);
  const Error ok = server_.StartRaypathAnalysis(Halo22Config("infinite"), FullSkyRequest());
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
  const uint64_t epoch_after_render = server_.CommittedEpoch();
  ASSERT_FALSE(server_.StartRaypathAnalysis(Halo22Config("infinite"), FullSkyRequest()));
  // The analysis is a submission: it minted an epoch of its own, past the render's.
  const uint64_t epoch_before = server_.CommittedEpoch();
  EXPECT_EQ(epoch_before, epoch_after_render + 1);
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
  ASSERT_FALSE(server_.StartRaypathAnalysis(Halo22Config("infinite"), FullSkyRequest()));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);

  // The new request carries its own finite budget, so the restarted run ends by itself
  // on an unlimited scene.
  RaypathAnalysisRequest cone = ConeOnHaloRequest();
  cone.ray_num_ = 20000;
  const Error ok = server_.StartRaypathAnalysis(Halo22Config("infinite"), cone);
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
  const Error ok = server_.StartRaypathAnalysis(Halo22Config(20000), FullSkyRequest());
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
  ASSERT_FALSE(server_.StartRaypathAnalysis(Halo22Config(20000), FullSkyRequest()));
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

// ---------------------------------------------------------------------------
// The scene is the call's own. Three propositions, each a case: no CommitConfig is needed
// before an analysis; the analysis traces the scene it is handed, not the last render's;
// and a scene it rejects leaves the server exactly as it was.
// ---------------------------------------------------------------------------

// A server that has never committed anything analyses: the scene comes with the call. The
// first epoch this server ever mints is the analysis's.
TEST_F(ServerAnalysisRun, AnalysisWithoutPriorCommitSucceeds) {
  ASSERT_EQ(server_.CommittedEpoch(), 0u) << "positive control: nothing submitted yet";
  const Error ok = server_.StartRaypathAnalysis(Halo22Config(20000), FullSkyRequest());
  EXPECT_FALSE(ok) << ok.message;
  EXPECT_EQ(server_.CommittedEpoch(), 1u) << "the analysis is a submission: it minted the epoch";
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  auto frame = server_.AcquireResultFrame();
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value());
  EXPECT_GT(TotalCount(*frame->raypath_histogram_result_), 0u);
  ASSERT_TRUE(frame->stats_result_.has_value());
  EXPECT_EQ(frame->stats_result_->sim_ray_num_, 20000u) << "the handed scene's budget";

  // And the render that follows is an ordinary first commit: it traces and carries an image.
  ASSERT_FALSE(server_.CommitConfig(Halo22Config(20000)));
  EXPECT_EQ(server_.CommittedEpoch(), 2u);
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  frame = server_.AcquireResultFrame();
  EXPECT_TRUE(frame->has_valid_data_);
  EXPECT_FALSE(frame->raypath_histogram_result_.has_value());
  EXPECT_EQ(frame->render_results_.size(), 1u);
}

// The analysis traces the document it is handed, not the one the last render committed.
// The two documents differ in the one thing the histogram names directly — the crystal's
// id — and in a colour config the render carried that names a crystal the analysed scene
// does not have. The second half is the load-bearing one: the simulator builds its colour
// gate table from (raypath_color, scene) on every batch, and a render's colour config left
// bound over an analysis of a different scene would throw on the worker thread. There is
// no ASSERT for "did not throw"; the run completing with the right crystal in its top row
// is that assertion.
TEST_F(ServerAnalysisRun, AnalysisTracesTheSceneItIsHandedNotTheLastRenders) {
  nlohmann::json render_doc = Halo22Config(20000);
  render_doc["raypath_color"] = {
    { "mode", "dominant" },
    { "classes",
      nlohmann::json::array({ { { "color", { 1.0f, 0.0f, 0.0f } },
                                { "match", nlohmann::json::array({ { { "layer", 0 }, { "crystal", 1 } } }) } } }) }
  };
  ASSERT_FALSE(server_.CommitConfig(render_doc));
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  server_.Stop();

  // Same halo, but the crystal is id 7 — and there is no crystal 1 for the render's colour
  // class to name.
  nlohmann::json analysis_doc = Halo22Config(20000);
  analysis_doc["crystal"][0]["id"] = 7;
  analysis_doc["scene"]["scattering"][0]["entries"][0]["crystal"] = 7;
  const Error ok = server_.StartRaypathAnalysis(analysis_doc, FullSkyRequest());
  ASSERT_FALSE(ok) << ok.message;
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  auto frame = server_.AcquireResultFrame();
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value());
  const auto& r = *frame->raypath_histogram_result_;
  ASSERT_FALSE(r.entries_.empty());
  ASSERT_EQ(r.entries_[0].chain_.size(), 1u);
  EXPECT_EQ(r.entries_[0].chain_[0].crystal_id, 7u) << "the analysed scene's crystal, not the render's";
  EXPECT_GT(TotalCount(r), 0u);

  // The render's bookkeeping survived the analysis untouched: the next commit of the
  // render's own document is judged against it and renders as before.
  ASSERT_FALSE(server_.CommitConfig(render_doc));
  ASSERT_EQ(WaitForRunToEnd(server_, 30000), SimLifecycle::kCompleted);
  frame = server_.AcquireResultFrame();
  EXPECT_TRUE(frame->has_valid_data_);
  EXPECT_EQ(frame->render_results_.size(), 1u);
}

// A document the parser rejects changes nothing: not the epoch, not the lifecycle, and
// not a session in flight. The three failure shapes each map onto their own code — the
// same three CommitConfig returns, since the two share the parser — and each is tried
// against a running analysis so "untouched" includes "not stopped".
TEST_F(ServerAnalysisRun, ARejectedSceneLeavesTheServerUntouched) {
  ASSERT_FALSE(server_.StartRaypathAnalysis(Halo22Config("infinite"), FullSkyRequest()));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  const uint64_t epoch = server_.CommittedEpoch();

  struct Shape {
    const char* name;
    nlohmann::json doc;
    ErrorCode code;
  };
  nlohmann::json missing = Halo22Config(20000);
  missing["scene"].erase("light_source");
  nlohmann::json wrong_type = Halo22Config(20000);
  wrong_type["scene"]["max_hits"] = "seven";
  nlohmann::json bad_value = Halo22Config(20000);
  bad_value["crystal"][0]["type"] = "dodecahedron";
  const Shape kShapes[] = {
    { "a missing field", missing, ErrorCode::kMissingField },
    { "a field of the wrong type", wrong_type, ErrorCode::kInvalidJson },
    { "a value the config rejects", bad_value, ErrorCode::kInvalidConfig },
  };
  for (const Shape& shape : kShapes) {
    SCOPED_TRACE(shape.name);
    const Error err = server_.StartRaypathAnalysis(shape.doc, FullSkyRequest());
    EXPECT_EQ(err.code, shape.code) << err.message;
    EXPECT_EQ(server_.CommittedEpoch(), epoch) << "a rejected scene must not mint an epoch";
    EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning) << "the analysis in flight was not stopped";
  }
  // The positive control for the code mapping: the same three documents get the same
  // three codes from CommitConfig.
  server_.Stop();
  for (const Shape& shape : kShapes) {
    SCOPED_TRACE(shape.name);
    EXPECT_EQ(server_.CommitConfig(shape.doc).code, shape.code);
  }
  EXPECT_EQ(server_.CommittedEpoch(), epoch);
}

// ---------------------------------------------------------------------------
// Step 3: an UNBOUNDED analysis run has no end of its own — Stop() is the only way to
// end it, and a stop is a reset (kIdle), not a completion. What the stop must NOT do is
// discard what the run accumulated: Stop() publishes the batches consumed since the
// last poll before it resets, so the frame acquired afterwards carries the histogram
// as it stood at the stop. The frame is acquired only AFTER Stop() returns and
// without any poll in between, so the assertion is on the stop's own publication, not
// on a snapshot some earlier read happened to take — without it the session's opening
// empty frame (no histogram at all) would still be the published one. has_valid_data_
// is NOT the oracle here: AcquireResultFrame re-stamps it from the live
// has_ever_consumed_, which the stop resets, so it reads false on a stopped analysis
// just as GetSimLifecycle() reads kIdle; the result travels in
// raypath_histogram_result_ (the C API's `present`), which is what is asserted.
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, AnalysisManualStopPreservesAccumulatedResults) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
  server_.Stop();
  ASSERT_FALSE(server_.StartRaypathAnalysis(Halo22Config("infinite"), ConeOnHaloRequest()));
  ASSERT_TRUE(WaitForFirstRays(server_, 5000));
  ASSERT_EQ(server_.GetSimLifecycle(), SimLifecycle::kRunning);

  server_.Stop();
  EXPECT_EQ(server_.GetSimLifecycle(), SimLifecycle::kIdle) << "Stop() is a reset, not a completion";
  // The producer stopped: the live count is finite and stays put.
  const size_t rays_at_end = server_.GetLiveSimRayCount();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(server_.GetLiveSimRayCount(), rays_at_end);

  auto frame = server_.AcquireResultFrame();
  EXPECT_FALSE(frame->has_valid_data_) << "the live flag mirrors kIdle; the result is carried by the histogram";
  ASSERT_TRUE(frame->raypath_histogram_result_.has_value()) << "the stop published the data consumed before it";
  const auto& r = *frame->raypath_histogram_result_;
  EXPECT_EQ(r.roi_mode_, RaypathRoiMode::kCone);
  EXPECT_GT(TotalCount(r), 0u) << "the histogram accumulated up to the stop is readable";
  // And the histogram is the 22° halo's: its top chain is one layer through crystal 1.
  ASSERT_FALSE(r.entries_.empty());
  ASSERT_EQ(r.entries_[0].chain_.size(), 1u);
  EXPECT_EQ(r.entries_[0].chain_[0].crystal_id, 1u);
  EXPECT_EQ(r.entries_[0].ring_energy_.size(), 5u);
  // The stats consumer was published by the same stop: its count is what the live
  // counter read, not zero.
  ASSERT_TRUE(frame->stats_result_.has_value());
  EXPECT_GT(frame->stats_result_->sim_ray_num_, 0u);
  EXPECT_LE(frame->stats_result_->sim_ray_num_, rays_at_end);
}

// ---------------------------------------------------------------------------
// The render AFTER an analysis — the owner's own sequence: Run, Analyze, Run again. The
// second Run is a plain CommitConfig (an Exclude in between is a GUI-side document edit
// that reaches the server only as part of that commit), so the proposition is the
// server's alone: a render session that follows an analysis session traces rays, and
// its frame carries the image.
//
// An analysis session is left behind in exactly one way now that a cone has no stop of
// its own: Stop() (a run that ends on its budget takes the same switch, through
// CommitConfig's was_analysis branch, with nothing extra to leave behind). A render that
// traced nothing after it would put the divergence on the session switch itself — the
// was_analysis branch and the empty frame it publishes — which is what this guards.
//
// What "traces nothing" reads as at this layer: GenerateScene's loop never runs, so no
// batch is ever queued or consumed, and the lifecycle settles as kIdle — the producer-
// side predicates are all quiet and has_ever_consumed_ never turns true — with a frame
// that has no valid data. (The GUI reads that as "Simulating forever": its sim_state
// leaves kSimulating on COMPLETED only, and nothing ever completes.)
// ---------------------------------------------------------------------------
TEST_F(ServerAnalysisRun, RenderAfterStoppedAnalysisProducesAFrame) {
  Server& server = server_;
  ASSERT_FALSE(server.CommitConfig(Halo22Config("infinite")));
  server.Stop();
  ASSERT_FALSE(server.StartRaypathAnalysis(Halo22Config("infinite"), ConeOnHaloRequest()));
  // An unbounded analysis has no end of its own: Stop() it once it has data.
  ASSERT_TRUE(WaitForFirstRays(server, 5000));
  server.Stop();
  ASSERT_EQ(server.GetSimLifecycle(), SimLifecycle::kIdle);

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

// The boundary of the analysis-only stop rule: a RENDER session's Stop() still reads as
// idle with no data (doc/capi-lifecycle-architecture.md §7.1) — the contract the GUI's
// re-simulate paths rest on, which the analysis branch in Stop() must not widen.
TEST_F(ServerAnalysisRun, RenderStopStillReadsAsIdleWithNoData) {
  ASSERT_FALSE(server_.CommitConfig(Halo22Config("infinite")));
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
  ASSERT_FALSE(server.StartRaypathAnalysis(Halo22Config(20000), FullSkyRequest()));
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
