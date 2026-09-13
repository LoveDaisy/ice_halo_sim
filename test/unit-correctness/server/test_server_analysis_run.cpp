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
//   AnalysisIsBitIdenticalAcrossSessionsOfOneServer: the fixed-seed determinism contract
//        holds for every session of one server, not only its first — a later analysis
//        (after a render + Stop(), or after another analysis) reproduces the first row
//        for row.
//   FourWorkersMergeIntoTheSameHistogramAsOne: an N-worker analysis is the same
//        histogram as a 1-worker one, up to the scene's measured noise — the worker
//        count is a merge-correctness axis of its own now that a server sizes it.
//   ServerAnalysisRunGpuPool.*: the GPU-preferred server's standing CPU analysis pool —
//        that it runs (a rate no single worker reaches), that a fixed seed collapses it,
//        that render and analysis alternate without deadlock or leaked results, and
//        that the dtor returns with an analysis in flight.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "core/math.hpp"
#include "server/server.hpp"
#include "util/cpu_info.hpp"

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
// The fixed-seed contract across sessions of ONE server. A non-zero sim_seed sizes the
// server to a single worker precisely so that the run is deterministic (the constructor
// says so: "deterministic CPU contract: fixed seed -> single worker"). That promise is only
// worth anything if it holds for EVERY session the server runs, not just its first: the GUI
// keeps one server for the life of the window, so every analysis a user starts after the
// first render is a later session. The three sessions here are the shapes that matter — a
// fresh server's first analysis; the analysis that follows a completed render and a Stop()
// (the shape the defect was first seen in); and an analysis started straight after another
// analysis — and every row of every one must agree bit for bit (== on the doubles, no
// tolerance: this is a determinism contract, not a precision question).
//
// The defect this pins is deterministic, not a timing window, so two sessions are enough
// to see it: the worker's own RNG was seeded once at construction and never re-seeded at
// the start of a Run(), so the second session picked up the stream wherever the first
// had left it. On the 22-degree scene it showed as a different total energy on every
// session after the first, 100% of the time, whatever the preceding session was; the
// three-session shape here is for coverage of the transitions, not for statistical power.
// ---------------------------------------------------------------------------
struct AnalysisFingerprint {
  std::vector<std::tuple<std::string, double, size_t>> rows;  // (display, energy, count), finest
  double other_energy = 0.0;
  size_t other_count = 0;
  uint64_t sim_ray_num = 0;
  bool operator==(const AnalysisFingerprint& o) const {
    return rows == o.rows && other_energy == o.other_energy && other_count == o.other_count &&
           sim_ray_num == o.sim_ray_num;
  }
};

AnalysisFingerprint FingerprintOf(const ResultFrame& frame) {
  AnalysisFingerprint fp;
  const RaypathHistogramResult& r = *frame.raypath_histogram_result_;
  fp.rows.reserve(r.entries_.size());
  for (const auto& e : r.entries_) {
    fp.rows.emplace_back(e.display_, e.energy_, e.count_);
  }
  fp.other_energy = r.other_energy_;
  fp.other_count = r.other_count_;
  fp.sim_ray_num = frame.stats_result_ ? frame.stats_result_->sim_ray_num_ : 0u;
  return fp;
}

double TotalEnergy(const AnalysisFingerprint& fp) {
  double e = fp.other_energy;
  for (const auto& row : fp.rows) {
    e += std::get<1>(row);
  }
  return e;
}

AnalysisFingerprint RunAnalysisAndFingerprint(Server& server, const nlohmann::json& scene) {
  const Error ok = server.StartRaypathAnalysis(scene, FullSkyRequest());
  EXPECT_FALSE(ok) << ok.message;
  EXPECT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  auto frame = server.AcquireResultFrame();
  EXPECT_TRUE(frame->raypath_histogram_result_.has_value());
  return FingerprintOf(*frame);
}

TEST(ServerAnalysisRunFixedSeed, AnalysisIsBitIdenticalAcrossSessionsOfOneServer) {
  constexpr uint32_t kSeed = 1;
  Server server(0, kSeed, BackendKind::kCpu);  // fixed seed: sized to one worker by the ctor
  const nlohmann::json scene = Halo22Config(20000);

  // Session 1: the fresh server's first run — the one the contract was always kept on.
  const AnalysisFingerprint first = RunAnalysisAndFingerprint(server, scene);
  ASSERT_FALSE(first.rows.empty());
  ASSERT_EQ(first.sim_ray_num, 20000u);

  // Session 2: after a completed render and a Stop() — the shape the defect was seen in.
  ASSERT_FALSE(server.CommitConfig(scene));
  ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  server.Stop();
  const AnalysisFingerprint after_render = RunAnalysisAndFingerprint(server, scene);

  // Session 3: straight after another analysis, no render in between.
  const AnalysisFingerprint after_analysis = RunAnalysisAndFingerprint(server, scene);

  EXPECT_EQ(after_render.rows.size(), first.rows.size());
  EXPECT_EQ(after_analysis.rows.size(), first.rows.size());
  EXPECT_EQ(TotalEnergy(after_render), TotalEnergy(first)) << "session 2 (after a render + Stop) drifted";
  EXPECT_EQ(TotalEnergy(after_analysis), TotalEnergy(first)) << "session 3 (after an analysis) drifted";
  EXPECT_TRUE(after_render == first) << "session 2 differs from session 1 row for row";
  EXPECT_TRUE(after_analysis == first) << "session 3 differs from session 1 row for row";
}

// ---------------------------------------------------------------------------
// AC2: under a GPU preference the analysis still runs on the CPU. Two observations, one
// direct (GetActiveBackend) and one that cannot be faked by a flag: a histogram with
// entries, which only the legacy CPU path produces (the trace-backend routes leave
// SimData::outgoing_chain_id_ empty). The render half of the case first shows the
// preference was honoured for a render, so the CPU reading is the session's doing.
// ---------------------------------------------------------------------------
TEST(ServerAnalysisRunGpu, AnalysisForcesCpuUnderGpuPreference) {
  // The whole body sits inside the platform guard: on a build with no GPU backend `kGpu` has no
  // value to take, and a GTEST_SKIP() alone does not stop the compiler from reading the lines
  // after it (the CI legs without Metal or CUDA are exactly the ones that compile this TU).
#if defined(__APPLE__) || defined(LUMICE_CUDA_ENABLED)
#if defined(__APPLE__)
  const BackendKind kGpu = BackendKind::kMetal;
#else
  const BackendKind kGpu = BackendKind::kCuda;
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
#else
  GTEST_SKIP() << "no GPU backend in this build; the CPU-route half is covered by the fixture above";
#endif
}


// ---------------------------------------------------------------------------
// Multi-worker analysis correctness (the standing-pool change made the worker count of an
// analysis a first-class property, so "N workers merge into the same histogram as one"
// needs a proposition of its own; until now no test compared an N>1 analysis to anything).
// Same scene, same ROI, same budget on a 1-worker and a 4-worker CPU-route server. The two
// runs are different random draws, so the comparison is statistical wherever it can be and
// exact only where the mechanism makes it so:
//   exact   — sim_ray_num equals the budget on both (the batch schedule sums to it);
//             no two rows carry the same chain (ChainIdMerger keyed the four producers'
//             tables into one id space: a merge failure shows as one chain split across
//             per-producer rows, which the 1-worker run cannot produce).
//   bounded — every major chain (>= kMajorFrac of the energy) of either run is a row of
//             the other with at least half that share (hysteresis, so a chain sitting on
//             the threshold cannot flip the verdict), and its energy share agrees within
//             kShareTol; the total counted rays and the total energy agree within a
//             tolerance set by the noise measured on this scene, not chosen.
// The tolerances come from measured noise, 5 + 5 runs at 200k rays on this scene
// (2026-09-13): the total energy spreads 4.89M-5.57M (about +-7% around a mean that is
// the same for N=1 and N=4, 5.16M vs 5.20M — the legacy grain draws one wavelength per
// 128-ray batch, so 1563 draws of Y(wl) set that spread), the eight ~3% chains' shares
// move 0.0296-0.0329 (up to ~11% relative between two runs), the ~2% chains 0.0196-0.0212,
// and the counted-ray total 962949-964260 (0.14%). A merge defect is not a few percent:
// a chain split across workers loses 75% of its share, a chain double-counted doubles it.
// ---------------------------------------------------------------------------
struct ChainShare {
  std::string display;
  double frac;
  size_t count;
};

std::vector<ChainShare> SharesOf(const AnalysisFingerprint& fp, double total) {
  std::vector<ChainShare> out;
  out.reserve(fp.rows.size());
  for (const auto& row : fp.rows) {
    out.push_back({ std::get<0>(row), std::get<1>(row) / total, std::get<2>(row) });
  }
  return out;
}

const ChainShare* FindShare(const std::vector<ChainShare>& shares, const std::string& display) {
  for (const auto& s : shares) {
    if (s.display == display) {
      return &s;
    }
  }
  return nullptr;
}

TEST(ServerAnalysisRunMultiWorker, FourWorkersMergeIntoTheSameHistogramAsOne) {
  constexpr size_t kRays = 200000;
  constexpr double kMajorFrac = 0.015;  // a chain worth >= 1.5% of the energy is "major"
  constexpr double kShareTol = 0.25;    // relative, on a major chain's energy share
  constexpr double kTotalTol = 0.25;    // relative, on the total energy (4% noise per run)
  constexpr double kCountTol = 0.02;    // relative, on the counted-ray total (0.2% noise)
  const nlohmann::json scene = Halo22Config(kRays);

  AnalysisFingerprint one;
  AnalysisFingerprint four;
  {
    Server server(1, 0, BackendKind::kCpu);
    one = RunAnalysisAndFingerprint(server, scene);
    server.Stop();
  }
  {
    Server server(4, 0, BackendKind::kCpu);
    four = RunAnalysisAndFingerprint(server, scene);
    server.Stop();
  }
  ASSERT_FALSE(one.rows.empty());
  ASSERT_FALSE(four.rows.empty());

  // Exact: the budget is traced in full on both, whatever the worker count.
  EXPECT_EQ(one.sim_ray_num, kRays);
  EXPECT_EQ(four.sim_ray_num, kRays);

  // Exact: one row per chain. Keyed on the display text, which is the chain's own
  // formatting (a per-producer split would show as the same text on several rows).
  {
    std::set<std::string> seen;
    for (const auto& row : four.rows) {
      EXPECT_TRUE(seen.insert(std::get<0>(row)).second) << "duplicate chain row: " << std::get<0>(row);
    }
  }

  const double total_one = TotalEnergy(one);
  const double total_four = TotalEnergy(four);
  ASSERT_GT(total_one, 0.0);
  ASSERT_GT(total_four, 0.0);
  EXPECT_NEAR(total_four / total_one, 1.0, kTotalTol)
      << "total energy: 1 worker " << total_one << ", 4 workers " << total_four;
  size_t count_one = one.other_count;
  size_t count_four = four.other_count;
  for (const auto& row : one.rows) {
    count_one += std::get<2>(row);
  }
  for (const auto& row : four.rows) {
    count_four += std::get<2>(row);
  }
  EXPECT_NEAR(static_cast<double>(count_four) / static_cast<double>(count_one), 1.0, kCountTol)
      << "counted rays: 1 worker " << count_one << ", 4 workers " << count_four;

  // Bounded: the major chains are the same set with the same shares, both directions.
  const std::vector<ChainShare> shares_one = SharesOf(one, total_one);
  const std::vector<ChainShare> shares_four = SharesOf(four, total_four);
  size_t major_checked = 0;
  const auto check_direction = [&](const std::vector<ChainShare>& a, const std::vector<ChainShare>& b,
                                   const char* a_name, const char* b_name) {
    for (const auto& sa : a) {
      if (sa.frac < kMajorFrac) {
        continue;  // rows are energy-descending, but the tail is what we skip, so keep scanning cheap
      }
      const ChainShare* sb = FindShare(b, sa.display);
      if (sb == nullptr) {
        ADD_FAILURE() << "chain " << sa.display << " holds " << sa.frac << " of the energy on " << a_name
                      << " and has no row on " << b_name;
        continue;
      }
      EXPECT_GE(sb->frac, sa.frac * 0.5) << "chain " << sa.display << ": " << a_name << " " << sa.frac << ", " << b_name
                                         << " " << sb->frac;
      EXPECT_NEAR(sb->frac / sa.frac, 1.0, kShareTol) << "chain " << sa.display << ": share " << sa.frac << " ("
                                                      << a_name << ") vs " << sb->frac << " (" << b_name << ")";
      major_checked++;
    }
  };
  check_direction(shares_one, shares_four, "1 worker", "4 workers");
  check_direction(shares_four, shares_one, "4 workers", "1 worker");
  // The 22-degree scene has eight ~3% chains and a dozen ~2% ones; a scene that yielded
  // no major chain at all would make the checks above vacuous.
  EXPECT_GE(major_checked, 8u) << "too few major chains to compare: " << major_checked;
}

// ---------------------------------------------------------------------------
// The standing analysis pool of a GPU-preferred server — four propositions, all
// GPU-gated the way AnalysisForcesCpuUnderGpuPreference is (the pool only exists on the
// GPU route, and the route only resolves where a GPU backend does):
//   PoolIsUsedUnderGpuPreference — existence: with num_workers=4 the analysis advances
//        its ray count at least twice as fast as with num_workers=1 over the same window,
//        which a single engine Simulator with CPU forced (the shape before the pool) could
//        not do; while it runs the lifecycle reads kRunning (GetStatus polls the pool, not
//        the idle engine); and the render after it is GPU again (the reset reached the
//        group that carried the analysis properties).
//   FixedSeedCollapsesThePoolToOneWorker — the deterministic contract: num_workers=4 with
//        a seed still reproduces bit for bit across sessions, and equals the CPU-route
//        server's own fixed-seed analysis of the same scene (both trace on one worker
//        seeded sim_seed), so the backend toggle does not change a seeded analysis.
//   AlternatingRenderAndAnalysisNeitherDeadlocksNorLeaks — the wake-up selection under the
//        two switch shapes that matter, five times over, every wait bounded: a render
//        submitted over a running analysis is refused and the analysis keeps running; a
//        render Stop()ped and an analysis started at once reaches its first rays; and no
//        frame of either session kind carries the other kind's result.
//   DestructionDuringAnalysisTerminates — the dormant engine thread and the busy pool
//        threads both leave on kTerminating: the wait predicate checks termination ahead
//        of the group filter, and a mistake there would hang the dtor's join.
// ---------------------------------------------------------------------------
#if defined(__APPLE__) || defined(LUMICE_CUDA_ENABLED)
#if defined(__APPLE__)
constexpr BackendKind kGpuBackend = BackendKind::kMetal;
#else
constexpr BackendKind kGpuBackend = BackendKind::kCuda;
#endif

bool GpuRouteAvailable() {
  Logger probe{ "ServerAnalysisRunGpuPool" };
  return ResolveGpuRoute(kGpuBackend, probe);
}

// Rays traced by an unbounded analysis over `window` after its first rays arrive.
size_t AnalysisRaysOverWindow(Server& server, const nlohmann::json& scene, std::chrono::milliseconds window) {
  EXPECT_FALSE(server.StartRaypathAnalysis(scene, FullSkyRequest()));
  EXPECT_TRUE(WaitForFirstRays(server, 10000));
  EXPECT_EQ(server.GetSimLifecycle(), SimLifecycle::kRunning) << "an analysis in flight must read as running";
  const size_t before = server.GetLiveSimRayCount();
  std::this_thread::sleep_for(window);
  const size_t after = server.GetLiveSimRayCount();
  server.Stop();
  return after - before;
}
#endif

TEST(ServerAnalysisRunGpuPool, PoolIsUsedUnderGpuPreference) {
#if defined(__APPLE__) || defined(LUMICE_CUDA_ENABLED)
  if (!GpuRouteAvailable()) {
    GTEST_SKIP() << "GPU route unavailable on this machine (or overridden by LUMICE_TRACE_BACKEND)";
  }
  if (PhysicalCoreCount() < 4) {
    GTEST_SKIP() << "fewer than 4 physical cores; a 4-worker pool cannot show a rate gain here";
  }
  const nlohmann::json scene = Halo22Config("infinite");
  constexpr auto kWindow = std::chrono::milliseconds(1500);
  // One worker first, then four, on separate servers: the pool is a construction-time
  // property. An existence threshold, not a performance figure: four pool workers measured
  // 3.4x, 4.3x and 5.0x one over this window on a 12-core development machine (2026-09-13);
  // 2x is what separates "the pool ran" from "one worker ran" with room for a loaded box.
  size_t rays_one = 0;
  {
    Server server(1, 0, kGpuBackend);
    rays_one = AnalysisRaysOverWindow(server, scene, kWindow);
  }
  ASSERT_GT(rays_one, 0u);
  Server server(4, 0, kGpuBackend);
  const size_t rays_four = AnalysisRaysOverWindow(server, scene, kWindow);
  EXPECT_GE(rays_four, rays_one * 2) << "1 worker traced " << rays_one << " rays over the window, 4 workers "
                                     << rays_four << ": the pool did not run";

  // The withdrawal of the analysis properties reached the pool: the render after it is
  // GPU, and its frame carries no histogram.
  ASSERT_FALSE(server.CommitConfig(Halo22Config(20000)));
  ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  EXPECT_EQ(server.GetActiveBackend(), kGpuBackend);
  EXPECT_FALSE(server.BackendFellBack());
  EXPECT_FALSE(server.AcquireResultFrame()->raypath_histogram_result_.has_value());
  server.Stop();
#else
  GTEST_SKIP() << "no GPU backend in this build; there is no standing pool on the CPU route";
#endif
}

TEST(ServerAnalysisRunGpuPool, FixedSeedCollapsesThePoolToOneWorker) {
#if defined(__APPLE__) || defined(LUMICE_CUDA_ENABLED)
  if (!GpuRouteAvailable()) {
    GTEST_SKIP() << "GPU route unavailable on this machine (or overridden by LUMICE_TRACE_BACKEND)";
  }
  constexpr uint32_t kSeed = 1;
  const nlohmann::json scene = Halo22Config(20000);
  // num_workers=4 asks for a four-worker pool; the seed must win (one worker), or the
  // sessions below diverge — the merge order of four producers is not deterministic.
  Server server(4, kSeed, kGpuBackend);
  const AnalysisFingerprint first = RunAnalysisAndFingerprint(server, scene);
  ASSERT_FALSE(first.rows.empty());
  ASSERT_EQ(first.sim_ray_num, 20000u);
  ASSERT_FALSE(server.CommitConfig(scene));  // a GPU render in between, then Stop()
  ASSERT_EQ(WaitForRunToEnd(server, 30000), SimLifecycle::kCompleted);
  server.Stop();
  const AnalysisFingerprint after_render = RunAnalysisAndFingerprint(server, scene);
  const AnalysisFingerprint after_analysis = RunAnalysisAndFingerprint(server, scene);
  EXPECT_TRUE(after_render == first) << "session 2 (after a GPU render + Stop) differs from session 1";
  EXPECT_TRUE(after_analysis == first) << "session 3 (after an analysis) differs from session 1";
  server.Stop();

  // And the same as the CPU-route server's: the pool's one worker and the CPU route's one
  // worker both hold sim_seed, so a seeded analysis does not depend on the backend toggle.
  Server cpu_server(0, kSeed, BackendKind::kCpu);
  const AnalysisFingerprint cpu = RunAnalysisAndFingerprint(cpu_server, scene);
  EXPECT_TRUE(cpu == first) << "the GPU-preferred server's seeded analysis differs from the CPU-preferred server's";
  cpu_server.Stop();
#else
  GTEST_SKIP() << "no GPU backend in this build";
#endif
}

TEST(ServerAnalysisRunGpuPool, AlternatingRenderAndAnalysisNeitherDeadlocksNorLeaks) {
#if defined(__APPLE__) || defined(LUMICE_CUDA_ENABLED)
  if (!GpuRouteAvailable()) {
    GTEST_SKIP() << "GPU route unavailable on this machine (or overridden by LUMICE_TRACE_BACKEND)";
  }
  const nlohmann::json unbounded = Halo22Config("infinite");
  Server server(4, 0, kGpuBackend);
  // One round, as a callable: every step's precondition is the step before it, so a
  // fatal assert must end the round (it returns from this lambda) — and the rounds are
  // not independent rows either, each starts from the Stop()ped state the last one left,
  // so a failed round ends the loop below explicitly rather than driving four more rounds
  // off a state that is not their precondition.
  const auto run_round = [&](int round) {
    SCOPED_TRACE("round " + std::to_string(round));
    // Shape 1: a render submitted over a running analysis. Refused, and the analysis is
    // untouched — still running, still counting.
    ASSERT_FALSE(server.StartRaypathAnalysis(unbounded, FullSkyRequest()));
    ASSERT_TRUE(WaitForFirstRays(server, 10000)) << "the analysis never reached its first rays";
    ASSERT_EQ(server.GetSimLifecycle(), SimLifecycle::kRunning);
    ASSERT_EQ(server.GetSessionKind(), SessionKind::kAnalysis);
    EXPECT_TRUE(server.CommitConfig(unbounded)) << "a commit over a running analysis must be refused";
    ASSERT_EQ(server.GetSessionKind(), SessionKind::kAnalysis);
    const size_t before = server.GetLiveSimRayCount();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_GT(server.GetLiveSimRayCount(), before) << "the refused commit stalled the analysis";
    EXPECT_EQ(server.GetActiveBackend(), BackendKind::kCpu);
    {
      auto frame = server.AcquireResultFrame();
      EXPECT_TRUE(frame->raypath_histogram_result_.has_value());
      EXPECT_TRUE(frame->render_results_.empty()) << "a render result inside an analysis session";
    }
    server.Stop();
    ASSERT_NE(server.GetSimLifecycle(), SimLifecycle::kRunning) << "Stop() returned with the analysis running";

    // Shape 2: a render, Stop()ped mid-flight, and an analysis started at once. The
    // engine must let go and the pool must take over, with no batch of the render in
    // the analysis's frame.
    ASSERT_FALSE(server.CommitConfig(unbounded));
    ASSERT_TRUE(WaitForFirstRays(server, 10000)) << "the render never reached its first rays";
    ASSERT_EQ(server.GetSessionKind(), SessionKind::kRender);
    EXPECT_EQ(server.GetActiveBackend(), kGpuBackend);
    {
      auto frame = server.AcquireResultFrame();
      EXPECT_FALSE(frame->raypath_histogram_result_.has_value()) << "a histogram inside a render session";
    }
    server.Stop();
    ASSERT_NE(server.GetSimLifecycle(), SimLifecycle::kRunning) << "Stop() returned with the render running";
    ASSERT_FALSE(server.StartRaypathAnalysis(unbounded, FullSkyRequest()));
    ASSERT_TRUE(WaitForFirstRays(server, 10000)) << "the analysis after a stopped render never started";
    {
      auto frame = server.AcquireResultFrame();
      EXPECT_TRUE(frame->raypath_histogram_result_.has_value());
      EXPECT_TRUE(frame->render_results_.empty()) << "the stopped render's result leaked into the analysis";
    }
    server.Stop();
    ASSERT_NE(server.GetSimLifecycle(), SimLifecycle::kRunning);
  };
  for (int round = 0; round < 5; round++) {
    run_round(round);
    if (::testing::Test::HasFailure()) {
      break;
    }
  }
#else
  GTEST_SKIP() << "no GPU backend in this build";
#endif
}

TEST(ServerAnalysisRunGpuPool, DestructionDuringAnalysisTerminates) {
#if defined(__APPLE__) || defined(LUMICE_CUDA_ENABLED)
  if (!GpuRouteAvailable()) {
    GTEST_SKIP() << "GPU route unavailable on this machine (or overridden by LUMICE_TRACE_BACKEND)";
  }
  auto server = std::make_unique<Server>(4, 0, kGpuBackend);
  ASSERT_FALSE(server->StartRaypathAnalysis(Halo22Config("infinite"), FullSkyRequest()));
  ASSERT_TRUE(WaitForFirstRays(*server, 10000));
  // Destroy from another thread so a hang is a bounded failure here, not a silent one at
  // the process's exit. A dtor that never returns leaves that thread blocked; the test
  // reports it and the process-level timeout finishes the job.
  std::atomic_bool destroyed{ false };
  std::thread destroyer([&] {
    server.reset();
    destroyed.store(true);
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!destroyed.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(destroyed.load()) << "~Server() did not return within 20 s with an analysis in flight";
  if (destroyed.load()) {
    destroyer.join();
  } else {
    destroyer.detach();
  }
#else
  GTEST_SKIP() << "no GPU backend in this build";
#endif
}

}  // namespace
}  // namespace lumice
