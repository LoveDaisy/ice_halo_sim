// Contract guard for the completion/drain signal (LUMICE_GetDrainStatus).
//
// The defect it pins: the server's IDLE verdict is derived from producer-side
// predicates only, so a reader that treats IDLE as "the run is over" can freeze a
// PARTIAL accumulator total — measured in CI as orientation_num 19616 against an
// expected 20000, short by a whole number of dispatch grains. The e2e suite used to
// paper over this by polling until the statistics stopped changing; that heuristic is
// gone, and this file is what keeps the mechanism itself honest rather than relying on
// the e2e suite's stability as indirect evidence.
//
// Both directions are covered on purpose, because the two ways this signal can fail are
// opposite and a one-sided test catches only one of them:
//   - fires too early  -> DrainedEpochImpliesFinalTotals (totals must not move afterwards)
//   - never fires      -> the WaitForDrain timeout inside that same case
//   - always true      -> NotDrainedWhileRunning, NotDrainedAfterFreshCommit
//
// Deliberately driven through the public C API rather than ServerImpl internals: the
// contract being pinned is the one external readers consume, and reaching into the impl
// would let the test pass on a build where the API forwards the wrong thing.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include "lumice.h"

namespace {

// Small and fast — this file asserts on counters, not on physics. ray_num is the number
// that has to come back exactly: GenerateScene issues 128-ray dispatches, so 20000 is
// 156 full grains plus a 32-ray remainder, and any unconsumed grain shows up as a
// short total in exactly the shape the original defect had.
constexpr unsigned long long kExpectedSimRays = 20000;

const char* kConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 20000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }]
})";

// Same config with an unbounded ray budget: production never ends, so the drain signal
// must never fire. This is the "always true" trap's most direct counter-example.
const char* kInfiniteConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": "infinite",
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }]
})";

LUMICE_ErrorCode CommitJson(LUMICE_Server* server, const std::string& json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

LUMICE_DrainResult ReadDrain(LUMICE_Server* server) {
  LUMICE_DrainResult drain{};
  EXPECT_EQ(LUMICE_GetDrainStatus(server, &drain), LUMICE_OK);
  return drain;
}

bool IsDrained(const LUMICE_DrainResult& d) {
  return d.drained_epoch == d.current_epoch;
}

// Reads the accumulated statistics through a result frame — the same route the real
// readers take, so a stats bug in the frame path is not hidden by a private shortcut.
LUMICE_StatsResult ReadStats(LUMICE_Server* server) {
  LUMICE_StatsResult stats{};
  LUMICE_ResultFrame* frame = nullptr;
  EXPECT_EQ(LUMICE_AcquireResultFrame(server, &frame), LUMICE_OK);
  if (frame != nullptr) {
    EXPECT_EQ(LUMICE_FrameGetStats(frame, &stats), LUMICE_OK);
    LUMICE_ReleaseResultFrame(frame);
  }
  return stats;
}

bool WaitForDrain(LUMICE_Server* server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    if (IsDrained(ReadDrain(server))) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

class DrainContract : public ::testing::Test {
 protected:
  void SetUp() override {
    LUMICE_ServerConfig cfg{};
    cfg.num_workers = 1;
    server_ = LUMICE_CreateServerEx(&cfg);
    ASSERT_NE(server_, nullptr);
  }

  void TearDown() override {
    if (server_ != nullptr) {
      LUMICE_StopServer(server_);
      LUMICE_DestroyServer(server_);
      server_ = nullptr;
    }
  }

  LUMICE_Server* server_ = nullptr;
};

// THE contract: once the current epoch reports drained, the accumulated totals are final.
//
// Repeated over several epochs because the original failure was intermittent — a single
// round would be a coin flip on the very race this signal exists to close. Each round
// re-commits, which mints a new epoch and restarts the pipeline.
TEST_F(DrainContract, DrainedEpochImpliesFinalTotals) {
  constexpr int kRounds = 6;
  for (int round = 0; round < kRounds; round++) {
    ASSERT_EQ(CommitJson(server_, kConfig), LUMICE_OK) << "round " << round;
    ASSERT_TRUE(WaitForDrain(server_, 30000)) << "epoch never reported drained, round " << round;

    // Read immediately — no settling, no re-poll. That is the whole point: the signal is
    // supposed to make the first read after it the correct one.
    const LUMICE_StatsResult at_drain = ReadStats(server_);
    EXPECT_EQ(at_drain.sim_ray_num, kExpectedSimRays)
        << "partial total at the moment drain was reported, round " << round;
    EXPECT_GT(at_drain.orientation_num, 0u) << "round " << round;

    // Nothing may still be arriving. If the signal fired while batches were queued, the
    // consumer keeps accumulating and this second read differs — which is exactly the
    // shape the retired e2e settle-loop was papering over.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const LUMICE_StatsResult after = ReadStats(server_);
    EXPECT_EQ(after.sim_ray_num, at_drain.sim_ray_num) << "totals still growing after drain, round " << round;
    EXPECT_EQ(after.crystal_num, at_drain.crystal_num) << "totals still growing after drain, round " << round;
    EXPECT_EQ(after.orientation_num, at_drain.orientation_num) << "totals still growing after drain, round " << round;
  }
}

// Guards the "always reports drained" failure mode from the other side: an unbounded run
// produces forever, so the signal must stay false no matter how long it is sampled.
TEST_F(DrainContract, NotDrainedWhileRunning) {
  ASSERT_EQ(CommitJson(server_, kInfiniteConfig), LUMICE_OK);

  // Sample across a stretch long enough for many batches to be produced and consumed —
  // an implementation that publishes on "queue momentarily empty" trips here.
  bool saw_running = false;
  for (int i = 0; i < 100; i++) {
    LUMICE_SimLifecycleResult life{};
    ASSERT_EQ(LUMICE_GetSimLifecycle(server_, &life), LUMICE_OK);
    const LUMICE_DrainResult drain = ReadDrain(server_);
    if (life.lifecycle == LUMICE_LIFECYCLE_RUNNING) {
      saw_running = true;
      ASSERT_FALSE(IsDrained(drain)) << "reported drained while the run was still producing (sample " << i << ")";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(saw_running) << "never observed the infinite run in RUNNING — the case proved nothing";

  // And it must still not be drained after Stop: stopping discards the queue, so the
  // epoch was never drained clean and must not be reported as if it were.
  LUMICE_StopServer(server_);
  EXPECT_FALSE(IsDrained(ReadDrain(server_))) << "a stopped (discarded) epoch was reported drained";
}

// A freshly committed epoch starts undrained even though the previous one finished. This
// is the reset-free design: the new epoch outruns drained_epoch, so nothing has to
// remember to clear anything — and a stale "drained" from the previous run must not leak
// through as a verdict on the new one.
TEST_F(DrainContract, NotDrainedAfterFreshCommit) {
  ASSERT_EQ(CommitJson(server_, kConfig), LUMICE_OK);
  ASSERT_TRUE(WaitForDrain(server_, 30000));
  const LUMICE_DrainResult first = ReadDrain(server_);

  ASSERT_EQ(CommitJson(server_, kInfiniteConfig), LUMICE_OK);
  const LUMICE_DrainResult second = ReadDrain(server_);
  EXPECT_GT(second.current_epoch, first.current_epoch) << "commit did not mint a new epoch";
  EXPECT_FALSE(IsDrained(second)) << "the previous epoch's drain leaked into the new one";
}

// A narrower race than NotDrainedAfterFreshCommit above: that test waits for
// WaitForDrain to observe full settlement before re-committing, so by the time the
// second CommitConfig lands, the previous epoch's terminal PublishDrainedEpochIfSettled
// call has already returned and nothing is in flight. This test re-commits with no
// settling gap at all, so the second CommitConfig's Stop() can land while the first
// epoch is still mid-flight (queued, generating, or between clearing scene_gen_active_
// and publishing). Pins that the epoch bump never becomes visible as "drained" before
// CommitConfig returns — see PublishDrainedEpochIfSettled's doc comment (server.cpp,
// "THE OTHER DIRECTION") for why CommitConfig's Stop() barrier closes this window.
TEST_F(DrainContract, NotDrainedImmediatelyAfterRapidRecommit) {
  constexpr int kRounds = 20;
  for (int round = 0; round < kRounds; round++) {
    ASSERT_EQ(CommitJson(server_, kConfig), LUMICE_OK) << "round " << round;
    // No wait here on purpose — re-commit as fast as possible, potentially while the
    // previous epoch is still producing or consuming.
    ASSERT_EQ(CommitJson(server_, kConfig), LUMICE_OK) << "round " << round;
    const LUMICE_DrainResult right_after = ReadDrain(server_);
    EXPECT_FALSE(IsDrained(right_after)) << "epoch " << right_after.current_epoch
                                         << " falsely reported drained immediately after "
                                            "a rapid re-commit, round "
                                         << round;
  }
}

// Pre-commit state, asserted rather than left to chance: both counters are 0, so the
// equality test reads "drained". That is deliberate and documented — a server that has
// produced nothing has had all zero of it consumed — but it means the comparison only
// carries information once a commit has minted a non-zero epoch. Pinned here so the
// property is a decision on record instead of an accident nobody re-checks.
TEST_F(DrainContract, NullArgsAndFreshServer) {
  LUMICE_DrainResult drain{};
  EXPECT_EQ(LUMICE_GetDrainStatus(nullptr, &drain), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_GetDrainStatus(server_, nullptr), LUMICE_ERR_NULL_ARG);

  ASSERT_EQ(LUMICE_GetDrainStatus(server_, &drain), LUMICE_OK);
  EXPECT_EQ(drain.current_epoch, 0u) << "no commit yet, so no epoch has been minted";
  EXPECT_EQ(drain.drained_epoch, 0u);
}

}  // namespace
