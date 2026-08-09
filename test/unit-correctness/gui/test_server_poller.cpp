// What the poller publishes while a real simulation runs — see
// doc/gui-preview-lifecycle-architecture.md §2/§6.
//
// Every case here drives a REAL server and, where the property is about cadence, a real worker
// thread. That is faithful rather than expensive: the invariants are about the handoff between a
// producing backend and a consuming GUI, and a mock producer would be a second implementation of
// the thing under test rather than an oracle for it. The pure decisions these cases feed —
// ReconcileSimState, ShouldUploadPayload, ShouldApplyStats, ShouldSelfPause — are asserted over
// their whole domain in test/composition-correctness/gui/test_run_lifecycle_chain.cpp; what is
// pinned HERE is that PollOnce feeds them the right signals and publishes a coherent bundle.
//
// The cases that also need a rendered frame stay in test/gui/functional/test_gui_lifecycle.cpp.
//
// Blueprint invariants covered: I1 (epoch-keyed truth), I3 (level-triggered self-heal — the
// terminal edge is never lost), I4 (lifecycle and stats readable without an expensive snapshot
// generation), I5 (versioned immutable bundle), I6 (a terminal frame reaches the screen however
// sparse it is).

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "gui/server_poller.hpp"
#include "lumice.h"
#include "support/gui_live_server.hpp"

namespace gui = lumice::gui;

using lumice::test::CommitSceneJson;
using lumice::test::CurrentEpoch;
using lumice::test::kColorSceneJson;
using lumice::test::kExactTotalsSceneJson;
using lumice::test::kExactTotalsSimRays;
using lumice::test::kFiniteSceneJson;
using lumice::test::kInfiniteSceneJson;
using lumice::test::LiveServer;
using lumice::test::PixelFingerprint;
using lumice::test::WaitFor;
using lumice::test::WaitForDrained;
using lumice::test::WaitForValidData;

namespace {

using RunIntent = gui::RunIntent;
using SimState = gui::GuiState::SimState;
using Snapshot = std::shared_ptr<const gui::PreviewSnapshot>;

// Detach the process-global poller and server from whatever a previous case left behind, and reset
// the reconcile INPUTS — never sim_state, which is derived once per frame and has a single owner.
void DetachGlobals() {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;
  gui::g_state.run_intent = RunIntent::kNone;
  gui::g_state.committed_epoch = 0;
  gui::g_state.display_epoch_floor = 0;
  gui::g_state.dirty = false;
}

// A private poller that stops itself. Stop() before the server dies is not tidiness: the worker
// dereferences the server, so skipping it is a use-after-free rather than a leak.
class ScopedPoller {
 public:
  ScopedPoller() = default;
  ~ScopedPoller() { p_.Stop(); }
  ScopedPoller(const ScopedPoller&) = delete;
  ScopedPoller& operator=(const ScopedPoller&) = delete;

  gui::ServerPoller& get() { return p_; }
  gui::ServerPoller* operator->() { return &p_; }

  // Re-arm the generation tracker — what a resume does, minus the worker — and take one synchronous
  // poll, returning the bundle it published.
  Snapshot Repoll(LUMICE_Server* server) {
    p_.ResetGenerationForTest();
    p_.PollOnceForTest(server);
    return p_.LoadSnapshot();
  }

 private:
  gui::ServerPoller p_;
};

// Arm the quality gate strictly above the ray count this run actually achieved, so the next poll is
// one the gate MUST reject. Derived from the measurement rather than hard-coded: the achieved count
// is core/RNG-dependent and pinning it would make the case platform-dependent. Returns that count —
// a genuinely zero-ray run takes the cold-start bypass and would exercise nothing.
LUMICE_RayCount ArmGateAboveAchieved(gui::ServerPoller& poller, LUMICE_Server* server) {
  LUMICE_StatsResult stats{};
  lumice::test::ScopedResultFrame frame(server);
  LUMICE_FrameGetStats(frame.get(), &stats);
  poller.SetCalibratedThreshold(stats.sim_ray_num + 1);
  return stats.sim_ray_num;
}

// Stop the server while its consumer's snapshot buffer is left untouched. This is the state the two
// per-resume cases below need and it is not the same as COMPLETED: LUMICE_StopServer resets
// has_ever_consumed_, so GetSimLifecycle reports IDLE, while snapshot_generation_ deliberately
// survives ("NOT reset on Stop (poller resets its own tracker)" — server.cpp). Under IDLE the
// terminal-frame rescue is structurally off the table, which is what leaves the per-resume field
// under test as the only thing that can re-open the materialize branch.
void FreezeIdleWithData(LUMICE_Server* server) {
  LUMICE_StopServer(server);
  LUMICE_SimLifecycleResult lc{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
  ASSERT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_IDLE));
}

}  // namespace

// ---- The terminal completion edge survives a poll that carries no new generation (I3/I4) ----
//
// The original dead-state bug's exact interleaving: a mid-run texture drop, then a terminal poll
// carrying NO new snapshot generation. Two halves, and the defect could live in either — the poller
// must still publish COMPLETED on that poll, and the app's single-owner reconcile must still reach
// kDone from it.
//
// Driven through the REAL gui::g_state / gui::g_server_poller globals rather than a local GuiState,
// and that is faithful rather than lazy: SyncFromPoller() and ReconcileSimState()'s caller take no
// state argument, so "a single owner reconciles the real global once per frame" IS the design under
// test. Injecting a local would exercise a path production does not have.
TEST(ServerPoller, TerminalPollWithNoNewGenerationStillReachesDone) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  const unsigned long long done_epoch = CurrentEpoch(srv);
  {
    LUMICE_SimLifecycleResult lc{};
    EXPECT_EQ(LUMICE_GetSimLifecycle(srv, &lc), LUMICE_OK);
    EXPECT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    EXPECT_GE(lc.epoch, 1u);
  }

  // (A) The poller half. Poll A is generation-bearing; poll B sees the same generation, so
  // has_new_snapshot is false. Reverting the unconditional per-poll `next->lifecycle = lc.lifecycle`
  // carry (gating it on has_new_snapshot instead) makes b->lifecycle stale. Reads are
  // non-destructive atomic loads.
  {
    ScopedPoller local;
    Snapshot a = local.Repoll(srv);
    ASSERT_TRUE(a != nullptr);
    EXPECT_TRUE(a->valid);
    EXPECT_EQ(a->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    EXPECT_EQ(a->epoch, done_epoch);
    EXPECT_GT(a->stats_sim_ray_num, 0u);  // poll A genuinely carried the generation-bearing frame

    local->PollOnceForTest(srv);
    Snapshot b = local->LoadSnapshot();
    ASSERT_TRUE(b != nullptr);
    // I5 bundle coherence: lifecycle + epoch + stats carry FORWARD across a no-new-generation poll
    // instead of being torn. The terminal COMPLETED edge survives.
    EXPECT_TRUE(b->valid);
    EXPECT_EQ(b->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    EXPECT_EQ(b->epoch, done_epoch);
    EXPECT_GT(b->stats_sim_ray_num, 0u);
  }

  // (B) The app half, plus the two transients that must NOT be classified done. Same interleaving
  // each time; only the reconcile inputs differ. InvalidateStagedTexture() nulls the published
  // payload so the terminal SyncFromPoller takes no GL upload path — this runs on a worker thread
  // with no current GL context.
  //
  // Reverting ReconcileSimState to ignore COMPLETED, or inverting its epoch compare, leaves row 1
  // stuck at kSimulating (the original bug). Dropping the epoch check entirely turns row 2 green for
  // the wrong reason; treating IDLE as completion does the same to row 3.
  struct SyncCase {
    const char* name;
    unsigned long long committed_epoch;
    bool stop_first;  // return the server to IDLE at the same epoch
    SimState expected;
  };
  const SyncCase kCases[] = {
    { "terminal COMPLETED for the committed generation", done_epoch, false, SimState::kDone },
    // The GUI has committed a newer generation the server has not produced yet, while the poller
    // still observes the old generation's COMPLETED. `fresh` requires an epoch match.
    { "COMPLETED from a generation the GUI has moved past", done_epoch + 1, false, SimState::kSimulating },
    // Stop returns the server to IDLE at the SAME epoch, so only the lifecycle discriminates.
    { "IDLE transient at a matching epoch", 0, true, SimState::kSimulating },
  };

  for (const SyncCase& c : kCases) {
    if (c.stop_first) {
      LUMICE_StopServer(srv);
    }
    gui::g_state.run_intent = RunIntent::kRunning;
    gui::g_state.committed_epoch = c.stop_first ? CurrentEpoch(srv) : c.committed_epoch;
    gui::g_state.dirty = false;

    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(srv);  // poll A, generation-bearing
    gui::g_server_poller.InvalidateStagedTexture();
    gui::g_server_poller.PollOnceForTest(srv);  // poll B, terminal, no new generation

    gui::SyncFromPoller();
    EXPECT_EQ(gui::g_state.sim_state, c.expected) << c.name;
  }

  DetachGlobals();
}

// ---- I5: one published value, every field of it from the same generation ----
//
// The atomic no-torn-read core is construction-guaranteed (shared_ptr<const PreviewSnapshot> plus
// one atomic load/store) and no test can drive it red without deliberately breaking atomicity. What
// IS assertable is the producer-side bundle coherence that construction argument does not cover: on
// a generation-bearing poll, the payload's own stamp and the stats' own stamp agree with the bundle
// epoch. (On a carry-forward poll both deliberately LAG it — see the restart cases below.)
TEST(ServerPoller, AGenerationBearingPollPublishesOneCoherentBundle) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());
  const unsigned long long done_epoch = CurrentEpoch(srv);

  ScopedPoller local;
  Snapshot s = local.Repoll(srv);
  ASSERT_TRUE(s != nullptr);
  EXPECT_TRUE(s->valid);
  EXPECT_EQ(s->epoch, done_epoch);
  EXPECT_EQ(s->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_GT(s->stats_sim_ray_num, 0u);
  EXPECT_EQ(s->stats_epoch, done_epoch);
  EXPECT_TRUE(s->has_new_texture);     // this poll genuinely materialized a texture
  ASSERT_TRUE(s->payload != nullptr);  // ... so the payload rides in the SAME bundle
  EXPECT_EQ(s->payload->payload_epoch, done_epoch);
  EXPECT_NE(s->texture_serial, 0u);  // a fresh monotonic serial was minted for it

  // Two consecutive loads observe the same whole object: the consumer can never see a half-swapped
  // value, because there is no reconstruction between reads.
  EXPECT_EQ(local->LoadSnapshot().get(), s.get());
}

// ---- I6: a COMPLETED run below the quality gate still puts a frame on screen ----
//
// The gate is a RUNNING-time anti-flicker device only: once the run has completed there is no better
// frame coming, so however sparse the result is, it is the result. The regression it guards: the
// gate rejected the terminal snapshot AND the COMPLETED self-pause fired in the same call, after the
// gate — so the gate's timeout fallback could never be reached (a finite low-ray run completes in
// ~80ms) and the preview stayed blank forever. The real GUI calibrates the threshold to 3-4×10^4
// rays at startup, so any user-configured run below that never showed a picture at all.
//
// Phase A alone would pass against a WRONG fix keyed on poller-lifetime state (e.g.
// `texture_serial_ == 0`): only the first low-ray completion in the process would be rescued. B and
// C exercise the two production wake seams that must re-arm the rescue — WakeForRestart (a fresh
// commit) and WakeForRefresh (a display-time refresh of an already-completed run, the colour /
// composite-EV push path, which likewise buys exactly one more poll).
TEST(ServerPoller, ATerminalFrameBelowTheQualityGateIsStillUploadedOnEveryResume) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  // Constructed AFTER the run, not before: last_quality_pass_time_ starts at construction and the
  // gate's timeout fallback is measured from there. Constructing it first would let the sim's own
  // wall-clock spend that budget, and the fallback — not the rescue under test — would be what puts
  // the frame on screen.
  ScopedPoller local;
  ASSERT_GT(ArmGateAboveAchieved(local.get(), srv), 0u);

  Snapshot sa = local.Repoll(srv);
  ASSERT_TRUE(sa != nullptr);
  EXPECT_EQ(sa->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_TRUE(sa->payload != nullptr);  // the terminal frame reached the consumer despite the gate
  EXPECT_TRUE(sa->has_new_texture);     // ... and THIS poll is what materialized it
  ASSERT_NE(sa->texture_serial, 0u);    // ... under a fresh monotonic serial, the upload key
  unsigned long long prev_serial = sa->texture_serial;

  // Phase B re-runs the sim and resumes through WakeForRestart; phase C changes nothing about the
  // sim and resumes through WakeForRefresh. Each wake releases the worker thread, so Stop() fences
  // it before the synchronous poll — it may still have landed one poll inside that window, which is
  // why the assertion is "the serial advanced" (true whichever thread materialized it) rather than
  // has_new_texture (true only for the poll that did).
  struct ResumeCase {
    const char* name;
    bool rerun;  // false = display-time refresh of the same completed run
  };
  const ResumeCase kCases[] = {
    { "a fresh commit, resumed through WakeForRestart", true },
    { "a display-time refresh, resumed through WakeForRefresh", false },
  };

  for (const ResumeCase& c : kCases) {
    if (c.rerun) {
      ASSERT_TRUE(srv.Run()) << c.name;
      ASSERT_GT(ArmGateAboveAchieved(local.get(), srv), 0u) << c.name;
      local->WakeForRestart(srv);
    } else {
      local->WakeForRefresh(srv);
    }
    local->Stop();
    local->PollOnceForTest(srv);

    Snapshot s = local->LoadSnapshot();
    ASSERT_TRUE(s != nullptr) << c.name;
    EXPECT_EQ(s->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED)) << c.name;
    EXPECT_TRUE(s->payload != nullptr) << c.name;
    EXPECT_NE(s->texture_serial, prev_serial) << c.name;
    prev_serial = s->texture_serial;
  }
}

// ---- The two wake seams differ in exactly one published field ----
//
// WakeForRestart publishes valid=false on the kPaused→kRunning edge: a fresh commit means consumers
// must ignore the stale terminal snapshot. WakeForRefresh preserves it: a display-time refresh must
// not fabricate a transient valid=false window, because ReconcileSimState reads that window as
// "no observation yet" and pulls a completed sim back into kSimulating (activity-bug root cause (a),
// doc/gui-state-governance.md §4 支柱 2).
//
// The valid=false publish is a TRANSIENT and the observation of it races the worker, which the wake
// releases in the same call and which republishes valid=true on its first poll. So the two arms are
// not symmetric statements and must not be written as one equality:
//   - refresh is a universal claim — valid must hold at every observation, before and after the
//     worker settles — so repeating it only strengthens it;
//   - restart is an existential one — the edge value must be observable at all — so it is asserted
//     as "seen at least once" across the same repetitions.
// Measured, not assumed: inserting a 50ms sleep between the wake and the read turns the restart arm
// red every time, and the flat equality this replaces failed roughly once per twenty full-suite runs
// under a loaded machine. There is no seam that removes the race — TransitionToRunning notifies the
// worker before returning — so the shape above is the honest formulation rather than a workaround.
TEST(ServerPoller, WakeForRefreshPreservesValidWhereWakeForRestartClearsIt) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  constexpr int kAttempts = 32;
  int restart_cleared = 0;

  for (int attempt = 0; attempt < kAttempts; ++attempt) {
    for (bool restart : { true, false }) {
      // Identical baseline for both arms: a fresh valid=true terminal snapshot, worker paused.
      gui::g_server_poller.Stop();
      gui::g_server_poller.ResetGenerationForTest();
      gui::g_server_poller.PollOnceForTest(srv);
      Snapshot seed = gui::g_server_poller.LoadSnapshot();
      ASSERT_TRUE(seed != nullptr);
      ASSERT_TRUE(seed->valid) << "attempt " << attempt;  // premise: valid BEFORE the wake
      gui::g_server_poller.Stop();

      if (restart) {
        gui::g_server_poller.WakeForRestart(srv);
      } else {
        gui::g_server_poller.WakeForRefresh(srv);
      }
      Snapshot after = gui::g_server_poller.LoadSnapshot();
      ASSERT_TRUE(after != nullptr);
      if (restart) {
        restart_cleared += after->valid ? 0 : 1;
      } else {
        // No repetition of this one may ever see an invalid window: that window IS the defect.
        EXPECT_TRUE(after->valid) << "WakeForRefresh fabricated an invalid window on attempt " << attempt;
      }
    }
  }
  EXPECT_GT(restart_cleared, 0) << "WakeForRestart never published valid=false in " << kAttempts << " attempts";

  DetachGlobals();
}

// ---- The two per-resume fields the terminal-rescue case structurally cannot reach ----
//
// ResetPerResumeState() re-arms three fields at every kPaused→kRunning edge, and the rescue case
// above pins only one of them (`uploaded_since_resume_`). The other two are invisible to it, and not
// by accident: that case resumes against a COMPLETED run, where force_final_upload is true on the
// first post-resume poll, and that single flag both enters the materialize branch on its own
// (making `last_generation_ = 0` redundant) and bypasses the quality gate outright (making
// `last_quality_pass_time_`'s timeout unreachable). So under COMPLETED, one field MASKS its two
// siblings: delete either alone and every other case stays green — measured by deleting each line
// individually against the full suite, not assumed.
//
// The masking lifts in exactly one state, the IDLE-with-data freeze both cases below install. Both
// drive the production WakeForRefresh seam rather than the test seam, so they fail if the reset is
// dropped from the production path only.

// A snapshot the poller has ALREADY consumed must read as new again after a resume, or a
// display-time refresh re-materializes nothing — the server's generation counter did not move, and
// force_final_upload is structurally false under IDLE.
TEST(ServerPoller, ResumeRearmsTheGenerationTracker) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  ScopedPoller local;
  Snapshot s0 = local.Repoll(srv);  // consumes generation G
  ASSERT_TRUE(s0 != nullptr);
  ASSERT_TRUE(s0->payload != nullptr);
  const unsigned long long serial0 = s0->texture_serial;
  ASSERT_NE(serial0, 0u);

  FreezeIdleWithData(srv);

  // Control, and the "before" half of the red-state pair: without a resume a poll in this state
  // materializes nothing. It proves the assertion below is driven by the resume and not by the
  // server handing out a fresh generation on its own.
  local->PollOnceForTest(srv);
  EXPECT_EQ(local->LoadSnapshot()->texture_serial, serial0);

  local->WakeForRefresh(srv);
  local->Stop();  // fence the worker thread
  local->PollOnceForTest(srv);
  EXPECT_NE(local->LoadSnapshot()->texture_serial, serial0);  // RED if `last_generation_ = 0` is dropped
}

// The gate's timeout fallback must be measured from the RESUME, not from whenever the gate last
// passed. Without the reset, time the poller spent paused — a user reading the last result before
// hitting Run again, seconds routinely — is charged against the new run's budget, so its first
// sparse poll is force-uploaded and the anti-flicker gate is silently disarmed exactly when it is
// supposed to engage.
TEST(ServerPoller, ResumeRearmsTheQualityGateClock) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  ScopedPoller local;
  Snapshot s0 = local.Repoll(srv);
  ASSERT_TRUE(s0 != nullptr);
  ASSERT_TRUE(s0->payload != nullptr);
  const unsigned long long serial0 = s0->texture_serial;

  FreezeIdleWithData(srv);
  ASSERT_GT(ArmGateAboveAchieved(local.get(), srv), 0u);

  // Spend more than the whole timeout budget while PAUSED. A correct reset makes this irrelevant; a
  // missing one makes it decisive.
  std::this_thread::sleep_for(std::chrono::milliseconds(gui::kQualityGateTimeoutMs + 100));

  local->WakeForRefresh(srv);
  local->Stop();
  local->PollOnceForTest(srv);

  // The gate rejects, because the resume restarted the timeout clock and ~0ms of the budget is
  // spent. Drop `last_quality_pass_time_ = now()` and the paused time above is charged instead, the
  // timeout fires, and this sparse frame is force-uploaded under a fresh serial.
  EXPECT_EQ(local->LoadSnapshot()->texture_serial, serial0);
}

// ---- The restart window: a new generation number over the previous run's data ----
//
// Between a re-commit and the new run's first produced batch, the server hands out a pair that is
// "new generation number, old content" — in all three channels. Neither value is torn; each is
// complete and self-consistent. It is the PAIRING that is wrong, which is why the "no
// new-epoch-with-stale-data tear can occur" reasoning at the stamp site does not cover it:
//   - stats: ServerImpl::Stop resets snapshot_dirty_/has_ever_consumed_ but does NOT replace
//     published_frame_, which only DoSnapshot's StorePublished overwrites.
//   - pixels: RenderConsumer::Reset deliberately does not zero snapshot_xyz_ ("PrepareSnapshot will
//     memcpy over it"), and PrepareSnapshot only runs when DoSnapshot sees snapshot_dirty_, which
//     Stop cleared. So the buffer still holds the previous run's image byte for byte.
//   - epoch: RawXyzResult::epoch is stamped from the LIVE committed_epoch_ on every read, which the
//     commit already bumped.
// has_valid_data is the ONLY field that distinguishes the stale frame from fresh data, so freshness
// must be decided by it; the epoch stamp then records which generation the content was produced
// under and travels with it.
//
// The downstream epoch floor cannot separate the two: MarkStructHardDirty raises the floor to the
// OLD epoch precisely so that generation's frames are fenced out, but a payload MIS-stamped with the
// NEW epoch clears that floor exactly like a genuinely fresh frame would. The floor filters by
// generation NUMBER and has no way to ask whether the content was really produced under it. So the
// gate has to be upstream, where a payload is allowed to be constructed at all.
//
// Each case below holds the window open BY CONSTRUCTION rather than racing the new run's first
// batch: Stop() unconditionally clears has_ever_consumed_ (so has_valid_data reads 0 whether or not
// the new run got anything in first) and clears snapshot_dirty_ (so no DoSnapshot from here on can
// refresh the buffers). An earlier draft polled straight after the commit and asserted on whichever
// state it happened to find; that version PASSED against the un-fixed code, because the new run
// routinely produced fresh data before the poll and the case never entered the branch it existed to
// check. No sleep, no "at least one hit in N tries".

TEST(ServerPoller, TheRestartWindowDoesNotRepublishThePriorRunsStats) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  ScopedPoller local;
  const unsigned long long epoch_a = CurrentEpoch(srv);
  Snapshot sa = local.Repoll(srv);
  ASSERT_TRUE(sa != nullptr);
  ASSERT_GT(sa->stats_sim_ray_num, 0u);
  EXPECT_EQ(sa->stats_epoch, epoch_a);               // fresh stats are stamped with the generation that made them
  EXPECT_TRUE(gui::ShouldApplyStats(*sa, epoch_a));  // ... and are applied
  const LUMICE_RayCount rays_a = sa->stats_sim_ray_num;

  ASSERT_EQ(CommitSceneJson(srv, kFiniteSceneJson), LUMICE_OK);
  const unsigned long long epoch_b = CurrentEpoch(srv);
  ASSERT_NE(epoch_b, epoch_a);  // the restart really did mint a new generation
  LUMICE_StopServer(srv);

  Snapshot sb = local.Repoll(srv);
  ASSERT_TRUE(sb != nullptr);
  EXPECT_EQ(sb->epoch, epoch_b);             // the bundle epoch is the NEW generation's...
  EXPECT_EQ(sb->stats_sim_ray_num, rays_a);  // ... while the stats riding in it are still run A's,
  EXPECT_EQ(sb->stats_epoch, epoch_a);       // ... and say so.
  // So the gate keeps them off the status bar. Before the fix this combination read as fresh
  // (bundle epoch == committed epoch, rays > 0) and run A's counts were displayed under run B.
  EXPECT_FALSE(gui::ShouldApplyStats(*sb, epoch_b));
}

// The pixel channel's sibling. Asserts the CONTENT fingerprint, not only the stamp: a case that
// checked payload_epoch alone would pass for the wrong reason on any run where the timing happened
// not to line up.
TEST(ServerPoller, TheRestartWindowDoesNotRepublishThePriorRunsPixels) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  ScopedPoller local;
  const unsigned long long epoch_a = CurrentEpoch(srv);
  Snapshot sa = local.Repoll(srv);
  ASSERT_TRUE(sa != nullptr);
  ASSERT_TRUE(sa->payload != nullptr);
  EXPECT_EQ(sa->payload->payload_epoch, epoch_a);
  EXPECT_GT(sa->payload->texture_ray_count, 0u);
  const auto pixel_count = [](const gui::TexturePayload& p) {
    return static_cast<size_t>(p.width) * static_cast<size_t>(p.height) * 3;
  };
  const uint64_t pixels_a = PixelFingerprint(sa->payload->xyz_buffer, pixel_count(*sa->payload));
  const unsigned long long serial_a = sa->texture_serial;

  ASSERT_EQ(CommitSceneJson(srv, kFiniteSceneJson), LUMICE_OK);
  ASSERT_GT(CurrentEpoch(srv), epoch_a);
  LUMICE_StopServer(srv);

  Snapshot sb = local.Repoll(srv);
  ASSERT_TRUE(sb != nullptr);
  ASSERT_TRUE(sb->payload != nullptr);
  // The image in the window IS run A's in both worlds — that is the premise of the case, not the
  // defect. What must not happen is that image being re-published as run B's work.
  EXPECT_EQ(PixelFingerprint(sb->payload->xyz_buffer, pixel_count(*sb->payload)), pixels_a);
  // The defect's exact shape, all four RED before the fix: the poller materializes a "fresh" payload
  // out of run A's pixels, stamps it with the new epoch, and mints a new serial for it.
  EXPECT_EQ(sb->payload->payload_epoch, epoch_a);  // carried forward, keeping its own generation
  EXPECT_EQ(sb->texture_serial, serial_a);         // carry-forward keeps the serial (anti-flicker)
  EXPECT_FALSE(sb->has_new_texture);
  EXPECT_FALSE(gui::ShouldUploadPayload(*sb, serial_a, /*display_epoch_floor=*/epoch_a));

  // The suppression is scoped to the window, not permanent: once a generation actually produces
  // data, its frame reaches the screen normally.
  ASSERT_EQ(CommitSceneJson(srv, kFiniteSceneJson), LUMICE_OK);
  const unsigned long long epoch_c = CurrentEpoch(srv);
  ASSERT_TRUE(WaitForValidData(srv));
  Snapshot sc = local.Repoll(srv);
  ASSERT_TRUE(sc != nullptr);
  ASSERT_TRUE(sc->payload != nullptr);
  EXPECT_EQ(sc->payload->payload_epoch, epoch_c);
  EXPECT_NE(sc->texture_serial, serial_a);
  EXPECT_TRUE(sc->has_new_texture);
  EXPECT_TRUE(gui::ShouldUploadPayload(*sc, serial_a, /*display_epoch_floor=*/epoch_a));
}

// The composite channel's sibling — not a courtesy duplicate. The composite payload fields are
// assigned INSIDE the very same `if (quality_ok)` block that builds the XYZ payload, so whatever
// gates one gates the other, and both directions of that coupling matter: the window must not
// publish a composite built from the previous run's lanes, and the window ending must not leave the
// composite path suppressed. It also covers the fire-gate's mode_changed OR-branch, which reads
// snap.payload and could otherwise fire an upload on a carried-forward frame.
TEST(ServerPoller, TheRestartWindowDoesNotRepublishThePriorRunsComposite) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kColorSceneJson));

  ScopedPoller local;
  const unsigned long long epoch_a = CurrentEpoch(srv);
  Snapshot sa = local.Repoll(srv);
  ASSERT_TRUE(sa != nullptr);
  ASSERT_TRUE(sa->payload != nullptr);
  // If this fails the case is not exercising the composite path at all and the rest proves nothing.
  ASSERT_TRUE(sa->payload->is_composite);
  ASSERT_TRUE(sa->payload->rgb_buffer != nullptr);
  EXPECT_EQ(sa->payload->payload_epoch, epoch_a);
  const unsigned long long serial_a = sa->texture_serial;

  ASSERT_EQ(CommitSceneJson(srv, kColorSceneJson), LUMICE_OK);
  ASSERT_GT(CurrentEpoch(srv), epoch_a);
  LUMICE_StopServer(srv);

  Snapshot sb = local.Repoll(srv);
  ASSERT_TRUE(sb != nullptr);
  ASSERT_TRUE(sb->payload != nullptr);
  // Carried forward, so it stays composite and keeps run A's stamp: the window publishes no new
  // composite rather than one rebuilt from the previous generation's lanes.
  EXPECT_TRUE(sb->payload->is_composite);
  EXPECT_EQ(sb->payload->payload_epoch, epoch_a);
  EXPECT_EQ(sb->texture_serial, serial_a);
  // The fire gate stays quiet: serial dedup is false, and mode_changed is false because the
  // carried-forward payload is composite exactly as the last upload was. A payload ABSENT from this
  // branch is what would make mode_changed misfire, so it is asserted rather than assumed.
  EXPECT_FALSE(gui::ShouldFireCompositeUpload(*sb, serial_a, /*display_epoch_floor=*/epoch_a,
                                              /*show_composite_preview=*/true,
                                              /*last_uploaded_as_composite=*/true));

  ASSERT_EQ(CommitSceneJson(srv, kColorSceneJson), LUMICE_OK);
  const unsigned long long epoch_c = CurrentEpoch(srv);
  ASSERT_TRUE(WaitForValidData(srv));
  Snapshot sc = local.Repoll(srv);
  ASSERT_TRUE(sc != nullptr);
  ASSERT_TRUE(sc->payload != nullptr);
  EXPECT_TRUE(sc->payload->is_composite);
  EXPECT_TRUE(sc->payload->rgb_buffer != nullptr);
  EXPECT_EQ(sc->payload->payload_epoch, epoch_c);
  EXPECT_TRUE(gui::ShouldFireCompositeUpload(*sc, serial_a, /*display_epoch_floor=*/epoch_a,
                                             /*show_composite_preview=*/true,
                                             /*last_uploaded_as_composite=*/true));
}

// ---- Carry-forward stays a pure refcount bump ----
//
// The payload no longer owns its pixels: it holds a share of the result frame and points into it.
// That makes `next->payload = prev->payload` load-bearing in a way it was not before — anyone
// rewriting it to build a fresh payload out of the previous one would either re-copy every pixel
// (the cost this change removed) or carry buffer pointers without the frame that keeps them alive.
// Neither shows up in the pixel or epoch assertions above: a deep copy compares byte-identical. So
// pointer identity is the whole assertion here; it is red for a deep copy and green only for a
// shared pointer.
TEST(ServerPoller, CarryForwardReusesThePayloadObjectRatherThanCopyingIt) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  ScopedPoller local;
  Snapshot s1 = local.Repoll(srv);
  ASSERT_TRUE(s1 != nullptr);
  ASSERT_TRUE(s1->payload != nullptr);
  ASSERT_TRUE(s1->has_new_texture);

  // Same generation, and this resume has already put a frame on screen, so neither the new-snapshot
  // branch nor the terminal rescue can fire and the publish below MUST take the carry-forward
  // branch. No re-arm in between: that is what a resume does, and a resume is exactly what this case
  // must not have.
  local->PollOnceForTest(srv);
  Snapshot s2 = local->LoadSnapshot();
  ASSERT_TRUE(s2 != nullptr);
  ASSERT_TRUE(s2->payload != nullptr);
  EXPECT_FALSE(s2->has_new_texture);  // the carry-forward branch really was the one taken
  EXPECT_EQ(s2->payload.get(), s1->payload.get());
  EXPECT_EQ(s2->payload->xyz_buffer, s1->payload->xyz_buffer);
  EXPECT_EQ(s2->texture_serial, s1->texture_serial);
  // The frame share travels with the payload, so the pixels stay readable for as long as either
  // snapshot is held — which is what makes pointing at them instead of copying them safe.
  EXPECT_TRUE(s2->payload->frame != nullptr);
}

// ---- I4a: cheap stats are read unconditionally, not gated behind has_new_snapshot ----
//
// The bug: LUMICE_FrameGetStats used to sit inside `if (has_new_snapshot || force_final_upload)`,
// 160 lines from the unconditional lifecycle read the same invariant requires. A poll that
// legitimately sees BOTH disjuncts false — generation tracker already caught up, terminal rescue
// already spent — skipped the stats read outright and fell through to the carry-forward branch.
// When that poll is also the poller's FIRST EVER, there is no `prev` to carry from, so the bundle's
// stats stayed at their zero default even though the frame acquired a few lines above carries real,
// valid, non-zero data.
//
// Constructed through two test-only seams because natural traffic cannot reach "gate closed AND no
// prior publish" on one poller instance: the re-arm's 0-start always reads the first real generation
// as new, and any earlier successful poll would have populated `prev` and carried the correct stats
// forward on its own — masking exactly the defect this case exists to pin.
TEST(ServerPoller, StatsAreReadOnAFirstPollWithBothMaterializeDoorsClosed) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.Run());

  // Peek the server's current snapshot generation without touching any poller's own tracker.
  uint64_t generation = 0;
  {
    lumice::test::ScopedResultFrame frame(srv);
    ASSERT_EQ(frame.err(), LUMICE_OK);
    LUMICE_RawXyzResult xyz[2]{};
    LUMICE_FrameGetRawXyz(frame.get(), xyz, 1);
    ASSERT_TRUE(xyz[0].has_valid_data);
    generation = xyz[0].snapshot_generation;
  }
  ASSERT_NE(generation, 0u);

  ScopedPoller local;
  local->SetGenerationForTest(generation);     // close has_new_snapshot
  local->SetUploadedSinceResumeForTest(true);  // close force_final_upload
  local->PollOnceForTest(srv);                 // first poll on this instance: no `prev`

  Snapshot snap = local->LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  EXPECT_GT(snap->stats_sim_ray_num, 0u);  // RED on the pre-fix code: both doors closed and no prev
  EXPECT_EQ(snap->stats_epoch, CurrentEpoch(srv));
}

// ---- I3: the terminal observation is level-triggered, not a single chance ----
//
// The defect family: PollOnce used to stop the worker the instant it saw lifecycle==COMPLETED, and
// the justification on record ("the last published snapshot carries COMPLETED, so the reconcile
// still reaches kDone") covered only the sim_state projection. The same bundle also carries four
// stats counters, carried FORWARD when a poll produces nothing new — so a poll that observed
// COMPLETED before the CONSUMER had drained published a partial total and then stopped, with no
// later poll to correct it. COMPLETED is a producer-side verdict; it asks nothing about the
// consumer's queue.
//
// Both halves are required, since fixing either alone leaves the other live ("a more accurate guard
// that is still one-shot", or "a heartbeat behind a guard that still decides wrong"): the guard now
// also requires the drain signal, AND after self-pausing the worker keeps a slow heartbeat instead
// of falling silent, so a wrong belief is recoverable rather than terminal.
//
// What is NOT pinned here, stated rather than papered over: the COMPLETED-but-not-yet-drained
// combination cannot be constructed on demand. It is a race window with no injection seam and it
// does not reproduce on macOS/Metal (the CI symptom was Linux + Mesa). That row is covered as pure
// logic in the run-lifecycle chain; the end-to-end evidence for it is a containerized repro, not
// this file.

// The wiring test for the guard: the chain's truth table proves the decision is right, this proves
// PollOnce feeds it the real signals and that the bundle published under that decision carries the
// true totals rather than a partial sum. Deterministic because it waits for the drain signal first.
TEST(ServerPoller, APollAtTheDrainedMomentPublishesFinalTotals) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.get() != nullptr);
  ASSERT_EQ(CommitSceneJson(srv, kExactTotalsSceneJson), LUMICE_OK);
  ASSERT_TRUE(WaitForDrained(srv, 30000)) << "epoch never reported drained";

  LUMICE_SimLifecycleResult lc{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(srv, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));

  // An independent cross-check, not a re-read of what the poller believes: ask the server directly.
  // A case asserting only on the poller's own published bundle would pass just as happily if the
  // poller and the code under test shared the same wrong assumption.
  LUMICE_DrainResult drain{};
  ASSERT_EQ(LUMICE_GetDrainStatus(srv, &drain), LUMICE_OK);
  EXPECT_EQ(drain.drained_epoch, lc.epoch);
  EXPECT_EQ(drain.current_epoch, lc.epoch);
  EXPECT_TRUE(gui::ShouldSelfPause(lc, drain));

  ScopedPoller local;
  Snapshot snap = local.Repoll(srv);
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  EXPECT_EQ(snap->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  // Exact, not approximate: with scattering prob 0.0 the totals are the configured ray budget, and a
  // missed 128-ray dispatch grain is visible as an exact shortfall.
  EXPECT_EQ(snap->stats_sim_ray_num, kExactTotalsSimRays);
  EXPECT_EQ(snap->stats_orientation_num, kExactTotalsSimRays);
}

// The guard's other failure mode: always true. An unbounded run never completes and never drains, so
// the worker must stay in the full-speed loop and no heartbeat tick may ever fire. Runs the REAL
// worker thread, because "did the worker change cadence" is the property under test.
TEST(ServerPoller, AnUnboundedRunNeverSelfPauses) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.get() != nullptr);
  ASSERT_EQ(CommitSceneJson(srv, kInfiniteSceneJson), LUMICE_OK);

  ScopedPoller local;
  local->Start(srv);
  ASSERT_TRUE(WaitFor([&local] { return local->LoadSnapshot() != nullptr; }, 5000)) << "worker never polled";

  // Long enough that a self-pause anywhere in this span leaves at least two ticks behind.
  std::this_thread::sleep_for(std::chrono::milliseconds(3 * gui::kIdleHeartbeatIntervalMs));
  EXPECT_EQ(local->HeartbeatTickCountForTest(), 0u) << "an unbounded run self-paused";

  Snapshot snap = local->LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_NE(snap->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));

  local->Stop();
  LUMICE_StopServer(srv);
}

// After self-pausing, the worker keeps a slow heartbeat. Before this, WorkerLoop sat in an untimed
// wait and all five wake entry points were user actions — so the single poll that observed the
// terminal edge was the ONLY chance to get the terminal truth on screen. The heartbeat is what makes
// that observation retryable, which is the whole difference between a level-triggered reconciler and
// an edge-triggered one. It also asserts what a tick must NOT do: republish a texture. Each tick is
// a safe no-op or the preview re-uploads twice a second forever.
TEST(ServerPoller, TheIdleHeartbeatKeepsTickingAndRepublishesNothing) {
  DetachGlobals();
  LiveServer srv;
  ASSERT_TRUE(srv.get() != nullptr);
  ASSERT_EQ(CommitSceneJson(srv, kFiniteSceneJson), LUMICE_OK);

  ScopedPoller local;
  local->Start(srv);
  // The first tick is itself the evidence of a self-pause: only the heartbeat branch bumps this
  // counter, and only PollOnce's drain-aware guard can enter that state.
  ASSERT_TRUE(WaitFor([&local] { return local->HeartbeatTickCountForTest() >= 1; }, 30000))
      << "worker never self-paused into the heartbeat";

  Snapshot before = local->LoadSnapshot();
  ASSERT_TRUE(before != nullptr);
  const uint64_t ticks_before = local->HeartbeatTickCountForTest();

  std::this_thread::sleep_for(std::chrono::milliseconds(5 * gui::kIdleHeartbeatIntervalMs / 2));
  EXPECT_GE(local->HeartbeatTickCountForTest() - ticks_before, 2u) << "the heartbeat stopped after its first tick";

  Snapshot after = local->LoadSnapshot();
  ASSERT_TRUE(after != nullptr);
  EXPECT_TRUE(after->valid);
  EXPECT_EQ(after->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_EQ(after->stats_sim_ray_num, before->stats_sim_ray_num);
  EXPECT_EQ(after->texture_serial, before->texture_serial) << "a heartbeat tick bumped the texture serial";
}

// The safety property the heartbeat endangers. Stop()'s callers destroy the server on the next line
// — MaybeReconstructServerForBackend does Stop() then LUMICE_DestroyServer, DoStop does Stop() then
// LUMICE_StopServer. Before the heartbeat, "self-paused" and "not touching the server" were the same
// state and Stop() could early-return on it. They are now different facts, and if Stop() still
// early-returned, a tick landing after it returned would dereference a destroyed server.
//
// The sequence below IS that use-after-free. This repository has no sanitizer build, so the
// tick-count assertion is the signal that does not depend on one — and it is a genuine assertion
// rather than a hope that a crash shows up, since "no tick after Stop() returned" IS the contract.
// It is not, and does not claim to be, proof of memory safety.
TEST(ServerPoller, StopQuiescesTheHeartbeatBeforeReturning) {
  DetachGlobals();
  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  ASSERT_EQ(CommitSceneJson(server, kFiniteSceneJson), LUMICE_OK);

  ScopedPoller local;
  local->Start(server);
  // Establish the premise first, or "no further ticks" below is vacuously true for a worker that
  // never entered the heartbeat at all.
  ASSERT_TRUE(WaitFor([&local] { return local->HeartbeatTickCountForTest() >= 1; }, 30000))
      << "worker never self-paused into the heartbeat";

  local->Stop();
  // Read the baseline AFTER Stop() returns, not before. Stop() is allowed to let an in-flight tick
  // finish — that is what waiting on active_ means — so a before/after comparison would go red on
  // the legitimate case of a tick that started microseconds before the call.
  const uint64_t ticks_at_return = local->HeartbeatTickCountForTest();

  LUMICE_DestroyServer(server);  // the UAF window, if the contract were broken

  std::this_thread::sleep_for(std::chrono::milliseconds(2 * gui::kIdleHeartbeatIntervalMs));
  EXPECT_EQ(local->HeartbeatTickCountForTest(), ticks_at_return)
      << "a heartbeat tick ran after Stop() returned — its caller had already destroyed the server";
}
