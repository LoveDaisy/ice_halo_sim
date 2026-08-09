// Composition chain: the simulation run lifecycle.
//
// Units in the chain: app × server_poller × gui_state.
//
// What the collaboration produces that is observable: what the user is told the simulation is
// doing, and which frame's pixels and numbers are on screen while it does it. Four of the six
// escaped defects this rebuild was told to pin live on this chain, and they share a shape — every
// one of them looked perfectly healthy on screen. A stale ray count is a plausible ray count. A
// preview holding the previous document's pixels is a preview. "Simulating" forever is indis-
// tinguishable from a slow simulation. None of them announce themselves, which is why the guard has
// to be an assertion rather than a look.
//
// Derived from the src call graph: app.cpp -> server_poller is 10 call sites, and app.hpp exposes
// the decision points of that collaboration as free predicates precisely so they can be driven
// without a window, a GL context or a live server.

#include <gtest/gtest.h>

#include <string>

#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "gui/server_poller.hpp"
#include "include/lumice.h"

namespace lumice::gui {
namespace {

PreviewSnapshot MakeSnapshot(unsigned long long epoch, int lifecycle) {
  PreviewSnapshot snap;
  snap.valid = true;
  snap.epoch = epoch;
  snap.lifecycle = lifecycle;
  return snap;
}

// ---------------------------------------------------------------------------------------------
// E8 — a restart must not put the previous run's numbers back on the status bar.
//
// The server's cached statistics survive a Stop: only the dirty/consumed flags are cleared. The
// bundle epoch, meanwhile, advances the moment a new generation is committed. So a carried-forward
// value and a freshly produced one are the same object unless the stats carry their OWN generation
// stamp — which is why ShouldApplyStats keys on stats_epoch and not on the bundle epoch.
//
// The user-visible failure has no tell: press Stop, press Run, and the status bar immediately shows
// a ray count. It is the last run's. Nothing about it looks stale.

struct StatsCase {
  const char* name;
  unsigned long long bundle_epoch;
  unsigned long long stats_epoch;
  LUMICE_RayCount rays;
  uint64_t committed_epoch;
  bool expect_applied;
};

TEST(RunLifecycleChain, StatsApplyKeysOnTheStatsOwnEpochNotTheBundleEpoch) {
  const StatsCase kCases[] = {
    { "fresh stats for the committed epoch", 7, 7, 1000, 7, true },
    // The regression itself: the poll happened during epoch 7, so the bundle says 7, but the
    // numbers were produced under epoch 6 and merely survived the restart.
    { "carried-forward stats under a newer bundle", 7, 6, 1000, 7, false },
    { "stats from a generation ahead of what the GUI committed", 7, 8, 1000, 7, false },
    // The zero lower bound is a separate rule and stays: a zero must not overwrite a number
    // already on screen, even when its epoch is impeccable.
    { "fresh but empty stats", 7, 7, 0, 7, false },
  };

  for (const StatsCase& c : kCases) {
    PreviewSnapshot snap = MakeSnapshot(c.bundle_epoch, LUMICE_LIFECYCLE_RUNNING);
    snap.stats_epoch = c.stats_epoch;
    snap.stats_sim_ray_num = c.rays;
    snap.stats_ray_seg_num = c.rays;
    EXPECT_EQ(ShouldApplyStats(snap, c.committed_epoch), c.expect_applied) << c.name;
  }
}

// The property behind the table, stated so a future change to the gate cannot satisfy the rows
// above by accident: two snapshots identical except for stats_epoch must decide differently. If
// they ever decide the same, the stamp has stopped being load-bearing whatever the table says.
TEST(RunLifecycleChain, StatsEpochIsTheOnlyFieldThatDecidesTheCarriedForwardCase) {
  PreviewSnapshot fresh = MakeSnapshot(9, LUMICE_LIFECYCLE_RUNNING);
  fresh.stats_epoch = 9;
  fresh.stats_sim_ray_num = 4242;
  fresh.stats_ray_seg_num = 4242;

  PreviewSnapshot carried = fresh;
  carried.stats_epoch = 8;

  EXPECT_TRUE(ShouldApplyStats(fresh, 9));
  EXPECT_FALSE(ShouldApplyStats(carried, 9));
}

// ---------------------------------------------------------------------------------------------
// E10 — a document switch fences off the previous document's pixels.
//
// display_epoch_floor is what New/Open raise so that a texture already in flight for the old
// generation cannot land on the new document's canvas. This is the defect that came back three
// times: the user opens a different file and the preview still holds the previous one's image.
//
// Serial dedup and the epoch floor are two different rules and both have to hold. Dedup alone lets
// a stale-generation payload through on its first sighting; the floor alone re-uploads the same
// payload every frame.

struct UploadCase {
  const char* name;
  unsigned long long payload_epoch;
  unsigned long long texture_serial;
  unsigned long long last_uploaded_serial;
  uint64_t display_epoch_floor;
  bool has_payload;
  float snapshot_intensity;
  LUMICE_RayCount texture_ray_count;
  bool expect_upload;
};

TEST(RunLifecycleChain, PayloadUploadNeedsAFreshSerialAnEpochAboveTheFloorAndContent) {
  const UploadCase kCases[] = {
    { "fresh payload above the floor", 5, 11, 10, 4, true, 1.0f, 100, true },
    { "already uploaded this serial", 5, 10, 10, 4, true, 1.0f, 100, false },
    // The stale-preview defect: a brand-new serial, so dedup waves it through, carrying pixels
    // produced before the document switch raised the floor.
    { "fresh serial but stale generation", 3, 11, 10, 4, true, 1.0f, 100, false },
    { "payload exactly at the floor", 4, 11, 10, 4, true, 1.0f, 100, false },
    { "no payload at all", 5, 11, 10, 4, false, 1.0f, 100, false },
    // A cold-start frame with no content is skipped so GL keeps showing the last real frame
    // rather than flashing black. Either signal of content is enough on its own: a terminal
    // frame can legitimately have zero intensity while still carrying rays.
    { "empty frame, no intensity and no rays", 5, 11, 10, 4, true, 0.0f, 0, false },
    { "zero intensity but rays present", 5, 11, 10, 4, true, 0.0f, 100, true },
  };

  for (const UploadCase& c : kCases) {
    PreviewSnapshot snap = MakeSnapshot(c.payload_epoch, LUMICE_LIFECYCLE_RUNNING);
    snap.texture_serial = c.texture_serial;
    if (c.has_payload) {
      auto payload = std::make_shared<TexturePayload>();
      payload->payload_epoch = c.payload_epoch;
      payload->width = 4;
      payload->height = 4;
      payload->snapshot_intensity = c.snapshot_intensity;
      payload->texture_ray_count = c.texture_ray_count;
      snap.payload = payload;
    }
    EXPECT_EQ(ShouldUploadPayload(snap, c.last_uploaded_serial, c.display_epoch_floor), c.expect_upload) << c.name;
  }
}

// ---------------------------------------------------------------------------------------------
// E9 — the poller may only stop polling once the terminal truth can no longer change.
//
// Stopping early is how a finite run ends with a blank preview: the completion edge arrives, the
// poller pauses, and the last frame never reaches the screen. The predicate's three conditions are
// not one condition written three ways, and each of the last two has its own way of failing.

struct SelfPauseCase {
  const char* name;
  int lifecycle;
  unsigned long long lc_epoch;
  unsigned long long drained_epoch;
  unsigned long long current_epoch;
  bool expect_pause;
};

TEST(RunLifecycleChain, SelfPauseNeedsCompletionAndADrainedUnsupersededEpoch) {
  const SelfPauseCase kCases[] = {
    { "completed, drained, nothing newer", LUMICE_LIFECYCLE_COMPLETED, 5, 5, 5, true },
    // An infinite run never completes; pausing here would freeze the preview mid-run.
    { "still running", LUMICE_LIFECYCLE_RUNNING, 5, 5, 5, false },
    // The producer says done while the consumer still has a queue. Pausing now freezes a partial
    // total on the status bar with no later poll to correct it.
    { "completed but the consumer has not drained", LUMICE_LIFECYCLE_COMPLETED, 5, 4, 5, false },
    // A commit landed while this poll was in flight. drained_epoch is monotonic, so condition 2
    // is still satisfied and only this one catches it.
    { "completed but a newer epoch was committed mid-poll", LUMICE_LIFECYCLE_COMPLETED, 5, 5, 6, false },
    // A transient restart IDLE is not a completion edge.
    { "idle", LUMICE_LIFECYCLE_IDLE, 5, 5, 5, false },
  };

  for (const SelfPauseCase& c : kCases) {
    LUMICE_SimLifecycleResult lc{};
    lc.lifecycle = static_cast<LUMICE_SimLifecycle>(c.lifecycle);
    lc.epoch = c.lc_epoch;
    LUMICE_DrainResult drain{};
    drain.drained_epoch = c.drained_epoch;
    drain.current_epoch = c.current_epoch;
    EXPECT_EQ(ShouldSelfPause(lc, drain), c.expect_pause) << c.name;
  }
}

// ---------------------------------------------------------------------------------------------
// E7 — the state the user reads off the screen is a function of intent, epoch and the last
// observation, and of nothing else.
//
// ReconcileSimState is the single writer of sim_state. A second writer is how "Simulating" outlives
// the simulation: some display-time action pokes the field, and the label never comes back. Driving
// the truth table directly is what keeps that single-writer claim checkable.

struct ReconcileCase {
  const char* name;
  RunIntent intent;
  uint64_t committed_epoch;
  bool have_snapshot;
  unsigned long long snap_epoch;
  int lifecycle;
  bool dirty;
  GuiState::SimState expected;
};

TEST(RunLifecycleChain, SimStateIsAFunctionOfIntentEpochAndObservation) {
  const ReconcileCase kCases[] = {
    { "never run", RunIntent::kNone, 0, false, 0, LUMICE_LIFECYCLE_IDLE, false, GuiState::SimState::kIdle },
    { "running, no observation yet", RunIntent::kRunning, 3, false, 0, LUMICE_LIFECYCLE_IDLE, false,
      GuiState::SimState::kSimulating },
    { "running, backend agrees", RunIntent::kRunning, 3, true, 3, LUMICE_LIFECYCLE_RUNNING, false,
      GuiState::SimState::kSimulating },
    // The completion edge for the epoch the GUI actually committed. This is the transition whose
    // absence reads as "it finished but the UI still says Simulating".
    { "completed at the committed epoch", RunIntent::kRunning, 3, true, 3, LUMICE_LIFECYCLE_COMPLETED, false,
      GuiState::SimState::kDone },
    // A completion belonging to a previous generation must not end the current run.
    { "completed at an older epoch", RunIntent::kRunning, 4, true, 3, LUMICE_LIFECYCLE_COMPLETED, false,
      GuiState::SimState::kSimulating },
    { "stopping", RunIntent::kStopping, 3, true, 3, LUMICE_LIFECYCLE_RUNNING, false, GuiState::SimState::kStopping },
  };

  for (const ReconcileCase& c : kCases) {
    PreviewSnapshot snap = MakeSnapshot(c.snap_epoch, c.lifecycle);
    const PreviewSnapshot* snap_ptr = c.have_snapshot ? &snap : nullptr;
    EXPECT_EQ(ReconcileSimState(c.intent, c.committed_epoch, snap_ptr, c.dirty), c.expected) << c.name;
  }
}

// A dirty edit demotes a FINISHED result and nothing else.
//
// The asymmetry is the point, and it is easy to get backwards: "the config changed" is only news
// about a result that exists. Demoting kSimulating would tell the user their running simulation is
// already invalid; demoting kIdle would put a "modified" badge on a document that has never run.
TEST(RunLifecycleChain, DirtyDemotesAFinishedResultAndOnlyAFinishedResult) {
  PreviewSnapshot running = MakeSnapshot(3, LUMICE_LIFECYCLE_RUNNING);
  PreviewSnapshot completed = MakeSnapshot(3, LUMICE_LIFECYCLE_COMPLETED);

  EXPECT_EQ(ReconcileSimState(RunIntent::kRunCompleted, 3, &completed, true), GuiState::SimState::kModified);
  EXPECT_EQ(ReconcileSimState(RunIntent::kLoaded, 3, nullptr, true), GuiState::SimState::kModified);

  EXPECT_EQ(ReconcileSimState(RunIntent::kNone, 3, &running, true), GuiState::SimState::kIdle);
  EXPECT_EQ(ReconcileSimState(RunIntent::kRunning, 3, &running, true), GuiState::SimState::kSimulating);
  EXPECT_EQ(ReconcileSimState(RunIntent::kStopping, 3, &running, true), GuiState::SimState::kStopping);
}

// ---------------------------------------------------------------------------------------------
// E10 (display-time half) — flipping the composite preview switch produces no new poller snapshot,
// so the upload branch has to fire on the display-mode change alone. Without that OR-branch the
// user toggles the switch and nothing happens until the next simulation batch lands, which on a
// finished run is never.

TEST(RunLifecycleChain, CompositeUploadFiresOnADisplayFlipThatProducesNoNewSnapshot) {
  PreviewSnapshot snap = MakeSnapshot(5, LUMICE_LIFECYCLE_COMPLETED);
  snap.texture_serial = 10;
  auto payload = std::make_shared<TexturePayload>();
  payload->payload_epoch = 5;
  payload->width = 4;
  payload->height = 4;
  payload->is_composite = true;
  snap.payload = payload;

  // Serial already consumed and no new generation: the standard gate says nothing to do.
  EXPECT_FALSE(ShouldUploadPayload(snap, 10, 4));
  // But the last upload was the non-composite view and the user has just asked for the composite
  // one, so the branch must still fire.
  EXPECT_TRUE(ShouldFireCompositeUpload(snap, 10, 4, /*show_composite_preview=*/true,
                                        /*last_uploaded_as_composite=*/false));
  // And must NOT fire when the screen already shows what was asked for — otherwise it re-uploads
  // every frame for the rest of the session.
  EXPECT_FALSE(ShouldFireCompositeUpload(snap, 10, 4, /*show_composite_preview=*/true,
                                         /*last_uploaded_as_composite=*/true));
}

TEST(RunLifecycleChain, CompositeUploadModeFoldsServerAvailabilityWithUserPreference) {
  struct Case {
    bool payload_is_composite;
    bool show_composite_preview;
    bool expected;
  };
  const Case kCases[] = {
    { true, true, true },
    // Asked for, not produced: falling back to the xyz view is the honest answer, not an empty one.
    { false, true, false },
    { true, false, false },
    { false, false, false },
  };
  for (const Case& c : kCases) {
    EXPECT_EQ(ShouldUseCompositeUpload(c.payload_is_composite, c.show_composite_preview), c.expected)
        << "payload_is_composite=" << c.payload_is_composite << " show_composite_preview=" << c.show_composite_preview;
  }
}

}  // namespace
}  // namespace lumice::gui
