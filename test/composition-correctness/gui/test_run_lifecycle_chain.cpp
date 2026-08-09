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
//
// This file owns those decisions over their whole domain. Its sibling,
// test/unit-correctness/gui/test_server_poller.cpp, owns the other half of the same chain: that a
// real poll against a real server feeds these predicates the right signals in the first place.

#include <gtest/gtest.h>

#include <memory>
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

  PreviewSnapshot fresh;
  for (const StatsCase& c : kCases) {
    PreviewSnapshot snap = MakeSnapshot(c.bundle_epoch, LUMICE_LIFECYCLE_RUNNING);
    snap.stats_epoch = c.stats_epoch;
    snap.stats_sim_ray_num = c.rays;
    snap.stats_ray_seg_num = c.rays;
    EXPECT_EQ(ShouldApplyStats(snap, c.committed_epoch), c.expect_applied) << c.name;
    if (c.expect_applied) {
      fresh = snap;
    }
  }

  // The property behind the table, stated so a future change to the gate cannot satisfy the rows
  // above by accident: two snapshots identical except for stats_epoch must decide differently. If
  // they ever decide the same, the stamp has stopped being load-bearing whatever the table says.
  PreviewSnapshot carried = fresh;
  carried.stats_epoch = fresh.stats_epoch - 1;
  EXPECT_TRUE(ShouldApplyStats(fresh, fresh.epoch));
  EXPECT_FALSE(ShouldApplyStats(carried, fresh.epoch));
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
    // Both epochs moved past the completion. Monotonicity means condition 2 stays satisfied here
    // too, so an implementation reading only drained_epoch pauses a generation that just started.
    { "completed but both drain numbers are a generation ahead", LUMICE_LIFECYCLE_COMPLETED, 5, 6, 6, false },
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
// E10 (edit-time half) — which kind of edit raises the fence, and what the user sees either way.
//
// The floor is raised by GuiState, read by the upload gate, and the pair of them is what decides
// between two behaviours the user can tell apart at a glance. A filter change invalidates the
// picture, so the old generation's frames must be fenced out and the canvas cleared immediately; a
// crystal scrub does not, so the last frame must stay up or the preview strobes black under the
// mouse. Neither half proves anything alone: MarkStructHardDirty raising the floor is only
// meaningful if the gate then rejects, and a gate that rejects is only correct if the other edit
// leaves the floor alone.

TEST(RunLifecycleChain, AStructuralEditFencesTheOldGenerationAndAScrubDoesNot) {
  constexpr unsigned long long kGen = 5;
  // A materialized payload from the OLD generation, non-empty so the content term is satisfied and
  // the epoch floor is the only thing that can block it.
  PreviewSnapshot old_snap = MakeSnapshot(kGen, LUMICE_LIFECYCLE_COMPLETED);
  auto old_payload = std::make_shared<TexturePayload>();
  old_payload->payload_epoch = kGen;
  old_payload->texture_ray_count = 100;
  old_payload->snapshot_intensity = 1.0f;
  old_snap.payload = old_payload;
  old_snap.texture_serial = 1;  // unseen; the cursor starts at 0

  struct EditCase {
    const char* name;
    bool structural;
    unsigned long long expect_floor;
    float expect_intensity;
    bool expect_old_frame_survives;
  };
  const EditCase kCases[] = {
    { "a filter change invalidates the picture", true, kGen, 0.0f, false },
    { "a crystal scrub does not", false, 0, 1.0f, true },
  };

  for (const EditCase& c : kCases) {
    GuiState st;
    st.committed_epoch = kGen;
    st.display_epoch_floor = 0;
    st.snapshot_intensity = 1.0f;
    if (c.structural) {
      st.MarkStructHardDirty();
    } else {
      st.MarkDirty();
    }
    EXPECT_EQ(st.display_epoch_floor, c.expect_floor) << c.name;
    EXPECT_EQ(st.snapshot_intensity, c.expect_intensity) << c.name;
    EXPECT_EQ(ShouldUploadPayload(old_snap, /*last_serial=*/0, st.display_epoch_floor), c.expect_old_frame_survives)
        << c.name;
  }

  // And the fence is scoped to the generation that raised it: the next generation's frame clears it,
  // exactly once. A floor that outlived its generation would freeze the preview permanently.
  PreviewSnapshot next = MakeSnapshot(kGen + 1, LUMICE_LIFECYCLE_RUNNING);
  auto next_payload = std::make_shared<TexturePayload>();
  next_payload->payload_epoch = kGen + 1;
  next_payload->texture_ray_count = 100;
  next_payload->snapshot_intensity = 1.0f;
  next.payload = next_payload;
  next.texture_serial = 2;
  EXPECT_TRUE(ShouldUploadPayload(next, /*last_serial=*/1, /*floor=*/kGen));
  EXPECT_FALSE(ShouldUploadPayload(next, /*last_serial=*/2, /*floor=*/kGen));
}

// Swapping the compute backend rebuilds the server, and a rebuilt server restarts its epoch
// authority at 0 — so the new backend's first frame carries a LOW epoch. Carried across the swap, a
// floor raised by any earlier edit fences that frame out forever and the stale texture sticks on
// screen: run on CPU, switch to GPU, press Run, nothing happens. Headless on purpose — the defect is
// pure epoch bookkeeping and independent of which backend is on either side of the swap.
TEST(RunLifecycleChain, ABackendSwapClearsTheFenceTheOldServersEpochsRaised) {
  PreviewSnapshot fresh = MakeSnapshot(1, LUMICE_LIFECYCLE_RUNNING);
  auto payload = std::make_shared<TexturePayload>();
  payload->payload_epoch = 1;  // the new backend's first commit mints epoch 1
  payload->texture_ray_count = 100;
  payload->snapshot_intensity = 1.0f;
  fresh.payload = payload;
  fresh.texture_serial = 42;  // the poller's serial is global-monotonic across the swap

  GuiState st;
  st.committed_epoch = 3;
  st.display_epoch_floor = 3;
  st.last_uploaded_texture_serial = 41;
  st.snapshot_intensity = 1.0f;
  EXPECT_FALSE(ShouldUploadPayload(fresh, st.last_uploaded_texture_serial, st.display_epoch_floor));

  st.ResetDisplayGenerationForBackendSwap();
  EXPECT_EQ(st.committed_epoch, 0u);
  EXPECT_EQ(st.display_epoch_floor, 0u);
  EXPECT_EQ(st.last_uploaded_texture_serial, 0ull);
  EXPECT_EQ(st.snapshot_intensity, 0.0f);  // the stale texture is cleared, not left to linger
  EXPECT_TRUE(ShouldUploadPayload(fresh, st.last_uploaded_texture_serial, st.display_epoch_floor));
}

// ---------------------------------------------------------------------------------------------
// E7 — the state the user reads off the screen is a function of intent, epoch and the last
// observation, and of nothing else.
//
// ReconcileSimState is the single writer of sim_state. A second writer is how "Simulating" outlives
// the simulation: some display-time action pokes the field, and the label never comes back. Driving
// the truth table directly is what keeps that single-writer claim checkable.

// `valid` is a column and not a fixture detail: a valid=false observation is what the poller
// publishes across a restart wake, and whether that transient counts as "an observation" is exactly
// what several rows below decide.
struct ReconcileCase {
  const char* name;
  RunIntent intent;
  uint64_t committed_epoch;
  bool have_snapshot;
  bool snap_valid;
  unsigned long long snap_epoch;
  int lifecycle;
  bool dirty;
  GuiState::SimState expected;
};

constexpr int kIdleLc = LUMICE_LIFECYCLE_IDLE;
constexpr int kRunLc = LUMICE_LIFECYCLE_RUNNING;
constexpr int kDoneLc = LUMICE_LIFECYCLE_COMPLETED;

TEST(RunLifecycleChain, SimStateIsAFunctionOfIntentEpochAndObservation) {
  const ReconcileCase kCases[] = {
    // --- kNone: never run. No observation and no edit can promote it out of idle.
    { "never run", RunIntent::kNone, 0, false, true, 0, kIdleLc, false, GuiState::SimState::kIdle },
    { "never run, edited", RunIntent::kNone, 3, true, true, 3, kRunLc, true, GuiState::SimState::kIdle },

    // --- kLoaded / kStopped: a result exists without this session having produced it. Both are
    // demotable by an edit, which is the entire difference between them and kStopping below.
    { "opened a document carrying a result", RunIntent::kLoaded, 0, false, true, 0, kIdleLc, false,
      GuiState::SimState::kDone },
    { "opened, then edited", RunIntent::kLoaded, 0, false, true, 0, kIdleLc, true, GuiState::SimState::kModified },
    { "stopped by the user", RunIntent::kStopped, 7, false, true, 0, kIdleLc, false, GuiState::SimState::kDone },
    { "stopped, then edited", RunIntent::kStopped, 7, false, true, 0, kIdleLc, true, GuiState::SimState::kModified },

    // --- kRunning: the only intent whose answer depends on what was observed.
    { "running, no observation yet", RunIntent::kRunning, 3, false, true, 0, kIdleLc, false,
      GuiState::SimState::kSimulating },
    { "running, backend agrees", RunIntent::kRunning, 3, true, true, 3, kRunLc, false,
      GuiState::SimState::kSimulating },
    // The completion edge for the epoch the GUI actually committed. This is the transition whose
    // absence reads as "it finished but the UI still says Simulating".
    { "completed at the committed epoch", RunIntent::kRunning, 3, true, true, 3, kDoneLc, false,
      GuiState::SimState::kDone },
    { "completed at the committed epoch, edited since", RunIntent::kRunning, 3, true, true, 3, kDoneLc, true,
      GuiState::SimState::kModified },
    // A completion belonging to a previous generation must not end the current run. Drop the epoch
    // term from the freshness test and this row turns green for the wrong reason.
    { "completed at an older epoch", RunIntent::kRunning, 4, true, true, 3, kDoneLc, false,
      GuiState::SimState::kSimulating },
    // IDLE is not completion — a Stop returns the server to IDLE at the same epoch.
    { "idle at the committed epoch", RunIntent::kRunning, 3, true, true, 3, kIdleLc, false,
      GuiState::SimState::kSimulating },
    // Not yet a real observation, whatever it says.
    { "invalid observation claiming completion", RunIntent::kRunning, 3, true, false, 3, kDoneLc, false,
      GuiState::SimState::kSimulating },

    // --- kRunCompleted: the latched natural completion. Load-bearing, because the latch is what
    // survives the two transients that produced the original "finished run flips back to
    // Simulating" bug — an invalid observation across a wake, and a stale-epoch one.
    { "latched completion, no observation", RunIntent::kRunCompleted, 5, false, true, 0, kIdleLc, false,
      GuiState::SimState::kDone },
    { "latched completion, invalid observation", RunIntent::kRunCompleted, 5, true, false, 5, kDoneLc, false,
      GuiState::SimState::kDone },
    { "latched completion, stale observation", RunIntent::kRunCompleted, 5, true, true, 3, kDoneLc, false,
      GuiState::SimState::kDone },
    { "latched completion, a fresh RUNNING observation", RunIntent::kRunCompleted, 5, true, true, 5, kRunLc, false,
      GuiState::SimState::kDone },
    { "latched completion, edited since", RunIntent::kRunCompleted, 5, false, true, 0, kIdleLc, true,
      GuiState::SimState::kModified },

    // --- kStopping: a draining run is not an editable completed result, so unlike every other
    // terminal-ish intent it is NOT demoted by dirty, and no observation pulls it either way.
    { "stopping", RunIntent::kStopping, 3, true, true, 3, kRunLc, false, GuiState::SimState::kStopping },
    { "stopping, edited", RunIntent::kStopping, 7, false, true, 0, kIdleLc, true, GuiState::SimState::kStopping },
    { "stopping, a fresh completion arrives", RunIntent::kStopping, 7, true, true, 7, kDoneLc, false,
      GuiState::SimState::kStopping },
    { "stopping, a fresh completion arrives after an edit", RunIntent::kStopping, 7, true, true, 7, kDoneLc, true,
      GuiState::SimState::kStopping },
  };

  for (const ReconcileCase& c : kCases) {
    PreviewSnapshot snap = MakeSnapshot(c.snap_epoch, c.lifecycle);
    snap.valid = c.snap_valid;
    const PreviewSnapshot* snap_ptr = c.have_snapshot ? &snap : nullptr;
    EXPECT_EQ(ReconcileSimState(c.intent, c.committed_epoch, snap_ptr, c.dirty), c.expected) << c.name;
  }

  // The property behind the demotion column, stated so a future change cannot satisfy those rows by
  // accident: an edit is only news about a result that EXISTS. Demoting kSimulating would tell the
  // user their running simulation is already invalid; demoting kIdle would put a "modified" badge
  // on a document that has never run.
  for (RunIntent intent : { RunIntent::kLoaded, RunIntent::kStopped, RunIntent::kRunCompleted }) {
    EXPECT_EQ(ReconcileSimState(intent, 3, nullptr, true), GuiState::SimState::kModified)
        << "intent " << static_cast<int>(intent);
  }
  PreviewSnapshot running = MakeSnapshot(3, LUMICE_LIFECYCLE_RUNNING);
  for (RunIntent intent : { RunIntent::kNone, RunIntent::kRunning, RunIntent::kStopping }) {
    EXPECT_EQ(ReconcileSimState(intent, 3, &running, true), ReconcileSimState(intent, 3, &running, false))
        << "intent " << static_cast<int>(intent);
  }
}

// ---------------------------------------------------------------------------------------------
// E10 (display-time half) — flipping the composite preview switch produces no new poller snapshot,
// so the upload branch has to fire on the display-mode change alone. Without that OR-branch the
// user toggles the switch and nothing happens until the next simulation batch lands, which on a
// finished run is never.

// A snapshot the display gate has already consumed once: the serial is spent and no newer
// generation exists, so nothing but the mode change can re-open the gate.
PreviewSnapshot MakeConsumedSnapshot(bool is_composite) {
  PreviewSnapshot snap = MakeSnapshot(5, LUMICE_LIFECYCLE_COMPLETED);
  snap.texture_serial = 10;
  auto payload = std::make_shared<TexturePayload>();
  payload->payload_epoch = 5;
  payload->width = 4;
  payload->height = 4;
  payload->snapshot_intensity = 1.0f;
  payload->is_composite = is_composite;
  snap.payload = payload;
  return snap;
}

// Walk the whole toggle session the user can drive from a finished run: first frame, settle, off,
// settle, back on, settle. Every "settle" row is as load-bearing as the flip beside it — a gate that
// fires on the flip but never settles re-uploads every frame for the rest of the session, which
// costs a texture upload per frame on a run that will never produce another pixel.
TEST(RunLifecycleChain, CompositeUploadFiresOnADisplayFlipThatProducesNoNewSnapshot) {
  const PreviewSnapshot snap = MakeConsumedSnapshot(/*is_composite=*/true);

  // The standard gate alone says there is nothing to do at any point in the sequence below; the
  // mode-change branch is the only thing that can ever fire it.
  EXPECT_FALSE(ShouldUploadPayload(snap, 10, 4));

  struct FlipCase {
    const char* name;
    unsigned long long last_serial;
    bool show_composite;
    bool last_uploaded_as_composite;
    bool expect_fire;
  };
  const FlipCase kCases[] = {
    // Nothing uploaded yet: the serial-dedup term alone opens the gate.
    { "the first frame after a run", 0, true, false, true },
    { "settled on the composite view", 10, true, true, false },
    // No new poll, so the serial and the floor are unchanged: the toggle is the only input that
    // moved. If this row were false, the button would visibly do nothing.
    { "the user switches the view off", 10, false, true, true },
    { "settled on the xyz view", 10, false, false, false },
    { "the user switches it back on", 10, true, false, true },
    { "settled again", 10, true, true, false },
  };

  for (const FlipCase& c : kCases) {
    EXPECT_EQ(ShouldFireCompositeUpload(snap, c.last_serial, /*display_epoch_floor=*/4, c.show_composite,
                                        c.last_uploaded_as_composite),
              c.expect_fire)
        << c.name;
  }

  // On a scene with no colour classes the same gate must collapse to the plain upload gate: with no
  // composite payload the effective mode is always non-composite, so the mode-change term is
  // structurally unreachable and no flip can slip an extra upload through. This is what keeps the
  // zero-colour path free of any cost from the feature.
  const PreviewSnapshot plain = MakeConsumedSnapshot(/*is_composite=*/false);
  for (unsigned long long last_serial : { 0ull, 10ull }) {
    for (bool show_composite : { false, true }) {
      EXPECT_EQ(ShouldFireCompositeUpload(plain, last_serial, /*display_epoch_floor=*/4, show_composite,
                                          /*last_uploaded_as_composite=*/false),
                ShouldUploadPayload(plain, last_serial, /*display_epoch_floor=*/4))
          << "last_serial=" << last_serial << " show_composite=" << show_composite;
    }
  }
}

// What the user's two write sites (the top-bar button and the Colours window's checkbox) mean, and
// what the mode they set folds together with. Opening the window turns the composite view on only
// when there is no preference to respect yet — i.e. when no classes exist; once they do, the
// remembered choice wins. The toggle itself is negation and nothing more, which is what lets the two
// write sites share one asserted meaning. And the effective upload mode is that preference ANDed
// with what the server actually produced.
TEST(RunLifecycleChain, TheColourViewPreferenceIsSetInOnePlaceAndFoldedWithServerAvailability) {
  EXPECT_TRUE(ShouldDefaultEnableColorsOnOpen(/*no_color_classes=*/true));
  EXPECT_FALSE(ShouldDefaultEnableColorsOnOpen(/*no_color_classes=*/false));

  GuiState s;
  s.show_composite_preview = false;
  ToggleCompositePreview(s);
  EXPECT_TRUE(s.show_composite_preview);
  ToggleCompositePreview(s);
  EXPECT_FALSE(s.show_composite_preview);

  for (bool payload_is_composite : { false, true }) {
    for (bool show_composite_preview : { false, true }) {
      // Asked for but not produced falls back to the xyz view — the honest answer, not an empty one.
      EXPECT_EQ(ShouldUseCompositeUpload(payload_is_composite, show_composite_preview),
                payload_is_composite && show_composite_preview)
          << "payload_is_composite=" << payload_is_composite << " show=" << show_composite_preview;
    }
  }
}

}  // namespace
}  // namespace lumice::gui
