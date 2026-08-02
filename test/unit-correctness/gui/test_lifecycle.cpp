// Preview-lifecycle invariants that need no frame — see doc/gui-preview-lifecycle-architecture.md
// §2/§6. Pins blueprint invariants I1 (epoch-keyed truth), I2 (single-owner ReconcileSimState),
// I3 (level-triggered self-heal — the terminal completion edge is never lost) and I4 (lifecycle
// readable without an expensive snapshot generation).
//
// sim_state is DERIVED once per frame by the pure ReconcileSimState(run_intent, committed_epoch,
// snapshot, dirty), and the terminal edge is the durable lifecycle==COMPLETED carried on every
// poll. The cases below drive the reconcile INPUTS (run_intent + committed_epoch) rather than
// writing sim_state directly, then call gui::SyncFromPoller() themselves.
//
// These operate on the REAL gui::g_state / gui::g_server_poller globals rather than on a local
// GuiState, and that is faithful rather than lazy: SyncFromPoller() and ReconcileSimState()'s
// caller take no state argument — "a single owner reconciles the real global once per frame" IS
// the design under test, so injecting a local would exercise a path production does not have.
// Each case resets the few fields it touches (run_intent / committed_epoch / display_epoch_floor /
// dirty) instead of reaching for a whole-harness reset.
//
// Cases:
//  1. terminal_idle_reaches_done — DETERMINISTIC white-box. Reproduces the exact poll/sync
//     interleaving from the original dead-state bug (a mid-run texture drop, then a terminal poll
//     carrying NO new snapshot generation) and asserts the completion edge still reaches kDone via
//     the single-owner reconcile. Boundary sub-checks pin that an IDLE transient (C1) and a
//     stale-epoch COMPLETED (C2) are NOT misclassified as done.
//  2. reconcile_truth_table — PURE unit test of ReconcileSimState (I2).
//  3. anti_flicker_epoch_floor — PURE structural test of the epoch-floor upload gate (I1/§3.3):
//     MarkStructHardDirty fences the old generation; MarkDirty carries it forward.
//  4. backend_swap_resets_epoch_fence
//  5. snapshot_bundle_coherence
//  6. completed_below_threshold_force_uploads
//  7. wake_for_refresh_preserves_valid
//
// Their five frame-driven siblings stay in test/gui/functional/test_gui_lifecycle.cpp.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <thread>

#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "gui/server_poller.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

namespace {

using SimState = gui::GuiState::SimState;
using RunIntent = gui::RunIntent;

// Small finite single-prism run: empty filter (all rays pass), rectangular lens, tiny
// resolution — completes in well under a second on CPU or Metal.
//
// The crystal block is in the canonical wire form (lowercase "prism", shape nested under
// "shape"). It used to read `"type": "Prism", "height": 1.0, "ratio": {...}` — which the core
// parser did NOT understand: an unrecognized type logs "Unknown crystal type!" and leaves the
// default-constructed param, so "height"/"ratio" were never read and this test silently ran a
// zero-face-distance crystal. The Scene parser rejects the unknown type outright
// (LUMICE_ERR_INVALID_VALUE) instead of defaulting, which is how the typo surfaced.
const char* kFiniteConfig = R"({
  "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.0}}],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0, "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{"id": 1, "lens": {"type": "rectangular", "fov": 180.0},
              "resolution": [256, 128], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
              "visible": "full", "background": [0, 0, 0], "opacity": 1.0, "intensity_factor": 1.0}]
})";

// Commit kFiniteConfig and block until the server reaches IDLE with produced data
// (has_valid_data is the C-API RawXyzResult contract field — untouched by 1.5). Returns false on
// timeout / commit failure.
// JSON -> Scene handle -> commit: the only commit path since v4.12 removed the JSON-string
// entry points. The handle is a local — LUMICE_CommitScene reads it as const and keeps no
// reference — so it is destroyed as soon as the commit returns.
LUMICE_ErrorCode CommitJsonConfig(LUMICE_Server* server, const char* json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json, &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

bool RunFiniteToCompletion(LUMICE_Server* server) {
  if (CommitJsonConfig(server, kFiniteConfig) != LUMICE_OK) {
    return false;
  }
  constexpr int kMaxWaitMs = 5000;
  for (int waited = 0; waited < kMaxWaitMs; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[2]{};
      LUMICE_GetRawXyzResults(server, xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// Read the server's current committed epoch (post-commit or post-stop) via the lifecycle clock.
unsigned long long CurrentEpoch(LUMICE_Server* server) {
  LUMICE_SimLifecycleResult lc{};
  LUMICE_GetSimLifecycle(server, &lc);
  return lc.epoch;
}

}  // namespace

// ---- Test 1: deterministic white-box interleaving regression ----
TEST(GuiLifecycle, terminal_idle_reaches_done) {
  // Clean baseline: detach the global poller from any prior test's server and reset the reconcile
  // INPUTS (not sim_state — sim_state is derived).
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;
  gui::g_state.run_intent = RunIntent::kNone;
  gui::g_state.committed_epoch = 0;
  gui::g_state.display_epoch_floor = 0;
  gui::g_state.dirty = false;

  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);  // finite run reached IDLE + has_valid_data (C-API contract)
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  // The terminal completion is readable as the explicit COMPLETED lifecycle with a minted epoch —
  // the SAME signal the poller now publishes and the reconcile keys on.
  const unsigned long long done_epoch = CurrentEpoch(server);
  {
    LUMICE_SimLifecycleResult lc{};
    EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
    EXPECT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    EXPECT_TRUE(lc.epoch >= 1);
  }

  // (A) POLLER HALF (server_poller.cpp / I4 + I5): a terminal poll that carries NO new snapshot
  // generation must still carry lifecycle==COMPLETED with the same epoch. This is the level signal
  // the fix decoupled from snapshot materialization. Reverting the unconditional per-poll
  // `next->lifecycle = lc.lifecycle` carry (gating it on has_new_snapshot instead) makes
  // b->lifecycle stale on poll B. Reads are non-destructive atomic snapshot loads (I5).
  {
    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);  // Poll A: generation G is new -> materializes stats + texture
    auto a = local.LoadSnapshot();  // non-destructive load of the published Poll A snapshot
    ASSERT_TRUE(a != nullptr);
    EXPECT_TRUE(a->valid);
    EXPECT_EQ(a->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    EXPECT_EQ(a->epoch, done_epoch);
    EXPECT_TRUE(a->stats_sim_ray_num > 0);  // Poll A genuinely carried the generation-bearing frame

    local.PollOnceForTest(server);  // Poll B: same generation G -> has_new_snapshot == false
    auto b = local.LoadSnapshot();
    ASSERT_TRUE(b != nullptr);
    EXPECT_TRUE(b->valid);
    // I5 bundle coherence: lifecycle + epoch + stats CARRY FORWARD across a no-new-generation
    // poll instead of being torn. The terminal COMPLETED edge survives (the fix, I4).
    EXPECT_EQ(b->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    EXPECT_EQ(b->epoch, done_epoch);
    EXPECT_TRUE(b->stats_sim_ray_num > 0);
  }

  // (B) APP HALF (app.cpp SyncFromPoller / I2 + I3): the real completion decision, exercised
  // through the actual global poller + SyncFromPoller with the same issue interleaving. The
  // terminal poll carries no new generation; the single-owner reconcile must still reach kDone.
  // Inputs are written (run_intent + committed_epoch), NOT sim_state. Reverting ReconcileSimState
  // to ignore COMPLETED (e.g. keying on snap->has_new_texture) or inverting the epoch compare
  // (`snap->epoch != committed_epoch`) leaves sim_state stuck at kSimulating (the original bug).
  //
  // Mid-op: InvalidateStagedTexture() nulls the published payload so the terminal SyncFromPoller
  // takes NO GL upload path (this coroutine runs on a worker thread with no current GL context).
  {
    gui::g_state.run_intent = RunIntent::kRunning;
    gui::g_state.committed_epoch = done_epoch;  // epoch match ⇒ observation is "fresh"
    gui::g_state.dirty = false;

    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);    // Poll A (generation-bearing)
    gui::g_server_poller.InvalidateStagedTexture();  // mid-op: drop staged texture, keep lifecycle
    gui::g_server_poller.PollOnceForTest(server);    // Poll B: terminal, no new generation

    gui::SyncFromPoller();  // single-owner reconcile consumes the terminal frame
    EXPECT_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
  }

  // (C) BOUNDARY (I1/I3): two transients that must NOT be classified done.
  //
  // C2 — STALE-EPOCH COMPLETED (pins I1 epoch keying): the GUI has committed a NEWER generation
  // (committed_epoch = done_epoch + 1) that the server hasn't produced yet, while the poller still
  // observes the OLD generation's COMPLETED@done_epoch. `fresh` requires epoch match, so the stale
  // COMPLETED is discarded → kSimulating. Dropping the epoch check in `fresh` would let the stale
  // COMPLETED masquerade as this generation's completion → kDone → RED.
  {
    gui::g_state.run_intent = RunIntent::kRunning;
    gui::g_state.committed_epoch = done_epoch + 1;  // GUI expects a newer generation
    gui::g_state.dirty = false;

    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);  // publishes COMPLETED@done_epoch
    gui::g_server_poller.InvalidateStagedTexture();
    gui::g_server_poller.PollOnceForTest(server);

    gui::SyncFromPoller();
    EXPECT_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));
  }

  // C1 — IDLE TRANSIENT (pins lifecycle discrimination): Stop returns the server to IDLE at the
  // same epoch. IDLE is not COMPLETED, so even with a matching epoch the reconcile stays
  // kSimulating. Treating IDLE as completion would flip it to kDone → RED.
  {
    LUMICE_StopServer(server);  // → IDLE at the same epoch
    const unsigned long long idle_epoch = CurrentEpoch(server);
    gui::g_state.run_intent = RunIntent::kRunning;
    gui::g_state.committed_epoch = idle_epoch;  // epoch matches; only lifecycle differs
    gui::g_state.dirty = false;

    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);
    gui::g_server_poller.InvalidateStagedTexture();
    gui::g_server_poller.PollOnceForTest(server);

    gui::SyncFromPoller();
    EXPECT_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));
  }

  // Cleanup: leave a clean global state (inputs) for subsequent tests.
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;
  gui::g_state.run_intent = RunIntent::kNone;
  gui::g_state.committed_epoch = 0;
  gui::g_state.display_epoch_floor = 0;
  LUMICE_DestroyServer(server);
}

// ---- Test 3: pure ReconcileSimState truth table (I2), no server / GL ----
// Row-by-row pin of the §1.3 table. This is the reconcile's single-owner contract; each row's
// RED手法 is noted. Building a PreviewSnapshot by hand keeps this fully headless.
TEST(GuiLifecycle, reconcile_truth_table) {
  auto mk = [](bool valid, unsigned long long epoch, int lifecycle) {
    gui::PreviewSnapshot s;
    s.valid = valid;
    s.epoch = epoch;
    s.lifecycle = lifecycle;
    return s;
  };
  const int kDone_lc = static_cast<int>(LUMICE_LIFECYCLE_COMPLETED);
  const int kRun_lc = static_cast<int>(LUMICE_LIFECYCLE_RUNNING);
  const int kIdle_lc = static_cast<int>(LUMICE_LIFECYCLE_IDLE);

  // kNone → kIdle regardless of observation / dirty.
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kNone, 0, nullptr, false)),
            static_cast<int>(SimState::kIdle));
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kNone, 0, nullptr, true)),
            static_cast<int>(SimState::kIdle));  // dirty never promotes kIdle

  // kLoaded → kDone; +dirty → kModified.
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kLoaded, 0, nullptr, false)),
            static_cast<int>(SimState::kDone));
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kLoaded, 0, nullptr, true)),
            static_cast<int>(SimState::kModified));

  // kStopped → kStopEndState (kDone, owner-decided 1.6); +dirty → kModified (kDone is demotable).
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopped, 7, nullptr, false)),
            static_cast<int>(SimState::kDone));
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopped, 7, nullptr, true)),
            static_cast<int>(SimState::kModified));

  // kRunCompleted (latched natural completion) → kDone regardless
  // of the observation. This is the load-bearing row: even a valid=false observation (root cause
  // (a)) or a stale-epoch observation must NOT pull the latched terminal back to kSimulating.
  // RED手法: forgetting the case in the switch would fall through to a default and mis-derive.
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, nullptr, false)),
            static_cast<int>(SimState::kDone));
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, nullptr, true)),
            static_cast<int>(SimState::kModified));  // +dirty → kModified (kDone is demotable)
  {
    auto s_invalid = mk(false, 5, kDone_lc);
    // valid=false observation must NOT pull kRunCompleted back to kSimulating — this row is the
    // structural guarantee AC1's activity bug root cause (b) relied on and this test pins.
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, &s_invalid, false)),
              static_cast<int>(SimState::kDone));
    auto s_stale = mk(true, 3, kDone_lc);  // stale epoch (< committed)
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, &s_stale, false)),
              static_cast<int>(SimState::kDone));
    auto s_running = mk(true, 5, kRun_lc);  // even a fresh RUNNING observation stays latched
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, &s_running, false)),
              static_cast<int>(SimState::kDone));
  }

  // kStopping (async Stop draining, 1.6) → kStopping, for ANY observation/dirty. Pure optimistic
  // intent: not pulled by a fresh COMPLETED, and NOT demoted by dirty (a draining run is not an
  // editable completed result). RED手法: mis-mapping the kStopping case to S::kSimulating turns
  // every row below RED.
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, nullptr, false)),
            static_cast<int>(SimState::kStopping));
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, nullptr, true)),
            static_cast<int>(SimState::kStopping));  // dirty does NOT demote kStopping
  {
    auto s = mk(true, 7, kDone_lc);  // even a fresh COMPLETED does not pull kStopping to kDone
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, &s, false)),
              static_cast<int>(SimState::kStopping));
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, &s, true)),
              static_cast<int>(SimState::kStopping));
  }

  // kRunning, no observation yet → kSimulating.
  EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, nullptr, false)),
            static_cast<int>(SimState::kSimulating));

  // kRunning, fresh COMPLETED@match → kDone. (RED: invert the epoch compare, or ignore COMPLETED.)
  {
    auto s = mk(true, 5, kDone_lc);
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
              static_cast<int>(SimState::kDone));
    // +dirty on a completed result → kModified.
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, true)),
              static_cast<int>(SimState::kModified));
  }

  // kRunning, fresh RUNNING → kSimulating.
  {
    auto s = mk(true, 5, kRun_lc);
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
              static_cast<int>(SimState::kSimulating));
  }

  // kRunning, fresh IDLE (C1 boundary) → kSimulating (IDLE ≠ completion).
  {
    auto s = mk(true, 5, kIdle_lc);
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
              static_cast<int>(SimState::kSimulating));
  }

  // kRunning, STALE-epoch COMPLETED (C2 boundary, pins I1) → kSimulating.
  // RED手法: if `fresh` drops the `snap->epoch == committed_epoch` term, the stale COMPLETED@5 is
  // treated as completion for gen 6 → kDone → this assertion goes RED.
  {
    auto s = mk(true, 5, kDone_lc);  // observation is a generation behind
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, /*committed=*/6, &s, false)),
              static_cast<int>(SimState::kSimulating));
  }

  // kRunning, invalid (valid=false) COMPLETED → kSimulating (not yet a real observation).
  {
    auto s = mk(false, 5, kDone_lc);
    EXPECT_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
              static_cast<int>(SimState::kSimulating));
  }
}

// ---- Test 4: anti-flicker epoch-floor upload gate (I1/§3.3), pure/headless ----
// Pins that MarkStructHardDirty raises display_epoch_floor to fence the OLD generation's payload
// (blocked by ShouldUploadPayload) while MarkDirty leaves the floor so a carried-forward payload
// passes. Bites the real production predicate (ShouldUploadPayload) + the real MarkStructHardDirty /
// MarkDirty split — the mechanism behind "filter change clears + no stale refill" vs "crystal
// scrub keeps the last frame with no black flicker", which manual Metal scrub can't check in CI.
TEST(GuiLifecycle, anti_flicker_epoch_floor) {
  const unsigned long long kGen = 5;  // the OLD generation's epoch

  // A materialized payload from the OLD generation (epoch == kGen), non-empty so intensity/ray
  // gate is satisfied — the only thing that can block it is the epoch floor.
  auto old_payload = std::make_shared<gui::TexturePayload>();
  old_payload->payload_epoch = kGen;
  old_payload->texture_ray_count = 100;  // valid frame
  old_payload->snapshot_intensity = 1.0f;
  gui::PreviewSnapshot snap;
  snap.valid = true;
  snap.epoch = kGen;
  snap.payload = old_payload;
  snap.texture_serial = 1;  // unseen (cursor starts at 0)

  // --- filter change: MarkStructHardDirty raises the floor to committed_epoch ⇒ old payload BLOCKED.
  {
    gui::GuiState st;
    st.committed_epoch = kGen;
    st.display_epoch_floor = 0;
    st.MarkStructHardDirty();
    EXPECT_EQ(st.display_epoch_floor, kGen);  // floor bumped to the current generation
    EXPECT_EQ(st.snapshot_intensity, 0.0f);   // immediate display clear preserved
    // payload_epoch (kGen) is NOT > floor (kGen) ⇒ gate rejects the stale texture.
    EXPECT_TRUE(!gui::ShouldUploadPayload(snap, /*last_serial=*/0, st.display_epoch_floor));
  }

  // --- crystal scrub: MarkDirty leaves the floor at 0 ⇒ carried-forward old payload PASSES.
  {
    gui::GuiState st;
    st.committed_epoch = kGen;
    st.display_epoch_floor = 0;
    st.MarkDirty();
    EXPECT_EQ(st.display_epoch_floor, 0u);  // floor untouched (no fence)
    // payload_epoch (kGen) > floor (0) ⇒ gate accepts (anti-flicker carry-forward).
    EXPECT_TRUE(gui::ShouldUploadPayload(snap, /*last_serial=*/0, st.display_epoch_floor));
  }

  // --- after re-commit the newer generation clears the fence (epoch kGen+1 > floor kGen).
  {
    auto new_payload = std::make_shared<gui::TexturePayload>();
    new_payload->payload_epoch = kGen + 1;
    new_payload->texture_ray_count = 100;
    new_payload->snapshot_intensity = 1.0f;
    gui::PreviewSnapshot new_snap;
    new_snap.valid = true;
    new_snap.epoch = kGen + 1;
    new_snap.payload = new_payload;
    new_snap.texture_serial = 2;
    EXPECT_TRUE(gui::ShouldUploadPayload(new_snap, /*last_serial=*/1, /*floor=*/kGen));
    // exact-once: an already-seen serial is rejected even when the epoch clears the floor.
    EXPECT_TRUE(!gui::ShouldUploadPayload(new_snap, /*last_serial=*/2, /*floor=*/kGen));
  }
}

// ---- Test 4b: backend swap (CPU<->GPU) resets the display epoch fence ----
// Pins the fix for "run on CPU, switch to GPU, click Run, preview does not update". A reconstructed
// server restarts its epoch authority at 0, so its first frame carries a LOW epoch (1). If the old
// server's display_epoch_floor (raised to >=1 by any prior filter edit) is carried across the swap,
// ShouldUploadPayload fences the new backend's frame forever and the stale texture sticks on screen.
// ResetDisplayGenerationForBackendSwap must clear the fence + carried texture so the new backend's
// frame uploads. Bites the real production predicate + the real reset method (fully headless — no
// GPU needed, since the bug is a pure epoch-bookkeeping defect independent of the backend kind).
TEST(GuiLifecycle, backend_swap_resets_epoch_fence) {
  // The new backend's first commit mints epoch 1 (server committed_epoch_ resets to 0, +1 on rebuild).
  auto fresh_payload = std::make_shared<gui::TexturePayload>();
  fresh_payload->payload_epoch = 1;
  fresh_payload->texture_ray_count = 100;  // valid, non-empty frame
  fresh_payload->snapshot_intensity = 1.0f;
  gui::PreviewSnapshot fresh_snap;
  fresh_snap.valid = true;
  fresh_snap.epoch = 1;
  fresh_snap.payload = fresh_payload;
  fresh_snap.texture_serial = 42;  // poller serial is global-monotonic across the swap

  // Accumulated CPU session: the floor was raised to a prior generation by an earlier filter edit,
  // and a texture from that session was already uploaded.
  gui::GuiState st;
  st.committed_epoch = 3;
  st.display_epoch_floor = 3;
  st.last_uploaded_texture_serial = 41;
  st.snapshot_intensity = 1.0f;

  // BUG condition (pre-reset): the new backend's epoch-1 frame is fenced out (1 > 3 is false), so
  // the preview would freeze on the previous backend's texture.
  EXPECT_TRUE(!gui::ShouldUploadPayload(fresh_snap, st.last_uploaded_texture_serial, st.display_epoch_floor));

  // FIX: the backend swap resets the display generation to epoch 0 and clears the carried texture.
  st.ResetDisplayGenerationForBackendSwap();
  EXPECT_EQ(st.committed_epoch, 0u);
  EXPECT_EQ(st.display_epoch_floor, 0u);
  EXPECT_EQ(st.last_uploaded_texture_serial, 0ull);
  EXPECT_EQ(st.snapshot_intensity, 0.0f);  // immediate clear of the stale texture

  // After the reset the new backend's epoch-1 frame clears the fence and uploads.
  EXPECT_TRUE(gui::ShouldUploadPayload(fresh_snap, st.last_uploaded_texture_serial, st.display_epoch_floor));
}

// ---- Test 6: I5 — versioned immutable snapshot bundle coherence + whole-object handoff ----
// Pins invariant I5 (blueprint §5/§9): the cross-thread handoff is ONE versioned immutable value;
// a consumer's single LoadSnapshot() yields a whole object whose fields are mutually coherent for
// the SAME committed generation — never a half-updated field combination (the §2 torn-read
// pathology). The atomic no-torn-read core is construction-guaranteed (shared_ptr<const
// PreviewSnapshot> + one atomic_load/store), which no test can drive RED without deliberately
// breaking atomicity; what this test DOES pin is the *producer bundle coherence* the construction
// argument cannot: on a generation-bearing poll every field (epoch, lifecycle, stats, payload,
// payload->payload_epoch, texture_serial) belongs to the same generation. Test 1 section A pins
// lifecycle+epoch+stats carry-forward; this adds the payload / payload_epoch coherence relation
// (asserted NOWHERE else) plus that consecutive loads return the identical whole object.
TEST(GuiLifecycle, snapshot_bundle_coherence) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }
  const unsigned long long done_epoch = CurrentEpoch(server);

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);  // generation-bearing poll: materializes the full bundle

  // Whole-object handoff: LoadSnapshot returns one non-null immutable object (single atomic_load).
  auto s = local.LoadSnapshot();
  ASSERT_TRUE(s != nullptr);
  EXPECT_TRUE(s->valid);

  // Bundle coherence: EVERY field of this one published value belongs to the SAME committed
  // generation (done_epoch); the terminal lifecycle is consistent with produced stats AND a
  // materialized payload. No field is torn across generations.
  EXPECT_EQ(s->epoch, done_epoch);
  EXPECT_EQ(s->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_TRUE(s->stats_sim_ray_num > 0);
  EXPECT_TRUE(s->has_new_texture);     // this poll genuinely materialized a texture
  ASSERT_TRUE(s->payload != nullptr);  // ... so the payload is present in the SAME bundle
  // The payload's OWN epoch stamp matches the bundle epoch on a fresh materialization — the
  // coherence relation section A omits. (On carry-forward payload_epoch may deliberately LAG the
  // bundle epoch; here the texture is freshly materialized so they MUST agree.)
  EXPECT_EQ(s->payload->payload_epoch, done_epoch);
  EXPECT_TRUE(s->texture_serial != 0);  // a fresh monotonic serial was minted for this materialization

  // Two consecutive loads observe the SAME whole object (atomic pointer handoff, no partial
  // reconstruction between reads) — the consumer can never see a half-swapped value.
  auto s2 = local.LoadSnapshot();
  EXPECT_TRUE(s2.get() == s.get());

  local.Stop();
  LUMICE_DestroyServer(server);
}

// ---- Test 6c: I6 — a COMPLETED run below the calibrated quality gate still puts a frame on screen ----
// Pins blueprint invariant I6 (§9 "任何显示 gate 不得压制生命周期跃迁；终帧无条件上屏", §7 rule 2
// "终帧永远上屏"). The gate is a RUNNING-time anti-flicker device only: once the run has completed
// there is no better frame coming, so however sparse the result is, it is the result.
//
// The regression this guards: the quality gate rejected the terminal snapshot AND PollOnce's
// COMPLETED self-pause fired in the SAME call, after the gate — so the gate's 500ms timeout
// fallback could never be reached (a finite low-ray run completes in ~80ms) and the preview
// stayed blank forever. The real GUI calibrates the threshold to 3-4×10^4 rays at startup, so
// any user-configured run below that never showed a picture at all.
//
// Three phases, each ending in "the consumer got a frame". Phase A alone would pass against a
// WRONG fix that keys off poller-lifetime state (e.g. `texture_serial_ == 0`) instead of
// per-resume state — only the first low-ray completion in the process would be rescued. B and C
// exercise the two production wake seams that must re-arm the rescue: WakeForRestart (fresh
// commit) and WakeForRefresh (display-time refresh of an already-completed run — the colour /
// composite-EV push path, which likewise wakes the poller for exactly one more poll and would
// otherwise be swallowed by the same gate).
TEST(GuiLifecycle, completed_below_threshold_force_uploads) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);

  // Phase A ---- first low-ray completion on a never-resumed poller.
  const bool completed_a = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed_a);
  if (!completed_a) {
    LUMICE_DestroyServer(server);
    return;
  }

  // `local` is constructed AFTER the run, not before: its last_quality_pass_time_ starts at
  // construction, and the gate's 500ms timeout fallback is measured from there. Constructing it
  // first would let the sim's own wall-clock spend that budget, and the fallback — not the fix
  // under test — would be what puts the frame on screen (an "invalid mutation produces a
  // reassuring green").
  gui::ServerPoller local;

  // Arm the gate strictly above the ray count THIS run actually achieved. Derived from the
  // measurement rather than hard-coded: the achieved count is a core/RNG-dependent number and
  // pinning it here would make the test platform-dependent. `+1` rejects deterministically
  // without assuming anything about its magnitude.
  auto arm_gate_above_achieved = [&](LUMICE_RayCount* out_rays) {
    LUMICE_StatsResult stats{};
    LUMICE_GetCachedStats(server, &stats);
    *out_rays = stats.sim_ray_num;
    local.SetCalibratedThreshold(stats.sim_ray_num + 1);
  };

  LUMICE_RayCount rays_a = 0;
  arm_gate_above_achieved(&rays_a);
  // A genuinely zero-ray run takes the cold-start bypass (`sim_ray_num == 0` passes the gate
  // unconditionally) and would exercise none of the path under test.
  EXPECT_TRUE(rays_a > 0);

  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  auto sa = local.LoadSnapshot();
  ASSERT_TRUE(sa != nullptr);
  EXPECT_EQ(sa->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_TRUE(sa->payload != nullptr);   // the terminal frame reached the consumer despite the gate
  EXPECT_TRUE(sa->has_new_texture);      // ... and THIS poll is what materialized it
  EXPECT_TRUE(sa->texture_serial != 0);  // ... under a fresh monotonic serial (the upload key)
  const unsigned long long serial_a = sa->texture_serial;

  // Phase B ---- second low-ray completion, re-armed through the production WakeForRestart seam
  // (what DoRun calls on a fresh commit). Drives the real kPaused→kRunning reset, not the
  // ResetGenerationForTest() test seam, so a rescue flag that is never re-armed shows up here.
  const bool completed_b = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed_b);
  if (!completed_b) {
    local.Stop();
    LUMICE_DestroyServer(server);
    return;
  }
  LUMICE_RayCount rays_b = 0;
  arm_gate_above_achieved(&rays_b);
  EXPECT_TRUE(rays_b > 0);

  local.WakeForRestart(server);
  // The wake releases `local`'s worker thread. Stop() blocks until it has left the poll loop, so
  // the synchronous PollOnceForTest below is the only writer — no data race on the poller's
  // per-resume state. The worker may still have landed one poll inside that window, which is why
  // the assertions below key on "the serial advanced" (true whichever thread materialized it)
  // rather than on has_new_texture (true only for the poll that did it).
  local.Stop();
  local.PollOnceForTest(server);

  auto sb = local.LoadSnapshot();
  ASSERT_TRUE(sb != nullptr);
  EXPECT_EQ(sb->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_TRUE(sb->payload != nullptr);
  EXPECT_TRUE(sb->texture_serial != serial_a);  // a SECOND materialization, not phase A's carried forward
  const unsigned long long serial_b = sb->texture_serial;

  // Phase C ---- display-time refresh of the SAME completed run, through WakeForRefresh (what the
  // colour-window PushDisplayState and the composite-EV push call). No new sim: the run is over
  // and the user only changed how it is displayed. That wake exists to buy exactly one more poll
  // before the poller self-pauses again, so if the gate swallows it the edit silently never shows.
  local.WakeForRefresh(server);
  local.Stop();  // same worker-race fence as phase B
  local.PollOnceForTest(server);

  auto sc = local.LoadSnapshot();
  ASSERT_TRUE(sc != nullptr);
  EXPECT_EQ(sc->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_TRUE(sc->payload != nullptr);
  EXPECT_TRUE(sc->texture_serial != serial_b);  // the refresh produced a frame of its own

  local.Stop();
  LUMICE_DestroyServer(server);
}

// ---- Test 7: WakeForRefresh preserves valid across the wake edge ----
// Pins the semantic distinction between the two poller wake seams introduced by M4:
//   WakeForRestart publishes valid=false on the kPaused→kRunning edge (fresh commit: consumers
//     must ignore stale terminal snapshots).
//   WakeForRefresh preserves valid across the wake edge (display-time refresh: SyncFromPoller
//     must not observe a transient valid=false window that would let ReconcileSimState pull a
//     completed sim back into kSimulating — activity bug AC1 root cause (a),
//     doc/gui-state-governance.md §4 支柱 2).
// Same-shape white-box test: bring poller to kPaused with a valid=true snapshot published, then
// exercise each wake variant and diff the immediately-following LoadSnapshot()->valid.
TEST(GuiLifecycle, wake_for_refresh_preserves_valid) {
  // Clean baseline: detach the global poller from any prior test's server.
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  // Baseline: publish a fresh valid=true terminal snapshot via a synchronous poll, then Stop the
  // worker so state_ == kPaused. Both wake variants below start from this identical baseline.
  auto seed_baseline = [server]() {
    gui::g_server_poller.Stop();
    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);
    auto snap = gui::g_server_poller.LoadSnapshot();
    ASSERT_TRUE(snap != nullptr);
    EXPECT_TRUE(snap->valid);  // baseline invariant: seed snapshot is valid before the wake
    gui::g_server_poller.Stop();
  };

  // (A) WakeForRestart: valid must flip to false after the wake (PublishValidReset called).
  seed_baseline();
  gui::g_server_poller.WakeForRestart(server);
  {
    auto snap = gui::g_server_poller.LoadSnapshot();
    ASSERT_TRUE(snap != nullptr);
    EXPECT_TRUE(!snap->valid);  // WakeForRestart publishes valid=false on the wake edge
  }

  // (B) WakeForRefresh: valid must be preserved as true (no PublishValidReset). This is the
  // load-bearing behavior for AC1 — a display-time edit's wake path must not fabricate a
  // valid=false window that ReconcileSimState would classify as kSimulating.
  seed_baseline();
  gui::g_server_poller.WakeForRefresh(server);
  {
    auto snap = gui::g_server_poller.LoadSnapshot();
    ASSERT_TRUE(snap != nullptr);
    EXPECT_TRUE(snap->valid);  // WakeForRefresh preserves valid across the wake edge
  }

  gui::g_server_poller.Stop();
  LUMICE_DestroyServer(server);
}
