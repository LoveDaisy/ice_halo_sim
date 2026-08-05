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
//  8a. resume_rearms_generation_tracker  } the two per-resume fields 6c structurally cannot reach
//  8b. resume_rearms_quality_gate_clock  } (see the block comment above them for why)
//  9a. stats_apply_gate_rejects_stale_generation — PURE test of the stats-apply gate
//  9b. restart_does_not_republish_prior_run_stats — producer end-to-end over the Run->Run edge
//  10a. restart_does_not_republish_prior_run_pixels  } 9b's texture-channel siblings: the same
//  10b. restart_window_does_not_republish_prior_run_composite } window, asserted on image content
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
#include "support/scoped_result_frame.hpp"

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
      lumice::test::ScopedResultFrame frame_xyz(server);
      LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1);
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

// Block until the server has drained to IDLE with produced data, for a config already committed by
// the caller. RunFiniteToCompletion above is this plus a kFiniteConfig commit; cases that need a
// different config (e.g. one carrying raypath_color) commit their own and then wait here.
bool WaitForValidData(LUMICE_Server* server) {
  constexpr int kMaxWaitMs = 5000;
  for (int waited = 0; waited < kMaxWaitMs; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[2]{};
      lumice::test::ScopedResultFrame frame_xyz(server);
      LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// Content fingerprint of a payload's pixels. The point of the cases below is that a payload can
// carry the PREVIOUS run's image, so they must compare the image itself — asserting only on
// payload_epoch would pass for the wrong reason whenever the timing happened not to line up
// (see the "assert pixel content, not just the stamp" requirement in the task's risk analysis).
// A plain FNV-1a over the raw float bytes: no tolerance, no interpretation, just "same bytes?".
uint64_t PixelFingerprint(const float* xyz, size_t float_count) {
  uint64_t h = 1469598103934665603ULL;
  const auto* bytes = reinterpret_cast<const unsigned char*>(xyz);
  const size_t n = float_count * sizeof(float);
  for (size_t i = 0; i < n; ++i) {
    h = (h ^ bytes[i]) * 1099511628211ULL;
  }
  return h;
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
  // Same relation for the stats' own stamp: freshly produced stats belong to the bundle's
  // generation. (On carry-forward stats_epoch may deliberately LAG, exactly like payload_epoch.)
  EXPECT_EQ(s->stats_epoch, done_epoch);
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
    lumice::test::ScopedResultFrame frame(server);
    LUMICE_FrameGetStats(frame.get(), &stats);
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

// ---- Tests 8a/8b: the two per-resume fields that test 6c structurally CANNOT reach ----
// ResetPerResumeState() re-arms three fields at every kPaused->kRunning edge. Test 6c
// (completed_below_threshold_force_uploads) pins `uploaded_since_resume_` — deleting that line
// makes it fail. The other two are invisible to it, and NOT by accident:
//
// 6c resumes against a COMPLETED run, and on a COMPLETED run `force_final_upload`
// (== COMPLETED && !uploaded_since_resume_ && buffer) is true on the first post-resume poll.
// That single flag both (a) enters the materialize branch on its own, making `last_generation_ = 0`
// redundant for has_new_snapshot, and (b) bypasses the quality gate outright, making
// `last_quality_pass_time_`'s 500ms timeout unreachable. So under COMPLETED, `uploaded_since_resume_`
// MASKS its two siblings: delete either one alone and every existing case stays green (measured, not
// assumed — each line was deleted individually against the full suite).
//
// The masking lifts in exactly one state: lifecycle == IDLE with a materialized snapshot still
// cached. LUMICE_StopServer resets has_ever_consumed_, so GetSimLifecycle reports IDLE rather than
// COMPLETED (server.cpp GetSimLifecycle), while snapshot_generation_ deliberately survives — "NOT
// reset on Stop (poller resets its own tracker)" (server.cpp declaration comment). That comment names
// this exact contract: the SERVER keeps its counter, so the POLLER must clear its own on resume.
// These two cases are that contract's red-state guard.
//
// Both drive the production WakeForRefresh seam (not the ResetGenerationForTest test seam), so they
// fail if the reset is dropped from the production path only.

// ---- Tests 9a/9b: stats carry their OWN generation stamp, so a stale carry-forward is rejected ----
// The bug: PollOnce() re-stamps next->epoch with the CURRENT lifecycle epoch on every publish, while
// the four stats fields are carried forward from prev when this poll produced none. After a restart
// those two facts combine into a snapshot whose stats belong to the PREVIOUS run but whose bundle
// epoch says "current" — and the consumer's gate (epoch match + rays > 0) is satisfied by exactly
// that combination, so the status bar showed the previous run's ray/crystal/sampling numbers.
//
// Measured server-side mechanics this rests on (probe run against a live server, not inference):
//   after RunFiniteToCompletion then a re-commit, BEFORE the new run produces anything —
//     epoch: 1 -> 2      (RawXyzResult.epoch_ is committed_epoch_ read LIVE in the getter,
//                         server.cpp; it is not stored with the snapshot)
//     cached stats: unchanged, still the previous run's 200000 rays
//                        (ServerImpl::Stop resets snapshot_dirty_/has_ever_consumed_ but does NOT
//                         clear cached_stats_result_, which only DoSnapshot() overwrites)
//     has_valid_data: 1 -> 0   <-- the ONLY field that distinguishes stale cache from fresh data
// So "stamp the stats with xyz_results[0].epoch" alone would stamp the STALE stats with the NEW
// epoch and change nothing. Freshness must be decided by has_valid_data; the epoch stamp then
// records which generation those stats were actually produced under and travels with them.
//
// 9a pins the consumer predicate; 9b pins the producer end-to-end against a real server.

// 9a — the pure gate. Mirrors ShouldUploadPayload's texture-side pattern on the stats side.
TEST(GuiLifecycle, stats_apply_gate_rejects_stale_generation) {
  constexpr uint64_t kOldGen = 7;
  constexpr uint64_t kNewGen = 8;

  // The bug scenario: stats carried forward from the previous generation. The BUNDLE epoch would
  // say kNewGen here (PollOnce re-stamps it every poll) — the gate must key on the stats' own stamp.
  {
    gui::PreviewSnapshot snap;
    snap.valid = true;
    snap.epoch = kNewGen;  // bundle epoch: current, and deliberately NOT what the gate reads
    snap.stats_epoch = kOldGen;
    snap.stats_sim_ray_num = 200000;  // a plausible non-zero leftover from the previous run
    EXPECT_TRUE(!gui::ShouldApplyStats(snap, kNewGen));
  }

  // Fresh stats produced under the committed generation: applied.
  {
    gui::PreviewSnapshot snap;
    snap.valid = true;
    snap.epoch = kNewGen;
    snap.stats_epoch = kNewGen;
    snap.stats_sim_ray_num = 1234;
    EXPECT_TRUE(gui::ShouldApplyStats(snap, kNewGen));
  }

  // The pre-existing lower bound must survive the change: zero rays never overwrite a shown value
  // (this is what keeps a torn zero off the status bar, and it is why the carry-forward exists).
  {
    gui::PreviewSnapshot snap;
    snap.valid = true;
    snap.epoch = kNewGen;
    snap.stats_epoch = kNewGen;
    snap.stats_sim_ray_num = 0;
    EXPECT_TRUE(!gui::ShouldApplyStats(snap, kNewGen));
  }
}

// 9b — the producer, end-to-end against a real server. This is the case that reproduces the actual
// user-visible defect: it drives the real Run -> restart edge and asserts the published bundle does
// not present the previous run's stats as belonging to the newly committed generation.
TEST(GuiLifecycle, restart_does_not_republish_prior_run_stats) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  const unsigned long long epoch_a = CurrentEpoch(server);
  LUMICE_RayCount rays_a = 0;
  {
    auto sa = local.LoadSnapshot();
    ASSERT_TRUE(sa != nullptr);
    ASSERT_TRUE(sa->stats_sim_ray_num > 0);
    EXPECT_EQ(sa->stats_epoch, epoch_a);               // fresh stats are stamped with the generation that made them
    EXPECT_TRUE(gui::ShouldApplyStats(*sa, epoch_a));  // ... and are applied
    rays_a = sa->stats_sim_ray_num;
  }

  // The Run -> Run edge: re-commit mints a new epoch. The server's cached stats still hold run A's
  // numbers at this instant (measured above), which is precisely the window the bug lived in.
  ASSERT_EQ(CommitJsonConfig(server, kFiniteConfig), LUMICE_OK);
  const unsigned long long epoch_b = CurrentEpoch(server);
  ASSERT_TRUE(epoch_b != epoch_a);  // the restart really did mint a new generation

  // Pin the bug's window open DETERMINISTICALLY instead of racing run B's first batch. Two facts
  // make this exact rather than lucky:
  //   - LUMICE_StopServer unconditionally clears has_ever_consumed_, so has_valid_data reads 0
  //     whether or not run B managed to consume anything first.
  //   - cached_stats_result_ is only ever overwritten inside DoSnapshot(), and DoSnapshot only runs
  //     from a Get*Results call. Nothing between the commit above and this line makes one, so the
  //     server's cached stats are still run A's — no timing assumption.
  // An earlier draft of this case polled straight after the commit and asserted on whichever state
  // it happened to find. That version PASSED against the un-fixed code: run B routinely produced
  // fresh stats before the poll, so the case never entered the branch it existed to check.
  LUMICE_StopServer(server);

  local.ResetGenerationForTest();  // what a resume does to the generation tracker, minus the worker
  local.PollOnceForTest(server);

  auto sb = local.LoadSnapshot();
  ASSERT_TRUE(sb != nullptr);
  // The bug's exact shape: the bundle epoch is the NEW generation's...
  EXPECT_EQ(sb->epoch, epoch_b);
  // ... while the stats riding in that bundle are still run A's, and say so.
  EXPECT_EQ(sb->stats_sim_ray_num, rays_a);
  EXPECT_EQ(sb->stats_epoch, epoch_a);
  // ... so the gate keeps them off the status bar. Before the fix this combination read as fresh
  // (bundle epoch == committed epoch, rays > 0) and run A's counts were displayed under run B.
  EXPECT_TRUE(!gui::ShouldApplyStats(*sb, epoch_b));

  local.Stop();
  LUMICE_DestroyServer(server);
}

// 8a — `last_generation_ = 0`: after a resume, a snapshot the poller has ALREADY consumed must be
// treated as new again, so a display-time refresh re-materializes the frame it is refreshing.
// Without the reset, has_new_snapshot stays false (the server's generation counter did not move)
// and — with force_final_upload structurally false under IDLE — the refresh produces nothing.
TEST(GuiLifecycle, resume_rearms_generation_tracker) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);  // consumes generation G: last_generation_ = G, uploaded = true
  auto s0 = local.LoadSnapshot();
  ASSERT_TRUE(s0 != nullptr);
  ASSERT_TRUE(s0->payload != nullptr);
  const unsigned long long serial0 = s0->texture_serial;
  EXPECT_TRUE(serial0 != 0);

  // Freeze into IDLE-with-data: this is what removes force_final_upload from the picture and
  // leaves last_generation_ as the ONLY thing that can re-open the materialize branch.
  LUMICE_StopServer(server);
  {
    LUMICE_SimLifecycleResult lc{};
    EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
    ASSERT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_IDLE));  // NOT COMPLETED — see header
  }

  // Control: without a resume, a poll in this state materializes nothing (same generation, and no
  // rescue). This is the "before" half of the red-state pair — it proves the assertion below is
  // driven by the resume and not by the server handing out a fresh generation on its own.
  local.PollOnceForTest(server);
  {
    auto s1 = local.LoadSnapshot();
    ASSERT_TRUE(s1 != nullptr);
    EXPECT_EQ(s1->texture_serial, serial0);
  }

  // Resume through the production seam: ResetPerResumeState() clears last_generation_, so the
  // already-consumed generation G reads as new and the frame re-materializes.
  local.WakeForRefresh(server);
  local.Stop();  // fence the worker thread (same discipline as test 6c phases B/C)
  local.PollOnceForTest(server);
  {
    auto s2 = local.LoadSnapshot();
    ASSERT_TRUE(s2 != nullptr);
    EXPECT_TRUE(s2->texture_serial != serial0);  // RED if `last_generation_ = 0` is dropped
  }

  local.Stop();
  LUMICE_DestroyServer(server);
}

// 8b — `last_quality_pass_time_`: the gate's 500ms timeout fallback must be measured from the
// RESUME, not from whenever the gate last passed in a previous run. Without the reset, time the
// poller spent paused (a user reading the last result before hitting Run again — seconds, routinely)
// is charged against the new run's budget, so its first sparse poll is force-uploaded and the
// anti-flicker gate is silently disarmed exactly when it is supposed to engage.
TEST(GuiLifecycle, resume_rearms_quality_gate_clock) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  auto s0 = local.LoadSnapshot();
  ASSERT_TRUE(s0 != nullptr);
  ASSERT_TRUE(s0->payload != nullptr);
  const unsigned long long serial0 = s0->texture_serial;

  // Same IDLE-with-data freeze as 8a, for the same reason: force_final_upload must be off the table
  // or it bypasses the gate before the timeout is ever consulted.
  LUMICE_StopServer(server);
  {
    LUMICE_SimLifecycleResult lc{};
    EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
    ASSERT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_IDLE));
  }

  // Arm the gate strictly above the ray count this run achieved, so the post-resume poll is one the
  // gate MUST reject. Derived from the measurement rather than hard-coded (same rationale as 6c).
  LUMICE_RayCount rays = 0;
  {
    LUMICE_StatsResult stats{};
    lumice::test::ScopedResultFrame frame(server);
    LUMICE_FrameGetStats(frame.get(), &stats);
    rays = stats.sim_ray_num;
    local.SetCalibratedThreshold(stats.sim_ray_num + 1);
  }
  EXPECT_TRUE(rays > 0);  // a zero-ray run takes the cold-start bypass and tests nothing

  // Spend more than the whole timeout budget while PAUSED. A correct reset makes this irrelevant;
  // a missing one makes it decisive.
  std::this_thread::sleep_for(std::chrono::milliseconds(gui::kQualityGateTimeoutMs + 100));

  local.WakeForRefresh(server);
  local.Stop();
  local.PollOnceForTest(server);

  // The gate rejects: the resume restarted the timeout clock, so ~0ms of the 500ms budget is spent.
  // Drop `last_quality_pass_time_ = now()` and the paused time above is charged instead, the timeout
  // fires, and this sparse frame is force-uploaded under a fresh serial.
  auto s1 = local.LoadSnapshot();
  ASSERT_TRUE(s1 != nullptr);
  EXPECT_EQ(s1->texture_serial, serial0);  // RED if `last_quality_pass_time_ = now()` is dropped

  local.Stop();
  LUMICE_DestroyServer(server);
}

// 10a — the texture-channel sibling of 9b, over the same Run -> restart edge and the same
// deterministically-pinned window. 9b showed the server's cached STATS survive a restart; the
// pixels survive it too, and for a different reason:
//   - RenderConsumer::Reset() deliberately does NOT zero snapshot_xyz_ ("PrepareSnapshot will
//     memcpy over it"), and PrepareSnapshot only runs when DoSnapshot sees snapshot_dirty_, which
//     Stop() cleared. So in the window the buffer still holds the PREVIOUS run's image, byte for
//     byte, and xyz_buffer is non-null because Stop() explicitly does not clear consumers_.
//   - RawXyzResult::epoch is stamped from the LIVE committed_epoch_ on every read, which the commit
//     already bumped. So the pair handed to the poller is "new generation number, old image".
// Neither value is torn — each is complete and self-consistent; it is the PAIRING that is wrong,
// which is why the "no new-epoch-with-stale-data tear can occur" reasoning at the stamp site does
// not cover this.
//
// The downstream epoch floor cannot separate the two: MarkStructHardDirty raises the floor to the
// OLD committed epoch precisely so the old generation's frames are fenced out until the new one
// produces something — but a payload mis-stamped with the NEW epoch clears that floor exactly like
// a genuinely fresh frame would. The floor filters by generation NUMBER; it has no way to ask
// whether the content was really produced under it. So the gate has to be upstream, at the point
// where a payload is allowed to be constructed at all.
//
// Asserts the CONTENT fingerprint, not only the stamp: a case that checked payload_epoch alone
// would pass for the wrong reason on any run where the timing happened not to line up.
TEST(GuiLifecycle, restart_does_not_republish_prior_run_pixels) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  const unsigned long long epoch_a = CurrentEpoch(server);
  uint64_t pixels_a = 0;
  unsigned long long serial_a = 0;
  {
    auto sa = local.LoadSnapshot();
    ASSERT_TRUE(sa != nullptr);
    ASSERT_TRUE(sa->payload != nullptr);
    EXPECT_EQ(sa->payload->payload_epoch, epoch_a);
    EXPECT_TRUE(sa->payload->texture_ray_count > 0);
    pixels_a =
        PixelFingerprint(sa->payload->xyz_buffer, static_cast<size_t>(sa->payload->width) * sa->payload->height * 3);
    serial_a = sa->texture_serial;
  }

  // The Run -> Run edge mints a new generation while the consumer's snapshot buffer is untouched.
  ASSERT_EQ(CommitJsonConfig(server, kFiniteConfig), LUMICE_OK);
  const unsigned long long epoch_b = CurrentEpoch(server);
  ASSERT_TRUE(epoch_b > epoch_a);

  // Pin the window open deterministically rather than racing run B's first batch — same two facts
  // 9b relies on, applied to the pixel channel: Stop() unconditionally clears has_ever_consumed_
  // (so has_valid_data reads 0 whether or not run B got anything in first) and clears
  // snapshot_dirty_ (so no DoSnapshot from here on can refresh snapshot_xyz_). No sleep, no
  // "at least one hit in N tries": the window is held by construction.
  LUMICE_StopServer(server);

  local.ResetGenerationForTest();  // what a resume does to the generation tracker, minus the worker
  local.PollOnceForTest(server);

  auto sb = local.LoadSnapshot();
  ASSERT_TRUE(sb != nullptr);
  ASSERT_TRUE(sb->payload != nullptr);

  // The image in the window IS run A's, in both worlds — this is the premise of the case, not the
  // defect. What must not happen is that image being re-published as run B's work.
  EXPECT_EQ(
      PixelFingerprint(sb->payload->xyz_buffer, static_cast<size_t>(sb->payload->width) * sb->payload->height * 3),
      pixels_a);

  // The defect's exact shape, all four RED before the fix: the poller materializes a "fresh"
  // payload out of run A's pixels, stamps it epoch_b, and mints a new serial for it.
  EXPECT_EQ(sb->payload->payload_epoch, epoch_a);  // carried forward, keeping its own generation
  EXPECT_EQ(sb->texture_serial, serial_a);         // carry-forward keeps the serial (anti-flicker)
  EXPECT_TRUE(!sb->has_new_texture);
  // ... and therefore the display gate keeps it off screen. Before the fix this returned true:
  // epoch_b clears a floor raised to epoch_a, and the serial is unseen, so the fence
  // MarkStructHardDirty put up around the old generation is defeated by the mis-stamp.
  EXPECT_TRUE(!gui::ShouldUploadPayload(*sb, serial_a, /*display_epoch_floor=*/epoch_a));

  // The suppression must be scoped to the window, not permanent: once a generation actually
  // produces data, its frame reaches the screen normally.
  ASSERT_EQ(CommitJsonConfig(server, kFiniteConfig), LUMICE_OK);
  const unsigned long long epoch_c = CurrentEpoch(server);
  ASSERT_TRUE(WaitForValidData(server));
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  {
    auto sc = local.LoadSnapshot();
    ASSERT_TRUE(sc != nullptr);
    ASSERT_TRUE(sc->payload != nullptr);
    EXPECT_EQ(sc->payload->payload_epoch, epoch_c);
    EXPECT_TRUE(sc->texture_serial != serial_a);
    EXPECT_TRUE(sc->has_new_texture);
    EXPECT_TRUE(gui::ShouldUploadPayload(*sc, serial_a, /*display_epoch_floor=*/epoch_a));
  }

  local.Stop();
  LUMICE_DestroyServer(server);
}

// 10c — carry-forward stays a pure refcount bump.
//
// The payload no longer owns its pixels: it holds a share of the result frame and points into it.
// That makes the anti-flicker carry-forward path (`next->payload = prev->payload`) load-bearing in
// a way it was not before — if anyone ever rewrote it to build a fresh payload out of the previous
// one, the rebuilt payload would either re-copy every pixel (the cost this whole change removed) or
// carry buffer pointers without the frame that keeps them alive. Neither shows up in the pixel or
// epoch assertions the sibling cases make: a deep copy compares byte-identical.
//
// So this case asserts the one thing those cannot — that the two polls hand out the SAME
// TexturePayload OBJECT. Pointer identity is the whole assertion; it is red for a deep copy and
// green only for a shared pointer.
TEST(GuiLifecycle, carry_forward_reuses_payload_pointer_not_copy) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  const bool completed = RunFiniteToCompletion(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);  // materializes a payload for this generation

  auto snap1 = local.LoadSnapshot();
  ASSERT_TRUE(snap1 != nullptr);
  ASSERT_TRUE(snap1->payload != nullptr);
  ASSERT_TRUE(snap1->has_new_texture);
  const gui::TexturePayload* first = snap1->payload.get();
  const float* first_pixels = snap1->payload->xyz_buffer;

  // Same generation, and this resume has already put a frame on screen — so neither the
  // new-snapshot branch nor the terminal-frame rescue can fire, and the publish below MUST take the
  // carry-forward branch. No ResetGenerationForTest() in between: that is what a resume does, and a
  // resume is exactly what this case must not have.
  local.PollOnceForTest(server);

  auto snap2 = local.LoadSnapshot();
  ASSERT_TRUE(snap2 != nullptr);
  ASSERT_TRUE(snap2->payload != nullptr);
  EXPECT_TRUE(!snap2->has_new_texture);  // the carry-forward branch really was the one taken
  EXPECT_EQ(snap2->payload.get(), first);
  EXPECT_EQ(snap2->payload->xyz_buffer, first_pixels);
  EXPECT_EQ(snap2->texture_serial, snap1->texture_serial);
  // The frame share travels with the payload, so the pixels stay readable for as long as either
  // snapshot is held — which is what makes pointing at them instead of copying them safe.
  EXPECT_TRUE(snap2->payload->frame != nullptr);

  local.Stop();
  LUMICE_DestroyServer(server);
}

// 10b — the composite (raypath-color) half of 10a. Not a courtesy duplicate: the composite payload
// fields are assigned INSIDE the very same `if (quality_ok)` block that builds the XYZ payload, so
// whatever gates one gates the other. This pins both directions of that coupling — the window must not publish a
// composite built from the previous run's lanes, and the window ending must not leave the composite
// path suppressed. It also checks the composite fire-gate's mode_changed OR-branch, which reads
// snap.payload and could otherwise fire an upload on a carried-forward frame.
TEST(GuiLifecycle, restart_window_does_not_republish_prior_run_composite) {
  gui::g_server_poller.Stop();
  gui::g_server = nullptr;

  // Match-all red class over a finite single-prism scene, so the RenderConsumer's ColoredMask() is
  // non-zero and DoSnapshot Phase-2 produces a composite for the generation (the img_buffer
  // sentinel the poller keys on). Copied from test_composite_preview.cpp's kColorConfig rather than
  // adapted from kFiniteConfig above: a class matching `layer: 0` needs rays that TERMINATE at
  // layer 0, so kFiniteConfig's scattering prob of 1.0 yields an empty mask and a non-composite
  // payload — which makes the case silently stop testing the composite path at all.
  const char* kColorConfig = R"({
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
      "ray_num": 200000,
      "max_hits": 8,
      "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{
      "id": 1,
      "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
      "resolution": [128, 64],
      "view": {"elevation": 0, "azimuth": 0, "roll": 0},
      "visible": "full", "background": [0, 0, 0],
      "opacity": 1.0, "intensity_factor": 1.0
    }],
    "raypath_color": {
      "mode": "dominant",
      "classes": [
        {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
      ]
    }
  })";

  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  ASSERT_EQ(CommitJsonConfig(server, kColorConfig), LUMICE_OK);
  const bool completed = WaitForValidData(server);
  EXPECT_TRUE(completed);
  if (!completed) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  const unsigned long long epoch_a = CurrentEpoch(server);
  unsigned long long serial_a = 0;
  {
    auto sa = local.LoadSnapshot();
    ASSERT_TRUE(sa != nullptr);
    ASSERT_TRUE(sa->payload != nullptr);
    // If this fails the case is not exercising the composite path at all and the rest proves nothing.
    ASSERT_TRUE(sa->payload->is_composite);
    ASSERT_TRUE(sa->payload->rgb_buffer != nullptr);
    EXPECT_EQ(sa->payload->payload_epoch, epoch_a);
    serial_a = sa->texture_serial;
  }

  ASSERT_EQ(CommitJsonConfig(server, kColorConfig), LUMICE_OK);
  const unsigned long long epoch_b = CurrentEpoch(server);
  ASSERT_TRUE(epoch_b > epoch_a);
  LUMICE_StopServer(server);

  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  {
    auto sb = local.LoadSnapshot();
    ASSERT_TRUE(sb != nullptr);
    ASSERT_TRUE(sb->payload != nullptr);
    // Carried forward, so it stays composite and keeps run A's stamp — the window publishes no new
    // composite rather than one rebuilt from the previous generation's lanes.
    EXPECT_TRUE(sb->payload->is_composite);
    EXPECT_EQ(sb->payload->payload_epoch, epoch_a);
    EXPECT_EQ(sb->texture_serial, serial_a);
    // The fire gate stays quiet: the serial-dedup branch is false (same serial, floor not cleared by
    // the carried-forward stamp) and mode_changed is false because the carried-forward payload is
    // composite exactly as the last upload was. A payload absent from this branch is what would
    // make mode_changed misfire, so it is asserted here rather than assumed.
    EXPECT_TRUE(!gui::ShouldFireCompositeUpload(*sb, serial_a, /*display_epoch_floor=*/epoch_a,
                                                /*show_composite_preview=*/true,
                                                /*last_uploaded_as_composite=*/true));
  }

  // Window ends: the composite path recovers, it is not suppressed past the window.
  ASSERT_EQ(CommitJsonConfig(server, kColorConfig), LUMICE_OK);
  const unsigned long long epoch_c = CurrentEpoch(server);
  ASSERT_TRUE(WaitForValidData(server));
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  {
    auto sc = local.LoadSnapshot();
    ASSERT_TRUE(sc != nullptr);
    ASSERT_TRUE(sc->payload != nullptr);
    EXPECT_TRUE(sc->payload->is_composite);
    EXPECT_TRUE(sc->payload->rgb_buffer != nullptr);
    EXPECT_EQ(sc->payload->payload_epoch, epoch_c);
    EXPECT_TRUE(gui::ShouldFireCompositeUpload(*sc, serial_a, /*display_epoch_floor=*/epoch_a,
                                               /*show_composite_preview=*/true,
                                               /*last_uploaded_as_composite=*/true));
  }

  local.Stop();
  LUMICE_DestroyServer(server);
}

// ---- Tests 11a-11e: the terminal observation is level-triggered, not a single chance ----
//
// The defect family these pin: PollOnce() used to stop the worker the instant it saw
// lifecycle==COMPLETED, and the justification on record ("the last published snapshot carries
// COMPLETED, so the reconcile still reaches kDone") only covered the sim_state projection. The same
// bundle also carries four stats counters, which are carried FORWARD when a poll produces nothing
// new — so a poll that observed COMPLETED before the CONSUMER had drained published a partial total
// and then stopped, with no later poll to correct it. COMPLETED is a producer-side verdict; it asks
// nothing about the consumer's queue.
//
// Two halves, and the ledger that produced this task requires both — fixing either alone leaves the
// other live ("a more accurate guard that is still one-shot", or "a heartbeat behind a guard that
// still decides wrong"):
//   I3a — the guard now also requires the drain signal (11a truth table, 11b end-to-end, 11c the
//         end-to-end negative).
//   I3b — after self-pausing the worker keeps a slow heartbeat instead of falling silent, so a
//         wrong belief is recoverable rather than terminal (11d).
// 11e guards the safety property the heartbeat introduces: Stop()'s callers destroy the server
// right after it returns, so a heartbeat tick must never outlive it.
//
// What is NOT pinned here, stated rather than papered over: the COMPLETED-but-not-yet-drained
// combination cannot be constructed on demand. It is a race window with no injection seam, and it
// does not reproduce on macOS/Metal (the CI symptom was Linux + Mesa). 11a covers that row as pure
// logic; the end-to-end evidence for it is the containerized repro in the task's verification plan,
// not this file.

namespace {

// Small finite run whose totals are exactly checkable, mirroring test_drain_contract.cpp's config
// and for the same reason: scattering prob 0.0 keeps orientation_num and ray_num strictly 1:1, and
// GenerateScene issues 128-ray dispatch grains, so any grain the consumer has not taken shows up as
// a recognizable shortfall rather than as noise. That shortfall — 19616 against 20000 — is the
// measured symptom this whole task exists to close.
const char* kExactTotalsConfig = R"({
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
    "id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0], "opacity": 1.0, "intensity_factor": 1.0
  }]
})";
constexpr LUMICE_RayCount kExactTotalsSimRays = 20000;

// Never-ending variant of kFiniteConfig: production never stops, so the run never completes and
// never drains. Used as the end-to-end negative for the self-pause guard.
const char* kInfiniteConfig = R"({
  "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.0}}],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0, "diameter": 0.5, "spectrum": "D65"},
    "ray_num": "infinite",
    "max_hits": 8,
    "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{"id": 1, "lens": {"type": "rectangular", "fov": 180.0},
              "resolution": [256, 128], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
              "visible": "full", "background": [0, 0, 0], "opacity": 1.0, "intensity_factor": 1.0}]
})";

// Spin until `pred` holds or the real-time budget runs out. Returns whether it held. Real time,
// deliberately: the heartbeat is driven by steady_clock inside the worker, so nothing in the test
// process can advance it.
template <typename Pred>
bool WaitFor(Pred pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

// Block until the server reports the current epoch fully drained (LUMICE_GetDrainStatus).
// Distinct from WaitForValidData above, which waits for the PRODUCER-side IDLE verdict — the whole
// point of these cases is that those two moments are not the same moment.
bool WaitForDrained(LUMICE_Server* server, int timeout_ms) {
  return WaitFor(
      [server] {
        LUMICE_DrainResult d{};
        LUMICE_GetDrainStatus(server, &d);
        return d.current_epoch != 0 && d.drained_epoch == d.current_epoch;
      },
      timeout_ms);
}

}  // namespace

// ---- 11a: the self-pause predicate, as a truth table (I3a) ----
// Pure — no server, no worker, no timing. Every row is a decision the live code cannot be made to
// take on demand, which is exactly why the predicate was extracted as a free function.
TEST(GuiLifecycle, self_pause_predicate_truth_table) {
  constexpr unsigned long long kN = 42;
  auto lc = [](int lifecycle, unsigned long long epoch) {
    LUMICE_SimLifecycleResult v{};
    v.lifecycle = lifecycle;
    v.epoch = epoch;
    return v;
  };
  auto drain = [](unsigned long long drained, unsigned long long current) {
    LUMICE_DrainResult v{};
    v.drained_epoch = drained;
    v.current_epoch = current;
    return v;
  };

  // 1-2: not the terminal edge. A running sim and a reset/idle server are both mid-flight; drained
  // numbers that happen to line up must not be read as completion.
  EXPECT_FALSE(gui::ShouldSelfPause(lc(LUMICE_LIFECYCLE_RUNNING, kN), drain(kN, kN)));
  EXPECT_FALSE(gui::ShouldSelfPause(lc(LUMICE_LIFECYCLE_IDLE, kN), drain(kN, kN)));

  // 3: the only true row — completed, drained, and nothing newer committed.
  EXPECT_TRUE(gui::ShouldSelfPause(lc(LUMICE_LIFECYCLE_COMPLETED, kN), drain(kN, kN)));

  // 4: THE row this task exists for. COMPLETED is a producer-side verdict and the consumer is still
  // draining, so the stats visible now are a partial total. Deleting the drained_epoch term makes
  // this the pre-fix behavior — worker stops, partial counts freeze on the status bar.
  EXPECT_FALSE(gui::ShouldSelfPause(lc(LUMICE_LIFECYCLE_COMPLETED, kN), drain(kN - 1, kN)));

  // 5-6: a NEWER epoch was committed between the lifecycle read at the top of PollOnce and the
  // drain read at the bottom (CommitScene + WakeForRestart on the main thread). Row 5 is the one
  // that catches an implementation testing only drained_epoch: drained == lc.epoch is satisfied
  // there, and monotonicity means it STAYS satisfied after the commit — so without the
  // current_epoch term the poller would pause a generation that just started.
  EXPECT_FALSE(gui::ShouldSelfPause(lc(LUMICE_LIFECYCLE_COMPLETED, kN), drain(kN, kN + 1)));
  EXPECT_FALSE(gui::ShouldSelfPause(lc(LUMICE_LIFECYCLE_COMPLETED, kN), drain(kN + 1, kN + 1)));
}

// ---- 11b: a poll taken at the drained moment publishes FINAL totals (I3a, end-to-end) ----
// Wiring test for the predicate: the truth table proves the decision is right, this proves PollOnce
// feeds it the real signals and that the bundle published under that decision carries the true
// totals rather than a partial sum. Deterministic because it waits for the drain signal first — the
// half that cannot be made deterministic (COMPLETED before drained) is called out in the block
// comment above.
TEST(GuiLifecycle, self_pause_publishes_final_totals) {
  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  ASSERT_EQ(CommitJsonConfig(server, kExactTotalsConfig), LUMICE_OK);
  ASSERT_TRUE(WaitForDrained(server, 30000)) << "epoch never reported drained";

  LUMICE_SimLifecycleResult lc{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));

  // Independent cross-check, not a re-read of what the poller believes: ask the server directly.
  // A test that only asserted on the poller's own published bundle would pass just as happily if
  // the poller and the code under test shared the same wrong assumption.
  LUMICE_DrainResult drain{};
  ASSERT_EQ(LUMICE_GetDrainStatus(server, &drain), LUMICE_OK);
  EXPECT_EQ(drain.drained_epoch, lc.epoch);
  EXPECT_EQ(drain.current_epoch, lc.epoch);
  EXPECT_TRUE(gui::ShouldSelfPause(lc, drain));

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  EXPECT_EQ(snap->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  // Exact, not approximate: with scattering prob 0.0 the totals are the configured ray budget, and
  // a missed 128-ray dispatch grain is visible as an exact shortfall.
  EXPECT_EQ(snap->stats_sim_ray_num, kExactTotalsSimRays);
  EXPECT_EQ(snap->stats_orientation_num, kExactTotalsSimRays);

  local.Stop();
  LUMICE_DestroyServer(server);
}

// ---- 11c: a still-producing run never self-pauses (I3a, end-to-end negative) ----
// The guard's other failure mode: always true. An unbounded run never completes and never drains,
// so the worker must stay in the full-speed loop — no heartbeat tick may ever fire. Runs the REAL
// worker thread (Start(), not the synchronous test seam) because "did the worker change cadence"
// is the property under test.
TEST(GuiLifecycle, running_sim_never_self_pauses) {
  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  ASSERT_EQ(CommitJsonConfig(server, kInfiniteConfig), LUMICE_OK);

  gui::ServerPoller local;
  local.Start(server);

  // Long enough for several heartbeat periods to have elapsed had the worker wrongly throttled:
  // a self-pause at any point in the first ~1.5s leaves at least two ticks behind.
  ASSERT_TRUE(WaitFor([&local] { return local.LoadSnapshot() != nullptr; }, 5000)) << "worker never polled";
  std::this_thread::sleep_for(std::chrono::milliseconds(3 * gui::kIdleHeartbeatIntervalMs));
  EXPECT_EQ(local.HeartbeatTickCountForTest(), 0u) << "an unbounded run self-paused";

  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_NE(snap->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));

  local.Stop();
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// ---- 11d: after self-pausing the worker keeps a slow heartbeat (I3b) ----
// I3's second clause, verbatim: idle may throttle to a slow heartbeat, but must not fall so silent
// it cannot self-heal. Before this, WorkerLoop sat in an untimed cv_.wait after self-pausing and
// all five wake entry points were user actions — so the single poll that observed the terminal edge
// was the ONLY chance to get the terminal truth on screen. The heartbeat is what makes that
// observation retryable, which is the whole difference between a level-triggered reconciler and an
// edge-triggered one.
//
// Also asserts what the heartbeat must NOT do: republish a texture. Each tick is a safe no-op —
// carry-forward, same serial — or the preview would re-upload twice a second forever.
TEST(GuiLifecycle, idle_heartbeat_ticks_after_self_pause) {
  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  ASSERT_EQ(CommitJsonConfig(server, kFiniteConfig), LUMICE_OK);

  gui::ServerPoller local;
  local.Start(server);

  // The first tick is itself the evidence of a self-pause: only the kIdleHeartbeat branch bumps
  // this counter, and only PollOnce's drain-aware guard can enter that state.
  ASSERT_TRUE(WaitFor([&local] { return local.HeartbeatTickCountForTest() >= 1; }, 30000))
      << "worker never self-paused into the heartbeat";

  auto before = local.LoadSnapshot();
  ASSERT_TRUE(before != nullptr);
  const unsigned long long serial_before = before->texture_serial;
  const LUMICE_RayCount rays_before = before->stats_sim_ray_num;
  const uint64_t ticks_before = local.HeartbeatTickCountForTest();

  std::this_thread::sleep_for(std::chrono::milliseconds(5 * gui::kIdleHeartbeatIntervalMs / 2));
  EXPECT_GE(local.HeartbeatTickCountForTest() - ticks_before, 2u) << "the heartbeat stopped after its first tick";

  auto after = local.LoadSnapshot();
  ASSERT_TRUE(after != nullptr);
  EXPECT_TRUE(after->valid);
  EXPECT_EQ(after->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_EQ(after->stats_sim_ray_num, rays_before);
  EXPECT_EQ(after->texture_serial, serial_before) << "a heartbeat tick bumped the texture serial";

  local.Stop();
  LUMICE_DestroyServer(server);
}

// ---- 11e: Stop() quiesces the heartbeat before returning (the contract the heartbeat endangers) ----
// Stop()'s callers destroy the server on the next line — MaybeReconstructServerForBackend does
// Stop() then LUMICE_DestroyServer, DoStop does Stop() then LUMICE_StopServer. Before the
// heartbeat, "self-paused" and "not touching the server" were the same state and Stop() could
// early-return on it. They are now different facts, and if Stop() still early-returned, a tick
// landing after it returned would dereference a destroyed server.
//
// The sequence below is that use-after-free: Stop(), then destroy, then wait out two heartbeat
// periods. This repository has no ASan/TSan build (verified: no -fsanitize anywhere under scripts/
// or the CMake tree), so the tick-count assertion is the signal that does not depend on one — and
// it is a genuine assertion rather than a hope that a crash shows up, since "no tick after Stop()
// returned" IS the contract. It is not, and does not claim to be, proof of memory safety; if a
// sanitizer build is ever added, this case belongs in it.
TEST(GuiLifecycle, stop_during_idle_heartbeat_quiesces_worker) {
  LUMICE_Server* server = LUMICE_CreateServer();
  ASSERT_TRUE(server != nullptr);
  ASSERT_EQ(CommitJsonConfig(server, kFiniteConfig), LUMICE_OK);

  gui::ServerPoller local;
  local.Start(server);

  // Establish the premise first. Without this, "no further ticks" below is vacuously true for a
  // worker that never entered the heartbeat at all.
  ASSERT_TRUE(WaitFor([&local] { return local.HeartbeatTickCountForTest() >= 1; }, 30000))
      << "worker never self-paused into the heartbeat";

  local.Stop();
  // Read the baseline AFTER Stop() returns, not before. Stop() is allowed to let an in-flight tick
  // finish — that is what waiting on active_ means — so a before/after comparison would go red on
  // the legitimate case of a tick that started microseconds before the call. The contract is about
  // what happens after the return, and that is what this measures.
  const uint64_t ticks_at_return = local.HeartbeatTickCountForTest();

  // The UAF window, if the contract were broken: from here on the worker has no valid server.
  LUMICE_DestroyServer(server);

  std::this_thread::sleep_for(std::chrono::milliseconds(2 * gui::kIdleHeartbeatIntervalMs));
  EXPECT_EQ(local.HeartbeatTickCountForTest(), ticks_at_return)
      << "a heartbeat tick ran after Stop() returned — its caller had already destroyed the server";
}
