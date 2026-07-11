// Regression guard for the "GPU finite run finished (backend IDLE) but GUI stuck in
// kSimulating" dead-state bug — see doc/gui-preview-lifecycle-architecture.md §2/§6 and
// scrum-gui-lifecycle-clock-decouple. Pins blueprint invariants I1 (epoch-keyed truth), I2
// (single-owner ReconcileSimState), I3 (level-triggered self-heal — the terminal completion edge
// is never lost) and I4 (lifecycle readable without an expensive snapshot generation).
//
// 1.5 removed the has_valid_data + server_state GUI side-signals: sim_state is now DERIVED once per
// frame by the pure ReconcileSimState(run_intent, committed_epoch, snapshot, dirty), and the
// terminal edge is the durable lifecycle==COMPLETED carried on every poll. The tests below drive
// the reconcile INPUTS (run_intent + committed_epoch) rather than writing sim_state directly.
//
// Tests:
//  1. gui_lifecycle/terminal_idle_reaches_done — DETERMINISTIC white-box. Reproduces the exact
//     poll/sync interleaving from the issue (a mid-run texture drop, then a terminal poll carrying
//     NO new snapshot generation) and asserts the completion edge still reaches kDone via the
//     single-owner reconcile. Boundary sub-checks pin that an IDLE transient (C1) and a stale-epoch
//     COMPLETED (C2) are NOT misclassified as done.
//  2. gui_lifecycle/gpu_run_reaches_done — INTEGRATION e2e through the real DoRun→completion flow.
//  3. gui_lifecycle/reconcile_truth_table — PURE unit test of ReconcileSimState (I2), no server/GL.
//  4. gui_lifecycle/anti_flicker_epoch_floor — PURE structural test of the epoch-floor upload gate
//     (I1/§3.3): MarkFilterDirty fences the old generation; MarkDirty carries it forward.
//  5. gui_lifecycle/optimistic_async_stop — INTEGRATION of the 1.6 async Stop (blueprint §5/§8):
//     DoStop returns immediately with run_intent==kStopping (optimistic, non-blocking); the reconcile
//     maps that to the kStopping display state; JoinPendingStop drains the background thread and the
//     next SyncFromPoller frames advance run_intent→kStopped and sim_state→kStopEndState (kDone) with
//     g_stop_inflight cleared. Uses JoinPendingStop (not sleep/poll) for determinism.

#include <chrono>
#include <memory>
#include <thread>

#include "IconsFontAwesome6.h"  // ICON_FA_* selectors for the real-UI-click AC1 regression (Test 8b).
#include "gui/server_poller.hpp"
#include "test_gui_shared.hpp"

namespace {

using SimState = gui::GuiState::SimState;
using RunIntent = gui::RunIntent;

// Small finite single-prism run: empty filter (all rays pass), rectangular lens, tiny
// resolution — completes in well under a second on CPU or Metal.
const char* kFiniteConfig = R"({
  "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "ratio": {"upper": 1.0, "lower": 1.0}}],
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
bool RunFiniteToCompletion(LUMICE_Server* server) {
  if (LUMICE_CommitConfig(server, kFiniteConfig) != LUMICE_OK) {
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

void RegisterLifecycleTests(ImGuiTestEngine* engine) {
  // ---- Test 1: deterministic white-box interleaving regression ----
  ImGuiTest* t = IM_REGISTER_TEST(engine, "gui_lifecycle", "terminal_idle_reaches_done");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Clean baseline: detach the global poller from any prior test's server and reset the reconcile
    // INPUTS (not sim_state — sim_state is derived).
    gui::g_server_poller.Stop();
    gui::g_server = nullptr;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
    gui::g_state.dirty = false;

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    bool completed = RunFiniteToCompletion(server);
    IM_CHECK(completed);  // finite run reached IDLE + has_valid_data (C-API contract)
    if (!completed) {
      LUMICE_DestroyServer(server);
      return;
    }

    // The terminal completion is readable as the explicit COMPLETED lifecycle with a minted epoch —
    // the SAME signal the poller now publishes and the reconcile keys on.
    const unsigned long long done_epoch = CurrentEpoch(server);
    {
      LUMICE_SimLifecycleResult lc{};
      IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
      IM_CHECK_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
      IM_CHECK(lc.epoch >= 1);
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
      IM_CHECK(a != nullptr);
      IM_CHECK(a->valid);
      IM_CHECK_EQ(a->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
      IM_CHECK_EQ(a->epoch, done_epoch);
      IM_CHECK(a->stats_sim_ray_num > 0);  // Poll A genuinely carried the generation-bearing frame

      local.PollOnceForTest(server);  // Poll B: same generation G -> has_new_snapshot == false
      auto b = local.LoadSnapshot();
      IM_CHECK(b != nullptr);
      IM_CHECK(b->valid);
      // I5 bundle coherence: lifecycle + epoch + stats CARRY FORWARD across a no-new-generation
      // poll instead of being torn. The terminal COMPLETED edge survives (the fix, I4).
      IM_CHECK_EQ(b->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
      IM_CHECK_EQ(b->epoch, done_epoch);
      IM_CHECK(b->stats_sim_ray_num > 0);
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
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
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
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));
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
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));
    }

    // Cleanup: leave a clean global state (inputs) for subsequent tests.
    gui::g_server_poller.Stop();
    gui::g_server = nullptr;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
    LUMICE_DestroyServer(server);
  };

  // ---- Test 2: integration e2e through the real DoRun -> completion flow ----
  ImGuiTest* t2 = IM_REGISTER_TEST(engine, "gui_lifecycle", "gpu_run_reaches_done");
  t2->TestFunc = [](ImGuiTestContext* ctx) {
    // Fresh server + baseline. Default GuiState already has a runnable single-prism scene.
    gui::g_server_poller.Stop();
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;  // re-establish backend-toggle detection invariant
    gui::g_state = gui::InitDefaultState();

    // Finite run so completion is a real terminal edge (not infinite accumulation).
    gui::g_state.sim.infinite = false;
    gui::g_state.sim.ray_num_millions = 0.5f;
    gui::g_state.sim.max_hits = 8;
#if defined(__APPLE__)
    gui::g_state.use_gpu_backend = true;  // -> Metal single engine via MaybeReconstructServerForBackend
#endif

    gui::DoRun();  // real product Run path: sets run_intent=kRunning + commits + starts poller.
    // DoRun no longer writes sim_state (it is reconcile-derived), so assert the INTENT it set.
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));

    // Drive the real display clock: each Yield runs a main-loop frame, which calls SyncFromPoller()
    // (test_gui_main.cpp) → ReconcileSimState. The finite run finishes and the GUI must reconcile to
    // kDone within the timeout — the behavior that used to hang.
    auto start = std::chrono::steady_clock::now();
    while (gui::g_state.sim_state != SimState::kDone) {
      ctx->Yield();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count();
      if (elapsed > 20) {
        break;
      }
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));

    // Cleanup
    gui::g_server_poller.Stop();
    if (gui::g_server) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
  };

  // ---- Test 3: pure ReconcileSimState truth table (I2), no server / GL ----
  // Row-by-row pin of the §1.3 table. This is the reconcile's single-owner contract; each row's
  // RED手法 is noted. Building a PreviewSnapshot by hand keeps this fully headless.
  ImGuiTest* t3 = IM_REGISTER_TEST(engine, "gui_lifecycle", "reconcile_truth_table");
  t3->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
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
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kNone, 0, nullptr, false)),
                static_cast<int>(SimState::kIdle));
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kNone, 0, nullptr, true)),
                static_cast<int>(SimState::kIdle));  // dirty never promotes kIdle

    // kLoaded → kDone; +dirty → kModified.
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kLoaded, 0, nullptr, false)),
                static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kLoaded, 0, nullptr, true)),
                static_cast<int>(SimState::kModified));

    // kStopped → kStopEndState (kDone, owner-decided 1.6); +dirty → kModified (kDone is demotable).
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopped, 7, nullptr, false)),
                static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopped, 7, nullptr, true)),
                static_cast<int>(SimState::kModified));

    // task-color-migration §3 D4 — kRunCompleted (latched natural completion) → kDone regardless
    // of the observation. This is the load-bearing row: even a valid=false observation (root cause
    // (a)) or a stale-epoch observation must NOT pull the latched terminal back to kSimulating.
    // RED手法: forgetting the case in the switch would fall through to a default and mis-derive.
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, nullptr, false)),
                static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, nullptr, true)),
                static_cast<int>(SimState::kModified));  // +dirty → kModified (kDone is demotable)
    {
      auto s_invalid = mk(false, 5, kDone_lc);
      // valid=false observation must NOT pull kRunCompleted back to kSimulating — this row is the
      // structural guarantee AC1's activity bug root cause (b) relied on and this test pins.
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, &s_invalid, false)),
                  static_cast<int>(SimState::kDone));
      auto s_stale = mk(true, 3, kDone_lc);  // stale epoch (< committed)
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, &s_stale, false)),
                  static_cast<int>(SimState::kDone));
      auto s_running = mk(true, 5, kRun_lc);  // even a fresh RUNNING observation stays latched
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunCompleted, 5, &s_running, false)),
                  static_cast<int>(SimState::kDone));
    }

    // kStopping (async Stop draining, 1.6) → kStopping, for ANY observation/dirty. Pure optimistic
    // intent: not pulled by a fresh COMPLETED, and NOT demoted by dirty (a draining run is not an
    // editable completed result). RED手法: mis-mapping the kStopping case to S::kSimulating turns
    // every row below RED.
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, nullptr, false)),
                static_cast<int>(SimState::kStopping));
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, nullptr, true)),
                static_cast<int>(SimState::kStopping));  // dirty does NOT demote kStopping
    {
      auto s = mk(true, 7, kDone_lc);  // even a fresh COMPLETED does not pull kStopping to kDone
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, &s, false)),
                  static_cast<int>(SimState::kStopping));
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kStopping, 7, &s, true)),
                  static_cast<int>(SimState::kStopping));
    }

    // kRunning, no observation yet → kSimulating.
    IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, nullptr, false)),
                static_cast<int>(SimState::kSimulating));

    // kRunning, fresh COMPLETED@match → kDone. (RED: invert the epoch compare, or ignore COMPLETED.)
    {
      auto s = mk(true, 5, kDone_lc);
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
                  static_cast<int>(SimState::kDone));
      // +dirty on a completed result → kModified.
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, true)),
                  static_cast<int>(SimState::kModified));
    }

    // kRunning, fresh RUNNING → kSimulating.
    {
      auto s = mk(true, 5, kRun_lc);
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
                  static_cast<int>(SimState::kSimulating));
    }

    // kRunning, fresh IDLE (C1 boundary) → kSimulating (IDLE ≠ completion).
    {
      auto s = mk(true, 5, kIdle_lc);
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
                  static_cast<int>(SimState::kSimulating));
    }

    // kRunning, STALE-epoch COMPLETED (C2 boundary, pins I1) → kSimulating.
    // RED手法: if `fresh` drops the `snap->epoch == committed_epoch` term, the stale COMPLETED@5 is
    // treated as completion for gen 6 → kDone → this assertion goes RED.
    {
      auto s = mk(true, 5, kDone_lc);  // observation is a generation behind
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, /*committed=*/6, &s, false)),
                  static_cast<int>(SimState::kSimulating));
    }

    // kRunning, invalid (valid=false) COMPLETED → kSimulating (not yet a real observation).
    {
      auto s = mk(false, 5, kDone_lc);
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(RunIntent::kRunning, 5, &s, false)),
                  static_cast<int>(SimState::kSimulating));
    }
  };

  // ---- Test 4: anti-flicker epoch-floor upload gate (I1/§3.3), pure/headless ----
  // Pins that MarkFilterDirty raises display_epoch_floor to fence the OLD generation's payload
  // (blocked by ShouldUploadPayload) while MarkDirty leaves the floor so a carried-forward payload
  // passes. Bites the real production predicate (ShouldUploadPayload) + the real MarkFilterDirty /
  // MarkDirty split — the mechanism behind "filter change clears + no stale refill" vs "crystal
  // scrub keeps the last frame with no black flicker", which manual Metal scrub can't check in CI.
  ImGuiTest* t4 = IM_REGISTER_TEST(engine, "gui_lifecycle", "anti_flicker_epoch_floor");
  t4->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
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

    // --- filter change: MarkFilterDirty raises the floor to committed_epoch ⇒ old payload BLOCKED.
    {
      gui::GuiState st;
      st.committed_epoch = kGen;
      st.display_epoch_floor = 0;
      st.MarkFilterDirty();
      IM_CHECK_EQ(st.display_epoch_floor, kGen);  // floor bumped to the current generation
      IM_CHECK_EQ(st.snapshot_intensity, 0.0f);   // immediate display clear preserved
      // payload_epoch (kGen) is NOT > floor (kGen) ⇒ gate rejects the stale texture.
      IM_CHECK(!gui::ShouldUploadPayload(snap, /*last_serial=*/0, st.display_epoch_floor));
    }

    // --- crystal scrub: MarkDirty leaves the floor at 0 ⇒ carried-forward old payload PASSES.
    {
      gui::GuiState st;
      st.committed_epoch = kGen;
      st.display_epoch_floor = 0;
      st.MarkDirty();
      IM_CHECK_EQ(st.display_epoch_floor, 0u);  // floor untouched (no fence)
      // payload_epoch (kGen) > floor (0) ⇒ gate accepts (anti-flicker carry-forward).
      IM_CHECK(gui::ShouldUploadPayload(snap, /*last_serial=*/0, st.display_epoch_floor));
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
      IM_CHECK(gui::ShouldUploadPayload(new_snap, /*last_serial=*/1, /*floor=*/kGen));
      // exact-once: an already-seen serial is rejected even when the epoch clears the floor.
      IM_CHECK(!gui::ShouldUploadPayload(new_snap, /*last_serial=*/2, /*floor=*/kGen));
    }
  };

  // ---- Test 4b: backend swap (CPU<->GPU) resets the display epoch fence ----
  // Pins the fix for "run on CPU, switch to GPU, click Run, preview does not update". A reconstructed
  // server restarts its epoch authority at 0, so its first frame carries a LOW epoch (1). If the old
  // server's display_epoch_floor (raised to >=1 by any prior filter edit) is carried across the swap,
  // ShouldUploadPayload fences the new backend's frame forever and the stale texture sticks on screen.
  // ResetDisplayGenerationForBackendSwap must clear the fence + carried texture so the new backend's
  // frame uploads. Bites the real production predicate + the real reset method (fully headless — no
  // GPU needed, since the bug is a pure epoch-bookkeeping defect independent of the backend kind).
  ImGuiTest* t4b = IM_REGISTER_TEST(engine, "gui_lifecycle", "backend_swap_resets_epoch_fence");
  t4b->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
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
    IM_CHECK(!gui::ShouldUploadPayload(fresh_snap, st.last_uploaded_texture_serial, st.display_epoch_floor));

    // FIX: the backend swap resets the display generation to epoch 0 and clears the carried texture.
    st.ResetDisplayGenerationForBackendSwap();
    IM_CHECK_EQ(st.committed_epoch, 0u);
    IM_CHECK_EQ(st.display_epoch_floor, 0u);
    IM_CHECK_EQ(st.last_uploaded_texture_serial, 0ull);
    IM_CHECK_EQ(st.snapshot_intensity, 0.0f);  // immediate clear of the stale texture

    // After the reset the new backend's epoch-1 frame clears the fence and uploads.
    IM_CHECK(gui::ShouldUploadPayload(fresh_snap, st.last_uploaded_texture_serial, st.display_epoch_floor));
  };

  // ---- Test 5: optimistic async Stop (1.6, blueprint §5/§8) ----
  // INTEGRATION through the real DoRun→DoStop→JoinPendingStop→SyncFromPoller flow on a live server.
  // Determinism: no sleep/poll guesses — run_intent is set synchronously by DoStop, the optimistic
  // display is checked via the pure reconcile (independent of the background drain timing), and the
  // terminal state is reached only after JoinPendingStop() has drained the background thread.
  ImGuiTest* t5 = IM_REGISTER_TEST(engine, "gui_lifecycle", "optimistic_async_stop");
  t5->TestFunc = [](ImGuiTestContext* ctx) {
    // Fresh CPU server + baseline. Infinite rays so the backend keeps running and the Stop happens
    // mid-run (the real Run→Stop transition the feature targets).
    gui::g_server_poller.Stop();
    gui::JoinPendingStop();  // clean any prior test's in-flight stop
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;
    gui::g_state = gui::InitDefaultState();
    gui::g_state.sim.infinite = true;

    gui::DoRun();  // real Run path: run_intent=kRunning, commit, start poller.
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));

    // Drive frames until the display reflects the running backend (kSimulating).
    {
      auto start = std::chrono::steady_clock::now();
      while (gui::g_state.sim_state != SimState::kSimulating) {
        ctx->Yield();
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 10) {
          break;
        }
      }
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));
    }

    // --- OPTIMISTIC / NON-BLOCKING: DoStop sets the intent synchronously and offloads the drain.
    gui::DoStop();
    // run_intent flips to kStopping synchronously (before any drain / SyncFromPoller). This is the
    // "immediate UI" guarantee — deterministic regardless of how fast the background thread runs
    // (the background thread never writes run_intent).
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopping));
    // The reconcile maps that intent to the kStopping display state (pure optimistic — needs no
    // fresh observation and is not demoted by dirty). This is what paints "Stopping…" that frame.
    {
      auto snap = gui::g_server_poller.LoadSnapshot();
      IM_CHECK_EQ(static_cast<int>(gui::ReconcileSimState(gui::g_state.run_intent, gui::g_state.committed_epoch,
                                                          snap.get(), gui::g_state.dirty)),
                  static_cast<int>(SimState::kStopping));
    }
    // Idempotent re-entry: a second DoStop while kStopping is a no-op (does not relaunch the future).
    gui::DoStop();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopping));

    // --- TERMINAL: join the background drain (deterministic, no sleep), then advance the intent.
    gui::JoinPendingStop();
    IM_CHECK(!gui::g_stop_inflight.load());  // background thread returned ⇒ backend drained
    // The intent-advance lives after the reconcile line in SyncFromPoller, so it takes two frames to
    // reach the terminal display: frame 1 advances kStopping→kStopped, frame 2 reconciles to kDone.
    for (int i = 0; i < 4 && gui::g_state.sim_state != SimState::kDone; ++i) {
      gui::SyncFromPoller();
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kStopped));
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));  // == kStopEndState
    IM_CHECK(!gui::g_stop_inflight.load());

    // Cleanup
    gui::g_server_poller.Stop();
    gui::JoinPendingStop();
    if (gui::g_server) {
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
  };

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
  ImGuiTest* t6 = IM_REGISTER_TEST(engine, "gui_lifecycle", "snapshot_bundle_coherence");
  t6->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    gui::g_server_poller.Stop();
    gui::g_server = nullptr;

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool completed = RunFiniteToCompletion(server);
    IM_CHECK(completed);
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
    IM_CHECK(s != nullptr);
    IM_CHECK(s->valid);

    // Bundle coherence: EVERY field of this one published value belongs to the SAME committed
    // generation (done_epoch); the terminal lifecycle is consistent with produced stats AND a
    // materialized payload. No field is torn across generations.
    IM_CHECK_EQ(s->epoch, done_epoch);
    IM_CHECK_EQ(s->lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    IM_CHECK(s->stats_sim_ray_num > 0);
    IM_CHECK(s->has_new_texture);     // this poll genuinely materialized a texture
    IM_CHECK(s->payload != nullptr);  // ... so the payload is present in the SAME bundle
    // The payload's OWN epoch stamp matches the bundle epoch on a fresh materialization — the
    // coherence relation section A omits. (On carry-forward payload_epoch may deliberately LAG the
    // bundle epoch; here the texture is freshly materialized so they MUST agree.)
    IM_CHECK_EQ(s->payload->payload_epoch, done_epoch);
    IM_CHECK(s->texture_serial != 0);  // a fresh monotonic serial was minted for this materialization

    // Two consecutive loads observe the SAME whole object (atomic pointer handoff, no partial
    // reconstruction between reads) — the consumer can never see a half-swapped value.
    auto s2 = local.LoadSnapshot();
    IM_CHECK(s2.get() == s.get());

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // ---- Test 6b: task-color-migration §3 D4 — kRunCompleted Mealy latch (integration) ----
  // Pins the SyncFromPoller-side Mealy edge: under a kRunning intent, the first fresh COMPLETED
  // observation at the committed epoch promotes the intent to kRunCompleted, and any later
  // valid=false observation cannot pull the reconciled state back to kSimulating (root cause (b)
  // of the AC1 activity bug — the structural belt paired with WakeForRefresh's suspenders).
  //
  // Uses the same DoRun-driven real-server flow as Test 2 (gpu_run_reaches_done) so the intent
  // advance goes through the actual production SyncFromPoller path, not a hand-rolled reconcile
  // call.
  ImGuiTest* t6b = IM_REGISTER_TEST(engine, "gui_lifecycle", "run_completed_latch_survives_invalid_snapshot");
  t6b->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Fresh server + baseline (mirrors gpu_run_reaches_done setup).
    gui::g_server_poller.Stop();
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;
    gui::g_state = gui::InitDefaultState();
    gui::g_state.sim.infinite = false;
    gui::g_state.sim.ray_num_millions = 0.5f;
    gui::g_state.sim.max_hits = 8;
#if defined(__APPLE__)
    gui::g_state.use_gpu_backend = true;
#endif

    gui::DoRun();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));

    // Drive SyncFromPoller until the reconcile settles on kDone (natural completion).
    for (int i = 0; i < 500 && static_cast<int>(gui::g_state.sim_state) != static_cast<int>(SimState::kDone); ++i) {
      ctx->Yield();
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));

    // Mealy latch invariant: on the first frame that observes fresh COMPLETED, SyncFromPoller
    // must have promoted the intent to kRunCompleted (kRunning → kRunCompleted). Give the poller
    // one more Yield to run the promotion path if it hasn't yet.
    for (int i = 0; i < 3 && gui::g_state.run_intent != RunIntent::kRunCompleted; ++i) {
      ctx->Yield();
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));

    // Load-bearing check: fabricate a valid=false observation on the global poller (the exact
    // shape that WakeForRestart's PublishValidReset used to produce, and that the AC1 root cause
    // (b) required to demote the completed state). With the kRunCompleted latch in effect, a
    // subsequent SyncFromPoller must NOT pull sim_state back to kSimulating — the intent-latched
    // terminal is structurally immune (this is the belt-and-suspenders duality with WakeForRefresh).
    gui::g_server_poller.PublishValidResetForTest();
    gui::SyncFromPoller();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));

    // Cleanup.
    gui::g_server_poller.Stop();
    LUMICE_DestroyServer(gui::g_server);
    gui::g_server = nullptr;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
  };

  // ---- Test 7: task-color-migration M4 — WakeForRefresh preserves valid across the wake edge ----
  // Pins the semantic distinction between the two poller wake seams introduced by M4:
  //   WakeForRestart publishes valid=false on the kPaused→kRunning edge (fresh commit: consumers
  //     must ignore stale terminal snapshots).
  //   WakeForRefresh preserves valid across the wake edge (display-time refresh: SyncFromPoller
  //     must not observe a transient valid=false window that would let ReconcileSimState pull a
  //     completed sim back into kSimulating — activity bug AC1 root cause (a),
  //     doc/gui-state-governance.md §4 支柱 2).
  // Same-shape white-box test: bring poller to kPaused with a valid=true snapshot published, then
  // exercise each wake variant and diff the immediately-following LoadSnapshot()->valid.
  ImGuiTest* t7 = IM_REGISTER_TEST(engine, "gui_lifecycle", "wake_for_refresh_preserves_valid");
  t7->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Clean baseline: detach the global poller from any prior test's server.
    gui::g_server_poller.Stop();
    gui::g_server = nullptr;

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool completed = RunFiniteToCompletion(server);
    IM_CHECK(completed);
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
      IM_CHECK(snap != nullptr);
      IM_CHECK(snap->valid);  // baseline invariant: seed snapshot is valid before the wake
      gui::g_server_poller.Stop();
    };

    // (A) WakeForRestart: valid must flip to false after the wake (PublishValidReset called).
    seed_baseline();
    gui::g_server_poller.WakeForRestart(server);
    {
      auto snap = gui::g_server_poller.LoadSnapshot();
      IM_CHECK(snap != nullptr);
      IM_CHECK(!snap->valid);  // WakeForRestart publishes valid=false on the wake edge
    }

    // (B) WakeForRefresh: valid must be preserved as true (no PublishValidReset). This is the
    // load-bearing behavior for AC1 — a display-time edit's wake path must not fabricate a
    // valid=false window that ReconcileSimState would classify as kSimulating.
    seed_baseline();
    gui::g_server_poller.WakeForRefresh(server);
    {
      auto snap = gui::g_server_poller.LoadSnapshot();
      IM_CHECK(snap != nullptr);
      IM_CHECK(snap->valid);  // WakeForRefresh preserves valid across the wake edge
    }

    gui::g_server_poller.Stop();
    LUMICE_DestroyServer(server);
  };

  // ---- Test 8: task-color-migration M7 — AC1/AC4 display-time edits do not disturb kDone ----
  // The user-visible activity bug (plan §1 偏离 A): after a finite run completes, toggling any of
  // color / visible / solo / z_order / mode used to flash Run→Stop and briefly display
  // "Simulating…" in the status bar. Two root causes had to be fixed together (doc §7 反模式
  // "同一 bug 连修错误层"):
  //   (a) M4 WakeForRefresh — display-time refresh must not publish valid=false through the
  //       poller's kPaused→kRunning wake edge (Test 7 pins the seam-level invariant);
  //   (b) M5 kRunCompleted latch — a completed intent is structurally immune to any late
  //       valid=false observation (Test 6b pins the reconcile-side latch);
  //   (c) M6 InvalidateEffectsBaselines — DoRun / DoRevert / backend-swap reset the display-push
  //       baseline so the next reconcile re-pushes (Test 2 in test_gui_state_reconcile pins the
  //       baseline invariant).
  //
  // This test is the END-TO-END integration guard for AC1/AC4: it drives a real finite DoRun→
  // completion, then exercises each of the five display-time edit categories by writing the
  // widget-level field directly (M3 made the widgets pure field-writers — the reconciler picks
  // up the diff on the frame tail — so a field poke is behaviorally identical to a `ctx->ItemClick`
  // on the corresponding widget, minus the ImGui click-plumbing complexity). Across a multi-frame
  // scan after each edit, sim_state must remain kDone (AC1), run_intent must remain kRunCompleted
  // and committed_epoch must NOT advance (AC4: display-time is inert to the sim-lifecycle clock).
  //
  // Multi-frame scan (not just one post-edit assert) is load-bearing: AC1 phrasing is "does not
  // flash" — a single-frame assertion would miss a one-frame regression window. The scan iterates
  // ctx->Yield() five times and asserts the invariant on every intermediate frame.
  ImGuiTest* t8 = IM_REGISTER_TEST(engine, "gui_lifecycle", "display_edits_do_not_disturb_done");
  t8->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Setup mirrors Test 6b (real-server finite DoRun through the reconcile pipeline).
    gui::g_server_poller.Stop();
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;
    gui::g_state = gui::InitDefaultState();
    gui::g_state.sim.infinite = false;
    gui::g_state.sim.ray_num_millions = 0.5f;
    gui::g_state.sim.max_hits = 8;
#if defined(__APPLE__)
    gui::g_state.use_gpu_backend = true;
#endif

    // Seed two color classes BEFORE DoRun so the committed config carries them and post-completion
    // display-time edits have a non-empty vector to push through the reconciler (empty raypath_color
    // short-circuits DiffAgainstDisplayBaseline; we want the actual push lane exercised).
    gui::ColorClassConfig c0;
    c0.color[0] = 1.0f;
    c0.visible = true;
    c0.solo = false;
    c0.z_order = 0;
    gui::ColorClassConfig c1;
    c1.color[1] = 1.0f;
    c1.visible = true;
    c1.solo = false;
    c1.z_order = 1;
    gui::g_state.raypath_color.push_back(c0);
    gui::g_state.raypath_color.push_back(c1);

    gui::DoRun();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));

    // Drive until sim_state == kDone AND intent has latched to kRunCompleted (Mealy edge in
    // SyncFromPoller runs one frame after the reconcile settles).
    auto start = std::chrono::steady_clock::now();
    while (gui::g_state.sim_state != SimState::kDone || gui::g_state.run_intent != RunIntent::kRunCompleted) {
      ctx->Yield();
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 20) {
        break;
      }
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));

    // AC4 baseline: display-time is inert to the sim-lifecycle clock. Capture the epoch here;
    // display-time edits must NOT advance it.
    const uint64_t baseline_epoch = gui::g_state.committed_epoch;

    // Multi-frame scan invariant: sim_state stays kDone, run_intent stays kRunCompleted,
    // committed_epoch does not advance, across N post-edit frames. RED手法: a single-frame
    // valid=false blip (pre-M4) would slip past a one-shot assertion but this loop catches it.
    auto scan_invariant = [&](const char* edit_label) {
      IM_UNUSED(edit_label);
      for (int frame = 0; frame < 5; ++frame) {
        ctx->Yield();
        IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
        IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));
        IM_CHECK_EQ(gui::g_state.committed_epoch, baseline_epoch);
      }
    };

    // Edit 1: color (ColorClassDisplayState.color[]) — the color swatch widget's field write.
    gui::g_state.raypath_color[0].color[0] = 0.5f;
    gui::g_state.raypath_color[0].color[1] = 0.25f;
    scan_invariant("color");

    // Edit 2: visible (eye icon plain click).
    gui::g_state.raypath_color[0].visible = false;
    scan_invariant("visible");

    // Edit 3: solo (eye icon Alt+click). Note: solo alone semantically implies visible=false on
    // the non-solo class; here we just flip solo on class 0 to exercise the display-state field.
    gui::g_state.raypath_color[0].solo = true;
    scan_invariant("solo");

    // Edit 4: z_order (up/down arrow buttons swap z_order values between two classes).
    std::swap(gui::g_state.raypath_color[0].z_order, gui::g_state.raypath_color[1].z_order);
    scan_invariant("z_order");

    // Edit 5: mode (composite mode combo).
    gui::g_state.raypath_color_mode = (gui::g_state.raypath_color_mode + 1) % 3;
    scan_invariant("mode");

    // Cleanup mirrors Test 2 / Test 6b.
    gui::g_server_poller.Stop();
    if (gui::g_server) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
  };

  // AC1 real-UI-click regression (code-review round-1 Major-3): Test 8 above proves the
  // reconciler-level AC1/AC4 invariants given a raw field write, but M3 made every color-window
  // widget a pure field-writer that is ALSO reached through real ImGui widget code (button click,
  // combo popup, held-Alt modifier, color-picker popup) before that field write happens — the
  // multi-frame widget interaction itself (popup open/close, a frame where the mouse is down but
  // the click hasn't registered yet, modifier-key state) is exactly the kind of thing a direct
  // field poke cannot exercise. This test drives the same five display-time categories through
  // their actual widget code paths (`ctx->ItemClick`/`ctx->ComboClick`, held Alt for solo) instead
  // of assigning GuiState fields directly.
  ImGuiTest* t8b = IM_REGISTER_TEST(engine, "gui_lifecycle", "display_edits_via_real_ui_clicks_do_not_disturb_done");
  t8b->TestFunc = [](ImGuiTestContext* ctx) {
    gui::g_server_poller.Stop();
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;
    gui::g_state = gui::InitDefaultState();
    gui::g_state.sim.infinite = false;
    gui::g_state.sim.ray_num_millions = 0.5f;
    gui::g_state.sim.max_hits = 8;
#if defined(__APPLE__)
    gui::g_state.use_gpu_backend = true;
#endif

    // Two match-all classes on the default crystal, seeded BEFORE DoRun (same rationale as Test
    // 8: a non-empty committed raypath_color so the push lane is actually exercised). z_order is
    // seeded in ascending rank order (rank 0 == phys 0) so the wildcard item lookups below —
    // which resolve to the FIRST matching item in this frame's submission order (see
    // ImGuiTestEngineHook_ItemInfo_ResolveFindByLabel: `OutItemId == 0` guard means first-match-
    // wins, not an ambiguity error) — deterministically hit the rank-0 row without needing a
    // fragile PushID(int)-based hardcoded path (the same fragility `toggle_whole_via_ui_marks_
    // modified` in test_gui_color_window.cpp already called out for `##body`).
    gui::ColorClassConfig c0;
    c0.color[0] = 1.0f;
    c0.visible = true;
    c0.solo = false;
    c0.z_order = 0;
    gui::ColorClassRefConfig ref0;
    ref0.layer_idx = 0;
    ref0.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
    ref0.match_all = true;
    c0.match.push_back(ref0);
    gui::ColorClassConfig c1;
    c1.color[1] = 1.0f;
    c1.visible = true;
    c1.solo = false;
    c1.z_order = 1;
    c1.match.push_back(ref0);
    gui::g_state.raypath_color.push_back(c0);
    gui::g_state.raypath_color.push_back(c1);

    gui::DoRun();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunning));
    auto start = std::chrono::steady_clock::now();
    while (gui::g_state.sim_state != SimState::kDone || gui::g_state.run_intent != RunIntent::kRunCompleted) {
      ctx->Yield();
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 20) {
        break;
      }
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));

    const uint64_t baseline_epoch = gui::g_state.committed_epoch;

    // Same multi-frame scan rationale as Test 8's scan_invariant: AC1 is "does not flash", so a
    // single post-click assertion would miss a one-frame regression window.
    auto scan_invariant = [&]() {
      for (int frame = 0; frame < 5; ++frame) {
        ctx->Yield();
        IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
        IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(RunIntent::kRunCompleted));
        IM_CHECK_EQ(gui::g_state.committed_epoch, baseline_epoch);
      }
    };

    gui::g_state.color_window_open = true;
    ctx->Yield(2);
    ctx->SetRef("//" ICON_FA_PALETTE " Colors");

    // Click 1: color swatch -> opens the ColorPicker popup -> click inside its "sv" square. The
    // default ItemClick position is the item rect's center, which is never the current pure-red
    // S=1,V=1 corner, so this always produces a value_changed edit (see ColorPicker4's `IsItemActive
    // () && !is_readonly` branch in imgui_widgets.cpp) -> close the popup.
    ctx->ItemClick("**/##color");
    ctx->Yield();
    ctx->ItemClick("**/sv");
    ctx->PopupCloseAll();
    scan_invariant();
    IM_CHECK_NE(gui::g_state.raypath_color[0].color[0], 1.0f);  // sanity: the click actually landed

    // Click 2: eye icon plain click -> visible toggle.
    IM_CHECK(gui::g_state.raypath_color[0].visible);
    ctx->ItemClick("**/" ICON_FA_EYE);
    scan_invariant();
    IM_CHECK(!gui::g_state.raypath_color[0].visible);  // sanity: the click actually landed

    // Click 3: Alt+eye icon -> solo. Rank-0's icon is now ICON_FA_EYE_SLASH (visible was just
    // toggled off above) while rank-1's is still plain ICON_FA_EYE, so this lookup is unambiguous
    // by construction, independent of the first-match tie-break described above.
    ctx->KeyDown(ImGuiMod_Alt);
    ctx->ItemClick("**/" ICON_FA_EYE_SLASH);
    ctx->KeyUp(ImGuiMod_Alt);
    scan_invariant();
    IM_CHECK(gui::g_state.raypath_color[0].solo);  // sanity: the click actually landed

    // Click 4: z_order down-arrow on the rank-0 row. Rank-0's up-arrow is disabled (top of stack);
    // its down-arrow is the enabled one, and first-match resolves to rank-0 since it is rendered
    // first in the z_order-sorted loop.
    const int z0_before = gui::g_state.raypath_color[0].z_order;
    const int z1_before = gui::g_state.raypath_color[1].z_order;
    ctx->ItemClick("**/" ICON_FA_ARROW_DOWN "##down");
    scan_invariant();
    IM_CHECK_EQ(gui::g_state.raypath_color[0].z_order, z1_before);  // sanity: the swap landed
    IM_CHECK_EQ(gui::g_state.raypath_color[1].z_order, z0_before);

    // Click 5: composite mode combo -> "painter" is combo item index 2 (see kModeNames in
    // RenderCompositeModeCombo), distinct from the default index 0 ("dominant").
    const int mode_before = gui::g_state.raypath_color_mode;
    ctx->ComboClick("##ColorMode/painter");
    scan_invariant();
    IM_CHECK_NE(gui::g_state.raypath_color_mode, mode_before);  // sanity: the click actually landed
    IM_CHECK_EQ(gui::g_state.raypath_color_mode, 2);

    ctx->SetRef("");
    gui::g_state.color_window_open = false;
    ctx->Yield(2);

    gui::g_server_poller.Stop();
    if (gui::g_server) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    gui::g_state.run_intent = RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
  };
}
