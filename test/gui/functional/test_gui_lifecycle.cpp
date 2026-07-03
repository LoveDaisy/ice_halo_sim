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
}
