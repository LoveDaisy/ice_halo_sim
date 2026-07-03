// Regression guard for the "GPU finite run finished (backend IDLE) but GUI stuck in
// kSimulating" dead-state bug — see doc/gui-preview-lifecycle-architecture.md §2/§6 and
// scrum-gui-lifecycle-clock-decouple. Pins blueprint invariants I3 (level-triggered
// self-heal — the terminal completion edge is never lost) and I4 (lifecycle readable
// without an expensive snapshot generation).
//
// Two tests:
//  1. gui_lifecycle/terminal_idle_reaches_done — DETERMINISTIC white-box. Reproduces the
//     exact poll/sync interleaving from the issue (a mid-run TrySyncData zeroes the staged
//     stats, then the terminal IDLE poll carries NO new snapshot generation) and asserts
//     the completion edge still reaches kDone. Also pins the restart/stop-transient IDLE
//     boundary (has_valid_data==false must NOT be classified done).
//  2. gui_lifecycle/gpu_run_reaches_done — INTEGRATION e2e. Drives the real DoRun→completion
//     flow (GPU/Metal on Apple, CPU elsewhere) and asserts sim_state reaches kDone. This is
//     the end-to-end verification that the dead-state is gone through the real Run path.

#include <chrono>
#include <thread>

#include "gui/server_poller.hpp"
#include "test_gui_shared.hpp"

namespace {

using SimState = gui::GuiState::SimState;

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
// (has_valid_data level true). Returns false on timeout / commit failure.
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

}  // namespace

void RegisterLifecycleTests(ImGuiTestEngine* engine) {
  // ---- Test 1: deterministic white-box interleaving regression ----
  ImGuiTest* t = IM_REGISTER_TEST(engine, "gui_lifecycle", "terminal_idle_reaches_done");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    // Clean baseline: detach the global poller from any prior test's server.
    gui::g_server_poller.Stop();
    gui::g_server = nullptr;
    gui::g_state.sim_state = SimState::kIdle;

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    bool completed = RunFiniteToCompletion(server);
    IM_CHECK(completed);  // finite run reached IDLE + has_valid_data
    if (!completed) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Orthogonal add-only pin (backend-lifecycle-epoch 1.3): the same terminal
    // completion the has_valid_data / kDone assertions below guard is now also
    // readable as the explicit COMPLETED lifecycle with a minted epoch. This does
    // NOT replace the level-signal regression baseline (those guard 1.5's later
    // side-signal removal).
    {
      LUMICE_SimLifecycleResult lc{};
      IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
      IM_CHECK_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
      IM_CHECK(lc.epoch >= 1);
    }

    // (A) POLLER HALF (server_poller.cpp / I4): a terminal IDLE poll that carries NO new
    // snapshot generation must still stage has_valid_data==true, even though its staged
    // stats are 0. This is the level signal that the fix decoupled from snapshot
    // materialization. Reverting the unconditional-stage line makes b.has_valid_data false.
    {
      gui::ServerPoller local;
      local.ResetGenerationForTest();
      local.PollOnceForTest(server);  // Poll A: generation G is new -> stages stats + has_valid_data
      gui::PollerData a;
      IM_CHECK(local.TrySyncData(a));  // consume Poll A; resets staged (this zeroes staged stats)
      IM_CHECK(a.valid);
      IM_CHECK(a.has_valid_data);
      IM_CHECK(a.stats_sim_ray_num > 0);  // Poll A genuinely carried the generation-bearing frame

      local.PollOnceForTest(server);  // Poll B: same generation G -> has_new_snapshot == false
      gui::PollerData b;
      IM_CHECK(local.TrySyncData(b));
      IM_CHECK(b.valid);
      IM_CHECK_EQ(static_cast<int>(b.server_state), static_cast<int>(LUMICE_SERVER_IDLE));
      // The fragile OLD guard (stats_sim_ray_num>0) would see 0 here and never fire:
      IM_CHECK_EQ(b.stats_sim_ray_num, static_cast<LUMICE_RayCount>(0));
      // The reliable level signal survives the no-new-generation poll (the fix):
      IM_CHECK(b.has_valid_data);
    }

    // (B) APP HALF (app.cpp SyncFromPoller / I3): the real completion decision, exercised
    // through the actual global poller + SyncFromPoller with the same issue interleaving.
    // A mid TrySyncData zeroes staged stats; the terminal IDLE poll carries no new
    // generation; SyncFromPoller must still reach kDone. Reverting the guard to
    // `stats_sim_ray_num>0` leaves sim_state stuck at kSimulating here (the original bug).
    {
      gui::g_server_poller.ResetGenerationForTest();
      gui::g_server_poller.PollOnceForTest(server);  // Poll A (generation-bearing)
      gui::PollerData tmp;
      IM_CHECK(gui::g_server_poller.TrySyncData(tmp));  // mid sync: swap resets staged stats to 0
      gui::g_server_poller.PollOnceForTest(server);     // Poll B: terminal IDLE, no new generation

      gui::g_state.sim_state = SimState::kSimulating;
      gui::SyncFromPoller();  // consumes the terminal frame
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kDone));
    }

    // (C) BOUNDARY (I3): a restart/stop-transient IDLE has has_valid_data==false (the server
    // resets has_ever_consumed_ on Stop) and must NOT be misclassified as done.
    {
      LUMICE_StopServer(server);  // IDLE + has_valid_data reset to false

      gui::ServerPoller local;
      local.PollOnceForTest(server);
      gui::PollerData f;
      IM_CHECK(local.TrySyncData(f));
      IM_CHECK(f.valid);
      IM_CHECK_EQ(static_cast<int>(f.server_state), static_cast<int>(LUMICE_SERVER_IDLE));
      IM_CHECK(!f.has_valid_data);  // no data produced since Stop

      // Same no-new-generation interleaving as section (B): drain the first poll, then feed
      // SyncFromPoller a terminal frame with no new texture (so it takes no GL upload path —
      // SyncFromPoller runs on the real main thread with a GL context; this coroutine has none).
      gui::g_server_poller.ResetGenerationForTest();
      gui::g_server_poller.PollOnceForTest(server);
      gui::PollerData drain;
      IM_CHECK(gui::g_server_poller.TrySyncData(drain));
      gui::g_server_poller.PollOnceForTest(server);  // terminal IDLE, no new generation
      gui::g_state.sim_state = SimState::kSimulating;
      gui::SyncFromPoller();
      // Must stay kSimulating — a transient IDLE without valid data is not completion.
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));
    }

    // Cleanup: leave a clean global state for subsequent tests.
    gui::g_server_poller.Stop();
    gui::g_server = nullptr;
    gui::g_state.sim_state = SimState::kIdle;
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

    gui::DoRun();  // real product Run path: commit + start poller (worker thread polls for real)
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(SimState::kSimulating));

    // Drive the real display clock: each Yield runs a main-loop frame, which calls
    // SyncFromPoller() (test_gui_main.cpp). The finite run finishes and the GUI must reconcile
    // to kDone within the timeout — the behavior that used to hang.
    auto start = std::chrono::steady_clock::now();
    while (gui::g_state.sim_state == SimState::kSimulating) {
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
    gui::g_state.sim_state = SimState::kIdle;
  };
}
