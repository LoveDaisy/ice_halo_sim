// Responsiveness: the GUI stays alive while the simulator is working.
//
// What these two cases are collaborating to falsify: that the app's DISPLAY clock can be
// starved by its SIMULATION clock. A user watching an infinite run sees a frozen window and a
// preview that never updates; a user dragging a slider sees the picture stop following the
// control. Both are the same failure at two duty cycles — one with nothing else happening,
// one with a parameter changing every frame — and neither can be falsified without real
// wall-clock time, which is why they live here and not in composition-correctness.
//
// These cases are in the REAL-TIMING pool (scripts/build.sh runs the correctness pool under
// --fixed-dt, which injects a synthetic 1/60s frame dt and would starve a wall-clock deadline).
// The pool split keys on the "perf_test" CATEGORY string, which appears in build.sh's two
// --filter arguments and is parsed back out of them by scripts/test.sh — renaming the category
// silently moves these cases into the pool that cannot run them.
//
// The budgets below are floors, not targets. They are set an order of magnitude under what any
// machine that can run this suite at all produces, because their job is to catch "the loop
// stopped", not to police throughput — throughput belongs to doc/performance-testing.md and its
// own harness, which reads the [PERF] line each case still prints.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>

#include "gui/file_io.hpp"
#include "gui/gui_logger.hpp"
#include "support/scoped_result_frame.hpp"
#include "test_gui_shared.hpp"

namespace {

// Floors, deliberately far below any real machine (see the header note). A machine that misses
// these is not slow, it is stuck.
constexpr int kMinFramesPerSecond = 5;
constexpr int kSteadyStateWindowSec = 2;
constexpr int kSliderDragWindowSec = 5;
constexpr int kFirstDataTimeoutSec = 10;

// Sun altitude sweep range, chosen to match a typical manual drag.
constexpr float kAltMin = 10.0f;
constexpr float kAltMax = 30.0f;

// Pump frames until the first batch of simulation data lands. Returns false on timeout, which
// the caller reports as a failure rather than silently measuring an idle server.
bool WaitForFirstBatch(ImGuiTestContext* ctx) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(kFirstDataTimeoutSec);
  while (gui::g_state.stats_sim_ray_num == 0) {
    ctx->Yield();
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
  }
  return true;
}

unsigned long ReadServerRays() {
  if (!gui::g_server) {
    return 0;
  }
  LUMICE_StatsResult stats{};
  lumice::test::ScopedResultFrame frame(gui::g_server);
  LUMICE_FrameGetStats(frame.get(), &stats);
  return stats.sim_ray_num;
}

}  // namespace

// Harness shared with test_gui_main.cpp's declaration. Both perf scenarios run against the same
// scene so their numbers are comparable across runs.
//
// The three LUMICE_PERF_* reads are test-infrastructure knobs (doc/env-var-policy.md's B class):
// they select a scene / a backend / a log route for a benchmark operator, and none of them
// changes what the product does for a user.
void StartPerfSimulation() {
  // LUMICE_PERF_CORELOG=1 routes Core logs to stderr so the actual backend-route decision
  // (simulator.cpp's "routing via MetalTraceBackend" vs its "falling back to legacy CPU"
  // warning) is observable — the perf harness otherwise drops Core logs.
  if (const char* c = std::getenv("LUMICE_PERF_CORELOG"); c && std::string(c) == "1") {
    LUMICE_SetLogCallback([](LUMICE_LogLevel level, const char* name, const char* message) {
      fprintf(stderr, "[CORE:%d %s] %s\n", static_cast<int>(level), name ? name : "?", message);
    });
  }

  gui::g_server = LUMICE_CreateServer();
  LUMICE_SetLogLevel(gui::g_server, static_cast<LUMICE_LogLevel>(g_core_log_level));
  gui::SetGuiLogLevel(static_cast<spdlog::level::level_enum>(g_gui_log_level));

#if defined(__APPLE__)
  // Fresh CPU server (LUMICE_CreateServer's default) — re-establish the toggle-detection
  // invariant before any DoRun. Perf scenarios run sequentially in one binary, so a prior Metal
  // reconstruction may have left g_server_is_gpu true.
  gui::g_server_is_gpu = false;

  // LUMICE_PERF_METAL=1 opts into Metal through the REAL product path: the flag is applied just
  // before each DoRun, never here, because the LUMICE_PERF_CONFIG branch below deserializes into
  // `state = GuiState{}` and would wipe an early assignment. DoRun then runs
  // MaybeReconstructServerForBackend and rebuilds into the single-engine Metal topology —
  // exactly what the GUI checkbox does. (A retired route flipped a backend flag on this CPU
  // N-worker server instead, which ran Metal kernels under an orchestration topology built for
  // many CPU workers and measured 0.58x of the CPU baseline — slower than the thing it replaced.
  // Reconstructing the server is what makes the measurement mean anything.) Pair with a
  // Metal-compatible LUMICE_PERF_CONFIG
  // (dual_fisheye / rectangular lens); the built-in scene below uses fisheye equal-area.
  bool want_metal = false;
  if (const char* m = std::getenv("LUMICE_PERF_METAL"); m && std::string(m) == "1") {
    want_metal = true;
    fprintf(stderr, "[PERF] LUMICE_PERF_METAL=1 -> single-engine Metal (server reconstruct on DoRun)\n");
  }
#endif

  // LUMICE_PERF_CONFIG points these scenarios at an arbitrary config JSON (e.g. a heavy
  // multi-MS / multi-crystal scene) through the same DeserializeFromJson path the lens_proj and
  // sim_e2e_smoke suites use, at faithful resolution. Kept for reuse by future GUI perf work —
  // this is a benchmark capability, not throwaway instrumentation.
  if (const char* cfg_path = std::getenv("LUMICE_PERF_CONFIG")) {
    std::ifstream in(cfg_path);
    if (in.is_open()) {
      std::string json_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      if (gui::DeserializeFromJson(json_str, gui::g_state)) {
        fprintf(stderr, "[PERF] loaded LUMICE_PERF_CONFIG=%s\n", cfg_path);
#if defined(__APPLE__)
        gui::g_state.use_gpu_backend = want_metal;  // AFTER deserialize — it resets g_state
#endif
        gui::DoRun(/*user_initiated=*/true);
        return;
      }
    }
    fprintf(stderr, "[PERF] ERROR: failed to load LUMICE_PERF_CONFIG=%s; falling back to default\n", cfg_path);
  }

  // Built-in scene: one prism, sun at 20 degrees, infinite rays, core resolution 1024x512.
  // Driven through DoRun so the server's config manager is populated by the same BuildScene path
  // the product uses.
  gui::g_state.sun.altitude = 20.0f;
  gui::g_state.sun.diameter = 0.5f;
  gui::g_state.sun.spectrum_index = 2;  // D65
  gui::g_state.sim.infinite = true;
  gui::g_state.sim.max_hits = 8;
  {
    auto& r = gui::g_state.renderer;
    r.lens_type = 1;  // Fisheye Equal Area (matches typical manual testing)
    r.fov = 360.0f;
    r.sim_resolution_index = 0;  // 512 -> core resolution [1024, 512]
    r.visible = 2;               // Full
    r.background[0] = r.background[1] = r.background[2] = 0.0f;
    r.exposure_offset = 0.0f;
  }
#if defined(__APPLE__)
  gui::g_state.use_gpu_backend = want_metal;
#endif
  gui::DoRun(/*user_initiated=*/true);
}

void StopPerfSimulation() {
  gui::g_server_poller.Stop();
  if (gui::g_server) {
    LUMICE_StopServer(gui::g_server);
    LUMICE_DestroyServer(gui::g_server);
    gui::g_server = nullptr;
  }
  // Reset the reconcile intent too — a leftover kRunning intent would flip sim_state back to
  // kSimulating on the next frame (sim_state is reconcile-derived, not a raw write).
  gui::g_state.run_intent = gui::RunIntent::kNone;
}

void RegisterPerfTests(ImGuiTestEngine* engine) {
  // An infinite run does not freeze the window, and the preview keeps arriving.
  //
  // Three independent things can each produce the same user-visible symptom, so all three are
  // asserted separately rather than folded into one throughput number: the frame loop can stop,
  // the simulator can stop producing rays, and the texture upload path can stop consuming them.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "perf_test", "steady_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();
      IM_CHECK(WaitForFirstBatch(ctx));

      const unsigned long start_rays = gui::g_state.stats_sim_ray_num;
      const auto uploads_before = gui::g_state.texture_upload_count;
      const auto start_time = std::chrono::steady_clock::now();
      const auto end_time = start_time + std::chrono::seconds(kSteadyStateWindowSec);

      int frame_count = 0;
      while (std::chrono::steady_clock::now() < end_time) {
        ctx->Yield();
        frame_count++;
      }

      const unsigned long end_rays = gui::g_state.stats_sim_ray_num;
      const auto uploads = gui::g_state.texture_upload_count - uploads_before;
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      const double rays_per_sec = elapsed > 0 ? static_cast<double>(end_rays - start_rays) / elapsed : 0.0;

      // Stop BEFORE asserting: a failing IM_CHECK aborts the test function, and the server would
      // then outlive it — its threads run during static destruction and crash on destroyed static
      // locals (e.g. the lens projection map in GetProjFunc).
      StopPerfSimulation();

      fprintf(stderr, "[PERF] steady_state: %.1f rays/sec, %d frames, %lu uploads in %.1fs\n", rays_per_sec,
              frame_count, static_cast<unsigned long>(uploads), elapsed);

      IM_CHECK_GT(rays_per_sec, 0.0);
      IM_CHECK_GE(frame_count, kMinFramesPerSecond * kSteadyStateWindowSec);
      IM_CHECK_GT(uploads, 0u);
    };
  }

  // A parameter that changes every frame still gets committed, and the picture still follows.
  //
  // There is no hot-update path: every parameter change restarts the run from zero, so a
  // continuously-dirty state is the worst case for the commit throttle. If backpressure from the
  // in-flight run wins every round, the user drags a slider and the preview never moves again.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "perf_test", "slider_drag");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();
      IM_CHECK(WaitForFirstBatch(ctx));

      g_main_loop_cumulative_rays = 0;
      g_main_loop_restart_count = 0;

      const auto start_time = std::chrono::steady_clock::now();
      const auto end_time = start_time + std::chrono::seconds(kSliderDragWindowSec);
      auto last_commit = start_time;
      const auto uploads_before = gui::g_state.texture_upload_count;
      auto last_upload_count = uploads_before;

      unsigned long cumulative_rays = 0;
      int frame_count = 0;
      int restart_count = 0;
      int uploads_after_a_commit = 0;
      bool awaiting_upload = false;
      // Mirror of the main loop's own restart counter, so the "a commit just happened" edge below
      // is observable in BOTH modes. Under --main-loop-commit the commit happens on the main
      // thread and this test thread never runs the DoRun branch, so watching this counter is the
      // only way to see the edge — arming awaiting_upload only inside that branch would leave
      // uploads_after_a_commit pinned at zero and fail the final assertion for a reason that has
      // nothing to do with the app.
      int last_seen_main_loop_restarts = g_main_loop_restart_count;

      while (std::chrono::steady_clock::now() < end_time) {
        // Continuous triangle wave over a 2s period, matching a smooth manual drag.
        const double elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        const double phase = std::fmod(elapsed_sec, 2.0);
        const float t = (phase < 1.0) ? static_cast<float>(phase) : static_cast<float>(2.0 - phase);
        gui::g_state.sun.altitude = kAltMin + t * (kAltMax - kAltMin);
        gui::g_state.dirty = true;
        frame_count++;

        // One yield per iteration = one frame, matching the real app's per-frame commit check.
        ctx->Yield();

        if (g_enable_main_loop_commit && g_main_loop_restart_count != last_seen_main_loop_restarts) {
          last_seen_main_loop_restarts = g_main_loop_restart_count;
          awaiting_upload = true;
        }

        if (gui::g_state.texture_upload_count != last_upload_count) {
          last_upload_count = gui::g_state.texture_upload_count;
          if (awaiting_upload) {
            uploads_after_a_commit++;
            awaiting_upload = false;
          }
        }

        // Under --main-loop-commit the main loop owns DoRun on the main thread (matching the real
        // app); otherwise it fires here on the test thread.
        // WARNING: MIRROR of the throttle/accounting block in src/gui/main.cpp (dirty-clear
        // timing, restart counting, DoRun return-value handling). If that block changes, mirror
        // the change here — the two are the same policy expressed twice on purpose, because this
        // case must exercise the throttle even when the main loop is not driving it.
        if (!g_enable_main_loop_commit) {
          const auto now = std::chrono::steady_clock::now();
          const auto since_commit = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (since_commit >= gui::kCommitIntervalMs) {
            // Read server rays BEFORE DoRun so a successful commit captures the just-ending run's
            // count. A refused commit means the previous epoch's run is still going and its
            // counter rolls into the next successful commit — no double count. `last_commit`
            // advances either way, so the cadence check is unchanged.
            const auto rays_this_cycle = ReadServerRays();
            if (gui::DoRun(/*user_initiated=*/false)) {
              cumulative_rays += rays_this_cycle;
              gui::g_state.dirty = false;
              awaiting_upload = true;
              restart_count++;
            }
            if (g_dorun_delay_ms > 0) {
              std::this_thread::sleep_for(std::chrono::milliseconds(g_dorun_delay_ms));
            }
            last_commit = now;
          }
        }
      }

      if (g_enable_main_loop_commit) {
        cumulative_rays = g_main_loop_cumulative_rays + ReadServerRays();
        restart_count = g_main_loop_restart_count;
      } else {
        cumulative_rays += ReadServerRays();
      }

      const auto uploads = gui::g_state.texture_upload_count - uploads_before;
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      const double rays_per_sec = elapsed > 0 ? static_cast<double>(cumulative_rays) / elapsed : 0.0;

      StopPerfSimulation();  // before the assertions — see steady_state

      fprintf(stderr,
              "[PERF] slider_drag: %.1f rays/sec, %d frames, %d restarts, %lu uploads (%d after a commit) in %.1fs\n",
              rays_per_sec, frame_count, restart_count, static_cast<unsigned long>(uploads), uploads_after_a_commit,
              elapsed);

      IM_CHECK_GT(rays_per_sec, 0.0);
      IM_CHECK_GE(frame_count, kMinFramesPerSecond * kSliderDragWindowSec);
      IM_CHECK_GT(restart_count, 0);
      IM_CHECK_GT(uploads_after_a_commit, 0);
    };
  }
}
