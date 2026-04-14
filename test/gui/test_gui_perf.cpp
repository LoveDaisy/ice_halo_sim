#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

#include "gui/gui_logger.hpp"
#include "test_gui_shared.hpp"

// ========== Performance Tests ==========

static const char* CreatePerfConfig() {
  // Minimal config: single prism crystal, sun at 20°, infinite rays, 1024x512 resolution
  return R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "ratio": {"upper": 1.0, "lower": 1.0}}],
    "filter": [],
    "scene": {
      "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0, "diameter": 0.5, "spectrum": "D65"},
      "ray_num": "infinite",
      "max_hits": 8,
      "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{"id": 1, "lens": {"type": "rectangular", "fov": 180.0},
                "resolution": [1024, 512], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
                "visible": "full", "background": [0, 0, 0], "opacity": 1.0, "intensity_factor": 1.0}]
  })";
}

void StartPerfSimulation() {
  gui::g_server = LUMICE_CreateServer();
  LUMICE_SetLogLevel(gui::g_server, static_cast<LUMICE_LogLevel>(g_core_log_level));
  gui::SetGuiLogLevel(static_cast<spdlog::level::level_enum>(g_gui_log_level));

  // Set up g_state to match perf config, then use DoRun() so the server's
  // config_manager_ is populated from the same SerializeCoreConfig path.
  gui::g_state.sun.altitude = 20.0f;
  gui::g_state.sun.diameter = 0.5f;
  gui::g_state.sun.spectrum_index = 2;  // D65
  gui::g_state.sim.infinite = true;
  gui::g_state.sim.max_hits = 8;
  {
    auto& r = gui::g_state.renderer;
    r.lens_type = 1;  // Fisheye Equal Area (matches typical manual testing)
    r.fov = 360.0f;
    r.sim_resolution_index = 0;  // 512 → Core resolution [1024, 512], matching CreatePerfConfig
    r.visible = 2;               // Full
    r.background[0] = r.background[1] = r.background[2] = 0.0f;
    r.exposure_offset = 0.0f;
  }
  gui::DoRun();
}

void StopPerfSimulation() {
  gui::g_server_poller.Stop();
  if (gui::g_server) {
    LUMICE_StopServer(gui::g_server);
    LUMICE_DestroyServer(gui::g_server);
    gui::g_server = nullptr;
  }
  gui::g_state.sim_state = gui::GuiState::SimState::kIdle;
}

static void ReportPerf(const char* label, unsigned long start_rays, unsigned long end_rays, double elapsed_sec) {
  unsigned long delta = end_rays - start_rays;
  double rays_per_sec = elapsed_sec > 0 ? static_cast<double>(delta) / elapsed_sec : 0;
  fprintf(stderr, "[PERF] %s: %.1f rays/sec (%lu rays in %.1fs)\n", label, rays_per_sec, delta, elapsed_sec);
}


void RegisterPerfTests(ImGuiTestEngine* engine) {
  // Scenario 1: Steady-state simulation (baseline)
  // Measures: rays/sec, texture update interval (avg/min/max), texture FPS
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "perf_test", "steady_state");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();

      // Wait for first batch of data
      auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > timeout) {
          fprintf(stderr, "[PERF] ERROR: No simulation data after 10s\n");
          break;
        }
      }

      // Measure for 2 seconds (short enough for test engine timeout)
      unsigned long start_rays = gui::g_state.stats_sim_ray_num;
      auto upload_before = gui::g_state.texture_upload_count;
      auto start_time = std::chrono::steady_clock::now();
      auto end_time = start_time + std::chrono::seconds(2);

      // Track texture update intervals
      auto last_upload_count = upload_before;
      auto last_upload_time = start_time;
      std::vector<double> texture_intervals_ms;

      int frame_count = 0;
      while (std::chrono::steady_clock::now() < end_time) {
        ctx->Yield();
        frame_count++;
        // Detect new texture upload
        if (gui::g_state.texture_upload_count != last_upload_count) {
          auto now = std::chrono::steady_clock::now();
          double interval = std::chrono::duration<double, std::milli>(now - last_upload_time).count();
          texture_intervals_ms.push_back(interval);
          last_upload_count = gui::g_state.texture_upload_count;
          last_upload_time = now;
        }
      }

      unsigned long end_rays = gui::g_state.stats_sim_ray_num;
      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

      fprintf(stderr, "[PERF] steady_state: %d frames in %.1fs (%.1f main_loop_FPS)\n", frame_count, elapsed,
              frame_count / elapsed);
      ReportPerf("steady_state", start_rays, end_rays, elapsed);
      double rays_per_sec = elapsed > 0 ? static_cast<double>(end_rays - start_rays) / elapsed : 0;
      IM_CHECK_GT(rays_per_sec, 0.0);

      // Report texture update frequency
      auto texture_uploads = gui::g_state.texture_upload_count - upload_before;
      if (!texture_intervals_ms.empty()) {
        std::sort(texture_intervals_ms.begin(), texture_intervals_ms.end());
        double sum = 0;
        for (auto v : texture_intervals_ms) {
          sum += v;
        }
        double avg = sum / texture_intervals_ms.size();
        double median = texture_intervals_ms[texture_intervals_ms.size() / 2];
        double min_val = texture_intervals_ms.front();
        double max_val = texture_intervals_ms.back();
        fprintf(stderr, "[PERF] steady_state: %lu texture updates in %.1fs (%.1f FPS)\n", texture_uploads, elapsed,
                texture_uploads / elapsed);
        fprintf(stderr, "[PERF] steady_state: texture interval avg=%.0fms median=%.0fms min=%.0fms max=%.0fms\n", avg,
                median, min_val, max_val);
      }

      StopPerfSimulation();
    };
  }

  // Scenario 2: Parameter drag (slider interaction during simulation)
  // All parameter changes trigger full restart (hot-update was removed in task-52.1).
  // Measures: rays/sec, restarts, upload ratio, per-restart ray count distribution
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "perf_test", "slider_drag");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      StartPerfSimulation();

      // Wait for first batch of data
      auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (gui::g_state.stats_sim_ray_num == 0) {
        ctx->Yield();
        if (std::chrono::steady_clock::now() > timeout) {
          fprintf(stderr, "[PERF] ERROR: No simulation data after 10s\n");
          break;
        }
      }

      // Reset main-loop-commit counters for this measurement window
      g_main_loop_cumulative_rays = 0;
      g_main_loop_restart_count = 0;

      // Measure: sweep sun altitude over 5 seconds, matching real manual slider drag.
      // Each frame: update altitude (continuous sweep) + set dirty.
      // When --main-loop-commit: DoRun fires on main thread (same as real app).
      // Otherwise: DoRun fires here on test thread (original behavior).
      auto start_time = std::chrono::steady_clock::now();
      auto end_time = start_time + std::chrono::seconds(5);
      auto last_commit = start_time;
      unsigned long cumulative_rays = 0;
      int iteration = 0;
      int restart_count = 0;
      auto upload_before = gui::g_state.texture_upload_count;
      std::vector<unsigned long> per_restart_rays;  // Ray count per restart cycle
      std::vector<unsigned long> per_upload_rays;   // Ray count at each texture upload (measures flicker)
      std::vector<double> first_upload_ms;          // Commit → first upload delay per restart (responsiveness)
      auto last_upload_count = upload_before;       // Independent tracker for upload detection
      auto last_dorun_time = start_time;            // Timestamp of most recent DoRun
      bool waiting_first_upload = false;            // True between DoRun and first upload

      auto read_server_rays = [&]() -> unsigned long {
        if (!gui::g_server) {
          return 0;
        }
        LUMICE_StatsResult stats[2]{};
        LUMICE_GetStatsResults(gui::g_server, stats, 1);
        return stats[0].sim_ray_num;
      };

      // Sweep sun altitude between 10° and 30° (typical manual drag range)
      constexpr float kAltMin = 10.0f;
      constexpr float kAltMax = 30.0f;

      while (std::chrono::steady_clock::now() < end_time) {
        // Continuous triangle wave: sweep up then down, matching smooth slider drag
        double elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        double phase = std::fmod(elapsed_sec, 2.0);  // 2s period (1s up + 1s down)
        float t = (phase < 1.0) ? static_cast<float>(phase) : static_cast<float>(2.0 - phase);
        gui::g_state.sun.altitude = kAltMin + t * (kAltMax - kAltMin);
        gui::g_state.dirty = true;
        iteration++;

        // One yield per iteration = one frame, matching real app's per-frame commit check
        ctx->Yield();

        // Track per-upload ray counts
        if (gui::g_state.texture_upload_count != last_upload_count) {
          per_upload_rays.push_back(gui::g_state.stats_sim_ray_num);
          if (waiting_first_upload) {
            auto delay = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_dorun_time);
            first_upload_ms.push_back(delay.count());
            waiting_first_upload = false;
          }
          last_upload_count = gui::g_state.texture_upload_count;
        }

        // Throttled commit: when --main-loop-commit, the main loop handles DoRun on the main thread
        // (matching real app behavior). Otherwise, DoRun fires here on the test thread.
        if (!g_enable_main_loop_commit) {
          auto now = std::chrono::steady_clock::now();
          auto commit_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_commit).count();
          if (commit_elapsed >= gui::kCommitIntervalMs) {
            auto rays_this_cycle = read_server_rays();
            per_restart_rays.push_back(rays_this_cycle);
            cumulative_rays += rays_this_cycle;
            gui::g_state.dirty = false;
            gui::DoRun();
            if (g_dorun_delay_ms > 0) {
              std::this_thread::sleep_for(std::chrono::milliseconds(g_dorun_delay_ms));
            }
            last_commit = now;
            last_dorun_time = now;
            waiting_first_upload = true;
            restart_count++;
          }
        }
      }

      if (g_enable_main_loop_commit) {
        // Main loop tracked rays and restarts; read final cycle and combine
        auto final_rays = read_server_rays();
        cumulative_rays = g_main_loop_cumulative_rays + final_rays;
        restart_count = g_main_loop_restart_count;
      } else {
        auto final_rays = read_server_rays();
        per_restart_rays.push_back(final_rays);
        cumulative_rays += final_rays;
      }

      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

      double rays_per_sec = elapsed > 0 ? static_cast<double>(cumulative_rays) / elapsed : 0;
      fprintf(stderr, "[PERF] slider_drag: %.1f rays/sec (%lu rays in %.1fs)\n", rays_per_sec, cumulative_rays,
              elapsed);
      fprintf(stderr, "[PERF] slider_drag: %d param changes, %d restarts in %.1fs\n", iteration, restart_count,
              elapsed);

      auto texture_uploads = gui::g_state.texture_upload_count - upload_before;
      double upload_ratio = restart_count > 0 ? static_cast<double>(texture_uploads) / restart_count : 0;
      fprintf(stderr, "[PERF] slider_drag: %lu texture uploads / %d restarts (ratio: %.2f)\n", texture_uploads,
              restart_count, upload_ratio);

      // Report commit → first upload delay (responsiveness)
      if (!first_upload_ms.empty()) {
        std::sort(first_upload_ms.begin(), first_upload_ms.end());
        double fsum = 0;
        for (auto v : first_upload_ms) {
          fsum += v;
        }
        double favg = fsum / first_upload_ms.size();
        double fmedian = first_upload_ms[first_upload_ms.size() / 2];
        double fmin = first_upload_ms.front();
        double fmax = first_upload_ms.back();
        int n_miss = restart_count - static_cast<int>(first_upload_ms.size());
        fprintf(stderr,
                "[PERF] slider_drag: first_upload avg=%.0fms median=%.0fms min=%.0fms max=%.0fms (n=%zu, %d missed)\n",
                favg, fmedian, fmin, fmax, first_upload_ms.size(), n_miss);
      }

      // Report per-restart ray count distribution
      if (!per_restart_rays.empty()) {
        std::sort(per_restart_rays.begin(), per_restart_rays.end());
        unsigned long sum = 0;
        for (auto v : per_restart_rays) {
          sum += v;
        }
        double avg = static_cast<double>(sum) / per_restart_rays.size();
        auto median = per_restart_rays[per_restart_rays.size() / 2];
        auto min_val = per_restart_rays.front();
        auto max_val = per_restart_rays.back();
        fprintf(stderr, "[PERF] slider_drag: rays/restart avg=%.0f median=%lu min=%lu max=%lu (n=%zu)\n", avg, median,
                min_val, max_val, per_restart_rays.size());
      }

      // Report per-upload ray count distribution and CV (measures actual texture flicker)
      if (!per_upload_rays.empty()) {
        std::sort(per_upload_rays.begin(), per_upload_rays.end());
        double usum = 0;
        for (auto v : per_upload_rays) {
          usum += v;
        }
        double uavg = usum / per_upload_rays.size();
        auto umedian = per_upload_rays[per_upload_rays.size() / 2];
        auto umin = per_upload_rays.front();
        auto umax = per_upload_rays.back();

        // Compute CV (coefficient of variation) = stddev / mean
        double sq_diff_sum = 0;
        for (auto v : per_upload_rays) {
          double diff = v - uavg;
          sq_diff_sum += diff * diff;
        }
        double stddev = std::sqrt(sq_diff_sum / per_upload_rays.size());
        double cv = uavg > 0 ? stddev / uavg * 100.0 : 0;

        fprintf(stderr, "[PERF] slider_drag: upload_rays avg=%.0f median=%lu min=%lu max=%lu (n=%zu)\n", uavg, umedian,
                umin, umax, per_upload_rays.size());
        fprintf(stderr, "[PERF] slider_drag: upload_rays CV=%.1f%%\n", cv);
      }

      // Stop simulation BEFORE assertions: if IM_CHECK fails and aborts the test function,
      // the server would leak and its threads would run during static destruction, crashing
      // when accessing destroyed static locals (e.g., lens_proj_map in GetProjFunc).
      StopPerfSimulation();

      IM_CHECK_GT(rays_per_sec, 0.0);
    };
  }
}
