#include "gui/server_poller.hpp"

#include <chrono>
#include <cstring>

#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"

namespace lumice::gui {

void ServerPoller::Start(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  // Re-entrant safety: stop existing thread first
  Stop();

  // Selective reset: clear validity and server_state to prevent SyncFromPoller() from acting
  // on a stale IDLE state from the previous poller run. Crucially, do NOT clear has_new_texture —
  // preserving the last good texture data keeps the preview on screen during the gap between
  // restart and first new snapshot, preventing visible flicker during slider scrubbing.
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    staged_.valid = false;
    staged_.server_state = LUMICE_SERVER_IDLE;
  }
  // Reset generation tracking so the new worker detects the first snapshot as new data.
  // Record restart time for texture hold delay (see WorkerLoop).
  // Both are outside data_mutex_ because only the Start() caller writes them before
  // spawning the worker thread.
  last_generation_ = 0;
  restart_time_ = std::chrono::steady_clock::now();

  running_ = true;
  worker_ = std::thread(&ServerPoller::WorkerLoop, this, server);
}

void ServerPoller::Stop() {
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool ServerPoller::TrySyncData(PollerData& out) {
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  std::swap(out, staged_);
  staged_.has_new_texture = false;
  return true;
}

void ServerPoller::WorkerLoop(LUMICE_Server* server) {
  while (running_) {
    // Query server state
    LUMICE_ServerState server_state{};
    LUMICE_QueryServerState(server, &server_state);

    // Get raw XYZ results (skips CPU XYZ→RGB, for GPU conversion)
    LUMICE_RawXyzResult xyz_results[2]{};
    LUMICE_GetRawXyzResults(server, xyz_results, 1);

    // Check if this is genuinely new snapshot data (generation changed)
    bool has_new_snapshot =
        xyz_results[0].xyz_buffer != nullptr && xyz_results[0].snapshot_generation != last_generation_;

    // Texture hold delay: after restart, skip early sparse snapshots for kTextureHoldMs.
    // The earliest snapshots contain very few rays (only ~5ms of accumulation), which would
    // cause visible brightness flicker if displayed. By holding off, we let rays accumulate
    // before the first texture upload of each restart cycle.
    auto since_restart =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - restart_time_).count();
    bool hold_by_time = since_restart < gui::kTextureHoldMs;

    // Threshold-based gating: skip snapshots with too few rays (alternative/complement to time-based).
    // Check cached stats for the current ray count.
    bool hold_by_threshold = false;
    if (gui::kMinRaysForUpload > 0 && has_new_snapshot) {
      LUMICE_StatsResult threshold_stats{};
      LUMICE_GetCachedStats(server, &threshold_stats);
      hold_by_threshold = threshold_stats.sim_ray_num < static_cast<unsigned long>(gui::kMinRaysForUpload);
    }

    bool in_hold_window = hold_by_time || hold_by_threshold;
    if (has_new_snapshot && in_hold_window) {
      LUMICE_StatsResult hold_stats{};
      LUMICE_GetCachedStats(server, &hold_stats);
      GUI_LOG_DEBUG("[Poller] hold: time={}ms threshold={} rays={}", since_restart, hold_by_threshold,
                    hold_stats.sim_ray_num);
    }

    // Stage all results under lock
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      staged_.valid = true;
      staged_.server_state = server_state;
      if (has_new_snapshot && !in_hold_window) {
        // Copy XYZ data from the snapshot. Stats are included in xyz_results (populated by
        // GetRawXyzResults server-side) so we don't need a separate GetStatsResults call.
        // This avoids triggering DoSnapshot → PostSnapshot (heavy CPU XYZ→RGB conversion)
        // that the GPU rendering path doesn't need.
        size_t float_count = static_cast<size_t>(xyz_results[0].img_width) * xyz_results[0].img_height * 3;
        staged_.xyz_data.resize(float_count);
        std::memcpy(staged_.xyz_data.data(), xyz_results[0].xyz_buffer, float_count * sizeof(float));
        staged_.texture_width = xyz_results[0].img_width;
        staged_.texture_height = xyz_results[0].img_height;
        staged_.snapshot_intensity = xyz_results[0].snapshot_intensity;
        staged_.intensity_factor = xyz_results[0].intensity_factor;
        staged_.has_new_texture = true;

        // Get stats from lightweight cached API (updated by GetRawXyzResults above)
        LUMICE_StatsResult cached_stats{};
        LUMICE_GetCachedStats(server, &cached_stats);
        if (cached_stats.sim_ray_num > 0) {
          staged_.stats_ray_seg_num = cached_stats.ray_seg_num;
          staged_.stats_sim_ray_num = cached_stats.sim_ray_num;
        }
        GUI_LOG_DEBUG("[Poller] staged: since_restart={}ms rays={} intensity={} gen={}", since_restart,
                      cached_stats.sim_ray_num, xyz_results[0].snapshot_intensity, xyz_results[0].snapshot_generation);
      }
    }
    // Update generation tracking outside the lock — last_generation_ is only accessed by the
    // worker thread (Start() writes it before spawning the thread, establishing happens-before).
    // Updated unconditionally on new snapshot: during hold window this ensures the first post-hold
    // snapshot with fresh accumulated rays triggers the upload.
    if (has_new_snapshot) {
      last_generation_ = xyz_results[0].snapshot_generation;
    }

    // If server is idle and has produced valid data, simulation is done — exit the polling loop.
    // During restart, the server may briefly report IDLE before GenerateScene starts producing
    // batches. Requiring has_valid_data prevents premature exit in that transient window.
    if (server_state == LUMICE_SERVER_IDLE && xyz_results[0].has_valid_data) {
      running_ = false;
      break;
    }

    // After a restart, the server needs a few ms to accumulate rays before the first snapshot
    // has data. Use a short sleep (5ms) when no valid data has been produced yet (has_valid_data
    // is false until ConsumeData processes the first batch), to catch the first texture quickly.
    // Once valid data is available, switch to normal poll interval.
    // Sleep in 1ms increments so Stop() can join quickly (max 1ms delay vs full sleep).
    int sleep_ms = xyz_results[0].has_valid_data ? gui::kPollIntervalMs : 5;
    for (int i = 0; i < sleep_ms && running_; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

}  // namespace lumice::gui
