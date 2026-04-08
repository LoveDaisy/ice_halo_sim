#include "gui/server_poller.hpp"

#include <chrono>
#include <cstring>

#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"

namespace lumice::gui {

ServerPoller::ServerPoller() {
  worker_ = std::thread(&ServerPoller::WorkerLoop, this);
}

ServerPoller::~ServerPoller() {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.store(State::kTerminating);
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void ServerPoller::Start(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  GUI_LOG_DEBUG("[Poller] Start: server={}", fmt::ptr(server));

  // Synchronously pause first (no-op if already paused)
  Stop();

  // Minimal reset: only clear the valid flag to prevent SyncFromPoller() from acting on
  // stale data. Do NOT clear has_new_texture or server_state — preserving the last good
  // texture data keeps the preview on screen during the gap between restart and first new
  // snapshot, preventing visible flicker during slider scrubbing.
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    staged_.valid = false;
  }
  // Reset generation tracking so the worker detects the first snapshot as new data.
  last_generation_ = 0;
  // Reset quality gate timeout so fallback doesn't fire prematurely after restart.
  last_quality_pass_time_ = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lk(mutex_);
    server_ = server;
    state_.store(State::kRunning);
  }
  cv_.notify_all();
}


void ServerPoller::Stop() {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_.load() != State::kRunning) {
      return;
    }
    state_.store(State::kPaused);
  }
  cv_.notify_all();

  // Wait for worker to confirm it has left the poll loop
  {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [this] { return !active_; });
  }
  GUI_LOG_DEBUG("[Poller] Stop: worker paused");
}

void ServerPoller::EnsureRunning(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_.load() == State::kRunning) {
      return;  // Already running — zero overhead
    }
    if (state_.load() == State::kTerminating) {
      return;
    }
    // kPaused → resume polling
    server_ = server;
    last_generation_ = 0;
    last_quality_pass_time_ = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      staged_.valid = false;
    }
    state_.store(State::kRunning);
  }
  cv_.notify_all();
  GUI_LOG_DEBUG("[Poller] EnsureRunning: resumed from paused");
}

void ServerPoller::InvalidateStagedTexture() {
  std::lock_guard<std::mutex> lk(data_mutex_);
  staged_.has_new_texture = false;
}

void ServerPoller::SetCalibratedThreshold(unsigned long threshold) {
  calibrated_min_rays_ = threshold;
  calibrated_ = true;
  GUI_LOG_INFO("[Poller] calibration set: threshold={}", threshold);
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

void ServerPoller::WorkerLoop() {
  while (true) {
    {
      std::unique_lock<std::mutex> lk(mutex_);
      active_ = false;
      cv_.notify_all();  // Signal Stop() that worker is paused
      cv_.wait(lk, [this] { return state_.load() != State::kPaused; });
      if (state_.load() == State::kTerminating) {
        return;
      }
      active_ = true;
    }

    // Inner poll loop — runs while kRunning
    while (state_.load() == State::kRunning) {
      PollOnce();

      // Sleep with cv.wait_for so Stop()/~dtor can wake immediately
      // (replaces the old 1ms-increment sleep_for loop)
      std::unique_lock<std::mutex> lk(mutex_);
      auto sleep_duration = std::chrono::milliseconds(gui::kPollIntervalMs);
      cv_.wait_for(lk, sleep_duration, [this] { return state_.load() != State::kRunning; });
    }
  }
}

void ServerPoller::PollOnce() {
  auto* server = server_;
  if (!server) {
    return;
  }

  // Query server state
  LUMICE_ServerState server_state{};
  LUMICE_QueryServerState(server, &server_state);

  // Get raw XYZ results (skips CPU XYZ→RGB, for GPU conversion)
  LUMICE_RawXyzResult xyz_results[2]{};
  LUMICE_GetRawXyzResults(server, xyz_results, 1);

  // Check if this is genuinely new snapshot data (generation changed)
  bool has_new_snapshot =
      xyz_results[0].xyz_buffer != nullptr && xyz_results[0].snapshot_generation != last_generation_;

  // Stage results under lock. Quality gate: only overwrite texture data when the snapshot
  // has enough rays to avoid visible flicker. Stats and server_state are always updated.
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    staged_.valid = true;
    staged_.server_state = server_state;
    if (has_new_snapshot) {
      // Always consume this generation (same generation data won't improve by waiting)
      last_generation_ = xyz_results[0].snapshot_generation;

      // Get stats (always update — stats are used independently for status bar display)
      LUMICE_StatsResult cached_stats{};
      LUMICE_GetCachedStats(server, &cached_stats);
      if (cached_stats.sim_ray_num > 0) {
        staged_.stats_ray_seg_num = cached_stats.ray_seg_num;
        staged_.stats_sim_ray_num = cached_stats.sim_ray_num;
      }

      // Quality gate: skip texture overwrite for sparse snapshots (too few rays = visible flicker).
      // Cold start (sim_ray_num == 0) is allowed through — no "old good texture" to preserve.
      unsigned long min_rays = calibrated_ ? calibrated_min_rays_ : gui::kMinRaysFloor;
      bool quality_ok = cached_stats.sim_ray_num == 0 || cached_stats.sim_ray_num >= min_rays;

      // Timeout fallback: if quality gate has been rejecting for too long (e.g. empty filter),
      // force upload so stale textures don't persist indefinitely.
      auto now = std::chrono::steady_clock::now();
      if (!quality_ok) {
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_quality_pass_time_).count();
        if (elapsed_ms > gui::kQualityGateTimeoutMs) {
          quality_ok = true;
          GUI_LOG_VERBOSE("[Poller] quality gate timeout: forcing upload after {}ms (rays={}, min={})", elapsed_ms,
                          cached_stats.sim_ray_num, min_rays);
        }
      }

      if (quality_ok) {
        last_quality_pass_time_ = now;
        size_t float_count = static_cast<size_t>(xyz_results[0].img_width) * xyz_results[0].img_height * 3;
        staged_.xyz_data.resize(float_count);
        std::memcpy(staged_.xyz_data.data(), xyz_results[0].xyz_buffer, float_count * sizeof(float));
        staged_.texture_width = xyz_results[0].img_width;
        staged_.texture_height = xyz_results[0].img_height;
        staged_.snapshot_intensity = xyz_results[0].snapshot_intensity;
        staged_.intensity_factor = xyz_results[0].intensity_factor;
        staged_.effective_pixels = xyz_results[0].effective_pixels;
        staged_.has_new_texture = true;
        staged_.texture_ray_count = cached_stats.sim_ray_num;
        GUI_LOG_VERBOSE("[Poller] staged: rays={} intensity={} gen={}", cached_stats.sim_ray_num,
                        xyz_results[0].snapshot_intensity, xyz_results[0].snapshot_generation);
      } else {
        GUI_LOG_VERBOSE("[Poller] quality gate: skipped rays={} (min={}) gen={}", cached_stats.sim_ray_num, min_rays,
                        xyz_results[0].snapshot_generation);
      }
    }
  }

  // Only stop on IDLE if the server has actually produced valid data.
  // During restart, the server may transiently report IDLE before simulation threads spin up.
  if (server_state == LUMICE_SERVER_IDLE && xyz_results[0].has_valid_data) {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.store(State::kPaused);
  }
}

}  // namespace lumice::gui
