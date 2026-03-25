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
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      staged_.valid = false;
    }
    state_.store(State::kRunning);
  }
  cv_.notify_all();
  GUI_LOG_DEBUG("[Poller] EnsureRunning: resumed from paused");
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

  // Stage all results under lock. No hold filtering here — the main thread (SyncFromPoller)
  // decides whether to upload based on time-since-restart. This ensures last_generation_ is
  // only updated when data is actually staged, avoiding the "consumed but not staged" gap.
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    staged_.valid = true;
    staged_.server_state = server_state;
    if (has_new_snapshot) {
      size_t float_count = static_cast<size_t>(xyz_results[0].img_width) * xyz_results[0].img_height * 3;
      staged_.xyz_data.resize(float_count);
      std::memcpy(staged_.xyz_data.data(), xyz_results[0].xyz_buffer, float_count * sizeof(float));
      staged_.texture_width = xyz_results[0].img_width;
      staged_.texture_height = xyz_results[0].img_height;
      staged_.snapshot_intensity = xyz_results[0].snapshot_intensity;
      staged_.intensity_factor = xyz_results[0].intensity_factor;
      staged_.has_new_texture = true;
      last_generation_ = xyz_results[0].snapshot_generation;

      // Get stats from lightweight cached API (updated by GetRawXyzResults above)
      LUMICE_StatsResult cached_stats{};
      LUMICE_GetCachedStats(server, &cached_stats);
      if (cached_stats.sim_ray_num > 0) {
        staged_.stats_ray_seg_num = cached_stats.ray_seg_num;
        staged_.stats_sim_ray_num = cached_stats.sim_ray_num;
      }
      GUI_LOG_DEBUG("[Poller] staged: rays={} intensity={} gen={}", cached_stats.sim_ray_num,
                    xyz_results[0].snapshot_intensity, xyz_results[0].snapshot_generation);
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
