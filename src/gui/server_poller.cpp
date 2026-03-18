#include "gui/server_poller.hpp"

#include <chrono>
#include <cstring>

#include "gui/gui_constants.hpp"

namespace lumice::gui {

void ServerPoller::Start(LUMICE_Server* server) {
  if (!server) {
    return;
  }
  // Re-entrant safety: stop existing thread first
  Stop();

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

    // Stage all results under lock
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      staged_.valid = true;
      staged_.server_state = server_state;
      if (xyz_results[0].xyz_buffer != nullptr) {
        // Stats are updated inside GetRawXyzResults
        LUMICE_StatsResult stats[2]{};
        LUMICE_GetStatsResults(server, stats, 1);
        if (stats[0].sim_ray_num > 0) {
          staged_.stats_ray_seg_num = stats[0].ray_seg_num;
          staged_.stats_sim_ray_num = stats[0].sim_ray_num;
        }

        size_t float_count = static_cast<size_t>(xyz_results[0].img_width) * xyz_results[0].img_height * 3;
        staged_.xyz_data.resize(float_count);
        std::memcpy(staged_.xyz_data.data(), xyz_results[0].xyz_buffer, float_count * sizeof(float));
        staged_.texture_width = xyz_results[0].img_width;
        staged_.texture_height = xyz_results[0].img_height;
        staged_.snapshot_intensity = xyz_results[0].snapshot_intensity;
        staged_.intensity_factor = xyz_results[0].intensity_factor;
        staged_.has_new_texture = true;
      }
    }

    // If server is idle, simulation is done — exit the polling loop
    if (server_state == LUMICE_SERVER_IDLE) {
      running_ = false;
      break;
    }

    // Sleep kPollIntervalMs between polls.
    // Max shutdown delay equals kPollIntervalMs (acceptable for values <= 200ms).
    std::this_thread::sleep_for(std::chrono::milliseconds(gui::kPollIntervalMs));
  }
}

}  // namespace lumice::gui
