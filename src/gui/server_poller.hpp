#ifndef LUMICE_GUI_SERVER_POLLER_HPP
#define LUMICE_GUI_SERVER_POLLER_HPP

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

// Data produced by the background polling thread, consumed by the main thread.
struct PollerData {
  LUMICE_ServerState server_state = LUMICE_SERVER_IDLE;
  unsigned long stats_ray_seg_num = 0;
  unsigned long stats_sim_ray_num = 0;
  std::vector<unsigned char> texture_data;
  int texture_width = 0;
  int texture_height = 0;
  bool has_new_texture = false;
};

// Polls the LUMICE server on a background thread (every ~1 second) and stages
// results for the main thread to pick up without blocking the UI.
class ServerPoller {
 public:
  ServerPoller() = default;
  ~ServerPoller() { Stop(); }

  // Non-copyable, non-movable
  ServerPoller(const ServerPoller&) = delete;
  ServerPoller& operator=(const ServerPoller&) = delete;

  // Start polling. If already running, stops first. No-op if server is null.
  void Start(LUMICE_Server* server);

  // Stop polling and join the background thread. Safe to call multiple times.
  void Stop();

  // Main thread: try to consume staged data. Returns true if new data was swapped in.
  // Uses try_lock so it never blocks the main thread.
  bool TrySyncData(PollerData& out);

 private:
  void WorkerLoop(LUMICE_Server* server);

  std::thread worker_;
  std::atomic<bool> running_{ false };
  std::mutex data_mutex_;
  PollerData staged_;
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SERVER_POLLER_HPP
