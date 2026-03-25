#ifndef LUMICE_GUI_SERVER_POLLER_HPP
#define LUMICE_GUI_SERVER_POLLER_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

// Data produced by the background polling thread, consumed by the main thread.
struct PollerData {
  bool valid = false;  // Set to true by worker after first successful poll
  LUMICE_ServerState server_state = LUMICE_SERVER_IDLE;
  unsigned long stats_ray_seg_num = 0;
  unsigned long stats_sim_ray_num = 0;
  std::vector<float> xyz_data;  // XYZ float texture data (for GPU conversion)
  int texture_width = 0;
  int texture_height = 0;
  float snapshot_intensity = 0;
  float intensity_factor = 1.0f;
  bool has_new_texture = false;
};

// Polls the LUMICE server on a background thread (every kPollIntervalMs) and stages
// results for the main thread to pick up without blocking the UI.
//
// Uses a persistent thread model: the worker thread is spawned once in the constructor
// and joined in the destructor. Start()/Stop() signal the thread via condition variable,
// avoiding the ~14ms join/spawn overhead on Windows.
class ServerPoller {
 public:
  ServerPoller();
  ~ServerPoller();

  // Non-copyable, non-movable
  ServerPoller(const ServerPoller&) = delete;
  ServerPoller& operator=(const ServerPoller&) = delete;

  // Start polling with the given server. Sets server pointer and wakes worker.
  void Start(LUMICE_Server* server);

  // Synchronously pause the worker. Returns only after the worker has confirmed
  // it is no longer accessing the server. Safe to call multiple times.
  void Stop();

  // Main thread: try to consume staged data. Returns true if new data was swapped in.
  // Uses try_lock so it never blocks the main thread.
  bool TrySyncData(PollerData& out);

 private:
  enum class State { kPaused, kRunning, kTerminating };

  void WorkerLoop();
  void PollOnce();

  std::thread worker_;
  // state_ is atomic for lock-free reads in the poll loop; all writes are under mutex_.
  std::atomic<State> state_{ State::kPaused };
  std::mutex mutex_;
  std::condition_variable cv_;
  bool active_{ false };  // True while worker is in the poll loop (not in cv.wait)

  LUMICE_Server* server_{ nullptr };
  std::mutex data_mutex_;
  PollerData staged_;
  uint64_t last_generation_{ 0 };  // Tracks snapshot generation to detect new data
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SERVER_POLLER_HPP
