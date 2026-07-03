#ifndef LUMICE_GUI_SERVER_POLLER_HPP
#define LUMICE_GUI_SERVER_POLLER_HPP

#include <atomic>
#include <chrono>
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
  // Lifecycle signal (blueprint clock ④, see doc/gui-preview-lifecycle-architecture.md §6).
  // Mirrors the server's has_ever_consumed_ level — the SAME reliable signal the poller uses
  // to self-pause. Written on EVERY poll (not gated on snapshot generation, per invariant I4),
  // so the terminal IDLE frame durably carries the completion edge until the main thread
  // consumes it. Kept distinct from the display payload (stats / texture) on purpose.
  bool has_valid_data = false;
  LUMICE_RayCount stats_ray_seg_num = 0;
  LUMICE_RayCount stats_sim_ray_num = 0;
  std::vector<float> xyz_data;  // XYZ float texture data (for GPU conversion)
  int texture_width = 0;
  int texture_height = 0;
  float snapshot_intensity = 0;
  // Auto-EV anchor.  When the poller is configured with kEvAutoDownsampleFactor > 1
  // (see gui_ev_auto.hpp), this is the **fine-equivalent P99** = P99_coarse / f^2 and
  // is NOT a raw per-pixel Y statistic — use it only as the EV anchor that feeds
  // ComputeEvAuto, never as a true Y measurement.
  float p99_y = 0;
  float intensity_factor = 1.0f;
  int effective_pixels = 0;
  bool has_new_texture = false;
  LUMICE_RayCount texture_ray_count = 0;  // Ray count at the time texture data was captured (not global stats)
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

  // Idempotent: ensure the worker is in kRunning state.
  // If already kRunning, this is a no-op (zero overhead for the hot slider path).
  // If kPaused (after Stop/DoStop/self-pause), resumes polling with the given server.
  void EnsureRunning(LUMICE_Server* server);

  // Main thread: try to consume staged data. Returns true if new data was swapped in.
  // Uses try_lock so it never blocks the main thread.
  bool TrySyncData(PollerData& out);

  // Discard any staged texture data. Called before unlocking intensity_locked to prevent
  // old simulation data from being uploaded after a filter change.
  void InvalidateStagedTexture();

  // Set calibrated quality gate threshold (called once at startup after calibration run).
  // Thread-safe: only called from main thread before any Start().
  void SetCalibratedThreshold(unsigned long long threshold);

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

  // Adaptive quality gate: calibrated threshold set once at startup via SetCalibratedThreshold().
  // If not set (calibrated_ == false), falls back to gui::kMinRaysFloor.
  bool calibrated_{ false };
  unsigned long long calibrated_min_rays_{ 0 };

  // Timeout fallback: force upload if quality gate has been rejecting for too long.
  // Reset in Start(), updated in PollOnce() on each quality_ok pass.
  std::chrono::steady_clock::time_point last_quality_pass_time_{ std::chrono::steady_clock::now() };
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SERVER_POLLER_HPP
