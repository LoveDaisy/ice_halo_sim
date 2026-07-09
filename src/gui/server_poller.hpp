#ifndef LUMICE_GUI_SERVER_POLLER_HPP
#define LUMICE_GUI_SERVER_POLLER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

// Texture payload: materialized once, then read-only. Anti-flicker carry-forward reuses the
// same shared_ptr across sparse polls (zero pixel copy — only a refcount bump). See §5.4 of
// doc/gui-preview-lifecycle-architecture.md and the versioned-snapshot-handoff plan.
struct TexturePayload {
  std::vector<float> xyz_data;  // XYZ float texture data (for GPU conversion)
  int width = 0;
  int height = 0;
  float snapshot_intensity = 0;
  float intensity_factor = 1.0f;
  int effective_pixels = 0;
  // Ray count at the time texture data was captured (not global stats).
  LUMICE_RayCount texture_ray_count = 0;
  // Auto-EV anchor.  When the poller is configured with kEvAutoDownsampleFactor > 1
  // (see gui_ev_auto.hpp), this is the **fine-equivalent P99** = P99_coarse / f^2 and
  // is NOT a raw per-pixel Y statistic — use it only as the EV anchor that feeds
  // ComputeEvAuto, never as a true Y measurement.
  float p99_y = 0;
  // RawXyzResult.epoch: the lifecycle epoch this texture was produced under. May lag the
  // bundle epoch when carried forward (1.5 display keying distinguishes the two).
  unsigned long long payload_epoch = 0;

  // task-342.4 Step 2: composite (raypath_color) surface.
  // rgb_data is a sRGB uint8 buffer (W*H*3), populated ONLY when raypath_color is
  // active on this snapshot. is_composite tells the main-thread upload path which
  // GL texture format + shader mode to use:
  //   is_composite == true  → UploadTexture(rgb_data, W, H) + u_xyz_mode=0
  //   is_composite == false → UploadXyzTexture(xyz_data, W, H) + u_xyz_mode=1 (unchanged)
  // xyz_data / p99_y / snapshot_intensity / effective_pixels are ALWAYS populated
  // (auto-EV + quality gate are not touched by this change), even when is_composite
  // is true — see plan §3 keypoint 3.
  std::vector<unsigned char> rgb_data;
  bool is_composite = false;
};

// The single cross-thread handoff unit. Constructed fully, then treated as const: the whole
// object is published/consumed atomically via a pointer swap (invariant I5 — the consumer
// never observes a half-updated field combination). Replaces the old torn-read PollerData
// field-bag + data_mutex_/staged_/std::swap channel.
struct PreviewSnapshot {
  bool valid = false;  // false until the worker's first successful poll
  // Bundle epoch: committed lifecycle epoch at poll time (LUMICE_GetSimLifecycle). One epoch
  // covers the whole coherent snapshot (stats + lifecycle + texture); see §6 (StatsResult.epoch
  // deliberately NOT added — the bundle epoch suffices).
  unsigned long long epoch = 0;
  // GetSimLifecycle.lifecycle (blueprint clock ④, see §6). This is the sole completion signal the
  // main thread reconciles on (ReconcileSimState: COMPLETED@matching-epoch → kDone) and the sole
  // signal the poller self-pauses on. Carried on EVERY poll (not gated on snapshot generation, per
  // invariant I4), so the terminal COMPLETED frame durably carries the completion edge until the
  // main thread consumes it. 1.5 removed the has_valid_data + server_state side-signals it replaced.
  int lifecycle = LUMICE_LIFECYCLE_IDLE;
  LUMICE_RayCount stats_ray_seg_num = 0;
  LUMICE_RayCount stats_sim_ray_num = 0;
  // Display payload: shared, immutable. Null / carried-forward on sparse / gate-rejected /
  // invalidated polls.
  std::shared_ptr<const TexturePayload> payload;
  // Bumped on each fresh texture materialization; kept unchanged on carry-forward. The consumer
  // dedups on serial (exact-once upload) instead of the old swap-reset, because immutable reads
  // are non-destructive (§5.3).
  unsigned long long texture_serial = 0;
  // Descriptive bit: did THIS poll just materialize a texture? Behavior-equivalent / observable;
  // the actual upload gate is driven by texture_serial dedup (§5), not this flag.
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

  // Idempotent: ensure the worker is in kRunning state.
  // If already kRunning, this is a no-op (zero overhead for the hot slider path).
  // If kPaused (after Stop/DoStop/self-pause), resumes polling with the given server.
  void EnsureRunning(LUMICE_Server* server);

  // Main thread (hot path, every frame): atomically load the latest published snapshot.
  // Lock-free (single atomic_load of a shared_ptr) — never blocks the main thread. Returns a
  // shared_ptr so the caller keeps the whole immutable snapshot alive while reading it. May
  // return null before the worker's first publish.
  std::shared_ptr<const PreviewSnapshot> LoadSnapshot() const { return LoadPublished(); }

  // Display-invalidate seam: publish a payload=null copy of the current snapshot so the consumer's
  // upload gate skips this frame (GL keeps the already-uploaded texture — no black flicker). Not
  // called in production anymore (the epoch floor fences stale data — see MarkFilterDirty); retained
  // as a test seam (test_gui_lifecycle drives no-GL-context interleavings through it) and for a
  // future optimistic-clear path.
  void InvalidateStagedTexture();

  // Set calibrated quality gate threshold (called once at startup after calibration run).
  // Thread-safe: only called from main thread before any Start().
  void SetCalibratedThreshold(unsigned long long threshold);

  // ---- Test-only synchronous seam (see test/gui/functional/test_gui_lifecycle.cpp) ----
  // Drive exactly ONE poll against `server` on the CALLING thread, bypassing the worker
  // thread, so a regression test can deterministically construct the poll/sync interleaving
  // that used to lose the terminal completion edge (doc/gui-preview-lifecycle-architecture.md
  // §2/§6, invariants I3/I4). Not used in production — the real path is Start()/WorkerLoop.
  void PollOnceForTest(LUMICE_Server* server) {
    server_ = server;
    PollOnce();
  }
  // Test-only: reset snapshot-generation tracking so the next PollOnceForTest() treats the
  // server's current generation as new (what Start() does, minus the worker thread).
  void ResetGenerationForTest() { last_generation_ = 0; }

  // Test-only: drives PopulateCompositePayload() directly with caller-supplied
  // composite_result so a regression test can pin the "composite bytes are
  // copied and is_composite becomes true" invariant against the same code path
  // PollOnce() drives. Post-345.2 there is no drift-guard branch to exercise
  // separately — the atomic combined C-API (LUMICE_GetRawXyzAndCompositeResults)
  // makes cross-generation pairing structurally impossible upstream. See
  // test/gui/functional/test_gui_composite_preview.cpp.
  void PopulateCompositePayloadForTest(const LUMICE_RenderResult& composite_result, TexturePayload* payload) {
    PopulateCompositePayload(composite_result, payload);
  }

 private:
  enum class State { kPaused, kRunning, kTerminating };

  void WorkerLoop();
  void PollOnce();

  // Copies composite_results[0]'s RGB bytes into payload->rgb_data and sets
  // is_composite=true. Split out of PollOnce() to keep its cognitive
  // complexity down. Post-345.2: no drift-guard — the caller already sourced
  // xyz + composite from a single LUMICE_GetRawXyzAndCompositeResults() call,
  // so they belong to the same server snapshot_generation by construction.
  void PopulateCompositePayload(const LUMICE_RenderResult& composite_result, TexturePayload* payload);

  // Published-snapshot access helpers. C++17 has no std::atomic<std::shared_ptr<T>>, so we use
  // the C++11 free functions std::atomic_load/atomic_store (deprecated in C++20 but valid here).
  // MIGRATION: when the project moves to C++20, change published_ to
  // std::atomic<std::shared_ptr<const PreviewSnapshot>> and rewrite these two helpers to use
  // published_.load()/.store() — only these two functions change.
  std::shared_ptr<const PreviewSnapshot> LoadPublished() const { return std::atomic_load(&published_); }
  void StorePublished(std::shared_ptr<const PreviewSnapshot> next) { std::atomic_store(&published_, std::move(next)); }

  // Publish a valid=false copy of the current snapshot (preserving payload for anti-flicker).
  // Serializes under publish_mutex_. Called from Start()/EnsureRunning() on the main thread.
  void PublishValidReset();

  std::thread worker_;
  // state_ is atomic for lock-free reads in the poll loop; all writes are under mutex_.
  std::atomic<State> state_{ State::kPaused };
  std::mutex mutex_;
  std::condition_variable cv_;
  bool active_{ false };  // True while worker is in the poll loop (not in cv.wait)

  LUMICE_Server* server_{ nullptr };
  // Single versioned immutable handoff (invariant I5). Accessed ONLY via LoadPublished/
  // StorePublished (atomic_load/store). Consumers read lock-free; producers serialize their
  // read-modify-write publish under publish_mutex_.
  std::shared_ptr<const PreviewSnapshot> published_;
  // Serializes producer-side RMW (build-from-prev + store) so concurrent publishers
  // (worker PollOnce ‖ main-thread InvalidateStagedTexture/Start/EnsureRunning) never lose an
  // update. Consumers never take this lock. The 24MB pixel memcpy is prepared OUTSIDE this lock;
  // the critical section is pointer/refcount-level only (see PollOnce).
  std::mutex publish_mutex_;
  std::atomic<uint64_t> texture_serial_{ 0 };  // Monotonic texture serial source (producer side)
  uint64_t last_generation_{ 0 };              // Tracks snapshot generation to detect new data

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
