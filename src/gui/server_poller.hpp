#ifndef LUMICE_GUI_SERVER_POLLER_HPP
#define LUMICE_GUI_SERVER_POLLER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "include/lumice.h"

namespace lumice::gui {

// Texture payload: materialized once, then read-only. Materializing it costs no pixel copy at
// all — it takes a share of the result frame's lifetime and points at the frame's own buffers.
// Anti-flicker carry-forward then reuses the same shared_ptr across sparse polls, so that path
// is a refcount bump on an object that is itself already copy-free (§5.4 of
// doc/gui-preview-lifecycle-architecture.md and the versioned-snapshot-handoff plan).
//
// INVARIANT: a TexturePayload is only ever shared whole, via shared_ptr<const TexturePayload>.
// It is never copied or sliced by value — the buffer pointers below and the `frame` that keeps
// them alive must not be separable. Copy is deleted below to make that a compile-time gate
// rather than a comment-only convention.
struct TexturePayload {
  TexturePayload() = default;
  TexturePayload(const TexturePayload&) = delete;
  TexturePayload& operator=(const TexturePayload&) = delete;

  // The sole reason xyz_buffer / rgb_buffer below are safe to dereference: a result frame keeps
  // its pixel storage alive for exactly as long as it is held, no matter how many further
  // snapshots the server publishes. As long as this TexturePayload lives, so does the frame,
  // and so do those buffers.
  std::shared_ptr<LUMICE_ResultFrame> frame;
  // XYZ float texture data (for GPU conversion), W*H*3 — a view INTO `frame`.
  // Only assign this together with `frame`; never point it at anything the frame does not own.
  const float* xyz_buffer = nullptr;
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
  // rgb_buffer is a sRGB uint8 buffer (W*H*3) inside the SAME `frame` as xyz_buffer, non-null
  // ONLY when raypath_color is active on this snapshot. Like xyz_buffer it is a view — only
  // assign it together with `frame`. is_composite tells the main-thread upload path which
  // GL texture format + shader mode to use:
  //   is_composite == true  → UploadTexture(rgb_buffer, W, H) + u_xyz_mode=0
  //   is_composite == false → UploadXyzTexture(xyz_buffer, W, H) + u_xyz_mode=1 (unchanged)
  // xyz_buffer / p99_y / snapshot_intensity / effective_pixels are ALWAYS populated
  // (auto-EV + quality gate are not touched by this change), even when is_composite
  // is true — see plan §3 keypoint 3.
  const unsigned char* rgb_buffer = nullptr;
  bool is_composite = false;
};

// The single cross-thread handoff unit. Constructed fully, then treated as const: the whole
// object is published/consumed atomically via a pointer swap (invariant I5 — the consumer
// never observes a half-updated field combination). Replaces the old torn-read PollerData
// field-bag + data_mutex_/staged_/std::swap channel.
struct PreviewSnapshot {
  bool valid = false;  // false until the worker's first successful poll
  // Bundle epoch: committed lifecycle epoch at poll time (LUMICE_GetSimLifecycle). Re-stamped on
  // EVERY publish, so it answers "which generation was committed when this poll ran" — NOT "which
  // generation produced the contents". Fields that can outlive their generation via carry-forward
  // therefore carry their own stamp beside it (payload_epoch for the texture, stats_epoch below).
  // This used to read "StatsResult.epoch deliberately NOT added — the bundle epoch suffices"; that
  // was wrong, and stale stats reaching the status bar after a restart is what disproved it.
  unsigned long long epoch = 0;
  // GetSimLifecycle.lifecycle (blueprint clock ④, see §6). This is the sole completion signal the
  // main thread reconciles on (ReconcileSimState: COMPLETED@matching-epoch → kDone) and the sole
  // signal the poller self-pauses on. Carried on EVERY poll (not gated on snapshot generation, per
  // invariant I4), so the terminal COMPLETED frame durably carries the completion edge until the
  // main thread consumes it. 1.5 removed the has_valid_data + server_state side-signals it replaced.
  int lifecycle = LUMICE_LIFECYCLE_IDLE;
  LUMICE_RayCount stats_ray_seg_num = 0;
  LUMICE_RayCount stats_sim_ray_num = 0;
  // Sampling-density counters, carried in the same coherent bundle as the two above.
  LUMICE_RayCount stats_crystal_num = 0;
  LUMICE_RayCount stats_orientation_num = 0;
  // The lifecycle epoch the four stats fields above were actually produced under. Symmetric to
  // TexturePayload::payload_epoch: it may LAG the bundle epoch when the stats are carried forward
  // across a restart, and that lag is exactly the signal the consumer needs. The server's cached
  // stats survive a restart (ServerImpl::Stop clears the dirty/consumed flags but not
  // cached_stats_result_) while the bundle epoch advances immediately, so without this stamp a
  // carried-forward value is indistinguishable from a freshly produced one and the previous run's
  // ray/crystal/sampling counts reappear on the status bar. Consumed via ShouldApplyStats.
  unsigned long long stats_epoch = 0;
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

// The self-pause predicate (invariant I3a). Pure: no locks, no member access, no server calls —
// which is the point. "May the poller stop polling?" is the single most consequential decision in
// the preview lifecycle (stop too early and the terminal truth never reaches the screen), so it is
// a free function with a truth table rather than a condition buried in PollOnce.
//
// Three conditions, and the last two are NOT the same one twice:
//   1. lc.lifecycle == COMPLETED — the durable terminal edge (infinite runs stay RUNNING; a
//      transient restart IDLE is not COMPLETED), so this never pauses mid-run.
//   2. drain.drained_epoch == lc.epoch — the CONSUMER has finished draining this epoch. COMPLETED
//      alone is a producer-side verdict: it is derived from "no simulator busy, nothing queued to
//      generate, generation done" and asks nothing about whether the consumer thread has emptied
//      its queue, so the accumulated statistics can still be a PARTIAL total at the instant
//      COMPLETED first appears (measured as orientation_num 19616 against 20000 — a whole number
//      of 128-ray dispatch grains short). Pausing there freezes those partial counts on the status
//      bar with no later poll to correct them. See LUMICE_DrainResult in the public header.
//   3. drain.current_epoch == lc.epoch — no NEWER epoch has been committed since `lc` was read at
//      the top of the poll. The whole function body sits between those two reads, so a
//      CommitScene + WakeForRestart from the main thread can land in the middle; without this
//      term the poller could read "epoch N drained" and pause a run that is already on epoch N+1.
//      Note that condition 2 does not subsume this: drained_epoch is monotonic and lc.epoch is a
//      snapshot, so `drained == lc.epoch` stays true across a commit that has bumped
//      current_epoch past both.
bool ShouldSelfPause(const LUMICE_SimLifecycleResult& lc, const LUMICE_DrainResult& drain);

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
  //
  // "No longer accessing the server" is load-bearing, not a nicety: callers destroy the server
  // immediately afterwards (MaybeReconstructServerForBackend does Stop() then
  // LUMICE_DestroyServer; DoStop does Stop() then LUMICE_StopServer), so any worker still inside
  // PollOnce() after this returns is a use-after-free. Both non-terminal states quiesce here —
  // kRunning (full-speed loop) and kIdleHeartbeat (slow self-heal loop) — and the wait is on
  // active_, which covers ANY in-flight PollOnce() regardless of which loop issued it.
  void Stop();

  // Idempotent: ensure the worker is in kRunning state for a fresh sim generation.
  // If already kRunning, this is a no-op (zero overhead for the hot slider path).
  // If kPaused (after Stop/DoStop/self-pause), resumes polling with the given server
  // AND publishes a valid=false copy of the current snapshot so consumers ignore stale
  // stats/lifecycle until the worker republishes. Callers: DoRun success (fresh commit),
  // DoRun failure recovery (task-color-migration Minor 4). Do NOT call this from display-
  // time edits (color/visible/solo/z_order/mode/composite EV) — see WakeForRefresh.
  void WakeForRestart(LUMICE_Server* server);

  // Idempotent: ensure the worker is in kRunning state for a display-time refresh, WITHOUT
  // publishing valid=false. The distinction is critical: display-time edits (after a finite
  // sim has completed) must not open a valid=false window that would let SyncFromPoller
  // observe an invalid snapshot and pull sim_state back into kSimulating (doc/gui-state-
  // governance.md §4 支柱 2, task-color-migration AC1 / activity bug root cause (a)).
  // Callers: PushDisplayState (color-window color/visible/solo/z_order/mode edits) and
  // app_panels composite EV push. If kPaused, resumes polling with the given server; if
  // already kRunning or kTerminating, no-op.
  void WakeForRefresh(LUMICE_Server* server);

  // Main thread (hot path, every frame): atomically load the latest published snapshot.
  // Lock-free (single atomic_load of a shared_ptr) — never blocks the main thread. Returns a
  // shared_ptr so the caller keeps the whole immutable snapshot alive while reading it. May
  // return null before the worker's first publish.
  std::shared_ptr<const PreviewSnapshot> LoadSnapshot() const { return LoadPublished(); }

  // Display-invalidate seam: publish a payload=null copy of the current snapshot so the consumer's
  // upload gate skips this frame (GL keeps the already-uploaded texture — no black flicker). serial
  // is left unchanged so a genuinely new future texture still gets a fresh serial and uploads.
  // Production callers: DoOpen (.json + .lmc) and DoNew — document-switch fences the staged
  // composite so SyncFromPoller won't re-upload the previous scene's snapshot over the just-cleared
  // preview (mode_changed OR-branch does not honor the epoch floor; must drop the payload). Also
  // retained as a test seam (test_server_poller.cpp and test_composite_preview.cpp, both under
  // test/unit-correctness/gui/, drive no-GL-context interleavings through it).
  void InvalidateStagedTexture();

  // Set calibrated quality gate threshold (called once at startup after calibration run).
  // Thread-safe: only called from main thread before any Start().
  void SetCalibratedThreshold(unsigned long long threshold);

  // ---- Test-only synchronous seam (see test/unit-correctness/gui/test_server_poller.cpp) ----
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

  // Test-only: pre-match the generation tracker to an arbitrary value (normally the server's
  // CURRENT generation) so the next PollOnceForTest() sees has_new_snapshot == false. Needed to
  // construct the "gate closed" side of invariant I4a on a poller's very first poll — a state
  // ResetGenerationForTest() cannot reach, since its 0-start always reads the first real
  // generation as new. Not used in production.
  void SetGenerationForTest(uint64_t generation) { last_generation_ = generation; }

  // Test-only: pre-arm uploaded_since_resume_ so force_final_upload reads false even under a
  // COMPLETED lifecycle. Paired with SetGenerationForTest() to close BOTH disjuncts of
  // `has_new_snapshot || force_final_upload` for a regression test — see the I4a case in
  // test/unit-correctness/gui/test_server_poller.cpp
  // (ServerPoller.StatsAreReadOnAFirstPollWithBothMaterializeDoorsClosed). Not used in production.
  void SetUploadedSinceResumeForTest(bool uploaded) { uploaded_since_resume_ = uploaded; }

  // Test-only: exposes the private PublishValidReset() seam so a regression test can construct the
  // exact "valid=false snapshot published while under a kRunning intent" observation that used to
  // pull a completed sim back into kSimulating (AC1 activity bug root cause (a), before the
  // WakeForRefresh split). Not used in production.
  void PublishValidResetForTest() { PublishValidReset(); }

  // Test-only: how many slow-heartbeat ticks the worker has run since construction. Monotonic;
  // bumped ONLY by the kIdleHeartbeat branch of WorkerLoop, never by the full-speed poll loop nor
  // by PollOnceForTest. Two things need to be observable and neither has another cheap signal:
  // that the heartbeat runs at all after a self-pause (I3b), and that it has stopped after Stop()
  // returned (the quiescence contract above).
  uint64_t HeartbeatTickCountForTest() const { return heartbeat_tick_count_.load(std::memory_order_relaxed); }

 private:
  // kPaused and kIdleHeartbeat are BOTH "not polling at full speed", and splitting them is what
  // makes the heartbeat safe rather than a UAF:
  //   kPaused         — explicit stop (Stop()). Zero polling, untimed cv_.wait. The worker will
  //                     not touch server_ again until someone resumes it, which is precisely the
  //                     guarantee Stop()'s callers destroy the server on.
  //   kIdleHeartbeat  — self-pause: the poller believes the run reached its terminal state and
  //                     drained. It keeps polling at kIdleHeartbeatIntervalMs so a wrong belief
  //                     self-corrects (I3b). Entered ONLY from kRunning, by PollOnce()'s tail.
  // Before the split, "self-paused" and "not touching the server" were the same state, so Stop()
  // could early-return on it. With a heartbeat running they are different facts and Stop() must
  // drive kIdleHeartbeat → kPaused and wait, or it would return while a tick was in flight.
  enum class State { kPaused, kRunning, kIdleHeartbeat, kTerminating };

  void WorkerLoop();
  void PollOnce();

  // Published-snapshot access helpers. C++17 has no std::atomic<std::shared_ptr<T>>, so we use
  // the C++11 free functions std::atomic_load/atomic_store (deprecated in C++20 but valid here).
  // MIGRATION: when the project moves to C++20, change published_ to
  // std::atomic<std::shared_ptr<const PreviewSnapshot>> and rewrite these two helpers to use
  // published_.load()/.store() — only these two functions change.
  std::shared_ptr<const PreviewSnapshot> LoadPublished() const { return std::atomic_load(&published_); }
  void StorePublished(std::shared_ptr<const PreviewSnapshot> next) { std::atomic_store(&published_, std::move(next)); }

  // Publish a valid=false copy of the current snapshot (preserving payload for anti-flicker).
  // Serializes under publish_mutex_. Called from Start()/WakeForRestart() on the main thread.
  void PublishValidReset();

  // Single owner of the per-RESUME state reset. The three fields it clears
  // (last_generation_ / last_quality_pass_time_ / uploaded_since_resume_) share one scope — they
  // describe "what has happened since the worker last resumed" — so they must be re-armed together
  // at every kPaused→kRunning edge or the survivors silently misreport the new resume as a
  // continuation of the old one. Start() does not route through TransitionToRunning(), so both call
  // this; before this method existed each carried its own verbatim copy of the same three lines and
  // the invariant was held only by a comment. Callers must already hold whatever lock their own
  // contract requires (TransitionToRunning calls it under mutex_; Start() before taking it) — this
  // method takes no lock of its own.
  void ResetPerResumeState();

  // Shared body of WakeForRestart/WakeForRefresh: idempotently drive kPaused → kRunning with
  // `server`, resetting the generation/quality-pass tracking under mutex_ and notifying the
  // worker. The ONLY behavioral difference between the two public callers is whether a
  // valid=false snapshot is published across the wake edge, so it is a single bool parameter:
  //   publish_reset=true  (WakeForRestart) — fresh sim generation; consumers must ignore stale
  //                        stats/lifecycle until the worker republishes.
  //   publish_reset=false (WakeForRefresh) — display-time refresh; the current snapshot must
  //                        stay `valid` across the wake so SyncFromPoller does not transiently
  //                        observe an invalid snapshot (doc/gui-state-governance.md §4 支柱 2).
  // Returns true iff it actually resumed (was kPaused); false on every no-op path (null server,
  // already kRunning, kTerminating) so callers log their resume line only on a real transition,
  // matching the pre-refactor behavior.
  bool TransitionToRunning(LUMICE_Server* server, bool publish_reset);

  std::thread worker_;
  // state_ is atomic for lock-free reads in the poll loop; all writes are under mutex_.
  std::atomic<State> state_{ State::kPaused };
  std::mutex mutex_;
  std::condition_variable cv_;
  // True while the worker is executing ANY PollOnce() — the full-speed loop or a single heartbeat
  // tick. It used to mean "inside the full-speed poll loop", which was the same thing when the
  // only alternative to polling was total silence; the heartbeat made those two statements
  // different, and it is this one — "might be dereferencing server_ right now" — that Stop()
  // needs to wait on.
  bool active_{ false };

  LUMICE_Server* server_{ nullptr };
  // Single versioned immutable handoff (invariant I5). Accessed ONLY via LoadPublished/
  // StorePublished (atomic_load/store). Consumers read lock-free; producers serialize their
  // read-modify-write publish under publish_mutex_.
  std::shared_ptr<const PreviewSnapshot> published_;
  // Serializes producer-side RMW (build-from-prev + store) so concurrent publishers
  // (worker PollOnce ‖ main-thread InvalidateStagedTexture/Start/WakeForRestart/WakeForRefresh) never lose an
  // update. Consumers never take this lock. The candidate payload is built OUTSIDE this lock; the
  // critical section is pointer/refcount-level only (see PollOnce).
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

  // Terminal-frame rescue (invariant I6, doc/gui-preview-lifecycle-architecture.md §7 rule 2 /
  // §9): has a payload been materialized since the worker last resumed? Scope is per-RESUME, not
  // per-poller-lifetime: it is reset alongside last_generation_/last_quality_pass_time_ at every
  // kPaused→kRunning edge (Start() and TransitionToRunning(), i.e. both WakeForRestart — a fresh
  // sim generation — and WakeForRefresh — one more poll to re-materialize a completed run under
  // new display state). A poller-lifetime flag would rescue only the first low-ray completion in
  // the process and silently leave every later one blank.
  //
  // PollOnce() consults it to let a COMPLETED generation bypass the quality gate exactly once:
  // the gate suppresses sparse frames because a better one is still coming, and once the run is
  // over that premise is false. `false` here is the sole guard keeping the bypass from firing on
  // every completed poll (which would overwrite a good frame with a sparse one — the very
  // flicker the gate exists to prevent), so it must stay in the condition.
  bool uploaded_since_resume_{ false };

  // Slow-heartbeat tick counter (see HeartbeatTickCountForTest). Atomic only because tests read it
  // from the main thread while the worker writes it; the worker is its sole writer.
  std::atomic<uint64_t> heartbeat_tick_count_{ 0 };

  // Probe latch for a state believed unreachable: observing lifecycle != COMPLETED while in
  // kIdleHeartbeat. A whole-tree enumeration of the paths that can move the lifecycle off
  // COMPLETED (CommitScene in DoRun, StopServer in DoStop, StopServer in the backend rebuild)
  // found every one of them immediately preceded or followed by a poller wake/stop, so no such
  // observation should exist. Rather than write a recovery branch for a state nothing can reach —
  // untestable by construction, and a permanent puzzle for the next reader — the belief is simply
  // instrumented: silent and free if it holds, and if it does not, we learn that instead of having
  // it papered over. Latched per episode (re-armed on the next COMPLETED observation) so a
  // standing violation logs once rather than twice a second forever.
  // Worker-thread-only; no synchronization needed.
  bool heartbeat_unreachable_logged_{ false };
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SERVER_POLLER_HPP
