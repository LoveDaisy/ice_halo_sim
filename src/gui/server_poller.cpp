#include "gui/server_poller.hpp"

#include <chrono>
#include <memory>

#include "gui/gui_constants.hpp"
#include "gui/gui_ev_auto.hpp"
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
  // stale data. Do NOT drop the payload — preserving the last good texture (carried forward via
  // the shared payload pointer) keeps the preview on screen during the gap between restart and
  // first new snapshot, preventing visible flicker during slider scrubbing.
  PublishValidReset();
  ResetPerResumeState();

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
    const State s = state_.load();
    // kIdleHeartbeat is NOT a rest state to early-return on: the worker still wakes every
    // kIdleHeartbeatIntervalMs and dereferences server_. Skipping the handshake for it would let
    // Stop() return with a tick either in flight or one timer edge away, and every caller of
    // Stop() destroys or stops the server right afterwards.
    if (s != State::kRunning && s != State::kIdleHeartbeat) {
      return;
    }
    state_.store(State::kPaused);
  }
  cv_.notify_all();

  // Wait for the worker to confirm it is not inside any PollOnce() — full-speed or heartbeat.
  {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [this] { return !active_; });
  }
  GUI_LOG_DEBUG("[Poller] Stop: worker paused");
}

// Shared kPaused → kRunning transition for both public wake entry points. The valid=false publish
// is the ONLY behavioral difference between restart and refresh, so it is a single bool parameter —
// see the header contract on TransitionToRunning / WakeForRestart / WakeForRefresh. Returns true iff
// it actually resumed, so callers log their resume line only on a real transition (pre-refactor
// behavior — no-op paths logged nothing).
bool ServerPoller::TransitionToRunning(LUMICE_Server* server, bool publish_reset) {
  if (!server) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_.load() == State::kRunning) {
      return false;  // Already running — zero overhead
    }
    if (state_.load() == State::kTerminating) {
      return false;
    }
    // kPaused → resume polling
    server_ = server;
    ResetPerResumeState();
    if (publish_reset) {
      PublishValidReset();
    }
    state_.store(State::kRunning);
  }
  cv_.notify_all();
  return true;
}

// task-color-migration M4: split from the former single `EnsureRunning` to give display-time
// refresh callers a wake path that does NOT publish valid=false — see doc/gui-state-
// governance.md §4 支柱 2 and the header contract on WakeForRefresh.
void ServerPoller::WakeForRestart(LUMICE_Server* server) {
  // publish_reset=true: fresh sim generation — publish valid=false so consumers ignore stale
  // stats/lifecycle until the worker republishes.
  if (TransitionToRunning(server, /*publish_reset=*/true)) {
    GUI_LOG_DEBUG("[Poller] WakeForRestart: resumed from paused (valid=false published)");
  }
}

void ServerPoller::WakeForRefresh(LUMICE_Server* server) {
  // publish_reset=false: display-time refresh — the current snapshot must stay `valid` across the
  // wake edge so SyncFromPoller does not transiently observe an invalid snapshot (which would let
  // ReconcileSimState pull a completed sim back into kSimulating — task-color-migration AC1
  // activity bug root cause (a)).
  if (TransitionToRunning(server, /*publish_reset=*/false)) {
    GUI_LOG_DEBUG("[Poller] WakeForRefresh: resumed from paused (valid preserved)");
  }
}

// Publish a valid=false copy of the current snapshot, preserving the payload (carry-forward)
// so the preview does not flicker between restart and the first new snapshot. The consumer's
// !valid early-return then ignores stale stats/lifecycle until the worker republishes.
void ServerPoller::PublishValidReset() {
  std::lock_guard<std::mutex> lk(publish_mutex_);
  auto prev = LoadPublished();
  auto next = prev ? std::make_shared<PreviewSnapshot>(*prev) : std::make_shared<PreviewSnapshot>();
  next->valid = false;
  next->has_new_texture = false;
  StorePublished(std::move(next));
}

// Single owner of the per-resume reset (see the header contract). All three fields describe "what
// has happened since the worker last resumed", so they are re-armed together at every
// kPaused→kRunning edge:
//   - last_generation_        : so the worker detects the first snapshot of the resume as new data.
//   - last_quality_pass_time_ : so the quality gate's 500ms timeout fallback doesn't fire
//                               prematurely against time that elapsed while paused.
//   - uploaded_since_resume_  : re-arms the terminal-frame rescue (I6 — see the header). Both wake
//                               variants reset it, and for the same reason: each buys the worker a
//                               poll it would otherwise never get. WakeForRestart opens a new sim
//                               generation; WakeForRefresh wakes an already-completed run for
//                               exactly ONE more poll to re-materialize it under new display state
//                               (colour edits / composite EV), after which PollOnce self-pauses
//                               again. If the gate swallows that single poll, a sparse completed
//                               run's display edits would silently never appear.
void ServerPoller::ResetPerResumeState() {
  last_generation_ = 0;
  last_quality_pass_time_ = std::chrono::steady_clock::now();
  uploaded_since_resume_ = false;
}

void ServerPoller::InvalidateStagedTexture() {
  // Suppress the last materialized-but-unuploaded texture by publishing a payload=null copy. The
  // consumer's `payload != nullptr` guard then skips upload; GL keeps showing the already-uploaded
  // frame (no black flicker). serial is left unchanged so a genuinely new future texture still gets
  // a fresh serial and uploads. Called from production (DoOpen/DoNew document-switch fencing) and
  // from tests (see header).
  std::lock_guard<std::mutex> lk(publish_mutex_);
  auto prev = LoadPublished();
  if (!prev) {
    return;
  }
  auto next = std::make_shared<PreviewSnapshot>(*prev);
  next->payload.reset();
  next->has_new_texture = false;
  StorePublished(std::move(next));
}

void ServerPoller::SetCalibratedThreshold(unsigned long long threshold) {
  calibrated_min_rays_ = threshold;
  calibrated_ = true;
  GUI_LOG_INFO("[Poller] calibration set: threshold={}", threshold);
}

// Three cadences, not two (invariant I3, doc/gui-preview-lifecycle-architecture.md §9):
//   kRunning        — poll every kPollIntervalMs.
//   kIdleHeartbeat  — poll every kIdleHeartbeatIntervalMs. The run is believed over and drained;
//                     the heartbeat is the second chance that makes that a BELIEF rather than a
//                     one-shot verdict. I3's own words: idle may throttle down, but must not fall
//                     so silent it cannot self-heal.
//   kPaused         — untimed wait, zero polling. Only Stop() puts the worker here, and its
//                     callers rely on that to destroy the server.
//
// Why the heartbeat is not simply "sleep longer in the kRunning loop": the wake entry points
// (WakeForRestart / WakeForRefresh / Start) and Stop() all key off state_, and a poller that
// stayed kRunning after completing would tell every one of them the wrong thing.
void ServerPoller::WorkerLoop() {
  while (true) {
    {
      std::unique_lock<std::mutex> lk(mutex_);
      // Wait-for-work loop. Re-entered after every heartbeat tick, which is why it re-publishes
      // active_ = false + notify each time round: a Stop() that arrives during a tick blocks on
      // active_ and must be released the moment the tick finishes.
      for (;;) {
        active_ = false;
        cv_.notify_all();  // Signal Stop()/dtor that the worker is not inside PollOnce()
        const State s = state_.load();
        if (s == State::kRunning || s == State::kTerminating) {
          break;
        }
        if (s == State::kIdleHeartbeat) {
          // Returns true iff the predicate held — i.e. someone moved us out of kIdleHeartbeat
          // (Stop / wake / dtor). false means the timer expired: that is a heartbeat tick.
          const bool state_changed = cv_.wait_for(lk, std::chrono::milliseconds(gui::kIdleHeartbeatIntervalMs),
                                                  [this] { return state_.load() != State::kIdleHeartbeat; });
          if (state_changed) {
            continue;  // re-dispatch on the new state at the top
          }
          // active_ is published under the same lock that wait_for reacquired, so a concurrent
          // Stop() either got here first (and the predicate above would have been true) or blocks
          // on active_ until this tick completes. There is no window in between.
          active_ = true;
          lk.unlock();
          heartbeat_tick_count_.fetch_add(1, std::memory_order_relaxed);
          PollOnce();
          lk.lock();
          continue;
        }
        // kPaused — explicit stop. Untimed wait; unchanged from before the heartbeat existed.
        cv_.wait(lk, [this] { return state_.load() != State::kPaused; });
      }
      if (state_.load() == State::kTerminating) {
        return;
      }
      active_ = true;
    }

    // Inner poll loop — runs while kRunning. Exits on Stop() (kPaused), on the dtor
    // (kTerminating), and on PollOnce()'s own self-pause (kIdleHeartbeat).
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

// Invariant I3a. See the header for why each of the three terms is load-bearing and why the last
// two are not the same test twice. Kept free of every member so the decision is exercisable as a
// truth table (test_lifecycle.cpp, self_pause_predicate_truth_table) rather than only through a
// live server plus a worker thread plus a race.
bool ShouldSelfPause(const LUMICE_SimLifecycleResult& lc, const LUMICE_DrainResult& drain) {
  if (lc.lifecycle != LUMICE_LIFECYCLE_COMPLETED) {
    return false;
  }
  return drain.drained_epoch == lc.epoch && drain.current_epoch == lc.epoch;
}

// See doc/accumulator-consumer-architecture.md §8.1 (polling contract), §8.3 (data flow).
//
// Reads the two server clocks (lifecycle heartbeat, raw XYZ) then builds ONE coherent immutable
// PreviewSnapshot and atomically publishes it (invariant I5). Building the candidate payload moves
// no pixels at all (it hands the payload a share of the frame), and it happens OUTSIDE
// publish_mutex_ anyway; only the pointer-level RMW (load prev → decide carry-forward → store) runs
// inside the lock (no TOCTOU). Replaces the old data_mutex_/staged_/std::swap incremental-write +
// torn-read channel.
void ServerPoller::PollOnce() {
  auto* server = server_;
  if (!server) {
    return;
  }

  // Cheap O(1) lifecycle + epoch heartbeat (clock ④, invariant I4). Read on EVERY poll, decoupled
  // from expensive snapshot materialization, so the terminal completion edge is never lost. This is
  // both the completion signal published to the consumer and the self-pause signal below (1.5
  // dropped the QueryServerState + has_valid_data side-channels this replaced).
  LUMICE_SimLifecycleResult lc{};
  LUMICE_GetSimLifecycle(server, &lc);

  // Probe, not a recovery branch (see heartbeat_unreachable_logged_ in the header). If this ever
  // fires it means the lifecycle left COMPLETED without anything waking the poller, and the right
  // response is to find out how — not to have a self-promotion branch quietly absorb it. Note the
  // heartbeat needs no such branch to keep the DISPLAY correct: it observes and publishes
  // unconditionally, so convergence never depended on being told. Only the poll RATE would be
  // affected, and a wrong rate is visible, benign, and healed by the next wake of any kind.
  if (state_.load() == State::kIdleHeartbeat) {
    if (lc.lifecycle != LUMICE_LIFECYCLE_COMPLETED) {
      if (!heartbeat_unreachable_logged_) {
        heartbeat_unreachable_logged_ = true;
        GUI_LOG_WARNING(
            "[Poller] heartbeat observed a lifecycle believed unreachable here: "
            "lifecycle={} epoch={} (expected COMPLETED). Polling stays at the heartbeat "
            "rate until the next wake.",
            lc.lifecycle, lc.epoch);
      }
    } else {
      heartbeat_unreachable_logged_ = false;  // re-arm: the next episode logs again
    }
  }

  // One frame per poll, held at least to the end of this function. Everything read out of it —
  // xyz, composite, stats — belongs to the same snapshot generation by construction, which is
  // what retired the earlier three-call sequence (GetRawXyz → GetComposite → GetRawXyz
  // recheck) and its drift-guard: the ~ms window between calls 1 and 3 was near-guaranteed
  // to be crossed by ConsumeData batch churn under an active sim, dropping composites every
  // poll until the sim stopped.
  //
  // The frame is also what makes the pointers below safe to read: they stay valid until this
  // frame is released, no matter how many snapshots the server publishes meanwhile. That is why
  // it is a shared_ptr rather than a unique_ptr — when this poll materializes a texture, the
  // payload takes a share of it and the frame outlives this call for as long as the payload is
  // on screen, which is what lets the payload point straight at these buffers instead of copying
  // them. On every other path the local share is the only one and the frame is released here.
  LUMICE_ResultFrame* raw_frame = nullptr;
  if (LUMICE_AcquireResultFrame(server, &raw_frame) != LUMICE_OK) {
    return;
  }
  std::shared_ptr<LUMICE_ResultFrame> frame(raw_frame, &LUMICE_ReleaseResultFrame);

  LUMICE_RawXyzResult xyz_results[2]{};
  LUMICE_RenderResult composite_results[2]{};
  LUMICE_FrameGetRawXyz(frame.get(), xyz_results, 1);
  LUMICE_FrameGetComposite(frame.get(), composite_results, 1);
  const unsigned long long captured_xyz_generation = xyz_results[0].snapshot_generation;

  // Check if this is genuinely new snapshot data (generation changed)
  bool has_new_snapshot = xyz_results[0].xyz_buffer != nullptr && captured_xyz_generation != last_generation_;

  // Terminal-frame rescue (invariant I6, "the final frame always reaches the screen" — the
  // blueprint's §7 rule 2 / §9; see doc/gui-preview-lifecycle-architecture.md). The quality gate below
  // suppresses sparse snapshots on the premise that a denser one is still coming; once the run has
  // COMPLETED that premise is false, and the sparse result IS the result. The gate's 500ms timeout
  // fallback cannot cover this: the self-pause at the bottom of this very function fires in the
  // SAME call, so a finite low-ray run (~80ms) is never polled again and the fallback is never
  // reached. Startup calibration sets the threshold to ~3-4×10^4 rays in the real GUI, so without
  // this any run configured below that showed a permanently blank preview.
  //
  // Deliberately evaluated OUTSIDE the has_new_snapshot branch: the lifecycle heartbeat and the
  // snapshot generation are decoupled clocks (§6), so COMPLETED may just as well arrive on a poll
  // that carries no new generation. Gating the rescue on has_new_snapshot would fix only the
  // interleaving that happened to be reproduced and leave the other one live.
  const bool force_final_upload =
      lc.lifecycle == LUMICE_LIFECYCLE_COMPLETED && !uploaded_since_resume_ && xyz_results[0].xyz_buffer != nullptr;

  // ---- Prepare candidate stats + texture payload OUTSIDE publish_mutex_ (no prev dependency).
  bool have_new_stats = false;
  LUMICE_RayCount new_ray_seg = 0;
  LUMICE_RayCount new_sim_ray = 0;
  LUMICE_RayCount new_crystal = 0;
  LUMICE_RayCount new_orientation = 0;
  unsigned long long new_stats_epoch = 0;
  std::shared_ptr<const TexturePayload> new_payload;  // non-null only when a fresh texture materialized

  if (has_new_snapshot || force_final_upload) {
    // Always consume this generation (same generation data won't improve by waiting). On the
    // force_final_upload-only path this re-assigns the value already stored — idempotent.
    last_generation_ = captured_xyz_generation;

    // Get stats (used independently for status bar display).
    //
    // has_valid_data is load-bearing here, not belt-and-braces: a restart does NOT clear the
    // stats a frame carries (ServerImpl::Stop resets snapshot_dirty_ + has_ever_consumed_ and
    // leaves the published frame alone — only a new snapshot replaces it), so between a commit
    // and the new run's first produced batch these are the PREVIOUS run's numbers with a
    // perfectly healthy-looking sim_ray_num > 0. has_valid_data mirrors has_ever_consumed_,
    // which the restart did reset, and is the only field here that tells the two apart.
    //
    // Reading the stats off the SAME frame as the xyz above removes what used to be a window:
    // these two reads can no longer straddle a snapshot, so "has_valid_data true" and "these
    // stats belong to that generation" are now one statement rather than an ordering argument.
    LUMICE_StatsResult cached_stats{};
    LUMICE_FrameGetStats(frame.get(), &cached_stats);
    if (cached_stats.sim_ray_num > 0 && xyz_results[0].has_valid_data) {
      have_new_stats = true;
      new_ray_seg = cached_stats.ray_seg_num;
      new_sim_ray = cached_stats.sim_ray_num;
      new_crystal = cached_stats.crystal_num;
      new_orientation = cached_stats.orientation_num;
      // Same read window as payload_epoch takes for the texture (see the quality_ok block below).
      new_stats_epoch = xyz_results[0].epoch;
    }

    // Quality gate: skip texture overwrite for sparse snapshots (too few rays = visible flicker).
    // Cold start (sim_ray_num == 0) is allowed through — no "old good texture" to preserve.
    unsigned long long min_rays = calibrated_ ? calibrated_min_rays_ : gui::kMinRaysFloor;
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

    // Third and last way past the gate: the run is over and this resume has yet to put anything on
    // screen (I6). Kept separate from the timeout above — that one rescues a RUNNING sim that will
    // never reach the threshold; this one rescues a run that already ended below it.
    if (!quality_ok && force_final_upload) {
      quality_ok = true;
      GUI_LOG_VERBOSE("[Poller] terminal frame: bypassing quality gate (rays={}, min={}) gen={}",
                      cached_stats.sim_ray_num, min_rays, xyz_results[0].snapshot_generation);
    }

    // Content-freshness gate: does the image about to be stamped with xyz_results[0].epoch actually
    // belong to that generation? Between a commit and the new run's first produced batch it does not
    // — the pixel buffer still holds the PREVIOUS run's image while RawXyzResult::epoch already reads
    // the bumped committed_epoch_, and the upload gate downstream can only compare epoch NUMBERS, so
    // a payload wearing the new number is indistinguishable there from a genuinely fresh frame. That
    // is why the decision has to be made here.
    //
    // The full mechanism (why the buffer survives a restart, why the floor cannot see it, and what
    // this gate restores) is written up once in doc/gui-preview-lifecycle-architecture.md §9, in the
    // "I1 的前提「世代号是诚实的」曾不成立" patch note — that section is the authority; keep the
    // reasoning there rather than growing a second copy here.
    //
    // What is NOT in the doc, because it is about this code's shape rather than the invariant:
    //
    // has_valid_data alone is too coarse to be the answer, even though it is exactly right for the
    // stats read above. It mirrors has_ever_consumed_, which Stop() clears, so it goes false for two
    // very different reasons: "a new generation was committed and has produced nothing yet" (stale —
    // suppress) and "the user pressed Stop, and the frame on hand is still this generation's"
    // (current — a display-time edit must still be able to repaint it). Gating on it alone silently
    // breaks the second, which is a live path, not a corner: DoStop → recolor → WakeForRefresh.
    //
    // So compare generations instead. While has_valid_data is false the buffer is frozen — only
    // ConsumeData can dirty a snapshot and it sets has_ever_consumed_ in the same critical section —
    // so the generation the pixels were produced under is the one the last materialized payload
    // recorded, and prev->payload->payload_epoch is that record. Equal to the live epoch means no
    // commit has happened since the pixels were made.
    //
    // Deliberately applied AFTER both bypasses above rather than folded into quality_ok's initial
    // value: the timeout fallback rescues a run producing too sparsely, the terminal-frame rescue a
    // run that ended below threshold — neither is a licence to publish another generation's pixels,
    // so neither may override this. I6 is unaffected in substance anyway: force_final_upload requires
    // lifecycle == COMPLETED, which the server derives from has_ever_consumed_, so a terminal frame
    // always has valid data and takes the first disjunct.
    //
    // The carry-forward below is what makes suppression the right answer rather than a black screen:
    // with no new payload the previous frame stays on screen under its OWN epoch (§5.4 anti-flicker),
    // which is the honest rendering of "this generation has produced nothing yet".
    //
    // Known narrow gap, accepted: if a generation produces data and is stopped without the poller
    // ever polling in between, no payload recorded its epoch, so its frame is suppressed and the
    // previous one stays up. Closing it needs the SERVER to stamp the generation it filled
    // snapshot_xyz_ under (a new C-API field) — no poller-side state can, since the poller by
    // definition never observed that generation. Left alone deliberately: that stamp is a wider
    // change than this defect warrants.
    if (quality_ok && !xyz_results[0].has_valid_data) {
      auto prev_published = LoadPublished();
      const bool content_belongs_to_this_epoch =
          prev_published && prev_published->payload && prev_published->payload->payload_epoch == xyz_results[0].epoch;
      if (!content_belongs_to_this_epoch) {
        quality_ok = false;
        // epoch and snapshot_generation are two INDEPENDENT server counters — epoch is what this gate
        // compares, snapshot_generation is what has_new_snapshot above compares. Spelled out in full
        // so a log line never reads as though they were the same number.
        GUI_LOG_VERBOSE(
            "[Poller] stale content: nothing produced under epoch {} yet, carrying previous frame "
            "(snapshot_generation={})",
            xyz_results[0].epoch, xyz_results[0].snapshot_generation);
      }
    }

    if (quality_ok) {
      last_quality_pass_time_ = now;
      auto payload = std::make_shared<TexturePayload>();
      payload->frame = frame;  // share the frame's lifetime — no pixel copy
      payload->xyz_buffer = xyz_results[0].xyz_buffer;
      payload->width = xyz_results[0].img_width;
      payload->height = xyz_results[0].img_height;
      payload->snapshot_intensity = xyz_results[0].snapshot_intensity;
      payload->intensity_factor = xyz_results[0].intensity_factor;
      payload->effective_pixels = xyz_results[0].effective_pixels;
      payload->texture_ray_count = cached_stats.sim_ray_num;
      payload->payload_epoch = xyz_results[0].epoch;

      // task-345.2: composite already fetched by the atomic combined call above (see xyz block).
      // The img_buffer sentinel (NULL when no raypath_color is configured; non-NULL when there is
      // at least one colored consumer with a composite this generation) IS the raypath-color-
      // active detector — no cross-thread state needs to be threaded through from the GUI.
      if (composite_results[0].img_buffer != nullptr) {
        // Same-generation guarantee is structural: this composite came out of the same frame as
        // the xyz above, so it belongs to the same snapshot_generation by construction. Cross-
        // generation pairing is not a window that can be missed, it is impossible upstream —
        // which is what eliminated the old "recheck → mismatch → drop" drift-guard branch and
        // the `GUI_LOG_VERBOSE("[Poller] composite generation drift ...")` line it printed.
        // payload->frame is already this frame (assigned with xyz_buffer above); the two views
        // share that one share rather than taking a second.
        payload->rgb_buffer = composite_results[0].img_buffer;
        payload->is_composite = true;
      }
      // task-345.3: composite P99 anchor comes from the server (union of participating
      // classes' unexposed lanes — see doc/ev-pipeline-architecture.md §2.4 for the
      // "P99 is composite-only C API field" carve-out). Mono/non-composite path stays on
      // the client-side xyz statistic; the two paths do NOT converge — mixing full-
      // spectrum pixels back in was the "composite too dim" root cause this task fixes.
      if (payload->is_composite) {
        payload->p99_y = composite_results[0].composite_p99_y;
      } else {
        payload->p99_y = ComputeP99Y(payload->xyz_buffer, payload->width, payload->height, kEvAutoDownsampleFactor);
      }
      // (payload is default-constructed with rgb_buffer null + is_composite=false,
      // so the not-active branch is a no-op; explicit reset would be redundant.)
      new_payload = std::move(payload);
      // A frame exists for this resume — disarm the terminal rescue so a later COMPLETED poll
      // cannot overwrite it with a sparser one (see the header: this is the anti-flicker guard).
      uploaded_since_resume_ = true;
      GUI_LOG_VERBOSE("[Poller] staged: rays={} intensity={} gen={}", cached_stats.sim_ray_num,
                      xyz_results[0].snapshot_intensity, xyz_results[0].snapshot_generation);
    } else {
      GUI_LOG_VERBOSE("[Poller] quality gate: skipped rays={} (min={}) gen={}", cached_stats.sim_ray_num, min_rays,
                      xyz_results[0].snapshot_generation);
    }
  }

  // ---- Publish: whole RMW (load prev → decide carry-forward → store) inside publish_mutex_.
  // Critical section is pointer/refcount-level only.
  {
    std::lock_guard<std::mutex> lk(publish_mutex_);
    auto prev = LoadPublished();
    auto next = std::make_shared<PreviewSnapshot>();
    next->valid = true;
    next->epoch = lc.epoch;
    // Lifecycle level signal (clock ④ / I4): carried on every poll.
    next->lifecycle = lc.lifecycle;
    // Stats: fresh value if this generation produced one; else carry forward prev's (coherent
    // bundle — no torn zero). The stats' own generation stamp travels WITH them in both branches:
    // freshly read stats are stamped with the generation that produced them, and a carry-forward
    // keeps the stamp it already had rather than being re-stamped with lc.epoch like the bundle
    // epoch above. That asymmetry is the whole point — it is what lets the consumer tell "this
    // poll produced no new stats, here are the current run's last known ones" apart from "this
    // poll produced no new stats because the run just restarted, and these belong to the old one".
    if (have_new_stats) {
      next->stats_ray_seg_num = new_ray_seg;
      next->stats_sim_ray_num = new_sim_ray;
      next->stats_crystal_num = new_crystal;
      next->stats_orientation_num = new_orientation;
      next->stats_epoch = new_stats_epoch;
    } else if (prev) {
      next->stats_ray_seg_num = prev->stats_ray_seg_num;
      next->stats_sim_ray_num = prev->stats_sim_ray_num;
      next->stats_crystal_num = prev->stats_crystal_num;
      next->stats_orientation_num = prev->stats_orientation_num;
      next->stats_epoch = prev->stats_epoch;
    }
    // Texture: a freshly materialized payload gets a new monotonic serial; otherwise carry the
    // previous payload pointer + serial forward (sparse / gate-rejected / no-new-generation) so
    // the consumer's serial dedup keeps the last frame on screen (anti-flicker, §5.4).
    if (new_payload) {
      next->payload = std::move(new_payload);
      next->texture_serial = texture_serial_.fetch_add(1) + 1;
      next->has_new_texture = true;
    } else if (prev) {
      next->payload = prev->payload;
      next->texture_serial = prev->texture_serial;
      next->has_new_texture = false;
    }
    StorePublished(std::move(next));
  }

  // Throttle down to the slow heartbeat once the run has genuinely completed AND its data has been
  // drained (invariant I3a — see ShouldSelfPause for the three terms). Two things changed here
  // relative to the original "COMPLETED ⇒ kPaused":
  //
  //  - The condition asks the consumer, not just the producer. It used to reason that the last
  //    published snapshot carries lifecycle==COMPLETED, so the main thread's reconcile still
  //    reaches kDone. That argument is sound — but only for the sim_state projection. The same
  //    bundle also carries the four stats counters, and those are carried FORWARD from the
  //    previous poll when this one produced nothing new, so a poll that observes COMPLETED before
  //    the consumer has drained publishes a partial total and then stops, leaving the status bar
  //    permanently short by whole dispatch grains. "May I stop looking?" has to mean "is what I
  //    can see now the truth?" for every field in the bundle, not just the one that motivated the
  //    guard.
  //  - The destination is kIdleHeartbeat, not kPaused. Even a correct guard is a single
  //    observation; the heartbeat is what keeps a wrong one recoverable (I3b).
  //
  // The kRunning re-check inside the lock is not optional garnish: a concurrent Stop() may have
  // just written kPaused (its caller is about to destroy the server) or a WakeForRestart may have
  // written kRunning for a newer epoch. Both write under this same mutex_, so re-reading here
  // makes the last writer win deterministically instead of this line clobbering either.
  if (lc.lifecycle == LUMICE_LIFECYCLE_COMPLETED) {
    LUMICE_DrainResult drain{};
    LUMICE_GetDrainStatus(server, &drain);
    if (ShouldSelfPause(lc, drain)) {
      std::lock_guard<std::mutex> lk(mutex_);
      if (state_.load() == State::kRunning) {
        state_.store(State::kIdleHeartbeat);
      }
    }
  }
}

}  // namespace lumice::gui
