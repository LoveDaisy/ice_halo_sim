#include "gui/server_poller.hpp"

#include <chrono>
#include <cstring>

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
    PublishValidReset();
    state_.store(State::kRunning);
  }
  cv_.notify_all();
  GUI_LOG_DEBUG("[Poller] EnsureRunning: resumed from paused");
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

// Composite width/height come from composite_result itself (not xyz's) — they must match by
// construction (same RenderConsumer), but taking the source's own dims makes the memcpy bounds
// self-consistent.
//
// Post-345.2: the drift-guard (regen_check + drop-and-clear path) is GONE. The caller now sources
// xyz + composite from a single LUMICE_GetRawXyzAndCompositeResults() call — one server-side
// DoSnapshot() with a Phase-2 rebuild inside — so composite is structurally guaranteed to belong
// to the same snapshot_generation as its paired xyz. Cross-generation pairing is no longer a
// window that can be missed; it is impossible upstream. That eliminates the entire "recheck →
// mismatch → drop" branch and the `GUI_LOG_VERBOSE("[Poller] composite generation drift ...")` log
// line the plan's AC5 tracked. See scratchpad/scrum-raypath-color-gui-polish/task-fix-live-color-refresh/plan.md §3.
void ServerPoller::PopulateCompositePayload(const LUMICE_RenderResult& composite_result, TexturePayload* payload) {
  const size_t rgb_bytes =
      static_cast<size_t>(composite_result.img_width) * static_cast<size_t>(composite_result.img_height) * 3;
  payload->rgb_data.resize(rgb_bytes);
  std::memcpy(payload->rgb_data.data(), composite_result.img_buffer, rgb_bytes);
  payload->is_composite = true;
}

// See doc/accumulator-consumer-architecture.md §8.1 (polling contract), §8.3 (data flow).
//
// Reads the two server clocks (lifecycle heartbeat, raw XYZ) then builds ONE coherent immutable
// PreviewSnapshot and atomically publishes it (invariant I5). The expensive candidate texture
// memcpy is prepared OUTSIDE publish_mutex_; only the pointer-level RMW (load prev → decide
// carry-forward → store) runs inside the lock (no TOCTOU, no 24MB copy under lock). Replaces the
// old data_mutex_/staged_/std::swap incremental-write + torn-read channel.
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

  // task-345.2: atomic combined C-API — single server-side DoSnapshot() plus one snapshot_mutex_
  // critical section produces xyz + composite that share the same snapshot_generation by
  // construction (④ drift-guard root cause fix). Replaces the pre-345.2 three-call sequence
  // (LUMICE_GetRawXyzResults → LUMICE_GetCompositeResults → LUMICE_GetRawXyzResults recheck)
  // whose ~ms window between call#1 and call#3 was near-guaranteed to be crossed by
  // ConsumeData batch churn under an active sim, dropping composites every poll until sim
  // stopped. See scratchpad/scrum-raypath-color-gui-polish/task-fix-live-color-refresh/plan.md §3.
  LUMICE_RawXyzResult xyz_results[2]{};
  LUMICE_RenderResult composite_results[2]{};
  LUMICE_GetRawXyzAndCompositeResults(server, xyz_results, 1, composite_results, 1);
  const unsigned long long captured_xyz_generation = xyz_results[0].snapshot_generation;

  // Check if this is genuinely new snapshot data (generation changed)
  bool has_new_snapshot = xyz_results[0].xyz_buffer != nullptr && captured_xyz_generation != last_generation_;

  // ---- Prepare candidate stats + texture payload OUTSIDE publish_mutex_ (no prev dependency).
  // The 24MB pixel memcpy happens here, never inside the publish critical section.
  bool have_new_stats = false;
  LUMICE_RayCount new_ray_seg = 0;
  LUMICE_RayCount new_sim_ray = 0;
  std::shared_ptr<const TexturePayload> new_payload;  // non-null only when a fresh texture materialized

  if (has_new_snapshot) {
    // Always consume this generation (same generation data won't improve by waiting)
    last_generation_ = captured_xyz_generation;

    // Get stats (used independently for status bar display)
    LUMICE_StatsResult cached_stats{};
    LUMICE_GetCachedStats(server, &cached_stats);
    if (cached_stats.sim_ray_num > 0) {
      have_new_stats = true;
      new_ray_seg = cached_stats.ray_seg_num;
      new_sim_ray = cached_stats.sim_ray_num;
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

    if (quality_ok) {
      last_quality_pass_time_ = now;
      auto payload = std::make_shared<TexturePayload>();
      size_t float_count = static_cast<size_t>(xyz_results[0].img_width) * xyz_results[0].img_height * 3;
      payload->xyz_data.resize(float_count);
      // Copy xyz bytes BEFORE any further Get*Results call (see GetCompositeResults()
      // call below): snapshot_xyz_ is a persistent per-RenderConsumer buffer that
      // PrepareSnapshot() overwrites IN PLACE (not reallocated) — xyz_results[0].xyz_buffer
      // aliases it, so a Get*Results call made after capturing this pointer but before this
      // memcpy could silently rewrite the memory out from under us if it consumes a
      // newly-landed dirty event. Copying first eliminates that window entirely rather than
      // trying to detect it after the fact.
      std::memcpy(payload->xyz_data.data(), xyz_results[0].xyz_buffer, float_count * sizeof(float));
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
        // Same-generation guarantee is structural (from LUMICE_GetRawXyzAndCompositeResults);
        // no host-side drift-check needed anymore. See PopulateCompositePayload's doc comment.
        PopulateCompositePayload(composite_results[0], payload.get());
      }
      // task-345.3: composite P99 anchor comes from the server (union of participating
      // classes' unexposed lanes — see doc/ev-pipeline-architecture.md §2.4 for the
      // "P99 is composite-only C API field" carve-out). Mono/non-composite path stays on
      // the client-side xyz_data statistic; the two paths do NOT converge — mixing full-
      // spectrum pixels back in was the "composite too dim" root cause this task fixes.
      if (payload->is_composite) {
        payload->p99_y = composite_results[0].composite_p99_y;
      } else {
        payload->p99_y = ComputeP99Y(payload->xyz_data, payload->width, payload->height, kEvAutoDownsampleFactor);
      }
      // (payload is default-constructed with rgb_data empty + is_composite=false,
      // so the not-active branch is a no-op; explicit reset would be redundant.)
      new_payload = std::move(payload);
      GUI_LOG_VERBOSE("[Poller] staged: rays={} intensity={} gen={}", cached_stats.sim_ray_num,
                      xyz_results[0].snapshot_intensity, xyz_results[0].snapshot_generation);
    } else {
      GUI_LOG_VERBOSE("[Poller] quality gate: skipped rays={} (min={}) gen={}", cached_stats.sim_ray_num, min_rays,
                      xyz_results[0].snapshot_generation);
    }
  }

  // ---- Publish: whole RMW (load prev → decide carry-forward → store) inside publish_mutex_.
  // Critical section is pointer/refcount-level only (the pixel memcpy already happened above).
  {
    std::lock_guard<std::mutex> lk(publish_mutex_);
    auto prev = LoadPublished();
    auto next = std::make_shared<PreviewSnapshot>();
    next->valid = true;
    next->epoch = lc.epoch;
    // Lifecycle level signal (clock ④ / I4): carried on every poll.
    next->lifecycle = lc.lifecycle;
    // Stats: fresh value if this generation produced one; else carry forward prev's (coherent
    // bundle — no torn zero). The consumer applies stats only when >0, so this is behavior-
    // equivalent to the old swap-to-0-then-skip, and is a positive I5 side effect.
    if (have_new_stats) {
      next->stats_ray_seg_num = new_ray_seg;
      next->stats_sim_ray_num = new_sim_ray;
    } else if (prev) {
      next->stats_ray_seg_num = prev->stats_ray_seg_num;
      next->stats_sim_ray_num = prev->stats_sim_ray_num;
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

  // Self-pause once the run has genuinely completed (finite run drained clean, incl. zero-output).
  // COMPLETED is the durable terminal edge (infinite runs stay RUNNING; a transient restart IDLE is
  // NOT COMPLETED), so this never pauses mid-run. The last published snapshot before pausing carries
  // lifecycle==COMPLETED, so the main thread's per-frame reconcile still reaches kDone (I3).
  if (lc.lifecycle == LUMICE_LIFECYCLE_COMPLETED) {
    std::lock_guard<std::mutex> lk(mutex_);
    state_.store(State::kPaused);
  }
}

}  // namespace lumice::gui
