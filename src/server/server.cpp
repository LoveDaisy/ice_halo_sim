#include "server/server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/color_gate_table.hpp"
#include "config/config_manager.hpp"
#include "config/raypath_color_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/def.hpp"
#include "core/simulator.hpp"
#if defined(LUMICE_CUDA_ENABLED)
#include "core/backend/cuda_trace_backend.hpp"  // CudaDeviceAvailable() for ResolveGpuRoute
#endif
#include "server/anchor_consumer.hpp"
#include "server/component_compositor.hpp"
#include "server/consumer.hpp"
#include "server/ray_num_semantics.hpp"
#include "server/raypath_histogram_consumer.hpp"
#include "server/render.hpp"
#include "server/scene_batch_publish.hpp"
#include "server/server.hpp"
#include "server/stats.hpp"
#include "util/color_space.hpp"
#include "util/contrast_headroom.hpp"
#include "util/cpu_info.hpp"
#include "util/env_knobs.hpp"
#include "util/logger.hpp"
#include "util/queue.hpp"

namespace lumice {

// =============== TicketMutex ===============
// A fair (FIFO) mutex that prevents starvation. On Windows, std::mutex uses SRWLOCK
// which doesn't guarantee fairness — a high-frequency locker (ConsumeData) can starve
// a low-frequency waiter (Poller) indefinitely. TicketMutex guarantees FIFO ordering:
// each waiter gets a ticket and is served in order.
// See doc/accumulator-consumer-architecture.md §4.1.
class TicketMutex {
 public:
  void lock() {  // NOLINT(readability-identifier-naming) — C++ Lockable requires lowercase
    auto ticket = next_ticket_.fetch_add(1, std::memory_order_relaxed);
    while (now_serving_.load(std::memory_order_acquire) != ticket) {
      std::this_thread::yield();
    }
  }

  void unlock() {  // NOLINT(readability-identifier-naming)
    now_serving_.fetch_add(1, std::memory_order_release);
  }

 private:
  std::atomic<uint32_t> next_ticket_{ 0 };
  std::atomic<uint32_t> now_serving_{ 0 };
};

// =============== ServerImpl ===============
class ServerImpl {
 public:
  explicit ServerImpl(int num_workers = 0, uint32_t sim_seed = 0, BackendKind preferred_backend = BackendKind::kCpu);
  ~ServerImpl();

  Error CommitConfig(const nlohmann::json& config_json, bool* out_reused = nullptr);
  // The analysis run (see Server::StartRaypathAnalysis for the contract). Same
  // Stop → rebuild consumers → Start shape as CommitConfig, on the scene `scene_json`
  // carries — its own submission, not the last render commit's.
  Error StartRaypathAnalysis(const nlohmann::json& scene_json, const RaypathAnalysisRequest& request);
  // See Server::GetActiveBackend. Structural kCpu in an analysis session; the
  // Simulator's own published answer otherwise.
  BackendKind GetActiveBackend() const;
  size_t GetLiveSimRayCount();

  // THE result entry point. Materializes a snapshot if one is pending, then
  // returns a share of the published frame — the caller's pointers stay valid for as long
  // as it keeps that share, independently of every later snapshot. Never returns null.
  std::shared_ptr<const ResultFrame> AcquireResultFrame();

  void Stop();
  void Start();
  ServerStatus GetStatus() const;
  SimLifecycle GetSimLifecycle() const;
  // See Server::GetSessionKind. A plain acquire load of mode_ — the same read the
  // CommitConfig reuse decision performs, so a caller predicting that decision reads
  // exactly what it will read.
  SessionKind GetSessionKind() const;
  uint64_t CommittedEpoch() const;
  uint64_t DrainedEpoch() const;
  bool IsIdle();
  void SetPreferredBackend(BackendKind backend);

  // task-342.2: display-time update of the committed color classes without restarting the
  // simulation. `class_count` must equal the currently active class count (mismatch =
  // InvalidConfig: caller must re-commit). `z_order` is optional (nullptr = leave unchanged);
  // when non-null, z_order must be a permutation of [0, class_count) where z_order[i] is the
  // new drawing rank of class i (sorted ascending by the compositor). Runs under consumer_mutex_ only — never touches
  // Stop/Start/scene_generation_/committed_epoch_/consumers_/scene_mutex_.
  Error SetRaypathColors(const ColorClassDisplay* classes, int class_count, const int* z_order, CompositeMode mode);

  // task-345.3: display-time update of the composite-path EV multiplier.
  // `ev_total` is applied as `2^ev_total` inside DoSnapshot Phase 2 (a single
  // scalar shared across every lane / every mode — per-lane renormalization
  // remains structurally excluded). No sim restart, no epoch bump: flips
  // snapshot_dirty_ so the next acquired result frame triggers one composite rebake
  // with the new EV. Mono path is untouched (structural AC4).
  Error SetCompositeExposure(float ev_total);

  // Display-time update of the composite-path background colour. `rgb` is ADDITIVE linear RGB (3 floats), added inside
  // DoSnapshot Phase 2 to every pixel the lens actually images. No sim restart,
  // no epoch bump: flips snapshot_dirty_ so the next acquired result frame
  // triggers one composite rebake with the new background. Mono path is
  // untouched (it takes its background from the committed RenderConfig).
  Error SetCompositeBackground(const float rgb[3]);

  // task-342.3 AC4: per-color-class empty-arc detector. Reads the frozen snapshot
  // lanes (no DoSnapshot trigger — the GUI polling loop is expected to have already
  // acquired a result frame). Writes 1 into out_flags[i] when
  // any RenderConsumer has a non-zero pixel in class i's snapshot Y-lane; 0
  // otherwise. class_count must equal the active color-class count.
  Error GetColorClassSignals(uint8_t* out_flags, int class_count);

  // task-gui-feedback-affordances Step 5 (AC1): synchronous accessor for the
  // component-bit overflow count captured in the most recent CommitConfig.
  size_t GetLastColorComponentOverflowCount() const {
    return last_color_component_overflow_count_.load(std::memory_order_acquire);
  }

  // task-color-degrade-gui-surfacing: accessor for the GPU color-degrade tally
  // (symmetry-group / OR-summand / color-class caps). Unlike the component count
  // above (set synchronously in CommitConfig), these are populated ASYNCHRONOUSLY
  // from the worker's first batch via ConsumeData, so the GUI must poll for them.
  ColorDegradeCounts GetLastColorDegradeCounts() const {
    return { last_color_symmetry_group_overflow_.load(std::memory_order_acquire),
             last_color_or_summand_overflow_.load(std::memory_order_acquire),
             last_color_class_overflow_.load(std::memory_order_acquire) };
  }

  // Has this server's GPU single-engine route lost its TraceBackend for
  // the remainder of the current Run()? (a BackendUnavailableError, or a
  // CreateBackend that could not honour the preference at all.) Always false on the
  // CPU route, where there is no backend to lose. Like the color-degrade tally above
  // this is an ASYNCHRONOUS fact discovered by the worker mid-run, so the GUI polls
  // it (LUMICE_GetBackendFallbackFlag) instead of the fallback living only as a core
  // WARN line no GUI user ever sees. Cheap: one atomic load under a short mutex.
  // An analysis session is not a fallback: its CPU route is asked for, not fallen to.
  bool BackendFellBack() const {
    return gpu_route_ && mode_.load(std::memory_order_acquire) != SessionKind::kAnalysis && !ReadBackendActive();
  }

 private:
  // The one owner of "a JSON document becomes a ConfigManager, or a return code". Both
  // submission entry points — CommitConfig (a render) and StartRaypathAnalysis (an analysis)
  // — parse through here, so the four failure shapes map onto the Error vocabulary in exactly
  // one place: nlohmann::json::out_of_range → MissingField, any other json exception →
  // InvalidJson, std::exception → InvalidConfig, anything else → InvalidConfig. `validate`
  // runs inside the same try on the parsed document (CommitConfig builds its colour tables
  // there, which throw std::invalid_argument on a config error); nullptr for no extra step.
  // On failure `*out` is untouched — the parse lands in a local first and is only moved into
  // `out` once every step has passed — and so is every other member, status_ included: a
  // rejected document is a return code, not a state. (CommitConfig used to write
  // status_ = kError here. No projection ever read that value as anything but "not running",
  // and while a session's workers were still tracing it made GetSimLifecycle report the run
  // as over until the next Stop()/Start() rewrote it — a lie about a live run, and one the
  // analysis path now reaches on purpose: a rejected scene over an analysis in flight must
  // leave that analysis readable as in flight.) `caller` prefixes the log line so the two
  // entry points stay distinguishable in a log.
  Error ParseConfigManager(const nlohmann::json& config_json, const char* caller,
                           const std::function<void(const ConfigManager&)>& validate, ConfigManager* out);

  // task-268.7: single-engine orchestration — server now runs exactly one
  // Simulator. The legacy kDefaultSimulatorCnt = PhysicalCoreCount() was removed
  // along with the 12-worker queue-per-Simulator pattern; num_workers is reserved
  // and ignored. See doc/gpu-single-engine-implementation.md §6.
  static constexpr int kMaxSceneCnt = 128;
  static constexpr size_t kDefaultRayNum = 128;
  // scrum-268.6: Metal single-engine needs a large GPU dispatch to saturate the
  // device — a 128-ray dispatch starves it (~0.04x legacy), while ~32768 peaks
  // at ~5.3x legacy on heavy multi-MS+filter scenes (sweep 2026-06-16; plateau
  // beyond, GPU-bound). CPU/legacy keeps the small 128 default (multi-worker
  // geometry-sampling cadence). Commit granularity (kCommitCap) stays fine
  // regardless, so "feed the GPU big, refresh the UI small" is one tunable.
  static constexpr size_t kDefaultMetalDispatchRayNum = 32768;
  // scrum-306.2: CUDA's optimum sits much higher than Metal's. After capping the
  // dead d_exit_ buffer (cuda_trace_backend.cu kCudaDeadExitCap), a large dispatch
  // amortizes the per-batch host stall (BeginSession/XYZ-readback/sync) that left
  // the GPU ~70% idle: dev49 idle-gated interleaved sweep on cfg_50m (50M-ray
  // multi) climbs 37M @32768 -> ~114M @262144 (= 85% of the 134M intrinsic kernel
  // rate, nsys), plateauing/declining beyond (~115M @1M, then cont/root-buffer
  // pressure). 262144 is the throughput plateau at modest memory. CUDA energy is
  // dispatch-invariant (scrum-306.4), and parity holds at this dispatch (full
  // suite 10/10 @262144/524288). Kept separate from the Metal default (Metal's own
  // optimum is unchanged; not re-measured here).
  static constexpr size_t kDefaultCudaDispatchRayNum = 262144;

  // Upper bound on the AUTOMATIC worker count only (num_workers == 0). An explicit
  // num_workers > 0 is honoured verbatim, above this value included — a caller who
  // names a number has said something this constant has no standing to overrule.
  //
  // Where the number comes from, and what it is not: it is not derived from any
  // mechanism. It is the empirical lower edge of a plateau — across 2 CPUs
  // (16-core x86 and a 12-core arm64), 3 operating systems (Linux/WSL2, native
  // Windows, macOS) and 2 scene families (single- and multi-scattering), raising the
  // worker count above 10 never once produced more throughput, while running at the
  // full physical core count cost up to 33% of it on the 16-core box. The measured
  // peak sat at 10 on BOTH machines despite their different core counts, which is why
  // this is an absolute constant rather than a fraction of PhysicalCoreCount().
  //
  // Honest boundary: the sample is 2 CPUs. A machine that genuinely scales past 10
  // workers would be left throughput on the table by this default — no such machine
  // was observed, but none was ruled out either. Such a machine's escape hatch is the
  // explicit path above (CLI --workers N, or the GUI's app-level worker preference),
  // and the CLI's --benchmark mode:multi pass still reports the full-core figure, so
  // the comparison that would reveal it stays available.
  static constexpr int kMaxDefaultWorkerCount = 10;

  void ConsumeData();
  void GenerateScene();
  // Publish drained_epoch_ if the current epoch is fully consumed.
  // Called from the two places a "last transition" can happen — see its definition.
  void PublishDrainedEpochIfSettled();
  // task-342.4 Step 1: unified snapshot consumer. Returns true iff this call
  // actually consumed a dirty snapshot (Phase 1..2 executed); false if
  // snapshot_dirty_ was clear on entry (nothing to do). Merges the previously
  // duplicate Phase-1 in the raw-xyz getter into a single dirty-flag owner, so every
  // kind of result materialized in one pass is coherent with the others.
  bool DoSnapshot();

  // Publish an EMPTY frame (no results of any kind, has_valid_data_ false, the current epoch
  // and snapshot generation), for the two session switches. DoSnapshot only publishes when a
  // batch has dirtied the snapshot, so between Stop() and the new session's first batch the
  // published frame would still be the previous session's — a render's image under an
  // analysis session, an analysis histogram under a render session — re-stamped as stale but
  // readable. Every FrameGet* on the empty frame writes its sentinel instead. Called between
  // Stop() and Start() only: Stop() has joined the workers, so no DoSnapshot can be
  // publishing a real frame of the NEW session that this would overwrite; the lock is against
  // a reader's DoSnapshot still in flight from before the Stop().
  void PublishEmptyFrame();

  // Persistent thread loop: wait for Start(), run work_fn, repeat until kTerminating.
  template <typename F>
  void RunPersistentLoop(F work_fn);

  ConfigManager config_manager_;

  // task-339.3: color-class table of the currently committed config. Compared
  // structurally against the incoming table in CommitConfig (see NeedsRebuild
  // in config/color_class_table.hpp) so any change in per-class (combine_,
  // member_bits_) forces a full consumer rebuild — a reused consumer would keep
  // a stale per-class lane layout. Default construction (empty classes_) equals
  // "no raypath_color configured" (referenced_mask_ = 0).
  ColorClassTable active_class_table_;

  // task-339.4: the composite mode (dominant/additive/painter) currently in
  // effect. Parsed in CommitConfig's try block from RaypathColorConfig::mode_;
  // DoSnapshot Phase 2 pairs it with active_class_table_ to drive
  // CompositeColorClassesLinear. Defaults to kDominant (matches the JSON schema
  // default; no raypath_color → color class table is empty → composite gate
  // skips the compositor anyway).
  CompositeMode active_composite_mode_ = CompositeMode::kDominant;

  // task-gui-feedback-affordances Step 5 (AC1): number of color predicates
  // that hit `kNoBit` in the most recent BuildColorGateTable call. Carried
  // out synchronously in CommitConfig (see server.cpp:506 nearby). Exposed
  // via LUMICE_GetColorOverflowInfo so DoRun's post-commit surface can pop a
  // "coloring degraded" modal. Written under status_mutex_ (single writer =
  // CommitConfig, atomic read by the C API path is safe).
  std::atomic<size_t> last_color_component_overflow_count_{ 0 };

  // task-color-degrade-gui-surfacing: GPU-only color-degrade tally, populated
  // ASYNCHRONOUSLY from the worker's SimData in ConsumeData (generation-matched
  // branch) — the caps fire on the backend's first batch, too late for the
  // synchronous CommitConfig path above. Reset to 0 synchronously in
  // CommitConfig (so a config switch does not leave a stale non-zero value that
  // would falsely trip the GUI degrade modal before the first batch lands).
  // Read atomically by the C API poll path (LUMICE_GetColorOverflowInfo).
  std::atomic<size_t> last_color_symmetry_group_overflow_{ 0 };
  std::atomic<size_t> last_color_or_summand_overflow_{ 0 };
  std::atomic<size_t> last_color_class_overflow_{ 0 };

  // task-345.3: display-time EV for the composite path only. Zero (default) →
  // 2^0 = 1.0 → composite behavior is bit-for-bit pre-345.3 (structural AC4
  // for the CLI path, which never touches SetCompositeExposure). Written by
  // SetCompositeExposure (also flips snapshot_dirty_) and consumed by
  // DoSnapshot Phase 2's CompositeColorClassesLinear call — nowhere else.
  // Read/write both go through consumer_mutex_ so it composes with the
  // display-time class table under one lock.
  float display_ev_total_ = 0.0f;

  // Display-time ADDITIVE linear-RGB background for the composite path only. All-zero (default) → adding it is an
  // algebraic no-op → composite behavior is bit-for-bit what it was before this setter existed (the CLI path never
  // calls SetCompositeBackground). Written by SetCompositeBackground (also flips snapshot_dirty_) and consumed by
  // DoSnapshot Phase 2's ApplyCompositeBackground call — nowhere else.
  // Read/write both go through consumer_mutex_ so it composes with the
  // display-time EV and class table under one lock.
  float composite_background_linear_[3] = { 0.0f, 0.0f, 0.0f };

  QueuePtrS<SimBatch> scene_queue_;
  QueuePtrS<SimData> data_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrS> consumers_;
  mutable TicketMutex consumer_mutex_;  // FIFO lock: prevents Poller starvation on Windows
  bool snapshot_dirty_{ false };        // Set by ConsumeData, cleared by DoSnapshot
  bool has_ever_consumed_{ false };     // True after first ConsumeData; reset on Stop (new consumers have no data)
  uint64_t snapshot_generation_{
    0
  };  // Increments on each PrepareSnapshot; NOT reset on Stop (poller resets its own tracker)

  // The one published snapshot, replacing the former
  // cached_render_results_/cached_stats_result_/cached_composite_results_ trio. Those were
  // MUTABLE caches: each DoSnapshot std::move'd over them, freeing pixels a reader was
  // still pointing at (the confirmed heap-use-after-free this task removes). A frame is
  // immutable and reference-counted instead — publishing a new one cannot disturb an old
  // reader, and the old frame dies when its last holder drops it.
  //
  // INVARIANT: never null. The constructor publishes an all-zero frame, so every reader
  // dereferences unconditionally — that is what makes lumice.h's "all-zero struct if no
  // snapshot has been taken yet" promise structural rather than a per-call null branch.
  std::shared_ptr<const ResultFrame> published_frame_;

  // Published-frame access helpers. C++17 has no std::atomic<std::shared_ptr<T>>, so we use
  // the C++11 free functions std::atomic_load/atomic_store (deprecated in C++20 but valid
  // here). Same idiom as ServerPoller's published_ (src/gui/server_poller.hpp).
  // MIGRATION: when the project moves to C++20, change published_frame_ to
  // std::atomic<std::shared_ptr<const ResultFrame>> and rewrite these two helpers to use
  // published_frame_.load()/.store() — only these two functions change.
  std::shared_ptr<const ResultFrame> LoadPublished() const { return std::atomic_load(&published_frame_); }
  void StorePublished(std::shared_ptr<const ResultFrame> next) {
    std::atomic_store(&published_frame_, std::move(next));
  }

  // Serializes the whole two-phase snapshot pass. Phase 1 takes consumer_mutex_ and Phase 2
  // takes nothing, so without this lock two DoSnapshot calls could interleave: one thread's
  // PrepareSnapshot could run while another was still reading the same consumer's snapshot
  // lanes in Phase 2. That was already a torn-frame source; with Phase 1 now re-pointing the
  // consumer's buffer members (see FrameBufferPool) it would be a pointer-level race, so the
  // pass is made mutually exclusive. Outer lock of the two — always taken BEFORE
  // consumer_mutex_, never while holding it.
  std::mutex do_snapshot_mutex_;

  // Scratch for DoSnapshot Phase 2's composite pass, and used nowhere else. It was a
  // local std::vector inside the per-consumer loop, i.e. one W*H*3 float allocation and
  // free per colored consumer per snapshot, for a buffer that is filled and consumed
  // within a single loop iteration and never published.
  //
  // Reuse is safe for two reasons, both of which must hold if this ever moves:
  // do_snapshot_mutex_ above serializes whole snapshot passes, so no two writers exist
  // at once; and CompositeColorClassesLinear assign()s the full size on entry, so no
  // iteration can read what a previous one left behind. Nothing holds a pointer,
  // reference or iterator into it past its loop iteration — keep it that way.
  std::vector<float> linear_rgb_scratch_;


  std::vector<std::thread> simulator_threads_;
  mutable std::mutex prod_mutex_;

  // Active scene and generation counter for batch staleness detection. The scene the
  // workers trace is the most recent SUBMISSION, whichever kind: CommitConfig (a render)
  // and StartRaypathAnalysis (an analysis) both bind it, under scene_mutex_, together with
  // scene_generation_ and committed_epoch_ below. GenerateScene reads it without asking
  // which kind bound it — that is the point, an analysis traces the document it was handed —
  // so nothing may read this field as "the last RENDER's scene"; config_manager_ is that.
  std::shared_ptr<const SceneConfig> active_scene_;
  // Snapshot of renderers paired with active_scene_ (task 252.3, TraceBackend seam).
  // Set in CommitConfig under scene_mutex_ in lockstep with active_scene_, then
  // attached to every SimBatch emitted by GenerateScene. Stays nullptr if no
  // CommitConfig has yet succeeded; consumers tolerate null. An analysis submission
  // binds it null too: it has no render output, and the forced CPU route reads no renderer.
  std::shared_ptr<const std::vector<RenderConfig>> active_renders_;
  // Design 2 (task-engine-redirect-design2): snapshot of raypath_color paired
  // with active_scene_ / active_renders_. Updated inside the same scene_mutex_
  // critical section so a concurrent CommitConfig cannot tear the
  // (scene, renders, raypath_color) triple. Null → no color configured (AC3
  // zero-cost path). An analysis submission binds it null as well, and not as an
  // economy: the simulator builds the colour gate table from (raypath_color, scene) on
  // every batch, and a colour config left over from the last render can name a crystal
  // the analysis's scene does not have — that is a throw on a worker thread.
  std::shared_ptr<const RaypathColorConfig> active_raypath_color_;
  std::atomic<uint64_t> scene_generation_{ 0 };
  // Published lifecycle epoch (the backend-owned truth authority). Distinct from
  // scene_generation_ (an internal batch-staleness key): keeping them separate
  // keeps the externally-published epoch from being polluted by batch-scheduling
  // details. ++ inside the scene_mutex_ critical section (next to scene_generation_)
  // of BOTH submission entry points — CommitConfig and StartRaypathAnalysis — on the
  // accumulator-reset action: every successful submission is reset-causing today, so
  // every success ++s, and an analysis is a submission of its own scene, not a re-run of
  // the last render's. A future "continue-same-config" path (append rays without reset)
  // must skip this ++. See plan §2 decision 3.
  std::atomic<uint64_t> committed_epoch_{ 0 };
  // Highest epoch whose data the CONSUMER has fully drained. Read
  // via DrainedEpoch() / LUMICE_GetDrainStatus; "current epoch is drained" is
  // drained_epoch_ == committed_epoch_. Deliberately NOT cleared by Stop() or
  // CommitConfig(): a new epoch bumps committed_epoch_ past this value, so the
  // comparison reads "not drained yet" on its own, with no reset site to forget
  // (same shape as the ResultFrame's `fresh = snap->epoch == committed_epoch`).
  // Published only by PublishDrainedEpochIfSettled(); monotonically non-decreasing.
  std::atomic<uint64_t> drained_epoch_{ 0 };

  // Persistent thread state machine: threads wait on start_cv_ when kStopped,
  // work when kRunning, and exit when kTerminating.
  enum class ServerState { kStopped, kRunning, kTerminating };
  std::atomic<ServerState> state_{ ServerState::kStopped };
  std::mutex start_mutex_;
  std::condition_variable start_cv_;
  // std::atomic for the lock-free read in Stop()'s wait predicate lambda; mutations
  // are ALSO guarded by start_mutex_ (see RunPersistentLoop) to close the CV
  // lost-wakeup window. Keep both layers — do not simplify to plain int.
  std::atomic<int> active_workers_{ 0 };
  std::atomic<uint64_t> start_generation_{ 0 };  // Incremented by Start(); prevents re-entry after natural completion

  std::atomic_bool work_started_{ false };
  std::atomic_bool scene_gen_active_{ false };  // True while GenerateScene is actively producing batches
  // task-296.7 instrumentation: ensures the first-kIdle WARN fires exactly once per run.
  // Reset alongside work_started_ in CommitConfig / Stop so each render cycle re-arms.
  // mutable: GetStatus() is const but flips this exactly-once flag on first kIdle.
  mutable std::atomic_bool idle_logged_{ false };

  // Preferred trace backend. Cached at server level so the preference survives
  // Stop()/Start() cycles and is the authoritative source for any future
  // simulator-rebuild path. Mirrored into every Simulator via
  // SetPreferredBackend(). Default is CPU.
  std::atomic<BackendKind> preferred_backend_{ BackendKind::kCpu };

  // Which kind of run the current session is. Written by the two entry points that
  // (re)start a run — CommitConfig (→ kRender) and StartRaypathAnalysis (→ kAnalysis) —
  // after their Stop() has joined the previous run's workers; read by GenerateScene
  // (the CPU-forcing half of the route decision), by Stop() (the analysis-only
  // materialisation below), and by the mutual-exclusion guards at both entry points.
  // Atomic for the same reason the analysis budget fields below are: control thread
  // writes, worker threads read, and a plain member would rest on Stop()'s join being a
  // happens-before that every future edit of this file preserves.
  // The enum itself is public (server.hpp SessionKind) so that GetSessionKind() can hand
  // the very same value to the C API instead of a mirrored copy of it.
  std::atomic<SessionKind> mode_{ SessionKind::kRender };

  // The analysis run's own ray budget (RaypathAnalysisRequest::ray_num_), carried from
  // StartRaypathAnalysis (control thread) to GenerateScene's budget ingest point (worker
  // thread) — the same write/read pair, and the same reason for atomics, as mode_
  // above. Two atomics rather than one optional because an optional is not lock-free
  // and the pair is only ever read under mode_ == kAnalysis, after both stores: the
  // flag says whether the value applies, the value is the budget (kInfSize = unlimited).
  // Written by StartRaypathAnalysis for every analysis session, on both branches, so a
  // session that inherits the scene's budget cannot read the previous session's
  // override; never read by a render session (the gate is kAnalysis).
  std::atomic<size_t> analysis_ray_num_override_{ 0 };
  std::atomic_bool analysis_ray_num_overridden_{ false };

  // ResolveGpuRoute's verdict at CONSTRUCTION time — the route this
  // server was actually sized for (worker_count, and hence simulators_.size()).
  // GenerateScene re-derives its own kGpuRoute from the live preferred_backend_
  // each call, deliberately: these are two call sites of the one ResolveGpuRoute
  // (there is no second implementation of the routing rule), asking two different
  // questions — "what was this server built as" vs "what does the current
  // preference imply". Today they cannot disagree (SetPreferredBackend has no
  // caller outside the C API; the GUI reconstructs the server on a backend
  // toggle), and GenerateScene logs it if they ever do, so the drift cannot be
  // silent. BackendFellBack must use THIS one: "fell back" is only meaningful
  // against the route that sized simulators_ to a single Simulator.
  bool gpu_route_ = false;

  // Set once per Run() by GenerateScene, when it has dropped the batches it had
  // queued at the GPU grain after the backend went away (see the invalidation there).
  // Written by that one thread, read by ConsumeData, and cleared at GenerateScene's
  // entry — so it is per-Run() the same way backend_active_ is. It exists to make the
  // work it gates one-shot: both the extra wake-up ConsumeData sends and the producer's
  // extra wait predicate are there only to get that invalidation to happen promptly,
  // and both are pure overhead on every batch afterwards.
  std::atomic_bool fallback_queue_invalidated_{ false };

  // Single owner of "read the GPU route's Simulator's backend liveness".
  // Both consumers (GenerateScene's per-batch grain, BackendFellBack's GUI signal)
  // go through here so the guard and the locking discipline cannot drift apart.
  // Returns true (i.e. "nothing has fallen back") whenever the question does not
  // apply — no simulator, or a multi-worker CPU server where no single simulator
  // owns the answer. Locking mirrors GetStatus(): simulators_ never changes size
  // after construction and BackendActive() is itself an atomic load, so the mutex
  // is for consistency with the surrounding code rather than for correctness.
  bool ReadBackendActive() const {
    std::lock_guard<std::mutex> lock(prod_mutex_);
    if (simulators_.size() != 1) {
      return true;
    }
    return simulators_[0].BackendActive();
  }

  std::atomic_int sim_scene_cnt_;
  std::mutex scene_mutex_;
  std::condition_variable scene_cv_;

  std::thread consume_data_thread_;
  std::thread generate_scene_thread_;

  mutable std::mutex status_mutex_;
  ServerStatus status_;

  Logger logger_{ "Server" };

 public:
  void SetLogLevel(LogLevel level);
  Logger& GetLogger() { return logger_; }
};

template <typename F>
void ServerImpl::RunPersistentLoop(F work_fn) {
  uint64_t my_gen = 0;
  while (true) {
    {
      std::unique_lock<std::mutex> lk(start_mutex_);
      start_cv_.wait(lk, [this, &my_gen] {
        return state_.load() == ServerState::kTerminating ||
               (state_.load() == ServerState::kRunning && start_generation_.load() != my_gen);
      });
      if (state_.load() == ServerState::kTerminating) {
        return;
      }
      my_gen = start_generation_.load();
      active_workers_.fetch_add(1);
    }
    work_fn();
    // Mutate the CV predicate var under start_mutex_ — the same lock Stop() holds
    // while checking active_workers_==0. Closes the lost-wakeup window where Stop()
    // saw the old value and was atomically releasing the lock to enter wait while
    // the worker's notify_all() fell into the release→park gap.
    bool last = false;
    {
      std::lock_guard<std::mutex> lk(start_mutex_);
      last = (active_workers_.fetch_sub(1) == 1);
    }
    if (last) {
      // Last active worker — notify Stop() if it's waiting
      start_cv_.notify_all();
    }
  }
}


// task-268.7 (owner 2026-06-15): the CPU and GPU routes do NOT mirror each other
// — each picks its own optimal orchestration, so the server runs two parallel
// shapes. The GPU/Metal route is a SINGLE engine (N engines would contend one
// GPU — explore-263); the legacy CPU route keeps MULTI-worker parallelism (that
// IS its performance model — collapsing it to 1 worker is a ~6x regression on
// the perf baseline + GUI default path). The route is fixed at construction; the
// GUI reconstructs the server when the Metal checkbox toggles. An env
// LUMICE_TRACE_BACKEND override (CLI / --benchmark) takes precedence over the
// preferred_backend argument, mirroring CreateBackend (simulator.cpp).
// ResolveGpuRoute — does this backend run the GPU single-engine route
// (worker_count=1 + large dispatch)? This MUST agree with CreateBackend's actual
// routing decision (simulator.cpp): if it answers "GPU" but CreateBackend falls
// back to legacy CPU (e.g. CUDA build with no device), the server would size a
// single-worker 32768-ray-batch pipeline onto the multi-core CPU path — a severe
// regression. So the CUDA branch gates on the same CudaDeviceAvailable() probe
// CreateBackend uses (cached std::once, cheap to call per GenerateScene). Metal
// stays optimistic on Apple (Metal is effectively always present; a PSO failure
// degrades to CPU via task-282, the accepted edge case).
// (296.6: generalized from the former Metal-only ResolveMetalRoute so CUDA also
// takes the single-engine route — see doc/seam-design.md §5.)
bool ResolveGpuRoute(BackendKind preferred_backend, Logger& logger, bool force_cpu) {
  // The analysis run's forced CPU route sits ABOVE the env override, as it does in
  // CreateBackend — the two must agree, and "CPU regardless of the environment" is the
  // contract being kept (see the declaration in server.hpp).
  if (force_cpu) {
    return false;
  }
  // Env override wins, mirroring CreateBackend's TraceBackendOverride handling.
  if (std::optional<std::string> override = env::TraceBackendOverride(logger)) {
    const std::string& name = *override;
    if (name == "cpu_backend" || name == "legacy") {
      return false;
    }
#if defined(__APPLE__)
    if (name == "metal") {
      return true;
    }
#endif
#if defined(LUMICE_CUDA_ENABLED)
    if (name == "cuda") {
      return CudaDeviceAvailable();
    }
#endif
    // Unknown / unavailable override name → fall through to preferred_backend.
  }
  // preferred_backend. CreateBackend (simulator.cpp) holds the exhaustive
  // -Wswitch over BackendKind that forces a new enum value to be handled; this
  // site must AGREE with it (a GPU answer here = single-engine sizing). Written
  // as guarded early-returns rather than a switch so the all-false config
  // (non-Apple, non-CUDA build) does not trip bugprone-branch-clone.
#if defined(__APPLE__)
  if (preferred_backend == BackendKind::kMetal) {
    return true;
  }
#endif
#if defined(LUMICE_CUDA_ENABLED)
  if (preferred_backend == BackendKind::kCuda) {
    return CudaDeviceAvailable();
  }
#endif
  return false;  // kCpu, or the requested GPU backend is unavailable in this build
}

// See the doc block on the declaration in server.hpp.
size_t EffectiveDispatchCap(bool gpu_route, bool backend_active, size_t nominal_cap, size_t fallback_cap) {
  if (gpu_route && !backend_active) {
    // min(), not fallback_cap: an explicit LUMICE_DISPATCH_RAY_NUM below the legacy
    // default is a deliberate request and must not be raised by a fallback.
    return std::min(nominal_cap, fallback_cap);
  }
  return nominal_cap;
}

ServerImpl::ServerImpl(int num_workers, uint32_t sim_seed, BackendKind preferred_backend)
    : config_manager_{}, scene_queue_(std::make_shared<Queue<SimBatch>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle) {
  // Publish an empty frame up front so published_frame_ is never null. A
  // reader that arrives before the first snapshot then gets an honest "nothing yet"
  // (no results, nullopt stats) instead of forcing every read path to carry a null
  // branch — and lumice.h's "all-zero struct if no snapshot has been taken yet" promise
  // for the cached-stats read holds by construction.
  StorePublished(std::make_shared<const ResultFrame>());
  preferred_backend_.store(preferred_backend, std::memory_order_release);
  gpu_route_ = ResolveGpuRoute(preferred_backend, logger_);
  int worker_count = 1;
  if (gpu_route_) {
    worker_count = 1;  // GPU route: single engine (task-268.7; CUDA joined 296.6)
  } else {
    worker_count = num_workers > 0 ? num_workers : std::min(PhysicalCoreCount(), kMaxDefaultWorkerCount);
    if (sim_seed != 0) {
      worker_count = 1;  // deterministic CPU contract: fixed seed → single worker
      // Also what keeps SimData::producer_effective_seed_ distinct per worker
      // (a fixed seed is returned verbatim as the effective seed): relaxing
      // this needs every worker's seed made distinct, or ChainIdMerger fuses
      // chains across workers silently. The per-index offset below is that
      // guard, dead today.
    }
  }
  // AC1 observability (296.6): the GPU single-engine route must run worker_count==1.
  ILOG_INFO(logger_, "ServerImpl: gpu_route={} worker_count={} (preferred_backend={})", gpu_route_, worker_count,
            static_cast<int>(preferred_backend));
  for (int i = 0; i < worker_count; i++) {
    uint32_t worker_seed = sim_seed != 0 ? sim_seed + static_cast<uint32_t>(i) : 0u;
    simulators_.emplace_back(scene_queue_, data_queue_, worker_seed);
  }

  // Propagate the construction-time backend into every simulator. The server-level
  // preferred_backend_ above only drives GenerateScene's dispatch sizing + worker
  // count; each Simulator owns its OWN preferred_backend_ (default kCpu) and
  // reads it at Run() to pick the trace backend (CreateBackend). Without this, a
  // server built via CreateServerEx(preferred_backend=metal) would size dispatches
  // for Metal yet still trace on the legacy CPU path — the runtime SetPreferredBackend
  // propagated, but the constructor did not (latent until the GUI backend-toggle
  // reconstruct made CreateServerEx the live route, scrum-268.6 Part C).
  for (auto& s : simulators_) {
    s.SetPreferredBackend(preferred_backend);
  }

  // Spawn persistent threads — they start in cv.wait(), not working. All
  // simulators_ are emplaced above first, so the &s references stay valid (no
  // further vector reallocation).
  for (auto& s : simulators_) {
    simulator_threads_.emplace_back([this, &s]() { RunPersistentLoop([&s] { s.Run(); }); });
  }
  consume_data_thread_ = std::thread([this]() { RunPersistentLoop([this]() { ConsumeData(); }); });
  generate_scene_thread_ = std::thread([this]() { RunPersistentLoop([this]() { GenerateScene(); }); });
}


ServerImpl::~ServerImpl() {
  // Stop first to drain workers and clean up consumers (if still running)
  Stop();

  // Shutdown queues to unblock any blocking Get() calls
  scene_queue_->Shutdown();
  data_queue_->Shutdown();

  // Signal all threads to terminate
  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    state_.store(ServerState::kTerminating);
  }
  start_cv_.notify_all();
  scene_cv_.notify_one();

  // Stop simulators to break their inner Run() loops
  for (auto& s : simulators_) {
    s.Stop();
  }

  // Join all persistent threads
  for (auto& t : simulator_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  if (consume_data_thread_.joinable()) {
    consume_data_thread_.join();
  }
  if (generate_scene_thread_.joinable()) {
    generate_scene_thread_.join();
  }
}


// Lifecycle reset sequence: see doc/capi-lifecycle-architecture.md §7.
// NOLINTNEXTLINE(readability-function-size)
namespace {

// ------------------------------------------------------------------------------------------------
// "print takes over the colour channel" — the one rule of doc/print-mode-subtractive-ink.md §7,
// stated once, on the CLI/server side.
//
// The print operator deposits a single neutral ink whose only degree of freedom is DENSITY, so
// every field whose job is to carry information in a HUE stops being readable the moment the tone
// is print. Three such fields exist in a RenderConfig today, and the point of putting all three in
// one function is that they are three instances of ONE rule rather than three special cases: a
// reader who has to decide what a fourth field should do reads this block, not four scattered ifs.
//
// NEW INSTANCES GO HERE, in the same shape — one `if` naming the field and one ILOG_WARN saying
// (a) what the config asked for, (b) what print does instead, (c) that the value is kept. Do NOT
// open a parallel judgement point elsewhere in the commit path; a second owner is how the CLI and
// the GUI came to disagree about other fields before.
//
// One warning per instance rather than one merged line, because the three have different reach:
// the composite is scene-level (one class table shared by every renderer), while ray_color and the
// annotation colours are each read off the renderer's own entry. A merged line would make the
// instances that did NOT fire read as if they had.
// ------------------------------------------------------------------------------------------------

// True when `c` differs from `ref` in any component. Colour comparison here is an EXACT float
// compare on purpose: the question is "did anybody write a value into this field", not "is this
// visually distinguishable from the default", and a tolerance would answer the second.
bool ColorDiffers(const float (&c)[3], const float (&ref)[3]) {
  return c[0] != ref[0] || c[1] != ref[1] || c[2] != ref[2];
}

// Whether any annotation in this renderer carries a non-default colour.
//
// The judgement is deliberately "is the colour non-default", NOT "would this annotation actually
// be drawn". Folding in opacity_ and the three per-family line switches would narrow the warning
// to configs that really do lose something visible, but it costs a cross-read of three independent
// flags to buy a lower false-positive rate on a NON-BLOCKING notice. One extra log line for a
// fully transparent line is a better failure than a missing line for a config the user is staring
// at and cannot explain.
bool HasNonDefaultAnnotationColour(const RenderConfig& rc) {
  const GridLineParam kGridDefault{};
  for (const auto* family : { &rc.angular_dist_grid_, &rc.elevation_grid_, &rc.longitude_grid_ }) {
    for (const auto& line : *family) {
      if (ColorDiffers(line.color_, kGridDefault.color_)) {
        return true;
      }
    }
  }
  const ZenithNadirParam kZenithNadirDefault{};
  if (rc.zenith_nadir_.enabled_ && ColorDiffers(rc.zenith_nadir_.color_, kZenithNadirDefault.color_)) {
    return true;
  }
  const MarkerStyleParam kMarkerDefault{};
  for (const auto& m : rc.markers_) {
    if (m.enabled_ && ColorDiffers(m.color_, kMarkerDefault.color_)) {
      return true;
    }
  }
  return false;
}

// `class_table` is the SCENE's table, the same object every RenderConsumer is constructed with and
// the same one RenderConsumer::ColoredMask() reads back — so `referenced_mask_ != 0` here and
// `rc->ColoredMask() != 0` in DoSnapshot's composite loop are two readings of one field, not two
// computations that have to be kept in step. That is what makes "warned" and "actually excluded"
// the same set rather than two sets that happen to agree today.
void WarnPrintModeIgnoresColourFields(Logger& logger, const std::map<IdType, RenderConfig>& renderers,
                                      const ColorClassTable& class_table) {
  const RenderConfig kDefaults{};
  for (const auto& [id, rc] : renderers) {
    if (rc.tone_ != RenderConfig::kPrint) {
      continue;
    }
    if (class_table.referenced_mask_ != 0) {
      ILOG_WARN(logger,
                "CommitConfig: render[{}] is tone=print, so the raypath-colour composite is not "
                "produced — print lays one neutral ink and carries no hue to tell the classes apart. "
                "The colour classes are kept and take effect again under tone=screen.",
                id);
    }
    if (ColorDiffers(rc.ray_color_, kDefaults.ray_color_)) {
      ILOG_WARN(logger,
                "CommitConfig: render[{}] sets ray_color, which tone=print does not read — the ink is "
                "neutral by construction. The value is kept and takes effect again under tone=screen.",
                id);
    }
    if (HasNonDefaultAnnotationColour(rc)) {
      ILOG_WARN(logger,
                "CommitConfig: render[{}] sets an annotation colour, which tone=print does not read — "
                "overlay lines are drawn by density on paper, not by hue. The values are kept and take "
                "effect again under tone=screen.",
                id);
    }
  }
}

// The other half of doc/print-mode-subtractive-ink.md §8, on the side that has no panel to put a
// notice in: a config whose zero-energy colour has run out of headroom renders a picture that is
// not there, and `Lumice -f that.json` would otherwise write the file and exit 0 with nothing said.
//
// The judgement itself is NOT made here — `ContrastHeadroomIsLow` is the single predicate the GUI
// also calls (util/contrast_headroom.hpp), so there is one threshold in the tree and not two. What
// is local to this side is the conversion: core holds both grounds as LINEAR RGB while the predicate
// is defined on the sRGB encoding the output actually lands in, so each component is encoded first.
//
// Which ground is read follows `tone_`, because only one of the two is the ground under either law:
// under screen a pitch-black paper is irrelevant, under print a blinding sky is. Reading both would
// warn about a field the live operator never touches.
void WarnLowContrastHeadroom(Logger& logger, const std::map<IdType, RenderConfig>& renderers) {
  for (const auto& [id, rc] : renderers) {
    const bool print = rc.tone_ == RenderConfig::kPrint;
    const float* ground_linear = print ? rc.paper_ : rc.background_;
    float ground_srgb[3]{};
    for (int j = 0; j < 3; j++) {
      ground_srgb[j] = LinearToSrgb(ground_linear[j]);
    }
    const ToneLawLimit limit = print ? ToneLawLimit::kBlack : ToneLawLimit::kWhite;
    if (!ContrastHeadroomIsLow(ground_srgb, limit)) {
      continue;
    }
    // Two wordings rather than one parameterised line: each has to say WHY the image looks broken,
    // and the two reasons are not the same sentence with a word swapped. Both name the fix, because
    // the user's starting position is "I changed nothing and the halo is gone".
    if (print) {
      ILOG_WARN(logger,
                "CommitConfig: render[{}] is tone=print and its paper is within {} 8-bit levels of "
                "black. Ink can only ever darken the paper and never reaches black, so every feature "
                "comes out within those few levels of the page and the image will look blank. Lighten "
                "the paper, or switch tone back to screen.",
                id, kContrastHeadroomWarnLevels);
    } else {
      ILOG_WARN(logger,
                "CommitConfig: render[{}] is tone=screen and its background is within {} 8-bit levels "
                "of white. Light is ADDED to that background and clamps at white, so a halo has "
                "almost nowhere left to go and will be invisible — while grid and overlay lines, "
                "which are blended rather than added, stay perfectly visible and make it look as "
                "though the simulation failed. Darken the background, or switch tone to print.",
                id, kContrastHeadroomWarnLevels);
    }
  }
}

}  // namespace

Error ServerImpl::ParseConfigManager(const nlohmann::json& config_json, const char* caller,
                                     const std::function<void(const ConfigManager&)>& validate, ConfigManager* out) {
  ConfigManager parsed;
  try {
    parsed = config_json.get<ConfigManager>();
    if (validate) {
      validate(parsed);
    }
  } catch (const nlohmann::json::out_of_range& e) {
    ILOG_ERROR(logger_, "{}: Missing field: {}", caller, e.what());
    return Error::MissingField(e.what());
  } catch (const nlohmann::json::exception& e) {
    ILOG_ERROR(logger_, "{}: JSON parsing error: {}", caller, e.what());
    return Error::InvalidJson(e.what());
  } catch (const std::exception& e) {
    ILOG_ERROR(logger_, "{}: Configuration error: {}", caller, e.what());
    return Error::InvalidConfig(e.what());
  } catch (...) {
    ILOG_ERROR(logger_, "{}: Unknown error", caller);
    return Error::InvalidConfig("Unknown configuration error");
  }
  *out = std::move(parsed);
  return Error::Success();
}

Error ServerImpl::CommitConfig(const nlohmann::json& config_json, bool* out_reused) {
  auto commit_start = std::chrono::steady_clock::now();
  ILOG_DEBUG(logger_, "CommitConfig: entry");

  // Mutual exclusion with the analysis run, this direction. "In progress" is
  // GetSimLifecycle() == kRunning at THIS moment, not "an analysis was ever started": a
  // completed (or stopped) analysis session is exactly the state a new render commit is
  // expected to replace. Checked before the parse so a rejected commit leaves the server
  // — config_manager_ included — untouched.
  if (mode_.load(std::memory_order_acquire) == SessionKind::kAnalysis && GetSimLifecycle() == SimLifecycle::kRunning) {
    ILOG_WARN(logger_, "CommitConfig: rejected — an analysis run is in progress; Stop() it first");
    return Error::ServerError("analysis run in progress; stop it before committing a render config");
  }

  // Parse into a temporary first so that a parse failure leaves the running server untouched.
  ConfigManager new_config;
  // Runtime color-class table (default = empty → no raypath_color, the pre-colour
  // behavior bit-for-bit). Declared outside the parse so it survives to
  // the reuse-judgment / consumer construction below.
  ColorClassTable class_table;
  // The parsed composite mode. Declared outside the parse so it
  // survives to the member assignment below, mirroring class_table.
  CompositeMode composite_mode = CompositeMode::kDominant;
  // The render's own validation step, run inside ParseConfigManager's try on the parsed
  // document so its throws map onto the same Error vocabulary as the parse itself.
  // Color-class schema build path. BuildColorClassTable
  // resolves id → ci (setting_[] slot) using the scene and may throw
  // std::invalid_argument on any config error (unknown combine, missing
  // (crystal,filter) pair, degenerate duplicate, out-of-range summand,
  // combine:"all" ban) — that lands in the std::exception catch → Error::InvalidConfig.
  // class_table feeds directly into the RenderConsumer (per-class Y-lane accumulation)
  // and into the compositor (CompositeColorClassesLinear); no legacy per-bit adapter layer.
  const auto build_colour_tables = [this, &class_table, &composite_mode](const ConfigManager& parsed) {
    ColorGateTable color_gate_table = BuildColorGateTable(parsed.raypath_color_, parsed.scene_);
    // task-gui-feedback-affordances Step 5 (AC1): carry the component-bit
    // overflow count (predicates that hit `kNoBit`) out so the GUI DoRun path
    // can surface a "coloring degraded" modal via LUMICE_GetColorOverflowInfo.
    // Written on the successful (non-throwing) branch — a parse/config error
    // above leaves the prior counter untouched, matching the "keep prior
    // committed state" semantics of the surrounding try/catch.
    last_color_component_overflow_count_.store(color_gate_table.component_overflow_count_, std::memory_order_release);
    // task-color-degrade-gui-surfacing: synchronously reset the async GPU
    // color-degrade tally on commit. These are re-populated from the worker's
    // first batch (ConsumeData); zeroing here ensures that switching FROM an
    // overflowing config TO a non-overflowing one does not leave a stale count
    // that the GUI poll path would read (before the first batch lands) and
    // falsely surface a degrade modal for the new, clean config.
    last_color_symmetry_group_overflow_.store(0, std::memory_order_release);
    last_color_or_summand_overflow_.store(0, std::memory_order_release);
    last_color_class_overflow_.store(0, std::memory_order_release);
    class_table = BuildColorClassTable(parsed.raypath_color_, parsed.scene_, color_gate_table);
    composite_mode = ParseCompositeMode(parsed.raypath_color_.mode_);
  };
  if (const Error err = ParseConfigManager(config_json, "CommitConfig", build_colour_tables, &new_config)) {
    return err;
  }

  // The one rule of doc/print-mode-subtractive-ink.md §7, applied to the config that is about to
  // become live. Placed after the parse (so class_table exists) and before Stop() (so the notice
  // reaches the log alongside the commit that caused it, not one restart later). Diagnostic only —
  // it never changes `new_config` and never fails the commit: none of the three is a configuration
  // ERROR, they are configurations whose colour half print has no way to show.
  WarnPrintModeIgnoresColourFields(logger_, new_config.renderers_, class_table);

  // §8's predicate, applied at the same point and with the same standing: diagnostic only, never a
  // reason to fail the commit. A ground with no headroom left is a legal configuration that renders
  // an image nobody can read, which is precisely why it has to be said out loud rather than refused.
  WarnLowContrastHeadroom(logger_, new_config.renderers_);

  // Stop → rebuild consumers → Start
  auto stop_start = std::chrono::steady_clock::now();
  Stop();
  auto stop_end = std::chrono::steady_clock::now();
  auto stop_ms = std::chrono::duration<double, std::milli>(stop_end - stop_start).count();

  // A successful commit is a RENDER session, whatever the previous one was. Written
  // unconditionally (also when already kRender) so that no early-return path above can be
  // the one that forgot to switch back. Stop() has joined the workers, so the three
  // per-Simulator analysis properties can be withdrawn here without racing a Run(): a
  // render session must not pay the chain-id cost (non-analysis mode is zero-cost by
  // contract) and must get the backend the user asked for back.
  const bool was_analysis = mode_.exchange(SessionKind::kRender, std::memory_order_acq_rel) == SessionKind::kAnalysis;
  if (was_analysis) {
    {
      std::lock_guard<std::mutex> lock(prod_mutex_);
      for (auto& s : simulators_) {
        s.SetAnalysisChainId(false, 0);
        s.SetAnalysisForceCpu(false);
      }
    }
    // Symmetric to StartRaypathAnalysis: the analysis histogram must not linger on the
    // render session's first frame. Gated on was_analysis so a render → render commit keeps
    // publishing exactly what it always did (the previous image, re-stamped stale).
    PublishEmptyFrame();
  }

  // Check if consumers can be reused (same renderer key set, no layout changes).
  // See doc/accumulator-consumer-architecture.md §5.4 (reuse eligibility).
  // An analysis session's consumer set (histogram + stats) is never reusable for a
  // render: it holds no RenderConsumer. `was_analysis` says so explicitly below rather
  // than leaning on the renderer-count comparison, which would let a zero-renderer
  // config (whose render set is also two consumers) reuse the histogram set by accident.
  auto old_renderers = config_manager_.renderers_;
  config_manager_ = std::move(new_config);

  // task-339.3: a change in the per-class (combine_, member_bits_) shape is a
  // scene-level (cross-renderer) change that must force a full consumer rebuild
  // — a reused RenderConsumer would keep its old per-class lane layout. Uses
  // structural NeedsRebuild instead of the plain uint64 mask compare so a
  // reshape at the same referenced_mask_ (e.g. "one 2-bit any class" ↔ "two
  // 1-bit classes") still triggers a rebuild. Orthogonal to the per-renderer
  // NeedsRebuild() layout check below.
  bool class_table_changed = NeedsRebuild(active_class_table_, class_table);
  active_class_table_ = class_table;
  // task-339.4: publish the composite mode in lockstep with the class table.
  // Even a reused consumer set (same class shape) may carry a new mode (or
  // new per-class colors / visibility already carried by active_class_table_),
  // so this is refreshed unconditionally. DoSnapshot reads it under
  // consumer_mutex_; Stop() above has drained all workers so there is no
  // concurrent DoSnapshot here.
  active_composite_mode_ = composite_mode;

  bool can_reuse = !consumers_.empty() && !was_analysis && !class_table_changed &&
                   (old_renderers.size() == config_manager_.renderers_.size());
  if (can_reuse) {
    auto old_it = old_renderers.begin();
    auto new_it = config_manager_.renderers_.begin();
    for (; old_it != old_renderers.end(); ++old_it, ++new_it) {
      if (old_it->first != new_it->first || NeedsRebuild(old_it->second, new_it->second)) {
        can_reuse = false;
        break;
      }
    }
  }

  auto rebuild_start = std::chrono::steady_clock::now();
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    if (can_reuse) {
      // Reuse path: reset accumulators + update appearance fields
      auto it = config_manager_.renderers_.begin();
      for (auto& c : consumers_) {
        if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
          rc->ResetWith(it->second, config_manager_.scene_.light_source_.param_);
          ++it;
        } else {
          c->Reset();  // StatsConsumer / AnchorConsumer
        }
      }
    } else {
      // Full rebuild path
      consumers_.clear();
      for (const auto& [_, r] : config_manager_.renderers_) {
        // task-339.3: pass the color-class table so each consumer allocates one
        // Y-lane per class (empty table → no lanes, pre-336 behavior). The sun comes from the
        // scene, not the renderer: it is what the angular-distance annotations are measured from.
        consumers_.emplace_back(
            std::make_shared<RenderConsumer>(r, active_class_table_, config_manager_.scene_.light_source_.param_));
      }
      consumers_.emplace_back(std::make_shared<StatsConsumer>());
      // One per SESSION, not one per renderer. See AnchorConsumer's own docs for why that
      // is the contract rather than an economy: every element of consumers_ receives the
      // same batch, so a second instance would double-count the same physical rays into a
      // scalar that is supposed to describe the sky.
      //
      // It needs no ResetWith counterpart on the reuse branch above: its buffer's shape is
      // a compile-time constant and depends on no user field, so there is nothing a config
      // change could invalidate — the plain Reset() the branch already calls is complete.
      consumers_.emplace_back(std::make_shared<AnchorConsumer>());
    }
  }
  auto rebuild_end = std::chrono::steady_clock::now();
  auto rebuild_ms = std::chrono::duration<double, std::milli>(rebuild_end - rebuild_start).count();
  ILOG_DEBUG(logger_, "CommitConfig: consumers {} ({:.1f}ms)", can_reuse ? "reused" : "rebuilt", rebuild_ms);
  if (out_reused) {
    *out_reused = can_reuse;
  }

  auto new_scene = std::make_shared<SceneConfig>(config_manager_.scene_);
  auto new_renders = std::make_shared<std::vector<RenderConfig>>();
  new_renders->reserve(config_manager_.renderers_.size());
  for (const auto& [_, r] : config_manager_.renderers_) {
    new_renders->push_back(r);
  }
  auto new_raypath_color = std::make_shared<const RaypathColorConfig>(config_manager_.raypath_color_);
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    active_scene_ = std::move(new_scene);
    active_renders_ = std::move(new_renders);
    active_raypath_color_ = std::move(new_raypath_color);
    scene_generation_.fetch_add(1);
    // Publish the new lifecycle epoch alongside the accumulator reset. Stop()
    // above has drained all workers, so no in-flight batch reads a half-updated
    // epoch. Pinned to the reset action (not to CommitConfig entry) so a future
    // continue-same-config path would correctly leave the epoch unchanged.
    committed_epoch_.fetch_add(1, std::memory_order_release);
  }

  auto start_start = std::chrono::steady_clock::now();
  Start();
  auto start_end = std::chrono::steady_clock::now();
  auto start_ms = std::chrono::duration<double, std::milli>(start_end - start_start).count();

  auto commit_end = std::chrono::steady_clock::now();
  ILOG_INFO(logger_, "CommitConfig: restart took {:.1f}ms (Stop {:.1f}ms + rebuild {:.1f}ms + Start {:.1f}ms)",
            std::chrono::duration<double, std::milli>(commit_end - commit_start).count(), stop_ms, rebuild_ms,
            start_ms);

  return Error::Success();
}


void ServerImpl::PublishEmptyFrame() {
  std::lock_guard<std::mutex> snapshot_pass(do_snapshot_mutex_);
  auto empty = std::make_shared<ResultFrame>();
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    // The generation is NOT bumped: this is not a new snapshot, it is the absence of one.
    // A poller comparing generations sees "nothing new", and has_valid_data_ says stale.
    empty->snapshot_generation_ = snapshot_generation_;
  }
  empty->epoch_ = committed_epoch_.load(std::memory_order_acquire);
  empty->has_valid_data_ = false;
  StorePublished(std::move(empty));
}

// The analysis run. Same lifecycle as a render commit — parse, Stop, swap the consumer
// set, bind the scene, Start — on the scene `scene_json` carries: the analysis is a
// submission of its own document, not a re-run of whatever the last CommitConfig left in
// config_manager_ (which it never touches — the next render commit still judges consumer
// reuse against the last RENDER). What makes it an analysis session is three per-Simulator
// properties (chain ids on, CPU forced) plus mode_, which GenerateScene / ConsumeData / the
// two entry guards read. Everything else — queues, threads, epoch, the drain signal,
// AcquireResultFrame — is the one lifecycle this server has, unchanged.
Error ServerImpl::StartRaypathAnalysis(const nlohmann::json& scene_json, const RaypathAnalysisRequest& request) {
  ILOG_DEBUG(logger_, "StartRaypathAnalysis: entry");

  // Mutual exclusion with the render run, this direction. Deliberately NOT "Stop the
  // render and go ahead": a caller that wants that says so by calling Stop() first. A
  // running ANALYSIS is not refused — the same call with a new ROI restarts it, which is
  // how a caller changes the cone without a Stop() round-trip. Checked before the parse so
  // a rejected request leaves the server untouched, as CommitConfig's guard does.
  if (mode_.load(std::memory_order_acquire) == SessionKind::kRender && GetSimLifecycle() == SimLifecycle::kRunning) {
    ILOG_WARN(logger_, "StartRaypathAnalysis: rejected — a render run is in progress; Stop() it first");
    return Error::ServerError("render run in progress; stop it before starting analysis");
  }

  // The scene this run traces, parsed into a local so that a rejected document changes
  // nothing — not the session in flight, not the scene the workers hold. The full
  // ConfigManager, not just its scene section, through the same owner CommitConfig parses
  // with: one grammar, one error mapping. No colour tables and none of the render's output
  // diagnostics (print-mode, contrast headroom): an analysis has no render output for them
  // to be about. ROI validation (zero centre, non-positive radius, ring count < 1) is the
  // consumer constructor's, and it degrades with a log line rather than failing — the
  // request still names a well-defined ROI.
  ConfigManager new_config;
  if (const Error err = ParseConfigManager(scene_json, "StartRaypathAnalysis", nullptr, &new_config)) {
    return err;
  }
  // The scene facts the read-time reduction needs, from the scene this run will trace — the
  // one bound below, which GenerateScene reads. Not from active_scene_: that is the previous
  // submission's until the bind, and a reduce context for a different scene than the one
  // traced would canonicalise chains by the wrong crystals' D parameters.
  RaypathReduceContext reduce_ctx = BuildRaypathReduceContext(new_config.scene_);

  Stop();
  mode_.store(SessionKind::kAnalysis, std::memory_order_release);
  // The render's image must not be readable off an analysis session's frame, not even
  // flagged stale: the first frame of this session is empty until its first batch.
  PublishEmptyFrame();
  // The session's ray budget: the request's own when it carries one, the scene's otherwise.
  // Value before flag, so a reader that sees the flag sees the value it belongs to.
  if (request.ray_num_.has_value()) {
    analysis_ray_num_override_.store(*request.ray_num_, std::memory_order_release);
    analysis_ray_num_overridden_.store(true, std::memory_order_release);
  } else {
    analysis_ray_num_overridden_.store(false, std::memory_order_release);
  }
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    consumers_.clear();
    consumers_.emplace_back(std::make_shared<RaypathHistogramConsumer>(request.roi_, std::move(reduce_ctx)));
    // StatsConsumer for the live ray count (GetLiveSimRayCount reads it by dynamic_cast)
    // — the run's only progress signal, since there is no image to watch grow. No
    // AnchorConsumer: it measures an exposure anchor, and nothing here is exposed.
    consumers_.emplace_back(std::make_shared<StatsConsumer>());
  }
  {
    std::lock_guard<std::mutex> lock(prod_mutex_);
    for (auto& s : simulators_) {
      // Finest, always: the reader reduces (RaypathAnalysisRequest says why).
      s.SetAnalysisChainId(true, FilterConfig::kSymNone);
      s.SetAnalysisForceCpu(true);
    }
  }
  // Said once, here, and not in ResolveGpuRoute / CreateBackend, which run on every
  // render batch: the analysis run overrides both the preference and the environment,
  // and the preference itself is untouched (GetActiveBackend is the readable form).
  ILOG_INFO(logger_,
            "StartRaypathAnalysis: forcing CPU route (analysis run session property; overrides "
            "preferred_backend={} and LUMICE_TRACE_BACKEND, neither is modified); roi_mode={} (chains recorded "
            "unreduced; symmetry is applied when the result is read)",
            static_cast<int>(preferred_backend_.load(std::memory_order_acquire)), static_cast<int>(request.roi_.mode_));
  // Bind the scene this session traces — the same three writes, under the same lock, on
  // the same reset action as CommitConfig's bind (see the fields' declarations for why an
  // analysis advances the epoch: it is a submission of its own scene, and a reader's "is
  // this the frame of my commit" test must say no to an analysis frame). Stop() above has
  // drained every worker, so no in-flight batch reads a half-updated triple, and
  // PublishDrainedEpochIfSettled's invariant (the epoch only moves past a drained one)
  // holds for the same reason it does in CommitConfig. renders and raypath_color are bound
  // null on purpose — their declarations say why.
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    active_scene_ = std::make_shared<const SceneConfig>(new_config.scene_);
    active_renders_.reset();
    active_raypath_color_.reset();
    scene_generation_.fetch_add(1);
    committed_epoch_.fetch_add(1, std::memory_order_release);
  }
  Start();
  return Error::Success();
}

BackendKind ServerImpl::GetActiveBackend() const {
  if (mode_.load(std::memory_order_acquire) == SessionKind::kAnalysis) {
    return BackendKind::kCpu;  // structural: SetAnalysisForceCpu(true) on every Simulator
  }
  std::lock_guard<std::mutex> lock(prod_mutex_);
  if (simulators_.empty()) {
    return BackendKind::kCpu;
  }
  // Every worker of a multi-worker CPU server resolves the same way; the GPU route has
  // exactly one. Reading the first is reading the session.
  return simulators_[0].ActiveBackend();
}


// See doc/accumulator-consumer-architecture.md §4.2 (two-phase snapshot protocol).
// task-342.4 Step 1: this is the single owner of the snapshot_dirty_ flag and
// snapshot_generation_ counter. Any result-frame acquisition that needs an up-to-date
// materialized snapshot funnels through here so that two consumers in the same
// poll tick (e.g. RawXyz + Composite) see a coherent Phase-1..2 atomic event
// rather than racing the dirty flag against each other (plan §3 keypoint 1).
bool ServerImpl::DoSnapshot() {
  // One snapshot pass at a time (see do_snapshot_mutex_'s declaration for why Phase 1's
  // consumer_mutex_ is not enough on its own).
  std::lock_guard<std::mutex> snapshot_pass(do_snapshot_mutex_);

  // Phase 1: memcpy under consumer_mutex_ (short hold).
  // Copy shared_ptrs so consumers stay alive even if Stop() clears consumers_.
  std::vector<ConsumerPtrS> snapshot_consumers;
  ColorClassTable snap_class_table;
  CompositeMode snap_composite_mode = CompositeMode::kDominant;
  float snap_display_ev_total = 0.0f;
  float snap_composite_background[3] = { 0.0f, 0.0f, 0.0f };
  uint64_t generation = 0;
  bool valid_data = false;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    if (!snapshot_dirty_) {
      ILOG_DEBUG(logger_, "DoSnapshot: skip (snapshot_dirty_=false)");
      return false;
    }
    // Clear every renderer's exposure anchor BEFORE anything measures a new one. The set again
    // happens in Phase 2, between the measurement and the bake; this is the half that makes a
    // MISSED set fail safely. Without it a renderer that Phase 2 somehow skipped would bake with
    // the previous pass's anchor — a plausible number, off by however much the scene moved —
    // instead of with 0, which ExposureScale already treats as "no anchor yet" and returns 0 for.
    // Cheap enough to be unconditional: one float store per renderer per snapshot.
    // Resets consumers_, not snapshot_consumers — deliberately: snapshot_consumers is the copy
    // taken a few lines below, so at that copy point the two are identical by construction (both
    // under this same consumer_mutex_ hold, with no consumer add/remove between). The Phase 2
    // push loop further down (which sets the REAL anchor before PostSnapshot bakes) walks
    // snapshot_consumers rather than consumers_ only because it runs outside this lock.
    for (const auto& c : consumers_) {
      if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
        rc->SetAnchorL99Sky(0.0f);
      }
    }
    for (const auto& c : consumers_) {
      c->PrepareSnapshot();
    }
    snapshot_consumers = consumers_;  // shared_ptr copy keeps consumers alive
    // task-339.4: pair the compositor inputs with the consumer set captured
    // above; copied under consumer_mutex_ so they stay consistent with
    // snapshot_consumers. `snap_class_table` is a small std::vector copy
    // (color-class count is O(config), typically ≤ tens) — same lock-hold
    // discipline as before, negligible cost per snapshot.
    snap_class_table = active_class_table_;
    snap_composite_mode = active_composite_mode_;
    snap_display_ev_total = display_ev_total_;
    std::copy(std::begin(composite_background_linear_), std::end(composite_background_linear_),
              std::begin(snap_composite_background));
    valid_data = has_ever_consumed_;
    snapshot_dirty_ = false;
    // Bumping the generation is the shared owner's responsibility (it once lived only
    // in the raw-xyz getter's Phase-1).
    // The counter is the single mechanism by which poller detects new
    // snapshots, so it must be tied to the dirty-consume event itself, not
    // to any one consumer accessor.
    snapshot_generation_++;
    generation = snapshot_generation_;
  }
  // Phase 1.5: pixel counting outside consumer_mutex_ (snapshot_xyz_ is stable here).
  for (const auto& c : snapshot_consumers) {
    if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
      rc->CountEffectivePixels();
    }
  }
  // Phase 2: XYZ→RGB, then ASSEMBLE a new frame. Takes no lock of its own — the whole pass
  // already holds do_snapshot_mutex_, and consumer_mutex_ is deliberately NOT held here.
  // The braces below are a plain scope marking the phase, NOT a lock scope: read them as
  // "Phase 2 runs here", not as "something is protected here".
  // Safe: snapshot_consumers holds shared_ptrs, objects won't be freed.
  //
  // The results are collected into a fresh ResultFrame that gets published at
  // the end. Nothing here overwrites data an earlier reader may still hold — that
  // overwrite (a std::move over cached_composite_results_) was the use-after-free.
  auto frame = std::make_shared<ResultFrame>();
  frame->snapshot_generation_ = generation;
  frame->epoch_ = committed_epoch_.load(std::memory_order_acquire);
  frame->has_valid_data_ = valid_data;
  // The exposure anchor, read once per snapshot from the session's single AnchorConsumer.
  // PrepareSnapshot (Phase 1, under consumer_mutex_) already froze it, so this reads a
  // value no concurrent Consume can move. Frames built before an AnchorConsumer exists —
  // there is none until the first CommitConfig — keep the 0 default.
  for (const auto& c : snapshot_consumers) {
    if (auto* ac = dynamic_cast<AnchorConsumer*>(c.get())) {
      frame->anchor_l99_sky_ = ac->SnapshotL99Sky();
      break;
    }
  }
  // Hand that anchor to the renderers BEFORE they bake with it. PostSnapshot below calls
  // ExposureScale, whose kRelative branch divides by this value, so the ordering here is not
  // bookkeeping — it is the difference between a correctly exposed frame and a black one. It sits
  // after the AnchorConsumer read for the same reason: `frame->anchor_l99_sky_` is the ONE
  // measurement of this pass, and every renderer must expose against that same one rather than
  // reach for the anchor itself and risk reading it at a different moment.
  //
  // No extra lock: the whole pass holds do_snapshot_mutex_, snapshot_consumers holds shared_ptrs
  // so nothing here can be freed, and this touches state only Phase 2 reads.
  for (const auto& c : snapshot_consumers) {
    if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
      rc->SetAnchorL99Sky(frame->anchor_l99_sky_);
    }
  }
  {
    for (const auto& c : snapshot_consumers) {
      c->PostSnapshot();
    }
    for (const auto& c : snapshot_consumers) {
      auto result = c->GetResult();
      if (auto* r = std::get_if<RenderResult>(&result)) {
        // The mono image buffer travels WITH the view: img_buffer_ points into the
        // storage anchor pushed alongside it, so the frame keeps it alive.
        auto* rc = dynamic_cast<RenderConsumer*>(c.get());
        frame->render_results_.push_back(*r);
        frame->render_storage_.push_back(rc != nullptr ? rc->SnapshotImageStorage() : nullptr);
      } else if (auto* s = std::get_if<StatsResult>(&result)) {
        frame->stats_result_ = *s;
      } else if (auto* h = std::get_if<RaypathHistogramResult>(&result)) {
        // Copied out of the consumer's snapshot into this frame, like the stats above:
        // no cache between snapshots, so a render session's frame (whose consumer set
        // has no histogram) keeps the nullopt default rather than a previous analysis.
        frame->raypath_histogram_result_ = *h;
        frame->raypath_reduce_cache_ = std::make_shared<ResultFrame::RaypathReduceCache>();
      }
    }
    // Raw XYZ views + their storage anchors, same treatment as the mono images above.
    // The lifecycle fields are stamped from the values read in Phase 1, so they describe
    // the state this frame was BUILT under; AcquireResultFrame re-stamps them for readers
    // if the live state has since moved (see there).
    for (const auto& c : snapshot_consumers) {
      auto* rc = dynamic_cast<RenderConsumer*>(c.get());
      if (rc == nullptr) {
        continue;
      }
      auto r = rc->GetRawXyzResult();
      r.has_valid_data_ = valid_data;
      r.snapshot_generation_ = generation;
      r.epoch_ = frame->epoch_;
      // Broadcast, not recompute: the anchor was measured ONCE for this snapshot, above.
      r.anchor_l99_sky_ = frame->anchor_l99_sky_;
      frame->xyz_results_.push_back(r);
      frame->xyz_storage_.push_back(rc->SnapshotXyzStorage());
    }
    // task-336.3: only colored consumers (ColoredMask()!=0) produce a composite.
    // Zero-config consumers (mask 0) are skipped → no composite, mono path
    // untouched (plan §0 / risk-5 rollback).
    //
    // The print operator is the second skip reason, and it is a DIFFERENT statement from the
    // first: mask 0 means "nothing was configured", print means "it was configured and cannot be
    // shown". Raypath colour puts the whole payload in the HUE — which path a ray took is read off
    // the colour and nothing else — while print deposits one neutral ink whose only degree of
    // freedom is density (doc/print-mode-subtractive-ink.md §7). Compositing under print would
    // collapse every class onto the same grey, i.e. produce a picture that answers no question.
    // The CONFIG is left completely alone (raypath_color_ and the class table are untouched, and
    // the consumers still accumulate their per-class lanes), so switching tone back to screen
    // restores the composite on the very next snapshot — that is the mechanism behind "the
    // configuration is not lost", not a promise.
    for (const auto& c : snapshot_consumers) {
      auto* rc = dynamic_cast<RenderConsumer*>(c.get());
      if (rc == nullptr || rc->ColoredMask() == 0 || rc->Tone() == RenderConfig::kPrint) {
        continue;
      }
      // task-345.3: display-time EV multiplier + participating-P99 anchor
      // both flow through the compositor in one pass. `display_exposure_scale
      // = 2^snap_display_ev_total`; CLI paths never call SetCompositeExposure
      // so snap_display_ev_total stays at 0 → 2^0 = 1.0 → no behavior change.
      const float display_exposure_scale = std::pow(2.0f, snap_display_ev_total);
      float participating_p99 = 0.0f;
      if (!CompositeColorClassesLinear(*rc, snap_class_table, snap_composite_mode, display_exposure_scale,
                                       linear_rgb_scratch_, &participating_p99)) {
        continue;
      }
      // The display-time background goes on here: after every mode's exposure handling (all of
      // which finishes inside the call above) and before the sRGB stage below, on the pixels the
      // lens images and nowhere else. Same colour, same ordering and the same mask as the mono
      // path's own background, which is what makes toggling raypath colour leave the background
      // pixels untouched. All-zero (the default) contributes nothing.
      ApplyCompositeBackground(rc->VisibleMask(), snap_composite_background, linear_rgb_scratch_);
      CompositeResult cr;
      cr.renderer_id_ = 0;
      // Recover the renderer id from the mono result (RenderResult carries it).
      auto mono = rc->GetResult();
      if (auto* r = std::get_if<RenderResult>(&mono)) {
        cr.renderer_id_ = r->renderer_id_;
      }
      cr.w_ = rc->ImageWidth();
      cr.h_ = rc->ImageHeight();
      auto rgb = std::make_shared<std::vector<uint8_t>>();
      LinearRgbToSrgbU8(linear_rgb_scratch_, *rgb);
      cr.rgb_ = std::move(rgb);
      cr.p99_y_ = participating_p99;
      frame->composite_results_.push_back(std::move(cr));
    }
  }
  // Publish. The frame just built becomes the current one; the previous frame stays alive
  // exactly as long as somebody still holds it, and its pixels are freed only then.
  StorePublished(std::move(frame));
  return true;
}

// The single result entry point. Materializes a pending snapshot (if any),
// then hands out a share of the published frame.
//
// The two lifecycle fields are stamped HERE, from the live server state, rather than being
// taken from the frame as built: has_valid_data_ / epoch_ are answers to "what is true of
// this server now", not "what was true when these pixels were made". Stop() clears
// has_ever_consumed_ and CommitConfig bumps committed_epoch_ WITHOUT producing a new
// snapshot, and the old getters read both live on every call — the GUI poller's staleness
// gates are built on exactly that. Re-stamping onto a shallow copy (cheap: every pixel
// payload sits behind a shared_ptr) preserves it without making frames mutable.
std::shared_ptr<const ResultFrame> ServerImpl::AcquireResultFrame() {
  DoSnapshot();
  auto frame = LoadPublished();  // never null (see published_frame_)

  bool valid_data = false;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    valid_data = has_ever_consumed_;
  }
  const uint64_t epoch = committed_epoch_.load(std::memory_order_acquire);
  if (frame->has_valid_data_ == valid_data && frame->epoch_ == epoch) {
    return frame;
  }

  auto restamped = std::make_shared<ResultFrame>(*frame);
  restamped->has_valid_data_ = valid_data;
  restamped->epoch_ = epoch;
  for (auto& r : restamped->xyz_results_) {
    r.has_valid_data_ = valid_data;
    r.epoch_ = epoch;
  }
  return restamped;
}

// task-317: cheap O(1) live sim-ray-count read for the --benchmark drain-count
// poll loop. Unlike acquiring a result frame (which calls DoSnapshot -> RenderConsumer
// sRGB every poll — the render-per-poll root cause), this only reads the running
// StatsConsumer counter under consumer_mutex_. No snapshot, no render, no XYZ
// copy — so the poll thread does not perturb the throughput measurement nor
// starve drain-window closure.
// Sentinel note (inherited ambiguity, not new): 0 means BOTH "no StatsConsumer
// registered" AND "StatsConsumer present but no rays yet" — the same conflation
// the old have_stats?…:0 stats-getter path had. Fine for the benchmark's
// monotonic-progress read; a caller needing to distinguish a lifecycle error
// from a cold start should switch to std::optional<size_t>.
size_t ServerImpl::GetLiveSimRayCount() {
  std::lock_guard<TicketMutex> lock(consumer_mutex_);
  for (const auto& c : consumers_) {
    if (const auto* sc = dynamic_cast<const StatsConsumer*>(c.get())) {
      return sc->LiveSimRays();
    }
  }
  return 0;
}


void ServerImpl::Start() {
  ILOG_DEBUG(logger_, "Start: entry");
  auto t0 = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    if (state_.load() != ServerState::kStopped) {
      return;
    }

    work_started_ = false;
    sim_scene_cnt_ = 0;
    idle_logged_ = false;  // task-296.7: re-arm first-kIdle WARN for new run

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kRunning;
    }

    // Start queues BEFORE waking threads to avoid Get() on shutdown queues
    data_queue_->Start();
    scene_queue_->Start();

    start_generation_.fetch_add(1);
    state_.store(ServerState::kRunning);
  }
  start_cv_.notify_all();

  auto t1 = std::chrono::steady_clock::now();
  ILOG_DEBUG(logger_, "Start: done ({:.1f}ms)", std::chrono::duration<double, std::milli>(t1 - t0).count());
}


void ServerImpl::Stop() {
  ILOG_DEBUG(logger_, "Stop: entry");
  auto t0 = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    if (state_.load() != ServerState::kRunning) {
      return;
    }
    state_.store(ServerState::kStopped);
  }

  // Break work loops: shutdown queues so blocking Get() calls return immediately
  scene_queue_->Shutdown();
  data_queue_->Shutdown();
  scene_cv_.notify_one();

  // Stop simulators to break their inner Run() loops
  for (auto& s : simulators_) {
    s.Stop();
  }

  // Wait for all workers to finish their current work cycle.
  // Workers notify start_cv_ when active_workers_ reaches 0.
  {
    std::unique_lock<std::mutex> lk(start_mutex_);
    start_cv_.wait(lk, [this] { return active_workers_.load() == 0; });
  }
  auto t1 = std::chrono::steady_clock::now();

  // Analysis session: publish what the stop leaves behind. An analysis run has no way to
  // end other than its ray budget or this call, and its result is a histogram that is
  // worth reading at whatever point it was stopped — so the batches consumed since the
  // last poll must not be skipped by the snapshot_dirty_ reset below (which never
  // publishes them: DoSnapshot() is the only materialisation point, and a cleared dirty
  // flag makes it a no-op). What the frame carries is raypath_histogram_result_ — the
  // C API's `present` — not has_valid_data_: that flag is re-stamped live by
  // AcquireResultFrame from has_ever_consumed_, so after the reset below it reads false,
  // exactly as GetSimLifecycle() reads kIdle. A stopped analysis is idle with a readable
  // partial result; it is not "completed".
  // Render sessions are deliberately untouched: their Stop() contract (a stop reads as
  // idle with no data, doc/capi-lifecycle-architecture.md §7.1) is what the GUI's
  // re-simulate paths rest on. No lock is held here; DoSnapshot() takes its own two.
  if (mode_.load(std::memory_order_acquire) == SessionKind::kAnalysis) {
    DoSnapshot();
  }

  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    // Log profiling stats before clearing
    for (auto& c : consumers_) {
      if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
        rc->LogConsumeProfile();
      }
    }
    // Don't clear consumers_ here — CommitConfig decides whether to rebuild or reuse.
    // Consumers are destroyed either by CommitConfig (rebuild path) or ~ServerImpl().
    snapshot_dirty_ = false;
    // Resets data-valid flag: see doc/capi-lifecycle-architecture.md §7.1.
    has_ever_consumed_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_ = ServerStatus::kIdle;
  }

  auto t2 = std::chrono::steady_clock::now();
  ILOG_DEBUG(logger_, "Stop: done ({:.1f}ms total: drain {:.1f}ms, cleanup {:.1f}ms)",
             std::chrono::duration<double, std::milli>(t2 - t0).count(),
             std::chrono::duration<double, std::milli>(t1 - t0).count(),
             std::chrono::duration<double, std::milli>(t2 - t1).count());
}

ServerStatus ServerImpl::GetStatus() const {
  // status_ is the authoritative state. Return immediately for non-running states.
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_ != ServerStatus::kRunning) {
      return status_;
    }
  }

  // status_ == kRunning: check if work is actually complete.
  // During the startup window (threads spawned but GenerateScene hasn't started yet),
  // work_started_ is false — report kRunning to avoid false idle.
  if (!work_started_) {
    return ServerStatus::kRunning;
  }

  // Pipeline has started. Poll simulators to detect completion.
  bool any_busy = false;
  {
    std::lock_guard<std::mutex> lock(prod_mutex_);
    for (const auto& s : simulators_) {
      if (!s.IsIdle()) {
        any_busy = true;
        break;
      }
    }
  }

  // NOTE (out-of-scope for this predicate — left as-is): this reads sim_scene_cnt_
  // BEFORE scene_gen_active_, the opposite order PublishDrainedEpochIfSettled's term 4
  // deliberately uses (see that function's doc comment). count==0 observed first, then
  // GenerateScene enqueues more and finishes, scene_gen_active_==false observed second —
  // a real hole where this reports idle with those batches still unconsumed. Plausible
  // mechanism for the whole-dispatch-grain deficit LUMICE_GetDrainStatus exists to catch;
  // not fixed here because GetStatus()'s producer-side semantics are unchanged by design.
  if (any_busy || sim_scene_cnt_ > 0 || scene_gen_active_) {
    return ServerStatus::kRunning;
  }

  // task-296.7 diagnostic (DEBUG): capture the four-predicate state on the
  // first kIdle transition each run. Originally added at WARN to chase the
  // pre-fix sim_scene_cnt_ counter imbalance (discrete-spectrum 1-vs-N pairing,
  // see GenerateScene below); kept at DEBUG so a similar regression — or any
  // future "kIdle came early" report — has a single grep target instead of
  // needing fresh instrumentation. Once-per-run via idle_logged_ (re-armed in
  // Start()).
  if (!idle_logged_.exchange(true)) {
    ILOG_DEBUG(logger_,
               "GetStatus: first kIdle — any_busy={} sim_scene_cnt_={} scene_gen_active_={} "
               "work_started_={}",
               any_busy, sim_scene_cnt_.load(), scene_gen_active_.load(), work_started_.load());
  }
  return ServerStatus::kIdle;
}

bool ServerImpl::IsIdle() {
  return GetStatus() == ServerStatus::kIdle;
}

// Single authoritative lifecycle derivation (plan §2 decision 4). GetStatus()
// (status_ + four-predicate completion) stays the running/idle authority;
// has_ever_consumed_ upgrades a settled kIdle into kCompleted vs kIdle. The two
// locks are held in sequence, never nested (status_mutex_/prod_mutex_ inside
// GetStatus() release before consumer_mutex_ is taken), avoiding lock-order
// inversion (plan §5-R4).
SimLifecycle ServerImpl::GetSimLifecycle() const {
  ServerStatus st = GetStatus();
  if (st == ServerStatus::kRunning) {
    return SimLifecycle::kRunning;
  }
  // kIdle or kError: distinguish "drained clean" from "never produced / reset".
  // has_ever_consumed_ is read under consumer_mutex_ (same lock as its writes at
  // ConsumeData 895/921 and its reset at Stop 691).
  bool consumed = false;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    consumed = has_ever_consumed_;
  }
  return consumed ? SimLifecycle::kCompleted : SimLifecycle::kIdle;
}

SessionKind ServerImpl::GetSessionKind() const {
  return mode_.load(std::memory_order_acquire);
}

uint64_t ServerImpl::CommittedEpoch() const {
  return committed_epoch_.load(std::memory_order_acquire);
}

uint64_t ServerImpl::DrainedEpoch() const {
  return drained_epoch_.load(std::memory_order_acquire);
}

// =============== The drain signal ===============
// GetStatus()'s kIdle is a PRODUCER-side verdict: it says nothing about whether
// the consumer thread has drained data_queue_, so a reader that trusts it can
// freeze a PARTIAL accumulator total (measured: orientation_num 19616 vs 20000).
// This publishes the separate, consumer-side answer instead of overloading kIdle
// with a third meaning (which is the documented way kIdle got ambiguous already).
//
// THE PREDICATE, and why each term is here / absent. Read order is load-bearing;
// the three atomics are seq_cst (plain `=` / `+=` writes), so program order here
// IS the observed order.
//
//   1. state_ == kRunning — Stop()/Terminate() throw away whatever is still
//      queued (Queue::Shutdown swaps the deque out), so a stopping server must
//      never be reported drained. Read first: everything below is meaningless
//      once the queues are being torn down.
//   2. epoch snapshot, taken BEFORE the predicates, never after. If a
//      CommitConfig lands mid-check, a stamp taken afterwards would attribute
//      the OLD epoch's emptiness to the NEW epoch. Taken first, the worst case
//      is publishing an already-superseded epoch number, which the reader's
//      `drained == current` test simply rejects.
//   3. work_started_ — GenerateScene sets it AFTER scene_gen_active_ (see the
//      ordering comment at its assignment), so `work_started_ && !scene_gen_active_`
//      means "generation ran and finished", never "generation has not started".
//      Without it, the window between Start() and GenerateScene's first line
//      satisfies every other term.
//   4. !scene_gen_active_ — no further batch will be enqueued for this epoch,
//      hence no further sim_scene_cnt_ increment. MUST be read BEFORE the count
//      (term 5): GetStatus() reads them the other way round, and that order has
//      a real hole — count==0 observed first, GenerateScene then enqueues more
//      and finishes, scene_gen_active_==false observed second, and the verdict
//      is "idle" with those batches still unconsumed. That is a plausible
//      mechanism for the whole-dispatch-grain deficit this task exists to fix.
//   5. sim_scene_cnt_ <= 0 — the producer++/consumer-- counter, incremented per
//      SimData a batch WILL yield (GenerateScene) and decremented only after
//      that SimData has been handed to the consumers (ConsumeData). With term 4
//      already true, "no outstanding credit" is exactly "every SimData this
//      epoch will ever produce has been consumed". Note this also covers the
//      device-fused third-clock accumulation window: an un-drained xyz_win_ holds
//      xyz_win_.calls credits that DrainDeviceXyz only settles when it emplaces
//      (simulator.cpp: sim_scene_credit_ = xyz_win_.calls), so a pending window
//      keeps this term false. No separate Simulator-side pending-work accessor
//      is needed; adding one would be a second copy of a fact this counter
//      already carries.
//   6. data_queue_->Empty() — strictly redundant given term 5, kept as cheap
//      defence in depth because term 5 rests on a counter that HAS drifted
//      before (the 1-vs-N credit imbalance fixed by counting SimData rather
//      than SimBatch — see GenerateScene's kNsimdataPerBatch). If it goes wrong
//      again, this term degrades the signal to "late" instead of "wrong". It
//      cannot deadlock the signal: only the consumer can leave the queue
//      non-empty, and the consumer re-runs this check after every item it takes.
//
//      SCOPE NOTE — this signal is correct, but it does not treat the failure
//      mode that produced the whole-dispatch-grain shortfalls quoted above.
//      Term 6's "late instead of wrong" degradation, and this predicate as a
//      whole, are about sim_scene_cnt_ settling at a wrong PERSISTENT value, or
//      about a batch that is still queued when someone asks. Neither describes a
//      producer that DISCARDS a batch's data while still balancing the counter:
//      that batch leaves the queue and the counter still nets to zero, so every
//      term here reads exactly as if nothing were wrong and the epoch is
//      reported drained — correctly, since nothing is outstanding; the data is
//      simply gone. That case is guarded elsewhere: AccountThenPublishBatch
//      (scene_batch_publish.hpp) makes the count reflect an in-flight batch
//      before the batch is reachable, and ConsumeData no longer gates
//      consumption on the count at all (its generation_ check is what discards).
//      Term 4's "plausible mechanism for the whole-dispatch-grain deficit" was
//      written before that mechanism was pinned down; the read-order hole it
//      describes is real and still worth the ordering it prescribes here, but it
//      is not what the observed shortfalls turned out to be.
//
// THE OTHER DIRECTION — a CommitConfig landing BEFORE this call reads `epoch`,
// not mid-check. Term 2's note above only argues the safe case (CommitConfig
// lands between the epoch read and the rest of the predicate). The mirror
// worry looks real from this function alone: could a delayed call for epoch X
// — e.g. GenerateScene suspended between clearing scene_gen_active_ (below)
// and reaching this call — execute AFTER a CommitConfig for X+1 has already
// bumped committed_epoch_, reading epoch==X+1 while work_started_/
// scene_gen_active_/sim_scene_cnt_ still describe X's just-settled state, and
// wrongly stamp drained_epoch_ at X+1? It cannot: CommitConfig's Stop() (see
// Stop()'s active_workers_==0 wait, this file) does not return — so
// committed_epoch_ cannot advance — until every RunPersistentLoop worker,
// including whichever thread is mid-call here for X, has returned from its
// work function. GenerateScene's call to this function is its LAST action
// before returning (see the call site at its exit below), so a suspended
// GenerateScene has not yet decremented active_workers_ and Stop() blocks on
// exactly it. The call therefore always completes — reading epoch==X, not
// X+1 — before Stop() can return and CommitConfig can bump the epoch.
//
// DELIBERATELY ABSENT: "all simulators are idle" (Simulator::IsIdle). It is
// implied by term 5, and adding it would introduce a livelock — the consumer can
// decrement the last credit while the worker has not yet reached its
// `idle_ = true` store, and since this check only re-runs when another item is
// consumed, there would be no later call to correct the verdict.
//
// Two call sites, because either thread can be the one that completes the last
// transition: the consumer after its final decrement, and GenerateScene after it
// clears scene_gen_active_. Whichever runs last sees the settled state.
void ServerImpl::PublishDrainedEpochIfSettled() {
  if (state_.load() != ServerState::kRunning) {
    return;
  }
  const uint64_t epoch = committed_epoch_.load(std::memory_order_acquire);
  if (!work_started_.load() || scene_gen_active_.load()) {
    return;
  }
  if (sim_scene_cnt_.load() > 0) {
    return;
  }
  if (!data_queue_->Empty()) {
    return;
  }
  // Monotonic max-store: the two call sites can race, and an older epoch must
  // never overwrite a newer one.
  uint64_t prev = drained_epoch_.load(std::memory_order_relaxed);
  while (prev < epoch &&
         !drained_epoch_.compare_exchange_weak(prev, epoch, std::memory_order_release, std::memory_order_relaxed)) {
  }
  ILOG_DEBUG(logger_, "PublishDrainedEpochIfSettled: epoch {} drained", epoch);
}


#define CHECK_STOP                                           \
  if (state_.load() != ServerState::kRunning) { /* NOLINT */ \
    break;                                                   \
  }


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void ServerImpl::ConsumeData() {
  ILOG_DEBUG(logger_, "ConsumeData: entry");
  bool first_consume_logged = false;
  // task-268.4 commit-granularity knob: backend exit-seam SimData are chunked
  // into kCommitCap-sized slices before Consume() so GUI snapshot cadence is
  // independent of the LUMICE_DISPATCH_RAY_NUM dispatch granularity. Falls
  // back to the historical LUMICE_BATCH_RAY_NUM env name when set so existing
  // scripts keep their commit cadence unchanged. Legacy CPU-path SimData
  // (non-zero ray_seg_count_) bypass the chunker — the consumer projects via
  // per-ray indices into its buffer, which cannot be sliced without recomputing
  // indices.
  // Commit granularity + its legacy LUMICE_BATCH_RAY_NUM fallback and one-time
  // deprecation WARN are all resolved inside util/env_knobs (the single
  // registered getenv site; see doc/env-var-policy.md). Re-resolved on each
  // ConsumeData entry (was a process-once static lambda) — intentional, mirrors
  // kDispatchCap's "NOT static" choice so a server reconstructed in-process picks
  // up the current env; env_knobs' once_flags keep the log line single.
  const size_t kCommitCap = env::CommitRayNum(logger_, kDefaultRayNum);
  while (true) {
    CHECK_STOP
    auto sim_data = data_queue_->Get();
    // Interruption sentinel (scrum-258.1 Step 3 — 协议固化):
    // a default-constructed SimData (queue shutdown / simulator early exit)
    // has ray_seg_count_ == 0 AND root_ray_count_ == 0. Discriminating on
    // root_ray_count_ correctly distinguishes the sentinel from:
    //   - legacy CPU path: ray_seg_count_ non-zero;
    //   - backend exit-seam path: ray_seg_count_ zero + outgoing_d_/w_ populated
    //     (or empty for a zero-exit batch) but root_ray_count_ = ray_num > 0.
    // The earlier is_backend_path_ key falsely flagged exit-seam batches as
    // sentinels (no ray segments + is_backend_path_ false), deadlocking the
    // consumer; root_ray_count_ is the protocol-level invariant for a real
    // produced batch and works uniformly across both paths.
    if (sim_data.ray_seg_count_ == 0 && sim_data.root_ray_count_ == 0) {
      // Simulation is interrupted.
      break;
    }
    CHECK_STOP

    ILOG_TRACE(logger_, "ConsumeData: get data: {}", fmt::ptr(&sim_data));

    // Bookkeeping sentinel only — does NOT gate consumption. AccountThenPublishBatch
    // (scene_batch_publish.hpp) credits sim_scene_cnt_ before a batch is ever published, so a
    // real, current-generation batch should never observe a non-positive count here while
    // state_ == kRunning; the generation_ check below is what actually discards stale data.
    // If this fires, sim_scene_cnt_ has drifted — check GenerateScene's kNsimdataPerBatch
    // pairing first, since that is the one precedent for this counter going wrong.
    if (sim_scene_cnt_.load() <= 0) {
      ILOG_WARN(logger_, "ConsumeData: sim_scene_cnt_={} non-positive while consuming a batch (root_ray_count={})",
                sim_scene_cnt_.load(), sim_data.root_ray_count_);
    }

    // Generation check: discard batches from outdated configs
    if (sim_data.generation_ != scene_generation_.load()) {
      ILOG_DEBUG(logger_, "ConsumeData: discarding batch (generation {} != {})", sim_data.generation_,
                 scene_generation_.load());
    } else {
      // This batch belongs to the current
      // committed config (generation matches) — publish its GPU color-degrade
      // tally to the atomics the C API poll path reads. OVERWRITE, not +=:
      // the value is a config constant, identical on every batch, so the last
      // writer simply refreshes it. GPU-only; CPU SimData carries all-zeros.
      last_color_symmetry_group_overflow_.store(sim_data.color_degrade_counts_.symmetry_group_overflow,
                                                std::memory_order_release);
      last_color_or_summand_overflow_.store(sim_data.color_degrade_counts_.or_summand_overflow,
                                            std::memory_order_release);
      last_color_class_overflow_.store(sim_data.color_degrade_counts_.color_class_overflow, std::memory_order_release);
      // 0-exit-batch guard: backend exit-seam path may emplace a SimData with
      // outgoing_d_ empty and ray_seg_count_ zero when all rays were filtered/absorbed
      // (e.g. selective BD filter). Skip consumer projection so we don't
      // dirty snapshot_dirty_/has_ever_consumed_ on a black contribution, but
      // STILL fall through to sim_scene_cnt_-- below — that decrement is the
      // counter invariant paired with GenerateScene's ++ (see simulator.cpp
      // exit-seam: empty Emplace must reach the consumer's --). Do not move
      // the -- inside this branch.
      // S1 device-fused route: the backend accumulates XYZ on-device and
      // emplaces a SimData carrying xyz_pixel_data_ with outgoing_d_ empty AND
      // ray_seg_count_ zero. That payload IS renderable — without this clause it would
      // be misclassified as a 0-exit black batch and the consumer skipped,
      // dropping the entire device-fused image (zero output, parity corr=0).
      bool has_renderable =
          !sim_data.outgoing_d_.empty() || sim_data.ray_seg_count_ != 0 || !sim_data.xyz_pixel_data_.empty();
      if (has_renderable) {
        auto t_lock0 = std::chrono::steady_clock::now();
        std::lock_guard<TicketMutex> lock(consumer_mutex_);
        auto t_lock1 = std::chrono::steady_clock::now();
        // WARNING for anyone adding a SimData field: the chunk below is a
        // hand-rolled field-by-field copy, NOT a copy of sim_data, so a new
        // field silently arrives at the consumers as a default-constructed 0
        // on this path — and ONLY on this path, which is what makes it hard to
        // see (the legacy CPU route hands the whole SimData over intact, so it
        // keeps working while the backend route quietly reports nothing).
        // Every stats field needs a line here, on the correct side of the
        // accumulated/overwritten split below.
        //
        // Chunk by kCommitCap on the backend exit-seam path
        // (outgoing_d_ populated AND ray_seg_count_ zero). Only the FIRST chunk
        // carries root_ray_count_ + the stochastic crystal-draw count —
        // StatsConsumer accumulates both, so spreading them across chunks
        // would N×-count and break the stats invariant. (The deterministic
        // crystal count is the opposite case; see the chunk fill below.)
        // Legacy CPU SimData (ray_seg_count_ non-zero) are
        // delivered whole because their consumers project via per-ray
        // indices into that batch, which has no clean sub-batch slice.
        // NOLINTNEXTLINE(readability-identifier-naming) — local const flag, snake_case is project style for
        // variables.
        const bool is_exit_seam_path = sim_data.ray_seg_count_ == 0 && !sim_data.outgoing_d_.empty();
        if (!is_exit_seam_path) {
          for (auto& c : consumers_) {
            c->Consume(sim_data);
          }
        } else {
          size_t exit_count = sim_data.outgoing_w_.size();
          size_t emitted = 0;
          do {
            size_t chunk_count = std::min(kCommitCap, exit_count - emitted);
            SimData chunk;
            chunk.curr_wl_ = sim_data.curr_wl_;
            chunk.generation_ = sim_data.generation_;
            // ACCUMULATED stats fields go on the first chunk only; the rest
            // carry 0 / empty so StatsConsumer's running sums land on the same
            // totals a single whole-Consume call would yield.
            if (emitted == 0) {
              chunk.root_ray_count_ = sim_data.root_ray_count_;
              // Same side of the split as root_ray_count_, for the same
              // reason: RenderConsumer adds it up, so repeating it on every
              // chunk would multiply the normalization denominator by the
              // chunk count and darken the image in proportion to the commit
              // grain. Omitting it entirely is the other failure — a zero
              // denominator on this path alone, i.e. a black image only when
              // the exit-seam backend is in use.
              chunk.emitted_energy_ = sim_data.emitted_energy_;
              chunk.stochastic_crystal_sample_count_ = sim_data.stochastic_crystal_sample_count_;
              chunk.stochastic_orientation_sample_count_ = sim_data.stochastic_orientation_sample_count_;
              chunk.crystals_ = sim_data.crystals_;
              chunk.crystal_axis_dists_ = sim_data.crystal_axis_dists_;
            }
            // OVERWRITTEN stats fields must go on EVERY chunk — the consumer
            // stores rather than adds, so leaving this at 0 on the trailing
            // chunks would have the last one wipe the value the first chunk
            // published. The inverse of the rule above, for the inverse
            // aggregation.
            chunk.deterministic_crystal_count_ = sim_data.deterministic_crystal_count_;
            chunk.deterministic_orientation_count_ = sim_data.deterministic_orientation_count_;
            if (chunk_count > 0) {
              chunk.outgoing_d_.assign(
                  sim_data.outgoing_d_.begin() + static_cast<std::ptrdiff_t>(emitted) * 3,
                  sim_data.outgoing_d_.begin() + static_cast<std::ptrdiff_t>(emitted + chunk_count) * 3);
              // Invariant: outgoing_w_ is sliced to exactly chunk_count, so
              // chunk.outgoing_w_.size() == chunk_count is the consumer's
              // per-chunk outgoing-ray count (it reads .size(), see render.cpp).
              chunk.outgoing_w_.assign(
                  sim_data.outgoing_w_.begin() + static_cast<std::ptrdiff_t>(emitted),
                  sim_data.outgoing_w_.begin() + static_cast<std::ptrdiff_t>(emitted + chunk_count));
              // DR-3: per-ray wavelength must be sliced in
              // lock-step with outgoing_w_ — omitting it here left chunked
              // SimData with empty outgoing_wl_, so the consumer fell back to
              // per-batch curr_wl_ and the CMF decoupled from the per-ray SPD
              // weight (flat / illuminant-independent color). Empty for CPU /
              // discrete-wl paths, where the fallback is correct.
              if (!sim_data.outgoing_wl_.empty()) {
                chunk.outgoing_wl_.assign(
                    sim_data.outgoing_wl_.begin() + static_cast<std::ptrdiff_t>(emitted),
                    sim_data.outgoing_wl_.begin() + static_cast<std::ptrdiff_t>(emitted + chunk_count));
              }
              if (sim_data.exit_records_.size() >= emitted + chunk_count) {
                chunk.exit_records_.assign(
                    sim_data.exit_records_.begin() + static_cast<std::ptrdiff_t>(emitted),
                    sim_data.exit_records_.begin() + static_cast<std::ptrdiff_t>(emitted + chunk_count));
              }
            }
            for (auto& c : consumers_) {
              c->Consume(chunk);
            }
            emitted += chunk_count;
          } while (emitted < exit_count);
        }
        auto t_consume = std::chrono::steady_clock::now();
        snapshot_dirty_ = true;
        has_ever_consumed_ = true;
        auto lock_us = std::chrono::duration<double, std::micro>(t_lock1 - t_lock0).count();
        auto consume_us = std::chrono::duration<double, std::micro>(t_consume - t_lock1).count();
        ILOG_DEBUG(logger_, "ConsumeData: batch rays={} outgoing={} lock={:.0f}us consume={:.0f}us",
                   sim_data.ray_seg_count_, sim_data.outgoing_w_.size(), lock_us, consume_us);
        if (!first_consume_logged) {
          ILOG_INFO(logger_, "ConsumeData: first batch consumed ({} ray segments)", sim_data.ray_seg_count_);
          first_consume_logged = true;
        }
      } else {
        // 0-exit batch on the exit-seam path (all rays filtered/absorbed →
        // outgoing_d_ empty AND ray_seg_count_ zero). A batch that ran to completion
        // with a legitimately all-black result is still *valid data*: the
        // simulation converged, the answer is just zero intensity. We flip
        // has_ever_consumed_ so the frame's xyz results report
        // has_valid_data=true, and dirty the snapshot so PrepareSnapshot
        // produces a clean zero frame (without this, an all-black simulation
        // — e.g. an impossible raypath filter — never sets has_valid_data, so
        // the buffered poller waits for "valid data" forever and times out at
        // 600s). The legacy CPU path never hit this because its ray_seg_count_
        // is always non-zero, so has_renderable stayed true; the exit-seam path
        // is the first to surface it. See doc/capi-lifecycle-architecture.md
        // ("zero-output completion").
        //
        // The batch contributes no pixels, but it did emit rays, and the
        // renderer's normalization divides by emitted energy — so it must be
        // consumed for its bookkeeping even though it has no image to add.
        // (It used to be dropped whole, which was right while the denominator
        // was the LANDED weight: a batch that landed nothing owed nothing.
        // Under an absolute scale that same drop would leave the denominator
        // counting only the batches that survived their filter, re-brightening
        // a filtered scene back to the unfiltered look — the exact
        // content-dependence the absolute scale exists to remove, and worst
        // where filtering is strictest.) So hand the consumers an
        // accounting-only SimData: no rays, no outgoing data, no image
        // contribution, just the counters. root_ray_count_ and the sample
        // counts ride along because the legacy CPU path already delivers them
        // for its all-filtered batches — leaving them out here would keep the
        // stats disagreeing across backends for the same scene. A black batch
        // still cannot bias the image: RenderConsumer accumulates nothing from
        // an empty payload.
        SimData accounting;
        accounting.curr_wl_ = sim_data.curr_wl_;
        accounting.generation_ = sim_data.generation_;
        accounting.root_ray_count_ = sim_data.root_ray_count_;
        accounting.emitted_energy_ = sim_data.emitted_energy_;
        accounting.stochastic_crystal_sample_count_ = sim_data.stochastic_crystal_sample_count_;
        accounting.stochastic_orientation_sample_count_ = sim_data.stochastic_orientation_sample_count_;
        accounting.deterministic_crystal_count_ = sim_data.deterministic_crystal_count_;
        accounting.deterministic_orientation_count_ = sim_data.deterministic_orientation_count_;
        {
          std::lock_guard<TicketMutex> lock(consumer_mutex_);
          for (auto& c : consumers_) {
            c->Consume(accounting);
          }
        }
        snapshot_dirty_ = true;
        has_ever_consumed_ = true;
        ILOG_DEBUG(logger_, "ConsumeData: 0-exit batch (all filtered) — marking valid_data, zero snapshot");
      }
    }
    // scrum-312 third-clock: a windowed device-fused SimData stands in for N
    // per-wavelength calls (sim_scene_credit_ == N); all other paths credit 1.
    // Decrement by the credit to keep the GenerateScene ++ / ConsumeData --
    // invariant balanced regardless of drain windowing.
    sim_scene_cnt_ -= static_cast<int>(sim_data.sim_scene_credit_);
    if (sim_scene_cnt_ < kMaxSceneCnt / 2) {
      scene_cv_.notify_one();
    } else if (gpu_route_ && !fallback_queue_invalidated_.load(std::memory_order_acquire) && !ReadBackendActive()) {
      // The queue is still deep, so the throttle above would keep the producer parked for
      // as long as the backlog takes to render — precisely the window GenerateScene's
      // batch invalidation exists to cut short. Wake it now instead. Repeated on every
      // consumed SimData rather than sent once, deliberately: neither the store this reads
      // nor this notify is synchronised with the producer's predicate evaluation, so a
      // single notify can fall into that window and be lost. It stops the moment the
      // producer reports the invalidation done, which is why that flag is read here — past
      // that point this branch would be a wakeup per batch that the producer can only go
      // back to sleep on.
      scene_cv_.notify_one();
    }
    // The consumer is one of the two threads that can complete the
    // last transition into "this epoch is fully drained" — publish from here,
    // immediately after the decrement that may have brought the credit to zero.
    // Placed after the CHECK_STOP-free decrement and before the loop's own
    // CHECK_STOP so a Stop racing us is caught by the state_ test inside.
    PublishDrainedEpochIfSettled();

    CHECK_STOP
  }
  ILOG_DEBUG(logger_, "ConsumeData exit");
}


void ServerImpl::GenerateScene() {
  ILOG_DEBUG(logger_, "GenerateScene entry");
  auto gen_start = std::chrono::steady_clock::now();
  scene_gen_active_ = true;  // Must be set before work_started_ to close the ordering window
  work_started_ = true;
  bool first_batch_logged = false;

  std::shared_ptr<const SceneConfig> scene;
  std::shared_ptr<const std::vector<RenderConfig>> renders;
  std::shared_ptr<const RaypathColorConfig> raypath_color;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    scene = active_scene_;
    renders = active_renders_;
    raypath_color = active_raypath_color_;
    generation = scene_generation_.load();
  }
  // task-268.4 commit↔batch decoupling: two independent knobs.
  //
  // LUMICE_DISPATCH_RAY_NUM (kDispatchCap): per-SimBatch ray count fed to the
  //   backend (GPU dispatch granularity). Higher amortizes Metal kernel
  //   launch overhead; tune against legacy crossover (~512).
  // LUMICE_COMMIT_RAY_NUM (kCommitCap): SimData-to-consumer commit granularity
  //   inside ConsumeData. Smaller commits keep GUI snapshot cadence fine
  //   regardless of dispatch size, so "feed GPU big, refresh UI small"
  //   becomes a single tunable.
  //
  // Backward compat: LUMICE_BATCH_RAY_NUM (historical "batch = commit
  // granularity" semantics) is honoured as a fallback for kCommitCap only;
  // dispatch granularity defaults to kDefaultRayNum unless LUMICE_DISPATCH_RAY_NUM
  // is explicitly set. Both env knobs are read through util/env_knobs (the single
  // registered getenv site; see doc/env-var-policy.md).
  // scrum-268.6: backend-aware dispatch default. Metal single-engine defaults
  // to a large dispatch (kDefaultMetalDispatchRayNum) to saturate the GPU;
  // CPU/legacy keeps the small kDefaultRayNum. An explicit LUMICE_DISPATCH_RAY_NUM
  // always wins. NOT static: a server reconstructed on a GUI backend toggle must
  // re-resolve the default for the new backend (commit↔batch decoupling, 268.4).
  // scrum-306.2: CUDA's dispatch optimum (262144) is much higher than Metal's
  // (32768) once the dead exit buffer is capped — select per backend so each GPU
  // route gets its own measured plateau. ResolveGpuRoute is env-override-aware
  // (LUMICE_TRACE_BACKEND wins over preferred_backend_, so keying on the latter
  // misses the --benchmark/CLI env path). Metal is Apple-only; CUDA is the only
  // GPU route on a non-Apple CUDA build — so there a true GPU route IS CUDA.
  const BackendKind kPref = preferred_backend_.load(std::memory_order_acquire);
  // The analysis session forces the CPU route here AND in the Simulator's CreateBackend
  // (through the same session flag) — the two halves of one routing decision, kept in
  // step by passing the same fact to both.
  const bool kAnalysis = mode_.load(std::memory_order_acquire) == SessionKind::kAnalysis;
  const bool kGpuRoute = ResolveGpuRoute(kPref, logger_, /*force_cpu=*/kAnalysis);
#if defined(LUMICE_CUDA_ENABLED) && !defined(__APPLE__)
  const bool kIsCudaRoute = kGpuRoute;
#else
  const bool kIsCudaRoute = false;  // Apple GPU route is Metal; non-CUDA build has none
#endif
  const size_t kDefaultDispatch = kIsCudaRoute ? kDefaultCudaDispatchRayNum :
                                  kGpuRoute    ? kDefaultMetalDispatchRayNum :
                                                 kDefaultRayNum;
  const size_t kDispatchCap = env::DispatchRayNum(logger_, kDefaultDispatch);
  const size_t kBatchCap = kDispatchCap;  // local alias for the loop below
  // The ctor-time route and this live re-derivation ask two different
  // questions off the same ResolveGpuRoute (see gpu_route_'s declaration). They
  // cannot disagree today; say so out loud if they ever do, rather than letting one
  // silently size the batches while the other decides whether a fallback happened.
  if (kGpuRoute != gpu_route_ && !kAnalysis) {
    // (An analysis session on a GPU-built server differs by design, not by drift: it is
    // still sized single-worker, and the forced CPU route is exactly the point.)
    ILOG_WARN(logger_,
              "GenerateScene: live gpu_route ({}) disagrees with the construction-time route ({}); this server was "
              "sized for the latter, so dispatch grain and the fallback signal are now keyed off different routes",
              kGpuRoute, gpu_route_);
  }

  // task-296.7: sim_scene_cnt_ semantic fix — count SimData (not SimBatch).
  // The simulator emits one SimData per wavelength inside SimulateOneWavelength*
  // (1 for illuminant; N for discrete spectrum lists). ConsumeData decrements
  // per SimData. Incrementing by 1 here (the historical behaviour) created a
  // 1-vs-N pairing imbalance on discrete-spectrum configs: sim_scene_cnt_ went
  // negative as the consumer drained N - 1 "extra" SimData per batch, GetStatus
  // saw the predicate fall to false, and CLI single-snapshot rendering reported
  // kIdle while 4/5 of the wavelengths' batches still got skip-consumed (via a
  // `if (sim_scene_cnt_ > 0)` gate ConsumeData no longer has — that gate has since
  // been removed, so a drifted counter can no longer cost data; it now only costs
  // an early "drained" verdict, and trips ConsumeData's ILOG_WARN sentinel).
  // Resolve by incrementing here by the same N the simulator will emplace, so the
  // counter matches the consumer's per-SimData decrement. The scene is captured under scene_mutex_
  // above and immutable for this GenerateScene invocation, so N is computed
  // once. Throttle/notify thresholds (kMaxSceneCnt, kMaxSceneCnt/2) now refer
  // to in-flight SimData, which is also the right quantity for memory control.
  const size_t kNsimdataPerBatch = std::holds_alternative<std::vector<WlParam>>(scene->light_source_.spectrum_) ?
                                       std::get<std::vector<WlParam>>(scene->light_source_.spectrum_).size() :
                                       static_cast<size_t>(1);

  // task-323: ray_num semantic unification. At this ingest point scene->ray_num_ is the TOTAL rays
  // across all wavelengths; the simulator loop below consumes a PER-WAVELENGTH budget. Keep the two
  // quantities in distinctly-named variables (avoids the "same name carrying two dimensions" trap):
  // per_wl = ceil(total / N_wl) guarantees at least `total` rays are traced across the spectrum.
  // Illuminant (N_wl=1) is the identity transform. kInfSize is passed through unchanged.
  // An analysis session with a budget of its own (RaypathAnalysisRequest::ray_num_) reads that
  // here, at the ONE ingest point, in the same total-across-wavelengths unit — the scene's own
  // ray_num_ is left as committed, so the render that follows traces what the document says.
  size_t total_ray_num = scene->ray_num_;
  if (kAnalysis && analysis_ray_num_overridden_.load(std::memory_order_acquire)) {
    total_ray_num = analysis_ray_num_override_.load(std::memory_order_acquire);
  }
  // A hand-written discrete config with total < N_wl asks for fewer rays than wavelengths; ceil still
  // yields >=1 per wavelength, so the actual total is rounded UP to N_wl. Warn so the author of a bad
  // config notices the bump (the GUI never hits this — total is always >> the wavelength count).
  if (total_ray_num != kInfSize && kNsimdataPerBatch > 1 && total_ray_num < kNsimdataPerBatch) {
    ILOG_WARN(logger_,
              "GenerateScene: ray_num ({}) < spectrum wavelength count ({}); rounding up to 1 ray/wavelength "
              "(actual total {} > requested {})",
              total_ray_num, kNsimdataPerBatch, kNsimdataPerBatch, total_ray_num);
  }
  size_t per_wl_ray_num = total_ray_num;
  if (per_wl_ray_num != kInfSize) {
    per_wl_ray_num = PerWavelengthRayNum(per_wl_ray_num, kNsimdataPerBatch);
  }
  size_t committed_num = 0;
  // One-shot, because backend_active_ only ever goes true -> false inside one Run():
  // the batches queued at the GPU grain are invalidated the first time the backend is
  // seen gone, and never again. Re-armed here rather than in Start() because this is the
  // only writer, and this is the entry point of the Run() the flag describes.
  fallback_queue_invalidated_.store(false, std::memory_order_release);
  while (per_wl_ray_num == kInfSize || committed_num < per_wl_ray_num) {
    const bool backend_active = ReadBackendActive();
    // Shrinking the grain only sizes the batches queued FROM HERE ON, and by the time a
    // fallback is noticed the queue is already full of batches sized for the GPU
    // (kMaxSceneCnt counts batches, not rays, so the backlog at the CUDA grain is
    // ~128 x 262144 rays — tens of seconds of single-wavelength frames on the legacy CPU
    // path, i.e. the whole of the symptom this shrink exists to remove). Those batches are
    // stale in size, not in content: drop them and re-emit the same ray budget at the new
    // grain. Under ray_num == "infinite" (the GUI default) they carry no budget at all,
    // just "trace N more rays", so nothing is lost either way.
    if (kGpuRoute && !backend_active && !fallback_queue_invalidated_.load(std::memory_order_acquire)) {
      fallback_queue_invalidated_.store(true, std::memory_order_release);
      size_t dropped_batches = 0;
      const size_t refunded_rays = DiscardQueuedBatchesThenRefund(sim_scene_cnt_, static_cast<int>(kNsimdataPerBatch),
                                                                  *scene_queue_, &dropped_batches);
      // Hand the dropped batches' budget back to the ledger, or a finite ray_num run
      // would trace exactly this many rays fewer than the config asked for. The clamp
      // cannot bind: every batch that can be in the queue was enqueued by this same loop
      // and added to committed_num at the bottom of its own iteration, so the refund is
      // always a subset of what was committed. It is here because the underflow it would
      // guard against is silent (size_t) and would read as an enormous outstanding budget.
      committed_num -= std::min(committed_num, refunded_rays);
      ILOG_WARN(logger_,
                "GenerateScene: backend dropped mid-run; discarded {} queued batches ({} rays refunded to the "
                "budget) sized for the GPU grain, re-emitting at the legacy grain",
                dropped_batches, refunded_rays);
    }
    // Re-decide the grain every iteration. A mid-Run() BackendUnavailableError
    // leaves the GPU route feeding the legacy CPU path, which samples ONE host
    // wavelength per batch — at the GPU grain that is one wavelength per 262144 rays,
    // i.e. seconds of strongly saturated single-colour frames. Named locals rather than
    // inline constants because EffectiveDispatchCap's last two parameters are both
    // size_t and a swap would not be a compile error.
    const size_t nominal_cap = kBatchCap;
    const size_t fallback_cap = kDefaultRayNum;
    const size_t iter_cap = EffectiveDispatchCap(kGpuRoute, backend_active, nominal_cap, fallback_cap);
    size_t batch_ray_num = std::min(iter_cap, per_wl_ray_num - committed_num);
    AccountThenPublishBatch(sim_scene_cnt_, static_cast<int>(kNsimdataPerBatch), *scene_queue_,
                            SimBatch{ batch_ray_num, scene, generation, renders, raypath_color });
    if (!first_batch_logged) {
      ILOG_INFO(logger_, "GenerateScene: first batch enqueued at {:.1f}ms after start",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - gen_start).count());
      first_batch_logged = true;
    }

    ILOG_TRACE(logger_, "GenerateScene: put a scene: ray({}/{}, {})", batch_ray_num, per_wl_ray_num, committed_num);
    CHECK_STOP

    if (sim_scene_cnt_ >= kMaxSceneCnt) {
      ILOG_DEBUG(logger_, "GenerateScene: too many scenes generated. wait for consumer");
      std::unique_lock<std::mutex> lock(scene_mutex_);
      // The third term is what makes the invalidation above reachable in time. Without it
      // this wait only ends when the consumer has drained the queue to kMaxSceneCnt/2 —
      // which, once every batch runs on the legacy CPU path at the GPU grain, is the
      // backlog itself, rendered in full before the producer ever wakes.
      // ReadBackendActive() takes prod_mutex_ while scene_mutex_ is held; that nesting is
      // safe in one direction only — prod_mutex_ is a leaf (its holders call nothing but
      // Simulator accessors) and nothing takes scene_mutex_ under it. Keep it that way.
      scene_cv_.wait(lock, [this, kGpuRoute]() {
        return state_.load() != ServerState::kRunning || sim_scene_cnt_ < kMaxSceneCnt ||
               (kGpuRoute && !fallback_queue_invalidated_.load(std::memory_order_acquire) && !ReadBackendActive());
      });
      ILOG_DEBUG(logger_, "GenerateScene: continue to generate scenes.");
    }
    CHECK_STOP
    // Advance by what was ACTUALLY queued, not by the nominal cap. Required
    // now that iter_cap can shrink mid-loop; equivalent to the former `+= kBatchCap` for
    // every run where iter_cap stays constant (the only case that exists when no fallback
    // happens): all iterations but the last have batch_ray_num == iter_cap == kBatchCap,
    // and on the last one the old code's overshoot past per_wl_ray_num was never read
    // again — the loop condition is already false either way, so the exit point, the
    // number of batches and each batch's size are all unchanged (AC4). Under
    // per_wl_ray_num == kInfSize the loop condition ignores committed_num entirely.
    committed_num += batch_ray_num;
    ILOG_TRACE(logger_, "GenerateScene: finish wl");
  }
  scene_gen_active_ = false;  // All exit paths (normal + CHECK_STOP break) converge here
  // The other half of the drain publication. If the consumer already
  // drained everything before this store, its own call saw scene_gen_active_
  // still true and declined — this call is the one that completes the verdict.
  // A CHECK_STOP break also lands here, which is why PublishDrainedEpochIfSettled
  // re-checks state_ (a stopped server discards its queue; it is not "drained").
  PublishDrainedEpochIfSettled();
  ILOG_DEBUG(logger_, "GenerateScene exit");
}


// =============== ServerImpl::SetPreferredBackend ===============
// preferred_backend_ is currently write-only: it is the authoritative cache for
// a future simulator-rebuild path (today simulators_ is built once in the ctor
// and never rebuilt, so the per-Simulator atomics below are the live source of
// truth). The cache store intentionally precedes prod_mutex_; any future reader
// outside this lock must treat it as "may lead the simulators_ state" and add
// its own ordering, or move the store inside the lock.
void ServerImpl::SetPreferredBackend(BackendKind backend) {
  preferred_backend_.store(backend, std::memory_order_release);
  std::lock_guard<std::mutex> lock(prod_mutex_);
  for (auto& s : simulators_) {
    s.SetPreferredBackend(backend);
  }
}


// =============== ServerImpl::SetRaypathColors ===============
// Display-time update of the color-class appearance without touching the simulation
// lifecycle. Runs entirely under consumer_mutex_: color/visible/solo/z_order are appearance
// fields (NeedsRebuild ignores them, so a full-rebuild is neither needed nor useful);
// active_composite_mode_ is a per-frame plumbing input read under the same lock by
// DoSnapshot. NO Stop/Start, NO scene_generation_/committed_epoch_ bump, NO consumers_
// rebuild, NO scene_mutex_. AC2/AC3 requirement.
Error ServerImpl::SetRaypathColors(const ColorClassDisplay* classes, int class_count, const int* z_order,
                                   CompositeMode mode) {
  if (class_count < 0 || (class_count > 0 && classes == nullptr)) {
    return Error::InvalidValue("SetRaypathColors", "classes is null or class_count is negative");
  }
  std::lock_guard<TicketMutex> lock(consumer_mutex_);
  if (static_cast<size_t>(class_count) != active_class_table_.classes_.size()) {
    return Error::InvalidConfig("SetRaypathColors: class_count (" + std::to_string(class_count) +
                                ") does not match active color-class count (" +
                                std::to_string(active_class_table_.classes_.size()) +
                                "); re-commit the config to change member structure");
  }
  // When supplied, z_order must be a permutation of [0, class_count): each z_order[i] is the
  // new drawing priority (rank) of class i, and the ranks are the integers 0..class_count-1 in
  // some order. Reject duplicates / out-of-range (e.g. {0,0,1}) as an all-or-nothing failure
  // before mutating any state (plan §3.2.3, AC3).
  if (z_order != nullptr) {
    std::vector<bool> seen(static_cast<size_t>(class_count), false);
    for (int k = 0; k < class_count; k++) {
      if (z_order[k] < 0 || z_order[k] >= class_count || seen[static_cast<size_t>(z_order[k])]) {
        return Error::InvalidConfig("SetRaypathColors: z_order must be a permutation of [0, class_count)");
      }
      seen[static_cast<size_t>(z_order[k])] = true;
    }
  }
  // Apply appearance fields.
  for (int i = 0; i < class_count; i++) {
    auto& cls = active_class_table_.classes_[static_cast<size_t>(i)];
    cls.color_[0] = classes[i].color_[0];
    cls.color_[1] = classes[i].color_[1];
    cls.color_[2] = classes[i].color_[2];
    cls.visible_ = classes[i].visible_;
    cls.solo_ = classes[i].solo_;
    if (z_order != nullptr) {
      cls.z_order_ = z_order[i];
    }
  }
  active_composite_mode_ = mode;
  // Force the next DoSnapshot to re-run the compositor even without new ray data — this is
  // the mechanism that makes "change color and immediately see the new pixels" work when
  // the accumulator is at steady state. snapshot_dirty_'s existing "has anything changed
  // since last snapshot" semantics remain valid (a display-time change IS such a change).
  snapshot_dirty_ = true;
  return Error::Success();
}


// =============== ServerImpl::SetCompositeExposure ===============
// task-345.3: display-time EV for the composite path. Same shape as
// SetRaypathColors: writes one field under consumer_mutex_ then flips
// snapshot_dirty_ so the next acquired result frame triggers exactly one composite
// rebake with the new EV. No validation on ev_total — any finite float is
// legitimate (the GUI already clamps to [-6, 6] before calling; the server
// intentionally does not double-clamp so a hypothetical future caller can
// pass through). No epoch bump, no consumers rebuild, no scene_mutex_
// touched — see SetRaypathColors for the identical discipline this follows.
Error ServerImpl::SetCompositeExposure(float ev_total) {
  std::lock_guard<TicketMutex> lock(consumer_mutex_);
  display_ev_total_ = ev_total;
  snapshot_dirty_ = true;
  return Error::Success();
}


// =============== ServerImpl::SetCompositeBackground ===============
// Display-time background colour for the composite path. Same shape as
// SetCompositeExposure: writes one field under
// consumer_mutex_ then flips snapshot_dirty_ so the next acquired result frame
// triggers exactly one composite rebake with the new background. No validation
// on the components — any finite float is legitimate (the caller owns the
// sRGB→linear conversion and any clamping; the compositor's final
// LinearRgbToSrgbU8 clamps to [0,1] anyway). No epoch bump, no consumers
// rebuild, no scene_mutex_ touched — see SetRaypathColors for the identical
// discipline this follows.
Error ServerImpl::SetCompositeBackground(const float rgb[3]) {
  std::lock_guard<TicketMutex> lock(consumer_mutex_);
  std::copy(rgb, rgb + 3, std::begin(composite_background_linear_));
  snapshot_dirty_ = true;
  return Error::Success();
}


// =============== ServerImpl::GetColorClassSignals ===============
// task-342.3 AC4: reads snapshot Y-lanes (no DoSnapshot trigger; caller has been
// polling composite/xyz results and thus has a fresh snapshot). Aggregates
// across RenderConsumer instances (OR), so a class with any signal on any
// renderer reads as present.
Error ServerImpl::GetColorClassSignals(uint8_t* out_flags, int class_count) {
  if (class_count < 0) {
    return Error::InvalidValue("GetColorClassSignals", "class_count is negative");
  }
  std::lock_guard<TicketMutex> lock(consumer_mutex_);
  if (static_cast<size_t>(class_count) != active_class_table_.classes_.size()) {
    return Error::InvalidConfig("GetColorClassSignals: class_count (" + std::to_string(class_count) +
                                ") does not match active color-class count (" +
                                std::to_string(active_class_table_.classes_.size()) + ")");
  }
  if (class_count == 0) {
    return Error::Success();
  }
  if (out_flags == nullptr) {
    return Error::InvalidValue("GetColorClassSignals", "out_flags is null");
  }
  for (int i = 0; i < class_count; i++) {
    out_flags[i] = 0;
  }
  for (const auto& c : consumers_) {
    const auto* rc = dynamic_cast<const RenderConsumer*>(c.get());
    if (rc == nullptr) {
      continue;
    }
    for (int i = 0; i < class_count; i++) {
      if (out_flags[i] == 0 && rc->HasColorClassSignal(static_cast<size_t>(i))) {
        out_flags[i] = 1;
      }
    }
  }
  return Error::Success();
}


// =============== ServerImpl::SetLogLevel ===============
void ServerImpl::SetLogLevel(LogLevel level) {
  logger_.SetLevel(level);
  std::lock_guard<std::mutex> lock(prod_mutex_);
  for (auto& s : simulators_) {
    s.SetLogLevel(level);
  }
}


// =============== Server ===============
Server::Server() : impl_(std::make_shared<ServerImpl>()) {}

Server::Server(int num_workers, uint32_t sim_seed, BackendKind preferred_backend)
    : impl_(std::make_shared<ServerImpl>(num_workers, sim_seed, preferred_backend)) {}

Error Server::CommitConfig(const nlohmann::json& config_json, bool* out_reused) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->CommitConfig(config_json, out_reused);
}

Error Server::CommitConfig(const std::string& config_str) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  try {
    auto config_json = nlohmann::json::parse(config_str);
    return CommitConfig(config_json);
  } catch (const nlohmann::json::parse_error& e) {
    ILOG_ERROR(impl_->GetLogger(), "CommitConfig: JSON parse error: {}", e.what());
    return Error::InvalidJson(e.what());
  } catch (...) {
    ILOG_ERROR(impl_->GetLogger(), "CommitConfig: Unknown error");
    return Error::InvalidJson("Unknown JSON parsing error");
  }
}

Error Server::StartRaypathAnalysis(const nlohmann::json& scene_json, const RaypathAnalysisRequest& request) {
  if (!impl_) {
    return Error::ServerNotReady();
  }
  return impl_->StartRaypathAnalysis(scene_json, request);
}

BackendKind Server::GetActiveBackend() const {
  if (!impl_) {
    return BackendKind::kCpu;
  }
  return impl_->GetActiveBackend();
}

size_t Server::GetLiveSimRayCount() {
  if (!impl_) {
    return 0;
  }
  return impl_->GetLiveSimRayCount();
}

std::shared_ptr<const ResultFrame> Server::AcquireResultFrame() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    // A terminated server still owes the caller a dereferenceable frame — an empty one
    // reads exactly as "no results", which is the truth here.
    return std::make_shared<const ResultFrame>();
  }
  return impl_->AcquireResultFrame();
}

void Server::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
}

void Server::Terminate() {
  if (!impl_) {
    return;
  }
  ILOG_DEBUG(impl_->GetLogger(), "Terminate: entry");
  impl_.reset();  // ~ServerImpl() handles Stop + thread join via RAII
}

void Server::SetLogLevel(LogLevel level) {
  if (impl_) {
    impl_->SetLogLevel(level);
  }
}

void Server::SetPreferredBackend(BackendKind backend) {
  if (impl_) {
    impl_->SetPreferredBackend(backend);
  }
}

ServerStatus Server::GetStatus() const {
  if (!impl_) {
    return ServerStatus::kError;
  }
  return impl_->GetStatus();
}

SimLifecycle Server::GetSimLifecycle() const {
  if (!impl_) {
    return SimLifecycle::kIdle;
  }
  return impl_->GetSimLifecycle();
}

SessionKind Server::GetSessionKind() const {
  if (!impl_) {
    return SessionKind::kRender;
  }
  return impl_->GetSessionKind();
}

uint64_t Server::CommittedEpoch() const {
  if (!impl_) {
    return 0;
  }
  return impl_->CommittedEpoch();
}

uint64_t Server::DrainedEpoch() const {
  if (!impl_) {
    return 0;
  }
  return impl_->DrainedEpoch();
}

bool Server::IsIdle() const {
  return GetStatus() == ServerStatus::kIdle;
}

size_t Server::GetLastColorComponentOverflowCount() const {
  if (!impl_) {
    return 0;
  }
  return impl_->GetLastColorComponentOverflowCount();
}

ColorDegradeCounts Server::GetLastColorDegradeCounts() const {
  if (!impl_) {
    return {};
  }
  return impl_->GetLastColorDegradeCounts();
}

bool Server::BackendFellBack() const {
  if (!impl_) {
    return false;
  }
  return impl_->BackendFellBack();
}

Error Server::SetRaypathColors(const ColorClassDisplay* classes, int class_count, const int* z_order,
                               CompositeMode mode) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->SetRaypathColors(classes, class_count, z_order, mode);
}

Error Server::SetCompositeExposure(float ev_total) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->SetCompositeExposure(ev_total);
}

Error Server::SetCompositeBackground(const float rgb[3]) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->SetCompositeBackground(rgb);
}

Error Server::GetColorClassSignals(uint8_t* out_flags, int class_count) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->GetColorClassSignals(out_flags, class_count);
}

}  // namespace lumice
