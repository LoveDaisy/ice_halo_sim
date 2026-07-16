#include "server/server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdlib>
#include <fstream>
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
#include "server/component_compositor.hpp"
#include "server/consumer.hpp"
#include "server/ray_num_semantics.hpp"
#include "server/render.hpp"
#include "server/server.hpp"
#include "server/stats.hpp"
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
  std::vector<RenderResult> GetRenderResults();
  std::vector<RawXyzResult> GetRawXyzResults();
  // task-345.2: atomic combined getter — one DoSnapshot() call, so xyz and
  // composite are guaranteed to belong to the same snapshot_generation_ by
  // construction (kills the poller's drift-guard, ④). See
  // Server::GetRawXyzAndCompositeResults docstring for rationale.
  void GetRawXyzAndCompositeResults(std::vector<RawXyzResult>& xyz_out, std::vector<RenderResult>& composite_out);
  std::optional<StatsResult> GetStatsResult();
  std::optional<StatsResult> GetCachedStatsResult();
  size_t GetLiveSimRayCount();

  void Stop();
  void Start();
  ServerStatus GetStatus() const;
  SimLifecycle GetSimLifecycle() const;
  uint64_t CommittedEpoch() const;
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
  // snapshot_dirty_ so the next Get*Results triggers one composite rebake
  // with the new EV. Mono path is untouched (structural AC4).
  Error SetCompositeExposure(float ev_total);

  // task-342.3 AC4: per-color-class empty-arc detector. Reads the frozen snapshot
  // lanes (no DoSnapshot trigger — GUI polling loop is expected to already query
  // GetCompositeResults / GetRawXyzResults). Writes 1 into out_flags[i] when
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

 private:
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

  void ConsumeData();
  void GenerateScene();
  // task-342.4 Step 1: unified snapshot consumer. Returns true iff this call
  // actually consumed a dirty snapshot (Phase 1..2 executed); false if
  // snapshot_dirty_ was clear on entry (nothing to do). Merges the previously
  // duplicate Phase-1 in GetRawXyzResults into a single dirty-flag owner so
  // any pair of Get*Results calls in the same poll tick see coherent state
  // (see plan §3 keypoint 1).
  bool DoSnapshot();

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

  QueuePtrS<SimBatch> scene_queue_;
  QueuePtrS<SimData> data_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrS> consumers_;
  mutable TicketMutex consumer_mutex_;  // FIFO lock: prevents Poller starvation on Windows
  mutable std::mutex snapshot_mutex_;   // Protects cached results
  bool snapshot_dirty_{ false };        // Set by ConsumeData, cleared by DoSnapshot
  bool has_ever_consumed_{ false };     // True after first ConsumeData; reset on Stop (new consumers have no data)
  uint64_t snapshot_generation_{
    0
  };  // Increments on each PrepareSnapshot; NOT reset on Stop (poller resets its own tracker)

  // Cached results under snapshot_mutex_ — populated by DoSnapshot, read by Get*Results
  std::vector<RenderResult> cached_render_results_;
  std::optional<StatsResult> cached_stats_result_;
  // task-336.3: owning cache of composited colored images, one per RenderConsumer
  // with a non-zero colored mask. Produced in DoSnapshot Phase 2, under
  // snapshot_mutex_. Export to the C-API is deferred to 336.4 — for now this is
  // internal (see GetCompositeResults() below). Empty when no raypath_color.
  struct CompositeResult {
    int renderer_id_;
    int w_;
    int h_;
    std::vector<uint8_t> rgb_;  // W*H*3 sRGB
    // task-345.3: P99 over the union of NON-ZERO UNEXPOSED (raw lane) Y
    // values across every participating class. The composite-path auto-EV
    // anchor consumed by the GUI (mono path keeps xyz_data-derived P99).
    // 0 when no participating class carries any positive Y.
    float p99_y_ = 0.0f;
  };
  std::vector<CompositeResult> cached_composite_results_;

 public:
  // task-336.4: composite results getter, mirroring GetRenderResults so the
  // C-API can export it via the existing RenderResult surface. Returns
  // RenderResult handles whose img_buffer_ points into the ServerImpl-owned
  // cached_composite_results_ (NOT owning copies) — same lifetime contract as
  // GetRenderResults: valid until the next DoSnapshot() rebuild or CommitConfig.
  // Empty when no raypath_color is configured (no colored consumer → no cache).
  std::vector<RenderResult> GetCompositeResults() {
    DoSnapshot();
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    std::vector<RenderResult> out;
    out.reserve(cached_composite_results_.size());
    for (const auto& cr : cached_composite_results_) {
      RenderResult r;
      r.renderer_id_ = cr.renderer_id_;
      r.img_width_ = cr.w_;
      r.img_height_ = cr.h_;
      r.img_buffer_ = cr.rgb_.data();  // points into ServerImpl-owned cache
      r.composite_p99_y_ = cr.p99_y_;
      out.push_back(r);
    }
    return out;
  }

 private:
  std::vector<std::thread> simulator_threads_;
  mutable std::mutex prod_mutex_;

  // Active scene and generation counter for batch staleness detection
  std::shared_ptr<const SceneConfig> active_scene_;
  // Snapshot of renderers paired with active_scene_ (task 252.3, TraceBackend seam).
  // Set in CommitConfig under scene_mutex_ in lockstep with active_scene_, then
  // attached to every SimBatch emitted by GenerateScene. Stays nullptr if no
  // CommitConfig has yet succeeded; consumers tolerate null.
  std::shared_ptr<const std::vector<RenderConfig>> active_renders_;
  // Design 2 (task-engine-redirect-design2): snapshot of raypath_color paired
  // with active_scene_ / active_renders_. Updated inside the same scene_mutex_
  // critical section so a concurrent CommitConfig cannot tear the
  // (scene, renders, raypath_color) triple. Null → no color configured (AC3
  // zero-cost path).
  std::shared_ptr<const RaypathColorConfig> active_raypath_color_;
  std::atomic<uint64_t> scene_generation_{ 0 };
  // Published lifecycle epoch (the backend-owned truth authority). Distinct from
  // scene_generation_ (an internal batch-staleness key): keeping them separate
  // keeps the externally-published epoch from being polluted by batch-scheduling
  // details. ++ inside CommitConfig's scene_mutex_ critical section (next to
  // scene_generation_) on the accumulator-reset action — every successful commit
  // is reset-causing today, so every success ++s. A future "continue-same-config"
  // path (append rays without reset) must skip this ++. See plan §2 decision 3.
  std::atomic<uint64_t> committed_epoch_{ 0 };

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
bool ResolveGpuRoute(BackendKind preferred_backend, Logger& logger) {
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

ServerImpl::ServerImpl(int num_workers, uint32_t sim_seed, BackendKind preferred_backend)
    : config_manager_{}, scene_queue_(std::make_shared<Queue<SimBatch>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle) {
  preferred_backend_.store(preferred_backend, std::memory_order_release);
  // NOLINTNEXTLINE(readability-identifier-naming) — local const flag, snake_case is project style for variables.
  const bool gpu_route = ResolveGpuRoute(preferred_backend, logger_);
  int worker_count = 1;
  if (gpu_route) {
    worker_count = 1;  // GPU route: single engine (task-268.7; CUDA joined 296.6)
  } else {
    worker_count = num_workers > 0 ? num_workers : PhysicalCoreCount();
    if (sim_seed != 0) {
      worker_count = 1;  // deterministic CPU contract: fixed seed → single worker
    }
  }
  // AC1 observability (296.6): the GPU single-engine route must run worker_count==1.
  ILOG_INFO(logger_, "ServerImpl: gpu_route={} worker_count={} (preferred_backend={})", gpu_route, worker_count,
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
Error ServerImpl::CommitConfig(const nlohmann::json& config_json, bool* out_reused) {
  auto commit_start = std::chrono::steady_clock::now();
  ILOG_DEBUG(logger_, "CommitConfig: entry");

  // Parse into a temporary first so that a parse failure leaves the running server untouched.
  ConfigManager new_config;
  // task-339.3: runtime color-class table (default = empty → no raypath_color,
  // pre-336 behavior bit-for-bit). Declared outside the try so it survives to
  // the reuse-judgment / consumer construction below.
  ColorClassTable class_table;
  // task-339.4: the parsed composite mode. Declared outside the try so it
  // survives to the member assignment below, mirroring class_table.
  CompositeMode composite_mode = CompositeMode::kDominant;
  try {
    new_config = config_json.get<ConfigManager>();
    // task-339.2/339.3/339.4: color-class schema build path. BuildColorClassTable
    // resolves id → ci (setting_[] slot) using new_config.scene_ and may throw
    // std::invalid_argument on any config error (unknown combine, missing
    // (crystal,filter) pair, degenerate duplicate, out-of-range summand,
    // combine:"all" ban) — that lands in the std::exception catch below →
    // Error::InvalidConfig. class_table feeds directly into the RenderConsumer
    // (per-class Y-lane accumulation) and into the compositor
    // (CompositeColorClassesLinear); no legacy per-bit adapter layer.
    ColorGateTable color_gate_table = BuildColorGateTable(new_config.raypath_color_, new_config.scene_);
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
    class_table = BuildColorClassTable(new_config.raypath_color_, new_config.scene_, color_gate_table);
    composite_mode = ParseCompositeMode(new_config.raypath_color_.mode_);
  } catch (const nlohmann::json::out_of_range& e) {
    ILOG_ERROR(logger_, "CommitConfig: Missing field: {}", e.what());
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::MissingField(e.what());
  } catch (const nlohmann::json::exception& e) {
    ILOG_ERROR(logger_, "CommitConfig: JSON parsing error: {}", e.what());
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::InvalidJson(e.what());
  } catch (const std::exception& e) {
    ILOG_ERROR(logger_, "CommitConfig: Configuration error: {}", e.what());
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::InvalidConfig(e.what());
  } catch (...) {
    ILOG_ERROR(logger_, "CommitConfig: Unknown error");
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::InvalidConfig("Unknown configuration error");
  }

  // Stop → rebuild consumers → Start
  auto stop_start = std::chrono::steady_clock::now();
  Stop();
  auto stop_end = std::chrono::steady_clock::now();
  auto stop_ms = std::chrono::duration<double, std::milli>(stop_end - stop_start).count();

  // Check if consumers can be reused (same renderer key set, no layout changes).
  // See doc/accumulator-consumer-architecture.md §5.4 (reuse eligibility).
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

  bool can_reuse =
      !consumers_.empty() && !class_table_changed && (old_renderers.size() == config_manager_.renderers_.size());
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
          rc->ResetWith(it->second);
          ++it;
        } else {
          c->Reset();  // StatsConsumer
        }
      }
    } else {
      // Full rebuild path
      consumers_.clear();
      for (const auto& [_, r] : config_manager_.renderers_) {
        // task-339.3: pass the color-class table so each consumer allocates one
        // Y-lane per class (empty table → no lanes, pre-336 behavior).
        consumers_.emplace_back(std::make_shared<RenderConsumer>(r, active_class_table_));
      }
      consumers_.emplace_back(std::make_shared<StatsConsumer>());
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


// See doc/accumulator-consumer-architecture.md §4.2 (two-phase snapshot protocol).
// task-342.4 Step 1: this is the single owner of the snapshot_dirty_ flag and
// snapshot_generation_ counter. Any Get*Results method that needs an up-to-date
// materialized snapshot funnels through here so that two consumers in the same
// poll tick (e.g. RawXyz + Composite) see a coherent Phase-1..2 atomic event
// rather than racing the dirty flag against each other (plan §3 keypoint 1).
bool ServerImpl::DoSnapshot() {
  // Phase 1: memcpy under consumer_mutex_ (short hold).
  // Copy shared_ptrs so consumers stay alive even if Stop() clears consumers_.
  std::vector<ConsumerPtrS> snapshot_consumers;
  ColorClassTable snap_class_table;
  CompositeMode snap_composite_mode = CompositeMode::kDominant;
  float snap_display_ev_total = 0.0f;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    if (!snapshot_dirty_) {
      ILOG_DEBUG(logger_, "DoSnapshot: skip (snapshot_dirty_=false)");
      return false;
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
    snapshot_dirty_ = false;
    // task-342.4 Step 1: bumping the generation is now the shared owner's
    // responsibility (previously lived only in GetRawXyzResults's Phase-1).
    // The counter is the single mechanism by which poller detects new
    // snapshots, so it must be tied to the dirty-consume event itself, not
    // to any one consumer accessor.
    snapshot_generation_++;
  }
  // Phase 1.5: pixel counting outside consumer_mutex_ (snapshot_xyz_ is stable here).
  for (const auto& c : snapshot_consumers) {
    if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
      rc->CountEffectivePixels();
    }
  }
  // Phase 2: XYZ→RGB + cache results under snapshot_mutex_ (no consumer_mutex_).
  // Safe: snapshot_consumers holds shared_ptrs, objects won't be freed.
  std::vector<RenderResult> render_results;
  std::optional<StatsResult> stats_result;
  // task-336.3: composite colored consumers into cached RGB images. Built
  // outside the snapshot_mutex_ block (pure reads of the frozen snapshot lanes)
  // and swapped in under the lock alongside the render results.
  std::vector<CompositeResult> composite_results;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    for (const auto& c : snapshot_consumers) {
      c->PostSnapshot();
    }
    for (const auto& c : snapshot_consumers) {
      auto result = c->GetResult();
      if (auto* r = std::get_if<RenderResult>(&result)) {
        render_results.push_back(*r);
      } else if (auto* s = std::get_if<StatsResult>(&result)) {
        stats_result = *s;
      }
    }
    // task-336.3: only colored consumers (ColoredMask()!=0) produce a composite.
    // Zero-config consumers (mask 0) are skipped → no composite, mono path
    // untouched (plan §0 / risk-5 rollback).
    for (const auto& c : snapshot_consumers) {
      auto* rc = dynamic_cast<RenderConsumer*>(c.get());
      if (rc == nullptr || rc->ColoredMask() == 0) {
        continue;
      }
      std::vector<float> linear_rgb;
      // task-345.3: display-time EV multiplier + participating-P99 anchor
      // both flow through the compositor in one pass. `display_exposure_scale
      // = 2^snap_display_ev_total`; CLI paths never call SetCompositeExposure
      // so snap_display_ev_total stays at 0 → 2^0 = 1.0 → no behavior change.
      const float display_exposure_scale = std::pow(2.0f, snap_display_ev_total);
      float participating_p99 = 0.0f;
      if (!CompositeColorClassesLinear(*rc, snap_class_table, snap_composite_mode, display_exposure_scale, linear_rgb,
                                       &participating_p99)) {
        continue;
      }
      CompositeResult cr;
      cr.renderer_id_ = 0;
      // Recover the renderer id from the mono result (RenderResult carries it).
      auto mono = rc->GetResult();
      if (auto* r = std::get_if<RenderResult>(&mono)) {
        cr.renderer_id_ = r->renderer_id_;
      }
      cr.w_ = rc->ImageWidth();
      cr.h_ = rc->ImageHeight();
      LinearRgbToSrgbU8(linear_rgb, cr.rgb_);
      cr.p99_y_ = participating_p99;
      composite_results.push_back(std::move(cr));
    }
    cached_render_results_ = std::move(render_results);
    cached_stats_result_ = stats_result;
    cached_composite_results_ = std::move(composite_results);
  }
  return true;
}

std::vector<RenderResult> ServerImpl::GetRenderResults() {
  DoSnapshot();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return cached_render_results_;
}

std::vector<RawXyzResult> ServerImpl::GetRawXyzResults() {
  // task-342.4 Step 1: funnel through the shared DoSnapshot() so the
  // dirty-flag / snapshot_generation_ / cached_* fields have exactly one
  // owner. Previously this method carried its own copy of Phase-1 (dirty
  // consume + generation bump + PrepareSnapshot loop + CountEffectivePixels
  // + non-RenderConsumer StatsResult cache update). All of that now lives
  // inside DoSnapshot(); calling it here yields identical semantics to the
  // old implementation when only RawXyz consumers are wired, plus race-free
  // interleaving with GetCompositeResults() (plan §3 keypoint 1).
  DoSnapshot();
  std::vector<ConsumerPtrS> snapshot_consumers;
  bool valid_data = false;
  uint64_t generation = 0;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    valid_data = has_ever_consumed_;
    generation = snapshot_generation_;
    snapshot_consumers = consumers_;
  }
  std::vector<RawXyzResult> results;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  for (const auto& c : snapshot_consumers) {
    auto* render_consumer = dynamic_cast<RenderConsumer*>(c.get());
    if (render_consumer) {
      auto r = render_consumer->GetRawXyzResult();
      r.has_valid_data_ = valid_data;
      r.snapshot_generation_ = generation;
      // Best-effort epoch stamp (plan §5-R5): consumer data + this read are
      // serialized under consumer_mutex_/snapshot_mutex_, so a "new epoch with
      // stale data" tear cannot occur; the atomic-single-value publish is 1.4's job.
      r.epoch_ = committed_epoch_.load(std::memory_order_acquire);
      results.push_back(r);
    }
  }
  // task-342.4 Step 1: the previous "if (did_snapshot) { cache non-RenderConsumer
  // StatsResult }" block that used to live here has been removed. It was a
  // duplicate of DoSnapshot() Phase-2's existing per-consumer GetResult() /
  // StatsResult caching (see the loop that writes cached_stats_result_ above),
  // which now runs unconditionally through DoSnapshot() from every accessor —
  // so cached_stats_result_ stays up to date via a single owner.
  return results;
}

// task-345.2: atomic combined read. Kills the ServerPoller drift-guard (④) by
// funneling xyz + composite through exactly ONE DoSnapshot() trigger — a
// concurrent ConsumeData bump cannot slip a second Phase-2 rebuild between the
// two reads, so composite_out is guaranteed to belong to xyz_out[0]'s
// snapshot_generation_ by construction (structural, not probabilistic).
//
// The lock discipline mirrors GetRawXyzResults + GetCompositeResults verbatim
// (consumer_mutex_ then snapshot_mutex_; the two mutexes are never held
// nested), so no new lock-ordering surface. Body kept a direct merge (not
// helper-extracted) — plan-review flagged the light duplication with the two
// existing getters as a code-review-time follow-up, not a blocker; see plan
// §7 risk 2 mitigation ("严格镜像现有加锁顺序").
void ServerImpl::GetRawXyzAndCompositeResults(std::vector<RawXyzResult>& xyz_out,
                                              std::vector<RenderResult>& composite_out) {
  xyz_out.clear();
  composite_out.clear();
  DoSnapshot();
  std::vector<ConsumerPtrS> snapshot_consumers;
  bool valid_data = false;
  uint64_t generation = 0;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    valid_data = has_ever_consumed_;
    generation = snapshot_generation_;
    snapshot_consumers = consumers_;
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  for (const auto& c : snapshot_consumers) {
    auto* render_consumer = dynamic_cast<RenderConsumer*>(c.get());
    if (render_consumer) {
      auto r = render_consumer->GetRawXyzResult();
      r.has_valid_data_ = valid_data;
      r.snapshot_generation_ = generation;
      r.epoch_ = committed_epoch_.load(std::memory_order_acquire);
      xyz_out.push_back(r);
    }
  }
  composite_out.reserve(cached_composite_results_.size());
  for (const auto& cr : cached_composite_results_) {
    RenderResult r;
    r.renderer_id_ = cr.renderer_id_;
    r.img_width_ = cr.w_;
    r.img_height_ = cr.h_;
    r.img_buffer_ = cr.rgb_.data();
    composite_out.push_back(r);
  }
}

std::optional<StatsResult> ServerImpl::GetStatsResult() {
  DoSnapshot();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return cached_stats_result_;
}

std::optional<StatsResult> ServerImpl::GetCachedStatsResult() {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return cached_stats_result_;
}

// task-317: cheap O(1) live sim-ray-count read for the --benchmark drain-count
// poll loop. Unlike GetStatsResult() (which calls DoSnapshot -> RenderConsumer
// sRGB every poll — the render-per-poll root cause), this only reads the running
// StatsConsumer counter under consumer_mutex_. No snapshot, no render, no XYZ
// copy — so the poll thread does not perturb the throughput measurement nor
// starve drain-window closure.
// Sentinel note (inherited ambiguity, not new): 0 means BOTH "no StatsConsumer
// registered" AND "StatsConsumer present but no rays yet" — the same conflation
// the old have_stats?…:0 GetStatsResults path had. Fine for the benchmark's
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

uint64_t ServerImpl::CommittedEpoch() const {
  return committed_epoch_.load(std::memory_order_acquire);
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
  // (non-empty rays_) bypass the chunker — the consumer projects via per-ray
  // indices into rays_, which cannot be sliced without recomputing indices.
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
    // has rays_ empty AND root_ray_count_ == 0. Discriminating on
    // root_ray_count_ correctly distinguishes the sentinel from:
    //   - legacy CPU path: rays_ non-empty;
    //   - backend exit-seam path: rays_ empty + outgoing_d_/w_ populated
    //     (or empty for a zero-exit batch) but root_ray_count_ = ray_num > 0.
    // The earlier is_backend_path_ key falsely flagged exit-seam batches as
    // sentinels (rays_ empty + is_backend_path_ false), deadlocking the
    // consumer; root_ray_count_ is the protocol-level invariant for a real
    // produced batch and works uniformly across both paths.
    if (sim_data.rays_.Empty() && sim_data.root_ray_count_ == 0) {
      // Simulation is interrupted.
      break;
    }
    CHECK_STOP

    ILOG_TRACE(logger_, "ConsumeData: get data: {}", fmt::ptr(&sim_data));

    if (sim_scene_cnt_ > 0) {
      // Generation check: discard batches from outdated configs
      if (sim_data.generation_ != scene_generation_.load()) {
        ILOG_DEBUG(logger_, "ConsumeData: discarding batch (generation {} != {})", sim_data.generation_,
                   scene_generation_.load());
      } else {
        // task-color-degrade-gui-surfacing: this batch belongs to the current
        // committed config (generation matches) — publish its GPU color-degrade
        // tally to the atomics the C API poll path reads. OVERWRITE, not +=:
        // the value is a config constant, identical on every batch, so the last
        // writer simply refreshes it. GPU-only; CPU SimData carries all-zeros.
        last_color_symmetry_group_overflow_.store(sim_data.color_degrade_counts_.symmetry_group_overflow,
                                                  std::memory_order_release);
        last_color_or_summand_overflow_.store(sim_data.color_degrade_counts_.or_summand_overflow,
                                              std::memory_order_release);
        last_color_class_overflow_.store(sim_data.color_degrade_counts_.color_class_overflow,
                                         std::memory_order_release);
        // 0-exit-batch guard: backend exit-seam path may emplace a SimData with
        // outgoing_d_/rays_ both empty when all rays were filtered/absorbed
        // (e.g. selective BD filter). Skip consumer projection so we don't
        // dirty snapshot_dirty_/has_ever_consumed_ on a black contribution, but
        // STILL fall through to sim_scene_cnt_-- below — that decrement is the
        // counter invariant paired with GenerateScene's ++ (see simulator.cpp
        // exit-seam: empty Emplace must reach the consumer's --). Do not move
        // the -- inside this branch.
        // S1 device-fused (scrum-302): the backend accumulates XYZ on-device and
        // emplaces a SimData carrying xyz_pixel_data_ with outgoing_d_ AND rays_
        // both empty. That payload IS renderable — without this clause it would
        // be misclassified as a 0-exit black batch and the consumer skipped,
        // dropping the entire device-fused image (zero output, parity corr=0).
        bool has_renderable =
            !sim_data.outgoing_d_.empty() || !sim_data.rays_.Empty() || !sim_data.xyz_pixel_data_.empty();
        if (has_renderable) {
          auto t_lock0 = std::chrono::steady_clock::now();
          std::lock_guard<TicketMutex> lock(consumer_mutex_);
          auto t_lock1 = std::chrono::steady_clock::now();
          // task-268.4: chunk by kCommitCap on the backend exit-seam path
          // (outgoing_d_ populated AND rays_ empty). Only the FIRST chunk
          // carries root_ray_count_ + crystals_ — StatsConsumer accumulates
          // both, so spreading them across chunks would N×-count and break
          // the stats invariant. Legacy CPU SimData (rays_ non-empty) are
          // delivered whole because their consumers project via per-ray
          // indices into rays_, which has no clean sub-batch slice.
          // NOLINTNEXTLINE(readability-identifier-naming) — local const flag, snake_case is project style for
          // variables.
          const bool is_exit_seam_path = sim_data.rays_.Empty() && !sim_data.outgoing_d_.empty();
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
              // Stats fields only on the first chunk; rest carry 0 / empty
              // so StatsConsumer's sim_rays_ / crystals_ accumulate to the
              // same totals as a single whole-Consume call would yield.
              if (emitted == 0) {
                chunk.root_ray_count_ = sim_data.root_ray_count_;
                chunk.crystal_count_ = sim_data.crystal_count_;
                chunk.crystals_ = sim_data.crystals_;
                chunk.crystal_axis_dists_ = sim_data.crystal_axis_dists_;
              }
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
                // scrum-268.8 (DR-3): per-ray wavelength must be sliced in
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
                     sim_data.rays_.size_, sim_data.outgoing_w_.size(), lock_us, consume_us);
          if (!first_consume_logged) {
            ILOG_INFO(logger_, "ConsumeData: first batch consumed ({} ray segments)", sim_data.rays_.size_);
            first_consume_logged = true;
          }
        } else {
          // 0-exit batch on the exit-seam path (all rays filtered/absorbed →
          // outgoing_d_ AND rays_ both empty). We deliberately do NOT call
          // c->Consume() — there is nothing to accumulate and a black batch must
          // not bias the image. BUT a batch that ran to completion with a
          // legitimately all-black result is still *valid data*: the simulation
          // converged, the answer is just zero intensity. We therefore flip
          // has_ever_consumed_ so GetRawXyzResults reports has_valid_data=true,
          // and dirty the snapshot so PrepareSnapshot produces a clean zero
          // frame (without this, an all-black simulation — e.g. an impossible
          // raypath filter — never sets has_valid_data, so the buffered poller
          // waits for "valid data" forever and times out at 600s). The legacy
          // CPU path never hit this because its rays_ is always non-empty, so
          // has_renderable stayed true; the exit-seam path (Metal + CUDA) is the
          // first to surface it. See doc/capi-lifecycle-architecture.md
          // ("zero-output completion").
          snapshot_dirty_ = true;
          has_ever_consumed_ = true;
          ILOG_DEBUG(logger_, "ConsumeData: 0-exit batch (all filtered) — marking valid_data, zero snapshot");
        }
      }
    } else {
      ILOG_DEBUG(logger_, "ConsumeData: skip consume (sim_scene_cnt_={})", sim_scene_cnt_.load());
    }
    // scrum-312 third-clock: a windowed device-fused SimData stands in for N
    // per-wavelength calls (sim_scene_credit_ == N); all other paths credit 1.
    // Decrement by the credit to keep the GenerateScene ++ / ConsumeData --
    // invariant balanced regardless of drain windowing.
    sim_scene_cnt_ -= static_cast<int>(sim_data.sim_scene_credit_);
    if (sim_scene_cnt_ < kMaxSceneCnt / 2) {
      scene_cv_.notify_one();
    }

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
  const bool kGpuRoute = ResolveGpuRoute(kPref, logger_);
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

  // task-296.7: sim_scene_cnt_ semantic fix — count SimData (not SimBatch).
  // The simulator emits one SimData per wavelength inside SimulateOneWavelength*
  // (1 for illuminant; N for discrete spectrum lists). ConsumeData decrements
  // per SimData. Incrementing by 1 here (the historical behaviour) created a
  // 1-vs-N pairing imbalance on discrete-spectrum configs: sim_scene_cnt_ went
  // negative as the consumer drained N - 1 "extra" SimData per batch, GetStatus
  // saw the predicate fall to false, and CLI single-snapshot rendering reported
  // kIdle while 4/5 of the wavelengths' batches still got skip-consumed (line
  // 772 `if (sim_scene_cnt_ > 0)` → else branch). Resolve by incrementing here
  // by the same N the simulator will emplace, so the counter matches the
  // consumer's per-SimData decrement. The scene is captured under scene_mutex_
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
  size_t total_ray_num = scene->ray_num_;
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
  while (per_wl_ray_num == kInfSize || committed_num < per_wl_ray_num) {
    size_t batch_ray_num = std::min(kBatchCap, per_wl_ray_num - committed_num);
    scene_queue_->Emplace(SimBatch{ batch_ray_num, scene, generation, renders, raypath_color });
    sim_scene_cnt_ += static_cast<int>(kNsimdataPerBatch);
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
      scene_cv_.wait(lock,
                     [this]() { return state_.load() != ServerState::kRunning || sim_scene_cnt_ < kMaxSceneCnt; });
      ILOG_DEBUG(logger_, "GenerateScene: continue to generate scenes.");
    }
    CHECK_STOP
    committed_num += kBatchCap;
    ILOG_TRACE(logger_, "GenerateScene: finish wl");
  }
  scene_gen_active_ = false;  // All exit paths (normal + CHECK_STOP break) converge here
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
// snapshot_dirty_ so the next Get*Results triggers exactly one composite
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

Error Server::CommitConfigFromFile(const std::filesystem::path& filename) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  std::ifstream f(filename);
  if (!f.is_open()) {
    return Error::InvalidConfig("Cannot open file: " + filename.u8string());
  }
  try {
    nlohmann::json config_json;
    f >> config_json;
    return impl_->CommitConfig(config_json);
  } catch (const nlohmann::json::parse_error& e) {
    ILOG_ERROR(impl_->GetLogger(), "CommitConfigFromFile: JSON parse error: {}", e.what());
    return Error::InvalidJson(e.what());
  } catch (...) {
    ILOG_ERROR(impl_->GetLogger(), "CommitConfigFromFile: Unknown error");
    return Error::InvalidJson("Unknown JSON parsing error");
  }
}

std::vector<RenderResult> Server::GetRenderResults() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return {};
  }
  return impl_->GetRenderResults();
}

std::vector<RenderResult> Server::GetCompositeResults() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return {};
  }
  return impl_->GetCompositeResults();
}

std::vector<RawXyzResult> Server::GetRawXyzResults() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return {};
  }
  return impl_->GetRawXyzResults();
}

void Server::GetRawXyzAndCompositeResults(std::vector<RawXyzResult>& xyz_out,
                                          std::vector<RenderResult>& composite_out) {
  xyz_out.clear();
  composite_out.clear();
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return;
  }
  impl_->GetRawXyzAndCompositeResults(xyz_out, composite_out);
}

std::optional<StatsResult> Server::GetStatsResult() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return std::nullopt;
  }
  return impl_->GetStatsResult();
}

std::optional<StatsResult> Server::GetCachedStatsResult() {
  if (!impl_) {
    return std::nullopt;
  }
  return impl_->GetCachedStatsResult();
}

size_t Server::GetLiveSimRayCount() {
  if (!impl_) {
    return 0;
  }
  return impl_->GetLiveSimRayCount();
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

uint64_t Server::CommittedEpoch() const {
  if (!impl_) {
    return 0;
  }
  return impl_->CommittedEpoch();
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

Error Server::GetColorClassSignals(uint8_t* out_flags, int class_count) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->GetColorClassSignals(out_flags, class_count);
}

}  // namespace lumice
