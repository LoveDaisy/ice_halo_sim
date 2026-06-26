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

#include "config/config_manager.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/def.hpp"
#include "core/simulator.hpp"
#include "server/consumer.hpp"
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
  std::optional<StatsResult> GetStatsResult();
  std::optional<StatsResult> GetCachedStatsResult();

  void Stop();
  void Start();
  ServerStatus GetStatus() const;
  bool IsIdle();
  void SetPreferredBackend(BackendKind backend);

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

  void ConsumeData();
  void GenerateScene();
  void DoSnapshot();

  // Persistent thread loop: wait for Start(), run work_fn, repeat until kTerminating.
  template <typename F>
  void RunPersistentLoop(F work_fn);

  ConfigManager config_manager_;

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
  std::vector<std::thread> simulator_threads_;
  mutable std::mutex prod_mutex_;

  // Active scene and generation counter for batch staleness detection
  std::shared_ptr<const SceneConfig> active_scene_;
  // Snapshot of renderers paired with active_scene_ (task 252.3, TraceBackend seam).
  // Set in CommitConfig under scene_mutex_ in lockstep with active_scene_, then
  // attached to every SimBatch emitted by GenerateScene. Stays nullptr if no
  // CommitConfig has yet succeeded; consumers tolerate null.
  std::shared_ptr<const std::vector<RenderConfig>> active_renders_;
  std::atomic<uint64_t> scene_generation_{ 0 };

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


namespace {
// task-268.7 (owner 2026-06-15): the CPU and GPU routes do NOT mirror each other
// — each picks its own optimal orchestration, so the server runs two parallel
// shapes. The GPU/Metal route is a SINGLE engine (N engines would contend one
// GPU — explore-263); the legacy CPU route keeps MULTI-worker parallelism (that
// IS its performance model — collapsing it to 1 worker is a ~6x regression on
// the perf baseline + GUI default path). The route is fixed at construction; the
// GUI reconstructs the server when the Metal checkbox toggles. An env
// LUMICE_TRACE_BACKEND override (CLI / --benchmark) takes precedence over the
// preferred_backend argument, mirroring CreateBackend (simulator.cpp).
bool ResolveMetalRoute(BackendKind preferred_backend, Logger& logger) {
#if defined(__APPLE__)
  if (std::optional<std::string> override = env::TraceBackendOverride(logger)) {
    const std::string& name = *override;
    if (name == "metal") {
      return true;
    }
    if (name == "cpu_backend" || name == "legacy") {
      return false;
    }
  }
  // -Wswitch: exhaustive over BackendKind, no `default:`. CUDA does not route
  // through the Metal/GPU sizing path until subtask 3 lands the CUDA backend.
  switch (preferred_backend) {
    case BackendKind::kMetal:
      return true;
    case BackendKind::kCpu:
    case BackendKind::kCuda:
      return false;
  }
  return false;
#else
  (void)preferred_backend;
  (void)logger;
  return false;  // Metal unavailable off-Apple → CPU multi-worker route
#endif
}
}  // namespace

ServerImpl::ServerImpl(int num_workers, uint32_t sim_seed, BackendKind preferred_backend)
    : config_manager_{}, scene_queue_(std::make_shared<Queue<SimBatch>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle) {
  preferred_backend_.store(preferred_backend, std::memory_order_release);
  int worker_count;
  if (ResolveMetalRoute(preferred_backend, logger_)) {
    worker_count = 1;  // GPU route: single engine (task-268.7)
  } else {
    worker_count = num_workers > 0 ? num_workers : PhysicalCoreCount();
    if (sim_seed != 0) {
      worker_count = 1;  // deterministic CPU contract: fixed seed → single worker
    }
  }
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
  try {
    new_config = config_json.get<ConfigManager>();
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

  bool can_reuse = !consumers_.empty() && (old_renderers.size() == config_manager_.renderers_.size());
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
        consumers_.emplace_back(std::make_shared<RenderConsumer>(r));
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
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    active_scene_ = std::move(new_scene);
    active_renders_ = std::move(new_renders);
    scene_generation_.fetch_add(1);
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
void ServerImpl::DoSnapshot() {
  // Phase 1: memcpy under consumer_mutex_ (short hold).
  // Copy shared_ptrs so consumers stay alive even if Stop() clears consumers_.
  std::vector<ConsumerPtrS> snapshot_consumers;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    if (!snapshot_dirty_) {
      ILOG_DEBUG(logger_, "DoSnapshot: skip (snapshot_dirty_=false)");
      return;
    }
    for (const auto& c : consumers_) {
      c->PrepareSnapshot();
    }
    snapshot_consumers = consumers_;  // shared_ptr copy keeps consumers alive
    snapshot_dirty_ = false;
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
    cached_render_results_ = std::move(render_results);
    cached_stats_result_ = stats_result;
  }
}

std::vector<RenderResult> ServerImpl::GetRenderResults() {
  DoSnapshot();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return cached_render_results_;
}

std::vector<RawXyzResult> ServerImpl::GetRawXyzResults() {
  // Phase 1: PrepareSnapshot under consumer_mutex_ (TicketMutex guarantees FIFO — no starvation).
  std::vector<ConsumerPtrS> snapshot_consumers;
  bool did_snapshot = false;
  bool valid_data = false;
  uint64_t generation = 0;
  {
    std::lock_guard<TicketMutex> lock(consumer_mutex_);
    if (snapshot_dirty_) {
      for (const auto& c : consumers_) {
        c->PrepareSnapshot();
      }
      snapshot_dirty_ = false;
      snapshot_generation_++;
      did_snapshot = true;
    }
    valid_data = has_ever_consumed_;
    generation = snapshot_generation_;
    snapshot_consumers = consumers_;
  }
  // Pixel counting outside consumer_mutex_ (snapshot_xyz_ is stable here).
  if (did_snapshot) {
    for (const auto& c : snapshot_consumers) {
      if (auto* rc = dynamic_cast<RenderConsumer*>(c.get())) {
        rc->CountEffectivePixels();
      }
    }
  }
  std::vector<RawXyzResult> results;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  for (const auto& c : snapshot_consumers) {
    auto* render_consumer = dynamic_cast<RenderConsumer*>(c.get());
    if (render_consumer) {
      auto r = render_consumer->GetRawXyzResult();
      r.has_valid_data_ = valid_data;
      r.snapshot_generation_ = generation;
      results.push_back(r);
    }
  }
  if (did_snapshot) {
    for (const auto& c : snapshot_consumers) {
      if (dynamic_cast<RenderConsumer*>(c.get()) != nullptr) {
        continue;
      }
      auto result = c->GetResult();
      if (auto* s = std::get_if<StatsResult>(&result)) {
        cached_stats_result_ = *s;
      }
    }
  }
  return results;
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

  return ServerStatus::kIdle;
}

bool ServerImpl::IsIdle() {
  return GetStatus() == ServerStatus::kIdle;
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
        // 0-exit-batch guard: backend exit-seam path may emplace a SimData with
        // outgoing_d_/rays_ both empty when all rays were filtered/absorbed
        // (e.g. selective BD filter). Skip consumer projection so we don't
        // dirty snapshot_dirty_/has_ever_consumed_ on a black contribution, but
        // STILL fall through to sim_scene_cnt_-- below — that decrement is the
        // counter invariant paired with GenerateScene's ++ (see simulator.cpp
        // exit-seam: empty Emplace must reach the consumer's --). Do not move
        // the -- inside this branch.
        bool has_renderable = !sim_data.outgoing_d_.empty() || !sim_data.rays_.Empty();
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
    sim_scene_cnt_--;
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
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    scene = active_scene_;
    renders = active_renders_;
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
  const size_t kDefaultDispatch = ResolveMetalRoute(preferred_backend_.load(std::memory_order_acquire), logger_) ?
                                      kDefaultMetalDispatchRayNum :
                                      kDefaultRayNum;
  const size_t kDispatchCap = env::DispatchRayNum(logger_, kDefaultDispatch);
  const size_t kBatchCap = kDispatchCap;  // local alias for the loop below

  auto ray_num = scene->ray_num_;
  size_t committed_num = 0;
  while (ray_num == kInfSize || committed_num < ray_num) {
    size_t batch_ray_num = std::min(kBatchCap, ray_num - committed_num);
    scene_queue_->Emplace(SimBatch{ batch_ray_num, scene, generation, renders });
    sim_scene_cnt_++;
    if (!first_batch_logged) {
      ILOG_INFO(logger_, "GenerateScene: first batch enqueued at {:.1f}ms after start",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - gen_start).count());
      first_batch_logged = true;
    }

    ILOG_TRACE(logger_, "GenerateScene: put a scene: ray({}/{}, {})", batch_ray_num, ray_num, committed_num);
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

std::vector<RawXyzResult> Server::GetRawXyzResults() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return {};
  }
  return impl_->GetRawXyzResults();
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

bool Server::IsIdle() const {
  return GetStatus() == ServerStatus::kIdle;
}

}  // namespace lumice
