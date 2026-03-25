#include "server/server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
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
#include "util/logger.hpp"
#include "util/queue.hpp"

namespace lumice {

// =============== ServerImpl ===============
class ServerImpl {
 public:
  ServerImpl();
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

 private:
  static const int kDefaultSimulatorCnt;  // runtime: max(1, hardware_concurrency - 2)
  static constexpr int kMaxSceneCnt = 128;
  static constexpr size_t kDefaultRayNum = 128;

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
  mutable std::mutex consumer_mutex_;  // Lock ordering: consumer_mutex_ < snapshot_mutex_
  mutable std::mutex snapshot_mutex_;  // Protects cached results
  bool snapshot_dirty_{ false };       // Set by ConsumeData, cleared by DoSnapshot
  bool has_ever_consumed_{ false };    // True after first ConsumeData; reset on Stop (new consumers have no data)
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
  std::atomic<uint64_t> scene_generation_{ 0 };

  // Persistent thread state machine: threads wait on start_cv_ when kStopped,
  // work when kRunning, and exit when kTerminating.
  enum class ServerState { kStopped, kRunning, kTerminating };
  std::atomic<ServerState> state_{ ServerState::kStopped };
  std::mutex start_mutex_;
  std::condition_variable start_cv_;
  std::atomic<int> active_workers_{ 0 };
  std::atomic<uint64_t> start_generation_{ 0 };  // Incremented by Start(); prevents re-entry after natural completion

  std::atomic_bool work_started_{ false };
  std::atomic_bool scene_gen_active_{ false };  // True while GenerateScene is actively producing batches
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

const int ServerImpl::kDefaultSimulatorCnt = std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 2);


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
    if (active_workers_.fetch_sub(1) == 1) {
      // Last active worker — notify Stop() if it's waiting
      start_cv_.notify_all();
    }
  }
}


ServerImpl::ServerImpl()
    : config_manager_{}, scene_queue_(std::make_shared<Queue<SimBatch>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle) {
  for (int i = 0; i < kDefaultSimulatorCnt; i++) {
    simulators_.emplace_back(scene_queue_, data_queue_);
  }

  // Spawn persistent threads — they start in cv.wait(), not working.
  // Each thread tracks start_generation_ to avoid re-entering work after natural completion.
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
    std::lock_guard<std::mutex> lock(consumer_mutex_);
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
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    active_scene_ = std::move(new_scene);
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


void ServerImpl::DoSnapshot() {
  // Phase 1: memcpy under consumer_mutex_ (short hold).
  // Copy shared_ptrs so consumers stay alive even if Stop() clears consumers_.
  std::vector<ConsumerPtrS> snapshot_consumers;
  {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    if (!snapshot_dirty_) {
      return;
    }
    for (const auto& c : consumers_) {
      c->PrepareSnapshot();
    }
    snapshot_consumers = consumers_;  // shared_ptr copy keeps consumers alive
    snapshot_dirty_ = false;
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
  // Only do Phase 1 (PrepareSnapshot memcpy), skip Phase 2 (PostSnapshot XYZ→RGB).
  // Copy shared_ptrs so consumers stay alive even if Stop() clears consumers_.
  std::vector<ConsumerPtrS> snapshot_consumers;
  bool did_snapshot = false;
  // Capture these under consumer_mutex_ to avoid reading them under a different lock (snapshot_mutex_).
  bool valid_data = false;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
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
    snapshot_consumers = consumers_;  // shared_ptr copy keeps consumers alive
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
  // Update cached stats from StatsConsumer only (skip RenderConsumer to avoid triggering
  // the heavy PostSnapshot XYZ→RGB conversion that the GPU path doesn't need).
  if (did_snapshot) {
    for (const auto& c : snapshot_consumers) {
      if (dynamic_cast<RenderConsumer*>(c.get()) != nullptr) {
        continue;  // Skip: RenderConsumer::GetResult() calls PostSnapshot()
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
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    // Don't clear consumers_ here — CommitConfig decides whether to rebuild or reuse.
    // Consumers are destroyed either by CommitConfig (rebuild path) or ~ServerImpl().
    snapshot_dirty_ = false;
    has_ever_consumed_ = false;  // New consumers have no data yet
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


void ServerImpl::ConsumeData() {
  ILOG_DEBUG(logger_, "ConsumeData: entry");
  bool first_consume_logged = false;
  while (true) {
    CHECK_STOP
    auto sim_data = data_queue_->Get();
    if (sim_data.rays_.Empty()) {
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
        std::lock_guard<std::mutex> lock(consumer_mutex_);
        for (auto& c : consumers_) {
          c->Consume(sim_data);
        }
        snapshot_dirty_ = true;
        has_ever_consumed_ = true;
        if (!first_consume_logged) {
          ILOG_INFO(logger_, "ConsumeData: first batch consumed ({} ray segments)", sim_data.rays_.size_);
          first_consume_logged = true;
        }
      }
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
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    scene = active_scene_;
    generation = scene_generation_.load();
  }
  auto ray_num = scene->ray_num_;
  size_t committed_num = 0;
  while (ray_num == kInfSize || committed_num < ray_num) {
    size_t batch_ray_num = std::min(kDefaultRayNum, ray_num - committed_num);
    scene_queue_->Emplace(SimBatch{ batch_ray_num, scene, generation });
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
    committed_num += kDefaultRayNum;
    ILOG_TRACE(logger_, "GenerateScene: finish wl");
  }
  scene_gen_active_ = false;  // All exit paths (normal + CHECK_STOP break) converge here
  ILOG_DEBUG(logger_, "GenerateScene exit");
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

void Server::InitLogger() {
  InitGlobalLogger();
}

void Server::SetLogLevel(LogLevel level) {
  if (impl_) {
    impl_->SetLogLevel(level);
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
