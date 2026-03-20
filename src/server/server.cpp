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

  Error CommitConfig(const nlohmann::json& config_json);
  std::vector<RenderResult> GetRenderResults();
  std::vector<RawXyzResult> GetRawXyzResults();
  std::optional<StatsResult> GetStatsResult();

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

  ConfigManager config_manager_;

  QueuePtrS<SimBatch> scene_queue_;
  QueuePtrS<SimData> data_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrS> consumers_;
  mutable std::mutex consumer_mutex_;  // Lock ordering: consumer_mutex_ < snapshot_mutex_
  mutable std::mutex snapshot_mutex_;  // Protects cached results
  bool snapshot_dirty_{ false };       // Set by ConsumeData, cleared by DoSnapshot
  bool has_ever_consumed_{ false };    // Monotonic: once true, never cleared (even across Stop/Start)
  uint64_t snapshot_generation_{ 0 };  // Increments on each PrepareSnapshot; monotonically increasing

  // Cached results under snapshot_mutex_ — populated by DoSnapshot, read by Get*Results
  std::vector<RenderResult> cached_render_results_;
  std::optional<StatsResult> cached_stats_result_;
  std::vector<std::thread> simulator_threads_;
  mutable std::mutex prod_mutex_;

  // Active scene and generation counter for batch staleness detection
  std::shared_ptr<const SceneConfig> active_scene_;
  std::atomic<uint64_t> scene_generation_{ 0 };

  std::atomic_bool stop_{ true };
  std::atomic_bool work_started_{ false };
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


ServerImpl::ServerImpl()
    : config_manager_{}, scene_queue_(std::make_shared<Queue<SimBatch>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle) {
  for (int i = 0; i < kDefaultSimulatorCnt; i++) {
    simulators_.emplace_back(scene_queue_, data_queue_);
  }
  // Don't Start() here — wait for CommitConfig to provide a valid project.
}


Error ServerImpl::CommitConfig(const nlohmann::json& config_json) {
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
  ILOG_INFO(logger_, "CommitConfig: Stop took {:.1f}ms",
            std::chrono::duration<double, std::milli>(stop_end - stop_start).count());

  config_manager_ = std::move(new_config);

  // Setup consumers from project config.
  // Lock consumer_mutex_ to prevent race with ServerPoller's GetRawXyzResults().
  ILOG_DEBUG(logger_, "CommitConfig: setup consumers");
  {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    consumers_.clear();
    for (const auto& [_, r] : config_manager_.renderers_) {
      consumers_.emplace_back(std::make_shared<RenderConsumer>(r));
    }
    consumers_.emplace_back(std::make_shared<StatsConsumer>());
  }

  auto new_scene = std::make_shared<SceneConfig>(config_manager_.scene_);
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    active_scene_ = std::move(new_scene);
    scene_generation_.fetch_add(1);
  }

  Start();

  auto commit_end = std::chrono::steady_clock::now();
  ILOG_INFO(logger_, "CommitConfig: restart took {:.1f}ms (Stop {:.1f}ms + rebuild + Start)",
            std::chrono::duration<double, std::milli>(commit_end - commit_start).count(),
            std::chrono::duration<double, std::milli>(stop_end - stop_start).count());

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
    snapshot_consumers = consumers_;  // shared_ptr copy keeps consumers alive
  }
  std::vector<RawXyzResult> results;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  for (const auto& c : snapshot_consumers) {
    auto* render_consumer = dynamic_cast<RenderConsumer*>(c.get());
    if (render_consumer) {
      auto r = render_consumer->GetRawXyzResult();
      r.has_valid_data_ = has_ever_consumed_;
      r.snapshot_generation_ = snapshot_generation_;
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
  // Attach cached stats to results so callers don't need a separate GetStatsResults call
  if (cached_stats_result_) {
    for (auto& r : results) {
      r.stats_ray_seg_num_ = cached_stats_result_->ray_seg_num_;
      r.stats_sim_ray_num_ = cached_stats_result_->sim_ray_num_;
    }
  }
  return results;
}

std::optional<StatsResult> ServerImpl::GetStatsResult() {
  DoSnapshot();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return cached_stats_result_;
}


void ServerImpl::Start() {
  ILOG_DEBUG(logger_, "Start: entry");
  if (!stop_) {
    return;
  }

  stop_ = false;
  work_started_ = false;
  sim_scene_cnt_ = 0;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_ = ServerStatus::kRunning;
  }

  // Start queues & jobs
  data_queue_->Start();
  scene_queue_->Start();
  {
    std::lock_guard<std::mutex> lock(prod_mutex_);
    ILOG_DEBUG(logger_, "Start: lock on prod_mutex_");
    for (auto& s : simulators_) {
      simulator_threads_.emplace_back(&Simulator::Run, &s);
    }
    ILOG_DEBUG(logger_, "Start: unlock prod_mutex_");
  }

  // Start main working thread & scene dispatch thread
  consume_data_thread_ = std::thread(&ServerImpl::ConsumeData, this);
  generate_scene_thread_ = std::thread(&ServerImpl::GenerateScene, this);
}


void ServerImpl::Stop() {
  ILOG_DEBUG(logger_, "Stop: entry");
  if (stop_) {
    return;
  }

  stop_ = true;

  // Stop queues
  scene_queue_->Shutdown();
  data_queue_->Shutdown();

  scene_cv_.notify_one();

  // Stop running jobs
  {
    std::lock_guard<std::mutex> lock(prod_mutex_);
    ILOG_DEBUG(logger_, "Stop: lock on prod_mutex_. stop simulators. clear simulator threads.");
    for (auto& s : simulators_) {
      s.Stop();
    }
    for (auto& t : simulator_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    simulator_threads_.clear();
    ILOG_DEBUG(logger_, "Stop: unlock prod_mutex_");
  }

  // Stop main working thread & scene dispatch thread.
  ILOG_DEBUG(logger_, "Stop: waiting main worker & dispatcher stop.");
  if (consume_data_thread_.joinable()) {
    consume_data_thread_.join();
  }
  if (generate_scene_thread_.joinable()) {
    generate_scene_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    consumers_.clear();
    snapshot_dirty_ = false;
    has_ever_consumed_ = false;  // New consumers have no data yet
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_ = ServerStatus::kIdle;
  }
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

  if (any_busy || sim_scene_cnt_ > 0) {
    return ServerStatus::kRunning;
  }

  return ServerStatus::kIdle;
}

bool ServerImpl::IsIdle() {
  return GetStatus() == ServerStatus::kIdle;
}


#define CHECK_STOP \
  if (stop_) {     \
    break;         \
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

    ILOG_DEBUG(logger_, "ConsumeData: get data: {}", fmt::ptr(&sim_data));

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
        has_ever_consumed_ = true;  // Monotonic: once set, never cleared
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

    ILOG_DEBUG(logger_, "GenerateScene: put a scene: ray({}/{}, {})", batch_ray_num, ray_num, committed_num);
    CHECK_STOP

    if (sim_scene_cnt_ >= kMaxSceneCnt) {
      ILOG_DEBUG(logger_, "GenerateScene: too many scenes generated. wait for consumer");
      std::unique_lock<std::mutex> lock(scene_mutex_);
      scene_cv_.wait(lock, [=]() { return stop_ || sim_scene_cnt_ < kMaxSceneCnt; });
      ILOG_DEBUG(logger_, "GenerateScene: continue to generate scenes.");
    }
    CHECK_STOP
    committed_num += kDefaultRayNum;
    ILOG_DEBUG(logger_, "GenerateScene: finish wl");
  }
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

Error Server::CommitConfig(const nlohmann::json& config_json) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  return impl_->CommitConfig(config_json);
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

Error Server::CommitConfigFromFile(const std::string& filename) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  std::ifstream f(filename);
  if (!f.is_open()) {
    return Error::InvalidConfig("Cannot open file: " + filename);
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
  impl_->Stop();
  impl_ = nullptr;
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
