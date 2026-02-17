#include "server/server.hpp"

#include <atomic>
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
  std::vector<RenderResult> GetRenderResults() const;
  std::optional<StatsResult> GetStatsResult() const;

  void Stop();
  void Start();
  ServerStatus GetStatus() const;
  bool IsIdle();

 private:
  static constexpr int kDefaultSimulatorCnt = 4;
  static constexpr int kMaxSceneCnt = 128;
  static constexpr size_t kDefaultRayNum = 128;

  void ConsumeData();
  void GenerateScene();

  ConfigManager config_manager_;

  QueuePtrS<SceneConfig> scene_queue_;
  QueuePtrS<SimData> data_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrU> consumers_;
  mutable std::mutex consumer_mutex_;
  std::vector<std::thread> simulator_threads_;
  mutable std::mutex prod_mutex;

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


ServerImpl::ServerImpl()
    : config_manager_{}, scene_queue_(std::make_shared<Queue<SceneConfig>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle) {
  for (int i = 0; i < kDefaultSimulatorCnt; i++) {
    simulators_.emplace_back(scene_queue_, data_queue_);
  }
  // Don't Start() here — wait for CommitConfig to provide a valid project.
}


Error ServerImpl::CommitConfig(const nlohmann::json& config_json) {
  ILOG_DEBUG(logger_, "CommitConfig: entry");
  try {
    config_manager_ = config_json.get<ConfigManager>();
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

  Stop();

  // Setup consumers from project config.
  ILOG_DEBUG(logger_, "CommitConfig: setup consumers for proj {}", config_manager_.project_.id_);
  consumers_.clear();
  for (const auto& r : config_manager_.project_.renderers_) {
    consumers_.emplace_back(std::make_unique<RenderConsumer>(r));
  }
  consumers_.emplace_back(std::make_unique<StatsConsumer>());

  Start();

  return Error::Success();
}


std::vector<RenderResult> ServerImpl::GetRenderResults() const {
  std::lock_guard<std::mutex> lock(consumer_mutex_);
  std::vector<RenderResult> results;
  for (const auto& c : consumers_) {
    auto result = c->GetResult();
    if (auto* render = std::get_if<RenderResult>(&result)) {
      results.push_back(*render);
    }
  }
  return results;
}

std::optional<StatsResult> ServerImpl::GetStatsResult() const {
  std::lock_guard<std::mutex> lock(consumer_mutex_);
  for (const auto& c : consumers_) {
    auto result = c->GetResult();
    if (auto* stats = std::get_if<StatsResult>(&result)) {
      return *stats;
    }
  }
  return std::nullopt;
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
    std::lock_guard<std::mutex> lock(prod_mutex);
    ILOG_DEBUG(logger_, "Start: lock on prod_mutex");
    for (auto& s : simulators_) {
      simulator_threads_.emplace_back(&Simulator::Run, &s);
    }
    ILOG_DEBUG(logger_, "Start: unlock prod_mutex");
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
    std::lock_guard<std::mutex> lock(prod_mutex);
    ILOG_DEBUG(logger_, "Stop: lock on prod_mutex. stop simulators. clear simulator threads.");
    for (auto& s : simulators_) {
      s.Stop();
    }
    for (auto& t : simulator_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    simulator_threads_.clear();
    ILOG_DEBUG(logger_, "Stop: unlock prod_mutex");
  }
  consumers_.clear();

  // Stop main working thread & scene dispatch thread
  ILOG_DEBUG(logger_, "Stop: waiting main worker & dispatcher stop.");
  if (consume_data_thread_.joinable()) {
    consume_data_thread_.join();
  }
  if (generate_scene_thread_.joinable()) {
    generate_scene_thread_.join();
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
    std::lock_guard<std::mutex> lock(prod_mutex);
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
      std::lock_guard<std::mutex> lock(consumer_mutex_);
      for (auto& c : consumers_) {
        c->Consume(sim_data);
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
  work_started_ = true;

  const auto& scene = config_manager_.project_.scene_;
  auto ray_num = scene.ray_num_;
  size_t committed_num = 0;
  while (ray_num == kInfSize || committed_num < ray_num) {
    auto curr_scene = scene;
    curr_scene.ray_num_ = std::min(kDefaultRayNum, ray_num - committed_num);
    scene_queue_->Emplace(curr_scene);
    sim_scene_cnt_++;

    ILOG_DEBUG(logger_, "GenerateScene: put a scene({}): ray({}/{}, {})", curr_scene.id_, curr_scene.ray_num_, ray_num,
               committed_num);
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
  std::lock_guard<std::mutex> lock(prod_mutex);
  for (auto& s : simulators_) {
    s.SetLogLevel(level);
  }
}


// =============== Server ===============
Server::Server() : impl_(std::make_shared<ServerImpl>()) {}

Error Server::CommitConfig(const std::string& config_str) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  try {
    auto config_json = nlohmann::json::parse(config_str);
    return impl_->CommitConfig(config_json);
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

std::vector<RenderResult> Server::GetRenderResults() const {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return {};
  }
  return impl_->GetRenderResults();
}

std::optional<StatsResult> Server::GetStatsResult() const {
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
