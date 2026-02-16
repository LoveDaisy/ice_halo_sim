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
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/def.hpp"
#include "core/simulator.hpp"
#include "server/consumer.hpp"
#include "server/render.hpp"
#include "server/server.hpp"
#include "server/stats.hpp"
#include "util/logger.hpp"
#include "util/queue.hpp"

namespace icehalo {

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

  Queue<ProjConfig> proj_queue_;
  QueuePtrS<SceneConfig> scene_queue_;
  QueuePtrS<SimData> data_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrU> consumers_;
  std::vector<std::thread> simulator_threads_;
  mutable std::mutex prod_mutex;

  std::atomic_bool stop_;
  std::atomic_bool work_started_;  // true after GenerateScene picks up first project
  std::atomic_int sim_scene_cnt_;
  std::mutex scene_mutex_;
  std::condition_variable scene_cv_;

  std::thread consume_data_thread_;
  std::thread generate_scene_thread_;

  mutable std::mutex status_mutex_;
  ServerStatus status_;

  std::shared_ptr<spdlog::logger> logger_;
};


ServerImpl::ServerImpl()
    : config_manager_{}, proj_queue_{}, scene_queue_(std::make_shared<Queue<SceneConfig>>()),
      data_queue_(std::make_shared<Queue<SimData>>()), status_(ServerStatus::kIdle), logger_(GetLogger("ServerImpl")) {
  for (int i = 0; i < kDefaultSimulatorCnt; i++) {
    simulators_.emplace_back(scene_queue_, data_queue_);
  }
  Start();
}


Error ServerImpl::CommitConfig(const nlohmann::json& config_json) {
  SPDLOG_LOGGER_DEBUG(logger_, "CommitConfig: entry");
  try {
    config_manager_ = config_json.get<ConfigManager>();
  } catch (const nlohmann::json::out_of_range& e) {
    SPDLOG_LOGGER_ERROR(logger_, "CommitConfig: Missing field: {}", e.what());
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::MissingField(e.what());
  } catch (const nlohmann::json::exception& e) {
    SPDLOG_LOGGER_ERROR(logger_, "CommitConfig: JSON parsing error: {}", e.what());
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::InvalidJson(e.what());
  } catch (const std::exception& e) {
    SPDLOG_LOGGER_ERROR(logger_, "CommitConfig: Configuration error: {}", e.what());
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::InvalidConfig(e.what());
  } catch (...) {
    SPDLOG_LOGGER_ERROR(logger_, "CommitConfig: Unknown error");
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_ = ServerStatus::kError;
    }
    return Error::InvalidConfig("Unknown configuration error");
  }

  Stop();
  Start();

  // Commit all projects
  for (const auto& [_, proj] : config_manager_.projects_) {
    SPDLOG_LOGGER_DEBUG(logger_, "CommitConfig: put proj {}", proj.id_);
    proj_queue_.Emplace(proj);
  }

  return Error::Success();
}


std::vector<RenderResult> ServerImpl::GetRenderResults() const {
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
  for (const auto& c : consumers_) {
    auto result = c->GetResult();
    if (auto* stats = std::get_if<StatsResult>(&result)) {
      return *stats;
    }
  }
  return std::nullopt;
}


void ServerImpl::Start() {
  SPDLOG_LOGGER_DEBUG(logger_, "Start: entry");
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
  proj_queue_.Start();
  {
    std::lock_guard<std::mutex> lock(prod_mutex);
    SPDLOG_LOGGER_DEBUG(logger_, "Start: lock on prod_mutex");
    for (auto& s : simulators_) {
      simulator_threads_.emplace_back(&Simulator::Run, &s);
    }
    SPDLOG_LOGGER_DEBUG(logger_, "Start: unlock prod_mutex");
  }

  // Start main working thread & scene dispatch thread
  consume_data_thread_ = std::thread(&ServerImpl::ConsumeData, this);
  generate_scene_thread_ = std::thread(&ServerImpl::GenerateScene, this);
}


void ServerImpl::Stop() {
  SPDLOG_LOGGER_DEBUG(logger_, "Stop: entry");
  if (stop_) {
    return;
  }

  stop_ = true;

  // Stop queues
  proj_queue_.Shutdown();
  scene_queue_->Shutdown();
  data_queue_->Shutdown();

  scene_cv_.notify_one();

  // Stop running jobs
  {
    std::lock_guard<std::mutex> lock(prod_mutex);
    SPDLOG_LOGGER_DEBUG(logger_, "Stop: lock on prod_mutex. stop simulators. clear simulator threads.");
    for (auto& s : simulators_) {
      s.Stop();
    }
    for (auto& t : simulator_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    simulator_threads_.clear();
    SPDLOG_LOGGER_DEBUG(logger_, "Stop: unlock prod_mutex");
  }
  consumers_.clear();

  // Stop main working thread & scene dispatch thread
  SPDLOG_LOGGER_DEBUG(logger_, "Stop: waiting main worker & dispatcher stop.");
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
  // During the startup window (threads spawned but GenerateScene hasn't picked up
  // a project yet), work_started_ is false — report kRunning to avoid false idle.
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
  SPDLOG_LOGGER_DEBUG(logger_, "ConsumeData: entry");
  while (true) {
    CHECK_STOP
    auto sim_data = data_queue_->Get();
    if (sim_data.rays_.Empty()) {
      // Simulation is interrupted.
      break;
    }
    CHECK_STOP

    SPDLOG_LOGGER_DEBUG(logger_, "ConsumeData: get data: {}", fmt::ptr(&sim_data));

    if (sim_scene_cnt_ > 0) {
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
  SPDLOG_LOGGER_DEBUG(logger_, "ConsumeData exit");
}


void ServerImpl::GenerateScene() {
  SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene entry");
  while (true) {
    CHECK_STOP
    auto proj_config = proj_queue_.Get();
    if (stop_ || (proj_config.id_ == 0 && proj_config.scene_.ray_num_ == 0)) {
      // Stop, or queue shutdown.
      break;
    }
    SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene: get proj: {}", proj_config.id_);
    work_started_ = true;
    CHECK_STOP

    // Setup consumers.
    std::vector<ConsumerPtrU> new_consumers;
    for (const auto& r : proj_config.renderers_) {
      new_consumers.emplace_back(ConsumerPtrU{ new RenderConsumer(r) });
    }
    new_consumers.emplace_back(ConsumerPtrU{ new StatsConsumer });
    consumers_.swap(new_consumers);

    auto ray_num = proj_config.scene_.ray_num_;
    size_t committed_num = 0;
    while (ray_num == kInfSize || committed_num < ray_num) {
      auto curr_scene = proj_config.scene_;
      curr_scene.ray_num_ = std::min(kDefaultRayNum, ray_num - committed_num);
      scene_queue_->Emplace(curr_scene);
      sim_scene_cnt_++;

      SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene: put a scene({}): ray({}/{}, {})", curr_scene.id_,
                          curr_scene.ray_num_, ray_num, committed_num);
      CHECK_STOP

      if (sim_scene_cnt_ >= kMaxSceneCnt) {
        SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene: too many scenes generated. wait for consumer");
        std::unique_lock<std::mutex> lock(scene_mutex_);
        scene_cv_.wait(lock, [=]() { return stop_ || sim_scene_cnt_ < kMaxSceneCnt; });
        SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene: continue to generate scenes.");
      }
      CHECK_STOP
      committed_num += kDefaultRayNum;
      SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene: finish wl");
    }
    SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene: finish ray_num");
  }
  SPDLOG_LOGGER_DEBUG(logger_, "GenerateScene exit");
}


// =============== Server ===============
static auto& ServerLogger() {
  static auto logger = GetLogger("Server");
  return logger;
}

Server::Server() : impl_(std::make_shared<ServerImpl>()) {}

Error Server::CommitConfig(const std::string& config_str) {
  if (!impl_) {
    return Error::ServerNotReady("Server is terminated");
  }
  try {
    auto config_json = nlohmann::json::parse(config_str);
    return impl_->CommitConfig(config_json);
  } catch (const nlohmann::json::parse_error& e) {
    SPDLOG_LOGGER_ERROR(ServerLogger(), "CommitConfig: JSON parse error: {}", e.what());
    return Error::InvalidJson(e.what());
  } catch (...) {
    SPDLOG_LOGGER_ERROR(ServerLogger(), "CommitConfig: Unknown error");
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
    SPDLOG_LOGGER_ERROR(ServerLogger(), "CommitConfigFromFile: JSON parse error: {}", e.what());
    return Error::InvalidJson(e.what());
  } catch (...) {
    SPDLOG_LOGGER_ERROR(ServerLogger(), "CommitConfigFromFile: Unknown error");
    return Error::InvalidJson("Unknown JSON parsing error");
  }
}

std::vector<RenderResult> Server::GetRenderResults() const {
  if (!impl_) {
    SPDLOG_LOGGER_WARN(ServerLogger(), "Server is terminated!");
    return {};
  }
  return impl_->GetRenderResults();
}

std::optional<StatsResult> Server::GetStatsResult() const {
  if (!impl_) {
    SPDLOG_LOGGER_WARN(ServerLogger(), "Server is terminated!");
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
  SPDLOG_LOGGER_DEBUG(ServerLogger(), "Terminate: entry");
  if (!impl_) {
    return;
  }
  impl_->Stop();
  impl_ = nullptr;
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

}  // namespace icehalo
