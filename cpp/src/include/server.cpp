#include "include/server.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "config/config_manager.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "consumer/consumer.hpp"
#include "consumer/render.hpp"
#include "core/def.hpp"
#include "core/simulator.hpp"
#include "include/result.hpp"
#include "json.hpp"
#include "util/log.hpp"
#include "util/queue.hpp"

namespace icehalo {
namespace v3 {

// =============== ServerImpl ===============
class ServerImpl {
 public:
  ServerImpl();

  void CommitConfig(const nlohmann::json& config_json);
  std::vector<Result> GetResults();

  void Stop();
  void Start();
  bool IsIdle() const;

 private:
  static constexpr int kDefaultSimulators = 4;
  static constexpr int kMaxSceneCnt = 100;
  static constexpr size_t kDefaultRayNum = 1024;

  void MainWorker();
  void SceneDispatcher();

  ConfigManager config_manager_;

  Queue<ProjConfig> proj_queue_;
  QueuePtrS<SceneConfig> scene_queue_;
  QueuePtrS<SimData> data_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrU> consumers_;
  std::vector<std::thread> simulator_threads_;
  std::vector<std::thread> consumer_threads_;
  std::mutex prod_cons_mutex_;

  std::atomic_bool stop_;
  std::atomic_int sim_scene_cnt_;
  std::atomic<SceneConfig*> curr_scene_ptr_;
  std::condition_variable scene_cv_;
  std::mutex scene_mutex_;

  std::thread main_working_thread_;
  std::thread scene_dispatch_thread_;
};


ServerImpl::ServerImpl()
    : config_manager_{}, proj_queue_{}, scene_queue_(std::make_shared<Queue<SceneConfig>>()),
      data_queue_(std::make_shared<Queue<SimData>>()) {
  for (int i = 0; i < kDefaultSimulators; i++) {
    simulators_.emplace_back(scene_queue_, data_queue_);
  }
  Start();
}


void ServerImpl::CommitConfig(const nlohmann::json& config_json) {
  LOG_DEBUG("ServerImpl::CommitConfig: entry");
  config_manager_ = config_json.get<ConfigManager>();

  Stop();
  Start();

  // Commit all projects
  for (const auto& [_, proj] : config_manager_.projects_) {
    LOG_DEBUG("ServerImpl::CommitConfig: put proj %u", proj.id_);
    proj_queue_.Emplace(proj);
  }
}


std::vector<Result> ServerImpl::GetResults() {
  std::vector<Result> results;
  std::unique_lock<std::mutex> lock(prod_cons_mutex_);
  LOG_DEBUG("ServerImpl::GetResults: lock on prod_cons_mutex");
  for (const auto& c : consumers_) {
    results.emplace_back(c->GetResult());
  }
  LOG_DEBUG("ServerImpl::GetResults: unlock prod_cons_mutex");
  return results;
}


void ServerImpl::Start() {
  LOG_DEBUG("ServerImpl::Start: entry");
  if (!stop_) {
    return;
  }

  stop_ = false;
  sim_scene_cnt_ = 0;
  curr_scene_ptr_ = nullptr;

  // Start queues & jobs
  data_queue_->Start();
  scene_queue_->Start();
  proj_queue_.Start();
  {
    std::unique_lock<std::mutex> lock(prod_cons_mutex_);
    LOG_DEBUG("ServerImpl::Start: lock on prod_cons_mutex");
    for (auto& s : simulators_) {
      simulator_threads_.emplace_back(&Simulator::Run, &s);
    }
    LOG_DEBUG("ServerImpl::Start: unlock prod_cons_mutex");
  }

  // Start main working thread & scene dispatch thread
  main_working_thread_ = std::thread(&ServerImpl::MainWorker, this);
  scene_dispatch_thread_ = std::thread(&ServerImpl::SceneDispatcher, this);
}


void ServerImpl::Stop() {
  LOG_DEBUG("ServerImpl::Stop: entry");
  if (stop_) {
    return;
  }

  stop_ = true;

  // Stop queues
  proj_queue_.Shutdown();
  scene_queue_->Shutdown();
  data_queue_->Shutdown();

  scene_cv_.notify_all();

  // Stop running jobs
  {
    std::unique_lock<std::mutex> lock(prod_cons_mutex_);
    LOG_DEBUG("ServerImpl::Stop: lock on prod_cons_mutex");
    for (auto& s : simulators_) {
      s.Stop();
    }
    LOG_DEBUG("ServerImpl::Stop: unlock prod_cons_mutex");
  }
  for (auto& t : simulator_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  for (auto& t : consumer_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  simulator_threads_.clear();
  consumer_threads_.clear();
  consumers_.clear();

  // Stop main working thread & scene dispatch thread
  LOG_DEBUG("ServerImpl::Stop: waiting main worker & dispatcher stop.");
  if (main_working_thread_.joinable()) {
    main_working_thread_.join();
  }
  if (scene_dispatch_thread_.joinable()) {
    scene_dispatch_thread_.join();
  }
}

bool ServerImpl::IsIdle() const {
  return !stop_ && curr_scene_ptr_ == nullptr;
}


#define CHECK_STOP \
  if (stop_) {     \
    break;         \
  }


void ServerImpl::MainWorker() {
  LOG_DEBUG("ServerImpl::MainWorker: entry");
  while (true) {
    CHECK_STOP
    auto proj_config = proj_queue_.Get();
    if (stop_ || (proj_config.id_ == 0 && proj_config.scene_.ray_num_ == 0)) {
      // Stop, or queue shutdown.
      break;
    }
    LOG_DEBUG("ServerImpl::MainWorker: get proj: %u", proj_config.id_);

    CHECK_STOP
    // Setup consumers.
    {
      std::unique_lock<std::mutex> lock(prod_cons_mutex_);
      LOG_DEBUG("ServerImpl::MainWorker: lock on prod_cons_mutex. init consumers.");
      for (const auto& r : proj_config.renderers_) {
        consumers_.emplace_back(ConsumerPtrU{ new Renderer(r) });
      }
      LOG_DEBUG("ServerImpl::MainWorker: unlock prod_cons_mutex. init consumers finishes.");
    }

    // Notify scene dispatch thread
    curr_scene_ptr_ = &proj_config.scene_;
    scene_cv_.notify_one();

    // Consume data & wait for current project finishing.
    while (true) {
      CHECK_STOP
      auto sim_data = data_queue_->Get();
      if (sim_data.rays_.Empty()) {
        // Simulation is interrupted.
        break;
      } else {
        sim_scene_cnt_--;
      }
      scene_cv_.notify_one();
      CHECK_STOP

      LOG_DEBUG("ServerImpl::MainWorker: get data: %p", &sim_data);

      {
        std::unique_lock<std::mutex> lock(prod_cons_mutex_);
        LOG_DEBUG("ServerImpl::MainWorker: lock on prod_cons_mutex. consume data.");
        for (auto& c : consumers_) {
          c->Consume(sim_data);
        }
        LOG_DEBUG("ServerImpl::MainWorker: unlock prod_cons_mutex. consume data finishes.");
      }

      if (sim_scene_cnt_ <= 0) {
        // Project is finished.
        break;
      }
    }
    CHECK_STOP
  }
  LOG_DEBUG("ServerImpl::MainWorker exit");
}


void ServerImpl::SceneDispatcher() {
  LOG_DEBUG("ServerImpl::SceneDispatcher entry");
  while (true) {
    CHECK_STOP
    if (!curr_scene_ptr_) {
      // All projects are finished. Just wait for next.
      std::unique_lock<std::mutex> lock(scene_mutex_);
      scene_cv_.wait(lock, [=]() { return stop_ || curr_scene_ptr_ != nullptr; });
    }
    CHECK_STOP

    auto ray_num = curr_scene_ptr_.load()->ray_num_;
    size_t committed_num = 0;
    while (curr_scene_ptr_ && (ray_num == kInfSize || committed_num < ray_num)) {
      const auto& wls = curr_scene_ptr_.load()->light_source_.wl_param_;
      for (const auto& wl_param : wls) {
        auto curr_scene = *curr_scene_ptr_;
        curr_scene.ray_num_ = std::min(kDefaultRayNum, ray_num - committed_num);
        curr_scene.light_source_.wl_param_.clear();
        curr_scene.light_source_.wl_param_.emplace_back(wl_param);
        scene_queue_->Emplace(curr_scene);
        sim_scene_cnt_++;
        committed_num += kDefaultRayNum;

        LOG_DEBUG("ServerImpl::SceneDispatcher: put a scene(%u): ray(%zu/%zu, %zu), wl(%.1f,%.2f)", curr_scene.id_,
                  curr_scene.ray_num_, ray_num, committed_num, wl_param.wl_, wl_param.weight_);

        CHECK_STOP
        if (sim_scene_cnt_ >= kMaxSceneCnt) {
          std::unique_lock<std::mutex> lock(scene_mutex_);
          scene_cv_.wait(lock, [=]() { return stop_ || sim_scene_cnt_ < kMaxSceneCnt || curr_scene_ptr_ == nullptr; });
        }
        CHECK_STOP
        if (!curr_scene_ptr_) {
          break;
        }
      }
      LOG_DEBUG("ServerImpl::SceneDispatcher: finish wl");
      CHECK_STOP
    }
    curr_scene_ptr_ = nullptr;
    LOG_DEBUG("ServerImpl::SceneDispatcher: finish ray_num");
  }
  LOG_DEBUG("ServerImpl::SceneDispatcher exit");
}


// =============== Server ===============
Server::Server() : impl_(std::make_shared<ServerImpl>()) {}

void Server::CommitConfig(std::ifstream& f) {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return;
  }
  nlohmann::json config_json;
  f >> config_json;
  impl_->CommitConfig(config_json);
}

void Server::CommitConfig(std::string config_str) {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return;
  }
  nlohmann::json config_json(config_str);
  impl_->CommitConfig(config_json);
}

std::vector<Result> Server::GetResults() {
  if (!impl_) {
    LOG_WARNING("Server is terminated!");
    return {};
  }
  return impl_->GetResults();
}

void Server::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
}

void Server::Terminate() {
  LOG_DEBUG("Server::Terminate: entry");
  if (!impl_) {
    return;
  }
  impl_->Stop();
  impl_ = nullptr;
}

bool Server::IsIdle() const {
  if (!impl_) {
    return false;
  }
  return impl_->IsIdle();
}

}  // namespace v3
}  // namespace icehalo
