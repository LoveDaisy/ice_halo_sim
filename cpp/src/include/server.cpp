#include "include/server.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
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
#include "util/queue.hpp"

namespace icehalo {
namespace v3 {

// =============== ServerImpl ===============
class ServerImpl {
 public:
  void CommitConfig(const nlohmann::json& config_json);
  std::vector<Result> GetResults() const;

  void Stop();
  void Start();

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

  std::atomic_bool stop_;
  std::atomic_int sim_scene_cnt_;
  std::atomic<SceneConfig*> curr_scene_ptr_;
  std::condition_variable scene_cv_;
  std::mutex scene_mutex_;

  std::thread main_working_thread_;
  std::thread scene_dispatch_thread_;
};

void ServerImpl::CommitConfig(const nlohmann::json& config_json) {
  config_manager_ = config_json.get<ConfigManager>();

  // Stop all running jobs and queues
  for (auto& s : simulators_) {
    s.Stop();
  }
  proj_queue_.Shutdown();
  scene_queue_->Shutdown();
  data_queue_->Shutdown();
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

  // Start all jobs and queues
  data_queue_->Start();
  scene_queue_->Start();
  proj_queue_.Start();
  for (auto& s : simulators_) {
    simulator_threads_.emplace_back([&s]() { s.Run(); });
  }

  // Commit all projects
  for (const auto& [_, proj] : config_manager_.projects_) {
    proj_queue_.Emplace(proj);
  }
}

std::vector<Result> ServerImpl::GetResults() const {
  std::vector<Result> results;
  for (const auto& c : consumers_) {
    results.emplace_back(c->GetResult());
  }
  return results;
}

#define CHECK_STOP \
  if (stop_) {     \
    break;         \
  }

void ServerImpl::MainWorker() {
  while (true) {
    CHECK_STOP
    auto proj_config = proj_queue_.Get();
    if (proj_config.id_ == 0 && proj_config.scene_.ray_num_ == 0) {
      // Queue is shutdown, so just continue to wait new config.
      continue;
    }

    CHECK_STOP
    // Setup consumers.
    for (const auto& r : proj_config.renderers_) {
      consumers_.emplace_back(ConsumerPtrU{ new Renderer(r) });
    }

    // Split config.
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
      for (auto& c : consumers_) {
        c->Consume(sim_data);
      }

      if (sim_scene_cnt_ <= 0) {
        // Project is finished.
        break;
      }
    }
    CHECK_STOP
  }
}

void ServerImpl::SceneDispatcher() {
  while (true) {
    CHECK_STOP
    if (!curr_scene_ptr_) {
      std::unique_lock<std::mutex> lock(scene_mutex_);
      scene_cv_.wait(lock, [=]() { return stop_ || curr_scene_ptr_ != nullptr; });
    }

    auto ray_num = curr_scene_ptr_.load()->ray_num_;
    size_t total_num = 0;
    while (curr_scene_ptr_ && (ray_num == kInfSize || total_num < ray_num)) {
      const auto& wls = curr_scene_ptr_.load()->light_source_.wl_param_;
      for (const auto& wl_param : wls) {
        auto curr_scene = *curr_scene_ptr_;
        curr_scene.ray_num_ = std::min(kDefaultRayNum, ray_num - total_num - 1);
        curr_scene.light_source_.wl_param_.clear();
        curr_scene.light_source_.wl_param_.emplace_back(wl_param);
        scene_queue_->Emplace(curr_scene);
        sim_scene_cnt_++;
        total_num += kDefaultRayNum;

        CHECK_STOP
        if (sim_scene_cnt_ >= kMaxSceneCnt) {
          std::unique_lock<std::mutex> lock(scene_mutex_);
          scene_cv_.wait(lock, [=]() { return stop_ || sim_scene_cnt_ < kMaxSceneCnt || curr_scene_ptr_ == nullptr; });
        }
        if (!curr_scene_ptr_) {
          break;
        }
        CHECK_STOP
      }
      CHECK_STOP
    }
  }
}

void ServerImpl::Start() {
  stop_ = false;
  sim_scene_cnt_ = 0;
  curr_scene_ptr_ = nullptr;
  main_working_thread_ = std::thread(&ServerImpl::MainWorker, this);
  scene_dispatch_thread_ = std::thread(&ServerImpl::SceneDispatcher, this);
}

void ServerImpl::Stop() {
  stop_ = true;
  if (main_working_thread_.joinable()) {
    main_working_thread_.join();
  }
  if (scene_dispatch_thread_.joinable()) {
    scene_dispatch_thread_.join();
  }
}


// =============== Server ===============
Server::Server() : impl_(std::make_shared<ServerImpl>()) {}

void Server::CommitConfig(std::ifstream& f) {
  nlohmann::json config_json;
  f >> config_json;
  impl_->CommitConfig(config_json);
}

std::vector<Result> Server::GetResults() const {
  return impl_->GetResults();
}

}  // namespace v3
}  // namespace icehalo
