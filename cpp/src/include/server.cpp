#include "include/server.hpp"

#include <cstddef>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>

#include "config/config_manager.hpp"
#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "consumer/consumer.hpp"
#include "consumer/render.hpp"
#include "core/simulator.hpp"
#include "json.hpp"
#include "util/queue.hpp"

namespace icehalo {
namespace v3 {

// =============== ServerImpl ===============
class ServerImpl {
 public:
  void CommitConfig(const nlohmann::json& config_json);
  Result GetResult();

 private:
  static constexpr int kDefaultSimulators = 4;
  static constexpr size_t kDefaultRayNum = 1024;

  void WorkerFunc();

  ConfigManager config_manager_;
  std::thread working_thread_;

  Queue<ProjConfig> proj_queue_;
  QueuePtrS<SceneConfig> scene_queue_;
  QueuePtrS<SimData> data_queue_;
  QueuePtrS<Result> result_queue_;

  std::vector<Simulator> simulators_;
  std::vector<ConsumerPtrU> consumers_;
  std::vector<std::thread> simulator_threads_;
  std::vector<std::thread> consumer_threads_;
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
  result_queue_->Shutdown();
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
  result_queue_->Start();
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

Result ServerImpl::GetResult() {
  return result_queue_->Get();
}

void ServerImpl::WorkerFunc() {
  while (true) {
    auto proj_config = proj_queue_.Get();
    if (proj_config.id_ == 0 && proj_config.scene_.ray_num_ == 0) {
      // Queue is shutdown, so just continue to wait new config.
      continue;
    }

    // Setup consumers.
    for (const auto& r : proj_config.renderers_) {
      consumers_.emplace_back(ConsumerPtrU{ new Renderer(r) });
    }

    // Split config.
    const auto& proj_scene = proj_config.scene_;
    int scene_cnt = 0;
    for (const auto& wl_param : proj_scene.light_source_.wl_param_) {
      for (size_t i = 0; i < proj_scene.ray_num_; i += kDefaultRayNum) {
        auto curr_scene = proj_scene;
        curr_scene.ray_num_ = std::min(kDefaultRayNum, proj_scene.ray_num_ - i - 1);
        curr_scene.light_source_.wl_param_.clear();
        curr_scene.light_source_.wl_param_.emplace_back(wl_param);
        scene_queue_->Emplace(curr_scene);
        scene_cnt++;
      }
    }

    // Consume data & wait for current project finishing.
    while (true) {
      auto sim_data = data_queue_->Get();
      if (sim_data.rays_.Empty()) {
        // Simulation is interrupted.
        break;
      } else {
        scene_cnt--;
      }

      for (auto& c : consumers_) {
        c->Consume(sim_data);
      }

      if (scene_cnt <= 0) {
        // Project is finished.
        break;
      }
    }
  }
}


// =============== Server ===============
Server::Server() : impl_(std::make_shared<ServerImpl>()) {}

void Server::CommitConfig(std::ifstream& f) {
  nlohmann::json config_json;
  f >> config_json;
  impl_->CommitConfig(config_json);
}

Result Server::GetResult() {
  return NoneResult{};
}

}  // namespace v3
}  // namespace icehalo
