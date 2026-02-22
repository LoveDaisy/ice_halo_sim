#include <benchmark/benchmark.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "config/config_manager.hpp"
#include "config/sim_data.hpp"
#include "core/simulator.hpp"
#include "util/queue.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace) benchmark code

namespace {

// Same minimal config as bench_scene.cpp.
const std::string kBenchConfigTemplate = R"({
  "light_source": [{
    "id": 1,
    "type": "sun",
    "altitude": 20.0,
    "azimuth": 0,
    "diameter": 0.5,
    "spectrum": [{"wavelength": 550, "weight": 1.0}]
  }],
  "crystal": [{
    "id": 1,
    "type": "prism",
    "shape": { "height": 1.3 }
  }],
  "filter": [{
    "id": 1,
    "type": "none"
  }],
  "scene": [{
    "id": 1,
    "light_source": 1,
    "ray_num": __RAY_NUM__,
    "max_hits": 7,
    "scattering": [{
      "crystal": [1],
      "proportion": [1],
      "prob": 0.0
    }]
  }],
  "render": [{
    "id": 1,
    "lens": { "type": "linear", "fov": 40 },
    "resolution": [320, 240]
  }],
  "project": {
    "id": 1,
    "scene": 1,
    "render": [1]
  }
})";

std::string MakeConfig(int64_t ray_num) {
  std::string config = kBenchConfigTemplate;
  auto pos = config.find("__RAY_NUM__");
  config.replace(pos, 11, std::to_string(ray_num));
  return config;
}

constexpr size_t kDefaultRayNum = 128;  // Same as ServerImpl::kDefaultRayNum

}  // namespace


class SimLoopFixture : public benchmark::Fixture {
 public:
  void SetUp(benchmark::State& state) override {
    auto ray_num = static_cast<size_t>(state.range(0));
    scene_queue_ = std::make_shared<Queue<SimBatch>>();
    data_queue_ = std::make_shared<Queue<SimData>>();
    simulator_ = std::make_unique<Simulator>(scene_queue_, data_queue_);

    auto config_json = nlohmann::json::parse(MakeConfig(ray_num));
    config_manager_ = config_json.get<ConfigManager>();
    scene_ = &config_manager_.project_.scene_;

    batch_count_ = (ray_num + kDefaultRayNum - 1) / kDefaultRayNum;
  }

  void TearDown(benchmark::State& /*state*/) override {
    simulator_.reset();
    scene_queue_.reset();
    data_queue_.reset();
  }

  QueuePtrS<SimBatch> scene_queue_;
  QueuePtrS<SimData> data_queue_;
  std::unique_ptr<Simulator> simulator_;
  ConfigManager config_manager_;
  const SceneConfig* scene_ = nullptr;
  size_t batch_count_ = 0;
};


BENCHMARK_DEFINE_F(SimLoopFixture, BM_SimLoop)(benchmark::State& state) {
  auto ray_num = state.range(0);

  for (auto _ : state) {
    // Enqueue all batches + termination signal.
    scene_queue_->Start();
    data_queue_->Start();
    size_t committed = 0;
    while (committed < static_cast<size_t>(ray_num)) {
      size_t batch_ray_num = std::min(kDefaultRayNum, static_cast<size_t>(ray_num) - committed);
      scene_queue_->Emplace(SimBatch{ batch_ray_num, scene_ });
      committed += kDefaultRayNum;
    }
    scene_queue_->Emplace(SimBatch{ 0, nullptr });  // Termination signal

    // Run simulator synchronously on current thread.
    simulator_->Run();

    // Drain data_queue to prevent accumulation.
    data_queue_->Shutdown();
    while (true) {
      auto data = data_queue_->Get();
      if (data.rays_.Empty()) {
        break;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * ray_num);
}
BENCHMARK_REGISTER_F(SimLoopFixture, BM_SimLoop)
    ->Arg(1000)
    ->Arg(10000)
    ->Arg(100000)
    ->Arg(1000000)
    ->Unit(benchmark::kMillisecond);
