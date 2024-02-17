#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <memory>
#include <thread>

#include "config/config_manager.hpp"
#include "core/simulator.hpp"
#include "include/log.hpp"
#include "server/show_rays.hpp"
#include "util/queue.hpp"

extern std::string config_file_name;
using namespace icehalo;
using namespace std::chrono_literals;

namespace {

class V3TestProj : public ::testing::Test {
 protected:
  void SetUp() override {
    std::ifstream f(config_file_name);
    f >> config_json_;
  }

  nlohmann::json config_json_;
};


// For test
class CopyRayDataConsumer : public v3::IConsume {
 public:
  CopyRayDataConsumer(float* output_data) : output_data_(output_data) {}

  void Consume(const v3::SimData& data) override {
    int offset = 0;
    for (const auto& r : data.rays_) {
      if (r.fid_ >= 0 || r.w_ < 0) {
        continue;
      }
      std::memcpy(output_data_ + offset * 7 + 0, r.p_, 3 * sizeof(float));
      std::memcpy(output_data_ + offset * 7 + 3, r.d_, 3 * sizeof(float));
      output_data_[offset * 7 + 6] = r.w_;
      offset++;
    }
  }

 private:
  float* output_data_;
};

class Consumer {
 public:
  Consumer(icehalo::v3::QueuePtrS<icehalo::v3::SimData> data_queue) : data_queue_(data_queue), stop_(false) {}

  void RegisterConsumer(icehalo::v3::ConsumerPtrU consumer) { consumers_.emplace_back(std::move(consumer)); }

  void Run() {
    while (true) {
      auto data = data_queue_->Get();
      if (stop_ || data.rays_.Empty()) {
        break;
      }

      for (auto& c : consumers_) {
        c->Consume(data);
      }
      if (stop_) {
        break;
      }
    }
  }

  void Stop() { stop_ = true; }

 private:
  std::vector<icehalo::v3::ConsumerPtrU> consumers_;
  icehalo::v3::QueuePtrS<icehalo::v3::SimData> data_queue_;
  std::atomic_bool stop_;
};

TEST_F(V3TestProj, SimpleProj) {
  v3::ConfigManager config_manager = config_json_.get<v3::ConfigManager>();

  auto config_queue = std::make_shared<v3::Queue<v3::SceneConfig>>();
  auto data_queue = std::make_shared<v3::Queue<v3::SimData>>();

  constexpr int kMaxHits = 8;
  std::unique_ptr<float[]> output_data{ new float[kMaxHits * 2 * 7]{} };

  v3::Simulator simulator(config_queue, data_queue);

  Consumer consumer(data_queue);
  consumer.RegisterConsumer(v3::ConsumerPtrU(new v3::ShowRayInfoConsumer));
  consumer.RegisterConsumer(v3::ConsumerPtrU(new CopyRayDataConsumer(output_data.get())));

  std::thread prod_thread([&simulator]() { simulator.Run(); });
  std::thread cons_thread([&consumer]() { consumer.Run(); });

  auto config = config_manager.scenes_.at(1);
  config_queue->Emplace(std::move(config));

  std::this_thread::sleep_for(500ms);
  simulator.Stop();
  consumer.Stop();

  cons_thread.join();
  prod_thread.join();

  float expect_out[kMaxHits * 2 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.302099,  0.040812,  0.600000,  -0.939667, 0.003329,  0.342074,  0.127290,  //
    0.165777,  -0.404289, -0.316693, -0.467876, -0.814561, -0.342904, 0.063834,  //
    -0.433013, 0.043416,  -0.115191, -0.405241, 0.003329,  -0.914204, 0.794171,  //
    -0.433013, -0.185722, -0.489431, -0.833166, 0.433879,  -0.342904, 0.917659,  //
    0.065299,  0.045181,  -0.600000, 0.939667,  0.003329,  -0.342074, 0.068542,  //
    0.433013,  0.046484,  -0.242251, 0.405241,  0.003329,  0.914204,  0.009098,  //
    0.433013,  0.130390,  -0.460739, 0.833166,  0.433879,  0.342904,  0.018141,  //
    -0.432697, 0.049551,  0.600000,  -0.939667, 0.003329,  0.342074,  0.000785,  //
    -0.224493, 0.370389,  -0.271062, -0.939368, 0.002088,  0.342904,  0.000343,  //
    -0.433013, 0.049552,  0.599693,  -0.405241, 0.003329,  -0.914204, 0.000104,  //
    -0.342463, -0.302279, -0.085994, -0.040832, -0.938483, 0.342904,  0.000023,  //
    0.433013,  0.052620,  -0.242864, 0.405241,  0.003329,  -0.914204, 0.000009,  //
    0.364310,  0.289665,  0.163836,  0.792334,  0.504604,  0.342904,  0.000000,  //
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 5e-5);
    }
  }
}

}  // namespace