#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
#include <memory>
#include <thread>

#include "config/config_manager.hpp"
#include "core/simulator.hpp"
#include "server/show_rays.hpp"
#include "util/queue.hpp"

extern std::string config_file_name;
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
class CopyRayDataConsumer : public lumice::IConsume {
 public:
  CopyRayDataConsumer(float* output_data) : output_data_(output_data) {}

  void Consume(const lumice::SimData& data) override {
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
  Consumer(lumice::QueuePtrS<lumice::SimData> data_queue) : data_queue_(data_queue), stop_(false) {}

  void RegisterConsumer(lumice::ConsumerPtrS consumer) { consumers_.emplace_back(std::move(consumer)); }

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
  std::vector<lumice::ConsumerPtrS> consumers_;
  lumice::QueuePtrS<lumice::SimData> data_queue_;
  std::atomic_bool stop_;
};

TEST_F(V3TestProj, SimpleProj) {
  lumice::ConfigManager config_manager = config_json_.get<lumice::ConfigManager>();

  auto config_queue = std::make_shared<lumice::Queue<lumice::SimBatch>>();
  auto data_queue = std::make_shared<lumice::Queue<lumice::SimData>>();

  constexpr int kMaxHits = 8;
  auto output_data = std::make_unique<float[]>(kMaxHits * 2 * 7);

  constexpr uint32_t kTestSeed = 42;
  lumice::Simulator simulator(config_queue, data_queue, kTestSeed);

  Consumer consumer(data_queue);
  consumer.RegisterConsumer(std::make_unique<lumice::ShowRayInfoConsumer>());
  consumer.RegisterConsumer(std::make_unique<CopyRayDataConsumer>(output_data.get()));

  std::thread prod_thread([&simulator]() { simulator.Run(); });
  std::thread cons_thread([&consumer]() { consumer.Run(); });

  const auto& config = config_manager.scene_;
  auto scene_ptr = std::make_shared<const lumice::SceneConfig>(config);
  config_queue->Emplace(lumice::SimBatch{ config.ray_num_, scene_ptr, 0 });

  std::this_thread::sleep_for(500ms);
  simulator.Stop();
  consumer.Stop();

  cons_thread.join();
  prod_thread.join();

  float expect_out[kMaxHits * 2 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.095397f,  0.444923f,  -0.145981f, -0.468416f, 0.813311f,  -0.345125f, 0.064267f,  //
    0.260609f,  0.349537f,  0.065001f,  -0.469655f, 0.814260f,  -0.341181f, 0.064202f,  //
    -0.433013f, 0.097239f,  -0.133934f, -0.834485f, -0.432701f, -0.341181f, 0.917328f,  //
    -0.433013f, 0.247318f,  -0.300137f, -0.470140f, -0.812316f, -0.345125f, 0.875597f,  //
    0.433013f,  -0.217769f, -0.382316f, 0.834485f,  -0.432700f, -0.341181f, 0.018106f,  //
    0.271664f,  -0.343155f, -0.551009f, 0.791578f,  -0.504274f, -0.345125f, 0.058947f,  //
    -0.132535f, -0.423481f, -0.544519f, -0.939997f, -0.000396f, -0.341181f, 0.000341f,  //
    0.137533f,  0.420595f,  -0.437393f, -0.468416f, 0.813311f,  0.345125f,  0.001113f,  //
    -0.268225f, 0.345140f,  -0.445112f, -0.042514f, 0.939035f,  0.341182f,  0.000023f,  //
    -0.433013f, 0.212492f,  -0.271672f, -0.832503f, -0.433390f, 0.345125f,  0.000075f,  //
    0.433013f,  -0.243513f, -0.198341f, 0.470341f,  -0.813864f, 0.341182f,  0.000000f,  //
    0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  //
    0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  //
    0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  //
    0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  //
    0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  //
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 5e-5);
    }
  }
}

}  // namespace