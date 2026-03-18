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

  void RegisterConsumer(lumice::ConsumerPtrU consumer) { consumers_.emplace_back(std::move(consumer)); }

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
  std::vector<lumice::ConsumerPtrU> consumers_;
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
    0.095397,  0.444923,  -0.145981, -0.470140, 0.812316,  -0.345125, 0.064824,  //
    0.260609,  0.349537,  0.065001,  -0.470342, 0.813863,  -0.341181, 0.064423,  //
    -0.433013, 0.097560,  -0.133905, -0.834737, -0.432214, -0.341181, 0.917116,  //
    -0.433013, 0.246560,  -0.300270, -0.468416, -0.813311, -0.345125, 0.874554,  //
    0.433013,  -0.217046, -0.382249, 0.834738,  -0.432214, -0.341181, 0.018096,  //
    0.270259,  -0.343966, -0.550857, 0.790838,  -0.505434, -0.345125, 0.059423,  //
    -0.133582, -0.422877, -0.544728, -0.939997, 0.000397,  -0.341181, 0.000341,  //
    0.134908,  0.422111,  -0.436862, -0.470140, 0.812316,  0.345125,  0.001121,  //
    -0.269372, 0.344478,  -0.445234, -0.043061, 0.939011,  0.341181,  0.000023,  //
    -0.433013, 0.215625,  -0.271965, -0.833138, -0.432169, 0.345125,  0.000076,  //
    0.433013,  -0.245627, -0.197975, 0.469655,  -0.814261, 0.341181,  0.000000,  //
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  //
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  //
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  //
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  //
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  //
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 5e-5);
    }
  }
}

}  // namespace