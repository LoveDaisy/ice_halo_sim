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

  const auto& config = config_manager.scenes_.at(1);
  config_queue->Emplace(lumice::SimBatch{ config.ray_num_, &config });

  std::this_thread::sleep_for(500ms);
  simulator.Stop();
  consumer.Stop();

  cons_thread.join();
  prod_thread.join();

  float expect_out[kMaxHits * 2 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.163830,  0.405413,  -0.118390, -0.470140, 0.812316,  -0.345125, 0.064824,  //
    0.239961,  0.361458,  0.412778,  -0.470342, 0.813863,  -0.341181, 0.064423,  //
    -0.433013, 0.188412,  -0.291685, -0.833138, -0.432169, -0.345125, 0.916688,  //
    -0.433013, 0.116982,  0.219793,  -0.834737, -0.432214, -0.341181, 0.917117,  //
    0.433013,  -0.126459, -0.543137, 0.833138,  -0.432169, -0.345125, 0.018123,  //
    0.433013,  -0.197624, -0.028551, 0.834737,  -0.432214, -0.341181, 0.018096,  //
    -0.154229, -0.410956, -0.196951, -0.939997, 0.000396,  -0.341181, 0.000341,  //
    -0.229672, -0.367399, -0.464452, -0.938556, 0.000995,  0.345125,  0.000342,  //
    -0.286191, 0.334767,  -0.401068, -0.043060, 0.939011,  -0.341181, 0.000023,  //
    -0.347479, 0.299383,  -0.279685, -0.042300, 0.937603,  0.345125,  0.000023,  //
    0.358220,  -0.293182, -0.028233, 0.790839,  -0.505435, 0.345125,  0.000000,  //
    0.419279,  -0.257929, -0.550587, 0.791677,  -0.506797, 0.341181,  0.000000,  //
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 5e-5);
    }
  }
}

}  // namespace