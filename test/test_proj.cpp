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

  // Fixture regenerated 2026-04-25 after rotation chain rework
  // (R = Rz(az - pi) * Ry(-zenith) * Rz(roll), see core/simulator.cpp::BuildCrystalRotation
  // and doc/coordinate-convention.md). Captured with seed=42 from
  // examples/config_example.json crystal id=1 (default axis = canonical pose).
  float expect_out[kMaxHits * 2 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.433013f,  -0.060825f, -0.335629f, 0.938556f,  -0.000995f, -0.345125f, 0.018287f,  //
    0.433013f,  -0.050926f, -0.412778f, 0.939997f,  -0.000397f, -0.341181f, 0.018278f,  //
    -0.433013f, -0.061507f, -0.571946f, -0.938556f, -0.000995f, -0.345125f, 0.963760f,  //
    -0.433013f, -0.051197f, -0.553803f, -0.939997f, -0.000397f, 0.341182f,  0.963778f,  //
    0.433013f,  -0.062189f, -0.391737f, 0.938556f,  -0.000996f, 0.345125f,  0.017624f,  //
    0.433013f,  -0.051469f, -0.320383f, 0.939997f,  -0.000397f, 0.341182f,  0.017616f,  //
    -0.433013f, -0.062870f, -0.155419f, -0.938556f, -0.000996f, 0.345125f,  0.000322f,  //
    -0.433013f, -0.051740f, -0.086963f, -0.939997f, -0.000397f, 0.341182f,  0.000322f,  //
    0.433013f,  -0.063552f, 0.080898f,  0.938556f,  -0.000996f, 0.345125f,  0.000006f,  //
    0.433013f,  -0.052012f, 0.146456f,  0.939997f,  -0.000397f, 0.341182f,  0.000006f,  //
    -0.433013f, -0.064234f, 0.317215f,  -0.938556f, -0.000996f, 0.345125f,  0.000000f,  //
    -0.433013f, -0.052284f, 0.379876f,  -0.939997f, -0.000397f, 0.341182f,  0.000000f,  //
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