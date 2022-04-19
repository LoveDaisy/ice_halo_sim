#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstring>
#include <memory>
#include <thread>

#include "process/simulation.hpp"
#include "protocol/config_manager.hpp"
#include "util/log.hpp"
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


TEST_F(V3TestProj, SimpleProj) {
  v3::ConfigManager config_manager = config_json_.get<v3::ConfigManager>();

  auto config_queue = std::make_shared<v3::Queue<v3::SceneConfigPtrU>>();
  auto data_queue = std::make_shared<v3::Queue<v3::SimBasicDataPtrU>>();

  constexpr int kMaxHits = 8;

  auto config = std::make_unique<v3::SceneConfig>(config_manager.scenes_.at(1));
  config_queue->Emplace(std::move(config));

  v3::Simulator simulator(config_queue, data_queue);
  std::unique_ptr<float[]> output_data{ new float[kMaxHits * 2 * 7]{} };
  float* output_data_ptr = output_data.get();

  std::thread producer([&simulator]() { simulator.Run(); });
  std::thread consumer([=]() {
    int offset = 0;
    while (true) {
      auto data = data_queue->Get();
      if (!data || data->Empty()) {
        break;
      }
      LOG_DEBUG("p  d  w");
      for (size_t i = 0; i < data->size_; i++) {
        const auto& ray = data->rays_[i];
        LOG_DEBUG("%.6f,%.6f,%.6f  %.6f,%.6f,%.6f  %.6f", ray.p_[0], ray.p_[1], ray.p_[2], ray.d_[0], ray.d_[1],
                  ray.d_[2], ray.w_);
        std::memcpy(output_data_ptr + offset * 7 + 0, ray.p_, 3 * sizeof(float));
        std::memcpy(output_data_ptr + offset * 7 + 3, ray.d_, 3 * sizeof(float));
        output_data_ptr[offset * 7 + 6] = ray.w_;
        offset++;
      }
    }
  });

  std::this_thread::sleep_for(500ms);
  simulator.Stop();

  consumer.join();
  producer.join();

  float expect_out[kMaxHits * 2 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.433013,  -0.203831, 0.316693,  1.000000,  -0.000000, 0.000000,  0.018111,  // data 1,1
    0.433013,  0.037085,  -0.134507, 1.000000,  -0.000000, 0.000000,  0.018111,  // data 1,2
    -0.433013, -0.203831, 0.316693,  -1.000000, -0.000000, 0.000000,  0.964105,  // data 2,1
    -0.433013, 0.037085,  -0.134507, -1.000000, -0.000000, -0.000000, 0.964105,  // data 2,2
    0.433013,  -0.203831, 0.316693,  1.000000,  -0.000000, 0.000000,  0.017461,  // data 3,1
    0.433013,  0.037085,  -0.134507, 1.000000,  -0.000000, -0.000000, 0.017461,  // data 3,2
    -0.433013, -0.203831, 0.316693,  -1.000000, -0.000000, 0.000000,  0.000316,  // data 4,1
    -0.433013, 0.037085,  -0.134507, -1.000000, -0.000000, -0.000000, 0.000316,  // data 4,2
    0.433013,  -0.203831, 0.316693,  1.000000,  -0.000000, 0.000000,  0.000006,  // data 5,1
    0.433013,  0.037085,  -0.134507, 1.000000,  -0.000000, -0.000000, 0.000006,  // data 5,2
    -0.433013, -0.203831, 0.316693,  -1.000000, -0.000000, 0.000000,  0.000000,  // data 6,1
    -0.433013, 0.037085,  -0.134507, -1.000000, -0.000000, -0.000000, 0.000000,  // data 6,2
    0.433013,  -0.203831, 0.316693,  1.000000,  -0.000000, 0.000000,  0.000000,  // data 7,1
    0.433013,  0.037085,  -0.134507, 1.000000,  -0.000000, -0.000000, 0.000000,  // data 7,2
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 1e-5);
    }
  }
}

}  // namespace