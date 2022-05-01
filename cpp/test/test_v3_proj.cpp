#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <memory>
#include <thread>

#include "config/config_manager.hpp"
#include "consumer/show_rays.hpp"
#include "core/simulator.hpp"
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


// For test
class CopyRayDataConsumer : public v3::IConsume {
 public:
  CopyRayDataConsumer(float* output_data) : output_data_(output_data) {}

  void Consume(const v3::SimData& data) override {
    int offset = 0;
    for (const auto& r : data.rays_) {
      if (r.fid_ > 0 || r.w_ < 0) {
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

TEST_F(V3TestProj, SimpleProj) {
  v3::ConfigManager config_manager = config_json_.get<v3::ConfigManager>();

  auto config_queue = std::make_shared<v3::Queue<v3::SceneConfig>>();
  auto data_queue = std::make_shared<v3::Queue<v3::SimData>>();

  constexpr int kMaxHits = 8;
  std::unique_ptr<float[]> output_data{ new float[kMaxHits * 2 * 7]{} };
  float* output_data_ptr = output_data.get();

  v3::Simulator simulator(config_queue, data_queue);

  v3::Consumer consumer(data_queue);
  consumer.RegisterConsumer(v3::ConsumerPtrU(new v3::ShowRaysInfo));
  consumer.RegisterConsumer(v3::ConsumerPtrU(new CopyRayDataConsumer(output_data_ptr)));

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
    -0.250816, 0.026764, 0.600000,  -0.940760, 0.003027, 0.339059,  0.129463,  // data 1,1
    0.433013,  0.037085, -0.134507, 0.939679,  0.003654, -0.342039, 0.018280,  // data 1,2
    -0.433013, 0.027350, 0.423164,  -0.407769, 0.003027, -0.913080, 0.793246,  // data 2,1
    -0.433013, 0.039586, -0.368557, -0.939679, 0.003654, -0.342039, 0.963774,  // data 2,2
    0.433013,  0.030137, -0.417380, 0.407769,  0.003027, -0.913080, 0.070429,  // data 3,1
    0.244857,  0.030742, -0.600000, -0.940760, 0.003027, -0.339058, 0.005974,  // data 4,1
    0.433013,  0.042086, -0.597393, 0.939679,  0.003654, 0.342039,  0.017618,  // data 4,2
    -0.433013, 0.032923, 0.057924,  -0.407769, 0.003027, 0.913080,  0.000810,  // data 5,1
    -0.433013, 0.044586, -0.363344, -0.939679, 0.003654, 0.342039,  0.000322,  // data 5,2
    0.125496,  0.034720, 0.600000,  0.940760,  0.003027, 0.339058,  0.000069,  // data 6,1
    0.433013,  0.047086, -0.129294, 0.939679,  0.003654, 0.342039,  0.000006,  // data 6,2
    0.433013,  0.035709, 0.301532,  0.407769,  0.003027, -0.913080, 0.000009,  // data 7,1
    -0.433013, 0.049587, 0.104756,  -0.939679, 0.003654, 0.342039,  0.000000,  // data 7,2
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 1e-5);
    }
  }
}

}  // namespace