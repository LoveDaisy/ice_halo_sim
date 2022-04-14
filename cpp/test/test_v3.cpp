#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstring>
#include <memory>
#include <thread>

#include "context/context.hpp"
#include "context/crystal_config.hpp"
#include "context/light_config.hpp"
#include "core/crystal.hpp"
#include "core/math.hpp"
#include "process/simulation.hpp"
#include "util/log.hpp"
#include "util/queue.hpp"

extern std::string config_file_name;
using namespace icehalo;
using namespace std::chrono_literals;

namespace {

class V3Test : public ::testing::Test {
 protected:
  void SetUp() override {
    context_ = ProjectContext::CreateFromFile(config_file_name.c_str());  // read from file
  }

  ProjectContextPtr context_;
};


TEST_F(V3Test, Simple) {
  auto config_queue = std::make_shared<v3::Queue<v3::SimConfigPtrU>>();
  auto data_queue = std::make_shared<v3::Queue<v3::SimBasicDataPtrU>>();

  auto config = std::make_unique<v3::SimConfig>();
  config->ray_num_ = 2;
  config->max_hits_ = 7;
  auto& light_source = config->light_source_;
  light_source.param_ = v3::SunParam{ 20.0f, 0.5f };
  light_source.wl_param_.emplace_back(v3::WlParam{ 550.0f, 1.0f });
  config->ms_param_.emplace_back(v3::SimConfig::SimMsParam{});
  auto& ms_param = config->ms_param_[0];
  ms_param.ms_prob_ = 0.0f;
  ms_param.crystal_info_.emplace_back(v3::SimConfig::SimCrystalInfo{});
  auto& crystal_info = ms_param.crystal_info_[0];
  crystal_info.proportion_ = 10.0f;
  crystal_info.crystal_.id_ = 1;
  crystal_info.crystal_.param_ =
      v3::PrismCrystalParam{ Distribution{ DistributionType::kNoRandom, 1.2f, 0.0f },
                             { Distribution{ DistributionType::kNoRandom, math::kSqrt3_4, 0.0f },
                               Distribution{ DistributionType::kNoRandom, math::kSqrt3_4, 0.0f },
                               Distribution{ DistributionType::kNoRandom, math::kSqrt3_4, 0.0f },
                               Distribution{ DistributionType::kNoRandom, math::kSqrt3_4, 0.0f },
                               Distribution{ DistributionType::kNoRandom, math::kSqrt3_4, 0.0f },
                               Distribution{ DistributionType::kNoRandom, math::kSqrt3_4, 0.0f } } };
  crystal_info.crystal_.axis_.azimuth_dist = { DistributionType::kUniform, 0.0f, 0.0f };
  crystal_info.crystal_.axis_.latitude_dist = { DistributionType::kNoRandom, 90.0f, 0.0f };
  crystal_info.crystal_.axis_.roll_dist = { DistributionType::kUniform, 0.0f, 0.0f };
  config_queue->Emplace(std::move(config));

  v3::Simulator simulator(config_queue, data_queue);
  std::unique_ptr<float[]> output_data{ new float[12 * 7]{} };
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

  producer.join();
  consumer.join();

  float expect_out[11 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.302099,  0.040812,  0.150000,  -0.939667, 0.003329,  0.342074,  0.127290,  // data 1,1
    0.165777,  -0.404289, -0.079173, -0.467876, -0.814561, -0.342904, 0.063834,  // data 1,2
    -0.006257, 0.041904,  -0.150000, -0.939667, 0.003329,  -0.342074, 0.761623,  // data 2,1
    -0.314613, 0.042997,  0.150000,  -0.939667, 0.003329,  0.342074,  0.096947,  // data 3,1
    -0.433013, -0.185722, -0.048089, -0.833166, 0.433879,  0.342904,  0.917659,  // data 3,2
    -0.433013, 0.043416,  0.034809,  -0.405241, 0.003329,  -0.914204, 0.012868,  // data 4,1
    -0.243057, 0.044089,  -0.150000, 0.939667,  0.003329,  -0.342074, 0.001111,  // data 5,1
    0.433013,  0.130390,  0.098259,  0.833166,  0.433879,  -0.342904, 0.018141,  // data 5,2
    0.065299,  0.045181,  0.150000,  0.939667,  0.003329,  0.342074,  0.000141,  // data 6,1
    -0.224493, 0.370389,  -0.091418, -0.939368, 0.002088,  -0.342904, 0.000343,  // data 6,2
    0.373655,  0.046274,  -0.150000, 0.939667,  0.003329,  -0.342074, 0.000018,  // data 7,1
  };

  for (int i = 0; i < 11; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 1e-5);
    }
  }
}

}  // namespace