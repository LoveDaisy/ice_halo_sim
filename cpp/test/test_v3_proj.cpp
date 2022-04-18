#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstring>
#include <memory>
#include <thread>

#include "process/simulation.hpp"
#include "util/log.hpp"
#include "util/queue.hpp"

extern std::string config_file_name;
using namespace icehalo;
using namespace std::chrono_literals;

namespace {

class V3TestProj : public ::testing::Test {
 protected:
  void SetUp() override {}
};


TEST_F(V3TestProj, SimpleProj) {
  auto config_queue = std::make_shared<v3::Queue<v3::SimConfigPtrU>>();
  auto data_queue = std::make_shared<v3::Queue<v3::SimBasicDataPtrU>>();

  constexpr int kMaxHits = 7;

  auto config = std::make_unique<v3::SimConfig>();
  config->ray_num_ = 2;
  config->max_hits_ = kMaxHits;
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
    0.433013,  0.047087, -0.129294, 0.939679,  0.003654, 0.342039,  0.000006,  // data 6,2
    0.433013,  0.035710, 0.301532,  0.407769,  0.003027, -0.913080, 0.000009,  // data 7,1
    -0.433013, 0.049587, 0.104756,  -0.939679, 0.003654, 0.342039,  0.000000,  // data 7,2
  };

  for (int i = 0; i < kMaxHits * 2; i++) {
    for (int j = 0; j < 7; j++) {
      EXPECT_NEAR(expect_out[i * 7 + j], output_data[i * 7 + j], 1e-5);
    }
  }
}

}  // namespace