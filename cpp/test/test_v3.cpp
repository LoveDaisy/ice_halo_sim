#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>

#include "context/context.hpp"
#include "core/crystal.hpp"
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
  auto data_queue = std::make_shared<v3::Queue<v3::SimDataPtrU>>();

  auto config = std::make_unique<v3::SimConfig>();
  config->sun_altitude_ = 20;
  config->sun_diameter_ = 0.5;
  config->wl_ = 550.0f;
  config->ray_num_ = 2;
  config->max_hits_ = 7;
  config->ms_num_ = 1;
  config->ms_prob_ = 1.0f;
  config->ms_crystal_[0] = v3::Crystal::CreatePrism(0.3f);
  config_queue->Emplace(std::move(config));

  v3::Simulator simulator(config_queue, data_queue);
  std::thread producer([&simulator]() { simulator.Run(); });
  std::thread consumer([=]() {
    while (true) {
      auto data = data_queue->Get();
      if (!data || data->Empty()) {
        break;
      }
      LOG_DEBUG("p  d  w");
      for (size_t i = 0; i < data->size_; i++) {
        LOG_DEBUG("%.6f,%.6f,%.6f  %.6f,%.6f,%.6f  %.6f", data->p()[i * 3 + 0], data->p()[i * 3 + 1],
                  data->p()[i * 3 + 2], data->d()[i * 3 + 0], data->d()[i * 3 + 1], data->d()[i * 3 + 2], data->w()[i]);
      }
    }
  });

  std::this_thread::sleep_for(500ms);
  simulator.Stop();

  producer.join();
  consumer.join();
}

}  // namespace