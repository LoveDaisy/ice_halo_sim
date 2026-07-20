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
      if (r.to_face_ != lumice::kInvalidId || r.w_ < 0) {
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

  // Fixture regenerated after Crystal::CreatePrism switched from the numerical
  // triangle-mesh pipeline (FillHexCrystalCoef + CreateConvexPolyhedronMesh +
  // math.cpp::Triangulate) to the closed-form path (ComputeClosedFormPrism +
  // fixed-fan triangulation in crystal.cpp::AdaptClosedFormPrismToCrystalGeom
  // / BuildMeshFromCfGeom). Captured with seed=42 from
  // examples/config_example.json crystal id=1 (default axis = canonical pose).
  //
  // Why the y/z values drift while d/w/p[0] stay bit-identical: sample_triangle
  // in simulator.cpp draws area-weighted from the crystal's triangle list. The
  // closed-form path produces a different specific triangulation of the same
  // per-face polygon, so categorical_sample under the same uniform sequence
  // lands on a different triangle / point on the same face. Physics-invariant
  // fields — w (Fresnel weight along the path history), d (exit direction from
  // the same face with the same incident direction), and p[0] (±0.433013 =
  // ±√3/4, the x of the ±x prism side face) — reproduce bit-for-bit vs the
  // pre-swap capture; only p[1] / p[2] (position within the same face) shifted.
  // That is the exact signature of "same distribution, different draw" — the
  // ray-count-1 + distribution-unchanged invariant Crystal::CreatePrism now
  // guarantees is the one this suite actually depends on.
  float expect_out[kMaxHits * 2 * 7]{
    /* --------- p --------------->|<-------------- d ------------->|<-- w -->|*/
    0.433013f,  -0.139846f, 0.145981f,  0.938556f,  -0.000995f, -0.345125f, 0.018287f,  //
    0.433013f,  -0.171991f, 0.122222f,  0.939997f,  -0.000397f, -0.341181f, 0.018278f,  //
    -0.433013f, -0.140527f, -0.090337f, -0.938556f, -0.000995f, -0.345125f, 0.963760f,  //
    -0.433013f, -0.172262f, -0.111198f, -0.939997f, -0.000397f, -0.341181f, 0.963778f,  //
    0.433013f,  -0.141209f, -0.326654f, 0.938556f,  -0.000996f, -0.345125f, 0.017624f,  //
    0.433013f,  -0.172534f, -0.344618f, 0.939997f,  -0.000397f, -0.341181f, 0.017616f,  //
    -0.433013f, -0.141891f, -0.562971f, -0.938556f, -0.000996f, -0.345125f, 0.000322f,  //
    -0.433013f, -0.172805f, -0.578037f, -0.939997f, -0.000397f, -0.341181f, 0.000322f,  //
    0.433013f,  -0.142573f, -0.400712f, 0.938556f,  -0.000996f, 0.345125f,  0.000006f,  //
    0.433013f,  -0.173077f, -0.388543f, 0.939997f,  -0.000397f, 0.341181f,  0.000006f,  //
    -0.433013f, -0.143254f, -0.164395f, -0.938556f, -0.000996f, 0.345125f,  0.000000f,  //
    -0.433013f, -0.173349f, -0.155124f, -0.939997f, -0.000397f, 0.341181f,  0.000000f,  //
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