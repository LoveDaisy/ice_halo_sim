#include <benchmark/benchmark.h>

#include <memory>
#include <string>

#include "context/context.hpp"
#include "core/crystal.hpp"
#include "core/optics.hpp"

std::string config_file;
std::string working_dir;

// ========== Define the test fixture ==========
class TestOptics : public ::benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) override {
    crystal_ = icehalo::Crystal::CreateHexPrism(1.2f);
    context_ = icehalo::ProjectContext::CreateFromFile(config_file.c_str());
  }

 protected:
  icehalo::CrystalPtrU crystal_;
  icehalo::ProjectContextPtr context_;
};

// ========== The original HitSurface ==========
BENCHMARK_DEFINE_F(TestOptics, TestHitSurface)(::benchmark::State& st) {
  int ray_num = st.range(0);
  std::unique_ptr<float[]> dir_in{ new float[ray_num * 3]{} };
  std::unique_ptr<float[]> w_in{ new float[ray_num]{} };
  std::unique_ptr<int[]> face_id_in{ new int[ray_num]{} };

  auto* rng = icehalo::RandomNumberGenerator::GetInstance();
  for (int i = 0; i < ray_num; i++) {
    icehalo::Vec3f d{ rng->GetGaussian(), rng->GetGaussian(), rng->GetGaussian() };
    d.Normalize();
    std::memcpy(dir_in.get() + i * 3, d.val(), 3 * sizeof(float));
    w_in[i] = rng->GetUniform();
    face_id_in[i] = 0;
  }

  std::unique_ptr<float[]> dir_out{ new float[ray_num * 6] };
  std::unique_ptr<float[]> w_out{ new float[ray_num * 2] };

  for (auto _ : st) {
    icehalo::Optics::HitSurface(crystal_.get(), 1.31f, ray_num,     // input
                                dir_in.get(), face_id_in.get(), w_in.get(),  // input
                                dir_out.get(), w_out.get());                 // output
  }
}
BENCHMARK_REGISTER_F(TestOptics, TestHitSurface)->Arg(10)->Arg(100)->Arg(1000);


// ========== The new HitSurface ==========

// ========== main ==========
int main(int argc, char** argv) {
  ::benchmark::Initialize(&argc, argv);

  if (argc == 3) {
    config_file = std::string(argv[1]);
    working_dir = std::string(argv[2]);
  } else {
    config_file = "";
    working_dir = "";
  }

  ::benchmark::RunSpecifiedBenchmarks();
  ::benchmark::Shutdown();
  return 0;
}