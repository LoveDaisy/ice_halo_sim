#include <benchmark/benchmark.h>

#include <chrono>
#include <string>
#include <thread>

#include "server/server.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace) benchmark code

namespace {

// Minimal config: single wavelength, single prism crystal, no multi-scattering.
// ray_num is patched at runtime via string replacement.
const std::string kBenchConfigTemplate = R"({
  "light_source": [{
    "id": 1,
    "type": "sun",
    "altitude": 20.0,
    "azimuth": 0,
    "diameter": 0.5,
    "wavelength": [550],
    "wl_weight": [1.0]
  }],
  "crystal": [{
    "id": 1,
    "type": "prism",
    "shape": { "height": 1.3 }
  }],
  "filter": [{
    "id": 1,
    "type": "none"
  }],
  "scene": [{
    "id": 1,
    "light_source": 1,
    "ray_num": __RAY_NUM__,
    "max_hits": 7,
    "scattering": [{
      "crystal": [1],
      "proportion": [1],
      "prob": 0.0
    }]
  }],
  "render": [{
    "id": 1,
    "lens": { "type": "linear", "fov": 40 },
    "resolution": [320, 240]
  }],
  "project": {
    "id": 1,
    "scene": 1,
    "render": [1]
  }
})";

std::string MakeConfig(int64_t ray_num) {
  std::string config = kBenchConfigTemplate;
  auto pos = config.find("__RAY_NUM__");
  config.replace(pos, 11, std::to_string(ray_num));
  return config;
}

}  // namespace


static void BM_Scene(benchmark::State& state) {
  auto ray_num = state.range(0);
  std::string config = MakeConfig(ray_num);

  for (auto _ : state) {
    Server server;
    auto err = server.CommitConfig(config);
    if (err) {
      state.SkipWithError("CommitConfig failed");
      return;
    }
    while (!server.IsIdle()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    server.Terminate();
  }
  state.SetItemsProcessed(state.iterations() * ray_num);
}
BENCHMARK(BM_Scene)->Arg(1000)->Arg(10000)->Arg(100000)->Arg(1000000)->Unit(benchmark::kMillisecond);
