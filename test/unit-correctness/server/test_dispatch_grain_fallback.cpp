// The dispatch grain must follow the backend that is ACTUALLY running,
// not the one the user asked for.
//
// The defect this pins: `GenerateScene` sized every batch from `ResolveGpuRoute`'s
// verdict — a statement about the user's *preference* — while the Simulator on the
// other side of the queue could have dropped its TraceBackend mid-run (`BackendUnavailableError`) and be tracing on the
// legacy CPU path. That path samples ONE host wavelength per batch, so a batch sized for CUDA (262144 rays) paints the
// whole frame with a single wavelength; the measured effect is seconds of strongly
// saturated, wildly shifting colour at the start of a run (parent scrum §1: an
// isolated A/B on one machine, legacy backend, 128-ray grain stays neutral from
// t=0.51s while a 262144-ray grain reads (0.7344, 0.2656) — pure red — at t=3.43s
// and (0.0360, 0.7575) — pure green — at t=4.06s).
//
// Note what is NOT asserted here: any particular colour. Which wavelength the first
// batch draws is a function of the seed — three runs of the same repro drew blue,
// red, and red-then-green. The mechanical quantity underneath, and the one this file
// asserts, is the ray count a single wavelength sample gets spread over.
//
// The two halves below cover the judgement and its guard:
//   - EffectiveDispatchCap: the pure judgement, exhaustively, with no backend or
//     thread needed — including the case that must NOT shrink (a healthy GPU route,
//     AC4's byte-identical requirement).
//   - BackendFellBack through the C API: the CPU route must never report a fallback,
//     even though its `Simulator::BackendActive()` is legitimately false the moment
//     Run() starts (no backend was ever created). That asymmetry is what the
//     construction-time `gpu_route_` guard exists for, and a drift in it would turn
//     every ordinary CPU run into a spurious "GPU stopped working" modal.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include "core/def.hpp"
#include "core/simulator.hpp"
#include "lumice.h"
#include "server/server.hpp"
#include "util/queue.hpp"

namespace lumice {
namespace {

// The three measured throughput plateaus GenerateScene selects between. Spelled out
// here rather than included from server.cpp's private constants on purpose: this file
// is guarding that they keep their values through a fallback, so reading them from the
// same place the production code does would make the assertion circular.
constexpr size_t kCudaGrain = 262144;
constexpr size_t kMetalGrain = 32768;
constexpr size_t kLegacyGrain = 128;

TEST(EffectiveDispatchCap, HealthyGpuRouteKeepsItsGrain) {
  // AC4: this is the path every normal GPU run takes, and it must be untouched.
  EXPECT_EQ(EffectiveDispatchCap(true, true, kCudaGrain, kLegacyGrain), kCudaGrain);
  EXPECT_EQ(EffectiveDispatchCap(true, true, kMetalGrain, kLegacyGrain), kMetalGrain);
}

TEST(EffectiveDispatchCap, FallenBackGpuRouteShrinksToLegacyGrain) {
  // The defect itself: GPU-sized batches being fed to the single-wavelength CPU path.
  EXPECT_EQ(EffectiveDispatchCap(true, false, kCudaGrain, kLegacyGrain), kLegacyGrain);
  EXPECT_EQ(EffectiveDispatchCap(true, false, kMetalGrain, kLegacyGrain), kLegacyGrain);
}

TEST(EffectiveDispatchCap, CpuRouteIsNeverRewritten) {
  // A CPU-route server has no backend to lose, so `backend_active` is false there as a
  // matter of course and must not be read as a fallback. Asserted with a nominal cap
  // that differs from the fallback cap, so a "return fallback_cap unconditionally" bug
  // cannot pass by the two values happening to coincide.
  EXPECT_EQ(EffectiveDispatchCap(false, false, kLegacyGrain, kLegacyGrain), kLegacyGrain);
  EXPECT_EQ(EffectiveDispatchCap(false, false, 999, kLegacyGrain), 999u);
  EXPECT_EQ(EffectiveDispatchCap(false, true, 999, kLegacyGrain), 999u);
}

TEST(EffectiveDispatchCap, NeverRaisesAnExplicitlySmallerGrain) {
  // An explicit LUMICE_DISPATCH_RAY_NUM below the legacy default is a deliberate
  // request (it is how the crossover sweeps are run). A fallback must not undo it.
  EXPECT_EQ(EffectiveDispatchCap(true, false, 16, kLegacyGrain), 16u);
}

// The Simulator half of the signal. Defaulting to `true` is the deliberate choice:
// between a Simulator's construction and its worker thread reaching CreateBackend
// there is a window where nothing has been resolved yet, and a `false` default would
// make that window read as "already fell back" — shrinking the grain and popping the
// GUI modal on a run that is about to come up healthy on the GPU.
TEST(SimulatorBackendActive, DefaultsToActiveBeforeRunResolvesTheBackend) {
  auto scene_queue = std::make_shared<Queue<SimBatch>>();
  auto data_queue = std::make_shared<Queue<SimData>>();
  Simulator sim(scene_queue, data_queue, /*seed=*/1234);
  EXPECT_TRUE(sim.BackendActive());
}

// Small and quick — this file asserts on a flag, not on physics.
const char* kConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 2000,
    "max_hits": 6,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }]
})";

LUMICE_ErrorCode CommitJson(LUMICE_Server* server, const std::string& json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

int ReadFallbackFlag(LUMICE_Server* server) {
  int fell_back = -1;
  EXPECT_EQ(LUMICE_GetBackendFallbackFlag(server, &fell_back), LUMICE_OK);
  return fell_back;
}

TEST(BackendFallbackFlag, NullArgumentsAreRejected) {
  int fell_back = 0;
  EXPECT_EQ(LUMICE_GetBackendFallbackFlag(nullptr, &fell_back), LUMICE_ERR_NULL_ARG);

  LUMICE_ServerConfig cfg{};
  cfg.num_workers = 1;
  LUMICE_Server* server = LUMICE_CreateServerEx(&cfg);
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(LUMICE_GetBackendFallbackFlag(server, nullptr), LUMICE_ERR_NULL_ARG);
  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

// The guard that matters in practice. A default (CPU-route) server's Simulator has
// `BackendActive() == false` for the whole of Run() — CreateBackend legitimately
// returned nullptr — so a fallback signal derived from that alone would fire on every
// ordinary CPU run. Sampled DURING a live run and again after it settles, because the
// flag is polled continuously by the GUI rather than read once.
TEST(BackendFallbackFlag, CpuRouteNeverReportsAFallback) {
  LUMICE_ServerConfig cfg{};
  cfg.num_workers = 1;
  LUMICE_Server* server = LUMICE_CreateServerEx(&cfg);
  ASSERT_NE(server, nullptr);

  EXPECT_EQ(ReadFallbackFlag(server), 0) << "reported a fallback before anything ran";
  ASSERT_EQ(CommitJson(server, kConfig), LUMICE_OK);

  // Poll across the run the way the GUI does. A single post-hoc read could miss a flag
  // that is only true while batches are in flight.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  bool drained = false;
  while (std::chrono::steady_clock::now() < deadline) {
    EXPECT_EQ(ReadFallbackFlag(server), 0) << "CPU route reported a GPU fallback mid-run";
    LUMICE_DrainResult drain{};
    ASSERT_EQ(LUMICE_GetDrainStatus(server, &drain), LUMICE_OK);
    if (drain.drained_epoch == drain.current_epoch) {
      drained = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(drained) << "run never drained; the polling above proved less than intended";
  EXPECT_EQ(ReadFallbackFlag(server), 0) << "CPU route reported a GPU fallback after the run";

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

}  // namespace
}  // namespace lumice
