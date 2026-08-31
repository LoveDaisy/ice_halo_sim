#ifndef LUMICE_TEST_SUPPORT_LIVE_SERVER_HPP_
#define LUMICE_TEST_SUPPORT_LIVE_SERVER_HPP_

// Shared live-server harness for the windowless GUI test targets.
//
// Several GUI test files drive a REAL LUMICE_Server — the poller's contract is about what happens
// between a running simulation and the screen, and a mock server would be a second implementation
// of the thing under test rather than an oracle for it. That means every such case pays the same
// prologue (create, commit, wait for produced data) and the same epilogue (stop, destroy), and
// carries its own copy of a scene config whose exact shape is load-bearing in ways easy to get
// wrong: `scattering.prob` 1.0 vs 0.0 decides whether a `layer: 0` colour class matches anything
// at all, and a `"type": "Prism"` typo parses to nothing rather than failing loudly.
//
// Those configs and that prologue were duplicated across files, which is how two copies of the
// "same" colour scene drifted apart. They live here now, in test/support/ next to the other
// cross-target fixtures (scoped_result_frame.hpp, scene_json_helpers.hpp) rather than beside any
// one caller, because no single caller owns them.
//
// C API only, deliberately: nothing here includes a gui/ header, so the header stays usable from
// any target that links lumice_obj and cannot pull a GUI dependency into one that has none.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>

#include "lumice.h"
#include "support/scoped_result_frame.hpp"

namespace lumice::test {

// A finite single-prism run: empty filter (all rays pass), rectangular lens, small resolution —
// completes in well under a second on CPU or Metal.
//
// The crystal block is in the canonical wire form (lowercase "prism", shape nested under "shape").
// It once read `"type": "Prism", "height": 1.0, "ratio": {...}`, which the core parser did NOT
// understand: an unrecognized type logged "Unknown crystal type!" and left the default-constructed
// param, so "height"/"ratio" were never read and the test silently ran a zero-face-distance
// crystal. The Scene parser now rejects the unknown type outright (LUMICE_ERR_INVALID_VALUE).
inline constexpr const char* kFiniteSceneJson = R"({
  "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.0}}],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0, "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{"id": 1, "lens": {"type": "rectangular", "fov": 180.0},
              "resolution": [256, 128], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
              "visible": "full", "background": [0, 0, 0], "intensity_factor": 1.0}]
})";

// Never-ending variant of the above: production never stops, so the run never completes and never
// drains. The end-to-end negative for anything that keys on a terminal edge.
inline constexpr const char* kInfiniteSceneJson = R"({
  "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.0}}],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0, "diameter": 0.5, "spectrum": "D65"},
    "ray_num": "infinite",
    "max_hits": 8,
    "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{"id": 1, "lens": {"type": "rectangular", "fov": 180.0},
              "resolution": [256, 128], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
              "visible": "full", "background": [0, 0, 0], "intensity_factor": 1.0}]
})";

// Monochrome oriented-prism scene. `scattering.prob` is 0.0 here and 1.0 above, and that difference
// is not cosmetic: a raypath_color class matching `layer: 0` needs rays that TERMINATE at layer 0,
// so the finite config above yields an empty ColoredMask and a NON-composite payload. Any case
// about the composite path must build on this scene, not that one.
inline constexpr const char* kMonoSceneJson = R"({
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
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }]
})";

// kMonoSceneJson plus a match-all red class, so the RenderConsumer's ColoredMask() is non-zero and
// DoSnapshot's second phase produces a composite for the generation.
inline constexpr const char* kColorSceneJson = R"({
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
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": "dominant",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  }
})";

// Two classes for the participating-exposure cases: class 0 is match-all (bright — every landed ray
// contributes to its Y lane), class 1 is an entry_exit filter with len == 3 (dim — only 3-hop paths
// contribute). The larger ray budget is what keeps class 1's lane statistically populated.
inline constexpr const char* kTwoColorSceneJson = R"({
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
    "ray_num": 400000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": "dominant",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]},
      {"color": [0.0, 0.0, 1.0], "match": [{"layer": 0, "crystal": 1,
                                            "type": "entry_exit",
                                            "min_len": 3, "max_len": 3}]}
    ]
  }
})";

// A run whose totals are exactly checkable: scattering prob 0.0 keeps orientation_num and ray_num
// strictly 1:1, and GenerateScene issues 128-ray dispatch grains, so any grain the consumer has not
// taken shows up as a recognizable shortfall rather than as noise.
inline constexpr const char* kExactTotalsSceneJson = R"({
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
    "ray_num": 20000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1, "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32], "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0], "intensity_factor": 1.0
  }]
})";
inline constexpr LUMICE_RayCount kExactTotalsSimRays = 20000;

// JSON -> Scene handle -> commit: the only commit path since v4.12 removed the JSON-string entry
// points. The handle is a local — LUMICE_CommitScene reads it as const and keeps no reference — so
// it is destroyed as soon as the commit returns.
inline LUMICE_ErrorCode CommitSceneJson(LUMICE_Server* server, const char* json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json, &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

// Spin until `pred` holds or the budget runs out; returns whether it held. Real time, deliberately:
// the signals being waited on are driven by worker threads and steady_clock, so nothing in the test
// process can advance them.
template <typename Pred>
bool WaitFor(Pred pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

// Wait for the PRODUCER-side verdict: the server has settled to IDLE and the published frame
// carries real data (has_valid_data is the RawXyzResult contract field).
inline bool WaitForValidData(LUMICE_Server* server, int timeout_ms = 5000) {
  return WaitFor(
      [server] {
        LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
        LUMICE_QueryServerState(server, &st);
        if (st != LUMICE_SERVER_IDLE) {
          return false;
        }
        LUMICE_RawXyzResult xyz[2]{};
        ScopedResultFrame frame(server);
        LUMICE_FrameGetRawXyz(frame.get(), xyz, 1);
        return xyz[0].has_valid_data != 0;
      },
      timeout_ms);
}

// Wait for the CONSUMER-side verdict: the current epoch is fully drained. Distinct from
// WaitForValidData above, and the distinction is the point — those two moments are not the same
// moment, which is the whole subject of the self-pause guard.
inline bool WaitForDrained(LUMICE_Server* server, int timeout_ms) {
  return WaitFor(
      [server] {
        LUMICE_DrainResult d{};
        LUMICE_GetDrainStatus(server, &d);
        return d.current_epoch != 0 && d.drained_epoch == d.current_epoch;
      },
      timeout_ms);
}

// The server's current committed epoch, read off the lifecycle clock.
inline unsigned long long CurrentEpoch(LUMICE_Server* server) {
  LUMICE_SimLifecycleResult lc{};
  LUMICE_GetSimLifecycle(server, &lc);
  return lc.epoch;
}

// Content fingerprint of a frame's pixels. Cases about a payload carrying the PREVIOUS run's image
// have to compare the image itself — asserting only on the epoch stamp would pass for the wrong
// reason whenever the timing happened not to line up. A plain FNV-1a over the raw float bytes: no
// tolerance, no interpretation, just "same bytes?".
inline uint64_t PixelFingerprint(const float* xyz, size_t float_count) {
  uint64_t h = 1469598103934665603ULL;
  const auto* bytes = reinterpret_cast<const unsigned char*>(xyz);
  const size_t n = float_count * sizeof(float);
  for (size_t i = 0; i < n; ++i) {
    h = (h ^ bytes[i]) * 1099511628211ULL;
  }
  return h;
}

// A server that destroys itself. `Run()` is the common prologue — commit a scene and block until it
// has produced data — and returns false on either failure so a case can bail without leaking.
class LiveServer {
 public:
  LiveServer() : server_(LUMICE_CreateServer()) {}
  ~LiveServer() {
    if (server_ != nullptr) {
      LUMICE_DestroyServer(server_);
    }
  }
  LiveServer(const LiveServer&) = delete;
  LiveServer& operator=(const LiveServer&) = delete;

  LUMICE_Server* get() const { return server_; }
  operator LUMICE_Server*() const { return server_; }  // NOLINT(google-explicit-constructor)

  bool Run(const char* json = kFiniteSceneJson, int timeout_ms = 5000) {
    return server_ != nullptr && CommitSceneJson(server_, json) == LUMICE_OK && WaitForValidData(server_, timeout_ms);
  }

 private:
  LUMICE_Server* server_ = nullptr;
};

}  // namespace lumice::test

#endif  // LUMICE_TEST_SUPPORT_LIVE_SERVER_HPP_
