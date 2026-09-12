// The ray-allocation pilot as CommitConfig runs it: when it runs, when it is skipped,
// and what the skip carries forward.
//
// The pass itself (tally, Neyman, floor, doubling) is pinned in test_simulator.cpp on
// Simulator::RunRayAllocationPilot directly. What this file owns is the half only the
// live server can show:
//
//   - proportional (the default) never runs it: the path is the pre-pilot one, and a
//     stray pilot there would be a synchronous trace on every commit for every user.
//   - adaptive runs it on the first commit, and again ONLY when a field the tally can
//     depend on changed. CommitConfig is a high-frequency path (the GUI recommits on
//     a 70ms cadence while a slider drags), so a commit that changed nothing the pilot
//     reads — a view, a lens, the ray budget — must carry the previous weights forward
//     rather than trace again. The debounce compares the two scenes' masked copies
//     through the tree's one SceneConfig equality, so the assertion is on the OUTCOME
//     (pilot ran / weights reused), read off the log lines the two branches emit.
//   - a scene whose layer shape changed (a scattering layer added) is a changed input.
//     A carried-forward weight set has no entry for the new layer, and the fallback
//     that would then apply is silent (ResolveLayerRayAllocation's per-layer
//     proportional fallback), so it has to be a re-run, not a reuse.
//
// The counting is by log line, not by elapsed time: the pilot's cost is well under a
// second here, and "the second commit was faster" is exactly the kind of assertion a
// loaded CI runner turns into a flake.

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "server/server.hpp"
#include "util/logger.hpp"

namespace lumice {
namespace {

class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(oss_)) { GetSharedSink()->add_sink(sink_); }
  ~LogCapture() { GetSharedSink()->remove_sink(sink_); }
  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return oss_.str(); }
  void Clear() {
    oss_.str("");
    oss_.clear();
  }

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
};

// The two branches of CommitConfig's adaptive block, by the line each emits.
constexpr const char* kPilotRan = "RayAllocationPilot: ran ";
constexpr const char* kWeightsReused = "reusing the previous weights";

size_t Count(const std::string& haystack, const char* needle) {
  size_t n = 0;
  for (size_t pos = haystack.find(needle); pos != std::string::npos; pos = haystack.find(needle, pos + 1)) {
    n++;
  }
  return n;
}

// Two prism entries, one of them filtered to a single raypath, so the pilot has a real
// energy skew to measure and the scene is cheap: the pilot's shipped budget for K = 2
// is 200k rays, well under a second on the legacy path.
nlohmann::json MakeConfig(const char* ray_allocation) {
  nlohmann::json root;
  root["crystal"] = nlohmann::json::array({
      { { "id", 1 }, { "type", "prism" }, { "shape", { { "height", 1.0f } } } },
      { { "id", 2 }, { "type", "prism" }, { "shape", { { "height", 1.0f } } } },
  });
  root["filter"] = nlohmann::json::array({ { { "id", 1 },
                                             { "type", "raypath" },
                                             { "raypath", { 3, 5, 7, 3 } },
                                             { "symmetry", "PBD" },
                                             { "action", "filter_in" } } });
  nlohmann::json scene;
  scene["light_source"] = {
    { "type", "sun" }, { "altitude", 20.0f }, { "azimuth", 0.0f }, { "diameter", 0.5f }, { "spectrum", "D65" }
  };
  scene["ray_num"] = 1000;
  scene["max_hits"] = 4;
  if (ray_allocation != nullptr) {
    scene["ray_allocation"] = ray_allocation;
  }
  scene["scattering"] =
      nlohmann::json::array({ { { "prob", 0.0f },
                                { "entries", nlohmann::json::array({
                                                 { { "crystal", 1 }, { "proportion", 1.0f } },
                                                 { { "crystal", 2 }, { "proportion", 1.0f }, { "filter", 1 } },
                                             }) } } });
  root["scene"] = scene;

  nlohmann::json rn;
  rn["id"] = 1;
  rn["lens"] = { { "type", "fisheye_equal_area" }, { "fov", 180.0f } };
  rn["resolution"] = { 32, 32 };
  rn["view"] = { { "elevation", 0.0f }, { "azimuth", 0.0f }, { "roll", 0.0f } };
  rn["visible"] = "full";
  rn["intensity_factor"] = 1.0f;
  root["render"] = nlohmann::json::array({ rn });
  return root;
}

}  // namespace

TEST(RayAllocationPilotCommit, ProportionalNeverRunsThePilot) {
  LogCapture capture;
  Server server(1);
  ASSERT_TRUE(server.CommitConfig(MakeConfig(nullptr)).IsSuccess());
  ASSERT_TRUE(server.CommitConfig(MakeConfig("proportional")).IsSuccess());
  server.Terminate();
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 0u);
  EXPECT_EQ(Count(capture.Text(), kWeightsReused), 0u);
}

TEST(RayAllocationPilotCommit, AdaptiveRunsOnceThenReusesUntilAPilotInputChanges) {
  LogCapture capture;
  Server server(1);

  // First adaptive commit: no previous weights exist, the pilot must run.
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 1u);
  EXPECT_EQ(Count(capture.Text(), kWeightsReused), 0u);

  // The GUI's slider cadence, on fields the pilot is indifferent to: a new view and
  // a new ray budget. Reused, not re-run — twice, to pin that reuse chains (the
  // second reuse carries forward what the first carried forward).
  capture.Clear();
  auto view_change = MakeConfig("adaptive");
  view_change["render"][0]["view"]["elevation"] = 30.0f;
  view_change["scene"]["ray_num"] = 5000;
  ASSERT_TRUE(server.CommitConfig(view_change).IsSuccess());
  view_change["render"][0]["view"]["azimuth"] = 45.0f;
  ASSERT_TRUE(server.CommitConfig(view_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 0u);
  EXPECT_EQ(Count(capture.Text(), kWeightsReused), 2u);

  // A proportion moved: the tally's denominators change, the pilot runs again.
  capture.Clear();
  auto proportion_change = MakeConfig("adaptive");
  proportion_change["scene"]["scattering"][0]["entries"][1]["proportion"] = 0.2f;
  ASSERT_TRUE(server.CommitConfig(proportion_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 1u);
  EXPECT_EQ(Count(capture.Text(), kWeightsReused), 0u);

  // The sun moved: every landed energy changes, the pilot runs again.
  capture.Clear();
  auto sun_change = proportion_change;
  sun_change["scene"]["light_source"]["altitude"] = 40.0f;
  ASSERT_TRUE(server.CommitConfig(sun_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 1u);

  // A scattering layer added: a changed shape, so a changed input — the carried
  // weights would have nothing for the new layer, and the layer would silently
  // fall back to proportional. Re-run, and every layer delivered.
  capture.Clear();
  auto layer_change = sun_change;
  layer_change["scene"]["scattering"][0]["prob"] = 0.5f;
  layer_change["scene"]["scattering"].push_back(layer_change["scene"]["scattering"][0]);
  layer_change["scene"]["scattering"][1]["prob"] = 0.0f;
  ASSERT_TRUE(server.CommitConfig(layer_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 1u);
  EXPECT_EQ(Count(capture.Text(), "RayAllocationPilot: layer 1 entry 1"), 1u);

  server.Terminate();
}

TEST(RayAllocationPilotCommit, SwitchingModesRerunsBecauseAProportionalSceneHasNoWeightsToCarry) {
  // proportional → adaptive on an otherwise identical scene: the previous scene's
  // weights are all at the -1 sentinel, so there is nothing to carry and the pilot
  // runs. adaptive → proportional → adaptive likewise: the proportional commit in
  // between left no weights behind. Neither transition may read as "unchanged".
  LogCapture capture;
  Server server(1);
  ASSERT_TRUE(server.CommitConfig(MakeConfig("proportional")).IsSuccess());
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 1u);
  capture.Clear();
  ASSERT_TRUE(server.CommitConfig(MakeConfig("proportional")).IsSuccess());
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kPilotRan), 1u);
  EXPECT_EQ(Count(capture.Text(), kWeightsReused), 0u);
  server.Terminate();
}

}  // namespace lumice
