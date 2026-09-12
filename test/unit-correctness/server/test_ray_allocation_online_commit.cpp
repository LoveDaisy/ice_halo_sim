// The online ray allocation as CommitConfig binds it: when a commit starts the
// statistic over, when it carries the accumulated tally forward, and — the half that
// replaced the pilot — that a commit traces nothing at all.
//
// The loop itself (tally, Neyman, floor, cold start, per-batch charge) is pinned in
// test_simulator.cpp on RayAllocationOnline and Simulator::Run directly; the backends'
// tallies in test_ray_allocation_backends.cpp. What this file owns is the half only
// the live server can show:
//
//   - proportional (the default) binds no RayAllocationOnline: the path is the
//     pre-allocation one, and no tally line of either kind is ever written.
//   - adaptive starts cold on the first commit, and starts cold again ONLY when a
//     field the tally can depend on changed. CommitConfig is a high-frequency path
//     (the GUI recommits on a 70ms cadence while a slider drags), so a commit that
//     changed nothing the statistic reads — a view, a lens, the ray budget — must
//     keep the tally it has rather than throw away what the run measured. The
//     debounce compares the two scenes' masked copies through the tree's one
//     SceneConfig equality, so the assertion is on the OUTCOME (cold start / kept),
//     read off the log lines the two branches emit.
//   - a scene whose layer shape changed (a scattering layer added) is a changed
//     input: the previous object has no row for the new layer, so it has to be a
//     cold start, not a carry.
//   - CommitConfig traces no ray. The pilot it replaced traced a few hundred
//     thousand of them synchronously on the calling thread, and did so through the
//     thread-local RandomNumberGenerator (it reseeded it, then drew from it). So the
//     probe is that generator: seed it, draw a reference sequence, seed it again,
//     commit, draw again — a commit that traced on this thread cannot leave the two
//     sequences equal. Under the pilot this read red; it is the AC5 red-state probe.
//   - the run publishes the q it converged on: milestone lines while it runs and a
//     final line at Stop, in the one shape the e2e mechanism test parses.
//
// The counting is by log line, not by elapsed time: "the second commit was faster" is
// exactly the kind of assertion a loaded CI runner turns into a flake.

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "core/math.hpp"
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
constexpr const char* kColdStart = "ray-allocation online tally started";
constexpr const char* kTallyKept = "keeping the online tally";
// The workers' milestone lines and the server's final line (LogRayAllocationState).
constexpr const char* kMilestone = "RayAllocationOnline: layer ";
constexpr const char* kFinal = "RayAllocationOnline(final): layer ";

size_t Count(const std::string& haystack, const char* needle) {
  size_t n = 0;
  for (size_t pos = haystack.find(needle); pos != std::string::npos; pos = haystack.find(needle, pos + 1)) {
    n++;
  }
  return n;
}

// Two prism entries, one of them filtered to a single raypath, so the statistic has
// a real energy skew to measure and the scene is cheap on the legacy path.
nlohmann::json MakeConfig(const char* ray_allocation, size_t ray_num = 1000) {
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
  scene["ray_num"] = ray_num;
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

// The calling thread's RandomNumberGenerator, as a sequence.
std::vector<float> DrawSequence(uint32_t seed, size_t n) {
  auto& rng = RandomNumberGenerator::GetInstance();
  rng.SetSeed(seed);
  std::vector<float> out;
  out.reserve(n);
  for (size_t i = 0; i < n; i++) {
    out.push_back(rng.GetUniform());
  }
  return out;
}

bool WaitUntilIdle(const Server& server, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (server.GetSimLifecycle() != SimLifecycle::kRunning) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

}  // namespace

TEST(RayAllocationOnlineCommit, ProportionalBindsNoTally) {
  LogCapture capture;
  Server server(1);
  ASSERT_TRUE(server.CommitConfig(MakeConfig(nullptr)).IsSuccess());
  ASSERT_TRUE(server.CommitConfig(MakeConfig("proportional")).IsSuccess());
  ASSERT_TRUE(WaitUntilIdle(server, std::chrono::seconds(30)));
  server.Terminate();
  EXPECT_EQ(Count(capture.Text(), kColdStart), 0u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);
  EXPECT_EQ(Count(capture.Text(), kMilestone), 0u);
  EXPECT_EQ(Count(capture.Text(), kFinal), 0u);
}

TEST(RayAllocationOnlineCommit, AdaptiveStartsColdThenKeepsTheTallyUntilAnInputChanges) {
  LogCapture capture;
  Server server(1);

  // First adaptive commit: nothing to carry, the statistic starts cold.
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 1u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);

  // The GUI's slider cadence, on fields the statistic is indifferent to: a new
  // view and a new ray budget. Kept, not restarted — twice, to pin that the carry
  // chains (the second commit keeps what the first kept).
  capture.Clear();
  auto view_change = MakeConfig("adaptive");
  view_change["render"][0]["view"]["elevation"] = 30.0f;
  view_change["scene"]["ray_num"] = 5000;
  ASSERT_TRUE(server.CommitConfig(view_change).IsSuccess());
  view_change["render"][0]["view"]["azimuth"] = 45.0f;
  ASSERT_TRUE(server.CommitConfig(view_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 0u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 2u);

  // A proportion moved: the statistic's shares change, cold start.
  capture.Clear();
  auto proportion_change = MakeConfig("adaptive");
  proportion_change["scene"]["scattering"][0]["entries"][1]["proportion"] = 0.2f;
  ASSERT_TRUE(server.CommitConfig(proportion_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 1u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);

  // The sun moved: every landed energy changes, cold start.
  capture.Clear();
  auto sun_change = proportion_change;
  sun_change["scene"]["light_source"]["altitude"] = 40.0f;
  ASSERT_TRUE(server.CommitConfig(sun_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 1u);

  // A scattering layer added: a changed shape, so a changed input — the previous
  // object has no row for the new layer. Cold start, and the final line at the
  // next Stop reports both layers.
  capture.Clear();
  auto layer_change = sun_change;
  layer_change["scene"]["scattering"][0]["prob"] = 0.5f;
  layer_change["scene"]["scattering"].push_back(layer_change["scene"]["scattering"][0]);
  layer_change["scene"]["scattering"][1]["prob"] = 0.0f;
  ASSERT_TRUE(server.CommitConfig(layer_change).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 1u);
  ASSERT_TRUE(WaitUntilIdle(server, std::chrono::seconds(30)));
  server.Terminate();
  EXPECT_EQ(Count(capture.Text(), "RayAllocationOnline(final): layer 1 entry 1"), 1u);
}

TEST(RayAllocationOnlineCommit, SwitchingModesStartsColdBecauseAProportionalSceneHasNoTally) {
  // proportional → adaptive on an otherwise identical scene: the previous commit
  // bound no object, so there is nothing to carry. adaptive → proportional →
  // adaptive likewise: the proportional commit in between dropped it. Neither
  // transition may read as "unchanged".
  LogCapture capture;
  Server server(1);
  ASSERT_TRUE(server.CommitConfig(MakeConfig("proportional")).IsSuccess());
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 1u);
  capture.Clear();
  ASSERT_TRUE(server.CommitConfig(MakeConfig("proportional")).IsSuccess());
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  EXPECT_EQ(Count(capture.Text(), kColdStart), 1u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);
  server.Terminate();
}

TEST(RayAllocationOnlineCommit, CommitConfigTracesNothingOnTheCallingThread) {
  // AC5's red-state probe. The pilot ran its trace synchronously inside
  // CommitConfig on the calling thread, reseeding and drawing from that thread's
  // RandomNumberGenerator; after such a commit the draws below could not match the
  // reference sequence (measured red on the pilot build of this same scene: the
  // pilot dealt 200k rays through it). Every worker thread owns its own instance
  // and seeds it on entry, so nothing a worker does can make this pass by accident.
  constexpr uint32_t kSeed = 0x5A11u;
  constexpr size_t kDraws = 64;
  const auto reference = DrawSequence(kSeed, kDraws);
  Server server(1);
  auto& rng = RandomNumberGenerator::GetInstance();
  rng.SetSeed(kSeed);
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive")).IsSuccess());
  std::vector<float> after;
  for (size_t i = 0; i < kDraws; i++) {
    after.push_back(rng.GetUniform());
  }
  server.Terminate();
  EXPECT_EQ(after, reference) << "CommitConfig consumed the calling thread's RNG — something traced on it";
}

TEST(RayAllocationOnlineCommit, TheRunPublishesItsQAsMilestoneAndFinalLines) {
  // 20k rays on the legacy path: the first layer's dealt count crosses 1, 2, 4,
  // ... 16384 — at least one milestone group. Both line shapes carry the same
  // fields, and the final one reports the run's converged shares: the filtered
  // entry's share must have moved well under its p share (1/2) on this scene.
  LogCapture capture;
  Server server(1);
  ASSERT_TRUE(server.CommitConfig(MakeConfig("adaptive", 20'000)).IsSuccess());
  ASSERT_TRUE(WaitUntilIdle(server, std::chrono::seconds(60)));
  server.Terminate();
  const std::string text = capture.Text();
  EXPECT_GE(Count(text, kMilestone), 2u);
  EXPECT_EQ(Count(text, kFinal), 2u);
  const std::regex final_re(
      R"(RayAllocationOnline\(final\): layer 0 entry (\d+): p=([0-9.eE+-]+) q=([0-9.eE+-]+) rays=(\d+))");
  double q_filtered = -1.0;
  size_t rays_total = 0;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), final_re); it != std::sregex_iterator(); ++it) {
    const auto& m = *it;
    if (m[1].str() == "1") {
      q_filtered = std::stod(m[3].str());
    }
    rays_total += static_cast<size_t>(std::stoull(m[4].str()));
  }
  ASSERT_GE(q_filtered, 0.0) << "no final line for the filtered entry in:\n" << text;
  EXPECT_LT(q_filtered, 0.25) << "the filtered entry lands far less per ray; its share must fall under p's 0.5";
  EXPECT_GT(q_filtered, 0.0);
  EXPECT_EQ(rays_total, 20'000u) << "the final tally counts every ray the run dealt on layer 0";
}

}  // namespace lumice
