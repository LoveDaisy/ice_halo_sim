// The online ray allocation as StartRaypathAnalysis binds it: the analysis session
// follows the same scene.ray_allocation field CommitConfig reads, binds the same
// RayAllocationOnline, and — the half that differs from the render commit — starts
// COLD on every submission, never carrying a tally forward.
//
// Why a separate file from test_ray_allocation_online_commit.cpp rather than more
// cases in it: the two publishers share the bind condition but not the start
// policy, and the carry-forward assertions over there are exactly what must NOT hold
// here. Keeping the analysis side's "always cold" as its own file is what makes a
// future port of CommitConfig's debounce into StartRaypathAnalysis read red instead
// of quietly widening a shared fixture. The log-capture scaffolding both files
// assert through is one copy, support/log_capture.hpp.
//
// The reason the analysis binds at all is the histogram's rows: each one's share is
// as much at the mercy of the sampling share as an image's pixels are, and on the
// 98/120/144 three-crystal scene the rare row's noise at equal ray budget is 10.5×
// lower under the online deal (its expectation unchanged — Σ(Y·w) carries the p/q
// correction). That statistic is pinned end to end in
// test/e2e-correctness/test_raypath_analysis_capi.py; this file pins the bind.
//
// Counted by log line, not by reading a private member: the bind decision's only
// externally visible channel on the live server is the line each branch emits.

#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "server/server.hpp"
#include "support/log_capture.hpp"

namespace lumice {
namespace {

using test::LogCapture;

// StartRaypathAnalysis's own cold-start line — the prefix is the calling site, so a
// CommitConfig cold start (same tail, "CommitConfig:" prefix) cannot satisfy it.
constexpr const char* kAnalysisColdStart = "StartRaypathAnalysis: ray-allocation online tally started";
// CommitConfig's carry-forward line: must never appear on the analysis path.
constexpr const char* kTallyKept = "keeping the online tally";
// Any bind at all, from either site, plus the run's own milestone/final lines.
constexpr const char* kAnyColdStart = "ray-allocation online tally started";
constexpr const char* kMilestone = "RayAllocationOnline: layer ";
constexpr const char* kFinal = "RayAllocationOnline(final): layer ";

size_t Count(const std::string& haystack, const char* needle) {
  return test::CountOccurrences(haystack, needle);
}

// Two prism entries, one filtered to a single raypath: a real energy skew for the
// statistic, cheap on the CPU path the analysis is forced onto. The render section is
// the loader's (a config without one is rejected as a missing field); the analysis
// binds no renderer off it.
nlohmann::json MakeScene(const char* ray_allocation, size_t ray_num = 1000) {
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

bool WaitUntilNotRunning(const Server& server, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (server.GetSimLifecycle() != SimLifecycle::kRunning) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// One submission, run to its end. A caller in a loop tests HasFatalFailure() after it: a
// rejected submission leaves nothing meaningful for the next iteration to count.
void SubmitAndFinish(Server& server, const nlohmann::json& scene) {
  ASSERT_TRUE(server.StartRaypathAnalysis(scene, RaypathAnalysisRequest{}).IsSuccess());
  ASSERT_TRUE(WaitUntilNotRunning(server, std::chrono::seconds(30)));
}

}  // namespace

TEST(RayAllocationOnlineAnalysis, ProportionalBindsNoTally) {
  // The default and the explicit spelling alike: no bind, so no line of any kind
  // — not the cold start, not a milestone, not the final report.
  LogCapture capture;
  Server server(1);
  SubmitAndFinish(server, MakeScene(nullptr));
  SubmitAndFinish(server, MakeScene("proportional"));
  server.Terminate();
  EXPECT_EQ(Count(capture.Text(), kAnyColdStart), 0u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);
  EXPECT_EQ(Count(capture.Text(), kMilestone), 0u);
  EXPECT_EQ(Count(capture.Text(), kFinal), 0u);
}

TEST(RayAllocationOnlineAnalysis, AdaptiveStartsColdOnEverySubmission) {
  // Three submissions of the SAME adaptive scene, each after the previous run ended:
  // three cold starts and zero carries. This is the assertion CommitConfig's own test
  // makes the other way round (an unchanged scene KEEPS the tally there), and the
  // regression gate against porting that debounce here.
  LogCapture capture;
  Server server(1);
  for (int i = 0; i < 3; i++) {
    SubmitAndFinish(server, MakeScene("adaptive"));
    if (HasFatalFailure()) {
      return;
    }
    EXPECT_EQ(Count(capture.Text(), kAnalysisColdStart), static_cast<size_t>(i + 1));
    EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);
  }
  server.Terminate();
  // Every cold start on this path is the analysis site's: nothing here went through
  // CommitConfig.
  EXPECT_EQ(Count(capture.Text(), kAnyColdStart), Count(capture.Text(), kAnalysisColdStart));
  // The run publishes what it converged on, once per submission (one layer, two
  // entries, one final line each).
  EXPECT_EQ(Count(capture.Text(), kFinal), 6u);
}

TEST(RayAllocationOnlineAnalysis, TheModeIsReadFromTheSubmittedSceneEachTime) {
  // proportional → adaptive → proportional → adaptive: the bind is decided by the
  // field of the scene being submitted, not by what the previous session was, and a
  // mode switch is neither a carry nor a leak — the two proportional submissions
  // write no line, the two adaptive ones each write one cold start.
  LogCapture capture;
  Server server(1);
  for (const char* mode : { "proportional", "adaptive", "proportional", "adaptive" }) {
    SubmitAndFinish(server, MakeScene(mode));
    if (HasFatalFailure()) {
      return;
    }
  }
  server.Terminate();
  EXPECT_EQ(Count(capture.Text(), kAnalysisColdStart), 2u);
  EXPECT_EQ(Count(capture.Text(), kTallyKept), 0u);
  EXPECT_EQ(Count(capture.Text(), kFinal), 4u);
}

}  // namespace lumice
