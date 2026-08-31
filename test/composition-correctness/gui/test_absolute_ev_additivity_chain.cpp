// Composition chain: does the number the GUI puts on screen mean what absolute mode promises?
//
// Units in the chain: the C API server running a real simulation x the poller's auto-EV
// derivation (LUMICE_ComputeP99Y -> LUMICE_ComputeEvAuto, exactly the two calls server_poller.cpp
// and app.cpp make) x the GUI's own exposure arithmetic (gui/mono_exposure_scale.hpp). No window,
// no GL, no rendered frame — but four units and a live simulation, which is why this is a
// composition chain and not a unit test.
//
// What the collaboration produces that is observable: the brightness the mono preview would draw
// with. The GUI does NOT ask the server for that scale; it pulls raw XYZ out of a result frame and
// multiplies client-side, so nothing downstream of `ComputeMonoExposure` can rescue a formula that
// is wrong here. The claim absolute mode makes about that number is a physical one — "this is how
// bright the scene really is, on a scale two different documents share" — and a physical claim
// needs a physical oracle rather than a screenshot.
//
// The oracle is additivity, reused from the CLI-side sibling at
// test/regression-sentinel/test_relative_ev_breaks_additivity.py: split one scene into a filtered
// half and its complement, and the two halves must add back up to the whole. It is the right shape
// for this because the absolute denominator is EMITTED energy, which does not know the filter
// exists — so the anchor is identical across all three runs and the displayed energies inherit the
// additivity the raw energies have. Relative divides each frame by its own landed intensity, so
// each half is re-brightened by roughly what its filter removed and the sum overshoots by ~100%.
// That second half is not a curiosity: it is what makes the first case load-bearing. A tolerance
// band wide enough to swallow the relative failure would pass on a build where the mode does
// nothing at all, and the mode doing nothing is the exact regression worth guarding.
//
// This is deliberately NOT a re-test of the sibling. That file measures the scale the SERVER bakes
// into an image, recovered from the rendered pixels; this one measures the scale the GUI computes
// for itself and never sends anywhere. They are two different formulas that are supposed to agree,
// and the day they stop agreeing only one of the two files moves.
//
// Cost: three simulations of one million rays each, single-threaded (a non-zero sim_seed clamps the
// CPU route to one worker, server.hpp), ~6.6s wall on the reference Mac. Measured before the test
// was written rather than after: the alternative was an offline XYZ fixture, which would have cost
// nothing but would also have stopped covering "the GUI can actually get emitted_energy out of the
// C API", which is half of what this file is for.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "core/color_util.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/mono_exposure_scale.hpp"
#include "lumice.h"

namespace lumice::gui {
namespace {

// The three splits, taken verbatim out of the shipped e2e corpus rather than rebuilt here.
//
// They are the fixtures the CLI-side oracle already runs on, so the scene, the filter and its
// complement have one definition between the two layers. Two properties of theirs are load-bearing
// and are asserted below rather than trusted: `scattering[0].prob` is 0 (Design A terminates a
// filter-failed ray mid-flight, which breaks additivity structurally at ms_prob > 0 and has
// nothing to do with ev_mode), and all three emit the same energy.
//
// Their `ev_mode` and `intensity_factor` fields are NOT used by anything this file measures, and
// that is worth stating because both look like they should be. `intensity_factor` is the server's
// EV knob, applied when the server bakes an image; the mono preview applies the user's EV slider
// itself and never reads it — so the deliberate half-stop the two halves carry does not have to be
// divided out here the way the CLI-side sibling has to divide it out. And the declared `ev_mode`
// only selects the anchor for that same server-side bake: the raw XYZ buffer is unexposed and comes
// out bit-identical under either declaration (verified by running all three splits under
// absolute_additivity_*_relative.json and comparing sum, P99 and snapshot_intensity). That is why
// ONE set of three runs serves both the absolute case and the relative negative control below —
// the mode being tested is the one passed to ComputeMonoExposure, not the one in the file.
const char* const kSplitNames[] = { "all", "in", "out" };
constexpr int kAll = 0;
constexpr int kIn = 1;
constexpr int kOut = 2;

// Pinned so the three runs are reproducible against each other and against the numbers quoted in
// the tolerance comments. Same value the CLI-side sibling pins, and the same consequence: a
// non-zero seed collapses the CPU route to a single worker.
constexpr unsigned int kSimSeed = 42;

struct Split {
  double sum_y = 0.0;         // raw (unexposed) luminance over the whole frame
  double sum_y_region = 0.0;  // ... and over the fixed centre box
  float emitted_energy = 0.0f;
  float snapshot_intensity = 0.0f;
  float p99_y = 0.0f;
  int width = 0;
  int height = 0;
};

std::string ReadConfig(const char* split) {
  const std::string path = std::string(LUMICE_E2E_CONFIG_DIR) + "/absolute_additivity_" + split + ".json";
  std::ifstream in(path);
  EXPECT_TRUE(in.is_open()) << "missing fixture: " << path;
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

bool WaitForIdle(LUMICE_Server* server, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    LUMICE_ServerState state{};
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// The region the "some area" half of the claim is measured over.
//
// A centred box covering half of each dimension, i.e. a quarter of the frame, holding ~15.6% of the
// scene's luminance. Its defining property is that it is chosen from the image GEOMETRY and not
// from the data. A mask picked by thresholding one run's own pixels — the top decile of `all`, say —
// looks like the obvious "bright region" choice and is a trap: it selects on that run's noise, the
// two halves regress toward the mean inside it, and the absolute residual lands at a stable -26%
// that reads exactly like a broken formula. Measured, not reasoned about. If this box is ever
// re-cut, re-cut it geometrically.
bool InRegion(int x, int y, int w, int h) {
  return x >= w / 4 && x < w - w / 4 && y >= h / 4 && y < h - h / 4;
}

Split MeasureSplit(const char* split) {
  Split out;
  LUMICE_ServerConfig cfg{};
  cfg.num_workers = 0;
  cfg.sim_seed = kSimSeed;
  cfg.preferred_backend = LUMICE_BACKEND_CPU;
  LUMICE_Server* server = LUMICE_CreateServerEx(&cfg);
  EXPECT_NE(server, nullptr);
  if (server == nullptr) {
    return out;
  }

  LUMICE_Scene* scene = nullptr;
  const LUMICE_ErrorCode parsed = LUMICE_SceneFromJson(ReadConfig(split).c_str(), &scene);
  EXPECT_EQ(parsed, LUMICE_OK) << split;
  if (parsed == LUMICE_OK) {
    EXPECT_EQ(LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr), LUMICE_OK) << split;
    LUMICE_SceneDestroy(scene);
    EXPECT_TRUE(WaitForIdle(server, 300000)) << split << ": simulation did not finish";

    LUMICE_ResultFrame* frame = nullptr;
    if (LUMICE_AcquireResultFrame(server, &frame) == LUMICE_OK) {
      LUMICE_RawXyzResult xyz[2]{};
      if (LUMICE_FrameGetRawXyz(frame, xyz, 1) == LUMICE_OK && xyz[0].has_valid_data != 0) {
        out.width = xyz[0].img_width;
        out.height = xyz[0].img_height;
        out.emitted_energy = xyz[0].emitted_energy;
        out.snapshot_intensity = xyz[0].snapshot_intensity;
        // The same two calls, with the same downsample factor, that server_poller.cpp and app.cpp
        // make to fill GuiState::p99_raw_y — not a reimplementation of the percentile here.
        out.p99_y = LUMICE_ComputeP99Y(xyz[0].xyz_buffer, out.width, out.height, kEvAutoDownsampleFactor);
        const float* buf = xyz[0].xyz_buffer;
        for (int y = 0; y < out.height; ++y) {
          for (int x = 0; x < out.width; ++x) {
            const double luminance = buf[(static_cast<size_t>(y) * out.width + x) * 3 + 1];
            out.sum_y += luminance;
            if (InRegion(x, y, out.width, out.height)) {
              out.sum_y_region += luminance;
            }
          }
        }
      }
      LUMICE_ReleaseResultFrame(frame);
    }
  }

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
  return out;
}

// The displayed luminance the mono preview would produce for one split, at one mode, at one EV.
//
// `ev_auto` is derived here exactly as the app derives it, from this frame's own P99 — which is the
// point of the relative branch: the anchor moves per frame. Under absolute the value is computed
// and passed in anyway, so that a formula which quietly folded it back in would show up as a
// failure rather than as an argument nobody passed.
double DisplayedLuminance(MonoEvMode mode, const Split& s, double raw_sum, float exposure_offset) {
  MonoExposureInput in;
  in.exposure_offset = exposure_offset;
  in.ev_auto = LUMICE_ComputeEvAuto(s.p99_y, s.snapshot_intensity, GuiState{}.target_white);
  in.snapshot_intensity = s.snapshot_intensity;
  in.snapshot_emitted_energy = s.emitted_energy;
  in.total_pixels = s.width * s.height;
  return raw_sum * ComputeMonoExposure(mode, in).intensity_scale;
}

// (in + out - all) / all on displayed luminance: 0 when the two halves partition the whole.
double AdditivityExcess(MonoEvMode mode, const Split* splits, bool region, float exposure_offset) {
  const auto pick = [region](const Split& s) { return region ? s.sum_y_region : s.sum_y; };
  const double all = DisplayedLuminance(mode, splits[kAll], pick(splits[kAll]), exposure_offset);
  const double in = DisplayedLuminance(mode, splits[kIn], pick(splits[kIn]), exposure_offset);
  const double out = DisplayedLuminance(mode, splits[kOut], pick(splits[kOut]), exposure_offset);
  EXPECT_GT(all, 0.0);
  return (in + out - all) / all;
}

// Measured spread of the absolute residual over six seeds (42, 7, 123, 2024, 555, 31337) at the
// fixture's one-million-ray budget: whole frame -2.31%..+2.34%, centre box -0.58%..+2.81% (box on
// the first four seeds). The band is ~2.5x the worst of those and ~35x under the relative effect,
// so it is wide enough not to chase Monte-Carlo noise and nowhere near wide enough to swallow what
// it exists to detect. It is not imported from the CLI-side sibling's identically-valued constant:
// that file measures a different quantity and the agreement is a result, not a shared definition.
constexpr double kAdditivityTol = 0.06;

// Measured relative excess on the same runs: +97.7%..+105.5% whole frame, +86.8%..+92.4% box. The
// floor sits at half the smallest of those — a margin no Monte-Carlo noise could produce, and far
// above the band the absolute case is held to, which is the gap the pair of cases is asserting.
constexpr double kBreakFloor = 0.50;

// One EV for all three splits, which is what "at the same EV" in the acceptance criterion means.
// Zero rather than an arbitrary non-zero value would leave the exposure_offset path untested, so
// the cases below run it at a non-zero stop count as well.
constexpr float kEv = 0.0f;

class AbsoluteEvAdditivity : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    for (int i = 0; i < 3; ++i) {
      splits_[i] = MeasureSplit(kSplitNames[i]);
    }
  }

  static Split splits_[3];
};

Split AbsoluteEvAdditivity::splits_[3];

// The premises the two oracle cases rest on, asserted rather than assumed: every one of them, if
// false, would make both cases below pass or fail for a reason that has nothing to do with the
// exposure mode.
TEST_F(AbsoluteEvAdditivity, the_three_runs_share_an_emitted_anchor_and_disagree_on_the_landed_one) {
  ASSERT_GT(splits_[kAll].sum_y, 0.0) << "the unfiltered run produced no light";
  ASSERT_GT(splits_[kIn].sum_y, 0.0) << "the filtered half produced no light";
  ASSERT_GT(splits_[kOut].sum_y, 0.0) << "the complementary half produced no light";
  ASSERT_GT(splits_[kAll].sum_y_region, 0.0) << "the centre box caught nothing";

  // The defining property of the absolute anchor, and the reason additivity can hold at all: the
  // denominator counts what the source emitted, so removing rays with a filter does not move it.
  // Equality here is exact-ish by construction (same ray budget, same light source), so the band is
  // a formatting allowance, not a statistical one.
  EXPECT_NEAR(splits_[kIn].emitted_energy, splits_[kAll].emitted_energy, splits_[kAll].emitted_energy * 1e-6f);
  EXPECT_NEAR(splits_[kOut].emitted_energy, splits_[kAll].emitted_energy, splits_[kAll].emitted_energy * 1e-6f);

  // And the property that makes the negative control non-trivial: the landed anchors genuinely
  // differ, so the relative formula really is dividing the three frames by three different numbers.
  // Without this, "relative breaks additivity" could be true for some other reason entirely.
  EXPECT_LT(splits_[kIn].snapshot_intensity, splits_[kAll].snapshot_intensity * 0.9f);
  EXPECT_LT(splits_[kOut].snapshot_intensity, splits_[kAll].snapshot_intensity * 0.9f);

  // The frames are the same size, which the region comparison silently depends on.
  EXPECT_EQ(splits_[kIn].width, splits_[kAll].width);
  EXPECT_EQ(splits_[kIn].height, splits_[kAll].height);
  EXPECT_EQ(splits_[kOut].width, splits_[kAll].width);
  EXPECT_EQ(splits_[kOut].height, splits_[kAll].height);
}

TEST_F(AbsoluteEvAdditivity, absolute_mode_makes_the_gui_brightness_add_up_the_way_the_physics_does) {
  EXPECT_NEAR(AdditivityExcess(MonoEvMode::kAbsolute, splits_, /*region=*/false, kEv), 0.0, kAdditivityTol);
  EXPECT_NEAR(AdditivityExcess(MonoEvMode::kAbsolute, splits_, /*region=*/true, kEv), 0.0, kAdditivityTol);

  // Same claim with the EV slider off zero. A scale that folded the manual stop count in twice, or
  // dropped it, would leave the ratio above untouched — it cancels — so the exposure_offset path
  // needs its own statement: the residual is unchanged, and the absolute brightness itself moves by
  // exactly the stops asked for.
  EXPECT_NEAR(AdditivityExcess(MonoEvMode::kAbsolute, splits_, /*region=*/false, 3.0f), 0.0, kAdditivityTol);
  const double at_zero = DisplayedLuminance(MonoEvMode::kAbsolute, splits_[kAll], splits_[kAll].sum_y, 0.0f);
  const double at_three = DisplayedLuminance(MonoEvMode::kAbsolute, splits_[kAll], splits_[kAll].sum_y, 3.0f);
  EXPECT_NEAR(at_three / at_zero, 8.0, 8.0 * 1e-5);
}

TEST_F(AbsoluteEvAdditivity, relative_mode_does_not_which_is_what_makes_the_absolute_case_worth_asserting) {
  // Not a defect. Relative is the default and a self-anchored picture is supposed to follow its own
  // content — that is what the mode is for. What must not happen is the two modes behaving alike,
  // because then everything this task added to the GUI is inert while a combo box, a document key
  // and an ABI slot all claim otherwise.
  EXPECT_GT(AdditivityExcess(MonoEvMode::kRelative, splits_, /*region=*/false, kEv), kBreakFloor);
  EXPECT_GT(AdditivityExcess(MonoEvMode::kRelative, splits_, /*region=*/true, kEv), kBreakFloor);
}

// gui_constants.hpp:kNormScale is a hand-mirrored copy of core's kNormScale (src=/gui/ may not
// #include core/ — AGENTS.md), kept in sync only because this assertion goes red when it drifts.
// No fixture: this is a compile-time-cheap equality check, not part of the additivity oracle above.
TEST(GuiConstants, ExposureScaleMirrorsCore) {
  EXPECT_EQ(lumice::gui::kNormScale, lumice::kNormScale);
}

}  // namespace
}  // namespace lumice::gui
