// The composite (raypath-colour) path's own background colour.
//
// Why this file exists: the composite image is baked server-side into sRGB uint8 before the GUI
// ever sees it, so the preview shader — which is where the mono path gets its background painted —
// receives pixels that are already gamma-encoded and cannot have a linear background added to them.
// The background therefore has to be added inside the compositor, and the propositions worth
// pinning are the two that a "just add it to every pixel" implementation gets wrong:
//
//   * the region OUTSIDE the lens's domain (beyond a fisheye's image circle, or the hemisphere
//     `visible` excludes) must stay black, exactly as the mono path leaves it. The compositor has
//     no projection state of its own, so it borrows RenderConsumer's per-pixel mask; a
//     re-derivation, or no mask at all, shows up here as painted corners.
//   * the two paths must agree. Toggling raypath colour is a display-time switch: if the mono and
//     composite backgrounds differ by even one byte, the picture visibly changes underneath the
//     user for a reason unrelated to what they toggled. That is asserted directly — same frame,
//     both buffers, memcmp over the background region — rather than by checking each path against
//     an expected value and inferring they must therefore match.
//
// Every case reads pixels that carry no halo energy, which is what makes the assertions exact
// rather than tolerance-based: with zero radiance the whole chain reduces to
// `clamp(background) -> sRGB curve -> truncate`, on both paths.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "lumice.h"
#include "util/color_space.hpp"

namespace {

// A 64x32 dual-fisheye canvas: two inscribed image circles of radius 16, so all four canvas
// corners AND the four corners of each half sit outside the lens domain. That is the geometry the
// masking assertions need; the resolution is tiny because these cases assert on bytes, not physics.
// The single colour class makes ColoredMask() non-zero, which is what makes DoSnapshot produce a
// composite at all. `background` stays [0,0,0] here: the mono path's background is set per-case
// (via a config string rewrite) only where the cross-path comparison needs it.
std::string ColorConfig(const char* mode, const char* background_srgb) {
  return std::string(R"({
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
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": )") +
         background_srgb + R"(,
    "opacity": 1.0, "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": ")" +
         mode + R"(",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  }
})";
}

constexpr int kWidth = 64;
constexpr int kHeight = 32;

// The sky colour every case pushes, as a picker would give it (sRGB, 0..1). Deliberately three
// distinct components so a channel swap in the copy cannot pass, and none of them 0 or 1 so a
// clamp cannot mask an error.
constexpr float kSkySrgb[3] = { 0.20f, 0.35f, 0.85f };

LUMICE_ErrorCode CommitJson(LUMICE_Server* server, const std::string& json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

bool WaitForIdle(LUMICE_Server* server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    LUMICE_ServerState state{};
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// Row-major pixel index. The four canvas corners plus the four corners of each 32x32 half — every
// one of them more than radius 16 from its half's centre, so all eight are outside the lens domain.
// Listing the inner-half corners too is what separates "the mask is applied" from "the outer frame
// happens to be dark".
const int kOutsidePixels[] = {
  0,                                    // (0, 0)
  kWidth - 1,                           // (63, 0)
  (kHeight - 1) * kWidth,               // (0, 31)
  (kHeight - 1) * kWidth + kWidth - 1,  // (63, 31)
  31,                                   // (31, 0)  left half, top-right corner
  32,                                   // (32, 0)  right half, top-left corner
  (kHeight - 1) * kWidth + 31,          // (31, 31)
  (kHeight - 1) * kWidth + 32,          // (32, 31)
};

class CompositeBackground : public ::testing::Test {
 protected:
  void SetUp() override {
    LUMICE_ServerConfig cfg{};
    cfg.num_workers = 1;
    server_ = LUMICE_CreateServerEx(&cfg);
    ASSERT_NE(server_, nullptr);
  }

  void TearDown() override {
    if (server_ != nullptr) {
      LUMICE_StopServer(server_);
      LUMICE_DestroyServer(server_);
      server_ = nullptr;
    }
  }

  void RunSim(const char* mode, const char* background_srgb) {
    ASSERT_EQ(CommitJson(server_, ColorConfig(mode, background_srgb)), LUMICE_OK);
    ASSERT_TRUE(WaitForIdle(server_, 20000)) << "simulation did not finish in 20s";
  }

  // Pushes kSkySrgb (converted to linear, which is the unit the setter takes) and returns the
  // freshly re-baked composite. The frame is kept alive by the caller-owned out_frame.
  void PushSkyAndAcquireComposite(LUMICE_ResultFrame** out_frame, LUMICE_RenderResult* out_comp) {
    float sky_linear[3];
    lumice::SrgbToLinearRgb(kSkySrgb, sky_linear);
    ASSERT_EQ(LUMICE_SetCompositeBackground(server_, sky_linear), LUMICE_OK);
    ASSERT_EQ(LUMICE_AcquireResultFrame(server_, out_frame), LUMICE_OK);
    LUMICE_RenderResult comp[2]{};
    ASSERT_EQ(LUMICE_FrameGetComposite(*out_frame, comp, 1), LUMICE_OK);
    ASSERT_NE(comp[0].img_buffer, nullptr) << "no composite produced — the case would be vacuous";
    ASSERT_EQ(comp[0].img_width, kWidth);
    ASSERT_EQ(comp[0].img_height, kHeight);
    *out_comp = comp[0];
  }

  LUMICE_Server* server_ = nullptr;
};

// The masking proposition, one case per composite mode. The three modes reach the final buffer by
// visibly different exposure routes (dominant/additive share the mono exposure scale, painter
// composites alpha off a self-anchor and post-multiplies), so a background injected at the wrong
// point in one of them can be right in the other two — hence three cases rather than one.
class CompositeBackgroundModes : public CompositeBackground, public ::testing::WithParamInterface<const char*> {};

TEST_P(CompositeBackgroundModes, OutsideTheLensDomainStaysBlack) {
  RunSim(GetParam(), "[0, 0, 0]");

  LUMICE_ResultFrame* frame = nullptr;
  LUMICE_RenderResult comp{};
  PushSkyAndAcquireComposite(&frame, &comp);

  for (int idx : kOutsidePixels) {
    const uint8_t* p = comp.img_buffer + static_cast<size_t>(idx) * 3;
    EXPECT_EQ(0, p[0]) << "pixel " << idx << " is outside the image circle but got red " << static_cast<int>(p[0]);
    EXPECT_EQ(0, p[1]) << "pixel " << idx << " is outside the image circle but got green " << static_cast<int>(p[1]);
    EXPECT_EQ(0, p[2]) << "pixel " << idx << " is outside the image circle but got blue " << static_cast<int>(p[2]);
  }

  LUMICE_ReleaseResultFrame(frame);
}

// The other half of the same proposition: where the lens DOES image, the background must actually
// be there and be exactly the colour that was pushed. Without this, an implementation that painted
// nothing at all would pass the masking case above.
//
// The pixels to check are found by measurement, not by naming coordinates: take the ones whose raw
// XYZ is exactly zero, i.e. no ray landed there at all, so every colour-class lane is zero too and
// the composite reduces to the background alone. (Reading "renders as black" instead would be too
// weak — a pixel holding a hundredth of a unit of radiance also truncates to byte 0 on its own, yet
// shifts the sum by a byte once a background is added underneath it.) After the push, every one of
// them must be either the background colour (it is inside the lens domain) or still black (it is
// outside), and nothing else; that "nothing else" is what would catch a background scaled by
// exposure or added on the wrong side of the transfer curve.
TEST_P(CompositeBackgroundModes, ZeroEnergyPixelsInsideTheDomainCarryExactlyThePushedColour) {
  RunSim(GetParam(), "[0, 0, 0]");

  LUMICE_ResultFrame* frame0 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame0), LUMICE_OK);
  LUMICE_RawXyzResult xyz[2]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame0, xyz, 1), LUMICE_OK);
  ASSERT_NE(xyz[0].xyz_buffer, nullptr);
  ASSERT_EQ(xyz[0].img_width, kWidth);
  ASSERT_EQ(xyz[0].img_height, kHeight);
  std::vector<int> zero_energy;
  for (int idx = 0; idx < kWidth * kHeight; ++idx) {
    const float* v = xyz[0].xyz_buffer + static_cast<size_t>(idx) * 3;
    if (v[0] == 0.0f && v[1] == 0.0f && v[2] == 0.0f) {
      zero_energy.push_back(idx);
    }
  }
  LUMICE_ReleaseResultFrame(frame0);
  ASSERT_FALSE(zero_energy.empty()) << "every pixel carries energy — no pixel can pin an exact background byte";

  // Bake 2: same lanes, sky pushed.
  LUMICE_ResultFrame* frame = nullptr;
  LUMICE_RenderResult comp{};
  PushSkyAndAcquireComposite(&frame, &comp);

  uint8_t expected[3];
  for (int j = 0; j < 3; ++j) {
    expected[j] = static_cast<uint8_t>(lumice::LinearToSrgb(lumice::SrgbToLinear(kSkySrgb[j])) * 255.0f);
  }

  size_t painted = 0;
  size_t left_black = 0;
  for (int idx : zero_energy) {
    const uint8_t* p = comp.img_buffer + static_cast<size_t>(idx) * 3;
    const bool is_background = p[0] == expected[0] && p[1] == expected[1] && p[2] == expected[2];
    const bool is_black = p[0] == 0 && p[1] == 0 && p[2] == 0;
    EXPECT_TRUE(is_background || is_black)
        << "zero-energy pixel " << idx << " came back (" << static_cast<int>(p[0]) << "," << static_cast<int>(p[1])
        << "," << static_cast<int>(p[2]) << "), which is neither the pushed background nor black";
    painted += is_background ? 1 : 0;
    left_black += is_black ? 1 : 0;
  }
  EXPECT_GT(painted, 0u) << "the background reached no zero-energy pixel at all";
  EXPECT_GT(left_black, 0u) << "no zero-energy pixel was left black — the mask withheld the background nowhere, so "
                               "this fixture no longer separates masked from unmasked";

  LUMICE_ReleaseResultFrame(frame);
}

INSTANTIATE_TEST_SUITE_P(AllModes, CompositeBackgroundModes, ::testing::Values("dominant", "additive", "painter"),
                         [](const ::testing::TestParamInfo<const char*>& info) { return std::string(info.param); });

}  // namespace
