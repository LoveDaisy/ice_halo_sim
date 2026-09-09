// The subtractive (RenderConfig::kPrint) operator on the CLI/server side, asserted on the
// properties the design owes rather than on a table of bytes.
//
// The byte-exact statement of the main formula lives next door, in
// test_render_consumer_post_snapshot_fusion.cpp's PrintToneSubtractive case — that file already
// owns "the fused loop computes exactly this and reorders nothing", and adding a second byte table
// here would be a second thing to keep in step. What this file owns is the set of claims the print
// mode makes that a formula check cannot see:
//
//   - print is BLIND to the colour controls the screen operator reads. Not "happens to compute
//     something similar" — literally byte-identical output when ray_color_, the annotation colours
//     or the sky background are changed underneath it. That is the executable form of "ink has one
//     degree of freedom" (doc/print-mode-subtractive-ink.md §5) and of AC4's "annotation does not
//     read the colour field", and it is stated as "change the input, the bytes do not move" rather
//     than by inspecting which functions the code calls, because the latter pins an implementation
//     detail instead of the promise.
//   - the paper's HUE survives. Every channel is the same transmittance times its own paper
//     component, so the output stays proportional to the paper at every exposure.
//   - the zero-energy colour is the paper, on all three of the paths that can produce a frame with
//     no light in it: an unimaged pixel, a snapshot before any intensity exists, and a snapshot
//     whose exposure scale is zero. The last two are early exits that never reach the pixel loop,
//     and before this change they memset the frame to black — a print document would have opened
//     on a black page.
//   - annotations DARKEN white paper. On the screen operator a line is visible because its colour
//     contrasts with the sky; on paper it is visible by construction, because multiplying by
//     (1 - alpha) can only go down. Worth an assertion precisely because it is the thing that lets
//     print drop the per-mode palette.
//
// Each case is a difference test against the screen operator or against a second print frame, so a
// pass says the change is attributable to the thing that was varied.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"
#include "support/render_anchor.hpp"
#include "util/color_space.hpp"
#include "util/ink_transfer.hpp"

namespace lumice {
namespace {

constexpr int kW = 64;
constexpr int kH = 64;
constexpr int kTotalPix = kW * kH;

// A warm off-white sheet. Three DISTINCT components on purpose: with {1,1,1} a bug that dropped the
// per-channel paper multiply and wrote the bare transmittance would produce identical bytes.
constexpr float kPaper[3]{ 0.94f, 0.90f, 0.82f };

SunParam MakeSun() {
  return SunParam{ 45.0f, 0.0f, 0.5f };
}

// A 180 deg equal-area fisheye pointed up, so the frame's corners fall outside the image circle and
// the "unimaged pixel" case has somewhere to happen. visible_ = kUpper for the same reason the
// other render_consumer fixtures use it: it is the default a real document lands on.
RenderConfig MakeConfig(RenderConfig::Tone tone) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = 90.0f;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.tone_ = tone;
  for (int j = 0; j < 3; j++) {
    cfg.paper_[j] = kPaper[j];
  }
  return cfg;
}

// A spread of directions rather than one straight-up ray: several exposure levels have to be
// present in a single frame for the hue and monotonicity checks to have anything to compare.
SimData MakeBatch() {
  SimData data;
  data.curr_wl_ = 550.0f;
  const float dirs[][3]{ { 0.0f, 0.0f, -1.0f },
                         { 0.3f, 0.0f, -0.954f },
                         { -0.3f, 0.0f, -0.954f },
                         { 0.0f, 0.3f, -0.954f },
                         { 0.0f, -0.3f, -0.954f } };
  const float weights[]{ 1.0f, 0.6f, 0.35f, 0.15f, 0.05f };
  for (size_t i = 0; i < std::size(weights); ++i) {
    for (int j = 0; j < 3; j++) {
      data.outgoing_d_.push_back(dirs[i][j]);
    }
    data.outgoing_w_.push_back(weights[i]);
  }
  return data;
}

std::vector<uint8_t> SnapshotOnce(const RenderConfig& cfg) {
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  auto data = MakeBatch();
  rc.Consume(data);
  lumice::test::TakeSnapshotAtFormerSelfAnchor(&rc);
  auto result = rc.GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  if (rr == nullptr || rr->img_buffer_ == nullptr) {
    return {};
  }
  return std::vector<uint8_t>(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
}

GridLineParam Line(float value_deg, float opacity, float r, float g, float b) {
  GridLineParam p;
  p.value_ = value_deg;
  p.opacity_ = opacity;
  p.color_[0] = r;
  p.color_[1] = g;
  p.color_[2] = b;
  return p;
}

// The paper as it comes out of the writer: the same clamp / transfer curve / narrowing the pixel
// loop applies to every channel, evaluated at transmittance 1.
std::vector<uint8_t> PaperBytes() {
  std::vector<uint8_t> out(3);
  for (int j = 0; j < 3; j++) {
    out[j] = static_cast<uint8_t>(LinearToSrgb(std::clamp(kPaper[j], 0.0f, 1.0f)) * 255);
  }
  return out;
}

bool PixelIs(const std::vector<uint8_t>& img, int i, const std::vector<uint8_t>& rgb) {
  const size_t base = static_cast<size_t>(i) * 3;
  return img[base] == rgb[0] && img[base + 1] == rgb[1] && img[base + 2] == rgb[2];
}

size_t CountPixelsEqualTo(const std::vector<uint8_t>& img, const std::vector<uint8_t>& rgb) {
  size_t n = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (PixelIs(img, i, rgb)) {
      ++n;
    }
  }
  return n;
}

// How many pixels differ between two frames. The unit of every difference test below.
size_t CountDifferingBytes(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
  size_t n = 0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    if (a[i] != b[i]) {
      ++n;
    }
  }
  return n;
}

// -----------------------------------------------------------------------------
// print does not read the colour path the screen operator reads.
//
// The print branch takes the exposed scalar the pixel loop has already computed and skips the
// use_real_color / ray_color_ machinery whole. That is a statement about the code; what is asserted
// here is its observable consequence, which is the part that has to hold no matter how the branch
// is written: two ray_color_ values that produce visibly different SCREEN frames must produce
// byte-identical PRINT frames.
// -----------------------------------------------------------------------------
TEST(RenderConsumerPrintMode, PrintIgnoresRayColor) {
  RenderConfig warm = MakeConfig(RenderConfig::kPrint);
  warm.ray_color_[0] = 1.0f;
  warm.ray_color_[1] = 0.5f;
  warm.ray_color_[2] = 0.2f;
  RenderConfig cool = warm;
  cool.ray_color_[0] = 0.2f;
  cool.ray_color_[1] = 0.5f;
  cool.ray_color_[2] = 1.0f;

  const auto print_warm = SnapshotOnce(warm);
  const auto print_cool = SnapshotOnce(cool);
  ASSERT_EQ(print_warm.size(), static_cast<size_t>(kTotalPix) * 3);
  EXPECT_EQ(CountDifferingBytes(print_warm, print_cool), 0u)
      << "ray_color_ reached the print branch — it must not; print is greyscale ink on paper "
         "(doc/print-mode-subtractive-ink.md §5). If this is red, do not relax it: the branch is "
         "reading a colour field it was designed to skip.";

  // The control: the same two configs on the SCREEN operator have to differ, or the case above is
  // vacuous — it would pass just as well against a ray_color_ that no operator ever reads.
  RenderConfig screen_warm = warm;
  screen_warm.tone_ = RenderConfig::kScreen;
  RenderConfig screen_cool = cool;
  screen_cool.tone_ = RenderConfig::kScreen;
  EXPECT_GT(CountDifferingBytes(SnapshotOnce(screen_warm), SnapshotOnce(screen_cool)), 0u)
      << "the screen control did not move either — this fixture has stopped covering ray_color_ at "
         "all, so the print assertion above proves nothing";
}

// print must also ignore the SKY. `background` and `paper` are separate fields precisely so that
// "print onto the default black background" cannot happen (doc/print-mode-subtractive-ink.md §6);
// the observable half of that is the print frame not moving when the sky colour does.
TEST(RenderConsumerPrintMode, PrintIgnoresSkyBackground) {
  RenderConfig black_sky = MakeConfig(RenderConfig::kPrint);
  RenderConfig blue_sky = black_sky;
  blue_sky.background_[0] = 0.1f;
  blue_sky.background_[1] = 0.3f;
  blue_sky.background_[2] = 0.7f;

  EXPECT_EQ(CountDifferingBytes(SnapshotOnce(black_sky), SnapshotOnce(blue_sky)), 0u)
      << "the sky background reached the print branch — under kPrint the ground is the paper, and "
         "the sky term is not part of the operator at all";

  RenderConfig screen_black = black_sky;
  screen_black.tone_ = RenderConfig::kScreen;
  RenderConfig screen_blue = blue_sky;
  screen_blue.tone_ = RenderConfig::kScreen;
  EXPECT_GT(CountDifferingBytes(SnapshotOnce(screen_black), SnapshotOnce(screen_blue)), 0u)
      << "the screen control did not move — this fixture has stopped covering the background";
}

// -----------------------------------------------------------------------------
// AC3 — the paper's hue survives every exposure.
//
// Each channel is the SAME transmittance times its OWN paper component, so the linear output stays
// proportional to the paper everywhere. Asserted in linear rather than on the output bytes: the
// sRGB transfer curve is not linear, so a ratio taken on bytes would fail for reasons that have
// nothing to do with the operator.
// -----------------------------------------------------------------------------
TEST(RenderConsumerPrintMode, PaperHueIsPreservedAtEveryExposure) {
  const auto img = SnapshotOnce(MakeConfig(RenderConfig::kPrint));
  ASSERT_EQ(img.size(), static_cast<size_t>(kTotalPix) * 3);

  size_t checked = 0;
  size_t distinct_levels = 0;
  float prev_level = -1.0f;
  for (int i = 0; i < kTotalPix; ++i) {
    const size_t base = static_cast<size_t>(i) * 3;
    float linear[3];
    for (int j = 0; j < 3; j++) {
      linear[j] = SrgbToLinear(static_cast<float>(img[base + j]) * (1.0f / 255.0f));
    }
    // The transmittance this pixel implies, read off the channel with the most headroom.
    const float t = linear[0] / kPaper[0];
    for (int j = 1; j < 3; j++) {
      // 1/255 of an sRGB step is worth more than this in linear near the paper's brightness, so the
      // tolerance is quantisation, not slack in the claim.
      EXPECT_NEAR(linear[j], t * kPaper[j], 6e-3f)
          << "pixel " << i << " channel " << j << " left the paper's hue: the three channels must "
          << "share one transmittance";
    }
    ++checked;
    if (std::fabs(t - prev_level) > 1e-3f) {
      ++distinct_levels;
      prev_level = t;
    }
  }
  EXPECT_EQ(checked, static_cast<size_t>(kTotalPix));
  EXPECT_GT(distinct_levels, 1u) << "every pixel came out at the same exposure — a flat frame would "
                                    "satisfy the hue check trivially";
}

// -----------------------------------------------------------------------------
// AC5 — the zero-energy colour, on all three paths that can produce one.
// -----------------------------------------------------------------------------

// (a) A pixel the lens does not image. 64x64 under a 180 deg equal-area fisheye leaves the four
// corners outside the image circle.
TEST(RenderConsumerPrintMode, UnimagedPixelsPrintAsBarePaper) {
  const auto img = SnapshotOnce(MakeConfig(RenderConfig::kPrint));
  ASSERT_EQ(img.size(), static_cast<size_t>(kTotalPix) * 3);
  const auto paper = PaperBytes();
  // The exact corners, which are the furthest outside the circle and so unambiguous.
  const int corners[]{ 0, kW - 1, (kH - 1) * kW, kTotalPix - 1 };
  for (int i : corners) {
    EXPECT_TRUE(PixelIs(img, i, paper)) << "corner pixel " << i
                                        << " is not bare paper: it images no sky, so it received no ink";
  }
  EXPECT_GT(CountPixelsEqualTo(img, paper), 4u) << "only the four probed corners came out as paper — the unimaged "
                                                   "region has shrunk to nothing and this case no longer covers it";
}

// (b) and (c) The two early exits that never reach the pixel loop. Before the print operator these
// were a memset to black, which would have opened a print document on a black page. Driven through
// PostSnapshot directly rather than through the anchor helper, because the point is precisely that
// these return before any of that machinery runs.
TEST(RenderConsumerPrintMode, EarlyExitsFillWithTheZeroEnergyColour) {
  struct Arm {
    const char* name;
    RenderConfig::Tone tone;
    std::vector<uint8_t> expected;
  };
  const auto paper = PaperBytes();
  const std::vector<Arm> arms{ { "print", RenderConfig::kPrint, paper },
                               { "screen", RenderConfig::kScreen, { 0, 0, 0 } } };

  // One arm per call rather than inline in the loop: the buffer checks below have to be fatal (the
  // read after them would be out of bounds otherwise), and a fatal assert in a loop body returns out
  // of the whole test, hiding every arm after the first failure. The screen arm is the control here,
  // so losing it silently is exactly the outcome to avoid.
  const auto check_arm = [](const Arm& arm) {
    // No Consume() at all: snapshot_intensity_ stays zero, which is the first early exit — and it
    // is the real "the simulation has not produced anything yet" first frame, not a synthetic one.
    RenderConsumer rc(MakeConfig(arm.tone), ColorClassTable{}, MakeSun());
    rc.PrepareSnapshot();
    rc.PostSnapshot();
    auto result = rc.GetResult();
    const auto* rr = std::get_if<RenderResult>(&result);
    ASSERT_NE(rr, nullptr) << arm.name;
    ASSERT_NE(rr->img_buffer_, nullptr) << arm.name;
    const std::vector<uint8_t> img(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
    EXPECT_EQ(CountPixelsEqualTo(img, arm.expected), static_cast<size_t>(kTotalPix))
        << arm.name << ": an unexposed frame must be that mode's zero-energy colour everywhere";
  };
  for (const Arm& arm : arms) {
    check_arm(arm);
  }
}

// The exposure-scale early exit, reached by consuming a batch whose weights are all zero: rays
// land (so snapshot_intensity_ is non-zero and the first exit is passed) but no energy is deposited.
TEST(RenderConsumerPrintMode, ZeroExposureScaleFillsWithTheZeroEnergyColour) {
  const auto paper = PaperBytes();
  RenderConsumer rc(MakeConfig(RenderConfig::kPrint), ColorClassTable{}, MakeSun());
  SimData data;
  data.curr_wl_ = 550.0f;
  data.outgoing_d_ = { 0.0f, 0.0f, -1.0f };
  data.outgoing_w_ = { 0.0f };
  rc.Consume(data);
  rc.PrepareSnapshot();
  rc.PostSnapshot();
  ASSERT_LE(rc.ExposureScale(), 0.0f) << "this case is only meaningful if the scale<=0 early exit is the one taken; "
                                         "if the scale is positive the frame went through the pixel loop instead";
  auto result = rc.GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  ASSERT_NE(rr, nullptr);
  const std::vector<uint8_t> img(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
  EXPECT_EQ(CountPixelsEqualTo(img, paper), static_cast<size_t>(kTotalPix));
}

// -----------------------------------------------------------------------------
// AC4 — annotations are ink: they darken the paper, and their colour is not read.
// -----------------------------------------------------------------------------

// Every annotation family at once, with colours chosen so that a channel LEAK would be loud. The
// grid lines are pure saturated primaries; on the screen operator they tint, on paper they must
// only darken.
RenderConfig MakeAnnotatedConfig(RenderConfig::Tone tone, float r, float g, float b) {
  RenderConfig cfg = MakeConfig(tone);
  cfg.elevation_grid_ = { Line(30.0f, 0.7f, r, g, b), Line(60.0f, 0.7f, r, g, b) };
  cfg.longitude_grid_ = { Line(0.0f, 0.7f, r, g, b), Line(90.0f, 0.7f, r, g, b) };
  cfg.angular_dist_grid_ = { Line(22.0f, 0.7f, r, g, b) };
  cfg.horizon_ = true;
  cfg.grid_label_ = true;
  cfg.angular_dist_label_ = true;
  MarkerStyleParam zenith;
  zenith.id_ = MarkerRefId::kZenith;
  zenith.enabled_ = true;
  zenith.color_[0] = r;
  zenith.color_[1] = g;
  zenith.color_[2] = b;
  cfg.markers_ = { zenith };
  return cfg;
}

TEST(RenderConsumerPrintMode, AnnotationColourDoesNotReachThePaper) {
  const auto red = SnapshotOnce(MakeAnnotatedConfig(RenderConfig::kPrint, 1.0f, 0.0f, 0.0f));
  const auto green = SnapshotOnce(MakeAnnotatedConfig(RenderConfig::kPrint, 0.0f, 1.0f, 0.0f));
  ASSERT_EQ(red.size(), static_cast<size_t>(kTotalPix) * 3);
  EXPECT_EQ(CountDifferingBytes(red, green), 0u)
      << "an annotation's colour reached the printed page. Under kPrint every annotation is ink and "
         "carries only a coverage — see BlendAnnotation() in src/server/render.cpp. Do not fix this "
         "by giving print its own line palette; that reintroduces exactly the shadow state the "
         "design rejected (doc/print-mode-subtractive-ink.md §5).";

  // The control, again: on the screen operator these two configs must differ, or "the colour did
  // not reach the page" is a statement about a fixture that draws no annotations.
  EXPECT_GT(CountDifferingBytes(SnapshotOnce(MakeAnnotatedConfig(RenderConfig::kScreen, 1.0f, 0.0f, 0.0f)),
                                SnapshotOnce(MakeAnnotatedConfig(RenderConfig::kScreen, 0.0f, 1.0f, 0.0f))),
            0u)
      << "the screen control did not move — no annotation was actually drawn, so the print "
         "assertion above is vacuous";
}

// The other half of AC4: not reading the colour would be worthless if it also meant not being
// visible. Ink on white paper can only darken, so the annotated frame must be strictly darker than
// the unannotated one wherever a line falls — and never brighter anywhere.
TEST(RenderConsumerPrintMode, AnnotationsDarkenWhitePaperAndNeverBrightenIt) {
  RenderConfig plain = MakeConfig(RenderConfig::kPrint);
  for (int j = 0; j < 3; j++) {
    plain.paper_[j] = 1.0f;  // the worst case for visibility: a pure white sheet
  }
  RenderConfig annotated = MakeAnnotatedConfig(RenderConfig::kPrint, 1.0f, 0.0f, 0.0f);
  for (int j = 0; j < 3; j++) {
    annotated.paper_[j] = 1.0f;
  }

  const auto without = SnapshotOnce(plain);
  const auto with = SnapshotOnce(annotated);
  ASSERT_EQ(without.size(), with.size());

  size_t darkened = 0;
  for (size_t i = 0; i < with.size(); ++i) {
    EXPECT_LE(with[i], without[i]) << "byte " << i
                                   << ": an annotation made the paper BRIGHTER. Ink is subtractive; "
                                      "this is the property that lets print drop the per-mode palette.";
    if (with[i] < without[i]) {
      ++darkened;
    }
  }
  EXPECT_GT(darkened, 0u) << "no pixel got darker — the annotations are invisible on white paper, which is the exact "
                             "failure AC4 is about";
}

}  // namespace
}  // namespace lumice
