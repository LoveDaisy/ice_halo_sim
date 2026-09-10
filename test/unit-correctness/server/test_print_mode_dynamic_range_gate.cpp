// The gate that measures why this mode exists: four decades of dynamic range, resolved in 8 bits.
//
// Every other print-mode fixture asserts that the subtractive operator computes what the design
// says it computes. This one asserts the thing the design was commissioned FOR. The user's report
// was not "the formula is wrong", it was "I picked a white background and the halo disappeared";
// the mechanism (doc/print-mode-subtractive-ink.md §1) is that `out = clamp(L * c + background)` is
// monotonically non-decreasing in radiance, so on a white ground every exposure saturates to the
// same white. A mode built to fix that owes a demonstration that it actually resolves a scene the
// additive path cannot, and that demonstration is what lives here.
//
// THE SCENE. One synthetic batch, two rays, weights 1.0 and 1e-4 — a brightest and a dimmest
// feature four orders of magnitude apart, which is the span a real halo display covers between a
// sun dog's core and the faint outer arcs it sits in. Plus a third probe pixel that receives no ray
// at all: the zero-energy colour. Three pixels, and the claim is about how they relate.
//
// WHAT EACH ARM SAYS.
//   - print                    : the three are PAIRWISE DIFFERENT. Both features are readable, and
//                                both are readable as distinct from blank paper. (AC1a)
//   - screen, white background : the three are BYTE-IDENTICAL, all pure white. Not "close" —
//                                identical, because clamp(rgb + 1.0, 0, 1) == 1.0 for every
//                                rgb >= 0 whatever the exposure is. This is the user's complaint,
//                                operationalised. (AC1b)
//   - screen, black background : all three separate, and that is the CONTROL. It establishes that
//                                this scene carries enough range for an 8-bit renderer to resolve
//                                — measured at 7/255 of separation on the faint probe's strongest
//                                channel — so the collapse on a white ground above is attributable
//                                to the ground and not to a fixture that had nothing in it.
//
// SO THE FAILURE THE ADDITIVE OPERATOR HAS IS GROUND-DEPENDENT, AND THAT IS THE POINT, not a
// weakening of it. `clamp(L * c + background)` loses the picture exactly when the background is
// light; the design document's diagnosis (§1) is about that case and claims nothing about black.
// What makes it the case that matters is §6: a printed page IS a light ground. "Use a black
// background instead" is available on a screen and unavailable on paper, which is why the fix had
// to be a second operator rather than advice. (An earlier draft of this file asserted that black
// could not hold four decades either; the measurement above refuted it, and the claim here is the
// measured one.)
//
// Note what the red-state story is. AC1b does NOT ask for a test that fails on screen — it asks for
// evidence the additive path cannot pass this bar, and the evidence is the white-background arm,
// which is a green assertion about a FAILURE to resolve. Both arms pass; read together they are the
// comparison. A separate hand probe (temporarily stubbing InkOpticalDensity to return 0) was run
// while writing this to confirm the print arm really does go red when the operator is removed; that
// is evidence about this file, not about the additive path, and it is not left in the tree as a
// disabled case.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"
#include "support/render_anchor.hpp"
#include "util/color_space.hpp"

namespace lumice {
namespace {

constexpr int kW = 64;
constexpr int kH = 64;
constexpr int kTotalPix = kW * kH;

// The dynamic range under test, as a ratio of deposited weights. Four decades exactly, so the
// assertion below can check the scene really spans what this file claims it spans rather than
// taking the constant's word for it.
constexpr float kFaintWeight = 1e-4f;

// A warm off-white sheet with three DISTINCT components: with {1,1,1} a bug that dropped the
// per-channel paper multiply would still produce the byte patterns this file compares. Deliberately
// a different sheet from test_render_consumer_print_mode.cpp's, so neither fixture can mask a
// paper-handling defect that happens to be benign for one particular colour.
constexpr float kPaper[3]{ 0.88f, 0.93f, 0.80f };

SunParam MakeSun() {
  return SunParam{ 45.0f, 0.0f, 0.5f };
}

// Pointed at the zenith under a 180 deg equal-area fisheye: the probe directions below land well
// inside the image circle, and the frame's corners fall outside it.
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

// Two rays, four decades apart, into two different pixels. The zenith ray lands at the centre; the
// second comes from 60 deg elevation, far enough off-axis at 64x64 that the two cannot share a bin.
//
// Nothing is fired into the rest of the frame, so every other imaged pixel is a zero-energy probe.
SimData MakeBatch() {
  SimData data;
  data.curr_wl_ = 550.0f;
  const float dirs[][3]{ { 0.0f, 0.0f, -1.0f }, { 0.5f, 0.0f, -0.866f } };
  const float weights[]{ 1.0f, kFaintWeight };
  for (size_t i = 0; i < std::size(weights); ++i) {
    for (int j = 0; j < 3; j++) {
      data.outgoing_d_.push_back(dirs[i][j]);
    }
    data.outgoing_w_.push_back(weights[i]);
  }
  return data;
}

struct Frame {
  std::vector<uint8_t> bytes;  // sRGB, 3 per pixel
  std::vector<float> y;        // CIE Y per pixel, straight off the accumulation buffer
};

// One snapshot through the real pixel loop. The raw Y comes back alongside the bytes because the
// scene's four-decade span is a property of the accumulation, not of the output, and has to be
// checked where it lives: reading it back off sRGB bytes would be circular, since whether those
// bytes preserve the span is the whole question.
Frame SnapshotOnce(const RenderConfig& cfg) {
  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  auto data = MakeBatch();
  rc.Consume(data);
  // The anchor a hand-driven RenderConsumer does not get, pushed in by the shared helper rather than
  // re-derived here — see support/render_anchor.hpp for what value it pushes and why that one.
  lumice::test::TakeSnapshotAtFormerSelfAnchor(&rc);

  Frame out;
  // Read after the bake: PostSnapshot leaves snapshot_xyz_ untouched precisely so this stays
  // available (see the fused pixel loop's comment in src/server/render.cpp).
  const RawXyzResult raw = rc.GetRawXyzResult();
  if (raw.xyz_buffer_ != nullptr) {
    out.y.resize(kTotalPix);
    for (int i = 0; i < kTotalPix; ++i) {
      out.y[i] = raw.xyz_buffer_[i * 3 + 1];
    }
  }
  auto result = rc.GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  if (rr != nullptr && rr->img_buffer_ != nullptr) {
    out.bytes.assign(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
  }
  return out;
}

// The paper as the writer leaves it: the same clamp / gamma / narrowing the pixel loop applies to
// every channel, evaluated at transmittance 1.
//
// This RE-IMPLEMENTS the quantisation rule at render.cpp's `LinearToSrgb(...) * 255` narrowing
// (truncation, not rounding), rather than calling it — the rule lives inline in the pixel loop and
// has no callable form. The two must stay in step: change the production narrowing to round and
// this assertion goes quietly wrong instead of failing, because it would be comparing the new
// pixels against the old rule. If that narrowing is ever lifted into a helper, call it here.
std::array<uint8_t, 3> PaperBytes() {
  std::array<uint8_t, 3> out{};
  for (int j = 0; j < 3; j++) {
    out[j] = static_cast<uint8_t>(LinearToSrgb(std::clamp(kPaper[j], 0.0f, 1.0f)) * 255);
  }
  return out;
}

std::array<uint8_t, 3> PixelAt(const std::vector<uint8_t>& img, int i) {
  const size_t base = static_cast<size_t>(i) * 3;
  return { img[base], img[base + 1], img[base + 2] };
}

// The three probe pixels, located by what the SCENE did rather than by recomputing the projection.
// Recomputing it would pin this fixture to the lens code it is not testing; reading the pixels the
// batch actually lit is both shorter and harder to get quietly wrong.
struct Probes {
  int bright = -1;  // the 1.0-weight ray
  int faint = -1;   // the 1e-4-weight ray
  int blank = -1;   // imaged, visible, and unlit
  size_t lit_count = 0;
};

Probes LocateProbes(const Frame& print_frame, const Frame& screen_white) {
  Probes p;
  // Lit pixels come from the accumulation buffer, which is tone-independent: the three arms share
  // one batch, so the same indices are the probes in all of them.
  std::vector<int> lit;
  for (int i = 0; i < kTotalPix; ++i) {
    if (print_frame.y[i] > 0.0f) {
      lit.push_back(i);
    }
  }
  if (lit.size() == 2) {
    const bool first_is_brighter = print_frame.y[lit[0]] > print_frame.y[lit[1]];
    p.bright = first_is_brighter ? lit[0] : lit[1];
    p.faint = first_is_brighter ? lit[1] : lit[0];
  }
  p.lit_count = lit.size();
  // The blank probe has to be inside the image circle AND in the hemisphere `visible` admits,
  // otherwise it is the trivially-unexposed "lens misses it" case and says nothing about dynamic
  // range. The white-background screen arm is the discriminator that proves it: a masked pixel is
  // cleared to black there (the display clip in render.cpp), an admitted one is painted the white
  // sky. So "pure white under a white sky" IS the proof of admittance, with no second projection
  // implementation to keep in step.
  for (int i = 0; i < kTotalPix; ++i) {
    if (i == p.bright || i == p.faint || print_frame.y[i] > 0.0f) {
      continue;
    }
    const auto px = PixelAt(screen_white.bytes, i);
    if (px[0] == 255 && px[1] == 255 && px[2] == 255) {
      p.blank = i;
      break;
    }
  }
  return p;
}

// -----------------------------------------------------------------------------
// The gate itself.
// -----------------------------------------------------------------------------
class PrintModeDynamicRangeGate : public ::testing::Test {
 protected:
  void SetUp() override {
    print_ = SnapshotOnce(MakeConfig(RenderConfig::kPrint));

    RenderConfig white_sky = MakeConfig(RenderConfig::kScreen);
    for (int j = 0; j < 3; j++) {
      white_sky.background_[j] = 1.0f;  // linear white: literally "I picked a white background"
    }
    screen_white_ = SnapshotOnce(white_sky);

    // background_ defaults to black, so the third arm needs no field set — stated rather than
    // relied on, because "the default happens to be the arm I wanted" is how a control quietly
    // stops being the control it is named after.
    RenderConfig black_sky = MakeConfig(RenderConfig::kScreen);
    ASSERT_EQ(black_sky.background_[0], 0.0f);
    ASSERT_EQ(black_sky.background_[1], 0.0f);
    ASSERT_EQ(black_sky.background_[2], 0.0f);
    screen_black_ = SnapshotOnce(black_sky);

    ASSERT_EQ(print_.bytes.size(), static_cast<size_t>(kTotalPix) * 3);
    ASSERT_EQ(screen_white_.bytes.size(), print_.bytes.size());
    ASSERT_EQ(screen_black_.bytes.size(), print_.bytes.size());
    ASSERT_EQ(print_.y.size(), static_cast<size_t>(kTotalPix));

    probes_ = LocateProbes(print_, screen_white_);
    ASSERT_EQ(probes_.lit_count, 2u) << "the batch lit " << probes_.lit_count
                                     << " pixels, not 2 — the two rays either share a bin (no range left to "
                                        "measure) or one spread across several (the span below is no longer "
                                        "the weight ratio)";
    ASSERT_GE(probes_.bright, 0);
    ASSERT_GE(probes_.faint, 0);
    ASSERT_GE(probes_.blank, 0) << "no imaged, visible, unlit pixel was found — the zero-energy probe "
                                   "has nowhere to live and every claim below would be about two pixels";

    // And the scene really does span four decades, measured on the accumulation rather than assumed
    // from the weights: a binning change that spread one ray over several pixels would shrink the
    // span without touching the constant above.
    const float ratio = print_.y[probes_.faint] / print_.y[probes_.bright];
    EXPECT_NEAR(ratio, kFaintWeight, kFaintWeight * 0.5f)
        << "the two probes are " << ratio << " apart, not " << kFaintWeight
        << " — this gate is named for four decades of dynamic range and has to actually contain them";
  }

  Frame print_;
  Frame screen_white_;
  Frame screen_black_;
  Probes probes_;
};

// AC1a — print resolves all three.
TEST_F(PrintModeDynamicRangeGate, PrintResolvesFourDecadesAgainstPaper) {
  const auto bright = PixelAt(print_.bytes, probes_.bright);
  const auto faint = PixelAt(print_.bytes, probes_.faint);
  const auto blank = PixelAt(print_.bytes, probes_.blank);

  EXPECT_NE(bright, faint) << "the brightest and faintest features print as the same bytes: four "
                              "decades collapsed to one ink level";
  EXPECT_NE(bright, blank) << "the brightest feature is indistinguishable from blank paper";
  EXPECT_NE(faint, blank) << "the faintest feature is indistinguishable from blank paper — this is the "
                             "half the additive operator fails, and the reason the mode exists";

  // Direction, not just difference: more light means more ink means a darker sheet. A frame that
  // merely differed in three arbitrary ways would satisfy the inequalities above.
  for (int j = 0; j < 3; j++) {
    EXPECT_LT(bright[j], faint[j]) << "channel " << j << ": the brighter feature laid LESS ink";
    EXPECT_LT(faint[j], blank[j]) << "channel " << j << ": the faint feature laid no ink at all";
  }
  EXPECT_EQ(blank, PaperBytes()) << "the zero-energy colour is not the paper";
}

// AC1b — the additive operator on a white ground resolves none of them. Identical, not merely close.
TEST_F(PrintModeDynamicRangeGate, ScreenOnWhiteBackgroundSaturatesAllThreeToOneWhite) {
  const auto bright = PixelAt(screen_white_.bytes, probes_.bright);
  const auto faint = PixelAt(screen_white_.bytes, probes_.faint);
  const auto blank = PixelAt(screen_white_.bytes, probes_.blank);
  const std::array<uint8_t, 3> white{ 255, 255, 255 };

  EXPECT_EQ(bright, white);
  EXPECT_EQ(faint, white);
  EXPECT_EQ(blank, white) << "all three probes must be pure white: clamp(rgb + 1.0, 0, 1) == 1.0 for "
                             "every rgb >= 0, whatever the exposure. If this is red the additive "
                             "operator has stopped being additive, and doc/print-mode-subtractive-ink.md "
                             "§1's diagnosis — the reason this whole mode was commissioned — no longer "
                             "describes the code.";
  EXPECT_EQ(bright, faint);
  EXPECT_EQ(bright, blank);
}

// The control that makes the white arm mean something: on a BLACK ground the same additive operator,
// the same scene and the same four decades come apart into three distinct bytes. So the collapse
// above is a property of the ground, not of a fixture with nothing in it.
//
// This is the assertion that would go red if this file ever stopped measuring anything — e.g. a
// binning or exposure change that left the faint probe below one 8-bit step. It is therefore the
// load-bearing half of AC1b, even though the headline claim is the white arm's.
TEST_F(PrintModeDynamicRangeGate, ScreenOnBlackBackgroundResolvesAllThreeAsTheControl) {
  const auto bright = PixelAt(screen_black_.bytes, probes_.bright);
  const auto faint = PixelAt(screen_black_.bytes, probes_.faint);
  const auto blank = PixelAt(screen_black_.bytes, probes_.blank);

  EXPECT_NE(bright, blank) << "even on black the bright feature is invisible — this fixture has lost its "
                              "discriminating power, so the white-background arm proves nothing about "
                              "the background and everything about the scene being empty";
  EXPECT_NE(faint, blank) << "the faint probe has fallen below one 8-bit step even on black. Then the "
                             "white arm's 'all three identical' no longer isolates the white ground, "
                             "because this scene would saturate nothing and resolve nothing either "
                             "way. Restore the range (raise kFaintWeight, or widen the probe spacing) "
                             "rather than deleting this line.";
  EXPECT_NE(bright, faint) << "the two features are one byte on black too — see above, the scene has "
                              "stopped spanning a range an 8-bit renderer can show";

  // The zero-energy colour on this side, stated so the asymmetry with the print arm is on the record
  // rather than inferred: screen's blank is black, print's is the paper. Two modes, two grounds.
  EXPECT_EQ(blank, (std::array<uint8_t, 3>{ 0, 0, 0 }));
}

}  // namespace
}  // namespace lumice
