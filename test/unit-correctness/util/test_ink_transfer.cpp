// The subtractive (density) transfer curve, asserted on the function itself.
//
// This file exists for the same reason test_label_viewport_clamp.cpp does: the rule has call sites
// that are expensive to stand up (the CLI's PostSnapshot needs a consumer and a snapshot; the
// preview shader needs a GL context), and what is left when both are stripped away is one float in
// and one float out. The renderer-level tests assert that each path USES this curve; this file
// asserts what the curve IS.
//
// A third implementation exists that no C++ test can reach — the preview fragment shader's
// `subtractiveInk()` — and it is held to this one by test/gui/functional/test_preview_print_mode.cpp
// comparing rendered pixels against values computed here.

#include <gtest/gtest.h>

#include <cmath>

#include "util/ink_transfer.hpp"

namespace lumice {
namespace {

// Zero exposure has to be the exact identity, not merely something close: it is the algebraic
// basis for "a pixel that received no light is the paper colour" (AC5), which the renderers lean on
// for the masked-out region and for the not-yet-simulated first frame. A curve that returned
// 0.9999 here would tint every empty pixel by a hair and pass any tolerance-based check.
TEST(InkTransfer, ZeroExposureIsTransparent) {
  EXPECT_EQ(InkOpticalDensity(0.0f), 0.0f);
  EXPECT_EQ(InkTransmittance(InkOpticalDensity(0.0f)), 1.0f);
}

// Negative input is clamped, not passed through: log10 of a negative is NaN, and one NaN pixel
// poisons the whole channel through the clamp/gamma chain downstream.
TEST(InkTransfer, NegativeExposureIsClampedToTransparent) {
  EXPECT_EQ(InkOpticalDensity(-1.0f), 0.0f);
  EXPECT_EQ(InkTransmittance(InkOpticalDensity(-1e-6f)), 1.0f);
}

// AC3's "monotonically darker, no hard knee", turned from an eyeball judgement into an executable
// one. log10(1 + e) is smooth over the whole domain and has no branch in it, so what this samples
// is that no one has ADDED a branch (a clamp, a shoulder, a piecewise "soft knee") later: the
// sequence has to be strictly decreasing everywhere, including across the decades where a
// hand-fitted shoulder would usually be spliced in.
TEST(InkTransfer, TransmittanceIsStrictlyDecreasingInExposure) {
  // Deliberately spanning several decades rather than a uniform ramp: the typical non-zero `e` in
  // this project's scenes is 1e-3..1e-1 (doc/print-mode-subtractive-ink.md §3.2), so a uniform
  // sample over [0, 10] would put almost no points where the picture actually lives.
  const float kSamples[] = { 0.0f, 1e-4f, 1e-3f, 3e-3f, 1e-2f, 3e-2f, 0.1f, 0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f };
  float prev = 2.0f;  // above any possible transmittance, so the first comparison is meaningful
  for (float e : kSamples) {
    const float t = InkTransmittance(InkOpticalDensity(e));
    EXPECT_LT(t, prev) << "transmittance did not decrease at e=" << e;
    EXPECT_GT(t, 0.0f) << "transmittance left the (0, 1] range at e=" << e;
    EXPECT_LE(t, 1.0f) << "ink brightened the paper at e=" << e;
    prev = t;
  }
}

// The structural property that makes print a separate mode rather than a parameter range of the
// additive one: ink cannot brighten paper. Asserted over a wide sweep including exposures far past
// anything a real scene produces, because the claim is about the operator, not about the scenes.
TEST(InkTransfer, InkNeverBrightensThePaper) {
  for (float e = 0.0f; e < 1000.0f; e = e * 1.5f + 1e-3f) {
    const float t = InkTransmittance(InkOpticalDensity(e));
    EXPECT_LE(t, 1.0f) << "at e=" << e;
    EXPECT_FALSE(std::isnan(t)) << "at e=" << e;
  }
}

// The formula, spelled out independently of the implementation. Not a tautology: it pins the ORDER
// of operations (gamma multiplies the log, it does not sit inside it) and the base (10, not e),
// which is where a transcription of this curve — the GLSL one especially, since GLSL has no log10
// builtin — is most likely to go wrong.
TEST(InkTransfer, MatchesTheClosedFormOfTheDesignDocument) {
  for (float e : { 1e-3f, 1e-2f, 0.1f, 1.0f, 7.5f }) {
    const double expected_d = 11.0 * std::log10(1.0 + static_cast<double>(e));
    const double expected_t = std::pow(10.0, -expected_d);
    EXPECT_NEAR(InkOpticalDensity(e), static_cast<float>(expected_d), 1e-5) << "at e=" << e;
    EXPECT_NEAR(InkTransmittance(InkOpticalDensity(e)), static_cast<float>(expected_t), 1e-6) << "at e=" << e;
  }
}

// kInkGamma is an owner-accepted calibration (doc/print-mode-subtractive-ink.md §3.2), not a free
// parameter. Pinning it does not make it right — it makes a change to it LOUD, so that whoever
// re-tunes the look of print has to face the fact that they are overruling a decision, rather than
// nudging a constant that nothing appeared to depend on.
//
// The trap this guards against is specific and measured: the intuitive range for a density slope is
// something like [0.3, 2], and every value in it renders these scenes as blank white paper, because
// the exposed scalar is 1e-3..1e-1 rather than the order-1 quantity intuition assumes.
TEST(InkTransfer, GammaIsThePinnedCalibratedValue) {
  EXPECT_EQ(kInkGamma, 11.0f);
}

}  // namespace
}  // namespace lumice
