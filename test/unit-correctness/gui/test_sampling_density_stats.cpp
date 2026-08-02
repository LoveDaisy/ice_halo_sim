// PURE unit tests for the status-bar sampling-density readout: the formatter is called directly
// and its output checked. No GL, no server, no frame — so they run in gui_unit_test, the
// windowless target that links lumice_gui_obj (the functions under test are declared in
// gui/app.hpp, which unit_correctness_test does not link).
//
// Their two frame-dependent siblings stay in test/gui/functional/test_gui_sampling_density_stats.cpp:
// status_bar_renders_sampling (differential framebuffer capture) and status_bar_width_budget
// (ImGui text metrics at the minimum window width).
//
// Tests:
//  1. ratio_ge_one      — the resampled-per-ray branch, including draws == rays
//  2. ratio_lt_one      — the barely-sampled branch and its x10^N notation
//  3. rays_zero         — cold start: no division by zero, no inf/nan text
//  4. draws_zero        — the SECOND division by zero (rays / draws)
//  5. cross_backend_note_present — the tooltip caveat is still in the tooltip
//  6. segment_shows_both_counters — both dimensions reach the segment text
//
// Cases 3 and 4 are not symmetric decoration. Case 4 guards `rays / draws` in the ratio < 1 branch,
// whose "cannot happen" rests on the two-term counter convention in trace_backend.hpp — an
// invariant owned by a different module, which can therefore change without any signal reaching
// this file.

#include <gtest/gtest.h>

#include <string>

#include "gui/app.hpp"

namespace {

using lumice::gui::FormatSamplingDensity;
using lumice::gui::FormatSamplingDensityCompact;
using lumice::gui::FormatSamplingSegment;
using lumice::gui::FormatSamplingTooltip;
using lumice::gui::kSamplingCrossBackendNote;

bool Contains(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

// Any of these appearing in user-facing text means a division went unguarded somewhere. Checked as
// text rather than as a float predicate because the text is what the user would actually see, and
// because snprintf spells them differently across platforms ("inf" / "-nan(ind)").
bool LooksNumericallyBroken(const std::string& s) {
  return Contains(s, "inf") || Contains(s, "nan") || Contains(s, "ind");
}

}  // namespace

// ---- Test 1: ratio >= 1 — this dimension is resampled per ray ----
TEST(SamplingDensity, ratio_ge_one) {
  // Exactly one draw per ray — the overwhelmingly common "random orientation" reading, and the
  // boundary between the two branches. It must land in the >= branch, not the < one.
  EXPECT_STREQ(FormatSamplingDensity(5419520, 5419520).c_str(), "1.00/ray");

  // Above one (multi-scattering layers can draw more than once per ray).
  EXPECT_STREQ(FormatSamplingDensity(200, 100).c_str(), "2.00/ray");

  // Rounding is to two decimals, and does not silently truncate to an integer.
  EXPECT_STREQ(FormatSamplingDensity(150, 100).c_str(), "1.50/ray");

  // The value is a ratio, not a raw count: two runs with the same density but wildly different
  // absolute sizes must read identically. This is the property that makes the number comparable
  // across runs at all.
  EXPECT_STREQ(FormatSamplingDensity(3, 2).c_str(), FormatSamplingDensity(3000000, 2000000).c_str());
}

// ---- Test 2: ratio < 1 — this dimension is barely sampled ----
TEST(SamplingDensity, ratio_lt_one) {
  // The measured "fixed shape + random orientation" case: 1 crystal over 5,419,520 rays. This is
  // the reading the whole feature exists to make visible, so it is pinned literally.
  EXPECT_STREQ(FormatSamplingDensity(1, 5419520).c_str(), "1 per 5.4 x10^6 rays");

  // x10^N notation matches the "Total rays" segment beside it (app_panels.cpp), across decades.
  EXPECT_STREQ(FormatSamplingDensity(1, 2000000000ULL).c_str(), "1 per 2.0 x10^9 rays");
  EXPECT_STREQ(FormatSamplingDensity(1, 50000).c_str(), "1 per 50.0 x10^3 rays");

  // Below 1e3 the scaled form would read "1 per 0.5 x10^3 rays"; print the plain integer instead.
  EXPECT_STREQ(FormatSamplingDensity(1, 500).c_str(), "1 per 500 rays");

  // The GPU route's one-geometry-per-batch reading — an order of magnitude below the CPU route's
  // for the same scene, and specifically NOT an error state. It must format normally.
  EXPECT_STREQ(FormatSamplingDensity(5, 5000000).c_str(), "1 per 1.0 x10^6 rays");

  // The compact spelling differs from the verbose one by the trailing word and nothing else --
  // the status bar and the tooltip must never disagree about the number itself.
  EXPECT_STREQ(FormatSamplingDensityCompact(1, 5419520).c_str(), "1 per 5.4 x10^6");
  EXPECT_STREQ((FormatSamplingDensityCompact(1, 5419520) + " rays").c_str(), FormatSamplingDensity(1, 5419520).c_str());
  // The branches with no trailing word are spelled identically by both.
  EXPECT_STREQ(FormatSamplingDensityCompact(100, 100).c_str(), FormatSamplingDensity(100, 100).c_str());
  EXPECT_STREQ(FormatSamplingDensityCompact(0, 100).c_str(), FormatSamplingDensity(0, 100).c_str());
}

// ---- Test 3: rays == 0 (cold start) ----
TEST(SamplingDensity, rays_zero) {
  const std::string zero_rays = FormatSamplingDensity(42, 0);
  EXPECT_STREQ(zero_rays.c_str(), "n/a");
  EXPECT_TRUE(!LooksNumericallyBroken(zero_rays));

  // Both counters zero: still no division, still no inf/nan.
  const std::string all_zero = FormatSamplingDensity(0, 0);
  EXPECT_STREQ(all_zero.c_str(), "n/a");
  EXPECT_TRUE(!LooksNumericallyBroken(all_zero));

  // The whole assembled segment and tooltip stay clean too — a guard in the formatter is worth
  // nothing if the strings built around it reintroduce the artifact.
  const std::string segment = FormatSamplingSegment(0, 0, 0);
  EXPECT_TRUE(!LooksNumericallyBroken(segment));
  const std::string tooltip = FormatSamplingTooltip(0, 0, 0);
  EXPECT_TRUE(!LooksNumericallyBroken(tooltip));
}

// ---- Test 4: draws == 0 with rays > 0 (the second division by zero) ----
TEST(SamplingDensity, draws_zero) {
  // Reaching the ratio < 1 branch with draws == 0 would evaluate rays / draws.
  const std::string no_draws = FormatSamplingDensity(0, 5419520);
  EXPECT_STREQ(no_draws.c_str(), "n/a");
  EXPECT_TRUE(!LooksNumericallyBroken(no_draws));

  // A run where one dimension reports draws and the other does not: the reporting dimension must
  // still format normally rather than the whole segment collapsing to a placeholder.
  const std::string segment = FormatSamplingSegment(0, 1000, 1000);
  EXPECT_TRUE(!LooksNumericallyBroken(segment));
  EXPECT_TRUE(Contains(segment, "n/a"));
  EXPECT_TRUE(Contains(segment, "1.00/ray"));
}

// ---- Test 5: the cross-backend caveat survives in the tooltip ----
TEST(SamplingDensity, cross_backend_note_present) {
  // This is the assertion that gives the caveat a failure signal. Without it the sentence is
  // ordinary tooltip prose, and prose disappears in refactors without anything going red — which
  // is the failure mode that produces false "the GPU backend regressed" bug reports.
  EXPECT_TRUE(Contains(FormatSamplingTooltip(1, 5419520, 5419520), kSamplingCrossBackendNote));

  // Present regardless of which branch the densities take, including the degenerate run.
  EXPECT_TRUE(Contains(FormatSamplingTooltip(5000000, 5000000, 5000000), kSamplingCrossBackendNote));
  EXPECT_TRUE(Contains(FormatSamplingTooltip(0, 0, 0), kSamplingCrossBackendNote));

  // The constant must actually say the thing, so that a future edit blanking it out (which would
  // keep the substring check trivially true) does not pass.
  const std::string note = kSamplingCrossBackendNote;
  EXPECT_GT(note.size(), 40u);
  EXPECT_TRUE(Contains(note, "backend"));
}

// ---- Test 6: both counters reach the segment, with the raw counts in the tooltip ----
TEST(SamplingDensity, segment_shows_both_counters) {
  // The six-orders-of-magnitude spread between the two counters IS the signal (a fixed shape next
  // to a per-ray orientation draw). Pinning both readings in one string is what proves the
  // segment does not quietly show one dimension twice.
  const std::string segment = FormatSamplingSegment(1, 5419520, 5419520);
  // The status bar carries the compact spelling (no trailing " rays"); the tooltip carries the
  // verbose one. Both are pinned so a change to either has to be deliberate.
  EXPECT_TRUE(Contains(segment, "shape 1 per 5.4 x10^6 \xC2\xB7"));
  EXPECT_TRUE(!Contains(segment, "rays"));
  EXPECT_TRUE(Contains(segment, "orient 1.00/ray"));

  // Raw counts belong to the tooltip, grouped, and locale-independently so a run on another
  // machine formats identically.
  const std::string tooltip = FormatSamplingTooltip(1, 5419520, 5419520);
  EXPECT_TRUE(Contains(tooltip, "5,419,520"));

  // AC6: ray-segment counts are deliberately absent from this readout — it answers "is sampling
  // dense enough", which ray_seg_num does not speak to. Nothing here should mention segments.
  EXPECT_TRUE(!Contains(segment, "seg"));
  EXPECT_TRUE(!Contains(tooltip, "Ray segments"));
}
