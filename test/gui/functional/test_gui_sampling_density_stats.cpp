// PURE unit tests for the status-bar sampling-density readout. No GL, no server — peer to
// test_gui_state_reconcile.cpp: the TestFunc calls the formatter directly and checks its output.
//
// Why these live in gui_test rather than the ctest unit binary: the functions under test are
// GUI-local (declared in gui/app.hpp), and src/gui/ is not linked into unit_correctness_test.
//
// Tests:
//  1. sampling_density/ratio_ge_one      — the resampled-per-ray branch, including draws == rays
//  2. sampling_density/ratio_lt_one      — the barely-sampled branch and its x10^N notation
//  3. sampling_density/rays_zero         — cold start: no division by zero, no inf/nan text
//  4. sampling_density/draws_zero        — the SECOND division by zero (rays / draws)
//  5. sampling_density/cross_backend_note_present — the tooltip caveat is still in the tooltip
//  6. sampling_density/segment_shows_both_counters — both dimensions reach the segment text
//
// Cases 3 and 4 are not symmetric decoration. Case 4 guards `rays / draws` in the ratio < 1 branch,
// whose "cannot happen" rests on the two-term counter convention in trace_backend.hpp — an
// invariant owned by a different module, which can therefore change without any signal reaching
// this file.

#include <string>

#include "gui/app.hpp"
#include "test_gui_shared.hpp"

namespace {

using lumice::gui::FormatSamplingDensity;
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

void RegisterSamplingDensityStatsTests(ImGuiTestEngine* engine) {
  // ---- Test 1: ratio >= 1 — this dimension is resampled per ray ----
  ImGuiTest* t1 = IM_REGISTER_TEST(engine, "sampling_density", "ratio_ge_one");
  t1->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // Exactly one draw per ray — the overwhelmingly common "random orientation" reading, and the
    // boundary between the two branches. It must land in the >= branch, not the < one.
    IM_CHECK_STR_EQ(FormatSamplingDensity(5419520, 5419520).c_str(), "1.00/ray");

    // Above one (multi-scattering layers can draw more than once per ray).
    IM_CHECK_STR_EQ(FormatSamplingDensity(200, 100).c_str(), "2.00/ray");

    // Rounding is to two decimals, and does not silently truncate to an integer.
    IM_CHECK_STR_EQ(FormatSamplingDensity(150, 100).c_str(), "1.50/ray");

    // The value is a ratio, not a raw count: two runs with the same density but wildly different
    // absolute sizes must read identically. This is the property that makes the number comparable
    // across runs at all.
    IM_CHECK_STR_EQ(FormatSamplingDensity(3, 2).c_str(), FormatSamplingDensity(3000000, 2000000).c_str());
  };

  // ---- Test 2: ratio < 1 — this dimension is barely sampled ----
  ImGuiTest* t2 = IM_REGISTER_TEST(engine, "sampling_density", "ratio_lt_one");
  t2->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // The measured "fixed shape + random orientation" case: 1 crystal over 5,419,520 rays. This is
    // the reading the whole feature exists to make visible, so it is pinned literally.
    IM_CHECK_STR_EQ(FormatSamplingDensity(1, 5419520).c_str(), "1 per 5.4 x10^6 rays");

    // x10^N notation matches the "Total rays" segment beside it (app_panels.cpp), across decades.
    IM_CHECK_STR_EQ(FormatSamplingDensity(1, 2000000000ULL).c_str(), "1 per 2.0 x10^9 rays");
    IM_CHECK_STR_EQ(FormatSamplingDensity(1, 50000).c_str(), "1 per 50.0 x10^3 rays");

    // Below 1e3 the scaled form would read "1 per 0.5 x10^3 rays"; print the plain integer instead.
    IM_CHECK_STR_EQ(FormatSamplingDensity(1, 500).c_str(), "1 per 500 rays");

    // The GPU route's one-geometry-per-batch reading — an order of magnitude below the CPU route's
    // for the same scene, and specifically NOT an error state. It must format normally.
    IM_CHECK_STR_EQ(FormatSamplingDensity(5, 5000000).c_str(), "1 per 1.0 x10^6 rays");
  };

  // ---- Test 3: rays == 0 (cold start) ----
  ImGuiTest* t3 = IM_REGISTER_TEST(engine, "sampling_density", "rays_zero");
  t3->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    const std::string zero_rays = FormatSamplingDensity(42, 0);
    IM_CHECK_STR_EQ(zero_rays.c_str(), "n/a");
    IM_CHECK(!LooksNumericallyBroken(zero_rays));

    // Both counters zero: still no division, still no inf/nan.
    const std::string all_zero = FormatSamplingDensity(0, 0);
    IM_CHECK_STR_EQ(all_zero.c_str(), "n/a");
    IM_CHECK(!LooksNumericallyBroken(all_zero));

    // The whole assembled segment and tooltip stay clean too — a guard in the formatter is worth
    // nothing if the strings built around it reintroduce the artifact.
    const std::string segment = FormatSamplingSegment(0, 0, 0);
    IM_CHECK(!LooksNumericallyBroken(segment));
    const std::string tooltip = FormatSamplingTooltip(0, 0, 0);
    IM_CHECK(!LooksNumericallyBroken(tooltip));
  };

  // ---- Test 4: draws == 0 with rays > 0 (the second division by zero) ----
  ImGuiTest* t4 = IM_REGISTER_TEST(engine, "sampling_density", "draws_zero");
  t4->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // Reaching the ratio < 1 branch with draws == 0 would evaluate rays / draws.
    const std::string no_draws = FormatSamplingDensity(0, 5419520);
    IM_CHECK_STR_EQ(no_draws.c_str(), "n/a");
    IM_CHECK(!LooksNumericallyBroken(no_draws));

    // A run where one dimension reports draws and the other does not: the reporting dimension must
    // still format normally rather than the whole segment collapsing to a placeholder.
    const std::string segment = FormatSamplingSegment(0, 1000, 1000);
    IM_CHECK(!LooksNumericallyBroken(segment));
    IM_CHECK(Contains(segment, "n/a"));
    IM_CHECK(Contains(segment, "1.00/ray"));
  };

  // ---- Test 5: the cross-backend caveat survives in the tooltip ----
  ImGuiTest* t5 = IM_REGISTER_TEST(engine, "sampling_density", "cross_backend_note_present");
  t5->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // This is the assertion that gives the caveat a failure signal. Without it the sentence is
    // ordinary tooltip prose, and prose disappears in refactors without anything going red — which
    // is the failure mode that produces false "the GPU backend regressed" bug reports.
    IM_CHECK(Contains(FormatSamplingTooltip(1, 5419520, 5419520), kSamplingCrossBackendNote));

    // Present regardless of which branch the densities take, including the degenerate run.
    IM_CHECK(Contains(FormatSamplingTooltip(5000000, 5000000, 5000000), kSamplingCrossBackendNote));
    IM_CHECK(Contains(FormatSamplingTooltip(0, 0, 0), kSamplingCrossBackendNote));

    // The constant must actually say the thing, so that a future edit blanking it out (which would
    // keep the substring check trivially true) does not pass.
    const std::string note = kSamplingCrossBackendNote;
    IM_CHECK_GT(note.size(), 40u);
    IM_CHECK(Contains(note, "backend"));
  };

  // ---- Test 6: both counters reach the segment, with the raw counts in the tooltip ----
  ImGuiTest* t6 = IM_REGISTER_TEST(engine, "sampling_density", "segment_shows_both_counters");
  t6->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // The six-orders-of-magnitude spread between the two counters IS the signal (a fixed shape next
    // to a per-ray orientation draw). Pinning both readings in one string is what proves the
    // segment does not quietly show one dimension twice.
    const std::string segment = FormatSamplingSegment(1, 5419520, 5419520);
    IM_CHECK(Contains(segment, "shape 1 per 5.4 x10^6 rays"));
    IM_CHECK(Contains(segment, "orient 1.00/ray"));

    // Raw counts belong to the tooltip, grouped, and locale-independently so a run on another
    // machine formats identically.
    const std::string tooltip = FormatSamplingTooltip(1, 5419520, 5419520);
    IM_CHECK(Contains(tooltip, "5,419,520"));

    // AC6: ray-segment counts are deliberately absent from this readout — it answers "is sampling
    // dense enough", which ray_seg_num does not speak to. Nothing here should mention segments.
    IM_CHECK(!Contains(segment, "seg"));
    IM_CHECK(!Contains(tooltip, "Ray segments"));
  };
}
