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
//  7. sampling_density/status_bar_renders_sampling — the readout reaches actual framebuffer pixels
//  8. sampling_density/status_bar_width_budget     — the row still fits at the minimum window width
//
// Tests 7 and 8 need a live frame; 1-6 are pure and touch neither GL nor the server.
//
// Cases 3 and 4 are not symmetric decoration. Case 4 guards `rays / draws` in the ratio < 1 branch,
// whose "cannot happen" rests on the two-term counter convention in trace_backend.hpp — an
// invariant owned by a different module, which can therefore change without any signal reaching
// this file.

#include <cmath>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "test_gui_shared.hpp"

namespace {

using lumice::gui::FormatSamplingDensity;
using lumice::gui::FormatSamplingDensityCompact;
using lumice::gui::FormatSamplingSegment;
using lumice::gui::FormatSamplingTooltip;
using lumice::gui::g_state;
using lumice::gui::kSamplingCrossBackendNote;

// Capture the live rectangle of the status-bar window out of the default framebuffer. Mirrors the
// coordinate conversion in the modal_layout suite: ImGui is origin-top-left in window coordinates,
// glReadPixels is origin-bottom-left in framebuffer pixels.
bool CaptureStatusBar(ImGuiTestContext* ctx, std::vector<unsigned char>* out) {
  ImGuiWindow* win = ctx->GetWindowByRef("##StatusBar");
  if (win == nullptr) {
    return false;
  }
  const ImGuiIO& io = ImGui::GetIO();
  const float win_h = io.DisplaySize.y;
  const float sx = io.DisplayFramebufferScale.x;
  const float sy = io.DisplayFramebufferScale.y;
  const ImVec2 vp_pos = ImGui::GetMainViewport()->Pos;
  const float lx = win->Pos.x - vp_pos.x;
  const float ly = win->Pos.y - vp_pos.y;

  g_fullframe_capture.Reset();
  g_fullframe_capture.rect_x = static_cast<int>(std::lround(lx * sx));
  g_fullframe_capture.rect_y = static_cast<int>(std::lround((win_h - (ly + win->Size.y)) * sy));
  g_fullframe_capture.rect_w = static_cast<int>(std::lround(win->Size.x * sx));
  g_fullframe_capture.rect_h = static_cast<int>(std::lround(win->Size.y * sy));
  g_fullframe_capture.requested.store(true);
  for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
    ctx->Yield(1);
  }
  if (!g_fullframe_capture.done.load()) {
    return false;
  }
  *out = g_fullframe_capture.pixels;
  return !out->empty();
}

// Inject a poll result directly. SyncFromPoller only writes these fields when a snapshot's epoch
// matches committed_epoch and its ray count is non-zero, and no run happens in this suite, so the
// injected values survive across frames.
void InjectStats(unsigned long long crystals, unsigned long long orientations, unsigned long long rays) {
  g_state.stats_crystal_num = crystals;
  g_state.stats_orientation_num = orientations;
  g_state.stats_sim_ray_num = rays;
}

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

    // The compact spelling differs from the verbose one by the trailing word and nothing else --
    // the status bar and the tooltip must never disagree about the number itself.
    IM_CHECK_STR_EQ(FormatSamplingDensityCompact(1, 5419520).c_str(), "1 per 5.4 x10^6");
    IM_CHECK_STR_EQ((FormatSamplingDensityCompact(1, 5419520) + " rays").c_str(),
                    FormatSamplingDensity(1, 5419520).c_str());
    // The branches with no trailing word are spelled identically by both.
    IM_CHECK_STR_EQ(FormatSamplingDensityCompact(100, 100).c_str(), FormatSamplingDensity(100, 100).c_str());
    IM_CHECK_STR_EQ(FormatSamplingDensityCompact(0, 100).c_str(), FormatSamplingDensity(0, 100).c_str());
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
    // The status bar carries the compact spelling (no trailing " rays"); the tooltip carries the
    // verbose one. Both are pinned so a change to either has to be deliberate.
    IM_CHECK(Contains(segment, "shape 1 per 5.4 x10^6 \xC2\xB7"));
    IM_CHECK(!Contains(segment, "rays"));
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

  // ---- Test 7: the readout reaches actual pixels, driven by the two new fields ----
  //
  // Why a differential capture rather than reading the item's text back: ImGui::TextUnformatted
  // submits its item with ID 0, so imgui_test_engine's ItemInfo (which is keyed by ID) cannot
  // address it. Holding stats_sim_ray_num fixed while varying only the two sampling counters makes
  // the "Total rays" segment byte-identical between the two frames, so ANY pixel difference in the
  // status bar has to have come from the sampling segment. That is a stronger statement than a
  // string comparison would be: it proves delivery all the way to the framebuffer, and it fails if
  // the segment is never rendered, is gated out, or reads the wrong field.
  ImGuiTest* t7 = IM_REGISTER_TEST(engine, "sampling_density", "status_bar_renders_sampling");
  t7->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);
    // A hovered status bar would pop the tooltip and contaminate the comparison.
    ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
    ctx->Yield(2);

    constexpr unsigned long long kRays = 5419520;

    // Frame A: fixed shape + per-ray orientation (the common configuration).
    InjectStats(1, kRays, kRays);
    ctx->Yield(3);
    std::vector<unsigned char> frame_a;
    IM_CHECK(CaptureStatusBar(ctx, &frame_a));

    // A capture that silently read back zeros would make every comparison below vacuous.
    bool has_nonzero = false;
    for (size_t i = 0; i < frame_a.size() && !has_nonzero; ++i) {
      has_nonzero = frame_a[i] != 0;
    }
    IM_CHECK(has_nonzero);

    // Frame B: same ray count, different sampling counters -> a different sampling segment
    // ("1 per 5.4 x10^6 rays" becomes "1.00/ray"), and an identical "Total rays" segment.
    IM_CHECK_STR_NE(FormatSamplingSegment(1, kRays, kRays).c_str(), FormatSamplingSegment(kRays, 1, kRays).c_str());
    InjectStats(kRays, 1, kRays);
    ctx->Yield(3);
    std::vector<unsigned char> frame_b;
    IM_CHECK(CaptureStatusBar(ctx, &frame_b));
    IM_CHECK_EQ(frame_a.size(), frame_b.size());
    IM_CHECK(frame_a != frame_b);

    // Frame C: back to frame A's values. Equality here is what makes the inequality above mean
    // something -- without it, frame_a != frame_b could just be capture noise.
    InjectStats(1, kRays, kRays);
    ctx->Yield(3);
    std::vector<unsigned char> frame_c;
    IM_CHECK(CaptureStatusBar(ctx, &frame_c));
    IM_CHECK(frame_a == frame_c);

    InjectStats(0, 0, 0);
  };

  // ---- Test 8: the status bar's left cluster still fits beside the Log button (AC7) ----
  //
  // The concrete failure this guards: ImGui does not ellipsize a SameLine run. An over-long left
  // cluster simply overflows, and the right-aligned Log button (positioned at WindowWidth - log_w -
  // WindowPadding.x) gets overlapped and pushed out of reach. Measuring text extents is what makes
  // this checkable without resizing a real window.
  //
  // The floor is kMinWindowWidth, which main.cpp hands to glfwSetWindowSizeLimits: the user cannot
  // drag the window narrower than that, so it is the actual narrowest case rather than a number
  // chosen here. The one budget figure this test does have to declare is the file name allowance,
  // because the file name is unbounded and can overflow this row with or without a sampling
  // readout -- a pre-existing property of the status bar, not something this feature introduces.
  ImGuiTest* t8 = IM_REGISTER_TEST(engine, "sampling_density", "status_bar_width_budget");
  t8->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);

    const ImGuiStyle& style = ImGui::GetStyle();
    const float spacing = style.ItemSpacing.x;

    // Worst case on every bounded segment simultaneously: the longest status word, a ten-digit ray
    // count, the longest lens name, and both sampling counters in their widest (barely-sampled)
    // form -- which is a real configuration, not a contrived one: a scene with a fixed shape AND a
    // fixed orientation reads "1 per N" on both.
    const std::string status_word = "Simulating...";
    const std::string total_rays = "| Total rays: 9.9 x10^9";
    const std::string sampling = FormatSamplingSegment(1, 1, 9900000000ULL);
    const std::string res_lens = "| 1024x512  Dual Fisheye  FOV:180";
    const std::string log_label = ICON_FA_CHEVRON_RIGHT " Log";

    // Both-barely-sampled really is the widest the segment gets, so the budget below is not
    // measuring an accidentally narrow case.
    const float sampling_w = ImGui::CalcTextSize(sampling.c_str()).x;
    IM_CHECK_GE(sampling_w, ImGui::CalcTextSize(FormatSamplingSegment(1, 1, 1).c_str()).x);
    IM_CHECK_GE(sampling_w, ImGui::CalcTextSize(FormatSamplingSegment(0, 0, 0).c_str()).x);

    // Declared allowance for the (unbounded) file name segment. "scene.lmc *" is the shape of the
    // repo's own example config plus the dirty marker.
    const float filename_allowance = ImGui::CalcTextSize("| scene.lmc *").x;

    const float bounded = style.WindowPadding.x + ImGui::CalcTextSize(status_word.c_str()).x + spacing +
                          ImGui::CalcTextSize(total_rays.c_str()).x + spacing + sampling_w + spacing +
                          ImGui::CalcTextSize(res_lens.c_str()).x + spacing;
    const float log_button = ImGui::CalcTextSize(log_label.c_str()).x + style.FramePadding.x * 2;
    const float required = bounded + filename_allowance + spacing + log_button + style.WindowPadding.x;
    const float floor_width = static_cast<float>(lumice::gui::kMinWindowWidth);

    fprintf(stderr,
            "[sampling_density] sampling=%.1f bounded=%.1f filename_allowance=%.1f log=%.1f required=%.1f "
            "min_window=%.1f headroom=%.1f\n",
            sampling_w, bounded, filename_allowance, log_button, required, floor_width, floor_width - required);

    IM_CHECK_LE(required, floor_width);

    // The sampling segment must not become the row's dominant consumer. Without this the check
    // above could be satisfied by shrinking the file name allowance to nothing.
    IM_CHECK_LT(sampling_w, ImGui::CalcTextSize(res_lens.c_str()).x + ImGui::CalcTextSize(total_rays.c_str()).x);
  };
}
