// The status-bar sampling-density readout's two FRAME-dependent cases. Both need a live window:
//
//  1. sampling_density/status_bar_renders_sampling — the readout reaches actual framebuffer pixels
//  2. sampling_density/status_bar_width_budget     — the row still fits at the minimum window width
//
// The six pure formatter cases that used to sit beside them moved to the windowless target,
// test/unit-correctness/gui/test_sampling_density_stats.cpp.

#include <cmath>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "test_gui_shared.hpp"

namespace {

using lumice::gui::FormatSamplingSegment;
using lumice::gui::g_state;

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

}  // namespace

void RegisterSamplingDensityStatsTests(ImGuiTestEngine* engine) {
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
