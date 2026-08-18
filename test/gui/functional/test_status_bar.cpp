// The status bar says what the app is doing. These cases falsify that it says it in pixels.
//
// Every segment of this row is an ImGui::Text* call, and ImGui submits those with item ID 0 — the
// test engine addresses items by ID, so not one of them can be read back through ItemInfo. The
// only way to assert that a segment reached the user is to read the framebuffer, which is what
// makes these B1 (a real frame is required) rather than string comparisons against the same
// formatter the production code called.
//
// The shape used throughout is a differential capture: hold everything else fixed, move ONE field,
// and require the captured rectangle to change — then move it back and require the original
// capture to reproduce. The second half is what makes the first half mean something; without it a
// difference could be capture noise. It is also a stronger statement than a string comparison: it
// fails if the segment is never rendered, is gated out, or reads the wrong field.
//
// The formatters themselves (FormatSamplingSegment and friends) are pure and are covered without a
// window in test/unit-correctness/gui/. Nothing here re-asserts what they return.

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

using Pixels = std::vector<unsigned char>;

// Capture the live rectangle of the status-bar window out of the default framebuffer. The
// coordinate flip and the Retina scaling live in CaptureWindowRect (test_gui_shared.hpp), which is
// where a new capture site should go rather than open-coding the arithmetic again.
bool CaptureStatusBar(ImGuiTestContext* ctx, Pixels* out) {
  int w = 0;
  int h = 0;
  return CaptureWindowRect(ctx, "##StatusBar", out, &w, &h) && !out->empty();
}

// Settle the frame, then capture. A hovered status bar would pop a tooltip over the rectangle and
// contaminate every comparison, so the cursor is parked off-window first.
Pixels SettleAndCapture(ImGuiTestContext* ctx) {
  ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
  ctx->Yield(3);
  Pixels out;
  CaptureStatusBar(ctx, &out);
  return out;
}

// Inject a poll result directly. SyncFromPoller only writes these fields when a snapshot's epoch
// matches committed_epoch and its ray count is non-zero, and no run happens in this suite, so the
// injected values survive across frames.
void InjectStats(unsigned long long crystals, unsigned long long orientations, unsigned long long rays) {
  g_state.stats_crystal_num = crystals;
  g_state.stats_orientation_num = orientations;
  g_state.stats_sim_ray_num = rays;
}

bool AnyNonZero(const Pixels& p) {
  for (unsigned char v : p) {
    if (v != 0) {
      return true;
    }
  }
  return false;
}

// One row of a differential table: a name to report and the state it puts the bar into.
struct Variant {
  const char* label;
  void (*apply)();
};

// Capture every variant, then report which ones failed to be distinguishable. The comparison
// happens AFTER the loop on purpose: IM_CHECK* expands to `return` in the enclosing function, so a
// fatal assert inside the loop would hide every row after the first failure — the exact defect
// scripts/check_loop_fatal_asserts.py exists to stop. Collecting first costs one vector and buys
// a full table evaluation on every run.
void CheckVariantsAreDistinct(ImGuiTestContext* ctx, const Variant* variants, int count) {
  std::vector<Pixels> shots;
  shots.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    variants[i].apply();
    shots.push_back(SettleAndCapture(ctx));
  }

  // Control: re-applying the first variant must reproduce its capture. Without this, any
  // inequality below could be noise from the capture path rather than from the state change.
  variants[0].apply();
  const Pixels control = SettleAndCapture(ctx);

  std::string empty_rows;
  std::string collisions;
  for (int i = 0; i < count; ++i) {
    if (shots[i].empty() || !AnyNonZero(shots[i])) {
      empty_rows += std::string(" ") + variants[i].label;
      continue;
    }
    for (int j = i + 1; j < count; ++j) {
      if (!shots[j].empty() && shots[i] == shots[j]) {
        collisions += std::string(" ") + variants[i].label + "==" + variants[j].label;
      }
    }
  }

  ctx->LogInfo("empty:[%s] collisions:[%s]", empty_rows.c_str(), collisions.c_str());
  IM_CHECK_STR_EQ(empty_rows.c_str(), "");
  IM_CHECK_STR_EQ(collisions.c_str(), "");
  IM_CHECK(control == shots[0]);
}

}  // namespace

void RegisterStatusBarTests(ImGuiTestEngine* engine) {
  // The simulation states are told apart on screen (P40). They differ in both word and colour, and
  // a user who cannot tell one from the next cannot tell whether what they just pressed did
  // anything.
  //
  // Each row sets the state's INPUTS, never sim_state itself. sim_state has a single owner —
  // SyncFromPoller assigns it from ReconcileSimState(run_intent, committed_epoch, snapshot, dirty)
  // on every tick (src/gui/app.cpp) — so a direct write to the field does not survive to the frame
  // that would draw it, and a case written that way silently captures identical bars for every
  // state. Driving the intent is also the honest statement of the dependency: what the bar reports
  // is a function of the run intent, not a field anyone may set.
  //
  // "Stopping..." is deliberately NOT one of the rows. It is the one state with no static input
  // combination: SyncFromPoller promotes a kStopping intent to kStopped the moment no stop is
  // actually in flight, and kStopped resolves to kDone — so with no live server the word is
  // unreachable by construction, and a row for it here would either fail or (worse) be "fixed" by
  // asserting it looks like Done. It is observed where it exists, in the run-lifecycle suite, with
  // a real async Stop draining.
  ImGuiTest* t_state = IM_REGISTER_TEST(engine, "status_bar", "sim_states_are_distinguishable");
  t_state->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);
    static const Variant kStates[] = {
      { "Idle",
        [] {
          g_state.run_intent = gui::RunIntent::kNone;
          g_state.dirty = false;
        } },
      { "Simulating", [] { g_state.run_intent = gui::RunIntent::kRunning; } },
      { "Done",
        [] {
          g_state.run_intent = gui::RunIntent::kLoaded;
          g_state.dirty = false;
        } },
      { "Modified",
        [] {
          g_state.run_intent = gui::RunIntent::kLoaded;
          g_state.dirty = true;
        } },
    };
    CheckVariantsAreDistinct(ctx, kStates, IM_ARRAYSIZE(kStates));
    g_state.run_intent = gui::RunIntent::kNone;
    g_state.dirty = false;
  };

  // The ray-count segment switches unit at each magnitude (P42), and disappears entirely — taking
  // the sampling readout with it — when no run has happened (P41). The unit switch is written
  // inline in RenderStatusBar rather than in a pure formatter, so pixels are the only place it can
  // be observed at all.
  ImGuiTest* t_rays = IM_REGISTER_TEST(engine, "status_bar", "ray_count_gate_and_magnitudes");
  t_rays->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);
    static const Variant kMagnitudes[] = {
      { "no-run", [] { InjectStats(0, 0, 0); } },
      { "thousands", [] { InjectStats(1, 5000, 5000); } },
      { "millions", [] { InjectStats(1, 5000000, 5000000); } },
      { "billions", [] { InjectStats(1, 5000000000ULL, 5000000000ULL); } },
    };
    CheckVariantsAreDistinct(ctx, kMagnitudes, IM_ARRAYSIZE(kMagnitudes));
    InjectStats(0, 0, 0);
  };

  // The sampling-density readout is driven by the two sampling counters and by nothing else (P43).
  //
  // Holding stats_sim_ray_num FIXED while varying only the two counters is what makes this precise:
  // the "Total rays" segment is byte-identical across the two frames, so any pixel difference has
  // to have come from the sampling segment. The readout is deliberately shown even when a dimension
  // is fixed — "1 per 5.4 x10^6 rays" IS the explanation for an over-sharp render, so suppressing
  // it would hide the answer in the case the user most needs it.
  ImGuiTest* t_sampling = IM_REGISTER_TEST(engine, "status_bar", "sampling_segment_reaches_pixels");
  t_sampling->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);
    constexpr unsigned long long kRays = 5419520;
    IM_CHECK_STR_NE(FormatSamplingSegment(1, kRays, kRays).c_str(), FormatSamplingSegment(kRays, 1, kRays).c_str());
    static const Variant kSampling[] = {
      { "per-ray-orientation", [] { InjectStats(1, 5419520, 5419520); } },
      { "per-ray-shape", [] { InjectStats(5419520, 1, 5419520); } },
      { "both-fixed", [] { InjectStats(1, 1, 5419520); } },
    };
    CheckVariantsAreDistinct(ctx, kSampling, IM_ARRAYSIZE(kSampling));
    InjectStats(0, 0, 0);
  };

  // The resolution / lens / FOV segment follows the renderer (P44), and the file-name segment tells
  // "no file" from "saved" from "unsaved edits" (P45). Both are single Text calls whose content is
  // built inline, so both are pixels-or-nothing.
  ImGuiTest* t_scene = IM_REGISTER_TEST(engine, "status_bar", "renderer_and_file_segments_follow_state");
  t_scene->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);
    // Every row states the WHOLE of what it varies rather than layering onto the row above: a
    // cumulative table cannot say which field a collision came from, and a row whose one change
    // happens to match the value already running reads as a passing row that tested nothing.
    static const Variant kScene[] = {
      { "lens0-fov60-nofile",
        [] {
          g_state.renderer.lens_type = 0;
          g_state.renderer.fov = 60.0f;
          g_state.current_file_path.clear();
          g_state.dirty = false;
        } },
      { "lens0-fov120-nofile",
        [] {
          g_state.renderer.lens_type = 0;
          g_state.renderer.fov = 120.0f;
          g_state.current_file_path.clear();
          g_state.dirty = false;
        } },
      { "lens1-fov60-nofile",
        [] {
          g_state.renderer.lens_type = 1;
          g_state.renderer.fov = 60.0f;
          g_state.current_file_path.clear();
          g_state.dirty = false;
        } },
      { "lens0-fov60-named-clean",
        [] {
          g_state.renderer.lens_type = 0;
          g_state.renderer.fov = 60.0f;
          g_state.current_file_path = "scene.lmc";
          g_state.dirty = false;
        } },
      { "lens0-fov60-named-dirty",
        [] {
          g_state.renderer.lens_type = 0;
          g_state.renderer.fov = 60.0f;
          g_state.current_file_path = "scene.lmc";
          g_state.dirty = true;
        } },
    };
    CheckVariantsAreDistinct(ctx, kScene, IM_ARRAYSIZE(kScene));
    g_state.current_file_path.clear();
    g_state.dirty = false;
  };

  // The left cluster still fits beside the right-aligned Log button at the narrowest window the
  // user can drag to (P41/P43 budget).
  //
  // The concrete failure: ImGui does not ellipsize a SameLine run. An over-long left cluster simply
  // overflows, and the Log button — positioned at WindowWidth - log_w - WindowPadding.x — gets
  // overlapped and pushed out of reach. Measuring text extents is what makes this checkable without
  // resizing a real window. The floor is kMinWindowWidth, which main.cpp hands to
  // glfwSetWindowSizeLimits, so it is the actual narrowest case rather than a number chosen here.
  // The right cluster ends at the window's right edge, whether or not the counters are in it.
  //
  // This is the mechanical half of the layout statement the pixel cases above cannot make: the
  // cluster is placed by measuring itself and starting that far in from the edge, so a counter
  // whose measured width differs from its drawn width pushes the Log button off the end by exactly
  // the difference. Log is the one item down here with an ID, which makes it the only thing in the
  // row an assertion can hold on to — and it is the LAST item in the cluster, so it is also the one
  // that moves when anything before it is measured wrong. Whether the counters render at all is a
  // separate question, answered in pixels by ray_count_gate_and_magnitudes above.
  ImGuiTest* t_flush = IM_REGISTER_TEST(engine, "status_bar", "right_cluster_stays_flush_to_the_edge");
  t_flush->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);

    std::string failures;
    // Both sides of the counters' gate: with no run they are absent and the cluster is the Log
    // button alone; with a run they precede it and the whole run must still end at the same edge.
    const unsigned long long kRayCounts[] = { 0ULL, 5'400'000ULL };
    for (unsigned long long rays : kRayCounts) {
      InjectStats(rays > 0 ? 12ULL : 0ULL, rays > 0 ? 340ULL : 0ULL, rays);
      ctx->Yield(3);

      ImGuiWindow* bar = ctx->GetWindowByRef("##StatusBar");
      if (bar == nullptr) {
        failures += " no-status-bar";
        continue;
      }
      const ImGuiTestItemInfo log =
          ctx->ItemInfo("##StatusBar/**/" ICON_FA_CHEVRON_RIGHT " Log", ImGuiTestOpFlags_NoError);
      if (log.ID == 0) {
        failures += " missing-log-button";
        continue;
      }
      const float edge = bar->Pos.x + bar->Size.x - ImGui::GetStyle().WindowPadding.x;
      if (ImFabs(log.RectFull.Max.x - edge) > 1.0f) {
        failures += (rays > 0 ? " with-counters:right-edge" : " no-counters:right-edge");
        ctx->LogInfo("rays=%llu: Log ends at %.1f, window edge is %.1f", rays, static_cast<double>(log.RectFull.Max.x),
                     static_cast<double>(edge));
      }
    }

    InjectStats(0, 0, 0);
    ctx->Yield(2);
    // After the loop, not inside it: a fatal assert in the body would hide the second row, which is
    // the one carrying the conditional members.
    IM_CHECK_STR_EQ(failures.c_str(), "");
  };

  // The row fits inside the narrowest window the app allows, with every bounded segment at its
  // widest at once.
  //
  // The arithmetic follows the layout rather than the reading order: the row is two clusters, one
  // starting at the left padding and one ending at the right, so what has to fit is
  // left + a gap + right, not the sum of every segment laid end to end. Summing everything in
  // sequence would have been the conservative choice while the row was one run; it is not
  // conservative now, it is a different quantity — and the difference is the gap, which is the only
  // thing between the two clusters and the first thing an overlong segment eats.
  ImGuiTest* t_budget = IM_REGISTER_TEST(engine, "status_bar", "width_budget_at_minimum_window");
  t_budget->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    ctx->Yield(3);

    const ImGuiStyle& style = ImGui::GetStyle();
    const float spacing = style.ItemSpacing.x;
    // A separator costs its own glyph plus the item spacing on either side of it.
    const float divider = spacing + ImGui::CalcTextSize("\xC2\xB7").x + spacing;

    // Worst case on every bounded segment at once: the longest status word, a ten-digit ray count,
    // the longest lens name, and both sampling counters in their widest (barely-sampled) form —
    // which is a real configuration, not a contrived one: a scene with a fixed shape AND a fixed
    // orientation reads "1 per N" on both.
    const std::string sampling = FormatSamplingSegment(1, 1, 9900000000ULL);
    const float sampling_w = ImGui::CalcTextSize(sampling.c_str()).x;
    const float total_rays_w = ImGui::CalcTextSize("Total rays: 9.9 x10^9").x;
    const float res_lens_w = ImGui::CalcTextSize("1024x512  Dual Fisheye  FOV:180").x;
    // The dot leads the word, so the state segment is wider than the word alone.
    const float status_w = ImGui::CalcTextSize(ICON_FA_CIRCLE " Simulating...").x;

    // Both-barely-sampled really is the widest the segment gets, so the budget is not measuring an
    // accidentally narrow case.
    IM_CHECK_GE(sampling_w, ImGui::CalcTextSize(FormatSamplingSegment(1, 1, 1).c_str()).x);
    IM_CHECK_GE(sampling_w, ImGui::CalcTextSize(FormatSamplingSegment(0, 0, 0).c_str()).x);

    // Declared allowance for the (unbounded) file-name segment: the repo's own example config name
    // plus the dirty marker.
    const float filename_allowance = ImGui::CalcTextSize("scene.lmc *").x;
    const float log_button = ImGui::CalcTextSize(ICON_FA_CHEVRON_RIGHT " Log").x + style.FramePadding.x * 2;

    const float left = status_w + divider + res_lens_w + divider + filename_allowance;
    const float right = total_rays_w + divider + sampling_w + spacing + log_button;
    // One item spacing is the least air the two clusters may have between them; below that they are
    // touching, which is the state the top bar's own alignment case calls an overlap.
    const float required = style.WindowPadding.x + left + spacing + right + style.WindowPadding.x;
    const float floor_width = static_cast<float>(lumice::gui::kMinWindowWidth);
    ctx->LogInfo("left=%.1f right=%.1f required=%.1f min_window=%.1f headroom=%.1f", left, right, required, floor_width,
                 floor_width - required);
    IM_CHECK_LE(required, floor_width);

    // The sampling segment must not become the row's dominant consumer. Without this the check
    // above could be satisfied by shrinking the file-name allowance to nothing.
    IM_CHECK_LT(sampling_w, res_lens_w + total_rays_w);
  };
}
