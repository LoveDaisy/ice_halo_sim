// The Raypath Analysis window (src/gui/analysis_panel.cpp), driven through real frames, real
// clicks and a real server: the one layer that can see the wiring the unit cases cannot — the
// preview click reaching LUMICE_UnprojectPixel through the panel's own DPI path, the Analyze
// button reaching LUMICE_StartRaypathAnalysis and its result reaching the list through the
// poller, the slider leaving the lifecycle alone, and the top-bar mutual exclusion.
//
// The scene is the 22-degree halo: one randomly oriented prism under a 20-degree sun, the same
// fixture the C API and e2e layers use for the same run. The camera looks 22 degrees above the
// sun on its azimuth, so the frame's centre pixel sits ON the halo ring — the point a user would
// click to ask "what makes this bright arc" — and a cone around it collects real chains.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/analysis_panel.hpp"
#include "gui/gui_constants.hpp"
#include "gui/sim_state_rules.hpp"
#include "imgui_internal.h"
#include "include/lumice.h"
#include "test_gui_shared.hpp"

namespace {

using SimState = gui::GuiState::SimState;

const char* const kWindowRef = "//" ICON_FA_ROUTE " Raypath Analysis";
const char* const kTopBarButton = "##TopBar/" ICON_FA_ROUTE " Analysis";
const char* const kAnalyzeButton = ICON_FA_PLAY " Analyze";
const char* const kPickButton = ICON_FA_CROSSHAIRS " Pick on preview";
const char* const kPickBanner = "//" ICON_FA_ROUTE " Raypath Analysis/##pick_banner";

const char* const kHalo22Json = R"({
  "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.2},
               "axis": {"zenith": {"type": "uniform", "mean": 90, "std": 360},
                        "azimuth": {"type": "uniform", "mean": 0, "std": 360}}}],
  "filter": [],
  "scene": {"light_source": {"type": "sun", "altitude": 20.0, "spectrum": "D65"},
            "ray_num": 100000, "max_hits": 7,
            "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 10}]}]},
  "render": [{"id": 1, "lens": {"type": "linear", "fov": 60},
              "resolution": [64, 64], "view": {"elevation": 42}}]
})";

// Same shape as test_gui_sim_smoke.cpp's guard, for the same reason: IM_CHECK returns out of
// the case, and a server left running would be inherited by the next one.
struct ScopedServerGuard {
  ~ScopedServerGuard() {
    if (gui::g_server != nullptr) {
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
      gui::g_state.analysis.started = false;
      gui::g_state.analysis_run_in_progress = false;
    }
    gui::g_state.use_gpu_backend = false;
    gui::ResetServerConstructionTrackers();
  }
};

template <typename Pred>
bool DriveUntil(ImGuiTestContext* ctx, Pred pred, int timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    ctx->Yield();
  }
  return true;
}

// Server up, halo scene loaded, one finite render run to completion — the state Analyze needs.
// `infinite` leaves the render running instead, for the mutual-exclusion case and for the
// run-after-analysis cases (an unbounded analysis is the one the cone target ends). `gpu` asks
// for the GPU backend the way the Settings box does: DoRun reconstructs the server for it
// (Metal here; where no device is available ResolveGpuBackend falls back to CPU, as in
// test_color_window.cpp's GPU case), so the CPU server created below is only ever the seed.
bool BringUpHaloScene(ImGuiTestContext* ctx, bool infinite, bool gpu = false) {
  ResetTestState();
  gui::g_server = LUMICE_CreateServer();
  IM_CHECK_RETV(gui::g_server != nullptr, false);
  gui::ResetServerConstructionTrackers();  // the seed is a CPU server; make the tracker say so
  LUMICE_SetLogLevel(gui::g_server, static_cast<LUMICE_LogLevel>(g_core_log_level));
  IM_CHECK_RETV(gui::DeserializeFromJson(kHalo22Json, gui::g_state), false);
  gui::g_state.renderer.sim_resolution_index = 0;
  gui::g_state.sim.infinite = infinite;
  gui::g_state.sim.ray_num_millions = 0.1f;
  gui::g_state.use_gpu_backend = gpu;
  ctx->Yield(2);
  gui::DoRun(/*user_initiated=*/true);
  if (infinite) {
    IM_CHECK_RETV(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kSimulating; }, 10), false);
  } else {
    IM_CHECK_RETV(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kDone; }, 60), false);
    // A completed run has put a texture up, so the preview viewport the click is measured in is
    // live. (Mid-run it may not be yet — the first upload waits on the quality gate.)
    IM_CHECK_RETV(gui::g_preview_vp.active, false);
  }
  return true;
}

void OpenWindow(ImGuiTestContext* ctx) {
  if (!gui::g_state.analysis.window_open) {
    ctx->ItemClick(kTopBarButton);
    ctx->Yield(2);
  }
  ctx->WindowMove(kWindowRef, ImVec2(60, 60));
  ctx->Yield(1);
}

// Click the preview's centre with the pick armed; returns the canvas pixel the click landed on.
gui::CanvasPixel PickPreviewCentre(ImGuiTestContext* ctx) {
  ImGuiWindow* w = ctx->GetWindowByRef("//##PreviewPanel");
  IM_CHECK_SILENT_RETV(w != nullptr, gui::CanvasPixel{});
  const float dpi_x = gui::g_preview_vp.dpi_scale_x;
  const float dpi_y = gui::g_preview_vp.dpi_scale_y;
  const float px_pt = static_cast<float>(gui::g_preview_vp.vp_w) * 0.5f / dpi_x;
  const float py_pt = static_cast<float>(gui::g_preview_vp.vp_h) * 0.5f / dpi_y;
  ctx->MouseMoveToPos(ImVec2(w->Pos.x + px_pt, w->Pos.y + py_pt));
  ctx->Yield(2);
  // Where the click actually lands, from the mouse's own position: MouseMoveToPos can be a pixel
  // off the request, and the oracle must be built from the same pixel the panel saw.
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  const std::optional<gui::CanvasPixel> px = gui::PreviewPointToCanvasPixel(
      mouse.x - w->Pos.x, mouse.y - w->Pos.y, dpi_x, dpi_y, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
  IM_CHECK_SILENT_RETV(px.has_value(), gui::CanvasPixel{});
  ctx->MouseClick(0);
  ctx->Yield(2);
  return *px;
}

// The cone marker's position on screen this frame (the panel's own projection, through the DPI
// path the drawing uses), or nullopt when there is none.
std::optional<ImVec2> MarkerScreenPos(ImGuiTestContext* ctx) {
  ImGuiWindow* w = ctx->GetWindowByRef("//##PreviewPanel");
  if (w == nullptr) {
    return std::nullopt;
  }
  const LUMICE_AnnotationView view =
      gui::PreviewAnnotationView(gui::g_state, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
  const std::optional<gui::CanvasPixel> px = gui::ProjectConeCenterMarker(gui::g_state, view);
  if (!px.has_value()) {
    return std::nullopt;
  }
  float x_pt = 0.0f;
  float y_pt = 0.0f;
  gui::CanvasPixelToPreviewPoint(px->px, px->py, gui::g_preview_vp.dpi_scale_x, gui::g_preview_vp.dpi_scale_y, &x_pt,
                                 &y_pt);
  return ImVec2(w->Pos.x + x_pt, w->Pos.y + y_pt);
}

// The direction LUMICE_UnprojectPixel gives for the mouse's CURRENT position on the preview — the
// oracle for "the centre is the direction under the cursor", built from the same pixel the panel
// saw (PickPreviewCentre's reasoning).
bool UnprojectMouse(float out[3]) {
  ImGuiWindow* w = ImGui::FindWindowByName("##PreviewPanel");
  if (w == nullptr) {
    return false;
  }
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  const std::optional<gui::CanvasPixel> px =
      gui::PreviewPointToCanvasPixel(mouse.x - w->Pos.x, mouse.y - w->Pos.y, gui::g_preview_vp.dpi_scale_x,
                                     gui::g_preview_vp.dpi_scale_y, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
  if (!px.has_value()) {
    return false;
  }
  const LUMICE_AnnotationView view =
      gui::PreviewAnnotationView(gui::g_state, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
  int valid = 0;
  return LUMICE_UnprojectPixel(&view, px->px, px->py, out, &valid) == LUMICE_OK && valid == 1;
}

// Whether the pick banner is on screen: the child window exists and was submitted last frame
// (a child that stopped being submitted stays in ImGui's window list, so existence alone says
// nothing — WasActive does).
bool PickBannerVisible(ImGuiTestContext* ctx) {
  const ImGuiTestItemInfo info = ctx->WindowInfo(kPickBanner, ImGuiTestOpFlags_NoError);
  return info.Window != nullptr && info.Window->WasActive;
}

bool RunPointAnalysisToCompletion(ImGuiTestContext* ctx) {
  ctx->SetRef(kWindowRef);
  ctx->ItemClick("Point");
  ctx->ItemClick(kPickButton);
  ctx->Yield(1);
  IM_CHECK_RETV(gui::g_state.analysis.pick_armed, false);
  ctx->SetRef("");
  PickPreviewCentre(ctx);
  IM_CHECK_RETV(gui::g_state.analysis.cone_center_valid, false);
  ctx->SetRef(kWindowRef);
  ctx->ItemClick(kAnalyzeButton);
  ctx->SetRef("");
  IM_CHECK_RETV(gui::g_state.analysis.started, false);
  IM_CHECK_RETV(
      DriveUntil(
          ctx, [] { return !gui::g_state.analysis_run_in_progress && gui::g_state.analysis_result.payload != nullptr; },
          60),
      false);
  return true;
}

// The owner's own sequence: Run, Analyze, Run again — and the second Run RENDERS. This is the
// one path through the panel that nothing above drives (every case so far ends on the analysis
// or on the slider), and the one that went black: a cone-ROI analysis that ends on its stop
// target leaves the server's early-stop flag up, and a render session that read the flag
// unconditionally traced nothing — no batch, no frame, a preview stuck on "Simulating" until a
// backend switch rebuilt the server. The server case that pins the mechanism is
// ServerAnalysisRun.RenderAfterConeStoppedAnalysisProducesAFrame; this one pins the user's view
// of it: sim_state reaches kDone and a texture goes up.
//
// The precondition is that the analysis ends on its CONE TARGET, not on the ray budget: the
// panel's target is kAnalysisConeStopTarget rays in the cone, which a 0.1 M-ray budget never
// reaches, so the render whose scene the analysis inherits is an unbounded one, stopped by hand
// once it has put a picture up (the pick needs a live preview). The run after the analysis is
// then made finite so that "it rendered" can be read as kDone rather than as "never ended".
//
// `exclude` adds the owner's step in between: select the top chain, press "Exclude this
// raypath", so the second Run commits a document with a filter in it. The bug does not need it
// (the server case has none), and it is driven here so the sequence as reported stays covered.
bool RunAfterAnalysisRenders(ImGuiTestContext* ctx, bool exclude, bool gpu) {
  IM_CHECK_RETV(BringUpHaloScene(ctx, /*infinite=*/true, gpu), false);
  IM_CHECK_RETV(
      DriveUntil(ctx, [] { return gui::g_preview_vp.active && gui::g_state.texture_upload_count > 0; }, 30), false);
  gui::DoStop();
  IM_CHECK_RETV(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kDone; }, 20), false);
  OpenWindow(ctx);
  IM_CHECK_RETV(RunPointAnalysisToCompletion(ctx), false);
  // Positive control on the precondition: an unbounded analysis can only have COMPLETED (not
  // merely stopped) by reaching its cone target, i.e. with the early-stop flag raised.
  LUMICE_SimLifecycleResult after_analysis{};
  LUMICE_GetSimLifecycle(gui::g_server, &after_analysis);
  IM_CHECK_RETV(after_analysis.lifecycle == static_cast<int>(LUMICE_LIFECYCLE_COMPLETED), false);
  IM_CHECK_RETV(gui::g_state.analysis_result.payload->roi_mode == LUMICE_RAYPATH_ROI_CONE, false);

  if (exclude) {
    const auto& view_result = gui::g_state.analysis_result;
    IM_CHECK_RETV(!view_result.display_order.empty(), false);
    const int top = view_result.display_order[0];
    ctx->SetRef(kWindowRef);
    // The row is a Selectable under PushID(original index) inside the results table; the
    // wildcard finds it by label through the table's and the id's anonymous path segments.
    const std::string row = std::string("**/") + view_result.payload->entries[static_cast<size_t>(top)].display;
    ctx->ItemClick(row.c_str());
    ctx->Yield(1);
    IM_CHECK_RETV(gui::g_state.analysis.selected_entry.has_value(), false);
    IM_CHECK_RETV(*gui::g_state.analysis.selected_entry == top, false);
    IM_CHECK_RETV(!IsDisabled(ctx->ItemInfo(ICON_FA_BAN " Exclude this raypath")), false);
    ctx->ItemClick(ICON_FA_BAN " Exclude this raypath");
    ctx->SetRef("");
    ctx->Yield(1);
    // The filter is in the document: the same entry is no longer excludable a second time.
    IM_CHECK_RETV(gui::EvaluateExcludeEligibility(gui::g_state, nullptr) == gui::ExcludeEligibility::kEntryHasFilter,
                  false);
  }

  // The second Run, from the top bar, on a finite budget.
  gui::g_state.sim.infinite = false;
  gui::g_state.sim.ray_num_millions = 0.1f;
  const unsigned long long uploads_before = gui::g_state.texture_upload_count;
  const unsigned long long serial_before = gui::g_state.last_uploaded_texture_serial;
  ctx->Yield(1);
  IM_CHECK_RETV(!IsDisabled(ctx->ItemInfo("##TopBar/" ICON_FA_PLAY " Run")), false);
  ctx->ItemClick("##TopBar/" ICON_FA_PLAY " Run");
  IM_CHECK_RETV(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kSimulating; }, 10), false);
  // Both halves of "it rendered": the run ended, and its picture went up. The document is
  // dirty here (the budget edit above; the Exclude before it), and a dirty document's finished
  // run reads kModified, not kDone — the real main loop clears dirty on its 70 ms auto-commit
  // tick while simulating (main.cpp), a tick gui_test's loop only has under --main-loop-commit,
  // so it is cleared by hand at the same moment the app would.
  gui::g_state.dirty = false;
  IM_CHECK_RETV(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kDone; }, 60), false);
  IM_CHECK_RETV(
      DriveUntil(ctx, [uploads_before] { return gui::g_state.texture_upload_count > uploads_before; }, 10), false);
  IM_CHECK_RETV(gui::g_state.last_uploaded_texture_serial != serial_before, false);
  return true;
}

}  // namespace

void RegisterRaypathAnalysisPanelTests(ImGuiTestEngine* engine) {
  // AC2, first case: open -> Point -> click the preview -> the centre is the click's unprojection
  // -> Analyze -> the list is non-empty and its first row carries the largest energy.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "point_pick_analyze_lists_chains");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/false));
      OpenWindow(ctx);
      IM_CHECK(gui::g_state.analysis.window_open);

      ctx->SetRef(kWindowRef);
      ctx->ItemClick("Point");
      IM_CHECK_EQ(gui::g_state.analysis.roi_mode, LUMICE_RAYPATH_ROI_CONE);
      ctx->ItemClick(kPickButton);
      ctx->Yield(1);
      IM_CHECK(gui::g_state.analysis.pick_armed);
      ctx->SetRef("");

      const gui::CanvasPixel px = PickPreviewCentre(ctx);
      // The ROI is placed: the pick is consumed, the centre is valid, and this frame's marker
      // projects back onto the pixel that was clicked.
      IM_CHECK(!gui::g_state.analysis.pick_armed);
      IM_CHECK(gui::g_state.analysis.cone_center_valid);
      const LUMICE_AnnotationView view =
          gui::PreviewAnnotationView(gui::g_state, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
      const std::optional<gui::CanvasPixel> marker = gui::ProjectConeCenterMarker(gui::g_state, view);
      IM_CHECK(marker.has_value());
      IM_CHECK_EQ(marker->px, px.px);
      IM_CHECK_EQ(marker->py, px.py);
      // And its direction is the click's unprojection under the view the picture is drawn with —
      // the oracle is the C API called directly on the same pixel, not the panel's own function.
      float want[3] = { 0.0f, 0.0f, 0.0f };
      int valid = 0;
      IM_CHECK_EQ(LUMICE_UnprojectPixel(&view, px.px, px.py, want, &valid), LUMICE_OK);
      IM_CHECK_EQ(valid, 1);
      IM_CHECK_FLOAT_NEAR(gui::g_state.analysis.cone_center_dir[0], want[0], 1e-6f);
      IM_CHECK_FLOAT_NEAR(gui::g_state.analysis.cone_center_dir[1], want[1], 1e-6f);
      IM_CHECK_FLOAT_NEAR(gui::g_state.analysis.cone_center_dir[2], want[2], 1e-6f);
      // The frame centre looks 42 degrees up: the centre direction says so (altitude = asin(-z)).
      IM_CHECK_FLOAT_NEAR(std::asin(-gui::g_state.analysis.cone_center_dir[2]) * 180.0f / 3.14159265f, 42.0f, 0.5f);
      // The ring is drawable: the local scale at the click resolves.
      IM_CHECK(gui::ConeRingRadiusCanvasPx(view, px.px, px.py, gui::g_state.analysis.cone_radius_deg).has_value());

      // Analyze, through the button.
      LUMICE_SimLifecycleResult before{};
      LUMICE_GetSimLifecycle(gui::g_server, &before);
      ctx->SetRef(kWindowRef);
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kAnalyzeButton)));
      ctx->ItemClick(kAnalyzeButton);
      ctx->SetRef("");
      IM_CHECK(gui::g_state.analysis.started);
      IM_CHECK(DriveUntil(
          ctx, [] { return !gui::g_state.analysis_run_in_progress && gui::g_state.analysis_result.payload != nullptr; },
          60));
      const auto& view_result = gui::g_state.analysis_result;
      IM_CHECK_EQ(view_result.payload->roi_mode, LUMICE_RAYPATH_ROI_CONE);
      IM_CHECK_GT(view_result.payload->entries.size(), 0u);
      IM_CHECK_EQ(view_result.display_order.size(), view_result.payload->entries.size());
      // First row = the largest displayed energy of all rows.
      const double first = view_result.display_energy[static_cast<size_t>(view_result.display_order[0])];
      IM_CHECK_GT(first, 0.0);
      const double largest = *std::max_element(view_result.display_energy.begin(), view_result.display_energy.end());
      IM_CHECK_EQ(first, largest);
      // The 22-degree halo IS the top chain at this point of the sky.
      IM_CHECK_STR_EQ(view_result.payload->entries[static_cast<size_t>(view_result.display_order[0])].display,
                      "crystal0(3-5)");
      // The analysis shares the render's epoch (server.cpp: same scene, same epoch) and the
      // picture on screen is still the render's: sim_state stayed kDone throughout.
      LUMICE_SimLifecycleResult after{};
      LUMICE_GetSimLifecycle(gui::g_server, &after);
      IM_CHECK_EQ(after.epoch, before.epoch);
      IM_CHECK_EQ((int)gui::g_state.sim_state, (int)SimState::kDone);
    };
  }

  // AC2, second case: a render in progress keeps Analyze shut, and the other direction — an
  // analysis in progress keeps Run shut on the top bar — with the window's own Stop ending it.
  // The scene is an INFINITE run on purpose: a whole-sky analysis of it has no end of its own
  // (lumice.h: an infinite budget runs until LUMICE_StopServer), so the in-progress state is
  // held open for as long as the assertions need, and Stop is the only way out — which is the
  // path this case exists to drive.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "analyze_and_run_exclude_each_other");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/true));
      OpenWindow(ctx);
      ctx->SetRef(kWindowRef);
      IM_CHECK(IsDisabled(ctx->ItemInfo(kAnalyzeButton)));
      IM_CHECK(!gui::CanStartAnalysis(true, true, gui::g_state.sim_state, gui::g_state.analysis_run_in_progress));
      // A click on it does nothing: no intent, no run.
      ctx->ItemClick(kAnalyzeButton);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.analysis.started);
      ctx->SetRef("");

      // Stop the render; Analyze opens up; start it; Run on the top bar is now the one shut.
      gui::DoStop();
      IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.sim_state == SimState::kDone; }, 20));
      ctx->SetRef(kWindowRef);
      ctx->ItemClick("Whole sky");
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kAnalyzeButton)));
      ctx->ItemClick(kAnalyzeButton);
      ctx->SetRef("");
      IM_CHECK(gui::g_state.analysis.started);
      IM_CHECK(gui::g_state.analysis_run_in_progress);
      ctx->Yield(2);
      IM_CHECK(gui::g_state.analysis_run_in_progress);
      IM_CHECK(IsDisabled(ctx->ItemInfo("##TopBar/" ICON_FA_PLAY " Run")));
      IM_CHECK(gui::IsBackendBusy(gui::g_state.sim_state, gui::g_state.analysis_run_in_progress));
      IM_CHECK_EQ((int)gui::g_state.sim_state, (int)SimState::kDone);  // the picture is still the render's
      // Partial results reach the list while the run is in progress.
      IM_CHECK(DriveUntil(ctx, [] { return gui::g_state.analysis_result.payload != nullptr; }, 20));
      IM_CHECK_EQ(gui::g_state.analysis_result.payload->roi_mode, LUMICE_RAYPATH_ROI_FULL_SKY);

      // Stop from the window: the intent is withdrawn at the command, the top bar reopens, and
      // the result adopted so far stays on show.
      const auto partial = gui::g_state.analysis_result.payload;
      ctx->SetRef(kWindowRef);
      ctx->ItemClick(ICON_FA_STOP " Stop");
      ctx->SetRef("");
      IM_CHECK(!gui::g_state.analysis.started);
      IM_CHECK(DriveUntil(
          ctx, [] { return !gui::g_state.analysis_run_in_progress && gui::g_state.sim_state == SimState::kDone; }, 20));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("##TopBar/" ICON_FA_PLAY " Run")));
      IM_CHECK(gui::g_state.analysis_result.payload == partial);
      ctx->SetRef(kWindowRef);
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kAnalyzeButton)));
      ctx->SetRef("");
    };
  }

  // AC3: the radius slider is display-time. Setting it changes the energies on the list and
  // starts no run — the lifecycle epoch, the in-progress flag and the result object are all the
  // same before and after, only the projection over its rings changed.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "radius_slider_is_display_time");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/false));
      OpenWindow(ctx);
      IM_CHECK(RunPointAnalysisToCompletion(ctx));
      const auto payload_before = gui::g_state.analysis_result.payload;
      const double total_before = gui::g_state.analysis_result.display_total;
      const int rings_before = gui::g_state.analysis_result.display_ring_count;
      LUMICE_SimLifecycleResult before{};
      LUMICE_GetSimLifecycle(gui::g_server, &before);
      const unsigned long long uploads_before = gui::g_state.texture_upload_count;

      ctx->SetRef(kWindowRef);
      ctx->ItemInputValue("Radius", 10.0f);
      ctx->SetRef("");
      ctx->Yield(5);

      IM_CHECK_FLOAT_NEAR(gui::g_state.analysis.cone_radius_deg, 10.0f, 1e-3f);
      IM_CHECK_GT(gui::g_state.analysis_result.display_ring_count, rings_before);
      // More rings, more of the cone's energy on the list — strictly, since the halo ring runs
      // through the cone and rays land at every distance from its centre.
      IM_CHECK_GT(gui::g_state.analysis_result.display_total, total_before);
      // And nothing ran: same result object, same epoch, no in-progress edge, no upload.
      IM_CHECK(gui::g_state.analysis_result.payload == payload_before);
      IM_CHECK(!gui::g_state.analysis_run_in_progress);
      LUMICE_SimLifecycleResult after{};
      LUMICE_GetSimLifecycle(gui::g_server, &after);
      IM_CHECK_EQ(after.epoch, before.epoch);
      IM_CHECK_EQ(after.lifecycle, before.lifecycle);
      IM_CHECK_EQ(gui::g_state.texture_upload_count, uploads_before);
      IM_CHECK_EQ((int)gui::g_state.sim_state, (int)SimState::kDone);
    };
  }

  // AC3: entering Point mode places a centre at once — the viewport's middle pixel, unprojected —
  // so there is a marker to drag before any pick. The oracle is the C API on that pixel.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "point_mode_defaults_centre_to_viewport_middle");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/false));
      OpenWindow(ctx);
      IM_CHECK(!gui::g_state.analysis.cone_center_valid);
      ctx->SetRef(kWindowRef);
      ctx->ItemClick("Point");
      ctx->SetRef("");
      ctx->Yield(1);
      IM_CHECK(gui::g_state.analysis.cone_center_valid);
      IM_CHECK(!gui::g_state.analysis.pick_armed);
      const LUMICE_AnnotationView view =
          gui::PreviewAnnotationView(gui::g_state, gui::g_preview_vp.vp_w, gui::g_preview_vp.vp_h);
      float want[3] = { 0.0f, 0.0f, 0.0f };
      int valid = 0;
      IM_CHECK_EQ(LUMICE_UnprojectPixel(&view, gui::g_preview_vp.vp_w / 2, gui::g_preview_vp.vp_h / 2, want, &valid),
                  LUMICE_OK);
      IM_CHECK_EQ(valid, 1);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[0], want[0]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[1], want[1]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[2], want[2]);
      // And the marker is on the picture, at that pixel.
      const std::optional<gui::CanvasPixel> marker = gui::ProjectConeCenterMarker(gui::g_state, view);
      IM_CHECK(marker.has_value());
      IM_CHECK_EQ(marker->px, gui::g_preview_vp.vp_w / 2);
      IM_CHECK_EQ(marker->py, gui::g_preview_vp.vp_h / 2);
      // Analyze is open at once: the centre the button needed is there.
      ctx->SetRef(kWindowRef);
      IM_CHECK(!IsDisabled(ctx->ItemInfo(kAnalyzeButton)));
      ctx->SetRef("");
    };
  }

  // AC1: the marker is the direction's projection on THIS view. A camera drag on the preview
  // (started away from the marker, so the camera and not the marker owns it) turns the view;
  // the marker's screen position moves with the picture and the direction does not change.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "marker_follows_the_view_drag");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/false));
      OpenWindow(ctx);
      ctx->SetRef(kWindowRef);
      ctx->ItemClick("Point");
      ctx->ItemClick(kPickButton);
      ctx->SetRef("");
      ctx->Yield(1);
      PickPreviewCentre(ctx);
      IM_CHECK(gui::g_state.analysis.cone_center_valid);
      IM_CHECK(!gui::g_state.analysis.pick_armed);
      const float dir_before[3] = { gui::g_state.analysis.cone_center_dir[0], gui::g_state.analysis.cone_center_dir[1],
                                    gui::g_state.analysis.cone_center_dir[2] };
      const std::optional<ImVec2> marker_before = MarkerScreenPos(ctx);
      IM_CHECK(marker_before.has_value());
      const float az_before = gui::g_state.renderer.azimuth;

      // Press well clear of the marker's grab radius (down-right of it, inside the preview) and
      // drag horizontally: an orbit of the camera, on the linear lens the scene renders with.
      const ImVec2 grab(marker_before->x + 150.0f, marker_before->y + 120.0f);
      ctx->MouseMoveToPos(grab);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.analysis.cone_marker_dragging);
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(ImVec2(grab.x + 60.0f, grab.y));
      ctx->MouseUp(0);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.analysis.cone_marker_dragging);
      IM_CHECK_NE(gui::g_state.renderer.azimuth, az_before);  // the view turned
      const std::optional<ImVec2> marker_after = MarkerScreenPos(ctx);
      IM_CHECK(marker_after.has_value());
      // The picture moved under the fixed direction, so the marker moved on screen — by a good
      // fraction of the mouse travel (an orbit moves the content one pixel per pixel of drag).
      IM_CHECK_GT(ImFabs(marker_after->x - marker_before->x), 20.0f);
      // The direction is the truth and did not change.
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[0], dir_before[0]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[1], dir_before[1]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[2], dir_before[2]);
    };
  }

  // AC2: the marker itself can be dragged. Hovering it shows the hand cursor; a press on it and a
  // move re-aim the centre to the direction under the cursor (LUMICE_UnprojectPixel on the
  // mouse's pixel — bit-identical, the drag transfers the inverse's output verbatim) while the
  // camera stays exactly where it was.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "marker_drag_moves_the_centre_not_the_view");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/false));
      OpenWindow(ctx);
      ctx->SetRef(kWindowRef);
      ctx->ItemClick("Point");
      ctx->SetRef("");
      ctx->Yield(1);
      IM_CHECK(gui::g_state.analysis.cone_center_valid);  // the default centre, AC3
      const std::optional<ImVec2> marker = MarkerScreenPos(ctx);
      IM_CHECK(marker.has_value());
      const float az_before = gui::g_state.renderer.azimuth;
      const float el_before = gui::g_state.renderer.elevation;
      const float fov_before = gui::g_state.renderer.fov;

      // Off the marker: the arrow. On it: the hand.
      ctx->MouseMoveToPos(ImVec2(marker->x + 120.0f, marker->y + 90.0f));
      ctx->Yield(2);
      IM_CHECK_EQ(ImGui::GetMouseCursor(), ImGuiMouseCursor_Arrow);
      ctx->MouseMoveToPos(*marker);
      ctx->Yield(2);
      IM_CHECK_EQ(ImGui::GetMouseCursor(), ImGuiMouseCursor_Hand);

      // Grab and drag to another point of the sky.
      ctx->MouseDown(0);
      ctx->Yield(1);
      IM_CHECK(gui::g_state.analysis.cone_marker_dragging);
      ctx->MouseMoveToPos(ImVec2(marker->x + 45.0f, marker->y - 30.0f));
      ctx->Yield(1);
      IM_CHECK(gui::g_state.analysis.cone_marker_dragging);
      IM_CHECK_EQ(ImGui::GetMouseCursor(), ImGuiMouseCursor_Hand);
      float want[3] = { 0.0f, 0.0f, 0.0f };
      IM_CHECK(UnprojectMouse(want));
      ctx->MouseUp(0);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.analysis.cone_marker_dragging);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[0], want[0]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[1], want[1]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[2], want[2]);
      // The marker now sits under the mouse (to the pixel rounding of the projection).
      const std::optional<ImVec2> marker_after = MarkerScreenPos(ctx);
      IM_CHECK(marker_after.has_value());
      IM_CHECK_LT(ImFabs(marker_after->x - ImGui::GetIO().MousePos.x), 2.0f);
      IM_CHECK_LT(ImFabs(marker_after->y - ImGui::GetIO().MousePos.y), 2.0f);
      // The camera did not move.
      IM_CHECK_EQ(gui::g_state.renderer.azimuth, az_before);
      IM_CHECK_EQ(gui::g_state.renderer.elevation, el_before);
      IM_CHECK_EQ(gui::g_state.renderer.fov, fov_before);
    };
  }

  // AC4: the pick mode is visible for as long as it is on. The button arms it and the banner
  // appears; Esc disarms it and the banner goes; armed again, the click on the preview sets the
  // centre and disarms.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "pick_mode_shows_a_banner_until_consumed");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(BringUpHaloScene(ctx, /*infinite=*/false));
      OpenWindow(ctx);
      ctx->SetRef(kWindowRef);
      ctx->ItemClick("Point");
      ctx->Yield(1);
      IM_CHECK(!PickBannerVisible(ctx));
      ctx->ItemClick(kPickButton);
      ctx->Yield(2);
      IM_CHECK(gui::g_state.analysis.pick_armed);
      IM_CHECK(PickBannerVisible(ctx));
      ctx->SetRef("");
      // Esc, from the preview (where the click would go): banner and flag both clear.
      ImGuiWindow* preview = ctx->GetWindowByRef("//##PreviewPanel");
      IM_CHECK(preview != nullptr);
      ctx->MouseMoveToPos(ImVec2(preview->Pos.x + preview->Size.x * 0.75f, preview->Pos.y + preview->Size.y * 0.75f));
      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.analysis.pick_armed);
      IM_CHECK(!PickBannerVisible(ctx));
      // Armed again, the click consumes it — at a point AWAY from the default marker (a click on
      // the marker is the drag gesture, which consumes the pick as well; the pick path proper is
      // what this case drives).
      ctx->SetRef(kWindowRef);
      ctx->ItemClick(kPickButton);
      ctx->SetRef("");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.analysis.pick_armed);
      IM_CHECK(PickBannerVisible(ctx));
      const std::optional<ImVec2> marker = MarkerScreenPos(ctx);
      IM_CHECK(marker.has_value());
      ctx->MouseMoveToPos(ImVec2(marker->x + 80.0f, marker->y + 60.0f));
      ctx->Yield(2);
      IM_CHECK_EQ(ImGui::GetMouseCursor(), ImGuiMouseCursor_Arrow);  // not the hand: not on the marker
      float want[3] = { 0.0f, 0.0f, 0.0f };
      IM_CHECK(UnprojectMouse(want));
      ctx->MouseClick(0);
      ctx->Yield(2);
      IM_CHECK(!gui::g_state.analysis.pick_armed);
      IM_CHECK(!PickBannerVisible(ctx));
      IM_CHECK(!gui::g_state.analysis.cone_marker_dragging);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[0], want[0]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[1], want[1]);
      IM_CHECK_EQ(gui::g_state.analysis.cone_center_dir[2], want[2]);
    };
  }

  // Run after a cone-stopped analysis renders — see RunAfterAnalysisRenders. Three cases: the
  // sequence itself on the CPU backend, the same with the owner's Exclude step in between, and
  // the sequence on the GPU backend (Metal on this tree's reference machine; the analysis is
  // forced to the CPU route inside the same server, so the render that follows is the GPU's
  // first session after an analysis — the shape the bug was reported on).
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "run_after_analysis_renders");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(RunAfterAnalysisRenders(ctx, /*exclude=*/false, /*gpu=*/false));
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "run_after_analysis_with_exclude_renders");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(RunAfterAnalysisRenders(ctx, /*exclude=*/true, /*gpu=*/false));
    };
  }
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "raypath_analysis", "run_after_analysis_renders_gpu");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ScopedServerGuard guard;
      IM_CHECK(RunAfterAnalysisRenders(ctx, /*exclude=*/false, /*gpu=*/true));
    };
  }
}
