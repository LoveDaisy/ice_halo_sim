#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "gui/gui_constants.hpp"
#include "include/lumice.h"
#include "test_gui_shared.hpp"

struct AutoEvScene {
  const char* name;
  const char* config_path;
  int render_w;
  int render_h;
  double psnr_threshold;
  // Overlay regression fields (defaults keep legacy scenes unchanged: enable_overlay=false
  // bypasses the overlay branch in TestFunc; remaining fields are inert in that case).
  bool enable_overlay = false;
  int lens_type = 0;
  float fov = 0.f;
  float elevation = 0.f;
  bool overlay_zenith_nadir = false;
  bool overlay_grid = false;
};

// clang-format off
// Per-scene PSNR thresholds are calibrated against the auto-EV-applied ("_on") reference
// only — the legacy "_off" (intensity_factor=1.0) mode was dropped (chore-auto-ev-
// regression-drop-off), since the GUI has no auto-EV toggle and off exercised no unique
// code path. Reference images are pixel-averaged means.
//
// Recalibrated 2026-07-07 (chore-regen-auto-ev-refs) from the FULL gui_test suite run ×25 pooled AFTER
// regenerating all references against the current orientation sampler. The prior refs predated
// scrum-328/330/332 (near-pole area-measure + unified cosine-measure LUT), so the intentional
// sampler change drifted PSNR ~0.3–0.8 dB below the stale-ref means → 31% run-level flake
// (explore-auto-ev-flake-followup). Regen also shifted per-scene NOISE floors (e.g. overlay_ea
// dropped to ~20.25, parhelion recovered to ~25.4), so thresholds were fully recalibrated, not
// just nudged. Stats are POOLED over 25 full-suite runs spanning idle AND loaded machine states:
// a single idle batch underestimates sigma (a loaded batch showed ~2x sigma, which left pyramid
// flaking when calibrated on idle-only). Refs/thresholds MUST come from the full suite, not `--filter auto_ev` (~1 dB
// optimistic — skips the ~240-test warm-up; that optimism was the original flake source).
// Each threshold = floor((mean − 4σ) · 2) / 2 over the 25 pooled full-suite PSNRs; all sit
// below every observed run's minimum with margin. (mean / σ / min per scene shown inline.)
static const AutoEvScene kScenes[] = {
  {"halo_22",    LUMICE_E2E_CONFIG_DIR "/halo_22.json",                           256, 256, 18.5},  // mean 20.00 σ0.141 (N=10 recalib 2026-07-31)
  {"multi_scat", LUMICE_E2E_CONFIG_DIR "/multi_scatter.json",                     256, 256, 16.0},  // mean 17.08 σ0.209 (N=10 recalib 2026-07-31) (regen: real 2-layer scattering)
  {"color",      LUMICE_E2E_CONFIG_DIR "/color.json",                             256, 256, 20.0},  // mean 21.13 σ0.227 (N=10 recalib 2026-07-31)
  {"pyramid",    LUMICE_E2E_CONFIG_DIR "/pyramid.json",                           256, 256, 19.5},  // mean 20.53 σ0.194 (N=10 recalib 2026-07-31)
  {"cza",        LUMICE_E2E_CONFIG_DIR "/cza.json",                               256, 256, 35.5},  // mean 36.70 σ0.242 (N=10 recalib 2026-07-31)
  {"parhelion",  LUMICE_E2E_CONFIG_DIR "/parhelion.json",                         256, 256, 26.0},  // mean 27.82 σ0.360 (N=10 recalib 2026-07-31)
  {"filters",    LUMICE_E2E_CONFIG_DIR "/filters.json",                           256, 256, 24.5},  // mean 25.91 σ0.239 (N=10 recalib 2026-07-31)
  {"rp46",       LUMICE_E2E_CONFIG_DIR "/raypath_symmetry_4_6.json",              256, 256, 27.5},  // mean 29.22 σ0.367 (N=10 recalib 2026-07-31)
  {"rp46_nof",   LUMICE_E2E_CONFIG_DIR "/raypath_symmetry_4_6_nofilter.json",     256, 256, 19.5},  // mean 20.59 σ0.184 (N=10 recalib 2026-07-31)
  // Overlay regression scene (task-288.7): fisheye EA at elevation=45° with zenith marker +
  // coordinate grid.
  {"overlay_ea", LUMICE_E2E_CONFIG_DIR "/halo_22.json",                           256, 256, 20.0,  // mean 21.30 σ0.165 (N=10 recalib 2026-07-31)
   true, lumice::gui::kLensTypeFisheyeEqualArea, 180.0f, 45.0f, true, true},
};
// clang-format on
static constexpr int kSceneCount = 10;

static bool RequestAndWaitExport(ImGuiTestContext* ctx, const gui::PreviewViewport& vp, const std::string& path) {
  g_auto_ev_export.export_path = path;
  g_auto_ev_export.custom_vp = vp;
  g_auto_ev_export.done.store(false);
  g_auto_ev_export.requested.store(true);
  for (int i = 0; i < 10 && !g_auto_ev_export.done.load(); ++i) {
    ctx->Yield(1);
  }
  return g_auto_ev_export.done.load() && g_auto_ev_export.result;
}

void RegisterAutoEvRegressionTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "auto_ev", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];
      ResetTestState();

      // 1. Create server and set log level before DoRun
      gui::g_server = LUMICE_CreateServer();
      LUMICE_SetLogLevel(gui::g_server, static_cast<LUMICE_LogLevel>(g_core_log_level));

      // 2. Read config JSON → DeserializeFromJson → fills g_state
      // DeserializeFromJson does state = GuiState{} internally (full reset, file_io.cpp:827);
      // layers are populated from crystal/scene arrays in the e2e JSON.
      {
        std::ifstream in(scene.config_path);
        IM_CHECK(in.is_open());
        std::string json_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        IM_CHECK(gui::DeserializeFromJson(json_str, gui::g_state));
      }

      // 3. Force sim_resolution_index=0 (512) to speed up CI without changing GUI option count
      gui::g_state.renderer.sim_resolution_index = 0;

      // 4. DoRun → triggers BuildScene (dual-fisheye override) → starts poller.
      // DoRun sets the intent (run_intent=kRunning); sim_state is reconcile-derived on the next
      // frame, so assert the intent here (the wait loop below drives frames to kSimulating/data).
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK_EQ((int)gui::g_state.run_intent, (int)gui::RunIntent::kRunning);

      // 5. Reset counter so the wait loop below starts from 0
      // (DeserializeFromJson already did a full reset, but DoRun may trigger callbacks)
      gui::g_state.texture_upload_count = 0;

      // 6. Wait for stable data (texture_upload_count >= 3).
      // rp46 scenes use a 60s timeout because ray path filtering makes photons sparse.
      const int timeout_frames = (ctx->Test->ArgVariant >= 7) ? 60 * 60 : 30 * 60;
      for (int i = 0; i < timeout_frames && gui::g_state.texture_upload_count < 3; ++i) {
        ctx->Yield(1);
      }
      IM_CHECK_GE((int)gui::g_state.texture_upload_count, 3);

      // 7. Stop simulation; ev_auto and snapshot_intensity remain in g_state
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;  // reconcile → kIdle (server gone)

      // 8. Capture exposure parameters
      const float si = gui::g_state.snapshot_intensity;
      const float ev = gui::g_state.ev_auto;
      IM_CHECK_GT(si, 0.0f);

      // 9. Build viewport with auto-EV applied — the actual GUI display state.
      // (The legacy "off" capture at intensity_factor=1.0 was dropped: GUI has no
      // auto-EV toggle anymore, so off is a degenerate non-default state and exercises
      // no code path the auto-EV-applied capture doesn't already cover. See
      // chore-auto-ev-regression-drop-off.)
      // view_proj mirrors app_panels.cpp:742-747 via helper; source mirrors 753-754.
      const auto& rc = gui::g_state.renderer;
      gui::PreviewViewport vp{};
      vp.params.view_proj = gui::BuildPreviewViewProjFromRenderer(rc);
      vp.params.source.max_abs_dz = gui::kDualFisheyeOverlap;
      vp.params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
      const float ev_factor = std::pow(2.0f, ev);
      vp.params.exposure.intensity_factor = ev_factor;
      vp.params.exposure.intensity_scale = ev_factor / si;
      vp.vp_w = scene.render_w;
      vp.vp_h = scene.render_h;

      // Overlay regression scenes (e.g. overlay_ea): override view_proj for marker visibility,
      // precompute zenith/nadir screen positions, and enable the requested overlay flags.
      // Mirrors the runtime path in app_panels.cpp:855-869.
      if (scene.enable_overlay) {
        vp.params.view_proj.lens_type = scene.lens_type;
        vp.params.view_proj.fov = scene.fov;
        vp.params.view_proj.elevation = scene.elevation;
        if (scene.overlay_zenith_nadir) {
          constexpr float kZenithWorldDir[3] = { 0.f, 0.f, -1.f };
          constexpr float kNadirWorldDir[3] = { 0.f, 0.f, 1.f };
          auto zpos = gui::ProjectWorldDirToScreen(vp.params.view_proj, kZenithWorldDir, vp.vp_w, vp.vp_h);
          auto npos = gui::ProjectWorldDirToScreen(vp.params.view_proj, kNadirWorldDir, vp.vp_w, vp.vp_h);
          vp.params.overlay.show_zenith_nadir = true;
          vp.params.overlay.zenith_screen_pos[0] = zpos[0];
          vp.params.overlay.zenith_screen_pos[1] = zpos[1];
          vp.params.overlay.nadir_screen_pos[0] = npos[0];
          vp.params.overlay.nadir_screen_pos[1] = npos[1];
        }
        if (scene.overlay_grid) {
          vp.params.overlay.show_grid = true;
          vp.params.overlay.grid_step = 10.f;
        }
      }

      // 10. Export capture, then compare against reference. The "_on" tag/filename is
      // retained so the existing auto_ev_<scene>_on.jpg references stay valid.
      const std::string path_on = std::string("/tmp/lumice_auto_ev_") + scene.name + "_on.png";
      const std::string ref_on = std::string(LUMICE_TEST_REF_DIR) + "/auto_ev_" + scene.name + "_on.jpg";
      IM_CHECK(RequestAndWaitExport(ctx, vp, path_on));

      // 11. Compare against reference
      IM_CHECK(lumice::test::CheckAgainstReference("auto_ev", (std::string(scene.name) + "_on").c_str(), path_on,
                                                   ref_on, scene.psnr_threshold, g_keep_export_png));
    };
  }
}
