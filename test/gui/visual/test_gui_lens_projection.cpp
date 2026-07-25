// Lens-projection pixel regression — a disk-reference baseline for the projection math in
// the preview fragment shader (src/gui/preview_renderer.cpp: fisheyeInverse /
// dualFisheyeInverse / rectangularInverse).
//
// Why this exists: before this suite, lens projections were covered only by source review,
// behavioral tests (switching lens clamps FOV), E2E smoke, and buffer-vs-buffer PSNR. None
// of those reads a committed image, so a refactor of a projection formula could change every
// rendered pixel without turning a test red.
//
// Capture path (deliberate): each scene renders through gui::ExportPreviewPng ->
// RenderExportToRgba (export_fbo_renderer.cpp), which allocates its own RGBA8 FBO at the
// caller-supplied dst_w x dst_h and calls renderer.Render(0, 0, dst_w, dst_h, params). The
// output is therefore independent of window size, panel layout and docking state — the
// on-screen preview viewport is never read. That is what makes these references permanent
// assets rather than something to re-shoot whenever the layout moves.
//
// Category "lens_proj" doubles as the "[lens_proj]" tag CheckAgainstReference prints, which
// is how scripts/regen_gui_test_refs.py attributes PSNR samples to this group in a shared
// full-suite stderr. It must therefore stay unique across groups.

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "include/lumice.h"
#include "test_gui_shared.hpp"

// How a scene installs its projection into PreviewParams.
enum class LensSetup {
  // Override view_proj.{lens_type,fov,elevation} on top of the renderer-derived viewport,
  // mirroring the runtime "user picked a lens in the render panel" path.
  kOverrideViewProj,
  // Delegate to the production export helpers, which overwrite view_proj wholesale with
  // kDualFisheyeExportViewProj / kEquirectExportViewProj (preview_renderer.hpp). These two
  // lenses ignore fov and the view matrix entirely, so overriding fields by hand would be a
  // silent no-op — a false-green lever this suite must not depend on.
  kDualFisheyeExport,
  kEquirectExport,
};

struct LensProjScene {
  const char* name;
  const char* config_path;
  int render_w;
  int render_h;
  double psnr_threshold;
  // Simulated rays to accumulate before capturing. Higher = less Monte-Carlo noise in the
  // capture, which is what limits how small a geometric error the PSNR comparison can
  // resolve; see the note above kScenes[].
  unsigned long long min_rays;
  LensSetup setup;
  // Consumed only when setup == kOverrideViewProj.
  int lens_type = 0;
  float fov = 0.f;
  float elevation = 0.f;
};

// clang-format off
// All scenes share one simulated frame source (halo_22.json: prism crystals, sun at 20°
// altitude) and differ only in how that frame is projected — so a PSNR drop localizes to
// the projection branch, not to the scene.
//
// lens_type/fov/elevation are pinned explicitly rather than inherited from the config's
// render block: this suite must keep testing the equal-area branch even if halo_22.json's
// own lens is later edited for unrelated reasons. elevation=20 matches the sun altitude so
// the 22° halo ring lands near the image center in both single-fisheye scenes.
//
// Output sizes: the single-fisheye scenes use a square frame (the projection inscribes a
// circle in the short edge). The dual-fisheye and equirect scenes use strict 2:1, matching
// the production export convention (app.cpp::DoExportEquirectangularPng) and the shaders'
// short_res = min(width/2, height): at 2:1 the two fisheye circles exactly tile the frame
// and the equirect map spans the full ±180° x ±90° sky. A square frame would only add
// black bands.
//
// Capture timing is gated on accumulated RAYS, not on a texture-upload count as the auto_ev
// scenes are. An upload carries however much data happened to arrive, so the same upload
// count buys different ray counts depending on machine load — measured here, 40 uploads was
// 6.43M rays on an idle machine and 4.56M under load. That leaks load into the noise level
// and therefore into PSNR, inflating the calibrated sigma (a contended 60-run batch measured
// 0.63 dB against 0.22 dB for the same scene run undisturbed) and forcing a threshold too
// loose to catch anything. Gating on rays makes the capture reproducible instead.
//
// min_rays differs per scene because detection power does. The comparison is one noisy
// capture against a smooth 10-run mean, so per-run Monte-Carlo noise sets the PSNR floor,
// and a projection error only moves PSNR by as much of the frame as it disturbs. The
// single-fisheye scenes fill their frame with the halo and resolve a perturbed projection
// easily at 0.4M rays. The two full-sky scenes spread the same structure over a much larger
// frame, leaving most pixels empty sky: at that budget, a lens-scale error that visibly
// stretches the halo ring into an ellipse moved PSNR only ~1.2 dB and did not even cross the
// threshold. They accumulate 5M rays instead, which drops the noise floor far enough for the
// geometric term to dominate, and stays well inside halo_22.json's 10M ray budget.
//
// References are pixel-averaged means of N=10 runs; thresholds come from
// scripts/regen_gui_test_refs.py --group lens_proj (Phase B, mean − 4σ floored to 0.5 dB,
// pooled over 60 runs), with the sampled mean/σ recorded per scene in
// test/gui/references/_thresholds.json and repeated inline below.
static const LensProjScene kScenes[] = {
  // mean 20.29 σ0.216
  {"fisheye_equal_area_120",       LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 19.0,  400000ULL,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeEqualArea,   120.0f, 20.0f},
  // mean 21.24 σ0.237
  {"fisheye_orthographic_180",     LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 20.0,  400000ULL,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeOrthographic, 180.0f, 20.0f},
  // mean 27.74 σ0.058
  {"dual_fisheye_equal_area_full", LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 128, 26.5,  5000000ULL,
   LensSetup::kDualFisheyeExport},
  // mean 26.59 σ0.069
  {"rectangular",                  LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 128, 25.5,  5000000ULL,
   LensSetup::kEquirectExport},
};
// clang-format on
static constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

// Drive the main loop's export hook (test_gui_main.cpp) and wait for the FBO readback.
// Structurally identical to the helper in test_gui_auto_ev.cpp; kept local rather than
// hoisted into test_screenshot.hpp because two 10-line copies are cheaper than an
// abstraction shared by exactly two callers. Promote it when a third one appears.
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

void RegisterLensProjectionTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "lens_proj", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];
      ResetTestState();

      // 1. Server up before DoRun, at the harness-selected core log level.
      gui::g_server = LUMICE_CreateServer();
      LUMICE_SetLogLevel(gui::g_server, static_cast<LUMICE_LogLevel>(g_core_log_level));

      // 2. Load the scene config (DeserializeFromJson does a full GuiState reset internally).
      {
        std::ifstream in(scene.config_path);
        IM_CHECK(in.is_open());
        std::string json_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        IM_CHECK(gui::DeserializeFromJson(json_str, gui::g_state));
      }

      // 3. Smallest sim resolution to keep the suite fast; does not affect the projection
      // under test (the shader resamples the source texture either way).
      gui::g_state.renderer.sim_resolution_index = 0;

      // 4. DoRun sets the intent; sim_state is reconcile-derived on the next frame.
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK_EQ((int)gui::g_state.run_intent, (int)gui::RunIntent::kRunning);

      // 5. Accumulate scene.min_rays simulated rays, then stop the simulation. stats_sim_ray_num
      // tracks the ray count behind the currently displayed texture (app.cpp:1363-1365). Both
      // counters are zeroed here rather than trusted from ResetTestState: a stale ray count
      // left by the previous scene satisfies the wait on the very first frame, and the capture
      // then reads whatever texture happened to still be bound.
      //
      // ImGuiTestEngine's watchdog measures simulated time (frame count * ConfigFixedDeltaTime
      // under --fixed-dt), not wall clock, and its default kill threshold is 60s. The two
      // full-sky scenes need up to 5M rays to reach a usable noise floor (see kScenes[] note),
      // which can take longer than 60 simulated seconds to accumulate — the loop bound below is
      // already 120s, so the watchdog default was simply never raised to match. Widen it only
      // for this wait so a slow-but-legitimate accumulation isn't mistaken for a hung test,
      // then restore it immediately so every other test keeps the tight default.
      ImGuiTestEngineIO* engine_io = ctx->EngineIO;
      const float saved_watchdog_warn = engine_io->ConfigWatchdogWarning;
      const float saved_watchdog_kill = engine_io->ConfigWatchdogKillTest;
      engine_io->ConfigWatchdogWarning = 150.0f;
      engine_io->ConfigWatchdogKillTest = 180.0f;
      gui::g_state.texture_upload_count = 0;
      gui::g_state.stats_sim_ray_num = 0;
      for (int i = 0;
           i < 120 * 60 && (gui::g_state.stats_sim_ray_num < scene.min_rays || gui::g_state.texture_upload_count == 0);
           ++i) {
        ctx->Yield(1);
      }
      engine_io->ConfigWatchdogWarning = saved_watchdog_warn;
      engine_io->ConfigWatchdogKillTest = saved_watchdog_kill;
      IM_CHECK_GE((unsigned long long)gui::g_state.stats_sim_ray_num, scene.min_rays);
      IM_CHECK_GT((int)gui::g_state.texture_upload_count, 0);

      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;  // reconcile → kIdle (server gone)

      // 6. Auto-EV state survives the stop; both fields feed the exposure uniforms below.
      const float si = gui::g_state.snapshot_intensity;
      const float ev = gui::g_state.ev_auto;
      IM_CHECK_GT(si, 0.0f);

      // 7. Build the viewport. source.{max_abs_dz,r_scale} must be set BEFORE the export
      // helpers run: those helpers deliberately leave `source` alone (export_fbo_renderer.cpp),
      // and sampleDualFisheye needs the overlap constants for both the dual-fisheye and the
      // equirect shader, which read the same dual-fisheye source texture.
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

      switch (scene.setup) {
        case LensSetup::kOverrideViewProj:
          vp.params.view_proj.lens_type = scene.lens_type;
          vp.params.view_proj.fov = scene.fov;
          vp.params.view_proj.elevation = scene.elevation;
          break;
        case LensSetup::kDualFisheyeExport:
          gui::ConfigureDualFisheyeExportParams(vp.params);
          break;
        case LensSetup::kEquirectExport:
          gui::ConfigureEquirectExportParams(vp.params);
          break;
      }

      // 8. Off-screen FBO capture, then compare against the committed reference.
      // The tmp filename must match ReferenceGroup.tmp_prefix in scripts/regen_gui_test_refs.py.
      const std::string tmp_path = std::string("/tmp/lumice_lens_proj_") + scene.name + ".png";
      const std::string ref_path = std::string(LUMICE_TEST_REF_DIR) + "/lens_proj_" + scene.name + ".jpg";
      IM_CHECK(RequestAndWaitExport(ctx, vp, tmp_path));

      IM_CHECK(lumice::test::CheckAgainstReference("lens_proj", scene.name, tmp_path, ref_path, scene.psnr_threshold,
                                                   g_keep_export_png));
    };
  }
}
