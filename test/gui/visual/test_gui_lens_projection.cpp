// Lens-projection pixel regression — a disk-reference baseline for the projection math in
// the preview fragment shader (src/gui/preview_renderer.cpp: linearInverse / fisheyeInverse /
// dualFisheyeInverse / rectangularInverse), plus the marker/grid overlay drawn on top of it
// (overlayAuxLines in the same shader) via the one scene that enables it.
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

#include <chrono>
#include <cmath>
#include <fstream>
#include <iterator>
#include <string>

#include "gui/annotation_overlay_cache.hpp"
#include "gui/app.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "include/lumice.h"
#include "test_gui_shared.hpp"
#include "util/color_space.hpp"

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
  // Ray budget for this scene's run, in millions. Higher = less Monte-Carlo noise in the
  // capture, which is what limits how small a geometric error the PSNR comparison can
  // resolve; see the note above kScenes[].
  float ray_num_millions;
  LensSetup setup;
  // Consumed only when setup == kOverrideViewProj.
  int lens_type = 0;
  float fov = 0.f;
  float elevation = 0.f;
  // Controlled exception to this group's "one scene per projection branch" invariant:
  // these three fields serve the single overlay_ea scene, which reuses an already-covered
  // projection branch (fisheye equal-area) to cover a DIFFERENT shader stage — the
  // zenith/nadir markers and coordinate grid overlayAuxLines() draws on top of the
  // projected frame. Adding a projection-branch scene needs none of them; leave them at
  // their defaults, which bypass the overlay block in TestFunc entirely.
  bool enable_overlay = false;
  bool overlay_zenith_nadir = false;
  bool overlay_grid = false;
  // Second controlled exception, on the same footing as the three above: the lens-border scenes
  // reuse an already-covered projection branch to cover a DIFFERENT shader stage — the image-circle
  // outline overlayLensBorder() draws over the projected frame. Independent of enable_overlay: the
  // border is drawn outside the visibility gate that overlayAuxLines sits behind, so putting the
  // two on one flag would stop the reference distinguishing which stage moved.
  bool enable_lens_border = false;
  // Third controlled exception, same footing as the two above: the sky-colour scene reuses an
  // already-covered projection branch to cover a DIFFERENT shader stage — the background addition
  // and the gate it sits behind. Zero here means "black", which is what every scene above renders
  // with, so their committed references are untouched by this field existing.
  float background_srgb[3] = { 0.f, 0.f, 0.f };
  // -1 = inherit whatever the loaded config asked for. Only the sky scene overrides it: with a
  // black background the visibility gate is invisible in the pixels (black either side of it), so
  // no earlier scene had any reason to vary this.
  int visible = -1;
};

// clang-format off
// All scenes share one simulated frame source (halo_22.json: prism crystals, sun at 20°
// altitude) and differ only in how that frame is projected — so a PSNR drop localizes to
// the projection branch, not to the scene. One scene per projection branch of the preview
// fragment shader; the last row (overlay_ea) is the deliberate exception, reusing the
// equal-area branch to cover the marker/grid overlay stage instead (see the overlay fields
// on LensProjScene).
//
// lens_type/fov/elevation are pinned explicitly rather than inherited from the config's
// render block: this suite must keep testing the equal-area branch even if halo_22.json's
// own lens is later edited for unrelated reasons. elevation=20 matches the sun altitude so
// the 22° halo ring lands near the image center in the view-transformed single-lens scenes.
// overlay_ea is the one exception at elevation=45: it inherits the working point the retired
// auto_ev group captured its overlay reference at, where tilting the camera off the sun puts
// the zenith marker and a useful spread of grid lines inside the frame at the same time.
//
// Output sizes: the single-fisheye scenes use a square frame (the projection inscribes a
// circle in the short edge). The dual-fisheye and equirect scenes use strict 2:1, matching
// the production export convention (app.cpp::DoExportEquirectangularPng) and the shaders'
// short_res = min(width/2, height): at 2:1 the two fisheye circles exactly tile the frame
// and the equirect map spans the full ±180° x ±90° sky. A square frame would only add
// black bands.
//
// Each scene configures a finite ray budget and captures the frame its run COMPLETES on,
// rather than watching a counter and stopping the run once it passes a threshold. Both forms
// fix the work behind a capture, but only the completed run fixes it exactly: waiting for a
// counter to cross a bound stops at the first poll that observed the crossing, and how far
// past the bound that poll lands depends on how big a batch the poller happened to pick up.
// The predecessor of both — capturing on a texture-upload count, as the auto_ev scenes did —
// did not fix the work at all: an upload carries however much data happened to arrive, so the
// same upload count bought 6.43M rays on an idle machine and 4.56M under load. That leaked
// load into the noise level and therefore into PSNR, inflating the calibrated sigma (a
// contended 60-run batch measured 0.63 dB against 0.22 dB for the same scene run undisturbed)
// and forcing a threshold too loose to catch anything.
//
// ray_num_millions differs per scene because detection power does. The comparison is one noisy
// capture against a smooth 10-run mean, so per-run Monte-Carlo noise sets the PSNR floor,
// and a projection error only moves PSNR by as much of the frame as it disturbs. The
// single-fisheye scenes fill their frame with the halo and resolve a perturbed projection
// easily at 0.4M rays. The two full-sky scenes spread the same structure over a much larger
// frame, leaving most pixels empty sky: at that budget, a lens-scale error that visibly
// stretches the halo ring into an ellipse moved PSNR only ~1.2 dB and did not even cross the
// threshold. They trace 5M rays instead, which drops the noise floor far enough for the
// geometric term to dominate, and stays well inside halo_22.json's 10M ray budget.
//
// The budgets of the four original scenes are unchanged from the ray counts they previously
// waited for. The two scenes added when the auto_ev group was retired follow the same rule:
// linear is a single-lens scene like its two neighbours and gets their 0.4M; overlay_ea keeps
// the 0.375M its capture was taken at in the retired group, so its reference describes the
// same working point it always did. Every value reaches its intended integer under truncation
// and rounding alike (0.4f * 1e6 is 400000.006; 0.375f and 5.0f are exact dyadic fractions),
// which is what lets ExpectedSimRayNum() assert the resulting ray count exactly — see its note
// in test_gui_shared.hpp before changing any of them.
//
// References are pixel-averaged means of N=10 runs; thresholds come from
// scripts/regen_gui_test_refs.py --group lens_proj (Phase B over 30 full-suite runs, except the
// three noted below), with the sampled mean/σ recorded per scene in
// test/gui/references/_thresholds.json and repeated inline below. The driver's rule is mean − max(4σ, 1.0 dB) floored to 0.5 dB, and on every scene here
// the 1.0 dB floor is what binds: the largest σ measured is ~0.11 dB, so 4σ never reaches it and
// each threshold sits 10σ or more below its mean. Read a red result accordingly — at that
// distance it is not run-to-run noise.
// THREE ROWS CARRY (N=10) AND A DIFFERENT REFERENCE VINTAGE: linear, fisheye_orthographic_180
// and rectangular. Those are exactly the group's NON-EQUAL-AREA projections, and they were
// re-shot when the preview shader gained the target lens's relative illumination — its per-pixel
// solid angle normalized on axis, which the CLI has always had baked in and the GUI had not (see
// doc/ev-pipeline-architecture.md §7.5, src/gui/preview_jacobian.hpp). Their pixels changed by
// construction: measured against the previous references, the new captures fall off toward the
// frame edge on linear (ratio 0.973 at the centre against 0.422 at 1.3 image radii), brighten
// toward the rim on orthographic (1.05 to 1.74), and darken toward the poles on rectangular
// (0.999 to 0.62) — the natural vignetting of cos^3, 1/cos and cos(lat) respectively.
//
// The other six rows were NOT re-shot, and that is a claim with evidence rather than an omission.
// Every equal-area branch's relative illumination is exactly 1, so those frames are unchanged
// bit-for-bit; the same measurement over fisheye_equal_area_120 finds 0.991 to 1.013 with no
// spatial trend at all, which is the run-to-run noise of a single capture against a 10-run mean.
// Re-shooting them would have replaced a 30-sample calibration with a 10-sample one and swapped
// one noise realization for another, for no change in what is being pinned. So their references,
// their thresholds and their (N=30) provenance stand.
//
// The three that did move were calibrated at N=10 rather than 30, deliberately. The driver's rule
// is mean − max(4σ, 1.0 dB), and on this group the 1.0 dB floor binds for every scene, so the
// sample size reaches the threshold only through the mean — determined here to ±0.03 dB, well
// inside the rule's own 0.5 dB quantization. The invariant that matters is unaffected either way:
// a floored mean − 1.0 dB always leaves at least 1.0 dB of margin, which on these sigmas is 10σ
// or more. Twenty more full-suite runs could have moved a threshold by at most one 0.5 dB step.
static const LensProjScene kScenes[] = {
  // mean 20.030 σ0.1040
  {"fisheye_equal_area_120",       LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 19.0,  0.4f,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeEqualArea,   120.0f, 20.0f},
  // mean 19.033 σ0.0799 (N=10)
  {"fisheye_orthographic_180",     LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 18.0,  0.4f,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeOrthographic, 180.0f, 20.0f},
  // mean 21.749 σ0.1033 (N=10)
  // fov=90 matches the Linear entry in kSingleLens[] (test_render_handedness_guard.cpp), so a
  // suspected linearInverse regression can be cross-read against that deterministic sign pin
  // at the same focal length.
  {"linear",                       LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 20.5,  0.4f,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeLinear,              90.0f, 20.0f},
  // mean 27.666 σ0.0827
  {"dual_fisheye_equal_area_full", LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 128, 26.5,  5.0f,
   LensSetup::kDualFisheyeExport},
  // mean 28.562 σ0.0591 (N=10)
  {"rectangular",                  LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 128, 27.5,  5.0f,
   LensSetup::kEquirectExport},
  // mean 20.734 σ0.0989
  // Overlay scene: same equal-area branch as the first row, tilted to elevation=45 with the
  // zenith/nadir markers and the coordinate grid enabled. It is the only committed pixel
  // coverage of overlayAuxLines(); it moved here from the retired auto_ev group, which had
  // been its sole owner.
  // RE-SHOT when the grid stopped being a shader expression and became a sampled core mask. The
  // curves moved: the analytic version derived its own half-width per fragment from fwidth() of
  // the altitude and azimuth fields, core derives one from the local gradient of the same fields on
  // the CPU, and near the rim of an equal-area frame those two disagree. Sigma is unchanged, so the
  // scene is no noisier than it was; only its operating point moved.
  {"overlay_ea",                   LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 19.5,  0.375f,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeEqualArea,   180.0f, 45.0f,
   /*enable_overlay=*/true, /*overlay_zenith_nadir=*/true, /*overlay_grid=*/true},
  // Lens-border scenes. Each reuses the setup of the scene named in its own name and changes ONE
  // thing — the border switch — so a PSNR drop on the pair localizes to overlayLensBorder() and a
  // drop on both the pair and its twin localizes to the projection underneath. Two of them, one per
  // shader branch that can draw a border: the single-lens radius formula and the dual-fisheye clip
  // circle, which are different code with different inputs.
  //
  // Ray budgets are inherited from the reused scenes for the reason those budgets exist: the border
  // is a thin bright curve, so its contribution to PSNR is small, and lowering the budget here would
  // raise the Monte-Carlo floor the border has to stand out from.
  // mean 20.013 σ0.0918
  {"fisheye_equal_area_120_border", LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 256, 19.0,  0.4f,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeEqualArea,   120.0f, 20.0f,
   /*enable_overlay=*/false, /*overlay_zenith_nadir=*/false, /*overlay_grid=*/false,
   /*enable_lens_border=*/true},
  // mean 27.071 σ0.0464 — 0.6 dB under its borderless twin, which is the border's own contribution
  // to the frame: the two circles are a thin bright curve over an otherwise identical capture.
  {"dual_fisheye_equal_area_full_border", LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 128, 26.0, 5.0f,
   LensSetup::kDualFisheyeExport,
   /*lens_type=*/0, /*fov=*/0.f, /*elevation=*/0.f,
   /*enable_overlay=*/false, /*overlay_zenith_nadir=*/false, /*overlay_grid=*/false,
   /*enable_lens_border=*/true},
  // The sky-colour scene, and the only one in this group whose frame is not black where nothing
  // was imaged. It exists because a background is the one thing that makes the shader's gate
  // VISIBLE: with the black default, "painted the sky where it should not have" and "left it
  // black" render identically, so no reference above can distinguish them.
  //
  // Three regions in one frame, which is why this geometry and not the group's usual square:
  //   * an equal-area fisheye at fov=180 on a 256x192 canvas puts the image circle at 96 px and
  //     the shader's own domain edge at 96*sqrt(2) = 135.8 px, while the corners sit at 160 px —
  //     outside it, and therefore black. On the square frame every other single-lens scene uses,
  //     the domain edge reaches exactly the corner (128*sqrt(2) = 181) and there is no addressable
  //     pixel outside it at all.
  //   * `visible: upper` with the camera on the horizon (elevation 0) puts the discarded half-sky
  //     in the lower half of the disc — black — and the kept half in the upper, painted.
  //   * everything else inside the circle is sky, at the colour below.
  // The same three regions the CLI fixture asserts numerically
  // (test/e2e-correctness/test_background_visible_hemisphere.py); here they are pinned as pixels,
  // including the exact shape of the boundaries between them, which region floors cannot see.
  //
  // The colour is the one the rest of this scrum probes with, so a byte read off this reference is
  // directly comparable with the numbers those suites assert.
  // mean 28.540 σ0.1645
  {"sky_colour_ea_180",            LUMICE_E2E_CONFIG_DIR "/halo_22.json", 256, 192, 27.5,  0.4f,
   LensSetup::kOverrideViewProj, lumice::gui::kLensTypeFisheyeEqualArea,   180.0f, 0.0f,
   /*enable_overlay=*/false, /*overlay_zenith_nadir=*/false, /*overlay_grid=*/false,
   /*enable_lens_border=*/false,
   /*background_srgb=*/{ 0.2f, 0.35f, 0.6f }, /*visible=*/lumice::gui::kVisibleUpper},
};
// clang-format on
static constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

// IM_CHECK*() macros expand to `return;` on failure, so any of them firing between server
// creation and the manual teardown below would skip Stop/Destroy and leak a running server +
// poller thread into the next IM_REGISTER_TEST case (ResetTestState() only nulls the pointer,
// it does not tear one down). The wait for the run to complete has an observed real chance of
// not finishing in time under load (see the note above), which made that early-return path
// newly reachable rather than theoretical. Ordinary `return` still unwinds the C++ stack,
// so a scope guard here is sufficient; disarm it via server_owned once the normal path has torn
// the server down itself, to avoid a double Stop/Destroy.
struct ScopedServerAndWatchdogGuard {
  ImGuiTestEngineIO* engine_io;
  float saved_watchdog_warn;
  float saved_watchdog_kill;
  bool server_owned = true;

  explicit ScopedServerAndWatchdogGuard(ImGuiTestEngineIO* io)
      : engine_io(io), saved_watchdog_warn(io->ConfigWatchdogWarning), saved_watchdog_kill(io->ConfigWatchdogKillTest) {
  }

  ~ScopedServerAndWatchdogGuard() {
    engine_io->ConfigWatchdogWarning = saved_watchdog_warn;
    engine_io->ConfigWatchdogKillTest = saved_watchdog_kill;
    if (server_owned && gui::g_server != nullptr) {
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
    }
  }
};

void RegisterLensProjectionTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "lens_proj", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];
      ResetTestState();

      // 1. Server up before DoRun, at the harness-selected core log level. The guard must be
      // constructed immediately after: every IM_CHECK* below this point can return early, and
      // only the guard's destructor unconditionally tears the server (and any widened watchdog
      // config) back down on that path.
      gui::g_server = LUMICE_CreateServer();
      ScopedServerAndWatchdogGuard guard(ctx->EngineIO);
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

      // 3b. Override the config's own ray budget with this scene's. halo_22.json asks for 10M
      // rays; the run has to be finite AND this size for step 5's wait to end on the working
      // point the references were captured at.
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = scene.ray_num_millions;

      // 4. DoRun sets the intent; sim_state is reconcile-derived on the next frame.
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK_EQ((int)gui::g_state.run_intent, (int)gui::RunIntent::kRunning);

      // 5. Let the run finish, then stop the simulation and capture its final frame.
      // sim_state reaching kDone means the backend reported LUMICE_LIFECYCLE_COMPLETED for the
      // committed epoch (ReconcileSimState, app.cpp) — every ray of the budget traced, and the
      // final frame pushed to the texture even if it is too sparse for the preview quality gate
      // (server_poller.cpp, force_final_upload; PR #240). Nothing needs zeroing beforehand:
      // unlike a counter threshold, a stale value from the previous scene cannot satisfy this
      // wait, because kDone is derived from the lifecycle of the epoch DoRun just committed.
      //
      // The wait below is bounded by real elapsed time, not iteration count: --fixed-dt injects a
      // fixed 1/60s frame dt but also skips the frame-limit sleep (runs "at full speed"), so an
      // iteration cap only approximates a real-time budget if each Yield() costs close to that
      // injected dt. Measured, it does not — the two full-sky scenes normally reach their 5M-ray
      // target in ~1.4s of real time via ~2,000 fast iterations. Under load each iteration can cost
      // much more real time without the sim thread's ray throughput scaling the same way, so a
      // fixed iteration-count cap can exhaust itself long before the sim thread gets the real
      // seconds it needs — this under-load early-exit was the root cause of an observed ~1.7%
      // flake on these two scenes (confirmed by reproducing it under synthetic CPU contention: the
      // old cap hit its bound at 3.8M/5M and 4.0M/5M rays). A wall-clock deadline instead gives the
      // sim thread the real time budget it actually needs, regardless of how fast or slow each GUI
      // iteration runs.
      //
      // ImGuiTestEngine's own watchdog is a second, *independent* timer that measures simulated
      // time (frame count * ConfigFixedDeltaTime), not wall clock — under --fixed-dt those two
      // clocks are decoupled in either direction: unloaded, ~1,400 Yield()s/real-second inflate
      // simulated time far ahead of real time; loaded, simulated time can still outrun a real-time
      // deadline whenever the iteration rate stays merely above 60 Hz while sim throughput does
      // not. Reproduced directly: with only the default widened threshold (180s) the watchdog's own
      // IM_CHECK(false) fired at frame ~10,798 under heavy contention while the wall-clock deadline
      // still had over 100 real seconds left. It is therefore raised far past anything the deadline
      // below could accumulate at any plausible iteration rate — the wall-clock deadline is what
      // actually bounds this wait against a genuine hang; the watchdog is left with no bound to
      // enforce as a result. The guard restores both configs afterward regardless of outcome, so
      // every other test still gets the tight default.
      guard.engine_io->ConfigWatchdogWarning = 50000.0f;
      guard.engine_io->ConfigWatchdogKillTest = 100000.0f;
      const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(170);
      while (gui::g_state.sim_state != gui::GuiState::SimState::kDone &&
             std::chrono::steady_clock::now() < wait_deadline) {
        ctx->Yield(1);
      }
      IM_CHECK_EQ((int)gui::g_state.sim_state, (int)gui::GuiState::SimState::kDone);
      // A completed run traced exactly its configured budget, so this number is identical on
      // every machine and every rerun — that is the property the capture inherits.
      IM_CHECK_EQ((unsigned long long)gui::g_state.stats_sim_ray_num, ExpectedSimRayNum(scene.ray_num_millions));

      guard.server_owned = false;
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

      // 7b. Overlay scenes only (overlay_ea): precompute the zenith/nadir marker positions
      // and turn the grid on. Mirrors the runtime path in app_panels.cpp — the markers are
      // positioned on the CPU by ProjectWorldDirToScreen and handed to the shader as
      // uniforms, so this block is part of what the reference covers, not test scaffolding.
      // Must run after the switch above: it reads the view_proj the scene just installed.
      if (scene.enable_overlay) {
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
          // The grid's geometry comes from core's annotation overlay now, not from a shader
          // uniform, so the reference covers the mask path the live preview and the CLI both use.
          // Computed AT THIS CANVAS for the reason ExportPreviewPng gives for doing the same: a
          // mask built for another size is an image rescale of the curves.
          static gui::AnnotationOverlayCache grid_overlay;
          gui::AnnotationViewInput in;
          in.lens_type = vp.params.view_proj.lens_type;
          in.fov = vp.params.view_proj.fov;
          in.azimuth = vp.params.view_proj.azimuth;
          in.elevation = vp.params.view_proj.elevation;
          in.roll = vp.params.view_proj.roll;
          in.visible = vp.params.view_proj.visible;
          in.front = vp.params.view_proj.front;
          in.overlap = gui::kDualFisheyeOverlap;
          const float step = gui::ComputeGridStep(vp.params.view_proj.fov);
          in.elevation_deg = gui::ComputeGridElevationAngles(step);
          in.longitude_deg = gui::ComputeGridLongitudeAngles(step);
          grid_overlay.Refresh(gui::MakeAnnotationViewKey(in, vp.vp_w, vp.vp_h));
          IM_CHECK(grid_overlay.HasResult());
          IM_CHECK(!grid_overlay.GridMask().empty());
          vp.params.overlay.grid_mask = grid_overlay.GridMask().data();
          vp.params.overlay.grid_mask_w = grid_overlay.Width();
          vp.params.overlay.grid_mask_h = grid_overlay.Height();
          vp.params.overlay.grid_mask_generation = grid_overlay.Generation();
        }
      }

      // 7c. Lens-border scenes. Colour and opacity are left at the OverlayDecoration defaults, so
      // this reference also pins those defaults — changing them is a re-shoot, which is the correct
      // coupling: they are what a user sees the first time the switch is turned on. No CPU
      // precompute of any kind: unlike the markers above, the shader derives the circle itself from
      // the lens, the FOV and the viewport.
      if (scene.enable_lens_border) {
        vp.params.overlay.show_lens_border = true;
      }

      // 7d. Sky colour. Converted here rather than stored linear in the table so the scene row
      // carries the number a colour picker shows — the same spelling the config JSON, the GUI
      // control and every other probe in this scrum use. Applied after the switch above for the
      // same reason 7b is: the visible override belongs to the view_proj the scene just installed.
      lumice::SrgbToLinearRgb(scene.background_srgb, vp.params.background_color_linear);
      if (scene.visible >= 0) {
        vp.params.view_proj.visible = scene.visible;
      }

      // 8. Off-screen FBO capture, then compare against the committed reference.
      // The tmp filename must match ReferenceGroup.tmp_prefix in scripts/regen_gui_test_refs.py.
      const std::string tmp_path = GuiTestTempPath(std::string("lumice_lens_proj_") + scene.name + ".png").string();
      const std::string ref_path = std::string(LUMICE_TEST_REF_DIR) + "/lens_proj_" + scene.name + ".jpg";
      IM_CHECK(RequestAndWaitPreviewExport(ctx, vp, tmp_path));

      IM_CHECK(lumice::test::CheckAgainstReference("lens_proj", scene.name, tmp_path, ref_path, scene.psnr_threshold,
                                                   g_keep_export_png));
    };
  }
}
