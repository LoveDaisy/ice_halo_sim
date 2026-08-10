// The pixel comparisons that are not part of a regeneratable reference group.
//
// Two subjects, both decided by looking at a rendered frame and nothing else:
//
//   - Committed references. Four crystal-preview scenes (one per crystal kind and shading style
//     the modal offers) and the left panel as a whole. These guard the two GL paths the rest of
//     the suite only ever asserts indirectly: the crystal mesh going through CrystalRenderer's
//     FBO, and the left panel's assembled cards going through ImGui into the default framebuffer.
//     A layout or shading change that no functional assertion is watching lands here first.
//
//   - A self-comparison with no committed reference: run a simulation, save it, reopen it, and
//     confirm the reopened document looks like the live display did. Its "reference" is the frame
//     the same process just exported, which is what lets it assert a property no fixed image can —
//     that two different code paths to the same picture agree.
//
// Why these are not a fifth reference group. The four registered groups
// (scripts/regen_gui_test_refs.py) exist so a deliberate visual change can be re-shot and
// re-calibrated by a driver. These five compare deterministic GL output at the repo's shared 40 dB
// floor, have no per-scene calibrated threshold to maintain, and are updated by copying one file.
// They call lumice::test::CheckAgainstReference like the groups do — same comparison, same stderr
// line — but under a tag the driver's registry does not know, so its PSNR sampling never picks
// them up.
//
// What a user sees when these break: the crystal preview in the edit modal draws the wrong solid,
// or draws it unshaded; the left panel's cards shift or lose a control; a saved file reopens
// looking brighter, darker, or offset from what was on screen when it was saved.

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// The repo-wide floor for a deterministic GL comparison. Not a calibrated mean − 4σ: these scenes
// render pixel-identical run to run, so there is no finite PSNR distribution to take a statistic
// from, and bit-exactness cannot be demanded of an image compared on another machine. The same
// number is used by capture_harness/fullframe and by the modal_layout scenes.
constexpr double kDeterministicThresholdDb = 40.0;

// Rebuild the crystal mesh (if changed) and render it into g_crystal_renderer's FBO so the capture
// below has something to read. Test-only: the left panel stopped driving this FBO every frame when
// the bottom preview was removed, and the modal guard mirrors the in-panel logic it replaced so an
// open edit modal's own FBO content is not overwritten.
void DriveCrystalPreviewFbo() {
  if (gui::IsEditModalOpen()) {
    return;
  }
  if (gui::g_state.layers.empty() || gui::g_state.layers[0].entries.empty()) {
    return;
  }
  const auto& cr = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id];
  const int hash = gui::CrystalParamHash(cr);
  if (hash != gui::g_crystal_mesh_hash) {
    if (gui::BuildAndUploadCrystalMesh(cr, gui::kPreviewFixedSampleSeed) != 0) {
      gui::g_crystal_mesh_hash = hash;
    }
  }
  gui::g_crystal_renderer.Render(gui::g_crystal_rotation, gui::g_crystal_zoom,
                                 static_cast<gui::CrystalStyle>(gui::g_crystal_style));
}

void CrystalCaptureGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_capture.capture_requested && !g_capture.capture_done) {
    DriveCrystalPreviewFbo();
    const int w = gui::g_crystal_renderer.Width();
    const int h = gui::g_crystal_renderer.Height();
    g_capture.pixels =
        lumice::test::ReadTexturePixels(static_cast<unsigned int>(gui::g_crystal_renderer.GetTextureId()), w, h);
    g_capture.width = w;
    g_capture.height = h;
    g_capture.capture_done = true;
  }
}

// One committed crystal-preview reference.
//
// The names carry a `crystal_preview_` prefix because the modal_layout reference group already owns
// scenes called `crystal_prism` and `crystal_pyramid`. Nothing breaks if they collide — the regen
// driver attributes PSNR lines by the `[<group>]` tag, and this suite's tag is `visual` — but
// `--filter crystal_prism` would select both suites, and a stderr log would show two lines a reader
// has to tell apart by their tag alone.
//
// `kKeepDefault` marks the field this scene does not touch, so each row states only what makes it
// different from the default modal preview — a row that set every field would make "which knob is
// this scene about" unreadable.
constexpr int kKeepDefault = -1;
struct CrystalScene {
  const char* name;      // registration name, stderr tag, and capture filename stem
  const char* ref_file;  // under test/gui/references/
  int type;              // gui::CrystalType, or kKeepDefault
  int style;             // gui::g_crystal_style (CrystalStyle index), or kKeepDefault
};
const CrystalScene kCrystalScenes[] = {
  // The default modal preview: a prism under Hidden Line. It is the scene every OTHER assertion in
  // the suite implicitly assumes is drawing something, so it is the one worth pinning first.
  { "crystal_preview_prism", "crystal_prism_default.jpg", kKeepDefault, kKeepDefault },
  // The other crystal kind. Its wedge geometry is generated by a different closed-form path than
  // the prism's, and nothing else in this suite looks at the result.
  { "crystal_preview_pyramid", "crystal_pyramid_default.jpg", static_cast<int>(gui::CrystalType::kPyramid),
    kKeepDefault },
  // The two shading styles that differ most from Hidden Line: one draws edges only, the other
  // fills faces. Between them they cover both halves of the renderer's draw path.
  { "crystal_preview_wireframe", "crystal_wireframe.jpg", kKeepDefault,
    static_cast<int>(gui::CrystalStyle::kWireframe) },
  { "crystal_preview_shaded", "crystal_shaded.jpg", kKeepDefault, static_cast<int>(gui::CrystalStyle::kShaded) },
};
constexpr int kCrystalSceneCount = sizeof(kCrystalScenes) / sizeof(kCrystalScenes[0]);

// A capture full of zeros compares as a black image, and against a mostly-dark reference that can
// clear a PSNR threshold. Every capture below therefore passes this gate first, so a readback that
// silently returned nothing fails saying so rather than passing quietly.
bool AnyNonZero(const std::vector<unsigned char>& pixels) {
  for (unsigned char v : pixels) {
    if (v != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

void RegisterPreviewPixelTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kCrystalSceneCount; ++idx) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", kCrystalScenes[idx].name);
    t->GuiFunc = CrystalCaptureGuiFunc;
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const CrystalScene& scene = kCrystalScenes[ctx->Test->ArgVariant];
      ResetTestState();
      ctx->Yield();

      if (scene.type != kKeepDefault && !gui::g_state.layers.empty() && !gui::g_state.layers[0].entries.empty()) {
        gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].type =
            static_cast<gui::CrystalType>(scene.type);
        gui::g_crystal_mesh_hash = -1;  // force the rebuild the changed type needs
      }
      if (scene.style != kKeepDefault) {
        gui::g_crystal_style = scene.style;
      }
      // Three frames: the state change, the mesh rebuild it triggers, and the FBO render.
      ctx->Yield(3);

      g_capture.Reset();
      g_capture.capture_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_capture.capture_done);
      IM_CHECK_EQ(static_cast<int>(g_capture.pixels.size()), g_capture.width * g_capture.height * 4);
      IM_CHECK(AnyNonZero(g_capture.pixels));

      const std::string tmp_path = GuiTestTempPath(std::string("lumice_") + scene.name + ".png").string();
      const std::string ref_path = std::string(LUMICE_TEST_REF_DIR "/") + scene.ref_file;
      auto rgb = lumice::test::StripAlpha(g_capture.pixels.data(), g_capture.width, g_capture.height);
      IM_CHECK(lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_capture.width, g_capture.height, 3));
      IM_CHECK(lumice::test::CheckAgainstReference("visual", scene.name, tmp_path, ref_path, kDeterministicThresholdDb,
                                                   g_keep_export_png));
    };
  }

  // The left panel as a whole, read back out of the DEFAULT framebuffer rather than an FBO — the
  // only committed reference in the repo that covers the cards, their spacing and their chrome as
  // ImGui actually lays them out.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "left_panel");
    // No GuiFunc: the capture happens in the main loop's post-RenderDrawData hook.
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      g_left_panel_capture.Reset();

      // Park the cursor off-screen. A highlighted card baked into the reference would make every
      // later no-hover run fail, which is how this reference was first captured wrong.
      ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
      ctx->Yield(3);

      g_left_panel_capture.requested.store(true);
      for (int i = 0; i < 10 && !g_left_panel_capture.done.load(); ++i) {
        ctx->Yield(1);
      }
      IM_CHECK(g_left_panel_capture.done.load());
      IM_CHECK(AnyNonZero(g_left_panel_capture.pixels));
      fprintf(stderr, "[visual] left_panel: captured size = %dx%d\n", g_left_panel_capture.width,
              g_left_panel_capture.height);

      const std::string tmp_path = GuiTestTempPath("lumice_left_panel.png").string();
      const std::string ref_path = std::string(LUMICE_TEST_REF_DIR "/left_panel_default.jpg");
      auto rgb = lumice::test::StripAlpha(g_left_panel_capture.pixels.data(), g_left_panel_capture.width,
                                          g_left_panel_capture.height);
      IM_CHECK(lumice::test::SavePng(tmp_path.c_str(), rgb.data(), g_left_panel_capture.width,
                                     g_left_panel_capture.height, 3));
      IM_CHECK(lumice::test::CheckAgainstReference("visual", "left_panel", tmp_path, ref_path,
                                                   kDeterministicThresholdDb, g_keep_export_png));
    };
  }

  // End-to-end: run a simulation, save it, reopen it, and compare the reopened preview against the
  // live one. The regression that put this here was PostSnapshot exporting with the EV captured at
  // CommitConfig time rather than the EV on screen, so the file reopened at a different brightness
  // than the display it was saved from.
  //
  // REAL-TIMING TEST: the preview only converges as the background simulation accumulates rays in
  // real wall-clock time. build.sh runs it in the isolated real-timing pool, NOT the --fixed-dt
  // pool. The wait below is gated on accumulated rays, so fixed-dt no longer starves it outright;
  // what keeps it here is that its bound is a real-time deadline, and --fixed-dt decouples the
  // engine watchdog's simulated clock from that deadline (the trap documented at length in
  // test_gui_lens_projection.cpp). If this test is renamed, update the --filter lists in
  // scripts/build.sh.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "visual", "save_open_visual_consistency");
    t->GuiFunc = [](ImGuiTestContext* /*ctx*/) {
      if (g_capture.capture_requested && !g_capture.capture_done) {
        gui::g_preview.UploadTexture(g_capture.pixels.data(), g_capture.width, g_capture.height);
        g_capture.capture_done = true;
      }
      if (g_export_test.export_requested && !g_export_test.export_done) {
        g_export_test.export_result =
            gui::ExportPreviewPng(g_export_test.export_path, gui::g_preview, gui::g_preview_vp);
        g_export_test.export_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // ev_auto and snapshot_intensity are runtime-only state (not persisted to .lmc) and they
      // decay to 0 on reload. The Phase 2 capture uses the live sim's snapshot_intensity (carried
      // in `live_snapshot_intensity` below); Phase 5 must restore the same value so intensity_scale
      // matches across the two captures. We also clear ev_auto in both phases so the manual EV
      // slider (exposure_offset = 0) is the only EV contribution.

      // --- Phase 1: Start simulation and wait for texture data ---
      gui::g_server = LUMICE_CreateServer();
      gui::g_state.sim.infinite = true;
      gui::g_state.sim.max_hits = 8;
      {
        auto& r = gui::g_state.renderer;
        r.lens_type = 0;  // Linear
        r.fov = 120.0f;
        r.sim_resolution_index = 0;
        r.visible = 2;
        r.exposure_offset = 0.0f;
      }
      gui::DoRun(/*user_initiated=*/true);

      // Accumulate a fixed WORKLOAD (simulated rays), not a fixed number of frames.
      //
      // What Phase 6 compares is two Monte-Carlo estimates of the same image: screenshot A is the
      // poller's last uploaded preview texture, screenshot B is the .lmc texture that
      // RefreshCpuTextureForSave() (app.cpp) takes fresh from the server. Their difference is
      // sampling noise, which falls as 1/sqrt(N) in the accumulated ray count N behind them.
      // Measured on this case over a 6.5x range of N: PSNR = 10*log10(N) - 32.0 dB, +/-0.7.
      //
      // The predecessor of this loop was `for (i < 30) ctx->Yield();`, which fixes the WALL CLOCK
      // rather than N: the real-timing pool keeps the ~16.7ms frame-limit sleep, so 30 Yields is a
      // hard ~575ms budget (measured invariant under load). N is then whatever CPU the simulation
      // threads happened to win inside that window — 2.4M rays idle, 0.37M rays under CPU
      // contention. That is a ~7 dB PSNR swing (31.6 dB -> 24.5 dB) against a 28 dB threshold, and
      // it is the whole reason this case went red on a busy machine. Gating on N makes the
      // comparison's noise floor a property of the test rather than of the machine's spare
      // capacity; the wall clock becomes an upper bound instead of the convergence condition.
      //
      // 3M rays puts the expected PSNR near 32.5 dB — over 4 dB of headroom above kPsnrThreshold —
      // and costs ~0.73s idle (the old fixed budget already bought ~2.4M, so this is not a
      // meaningful slowdown of the idle path). The deadline is a hang bound, NOT a budget: the
      // target was reached in 0.73s idle and in 1.7-1.8s under enough CPU contention to
      // oversubscribe every core, so 20s leaves an order of magnitude. It also deliberately stays
      // under ImGuiTestEngine's 30s default ConfigWatchdogWarning (imgui_te_engine.h): this case
      // runs WITHOUT --fixed-dt, so the watchdog's simulated clock is the wall clock here, and
      // keeping the deadline below it is what lets this test skip the watchdog-widening dance that
      // the --fixed-dt cases need.
      constexpr unsigned long long kMinAccumulatedRays = 3'000'000;
      // Zeroed here rather than trusted from ResetTestState(): a stale count carried over from the
      // previous test would satisfy the wait on its very first frame, and the capture below would
      // then read whatever texture happened to still be bound.
      gui::g_state.stats_sim_ray_num = 0;
      gui::g_state.texture_upload_count = 0;
      const auto accumulate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
      while (((unsigned long long)gui::g_state.stats_sim_ray_num < kMinAccumulatedRays ||
              gui::g_state.texture_upload_count == 0) &&
             std::chrono::steady_clock::now() < accumulate_deadline) {
        ctx->Yield();
      }
      IM_CHECK_GE((unsigned long long)gui::g_state.stats_sim_ray_num, kMinAccumulatedRays);
      IM_CHECK_GT((int)gui::g_state.texture_upload_count, 0);
      // Captured now, not read again in Phase 6: LoadLmcFile deserializes a whole GuiState over
      // g_state, which resets the live counters back to zero.
      const unsigned long long accumulated_rays = gui::g_state.stats_sim_ray_num;
      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK(gui::g_preview_vp.vp_w > 0);
      IM_CHECK(gui::g_preview_vp.vp_h > 0);

      // --- Phase 2: Stop simulation, then capture (so Save uses same data as display) ---
      gui::DoStop();
      ctx->Yield(5);  // Let the final poller sync complete

      // ExportPreviewPng reads gui::g_preview_vp.params, which is refreshed only by the panel
      // render — clear ev_auto first, Yield(1) to bake the new params, then trigger the export.
      // Capture snapshot_intensity for the Phase 5 replay.
      const std::string path_a = GuiTestTempPath("lumice_save_visual_A.png").string();
      g_export_test.Reset();
      g_export_test.export_path = path_a;
      gui::g_state.ev_auto = 0.0f;
      ctx->Yield(1);
      const float live_snapshot_intensity = gui::g_state.snapshot_intensity;
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // --- Phase 3: Save ---
      const std::string lmc_path = GuiTestTempPath("lumice_save_visual_test.lmc").string();
      gui::g_state.current_file_path = lmc_path;
      gui::DoSave();

      // --- Phase 4: Destroy server and Open the saved file ---
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      // A loaded .lmc result follows (tex non-empty) → intent kLoaded so the reconcile keeps the
      // preview in a kDone state instead of the stale kRunning intent flipping it to kSimulating.
      gui::g_state.run_intent = gui::RunIntent::kLoaded;

      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(lmc_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);
      IM_CHECK(!tex_data.empty());

      // Upload loaded texture (GuiFunc will handle GL)
      g_capture.Reset();
      g_capture.pixels = tex_data;
      g_capture.width = tex_w;
      g_capture.height = tex_h;
      g_capture.capture_requested = true;
      ctx->Yield(2);

      // --- Phase 5: Capture loaded preview (screenshot B) ---
      const std::string path_b = GuiTestTempPath("lumice_save_visual_B.png").string();
      g_export_test.Reset();
      g_export_test.export_path = path_b;
      gui::g_state.ev_auto = 0.0f;
      gui::g_state.snapshot_intensity = live_snapshot_intensity;
      ctx->Yield(1);
      g_export_test.export_requested = true;
      ctx->Yield(2);
      IM_CHECK(g_export_test.export_done);
      IM_CHECK(g_export_test.export_result);

      // --- Phase 6: Compare screenshots A and B ---
      std::vector<unsigned char> img_a, img_b;
      int wa = 0, ha = 0, ca = 0;
      int wb = 0, hb = 0, cb = 0;
      IM_CHECK(lumice::test::LoadPng(path_a.c_str(), img_a, wa, ha, ca));
      IM_CHECK(lumice::test::LoadPng(path_b.c_str(), img_b, wb, hb, cb));
      IM_CHECK_EQ(wa, wb);
      IM_CHECK_EQ(ha, hb);

      // Compare in RGB (strip alpha if needed)
      std::vector<unsigned char> rgb_a, rgb_b;
      if (ca == 4) {
        rgb_a = lumice::test::StripAlpha(img_a.data(), wa, ha);
      } else {
        rgb_a = img_a;
      }
      if (cb == 4) {
        rgb_b = lumice::test::StripAlpha(img_b.data(), wb, hb);
      } else {
        rgb_b = img_b;
      }

      // PSNR threshold: the save path gets a fresh snapshot from the server (slightly more
      // accumulated data than the poller's last upload), plus float→uint8 quantization (~48 dB
      // theoretical max). 28 dB catches major conversion errors (wrong EV, wrong matrix) while
      // allowing for timing noise.
      //
      // Headroom, measured: with the Phase 1 wait gated on kMinAccumulatedRays the observed PSNR is
      // 33.0 dB +/- 0.15 whether the machine is idle or heavily contended, so the margin here is
      // ~5 dB rather than the ~0.2 dB it used to be on a busy machine. Injected defects of the two
      // classes named above land at 22.7 dB (a one-stop EV mismatch baked in at save time) and
      // 20.3 dB (the reloaded image shifted by 3 rows) — both still comfortably red.
      constexpr double kPsnrThreshold = 28.0;
      double psnr = lumice::test::ComputePsnr(rgb_a.data(), rgb_b.data(), wa, ha, 3);
      // The accumulated ray count is printed alongside because it is what sets the noise floor this
      // threshold is measured against: if this case ever fails, "did it reach the ray target?" is
      // the first question, and the answer belongs in the failure output rather than in a rerun.
      fprintf(stderr, "[visual] save_open_visual_consistency: PSNR = %.2f dB (threshold = %.1f dB, rays = %llu)\n",
              psnr, kPsnrThreshold, accumulated_rays);
      IM_CHECK(psnr > kPsnrThreshold);

      std::remove(path_a.c_str());
      std::remove(path_b.c_str());
      std::remove(lmc_path.c_str());
    };
  }
}
