// End-to-end proof that a simulation config makes it through the whole GUI pipeline —
// server -> poller -> texture upload -> preview shader -> off-screen export — and comes back
// as a frame with signal in it.
//
// Why this is a functional test and not a visual one: it compares against nothing. There is no
// reference image, no PSNR, no threshold to calibrate. What it asserts is that the pipeline RAN
// and DELIVERED, which is a behavior, not an appearance. That is also why it survives when the
// pixel-level scenes it replaces do not: those scenes varied the simulation while pinning the
// display, and the simulation axis is covered more strictly (and on every PR, which gui_test is
// not — it is build-only in CI) by test/e2e-correctness/test_smoke.py over the same configs via
// the CLI. The one thing the CLI route cannot see is the GUI pipeline itself, so that is exactly
// and only what this file keeps.
//
// Two configs, chosen rather than exhaustive:
//   multi_scat — the only fixture in this family known to have silently broken before (its
//                second scattering layer once died with prob=0.0 and every mean-image
//                comparison of the day still passed, because a mean reference answers "same as
//                last time?", never "is this right?"). If the multi-layer path is going to go
//                quiet again, it goes quiet here first.
//   color      — the single-layer counterweight, differing in spectrum configuration, so the
//                pair spans the two branches most able to fail silently: multi-layer scattering
//                and multi-wavelength input.
//
// Category "sim_e2e_smoke" avoids the substring "smoke" colliding with gui_test's --filter,
// which is a case-insensitive SUBSTRING match on name OR category: a category of "smoke" would
// also select the gui_smoke and capture_harness cases (see the note in test_gui_capture_smoke.cpp).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "gui/gui_constants.hpp"
#include "include/lumice.h"
#include "test_gui_shared.hpp"

namespace {

struct SimSmokeScene {
  const char* name;
  const char* config_path;
  int render_w;
  int render_h;
  float ray_num_millions;
};

// The export size is deliberately neither square nor 2:1, so a width/height mix-up anywhere in
// the export path fails the size assertion instead of producing a plausible-looking frame.
//
// ray_num_millions is far below what the retired pixel-reference scenes used (0.1875M-0.5M):
// nothing here is compared against a mean image, so there is no Monte-Carlo noise floor to buy
// down — only enough rays for the frame to be visibly populated. Both values are dyadic, so
// ExpectedSimRayNum() can assert the resulting count exactly (see its note in test_gui_shared.hpp).
const SimSmokeScene kScenes[] = {
  { "multi_scat", LUMICE_E2E_CONFIG_DIR "/multi_scatter.json", 160, 96, 0.125f },
  { "color", LUMICE_E2E_CONFIG_DIR "/color.json", 160, 96, 0.125f },
};
constexpr int kSceneCount = sizeof(kScenes) / sizeof(kScenes[0]);

// Lower bound on the share of exported pixels that must carry any light at all. It is a
// structural statement — "the pipeline delivered a populated frame, not an empty or black one" —
// not a calibrated quantity, and it is set an order of magnitude under what these scenes actually
// render so that ordinary changes in sampling, exposure or ray budget never bring it into play. A
// dead poller, a dropped final upload or a shader that samples nothing all land at (or very near)
// zero, which is the distance this bound has to resolve.
constexpr double kMinLitPixelFraction = 0.02;

// Tear the server down on EVERY exit path. IM_CHECK*() expands to `return;`, so a failing
// assertion between server creation and the manual teardown would otherwise leave a running
// server and poller thread for the next registered case to inherit (ResetTestState() only nulls
// the pointer). Deliberately NOT the ScopedServerAndWatchdogGuard from
// test_gui_lens_projection.cpp: that one also widens the test-engine watchdog, which those
// scenes need because they trace 5M rays and this file does not — sharing it would mean
// weakening a watchdog here for a reason that does not apply.
struct ScopedServerGuard {
  bool server_owned = true;

  ~ScopedServerGuard() {
    if (server_owned && gui::g_server != nullptr) {
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;
    }
  }
};

}  // namespace

void RegisterSimE2eSmokeTests(ImGuiTestEngine* engine) {
  for (int idx = 0; idx < kSceneCount; idx++) {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "sim_e2e_smoke", kScenes[idx].name);
    t->ArgVariant = idx;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const auto& scene = kScenes[ctx->Test->ArgVariant];
      ResetTestState();

      // 1. Server up before DoRun; the guard owns its teardown from here on.
      gui::g_server = LUMICE_CreateServer();
      ScopedServerGuard guard;
      LUMICE_SetLogLevel(gui::g_server, static_cast<LUMICE_LogLevel>(g_core_log_level));

      // 2. Load the config exactly as the GUI would (DeserializeFromJson full-resets GuiState).
      {
        std::ifstream in(scene.config_path);
        IM_CHECK(in.is_open());
        std::string json_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        IM_CHECK(gui::DeserializeFromJson(json_str, gui::g_state));
      }

      // 3. Smallest sim resolution, and a finite budget: the e2e configs are sized for a CLI
      // render and some ask for an infinite run, which would never reach COMPLETED.
      gui::g_state.renderer.sim_resolution_index = 0;
      gui::g_state.sim.infinite = false;
      gui::g_state.sim.ray_num_millions = scene.ray_num_millions;

      // 4. Run. DoRun sets the intent; sim_state is reconcile-derived on the next frame.
      gui::DoRun(/*user_initiated=*/true);
      IM_CHECK_EQ((int)gui::g_state.run_intent, (int)gui::RunIntent::kRunning);

      // 5. Wait for the run to complete. kDone means the backend reported
      // LUMICE_LIFECYCLE_COMPLETED for the epoch DoRun just committed, so the budget was traced
      // in full and the final frame was pushed to the texture even if it is too sparse for the
      // preview quality gate (server_poller.cpp, force_final_upload). The deadline is wall-clock
      // rather than an iteration count because --fixed-dt decouples the two: it injects a fixed
      // frame dt and skips the frame-limit sleep, so iterations say nothing about how many real
      // seconds the sim thread has been given. It bounds a hang; it is not a budget.
      const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
      while (gui::g_state.sim_state != gui::GuiState::SimState::kDone &&
             std::chrono::steady_clock::now() < wait_deadline) {
        ctx->Yield(1);
      }
      IM_CHECK_EQ((int)gui::g_state.sim_state, (int)gui::GuiState::SimState::kDone);
      // Assertion 1: the pipeline traced the budget it was given, not some prefix of it.
      IM_CHECK_EQ((unsigned long long)gui::g_state.stats_sim_ray_num, ExpectedSimRayNum(scene.ray_num_millions));

      guard.server_owned = false;
      gui::g_server_poller.Stop();
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
      gui::g_state.run_intent = gui::RunIntent::kNone;  // reconcile -> kIdle (server gone)

      // 6. Build the display viewport from the config's own render settings — no lens override.
      // What is under test is that whatever the config asked for survives the pipeline, so the
      // lens must come from the config rather than from this file.
      const float si = gui::g_state.snapshot_intensity;
      const float ev = gui::g_state.ev_auto;
      IM_CHECK_GT(si, 0.0f);

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

      const std::string tmp_path = std::string("/tmp/lumice_sim_e2e_smoke_") + scene.name + ".png";
      IM_CHECK(RequestAndWaitPreviewExport(ctx, vp, tmp_path));

      // 7. Read the export back off disk. Going through the file (rather than trusting the
      // export's return value) is what makes this end-to-end: a frame that never reached the
      // texture still produces a successfully written, entirely black PNG.
      std::vector<unsigned char> pixels;
      int w = 0;
      int h = 0;
      int channels = 0;
      IM_CHECK(lumice::test::LoadPng(tmp_path.c_str(), pixels, w, h, channels));

      // Assertion 2: the export honored the requested size.
      IM_CHECK_EQ(w, scene.render_w);
      IM_CHECK_EQ(h, scene.render_h);
      IM_CHECK_GE(channels, 3);

      // Assertion 3: the frame carries light over a real share of its area.
      long long lit = 0;
      const long long total = static_cast<long long>(w) * h;
      for (long long i = 0; i < total; ++i) {
        const unsigned char* px = pixels.data() + i * channels;
        if (px[0] != 0 || px[1] != 0 || px[2] != 0) {
          ++lit;
        }
      }
      const double lit_fraction = total > 0 ? static_cast<double>(lit) / static_cast<double>(total) : 0.0;
      fprintf(stderr, "[sim_e2e_smoke] %s: %dx%d, lit pixels %lld/%lld (%.1f%%)\n", scene.name, w, h, lit, total,
              lit_fraction * 100.0);
      IM_CHECK_GT(lit_fraction, kMinLitPixelFraction);

      std::remove(tmp_path.c_str());
    };
  }
}
