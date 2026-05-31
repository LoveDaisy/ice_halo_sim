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
};

// clang-format off
// Per-scene PSNR thresholds are mean − 3σ (floored to 0.5 dB), computed via
// scripts/regen_gui_test_refs.py. Each scene threshold uses min(off, on) so a single
// number covers both modes. Reference images are pixel-averaged means.
//
// Last regenerated in task-land-global-downsample-metric (2026-05-31) after landing the
// auto-EV box-sum downsample (f=8, see kEvAutoDownsampleFactor in gui_ev_auto.hpp).
// Calibration ran at N=10 for the mean-ref (Phase A) via scripts/regen_gui_test_refs.py.
//
// NOTE: regen_gui_test_refs.py runs `LumiceGUITests --filter auto_ev`, which produces
// PSNRs ~1 dB higher than the full-suite invocation (-gtj release) because the full
// suite warms up scheduler/cache state via the other ~240 tests first.  Thresholds
// below are set from full-suite mean-4σ (5 runs) instead of the script's mean-3σ
// recommendation, to absorb the bias and a sample-size safety margin.
static const AutoEvScene kScenes[] = {
  {"halo_22",    LUMICE_E2E_CONFIG_DIR "/halo_22.json",                           256, 256, 18.0},  // min(off=18.0, on=18.5)
  {"multi_scat", LUMICE_E2E_CONFIG_DIR "/multi_scatter.json",                     256, 256, 18.0},  // min(off=18.5, on=19.0) − 0.5 safety
  {"color",      LUMICE_E2E_CONFIG_DIR "/color.json",                             256, 256, 18.5},  // min(off=20.0, on=19.0) − 0.5 safety
  {"pyramid",    LUMICE_E2E_CONFIG_DIR "/pyramid.json",                           256, 256, 18.0},  // min(off=18.0, on=19.5)
  {"cza",        LUMICE_E2E_CONFIG_DIR "/cza.json",                               256, 256, 30.0},  // min(off=30.0, on=33.5)
  {"parhelion",  LUMICE_E2E_CONFIG_DIR "/parhelion.json",                         256, 256, 18.5},  // min(off=20.0, on=18.5)
  {"filters",    LUMICE_E2E_CONFIG_DIR "/filters.json",                           256, 256, 18.5},  // min(off=18.5, on=23.5)
  {"rp46",       LUMICE_E2E_CONFIG_DIR "/raypath_symmetry_4_6.json",              256, 256, 20.0},  // min(off=20.0, on=27.5)
  {"rp46_nof",   LUMICE_E2E_CONFIG_DIR "/raypath_symmetry_4_6_nofilter.json",     256, 256, 18.5},  // min(off=18.5, on=19.5)
};
// clang-format on
static constexpr int kSceneCount = 9;

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

// Compare exported PNG at tmp_path against reference; prints instructions if reference missing.
// Returns true if check passed (or was skipped because both ok), false if test should fail.
static bool CheckAgainstReference(const char* group, const char* tag, const std::string& tmp_path,
                                  const std::string& ref_path, double threshold) {
  std::vector<unsigned char> ref_data;
  int ref_w = 0;
  int ref_h = 0;
  int ref_ch = 0;
  bool loaded = lumice::test::LoadPng(ref_path.c_str(), ref_data, ref_w, ref_h, ref_ch);
  if (!loaded) {
    fprintf(stderr, "[%s] %s: reference not found at %s\n", group, tag, ref_path.c_str());
    fprintf(stderr, "[%s] %s: Copy %s to %s\n", group, tag, tmp_path.c_str(), ref_path.c_str());
    return false;
  }

  std::vector<unsigned char> cap_data;
  int cap_w = 0;
  int cap_h = 0;
  int cap_ch = 0;
  if (!lumice::test::LoadPng(tmp_path.c_str(), cap_data, cap_w, cap_h, cap_ch)) {
    fprintf(stderr, "[%s] %s: failed to load capture from %s\n", group, tag, tmp_path.c_str());
    return false;
  }

  if (cap_w != ref_w || cap_h != ref_h) {
    fprintf(stderr, "[%s] %s: size mismatch cap=%dx%dx%d ref=%dx%dx%d\n", group, tag, cap_w, cap_h, cap_ch, ref_w,
            ref_h, ref_ch);
    return false;
  }

  // When ref is RGB (e.g. JPEG) and capture is RGBA, strip alpha before comparison.
  const unsigned char* cmp_ptr = cap_data.data();
  std::vector<unsigned char> converted;
  int cmp_channels = cap_ch;
  if (ref_ch == 3 && cap_ch == 4) {
    converted = lumice::test::StripAlpha(cap_data.data(), cap_w, cap_h);
    cmp_ptr = converted.data();
    cmp_channels = 3;
  } else if (ref_ch != cap_ch) {
    fprintf(stderr, "[%s] %s: channel mismatch cap=%d ref=%d\n", group, tag, cap_ch, ref_ch);
    return false;
  }

  double psnr = lumice::test::ComputePsnr(cmp_ptr, ref_data.data(), ref_w, ref_h, cmp_channels);
  fprintf(stderr, "[%s] %s: PSNR=%.2f dB (threshold=%.1f dB)\n", group, tag, psnr, threshold);
  if (psnr < threshold) {
    fprintf(stderr, "[%s] %s: PSNR below threshold — possible regression\n", group, tag);
    return false;
  }
  // g_keep_export_png is set by --keep-export-png; scripts/regen_gui_test_refs.py uses this flag.
  if (!g_keep_export_png) {
    std::remove(tmp_path.c_str());
  }
  return true;
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

      // 4. DoRun → triggers SerializeCoreConfig (dual-fisheye override) → starts poller
      gui::DoRun();
      IM_CHECK_EQ((int)gui::g_state.sim_state, (int)gui::GuiState::SimState::kSimulating);

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
      gui::g_state.sim_state = gui::GuiState::SimState::kIdle;

      // 8. Capture exposure parameters
      const float si = gui::g_state.snapshot_intensity;
      const float ev = gui::g_state.ev_auto;
      IM_CHECK_GT(si, 0.0f);

      // 9. Build viewports: same projection, differing only in exposure
      // view_proj mirrors app_panels.cpp:742-747 via helper; source mirrors 753-754.
      const auto& rc = gui::g_state.renderer;
      gui::PreviewViewport vp_off{};
      vp_off.params.view_proj = gui::BuildPreviewViewProjFromRenderer(rc);
      vp_off.params.source.max_abs_dz = gui::kDualFisheyeOverlap;
      vp_off.params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
      vp_off.params.exposure.intensity_factor = 1.0f;
      vp_off.params.exposure.intensity_scale = 1.0f / si;
      vp_off.vp_w = scene.render_w;
      vp_off.vp_h = scene.render_h;

      gui::PreviewViewport vp_on = vp_off;
      const float ev_factor = std::pow(2.0f, ev);
      vp_on.params.exposure.intensity_factor = ev_factor;
      vp_on.params.exposure.intensity_scale = ev_factor / si;

      // 10. Export both off and on captures before checking references.
      // Exports happen first so that both /tmp images are generated even when
      // references are missing (allowing a single-pass reference generation).
      const std::string path_off = std::string("/tmp/lumice_auto_ev_") + scene.name + "_off.png";
      const std::string path_on = std::string("/tmp/lumice_auto_ev_") + scene.name + "_on.png";
      const std::string ref_off = std::string(LUMICE_TEST_REF_DIR) + "/auto_ev_" + scene.name + "_off.jpg";
      const std::string ref_on = std::string(LUMICE_TEST_REF_DIR) + "/auto_ev_" + scene.name + "_on.jpg";
      IM_CHECK(RequestAndWaitExport(ctx, vp_off, path_off));
      IM_CHECK(RequestAndWaitExport(ctx, vp_on, path_on));

      // 11. Compare against references (deferred until both exports are done)
      IM_CHECK(CheckAgainstReference("auto_ev", (std::string(scene.name) + "_off").c_str(), path_off, ref_off,
                                     scene.psnr_threshold));
      IM_CHECK(CheckAgainstReference("auto_ev", (std::string(scene.name) + "_on").c_str(), path_on, ref_on,
                                     scene.psnr_threshold));
    };
  }
}
