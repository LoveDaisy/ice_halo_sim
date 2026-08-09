// One family of four: entering a document must fence the composite the previous one staged.
//
// The four cases pin the same seam across the four document-entry paths (.json import, .lmc
// without baked colors, .lmc with baked colors, New). Clearing the on-screen preview is not
// enough — the poller can still be holding a staged composite snapshot, and the next sync
// re-uploads it, so the user opens a fresh document and sees the previous run's coloured image.
// Three of the four need a GL texture or the harness reset; the fourth is kept beside them rather
// than split off, because the seam they guard is the family, not the individual path.
//
// The colour-window cases that used to share this file (z_order priority across a re-run, and
// Revert re-pushing display state) moved to functional/test_color_window.cpp, where the controls
// they drive live. The 17 frame-independent siblings are in
// test/unit-correctness/gui/test_composite_preview.cpp.

#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#include "gui/app.hpp"                  // g_server_poller / g_preview / DoOpen / DoNew / SyncFromPoller
#include "gui/export_fbo_renderer.hpp"  // RenderExportToRgba for the pixel-level assertion
#include "gui/file_io.hpp"              // BuildExportJsonOrWarn / ExportConfigJson / SaveLmcFile
#include "gui/gui_state.hpp"
#include "gui/server_poller.hpp"
#include "support/scoped_result_frame.hpp"
#include "test_gui_shared.hpp"

namespace {

// Minimal single-prism config in the canonical wire format LUMICE_SceneFromJson reads
// (lowercase "prism", nested "shape"). Mirrors
// MakeSmallSimConfigJson / MakeMinimalConfigJson in test/unit-correctness/server/
// test_c_api.cpp; those fixtures live in a TU-private anon namespace and are
// not linkable across the gui_test target, so we recreate their shape here.
const char* kMonoConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "opacity": 1.0, "intensity_factor": 1.0
  }]
})";

// Same as kMonoConfig plus a match-all red raypath_color class → the
// RenderConsumer's ColoredMask() is non-zero so DoSnapshot Phase-2 produces
// a composite for this generation. AC1 anchor.
const char* kColorConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 200000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [128, 64],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "opacity": 1.0, "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": "dominant",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  }
})";

// task-fix-open-stale-composite-reupload wait-until helper (mirrors test_gui_import_export.cpp).
// Yields the TestFunc coroutine until `condition()` returns true or `max_yields` frames elapse.
// The predicate should mirror the immediately-following IM_CHECK so a real regression still
// surfaces at the same assertion line — bounded, not silent (349.4 教训).
constexpr int kOpenStaleYieldLimit = 60;

template <typename Fn>
void YieldUntilTrue(ImGuiTestContext* ctx, int max_yields, Fn&& condition) {
  for (int i = 0; i < max_yields && !condition(); ++i) {
    ctx->Yield();
  }
}

// task-fix-open-stale-composite-reupload Step 4 AC2 GL-op scaffolding — mirror of the pattern in
// test_gui_import_export.cpp::GlOpTestState/GlOpGuiFunc. Kept file-scope-local so GuiFunc can be a
// plain non-capturing function pointer (ImGuiTestGuiFunc rejects capturing lambdas).
struct FenceGlOpState {
  bool requested = false;
  bool done = false;
  // Output: center-pixel RGB read back from RenderExportToRgba.
  bool export_ok = false;
  unsigned char center_r = 0;
  unsigned char center_g = 0;
  unsigned char center_b = 0;
  int dst_w = 32;
  int dst_h = 16;
  void Reset() { *this = FenceGlOpState{}; }
};

FenceGlOpState g_fence_gl_op;

void FenceExportGuiFunc(ImGuiTestContext*) {
  if (!g_fence_gl_op.requested || g_fence_gl_op.done) {
    return;
  }
  lumice::gui::PreviewParams params;
  lumice::gui::ConfigureEquirectExportParams(params);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // RGB (non-XYZ) mode: texture sampled as-is
  auto rgba = lumice::gui::RenderExportToRgba(lumice::gui::g_preview, params, g_fence_gl_op.dst_w, g_fence_gl_op.dst_h,
                                              std::nullopt);
  g_fence_gl_op.export_ok = !rgba.empty();
  if (g_fence_gl_op.export_ok) {
    const int cx = g_fence_gl_op.dst_w / 2;
    const int cy = g_fence_gl_op.dst_h / 2;
    const size_t off = (static_cast<size_t>(cy) * g_fence_gl_op.dst_w + cx) * 4;
    g_fence_gl_op.center_r = rgba[off + 0];
    g_fence_gl_op.center_g = rgba[off + 1];
    g_fence_gl_op.center_b = rgba[off + 2];
  }
  g_fence_gl_op.done = true;
}

// JSON -> Scene handle -> commit: the only commit path since v4.12 removed the JSON-string
// entry points. The handle is a local — LUMICE_CommitScene reads it as const and keeps no
// reference — so it is destroyed as soon as the commit returns.
LUMICE_ErrorCode CommitJsonConfig(LUMICE_Server* server, const char* json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json, &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

bool RunToIdleWithData(LUMICE_Server* server, const char* json) {
  if (CommitJsonConfig(server, json) != LUMICE_OK) {
    return false;
  }
  for (int waited = 0; waited < 5000; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[2]{};
      lumice::test::ScopedResultFrame frame_xyz(server);
      LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1);
      if (xyz[0].has_valid_data) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

void RegisterCompositePreviewTests(ImGuiTestEngine* engine) {
  // ================================================================================================
  // task-fix-open-stale-composite-reupload: DoOpen/DoNew must fence poller-side staged composite.
  //
  // Owner复验 task-350（ClearTexture GL blank）后仍复现"完成染色后 Open .lmc/.json 画面回到上一次
  // 染色结果"。DIAG 已 empirically 定位机制层根因：poller 侧仍 staged 的旧 composite 快照，DoOpen/
  // DoNew 只 clear g_preview 但没 fence poller，下一次 SyncFromPoller 的 ShouldFireCompositeUpload
  // 两 OR 分支（ShouldUploadPayload / mode_changed）在 Open 重置态下都会为真 → 重传旧场景。
  // 修复：DoOpen (.json / .lmc 两分支) + DoNew 都调 g_server_poller.InvalidateStagedTexture()。
  //
  // 四路测试分层（plan §4 Step 3-5）：
  //   - AC1 (决策层, headless): ShouldFireCompositeUpload fire 1→0（无 GL 上下文，谓词级别）。
  //         现居 test/unit-correctness/gui/test_composite_preview.cpp 的
  //         should_fire_composite_upload_fires_on_stale_staged_snapshot_until_invalidated ——
  //         它零 GL 依赖，是搬去 gui_unit_test 的"无帧"用例之一，不在下面 AC2/AC3 这四个
  //         *_fences_stale_composite 家族成员之列（该家族定义见本文件顶部注释）。
  //   - AC2 (端到端 GL, .json 路径): 主线程 SyncFromPoller 真上屏 → DoOpen(.json) → 断言不再重传，
  //         并用 RenderExportToRgba 像素级读回 fbo 证明帧缓冲本身不再显示彩色场景（plan-review
  //         round 1 Major：`HasTexture()` 不单独作为"非 proxy"证据）。
  //   - AC3 (机制级, .lmc 无 baked / .lmc 有 baked / DoNew): 断言 poller 快照 `payload == nullptr`；
  //         四路共享同一 `InvalidateStagedTexture()` 插入点，像素级已在 AC2 覆盖。

  // AC2: 端到端 GL-context 回归 (.json 路径) — 主线程 SyncFromPoller + RenderExportToRgba 像素级读回。
  //
  // 与 AC1 的区别：AC1 只证谓词（现居 gui_unit_test，见上），AC2 证生产路径（主线程 SyncFromPoller
  // 会真调 UploadTexture / Render GL 调用，`HasTexture()` 前置信号 + fbo 像素级 sample 作为唯一
  // "非 proxy"证据，覆盖 2026-07-11 同日 `fix-clear-texture-gl-stale` 教训："CPU 字段翻转 ≠
  // 帧缓冲真实内容"）。
  ImGuiTest* t_fence_ac2 =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "open_json_fences_stale_composite_after_color_render");
  t_fence_ac2->GuiFunc = FenceExportGuiFunc;
  t_fence_ac2->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    g_fence_gl_op.Reset();

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Seed the GLOBAL poller (not a local instance): production SyncFromPoller reads only from
    // g_server_poller, so we must publish the composite snapshot there for the main thread to
    // observe it. show_composite_preview defaults to true; last_uploaded_texture_serial defaults
    // to 0 → SyncFromPoller will fire UploadTexture on the composite payload.
    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);

    // Frame A: yield until the main-thread SyncFromPoller loop has actually uploaded the composite
    // to GL. HasTexture() is the fast/precondition signal — pixel readback (Frame C) is the
    // non-proxy evidence.
    YieldUntilTrue(ctx, kOpenStaleYieldLimit, [] { return gui::g_preview.HasTexture(); });
    IM_CHECK(gui::g_preview.HasTexture());  // Precondition: composite really on GL, not CPU-only.

    // Write a mono default-state .json (no raypath_color) so DoOpen(.json) hits the JSON-import
    // branch and resets g_state via InitDefaultState — the exact production shape of the bug.
    std::string json;
    IM_CHECK(gui::BuildExportJsonOrWarn(gui::InitDefaultState(), &json, nullptr));
    const std::string tmp_path = GuiTestTempPath("lumice_fence_open_json.json").string();
    IM_CHECK(gui::ExportConfigJson(tmp_path, json));

    // Frame B: DoOpen(.json) runs synchronously from TestFunc coroutine (JSON branch is CPU-only:
    // ClearTexture is deferred-blank per task-350, InvalidateStagedTexture is a mutex-guarded
    // atomic pointer swap — no GL calls). Pre-fix: g_preview cleared but poller still staged →
    // next SyncFromPoller re-uploads → HasTexture flips back to true within a few frames.
    gui::DoOpen(tmp_path);
    // ClearTexture's CPU-side dims are zeroed synchronously; HasTexture must be false now.
    IM_CHECK(!gui::g_preview.HasTexture());

    // Yield a bounded number of frames — pre-fix the main-thread SyncFromPoller would re-upload
    // the staged composite within one or two frames; if HasTexture ever becomes true again the
    // condition triggers and the assertion below fails deterministically at this exact line.
    YieldUntilTrue(ctx, kOpenStaleYieldLimit, [] { return gui::g_preview.HasTexture(); });
    IM_CHECK(!gui::g_preview.HasTexture());  // FIXED: no re-upload.

    // Frame C: pixel-level readback (plan-review round 1 Major — required, not optional). Ask the
    // GuiFunc on the main thread to run RenderExportToRgba and sample the center pixel. kColorConfig
    // renders a red-dominant composite (class 0 is match-all red); after Open + fence, the fbo
    // must NOT be red-dominant — the sim layer should be blanked/uninitialized so no red channel
    // dominance survives. This is the only assertion that directly falsifies "帧缓冲仍显示旧彩色场景".
    g_fence_gl_op.Reset();
    g_fence_gl_op.requested = true;
    YieldUntilTrue(ctx, kOpenStaleYieldLimit, [] { return g_fence_gl_op.done; });
    IM_CHECK(g_fence_gl_op.done);
    IM_CHECK(g_fence_gl_op.export_ok);
    // Red-dominance discriminator: pre-fix the composite was heavily red (class 0 red, class list =
    // {red}, `background:[0,0,0]`), so R would dominate G and B by a wide margin. Post-fix the
    // ClearTexture-driven blank leaves the sim layer at zero → no channel dominance.
    const int r = g_fence_gl_op.center_r;
    const int g = g_fence_gl_op.center_g;
    const int b = g_fence_gl_op.center_b;
    IM_CHECK(!(r > 32 && r > g + 32 && r > b + 32));  // NOT strongly red-dominant = not the stale composite.

    gui::g_server_poller.Stop();
    std::remove(tmp_path.c_str());
    LUMICE_DestroyServer(server);
  };

  // AC3 shared setup: seed the global poller with a real color-config composite snapshot, then
  // invoke each Open/New path and assert the payload has been fenced (dropped to nullptr). Three
  // independent tests, one per path, so failures pinpoint the missing insertion (plan §4 Step 5).
  //
  // These豁免像素级读回 (unlike AC2)：four paths share the same `InvalidateStagedTexture()` seam,
  // AC2 already证明"fence 生效 → 帧缓冲不重传"。这三路只需证明各自"确实调到了" fencing 动作。

  // AC3 sub-1: DoOpen(.lmc) without baked preview.
  ImGuiTest* t_fence_ac3_lmc_no_baked =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "open_lmc_no_baked_fences_stale_composite");
  t_fence_ac3_lmc_no_baked->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    ResetTestState();

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Save a .lmc without baked preview (save_texture=false) — hits the no-baked sub-branch.
    const std::string tmp_path = GuiTestTempPath("lumice_fence_open_lmc_no_baked.lmc").string();
    IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*save_texture=*/false));

    // Seed global poller with the color composite from the running sim.
    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);
    auto snap_pre = gui::g_server_poller.LoadSnapshot();
    IM_CHECK(snap_pre != nullptr);
    IM_CHECK(snap_pre->payload != nullptr);  // Precondition: staged composite really present.
    IM_CHECK(snap_pre->payload->is_composite);

    // DoOpen(.lmc) from TestFunc coroutine — no baked img branch is pure CPU (ClearTexture is
    // deferred blank; LoadLmcFile only touches file/state; InvalidateStagedTexture is thread-safe).
    gui::DoOpen(tmp_path);

    // Fence-mechanism assertion: poller-side payload dropped, serial preserved so a genuinely
    // new future texture still gets a fresh serial.
    auto snap_post = gui::g_server_poller.LoadSnapshot();
    IM_CHECK(snap_post != nullptr);
    IM_CHECK(snap_post->payload == nullptr);
    IM_CHECK_EQ(snap_post->texture_serial, snap_pre->texture_serial);

    gui::g_server_poller.Stop();
    std::remove(tmp_path.c_str());
    LUMICE_DestroyServer(server);
  };

  // AC3 sub-2: DoOpen(.lmc) with baked preview. Open-side calls UploadTexture (GL), so DoOpen must
  // be driven from GuiFunc (main thread) — mirrors test_gui_import_export.cpp's
  // "open_lmc_with_preview_replaces_stale_texture" dispatch pattern.
  static bool s_fence_lmc_baked_done = false;
  static std::string s_fence_lmc_baked_path;
  ImGuiTest* t_fence_ac3_lmc_baked =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "open_lmc_with_baked_fences_stale_composite");
  t_fence_ac3_lmc_baked->GuiFunc = [](ImGuiTestContext*) {
    if (!s_fence_lmc_baked_done && !s_fence_lmc_baked_path.empty()) {
      gui::DoOpen(s_fence_lmc_baked_path);
      s_fence_lmc_baked_done = true;
    }
  };
  t_fence_ac3_lmc_baked->TestFunc = [](ImGuiTestContext* ctx) {
    ResetTestState();
    s_fence_lmc_baked_done = false;
    s_fence_lmc_baked_path.clear();

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Prime a distinctive CPU-side texture and save it as baked into a .lmc (save_texture=true).
    const int baked_w = 12;
    const int baked_h = 6;
    std::vector<unsigned char> baked_pixels(baked_w * baked_h * 3, 0x55);
    gui::g_preview.UpdateCpuTextureData(baked_pixels.data(), baked_w, baked_h);
    const std::string tmp_path = GuiTestTempPath("lumice_fence_open_lmc_baked.lmc").string();
    IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*save_texture=*/true));

    // Seed global poller with the color composite.
    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);
    auto snap_pre = gui::g_server_poller.LoadSnapshot();
    IM_CHECK(snap_pre != nullptr);
    IM_CHECK(snap_pre->payload != nullptr);
    const auto serial_pre = snap_pre->texture_serial;

    // Drive DoOpen from GuiFunc (main-thread GL context available) and wait for it to fire.
    s_fence_lmc_baked_path = tmp_path;
    YieldUntilTrue(ctx, kOpenStaleYieldLimit, [] { return s_fence_lmc_baked_done; });
    IM_CHECK(s_fence_lmc_baked_done);

    // Fence assertion: even on the baked-img sub-branch (which uploads a texture), the poller-side
    // staged composite must still be dropped so a subsequent SyncFromPoller does not overwrite the
    // baked preview with the stale color snapshot.
    auto snap_post = gui::g_server_poller.LoadSnapshot();
    IM_CHECK(snap_post != nullptr);
    IM_CHECK(snap_post->payload == nullptr);
    IM_CHECK_EQ(snap_post->texture_serial, serial_pre);

    gui::g_server_poller.Stop();
    std::remove(tmp_path.c_str());
    LUMICE_DestroyServer(server);
  };

  // AC3 sub-3: DoNew().
  ImGuiTest* t_fence_ac3_new = IM_REGISTER_TEST(engine, "gui_composite_preview", "do_new_fences_stale_composite");
  t_fence_ac3_new->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    ResetTestState();

    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::g_server_poller.ResetGenerationForTest();
    gui::g_server_poller.PollOnceForTest(server);
    auto snap_pre = gui::g_server_poller.LoadSnapshot();
    IM_CHECK(snap_pre != nullptr);
    IM_CHECK(snap_pre->payload != nullptr);
    const auto serial_pre = snap_pre->texture_serial;

    // DoNew is pure CPU — call from TestFunc coroutine directly.
    gui::DoNew();

    auto snap_post = gui::g_server_poller.LoadSnapshot();
    IM_CHECK(snap_post != nullptr);
    IM_CHECK(snap_post->payload == nullptr);
    IM_CHECK_EQ(snap_post->texture_serial, serial_pre);

    gui::g_server_poller.Stop();
    LUMICE_DestroyServer(server);
  };
}
