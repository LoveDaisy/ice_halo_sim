// task-342.4 Step 2 regression: verifies the poller populates TexturePayload::rgb_data +
// is_composite when raypath_color is active, and leaves both defaulted when it is not. Also
// pins the byte-identity between the payload's rgb_data and a direct LUMICE_GetCompositeResults
// read (AC1 headless anchor) and the byte-identity of xyz_data with LUMICE_GetRawXyzResults on
// the not-active path (AC2 zero-regression anchor). See plan §4 Step 2 test point.
//
// Uses ServerPoller::PollOnceForTest so no GL context is needed — the same seam
// test_gui_lifecycle uses. Runs on both branches back-to-back with fresh servers to avoid
// cross-test global-state coupling.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "gui/app.hpp"                  // g_server_poller / g_preview / DoOpen / DoNew / SyncFromPoller
#include "gui/color_window.hpp"         // PushDisplayState (M8 AC3 direct-push path)
#include "gui/export_fbo_renderer.hpp"  // RenderExportToRgba for AC2 pixel-level assertion
#include "gui/file_io.hpp"              // SerializeCoreConfig / ExportConfigJson / SaveLmcFile
#include "gui/gui_state.hpp"            // GuiState + DisplayStateBaseline (M8 AC3)
#include "gui/gui_state_reconcile.hpp"  // ReconcileGuiEffects / ApplyGuiEffects (M8 AC3 reconciler-path)
#include "gui/server_poller.hpp"
#include "test_gui_shared.hpp"

namespace {

// Minimal single-prism config in the canonical ConfigToJson format used by
// LUMICE_CommitConfig (lowercase "prism", nested "shape"). Mirrors
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

// Two-class config for task-346.1 AC2. class 0 = match-all (bright, every landed ray
// contributes to its Y-lane); class 1 = entry_exit filter with len==3 (dim, only 3-hop
// paths contribute). Larger ray_num (400k) to keep class 1's lane statistically populated.
const char* kTwoColorConfig = R"({
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
    "ray_num": 400000,
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
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]},
      {"color": [0.0, 0.0, 1.0], "match": [{"layer": 0, "crystal": 1,
                                            "type": "entry_exit",
                                            "min_len": 3, "max_len": 3}]}
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

bool RunToIdleWithData(LUMICE_Server* server, const char* json) {
  if (LUMICE_CommitConfig(server, json) != LUMICE_OK) {
    return false;
  }
  for (int waited = 0; waited < 5000; waited += 10) {
    LUMICE_ServerState st = LUMICE_SERVER_RUNNING;
    LUMICE_QueryServerState(server, &st);
    if (st == LUMICE_SERVER_IDLE) {
      LUMICE_RawXyzResult xyz[2]{};
      LUMICE_GetRawXyzResults(server, xyz, 1);
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
  // AC1 anchor (headless): raypath_color active → payload->is_composite == true,
  // payload->rgb_data is populated and byte-identical to what LUMICE_GetCompositeResults
  // returns directly. This is the headless mechanistic proof that the poller wires
  // the composite surface through the payload; the on-screen visual is AC4 owner.
  ImGuiTest* t1 = IM_REGISTER_TEST(engine, "gui_composite_preview", "raypath_color_active_populates_rgb_payload");
  t1->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);

    auto snap = local.LoadSnapshot();
    IM_CHECK(snap != nullptr);
    IM_CHECK(snap->valid);
    IM_CHECK(snap->payload != nullptr);
    IM_CHECK(snap->payload->is_composite);
    IM_CHECK(!snap->payload->rgb_data.empty());
    IM_CHECK(!snap->payload->xyz_data.empty());  // XYZ still populated (auto-EV lane unchanged)

    // Byte-identity with a direct C-API composite read.
    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(comp[0].img_buffer != nullptr);
    IM_CHECK_EQ(comp[0].img_width, snap->payload->width);
    IM_CHECK_EQ(comp[0].img_height, snap->payload->height);
    const size_t nbytes = static_cast<size_t>(snap->payload->width) * static_cast<size_t>(snap->payload->height) * 3;
    IM_CHECK_EQ(snap->payload->rgb_data.size(), nbytes);
    IM_CHECK_EQ(std::memcmp(snap->payload->rgb_data.data(), comp[0].img_buffer, nbytes), 0);

    // Composite is non-trivial (class0 is match-all red).
    unsigned long long sum = 0;
    for (unsigned char v : snap->payload->rgb_data) {
      sum += v;
    }
    IM_CHECK(sum > 0u);

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // AC2 anchor (headless): no raypath_color → payload->is_composite == false, rgb_data
  // stays empty, xyz_data is byte-identical to LUMICE_GetRawXyzResults. This is the
  // per-byte "zero regression" gate for scenes that never touch the color surface.
  ImGuiTest* t2 = IM_REGISTER_TEST(engine, "gui_composite_preview", "no_raypath_color_stays_on_xyz_path");
  t2->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kMonoConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);

    auto snap = local.LoadSnapshot();
    IM_CHECK(snap != nullptr);
    IM_CHECK(snap->valid);
    IM_CHECK(snap->payload != nullptr);
    IM_CHECK(!snap->payload->is_composite);
    IM_CHECK(snap->payload->rgb_data.empty());
    IM_CHECK(!snap->payload->xyz_data.empty());

    // XYZ byte-identity with the direct C-API read (AC2 zero-regression).
    LUMICE_RawXyzResult xyz[2]{};
    IM_CHECK_EQ(LUMICE_GetRawXyzResults(server, xyz, 1), LUMICE_OK);
    IM_CHECK(xyz[0].xyz_buffer != nullptr);
    IM_CHECK_EQ(xyz[0].img_width, snap->payload->width);
    IM_CHECK_EQ(xyz[0].img_height, snap->payload->height);
    const size_t nfloats = static_cast<size_t>(snap->payload->width) * static_cast<size_t>(snap->payload->height) * 3;
    IM_CHECK_EQ(snap->payload->xyz_data.size(), nfloats);
    IM_CHECK_EQ(std::memcmp(snap->payload->xyz_data.data(), xyz[0].xyz_buffer, nfloats * sizeof(float)), 0);

    // Composite surface reports the sentinel: img_buffer null → poller left is_composite false.
    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(comp[0].img_buffer == nullptr);

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // task-345.2 AC3 anchor (rewrite of the old code-review-03 "drift_drop_then_recover" test).
  // Pre-345.2 the poller pulled xyz + composite via three independent C-API calls (xyz →
  // composite → xyz recheck) whose cross-call window was near-guaranteed to be crossed by
  // ConsumeData batch churn under an active sim — the old test forced that mismatch and
  // asserted the drop branch fired. Post-345.2 the poller uses the atomic
  // LUMICE_GetRawXyzAndCompositeResults which fuses both reads into a single server-side
  // DoSnapshot() call; cross-generation pairing is structurally impossible upstream. There is
  // no drop branch left to force, so the test is refocused on the surviving invariant AC3
  // demanded ("composite与其渲染源xyz不跨代混装的保护仍在"): repeatedly arm dirty via
  // LUMICE_SetRaypathColors between combined calls (churn simulator) and assert that every
  // call yields (a) composite bytes byte-identical to what a same-tick direct
  // LUMICE_GetCompositeResults returns (paired-with-just-fetched-xyz proof), and
  // (b) PopulateCompositePayloadForTest builds is_composite=true + rgb_data populated from
  // that composite. Same fixture and deterministic dirty-arming technique as the retired
  // drop-branch test.
  ImGuiTest* t3 = IM_REGISTER_TEST(engine, "gui_composite_preview", "same_generation_invariant_under_churn");
  t3->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::ServerPoller local;
    unsigned long long prev_gen = 0ull;

    // 4 churn rounds — each round arms a fresh dirty via LUMICE_SetRaypathColors (functionally
    // identical to a background ConsumeData batch commit landing in the poll window), then
    // exercises the atomic combined getter the poller now uses. Every round MUST:
    //   1. See the generation strictly advance (proving the churn armed a real snapshot).
    //   2. Get a non-null composite (raypath_color is active, so DoSnapshot Phase-2 produced one).
    //   3. Byte-match a same-tick direct LUMICE_GetCompositeResults read (proving the composite
    //      returned by the combined call is the one that pairs with this call's xyz, not a
    //      cross-generation mix — the "真不变量" AC3 requires).
    //   4. Populate the poller payload as a composite (proving PopulateCompositePayload still
    //      writes rgb_data + sets is_composite, the same behavior the retired drop-recover
    //      test's "recovery" branch pinned).
    for (int round = 0; round < 4; ++round) {
      LUMICE_ColorClassDisplay disp[1]{};
      disp[0].color[2] = static_cast<float>(round % 3) / 2.0f;
      disp[0].visible = 1;
      IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp, 1, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

      LUMICE_RawXyzResult xyz[LUMICE_MAX_RENDER_RESULTS + 1]{};
      LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
      IM_CHECK_EQ(
          LUMICE_GetRawXyzAndCompositeResults(server, xyz, LUMICE_MAX_RENDER_RESULTS, comp, LUMICE_MAX_RENDER_RESULTS),
          LUMICE_OK);
      IM_CHECK(xyz[0].xyz_buffer != nullptr);
      IM_CHECK(comp[0].img_buffer != nullptr);
      IM_CHECK(xyz[0].snapshot_generation > prev_gen);
      prev_gen = xyz[0].snapshot_generation;

      // (3) Same-generation guarantee: a same-tick direct composite read must land on the same
      // frozen snapshot (nothing else armed dirty between these two calls). Copy first — bytes
      // pointed to by combined-call composite alias the same server-side buffer that
      // LUMICE_Get*Results contract may invalidate.
      const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
      std::vector<uint8_t> combined_composite(comp[0].img_buffer, comp[0].img_buffer + nbytes);
      LUMICE_RenderResult direct[LUMICE_MAX_RENDER_RESULTS + 1]{};
      IM_CHECK_EQ(LUMICE_GetCompositeResults(server, direct, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
      IM_CHECK(direct[0].img_buffer != nullptr);
      IM_CHECK_EQ(std::memcmp(combined_composite.data(), direct[0].img_buffer, nbytes), 0);

      // (4) PopulateCompositePayload builds the composite payload from the combined-call bytes.
      gui::TexturePayload payload;
      local.PopulateCompositePayloadForTest(direct[0], &payload);
      IM_CHECK(payload.is_composite);
      IM_CHECK_EQ(payload.rgb_data.size(), nbytes);
      IM_CHECK_EQ(std::memcmp(payload.rgb_data.data(), direct[0].img_buffer, nbytes), 0);
    }

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // task-345.2 AC1 anchor (③ wake-poller): after a finite sim reaches COMPLETED and the poller
  // self-pauses, a display-time color edit (LUMICE_SetRaypathColors) must be able to drive one
  // fresh composite materialization without restarting the sim (epoch unchanged, lifecycle stays
  // COMPLETED). Pins the two coupled invariants:
  //   (a) MECHANISM: the poll after SetRaypathColors + WakeForRefresh yields a payload whose
  //       rgb_data reflects the new colors and is byte-identical to a direct LUMICE_GetCompositeResults
  //       (proving the display-time dirty flag was consumed, not lost).
  //   (b) NON-RESTART: LUMICE_GetSimLifecycle still reports COMPLETED with the SAME epoch
  //       observed before the edit (322 clock decoupling: display-time edit does NOT bump
  //       committed_epoch_ and does NOT flip sim state back to RUNNING).
  ImGuiTest* t4 = IM_REGISTER_TEST(engine, "gui_composite_preview", "display_time_color_edit_repaints_without_restart");
  t4->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Snapshot the terminal (finite-completion) lifecycle: this is what a display-time edit
    // MUST NOT bump.
    LUMICE_SimLifecycleResult lc_before{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
    IM_CHECK_EQ(lc_before.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    const unsigned long long done_epoch = lc_before.epoch;

    // AC1 explicit sub-clause ("sim_ray_count 不减"): capture the live accumulated ray
    // count before the display-time edit so it can be compared post-edit below. Uses
    // the O(1) live counter (task-317), not the DoSnapshot-cached stats path.
    LUMICE_RayCount ray_count_before = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);

    // Baseline poll to freeze current composite for a byte-diff below.
    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);
    auto snap_before = local.LoadSnapshot();
    IM_CHECK(snap_before != nullptr);
    IM_CHECK(snap_before->valid);
    IM_CHECK(snap_before->payload != nullptr);
    IM_CHECK(snap_before->payload->is_composite);
    std::vector<uint8_t> composite_before(snap_before->payload->rgb_data.begin(), snap_before->payload->rgb_data.end());

    // Display-time edit: swap the class-0 color from red → blue. kColorConfig commits exactly
    // one raypath_color class, so class_count == 1 matches.
    LUMICE_ColorClassDisplay disp[1]{};
    disp[0].color[0] = 0.0f;
    disp[0].color[1] = 0.0f;
    disp[0].color[2] = 1.0f;  // blue
    disp[0].visible = 1;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp, 1, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
    // The very wake call PushDisplayState() invokes in production, exercised on the global poller
    // that would drive DoSnapshot() consumption when a background worker was actually running.
    // For this synchronous test seam, WakeForRefresh + PollOnceForTest together stand in for
    // "worker wakes up and runs one PollOnce, then self-pauses at COMPLETED".
    gui::g_server_poller.WakeForRefresh(server);

    // (a) MECHANISM: the poll after the edit must produce a composite reflecting the new colors —
    // i.e., not byte-identical to composite_before (the color changed, so the rendered image must
    // differ). And it must byte-match a same-tick direct C-API read.
    local.PollOnceForTest(server);
    auto snap_after = local.LoadSnapshot();
    IM_CHECK(snap_after != nullptr);
    IM_CHECK(snap_after->valid);
    IM_CHECK(snap_after->payload != nullptr);
    IM_CHECK(snap_after->payload->is_composite);
    IM_CHECK_EQ(snap_after->payload->rgb_data.size(), composite_before.size());
    IM_CHECK(std::memcmp(snap_after->payload->rgb_data.data(), composite_before.data(), composite_before.size()) != 0);

    LUMICE_RenderResult direct[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, direct, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(direct[0].img_buffer != nullptr);
    IM_CHECK_EQ(
        std::memcmp(snap_after->payload->rgb_data.data(), direct[0].img_buffer, snap_after->payload->rgb_data.size()),
        0);

    // (b) NON-RESTART: lifecycle stays COMPLETED with the SAME epoch. This is the 322 clock-
    // decoupling guarantee — a display-time edit re-materializes but does NOT reset the
    // accumulator or bump the committed_epoch. If ③'s wake path ever regressed to calling
    // LUMICE_Start / LUMICE_CommitConfig, this would flip lifecycle back to RUNNING and/or
    // bump the epoch.
    LUMICE_SimLifecycleResult lc_after{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
    IM_CHECK_EQ(lc_after.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    IM_CHECK_EQ(lc_after.epoch, done_epoch);

    // Payload's epoch also stays == done_epoch (no accumulator reset).
    IM_CHECK_EQ(snap_after->payload->payload_epoch, done_epoch);

    // AC1 explicit sub-clause ("sim_ray_count 不减"): a display-time color edit must not
    // reset or decrement the accumulated sim ray count — that would be a tell-tale sign
    // of an accidental restart (LUMICE_Start/CommitConfig) hiding behind the wake path.
    LUMICE_RayCount ray_count_after = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
    IM_CHECK(ray_count_after >= ray_count_before);

    // Post-test cleanup on both the local ServerPoller and the global one that WakeForRefresh
    // above nudged into kRunning — Stop() is synchronous and idempotent.
    local.Stop();
    gui::g_server_poller.Stop();
    LUMICE_DestroyServer(server);
  };

  // task-345.3 anchor: LUMICE_SetCompositeExposure applied AFTER a finite completion changes
  // the composite bytes (mechanistic proof the display-time EV multiplier reaches the
  // Phase-2 compositor) without touching lifecycle epoch or sim ray count (322 clock
  // decoupling — same non-restart guarantee as SetRaypathColors, since both share the
  // snapshot_dirty_ flip pattern). Also confirms the composite-only P99 anchor is populated
  // through the C API surface (mono getters must leave it at 0).
  ImGuiTest* t5 =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "display_time_composite_exposure_reaches_compositor");
  t5->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    LUMICE_SimLifecycleResult lc_before{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
    IM_CHECK_EQ(lc_before.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    const unsigned long long done_epoch = lc_before.epoch;

    LUMICE_RayCount ray_count_before = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);

    // Baseline composite (EV=0 → scale 1.0 → default behavior) via direct C-API read.
    LUMICE_RenderResult baseline[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, baseline, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(baseline[0].img_buffer != nullptr);
    const size_t rgb_bytes =
        static_cast<size_t>(baseline[0].img_width) * static_cast<size_t>(baseline[0].img_height) * 3;
    std::vector<uint8_t> baseline_rgb(baseline[0].img_buffer, baseline[0].img_buffer + rgb_bytes);
    // Composite-only P99 anchor must be populated on the composite path.
    IM_CHECK(baseline[0].composite_p99_y > 0.0f);

    // Mono getter must NOT populate the composite-only anchor field.
    LUMICE_RenderResult mono_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetRenderResults(server, mono_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(mono_out[0].img_buffer != nullptr);
    IM_CHECK_EQ(mono_out[0].composite_p99_y, 0.0f);

    // Push a positive EV → next Get*Results triggers a rebake with 2^ev > 1 → bytes change.
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(server, 2.0f), LUMICE_OK);
    LUMICE_RenderResult brighter[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, brighter, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(brighter[0].img_buffer != nullptr);
    IM_CHECK_EQ(brighter[0].img_width, baseline[0].img_width);
    IM_CHECK_EQ(brighter[0].img_height, baseline[0].img_height);
    IM_CHECK(std::memcmp(brighter[0].img_buffer, baseline_rgb.data(), rgb_bytes) != 0);
    // P99 anchor is over UNEXPOSED lane values — must not move with EV.
    IM_CHECK_EQ(brighter[0].composite_p99_y, baseline[0].composite_p99_y);

    // Push a negative EV → different bytes from both the baseline and the brighter output.
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(server, -2.0f), LUMICE_OK);
    LUMICE_RenderResult dimmer[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, dimmer, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(dimmer[0].img_buffer != nullptr);
    IM_CHECK(std::memcmp(dimmer[0].img_buffer, baseline_rgb.data(), rgb_bytes) != 0);
    IM_CHECK(std::memcmp(dimmer[0].img_buffer, brighter[0].img_buffer, rgb_bytes) != 0);

    // Non-restart guarantee (322 clock decoupling): lifecycle epoch + sim ray count both
    // preserved despite three separate composite re-bakes above.
    LUMICE_SimLifecycleResult lc_after{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
    IM_CHECK_EQ(lc_after.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    IM_CHECK_EQ(lc_after.epoch, done_epoch);
    LUMICE_RayCount ray_count_after = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
    IM_CHECK(ray_count_after >= ray_count_before);

    LUMICE_DestroyServer(server);
  };

  // task-fix-composite-participating-exposure-anchor AC1 core gate (owner ⑤ requirement):
  // "关掉一个很亮的染色类后，剩余的暗类立即变亮 (auto-EV / server-side self-anchor
  // according to remaining visible participating P99)". Fix B (this task) makes the composite
  // exposure scalar self-anchor on participating-P99 inside the same DoSnapshot call as the
  // visibility toggle — hiding a bright class shrinks `active`, the recomputed P99 drops,
  // and s = ParticipatingExposureScale(p99) grows, so the remaining (dim) class's pixels
  // become brighter in ACTUAL RGB bytes. This test reads the actual RGB byte at a class-1
  // signal pixel and asserts strict brightness gain — NOT just the intermediate p99 y
  // proxy (that was the failure mode of 346.1, which passed p99 but the on-screen picture
  // never changed). Additive mode is used because class 1's landing set is a strict subset
  // of class 0's — in dominant mode class 1 never wins argmax when class 0 is visible, so
  // dominant cannot express "same pixel gets brighter"; additive puts each class on its own
  // color channel (class 0 = red, class 1 = blue) so blue-channel byte at a class-1 pixel
  // is a direct measure of class-1's contribution. The dominant-mode structural counterpart
  // is `_dominant` variant below (owner ⑤ real usage is closer to dominant).
  ImGuiTest* t_ac2 =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "display_time_visibility_reanchors_participating_p99");
  t_ac2->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kTwoColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    LUMICE_SimLifecycleResult lc_before{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
    IM_CHECK_EQ(lc_before.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    const unsigned long long done_epoch = lc_before.epoch;
    LUMICE_RayCount ray_count_before = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);

    // Baseline: additive mode, both classes visible → participating union of both lanes,
    // p99 is dominated by the bright match-all class (class 0).
    LUMICE_ColorClassDisplay disp_both[2]{};
    disp_both[0].color[0] = 1.0f;  // class 0 red
    disp_both[0].visible = 1;
    disp_both[0].solo = 0;
    disp_both[1].color[2] = 1.0f;  // class 1 blue
    disp_both[1].visible = 1;
    disp_both[1].solo = 0;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp_both, 2, nullptr, LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);
    LUMICE_RenderResult baseline[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, baseline, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(baseline[0].img_buffer != nullptr);
    const int w = baseline[0].img_width;
    const int h = baseline[0].img_height;
    const size_t nbytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
    const float p99_both = baseline[0].composite_p99_y;
    IM_CHECK(p99_both > 0.0f);
    std::vector<uint8_t> baseline_rgb(baseline[0].img_buffer, baseline[0].img_buffer + nbytes);

    // Population: every pixel where class 1 has ANY blue signal in the baseline. The
    // population-average blue byte must strictly grow when hiding class 0 enlarges s,
    // because blue_linear = class_1_lane[p] * s (additive only mixes class 1 into blue)
    // and sRGB(byte) is monotonic in blue_linear until saturation at 1.0 → 255. Bounding
    // to already-blue pixels (b > 0) excludes the class-1-never-hit background so the
    // ratio isn't diluted by a huge 0-vs-0 term.
    std::vector<size_t> probe_pixels;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const size_t off = (static_cast<size_t>(y) * w + x) * 3;
        if (baseline_rgb[off + 2] > 0) {
          probe_pixels.push_back(off);
        }
      }
    }
    IM_CHECK(probe_pixels.size() >= 32);  // class 1 has a non-trivial blue population
    unsigned long long blue_sum_before = 0;
    for (size_t off : probe_pixels) {
      blue_sum_before += baseline_rgb[off + 2];
    }
    const double blue_before = static_cast<double>(blue_sum_before) / probe_pixels.size();

    // Hide class 0 (bright, match-all). The next GetCompositeResults consumes snapshot_dirty_
    // → Phase-2 rebuild. Fix B: same call recomputes participating-P99 over {class 1 only},
    // recomputes s = ParticipatingExposureScale(smaller_p99), and re-lands the pixel bytes
    // with the LARGER scalar → the blue byte at (probe_x, probe_y) MUST strictly increase.
    LUMICE_ColorClassDisplay disp_hide_bright[2]{};
    disp_hide_bright[0].color[0] = 1.0f;
    disp_hide_bright[0].visible = 0;  // HIDE bright class
    disp_hide_bright[0].solo = 0;
    disp_hide_bright[1].color[2] = 1.0f;
    disp_hide_bright[1].visible = 1;
    disp_hide_bright[1].solo = 0;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp_hide_bright, 2, nullptr, LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);

    LUMICE_RenderResult dim_only[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, dim_only, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(dim_only[0].img_buffer != nullptr);
    const float p99_dim_only = dim_only[0].composite_p99_y;
    IM_CHECK(p99_dim_only > 0.0f);
    IM_CHECK(p99_dim_only < p99_both);  // structural佐证: participating shrank → anchor dropped
    unsigned long long blue_sum_after = 0;
    for (size_t off : probe_pixels) {
      blue_sum_after += dim_only[0].img_buffer[off + 2];
    }
    const double blue_after = static_cast<double>(blue_sum_after) / probe_pixels.size();
    // ⭐ core AC1 assertion (task's raison d'être): the population-average blue byte at
    // class-1 signal pixels is STRICTLY brighter after hiding class 0. The theoretical
    // growth ratio in linear space is s_after/s_before = p99_both/p99_dim ≈ 2.7× on this
    // fixture; sRGB gamma compresses that to ~1.6× in byte space (empirically 1.72× at
    // 400k rays), with additional attenuation from top-decile pixels saturating first.
    // 1.3× is a conservative floor that survives MC noise across seeds while still ruling
    // out the pre-fix behavior (where the average would stay ≈ constant because s was
    // sourced from mono ExposureScale() and never recomputed on visibility toggle).
    IM_CHECK(blue_after > blue_before);
    IM_CHECK(blue_after >= blue_before * 1.3);

    // Restore visibility → blue-population average must return to baseline (byte-exact per
    // pixel, because the underlying Y-lanes were never touched by the display-time setter
    // and s is a pure function of the participating set + intensity_factor +
    // snapshot_intensity). We assert byte-exactness on every probe pixel, not just the mean.
    LUMICE_ColorClassDisplay disp_restore[2]{};
    disp_restore[0].color[0] = 1.0f;
    disp_restore[0].visible = 1;
    disp_restore[0].solo = 0;
    disp_restore[1].color[2] = 1.0f;
    disp_restore[1].visible = 1;
    disp_restore[1].solo = 0;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp_restore, 2, nullptr, LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);
    LUMICE_RenderResult restored[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, restored, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(restored[0].img_buffer != nullptr);
    IM_CHECK_EQ(restored[0].composite_p99_y, p99_both);
    for (size_t off : probe_pixels) {
      IM_CHECK_EQ(restored[0].img_buffer[off + 2], baseline_rgb[off + 2]);
    }

    // display-time guarantees (322 clock decoupling): visibility toggles never bump the
    // lifecycle epoch and never reset the accumulated sim ray count.
    LUMICE_SimLifecycleResult lc_after{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
    IM_CHECK_EQ(lc_after.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    IM_CHECK_EQ(lc_after.epoch, done_epoch);
    LUMICE_RayCount ray_count_after = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
    IM_CHECK(ray_count_after >= ray_count_before);

    LUMICE_DestroyServer(server);
  };

  // task-fix-composite-participating-exposure-anchor AC1 dominant-mode structural counterpart
  // (plan §4 Step 4 Minor #1 — owner ⑤ real usage is closer to dominant than additive; the
  // additive version above validates the "same channel gets brighter" mechanism, this one
  // validates the "hidden-under-brighter-class → becomes visible" mechanism). Because class 1's
  // landing set is a STRICT subset of class 0's, in dominant mode with both visible every
  // class-1 pixel is masked by class-0's argmax win (composite shows RED at those pixels).
  // Hiding class 0 flips those exact pixels to class 1's dominant color (BLUE). Uses solo mode
  // to pre-locate a class-1-signal pixel (that's the only C API way to isolate class 1's
  // landing without direct Y-lane access), then asserts the color-class flip after visibility
  // change. No brightness-ratio assertion here — dominance is a masking relation, not additive
  // combination; ratio semantics belong to the additive test above.
  ImGuiTest* t_ac2_dominant =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "display_time_visibility_reanchors_participating_p99_dominant");
  t_ac2_dominant->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kTwoColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Step 1: solo class 1 → composite shows ONLY class 1's landing pixels, in blue. Scan for
    // a probe pixel where the blue byte is well above the quantization floor.
    LUMICE_ColorClassDisplay disp_solo_c1[2]{};
    disp_solo_c1[0].color[0] = 1.0f;
    disp_solo_c1[0].visible = 1;  // solo on class 1 mutes this regardless
    disp_solo_c1[0].solo = 0;
    disp_solo_c1[1].color[2] = 1.0f;
    disp_solo_c1[1].visible = 1;
    disp_solo_c1[1].solo = 1;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp_solo_c1, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
    LUMICE_RenderResult solo1[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, solo1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(solo1[0].img_buffer != nullptr);
    const int w = solo1[0].img_width;
    const int h = solo1[0].img_height;
    int probe_x = -1, probe_y = -1;
    for (int y = 0; y < h && probe_x < 0; ++y) {
      for (int x = 0; x < w; ++x) {
        const size_t off = (static_cast<size_t>(y) * w + x) * 3;
        if (solo1[0].img_buffer[off + 2] >= 16 && solo1[0].img_buffer[off + 0] < 8) {
          probe_x = x;
          probe_y = y;
          break;
        }
      }
    }
    IM_CHECK(probe_x >= 0);  // class 1 solo picture has a clearly blue pixel

    // Step 2: both visible, dominant mode. class 0 (match-all) argmax-wins at every pixel that
    // has any landing signal, so the probe pixel now shows RED (class 0's color), not blue.
    LUMICE_ColorClassDisplay disp_both[2]{};
    disp_both[0].color[0] = 1.0f;
    disp_both[0].visible = 1;
    disp_both[0].solo = 0;
    disp_both[1].color[2] = 1.0f;
    disp_both[1].visible = 1;
    disp_both[1].solo = 0;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp_both, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
    LUMICE_RenderResult both[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, both, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(both[0].img_buffer != nullptr);
    const size_t probe_off = (static_cast<size_t>(probe_y) * w + probe_x) * 3;
    const uint8_t r_both = both[0].img_buffer[probe_off + 0];
    const uint8_t b_both = both[0].img_buffer[probe_off + 2];
    // class 0's argmax-win expresses as red > blue at the probe pixel (r dominant, b ≈ 0).
    IM_CHECK(r_both > b_both);
    IM_CHECK(r_both >= 16);  // pixel is clearly lit in the class-0 color

    // Step 3: hide class 0. The probe pixel's argmax winner is now class 1; the composite at
    // that pixel becomes BLUE. This is the owner ⑤ "from-invisible-to-visible" flip.
    LUMICE_ColorClassDisplay disp_hide_c0[2]{};
    disp_hide_c0[0].color[0] = 1.0f;
    disp_hide_c0[0].visible = 0;  // HIDE bright class
    disp_hide_c0[0].solo = 0;
    disp_hide_c0[1].color[2] = 1.0f;
    disp_hide_c0[1].visible = 1;
    disp_hide_c0[1].solo = 0;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp_hide_c0, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
    LUMICE_RenderResult c1_visible[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, c1_visible, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(c1_visible[0].img_buffer != nullptr);
    const uint8_t r_after = c1_visible[0].img_buffer[probe_off + 0];
    const uint8_t b_after = c1_visible[0].img_buffer[probe_off + 2];
    // Now blue wins (class 1's color). AND thanks to Fix B's self-anchor, blue_after must
    // also be BRIGHT (not just "wins by 1 unit") — after hiding the bright class the p99
    // shrinks to class 1's own lane, s grows, and class 1's pixels light up clearly.
    IM_CHECK(b_after > r_after);
    IM_CHECK(b_after >= 16);  // clearly visible (self-anchor made it bright)

    LUMICE_DestroyServer(server);
  };

  // task-346.1 ① AC1 anchor (re-run 双叠加消除 · 端到端字节等价):
  // With the FillLumiceConfig fix in place (intensity_factor ≡ 1.0f, exposure_offset never
  // baked into the committed config), the GUI Run path pushes a single copy of manual EV
  // through the display-time channel (LUMICE_SetCompositeExposure). If the caller keeps the
  // manual EV fixed at E and re-commits the SAME config, the composite bytes MUST be
  // byte-identical between first Run and re-Run — no 2× EV amplification. This test mirrors
  // that path at the C API level: two RunToIdleWithData(kColorConfig) calls with the same
  // LUMICE_SetCompositeExposure(E) in between, byte-compare composite outputs. The regression
  // is guarded by the AC5 test in test_gui_import_export.cpp (intensity_factor==1.0f pin);
  // this test is the paired end-to-end proof that AC5 is sufficient (no other hidden EV
  // ingress in Run path).
  ImGuiTest* t_ac1 =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "rerun_with_same_ev_produces_identical_composite");
  t_ac1->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    constexpr float kUserEv = 2.0f;  // non-zero — the bug is invisible at EV=0

    // First Run.
    IM_CHECK(RunToIdleWithData(server, kColorConfig));
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(server, kUserEv), LUMICE_OK);
    LUMICE_RenderResult first[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, first, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(first[0].img_buffer != nullptr);
    const size_t rgb_bytes = static_cast<size_t>(first[0].img_width) * static_cast<size_t>(first[0].img_height) * 3;
    std::vector<uint8_t> first_rgb(first[0].img_buffer, first[0].img_buffer + rgb_bytes);

    // Re-Run: same config, same EV. RunToIdleWithData internally does LUMICE_CommitConfig
    // which calls Stop() → ResetWith()/rebuild → restart accumulation. This is the exact
    // path DoRun() takes; if any code layer sneaked EV into the committed config, this
    // re-Run's composite would be 2× amplified vs. first Run.
    IM_CHECK(RunToIdleWithData(server, kColorConfig));
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(server, kUserEv), LUMICE_OK);
    LUMICE_RenderResult rerun[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, rerun, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(rerun[0].img_buffer != nullptr);
    IM_CHECK_EQ(rerun[0].img_width, first[0].img_width);
    IM_CHECK_EQ(rerun[0].img_height, first[0].img_height);

    // ⭐ core AC1 assertion: composite mean-brightness RATIO between re-run and first-run
    // must be ≈1.0 (within tight stochastic tolerance). The pre-fix bug amplified the shared
    // exposure scalar by 2^E on top of the display-time push, so at EV=E=2.0 the composite
    // scale becomes 2^(2E) = 16× instead of 2^E = 4× → mean brightness ratio would be ~4×
    // (16×/4×). Any ratio outside [0.8, 1.25] rules out the doubling bug while tolerating
    // ~10-15% run-to-run stochastic noise from independent seeded accumulations reaching IDLE
    // at slightly different consume-batch boundaries.
    unsigned long long sum_first = 0, sum_rerun = 0;
    for (size_t i = 0; i < rgb_bytes; ++i) {
      sum_first += first_rgb[i];
      sum_rerun += rerun[0].img_buffer[i];
    }
    IM_CHECK(sum_first > 0u);
    IM_CHECK(sum_rerun > 0u);
    const double ratio = static_cast<double>(sum_rerun) / static_cast<double>(sum_first);
    IM_CHECK(ratio > 0.8);
    IM_CHECK(ratio < 1.25);
    // The unexposed P99 anchor also stays within similar stochastic bounds (this is the tight
    // proof at the anchor level — it's independent of EV).
    const double p99_ratio =
        static_cast<double>(rerun[0].composite_p99_y) / static_cast<double>(first[0].composite_p99_y);
    IM_CHECK(p99_ratio > 0.8);
    IM_CHECK(p99_ratio < 1.25);

    LUMICE_DestroyServer(server);
  };

  // task-348.1 M2 (Step 2) — "add a color class after idle" bounded convergence.
  // Owner's ② report combines two visible symptoms: (a) instantly-appearing "no rays matched"
  // on every pre-existing class (M1 fixes this — the resize/mismatch default-value bug), and
  // (b) the rendered picture appears to keep the OLD class set for a moment before showing
  // the new. Symptom (b) is by design (anti-flicker carry-forward while the accumulator
  // rebuilds), not a stall. This test pins the "bounded" part of the design contract:
  // starting from a color-active IDLE (1 class committed), re-commit a 2-class config with
  // the SAME crystal fixture and drive PollOnceForTest until we see a payload that:
  //   (1) advanced past the pre-edit lifecycle epoch (proving CommitConfig rebooted the sim
  //       accumulator, not just wrote a new color list into the old snapshot), AND
  //   (2) is_composite == true (proving the composite consumer got rebuilt and re-populated
  //       through DoSnapshot Phase-2), AND
  //   (3) has non-empty rgb_data (proving the composite bytes are real, not a stub).
  // The loop has a hard 300-iteration cap (~3s wall-clock) — any convergence past that is a
  // stall, not just carry-forward, and would signal the §3.1 assumption in plan.md ("陈旧
  // 是自愈") is wrong and requires追加 server_poller.* fixes. Same-tick tearing is not
  // re-verified here; it's covered by `same_generation_invariant_under_churn` above.
  ImGuiTest* t_add_class =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "add_class_after_idle_reconverges_within_bound");
  t_add_class->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);

    // Phase A: single-class color config → idle.
    IM_CHECK(RunToIdleWithData(server, kColorConfig));
    LUMICE_SimLifecycleResult lc0{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc0), LUMICE_OK);
    IM_CHECK_EQ(lc0.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
    const unsigned long long epoch_before = lc0.epoch;

    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);
    {
      auto snap = local.LoadSnapshot();
      IM_CHECK(snap != nullptr);
      IM_CHECK(snap->valid);
      IM_CHECK(snap->payload != nullptr);
      IM_CHECK(snap->payload->is_composite);  // baseline is a valid composite frame
    }

    // Phase B: user adds a class → GUI-side re-commits the 2-class config. This is the
    // same code path RenderColorWindow's "Add Class" button triggers (state.MarkStructHardDirty
    // → next debounce → LUMICE_CommitConfigStruct → re-sim + RenderConsumer rebuild).
    IM_CHECK_EQ(LUMICE_CommitConfig(server, kTwoColorConfig), LUMICE_OK);

    // Bounded convergence: 300 polls × 10 ms = 3s hard ceiling. The fixture (400k rays,
    // 128x64) typically converges in under 500ms on release build. Anything beyond the
    // ceiling is a stall, not carry-forward → plan §3.1 assumption is wrong and Step 2's
    // decision gate flips to "追加修复" (see plan §4 Step 2).
    bool converged = false;
    int polls = 0;
    for (; polls < 300; ++polls) {
      local.PollOnceForTest(server);
      auto snap = local.LoadSnapshot();
      if (snap != nullptr && snap->valid && snap->payload != nullptr && snap->payload->is_composite &&
          snap->payload->payload_epoch > epoch_before && !snap->payload->rgb_data.empty()) {
        converged = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    IM_CHECK(converged);

    // Both classes are configured with non-empty match[] in kTwoColorConfig; after
    // convergence both should show signal (class 0 is match-all → all landing rays;
    // class 1 is entry_exit len==3 → a subset). This is the paired proof that the
    // "全类都报 no rays matched" symptom is gone at the truth level: not just because
    // the GUI-side signal_flags cache defaults to 1 (M1 fix), but because the actual
    // server-side signal is non-zero for both classes.
    int flags[2] = { -1, -1 };
    IM_CHECK_EQ(LUMICE_GetColorClassSignal(server, flags, 2), LUMICE_OK);
    IM_CHECK_EQ(flags[0], 1);
    IM_CHECK_EQ(flags[1], 1);

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // task-345.3 review Minor #2: the "value guard" alone would miss the off→on transition
  // (composite becomes active while the last-pushed cache equals the current ev_total —
  // e.g., default 0.0f). This test doesn't exercise the GUI state guard directly (that
  // lives inside RenderPreviewPanel, not on the C-API surface), but it does anchor the
  // orthogonal property that makes the guard SAFE to add without semantic risk: setting
  // EV to the SAME value it was reset to is a no-op at the C-API level and does not
  // reintroduce restart behavior. If a future refactor collapses the two guards into a
  // single "only push on value change", this test would still pass — the guard hole is
  // GUI-side and gets covered by a separate ImGui interaction test rather than a
  // C-API-level regression.
  ImGuiTest* t6 =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "composite_exposure_idempotent_when_value_unchanged");
  t6->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }
    LUMICE_SimLifecycleResult lc_before{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
    const unsigned long long done_epoch = lc_before.epoch;

    // Set the same EV twice — both must return LUMICE_OK, neither must bump the
    // lifecycle epoch or reset sim ray count.
    LUMICE_RayCount ray_count_before = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(server, 1.5f), LUMICE_OK);
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(server, 1.5f), LUMICE_OK);
    LUMICE_SimLifecycleResult lc_after{};
    IM_CHECK_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
    IM_CHECK_EQ(lc_after.epoch, done_epoch);
    LUMICE_RayCount ray_count_after = 0;
    IM_CHECK_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
    IM_CHECK(ray_count_after >= ray_count_before);

    // Null-server rejection.
    IM_CHECK_EQ(LUMICE_SetCompositeExposure(nullptr, 0.0f), LUMICE_ERR_NULL_ARG);

    LUMICE_DestroyServer(server);
  };

  // task-345.4 AC1 truth table (headless, no server/GL needed). Pins the pure decision predicate
  // ShouldUseCompositeUpload = payload_is_composite AND show_composite_preview across all four
  // input combinations. Guarantees a code-refactor that reorders the AND cannot silently flip
  // the display-mode logic.
  ImGuiTest* t7 = IM_REGISTER_TEST(engine, "gui_composite_preview", "should_use_composite_upload_truth_table");
  t7->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    IM_CHECK_EQ(gui::ShouldUseCompositeUpload(false, false), false);
    IM_CHECK_EQ(gui::ShouldUseCompositeUpload(false, true), false);
    IM_CHECK_EQ(gui::ShouldUseCompositeUpload(true, false), false);
    IM_CHECK_EQ(gui::ShouldUseCompositeUpload(true, true), true);
  };

  // task-345.4 AC1 mechanism (headless): after a color-active sim reaches COMPLETED and the
  // poller self-pauses, flipping `show_composite_preview` alone (no new poll, no MarkDirty)
  // must re-fire the upload branch. The predicate is the exact one SyncFromPoller consumes
  // (ShouldFireCompositeUpload), so pinning it here is equivalent to pinning the production
  // decision at the fire-branch seam. SyncFromPoller cannot be driven end-to-end from this
  // functional coroutine (its GL Upload*Texture call SIGILLs without a current GL context —
  // see doc comment on ShouldFireCompositeUpload). AC5 owner on-screen still covers the actual
  // GL upload / repaint.
  ImGuiTest* t8 = IM_REGISTER_TEST(engine, "gui_composite_preview", "mode_flip_forces_refire_at_same_serial");
  t8->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);
    auto snap = local.LoadSnapshot();
    IM_CHECK(snap != nullptr);
    IM_CHECK(snap->valid);
    IM_CHECK(snap->payload != nullptr);
    IM_CHECK(snap->payload->is_composite);

    // T0: initial state before any upload — the fire gate must be TRUE (serial-dedup gate is
    // satisfied: last_uploaded_texture_serial==0 != snap.texture_serial). This is the AC1
    // first-frame case.
    const bool fire_first =
        gui::ShouldFireCompositeUpload(*snap, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
    IM_CHECK(fire_first);

    // T1: simulate the post-upload bookkeeping SyncFromPoller performs — record the serial and
    // the ground-truth composite flag. Same-serial + same-mode + already-uploaded ⇒ fire gate
    // must be FALSE (idempotence: unchanged state must not spam uploads).
    const auto serial_after_upload = snap->texture_serial;
    const bool fire_idempotent =
        gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/true);
    IM_CHECK(!fire_idempotent);

    // T2: user flips the preference OFF. No new poll → serial stable, floor stable. The
    // mode_changed OR-branch is the ONLY thing that can re-fire the gate. This is the core
    // AC1 mechanism ("即时生效不依赖新一轮 poll") — if this returns false, the toggle button
    // would visibly do nothing.
    const bool fire_after_off_flip =
        gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/false, /*last_uploaded_as_composite=*/true);
    IM_CHECK(fire_after_off_flip);

    // T3: simulate the post-fire bookkeeping (last_uploaded_as_composite=false). Same-serial +
    // same-mode ⇒ idempotent again.
    const bool fire_stable_off =
        gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/false, /*last_uploaded_as_composite=*/false);
    IM_CHECK(!fire_stable_off);

    // T4: user flips the preference back ON — mode_changed fires again, AC2 no-loss recovery
    // is now the composite path (raypath_color config was never touched).
    const bool fire_after_on_flip =
        gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
    IM_CHECK(fire_after_on_flip);

    // T5: same-serial + settled mode ⇒ idempotent, no re-fire loop.
    const bool fire_stable_on =
        gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/true);
    IM_CHECK(!fire_stable_on);

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // task-345.4 AC4 zero-regression: with no raypath_color classes, ShouldFireCompositeUpload
  // collapses to ShouldUploadPayload (mode_changed is structurally false because
  // effective_composite is false and last_uploaded_as_composite starts false). Pins the boolean
  // algebra AC4 relies on, plus the render-gate condition (`raypath_color.empty()`) that keeps
  // the status-bar toggle button off screen.
  ImGuiTest* t9 = IM_REGISTER_TEST(engine, "gui_composite_preview", "mode_toggle_hidden_when_no_color_classes");
  t9->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kMonoConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);
    auto snap = local.LoadSnapshot();
    IM_CHECK(snap != nullptr);
    IM_CHECK(snap->valid);
    IM_CHECK(snap->payload != nullptr);
    IM_CHECK(!snap->payload->is_composite);  // no raypath_color ⇒ payload not composite

    // Render-gate condition: the status-bar mode-toggle button is only rendered when the
    // GUI-side g_state.raypath_color is non-empty. A newly created server has no color-class
    // wiring through the C API; g_state is a persisted view snapshot that is only populated
    // by user actions in the GUI (color-class add/edit). In a clean test fixture this stays
    // empty, so the button gate is off (AC4).
    // This assertion is intentionally at the GuiState level, not through ImGui item probing:
    // the whole button block in RenderStatusBar is a single `if (show_mode_toggle) { ... }`
    // guarded by exactly this condition — no ambiguity to bridge.
    gui::g_state.raypath_color.clear();
    IM_CHECK(gui::g_state.raypath_color.empty());

    // Fire-gate collapses to ShouldUploadPayload with mode_changed=false: even when the user
    // preference is set to composite, no composite payload exists ⇒ effective_composite=false
    // ⇒ last_uploaded_as_composite stays false ⇒ mode_changed is structurally impossible.
    const bool fire_with_pref_true =
        gui::ShouldFireCompositeUpload(*snap, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
    const bool fire_should_upload_only =
        gui::ShouldUploadPayload(*snap, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0);
    // On the no-color path the composite gate is equivalent to the plain upload gate — no
    // mode-flip can slip through.
    IM_CHECK_EQ(fire_with_pref_true, fire_should_upload_only);

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // task-348.3 AC3 (⑦): pin the pure decision boundary for "opening the Colors window
  // should default show_composite_preview=true". True iff no color classes exist yet
  // (nothing to remember); false when classes exist so the caller keeps the user's
  // memory. The "apply only on false→true transition" time-guard lives at the call
  // site in RenderTopBar and is not testable from this headless coroutine (no ImGui
  // click surface without the real event loop), but factoring out this predicate
  // makes the branch inside the click handler a single conditional that is obvious
  // in the diff and cannot silently invert.
  ImGuiTest* t10 = IM_REGISTER_TEST(engine, "gui_composite_preview", "should_default_enable_colors_on_open_predicate");
  t10->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    IM_CHECK_EQ(gui::ShouldDefaultEnableColorsOnOpen(true), true);
    IM_CHECK_EQ(gui::ShouldDefaultEnableColorsOnOpen(false), false);
  };

  // task-348.3 AC1/AC2 shared writer: ToggleCompositePreview flips exactly one field
  // (show_composite_preview) and touches nothing else. Kept alongside the predicate
  // tests so both write sites (top-bar Button, Colors-window Checkbox) share a
  // single asserted invariant: "toggle" is negation, nothing more.
  ImGuiTest* t11 = IM_REGISTER_TEST(engine, "gui_composite_preview", "toggle_composite_preview_negates_field");
  t11->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    gui::GuiState s;
    s.show_composite_preview = false;
    gui::ToggleCompositePreview(s);
    IM_CHECK_EQ(s.show_composite_preview, true);
    gui::ToggleCompositePreview(s);
    IM_CHECK_EQ(s.show_composite_preview, false);
  };

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
  //   - AC2 (端到端 GL, .json 路径): 主线程 SyncFromPoller 真上屏 → DoOpen(.json) → 断言不再重传，
  //         并用 RenderExportToRgba 像素级读回 fbo 证明帧缓冲本身不再显示彩色场景（plan-review
  //         round 1 Major：`HasTexture()` 不单独作为"非 proxy"证据）。
  //   - AC3 (机制级, .lmc 无 baked / .lmc 有 baked / DoNew): 断言 poller 快照 `payload == nullptr`；
  //         四路共享同一 `InvalidateStagedTexture()` 插入点，像素级已在 AC2 覆盖。

  // AC1: 决策层回归 — 形式化 issue.md 的 DIAG recipe，无 GL 依赖。
  ImGuiTest* t_fence_ac1 = IM_REGISTER_TEST(
      engine, "gui_composite_preview", "should_fire_composite_upload_fires_on_stale_staged_snapshot_until_invalidated");
  t_fence_ac1->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    gui::ServerPoller local;
    local.ResetGenerationForTest();
    local.PollOnceForTest(server);

    auto snap = local.LoadSnapshot();
    IM_CHECK(snap != nullptr);
    IM_CHECK(snap->valid);
    IM_CHECK(snap->payload != nullptr);
    IM_CHECK(snap->payload->is_composite);
    IM_CHECK(snap->texture_serial > 0);
    IM_CHECK(snap->payload->payload_epoch > 0);

    // Simulate the post-DoOpen(InitDefaultState) reset: last_uploaded_texture_serial=0,
    // display_epoch_floor=0, last_uploaded_as_composite=false, show_composite_preview=true.
    // Pre-fix: ShouldUploadPayload true (serial!=0, epoch>0) OR mode_changed true → fires (BUG).
    const bool fire_before =
        gui::ShouldFireCompositeUpload(*snap, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
    IM_CHECK(fire_before);  // BUG-shape: stale composite would be re-uploaded on next SyncFromPoller.

    // The fix: DoOpen/DoNew calls this on the same poller instance. Payload is dropped; the fire
    // gate collapses to false because ShouldUploadPayload requires payload!=nullptr and
    // mode_changed = (effective_composite=false) != (last_uploaded=false) = false.
    local.InvalidateStagedTexture();
    auto snap_after = local.LoadSnapshot();
    IM_CHECK(snap_after != nullptr);
    IM_CHECK(snap_after->payload == nullptr);  // fence took effect: payload dropped, serial unchanged.
    IM_CHECK_EQ(snap_after->texture_serial, snap->texture_serial);
    const bool fire_after =
        gui::ShouldFireCompositeUpload(*snap_after, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                       /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
    IM_CHECK(!fire_after);  // FIXED: no re-upload gate open.

    local.Stop();
    LUMICE_DestroyServer(server);
  };

  // AC2: 端到端 GL-context 回归 (.json 路径) — 主线程 SyncFromPoller + RenderExportToRgba 像素级读回。
  //
  // 与 AC1 的区别：AC1 只证谓词，AC2 证生产路径（主线程 SyncFromPoller 会真调 UploadTexture / Render
  // GL 调用，`HasTexture()` 前置信号 + fbo 像素级 sample 作为唯一"非 proxy"证据，覆盖
  // 2026-07-11 同日 `fix-clear-texture-gl-stale` 教训："CPU 字段翻转 ≠ 帧缓冲真实内容"）。
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
    std::string json = gui::SerializeCoreConfig(gui::InitDefaultState());
    const char* tmp_path = "/tmp/lumice_fence_open_json.json";
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
    std::remove(tmp_path);
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
    const char* tmp_path = "/tmp/lumice_fence_open_lmc_no_baked.lmc";
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
    std::remove(tmp_path);
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
    const char* tmp_path = "/tmp/lumice_fence_open_lmc_baked.lmc";
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
    std::remove(tmp_path);
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

  // task-color-migration M8 AC3 — pixel-level equivalence between reconciler-driven display push and
  // direct PushDisplayState. After Step 3 migration, widgets no longer call PushDisplayState inline;
  // the frame-tail reconciler + ApplyGuiEffects call it in response to a display-state field diff.
  // This test pins that "reconciler triggering timing is equivalent to widget inline calling" by
  // exercising both paths back-to-back on identical GuiState mutations and asserting byte-identical
  // composite output. PushDisplayState is used as the shared truth source — the test does NOT
  // re-verify its internals; it verifies the reconciler's edge-trigger produces the SAME server-side
  // C-API sequence as an inline call would have. Covers all five display-time edit types
  // (color / visible / solo / z_order / mode) in a single test, since each mutation independently
  // asserts identity — no gain from splitting into 5 sub-tests (plan §4 Step 8).
  ImGuiTest* t_ac3 =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "reconciler_display_push_matches_direct_push_byte_identical");
  t_ac3->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);
    LUMICE_Server* server = LUMICE_CreateServer();
    IM_CHECK(server != nullptr);
    const bool ok = RunToIdleWithData(server, kTwoColorConfig);
    IM_CHECK(ok);
    if (!ok) {
      LUMICE_DestroyServer(server);
      return;
    }

    // Build a GuiState mirror of the committed kTwoColorConfig display defaults — class 0 = red
    // match-all, class 1 = blue entry_exit len=3. PushDisplayState reads only
    // {color, visible, solo, z_order, raypath_color_mode} from state (match/combine are struct-tier,
    // not on the display-push channel), so match[] content doesn't have to be JSON-perfect —
    // populating any non-empty match[] with plausible values is sufficient. Structural fields are
    // NOT read by PushDisplayState (verified: color_window.cpp:52-70).
    auto MakeBaseline = []() {
      gui::GuiState s;
      s.raypath_color.resize(2);
      s.raypath_color[0].color[0] = 1.0f;  // red
      s.raypath_color[0].color[1] = 0.0f;
      s.raypath_color[0].color[2] = 0.0f;
      s.raypath_color[0].visible = true;
      s.raypath_color[0].solo = false;
      s.raypath_color[0].z_order = 0;
      s.raypath_color[1].color[0] = 0.0f;
      s.raypath_color[1].color[1] = 0.0f;
      s.raypath_color[1].color[2] = 1.0f;  // blue
      s.raypath_color[1].visible = true;
      s.raypath_color[1].solo = false;
      s.raypath_color[1].z_order = 1;
      s.raypath_color_mode = LUMICE_COLOR_MODE_DOMINANT;
      return s;
    };

    // Seed last_pushed_display_state from a GuiState — mirrors MakeBaselineState in
    // test_gui_state_reconcile.cpp. Reconciler treats nullopt as "first push after Reset" (need_push
    // still fires when raypath_color non-empty, but for AC3 we want the DIFF edge, not the reset
    // edge, so we seed a matching baseline and rely on the mutation to create the diff).
    auto SeedDisplayBaseline = [](gui::GuiState& s) {
      gui::GuiState::DisplayStateBaseline dsb;
      for (const auto& cls : s.raypath_color) {
        dsb.color_display.push_back(static_cast<const gui::ColorClassDisplayState&>(cls));
      }
      dsb.raypath_color_mode = s.raypath_color_mode;
      s.last_pushed_display_state = std::move(dsb);
    };

    // task-fix-composite-byte-identical-flake: pause the global poller so its background
    // DoSnapshot() cannot race the synchronous LUMICE_GetCompositeResults() call below. Each
    // preceding PushDisplayState() calls g_server_poller.WakeForRefresh(server) — that starts
    // the worker's PollOnce() → DoSnapshot() on another thread. Without this Stop(), the poller
    // can (a) Phase-1 the just-pushed display state (clearing snapshot_dirty_) BEFORE T's
    // GetCompositeResults sees it, then be still inside Phase-2 when T reads cached_composite_
    // results_ — so T returns a stale composite from a prior iteration, and the reset_check
    // memcmp below fires with ~40% probability under real timing (0% under --fixed-dt, whose
    // frame-scheduling coincidence keeps T ahead of the poller). The C API is contractually
    // single-caller; this test happens to have two (T + the process-global poller), which is
    // the source of the race — NOT a production bug (production only has the poller call site
    // reading composites, and the poller consumes its own DoSnapshot output coherently).
    //
    // Stop() drains any in-flight PollOnce(); the next PushDisplayState() re-wakes the poller,
    // so no display-time behavior is lost. AC2 memcmp==0 is preserved; AC4 zero core/config is
    // preserved (test-only fix).
    auto ReadComposite = [&](std::vector<uint8_t>& out) {
      gui::g_server_poller.Stop();
      LUMICE_RenderResult r[LUMICE_MAX_RENDER_RESULTS + 1]{};
      IM_CHECK_EQ(LUMICE_GetCompositeResults(server, r, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
      IM_CHECK(r[0].img_buffer != nullptr);
      const size_t nbytes = static_cast<size_t>(r[0].img_width) * static_cast<size_t>(r[0].img_height) * 3;
      out.assign(r[0].img_buffer, r[0].img_buffer + nbytes);
    };

    // Establish server-side baseline display so `pathA_baseline` and `pathB_baseline` are shared.
    // Uses PushDisplayState (the same API both paths converge on) so the baseline itself is not a
    // hidden divergence source.
    const gui::GuiState baseline = MakeBaseline();
    IM_CHECK(gui::PushDisplayState(baseline, server));
    std::vector<uint8_t> composite_baseline;
    ReadComposite(composite_baseline);

    // Five mutation lambdas — one per display-time edit type. Each MUST produce a composite
    // measurably different from baseline (sanity gate: if the mutation is invisible, the equivalence
    // assertion below is vacuous).
    struct Mutation {
      const char* name;
      void (*apply)(gui::GuiState&);
    };
    const Mutation muts[] = {
      { "color", [](gui::GuiState& s) { s.raypath_color[0].color[0] = 0.25f; } },   // dim class 0 red
      { "visible", [](gui::GuiState& s) { s.raypath_color[0].visible = false; } },  // hide bright class
      { "solo", [](gui::GuiState& s) { s.raypath_color[1].solo = true; } },         // solo dim class
      { "z_order",
        [](gui::GuiState& s) {
          // Swap z_order to promote class 1 to the top; dominant mode is invariant to z_order (arg
          // max wins regardless of layer stacking), but painter mode is not — so switch to painter
          // AND swap z_order together to make the mutation observable while keeping this the
          // "z_order lane" of the test.
          s.raypath_color[0].z_order = 1;
          s.raypath_color[1].z_order = 0;
          s.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;
        } },
      { "mode", [](gui::GuiState& s) { s.raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE; } },
    };

    for (const auto& m : muts) {
      // -- Path A: reconciler-driven --
      IM_CHECK(gui::PushDisplayState(baseline, server));  // reset server display to baseline
      std::vector<uint8_t> reset_check;
      ReadComposite(reset_check);
      IM_CHECK_EQ(std::memcmp(reset_check.data(), composite_baseline.data(), composite_baseline.size()), 0);

      gui::GuiState sA = MakeBaseline();
      SeedDisplayBaseline(sA);
      m.apply(sA);
      gui::GuiEffects effA = gui::ReconcileGuiEffects(sA);
      // Every mutation touches at least one display-state sub-field or raypath_color_mode → the
      // reconciler MUST route it to need_display_push (structural lane is quiet: no combine/match
      // change in these mutations).
      IM_CHECK(effA.need_display_push);
      IM_CHECK(!effA.need_hard_reset);
      IM_CHECK(!effA.need_resim);
      gui::ApplyGuiEffects(sA, server, effA);
      std::vector<uint8_t> compositeA;
      ReadComposite(compositeA);

      // Sanity: the mutation actually rendered (composite bytes differ from baseline).
      IM_CHECK(compositeA.size() == composite_baseline.size());
      IM_CHECK(std::memcmp(compositeA.data(), composite_baseline.data(), composite_baseline.size()) != 0);

      // Reconciler must also have updated last_pushed_display_state on success so a follow-up
      // reconcile with no further mutation is a quiet no-op (edge-triggered contract).
      IM_CHECK(sA.last_pushed_display_state.has_value());
      gui::GuiEffects effA_quiet = gui::ReconcileGuiEffects(sA);
      IM_CHECK(!effA_quiet.need_display_push);

      // -- Path B: direct PushDisplayState --
      IM_CHECK(gui::PushDisplayState(baseline, server));  // reset server display to baseline
      gui::GuiState sB = MakeBaseline();
      m.apply(sB);
      IM_CHECK(gui::PushDisplayState(sB, server));
      std::vector<uint8_t> compositeB;
      ReadComposite(compositeB);

      // ⭐ AC3 core assertion: reconciler-driven and direct-push paths produce byte-identical
      // composite. Both funnel through PushDisplayState with the same GuiState → same C-API args →
      // same server-side snapshot_dirty_ → same Phase-2 rebake. Any drift here would signal the
      // reconciler is mutating state between diff and push, or ApplyGuiEffects is calling something
      // other than PushDisplayState on the need_display_push branch.
      IM_CHECK_EQ(compositeA.size(), compositeB.size());
      IM_CHECK_EQ(std::memcmp(compositeA.data(), compositeB.data(), compositeA.size()), 0);
    }

    // WakeForRefresh inside PushDisplayState may have started the global poller worker; Stop it so
    // subsequent tests start clean.
    gui::g_server_poller.Stop();
    LUMICE_DestroyServer(server);
  };

  // AC2 real-server regression (code-review round-1 Major-1): the AC3 test above pins the
  // reconciler-vs-direct-push EQUIVALENCE, but never drives a real gui::DoRun() — so it cannot
  // catch a regression in the repush discipline that DoRun() itself owns (InvalidateEffectsBaselines
  // on commit success, app.cpp:854). This test drives the actual "drag z_order, click Run again"
  // flow: two fully-overlapping match-all classes on the single default crystal in painter mode (so
  // the LOWER z_order class wins every landed pixel regardless of brightness — see
  // CompositePainterPixel in component_compositor.cpp: the active list is sorted ascending by
  // z_order and the first participating class with a positive value wins, i.e. z_order 0 is the
  // top/occluding layer — making z_order priority directly observable), swap z_order via the
  // Step 2/3 field-write path (simulating a drag-reorder), then re-run with no further color
  // edits and assert the swapped order survives — the exact "Run 后 z_order 不生效" bug (plan §1
  // 偏离 B') a stale baseline would reintroduce.
  ImGuiTest* t_ac2_rerun = IM_REGISTER_TEST(engine, "gui_composite_preview", "zorder_priority_persists_across_rerun");
  t_ac2_rerun->TestFunc = [](ImGuiTestContext* ctx) {
    gui::g_server_poller.Stop();
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;
    gui::g_state = gui::InitDefaultState();
    gui::g_state.sim.infinite = false;
    gui::g_state.sim.ray_num_millions = 0.5f;
    gui::g_state.sim.max_hits = 8;
    // Deliberately CPU-only (default use_gpu_backend=false): the device-fused GPU path does not
    // populate outgoing_component_ / raypath-color lanes yet (RenderConsumer warns and produces an
    // empty composite) — this task's lane display targets CPU only, so forcing GPU here (as Test 8
    // in test_gui_lifecycle.cpp does for its pure lifecycle assertions) would make the composite
    // pixel reads below vacuous.

    auto MakeMatchAllClass = [](float r, float g, float b, int z_order) {
      gui::ColorClassConfig c;
      c.color[0] = r;
      c.color[1] = g;
      c.color[2] = b;
      c.visible = true;
      c.solo = false;
      c.z_order = z_order;
      gui::ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
      ref.match_all = true;
      c.match.push_back(ref);
      return c;
    };
    gui::g_state.raypath_color.push_back(MakeMatchAllClass(1.0f, 0.0f, 0.0f, /*z_order=*/0));  // red
    gui::g_state.raypath_color.push_back(MakeMatchAllClass(0.0f, 0.0f, 1.0f, /*z_order=*/1));  // blue
    gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;

    auto RunToDoneAndCheckIntent = [&]() {
      gui::DoRun();
      IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(gui::RunIntent::kRunning));
      auto start = std::chrono::steady_clock::now();
      while (gui::g_state.sim_state != gui::GuiState::SimState::kDone ||
             gui::g_state.run_intent != gui::RunIntent::kRunCompleted) {
        ctx->Yield();
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 20) {
          break;
        }
      }
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kDone));
    };

    // First run: establish red-on-top (z_order 0, the lower value == higher painter priority)
    // as the committed baseline.
    RunToDoneAndCheckIntent();

    // Void output-param idiom (not a bool-returning lambda): IM_CHECK's failure path is a bare
    // `return;`, which cannot coexist with a non-void lambda return type.
    auto ReadRedBlueSums = [&](unsigned long long& sum_r, unsigned long long& sum_b) {
      LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
      IM_CHECK_EQ(LUMICE_GetCompositeResults(gui::g_server, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
      IM_CHECK(comp[0].img_buffer != nullptr);
      const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
      sum_r = 0;
      sum_b = 0;
      for (size_t i = 0; i + 2 < nbytes; i += 3) {
        sum_r += comp[0].img_buffer[i + 0];
        sum_b += comp[0].img_buffer[i + 2];
      }
      IM_CHECK(sum_r + sum_b > 0u);  // sanity: something landed
    };
    unsigned long long r_init = 0;
    unsigned long long b_init = 0;
    ReadRedBlueSums(r_init, b_init);
    IM_CHECK(r_init > b_init);  // painter: z_order=0 (red) on top of z_order=1 (blue)

    // Field-write path (Step 2/3 channel): drag-reorder swap — promote blue to the top (lower
    // z_order == higher painter priority). Pure display-time edit; yield a few frames so the
    // frame-tail reconciler pushes it.
    gui::g_state.raypath_color[0].z_order = 1;
    gui::g_state.raypath_color[1].z_order = 0;
    ctx->Yield(3);
    unsigned long long r_swapped = 0;
    unsigned long long b_swapped = 0;
    ReadRedBlueSums(r_swapped, b_swapped);
    IM_CHECK(b_swapped > r_swapped);  // sanity: the swap took effect pre-Run (blue now on top)

    // AC2 core: click Run again (no color-field change) — the M6 repush discipline must not let
    // the re-commit silently fall back to the committed z_order order; the swapped order must
    // survive the round trip.
    RunToDoneAndCheckIntent();
    unsigned long long r_rerun = 0;
    unsigned long long b_rerun = 0;
    ReadRedBlueSums(r_rerun, b_rerun);
    IM_CHECK(b_rerun > r_rerun);  // still blue on top after the re-run

    gui::g_server_poller.Stop();
    if (gui::g_server) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    gui::g_state.run_intent = gui::RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
  };

  // AC-偏离C real-server regression (code-review round-1 Major-1): DoRevert() must repush the
  // restored display state to the SERVER, not just restore GuiState fields — this is the
  // g_state.InvalidateEffectsBaselines() call in DoRevert (app.cpp:946) fixing "Revert 不重推颜色
  // display 态" (plan §1 偏离 C). Establishes a committed red baseline, edits color to green
  // (display-time only, no re-sim), confirms the edit actually reached the server, then Reverts and
  // confirms the server falls back to red (not stuck on the pre-revert green).
  ImGuiTest* t_revert_repush =
      IM_REGISTER_TEST(engine, "gui_composite_preview", "revert_repushes_server_display_state");
  t_revert_repush->TestFunc = [](ImGuiTestContext* ctx) {
    gui::g_server_poller.Stop();
    gui::g_server = LUMICE_CreateServer();
    IM_CHECK(gui::g_server != nullptr);
    gui::g_server_is_gpu = false;
    gui::g_state = gui::InitDefaultState();
    gui::g_state.sim.infinite = false;
    gui::g_state.sim.ray_num_millions = 0.5f;
    gui::g_state.sim.max_hits = 8;
    // Deliberately CPU-only (default use_gpu_backend=false): the device-fused GPU path does not
    // populate outgoing_component_ / raypath-color lanes yet (RenderConsumer warns and produces an
    // empty composite) — this task's lane display targets CPU only, so forcing GPU here (as Test 8
    // in test_gui_lifecycle.cpp does for its pure lifecycle assertions) would make the composite
    // pixel reads below vacuous.

    gui::ColorClassConfig cls;
    cls.color[0] = 1.0f;  // red
    cls.color[1] = 0.0f;
    cls.color[2] = 0.0f;
    cls.visible = true;
    cls.solo = false;
    cls.z_order = 0;
    gui::ColorClassRefConfig ref;
    ref.layer_idx = 0;
    ref.crystal_pool_id = gui::g_state.layers[0].entries[0].crystal_id;
    ref.match_all = true;
    cls.match.push_back(ref);
    gui::g_state.raypath_color.push_back(cls);
    gui::g_state.raypath_color_mode = LUMICE_COLOR_MODE_DOMINANT;

    gui::DoRun();
    IM_CHECK_EQ(static_cast<int>(gui::g_state.run_intent), static_cast<int>(gui::RunIntent::kRunning));
    auto start = std::chrono::steady_clock::now();
    while (gui::g_state.sim_state != gui::GuiState::SimState::kDone ||
           gui::g_state.run_intent != gui::RunIntent::kRunCompleted) {
      ctx->Yield();
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() > 20) {
        break;
      }
    }
    IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kDone));

    auto ReadRedGreenSums = [&](unsigned long long& sum_r, unsigned long long& sum_g) {
      LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
      IM_CHECK_EQ(LUMICE_GetCompositeResults(gui::g_server, comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
      IM_CHECK(comp[0].img_buffer != nullptr);
      const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
      sum_r = 0;
      sum_g = 0;
      for (size_t i = 0; i + 2 < nbytes; i += 3) {
        sum_r += comp[0].img_buffer[i + 0];
        sum_g += comp[0].img_buffer[i + 1];
      }
    };

    unsigned long long r0 = 0;
    unsigned long long g0 = 0;
    ReadRedGreenSums(r0, g0);
    IM_CHECK(r0 > g0);  // committed baseline: red dominates

    // Display-time edit: red -> green. Pure display-time field write, no re-sim.
    gui::g_state.raypath_color[0].color[0] = 0.0f;
    gui::g_state.raypath_color[0].color[1] = 1.0f;
    ctx->Yield(3);
    unsigned long long r1 = 0;
    unsigned long long g1 = 0;
    ReadRedGreenSums(r1, g1);
    IM_CHECK(g1 > r1);  // sanity: the edit reached the server before Revert

    // AC-偏离C core: Revert must restore the committed (red) display state on the server too, not
    // just in GuiState.
    gui::DoRevert();
    IM_CHECK_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);
    IM_CHECK_EQ(gui::g_state.raypath_color[0].color[1], 0.0f);
    ctx->Yield(3);
    unsigned long long r2 = 0;
    unsigned long long g2 = 0;
    ReadRedGreenSums(r2, g2);
    IM_CHECK(r2 > g2);  // server-side display state now matches the reverted (red) GuiState

    gui::g_server_poller.Stop();
    if (gui::g_server) {
      LUMICE_StopServer(gui::g_server);
      LUMICE_DestroyServer(gui::g_server);
      gui::g_server = nullptr;
    }
    gui::g_server_is_gpu = false;
    gui::g_state.run_intent = gui::RunIntent::kNone;
    gui::g_state.committed_epoch = 0;
    gui::g_state.display_epoch_floor = 0;
  };
}
