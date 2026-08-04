// Composite-preview payload / display-push contract tests that need no window.
//
// Everything here drives the poller through ServerPoller::PollOnceForTest and the C API directly,
// then asserts on the payload, on the compositor output, or on a pure predicate. Real simulations
// run — that is orthogonal to needing a frame, since gui_unit_test links lumice_obj and the whole
// C API is available.
//
// Six siblings stay in test/gui/functional/test_gui_composite_preview.cpp: the four
// *_fences_stale_composite cases (one family pinning the same stale-composite fence across four
// document-entry paths — three of them need a GL texture or the harness reset, and splitting a
// family across two targets leaves the seam unowned), plus zorder_priority_persists_across_rerun
// and revert_repushes_server_display_state, which pump frames with ctx->Yield().

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/gui_state_reconcile.hpp"
#include "gui/server_poller.hpp"
#include "lumice.h"
#include "support/scoped_result_frame.hpp"

namespace gui = lumice::gui;

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

// Two-class config for the participating-exposure cases. class 0 = match-all (bright, every landed ray
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

// AC1 anchor (headless): raypath_color active → payload->is_composite == true,
// payload->rgb_data is populated and byte-identical to what LUMICE_GetCompositeResults
// returns directly. This is the headless mechanistic proof that the poller wires
// the composite surface through the payload; the on-screen visual is AC4 owner.
TEST(CompositePreview, raypath_color_active_populates_rgb_payload) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  ASSERT_TRUE(snap->payload != nullptr);
  EXPECT_TRUE(snap->payload->is_composite);
  EXPECT_TRUE(!snap->payload->rgb_data.empty());
  EXPECT_TRUE(!snap->payload->xyz_data.empty());  // XYZ still populated (auto-EV lane unchanged)

  // Byte-identity with a direct C-API composite read.
  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_comp(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_comp.get(), comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(comp[0].img_buffer != nullptr);
  EXPECT_EQ(comp[0].img_width, snap->payload->width);
  EXPECT_EQ(comp[0].img_height, snap->payload->height);
  const size_t nbytes = static_cast<size_t>(snap->payload->width) * static_cast<size_t>(snap->payload->height) * 3;
  EXPECT_EQ(snap->payload->rgb_data.size(), nbytes);
  EXPECT_EQ(std::memcmp(snap->payload->rgb_data.data(), comp[0].img_buffer, nbytes), 0);

  // Composite is non-trivial (class0 is match-all red).
  unsigned long long sum = 0;
  for (unsigned char v : snap->payload->rgb_data) {
    sum += v;
  }
  EXPECT_TRUE(sum > 0u);

  local.Stop();
  LUMICE_DestroyServer(server);
}

// AC2 anchor (headless): no raypath_color → payload->is_composite == false, rgb_data
// stays empty, xyz_data is byte-identical to LUMICE_GetRawXyzResults. This is the
// per-byte "zero regression" gate for scenes that never touch the color surface.
TEST(CompositePreview, no_raypath_color_stays_on_xyz_path) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kMonoConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  ASSERT_TRUE(snap->payload != nullptr);
  EXPECT_TRUE(!snap->payload->is_composite);
  EXPECT_TRUE(snap->payload->rgb_data.empty());
  EXPECT_TRUE(!snap->payload->xyz_data.empty());

  // XYZ byte-identity with the direct C-API read (AC2 zero-regression).
  LUMICE_RawXyzResult xyz[2]{};
  lumice::test::ScopedResultFrame frame_xyz(server);
  EXPECT_EQ(LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1), LUMICE_OK);
  EXPECT_TRUE(xyz[0].xyz_buffer != nullptr);
  EXPECT_EQ(xyz[0].img_width, snap->payload->width);
  EXPECT_EQ(xyz[0].img_height, snap->payload->height);
  const size_t nfloats = static_cast<size_t>(snap->payload->width) * static_cast<size_t>(snap->payload->height) * 3;
  EXPECT_EQ(snap->payload->xyz_data.size(), nfloats);
  EXPECT_EQ(std::memcmp(snap->payload->xyz_data.data(), xyz[0].xyz_buffer, nfloats * sizeof(float)), 0);

  // Composite surface reports the sentinel: img_buffer null → poller left is_composite false.
  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_comp(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_comp.get(), comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(comp[0].img_buffer == nullptr);

  local.Stop();
  LUMICE_DestroyServer(server);
}

// AC3 anchor.
// Pre-345.2 the poller pulled xyz + composite via three independent C-API calls (xyz →
// composite → xyz recheck) whose cross-call window was near-guaranteed to be crossed by
// ConsumeData batch churn under an active sim — the old test forced that mismatch and
// asserted the drop branch fired. 345.2 fused both reads into one combined getter, which
// closed the window for the pair but left it open for every other reader. The result frame
// closes it for all of them: xyz and composite are read off ONE immutable frame, so a
// cross-generation mix has nowhere left to come from.
// The invariant AC3 demanded ("composite与其渲染源xyz不跨代混装的保护仍在") is unchanged and
// still asserted here — only its mechanism moved. Every churn round must yield (a) a strictly
// advanced generation, (b) composite bytes byte-identical to what a same-tick second frame
// reports, and (c) PopulateCompositePayloadForTest building is_composite=true + rgb_data from
// that composite. Same fixture and deterministic dirty-arming technique as before.
TEST(CompositePreview, same_generation_invariant_under_churn) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
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
  //   3. Byte-match a same-tick second frame (proving the composite read off this frame is the
  //      one that pairs with this frame's xyz, not a cross-generation mix — the "真不变量"
  //      AC3 requires).
  //   4. Populate the poller payload as a composite (proving PopulateCompositePayload still
  //      writes rgb_data + sets is_composite, the same behavior the retired drop-recover
  //      test's "recovery" branch pinned).
  for (int round = 0; round < 4; ++round) {
    LUMICE_ColorClassDisplay disp[1]{};
    disp[0].color[2] = static_cast<float>(round % 3) / 2.0f;
    disp[0].visible = 1;
    EXPECT_EQ(LUMICE_SetRaypathColors(server, disp, 1, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

    LUMICE_RawXyzResult xyz[LUMICE_MAX_RENDER_RESULTS + 1]{};
    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    lumice::test::ScopedResultFrame frame(server);
    EXPECT_EQ(LUMICE_FrameGetRawXyz(frame.get(), xyz, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_EQ(LUMICE_FrameGetComposite(frame.get(), comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_TRUE(xyz[0].xyz_buffer != nullptr);
    EXPECT_TRUE(comp[0].img_buffer != nullptr);
    EXPECT_TRUE(xyz[0].snapshot_generation > prev_gen);
    prev_gen = xyz[0].snapshot_generation;

    // (3) Same-generation guarantee: a same-tick second frame must land on the same frozen
    // snapshot (nothing else armed dirty between the two acquires). No defensive copy is taken
    // first: `comp` reads into the frame this scope holds, and holding it is what keeps those
    // bytes valid — which is the property the frame handle exists to provide.
    const size_t nbytes = static_cast<size_t>(comp[0].img_width) * static_cast<size_t>(comp[0].img_height) * 3;
    LUMICE_RenderResult direct[LUMICE_MAX_RENDER_RESULTS + 1]{};
    lumice::test::ScopedResultFrame frame_direct(server);
    EXPECT_EQ(LUMICE_FrameGetComposite(frame_direct.get(), direct, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_TRUE(direct[0].img_buffer != nullptr);
    EXPECT_EQ(std::memcmp(comp[0].img_buffer, direct[0].img_buffer, nbytes), 0);

    // (4) PopulateCompositePayload builds the composite payload from those bytes.
    gui::TexturePayload payload;
    local.PopulateCompositePayloadForTest(direct[0], &payload);
    EXPECT_TRUE(payload.is_composite);
    EXPECT_EQ(payload.rgb_data.size(), nbytes);
    EXPECT_EQ(std::memcmp(payload.rgb_data.data(), direct[0].img_buffer, nbytes), 0);
  }

  local.Stop();
  LUMICE_DestroyServer(server);
}

// AC1 anchor (wake-poller): after a finite sim reaches COMPLETED and the poller
// self-pauses, a display-time color edit (LUMICE_SetRaypathColors) must be able to drive one
// fresh composite materialization without restarting the sim (epoch unchanged, lifecycle stays
// COMPLETED). Pins the two coupled invariants:
//   (a) MECHANISM: the poll after SetRaypathColors + WakeForRefresh yields a payload whose
//       rgb_data reflects the new colors and is byte-identical to a direct LUMICE_GetCompositeResults
//       (proving the display-time dirty flag was consumed, not lost).
//   (b) NON-RESTART: LUMICE_GetSimLifecycle still reports COMPLETED with the SAME epoch
//       observed before the edit (322 clock decoupling: display-time edit does NOT bump
//       committed_epoch_ and does NOT flip sim state back to RUNNING).
TEST(CompositePreview, display_time_color_edit_repaints_without_restart) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  // Snapshot the terminal (finite-completion) lifecycle: this is what a display-time edit
  // MUST NOT bump.
  LUMICE_SimLifecycleResult lc_before{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
  EXPECT_EQ(lc_before.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  const unsigned long long done_epoch = lc_before.epoch;

  // AC1 explicit sub-clause ("sim_ray_count 不减"): capture the live accumulated ray
  // count before the display-time edit so it can be compared post-edit below. Uses
  // the O(1) live counter, not the DoSnapshot-cached stats path.
  LUMICE_RayCount ray_count_before = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);

  // Baseline poll to freeze current composite for a byte-diff below.
  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  auto snap_before = local.LoadSnapshot();
  ASSERT_TRUE(snap_before != nullptr);
  EXPECT_TRUE(snap_before->valid);
  ASSERT_TRUE(snap_before->payload != nullptr);
  EXPECT_TRUE(snap_before->payload->is_composite);
  std::vector<uint8_t> composite_before(snap_before->payload->rgb_data.begin(), snap_before->payload->rgb_data.end());
  // Kept so the post-edit poll can be shown to have MATERIALIZED a payload rather than carried the
  // previous one forward — see the serial assertion below.
  const unsigned long long serial_before = snap_before->texture_serial;

  // Display-time edit: swap the class-0 color from red → blue. kColorConfig commits exactly
  // one raypath_color class, so class_count == 1 matches.
  LUMICE_ColorClassDisplay disp[1]{};
  disp[0].color[0] = 0.0f;
  disp[0].color[1] = 0.0f;
  disp[0].color[2] = 1.0f;  // blue
  disp[0].visible = 1;
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp, 1, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
  // The very wake call PushDisplayState() invokes in production, exercised on the global poller
  // that would drive DoSnapshot() consumption when a background worker was actually running.
  // For this synchronous test seam, WakeForRefresh + PollOnceForTest together stand in for
  // "worker wakes up and runs one PollOnce, then self-pauses at COMPLETED".
  gui::g_server_poller.WakeForRefresh(server);

  // The wake above starts a REAL background thread that polls this same server, so from here to
  // the read below there are two consumers driving ServerImpl::DoSnapshot() concurrently — and
  // that getter family is not safe for two. DoSnapshot's three segments are not jointly atomic:
  // Phase 1 consumes snapshot_dirty_ and bumps snapshot_generation_ under consumer_mutex_,
  // Phase 1.5 counts effective pixels under NO lock, and only Phase 2 rebuilds
  // cached_composite_results_ under snapshot_mutex_ (server.cpp:702-799). A second caller arriving
  // while the first is in Phase 1.5 finds snapshot_dirty_ already false, returns early WITHOUT
  // waiting for that Phase 2, and then reports the already-bumped generation beside the PREVIOUS
  // generation's pixels. That is exactly the failure this case hit on CI: the payload came back
  // byte-identical to composite_before while a direct read a few lines later was already the new
  // image. Worse, img_buffer is a borrowed pointer that Phase 2's cache swap frees, so the loser
  // is also reading memory that may have been released.
  //
  // Stop() drains the worker synchronously (it returns only once the worker has left the poll
  // loop), collapsing the two consumers back to one. That removes the window structurally rather
  // than narrowing it — no sleep, no retry, no tolerance. It is the same serialization the sibling
  // cases in this file and in test/gui/functional/test_gui_composite_preview.cpp already apply
  // before every composite read; this case woke the poller without it.
  //
  // The yield is the nail rather than an incidental detail: without it the woken worker usually
  // never gets scheduled before Stop() lands, so the two-consumer interleaving this case is
  // supposed to survive is barely exercised and deleting the Stop() would mostly go unnoticed —
  // that is precisely how the CI flake survived here. A --gtest_repeat sweep measured the yield to
  // raise detection of a missing Stop() by roughly two orders of magnitude (per-run rates and the
  // exact arms are in commit e5cbb170; they are one machine's one-off sample, not an invariant, and
  // will drift with hardware, pixel count and scheduler). What does NOT drift, and is the reason
  // the yield stays: it cannot produce a false red — it only hands the CPU to a worker that a
  // correct sequence has already serialized away, so a green run never depends on the sample above.
  std::this_thread::yield();
  gui::g_server_poller.Stop();

  // (a) MECHANISM: the poll after the edit must produce a composite reflecting the new colors —
  // i.e., not byte-identical to composite_before (the color changed, so the rendered image must
  // differ). And it must byte-match a same-tick direct C-API read.
  local.PollOnceForTest(server);
  auto snap_after = local.LoadSnapshot();
  ASSERT_TRUE(snap_after != nullptr);
  EXPECT_TRUE(snap_after->valid);
  ASSERT_TRUE(snap_after->payload != nullptr);
  EXPECT_TRUE(snap_after->payload->is_composite);
  // This poll must have materialized a payload of its own, not carried snap_before's forward. The
  // two ways this case can go stale look identical at the byte level but differ here: a torn read
  // publishes a NEW payload (serial advances) holding the old pixels, whereas a quality-gate or
  // content-freshness rejection republishes the OLD payload pointer (serial unchanged). Asserting
  // the serial separately keeps a future failure self-diagnosing instead of collapsing both into
  // one "bytes are stale" symptom.
  EXPECT_TRUE(snap_after->texture_serial != serial_before);
  EXPECT_EQ(snap_after->payload->rgb_data.size(), composite_before.size());
  EXPECT_TRUE(std::memcmp(snap_after->payload->rgb_data.data(), composite_before.data(), composite_before.size()) != 0);

  LUMICE_RenderResult direct[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_direct(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_direct.get(), direct, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(direct[0].img_buffer != nullptr);
  EXPECT_EQ(
      std::memcmp(snap_after->payload->rgb_data.data(), direct[0].img_buffer, snap_after->payload->rgb_data.size()), 0);

  // (b) NON-RESTART: lifecycle stays COMPLETED with the SAME epoch. This is the 322 clock-
  // decoupling guarantee — a display-time edit re-materializes but does NOT reset the
  // accumulator or bump the committed_epoch. If ③'s wake path ever regressed to calling
  // LUMICE_Start / LUMICE_CommitScene, this would flip lifecycle back to RUNNING and/or
  // bump the epoch.
  LUMICE_SimLifecycleResult lc_after{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
  EXPECT_EQ(lc_after.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_EQ(lc_after.epoch, done_epoch);

  // Payload's epoch also stays == done_epoch (no accumulator reset).
  EXPECT_EQ(snap_after->payload->payload_epoch, done_epoch);

  // AC1 explicit sub-clause ("sim_ray_count 不减"): a display-time color edit must not
  // reset or decrement the accumulated sim ray count — that would be a tell-tale sign
  // of an accidental restart (LUMICE_Start/CommitConfig) hiding behind the wake path.
  LUMICE_RayCount ray_count_after = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
  EXPECT_TRUE(ray_count_after >= ray_count_before);

  // Post-test cleanup on both the local ServerPoller and the global one that WakeForRefresh
  // above nudged into kRunning — Stop() is synchronous and idempotent.
  local.Stop();
  gui::g_server_poller.Stop();
  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, display_time_composite_exposure_reaches_compositor) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  LUMICE_SimLifecycleResult lc_before{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
  EXPECT_EQ(lc_before.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  const unsigned long long done_epoch = lc_before.epoch;

  LUMICE_RayCount ray_count_before = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);

  // Baseline composite (EV=0 → scale 1.0 → default behavior) via direct C-API read.
  LUMICE_RenderResult baseline[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_baseline(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_baseline.get(), baseline, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(baseline[0].img_buffer != nullptr);
  const size_t rgb_bytes = static_cast<size_t>(baseline[0].img_width) * static_cast<size_t>(baseline[0].img_height) * 3;
  std::vector<uint8_t> baseline_rgb(baseline[0].img_buffer, baseline[0].img_buffer + rgb_bytes);
  // Composite-only P99 anchor must be populated on the composite path.
  EXPECT_TRUE(baseline[0].composite_p99_y > 0.0f);

  // Mono getter must NOT populate the composite-only anchor field.
  LUMICE_RenderResult mono_out[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_mono_out(server);
  EXPECT_EQ(LUMICE_FrameGetRender(frame_mono_out.get(), mono_out, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(mono_out[0].img_buffer != nullptr);
  EXPECT_EQ(mono_out[0].composite_p99_y, 0.0f);

  // Push a positive EV → next Get*Results triggers a rebake with 2^ev > 1 → bytes change.
  EXPECT_EQ(LUMICE_SetCompositeExposure(server, 2.0f), LUMICE_OK);
  LUMICE_RenderResult brighter[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_brighter(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_brighter.get(), brighter, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(brighter[0].img_buffer != nullptr);
  EXPECT_EQ(brighter[0].img_width, baseline[0].img_width);
  EXPECT_EQ(brighter[0].img_height, baseline[0].img_height);
  EXPECT_TRUE(std::memcmp(brighter[0].img_buffer, baseline_rgb.data(), rgb_bytes) != 0);
  // P99 anchor is over UNEXPOSED lane values — must not move with EV.
  EXPECT_EQ(brighter[0].composite_p99_y, baseline[0].composite_p99_y);

  // Push a negative EV → different bytes from both the baseline and the brighter output.
  EXPECT_EQ(LUMICE_SetCompositeExposure(server, -2.0f), LUMICE_OK);
  LUMICE_RenderResult dimmer[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_dimmer(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_dimmer.get(), dimmer, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(dimmer[0].img_buffer != nullptr);
  EXPECT_TRUE(std::memcmp(dimmer[0].img_buffer, baseline_rgb.data(), rgb_bytes) != 0);
  EXPECT_TRUE(std::memcmp(dimmer[0].img_buffer, brighter[0].img_buffer, rgb_bytes) != 0);

  // Non-restart guarantee (322 clock decoupling): lifecycle epoch + sim ray count both
  // preserved despite three separate composite re-bakes above.
  LUMICE_SimLifecycleResult lc_after{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
  EXPECT_EQ(lc_after.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_EQ(lc_after.epoch, done_epoch);
  LUMICE_RayCount ray_count_after = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
  EXPECT_TRUE(ray_count_after >= ray_count_before);

  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, display_time_visibility_reanchors_participating_p99) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kTwoColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  LUMICE_SimLifecycleResult lc_before{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
  EXPECT_EQ(lc_before.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  const unsigned long long done_epoch = lc_before.epoch;
  LUMICE_RayCount ray_count_before = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);

  // Baseline: additive mode, both classes visible → participating union of both lanes,
  // p99 is dominated by the bright match-all class (class 0).
  LUMICE_ColorClassDisplay disp_both[2]{};
  disp_both[0].color[0] = 1.0f;  // class 0 red
  disp_both[0].visible = 1;
  disp_both[0].solo = 0;
  disp_both[1].color[2] = 1.0f;  // class 1 blue
  disp_both[1].visible = 1;
  disp_both[1].solo = 0;
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp_both, 2, nullptr, LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);
  LUMICE_RenderResult baseline[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_baseline(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_baseline.get(), baseline, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(baseline[0].img_buffer != nullptr);
  const int w = baseline[0].img_width;
  const int h = baseline[0].img_height;
  const size_t nbytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
  const float p99_both = baseline[0].composite_p99_y;
  EXPECT_TRUE(p99_both > 0.0f);
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
  EXPECT_TRUE(probe_pixels.size() >= 32);  // class 1 has a non-trivial blue population
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
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp_hide_bright, 2, nullptr, LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);

  LUMICE_RenderResult dim_only[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_dim_only(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_dim_only.get(), dim_only, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(dim_only[0].img_buffer != nullptr);
  const float p99_dim_only = dim_only[0].composite_p99_y;
  EXPECT_TRUE(p99_dim_only > 0.0f);
  EXPECT_TRUE(p99_dim_only < p99_both);  // structural佐证: participating shrank → anchor dropped
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
  EXPECT_TRUE(blue_after > blue_before);
  EXPECT_TRUE(blue_after >= blue_before * 1.3);

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
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp_restore, 2, nullptr, LUMICE_COLOR_MODE_ADDITIVE), LUMICE_OK);
  LUMICE_RenderResult restored[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_restored(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_restored.get(), restored, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(restored[0].img_buffer != nullptr);
  EXPECT_EQ(restored[0].composite_p99_y, p99_both);
  for (size_t off : probe_pixels) {
    EXPECT_EQ(restored[0].img_buffer[off + 2], baseline_rgb[off + 2]);
  }

  // display-time guarantees (322 clock decoupling): visibility toggles never bump the
  // lifecycle epoch and never reset the accumulated sim ray count.
  LUMICE_SimLifecycleResult lc_after{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
  EXPECT_EQ(lc_after.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_EQ(lc_after.epoch, done_epoch);
  LUMICE_RayCount ray_count_after = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
  EXPECT_TRUE(ray_count_after >= ray_count_before);

  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, display_time_visibility_reanchors_participating_p99_dominant) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kTwoColorConfig);
  EXPECT_TRUE(ok);
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
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp_solo_c1, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
  LUMICE_RenderResult solo1[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_solo1(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_solo1.get(), solo1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(solo1[0].img_buffer != nullptr);
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
  EXPECT_TRUE(probe_x >= 0);  // class 1 solo picture has a clearly blue pixel

  // Step 2: both visible, dominant mode. class 0 (match-all) argmax-wins at every pixel that
  // has any landing signal, so the probe pixel now shows RED (class 0's color), not blue.
  LUMICE_ColorClassDisplay disp_both[2]{};
  disp_both[0].color[0] = 1.0f;
  disp_both[0].visible = 1;
  disp_both[0].solo = 0;
  disp_both[1].color[2] = 1.0f;
  disp_both[1].visible = 1;
  disp_both[1].solo = 0;
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp_both, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
  LUMICE_RenderResult both[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_both(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_both.get(), both, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(both[0].img_buffer != nullptr);
  const size_t probe_off = (static_cast<size_t>(probe_y) * w + probe_x) * 3;
  const uint8_t r_both = both[0].img_buffer[probe_off + 0];
  const uint8_t b_both = both[0].img_buffer[probe_off + 2];
  // class 0's argmax-win expresses as red > blue at the probe pixel (r dominant, b ≈ 0).
  EXPECT_TRUE(r_both > b_both);
  EXPECT_TRUE(r_both >= 16);  // pixel is clearly lit in the class-0 color

  // Step 3: hide class 0. The probe pixel's argmax winner is now class 1; the composite at
  // that pixel becomes BLUE. This is the owner ⑤ "from-invisible-to-visible" flip.
  LUMICE_ColorClassDisplay disp_hide_c0[2]{};
  disp_hide_c0[0].color[0] = 1.0f;
  disp_hide_c0[0].visible = 0;  // HIDE bright class
  disp_hide_c0[0].solo = 0;
  disp_hide_c0[1].color[2] = 1.0f;
  disp_hide_c0[1].visible = 1;
  disp_hide_c0[1].solo = 0;
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp_hide_c0, 2, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);
  LUMICE_RenderResult c1_visible[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_c1_visible(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_c1_visible.get(), c1_visible, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(c1_visible[0].img_buffer != nullptr);
  const uint8_t r_after = c1_visible[0].img_buffer[probe_off + 0];
  const uint8_t b_after = c1_visible[0].img_buffer[probe_off + 2];
  // Now blue wins (class 1's color). AND thanks to Fix B's self-anchor, blue_after must
  // also be BRIGHT (not just "wins by 1 unit") — after hiding the bright class the p99
  // shrinks to class 1's own lane, s grows, and class 1's pixels light up clearly.
  EXPECT_TRUE(b_after > r_after);
  EXPECT_TRUE(b_after >= 16);  // clearly visible (self-anchor made it bright)

  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, rerun_with_same_ev_produces_identical_composite) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  constexpr float kUserEv = 2.0f;  // non-zero — the bug is invisible at EV=0

  // First Run.
  EXPECT_TRUE(RunToIdleWithData(server, kColorConfig));
  EXPECT_EQ(LUMICE_SetCompositeExposure(server, kUserEv), LUMICE_OK);
  LUMICE_RenderResult first[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_first(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_first.get(), first, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(first[0].img_buffer != nullptr);
  const size_t rgb_bytes = static_cast<size_t>(first[0].img_width) * static_cast<size_t>(first[0].img_height) * 3;
  std::vector<uint8_t> first_rgb(first[0].img_buffer, first[0].img_buffer + rgb_bytes);

  // Re-Run: same config, same EV. RunToIdleWithData internally commits the scene, which
  // calls Stop() → ResetWith()/rebuild → restart accumulation. This is the exact
  // path DoRun() takes; if any code layer sneaked EV into the committed config, this
  // re-Run's composite would be 2× amplified vs. first Run.
  EXPECT_TRUE(RunToIdleWithData(server, kColorConfig));
  EXPECT_EQ(LUMICE_SetCompositeExposure(server, kUserEv), LUMICE_OK);
  LUMICE_RenderResult rerun[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_rerun(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_rerun.get(), rerun, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(rerun[0].img_buffer != nullptr);
  EXPECT_EQ(rerun[0].img_width, first[0].img_width);
  EXPECT_EQ(rerun[0].img_height, first[0].img_height);

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
  EXPECT_TRUE(sum_first > 0u);
  EXPECT_TRUE(sum_rerun > 0u);
  const double ratio = static_cast<double>(sum_rerun) / static_cast<double>(sum_first);
  EXPECT_TRUE(ratio > 0.8);
  EXPECT_TRUE(ratio < 1.25);
  // The unexposed P99 anchor also stays within similar stochastic bounds (this is the tight
  // proof at the anchor level — it's independent of EV).
  const double p99_ratio =
      static_cast<double>(rerun[0].composite_p99_y) / static_cast<double>(first[0].composite_p99_y);
  EXPECT_TRUE(p99_ratio > 0.8);
  EXPECT_TRUE(p99_ratio < 1.25);

  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, add_class_after_idle_reconverges_within_bound) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);

  // Phase A: single-class color config → idle.
  EXPECT_TRUE(RunToIdleWithData(server, kColorConfig));
  LUMICE_SimLifecycleResult lc0{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc0), LUMICE_OK);
  EXPECT_EQ(lc0.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  const unsigned long long epoch_before = lc0.epoch;

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  {
    auto snap = local.LoadSnapshot();
    ASSERT_TRUE(snap != nullptr);
    EXPECT_TRUE(snap->valid);
    ASSERT_TRUE(snap->payload != nullptr);
    EXPECT_TRUE(snap->payload->is_composite);  // baseline is a valid composite frame
  }

  // Phase B: user adds a class → GUI-side re-commits the 2-class config. This is the
  // same code path RenderColorWindow's "Add Class" button triggers (state.MarkStructHardDirty
  // → next debounce → LUMICE_CommitScene → re-sim + RenderConsumer rebuild).
  EXPECT_EQ(CommitJsonConfig(server, kTwoColorConfig), LUMICE_OK);

  // Bounded convergence: 300 polls × 10 ms = 3s hard ceiling. The fixture (400k rays,
  // 128x64) typically converges in under 500ms on release build. Anything beyond the
  // ceiling is a stall rather than carry-forward, which would mean the convergence assumption
  // this case rests on is wrong.
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
  EXPECT_TRUE(converged);

  // Both classes are configured with non-empty match[] in kTwoColorConfig; after
  // convergence both should show signal (class 0 is match-all → all landing rays;
  // class 1 is entry_exit len==3 → a subset). This is the paired proof that the
  // "全类都报 no rays matched" symptom is gone at the truth level: not just because
  // the GUI-side signal_flags cache defaults to 1 (M1 fix), but because the actual
  // server-side signal is non-zero for both classes.
  int flags[2] = { -1, -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(server, flags, 2), LUMICE_OK);
  EXPECT_EQ(flags[0], 1);
  EXPECT_EQ(flags[1], 1);

  local.Stop();
  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, composite_exposure_idempotent_when_value_unchanged) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }
  LUMICE_SimLifecycleResult lc_before{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_before), LUMICE_OK);
  const unsigned long long done_epoch = lc_before.epoch;

  // Set the same EV twice — both must return LUMICE_OK, neither must bump the
  // lifecycle epoch or reset sim ray count.
  LUMICE_RayCount ray_count_before = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_before), LUMICE_OK);
  EXPECT_EQ(LUMICE_SetCompositeExposure(server, 1.5f), LUMICE_OK);
  EXPECT_EQ(LUMICE_SetCompositeExposure(server, 1.5f), LUMICE_OK);
  LUMICE_SimLifecycleResult lc_after{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc_after), LUMICE_OK);
  EXPECT_EQ(lc_after.epoch, done_epoch);
  LUMICE_RayCount ray_count_after = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &ray_count_after), LUMICE_OK);
  EXPECT_TRUE(ray_count_after >= ray_count_before);

  // Null-server rejection.
  EXPECT_EQ(LUMICE_SetCompositeExposure(nullptr, 0.0f), LUMICE_ERR_NULL_ARG);

  LUMICE_DestroyServer(server);
}

// AC1 truth table (headless, no server/GL needed). Pins the pure decision predicate
// ShouldUseCompositeUpload = payload_is_composite AND show_composite_preview across all four
// input combinations. Guarantees a code-refactor that reorders the AND cannot silently flip
// the display-mode logic.
TEST(CompositePreview, should_use_composite_upload_truth_table) {
  EXPECT_EQ(gui::ShouldUseCompositeUpload(false, false), false);
  EXPECT_EQ(gui::ShouldUseCompositeUpload(false, true), false);
  EXPECT_EQ(gui::ShouldUseCompositeUpload(true, false), false);
  EXPECT_EQ(gui::ShouldUseCompositeUpload(true, true), true);
}

// AC1 mechanism (headless): after a color-active sim reaches COMPLETED and the
// poller self-pauses, flipping `show_composite_preview` alone (no new poll, no MarkDirty)
// must re-fire the upload branch. The predicate is the exact one SyncFromPoller consumes
// (ShouldFireCompositeUpload), so pinning it here is equivalent to pinning the production
// decision at the fire-branch seam. SyncFromPoller cannot be driven end-to-end from this
// functional coroutine (its GL Upload*Texture call SIGILLs without a current GL context —
// see doc comment on ShouldFireCompositeUpload). AC5 owner on-screen still covers the actual
// GL upload / repaint.
TEST(CompositePreview, mode_flip_forces_refire_at_same_serial) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  ASSERT_TRUE(snap->payload != nullptr);
  EXPECT_TRUE(snap->payload->is_composite);

  // T0: initial state before any upload — the fire gate must be TRUE (serial-dedup gate is
  // satisfied: last_uploaded_texture_serial==0 != snap.texture_serial). This is the AC1
  // first-frame case.
  const bool fire_first =
      gui::ShouldFireCompositeUpload(*snap, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
  EXPECT_TRUE(fire_first);

  // T1: simulate the post-upload bookkeeping SyncFromPoller performs — record the serial and
  // the ground-truth composite flag. Same-serial + same-mode + already-uploaded ⇒ fire gate
  // must be FALSE (idempotence: unchanged state must not spam uploads).
  const auto serial_after_upload = snap->texture_serial;
  const bool fire_idempotent =
      gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/true);
  EXPECT_TRUE(!fire_idempotent);

  // T2: user flips the preference OFF. No new poll → serial stable, floor stable. The
  // mode_changed OR-branch is the ONLY thing that can re-fire the gate. This is the core
  // AC1 mechanism ("即时生效不依赖新一轮 poll") — if this returns false, the toggle button
  // would visibly do nothing.
  const bool fire_after_off_flip =
      gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/false, /*last_uploaded_as_composite=*/true);
  EXPECT_TRUE(fire_after_off_flip);

  // T3: simulate the post-fire bookkeeping (last_uploaded_as_composite=false). Same-serial +
  // same-mode ⇒ idempotent again.
  const bool fire_stable_off =
      gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/false, /*last_uploaded_as_composite=*/false);
  EXPECT_TRUE(!fire_stable_off);

  // T4: user flips the preference back ON — mode_changed fires again, AC2 no-loss recovery
  // is now the composite path (raypath_color config was never touched).
  const bool fire_after_on_flip =
      gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
  EXPECT_TRUE(fire_after_on_flip);

  // T5: same-serial + settled mode ⇒ idempotent, no re-fire loop.
  const bool fire_stable_on =
      gui::ShouldFireCompositeUpload(*snap, serial_after_upload, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/true);
  EXPECT_TRUE(!fire_stable_on);

  local.Stop();
  LUMICE_DestroyServer(server);
}

// AC4 zero-regression: with no raypath_color classes, ShouldFireCompositeUpload
// collapses to ShouldUploadPayload (mode_changed is structurally false because
// effective_composite is false and last_uploaded_as_composite starts false). Pins the boolean
// algebra AC4 relies on, plus the render-gate condition (`raypath_color.empty()`) that keeps
// the status-bar toggle button off screen.
TEST(CompositePreview, mode_toggle_hidden_when_no_color_classes) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kMonoConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);
  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  ASSERT_TRUE(snap->payload != nullptr);
  EXPECT_TRUE(!snap->payload->is_composite);  // no raypath_color ⇒ payload not composite

  // Render-gate condition: the status-bar mode-toggle button is only rendered when the
  // GUI-side g_state.raypath_color is non-empty. A newly created server has no color-class
  // wiring through the C API; g_state is a persisted view snapshot that is only populated
  // by user actions in the GUI (color-class add/edit). In a clean test fixture this stays
  // empty, so the button gate is off (AC4).
  // This assertion is intentionally at the GuiState level, not through ImGui item probing:
  // the whole button block in RenderStatusBar is a single `if (show_mode_toggle) { ... }`
  // guarded by exactly this condition — no ambiguity to bridge.
  gui::g_state.raypath_color.clear();
  EXPECT_TRUE(gui::g_state.raypath_color.empty());

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
  EXPECT_EQ(fire_with_pref_true, fire_should_upload_only);

  local.Stop();
  LUMICE_DestroyServer(server);
}

// AC3: pin the pure decision boundary for "opening the Colors window
// should default show_composite_preview=true". True iff no color classes exist yet
// (nothing to remember); false when classes exist so the caller keeps the user's
// memory. The "apply only on false→true transition" time-guard lives at the call
// site in RenderTopBar and is not testable from this headless coroutine (no ImGui
// click surface without the real event loop), but factoring out this predicate
// makes the branch inside the click handler a single conditional that is obvious
// in the diff and cannot silently invert.
TEST(CompositePreview, should_default_enable_colors_on_open_predicate) {
  EXPECT_EQ(gui::ShouldDefaultEnableColorsOnOpen(true), true);
  EXPECT_EQ(gui::ShouldDefaultEnableColorsOnOpen(false), false);
}

// AC1/AC2 shared writer: ToggleCompositePreview flips exactly one field
// (show_composite_preview) and touches nothing else. Kept alongside the predicate
// tests so both write sites (top-bar Button, Colors-window Checkbox) share a
// single asserted invariant: "toggle" is negation, nothing more.
TEST(CompositePreview, toggle_composite_preview_negates_field) {
  gui::GuiState s;
  s.show_composite_preview = false;
  gui::ToggleCompositePreview(s);
  EXPECT_EQ(s.show_composite_preview, true);
  gui::ToggleCompositePreview(s);
  EXPECT_EQ(s.show_composite_preview, false);
}

// AC1: 决策层回归 —— 把复现该缺陷的 DIAG 步骤形式化为断言，无 GL 依赖。
TEST(CompositePreview, should_fire_composite_upload_fires_on_stale_staged_snapshot_until_invalidated) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kColorConfig);
  EXPECT_TRUE(ok);
  if (!ok) {
    LUMICE_DestroyServer(server);
    return;
  }

  gui::ServerPoller local;
  local.ResetGenerationForTest();
  local.PollOnceForTest(server);

  auto snap = local.LoadSnapshot();
  ASSERT_TRUE(snap != nullptr);
  EXPECT_TRUE(snap->valid);
  ASSERT_TRUE(snap->payload != nullptr);
  EXPECT_TRUE(snap->payload->is_composite);
  EXPECT_TRUE(snap->texture_serial > 0);
  EXPECT_TRUE(snap->payload->payload_epoch > 0);

  // Simulate the post-DoOpen(InitDefaultState) reset: last_uploaded_texture_serial=0,
  // display_epoch_floor=0, last_uploaded_as_composite=false, show_composite_preview=true.
  // Pre-fix: ShouldUploadPayload true (serial!=0, epoch>0) OR mode_changed true → fires (BUG).
  const bool fire_before =
      gui::ShouldFireCompositeUpload(*snap, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
  EXPECT_TRUE(fire_before);  // BUG-shape: stale composite would be re-uploaded on next SyncFromPoller.

  // The fix: DoOpen/DoNew calls this on the same poller instance. Payload is dropped; the fire
  // gate collapses to false because ShouldUploadPayload requires payload!=nullptr and
  // mode_changed = (effective_composite=false) != (last_uploaded=false) = false.
  local.InvalidateStagedTexture();
  auto snap_after = local.LoadSnapshot();
  ASSERT_TRUE(snap_after != nullptr);
  EXPECT_TRUE(snap_after->payload == nullptr);  // fence took effect: payload dropped, serial unchanged.
  EXPECT_EQ(snap_after->texture_serial, snap->texture_serial);
  const bool fire_after =
      gui::ShouldFireCompositeUpload(*snap_after, /*last_uploaded_texture_serial=*/0, /*display_epoch_floor=*/0,
                                     /*show_composite_preview=*/true, /*last_uploaded_as_composite=*/false);
  EXPECT_TRUE(!fire_after);  // FIXED: no re-upload gate open.

  local.Stop();
  LUMICE_DestroyServer(server);
}

TEST(CompositePreview, reconciler_display_push_matches_direct_push_byte_identical) {
  LUMICE_Server* server = LUMICE_CreateServer();
  EXPECT_TRUE(server != nullptr);
  const bool ok = RunToIdleWithData(server, kTwoColorConfig);
  EXPECT_TRUE(ok);
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

  // Pause the global poller so its background
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
  //
  // This Stop() only serializes ServerImpl::DoSnapshot()/cached_composite_results_ — a
  // separate data path from g_server_poller's published_/staged-snapshot pipeline
  // (StorePublished/LoadSnapshot, consumed by SyncFromPoller/g_preview). This test reads
  // composites via the C API (LUMICE_GetCompositeResults) directly, never through
  // LoadSnapshot(), so a stale published_ snapshot cannot leak through this path. Leaving the
  // poller paused at test end until the next PushDisplayState wakes it is also the same
  // convention already used at every other test's teardown in this file (see the sibling
  // g_server_poller.Stop() call sites above and below), so this is not a new cross-test risk.
  auto ReadComposite = [&](std::vector<uint8_t>& out) {
    gui::g_server_poller.Stop();
    LUMICE_RenderResult r[LUMICE_MAX_RENDER_RESULTS + 1]{};
    lumice::test::ScopedResultFrame frame_r(server);
    EXPECT_EQ(LUMICE_FrameGetComposite(frame_r.get(), r, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_TRUE(r[0].img_buffer != nullptr);
    const size_t nbytes = static_cast<size_t>(r[0].img_width) * static_cast<size_t>(r[0].img_height) * 3;
    out.assign(r[0].img_buffer, r[0].img_buffer + nbytes);
  };

  // Establish server-side baseline display so `pathA_baseline` and `pathB_baseline` are shared.
  // Uses PushDisplayState (the same API both paths converge on) so the baseline itself is not a
  // hidden divergence source.
  const gui::GuiState baseline = MakeBaseline();
  EXPECT_TRUE(gui::PushDisplayState(baseline, server));
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
    EXPECT_TRUE(gui::PushDisplayState(baseline, server));  // reset server display to baseline
    std::vector<uint8_t> reset_check;
    ReadComposite(reset_check);
    EXPECT_EQ(std::memcmp(reset_check.data(), composite_baseline.data(), composite_baseline.size()), 0);

    gui::GuiState sA = MakeBaseline();
    SeedDisplayBaseline(sA);
    m.apply(sA);
    gui::GuiEffects effA = gui::ReconcileGuiEffects(sA);
    // Every mutation touches at least one display-state sub-field or raypath_color_mode → the
    // reconciler MUST route it to need_display_push (structural lane is quiet: no combine/match
    // change in these mutations).
    EXPECT_TRUE(effA.need_display_push);
    EXPECT_TRUE(!effA.need_hard_reset);
    EXPECT_TRUE(!effA.need_resim);
    gui::ApplyGuiEffects(sA, server, effA);
    std::vector<uint8_t> compositeA;
    ReadComposite(compositeA);

    // Sanity: the mutation actually rendered (composite bytes differ from baseline).
    EXPECT_TRUE(compositeA.size() == composite_baseline.size());
    EXPECT_TRUE(std::memcmp(compositeA.data(), composite_baseline.data(), composite_baseline.size()) != 0);

    // Reconciler must also have updated last_pushed_display_state on success so a follow-up
    // reconcile with no further mutation is a quiet no-op (edge-triggered contract).
    EXPECT_TRUE(sA.last_pushed_display_state.has_value());
    gui::GuiEffects effA_quiet = gui::ReconcileGuiEffects(sA);
    EXPECT_TRUE(!effA_quiet.need_display_push);

    // -- Path B: direct PushDisplayState --
    EXPECT_TRUE(gui::PushDisplayState(baseline, server));  // reset server display to baseline
    gui::GuiState sB = MakeBaseline();
    m.apply(sB);
    EXPECT_TRUE(gui::PushDisplayState(sB, server));
    std::vector<uint8_t> compositeB;
    ReadComposite(compositeB);

    // ⭐ AC3 core assertion: reconciler-driven and direct-push paths produce byte-identical
    // composite. Both funnel through PushDisplayState with the same GuiState → same C-API args →
    // same server-side snapshot_dirty_ → same Phase-2 rebake. Any drift here would signal the
    // reconciler is mutating state between diff and push, or ApplyGuiEffects is calling something
    // other than PushDisplayState on the need_display_push branch.
    EXPECT_EQ(compositeA.size(), compositeB.size());
    EXPECT_EQ(std::memcmp(compositeA.data(), compositeB.data(), compositeA.size()), 0);
  }

  // WakeForRefresh inside PushDisplayState may have started the global poller worker; Stop it so
  // subsequent tests start clean.
  gui::g_server_poller.Stop();
  LUMICE_DestroyServer(server);
}
