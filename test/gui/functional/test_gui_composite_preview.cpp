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
#include <cstring>
#include <thread>
#include <vector>

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
  //   (a) MECHANISM: the poll after SetRaypathColors + EnsureRunning yields a payload whose
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
    // For this synchronous test seam, EnsureRunning + PollOnceForTest together stand in for
    // "worker wakes up and runs one PollOnce, then self-pauses at COMPLETED".
    gui::g_server_poller.EnsureRunning(server);

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

    // Post-test cleanup on both the local ServerPoller and the global one that EnsureRunning
    // above nudged into kRunning — Stop() is synchronous and idempotent.
    local.Stop();
    gui::g_server_poller.Stop();
    LUMICE_DestroyServer(server);
  };
}
