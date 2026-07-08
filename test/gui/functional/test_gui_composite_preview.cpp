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

  // code-review-03 regression: actually drives ServerPoller::PopulateCompositePayload()'s
  // generation-drift decision branch (not just the C-API-level drift precondition that
  // RaypathColorApi.CompositeGenerationDriftDetectableViaRecheck in test_c_api.cpp pins).
  // Arms a REAL drift via LUMICE_SetRaypathColors (same deterministic technique as that C-API
  // test) between an xyz-generation capture and the composite call, then calls
  // PopulateCompositePayloadForTest with the STALE captured generation so the function's own
  // internal regen_check genuinely observes a mismatch — proving the drop branch
  // (rgb_data cleared, is_composite left false) actually executes. Then repeats the sequence
  // without an intervening drift to prove the very next pairing recovers (is_composite==true,
  // bytes match a direct LUMICE_GetCompositeResults read).
  ImGuiTest* t3 = IM_REGISTER_TEST(engine, "gui_composite_preview", "generation_drift_drop_then_recover");
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

    // ---- Drop branch: arm a real drift in the exact window PopulateCompositePayload guards. ----
    LUMICE_RawXyzResult xyz1[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetRawXyzResults(server, xyz1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    const unsigned long long stale_xyz_generation = xyz1[0].snapshot_generation;

    // kColorConfig commits exactly one raypath_color class — class_count must match.
    LUMICE_ColorClassDisplay disp[1]{};
    disp[0].color[2] = 1.0f;  // red->blue: forces a visible display-state change (dirty).
    disp[0].visible = 1;
    IM_CHECK_EQ(LUMICE_SetRaypathColors(server, disp, 1, nullptr, LUMICE_COLOR_MODE_DOMINANT), LUMICE_OK);

    // Consumes the freshly-armed dirty flag: comp1 belongs to a generation newer than
    // stale_xyz_generation — exactly the mismatch PopulateCompositePayload's regen_check must
    // detect when handed stale_xyz_generation below.
    LUMICE_RenderResult comp1[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, comp1, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(comp1[0].img_buffer != nullptr);

    gui::TexturePayload drop_payload;
    local.PopulateCompositePayloadForTest(server, comp1[0], stale_xyz_generation, &drop_payload);
    IM_CHECK(!drop_payload.is_composite);
    IM_CHECK(drop_payload.rgb_data.empty());

    // ---- Recovery: the very next correctly-paired call must succeed. ----
    LUMICE_RawXyzResult xyz2[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetRawXyzResults(server, xyz2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    const unsigned long long paired_xyz_generation = xyz2[0].snapshot_generation;

    LUMICE_RenderResult comp2[LUMICE_MAX_RENDER_RESULTS + 1]{};
    IM_CHECK_EQ(LUMICE_GetCompositeResults(server, comp2, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    IM_CHECK(comp2[0].img_buffer != nullptr);
    const size_t nbytes = static_cast<size_t>(comp2[0].img_width) * static_cast<size_t>(comp2[0].img_height) * 3;

    gui::TexturePayload recovered_payload;
    local.PopulateCompositePayloadForTest(server, comp2[0], paired_xyz_generation, &recovered_payload);
    IM_CHECK(recovered_payload.is_composite);
    IM_CHECK_EQ(recovered_payload.rgb_data.size(), nbytes);
    IM_CHECK_EQ(std::memcmp(recovered_payload.rgb_data.data(), comp2[0].img_buffer, nbytes), 0);

    local.Stop();
    LUMICE_DestroyServer(server);
  };
}
