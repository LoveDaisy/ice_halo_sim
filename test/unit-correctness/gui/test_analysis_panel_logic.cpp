// The Raypath Analysis panel's logic (src/gui/analysis_panel.cpp), driven without a frame: the
// in-progress derivation, result adoption and the generation gate, the display-time ring sum and
// re-sort, the click's pixel arithmetic, the request, and the exclude closure's eligibility and
// filter write. Each case is a proposition about one unit; the wiring into frames and the server
// is gui_test's (test/gui/functional/test_raypath_analysis_panel.cpp).

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "gui/analysis_panel.hpp"
#include "gui/annotation_anchors.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/server_poller.hpp"
#include "include/lumice.h"

namespace lumice::gui {
namespace {

// A payload with `n` single-crystal entries whose energies are NOT in descending order (the C API
// would deliver them sorted; the cases below need to prove the sort is done here and not assumed).
// Entry i: crystal 0, faces (i+1)-(i+2), energy given, count 100*(i+1), rings summing to energy.
std::shared_ptr<AnalysisPayload> MakePayload(unsigned long long gen, int roi_mode, const std::vector<double>& energies,
                                             int ring_count = 0, float cone_radius_deg = 0.0f) {
  auto p = std::make_shared<AnalysisPayload>();
  p->snapshot_generation = gen;
  p->roi_mode = roi_mode;
  p->cone_ring_count = ring_count;
  p->cone_radius_rad = cone_radius_deg * 3.14159265358979323846f / 180.0f;
  for (size_t i = 0; i < energies.size(); ++i) {
    LUMICE_RaypathHistogramEntry e{};
    e.chain_len = 1;
    e.chain[0].crystal_id = 0;
    e.chain[0].segment_len = 2;
    e.chain[0].segment[0] = static_cast<int>(i) + 1;
    e.chain[0].segment[1] = static_cast<int>(i) + 2;
    snprintf(e.display, sizeof(e.display), "%d-%d", e.chain[0].segment[0], e.chain[0].segment[1]);
    e.energy = energies[i];
    e.count = 100 * (static_cast<LUMICE_RayCount>(i) + 1);
    e.ring_count = ring_count;
    // Rings: all of the energy in ring (i mod ring_count), so which rings the slider covers decides
    // which entries show energy at all.
    if (ring_count > 0) {
      e.ring_energy[static_cast<size_t>(i) % static_cast<size_t>(ring_count)] = energies[i];
    }
    p->entries.push_back(e);
  }
  return p;
}

PreviewSnapshot Snap(bool valid, int lifecycle) {
  PreviewSnapshot s;
  s.valid = valid;
  s.lifecycle = lifecycle;
  return s;
}

// ---- lifecycle ----

TEST(AnalysisPanelLogic, InProgressIsIntentAndObservation) {
  const PreviewSnapshot running = Snap(true, LUMICE_LIFECYCLE_RUNNING);
  const PreviewSnapshot completed = Snap(true, LUMICE_LIFECYCLE_COMPLETED);
  const PreviewSnapshot idle = Snap(true, LUMICE_LIFECYCLE_IDLE);
  const PreviewSnapshot wake_edge = Snap(false, LUMICE_LIFECYCLE_RUNNING);
  // Not started: never in progress, whatever is observed.
  for (const PreviewSnapshot* s :
       { &running, &completed, &idle, &wake_edge, static_cast<const PreviewSnapshot*>(nullptr) }) {
    EXPECT_FALSE(DeriveAnalysisInProgress(false, s));
  }
  // Started: in progress until a VALID observation says the run is not RUNNING.
  EXPECT_TRUE(DeriveAnalysisInProgress(true, nullptr));
  EXPECT_TRUE(DeriveAnalysisInProgress(true, &wake_edge));
  EXPECT_TRUE(DeriveAnalysisInProgress(true, &running));
  EXPECT_FALSE(DeriveAnalysisInProgress(true, &completed));
  EXPECT_FALSE(DeriveAnalysisInProgress(true, &idle)) << "a Stop reads IDLE and ends the run";
}

// ---- adoption / the generation gate ----

TEST(AnalysisPanelLogic, NullPayloadIsNeverAdopted) {
  GuiState state;
  EXPECT_FALSE(AdoptAnalysisPayloadIfNew(state, nullptr));
  EXPECT_EQ(state.analysis_result.payload, nullptr);
}

TEST(AnalysisPanelLogic, FirstPayloadIsAdoptedAndOrderedWithoutAnySliderEvent) {
  GuiState state;
  // Full sky: unsorted energies must come out energy-descending from adoption alone.
  auto p = MakePayload(7, LUMICE_RAYPATH_ROI_FULL_SKY, { 1.0, 5.0, 3.0 });
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));
  ASSERT_EQ(state.analysis_result.payload, p);
  ASSERT_EQ(state.analysis_result.display_order.size(), 3u);
  EXPECT_EQ(state.analysis_result.display_order[0], 1);
  EXPECT_EQ(state.analysis_result.display_order[1], 2);
  EXPECT_EQ(state.analysis_result.display_order[2], 0);
  EXPECT_DOUBLE_EQ(state.analysis_result.display_total, 9.0);
  EXPECT_EQ(state.analysis_result.display_ring_count, 0) << "no rings outside CONE";
}

TEST(AnalysisPanelLogic, ConePayloadIsOrderedByTheDefaultRadiusOnAdoption) {
  GuiState state;
  // 4 rings over 4 degrees = 1 degree each; the default radius (2 degrees) covers rings 0 and 1.
  // Entry i puts its energy in ring i mod 4: entries 0,1 are inside, 2,3 outside.
  state.analysis.cone_radius_deg = 2.0f;
  auto p = MakePayload(3, LUMICE_RAYPATH_ROI_CONE, { 2.0, 9.0, 50.0, 60.0 }, 4, 4.0f);
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));
  EXPECT_EQ(state.analysis_result.display_ring_count, 2);
  ASSERT_EQ(state.analysis_result.display_order.size(), 4u);
  EXPECT_EQ(state.analysis_result.display_order[0], 1) << "9.0 inside the radius beats 50/60 outside it";
  EXPECT_EQ(state.analysis_result.display_order[1], 0);
  EXPECT_DOUBLE_EQ(state.analysis_result.display_energy[2], 0.0);
  EXPECT_DOUBLE_EQ(state.analysis_result.display_energy[3], 0.0);
  EXPECT_DOUBLE_EQ(state.analysis_result.display_total, 11.0);
}

// The generation gate. The same result observed over many polls (the poller carries the payload
// forward, and `present` is true on every one of them) must not reset the selection or reorder
// the list; only a payload with a NEW generation does. Red-state probe: make the gate adopt on
// non-null alone (what a `present`-keyed gate amounts to) and the "same generation" half fails.
TEST(AnalysisPanelLogic, SameGenerationObservedRepeatedlyKeepsSelectionAndOrder) {
  GuiState state;
  auto p = MakePayload(11, LUMICE_RAYPATH_ROI_FULL_SKY, { 1.0, 5.0, 3.0 });
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));
  state.analysis.selected_entry = "3-4";
  const std::vector<int> order_before = state.analysis_result.display_order;
  // Three more polls of the same generation — a carried-forward pointer and a fresh copy alike.
  for (int i = 0; i < 3; ++i) {
    auto same = i == 0 ? p : std::make_shared<AnalysisPayload>(*p);
    EXPECT_FALSE(AdoptAnalysisPayloadIfNew(state, same)) << "poll " << i;
    EXPECT_EQ(state.analysis.selected_entry, std::optional<std::string>{ "3-4" }) << "poll " << i;
    EXPECT_EQ(state.analysis_result.display_order, order_before);
    EXPECT_EQ(state.analysis_result.payload, p) << "the held object is not replaced either";
  }
  // Generation 12: adopted, selection reset, order recomputed for the new entries.
  auto next = MakePayload(12, LUMICE_RAYPATH_ROI_FULL_SKY, { 4.0, 1.0 });
  EXPECT_TRUE(AdoptAnalysisPayloadIfNew(state, next));
  EXPECT_FALSE(state.analysis.selected_entry.has_value());
  EXPECT_EQ(state.analysis_result.payload, next);
  ASSERT_EQ(state.analysis_result.display_order.size(), 2u);
  EXPECT_EQ(state.analysis_result.display_order[0], 0);
}

// ---- the selection, by chain ----

// The selection is the chain's text, so it is found again in any payload that still has that
// chain as a row — and is nothing in one that does not (a chain merged away by a symmetry
// change, or a different result), without ever indexing past the entries.
TEST(AnalysisPanelLogic, SelectedAnalysisEntryIsFoundByChainTextOrNotAtAll) {
  GuiState state;
  EXPECT_EQ(SelectedAnalysisEntry(state), nullptr) << "no result, no selection";
  auto p = MakePayload(3, LUMICE_RAYPATH_ROI_FULL_SKY, { 1.0, 5.0, 3.0 });
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));
  EXPECT_EQ(SelectedAnalysisEntry(state), nullptr) << "a result, no selection";
  state.analysis.selected_entry = "2-3";
  EXPECT_EQ(SelectedAnalysisEntry(state), &p->entries[1]);
  state.analysis.selected_entry = "9-9";
  EXPECT_EQ(SelectedAnalysisEntry(state), nullptr) << "no row carries that chain";
  // The same chain in a re-read payload (another symmetry, same generation) is found again —
  // the payload object is replaced, the key still names a row.
  auto reread = MakePayload(3, LUMICE_RAYPATH_ROI_FULL_SKY, { 4.0, 2.0 });
  state.analysis.selected_entry = "2-3";
  state.analysis_result.payload = reread;
  EXPECT_EQ(SelectedAnalysisEntry(state), &reread->entries[1]);
  state.analysis.selected_entry = "3-4";
  EXPECT_EQ(SelectedAnalysisEntry(state), nullptr) << "merged away: cleared by definition";
}

// ---- the symmetry, and when the entries need re-reading ----

TEST(AnalysisPanelLogic, SymmetryBitsFollowTheCheckboxesAndDefaultToAll) {
  GuiState state;
  EXPECT_EQ(AnalysisSymmetryBits(state),
            LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D)
      << "the default is the P|B|D every analysis used to be recorded under";
  state.analysis.symmetry_d = false;
  EXPECT_EQ(AnalysisSymmetryBits(state), LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B);
  state.analysis.symmetry_p = false;
  EXPECT_EQ(AnalysisSymmetryBits(state), LUMICE_RAYPATH_SYMMETRY_B);
  state.analysis.symmetry_b = false;
  EXPECT_EQ(AnalysisSymmetryBits(state), 0);
}

// The refresh gate: stale iff a result is held and it has not been read as (this generation,
// these bits). "Never read" is the explicit flag, not a sentinel value — a zeroed record with
// fetched_once false must read as stale even against a payload whose generation happened to be
// what the record holds.
TEST(AnalysisPanelLogic, EntriesNeedRefreshOnFirstReadNewGenerationOrNewSymmetry) {
  GuiState state;
  EXPECT_FALSE(AnalysisEntriesNeedRefresh(state)) << "nothing held, nothing to read";
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, MakePayload(7, LUMICE_RAYPATH_ROI_FULL_SKY, { 1.0 })));
  EXPECT_TRUE(AnalysisEntriesNeedRefresh(state)) << "held, never read";
  // As RefreshAnalysisEntries records a read.
  state.analysis.fetched_once = true;
  state.analysis.fetched_generation = 7;
  state.analysis.fetched_symmetry = AnalysisSymmetryBits(state);
  EXPECT_FALSE(AnalysisEntriesNeedRefresh(state)) << "read as (7, P|B|D): up to date";
  state.analysis.symmetry_b = false;
  EXPECT_TRUE(AnalysisEntriesNeedRefresh(state)) << "the bits changed";
  state.analysis.fetched_symmetry = AnalysisSymmetryBits(state);
  EXPECT_FALSE(AnalysisEntriesNeedRefresh(state));
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, MakePayload(8, LUMICE_RAYPATH_ROI_FULL_SKY, { 1.0 })));
  EXPECT_TRUE(AnalysisEntriesNeedRefresh(state)) << "a new generation";
  state.analysis.fetched_generation = 8;
  EXPECT_FALSE(AnalysisEntriesNeedRefresh(state));
  state.analysis.fetched_once = false;
  EXPECT_TRUE(AnalysisEntriesNeedRefresh(state)) << "the flag alone decides 'never read', not the values";
  // And with no server there is nothing to read from: a no-op, the record untouched.
  EXPECT_FALSE(RefreshAnalysisEntries(state, nullptr));
  EXPECT_FALSE(state.analysis.fetched_once);
}

// ---- rings and the slider ----

TEST(AnalysisPanelLogic, RingsWithinRadiusCoversTheRadiusAndClamps) {
  // 30 rings over 15 degrees: 0.5 degrees each.
  EXPECT_EQ(RingsWithinRadius(2.0f, 15.0f, 30), 4);
  EXPECT_EQ(RingsWithinRadius(0.5f, 15.0f, 30), 1);
  EXPECT_EQ(RingsWithinRadius(0.75f, 15.0f, 30), 2) << "a partial ring counts whole";
  EXPECT_EQ(RingsWithinRadius(15.0f, 15.0f, 30), 30);
  EXPECT_EQ(RingsWithinRadius(99.0f, 15.0f, 30), 30) << "clamped to the cone";
  EXPECT_EQ(RingsWithinRadius(0.0f, 15.0f, 30), 1) << "never fewer than one ring";
  EXPECT_EQ(RingsWithinRadius(-1.0f, 15.0f, 30), 1);
  EXPECT_EQ(RingsWithinRadius(2.0f, 15.0f, 0), 0) << "no rings, no answer";
}

TEST(AnalysisPanelLogic, SumRingEnergyIsAPrefixSumClampedToTheEntry) {
  LUMICE_RaypathHistogramEntry e{};
  e.ring_count = 3;
  e.ring_energy[0] = 1.0;
  e.ring_energy[1] = 2.0;
  e.ring_energy[2] = 4.0;
  e.ring_energy[3] = 100.0;  // past ring_count: must never be read
  EXPECT_DOUBLE_EQ(SumRingEnergy(e, 0), 0.0);
  EXPECT_DOUBLE_EQ(SumRingEnergy(e, 1), 1.0);
  EXPECT_DOUBLE_EQ(SumRingEnergy(e, 2), 3.0);
  EXPECT_DOUBLE_EQ(SumRingEnergy(e, 3), 7.0);
  EXPECT_DOUBLE_EQ(SumRingEnergy(e, 8), 7.0);
  EXPECT_DOUBLE_EQ(SumRingEnergy(e, -1), 0.0);
}

// The slider re-sorts the rows and moves the selection WITH its entry: the selection names the
// chain, and SelectedAnalysisEntry finds the same entry whatever row it is shown on. Also the AC3
// statement at the unit level — nothing in this path touches the lifecycle (there is no server
// here to touch).
TEST(AnalysisPanelLogic, SliderReorderMovesRowsNotTheSelection) {
  GuiState state;
  state.analysis.cone_radius_deg = 1.0f;  // ring 0 only
  auto p = MakePayload(5, LUMICE_RAYPATH_ROI_CONE, { 2.0, 9.0, 50.0, 60.0 }, 4, 4.0f);
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));
  EXPECT_EQ(state.analysis_result.display_ring_count, 1);
  EXPECT_EQ(state.analysis_result.display_order[0], 0) << "only entry 0 is inside ring 0";
  state.analysis.selected_entry = "1-2";
  EXPECT_EQ(SelectedAnalysisEntry(state), &p->entries[0]);

  state.analysis.cone_radius_deg = 4.0f;  // every ring
  RecomputeAnalysisDisplayOrder(state);
  EXPECT_EQ(state.analysis_result.display_ring_count, 4);
  ASSERT_EQ(state.analysis_result.display_order.size(), 4u);
  EXPECT_EQ(state.analysis_result.display_order[0], 3);
  EXPECT_EQ(state.analysis_result.display_order[3], 0) << "entry 0 is now the last row";
  ASSERT_TRUE(state.analysis.selected_entry.has_value());
  EXPECT_EQ(SelectedAnalysisEntry(state), &p->entries[0]) << "still entry 0, shown on the last row";
  EXPECT_EQ(state.analysis_result.payload, p) << "the data itself is untouched";
  EXPECT_DOUBLE_EQ(state.analysis_result.display_total, 121.0);
}

// ---- the click ----

TEST(AnalysisPanelLogic, PreviewPointToCanvasPixelFloorsAndScalesByDpi) {
  // 1x: points are pixels.
  auto p = PreviewPointToCanvasPixel(10.7f, 3.2f, 1.0f, 1.0f, 100, 50);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->px, 10);
  EXPECT_EQ(p->py, 3);
  // Retina 2x: a point is two device pixels, so (10.7, 3.2) points is pixel (21, 6).
  p = PreviewPointToCanvasPixel(10.7f, 3.2f, 2.0f, 2.0f, 100, 50);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->px, 21);
  EXPECT_EQ(p->py, 6);
  // Outside the canvas on either side is no pixel at all.
  EXPECT_FALSE(PreviewPointToCanvasPixel(-0.5f, 3.0f, 1.0f, 1.0f, 100, 50).has_value());
  EXPECT_FALSE(PreviewPointToCanvasPixel(50.0f, 25.0f, 2.0f, 2.0f, 100, 50).has_value()) << "2x: pixel 100 is off";
  EXPECT_TRUE(PreviewPointToCanvasPixel(49.9f, 24.9f, 2.0f, 2.0f, 100, 50).has_value());
  // And back: the pixel's centre, in points.
  float x = 0.0f;
  float y = 0.0f;
  CanvasPixelToPreviewPoint(21, 6, 2.0f, 2.0f, &x, &y);
  EXPECT_FLOAT_EQ(x, 10.75f);
  EXPECT_FLOAT_EQ(y, 3.25f);
}

TEST(AnalysisPanelLogic, BuildAnnotationViewCopiesEveryKeyField) {
  AnnotationAnchors::ViewKey key;
  key.width = 640;
  key.height = 480;
  key.lens_type = LUMICE_LENS_TYPE_LINEAR;
  key.fov = 42.5f;
  key.azimuth = 12.0f;
  key.elevation = -7.0f;
  key.roll = 3.0f;
  key.visible = LUMICE_VISIBLE_UPPER;
  key.overlap = 0.25f;
  key.front = true;
  const LUMICE_AnnotationView v = BuildAnnotationView(key);
  EXPECT_EQ(v.width, 640);
  EXPECT_EQ(v.height, 480);
  EXPECT_EQ(v.lens_type, LUMICE_LENS_TYPE_LINEAR);
  EXPECT_FLOAT_EQ(v.lens_fov, 42.5f);
  EXPECT_FLOAT_EQ(v.view_azimuth, 12.0f);
  EXPECT_FLOAT_EQ(v.view_elevation, -7.0f);
  EXPECT_FLOAT_EQ(v.view_roll, 3.0f);
  EXPECT_EQ(v.visible, LUMICE_VISIBLE_UPPER);
  EXPECT_FLOAT_EQ(v.overlap, 0.25f);
  EXPECT_EQ(v.front, 1);
  EXPECT_EQ(v.lens_shift[0], 0);
  EXPECT_EQ(v.lens_shift[1], 0);
  key.front = false;
  EXPECT_EQ(BuildAnnotationView(key).front, 0);
}

TEST(AnalysisPanelLogic, PickWritesTheCentreOnSkyAndNothingOffSky) {
  GuiState state;
  state.renderer.lens_type = LUMICE_LENS_TYPE_LINEAR;
  state.renderer.fov = 60.0f;
  state.renderer.azimuth = 0.0f;
  state.renderer.elevation = 30.0f;
  state.renderer.roll = 0.0f;
  const LUMICE_AnnotationView view = PreviewAnnotationView(state, 200, 100);
  state.analysis.pick_armed = true;
  ASSERT_TRUE(PickAnalysisConeCenter(state, view, 100, 50));
  EXPECT_TRUE(state.analysis.cone_center_valid);
  EXPECT_FALSE(state.analysis.pick_armed) << "the click that picks disarms";
  // The same pixel through the C API directly: identical direction (AC2's "the centre direction
  // is the click's unprojection", stated as equality with the oracle rather than as a property).
  float expect[3] = { 0.0f, 0.0f, 0.0f };
  int valid = 0;
  ASSERT_EQ(LUMICE_UnprojectPixel(&view, 100, 50, expect, &valid), LUMICE_OK);
  ASSERT_EQ(valid, 1);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(state.analysis.cone_center_dir[i], expect[i]);
  }
  // The frame centre at elevation 30 looks 30 degrees up: altitude = asin(-z).
  EXPECT_NEAR(std::asin(-state.analysis.cone_center_dir[2]) * 180.0f / 3.14159265f, 30.0f, 0.5f);

  // Off sky: a pixel outside the canvas writes nothing and leaves the pick armed.
  state.analysis.pick_armed = true;
  const float before[3] = { state.analysis.cone_center_dir[0], state.analysis.cone_center_dir[1],
                            state.analysis.cone_center_dir[2] };
  EXPECT_FALSE(PickAnalysisConeCenter(state, view, 5000, 50));
  EXPECT_TRUE(state.analysis.pick_armed);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(state.analysis.cone_center_dir[i], before[i]);
  }
}

// ---- the marker: the direction placed on the picture each frame ----

TEST(AnalysisPanelLogic, MarkerIsThePickedPixelAndFollowsTheView) {
  GuiState state;
  state.renderer.lens_type = LUMICE_LENS_TYPE_LINEAR;
  state.renderer.fov = 60.0f;
  state.renderer.azimuth = 0.0f;
  state.renderer.elevation = 30.0f;
  const LUMICE_AnnotationView view = PreviewAnnotationView(state, 200, 100);
  EXPECT_FALSE(ProjectConeCenterMarker(state, view).has_value()) << "no centre, no marker";
  ASSERT_TRUE(PickAnalysisConeCenter(state, view, 120, 40));
  // Round trip: the pixel picked is the pixel the marker lands on, exactly (the inverse returns
  // the pixel's centre and the forward bins it back — the C API's own round-trip guarantee).
  const std::optional<CanvasPixel> same = ProjectConeCenterMarker(state, view);
  ASSERT_TRUE(same.has_value());
  EXPECT_EQ(same->px, 120);
  EXPECT_EQ(same->py, 40);
  // The view turns; the direction does not; the marker moves with the picture. The camera
  // yawing 10 degrees (azimuth +10) shifts the fixed direction sideways by roughly 10 deg of
  // bearing foreshortened by the 30-degree elevation, at the focal length the SHORT side sets:
  // 100 / (2 tan 30) * tan(10 cos 30) = 13 px. A loose oracle on purpose — the sign and the
  // exact figure are the lens's business (pinned in core); what this case pins is that the
  // marker moves by the view's amount while the direction stays put.
  const float dir_before[3] = { state.analysis.cone_center_dir[0], state.analysis.cone_center_dir[1],
                                state.analysis.cone_center_dir[2] };
  state.renderer.azimuth = 10.0f;
  const LUMICE_AnnotationView turned = PreviewAnnotationView(state, 200, 100);
  const std::optional<CanvasPixel> moved = ProjectConeCenterMarker(state, turned);
  ASSERT_TRUE(moved.has_value());
  EXPECT_NE(moved->px, 120);
  EXPECT_NEAR(std::abs(moved->px - 120), 13, 4);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(state.analysis.cone_center_dir[i], dir_before[i]) << "the direction is the truth";
  }
  // Turned far enough that the direction leaves the 60-degree frame: no marker, the centre
  // still valid (the list is unaffected; only the drawing is absent).
  state.renderer.azimuth = 90.0f;
  EXPECT_FALSE(ProjectConeCenterMarker(state, PreviewAnnotationView(state, 200, 100)).has_value());
  EXPECT_TRUE(state.analysis.cone_center_valid);
}

// AC5: the arbiter's truth table. Three inputs combinations that matter, three exclusive owners,
// and the one ruling worth pinning on its own — hover beats an armed pick.
TEST(AnalysisPanelLogic, ConeInputArbiterTruthTable) {
  EXPECT_EQ(ArbitrateConeInput(false, false), ConeInputOwner::kCamera);
  EXPECT_EQ(ArbitrateConeInput(false, true), ConeInputOwner::kPickClick);
  EXPECT_EQ(ArbitrateConeInput(true, false), ConeInputOwner::kMarkerDrag);
  EXPECT_EQ(ArbitrateConeInput(true, true), ConeInputOwner::kMarkerDrag) << "the marker under the cursor wins";
}

TEST(AnalysisPanelLogic, DragWritesTheDirectionOnSkyAndNothingElse) {
  // A linear lens on the horizon showing the upper hemisphere only: the top half of the canvas
  // is sky, the bottom half is clipped — a drag that crosses the horizon must stop writing.
  GuiState state;
  state.renderer.lens_type = LUMICE_LENS_TYPE_LINEAR;
  state.renderer.fov = 60.0f;
  state.renderer.elevation = 0.0f;
  state.renderer.visible = LUMICE_VISIBLE_UPPER;
  const LUMICE_AnnotationView view = PreviewAnnotationView(state, 200, 100);
  state.analysis.pick_armed = true;
  ASSERT_TRUE(PickAnalysisConeCenter(state, view, 100, 20));
  state.analysis.pick_armed = true;  // a drag must leave an armed pick armed
  float want[3] = { 0.0f, 0.0f, 0.0f };
  int valid = 0;
  ASSERT_EQ(LUMICE_UnprojectPixel(&view, 130, 30, want, &valid), LUMICE_OK);
  ASSERT_EQ(valid, 1);
  EXPECT_TRUE(DragAnalysisConeCenter(state, view, 130, 30));
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(state.analysis.cone_center_dir[i], want[i]) << "the drag is the pixel's unprojection, verbatim";
  }
  EXPECT_TRUE(state.analysis.pick_armed);
  EXPECT_TRUE(state.analysis.cone_center_valid);
  // Below the horizon (clipped by `upper`): nothing written, the last sky direction kept.
  int below_valid = 1;
  float below[3] = { 0.0f, 0.0f, 0.0f };
  ASSERT_EQ(LUMICE_UnprojectPixel(&view, 100, 90, below, &below_valid), LUMICE_OK);
  ASSERT_EQ(below_valid, 0) << "the case needs a pixel that is not sky";
  EXPECT_FALSE(DragAnalysisConeCenter(state, view, 100, 90));
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(state.analysis.cone_center_dir[i], want[i]);
  }
  EXPECT_FALSE(DragAnalysisConeCenter(state, view, 5000, 30)) << "outside the canvas";
}

TEST(AnalysisPanelLogic, DefaultCentreIsTheViewportCentreOnceAndOnlyWhenOnSky) {
  GuiState state;
  state.renderer.lens_type = LUMICE_LENS_TYPE_LINEAR;
  state.renderer.fov = 60.0f;
  state.renderer.elevation = 30.0f;
  const LUMICE_AnnotationView view = PreviewAnnotationView(state, 200, 100);
  float want[3] = { 0.0f, 0.0f, 0.0f };
  int valid = 0;
  ASSERT_EQ(LUMICE_UnprojectPixel(&view, 100, 50, want, &valid), LUMICE_OK);
  ASSERT_EQ(valid, 1);
  EXPECT_TRUE(EnsureDefaultConeCenter(state, view, 200, 100));
  EXPECT_TRUE(state.analysis.cone_center_valid);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(state.analysis.cone_center_dir[i], want[i]);
  }
  // A view whose centre pixel is not sky: `lower` hides the direction the centre looks at
  // (elevation 30, above the horizon). Nothing placed, nothing marked valid.
  GuiState lower;
  lower.renderer.lens_type = LUMICE_LENS_TYPE_LINEAR;
  lower.renderer.fov = 60.0f;
  lower.renderer.elevation = 30.0f;
  lower.renderer.visible = LUMICE_VISIBLE_LOWER;
  EXPECT_FALSE(EnsureDefaultConeCenter(lower, PreviewAnnotationView(lower, 200, 100), 200, 100));
  EXPECT_FALSE(lower.analysis.cone_center_valid);
}

TEST(AnalysisPanelLogic, DriftHintNeedsAConeResultAndAMovedCentre) {
  GuiState state;
  state.analysis.cone_center_valid = true;
  state.analysis.cone_center_dir[0] = 0.0f;
  state.analysis.cone_center_dir[1] = 0.0f;
  state.analysis.cone_center_dir[2] = -1.0f;
  std::copy(state.analysis.cone_center_dir, state.analysis.cone_center_dir + 3,
            state.analysis.analyzed_cone_center_dir);
  EXPECT_FALSE(ConeCenterDriftedFromResult(state)) << "no result at all";
  state.analysis_result.payload = MakePayload(1, LUMICE_RAYPATH_ROI_FULL_SKY, { 1.0 });
  EXPECT_FALSE(ConeCenterDriftedFromResult(state)) << "not a cone result";
  state.analysis_result.payload = MakePayload(2, LUMICE_RAYPATH_ROI_CONE, { 1.0 }, 4, 2.0f);
  EXPECT_FALSE(ConeCenterDriftedFromResult(state)) << "same centre";
  // Float noise of a round trip is not a move; a visible move is.
  state.analysis.cone_center_dir[0] = 1e-5f;
  EXPECT_FALSE(ConeCenterDriftedFromResult(state));
  state.analysis.cone_center_dir[0] = std::sin(2.0f * 3.14159265f / 180.0f);
  state.analysis.cone_center_dir[2] = -std::cos(2.0f * 3.14159265f / 180.0f);
  EXPECT_TRUE(ConeCenterDriftedFromResult(state)) << "two degrees is a move";
  state.analysis.cone_center_valid = false;
  EXPECT_FALSE(ConeCenterDriftedFromResult(state)) << "no centre, nothing to compare";
}

TEST(AnalysisPanelLogic, RingRadiusFollowsTheLensScaleAtTheCentre) {
  GuiState state;
  state.renderer.lens_type = LUMICE_LENS_TYPE_LINEAR;
  state.renderer.fov = 40.0f;
  state.renderer.elevation = 20.0f;
  const LUMICE_AnnotationView view = PreviewAnnotationView(state, 400, 400);
  // A 40-degree linear lens on 400 px: 2*tan(20 deg) spans 400 px, so at the centre one degree is
  // about 400 / (2 * tan 20) * tan 1 = 9.6 px. Within 10 % — it is a local linearisation.
  const std::optional<float> r2 = ConeRingRadiusCanvasPx(view, 200, 200, 2.0f);
  ASSERT_TRUE(r2.has_value());
  EXPECT_NEAR(*r2, 19.2f, 2.0f);
  const std::optional<float> r4 = ConeRingRadiusCanvasPx(view, 200, 200, 4.0f);
  ASSERT_TRUE(r4.has_value());
  EXPECT_NEAR(*r4, 2.0f * *r2, 1e-3f) << "linear in the radius by construction";
  EXPECT_FALSE(ConeRingRadiusCanvasPx(view, 5000, 200, 2.0f).has_value()) << "no sky, no ring";
}

// ---- the request ----

TEST(AnalysisPanelLogic, RequestCarriesTheFullConeAndTheSessionBudget) {
  GuiState state;
  state.analysis.roi_mode = LUMICE_RAYPATH_ROI_CONE;
  state.analysis.cone_center_valid = true;
  state.analysis.cone_center_dir[0] = 0.1f;
  state.analysis.cone_center_dir[1] = 0.2f;
  state.analysis.cone_center_dir[2] = -0.9f;
  state.analysis.cone_radius_deg = 1.0f;  // the slider: must NOT reach the request
  state.analysis.ray_num_millions = 12.5f;
  state.analysis.infinite = false;
  const LUMICE_RaypathAnalysisRequest req = BuildAnalysisRequest(state, 100, 100);
  EXPECT_EQ(req.roi_mode, LUMICE_RAYPATH_ROI_CONE);
  EXPECT_FLOAT_EQ(req.cone_center[0], 0.1f);
  EXPECT_FLOAT_EQ(req.cone_center[2], -0.9f);
  EXPECT_NEAR(req.cone_radius_rad, kAnalysisConeMaxRadiusDeg * 3.14159265f / 180.0f, 1e-6f);
  EXPECT_EQ(req.cone_ring_count, kAnalysisConeRingCount);
  // The budget is the session's, explicit — never the scene-default sentinel — and to the ray.
  EXPECT_EQ(req.infinite, 0);
  EXPECT_EQ(req.ray_num, 12500000u);

  // An unlimited budget reaches the request as such.
  state.analysis.infinite = true;
  const LUMICE_RaypathAnalysisRequest cone2 = BuildAnalysisRequest(state, 100, 100);
  EXPECT_EQ(cone2.infinite, 1);
  state.analysis.infinite = false;

  state.analysis.roi_mode = LUMICE_RAYPATH_ROI_IN_FRAME;
  state.renderer.lens_type = LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA;
  state.renderer.fov = 120.0f;
  const LUMICE_RaypathAnalysisRequest in_frame = BuildAnalysisRequest(state, 320, 200);
  EXPECT_EQ(in_frame.roi_mode, LUMICE_RAYPATH_ROI_IN_FRAME);
  EXPECT_EQ(in_frame.frame_view.width, 320);
  EXPECT_EQ(in_frame.frame_view.height, 200);
  EXPECT_EQ(in_frame.frame_view.lens_type, LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA);
  EXPECT_FLOAT_EQ(in_frame.frame_view.lens_fov, 120.0f);
  EXPECT_EQ(in_frame.ray_num, 12500000u) << "the budget is not a CONE-only field";

  state.analysis.roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  const LUMICE_RaypathAnalysisRequest full = BuildAnalysisRequest(state, 1, 1);
  EXPECT_EQ(full.roi_mode, LUMICE_RAYPATH_ROI_FULL_SKY);
  EXPECT_EQ(full.ray_num, 12500000u);
  EXPECT_EQ(full.infinite, 0);
}

// The session's ray budget is seeded from the document ONCE — the first call copies, a later
// document edit does not reach it, and a session that was seeded is not re-seeded.
TEST(AnalysisPanelLogic, EnsureDefaultAnalysisRayBudgetSeedsOnceFromTheDocument) {
  GuiState state;
  state.sim.ray_num_millions = 33.0f;
  state.sim.infinite = true;
  EXPECT_FALSE(state.analysis.ray_budget_initialized);
  EnsureDefaultAnalysisRayBudget(state);
  EXPECT_TRUE(state.analysis.ray_budget_initialized);
  EXPECT_FLOAT_EQ(state.analysis.ray_num_millions, 33.0f);
  EXPECT_TRUE(state.analysis.infinite);

  state.sim.ray_num_millions = 99.0f;
  state.sim.infinite = false;
  EnsureDefaultAnalysisRayBudget(state);
  EXPECT_FLOAT_EQ(state.analysis.ray_num_millions, 33.0f) << "seeded once; the document's later edit stays its own";
  EXPECT_TRUE(state.analysis.infinite);

  // The user's edit survives the same way.
  state.analysis.ray_num_millions = 2.0f;
  state.analysis.infinite = false;
  EnsureDefaultAnalysisRayBudget(state);
  EXPECT_FLOAT_EQ(state.analysis.ray_num_millions, 2.0f);
  EXPECT_FALSE(state.analysis.infinite);

  // A fresh session (what a document switch leaves behind) seeds again, from the document as it
  // is then.
  state.analysis = GuiState::RaypathAnalysisSession{};
  EnsureDefaultAnalysisRayBudget(state);
  EXPECT_FLOAT_EQ(state.analysis.ray_num_millions, 99.0f);
  EXPECT_FALSE(state.analysis.infinite);
}

// ---- the exclude closure ----

// Two crystals in the pool, three entries over two layers: pool 1 is referenced first (so it is
// scene crystal 0), pool 0 second (scene crystal 1), and pool 0 twice — once per layer.
GuiState TwoCrystalDocument() {
  GuiState state;
  state.crystals.assign(2, CrystalConfig{});
  Layer l0;
  l0.probability = 1.0f;
  EntryCard e;
  e.crystal_id = 1;
  e.proportion = 50.0f;
  l0.entries.push_back(e);
  e.crystal_id = 0;
  l0.entries.push_back(e);
  Layer l1;
  l1.probability = 0.5f;
  e.crystal_id = 0;
  l1.entries.push_back(e);
  state.layers = { l0, l1 };
  return state;
}

TEST(AnalysisPanelLogic, CrystalPoolToCoreIdMapNumbersByFirstReference) {
  const GuiState state = TwoCrystalDocument();
  const std::map<int, int> m = ComputeCrystalPoolToCoreIdMap(state);
  ASSERT_EQ(m.size(), 2u);
  EXPECT_EQ(m.at(1), 0) << "pool 1 is referenced first";
  EXPECT_EQ(m.at(0), 1);
  // An unreferenced crystal is not committed and so has no id.
  GuiState spare = state;
  spare.crystals.emplace_back();
  EXPECT_EQ(ComputeCrystalPoolToCoreIdMap(spare).size(), 2u);
  // Empty document: empty map.
  EXPECT_TRUE(ComputeCrystalPoolToCoreIdMap(GuiState{}).empty());
}

TEST(AnalysisPanelLogic, ExcludeEligibilityDeniesEachReasonOnItsOwn) {
  GuiState state = TwoCrystalDocument();
  std::string why;
  EXPECT_EQ(EvaluateExcludeEligibility(state, &why), ExcludeEligibility::kNoSelection);
  EXPECT_FALSE(why.empty());

  auto p = MakePayload(1, LUMICE_RAYPATH_ROI_FULL_SKY, { 5.0, 3.0 });
  // Entry 1: a two-layer chain.
  p->entries[1].chain_len = 2;
  p->entries[1].chain[1] = p->entries[1].chain[0];
  // Both chains name scene crystal 1 (= pool 0).
  p->entries[0].chain[0].crystal_id = 1;
  p->entries[1].chain[0].crystal_id = 1;
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));

  state.analysis.selected_entry = "2-3";
  EXPECT_EQ(EvaluateExcludeEligibility(state, &why), ExcludeEligibility::kMultiSegment);
  state.analysis.selected_entry = "7-8";
  EXPECT_EQ(EvaluateExcludeEligibility(state, &why), ExcludeEligibility::kNoSelection)
      << "a chain no row carries is no selection";
  state.analysis.selected_entry = "1-2";
  EXPECT_EQ(EvaluateExcludeEligibility(state, &why), ExcludeEligibility::kOk);
  EXPECT_TRUE(why.empty());

  // The crystal list changed since the analysis: scene crystal 1 no longer exists.
  GuiState edited = state;
  edited.layers[0].entries.pop_back();
  edited.layers.pop_back();
  EXPECT_EQ(EvaluateExcludeEligibility(edited, &why), ExcludeEligibility::kCrystalNotInScene);

  // One of the entries using pool 0 already has a filter. What it does depends on the filter's
  // action: an In filter (the default-constructed FilterConfig, action 0) denies — "keep only
  // these" and "also drop this" do not compose — while an Out filter is the append case.
  GuiState filtered = state;
  filtered.filters.emplace_back();
  filtered.filters[0].name = "keep";
  filtered.layers[1].entries[0].filter_id = 0;
  EXPECT_EQ(EvaluateExcludeEligibility(filtered, &why), ExcludeEligibility::kEntryHasInFilter);
  EXPECT_NE(why.find("In filter"), std::string::npos) << "the denial names the reason: " << why;
  EXPECT_TRUE(ExcludeAppendNotice(filtered).empty()) << "nothing to append to when denied";
  EXPECT_FALSE(ApplyExcludeSelectedRaypath(filtered)) << "a denial writes nothing";
  EXPECT_EQ(filtered.filters.size(), 1u);
  EXPECT_TRUE(filtered.filters[0].param.empty()) << "and the In filter is not touched";

  filtered.filters[0].action = 1;  // the same filter, now Out
  EXPECT_EQ(EvaluateExcludeEligibility(filtered, &why), ExcludeEligibility::kOk);
  EXPECT_TRUE(why.empty());
  const std::string notice = ExcludeAppendNotice(filtered);
  EXPECT_NE(notice.find("\"keep\""), std::string::npos)
      << "the OK tooltip says which filter the chain joins: " << notice;
  // The crystal's other entry (layer 0) holds no filter: the tooltip says so, and the apply
  // reaches both — the Out filter gains the row, the filter-less entry gets a fresh one.
  EXPECT_NE(notice.find("1 entry without a filter gets a new one"), std::string::npos) << notice;
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(filtered));
  ASSERT_EQ(filtered.filters.size(), 2u);
  ASSERT_EQ(filtered.filters[0].param.size(), 1u);
  EXPECT_EQ(filtered.filters[0].param[0].text, "1-2");
  EXPECT_EQ(filtered.filters[0].name, "keep") << "extended in place";
  EXPECT_EQ(filtered.filters[1].action, 1);
  EXPECT_EQ(filtered.filters[1].RaypathText(), "1-2");
  EXPECT_EQ(*filtered.layers[1].entries[0].filter_id, 0);
  EXPECT_EQ(*filtered.layers[0].entries[1].filter_id, 1);
  EXPECT_FALSE(filtered.layers[0].entries[0].filter_id.has_value()) << "pool 1's entry is not the crystal's";
}

TEST(AnalysisPanelLogic, FormatSegmentRaypathTextJoinsFacesWithDashes) {
  LUMICE_RaypathChainSegment seg{};
  seg.segment_len = 3;
  seg.segment[0] = 3;
  seg.segment[1] = 1;
  seg.segment[2] = 5;
  EXPECT_EQ(FormatSegmentRaypathText(seg), "3-1-5");
  seg.segment_len = 1;
  EXPECT_EQ(FormatSegmentRaypathText(seg), "3");
  seg.segment_len = 0;
  EXPECT_EQ(FormatSegmentRaypathText(seg), "");
}

TEST(AnalysisPanelLogic, ExcludeWritesOneFilterOutFilterBoundToEveryEntryOfTheCrystal) {
  GuiState state = TwoCrystalDocument();
  auto p = MakePayload(1, LUMICE_RAYPATH_ROI_FULL_SKY, { 5.0 });
  p->entries[0].chain[0].crystal_id = 1;  // scene crystal 1 = pool 0, used by two entries
  p->entries[0].chain[0].segment_len = 2;
  p->entries[0].chain[0].segment[0] = 3;
  p->entries[0].chain[0].segment[1] = 5;
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p));
  // The list on show was read under P|B (as RefreshAnalysisEntries records it); the filter
  // follows THAT, not the checkboxes, which may since have moved on to bits the list does not show.
  state.analysis_result.entries_symmetry = LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B;
  state.analysis.symmetry_d = true;
  state.analysis.selected_entry = "1-2";
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(state));

  ASSERT_EQ(state.filters.size(), 1u);
  const FilterConfig& f = state.filters[0];
  EXPECT_EQ(f.action, 1) << "filter_out";
  EXPECT_TRUE(f.sym_p && f.sym_b) << "the reduction the row was counted under";
  EXPECT_FALSE(f.sym_d) << "and not a bit more";
  ASSERT_TRUE(f.IsRaypath());
  EXPECT_EQ(f.RaypathText(), "3-5");
  EXPECT_NE(f.name.find("1-2"), std::string::npos) << "named after the chain's display text";
  // Bound to both entries of pool 0, and to neither entry of pool 1.
  ASSERT_TRUE(state.layers[0].entries[1].filter_id.has_value());
  EXPECT_EQ(*state.layers[0].entries[1].filter_id, 0);
  ASSERT_TRUE(state.layers[1].entries[0].filter_id.has_value());
  EXPECT_EQ(*state.layers[1].entries[0].filter_id, 0);
  EXPECT_FALSE(state.layers[0].entries[0].filter_id.has_value());
  // Now that the crystal has an Out filter, a further exclude is still allowed: it extends that
  // filter rather than stacking a second one. Idempotent first — the same chain again adds no
  // row and touches no pool slot.
  EXPECT_EQ(EvaluateExcludeEligibility(state, nullptr), ExcludeEligibility::kOk);
  const std::string notice = ExcludeAppendNotice(state);
  EXPECT_NE(notice.find("Exclude 1-2"), std::string::npos) << notice;
  EXPECT_NE(notice.find("Shared with 1 other entry"), std::string::npos) << "two entries share the slot: " << notice;
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(state));
  ASSERT_EQ(state.filters.size(), 1u);
  EXPECT_EQ(state.filters[0].param.size(), 1u) << "the same chain is not added twice";
  EXPECT_EQ(state.filters[0].RaypathText(), "3-5");

  // A different chain of the same crystal: one more OR row on the same filter, whose name,
  // action and symmetry stay what they were — even though the list has since been re-read under
  // different bits (P|B|D here), the row joins the filter's own P|B.
  auto p2 = MakePayload(2, LUMICE_RAYPATH_ROI_FULL_SKY, { 5.0 });
  p2->entries[0].chain[0].crystal_id = 1;
  p2->entries[0].chain[0].segment_len = 3;
  p2->entries[0].chain[0].segment[0] = 3;
  p2->entries[0].chain[0].segment[1] = 1;
  p2->entries[0].chain[0].segment[2] = 5;
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(state, p2));
  state.analysis_result.entries_symmetry =
      LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;
  state.analysis.selected_entry = "1-2";
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(state));
  ASSERT_EQ(state.filters.size(), 1u) << "extended, not stacked";
  const FilterConfig& g = state.filters[0];
  ASSERT_EQ(g.param.size(), 2u);
  EXPECT_EQ(g.param[0].text, "3-5");
  EXPECT_EQ(g.param[1].text, "3-1-5");
  EXPECT_EQ(g.action, 1);
  EXPECT_EQ(g.name, f.name) << "the filter keeps its name";
  EXPECT_TRUE(g.sym_p && g.sym_b);
  EXPECT_FALSE(g.sym_d) << "the filter's own symmetry, not the list's current one";
  // Still bound to both entries of pool 0 through the same slot (in-place overwrite), and to
  // neither entry of pool 1.
  EXPECT_EQ(*state.layers[0].entries[1].filter_id, 0);
  EXPECT_EQ(*state.layers[1].entries[0].filter_id, 0);
  EXPECT_FALSE(state.layers[0].entries[0].filter_id.has_value());
}

}  // namespace
}  // namespace lumice::gui
