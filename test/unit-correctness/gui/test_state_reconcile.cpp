// The field-tier reconciler, as a truth table over one input at a time.
//
// ReconcileGuiEffects answers one question per frame: given the document as it is now and the
// document as it was committed, which of three things must happen — re-run the simulation, tear the
// display down first, re-push the display state. Every widget in the GUI writes a field and leaves
// that decision to this one function, so a field wired into the wrong lane is not a local bug: it
// is either a simulation that restarts when the user drags the camera, or a change that never takes
// effect at all.
//
// No GL, no server. The reconciler is a pure function of two structs.

#include <gtest/gtest.h>

#include <cstdint>
#include <set>
#include <string>

#include "gui/gui_state.hpp"
#include "gui/gui_state_reconcile.hpp"
#include "gui/gui_state_tiers.hpp"

namespace gui = lumice::gui;

namespace {

using gui::ApplyGuiEffects;
using gui::FieldTier;
using gui::GuiEffects;
using gui::GuiState;
using gui::ReconcileGuiEffects;

// A state whose two baselines both match it, so a bare reconcile is completely quiet. Each row
// below mutates exactly one thing on top of it; the quiet baseline is what makes "this row's
// mutation caused this effect" a true statement rather than a coincidence.
//
// The collections are non-empty on purpose: a vector-vs-vector comparison of two empty vectors is
// equal for reasons that have nothing to do with the fields inside them.
GuiState MakeBaselineState() {
  GuiState s;
  s.crystals.emplace_back();
  s.filters.emplace_back();
  s.layers.emplace_back();
  s.raypath_color.emplace_back();
  s.last_committed_state = GuiState::ConfigSnapshot::From(s);
  GuiState::DisplayStateBaseline display;
  for (const auto& cls : s.raypath_color) {
    display.color_display.push_back(static_cast<const gui::ColorClassDisplayState&>(cls));
  }
  display.raypath_color_mode = s.raypath_color_mode;
  s.last_pushed_display_state = std::move(display);
  return s;
}

struct Row {
  const char* what_changed;
  void (*mutate)(GuiState&);
  bool resim;
  bool hard_reset;
  bool display_push;
};

// Structural fields: the simulation itself would produce different rays, so it has to run again.
// `filters` and the structural half of `raypath_color` additionally need the display torn down
// first, because what is already on screen was accumulated under the old filter and cannot be
// carried forward.
const Row kStructuralRows[] = {
  { "crystals", [](GuiState& s) { s.crystals.emplace_back(); }, true, false, false },
  { "layers", [](GuiState& s) { s.layers.emplace_back(); }, true, false, false },
  { "sun", [](GuiState& s) { s.sun.altitude += 1.0f; }, true, false, false },
  { "sim", [](GuiState& s) { s.sim.ray_num_millions += 1.0f; }, true, false, false },
  // The one RenderConfig field that IS structural: it changes the grid the simulation renders
  // into. Stated here so the view exclusions below cannot be over-broadened to swallow the
  // whole struct.
  { "renderer.sim_resolution_index", [](GuiState& s) { s.renderer.sim_resolution_index += 1; }, true, false, false },
  { "filters", [](GuiState& s) { s.filters.emplace_back(); }, true, true, false },
  { "raypath_color cardinality", [](GuiState& s) { s.raypath_color.emplace_back(); }, true, true, false },
  { "raypath_color combine rule", [](GuiState& s) { s.raypath_color[0].combine = 1; }, true, true, false },
};

// Display-only fields: the same rays, shown differently. Re-running would throw away a finished
// simulation to arrive at the same numbers.
const Row kDisplayRows[] = {
  { "a class colour", [](GuiState& s) { s.raypath_color[0].color[0] = 0.5f; }, false, false, true },
  { "the compositing mode", [](GuiState& s) { s.raypath_color_mode = s.raypath_color_mode == 0 ? 1 : 0; }, false, false,
    true },
};

// Fields that must drive nothing at all.
//
// The seven view fields are the reason this group exists. The simulation renders a fixed full-sky
// image and the preview shader reprojects it, so lens, field of view, camera angles and the
// hemisphere clip are pure client-side maths. Before that was pinned, dragging the view restarted
// the simulation — silently in infinite-rays mode, and as a "configuration changed" banner in
// finite mode, over a change the user could not have meant as a configuration edit.
//
// Exposure is here for the same reason and is worth stating separately: it is the field a user is
// most likely to be dragging when a long run is nearly finished.
const Row kInertRows[] = {
  { "renderer.lens_type", [](GuiState& s) { s.renderer.lens_type = 1; }, false, false, false },
  { "renderer.fov", [](GuiState& s) { s.renderer.fov = 45.0f; }, false, false, false },
  { "renderer.elevation", [](GuiState& s) { s.renderer.elevation = 15.0f; }, false, false, false },
  { "renderer.azimuth", [](GuiState& s) { s.renderer.azimuth = 30.0f; }, false, false, false },
  { "renderer.roll", [](GuiState& s) { s.renderer.roll = 10.0f; }, false, false, false },
  { "renderer.visible", [](GuiState& s) { s.renderer.visible = 0; }, false, false, false },
  { "renderer.front", [](GuiState& s) { s.renderer.front = !s.renderer.front; }, false, false, false },
  { "renderer.exposure_offset", [](GuiState& s) { s.renderer.exposure_offset = 2.0f; }, false, false, false },
  // Not part of the committed snapshot at all, so it cannot participate in the diff even in
  // principle — the legacy wrapper owns it.
  { "use_gpu_backend", [](GuiState& s) { s.use_gpu_backend = !s.use_gpu_backend; }, false, false, false },
};

void CheckRows(const Row* rows, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const Row& row = rows[i];
    SCOPED_TRACE(row.what_changed);
    GuiState s = MakeBaselineState();
    ASSERT_EQ(ReconcileGuiEffects(s), GuiEffects{}) << "the baseline is not quiet, so this row proves nothing";
    row.mutate(s);
    const GuiEffects e = ReconcileGuiEffects(s);
    EXPECT_EQ(e.need_resim, row.resim);
    EXPECT_EQ(e.need_hard_reset, row.hard_reset);
    EXPECT_EQ(e.need_display_push, row.display_push);
  }
}

}  // namespace

TEST(GuiStateReconcile, StructuralChangesReRunTheSimulation) {
  CheckRows(kStructuralRows, std::size(kStructuralRows));
}

TEST(GuiStateReconcile, DisplayChangesArePushedWithoutReRunning) {
  CheckRows(kDisplayRows, std::size(kDisplayRows));
}

TEST(GuiStateReconcile, ViewAndExposureChangesDriveNothing) {
  CheckRows(kInertRows, std::size(kInertRows));
}

// Before the first commit there is no baseline to compare against, and the reconciler must be
// silent rather than treat "everything" as changed — otherwise the app would restart a simulation
// on its first frame.
TEST(GuiStateReconcile, WithNoCommittedBaselineNothingIsReported) {
  GuiState s;
  ASSERT_FALSE(s.last_committed_state.has_value());
  EXPECT_EQ(ReconcileGuiEffects(s), GuiEffects{});
}

// Attaching or detaching a filter is a structural change even when nothing else about the entry
// moved; re-binding one filter to another is not. The three rows are pinned together because the
// obvious over-correction — "any change to filter_id is hard" — passes the first two and fails the
// third, and the obvious under-correction passes the third and fails the first two.
TEST(GuiStateReconcile, AttachingOrDetachingAFilterIsHardButRebindingIsNot) {
  struct Case {
    const char* name;
    void (*setup)(GuiState&);   // runs before the baseline is taken
    void (*mutate)(GuiState&);  // runs after
    bool expect_hard;
  };
  const Case kCases[] = {
    { "none -> some", [](GuiState& s) { s.layers[0].entries.emplace_back(); },
      [](GuiState& s) { s.layers[0].entries[0].filter_id = 0; }, true },
    { "some -> none",
      [](GuiState& s) {
        s.layers[0].entries.emplace_back();
        s.layers[0].entries[0].filter_id = 0;
      },
      [](GuiState& s) { s.layers[0].entries[0].filter_id.reset(); }, true },
    { "some(A) -> some(B)",
      [](GuiState& s) {
        s.filters.emplace_back();
        s.layers[0].entries.emplace_back();
        s.layers[0].entries[0].filter_id = 0;
      },
      [](GuiState& s) { s.layers[0].entries[0].filter_id = 1; }, false },
    // A new entry with no filter changes the shape of the comparison itself. It is an ordinary
    // structural edit, and must not be mistaken for a presence toggle by a detector that reads
    // past the overlap.
    { "an added entry with no filter", [](GuiState& s) { s.layers[0].entries.emplace_back(); },
      [](GuiState& s) { s.layers[0].entries.emplace_back(); }, false },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    GuiState s = MakeBaselineState();
    c.setup(s);
    s.last_committed_state = GuiState::ConfigSnapshot::From(s);
    c.mutate(s);
    const GuiEffects e = ReconcileGuiEffects(s);
    EXPECT_TRUE(e.need_resim) << "every one of these is at least a structural change";
    EXPECT_EQ(e.need_hard_reset, c.expect_hard);
  }
}

// Clearing the display baseline is how a run, a revert or a backend swap forces the whole display
// state to be pushed again. It has to fire, it has to have something to push, and it must not
// disturb the commit baseline beside it — that one belongs to Run and Revert, and clearing it here
// would break Revert with no other symptom.
TEST(GuiStateReconcile, ClearingTheDisplayBaselineRePushesWithoutTouchingTheCommitBaseline) {
  GuiState s = MakeBaselineState();
  ASSERT_FALSE(s.raypath_color.empty());
  ASSERT_TRUE(s.last_pushed_display_state.has_value());

  s.InvalidateEffectsBaselines();
  EXPECT_FALSE(s.last_pushed_display_state.has_value());
  EXPECT_TRUE(s.last_committed_state.has_value()) << "the commit baseline was cleared along with the display one";

  GuiEffects e = ReconcileGuiEffects(s);
  EXPECT_TRUE(e.need_display_push);
  EXPECT_FALSE(e.need_resim);
  EXPECT_FALSE(e.need_hard_reset);

  // ...and with nothing to push, it stays quiet. Otherwise every reset would fire a push of an
  // empty list.
  GuiState empty = MakeBaselineState();
  empty.raypath_color.clear();
  empty.InvalidateEffectsBaselines();
  EXPECT_FALSE(ReconcileGuiEffects(empty).need_display_push);
}

// A display push is only meaningful when the live list and the pushed one have the same shape: the
// receiving API rejects a mismatched one, so firing here would be a push that silently does nothing.
TEST(GuiStateReconcile, ADisplayPushIsSuppressedWhileTheListLengthDisagrees) {
  GuiState s = MakeBaselineState();
  s.last_pushed_display_state = GuiState::DisplayStateBaseline{};  // pushed nothing; live holds one class
  EXPECT_FALSE(ReconcileGuiEffects(s).need_display_push);
}

// The reconciler's field set and the tier registry must be the same set, by NAME.
//
// Two ways to drift, and only one of them is caught here. Registering a field without wiring it in
// makes its mutation produce no effect, which is what the mutator list below detects. Wiring one in
// without registering it is caught at build time by the field-tier registration policy gate — there
// is no reflection available here to enumerate struct fields, and a check that pretended otherwise
// would be a check on a hand-written list, not on the code.
//
// The comparison is on names rather than counts on purpose: an equal-size swap — excluding one
// field while un-excluding another — passes a count and leaves the reconciler testing the wrong
// fields entirely.
TEST(GuiStateReconcile, EveryRegisteredStructuralFieldIsActuallyDiffed) {
  struct Mutator {
    const char* field;
    void (*apply)(GuiState&);
  };
  const Mutator kMutators[] = {
    { "crystals", [](GuiState& s) { s.crystals.emplace_back(); } },
    { "layers", [](GuiState& s) { s.layers.emplace_back(); } },
    { "sun", [](GuiState& s) { s.sun.altitude += 1.0f; } },
    { "sim", [](GuiState& s) { s.sim.ray_num_millions += 1.0f; } },
    { "renderer", [](GuiState& s) { s.renderer.sim_resolution_index += 1; } },
    { "filters", [](GuiState& s) { s.filters.emplace_back(); } },
    // The structural half, so this row asserts re-sim like the others; the display half has its
    // own row in the display table above.
    { "raypath_color", [](GuiState& s) { s.raypath_color[0].combine = 1; } },
  };

  std::set<std::string> mutated;
  for (const Mutator& mut : kMutators) {
    SCOPED_TRACE(mut.field);
    mutated.insert(mut.field);
    GuiState s = MakeBaselineState();
    mut.apply(s);
    EXPECT_TRUE(ReconcileGuiEffects(s).need_resim) << "registered as structural but the reconciler does not diff it";
  }

  std::set<std::string> registered;
  int excluded_struct_fields = 0;
  for (const auto& entry : gui::kFieldTierTable) {
    if (entry.tier != FieldTier::kStructHard && entry.tier != FieldTier::kStructSoft) {
      continue;
    }
    if (entry.auto_diff_excluded) {
      ++excluded_struct_fields;
      continue;
    }
    registered.insert(entry.name);
  }
  EXPECT_TRUE(registered == mutated) << "the reconciler's field set and the tier registry have drifted apart";
  // Exactly one structural field is held out of the automatic diff (use_gpu_backend, which is not
  // in the committed snapshot). A second one appearing means a field was excluded without a
  // hand-written replacement being wired up alongside.
  EXPECT_EQ(excluded_struct_fields, 1);
}

// Applying the effects: a hard reset is a superset of a re-run, so it takes over rather than
// running alongside. The distinction is visible in what it clears — the epoch floor, which is what
// stops the previous run's pixels being drawn, and the accumulated intensity the exposure is
// derived from.
TEST(GuiStateReconcile, ApplyingEffectsLetsAHardResetTakeOverFromAPlainReRun) {
  struct Case {
    const char* name;
    GuiEffects effects;
    bool expect_dirty;
    uint64_t expect_floor;
    float expect_p99;
  };
  const Case kCases[] = {
    { "nothing to do", GuiEffects{}, false, 0u, 1.5f },
    { "re-run only",
      [] {
        GuiEffects e;
        e.need_resim = true;
        return e;
      }(),
      true, 0u, 1.5f },
    { "hard reset",
      [] {
        GuiEffects e;
        e.need_resim = true;
        e.need_hard_reset = true;
        return e;
      }(),
      true, 5u, 0.0f },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    GuiState s;
    s.committed_epoch = 5;
    s.display_epoch_floor = 0;
    s.p99_raw_y = 1.5f;
    s.snapshot_intensity = 2.5f;

    ApplyGuiEffects(s, nullptr, c.effects);

    EXPECT_EQ(s.dirty, c.expect_dirty);
    EXPECT_EQ(s.display_epoch_floor, c.expect_floor);
    EXPECT_FLOAT_EQ(s.p99_raw_y, c.expect_p99);
    EXPECT_FLOAT_EQ(s.snapshot_intensity, c.expect_p99 == 0.0f ? 0.0f : 2.5f);
  }
}
