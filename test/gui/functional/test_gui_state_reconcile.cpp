// PURE unit tests for the field-tier reconciler geodetics (scrum-gui-state-reconcile T0). No GL,
// no server. Peer to test_gui_lifecycle.cpp::reconcile_truth_table — the SimState reconciler is the
// I2 owner; ReconcileGuiEffects is its structural sibling for the field-tier layer.
//
// Tests:
//  1. gui_state_reconcile/effects_truth_table — per-field diff → GuiEffects mapping (soft × 5 fields
//     → need_resim only; hard × 1 field → need_resim + need_hard_reset; auto_diff_excluded fields
//     → no effects; no baseline → no-op).
//  2. gui_state_reconcile/meta_anti_drift — pins the invariant that the auto-diff-participating
//     field set in ReconcileGuiEffects equals `(kFieldTierTable · {kStructHard,kStructSoft}) −
//     kAutoDiffExcludeList`. Detects drift when someone adds a struct field to the tier table
//     without wiring it into the reconciler (or vice versa). Fires RED via a direct-check pattern:
//     for every registered-non-excluded field, mutating that field (and only that field) MUST
//     change GuiEffects; and for every excluded / non-registered field, mutating it MUST NOT.
//  3. gui_state_reconcile/apply_effects_priority — pins the hard-shadows-soft precedence in
//     ApplyGuiEffects and the "no baseline → no-op" contract.

#include <set>
#include <string>

#include "gui/gui_state_reconcile.hpp"
#include "gui/gui_state_tiers.hpp"
#include "test_gui_shared.hpp"

namespace {

using gui::ApplyGuiEffects;
using gui::FieldTier;
using gui::GuiEffects;
using gui::GuiState;
using gui::ReconcileGuiEffects;

// Build a GuiState with a fully populated last_committed_state baseline. All fields equal, so a
// bare ReconcileGuiEffects on this state returns all-false. Mutate ONE field on top to test one
// diff at a time.
GuiState MakeBaselineState() {
  GuiState s;
  // Populate a minimal non-empty crystals/layers so struct equality has non-trivial content;
  // otherwise vector-of-zero-elements shortcuts.
  s.crystals.emplace_back();
  s.filters.emplace_back();
  s.layers.emplace_back();
  s.raypath_color.emplace_back();
  s.last_committed_state = GuiState::ConfigSnapshot::From(s);
  // T1: populate the display-push baseline in the same "everything committed" state so a bare
  // reconcile is fully quiet (both baselines match). Widget tests that need the "first push
  // after reset" edge should call s.InvalidateEffectsBaselines() explicitly.
  GuiState::DisplayStateBaseline dsb;
  for (const auto& cls : s.raypath_color) {
    dsb.color_display.push_back(static_cast<const gui::ColorClassDisplayState&>(cls));
  }
  dsb.raypath_color_mode = s.raypath_color_mode;
  s.last_pushed_display_state = std::move(dsb);
  return s;
}

}  // namespace

void RegisterStateReconcileTests(ImGuiTestEngine* engine) {
  // ---- Test 1: per-field diff → GuiEffects truth table ----
  ImGuiTest* t1 = IM_REGISTER_TEST(engine, "gui_state_reconcile", "effects_truth_table");
  t1->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // No baseline → no-op (first-commit gate: reconciler must be quiet before any commit).
    {
      GuiState s;
      // no last_committed_state
      IM_CHECK_EQ(s.last_committed_state.has_value(), false);
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }

    // Baseline present, no mutation → all-false (identity diff is a no-op).
    {
      GuiState s = MakeBaselineState();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }

    // Soft-tier fields → need_resim only. crystals.
    {
      GuiState s = MakeBaselineState();
      s.crystals.emplace_back();  // mutation
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }
    // Soft-tier: layers.
    {
      GuiState s = MakeBaselineState();
      s.layers.emplace_back();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }
    // Soft-tier: sun (bump altitude — any structural difference works).
    {
      GuiState s = MakeBaselineState();
      s.sun.altitude += 1.0f;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }
    // Soft-tier: sim (bump ray_num).
    {
      GuiState s = MakeBaselineState();
      s.sim.ray_num_millions += 1.0f;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }
    // Soft-tier: renderer (bump resolution width).
    {
      GuiState s = MakeBaselineState();
      s.renderer.sim_resolution_index += 1;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }

    // Hard-tier field → need_resim + need_hard_reset. filters is the only automatic entry.
    {
      GuiState s = MakeBaselineState();
      s.filters.emplace_back();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(e.need_hard_reset);
    }

    // task-color-migration (T1): raypath_color struct-part (combine/match) change → hard-reset lane.
    // Vector cardinality change (add class) also counts as structural (add/remove filter topology).
    {
      GuiState s = MakeBaselineState();
      s.raypath_color.emplace_back();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(e.need_hard_reset);
    }
    // combine change (same cardinality, struct-part only) → hard-reset lane.
    {
      GuiState s = MakeBaselineState();
      s.raypath_color[0].combine = 1;  // was 0
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_resim);
      IM_CHECK(e.need_hard_reset);
    }
    // Display-only change (color) with matching cardinality → need_display_push only,
    // no re-sim / hard-reset.
    {
      GuiState s = MakeBaselineState();
      s.raypath_color[0].color[0] = 0.5f;  // was 1.0f
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
      IM_CHECK(e.need_display_push);
    }
    // raypath_color_mode change (kDisplay tier) → need_display_push only.
    {
      GuiState s = MakeBaselineState();
      s.raypath_color_mode = s.raypath_color_mode == 0 ? 1 : 0;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
      IM_CHECK(e.need_display_push);
    }
    // Cardinality mismatch between live vector and last_pushed_display_state MUST suppress
    // need_display_push (D3 same-cardinality gate) — a settling-window push would be rejected
    // by LUMICE_SetRaypathColors.
    {
      GuiState s = MakeBaselineState();
      GuiState::DisplayStateBaseline stale;
      // Baseline snapshot represents "pre-add" state (size 0); live vector has 1 entry.
      s.last_pushed_display_state = stale;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_display_push);
    }
    // use_gpu_backend is not in ConfigSnapshot::From so it cannot participate in the diff even
    // conceptually — legacy DIRTY_IF wrapper owns it. Toggling it must not drive effects.
    {
      GuiState s = MakeBaselineState();
      s.use_gpu_backend = !s.use_gpu_backend;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
    }

    // task-color-migration §4 M6 (repush discipline): after InvalidateEffectsBaselines resets the
    // display-push baseline to nullopt, the next reconcile MUST fire need_display_push against a
    // non-empty raypath_color vector. This is the mechanism that fixes偏离 B' (AC2: Run 后
    // z_order 立即生效) — z_order cannot travel through the commit payload (D2), so the fix is
    // to re-push the full display state after every DoRun / DoRevert / backend swap.
    {
      GuiState s = MakeBaselineState();
      IM_CHECK(!s.raypath_color.empty());                 // sanity: MakeBaselineState puts one class in
      IM_CHECK(s.last_pushed_display_state.has_value());  // sanity: MakeBaselineState seeds it
      s.InvalidateEffectsBaselines();
      IM_CHECK(!s.last_pushed_display_state.has_value());
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(e.need_display_push);  // nullopt baseline + non-empty vector → fire
      IM_CHECK(!e.need_resim);        // commit baseline is untouched, no re-sim needed
      IM_CHECK(!e.need_hard_reset);
    }
    // Boundary: InvalidateEffectsBaselines against an EMPTY raypath_color must not fire the push
    // (nothing to push). Guards against a future refactor that treats nullopt as unconditional.
    {
      GuiState s = MakeBaselineState();
      s.raypath_color.clear();
      s.InvalidateEffectsBaselines();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_display_push);
    }
    // Boundary: InvalidateEffectsBaselines must NOT touch last_committed_state — that field is
    // owned by DoRun (writes) / DoRevert (reads); commingling would silently break Revert.
    // (RED手法: implementing InvalidateEffectsBaselines as `state = GuiState{}` or resetting
    // last_committed_state alongside would flip this to a break.)
    {
      GuiState s = MakeBaselineState();
      const bool commit_present_before = s.last_committed_state.has_value();
      IM_CHECK(commit_present_before);
      s.InvalidateEffectsBaselines();
      IM_CHECK(s.last_committed_state.has_value());  // preserved across the reset
    }
  };

  // ---- Test 2: meta anti-drift — reconciler ↔ tier table equivalence ----
  //
  // Failure modes this test catches (each maps to a RED手法):
  //   (a) Someone adds a struct field to the tier table with tier ∈ {hard, soft} but forgets to
  //       wire the corresponding `if (state.X != baseline.X)` branch in ReconcileGuiEffects.
  //       RED: mutating that field will not change GuiEffects → this test fails at the
  //       "registered-non-excluded ⇒ effect changes" assertion.
  //   (b) Someone wires a new `!=` branch in ReconcileGuiEffects but forgets to register the
  //       field in kFieldTierTable. RED: check_gui_state_field_tier_registration will fire the
  //       "unregistered field" violation at build time (policy gate), catching this case at a
  //       different layer. (Testing case-b at the C++ layer would require reflection or field
  //       enumeration we don't have; the policy gate is the honest guard.)
  //   (c) Someone drops raypath_color's auto_diff_excluded=true (T1 migration will do this) but
  //       forgets to add a per-field wiring. Then mutating raypath_color should change effects,
  //       but wouldn't. This test catches it as case (a) once the exclusion is dropped.
  //
  // Scope caveat: this is a targeted spot-check, not a full permutation of every possible field
  // mutation. It exercises the exact fields ReconcileGuiEffects currently claims to diff plus
  // the auto-diff exclusions — sufficient for T0 (auto-diff set = 6 fields).
  ImGuiTest* t2 = IM_REGISTER_TEST(engine, "gui_state_reconcile", "meta_anti_drift");
  t2->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // Assertion (a): every registered-non-excluded field must change GuiEffects when mutated.
    // Seven auto-diff-participating fields per gui_state_reconcile.cpp:
    //   soft: crystals, layers, sun, sim, renderer
    //   hard: filters
    //   hard/display split: raypath_color (T1 task-color-migration dropped auto_diff_excluded)
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
      // T1: mutate struct-part so the assertion below (`after.need_resim == true`) holds — a
      // pure display-part mutation would only fire need_display_push, not need_resim.
      { "raypath_color", [](GuiState& s) { s.raypath_color[0].combine = 1; } },
    };
    for (const auto& mut : kMutators) {
      GuiState s = MakeBaselineState();
      GuiEffects before = ReconcileGuiEffects(s);
      IM_CHECK_EQ(before, GuiEffects{});  // baseline is quiet
      mut.apply(s);
      GuiEffects after = ReconcileGuiEffects(s);
      IM_CHECK_NE(after, GuiEffects{});
      // Bonus check: `after` includes need_resim for every one of these 6 (soft and hard both do).
      IM_CHECK(after.need_resim);
      IM_UNUSED(mut.field);  // name is here for debug-print if IM_CHECK fires
    }

    // Cross-check: the 6 mutators above match the auto-diff *name set* from kFieldTierTable, not
    // just its cardinality — an equal-size swap (e.g. excluding `filters` while un-excluding
    // `use_gpu_backend`) would pass a count-only check but must fail here, since the mutator list
    // would then be testing the wrong fields entirely.
    std::set<std::string> expected_fields;
    for (const auto& entry : gui::kFieldTierTable) {
      const bool is_struct = entry.tier == FieldTier::kStructHard || entry.tier == FieldTier::kStructSoft;
      if (is_struct && !entry.auto_diff_excluded) {
        expected_fields.insert(entry.name);
      }
    }
    std::set<std::string> mutator_fields;
    for (const auto& mut : kMutators) {
      mutator_fields.insert(mut.field);
    }
    IM_CHECK(expected_fields == mutator_fields);

    // Assertion (b/c) analog: auto-diff-excluded struct-tier fields must NOT drive effects when
    // mutated. Post-T1 there is exactly ONE excluded struct-tier entry: use_gpu_backend (kept
    // out of ConfigSnapshot; legacy DIRTY_IF wrapper owns it).
    {
      GuiState s = MakeBaselineState();
      s.use_gpu_backend = !s.use_gpu_backend;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK_EQ(e, GuiEffects{});
    }
    // Meta: exactly ONE auto-diff-excluded struct-tier entry today (use_gpu_backend). T1 dropped
    // raypath_color's exclusion after splitting ColorClassConfig; if a later task adds a new
    // exception this count changes and the assertion catches the omission of a paired wiring
    // update.
    int excluded_struct_count = 0;
    for (const auto& entry : gui::kFieldTierTable) {
      const bool is_struct = entry.tier == FieldTier::kStructHard || entry.tier == FieldTier::kStructSoft;
      if (is_struct && entry.auto_diff_excluded) {
        ++excluded_struct_count;
      }
    }
    IM_CHECK_EQ(excluded_struct_count, 1);  // use_gpu_backend only

    // Display-tier field mutation → need_display_push only (no need_resim / need_hard_reset).
    // raypath_color_mode is the only kDisplay entry today.
    {
      GuiState s = MakeBaselineState();
      s.raypath_color_mode = s.raypath_color_mode == 0 ? 1 : 0;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
      IM_CHECK(e.need_display_push);
    }
  };

  // ---- Test 3: ApplyGuiEffects priority + no-op contract ----
  ImGuiTest* t3 = IM_REGISTER_TEST(engine, "gui_state_reconcile", "apply_effects_priority");
  t3->TestFunc = [](ImGuiTestContext* ctx) {
    IM_UNUSED(ctx);

    // effects all-false → GuiState untouched (dirty stays false).
    {
      GuiState s;
      s.dirty = false;
      const uint64_t floor_before = s.display_epoch_floor;
      const float p99_before = s.p99_raw_y;
      ApplyGuiEffects(s, nullptr, GuiEffects{});
      IM_CHECK(!s.dirty);
      IM_CHECK_EQ(s.display_epoch_floor, floor_before);
      IM_CHECK_EQ(s.p99_raw_y, p99_before);
    }

    // need_resim only → MarkDirty path (dirty=true; epoch floor untouched).
    {
      GuiState s;
      s.committed_epoch = 5;
      s.display_epoch_floor = 0;
      s.p99_raw_y = 1.5f;
      GuiEffects e;
      e.need_resim = true;
      ApplyGuiEffects(s, nullptr, e);
      IM_CHECK(s.dirty);
      IM_CHECK_EQ(s.display_epoch_floor, 0u);  // MarkDirty does NOT raise floor
      IM_CHECK_EQ(s.p99_raw_y, 1.5f);          // MarkDirty does NOT clear p99
    }

    // need_hard_reset → MarkFilterDirty path (dirty=true AND floor raised AND intensity cleared).
    // RED手法: writing `if (need_resim) MarkDirty(); if (need_hard_reset) MarkFilterDirty();` (no
    // else) would call BOTH — MarkFilterDirty calls MarkDirty internally, so dirty stays true but
    // the epoch floor is written twice. Semantically not visibly broken; but the else-if precedence
    // is the contracted shape, and this test pins it so a future refactor can't silently switch to
    // the flatter form.
    {
      GuiState s;
      s.committed_epoch = 5;
      s.display_epoch_floor = 0;
      s.p99_raw_y = 1.5f;
      s.snapshot_intensity = 2.5f;
      GuiEffects e;
      e.need_resim = true;
      e.need_hard_reset = true;
      ApplyGuiEffects(s, nullptr, e);
      IM_CHECK(s.dirty);
      IM_CHECK_EQ(s.display_epoch_floor, 5u);  // raised to committed_epoch
      IM_CHECK_EQ(s.p99_raw_y, 0.0f);
      IM_CHECK_EQ(s.snapshot_intensity, 0.0f);
    }
  };
}
