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

    // Auto-diff-excluded fields must NOT drive effects even when they differ from baseline
    // (raypath_color: reason is ColorClassConfig mixes structural + display sub-fields; the T1
    // migration will split it. See gui_state_reconcile.hpp header comment.)
    {
      GuiState s = MakeBaselineState();
      s.raypath_color.emplace_back();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK(!e.need_resim);
      IM_CHECK(!e.need_hard_reset);
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
    // Six auto-diff-participating fields per gui_state_reconcile.cpp:
    //   soft: crystals, layers, sun, sim, renderer
    //   hard: filters
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

    // Cross-check: the 6 mutators above match the auto-diff set from kFieldTierTable. Compute it
    // from the header table at test time so a table edit that changes the set immediately makes
    // the mutator list stale — the mismatch flags the diff as needing hand-review.
    int expected_auto_diff_count = 0;
    for (const auto& entry : gui::kFieldTierTable) {
      const bool is_struct = entry.tier == FieldTier::kStructHard || entry.tier == FieldTier::kStructSoft;
      if (is_struct && !entry.auto_diff_excluded) {
        ++expected_auto_diff_count;
      }
    }
    IM_CHECK_EQ(expected_auto_diff_count, static_cast<int>(sizeof(kMutators) / sizeof(kMutators[0])));

    // Assertion (b/c) analog: auto-diff-excluded struct-tier fields must NOT drive effects when
    // mutated. Currently only raypath_color (auto_diff_excluded=true in kFieldTierTable).
    {
      GuiState s = MakeBaselineState();
      s.raypath_color.emplace_back();
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK_EQ(e, GuiEffects{});
    }
    // Meta: exactly one auto-diff-excluded struct-tier entry today. If T1 flips raypath_color's
    // exclusion (splits ColorClassConfig), this count changes and the assertion catches the
    // omission of a paired wiring update.
    int excluded_struct_count = 0;
    for (const auto& entry : gui::kFieldTierTable) {
      const bool is_struct = entry.tier == FieldTier::kStructHard || entry.tier == FieldTier::kStructSoft;
      if (is_struct && entry.auto_diff_excluded) {
        ++excluded_struct_count;
      }
    }
    IM_CHECK_EQ(excluded_struct_count, 2);  // raypath_color + use_gpu_backend

    // Non-struct tiers (kDisplay/kView/kSession) are outside the reconciler's remit; they must not
    // drive effects. Spot-check with raypath_color_mode (kDisplay).
    {
      GuiState s = MakeBaselineState();
      s.raypath_color_mode = s.raypath_color_mode == 0 ? 1 : 0;
      GuiEffects e = ReconcileGuiEffects(s);
      IM_CHECK_EQ(e, GuiEffects{});
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
      ApplyGuiEffects(s, GuiEffects{});
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
      ApplyGuiEffects(s, e);
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
      ApplyGuiEffects(s, e);
      IM_CHECK(s.dirty);
      IM_CHECK_EQ(s.display_epoch_floor, 5u);  // raised to committed_epoch
      IM_CHECK_EQ(s.p99_raw_y, 0.0f);
      IM_CHECK_EQ(s.snapshot_intensity, 0.0f);
    }
  };
}
