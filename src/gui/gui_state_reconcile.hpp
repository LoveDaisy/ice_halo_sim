#ifndef LUMICE_GUI_STATE_RECONCILE_HPP
#define LUMICE_GUI_STATE_RECONCILE_HPP

#include "include/lumice.h"  // LUMICE_Server (opaque handle in ApplyGuiEffects signature)

// GUI-state reconciler (scrum-gui-state-reconcile T0 geodetics). Blueprint doc:
// doc/gui-state-governance.md. Field-tier registry: gui_state_tiers.hpp.
//
// This is the second single-owner reconciler in the GUI (companion to ReconcileSimState in app.cpp).
// It closes the "widget → dirty" seam that today is scattered across ~30 DIRTY_IF / MarkDirty /
// MarkFilterDirty call sites in panels.cpp / edit_modals.cpp / app_panels.cpp.
//
// Contract:
//   ReconcileGuiEffects(state) — PURE. Diffs two independent baselines against state:
//     - `last_committed_state` (Revert baseline) drives need_resim / need_hard_reset;
//     - `last_pushed_display_state` (display-state baseline, T1 addition) drives need_display_push.
//     If either baseline is nullopt, THAT particular diff is skipped (first-commit gate for
//     re-sim; first-repush-after-DoRun/Revert gate for display push — both semantic no-ops).
//     Reads state; does not write.
//   ApplyGuiEffects(state, server, effects) — writes the effect via existing single-writer
//     methods:
//       need_resim / need_hard_reset → MarkDirty / MarkFilterDirty (idempotent; hard shadows soft).
//       need_display_push            → PushDisplayState(state, server), and on success updates
//                                       `last_pushed_display_state` so the next reconcile diff
//                                       is quiet (edge-triggered).
//     Server is required when need_display_push may fire; unit-test fixtures that call
//     ApplyGuiEffects(state, nullptr, effects) MUST ensure effects.need_display_push == false
//     (dereferencing a nullptr server is a caller-contract violation, not a defensive-check
//     boundary — see plan §4 Step 2 Minor 3).
//
// Coexistence during migration: the existing DIRTY_IF-wrapped widget call sites remain in place
// during T0-T4; both mechanisms write dirty=true into the same field. Because MarkDirty is
// idempotent within a frame, dual-writing is harmless. T2-T4 will remove the DIRTY_IF wrappers as
// each subsystem migrates. (T1 fully migrates the color-window edit sites — see
// doc/gui-state-governance.md.)
//
// Fields participating in the auto-diff (kFieldTierTable entries with auto_diff_excluded=false and
// tier in {kStructHard, kStructSoft, kDisplay}):
//   soft  → need_resim                   : crystals, layers, sun, sim, renderer
//   hard  → need_resim + need_hard_reset : filters
//   hard/display split → need_hard_reset OR need_display_push (see routing below):
//                                          raypath_color (struct-part vs display-part diff on
//                                          ColorClassConfig sub-structs); raypath_color_mode
//                                          → need_display_push only.
//
// Explicitly EXCLUDED from auto-diff (auto_diff_excluded=true or field not in ConfigSnapshot):
//   use_gpu_backend     : registered kStructSoft in the tier table (for governance-union coverage),
//                         but ConfigSnapshot::From/ApplyTo does NOT include it (view/session field,
//                         intentionally excluded from Revert baseline — see gui_state.hpp
//                         field-sync scope comment). No baseline → no diff possible at this task
//                         scope. Legacy DIRTY_IF wrapper at panels.cpp:1067 keeps working.

namespace lumice::gui {

struct GuiState;  // forward decl (gui_state.hpp)

struct GuiEffects {
  bool need_resim = false;
  bool need_hard_reset = false;
  // task-color-migration (T1) — new display-state edge-trigger effect. Fired when the display
  // sub-part of raypath_color or raypath_color_mode differs from last_pushed_display_state and
  // the vector cardinalities match (same-cardinality gate; see gui_state_reconcile.cpp).
  bool need_display_push = false;

  friend bool operator==(const GuiEffects& a, const GuiEffects& b) {
    return a.need_resim == b.need_resim && a.need_hard_reset == b.need_hard_reset &&
           a.need_display_push == b.need_display_push;
  }
  friend bool operator!=(const GuiEffects& a, const GuiEffects& b) { return !(a == b); }
  // TODO(T3): add need_reset when ResetFrontendState command owner lands
  // (doc/gui-state-governance.md T3).
};

// Pure function. Reads `state` (its auto-diff-participating fields, last_committed_state, and
// last_pushed_display_state); does not write. Baselines that are nullopt make THAT particular
// diff a no-op (both first-commit and first-repush-after-Reset gates are natural nullopt).
GuiEffects ReconcileGuiEffects(const GuiState& state);

// Applies the effect via existing single-writer methods on GuiState (MarkDirty / MarkFilterDirty)
// and, for need_display_push, via PushDisplayState(state, server) with baseline update on
// success. Uses explicit else-if precedence between resim and hard-reset (MarkFilterDirty
// internally calls MarkDirty; else-if avoids double-invocation). need_display_push is orthogonal
// to the resim/hard-reset lane and always evaluated on top.
void ApplyGuiEffects(GuiState& state, LUMICE_Server* server, const GuiEffects& effects);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_RECONCILE_HPP
