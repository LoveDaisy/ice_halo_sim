#ifndef LUMICE_GUI_STATE_RECONCILE_HPP
#define LUMICE_GUI_STATE_RECONCILE_HPP

// GUI-state reconciler (scrum-gui-state-reconcile T0 geodetics). Blueprint doc:
// doc/gui-state-governance.md. Field-tier registry: gui_state_tiers.hpp.
//
// This is the second single-owner reconciler in the GUI (companion to ReconcileSimState in app.cpp).
// It closes the "widget → dirty" seam that today is scattered across ~30 DIRTY_IF / MarkDirty /
// MarkFilterDirty call sites in panels.cpp / edit_modals.cpp / app_panels.cpp.
//
// Contract:
//   ReconcileGuiEffects(state) — PURE. Diffs the auto-diff-participating fields of `state` against
//     `state.last_committed_state`; if the baseline is nullopt (before first commit) or a
//     participating field differs, sets the corresponding effect bit. Reads state; does not write.
//   ApplyGuiEffects(state, effects) — writes the effect via the existing MarkDirty /
//     MarkFilterDirty single-writer methods on GuiState. The two methods are idempotent
//     (dirty=true absorbs re-application; MarkFilterDirty raises the epoch floor to the current
//     committed_epoch, idempotent within a frame).
//
// Coexistence during migration: the existing DIRTY_IF-wrapped widget call sites remain in place
// during T0-T4; both mechanisms write dirty=true into the same field. Because MarkDirty is
// idempotent within a frame, dual-writing is harmless. T1-T4 will remove the DIRTY_IF wrappers as
// each subsystem migrates.
//
// Fields participating in the auto-diff (kFieldTierTable entries with auto_diff_excluded=false and
// tier in {kStructHard, kStructSoft}):
//   soft  → need_resim         : crystals, layers, sun, sim, renderer
//   hard  → need_resim + need_hard_reset : filters
//
// Explicitly EXCLUDED from auto-diff (auto_diff_excluded=true or field not in ConfigSnapshot):
//   raypath_color       : nominally kStructHard but its ColorClassConfig mixes structural
//                         (combine/match) and display-only (color/visible/solo/z_order) sub-fields;
//                         auto-diffing would spuriously fire re-sim on pure display edits until
//                         T1 splits ColorClassConfig (doc/gui-state-governance.md T1). Legacy
//                         MarkFilterDirty at edit_modals.cpp / color_window.cpp keeps working.
//   use_gpu_backend     : registered kStructSoft in the tier table (for governance-union coverage),
//                         but ConfigSnapshot::From/ApplyTo does NOT include it (view/session field,
//                         intentionally excluded from Revert baseline — see gui_state.hpp:807
//                         field-sync scope comment). No baseline → no diff possible at this task
//                         scope. Legacy DIRTY_IF wrapper at panels.cpp:1067 keeps working.
//                         SUMMARY.md AC1 documents this as the second predicted exception found
//                         during M2 implementation (per plan §7 risk 5 门槛).

namespace lumice::gui {

struct GuiState;  // forward decl (gui_state.hpp)

struct GuiEffects {
  bool need_resim = false;
  bool need_hard_reset = false;

  friend bool operator==(const GuiEffects& a, const GuiEffects& b) {
    return a.need_resim == b.need_resim && a.need_hard_reset == b.need_hard_reset;
  }
  friend bool operator!=(const GuiEffects& a, const GuiEffects& b) { return !(a == b); }
  // TODO(T1/T3): add need_display_push / need_reset when raypath_color display migration and
  // ResetFrontendState command owner land (doc/gui-state-governance.md T1/T3).
};

// Pure function. Reads `state` (specifically its auto-diff-participating fields and
// last_committed_state); does not write. If last_committed_state is nullopt (before first commit),
// returns default-constructed GuiEffects (all false) — the reconciler is a no-op before the first
// commit, because there is no baseline to diff against.
GuiEffects ReconcileGuiEffects(const GuiState& state);

// Applies the effect via existing single-writer methods on GuiState (MarkDirty / MarkFilterDirty).
// Uses explicit priority (hard-reset shadows resim) because MarkFilterDirty internally calls
// MarkDirty; the else-if avoids double-invocation.
void ApplyGuiEffects(GuiState& state, const GuiEffects& effects);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_STATE_RECONCILE_HPP
