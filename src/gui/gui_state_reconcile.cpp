#include "gui/gui_state_reconcile.hpp"

#include "gui/gui_state.hpp"
// Tier semantics for the fields diffed below are documented in gui/gui_state_tiers.hpp — the .cpp
// intentionally does not include it, because the reconciler hardcodes the field set for readability
// (the meta-test in M4 is what verifies the tier table and this file agree).

namespace lumice::gui {

GuiEffects ReconcileGuiEffects(const GuiState& state) {
  GuiEffects effects;
  if (!state.last_committed_state.has_value()) {
    return effects;  // no baseline yet → no-op (first-commit gate)
  }
  const auto& baseline = *state.last_committed_state;

  // T-struct·soft: re-sim carry-forward. Any change → need_resim=true.
  if (state.crystals != baseline.crystals) {
    effects.need_resim = true;
  }
  if (state.layers != baseline.layers) {
    effects.need_resim = true;
  }
  if (state.sun != baseline.sun) {
    effects.need_resim = true;
  }
  if (state.sim != baseline.sim) {
    effects.need_resim = true;
  }
  if (state.renderer != baseline.renderer) {
    effects.need_resim = true;
  }

  // T-struct·hard: re-sim + hard reset (clear display + raise epoch floor).
  if (state.filters != baseline.filters) {
    effects.need_resim = true;
    effects.need_hard_reset = true;
  }

  // raypath_color: explicitly excluded from auto-diff (see gui_state_reconcile.hpp header comment
  // and doc/gui-state-governance.md T1). Legacy MarkFilterDirty call sites handle it.
  //
  // use_gpu_backend: not in ConfigSnapshot baseline; legacy DIRTY_IF wrapper handles it.
  return effects;
}

void ApplyGuiEffects(GuiState& state, const GuiEffects& effects) {
  // Explicit priority: MarkFilterDirty internally calls MarkDirty, so an else-if avoids
  // double-invocation and makes the "hard shadows soft" precedence obvious to a future reader.
  if (effects.need_hard_reset) {
    state.MarkFilterDirty();
  } else if (effects.need_resim) {
    state.MarkDirty();
  }
}

}  // namespace lumice::gui
