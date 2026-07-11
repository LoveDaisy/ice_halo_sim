#include "gui/gui_state_reconcile.hpp"

#include <cassert>

// T1 (task-color-migration) landed this as the reconciler's only concrete-widget dependency,
// inverting the intended "widget depends on reconciler abstraction" layering direction (doc/
// gui-state-governance.md §4 支柱 2) for a single domain. Do NOT copy this include-and-call
// pattern for T2/T3 — evaluate a push-handler registration/dispatch abstraction before a second
// domain would otherwise add a second widget-header include here.
#include "gui/color_window.hpp"  // PushDisplayState (T1 export)
#include "gui/gui_state.hpp"
// Tier semantics for the fields diffed below are documented in gui/gui_state_tiers.hpp — the .cpp
// intentionally does not include it, because the reconciler hardcodes the field set for readability
// (the meta-test in M4 is what verifies the tier table and this file agree).

namespace lumice::gui {

namespace {

// task-color-migration (T1) — build the display-state baseline snapshot from the current
// raypath_color vector's ColorClassDisplayState sub-part + raypath_color_mode. Kept out of the
// pure ReconcileGuiEffects body because ApplyGuiEffects reuses it on successful push.
GuiState::DisplayStateBaseline SnapshotDisplayState(const GuiState& state) {
  GuiState::DisplayStateBaseline b;
  b.color_display.reserve(state.raypath_color.size());
  for (const auto& cls : state.raypath_color) {
    // Slice: ColorClassConfig publicly inherits ColorClassDisplayState, so this copy captures
    // exactly the display-side sub-part (color/visible/solo/z_order) and drops struct-side
    // fields (combine/match). Precisely what D3's edge-trigger baseline needs.
    b.color_display.push_back(static_cast<const ColorClassDisplayState&>(cls));
  }
  b.raypath_color_mode = state.raypath_color_mode;
  return b;
}

// Compare the display-part of two raypath_color vectors of equal size, plus raypath_color_mode.
// Callers gate on `state.raypath_color.size() == baseline.color_display.size()` before calling.
bool DisplayStateEqualAtCurrentSize(const GuiState& state, const GuiState::DisplayStateBaseline& baseline) {
  if (state.raypath_color_mode != baseline.raypath_color_mode) {
    return false;
  }
  for (size_t i = 0; i < state.raypath_color.size(); ++i) {
    const auto& live = static_cast<const ColorClassDisplayState&>(state.raypath_color[i]);
    if (live != baseline.color_display[i]) {
      return false;
    }
  }
  return true;
}

// task-color-migration (T1) — raypath_color STRUCT part diff. Vector-cardinality change or any
// per-entry ColorClassStructState (combine/match) mismatch means the physical filter topology
// changed → hard-reset lane.
bool RaypathColorStructChanged(const GuiState& state, const GuiState::ConfigSnapshot& baseline) {
  if (state.raypath_color.size() != baseline.raypath_color.size()) {
    return true;
  }
  for (size_t i = 0; i < state.raypath_color.size(); ++i) {
    const auto& live_struct = static_cast<const ColorClassStructState&>(state.raypath_color[i]);
    const auto& base_struct = static_cast<const ColorClassStructState&>(baseline.raypath_color[i]);
    if (live_struct != base_struct) {
      return true;
    }
  }
  return false;
}

// Commit-baseline diff (resim / hard-reset lane). Extracted from ReconcileGuiEffects so its
// cognitive complexity stays below clang-tidy's threshold (25). Reads state; does not write.
void DiffAgainstCommitBaseline(const GuiState& state, GuiEffects& effects) {
  if (!state.last_committed_state.has_value()) {
    return;
  }
  const auto& baseline = *state.last_committed_state;

  // T-struct·soft: re-sim carry-forward.
  if (state.crystals != baseline.crystals || state.layers != baseline.layers || state.sun != baseline.sun ||
      state.sim != baseline.sim || state.renderer != baseline.renderer) {
    effects.need_resim = true;
  }

  // T-struct·hard: re-sim + hard reset (clear display + raise epoch floor).
  if (state.filters != baseline.filters || RaypathColorStructChanged(state, baseline)) {
    effects.need_resim = true;
    effects.need_hard_reset = true;
  }
  // use_gpu_backend: not in ConfigSnapshot baseline; legacy DIRTY_IF wrapper handles it.
}

// Display-push baseline diff (independent lane). Nullopt baseline = "first push after
// DoRun/Revert/backend-swap repush discipline reset" → force a push on the next non-empty
// frame. Cardinality mismatch during settling window is silently skipped (D3).
void DiffAgainstDisplayBaseline(const GuiState& state, GuiEffects& effects) {
  if (state.raypath_color.empty()) {
    return;  // nothing to push; leave baseline as-is
  }
  if (!state.last_pushed_display_state.has_value()) {
    effects.need_display_push = true;
    return;
  }
  const auto& push_base = *state.last_pushed_display_state;
  if (push_base.color_display.size() != state.raypath_color.size()) {
    return;  // same-cardinality gate — settling; the next commit's baseline reset heals
  }
  if (!DisplayStateEqualAtCurrentSize(state, push_base)) {
    effects.need_display_push = true;
  }
}

}  // namespace

GuiEffects ReconcileGuiEffects(const GuiState& state) {
  GuiEffects effects;
  DiffAgainstCommitBaseline(state, effects);
  DiffAgainstDisplayBaseline(state, effects);
  return effects;
}

void ApplyGuiEffects(GuiState& state, LUMICE_Server* server, const GuiEffects& effects) {
  // Explicit priority: MarkFilterDirty internally calls MarkDirty, so an else-if avoids
  // double-invocation and makes the "hard shadows soft" precedence obvious to a future reader.
  if (effects.need_hard_reset) {
    state.MarkFilterDirty();
  } else if (effects.need_resim) {
    state.MarkDirty();
  }

  // Orthogonal lane: display-state edge-trigger push. Only reads `server` when this bit fires;
  // unit-test fixtures that pass server=nullptr must ensure effects.need_display_push==false
  // (contract documented in gui_state_reconcile.hpp — dereferencing nullptr here is a caller
  // violation, not a case we defensively guard against). Debug-only assert makes the contract
  // verifiable instead of a purely textual claim; no release-mode guard per this task's
  // established "contract violation, not a defensive boundary" convention.
  if (effects.need_display_push) {
    assert(server != nullptr);
    if (PushDisplayState(state, server)) {
      state.last_pushed_display_state = SnapshotDisplayState(state);
    }
    // On failure PushDisplayState logs at debug level; leave the baseline as-is so the next
    // reconcile retries the push automatically once the settling window closes.
  }
}

}  // namespace lumice::gui
