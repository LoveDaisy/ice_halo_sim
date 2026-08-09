#ifndef LUMICE_GUI_SIM_STATE_RULES_HPP
#define LUMICE_GUI_SIM_STATE_RULES_HPP

// What the simulation lifecycle state permits, as data rather than as an inline `if` at each
// widget.
//
// Every predicate here answers one question — "given only the lifecycle state (and, for the save
// modal, whether a server exists), is this command available?" — and answers it from its
// arguments alone: no GuiState, no globals, no ImGui. That is what lets a test enumerate
// SimState's five values against each predicate instead of driving a window to reach one branch
// at a time; the call sites in app_panels.cpp then read `ImGui::BeginDisabled(IsBusy(...))`
// rather than restating the comparison.
//
// The extraction is behaviour-preserving by construction: each function body is the expression
// that stood at its call site verbatim. `CanRunFromModal` is the one that gained a dependency —
// it now spells its "no run in flight" half as `!IsBusy(state)`, which is the same predicate the
// top bar uses. That is deliberate and is what the call site's own comment asked for ("matches
// the top-bar Run button gating semantics; single-source would be nicer but the top bar's enable
// predicate is inlined and not exported"). It is NOT a merge of the two gates: the modal keeps
// its own named predicate, because it also requires a live server and the top bar does not.

#include "gui/gui_state.hpp"

namespace lumice::gui {

// A run is in flight.
inline bool IsSimulating(GuiState::SimState state) {
  return state == GuiState::SimState::kSimulating;
}

// An async Stop is still draining in the backend.
inline bool IsStopping(GuiState::SimState state) {
  return state == GuiState::SimState::kStopping;
}

// The backend is unavailable for a document-level command. Wider than "simulating": New / Open /
// Save must also stay shut while a Stop drains, or they would act on a backend mid-teardown.
inline bool IsBusy(GuiState::SimState state) {
  return IsSimulating(state) || IsStopping(state);
}

// The config has changed since the last run, so the on-screen preview no longer reflects it.
// Drives both the ⚠ + Revert affordance and the Save-Modified popup.
inline bool IsModified(GuiState::SimState state) {
  return state == GuiState::SimState::kModified;
}

// "Run first" in the Save-Modified popup: meaningful only when there is a live server to run on
// AND no run is already in flight.
inline bool CanRunFromModal(bool has_server, GuiState::SimState state) {
  return has_server && !IsBusy(state);
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SIM_STATE_RULES_HPP
