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

// The backend is unavailable for a document-level command OR for a render commit, counting the
// analysis run beside the render run. The two runs share one server and exclude each other at the
// C API (LUMICE_StartRaypathAnalysis / LUMICE_CommitScene each return LUMICE_ERR_SERVER while the
// other is in progress), and an analysis run is deliberately NOT a SimState value: sim_state
// answers "what does the picture on screen reflect", and an analysis changes nothing about the
// picture. So the top bar's Run / New / Open, and the modal's "Run first", take the analysis flag
// as a second argument rather than reading it off a sixth enum value. `analysis_in_progress` is
// GuiState::analysis_run_in_progress, derived each frame by SyncFromPoller.
inline bool IsBackendBusy(GuiState::SimState state, bool analysis_in_progress) {
  return IsBusy(state) || analysis_in_progress;
}

// The config has changed since the last run, so the on-screen preview no longer reflects it.
// Drives both the ⚠ + Revert affordance and the Save-Modified popup.
inline bool IsModified(GuiState::SimState state) {
  return state == GuiState::SimState::kModified;
}

// "Run first" in the Save-Modified popup: meaningful only when there is a live server to run on
// AND no run — render or analysis — is already in flight.
inline bool CanRunFromModal(bool has_server, GuiState::SimState state, bool analysis_in_progress) {
  return has_server && !IsBackendBusy(state, analysis_in_progress);
}

// The panel's Analyze button. An analysis submits the document on the panels itself
// (DoAnalyze builds the scene the way DoRun does and hands it to LUMICE_StartRaypathAnalysis), so
// it needs no run before it and no picture that matches: a fresh document, a loaded .lmc that was
// never run, and an edited (kModified) document all analyse — what the list describes is always
// the configured scene, and the panel says so in a status line when the picture on screen is
// not of it. The one thing it does need is a backend with nothing in flight (the C API's mutual
// exclusion, surfaced as a disabled button rather than as an error line after the click).
inline bool CanStartAnalysis(bool has_server, GuiState::SimState state, bool analysis_in_progress) {
  return has_server && !IsBackendBusy(state, analysis_in_progress);
}

// The panel's notice that the picture on screen is not of the document the list describes —
// the text to show, or nullptr when there is nothing to say. Two cases and they cannot both
// hold: an intent of kNone (fresh / New / a JSON import / an .lmc with no baked picture) means no
// picture of this document was ever made, and it reconciles to kIdle, which dirty never lifts
// to kModified; kModified means there is a picture, of the configuration before the edit. A
// notice, not a refusal: CanStartAnalysis does not read either.
inline const char* AnalysisPictureNotice(RunIntent intent, GuiState::SimState state) {
  if (intent == RunIntent::kNone) {
    return "No rendered image for this document yet \xe2\x80\x94 the list describes the configured scene.";
  }
  if (IsModified(state)) {
    return "Image is from a previous configuration \xe2\x80\x94 the list describes the current one.";
  }
  return nullptr;
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SIM_STATE_RULES_HPP
