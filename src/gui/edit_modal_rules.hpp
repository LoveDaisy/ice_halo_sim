#ifndef LUMICE_GUI_EDIT_MODAL_RULES_HPP
#define LUMICE_GUI_EDIT_MODAL_RULES_HPP

// The Edit-Entry modal's list-editing rules: the OR-row list's caps and delete gate, the custom
// spectrum's caps, and which validation states block a commit.
//
// Each of these stood as an inline expression in a draw function, reachable only by opening the
// modal and clicking through to the state in question. They are pure functions of their arguments
// — no globals, no ImGui — so a test can check each cap at cap-1 / cap / cap+1 directly.
//
// Extracted verbatim: each body is the expression that stood at its call site. The one that
// gained a second consumer is SummandRowBlocksCommit: the OK-button gate (RenderFilterSubPanel's
// action row) and the commit path (ApplyBuffersToEntry) each carried their own copy of "a row is
// acceptable iff it validated kValid", which is exactly the pair that must never disagree — a row
// the OK button accepts but the committer rejects is silently dropped. One owner now, read twice.

#include <algorithm>
#include <cstddef>
#include <string>

#include "gui/raypath_segments.hpp"  // GuiValidationResult
#include "include/lumice.h"          // LUMICE_RaypathValidationState

namespace lumice::gui {

// UI soft cap on OR rows: prevents unbounded row growth from the "+ Add" button before the real
// ABI-layer limits kick in. The authoritative overflow gates live in file_io.cpp::BuildScene
// (ExpandSopToClauses → clauses vec with LUMICE_MAX_CONFIG_CLAUSES cap) and
// BuildExportJsonOrWarn; those remain the last-word validators. The ABI ceiling above this is far
// higher (v4.9 raised LUMICE_MAX_CONFIG_CLAUSES to 4096), but this UI soft cap sits below it —
// ImGui re-renders every row per frame without virtualization, so several-thousand rows would tank
// the editor's frame rate. 256 covers real "few-hundred OR summands" use cases with comfortable
// headroom while staying inside the practical UI budget.
constexpr std::size_t kMaxSummandRows = 256;
// Compile-time sentinel — the whole point of the v4.9 ABI widening is to let the GUI accept > 16
// OR rows without the pre-v4.9 hard cap. If a future change accidentally lowers this back to the
// historical 16, the intent is lost silently — the assert makes that regression a build-time
// failure.
static_assert(kMaxSummandRows > 16, "kMaxSummandRows must stay above the pre-v4.9 cap of 16");

// The "+ Add OR row" button has run out of room.
inline bool AtSummandRowCap(std::size_t row_count) {
  return row_count >= kMaxSummandRows;
}

// A row's delete button is live. The last row may not be removed: an empty list has no way back
// to a filter, so the editor keeps one row that the user can blank instead.
inline bool CanDeleteSummandRow(std::size_t row_count) {
  return row_count > 1;
}

// Which validation verdicts stop a row from being committed. Blank rows validate as kValid and so
// pass here — "empty ≡ no filter" is resolved by stripping them at commit time, not by this gate.
inline bool SummandRowBlocksCommit(LUMICE_RaypathValidationState state) {
  return state != LUMICE_RAYPATH_VALID;
}

// What the disabled OK button says about the first row that blocks it. Empty string when the row
// does not block — the caller keeps walking. kIncomplete is phrased as "still typing" rather than
// as an error, because it is the state every half-typed raypath passes through.
inline std::string SummandRowOkTooltip(std::size_t row_index, const GuiValidationResult& v) {
  if (!SummandRowBlocksCommit(v.state)) {
    return std::string();
  }
  const std::string prefix = "Row " + std::to_string(row_index + 1) + ": ";
  if (v.state == LUMICE_RAYPATH_INCOMPLETE) {
    return prefix + "finish typing (incomplete)";
  }
  return prefix + (v.message.empty() ? "invalid" : v.message);
}

// The custom-spectrum editor's "Add row" button has run out of room. kSpectrumHardMax mirrors
// core's wl_pool.hpp::kWlPoolSizeMax (gui_state.hpp).
inline bool AtSpectrumRowCap(int row_count) {
  return row_count >= kSpectrumHardMax;
}

// The custom-spectrum editor's OK button. An empty list cannot become a spectrum, and committing
// one would break the invariant "spectrum_index == custom ⟹ buffer non-empty".
inline bool SpectrumCommitBlocked(std::size_t row_count) {
  return row_count == 0;
}

// The visible band a custom-spectrum wavelength is held to. Two sites need it — the seed for a
// newly added row and the sanitize pass on OK — and they must agree, or "Add row" can propose a
// value the commit then silently moves.
constexpr float kSpectrumWavelengthMinNm = 380.0f;
constexpr float kSpectrumWavelengthMaxNm = 780.0f;

inline float ClampSpectrumWavelengthNm(float wavelength_nm) {
  return std::clamp(wavelength_nm, kSpectrumWavelengthMinNm, kSpectrumWavelengthMaxNm);
}

// Weights are amplitudes, so a negative one is not a dim row — it is a row that subtracts light.
inline float ClampSpectrumWeight(float weight) {
  return std::max(weight, 0.0f);
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EDIT_MODAL_RULES_HPP
