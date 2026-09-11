#ifndef LUMICE_GUI_ANALYSIS_RESULT_HPP
#define LUMICE_GUI_ANALYSIS_RESULT_HPP

// The raypath-analysis result as the GUI carries it (doc/raypath-analysis-panel.md): the entries
// of one analysis frame, copied out of the C API, plus the frame's identity. Shared between the
// poller (which reads it off the result frame on its own thread) and GuiState (which holds the
// one currently shown), so it lives in a header neither of those two owns.
//
// Immutable once published — the poller builds one, hands it over as shared_ptr<const>, and the
// main thread only ever replaces the pointer. Display-time derivations (which rings count, how the
// rows are ordered) do NOT live here: they are recomputed by the main thread from these entries
// and kept beside them in GuiState::analysis_result, so a slider drag never touches the data a
// still-running analysis is being compared against.

#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

struct AnalysisPayload {
  // ResultFrame::snapshot_generation_ as LUMICE_RaypathAnalysisInfo exports it: the ONE "is this a
  // new result" signal. `present` is not one — it holds on every poll of the session.
  unsigned long long snapshot_generation = 0;
  int roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;  // echo of the request
  int cone_ring_count = 0;                     // CONE only, else 0
  float cone_radius_rad = 0.0f;                // CONE only, else 0
  // Energy-descending, as LUMICE_FrameGetRaypathAnalysis returns them. Never reordered here: the
  // list's display order is a separate index array over this vector (GuiState::analysis_result),
  // which is what lets a selected row survive a re-sort.
  std::vector<LUMICE_RaypathHistogramEntry> entries;
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_ANALYSIS_RESULT_HPP
