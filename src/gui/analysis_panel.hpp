#ifndef LUMICE_GUI_ANALYSIS_PANEL_HPP
#define LUMICE_GUI_ANALYSIS_PANEL_HPP

// The "Raypath Analysis" window (doc/raypath-analysis-panel.md): a non-modal window beside the
// preview that starts an analysis run on the committed scene, lets the user say WHERE in the sky
// they are asking about (the whole sky, the frame on screen, or a cone around a point they click
// on the preview), lists the raypath chains that delivered energy there, and turns a selected
// single-crystal chain into a filter that excludes it.
//
// Pure C API consumer. Everything the window does reaches core through lumice.h — the run
// (LUMICE_StartRaypathAnalysis), the result (LUMICE_FrameGetRaypathAnalysis via the poller), and
// the click's direction (LUMICE_UnprojectPixel) — and every piece of logic that is not an ImGui
// call is a free function below, so the unit layer can drive it without a frame.
//
// Ownership, stated once. The panel's own state is GuiState::analysis (session tier); the result
// is GuiState::analysis_result (derived); the in-progress flag is GuiState::analysis_run_in_progress
// (derived). The panel writes the first, the adoption below writes the second, SyncFromPoller
// writes the third through DeriveAnalysisInProgress. No widget writes a document field.

#include <memory>
#include <optional>
#include <string>

#include "gui/analysis_result.hpp"
#include "gui/gui_state.hpp"
#include "include/lumice.h"

struct ImVec2;

namespace lumice::gui {

struct PreviewSnapshot;

// ---- Lifecycle -----------------------------------------------------------------------------------

// The in-progress flag, from the analysis intent and the poller's last observation. Level-
// triggered, like ReconcileSimState: not started -> false; started but nothing valid observed
// since (the wake edge publishes valid=false) -> true, the run was just submitted; otherwise the
// lifecycle the poller read. RUNNING is the only in-progress answer — a natural end reads
// COMPLETED, a Stop reads IDLE (has_ever_consumed_ is reset by Stop), and both mean "not now".
bool DeriveAnalysisInProgress(bool started, const PreviewSnapshot* snap);

// ---- Result adoption and the display-time projection ----------------------------------------------

// Adopt `payload` as the result on show iff it is a NEW result: non-null and carrying a
// snapshot_generation different from the one already held. The ONE place that decides "new",
// called by SyncFromPoller every frame with whatever the snapshot carries (carry-forwards and all)
// and by tests directly. On adoption: the selection is cleared (it indexed the old entries) and the
// display order is recomputed for the current radius. Returns whether it adopted.
//
// The held generation starts at 0, and 0 is what a payload can never carry (the server's counter
// is incremented before it is stamped on a frame — server.cpp DoSnapshot), so the first real
// result is always adopted; and since the counter only grows, a second analysis session on the
// same server can never repeat a value this view has already held.
bool AdoptAnalysisPayloadIfNew(GuiState& state, const std::shared_ptr<const AnalysisPayload>& payload);

// How many of a CONE result's rings lie within `radius_deg` of the centre, given the request's
// full cone of `cone_radius_deg` split into `ring_count` equal rings: the smallest k such that
// k rings cover the radius, clamped to [1, ring_count]. ring_count <= 0 answers 0.
int RingsWithinRadius(float radius_deg, float cone_radius_deg, int ring_count);

// Energy of the first `rings` rings of an entry (clamped to what the entry holds). Rings <= 0 is 0.
double SumRingEnergy(const LUMICE_RaypathHistogramEntry& entry, int rings);

// Rebuild analysis_result's display_energy / display_order / display_total from the payload and
// the slider (state.analysis.cone_radius_deg) — a CONE result sums the rings inside the radius,
// every other mode shows `energy` as delivered. Pure re-projection of data already on hand:
// touches no lifecycle, no dirty, no server. Called on adoption and on every slider change.
void RecomputeAnalysisDisplayOrder(GuiState& state);

// ---- ROI: the click on the preview ---------------------------------------------------------------

// A point on the preview panel, in logical points relative to the panel window's origin, to the
// pixel index on the vp_w x vp_h canvas the annotation anchors and LUMICE_UnprojectPixel work in.
// The DPI factors are the panel's own (PreviewViewport::dpi_scale_*): points times DPI is device
// pixels, floored (lumice.h: "round a sub-pixel position down before calling"). nullopt when the
// point falls outside the canvas.
struct CanvasPixel {
  int px = 0;
  int py = 0;
};
std::optional<CanvasPixel> PreviewPointToCanvasPixel(float rel_x_pt, float rel_y_pt, float dpi_scale_x,
                                                     float dpi_scale_y, int vp_w, int vp_h);

// The inverse of the above for one canvas pixel's CENTRE — where the ROI ring is drawn.
void CanvasPixelToPreviewPoint(int px, int py, float dpi_scale_x, float dpi_scale_y, float* out_x_pt, float* out_y_pt);

// The LUMICE_AnnotationView describing the preview as drawn NOW on a canvas_w x canvas_h canvas:
// the same view the overlay anchors are computed for (AnnotationViewInputFor ->
// MakeAnnotationViewKey -> BuildAnnotationView), so a click is unprojected through the projection
// the picture under it was drawn with.
LUMICE_AnnotationView PreviewAnnotationView(const GuiState& state, int canvas_w, int canvas_h);

// Turn the click at canvas pixel (px, py) into the cone centre: LUMICE_UnprojectPixel through
// `view`. On sky, writes cone_center_dir / cone_center_px / cone_center_valid and disarms the
// pick; off sky (letterbox, outside the lens's image circle, the clipped hemisphere) writes
// nothing and leaves the pick armed, so the user can aim again. Returns whether it hit sky.
bool PickAnalysisConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int px, int py);

// The on-screen radius, in canvas pixels, of a `radius_deg` cone around the pixel (px, py) under
// `view` — the LOCAL scale, measured by unprojecting a neighbouring pixel and reading the angle
// between the two directions, so it is a linearisation of whatever lens is on screen at that
// point. Nothing judges by it (the ROI test is the server's, on directions); it is the ring the
// user sees, and near a fisheye's rim or for a wide cone it is visibly approximate. nullopt when
// either pixel misses the sky, in which case there is no ring to draw.
std::optional<float> ConeRingRadiusCanvasPx(const LUMICE_AnnotationView& view, int px, int py, float radius_deg);

// ---- The request ---------------------------------------------------------------------------------

// The LUMICE_RaypathAnalysisRequest for the session's ROI. IN_FRAME takes the preview view on the
// canvas_w x canvas_h canvas; CONE takes the picked centre with the FULL cone
// (kAnalysisConeMaxRadiusDeg / kAnalysisConeRingCount / kAnalysisConeStopTarget) — the slider is
// applied at display time, never here. Symmetry is always the session default.
LUMICE_RaypathAnalysisRequest BuildAnalysisRequest(const GuiState& state, int canvas_w, int canvas_h);

// ---- Exclude this raypath ------------------------------------------------------------------------

enum class ExcludeEligibility {
  kOk,
  kNoSelection,        // no row selected, or the index no longer addresses an entry
  kMultiSegment,       // the chain crosses scattering layers; a filter on one crystal cannot say it
  kCrystalNotInScene,  // the chain's crystal id is not one the CURRENT document commits (edited
                       // since the analysis) — never guess which crystal was meant
  kEntryHasFilter,     // an entry using that crystal already has a filter; no automatic merge
};

// Why the Exclude button is, or is not, enabled for the current selection. `why` (optional)
// receives the tooltip text for a denial. Pure: reads the document and the result, writes nothing.
ExcludeEligibility EvaluateExcludeEligibility(const GuiState& state, std::string* why);

// "3-5": the face sequence of one chain segment in the raypath grammar the filter editor uses.
std::string FormatSegmentRaypathText(const LUMICE_RaypathChainSegment& segment);

// The exclusion itself, on an eligible selection: a filter_out filter on the chain's face
// sequence, P|B|D symmetric (the same reduction the chain was counted under), bound to every entry
// that uses the chain's crystal through the filter editor's own pool-write path. The frame-tail
// reconciler sees the filters diff and marks the document hard-dirty; the user re-runs. Returns
// false, writing nothing, when the selection is not eligible.
bool ApplyExcludeSelectedRaypath(GuiState& state);

// ---- Rendering -----------------------------------------------------------------------------------

// The window. No-op while state.analysis.window_open is false.
void RenderAnalysisPanel(GuiState& state, LUMICE_Server* server);

// The ROI ring on the preview, drawn by RenderPreviewPanel after its interaction button: CONE mode
// with a valid centre draws the ring of the slider's radius around the cached click pixel; every
// other state draws nothing. `origin` is the preview window's top-left in screen points.
void DrawAnalysisRoiRing(const GuiState& state, const LUMICE_AnnotationView& view, const ImVec2& origin,
                         float dpi_scale_x, float dpi_scale_y);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_ANALYSIS_PANEL_HPP
