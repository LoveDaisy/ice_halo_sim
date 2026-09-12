#ifndef LUMICE_GUI_ANALYSIS_PANEL_HPP
#define LUMICE_GUI_ANALYSIS_PANEL_HPP

// The "Raypath Analysis" window (doc/raypath-analysis-panel.md): a non-modal window beside the
// preview that starts an analysis run on the committed scene, lets the user say WHERE in the sky
// they are asking about (the whole sky, the frame on screen, or a cone around a point they click
// on the preview), lists the raypath chains that delivered energy there, and turns a selected
// single-crystal chain into a filter that excludes it.
//
// Pure C API consumer. Everything the window does reaches core through lumice.h — the run
// (LUMICE_StartRaypathAnalysis), the result (LUMICE_FrameGetRaypathAnalysis, read on the main
// thread under the P/B/D symmetry the panel's checkboxes name — the reduction is the server's,
// done on every read, so a toggle re-reads the result on hand and starts no run), and the
// click's direction (LUMICE_UnprojectPixel) — and every piece of logic that is not an ImGui
// call is a free function below, so the unit layer can drive it without a frame.
//
// Ownership, stated once. The panel's own state is GuiState::analysis (session tier); the result
// is GuiState::analysis_result (derived); the in-progress flag is GuiState::analysis_run_in_progress
// (derived). The panel writes the first, the adoption below writes the second, SyncFromPoller
// writes the third through DeriveAnalysisInProgress. No widget writes a document field.

#include <cstdint>
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
// snapshot_generation NEWER than the one already held. The ONE place that decides "new",
// called by SyncFromPoller every frame with whatever the snapshot carries (carry-forwards and all)
// and by tests directly. On adoption: the selection is cleared (it named a chain of the old
// result) and the display order is recomputed for the current radius. Returns whether it adopted.
// A payload the poller published carries no entries; SyncFromPoller follows an adoption with
// RefreshAnalysisEntries, which reads them under the panel's symmetry.
//
// The held generation starts at 0, and 0 is what a payload can never carry (the server's counter
// is incremented before it is stamped on a frame — server.cpp DoSnapshot), so the first real
// result is always adopted; and since the counter only grows, a second analysis session on the
// same server can never repeat a value this view has already held. "Newer", not "different":
// RefreshAnalysisEntries may put a frame on show that is one snapshot AHEAD of what the poller
// last published (a Stop publishes the run's final snapshot after the poller has paused, and
// the poller never catches up), and the poller's carry-forward of the older one must not
// replace it — that would clear the selection every frame, on a result that never changed.
bool AdoptAnalysisPayloadIfNew(GuiState& state, const std::shared_ptr<const AnalysisPayload>& payload);

// How many of a CONE result's rings lie within `radius_deg` of the centre, given the request's
// full cone of `cone_radius_deg` split into `ring_count` equal rings: the smallest k such that
// k rings cover the radius, clamped to [1, ring_count]. ring_count <= 0 answers 0.
int RingsWithinRadius(float radius_deg, float cone_radius_deg, int ring_count);

// Energy of the first `rings` rings of an entry (clamped to what the entry holds). Rings <= 0 is 0.
double SumRingEnergy(const LUMICE_RaypathHistogramEntry& entry, int rings);

// Rebuild analysis_result's display_energy / display_order / display_cumulative_pct /
// display_total from the payload and the slider (state.analysis.cone_radius_deg) — a CONE result
// sums the rings inside the radius, every other mode shows `energy` as delivered; the payload's
// other bucket enters the total whole (it is not ring-split) and closes the cumulative column.
// Pure re-projection of data already on hand: touches no lifecycle, no dirty, no server. Called on
// adoption, on every entry re-read, and on every slider change.
void RecomputeAnalysisDisplayOrder(GuiState& state);

// The share of the total that the fixed "other" line shows: the other bucket's energy over
// display_total, as a percentage; 0 without a payload or with an empty bucket.
double AnalysisOtherPct(const GuiState& state);

// The label of that line — in the header so a test can address the row. No chain formats to
// this (a chain's text is digits, dashes, parentheses, "C<id>" and " -> "), so it can never
// name an entry: SelectedAnalysisEntry finds nothing for it even if it were ever written into
// analysis.selected_entry, and the row is a disabled selectable so it never is.
inline constexpr const char* kAnalysisOtherRowLabel = "other (not recorded)";

// ---- The symmetry, and the read of the entries under it ------------------------------------------

// The panel's three checkboxes as the LUMICE_RAYPATH_SYMMETRY_* bit set a read takes.
uint8_t AnalysisSymmetryBits(const GuiState& state);

// Whether the entries on show are stale against what the panel asks for: a result is held and
// it has not yet been read as (its snapshot_generation, AnalysisSymmetryBits) — never read at all
// (fetched_once false), or read for another generation or another symmetry. Pure.
bool AnalysisEntriesNeedRefresh(const GuiState& state);

// Read the held result's entries under the panel's symmetry when AnalysisEntriesNeedRefresh says
// they are stale: LUMICE_AcquireResultFrame -> LUMICE_FrameGetRaypathAnalysisInfo /
// LUMICE_FrameGetRaypathAnalysis under AnalysisSymmetryBits -> release, on the calling (main)
// thread, and replace analysis_result.payload with one carrying the entries (and the frame's own
// identity and echo fields), recompute the display order and record what was read. The selection
// is kept: it names a chain, and SelectedAnalysisEntry finds it again iff the chain is still a
// row under the new symmetry. Starts no run and never waits for a poll — this is what makes a
// checkbox immediate.
//
// The frame the server hands back can be past the poller's payload (a newer snapshot landed
// since); the payload then takes the frame's generation, so what is shown is one consistent
// frame. If the server no longer holds an analysis frame (a render has since been committed —
// the result on show is deliberately kept across that, app.cpp DoRun), nothing can be re-read:
// the entries and entries_symmetry stay as they are, the attempt is recorded so it is not
// repeated every frame, and the panel shows the difference. Returns whether entries were read.
// No-op with a null server.
bool RefreshAnalysisEntries(GuiState& state, LUMICE_Server* server);

// The selected row's entry — the entry of analysis_result.payload whose `display` equals
// analysis.selected_entry — or nullptr when nothing is selected or no row carries that text
// (the chain was merged away by a symmetry change, or the result is a new one).
const LUMICE_RaypathHistogramEntry* SelectedAnalysisEntry(const GuiState& state);

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
// `view`. On sky, writes cone_center_dir / cone_center_valid and disarms the pick; off sky
// (letterbox, outside the lens's image circle, the clipped hemisphere) writes nothing and leaves
// the pick armed, so the user can aim again. Returns whether it hit sky.
bool PickAnalysisConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int px, int py);

// Where the cone centre sits on the canvas THIS frame: cone_center_dir forward-projected through
// `view` by LUMICE_ProjectDirection (the sampler the sky-reference markers use, so it moves with
// the view and shows/hides at the hemisphere edge as they do), rounded to the nearest canvas
// pixel and clamped into the canvas. nullopt when there is no valid centre or the direction is
// not on this view's picture — then nothing is drawn and nothing can be grabbed; the list is
// unaffected. Rounded to the integer CanvasPixel on purpose: the marker is drawn and hit-tested
// through the same CanvasPixelToPreviewPoint the ring uses, and half a canvas pixel is invisible
// under a 3-point dot.
std::optional<CanvasPixel> ProjectConeCenterMarker(const GuiState& state, const LUMICE_AnnotationView& view);

// Who owns the mouse over the preview while the analysis window is in CONE mode. ONE arbiter
// for three gestures that share the same rectangle, evaluated once per frame and read by every
// branch: the marker under the cursor (or a drag already in flight) wins over an armed pick,
// which wins over the camera. Hover-over-pick is the ruling stated in the design (a cursor on
// the old marker while a new pick is armed grabs the marker: the two overlap only there, and
// dragging what is under the cursor is the less surprising outcome). Outside CONE mode the
// caller does not consult this and the camera owns the preview as it always did.
enum class ConeInputOwner {
  kMarkerDrag,  // hover on the marker, or a drag in progress
  kPickClick,   // pick armed: the next click sets the centre
  kCamera,      // neither: orbit / zoom / background gestures as before
};
ConeInputOwner ArbitrateConeInput(bool marker_hover_or_dragging, bool pick_armed);

// Move the centre to the direction under canvas pixel (px, py) — the drag's per-frame write.
// The sibling of PickAnalysisConeCenter: the same LUMICE_UnprojectPixel through the same view,
// the same "on sky writes, off sky writes nothing" rule (a drag past the picture's edge keeps the
// last direction that was sky rather than parking the centre on garbage). Touches nothing else:
// not pick_armed, not cone_center_valid (a drag can only start on a marker, which only exists
// while the centre is valid), and it does not log per frame. Returns whether it hit sky.
bool DragAnalysisConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int px, int py);

// Give CONE mode a centre when it has none: the direction under the viewport's centre pixel
// (vp_w/2, vp_h/2) through `view`. Level-triggered by the panel each frame CONE mode has no
// valid centre and a preview is on screen, so a mode switch before the first picture still gets
// its default once the picture arrives. Returns whether a centre was placed; a miss (the centre
// pixel not on sky — a degenerate view) writes nothing and the caller tries again next frame.
bool EnsureDefaultConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int vp_w, int vp_h);

// "The centre has moved since the result on show was computed": a CONE result is held, the
// centre is valid, and cone_center_dir differs from the direction that result was requested
// with (analysis.analyzed_cone_center_dir) by more than float noise — the dot product of the two
// unit vectors under 1 - 1e-6. Not bitwise, because the drag path re-derives the direction
// through LUMICE_UnprojectPixel while the request copied the field, and any move the eye can
// see is far past that band. False whenever there is no CONE result to compare against.
bool ConeCenterDriftedFromResult(const GuiState& state);

// The on-screen radius, in canvas pixels, of a `radius_deg` cone around the pixel (px, py) under
// `view` — the LOCAL scale, measured by unprojecting a neighbouring pixel and reading the angle
// between the two directions, so it is a linearisation of whatever lens is on screen at that
// point. Nothing judges by it (the ROI test is the server's, on directions); it is the ring the
// user sees, and near a fisheye's rim or for a wide cone it is visibly approximate. nullopt when
// either pixel misses the sky, in which case there is no ring to draw.
std::optional<float> ConeRingRadiusCanvasPx(const LUMICE_AnnotationView& view, int px, int py, float radius_deg);

// ---- The request ---------------------------------------------------------------------------------

// Seed the session's own ray budget (analysis.ray_num_millions / analysis.infinite) from the
// document's (state.sim.ray_num_millions / state.sim.infinite) once — level-triggered on
// !ray_budget_initialized, the shape EnsureDefaultConeCenter has, so the first frame the window
// shows carries the document's Rays and every later frame carries whatever the user left. Nothing
// re-syncs it to a later document edit; a document switch resets the whole session (app.cpp
// ResetFrontendState) and so re-seeds from the new document.
void EnsureDefaultAnalysisRayBudget(GuiState& state);

// The LUMICE_RaypathAnalysisRequest for the session's ROI. IN_FRAME takes the preview view on the
// canvas_w x canvas_h canvas; CONE takes the picked centre with the FULL cone
// (kAnalysisConeMaxRadiusDeg / kAnalysisConeRingCount) — the slider is applied at display time,
// never here, and so is the symmetry (the request has none; v4.33). The ray budget is the session's own
// (ray_num_millions / infinite), always explicit: the GUI never sends LUMICE_RAYPATH_RAY_BUDGET_SCENE_DEFAULT, because
// it already holds the value the sentinel would stand for.
LUMICE_RaypathAnalysisRequest BuildAnalysisRequest(const GuiState& state, int canvas_w, int canvas_h);

// ---- Exclude this raypath ------------------------------------------------------------------------

enum class ExcludeEligibility {
  kOk,                 // no filter on that crystal yet (a fresh Out filter is written), or an Out
                       // filter already (this chain is appended to it as one more OR row)
  kNoSelection,        // no row selected, or no row of the result on show carries that chain
  kMultiSegment,       // the chain crosses scattering layers; a filter on one crystal cannot say it
  kCrystalNotInScene,  // the chain's crystal id is not one the CURRENT document commits (edited
                       // since the analysis) — never guess which crystal was meant
  kEntryHasInFilter,   // an entry using that crystal already has an IN filter: "keep only these
                       // paths" and "also drop this one" do not compose into one filter, and
                       // per-row In/Out is not a thing the filter model has; the user edits it
};

// Why the Exclude button is, or is not, enabled for the current selection. `why` (optional)
// receives the tooltip text for a denial. Pure: reads the document and the result, writes nothing.
ExcludeEligibility EvaluateExcludeEligibility(const GuiState& state, std::string* why);

// The sentence the Exclude button's tooltip adds when the exclusion would EXTEND a filter rather
// than only create one: an entry of the chain's crystal already carries an Out filter, so the
// selection becomes one more OR row of it — and, when other entries share that filter's pool
// slot, how many, in the words the entry card's link badge uses (CountEntriesSharing); when the
// crystal's entries hold several distinct Out filters, how many; and when some of them hold none,
// that those get a fresh filter. Empty when there is no filter to extend or the selection is not
// eligible. Pure.
std::string ExcludeAppendNotice(const GuiState& state);

// "3-5": the face sequence of one chain segment in the raypath grammar the filter editor uses.
std::string FormatSegmentRaypathText(const LUMICE_RaypathChainSegment& segment);

// The exclusion itself, on an eligible selection, reaching every entry that uses the chain's
// crystal (the histogram counted it wherever it was used). Entries with no filter get one: a
// filter_out filter on the chain's face sequence, symmetric under the bits the list on show was
// reduced with (analysis_result.entries_symmetry — the reduction the selected row was counted
// under, so the filter removes exactly the rows the user sees merged into it, no wider and no
// narrower), written through the filter editor's own pool-write path, which binds it to the
// whole (crystal, no-filter) group. Entries with an Out filter get the face sequence appended to
// it as one more OR row (the filter's name, action and symmetry stay what they were), the pool
// slot overwritten in place so every entry sharing it sees the row; a row already in it is not
// added twice, and a slot with nothing to add is not written at all. The frame-tail reconciler
// sees the filters diff and marks the document hard-dirty; the user re-runs. Returns false,
// writing nothing, when the selection is not eligible.
bool ApplyExcludeSelectedRaypath(GuiState& state);

// ---- Rendering -----------------------------------------------------------------------------------

// The window. No-op while state.analysis.window_open is false.
void RenderAnalysisPanel(GuiState& state, LUMICE_Server* server);

// The ROI marker and ring on the preview, drawn by RenderPreviewPanel after its interaction
// button: CONE mode with a valid centre that projects onto this frame's picture draws the centre
// dot and the ring of the slider's radius around it (ProjectConeCenterMarker); every other state
// draws nothing. `origin` is the preview window's top-left in screen points.
void DrawAnalysisRoiRing(const GuiState& state, const LUMICE_AnnotationView& view, const ImVec2& origin,
                         float dpi_scale_x, float dpi_scale_y);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_ANALYSIS_PANEL_HPP
