#ifndef LUMICE_GUI_ANNOTATION_ANCHORS_HPP
#define LUMICE_GUI_ANNOTATION_ANCHORS_HPP

#include <array>
#include <string>
#include <vector>

#include "include/lumice.h"  // LUMICE_ANNOTATION_MARKER_COUNT — the id space the marker table is indexed by

namespace lumice::gui {

// The GUI's single owner of LUMICE_ComputeAnnotationAnchors: where the overlay's TEXT and the sky
// reference-point MARKERS land on a canvas, asked for and answered every frame.
//
// A CALL, NOT A CACHE. The curves themselves — horizon, grid, angular-distance circles — are not
// answered here at all: the preview shader evaluates them per fragment from the fragment's own
// world direction (preview_renderer.cpp overlayAuxLines), so they re-project with the picture on
// the frame the view changes. What a fragment cannot know is where along its curve the number
// should sit, or where a named direction lands; that needs a forward projection walked along the
// curve, and core does that walk in tens of microseconds. So this is called once per frame with
// the view as it stands and answers for that frame — the text and the rings move with the drag,
// exactly as the lines do. There is no debounce, no settle count and no held result from an
// earlier view: a frame's anchors are that frame's, or they are nothing.
//
// (Its predecessor cached a width*height mask sweep and recomputed only after the view had held
// still for three frames, which is what froze every annotation for the duration of a drag. The
// sweep is gone from the C API with v4.28, and the reason to wait went with it.)
//
// ONE CALL SERVES EVERY FAMILY. The label anchors of all four curve families and the six marker
// points come out of one request, so a marker's name and its ring, or a circle and its number,
// cannot be answered for two different views.
class AnnotationAnchors {
 public:
  // Everything the answer depends on: the view, and which curves and markers to answer for.
  struct ViewKey {
    int width = 0;
    int height = 0;
    int lens_type = 0;
    float fov = 0.0f;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    float roll = 0.0f;
    int visible = 0;
    // Dual-fisheye overlap band, as core's projection understands it (r_scale). Not merely
    // another view field: it changes where each disc's rim lands, so it must state the projection
    // the picture being annotated was actually drawn with — which for the preview shader is NO
    // band (AnnotationViewInputFor says why).
    float overlap = 0.0f;
    bool front = false;
    float sun_dir[3] = { 0.0f, 0.0f, 0.0f };
    std::vector<float> angular_dist_deg;
    // The coordinate grid: parallels and meridians, expanded from the FOV-adaptive step by
    // ComputeGridElevationAngles / ComputeGridLongitudeAngles (app.hpp).
    std::vector<float> elevation_deg;
    std::vector<float> longitude_deg;
    // Which named reference directions to report a canvas position for, as
    // LUMICE_ANNOTATION_MARKER_* ids. Same "whoever is switched on joins the list" rule the three
    // angle lists above follow, and for the same reason — an id in this list is a direction core
    // projects, so an unwanted one is work nobody reads.
    std::vector<int> marker_ids;
    // The celestial horizon: one curve, at altitude 0, asked for by EITHER of its two switches
    // like the three families above — a user drawing only the line still gets no anchor for it
    // unless the label switch is on, but the request is built from both so the builder has one
    // rule rather than four.
    bool horizon = false;
  };

  struct Label {
    float px = 0.0f;  // canvas pixel, x right, y down, origin top-left
    float py = 0.0f;
    float value_deg = 0.0f;
    std::string text;
  };

  // Where one marker landed, in the same canvas pixel space Label uses. `valid` false means the
  // view does not image that direction at all — which is the ordinary case for a single lens, not
  // an error, since zenith and nadir are opposite directions.
  struct Point {
    float px = 0.0f;
    float py = 0.0f;
    bool valid = false;
  };

  // Compute the anchors for this key, NOW, replacing whatever was held. Called once per frame by
  // the live preview and once per export by the off-screen path; a key that asks for nothing
  // (every list empty, horizon off) clears the result without calling core.
  void Compute(const ViewKey& key);

  // True when the last Compute produced a result. False before the first call, after a key that
  // asked for nothing, and after a call core rejected.
  bool HasResult() const { return has_result_; }
  int Width() const { return width_; }
  int Height() const { return height_; }
  // Anchors for the angular-distance circles only.
  const std::vector<Label>& AngularDistLabels() const { return angular_dist_labels_; }
  // Anchors for both grid families, merged: core has already formatted each label's text, and the
  // consumer draws them in one style.
  const std::vector<Label>& GridLabels() const { return grid_labels_; }
  // Anchors for the celestial horizon. Its own list rather than merged into the grid's, even
  // though the horizon IS the parallel at altitude 0 and shares the grid's collision group: the
  // GUI colours it separately (a red of its own, against the grid's shared colour), and an
  // appearance boundary is exactly what a separate list is for.
  const std::vector<Label>& HorizonLabels() const { return horizon_labels_; }
  // Where one named reference direction landed, INDEXED BY THE CORE MARKER ID rather than by the
  // position of that id in the request list — so a caller reads MarkerPoint(MARKER_SUBSUN) without
  // knowing, or having to keep, which slot it asked for it in. Invalid unless HasResult() and the
  // key asked for that id; each carries its own `valid`, because a view images some of the six far
  // more often than others (zenith and nadir are opposite directions and rarely both on canvas).
  // An id outside the range returns a permanently-invalid point rather than reading out of bounds.
  const Point& MarkerPoint(int marker_id) const {
    static const Point kNone{};
    return (marker_id < 0 || marker_id >= LUMICE_ANNOTATION_MARKER_COUNT) ? kNone : marker_points_[marker_id];
  }

 private:
  bool has_result_ = false;
  int width_ = 0;
  int height_ = 0;
  std::vector<Label> angular_dist_labels_;
  std::vector<Label> grid_labels_;
  std::vector<Label> horizon_labels_;
  // Indexed by core marker id, not by request order — see MarkerPoint().
  std::array<Point, LUMICE_ANNOTATION_MARKER_COUNT> marker_points_;
};

// Build the request key for a view. One owner, because the live preview and the off-screen export
// must not describe the same view differently — including the full-sky rule, where the shader
// applies no view transform and core would, so the angles are zeroed here for that lens family.
// `width`/`height` are the CANVAS the answer is wanted in, which is the export's own size rather
// than the viewport's whenever the two differ.
struct AnnotationViewInput {
  int lens_type = 0;
  float fov = 0.0f;
  float azimuth = 0.0f;
  float elevation = 0.0f;
  float roll = 0.0f;
  int visible = 0;
  float overlap = 0.0f;
  bool front = false;
  float sun_altitude_deg = 0.0f;
  std::vector<float> angular_dist_deg;
  std::vector<float> elevation_deg;
  std::vector<float> longitude_deg;
  std::vector<int> marker_ids;
  bool horizon = false;
};
AnnotationAnchors::ViewKey MakeAnnotationViewKey(const AnnotationViewInput& in, int width, int height);

// The C view struct for a key: the ONE translation from the GUI's description of a view into what
// core is asked about. Three callers — AnnotationAnchors::Compute (the anchors), the analysis
// panel's click (LUMICE_UnprojectPixel) and its IN_FRAME request (LUMICE_RaypathAnalysisRequest
// .frame_view) — and a click has to be unprojected through exactly the projection the anchors
// were placed with, or the ring lands beside the label. A second copy of these ten assignments is
// how that agreement would end.
LUMICE_AnnotationView BuildAnnotationView(const AnnotationAnchors::ViewKey& key);

// A core canvas point as the preview shader's marker uniforms want it. The two spaces are not the
// same and differ in two ways at once: core's origin is the top-left corner with y DOWN
// (annotation_overlay.hpp), the shader's is the canvas centre with y UP (`pos = v_ndc *
// u_resolution * 0.5`, preview_renderer.cpp). So a translation AND a y flip, not just one.
//
// EXCEPT FOR THE FULL-SKY FAMILY, which is why `lens_type` is a parameter and not a default. The
// shader hands overlayAuxLines `pos_ovl`, not `pos`, and for the core-pixel-inverse lenses
// (kFullSkyLensTypes: the three dual fisheyes, rectangular, dual orthographic) `pos_ovl` is
// `vec2(pos.x, -pos.y)` — that flip exists so the GUI's picture matches the CLI's, which inverts
// core's y-DOWN pixel layout. So in the overlay's own space those five lenses are y-DOWN and the
// second flip must not be applied. overlay_labels.cpp's CPU mirror of the shader already carries
// the same branch, in the same three places, for the same reason.
//
// This was invisible for as long as the family had exactly two members: on a full-sky lens the
// zenith and the nadir both land on the canvas's horizontal centre line, where py == h/2 and the
// flip is the identity. It becomes visible the moment a marker sits off that line — on rectangular
// it always did, and the sun-relative four do on every lens.
//
// A point that missed becomes the sentinel the shader's distance test already rejects, which is
// what stops an unimaged marker from drawing a ring at the canvas corner.
//
// One owner because there are two callers — the live preview and the off-screen export — and they
// pass DIFFERENT canvas sizes. A second copy of a formula that is only correct relative to the
// size it was given is exactly the kind of duplicate this task exists to remove.
void CanvasPointToShaderScreenPos(const AnnotationAnchors::Point& p, int lens_type, int canvas_w, int canvas_h,
                                  float out[2]);

// The world direction of the sun as the GUI means it, matching core's annotation::SunWorldDir at
// azimuth 0 — which is every case the GUI has, since it exposes no sun azimuth control. Shared so
// the preview's request, the shader's reference direction and any other consumer cannot drift
// into two spellings of one formula.
void GuiSunWorldDir(float altitude_deg, float out[3]);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_ANNOTATION_ANCHORS_HPP
