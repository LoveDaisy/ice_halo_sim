#include "gui/annotation_anchors.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"
#include "gui/preview_renderer.hpp"  // kOverlaySentinel (the shader's "no marker here" position)
#include "lumice.h"

namespace lumice::gui {

void GuiSunWorldDir(float altitude_deg, float out[3]) {
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  const float sa = altitude_deg * kDeg2Rad;
  out[0] = -std::cos(sa);
  out[1] = 0.0f;
  out[2] = -std::sin(sa);
}

AnnotationAnchors::ViewKey MakeAnnotationViewKey(const AnnotationViewInput& in, int width, int height) {
  AnnotationAnchors::ViewKey key;
  key.width = width;
  key.height = height;
  key.lens_type = in.lens_type;
  key.fov = in.fov;
  key.visible = in.visible;
  key.overlap = in.overlap;
  key.front = in.front;
  // Zeroed for the full-sky lenses, matching the preview shader: those branches skip the view
  // matrix entirely (LensIsFullSky, and `needs_view_transform` in preview_renderer.cpp), so their
  // canvas is pinned to world azimuth. Core honours the camera angles unconditionally, so handing
  // them over would put the anchors somewhere the picture is not looking.
  const bool needs_view = !LensIsFullSky(in.lens_type);
  key.azimuth = needs_view ? in.azimuth : 0.0f;
  key.elevation = needs_view ? in.elevation : 0.0f;
  key.roll = needs_view ? in.roll : 0.0f;
  GuiSunWorldDir(in.sun_altitude_deg, key.sun_dir);
  const size_t n = std::min(in.angular_dist_deg.size(), static_cast<size_t>(kMaxAnnotationCircles));
  key.angular_dist_deg.assign(in.angular_dist_deg.begin(), in.angular_dist_deg.begin() + n);
  // The same per-family ceiling for the axis-referenced list: the bound is the shader's canvas for
  // one family's levels, and each family has its own (gui_constants.hpp).
  const size_t nv = std::min(in.view_dist_deg.size(), static_cast<size_t>(kMaxAnnotationCircles));
  key.view_dist_deg.assign(in.view_dist_deg.begin(), in.view_dist_deg.begin() + nv);
  // Clamped to the API's own ceiling rather than passed through: a request past it is REJECTED,
  // not truncated (lumice.h), which would drop the circles and the grid together over a limit only
  // one family exceeded. The narrowest FOV the GUI allows expands to 720 meridians, so this is a
  // reachable clamp and not a defensive one.
  const size_t ne = std::min(in.elevation_deg.size(), static_cast<size_t>(LUMICE_MAX_ANNOTATION_LINES));
  key.elevation_deg.assign(in.elevation_deg.begin(), in.elevation_deg.begin() + ne);
  const size_t nl = std::min(in.longitude_deg.size(), static_cast<size_t>(LUMICE_MAX_ANNOTATION_LINES));
  key.longitude_deg.assign(in.longitude_deg.begin(), in.longitude_deg.begin() + nl);
  // No clamp against LUMICE_MAX_ANNOTATION_MARKERS here, unlike the three angle lists above, and
  // the difference is structural rather than an omission: the GUI switches each of the six ids on
  // or off individually, so the list it can produce is at most the id space itself — six, against
  // a ceiling of sixteen. There is no document field a user can grow past it.
  key.marker_ids = in.marker_ids;
  key.horizon = in.horizon;
  return key;
}

void CanvasPointToShaderScreenPos(const AnnotationAnchors::Point& p, int lens_type, int canvas_w, int canvas_h,
                                  float out[2]) {
  if (!p.valid || canvas_w <= 0 || canvas_h <= 0) {
    out[0] = kOverlaySentinel;
    out[1] = kOverlaySentinel;
    return;
  }
  out[0] = p.px - static_cast<float>(canvas_w) * 0.5f;
  // kFullSkyLensTypes is exactly the set whose shader branch hands overlayAuxLines a y-DOWN
  // `pos_ovl` — see the header. Reusing that predicate rather than restating the five ids keeps the
  // classification in one place; the shader's own list and this one are then two readings of one
  // fact instead of two lists.
  const float centred_y = p.py - static_cast<float>(canvas_h) * 0.5f;
  out[1] = LensIsFullSky(lens_type) ? centred_y : -centred_y;
}

LUMICE_AnnotationView BuildAnnotationView(const AnnotationAnchors::ViewKey& key) {
  LUMICE_AnnotationView view{};
  view.width = key.width;
  view.height = key.height;
  view.lens_type = key.lens_type;
  view.lens_fov = key.fov;
  view.view_azimuth = key.azimuth;
  view.view_elevation = key.elevation;
  view.view_roll = key.roll;
  view.visible = key.visible;
  view.overlap = key.overlap;
  view.front = key.front ? 1 : 0;
  // lens_shift stays 0: the preview has no shift control (doc/gui-state-governance.md §9 names it
  // among the fields the GUI does not expose), and the anchors were always placed with 0.
  return view;
}

void AnnotationAnchors::Compute(const ViewKey& key) {
  has_result_ = false;
  width_ = 0;
  height_ = 0;
  angular_dist_labels_.clear();
  view_dist_labels_.clear();
  grid_labels_.clear();
  horizon_labels_.clear();
  marker_points_.fill(Point{});

  // `marker_ids` belongs in this guard, not only in the request below: it is a fourth thing the
  // caller can ask for, and a user who turns the markers on while every angle list is empty is the
  // ordinary case, not a corner one. Left out, the function would return before calling core at
  // all and the markers would silently never appear.
  if ((key.angular_dist_deg.empty() && key.view_dist_deg.empty() && key.elevation_deg.empty() &&
       key.longitude_deg.empty() && key.marker_ids.empty() && !key.horizon) ||
      key.width <= 0 || key.height <= 0) {
    return;
  }

  LUMICE_AnnotationRequest req{};
  req.view = BuildAnnotationView(key);
  req.angular_dist_deg = key.angular_dist_deg.data();
  req.angular_dist_count = static_cast<int>(key.angular_dist_deg.size());
  req.view_dist_deg = key.view_dist_deg.data();
  req.view_dist_count = static_cast<int>(key.view_dist_deg.size());
  req.elevation_deg = key.elevation_deg.data();
  req.elevation_count = static_cast<int>(key.elevation_deg.size());
  req.longitude_deg = key.longitude_deg.data();
  req.longitude_count = static_cast<int>(key.longitude_deg.size());
  req.marker_ids = key.marker_ids.data();
  req.marker_count = static_cast<int>(key.marker_ids.size());
  req.horizon = key.horizon ? 1 : 0;
  std::copy(std::begin(key.sun_dir), std::end(key.sun_dir), std::begin(req.reference_dir));

  LUMICE_AnnotationAnchors out{};
  const LUMICE_ErrorCode err = LUMICE_ComputeAnnotationAnchors(&req, &out);
  if (err != LUMICE_OK) {
    GUI_LOG_WARNING("[Overlay] LUMICE_ComputeAnnotationAnchors failed: {}", static_cast<int>(err));
    return;
  }

  width_ = key.width;
  height_ = key.height;
  // Re-INDEXED from request order into core-id order, which is the whole reason MarkerPoint()
  // takes an id: out.marker_points[i] answers for key.marker_ids[i], so a consumer reading it
  // positionally would have to carry the request list around with the result. Core's own bound
  // is honoured rather than assumed — a shorter reply than the request is a core-side error, and
  // reading past it would be one here.
  const int n_markers = std::min(out.marker_count, static_cast<int>(key.marker_ids.size()));
  for (int i = 0; i < n_markers; ++i) {
    const int id = key.marker_ids[static_cast<size_t>(i)];
    if (id < 0 || id >= LUMICE_ANNOTATION_MARKER_COUNT) {
      continue;
    }
    const LUMICE_AnnotationMarkerPoint& mp = out.marker_points[i];
    marker_points_[static_cast<size_t>(id)] = Point{ mp.px, mp.py, mp.valid != 0 };
  }
  for (int i = 0; i < out.label_count; ++i) {
    const LUMICE_AnnotationLabel& l = out.labels[i];
    // Core answers for every requested category, and every category the GUI draws text for is
    // read here. The reference-point markers appear in no `kind` at all: core returns them as
    // POINTS (read above), not labels, because a marker carries no text of core's. The GUI draws a
    // NAME beside one when asked, but that string is kMarkerDisplayNames — the consumer's
    // vocabulary, which is exactly why core does not ship it.
    if (l.kind == LUMICE_ANNOTATION_ANGULAR_DIST) {
      angular_dist_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
    } else if (l.kind == LUMICE_ANNOTATION_VIEW_DIST) {
      view_dist_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
    } else if (l.kind == LUMICE_ANNOTATION_ELEVATION || l.kind == LUMICE_ANNOTATION_LONGITUDE) {
      grid_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
    } else if (l.kind == LUMICE_ANNOTATION_HORIZON) {
      horizon_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
    }
  }
  has_result_ = true;
  LUMICE_ReleaseAnnotationAnchors(&out);
}

}  // namespace lumice::gui
