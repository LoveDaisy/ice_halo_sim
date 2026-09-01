#include "gui/annotation_overlay_cache.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"
#include "gui/preview_renderer.hpp"  // kOverlaySentinel (the shader's "no marker here" position)
#include "lumice.h"

namespace lumice::gui {

bool AnnotationOverlayCache::ViewKey::operator==(const ViewKey& o) const {
  return width == o.width && height == o.height && lens_type == o.lens_type && fov == o.fov && azimuth == o.azimuth &&
         elevation == o.elevation && roll == o.roll && visible == o.visible && front == o.front &&
         std::equal(std::begin(sun_dir), std::end(sun_dir), std::begin(o.sun_dir)) &&
         angular_dist_deg == o.angular_dist_deg && elevation_deg == o.elevation_deg &&
         longitude_deg == o.longitude_deg && zenith_nadir == o.zenith_nadir;
}

void AnnotationOverlayCache::Update(const ViewKey& key) {
  if (key != pending_) {
    pending_ = key;
    stable_frames_ = 1;
    return;
  }
  // Already settled on this key and already built for it: nothing to do, and in particular no
  // reason to let stable_frames_ grow without bound.
  if (has_built_ && built_from_ == key) {
    return;
  }
  if (stable_frames_ < kSettleFrames) {
    ++stable_frames_;
    return;
  }
  Recompute(key);
}

void GuiSunWorldDir(float altitude_deg, float out[3]) {
  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  const float sa = altitude_deg * kDeg2Rad;
  out[0] = -std::cos(sa);
  out[1] = 0.0f;
  out[2] = -std::sin(sa);
}

void AnnotationOverlayCache::Refresh(const ViewKey& key) {
  // Keep the debounce state consistent with the result, so a later Update with this same key does
  // not immediately recompute what was just built.
  pending_ = key;
  stable_frames_ = kSettleFrames;
  if (has_built_ && built_from_ == key) {
    return;
  }
  Recompute(key);
}

AnnotationOverlayCache::ViewKey MakeAnnotationViewKey(const AnnotationViewInput& in, int width, int height) {
  AnnotationOverlayCache::ViewKey key;
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
  // them over would put the circles somewhere the picture is not looking.
  const bool needs_view = !LensIsFullSky(in.lens_type);
  key.azimuth = needs_view ? in.azimuth : 0.0f;
  key.elevation = needs_view ? in.elevation : 0.0f;
  key.roll = needs_view ? in.roll : 0.0f;
  GuiSunWorldDir(in.sun_altitude_deg, key.sun_dir);
  const size_t n = std::min(in.angular_dist_deg.size(), static_cast<size_t>(kMaxSunCircles));
  key.angular_dist_deg.assign(in.angular_dist_deg.begin(), in.angular_dist_deg.begin() + n);
  // Clamped to the API's own ceiling rather than passed through: a request past it is REJECTED,
  // not truncated (lumice.h), which would drop the circles and the grid together over a limit only
  // one family exceeded. The narrowest FOV the GUI allows expands to 720 meridians, so this is a
  // reachable clamp and not a defensive one.
  const size_t ne = std::min(in.elevation_deg.size(), static_cast<size_t>(LUMICE_MAX_ANNOTATION_LINES));
  key.elevation_deg.assign(in.elevation_deg.begin(), in.elevation_deg.begin() + ne);
  const size_t nl = std::min(in.longitude_deg.size(), static_cast<size_t>(LUMICE_MAX_ANNOTATION_LINES));
  key.longitude_deg.assign(in.longitude_deg.begin(), in.longitude_deg.begin() + nl);
  key.zenith_nadir = in.zenith_nadir;
  return key;
}

void CanvasPointToShaderScreenPos(const AnnotationOverlayCache::Point& p, int canvas_w, int canvas_h, float out[2]) {
  if (!p.valid || canvas_w <= 0 || canvas_h <= 0) {
    out[0] = kOverlaySentinel;
    out[1] = kOverlaySentinel;
    return;
  }
  out[0] = p.px - static_cast<float>(canvas_w) * 0.5f;
  out[1] = static_cast<float>(canvas_h) * 0.5f - p.py;
}

void AnnotationOverlayCache::Recompute(const ViewKey& key) {
  // Process-wide, so two caches' results are never confused for each other. See Generation().
  static uint64_t next_generation = 0;
  generation_ = ++next_generation;
  built_from_ = key;
  has_built_ = true;
  has_result_ = false;
  width_ = 0;
  height_ = 0;
  angular_dist_mask_.clear();
  angular_dist_labels_.clear();
  grid_mask_.clear();
  grid_labels_.clear();
  zenith_ = {};
  nadir_ = {};

  // `zenith_nadir` belongs in this guard, not only in the request below: it is a fourth thing the
  // caller can ask for, and a user who turns the markers on while every angle list is empty is the
  // ordinary case, not a corner one. Left out, the function would return before calling core at
  // all and the markers would silently never appear.
  if ((key.angular_dist_deg.empty() && key.elevation_deg.empty() && key.longitude_deg.empty() && !key.zenith_nadir) ||
      key.width <= 0 || key.height <= 0) {
    return;
  }
  // The ceiling is a rejection, not a truncation, on core's side; clamp here so a user with an
  // absurd list gets the first LUMICE_MAX_ANNOTATION_CIRCLES drawn rather than none.
  const int count = std::min(static_cast<int>(key.angular_dist_deg.size()), LUMICE_MAX_ANNOTATION_CIRCLES);

  LUMICE_AnnotationRequest req{};
  req.view.width = key.width;
  req.view.height = key.height;
  req.view.lens_type = key.lens_type;
  req.view.lens_fov = key.fov;
  req.view.view_azimuth = key.azimuth;
  req.view.view_elevation = key.elevation;
  req.view.view_roll = key.roll;
  req.view.visible = key.visible;
  req.view.overlap = key.overlap;
  req.view.front = key.front ? 1 : 0;
  req.angular_dist_deg = key.angular_dist_deg.data();
  req.angular_dist_count = count;
  req.elevation_deg = key.elevation_deg.data();
  req.elevation_count = static_cast<int>(key.elevation_deg.size());
  req.longitude_deg = key.longitude_deg.data();
  req.longitude_count = static_cast<int>(key.longitude_deg.size());
  req.zenith_nadir = key.zenith_nadir ? 1 : 0;
  std::copy(std::begin(key.sun_dir), std::end(key.sun_dir), std::begin(req.reference_dir));
  // Masks AND anchors in one call: they are two readings of the same sweep, and asking twice would
  // pay for it twice and admit the possibility of them disagreeing.
  req.want_labels = 1;

  LUMICE_AnnotationOverlay out{};
  const LUMICE_ErrorCode err = LUMICE_ComputeAnnotationOverlay(&req, &out);
  if (err != LUMICE_OK) {
    GUI_LOG_WARNING("[Overlay] LUMICE_ComputeAnnotationOverlay failed: {}", static_cast<int>(err));
    return;
  }

  if (out.width > 0 && out.height > 0) {
    width_ = out.width;
    height_ = out.height;
    const size_t n = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
    if (out.angular_dist != nullptr) {
      angular_dist_mask_.assign(out.angular_dist, out.angular_dist + n);
    }
    // Parallels OR meridians, into one buffer. Core returns a mask per CATEGORY; merging them here
    // rather than uploading two textures is the same decision as GridMask()'s own comment, and it
    // is safe precisely because the two families cannot be told apart downstream anyway.
    if (out.elevation != nullptr || out.longitude != nullptr) {
      grid_mask_.assign(n, uint8_t{ 0 });
      for (size_t i = 0; i < n; ++i) {
        const bool lit =
            (out.elevation != nullptr && out.elevation[i] != 0) || (out.longitude != nullptr && out.longitude[i] != 0);
        grid_mask_[i] = lit ? uint8_t{ 1 } : uint8_t{ 0 };
      }
    }
    zenith_ = { out.zenith_px, out.zenith_py, out.zenith_valid != 0 };
    nadir_ = { out.nadir_px, out.nadir_py, out.nadir_valid != 0 };
    for (int i = 0; i < out.label_count; ++i) {
      const LUMICE_AnnotationLabel& l = out.labels[i];
      // Core answers for every requested category; the horizon has not moved off the GUI's own
      // walk yet, so its anchors are dropped here. The zenith/nadir markers appear in no `kind` at
      // all — core returns them as POINTS (read above), not labels, because a marker carries no
      // text.
      if (l.kind == LUMICE_ANNOTATION_ANGULAR_DIST) {
        angular_dist_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
      } else if (l.kind == LUMICE_ANNOTATION_ELEVATION || l.kind == LUMICE_ANNOTATION_LONGITUDE) {
        grid_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
      }
    }
    has_result_ = true;
  }
  LUMICE_ReleaseAnnotationOverlay(&out);
}

}  // namespace lumice::gui
