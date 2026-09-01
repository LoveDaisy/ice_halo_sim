#include "gui/annotation_overlay_cache.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "gui/gui_logger.hpp"
#include "lumice.h"

namespace lumice::gui {

bool AnnotationOverlayCache::ViewKey::operator==(const ViewKey& o) const {
  return width == o.width && height == o.height && lens_type == o.lens_type && fov == o.fov && azimuth == o.azimuth &&
         elevation == o.elevation && roll == o.roll && visible == o.visible && front == o.front &&
         std::equal(std::begin(sun_dir), std::end(sun_dir), std::begin(o.sun_dir)) &&
         angular_dist_deg == o.angular_dist_deg;
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

void AnnotationOverlayCache::Recompute(const ViewKey& key) {
  ++generation_;
  built_from_ = key;
  has_built_ = true;
  has_result_ = false;
  width_ = 0;
  height_ = 0;
  angular_dist_mask_.clear();
  angular_dist_labels_.clear();

  if (key.angular_dist_deg.empty() || key.width <= 0 || key.height <= 0) {
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
  req.view.front = key.front ? 1 : 0;
  req.angular_dist_deg = key.angular_dist_deg.data();
  req.angular_dist_count = count;
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

  if (out.width > 0 && out.height > 0 && out.angular_dist != nullptr) {
    width_ = out.width;
    height_ = out.height;
    const size_t n = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
    angular_dist_mask_.assign(out.angular_dist, out.angular_dist + n);
    for (int i = 0; i < out.label_count; ++i) {
      const LUMICE_AnnotationLabel& l = out.labels[i];
      if (l.kind != LUMICE_ANNOTATION_ANGULAR_DIST) {
        continue;  // core answers for every requested category; only this one has moved over
      }
      angular_dist_labels_.push_back(Label{ l.px, l.py, l.value_deg, std::string(l.text) });
    }
    has_result_ = true;
  }
  LUMICE_ReleaseAnnotationOverlay(&out);
}

}  // namespace lumice::gui
