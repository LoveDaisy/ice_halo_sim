#ifndef LUMICE_GUI_ANNOTATION_OVERLAY_CACHE_HPP
#define LUMICE_GUI_ANNOTATION_OVERLAY_CACHE_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace lumice::gui {

// The GUI's single owner of LUMICE_ComputeAnnotationOverlay.
//
// WHY A CACHE AND NOT A CALL. That API is explicitly not a per-frame one: every mask it returns
// costs a width*height inverse-projection sweep, single-digit to tens of milliseconds at preview
// sizes, which is a whole 60 fps frame budget on its own. Its contract asks a caller driving an
// interactive control to debounce or to freeze the annotation for the duration of a drag. This
// class is where the GUI keeps that promise: it is handed the current view every frame and only
// recomputes once the view has HELD STILL for kSettleFrames frames.
//
// The visible consequence is intended, not a defect to be tuned away: while the user drags the
// camera the circles stay where they last landed and snap into place a few frames after the drag
// stops. The alternative is not "circles that track perfectly" — it is a preview that drops to a
// handful of frames per second while being dragged.
//
// Frames, not milliseconds, because gui_test runs on a fixed simulated dt whose clock is decoupled
// from wall time; a wall-clock debounce would settle at a different point there than in the app.
//
// ONE CALL SERVES BOTH CONSUMERS. The masks and the label anchors come out of the same request
// (want_labels is a field on it, not a second entry point), so the preview's texture and the
// on-screen text are two readings of one computation and cannot disagree about where a circle is.
class AnnotationOverlayCache {
 public:
  // Everything the answer depends on. Two views that compare equal have the same overlay, which is
  // what makes "has the view held still" a comparison rather than a heuristic.
  struct ViewKey {
    int width = 0;
    int height = 0;
    int lens_type = 0;
    float fov = 0.0f;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    float roll = 0.0f;
    int visible = 0;
    // Dual-fisheye overlap band. Not merely another view field: it changes where the two discs
    // meet, so a request that leaves it at zero annotates a slightly different projection than the
    // one the picture was drawn with — worth a couple of dB of a cross-process comparison.
    float overlap = 0.0f;
    bool front = false;
    float sun_dir[3] = { 0.0f, 0.0f, 0.0f };
    std::vector<float> angular_dist_deg;
    // The coordinate grid: parallels and meridians, expanded from the FOV-adaptive step by
    // ComputeGridElevationAngles / ComputeGridLongitudeAngles (app.hpp). They ride in the same key
    // as the circles because they come out of the same call — see ONE CALL SERVES BOTH CONSUMERS
    // above, which is now one call serving three families.
    std::vector<float> elevation_deg;
    std::vector<float> longitude_deg;

    bool operator==(const ViewKey& o) const;
    bool operator!=(const ViewKey& o) const { return !(*this == o); }
  };

  struct Label {
    float px = 0.0f;  // canvas pixel, x right, y down, origin top-left
    float py = 0.0f;
    float value_deg = 0.0f;
    std::string text;
  };

  // Call once per frame with the view as it stands. Recomputes only when the key has been
  // unchanged for kSettleFrames consecutive calls AND differs from what the held result was built
  // from; otherwise keeps the previous result, stale or not. An empty angle list clears it.
  void Update(const ViewKey& key);

  // Compute NOW, skipping the debounce, unless the held result was already built for this key.
  // For one-shot callers — an off-screen export renders a single frame at a size of its own, so it
  // has no run of frames to settle over and no draw loop to protect. Update() is for the ones that
  // do.
  void Refresh(const ViewKey& key);

  // True when a result is held. False before the first settle, and after a key whose computation
  // failed or asked for no circles.
  bool HasResult() const { return has_result_; }
  int Width() const { return width_; }
  int Height() const { return height_; }
  // Row-major width*height, 1 where an angular-distance circle passes. Empty unless HasResult().
  const std::vector<uint8_t>& AngularDistMask() const { return angular_dist_mask_; }
  // Anchors for the angular-distance circles only.
  const std::vector<Label>& AngularDistLabels() const { return angular_dist_labels_; }
  // Row-major width*height, 1 where a parallel OR a meridian passes. ONE mask for both families,
  // not two: the GUI has a single colour picker and a single alpha slider for the whole grid, so
  // the two are visually indistinguishable and a second texture would cost a unit and an upload
  // to produce identical pixels. Empty unless HasResult().
  const std::vector<uint8_t>& GridMask() const { return grid_mask_; }
  // Anchors for both grid families, merged for the same reason the mask is: core has already
  // formatted each label's text, and the consumer draws them in one style.
  const std::vector<Label>& GridLabels() const { return grid_labels_; }

  // Bumped on every recompute, from a counter shared by ALL instances. A consumer that caches
  // something derived from a result — PreviewRenderer's GL texture — compares this instead of the
  // buffer contents to decide whether to redo that work, and there is more than one cache feeding
  // it: the live preview's and the off-screen export's. Per-instance counters would both start at
  // one and the renderer could not tell a fresh export mask from the preview mask it already
  // holds, so it would keep showing the preview's — at the preview's size.
  uint64_t Generation() const { return generation_; }

  // Number of consecutive identical keys before a recompute. 3 at 60 fps is ~50 ms of stillness:
  // long enough that a drag never triggers one, short enough to read as immediate.
  static constexpr int kSettleFrames = 3;

 private:
  void Recompute(const ViewKey& key);

  ViewKey pending_;         // the key seen on the last Update
  int stable_frames_ = 0;   // how many consecutive Updates have seen `pending_`
  ViewKey built_from_;      // the key the held result was computed for
  bool has_built_ = false;  // whether built_from_ means anything yet

  bool has_result_ = false;
  int width_ = 0;
  int height_ = 0;
  std::vector<uint8_t> angular_dist_mask_;
  std::vector<Label> angular_dist_labels_;
  std::vector<uint8_t> grid_mask_;
  std::vector<Label> grid_labels_;
  uint64_t generation_ = 0;  // 0 = never computed, which a consumer's "nothing uploaded" sentinel matches
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
};
AnnotationOverlayCache::ViewKey MakeAnnotationViewKey(const AnnotationViewInput& in, int width, int height);

// The world direction of the sun as the GUI means it, matching core's annotation::SunWorldDir at
// azimuth 0 — which is every case the GUI has, since it exposes no sun azimuth control. Shared so
// the preview's request and any other consumer cannot drift into two spellings of one formula.
void GuiSunWorldDir(float altitude_deg, float out[3]);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_ANNOTATION_OVERLAY_CACHE_HPP
