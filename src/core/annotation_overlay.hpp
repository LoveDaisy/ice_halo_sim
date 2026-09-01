#ifndef CORE_ANNOTATION_OVERLAY_H_
#define CORE_ANNOTATION_OVERLAY_H_

#include <cstdint>
#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/lens_proj_build.hpp"

namespace lumice::annotation {

// =================================================================================================
// Annotation overlay geometry: "what does the current view look like" in, "where do the auxiliary
// lines and their labels land, in pixels" out.
//
// THIS IS NOT A PER-FRAME CALL. The whole result is a function of the VIEW SNAPSHOT (lens, fov,
// resolution, view angles, visible hemisphere) plus the line definitions, and every mask it
// returns costs a W*H inverse-projection sweep — measured at 8-96 ms across the lens/resolution
// space this renderer supports, i.e. above a 60 fps frame budget from 1024x1024 upward even
// after the row-parallel split below. Compute it once when the view settles and cache the result;
// a caller that drives an interactive control must debounce (or freeze the annotation during a
// drag) rather than call this from its draw loop. The contract is stated here and repeated on
// LUMICE_ComputeAnnotationOverlay because it is a property of the computation, not of any one
// caller's discipline.
//
// The geometry is expressed as W*H byte masks in the SAME row-major layout as
// BuildVisibleMask / BuildHorizonMask, and label anchors as pixel coordinates in that same image
// space (x right, y down, origin at the top-left corner). Rendering — colour, line style, glyphs,
// collision avoidance — belongs to the consumer; two consumers (the GUI preview and the CLI
// renderer) draw the same geometry their own way, which is the whole point of returning geometry
// instead of pixels.
//
// SINGLE SOURCE OF PROJECTION. Every direction here goes through lm_proj::ProjectExitToPixel, the
// same forward the three trace backends run, and every pixel-to-direction step goes through
// mask_detail::PixelToWorld, the inverse the render-domain mask is built from. The GUI carries a
// second, deliberately duplicated copy of this lens math for its display re-projection
// (overlay_labels.cpp's WorldDirToPixel, mirroring the preview shader); this file does NOT port
// that copy. test/unit-correctness/gui/test_annotation_overlay_gui_parity.cpp is the gate that
// keeps the two answers on top of each other, and pins the places they are known to differ.
// =================================================================================================

// The view the annotation is computed for. Deliberately its own struct rather than a RenderConfig:
// it carries `front`, which RenderConfig does not have yet, and it is consumed by a pure function
// with no Scene / Server lifetime around it.
struct ViewSnapshot {
  int width = 0;
  int height = 0;
  LensParam::LensType lens_type = LensParam::kDualFisheyeEqualArea;
  float fov_deg = 180.0f;
  int lens_shift[2] = { 0, 0 };
  float overlap = 0.0f;
  float az_deg = 0.0f;
  float el_deg = 0.0f;
  float roll_deg = 0.0f;
  RenderConfig::VisibleRange visible = RenderConfig::kFull;
  // Additional clip to the camera-facing hemisphere, on top of `visible`. Independent of it:
  // `visible` says which half of the SKY exists, `front` says the viewer only wants what is in
  // front of the camera.
  bool front = false;
};

// Which family a label belongs to. The consumer decides colour / font / background from this;
// core does not encode appearance.
enum LabelKind : int {
  kLabelHorizon = 0,
  kLabelElevation = 1,
  kLabelLongitude = 2,
  kLabelAngularDist = 3,
};

struct Label {
  float px = 0.0f;
  float py = 0.0f;
  LabelKind kind = kLabelHorizon;
  // Index into the request list this label's curve came from (-1 for the horizon, which has no
  // list). Lets a consumer map a label back to the line it annotates without re-deriving it from
  // the text.
  int index = -1;
  float value_deg = 0.0f;
  std::string text;
};

// What to draw. Angles are in degrees.
struct Request {
  ViewSnapshot view;

  // The celestial horizon (altitude = 0). Named separately from `elevation_deg` rather than being
  // "the 0 entry" of it because consumers colour it separately, exactly as the GUI does.
  bool horizon = false;

  // Parallels (constant altitude) and meridians (constant azimuth).
  std::vector<float> elevation_deg;
  std::vector<float> longitude_deg;

  // Circles of constant angular distance from `reference_dir` — the sun, in every use so far.
  // `reference_dir` need not be normalized; it is normalized on entry.
  std::vector<float> angular_dist_deg;
  float reference_dir[3] = { 0.0f, 0.0f, -1.0f };

  // Report where zenith / nadir land. Points, not curves: they carry no mask and no text (the
  // marker's appearance, glyph included, belongs to the consumer).
  bool zenith_nadir = false;

  // Skip the curve walk entirely. The masks alone are ~4x cheaper than masks + anchors, and a
  // consumer that draws no text has no use for the anchors.
  bool labels = true;
};

// A projected point in image pixel space, or a miss.
struct CanvasPoint {
  float px = 0.0f;
  float py = 0.0f;
  bool valid = false;
};

struct Overlay {
  int width = 0;
  int height = 0;

  // 1 where the lens images a piece of sky the request is allowed to annotate: imaged, inside
  // `visible`, and inside the front hemisphere when `front` is set. Empty if the view is
  // degenerate. Every mask below is a subset of this one.
  std::vector<uint8_t> drawable;

  // One mask per annotation CATEGORY, each the union of that category's level sets. Empty when
  // the category was not requested. Row-major W*H, same indexing as `drawable`.
  std::vector<uint8_t> horizon;
  std::vector<uint8_t> elevation;
  std::vector<uint8_t> longitude;
  std::vector<uint8_t> angular_dist;

  CanvasPoint zenith;
  CanvasPoint nadir;

  std::vector<Label> labels;
};

// Half a degree of slack at the hemisphere edge, ported verbatim from the GUI's
// ComputeOverlayLabels: a label anchor that lands exactly on the boundary is kept rather than
// dropped, which is what stops every curve tangent to the horizon from losing its label.
//
// The MASKS do not get this slack and must not: a mask pixel is a pixel that gets painted, and
// painting half a degree past the edge of the rendered hemisphere promises sky the renderer never
// produces. An anchor is a text position, and half a degree of it is invisible.
inline constexpr float kLabelHemisphereToleranceDeg = 0.5f;

// The float-noise band at the front-hemisphere boundary that a strict `dot > 0` cull lets fall
// through; ~sin(0.57 deg). Ported from the GUI's kFrontEps for the same reason as the tolerance
// above — the two implementations have to agree about which samples are in.
inline constexpr float kFrontEps = 0.01f;

// The view snapshot as the RenderConfig the projection helpers take. `front` has no RenderConfig
// home yet and is applied by this layer instead.
RenderConfig ToRenderConfig(const ViewSnapshot& view);

// Forward-project a world direction onto the canvas. Thin wrapper over lm_proj::ProjectExitToPixel
// (primary hit only — a dual-fisheye overlap dual-write is a second copy of the same sky, and an
// annotation drawn twice in the overlap band is a defect, not coverage). `valid` is the lens's own
// domain verdict; it says nothing about the canvas bounds or the visible hemisphere, both of which
// this layer decides.
CanvasPoint ProjectWorldDir(const lm_proj::ProjParams& p, float wx, float wy, float wz);

// The whole computation. Returns an Overlay with width/height zero for a degenerate view.
Overlay ComputeOverlay(const Request& req);

}  // namespace lumice::annotation

#endif  // CORE_ANNOTATION_OVERLAY_H_
