#ifndef LUMICE_GUI_CONSTANTS_HPP
#define LUMICE_GUI_CONSTANTS_HPP

#include <cstddef>

namespace lumice::gui {

// Layout constants
// Initial window size. Height is bound to the right-panel content footprint:
// when adding new control groups / expanding existing groups, re-evaluate this
// constant to avoid spawning a scrollbar on fresh install. On constrained
// displays (e.g. 1080p + large Dock, Windows 125% scaling), main.cpp clamps
// the actual creation size via glfwGetMonitorWorkarea — see
// ClampInitWindowSize() in main.cpp / ClampWindowSizeToWorkarea() in
// window_sizing.hpp.
constexpr int kInitWindowWidth = 1600;
constexpr int kInitWindowHeight = 980;
constexpr int kMinWindowWidth = 1024;
constexpr int kMinWindowHeight = 640;
// Safe margin for OS window decorations (title bar + borders). Deducted from
// the monitor work area (which already excludes menubar/Dock/taskbar) to
// compute the usable creation size. 50 px covers the typical 28-32 px
// decoration on macOS/Windows/Linux with ~1.5x buffer.
constexpr int kWindowDecorationMargin = 50;
constexpr float kLeftPanelWidth = 400.0f;
constexpr float kRightPanelWidth = 300.0f;
constexpr float kTopBarHeight = 40.0f;
constexpr float kStatusBarHeight = 28.0f;

// Auto-EV downsample factor for the box-sum coarse-bin metric — the mono path's choice of the
// coarse branch of LUMICE_ComputeP99Y (the algorithm itself lives in core, behind the C API;
// this constant is the call-site decision, not part of it).
// Rationale: coarse bins have f^2 larger expected hit count than fine pixels in sparse scenes,
// so the P99-over-lit anchor stabilises earlier and 77halo previews brighten faster. Math
// equivalence:
//   ev = log2(target_linear * snapshot_fine / (P99_coarse / f^2))
// Display path remains fine-res. Final f=8 confirmed by the 25-scene gold harness
// (22/25 in-band, only ms05_prob0.5_EV0.5/EV1.5 dropped vs 23/25 fine).
constexpr int kEvAutoDownsampleFactor = 8;

// Display brightness baseline, mirroring lumice::kNormScale (core/color_util.hpp). The GUI needs
// it because absolute exposure mode is computed client-side (mono_exposure_scale.hpp) rather than
// read back from the server, and reproducing RenderConsumer::ExposureScale's absolute branch
// requires the same constant it uses.
//
// A mirrored constant is a drift risk, so it is not left to a comment: the
// composition_correctness_test case ExposureScaleMirrorsCore asserts this equals the core value,
// which turns "remember to change both" into a red test rather than a hope. The same shape as
// kEvAutoDownsampleFactor above, which mirrors a call-site decision into the GUI for the same
// no-core-includes reason (src/gui/ may not include core/ — see AGENTS.md).
constexpr float kNormScale = 0.08f;

// Live-edit timing constants
// Invariant: kCommitIntervalMs >= kPollIntervalMs (commit should not be faster than poll)
constexpr int kCommitIntervalMs =
    70;  // Min interval between auto-commits (T_commit, ms).
         // 70ms gives Windows enough headroom: first_upload avg ~48ms + poll ~20ms needs >60ms window.
constexpr int kPollIntervalMs = 20;  // Server poll interval (T_poll, ms). Shorter than VSync frame (16.67ms at 60fps)
                                     // to ensure each frame has fresh data available via LoadSnapshot().
// Slow heartbeat used once the poller believes the run has reached its terminal state and has
// self-paused (ServerPoller::State::kIdleHeartbeat). Invariant I3's second clause — "idle may
// throttle to a slow heartbeat, but must never fall so silent it cannot self-heal"
// (doc/gui-preview-lifecycle-architecture.md §9) — is what this exists for: without it, the one
// poll that observes the terminal edge is the ONLY chance to get the terminal truth on screen,
// which turns a level-triggered reconciler back into an edge-triggered one.
// Invariant: kIdleHeartbeatIntervalMs > kPollIntervalMs (a heartbeat that is not a throttle down
// from the running cadence is not a heartbeat). The specific value is a pure UX/cost tradeoff with
// no correctness content — I3 demands "not silent", not any particular rate.
constexpr int kIdleHeartbeatIntervalMs = 500;
static_assert(kIdleHeartbeatIntervalMs > kPollIntervalMs,
              "the idle heartbeat must be a throttled-DOWN cadence relative to the running poll");
// Crystal edit-modal preview animation tick (ms between successive sample_seed advances while a
// shape distribution is active). ~3.3 Hz sits in the 2-4 Hz visual-comfort band (faster reads as
// noise); it is a pure UX cadence choice, not a correctness constraint.
constexpr int kCrystalPreviewAnimIntervalMs = 300;
constexpr int kTargetFrameTimeMs = 16;  // Fallback frame time limit (ms). Prevents busy-wait when VSync fails
                                        // (known issue on Windows+NVIDIA, GLFW #1559/#2049).
// Calibration window for quality gate threshold calculation (ms).
// Decoupled from kCommitIntervalMs so that changing commit timing doesn't
// accidentally tighten/loosen the quality gate.
constexpr int kCalibrationWindowMs = 50;

// Timeout for quality gate fallback (ms). If the quality gate continuously rejects uploads
// for this duration, force-upload the current buffer (may be empty → black screen).
// This handles edge cases where sim_ray_num grows very slowly (e.g. very few simulation threads).
// The main stale-texture fix is MarkStructHardDirty (anchor reset + display epoch floor) + the
// SyncFromPoller epoch-keyed upload gate; this timeout is defense-in-depth.
// Rationale: normal first upload takes 100-200ms; 500ms is 2.5-5x margin.
constexpr int kQualityGateTimeoutMs = 500;

// Floor for the adaptive quality gate threshold (min sim_ray_num for texture upload).
// After calibration, the actual threshold may be higher (adapted to platform throughput).
// This floor ensures Windows (lower throughput) never drops below a safe minimum.
constexpr unsigned long long kMinRaysFloor = 5000;

// SliderWithInput / control alignment layout constants
constexpr float kLabelColWidth = 70.0f;
constexpr float kInputWidth = 60.0f;

// How many characters of a filter's raypath body the entry card's summary keeps before it cuts to
// "...". A named constant rather than a literal at each of its two sites, so that the number and
// the reasoning about the card column it was measured against (FilterSummary in panels.cpp) can be
// found from either end — the previous literal could only be reached by grepping the function name.
constexpr size_t kFilterSummaryBodyChars = 16;

// Reference drag track length, in pixels: the horizontal distance a DragFloat traverses to
// cross its whole domain. It is the width SliderWithInput's slider half gets, so a merged
// single control keeps the same "how far do I have to drag" feel rather than inheriting
// ImGui's default (range/100, i.e. 100 px for the full domain — more than twice as twitchy).
// Consumed by DragFloatField (panels.cpp), which derives v_speed = (max - min) / this. In
// logarithmic mode ImGui divides the delta by the domain size itself, so the same formula
// lands the same 230 px full-domain traversal there.
constexpr float kDragTrackReferenceWidth = 230.0f;

// Card thumbnail (offscreen crystal rendering)
// Currently used for both FBO render resolution and UI display size.
// If HiDPI support is needed later, split into separate render/display constants.
constexpr int kThumbnailSize = 96;
constexpr int kMaxThumbnailUpdatesPerFrame = 2;

// Vertical gap between stacked hover-action buttons (Delete on top, Duplicate below).
constexpr float kHoverBtnGap = 4.0f;

// Border thickness applied to the entry card while its edit modal is open.
// Default ImGui ChildBorderSize is 1.0f; 2.0f provides a clearly visible
// distinction without over-thickening. Consumed by RenderEntryCard via
// PushStyleVar(ImGuiStyleVar_ChildBorderSize).
constexpr float kActiveCardBorder = 2.0f;

// Border strength of a card that shares its (crystal, filter) pair with the one whose edit modal
// is open. The accent at this alpha is theme.cpp's established "about to be acted on" marker.
constexpr float kCoSharedBorderAlpha = 0.55f;

// Border strength of the card the pointer is over. Deliberately the WEAKEST of the three card
// border states: the other two say something about the open edit modal, this one only says the
// pointer is here, and a hover that competed with them would make the informative pair harder to
// distinguish from each other. Consumed by RenderEntryCard.
constexpr float kCardHoverBorderAlpha = 0.30f;

// Outline drawn over the entry card's "Link to" rail slot when that card shares its
// (crystal, filter) pair with another. An outline rather than a different glyph: what the button
// DOES is constant, and sharing is a separate fact drawn on top of it.
constexpr float kSharedOutlineThickness = 1.5f;

// Opacity of an excluded entry card's thumbnail area — the rendered image, the placeholder that
// stands in for it, and the frame around both. One constant rather than a per-branch literal so
// "excluded" is one visual statement instead of two that can drift apart.
constexpr float kExcludedThumbAlpha = 70.0f / 255.0f;

// Default camera zoom for the crystal renderer. Lower value → crystal fills
// more of the canvas (screen coverage ≈ 1/zoom). Must stay in sync between
// the thumbnail cache and the edit-modal preview so the crystal does not
// visually jump when opening the modal.
constexpr float kDefaultCrystalZoom = 1.4f;

// Camera elevation (downward pitch) for the modal/thumbnail crystal preview,
// in degrees. The camera sits at world (0, -dist, dist·tan(kCameraTiltDeg))
// looking at the origin, with world +z up. Implemented as a fixed rotation
// V_rot = Rx(+kCameraTiltDeg) inside CrystalRenderer::BuildViewRotation —
// the additional -90° remap from world (+z up) to OpenGL eye-space (+y up)
// is provided implicitly by the Y-Z swap in BuildCrystalMeshData, so V_rot
// only needs to add the camera elevation. The +sign on the Rx angle tilts
// world +z (mesh +y, the crystal's c-axis) TOWARD the camera so the top
// face is visible — matching the "elevated camera looking down" intent
// (kPlate shows face 1, not face 2). Mouse-drag rotates the crystal in
// world coordinates while the camera position stays put.
constexpr float kCameraTiltDeg = 15.0f;

// Auxiliary line overlay
constexpr int kMaxSunCircles = 16;

// Lens projection types. Order must match kLensTypeNames in gui_state.hpp
// and Core's LensParam::LensType enum. Static asserts in gui_state.hpp guard
// the array length; ordering itself must be kept in sync manually.
enum LensType : int {
  kLensTypeLinear = 0,
  kLensTypeFisheyeEqualArea = 1,
  kLensTypeFisheyeEquidist = 2,
  kLensTypeFisheyeStereographic = 3,
  kLensTypeDualFisheyeEqualArea = 4,
  kLensTypeDualFisheyeEquidist = 5,
  kLensTypeDualFisheyeStereographic = 6,
  kLensTypeRectangular = 7,
  kLensTypeFisheyeOrthographic = 8,
  kLensTypeDualFisheyeOrthographic = 9,
  kLensTypeGlobe = 10,
};

// Lenses whose shader path skips the view matrix (full-sphere mapping):
// dual fisheye (4-6), rectangular (7), dual orthographic (9). For these,
// elevation/azimuth/roll/FOV slider have no visual effect and are disabled in UI.
// SINGLE SOURCE OF TRUTH for the "full-sky" set.
//
// Two-layer guarding:
//   * kLensTypePresentationOrder's static_assert(== kLensTypeCount) in
//     gui_state.hpp forces a developer to ACK every new LensType — that's
//     where the "did you classify this lens?" prompt fires.
//   * The static_assert below only guards kFullSkyLensTypes itself: it
//     catches accidental edits to this array (size drift) and reminds the
//     reviewer to update the literal alongside any policy change here.
inline constexpr int kFullSkyLensTypes[] = {
  kLensTypeDualFisheyeEqualArea, kLensTypeDualFisheyeEquidist,     kLensTypeDualFisheyeStereographic,
  kLensTypeRectangular,          kLensTypeDualFisheyeOrthographic,
};
inline constexpr int kFullSkyLensTypeCount = sizeof(kFullSkyLensTypes) / sizeof(*kFullSkyLensTypes);
static_assert(kFullSkyLensTypeCount == 5,
              "kFullSkyLensTypes count changed: update both the array and this "
              "literal in lockstep with the policy change.");

inline constexpr bool LensIsFullSky(int lens_type) {
  for (int v : kFullSkyLensTypes) {
    if (v == lens_type) {
      return true;
    }
  }
  return false;
}

// Lenses that have a lens-border circle: the projection's own valid image region
// is bounded, so the area outside it renders pure black and is indistinguishable
// from the background unless outlined. SINGLE SOURCE OF TRUTH for the "has border"
// set; the shader-side implementation is overlayLensBorder() in preview_renderer.cpp
// and must stay in lockstep with this array.
//
// The three single-lens members are exactly the fisheyeInverse branches carrying a
// domain guard (equal-area / equidistant asin+theta guards, orthographic asin guard).
// Single-lens stereographic is excluded on the owner's call: theta never reaches 180
// so it always fills the display and leaves no black region.
//
// All four dual-fisheye variants are included. Their black region does not come from
// the projection formula at all but from the hard circle clip in dualFisheyeInverse
// (`if (!in_left && !in_right) return 0`), which sits ahead of the per-type theta
// branch and therefore applies to stereographic too. Note this differs from the
// single-lens ruling above for a mechanical reason, not an inconsistent one.
// linear and rectangular have no bounded image circle at all. globe DOES have one --
// globeInverse rejects every ray that misses the sphere (`disc < 0.0`), so the disc
// edge is a genuine domain boundary -- and it is excluded here on the owner's product
// call, not for want of a boundary.
//
// WHAT THIS ARRAY IS: the set of lenses whose image circle is worth OUTLINING -- a
// product judgement. It is NOT a predicate for "is this pixel inside the projection
// domain". The two coincide over the seven members listed here, which is exactly why
// the distinction is invisible at this call site and worth stating: a per-pixel domain
// test may exclude no lens at all, globe least of all. Do not reuse this array as one.
//
// The static_assert below guards array size; ordering is enforced by
// kLensTypePresentationOrder in gui_state.hpp.
inline constexpr int kLensBorderLensTypes[] = {
  kLensTypeFisheyeEqualArea,        kLensTypeFisheyeEquidist,     kLensTypeFisheyeOrthographic,
  kLensTypeDualFisheyeEqualArea,    kLensTypeDualFisheyeEquidist, kLensTypeDualFisheyeStereographic,
  kLensTypeDualFisheyeOrthographic,
};
inline constexpr int kLensBorderLensTypeCount = sizeof(kLensBorderLensTypes) / sizeof(*kLensBorderLensTypes);
static_assert(kLensBorderLensTypeCount == 7,
              "kLensBorderLensTypes count changed: update both the array and this "
              "literal in lockstep with the policy change.");

inline constexpr bool LensHasBorder(int lens_type) {
  for (int v : kLensBorderLensTypes) {
    if (v == lens_type) {
      return true;
    }
  }
  return false;
}

// Lenses whose default FOV is 180 degrees (full hemispheric fisheye family).
// SINGLE SOURCE OF TRUTH for the "FOV=180" set; do NOT confuse with
// kFullSkyLensTypes (which classifies skip-view-matrix shader paths). These
// two sets overlap but are not equal: e.g. kLensTypeRectangular is full-sky
// but its default FOV is 90, not 180.
//
// The static_assert below guards array size; ordering is enforced by
// kLensTypePresentationOrder in gui_state.hpp.
inline constexpr int kFov180LensTypes[] = {
  kLensTypeFisheyeEqualArea,     kLensTypeFisheyeEquidist,     kLensTypeFisheyeStereographic,
  kLensTypeDualFisheyeEqualArea, kLensTypeDualFisheyeEquidist, kLensTypeDualFisheyeStereographic,
};
inline constexpr int kFov180LensTypeCount = sizeof(kFov180LensTypes) / sizeof(*kFov180LensTypes);
static_assert(kFov180LensTypeCount == 6,
              "kFov180LensTypes count changed: update both the array and this "
              "literal in lockstep with the policy change.");

inline constexpr bool LensIsFov180(int lens_type) {
  for (int v : kFov180LensTypes) {
    if (v == lens_type) {
      return true;
    }
  }
  return false;
}

// Globe lens camera distance (eye-space). The camera sits at (0, 0, D) looking
// toward the unit sphere centered at the origin. Must match GLSL globeInverse
// in preview_renderer.cpp shader source (search "kGlobeCameraD" anchor in the
// shader string).
inline constexpr float kGlobeCameraD = 4.0f;

// Visible region selector (base hemisphere). Order must match kVisibleNames in gui_state.hpp.
// Front-hemisphere clipping is an independent flag (RenderConfig::front), not an enum value.
enum Visible : int {
  kVisibleUpper = 0,
  kVisibleLower = 1,
  kVisibleFull = 2,
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
