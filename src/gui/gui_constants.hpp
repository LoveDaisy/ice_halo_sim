#ifndef LUMICE_GUI_CONSTANTS_HPP
#define LUMICE_GUI_CONSTANTS_HPP

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

// Live-edit timing constants
// Invariant: kCommitIntervalMs >= kPollIntervalMs (commit should not be faster than poll)
constexpr int kCommitIntervalMs =
    70;  // Min interval between auto-commits (T_commit, ms).
         // 70ms gives Windows enough headroom: first_upload avg ~48ms + poll ~20ms needs >60ms window.
constexpr int kPollIntervalMs = 20;  // Server poll interval (T_poll, ms). Shorter than VSync frame (16.67ms at 60fps)
                                     // to ensure each frame has fresh data available via TrySyncData().
constexpr int kTargetFrameTimeMs = 16;  // Fallback frame time limit (ms). Prevents busy-wait when VSync fails
                                        // (known issue on Windows+NVIDIA, GLFW #1559/#2049).
// Calibration window for quality gate threshold calculation (ms).
// Decoupled from kCommitIntervalMs so that changing commit timing doesn't
// accidentally tighten/loosen the quality gate.
constexpr int kCalibrationWindowMs = 50;

// Timeout for quality gate fallback (ms). If the quality gate continuously rejects uploads
// for this duration, force-upload the current buffer (may be empty → black screen).
// This handles edge cases where sim_ray_num grows very slowly (e.g. very few simulation threads).
// The main stale-texture fix is the filter_changed flag in GuiState; this timeout is defense-in-depth.
// Rationale: normal first upload takes 100-200ms; 500ms is 2.5-5x margin.
constexpr int kQualityGateTimeoutMs = 500;

// Floor for the adaptive quality gate threshold (min sim_ray_num for texture upload).
// After calibration, the actual threshold may be higher (adapted to platform throughput).
// This floor ensures Windows (lower throughput) never drops below a safe minimum.
constexpr unsigned long kMinRaysFloor = 5000;

// SliderWithInput / control alignment layout constants
constexpr float kLabelColWidth = 70.0f;
constexpr float kInputWidth = 60.0f;

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

// Visible region selector. Order must match kVisibleNames in gui_state.hpp.
enum Visible : int {
  kVisibleUpper = 0,
  kVisibleLower = 1,
  kVisibleFull = 2,
  kVisibleFront = 3,
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
