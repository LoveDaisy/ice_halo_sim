#ifndef LUMICE_GUI_CONSTANTS_HPP
#define LUMICE_GUI_CONSTANTS_HPP

namespace lumice::gui {

// Layout constants
constexpr int kInitWindowWidth = 1600;
constexpr int kInitWindowHeight = 900;
constexpr int kMinWindowWidth = 1024;
constexpr int kMinWindowHeight = 640;
constexpr float kLeftPanelWidth = 360.0f;
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
constexpr int kThumbnailSize = 64;
constexpr int kMaxThumbnailUpdatesPerFrame = 2;

// Default camera zoom for the crystal renderer. Lower value → crystal fills
// more of the canvas (screen coverage ≈ 1/zoom). Must stay in sync between
// the thumbnail cache and the edit-modal preview so the crystal does not
// visually jump when opening the modal.
constexpr float kDefaultCrystalZoom = 1.4f;

// Auxiliary line overlay
constexpr int kMaxSunCircles = 16;

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
