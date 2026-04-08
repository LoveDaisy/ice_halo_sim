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

// Floor for the adaptive quality gate threshold (min sim_ray_num for texture upload).
// After calibration, the actual threshold may be higher (adapted to platform throughput).
// This floor ensures Windows (lower throughput) never drops below a safe minimum.
constexpr unsigned long kMinRaysFloor = 5000;

// SliderWithInput / control alignment layout constants
constexpr float kLabelColWidth = 70.0f;
constexpr float kInputWidth = 60.0f;

// Auxiliary line overlay
constexpr int kMaxSunCircles = 16;

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
