#ifndef LUMICE_GUI_CONSTANTS_HPP
#define LUMICE_GUI_CONSTANTS_HPP

namespace lumice::gui {

// Layout constants
constexpr int kInitWindowWidth = 1280;
constexpr int kInitWindowHeight = 720;
constexpr int kMinWindowWidth = 800;
constexpr int kMinWindowHeight = 600;
constexpr float kLeftPanelWidth = 380.0f;
constexpr float kTopBarHeight = 40.0f;
constexpr float kStatusBarHeight = 28.0f;

// Live-edit timing constants
// Invariant: kCommitIntervalMs >= kPollIntervalMs (commit should not be faster than poll)
constexpr int kCommitIntervalMs = 50;  // Min interval between auto-commits (T_commit, ms)
constexpr int kPollIntervalMs = 20;    // Server poll interval (T_poll, ms). Shorter than VSync frame (16.67ms at 60fps)
                                       // to ensure each frame has fresh data available via TrySyncData().
constexpr int kTargetFrameTimeMs = 16;  // Fallback frame time limit (ms). Prevents busy-wait when VSync fails
                                        // (known issue on Windows+NVIDIA, GLFW #1559/#2049).
constexpr int kTextureHoldMs = 30;      // After restart, hold old texture for this long before uploading new data.
                                        // Ensures consistent responsiveness across all devices.

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
