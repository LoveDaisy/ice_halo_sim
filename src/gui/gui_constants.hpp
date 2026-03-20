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
constexpr int kTimingIntervalMs = 50;                 // Base interval for commit and poll (ms)
constexpr int kCommitIntervalMs = kTimingIntervalMs;  // Min interval between auto-commits (T_commit)
constexpr int kPollIntervalMs = kTimingIntervalMs;    // Server poll interval (T_poll)

// Texture upload hold constants (prevent black frame flash after simulation restart)
// After a restart, the first few snapshots have very low intensity (few rays), producing sparse/dark frames.
// Hold the old texture until the new one reaches a meaningful level OR the hold timeout expires.
constexpr float kMinIntensityRatio = 0.05f;  // New data must reach 5% of current displayed intensity
constexpr int kMaxTextureHoldMs = 500;       // Unconditional upload after 500ms (extreme param change fallback)

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
