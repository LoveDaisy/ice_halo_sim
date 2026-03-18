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

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
