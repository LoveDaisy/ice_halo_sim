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

// Texture upload hold: prevent black frame flash after simulation restart.
// The first few snapshots have very few rays, producing sparse/dark frames.
// Require a minimum ray count before replacing the displayed texture.
// At ~3M rays/sec and 50ms commit interval, ~150k rays accumulate per cycle; 50k is reachable in ~17ms.
constexpr unsigned long kMinRaysForDisplay = 50000;

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
