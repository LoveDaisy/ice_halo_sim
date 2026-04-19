#include <gtest/gtest.h>

#include "gui/gui_constants.hpp"
#include "gui/window_sizing.hpp"

namespace {

using lumice::gui::ClampWindowSizeToWorkarea;
using lumice::gui::kMinWindowHeight;
using lumice::gui::kMinWindowWidth;

// AC2 core path: 1080p laptop workarea ≈ 1920×900 after menubar/Dock.
// Default (1600, 980) height must be clamped to 900 - 50 = 850.
TEST(WindowSizingTest, ClampsHeightOnConstrained1080p) {
  auto [w, h] = ClampWindowSizeToWorkarea(1600, 980, 1920, 900);
  EXPECT_EQ(w, 1600);
  EXPECT_EQ(h, 850);
}

// High-DPI dev monitor (e.g. MacBook 2880×1800 workarea): default size fits,
// no clamp should happen.
TEST(WindowSizingTest, NoClampOnHighResDisplay) {
  auto [w, h] = ClampWindowSizeToWorkarea(1600, 980, 2880, 1800);
  EXPECT_EQ(w, 1600);
  EXPECT_EQ(h, 980);
}

// Small 1366×768 laptop: both width and height must clamp.
TEST(WindowSizingTest, ClampsBothWidthAndHeight) {
  auto [w, h] = ClampWindowSizeToWorkarea(1600, 980, 1366, 768);
  EXPECT_EQ(w, 1316);  // 1366 - 50
  EXPECT_EQ(h, 718);   // 768 - 50
}

// Pathological tiny workarea: floor must hold at kMinWindow{Width,Height}
// rather than shrinking below the documented minimum.
TEST(WindowSizingTest, FloorsAtMinWindowSize) {
  auto [w, h] = ClampWindowSizeToWorkarea(1600, 980, 800, 600);
  EXPECT_EQ(w, kMinWindowWidth);
  EXPECT_EQ(h, kMinWindowHeight);
}

// Boundary: workarea - margin == desired → no clamp change.
TEST(WindowSizingTest, ExactBoundaryNoChange) {
  auto [w, h] = ClampWindowSizeToWorkarea(1600, 980, 1650, 1030);
  EXPECT_EQ(w, 1600);
  EXPECT_EQ(h, 980);
}

}  // namespace
