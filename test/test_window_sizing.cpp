#include <gtest/gtest.h>

#include "gui/gui_constants.hpp"
#include "gui/window_sizing.hpp"

namespace {

using lumice::gui::ClampWindowSizeToWorkarea;
using lumice::gui::kMinWindowHeight;
using lumice::gui::kMinWindowWidth;
using lumice::gui::MonitorRect;
using lumice::gui::SelectMonitorIndexByCenter;

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

// ========== Monitor selection (multi-monitor aspect ratio fix) ==========

TEST(MonitorSelectionTest, EmptyListReturnsNegative) {
  EXPECT_EQ(SelectMonitorIndexByCenter(100, 100, nullptr, 0), -1);
}

TEST(MonitorSelectionTest, SingleMonitorCenterInside) {
  constexpr MonitorRect kRects[] = { { 0, 0, 1920, 1080 } };
  EXPECT_EQ(SelectMonitorIndexByCenter(960, 540, kRects, 1), 0);
}

TEST(MonitorSelectionTest, SingleMonitorCenterOutside) {
  constexpr MonitorRect kRects[] = { { 0, 0, 1920, 1080 } };
  EXPECT_EQ(SelectMonitorIndexByCenter(2000, 500, kRects, 1), -1);
}

TEST(MonitorSelectionTest, DualMonitorPrimaryCenter) {
  constexpr MonitorRect kRects[] = { { 0, 0, 2560, 1440 }, { 2560, 0, 1920, 1080 } };
  EXPECT_EQ(SelectMonitorIndexByCenter(1200, 700, kRects, 2), 0);
}

TEST(MonitorSelectionTest, DualMonitorSecondaryCenter) {
  constexpr MonitorRect kRects[] = { { 0, 0, 2560, 1440 }, { 2560, 0, 1920, 1080 } };
  EXPECT_EQ(SelectMonitorIndexByCenter(3500, 500, kRects, 2), 1);
}

// Window straddles the seam between monitors; center point wins.
TEST(MonitorSelectionTest, CrossMonitorCenterPicksByCenter) {
  constexpr MonitorRect kRects[] = { { 0, 0, 2560, 1440 }, { 2560, 0, 1920, 1080 } };
  // Window x=2400, w=400 → center x = 2600 ∈ secondary.
  EXPECT_EQ(SelectMonitorIndexByCenter(2600, 200, kRects, 2), 1);
}

// Left/top edge is inclusive.
TEST(MonitorSelectionTest, BoundaryLeftEdgeInclusive) {
  constexpr MonitorRect kRects[] = { { 100, 200, 300, 400 } };
  EXPECT_EQ(SelectMonitorIndexByCenter(100, 200, kRects, 1), 0);
}

// Right/bottom edge is exclusive — prevents adjacent monitors double-claiming.
TEST(MonitorSelectionTest, BoundaryRightEdgeExclusive) {
  constexpr MonitorRect kRects[] = { { 100, 200, 300, 400 } };
  EXPECT_EQ(SelectMonitorIndexByCenter(400, 200, kRects, 1), -1);
}

}  // namespace
