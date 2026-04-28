#include <gtest/gtest.h>

#include <cmath>

#include "gui/gui_constants.hpp"
#include "gui/window_sizing.hpp"

namespace {

using lumice::gui::AspectFitResult;
using lumice::gui::ClampWindowSizeToWorkarea;
using lumice::gui::kAspectClampTolerance;
using lumice::gui::kLeftPanelWidth;
using lumice::gui::kMinWindowHeight;
using lumice::gui::kMinWindowWidth;
using lumice::gui::kRightPanelWidth;
using lumice::gui::kStatusBarHeight;
using lumice::gui::kTopBarHeight;
using lumice::gui::MonitorRect;
using lumice::gui::ResolveAspectFit;
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

// ========== Aspect-fit clamp detection (screen-too-small feedback) ==========
//
// Tests use the production layout constants (kLeftPanelWidth=400, kRightPanelWidth=300,
// kTopBarHeight=40, kStatusBarHeight=28) so they exercise the same arithmetic
// path as ApplyAspectRatio.

namespace {
constexpr float kCollapsedStripWidth = 20.0f;  // Mirror app.cpp's local constant.
}

// Wide work area easily accommodates 2:1 — no clamp.
TEST(AspectFitTest, FitsWithoutClampOnWideMonitor) {
  AspectFitResult fit = ResolveAspectFit(/*current_win_w=*/1600, /*ratio=*/2.0f,
                                         /*work_w=*/2880, /*work_h=*/1800, kLeftPanelWidth, kRightPanelWidth,
                                         kTopBarHeight, kStatusBarHeight);
  EXPECT_FALSE(fit.was_clamped);
  EXPECT_FLOAT_EQ(fit.requested_preview_ratio, 2.0f);
  EXPECT_LT(std::abs(fit.achieved_preview_ratio - 2.0f) / 2.0f, kAspectClampTolerance);
}

// 1280×720 picking 2:1 → preview region cannot reach 2:1 because chrome
// (panels + topbar + statusbar) eats too much; was_clamped must fire.
TEST(AspectFitTest, ClampsOnSmallScreen2x1) {
  AspectFitResult fit = ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                                         /*work_w=*/1280, /*work_h=*/720, kLeftPanelWidth, kRightPanelWidth,
                                         kTopBarHeight, kStatusBarHeight);
  EXPECT_TRUE(fit.was_clamped);
  // Achieved preview ratio should be much smaller than 2:1 — sanity-check
  // it stays in a plausible band rather than asserting an exact value, since
  // the recalc_w gate behavior matters more than the precise numerics.
  EXPECT_LT(fit.achieved_preview_ratio, 2.0f);
  EXPECT_GT(fit.achieved_preview_ratio, 0.5f);
  EXPECT_FLOAT_EQ(fit.requested_preview_ratio, 2.0f);
}

// 16:9 on a 1280×720 monitor: depending on chrome the achieved ratio may or
// may not exceed the 5% tolerance. We assert numerically that the helper does
// not silently misreport — was_clamped must mirror the actual deviation.
TEST(AspectFitTest, ClampsOnSmallScreen16x9) {
  constexpr float kRatio = 16.0f / 9.0f;
  AspectFitResult fit = ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/kRatio,
                                         /*work_w=*/1280, /*work_h=*/720, kLeftPanelWidth, kRightPanelWidth,
                                         kTopBarHeight, kStatusBarHeight);
  // Whatever the answer, was_clamped and the deviation must agree.
  float deviation = std::abs(fit.achieved_preview_ratio - kRatio) / kRatio;
  EXPECT_EQ(fit.was_clamped, deviation >= kAspectClampTolerance);
  EXPECT_FLOAT_EQ(fit.requested_preview_ratio, kRatio);
}

// Portrait 2:1 → 0.5 ratio. With wide work area the preview-w shrinks; this
// path is chosen for coverage of the ratio < 1 branch (height-driven).
TEST(AspectFitTest, PortraitRatio) {
  AspectFitResult fit = ResolveAspectFit(/*current_win_w=*/1600, /*ratio=*/0.5f,
                                         /*work_w=*/2880, /*work_h=*/1800, kLeftPanelWidth, kRightPanelWidth,
                                         kTopBarHeight, kStatusBarHeight);
  EXPECT_FLOAT_EQ(fit.requested_preview_ratio, 0.5f);
  // Achieved ratio is what the helper computes from the post-clamp
  // target_w/target_h; for portrait+wide-monitor it should still be ≤
  // requested (preview_h grows past the screen if anything, not preview_w).
  float deviation = std::abs(fit.achieved_preview_ratio - 0.5f) / 0.5f;
  EXPECT_EQ(fit.was_clamped, deviation >= kAspectClampTolerance);
}

// Collapsed panels → less chrome → small monitor can now fit 2:1 closer.
// We don't assert clamp polarity strictly (depends on numerics), but assert
// the helper consumes the reduced overhead by producing a *less* clamped
// achieved ratio than the equivalent expanded-panels case.
TEST(AspectFitTest, PanelsCollapsedReducesOverhead) {
  AspectFitResult expanded = ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                                              /*work_w=*/1280, /*work_h=*/720, kLeftPanelWidth, kRightPanelWidth,
                                              kTopBarHeight, kStatusBarHeight);
  AspectFitResult collapsed = ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                                               /*work_w=*/1280, /*work_h=*/720, kCollapsedStripWidth,
                                               kCollapsedStripWidth, kTopBarHeight, kStatusBarHeight);
  // Collapsing panels shouldn't make the achieved ratio worse on a small
  // screen — preview region grows toward the full window width.
  EXPECT_GE(collapsed.achieved_preview_ratio, expanded.achieved_preview_ratio);
}

// Pathological case: chrome eats 100% of available height → helper must not
// divide by zero / NaN; was_clamped should stay false (we have no signal).
TEST(AspectFitTest, ChromeExceedsHeightYieldsBenignDefault) {
  AspectFitResult fit =
      ResolveAspectFit(/*current_win_w=*/1280, /*ratio=*/2.0f,
                       /*work_w=*/1280, /*work_h=*/kMinWindowHeight, kLeftPanelWidth, kRightPanelWidth,
                       /*topbar_h=*/kMinWindowHeight,
                       /*statusbar_h=*/0.0f);
  // Whatever the achieved ratio is, was_clamped must be deterministic (not
  // NaN-driven). The function falls back to "achieved == requested" on the
  // pathological branch.
  EXPECT_FALSE(fit.was_clamped);
  EXPECT_FLOAT_EQ(fit.achieved_preview_ratio, fit.requested_preview_ratio);
}

}  // namespace
