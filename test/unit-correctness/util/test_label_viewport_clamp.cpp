// The shared overlay-label viewport clamp, asserted on the function itself.
//
// This file exists because the rule now has TWO call sites — the GUI's draw-list pass and the
// CLI/server compositor — and neither of them is a good place to pin the arithmetic: one needs an
// ImGui font atlas to produce a text size, the other needs a rendered frame. What is left when
// both are stripped away is eight numbers in and two out, which is what is asserted here.
//
// The case table is deliberately the same one as
// test/unit-correctness/gui/test_overlay_labels.cpp's ALabelIsPulledBackInsideTheViewportOnEveryEdge,
// numbers included. That is not duplication for its own sake: the GUI case goes through
// gui::detail::ClampLabelPosToViewport (the ImVec2 wrapper) and this one through
// lumice::ClampLabelPosToViewport (the shared rule), so the pair says the wrapper is a wrapper. If
// the two ever disagree, the unpacking layer has grown a decision of its own — which is exactly the
// GUI/CLI divergence this shared header was written to make impossible.

#include <gtest/gtest.h>

#include "util/label_viewport_clamp.hpp"

namespace lumice {
namespace {

// A 30x14 glyph run in a 200x100 viewport pinned at (10, 20) — an offset origin rather than (0, 0)
// so a rule that confused "viewport coordinate" with "distance from the left edge" fails here
// instead of passing on a canvas that happens to start at zero. The CLI's own viewport does start
// at (0, 0), so this is the axis its call site cannot test.
constexpr float kTextW = 30.0f;
constexpr float kTextH = 14.0f;
constexpr LabelViewport kVp{ 10.0f, 20.0f, 200.0f, 100.0f };

TEST(LabelViewportClamp, ALabelIsPulledBackInsideTheViewportOnEveryEdge) {
  struct Case {
    const char* name;
    LabelPos pos;
    float vp_w;
    LabelPos expected;
  };
  const Case kCases[] = {
    // x=5 would start left of the viewport at x=10; the inset puts it at 12.
    { "past the left edge", { 5.0f, 50.0f }, kVp.w, { 12.0f, 50.0f } },
    // x=195 plus 30 of text ends at 225, past the right edge at 210; 210 - 30 - 2 = 178.
    { "past the right edge", { 195.0f, 50.0f }, kVp.w, { 178.0f, 50.0f } },
    { "above the top edge", { 50.0f, 15.0f }, kVp.w, { 50.0f, 22.0f } },
    // 20 + 100 - 14 - 2 = 104.
    { "below the bottom edge", { 50.0f, 110.0f }, kVp.w, { 50.0f, 104.0f } },
    { "already inside", { 50.0f, 50.0f }, kVp.w, { 50.0f, 50.0f } },
    // 20 wide cannot hold 30 of text plus two insets, so x is left alone; y is processed normally
    // and happens to need no adjustment.
    { "a viewport too narrow for the text", { 5.0f, 50.0f }, 20.0f, { 5.0f, 50.0f } },
  };

  for (const Case& c : kCases) {
    const LabelViewport vp{ kVp.x, kVp.y, c.vp_w, kVp.h };
    const LabelPos clamped = ClampLabelPosToViewport(c.pos, LabelSize{ kTextW, kTextH }, vp);
    // Non-fatal: this loop is a table of independent propositions, and one bad edge must not hide
    // the state of the other five (doc/testing-architecture.md §4.9).
    EXPECT_EQ(clamped.x, c.expected.x) << c.name;
    EXPECT_EQ(clamped.y, c.expected.y) << c.name;
  }
}

// The two axes are independent. A label past the left edge AND below the bottom moves on both; the
// cases above only ever needed one axis at a time, so a rule that returned early after the first
// adjustment would pass every one of them.
TEST(LabelViewportClamp, BothAxesMoveWhenBothAreOutside) {
  const LabelPos clamped = ClampLabelPosToViewport(LabelPos{ 5.0f, 110.0f }, LabelSize{ kTextW, kTextH }, kVp);
  EXPECT_EQ(clamped.x, 12.0f);
  EXPECT_EQ(clamped.y, 104.0f);
}

// The property the CLI compositor actually depends on: after clamping, the whole glyph rect is
// inside the canvas, so no row or column of it can fall off the edge and be dropped. Asserted over
// a sweep rather than at hand-picked points, because the call site's anchors come from a curve walk
// and land anywhere.
TEST(LabelViewportClamp, TheWholeRectEndsUpInsideACanvasThatCanHoldIt) {
  constexpr LabelViewport kCanvas{ 0.0f, 0.0f, 128.0f, 96.0f };
  const LabelSize size{ 21.0f, 15.0f };  // about what a two-glyph label rasterizes to at 15 px

  // The sweep reports its FIRST offending anchor and nothing else: tens of thousands of rows would
  // all fail for one reason, and a single located counterexample is the whole diagnostic. Recorded
  // here and asserted after the loop, so no report is made from inside it.
  int offenders = 0;
  LabelPos first_in{ 0.0f, 0.0f };
  LabelPos first_out{ 0.0f, 0.0f };
  for (float y = -30.0f; y <= 130.0f; y += 1.0f) {
    for (float x = -30.0f; x <= 160.0f; x += 1.0f) {
      const LabelPos p = ClampLabelPosToViewport(LabelPos{ x, y }, size, kCanvas);
      const bool inside = p.x >= kCanvas.x && p.y >= kCanvas.y && p.x + size.w <= kCanvas.x + kCanvas.w &&
                          p.y + size.h <= kCanvas.y + kCanvas.h;
      if (inside) {
        continue;
      }
      if (offenders == 0) {
        first_in = LabelPos{ x, y };
        first_out = p;
      }
      ++offenders;
    }
  }
  EXPECT_EQ(offenders, 0) << "first of " << offenders << ": anchor (" << first_in.x << ", " << first_in.y
                          << ") clamped to (" << first_out.x << ", " << first_out.y << "), whose " << size.w << "x"
                          << size.h << " rect leaves the " << kCanvas.w << "x" << kCanvas.h << " canvas";
}

// A label already comfortably inside is returned bit-identical. This is what makes the CLI change
// affect the rim and nothing else — every existing render whose labels were clear of the edges must
// composite the same bytes it did before the clamp existed.
TEST(LabelViewportClamp, AnInteriorLabelIsUntouched) {
  // Same shape as the sweep above, and for the same reason: locate the first anchor the clamp moved
  // and report that one, rather than every anchor in the interior.
  int moved = 0;
  LabelPos first_in{ 0.0f, 0.0f };
  LabelPos first_out{ 0.0f, 0.0f };
  for (float y = 40.0f; y <= 60.0f; y += 0.5f) {
    for (float x = 40.0f; x <= 60.0f; x += 0.5f) {
      const LabelPos p = ClampLabelPosToViewport(LabelPos{ x, y }, LabelSize{ kTextW, kTextH }, kVp);
      if (p.x == x && p.y == y) {
        continue;
      }
      if (moved == 0) {
        first_in = LabelPos{ x, y };
        first_out = p;
      }
      ++moved;
    }
  }
  EXPECT_EQ(moved, 0) << "first of " << moved << ": an interior anchor (" << first_in.x << ", " << first_in.y
                      << ") was moved to (" << first_out.x << ", " << first_out.y << ")";
}

}  // namespace
}  // namespace lumice
