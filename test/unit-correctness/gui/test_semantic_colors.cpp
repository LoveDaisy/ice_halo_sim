// The GUI's three content-semantic colour grades (good / warning / destructive).
//
// What is worth pinning here is not the RGB digits — those are a design choice and are expected to
// move when the palette is retuned. What must survive a retune is the set of properties every call
// site silently relies on:
//   - the three grades stay telling apart, in both consumption forms;
//   - each grade's muted fill really is muted relative to its bright text form, since one is used as
//     a filled input background behind 15px text and the other as the text itself;
//   - the alpha a caller passes to *FillColor reaches the result unchanged and does not disturb RGB
//     (call sites deliberately pass different alphas: 0.5 for a validation cell, 0.6 for the
//     Resolution input);
//   - each grade still sits in the colour family its name promises. A palette edit that left
//     "warning" bluer than it is red would keep every equality test above green while making the
//     Resolution input read as an accent rather than a warning.
//
// The Push*/Pop* button helpers are not covered here: they call ImGui::PushStyleColor, which needs a
// live ImGui context this target deliberately does not create. Their pairing is exercised by the
// rendered-frame suites.

#include <gtest/gtest.h>

#include "gui/semantic_colors.hpp"
#include "imgui.h"

namespace {

using lumice::gui::DestructiveFillColor;
using lumice::gui::DestructiveTextColor;
using lumice::gui::GoodFillColor;
using lumice::gui::GoodTextColor;
using lumice::gui::WarningFillColor;
using lumice::gui::WarningTextColor;

// Rec. 601 luma, the same weighting panels.cpp uses to pick a contrasting label colour.
float Luma(const ImVec4& c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

bool SameRgb(const ImVec4& a, const ImVec4& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

TEST(SemanticColors, TextGradesAreMutuallyDistinct) {
  EXPECT_FALSE(SameRgb(GoodTextColor(), WarningTextColor()));
  EXPECT_FALSE(SameRgb(GoodTextColor(), DestructiveTextColor()));
  EXPECT_FALSE(SameRgb(WarningTextColor(), DestructiveTextColor()));
}

TEST(SemanticColors, FillGradesAreMutuallyDistinct) {
  const ImVec4 good = GoodFillColor(0.5f);
  const ImVec4 warn = WarningFillColor(0.5f);
  const ImVec4 destr = DestructiveFillColor(0.5f);
  EXPECT_FALSE(SameRgb(good, warn));
  EXPECT_FALSE(SameRgb(good, destr));
  EXPECT_FALSE(SameRgb(warn, destr));
}

TEST(SemanticColors, TextFormsAreOpaque) {
  EXPECT_FLOAT_EQ(GoodTextColor().w, 1.0f);
  EXPECT_FLOAT_EQ(WarningTextColor().w, 1.0f);
  EXPECT_FLOAT_EQ(DestructiveTextColor().w, 1.0f);
}

TEST(SemanticColors, FillAlphaIsCallerSuppliedAndDoesNotDisturbRgb) {
  for (const float alpha : { 0.0f, 0.25f, 0.5f, 0.6f, 1.0f }) {
    EXPECT_FLOAT_EQ(GoodFillColor(alpha).w, alpha);
    EXPECT_FLOAT_EQ(WarningFillColor(alpha).w, alpha);
    EXPECT_FLOAT_EQ(DestructiveFillColor(alpha).w, alpha);

    EXPECT_TRUE(SameRgb(GoodFillColor(alpha), GoodFillColor(1.0f)));
    EXPECT_TRUE(SameRgb(WarningFillColor(alpha), WarningFillColor(1.0f)));
    EXPECT_TRUE(SameRgb(DestructiveFillColor(alpha), DestructiveFillColor(1.0f)));
  }
}

TEST(SemanticColors, FillFormIsMutedRelativeToTextForm) {
  // A background tint sitting behind body text has to be the darker of the pair, or the text on top
  // of it stops being readable — this is the property that makes two forms per grade necessary at
  // all, rather than one colour reused everywhere.
  EXPECT_LT(Luma(GoodFillColor(1.0f)), Luma(GoodTextColor()));
  EXPECT_LT(Luma(WarningFillColor(1.0f)), Luma(WarningTextColor()));
  EXPECT_LT(Luma(DestructiveFillColor(1.0f)), Luma(DestructiveTextColor()));
}

TEST(SemanticColors, EachGradeStaysInItsColourFamily) {
  // good: green leads.
  for (const ImVec4& c : { GoodTextColor(), GoodFillColor(1.0f) }) {
    EXPECT_GT(c.y, c.x);
    EXPECT_GT(c.y, c.z);
  }
  // warning: amber — red leads, green follows well ahead of blue.
  for (const ImVec4& c : { WarningTextColor(), WarningFillColor(1.0f) }) {
    EXPECT_GE(c.x, c.y);
    EXPECT_GT(c.y, c.z);
  }
  // destructive: red leads, and the two remaining channels stay level so it does not drift into
  // amber (which is warning's family) or magenta.
  for (const ImVec4& c : { DestructiveTextColor(), DestructiveFillColor(1.0f) }) {
    EXPECT_GT(c.x, c.y);
    EXPECT_GT(c.x, c.z);
    EXPECT_FLOAT_EQ(c.y, c.z);
  }
}

}  // namespace
