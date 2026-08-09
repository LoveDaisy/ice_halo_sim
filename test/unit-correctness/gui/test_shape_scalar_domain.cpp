#include <gtest/gtest.h>

#include <string>

#include "gui/shape_scalar_domain.hpp"

namespace lumice::gui {
namespace {

// The shape-scalar domain table. Before it was data, these numbers were written out at each of the
// ten RenderShapeDistTableRow call sites and described in a comment nothing could check, so the
// only way to ask "what does Prism H allow" was to open the crystal modal and drag its slider.
//
// The two assertions worth making about a table are total ones: every slot has a row, and the row
// each slot has is the one it had before the move.

// The values as they stood at the call sites in edit_modals.cpp before extraction. This is the
// "truth table is identical either side of the refactor" check, spelled out independently rather
// than by re-reading the table under test.
struct ExpectedRow {
  int slot;
  const char* name;
  float min_value;
  float max_value;
  const char* fmt;
  SliderScale scale;
};

constexpr ExpectedRow kExpected[] = {
  { LUMICE_SHAPE_SCALAR_HEIGHT, "Height", 0.01f, 100.0f, "%.2f", SliderScale::kLog },
  { LUMICE_SHAPE_SCALAR_UPPER_H, "Upper H", 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_PRISM_H, "Prism H", 0.0f, 100.0f, "%.4f", SliderScale::kLogLinear },
  { LUMICE_SHAPE_SCALAR_LOWER_H, "Lower H", 0.0f, 1.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 0, "Face 0", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 1, "Face 1", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 2, "Face 2", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 3, "Face 3", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 4, "Face 4", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
  { LUMICE_SHAPE_SCALAR_FACE_0 + 5, "Face 5", 0.0f, 2.0f, "%.3f", SliderScale::kLinear },
};
static_assert(sizeof(kExpected) / sizeof(kExpected[0]) == LUMICE_SHAPE_SCALAR_COUNT,
              "the expectation table must cover every shape scalar slot");

TEST(ShapeScalarDomain, EverySlotKeepsTheDomainItsCallSiteUsedToSpell) {
  for (const ExpectedRow& row : kExpected) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(row.slot);
    EXPECT_FLOAT_EQ(d.min_value, row.min_value) << row.name;
    EXPECT_FLOAT_EQ(d.max_value, row.max_value) << row.name;
    EXPECT_EQ(std::string(d.fmt), std::string(row.fmt)) << row.name;
    EXPECT_EQ(d.scale, row.scale) << row.name;
  }
}

TEST(ShapeScalarDomain, EverySlotHasANonEmptyDomain) {
  // Total over the enum rather than over the rows the crystal modal happens to draw: a slot added
  // to lumice.h without a domain here would otherwise stay invisible until someone opened the tab
  // that renders it.
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(slot);
    EXPECT_LT(d.min_value, d.max_value) << "slot " << slot;
    EXPECT_GE(d.min_value, 0.0f) << "slot " << slot;
    ASSERT_NE(d.fmt, nullptr) << "slot " << slot;
    EXPECT_EQ(d.fmt[0], '%') << "slot " << slot;
  }
}

TEST(ShapeScalarDomain, TheSixFaceSlotsShareOneDomain) {
  // Shared on purpose today (one kFaceSpreadMax for all six). Stored per-slot rather than as a
  // range test, so giving one face its own band later is a data edit — this assertion is what
  // would then have to be updated deliberately instead of the behavior changing silently.
  const ShapeScalarDomain& first = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_FACE_0);
  for (int i = 1; i < 6; ++i) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_FACE_0 + i);
    EXPECT_FLOAT_EQ(d.min_value, first.min_value) << "face " << i;
    EXPECT_FLOAT_EQ(d.max_value, first.max_value) << "face " << i;
    EXPECT_EQ(std::string(d.fmt), std::string(first.fmt)) << "face " << i;
    EXPECT_EQ(d.scale, first.scale) << "face " << i;
  }
}

TEST(ShapeScalarDomain, LogScaledSlotsHaveAStrictlyPositiveLowerBound) {
  // A log mapping divides by min_value (slider_mapping.hpp::LogValueToNorm), so a kLog slot with
  // min 0 is not a tuning choice — it is a division by zero waiting for a user to reach it.
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(slot);
    if (d.scale == SliderScale::kLog) {
      EXPECT_GT(d.min_value, 0.0f) << "slot " << slot;
    }
  }
}

}  // namespace
}  // namespace lumice::gui
