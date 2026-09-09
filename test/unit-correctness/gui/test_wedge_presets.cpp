// TEMPORARY red-state probe (AC0) — this file is reverted immediately after the red is recorded,
// and Step 4 writes a different test at the same path against the derived table.
//
// The proposition: every angle the wedge-preset dropdown shows equals what
// LUMICE_ConvertMillerIndexToWedgeAngle answers for the Miller indices that row's label names.
// Today three of the four rows were transcribed with the ratio inverted (l/h instead of h/l), so
// this must fail on three rows before the table is changed — otherwise "the constants are right"
// would rest on nothing but a second hand-transcription.

#include <gtest/gtest.h>

#include "gui/edit_modals.hpp"
#include "include/lumice.h"

namespace gui = lumice::gui;

namespace {

TEST(WedgePresetsLegacyProbe, EveryPresetValueMatchesTheOwner) {
  const gui::WedgePresetLegacyRow* rows = nullptr;
  const int count = gui::GetWedgePresetLegacyValuesForTest(&rows);
  ASSERT_NE(rows, nullptr);
  ASSERT_EQ(count, 4);

  for (int i = 0; i < count; ++i) {
    SCOPED_TRACE(testing::Message() << "preset[" << i << "] label=" << rows[i].label << " h=" << rows[i].h
                                    << " l=" << rows[i].l);
    LUMICE_MillerConversionState state = LUMICE_MILLER_INVALID;
    float angle = 0.0f;
    // Non-fatal, then `continue`: a fatal assert here would return out of the whole loop and hide
    // every row after the first bad one — which is precisely the census this probe exists to take.
    const LUMICE_ErrorCode err =
        LUMICE_ConvertMillerIndexToWedgeAngle(rows[i].h, 0, rows[i].l, 3, &state, &angle, nullptr);
    if (err != LUMICE_OK) {
      ADD_FAILURE() << "conversion call failed with error code " << err;
      continue;
    }
    EXPECT_EQ(state, LUMICE_MILLER_VALID);
    EXPECT_FLOAT_EQ(rows[i].value, angle);
  }
}

}  // namespace
