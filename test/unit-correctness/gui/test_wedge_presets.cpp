// The wedge-angle preset dropdown must show the angle its label's Miller indices actually mean.
//
// It did not, for as long as the table existed. The four rows were transcribed by hand with the
// ratio inverted (l/h where the formula wants h/l), so {2,0,-2,1} offered 47.300° where the face
// is at 14.886°, {1,0,-1,2} offered 14.700° where it is at 46.756°, and {1,0,-1,0} offered 90.000°
// for a prism face that has no wedge angle at all. Picking one drew a crystal that was not the
// crystal the label named, and nothing anywhere disagreed.
//
// So this does not check four expected numbers — a second hand-transcription is how the first one
// would be re-blessed. It reads the table the dropdown itself renders (GetWedgePresets, the same
// call RenderWedgeTableRow makes) and puts every row back through
// LUMICE_ConvertMillerIndexToWedgeAngle, the single owner of what Miller indices mean. Add a row
// with the wrong indices and this goes red without anyone updating the test.
//
// One unit's pure logic, no frame and no input event, so it lives here rather than in gui_test —
// which matters twice over, because CI runs gui_test's `modal_layout` / `defaults_panel_layout`
// groups and nothing else, and would never have evaluated it there.

#include <gtest/gtest.h>

#include <cstdio>

#include "gui/edit_modals.hpp"
#include "include/lumice.h"

namespace gui = lumice::gui;

namespace {

TEST(WedgePresets, EveryPresetAngleIsWhatItsMillerIndicesMean) {
  int count = 0;
  const gui::WedgePreset* presets = gui::GetWedgePresets(&count);
  ASSERT_NE(presets, nullptr);
  ASSERT_GT(count, 0);

  for (int i = 0; i < count; ++i) {
    const gui::WedgePreset& p = presets[i];
    SCOPED_TRACE(testing::Message() << "preset[" << i << "] label=" << p.label << " h=" << p.h << " l=" << p.l);

    LUMICE_MillerConversionState state = LUMICE_MILLER_INVALID;
    float angle = 0.0f;
    // Non-fatal, then `continue`: a fatal assert would return out of the loop and hide every row
    // after the first bad one, which is the census this test exists to take.
    const LUMICE_ErrorCode err = LUMICE_ConvertMillerIndexToWedgeAngle(p.h, 0, p.l, 3, &state, &angle, nullptr);
    if (err != LUMICE_OK) {
      ADD_FAILURE() << "conversion call failed with error code " << err;
      continue;
    }

    // A preset the owner will not build is one the dropdown must not offer: picking it would set a
    // wedge angle the mesh builder then silently drops.
    EXPECT_EQ(state, LUMICE_MILLER_VALID);
    EXPECT_FLOAT_EQ(p.value, angle);

    // The label carries the angle too, so it is a second place the number can be wrong. Today both
    // come from one call and this is redundant; it is here for the refactor that separates them.
    char expected_label[sizeof(p.label)];
    std::snprintf(expected_label, sizeof(expected_label), "{%d,0,%d,%d} %.3f\xc2\xb0", p.h, -p.h, p.l,
                  static_cast<double>(angle));
    EXPECT_STREQ(p.label, expected_label);
  }
}

}  // namespace
