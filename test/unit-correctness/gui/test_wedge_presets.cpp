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
#include <string>
#include <vector>

#include "gui/edit_modals.hpp"
#include "gui/user_defaults.hpp"
#include "include/lumice.h"

namespace gui = lumice::gui;

namespace {

TEST(WedgePresets, EveryPresetAngleIsWhatItsMillerIndicesMean) {
  // No user shortcuts seeded: this case is about the built-in table. The reset is not decoration —
  // gui_unit_test runs every case in one process, so a list left behind by the cases below would
  // otherwise be part of what this one measures.
  gui::ResetUserWedgePresets();

  const std::vector<gui::WedgePreset> presets = gui::GetWedgePresets();
  ASSERT_FALSE(presets.empty());

  for (size_t i = 0; i < presets.size(); ++i) {
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

// The dropdown offers the user's saved shortcuts too (523.5). Two propositions, and neither is
// about the four built-ins:
//   - a saved triple reaches the list, with the label and angle its indices mean;
//   - the built-ins are still all there, because the user's list APPENDS rather than replaces. That
//     is what lets a future correction to a factory value follow through to everyone, and what
//     stops a user emptying the dropdown into an unusable state.
TEST(WedgePresets, ASavedPresetIsAppendedToTheBuiltInsRatherThanReplacingThem) {
  gui::ResetUserWedgePresets();
  const size_t built_in_count = gui::GetWedgePresets().size();
  ASSERT_GT(built_in_count, 0u);

  // {3,0,-3,2} is not one of the four; it converts, so the merge has no excuse to drop it.
  gui::AdoptWedgePresetOverridesInMemory({ { 3, 0, 2 } });
  const std::vector<gui::WedgePreset> merged = gui::GetWedgePresets();
  ASSERT_EQ(merged.size(), built_in_count + 1);

  const gui::WedgePreset& added = merged.back();
  EXPECT_EQ(added.h, 3);
  EXPECT_EQ(added.l, 2);

  LUMICE_MillerConversionState state = LUMICE_MILLER_INVALID;
  float angle = 0.0f;
  ASSERT_EQ(LUMICE_ConvertMillerIndexToWedgeAngle(3, 0, 2, 3, &state, &angle, nullptr), LUMICE_OK);
  ASSERT_EQ(state, LUMICE_MILLER_VALID);
  EXPECT_FLOAT_EQ(added.value, angle);
  EXPECT_EQ(std::string(added.label), gui::FormatWedgePresetLabel(3, 2, angle));

  gui::ResetUserWedgePresets();
  EXPECT_EQ(gui::GetWedgePresets().size(), built_in_count);
}

// De-duplication, against the built-ins and within the user's own list. A file can hold either kind
// of repeat (hand-edited, or written by a build whose factory table has since grown a row the user
// had already saved), and showing one twice is a list the user cannot tell apart.
TEST(WedgePresets, RepeatedIndicesAreListedOnce) {
  gui::ResetUserWedgePresets();
  const size_t built_in_count = gui::GetWedgePresets().size();
  ASSERT_GT(built_in_count, 0u);

  // {1,0,-1,1} is the first built-in; that is the point of picking it.
  ASSERT_TRUE(gui::IsBuiltInWedgeMillerIndex(1, 0, 1));
  gui::AdoptWedgePresetOverridesInMemory({ { 1, 0, 1 }, { 3, 0, 2 }, { 3, 0, 2 } });

  const std::vector<gui::WedgePreset> merged = gui::GetWedgePresets();
  EXPECT_EQ(merged.size(), built_in_count + 1);

  int matches = 0;
  for (const gui::WedgePreset& row : merged) {
    if (row.h == 1 && row.l == 1) {
      ++matches;
    }
  }
  EXPECT_EQ(matches, 1);

  gui::ResetUserWedgePresets();
}

// An unbuildable triple is skipped rather than asserted on: it can only come from a file a user can
// edit. The load path already drops such rows with a notice, so this is the belt to that braces —
// and the difference from the built-in loop, which DOES assert, is deliberate.
TEST(WedgePresets, AnUnbuildableSavedTripleIsSkippedRatherThanOffered) {
  gui::ResetUserWedgePresets();
  const size_t built_in_count = gui::GetWedgePresets().size();

  gui::AdoptWedgePresetOverridesInMemory({ { 1, 1, 1 }, { 0, 0, 1 } });
  EXPECT_EQ(gui::GetWedgePresets().size(), built_in_count);

  gui::ResetUserWedgePresets();
}
