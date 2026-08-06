// The Visibility group's enablement rule, as a rule rather than as a grid of examples.
//
// `renderer.visible` (the Upper/Full/Lower radio group) and `renderer.front` (the Front checkbox)
// are gated in the field-editor registry by NotUnderFullSky and NotUnderFullSkyOrGlobe. Before
// task 431.1 the main UI did not read either: app_panels.cpp spelled the same two conditions out
// again as a hand-paired `BeginDisabled` nest, and the only thing pinning the pair was nine
// near-identical gui_test cases, one per (lens, expectation) cell somebody thought to write down.
//
// This case is the WHAT — the semantics of the two registry gates over every lens type. Its
// sibling `p2_render/visibility_enablement_matrix` in test/gui/functional/test_gui_interaction.cpp
// is the WHERE — that the widgets app_panels.cpp actually draws carry exactly these values. The
// split is the link boundary AGENTS.md draws: the registry gate is pure logic and evaluates with
// no ImGui context, so it runs here and therefore on all three CI platforms; "is the real widget
// greyed" needs a live frame and can only run where gui_test does.
//
// Scope of the claim, stated so it is not read as wider than it is: LensIsFullSky is the repo's
// single source of truth for the full-sky set (gui_constants.hpp), and the expectation below calls
// it, exactly as the retired inline code did. So this pins WHICH gate each field carries and what
// that gate computes — not the membership of kFullSkyLensTypes, which is a policy statement
// guarded by its own static_assert rather than an invariant anything can derive.

#include <gtest/gtest.h>

#include "gui/field_editor_registry.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"

// AC2/AC4 — universally quantified over kLensTypeCount x {visible, front}, so a lens added to the
// enum is covered the moment it exists, with no cell to remember to write. The expectation is
// re-derived from LensIsFullSky / kLensTypeGlobe rather than read back out of ConstraintFor: a
// predicate compared against itself agrees with itself no matter what it computes.
//
// This also carries AC4's dynamic half. The retired inline gate was
//   radios disabled  <=>  full_sky
//   Front  disabled  <=>  full_sky || (!full_sky && is_globe)  ==  full_sky || is_globe
// which is what the two EXPECT_EQ lines below assert of the registry, cell for cell.
TEST(VisibilityEnablement, inline_ac2_registry_gate_matches_semantics_for_every_lens) {
  for (int lens = 0; lens < lumice::gui::kLensTypeCount; ++lens) {
    lumice::gui::GuiState state;
    state.renderer.lens_type = lens;

    const bool full_sky = lumice::gui::LensIsFullSky(lens);
    const bool is_globe = (lens == lumice::gui::kLensTypeGlobe);

    const lumice::gui::FieldEditorConstraint visible_c = lumice::gui::ConstraintFor("renderer.visible", state);
    const lumice::gui::FieldEditorConstraint front_c = lumice::gui::ConstraintFor("renderer.front", state);

    EXPECT_EQ(visible_c.enabled, !full_sky) << "renderer.visible, lens=" << lens;
    EXPECT_EQ(front_c.enabled, !(full_sky || is_globe)) << "renderer.front, lens=" << lens;
  }
}

// The set the loop above quantifies over must be the whole enum, not a prefix of it. kLensTypeCount
// is asserted against kLensTypeNames and kLensTypePresentationOrder in gui_state.hpp; what is NOT
// asserted anywhere else is that the terminal enumerator is reachable from a `< kLensTypeCount`
// loop, which is the one property this case's coverage rests on.
TEST(VisibilityEnablement, inline_ac2_lens_loop_bound_reaches_every_enumerator) {
  EXPECT_LT(lumice::gui::kLensTypeGlobe, lumice::gui::kLensTypeCount);
  EXPECT_EQ(lumice::gui::kLensTypeLinear, 0);
}
