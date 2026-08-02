// The color-window cases that never need a live ImGui context.
//
// test_gui_color_window.cpp splits in two: 39 cases exercise z-order, visibility and predicate
// logic by calling production functions and asserting on GuiState, and 14 need a real frame —
// 9 drive the window through ImGuiTestContext (clicking the eye icon, toggling symmetry
// checkboxes, reading control-disabled state and icon glyphs), and 4 more call
// gui::RefreshColorClassSignals, whose body reaches ImGui::GetTime() (color_window.cpp). That
// last group is the interesting one: those four test bodies contain no ImGui call of their own,
// so a criterion that only greps the TEST body for `ImGui::` passes them as pure logic — and
// without a context the result is a segfault, not a failed assertion. A production function's
// own context requirement is part of the case's requirement.
//
// Two mechanical notes on the translation from the IM_* form:
//   * IM_CHECK / IM_CHECK_EQ RETURN from the test function when they fail
//     (imgui_te_context.h: `if (!res) return;`, and IM_CHECK_OP's `_RETURN` argument is `true`
//     for every non-`_NO_RET` form). ASSERT_*, not EXPECT_*, is therefore the mapping that
//     preserves what these cases did before the move.
//   * ResetTestState() becomes gui::DoNew() plus ResetColorClassSignalCacheForTest(). The second
//     half is not decoration: the signal cache is a file-scope static in color_window.cpp keyed
//     on (server, epoch), and gui_test got it reset between cases for free. Dropping it here
//     would let one case's keys leak into the next.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "gui/raypath_segments.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

// SwapZOrder swaps only the z_order scalars; vector entries stay pinned
// to their physical index so the C-API lane binding (GetColorClassLaneY(i))
// keeps pointing at the same class.
TEST(ColorWindow, swap_zorder_does_not_reorder_vector) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig a;
  a.color[0] = 1.0f;
  a.color[1] = 0.0f;
  a.color[2] = 0.0f;
  a.z_order = 0;
  gui::ColorClassConfig b;
  b.color[0] = 0.0f;
  b.color[1] = 1.0f;
  b.color[2] = 0.0f;
  b.z_order = 1;
  gui::g_state.raypath_color.push_back(a);
  gui::g_state.raypath_color.push_back(b);

  gui::SwapZOrder(gui::g_state, 0, 1);

  // Vector index 0 must still be the red class; only z_order was flipped.
  ASSERT_EQ(gui::g_state.raypath_color[0].color[0], 1.0f);
  ASSERT_EQ(gui::g_state.raypath_color[1].color[1], 1.0f);
  ASSERT_EQ(gui::g_state.raypath_color[0].z_order, 1);
  ASSERT_EQ(gui::g_state.raypath_color[1].z_order, 0);
}

// Out-of-range and self-swap are no-ops (defensive: the caller is UI code
// that iterates on user input; a bad index should not corrupt state).
TEST(ColorWindow, swap_zorder_ignores_bad_indices) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig c;
  c.z_order = 7;
  gui::g_state.raypath_color.push_back(c);

  gui::SwapZOrder(gui::g_state, 0, 0);     // self-swap
  gui::SwapZOrder(gui::g_state, 0, 42);    // out-of-range
  gui::SwapZOrder(gui::g_state, 99, 100);  // both out-of-range
  ASSERT_EQ(gui::g_state.raypath_color[0].z_order, 7);
}

// CompactZOrder rebuilds z_order to [0, N) preserving the user's visible
// order. Called after delete-class which leaves a hole (e.g. deleting the
// middle class in [z=0, z=1, z=2] leaves [z=0, z=2] which is not a
// permutation LUMICE_SetRaypathColors will accept).
TEST(ColorWindow, compact_zorder_fills_gaps) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  auto make_cls = [](int z) {
    gui::ColorClassConfig c;
    c.z_order = z;
    return c;
  };
  gui::g_state.raypath_color.push_back(make_cls(5));  // physical 0
  gui::g_state.raypath_color.push_back(make_cls(2));  // physical 1
  gui::g_state.raypath_color.push_back(make_cls(9));  // physical 2

  gui::CompactZOrder(gui::g_state);

  // The visible order (by ascending z) was phys=1, phys=0, phys=2 —
  // compaction assigns 0, 1, 2 in that same rank order.
  ASSERT_EQ(gui::g_state.raypath_color[1].z_order, 0);
  ASSERT_EQ(gui::g_state.raypath_color[0].z_order, 1);
  ASSERT_EQ(gui::g_state.raypath_color[2].z_order, 2);
}

// Delete-in-the-middle simulates the RenderColorWindow post-erase step:
// erase(1) removes the middle physical class; CompactZOrder then packs the
// remaining z_order values into [0, N).
TEST(ColorWindow, compact_zorder_after_delete_middle) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  auto make_cls = [](int z) {
    gui::ColorClassConfig c;
    c.z_order = z;
    return c;
  };
  gui::g_state.raypath_color.push_back(make_cls(0));
  gui::g_state.raypath_color.push_back(make_cls(1));
  gui::g_state.raypath_color.push_back(make_cls(2));

  // Simulate delete of physical index 1 (per RenderColorWindow pending_delete).
  gui::g_state.raypath_color.erase(gui::g_state.raypath_color.begin() + 1);
  gui::CompactZOrder(gui::g_state);

  ASSERT_EQ(static_cast<int>(gui::g_state.raypath_color.size()), 2);
  ASSERT_EQ(gui::g_state.raypath_color[0].z_order, 0);
  ASSERT_EQ(gui::g_state.raypath_color[1].z_order, 1);
}

// ValidateSingleAtomText — plan §3 decision 3 gate. Empty is treated as
// whole-crystal (valid); a single Factor with a single alternative is valid;
// multi-factor "A & B" AND-inside-atom is rejected; multi-alternative "A;B"
// is rejected (a LUMICE_ColorPredicate is a single atom, cross-ref
// combine:all is the only AND path).
TEST(ColorWindow, validate_single_atom_accepts_empty) {
  auto v = gui::ValidateSingleAtomText("");
  ASSERT_EQ(v.state, LUMICE_RAYPATH_VALID);
}

TEST(ColorWindow, validate_single_atom_accepts_single_raypath) {
  auto v = gui::ValidateSingleAtomText("3-5-1");
  ASSERT_EQ(v.state, LUMICE_RAYPATH_VALID);
}

TEST(ColorWindow, validate_single_atom_rejects_multi_factor) {
  auto v = gui::ValidateSingleAtomText("3-5 & entry:2");
  ASSERT_EQ(v.state, LUMICE_RAYPATH_INVALID);
  ASSERT_TRUE(!v.message.empty());
}

TEST(ColorWindow, validate_single_atom_rejects_semicolon_or) {
  // "1-3;5-7" uses only legal PRISM face numbers (1-8), so the base
  // ValidateSummandText / face-range checks pass — the rejection here
  // must come from the alternative-count check (CountFactorAlternatives
  // == 2), NOT a coincidental face-number-out-of-range failure. This
  // exercises the same gate FillColorPredicate enforces at commit time
  // a ';' inside a single ref is a
  // summand-level OR-separator that expands to >1 alternative, which a
  // single-atom LUMICE_ColorPredicate cannot carry — cross-ref OR is
  // expressed with combine:any across refs instead.
  auto v = gui::ValidateSingleAtomText("1-3;5-7");
  ASSERT_EQ(v.state, LUMICE_RAYPATH_INVALID);
  ASSERT_TRUE(!v.message.empty());
}

// Per-ref symmetry read/write is a pure struct-field concern
// (writes land on ColorClassStructState via the reconciler's frame-tail diff).
// These direct-manipulation tests nail down the field independence + the
// whole-crystal freeze predicate without needing to drive ImGui.
TEST(ColorWindow, ref_symmetry_defaults_and_single_bits) {
  gui::ColorClassRefConfig ref;
  // Default: mirrors core RaypathColorRef::symmetry_ = kSymNone. Diverges from
  // FilterConfig deliberately (see raypath_color_config.hpp:32-38).
  ASSERT_TRUE(!ref.sym_p);
  ASSERT_TRUE(!ref.sym_b);
  ASSERT_TRUE(!ref.sym_d);

  ref.sym_p = true;
  ASSERT_TRUE(ref.sym_p);
  ASSERT_TRUE(!ref.sym_b);
  ASSERT_TRUE(!ref.sym_d);

  gui::ColorClassRefConfig ref_b;
  ref_b.sym_b = true;
  ASSERT_TRUE(!ref_b.sym_p);
  ASSERT_TRUE(ref_b.sym_b);
  ASSERT_TRUE(!ref_b.sym_d);

  gui::ColorClassRefConfig ref_d;
  ref_d.sym_d = true;
  ASSERT_TRUE(!ref_d.sym_p);
  ASSERT_TRUE(!ref_d.sym_b);
  ASSERT_TRUE(ref_d.sym_d);
}

TEST(ColorWindow, ref_symmetry_combination_bits_independent) {
  gui::ColorClassRefConfig ref;
  ref.sym_p = true;
  ref.sym_d = true;
  ASSERT_TRUE(ref.sym_p);
  ASSERT_TRUE(!ref.sym_b);
  ASSERT_TRUE(ref.sym_d);
  // Clearing one leaves the other untouched.
  ref.sym_p = false;
  ASSERT_TRUE(!ref.sym_p);
  ASSERT_TRUE(ref.sym_d);
}

// operator== must see all three new fields — otherwise the frame-tail
// reconciler (gui_state_reconcile.cpp RaypathColorStructChanged) would
// silently miss a symmetry edit and never trigger the re-sim path. This
// guard exercises each bit independently so a partial addition to
// operator== fails here rather than surfacing as a subtle re-sim miss.
TEST(ColorWindow, ref_symmetry_operator_eq_covers_all_bits) {
  gui::ColorClassRefConfig a;
  gui::ColorClassRefConfig b;
  ASSERT_TRUE(a == b);
  b.sym_p = true;
  ASSERT_TRUE(a != b);
  b = a;
  b.sym_b = true;
  ASSERT_TRUE(a != b);
  b = a;
  b.sym_d = true;
  ASSERT_TRUE(a != b);
}

// Whole-crystal freeze: a match_all ref matches every raypath through the
// placement, so per-ref P/B/D is a no-op. UI freezes the checkboxes rather
// than clearing them so a subsequent un-whole restores prior selections.
TEST(ColorWindow, ref_symmetry_editable_only_when_not_whole) {
  gui::ColorClassRefConfig ref;
  ref.match_all = true;
  ASSERT_TRUE(!gui::IsRefSymmetryEditable(ref));
  ref.match_all = false;
  ASSERT_TRUE(gui::IsRefSymmetryEditable(ref));
}

// BuildClassFromFilter — AC5 "Import from filter" backbone. Verifies:
//   (a) each single-factor SoP row lands as one ref with the same text
//       (empty → match_all = true; non-empty → predicate_text mirrors);
//   (b) multi-factor AND rows are skipped and counted in skipped_rows
//       (LUMICE_ColorPredicate is a single atom, no intra-atom AND);
//   (c) combine defaults to LUMICE_COLOR_COMBINE_ANY (rows OR'd together —
//       blueprint case B).
//   (d) the produced ColorClassConfig owns its match[] by value: mutating
//       the source filter's SoP after import does not touch the class.
TEST(ColorWindow, build_class_from_filter_single_atoms) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::FilterConfig f;
  f.name = "src_filter";
  // Fill f.param with three rows: two single-atom, one AND row.
  f.param.clear();
  f.param.push_back(
      gui::SummandText{ std::string{ "3-5" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });
  f.param.push_back(
      gui::SummandText{ std::string{ "1-2-3" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });
  // AND row: 2 factors — Build must skip it.
  f.param.push_back(gui::SummandText{
      std::string{ "5-7 & entry:2" },
      std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} }, gui::Factor{ gui::EntryExitParams{} } } });

  int skipped = -1;
  gui::ColorClassConfig cls = gui::BuildClassFromFilter(0, 7, f, skipped);

  ASSERT_EQ(skipped, 1);
  ASSERT_EQ(cls.combine, LUMICE_COLOR_COMBINE_ANY);
  ASSERT_EQ(static_cast<int>(cls.match.size()), 2);
  ASSERT_EQ(cls.match[0].layer_idx, 0);
  ASSERT_EQ(cls.match[0].crystal_pool_id, 7);
  ASSERT_TRUE(!cls.match[0].match_all);
  ASSERT_EQ(cls.match[0].predicate_text, "3-5");
  ASSERT_EQ(cls.match[1].predicate_text, "1-2-3");

  // Every ref built by BuildClassFromFilter
  // defaults to P|B|D symmetry (owner-preferred). Covers the loop-body
  // (multi-iteration) placement of the flag set — not once-outside-loop.
  ASSERT_TRUE(cls.match[0].sym_p);
  ASSERT_TRUE(cls.match[0].sym_b);
  ASSERT_TRUE(cls.match[0].sym_d);
  ASSERT_TRUE(cls.match[1].sym_p);
  ASSERT_TRUE(cls.match[1].sym_b);
  ASSERT_TRUE(cls.match[1].sym_d);

  // (d) AC5 decoupling: mutating the filter after import must not touch
  // the class. Change the source filter row text and verify the class's
  // ref text stays as it was at import time.
  f.param[0].text = "MUTATED";
  ASSERT_EQ(cls.match[0].predicate_text, "3-5");
}

// BuildClassFromFilter — empty text on a single-factor row means "the whole
// crystal" (mirrors the ref's match_all semantics), used when a filter row
// has no meaningful raypath text but was inserted as a placeholder.
TEST(ColorWindow, build_class_from_filter_empty_text_is_match_all) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::FilterConfig f;
  f.param.clear();
  f.param.push_back(gui::SummandText{ std::string{}, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });

  int skipped = -1;
  gui::ColorClassConfig cls = gui::BuildClassFromFilter(1, 3, f, skipped);
  ASSERT_EQ(skipped, 0);
  ASSERT_EQ(static_cast<int>(cls.match.size()), 1);
  ASSERT_TRUE(cls.match[0].match_all);
  ASSERT_EQ(cls.match[0].layer_idx, 1);
  ASSERT_EQ(cls.match[0].crystal_pool_id, 3);
}

// PollColorClassSignal resize semantics.
// Pre-fix: `assign(n, 0)` unconditionally zeroed every class on every call, so
// `state.raypath_color.push_back()` immediately made every pre-existing class
// read signal==0 → "no rays matched" warning on ALL classes for one frame.
// Post-fix: `resize(n, 1)` preserves existing signals and defaults new entries
// to 1 (settling / no warning). This test pins the resize invariant directly
// (no need for a real server, no C-API roundtrip, no throttle timing).
TEST(ColorWindow, poll_signal_resize_preserves_existing_and_defaults_new) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  // Seed g_state with 2 classes and a caller-owned buffer that already carries
  // "known matched" for both. The buffer stands in for the shared cache the
  // orchestration wrapper would hold across frames.
  gui::ColorClassConfig c;
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  std::vector<int> flags = { 1, 1 };

  // Simulate "user added a third class in this frame": grow raypath_color
  // to 3 WITHOUT a server commit. server=nullptr forces the early return
  // AFTER resize, so we exercise the resize path in isolation.
  gui::g_state.raypath_color.push_back(c);
  gui::PollColorClassSignal(gui::g_state, nullptr, flags);

  // Pre-fix: [0,0,0]. Post-fix: existing [0]/[1] preserved as 1; new [2] defaults to 1.
  ASSERT_EQ(static_cast<int>(flags.size()), 3);
  ASSERT_EQ(flags[0], 1);
  ASSERT_EQ(flags[1], 1);
  ASSERT_EQ(flags[2], 1);
}

// Shrink path: user deletes a class. resize(n, 1) still applies — extra entries
// beyond the new size are dropped; kept prefix retains its values.
TEST(ColorWindow, poll_signal_resize_shrinks_and_keeps_prefix) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig c;
  gui::g_state.raypath_color.push_back(c);
  std::vector<int> flags = { 1, 1, 0 };  // caller cache larger than state

  gui::PollColorClassSignal(gui::g_state, nullptr, flags);

  ASSERT_EQ(static_cast<int>(flags.size()), 1);
  ASSERT_EQ(flags[0], 1);
}

// Aggregate predicate for the top-bar warning. Same
// semantics as the per-row warning in RenderColorWindow: warn only when the
// user has configured at least one class with non-empty match[] AND every
// such class currently reports no signal.
TEST(ColorWindow, no_visible_matched_returns_false_when_empty_pool) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  std::vector<int> flags;
  ASSERT_TRUE(!gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

TEST(ColorWindow, no_visible_matched_returns_false_when_no_match_configured) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  // Two classes, both with empty match[] (user added them but hasn't wired refs yet).
  gui::ColorClassConfig c;
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  std::vector<int> flags = { 0, 0 };
  // No configured refs anywhere ⇒ nothing to warn about, top bar stays quiet.
  ASSERT_TRUE(!gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

TEST(ColorWindow, no_visible_matched_true_when_every_configured_class_is_zero) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig c;
  gui::ColorClassRefConfig r;
  c.match.push_back(r);
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  std::vector<int> flags = { 0, 0 };
  ASSERT_TRUE(gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

TEST(ColorWindow, no_visible_matched_false_when_any_configured_class_has_signal) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig c;
  gui::ColorClassRefConfig r;
  c.match.push_back(r);
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  std::vector<int> flags = { 0, 1 };
  ASSERT_TRUE(!gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

// Mixed pool: class[0] configured but silent, class[1] empty match. The
// top-bar warning should fire (class[0] alone is enough — class[1] adds
// nothing to warn about since it has no configured refs).
TEST(ColorWindow, no_visible_matched_ignores_empty_match_classes) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig configured;
  gui::ColorClassRefConfig r;
  configured.match.push_back(r);
  gui::ColorClassConfig empty_cls;
  gui::g_state.raypath_color.push_back(configured);
  gui::g_state.raypath_color.push_back(empty_cls);
  std::vector<int> flags = { 0, 0 };
  ASSERT_TRUE(gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

// A configured class whose index falls outside signal_flags (caller's cache
// hasn't grown to match state.raypath_color yet) must be treated as "unknown",
// not as a confirmed no-signal — matching
// PollColorClassSignal's own "we don't know yet" default (resize(n, 1)). The
// only configured class is out of range: pre-fix, `any_configured` was set
// before the bounds check and the missing entry could never disqualify the
// warning, so this would have returned true (false-positive warn). Post-fix
// it must return false — there is no known data, so nothing to warn about yet.
TEST(ColorWindow, no_visible_matched_treats_out_of_range_index_as_unknown) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig configured;
  gui::ColorClassRefConfig r;
  configured.match.push_back(r);
  gui::g_state.raypath_color.push_back(configured);
  std::vector<int> flags;  // empty: index 0 is out of range
  ASSERT_TRUE(!gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

// A genuinely known no-signal class must still trigger the warning even when a
// second, out-of-range (unknown) class is also configured — the fix must not
// over-correct into "any unknown entry suppresses the whole aggregate".
TEST(ColorWindow, no_visible_matched_true_when_known_entry_unmatched_despite_unknown_peer) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig configured;
  gui::ColorClassRefConfig r;
  configured.match.push_back(r);
  gui::g_state.raypath_color.push_back(configured);
  gui::g_state.raypath_color.push_back(configured);
  std::vector<int> flags = { 0 };  // index 0 known + unmatched; index 1 out of range (unknown)
  ASSERT_TRUE(gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

// SetRefMatchAll must NOT clear predicate_text.
// Root fix — pre-fix the whole checkbox's `.clear()` path wiped the field so
// un-checking whole left the row blank. AC2 machine gate.
TEST(ColorWindow, set_ref_match_all_true_preserves_predicate_text) {
  gui::ColorClassRefConfig ref;
  ref.predicate_text = "3-5";
  ref.match_all = false;
  gui::SetRefMatchAll(ref, true);
  ASSERT_TRUE(ref.match_all);
  ASSERT_EQ(ref.predicate_text, std::string("3-5"));
}

TEST(ColorWindow, set_ref_match_all_false_restores_editability_with_original_text) {
  gui::ColorClassRefConfig ref;
  ref.predicate_text = "3-5";
  ref.match_all = true;  // as if user had checked "whole"
  gui::SetRefMatchAll(ref, false);
  ASSERT_TRUE(!ref.match_all);
  ASSERT_EQ(ref.predicate_text, std::string("3-5"));
}

// HandleEyeClick — plain click only touches
// `visible`, Alt+click enforces exclusive solo. AC3 machine gate.
TEST(ColorWindow, handle_eye_click_plain_click_toggles_visible_only) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig a;
  a.visible = true;
  a.solo = false;
  gui::ColorClassConfig b = a;
  gui::g_state.raypath_color.push_back(a);
  gui::g_state.raypath_color.push_back(b);

  gui::HandleEyeClick(gui::g_state.raypath_color, 0, /*alt_down=*/false);

  ASSERT_TRUE(!gui::g_state.raypath_color[0].visible);
  ASSERT_TRUE(gui::g_state.raypath_color[1].visible);
  ASSERT_TRUE(!gui::g_state.raypath_color[0].solo);
  ASSERT_TRUE(!gui::g_state.raypath_color[1].solo);
}

TEST(ColorWindow, handle_eye_click_alt_sets_exclusive_solo) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  for (int i = 0; i < 3; i++) {
    gui::ColorClassConfig c;
    c.visible = true;
    c.solo = false;
    gui::g_state.raypath_color.push_back(c);
  }
  gui::HandleEyeClick(gui::g_state.raypath_color, 1, /*alt_down=*/true);

  ASSERT_TRUE(!gui::g_state.raypath_color[0].solo);
  ASSERT_TRUE(gui::g_state.raypath_color[1].solo);
  ASSERT_TRUE(!gui::g_state.raypath_color[2].solo);
}

TEST(ColorWindow, handle_eye_click_alt_second_click_clears_solo) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  for (int i = 0; i < 3; i++) {
    gui::ColorClassConfig c;
    c.visible = true;
    c.solo = false;
    gui::g_state.raypath_color.push_back(c);
  }
  // First Alt+click: idx=1 becomes solo.
  gui::HandleEyeClick(gui::g_state.raypath_color, 1, /*alt_down=*/true);
  ASSERT_TRUE(gui::g_state.raypath_color[1].solo);
  // Second Alt+click on the same idx clears every solo (compositor's any_solo
  // becomes false, falls back to per-visible composition).
  gui::HandleEyeClick(gui::g_state.raypath_color, 1, /*alt_down=*/true);
  ASSERT_TRUE(!gui::g_state.raypath_color[0].solo);
  ASSERT_TRUE(!gui::g_state.raypath_color[1].solo);
  ASSERT_TRUE(!gui::g_state.raypath_color[2].solo);
}

TEST(ColorWindow, handle_eye_click_alt_switching_target_moves_exclusively) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  for (int i = 0; i < 3; i++) {
    gui::ColorClassConfig c;
    c.visible = true;
    c.solo = false;
    gui::g_state.raypath_color.push_back(c);
  }
  gui::g_state.raypath_color[0].solo = true;  // seed: someone else already solo'd
  gui::HandleEyeClick(gui::g_state.raypath_color, 2, /*alt_down=*/true);
  // Even though phys=2 was not previously solo, the seeded phys=0 must be
  // cleared — the "clear all → set target" ordering (not just "toggle self")
  // enforces the at-most-one invariant across the full class list.
  ASSERT_TRUE(!gui::g_state.raypath_color[0].solo);
  ASSERT_TRUE(!gui::g_state.raypath_color[1].solo);
  ASSERT_TRUE(gui::g_state.raypath_color[2].solo);
}

TEST(ColorWindow, handle_eye_click_out_of_range_is_noop) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::ColorClassConfig c;
  c.visible = true;
  c.solo = false;
  gui::g_state.raypath_color.push_back(c);

  gui::HandleEyeClick(gui::g_state.raypath_color, 99, /*alt_down=*/false);
  gui::HandleEyeClick(gui::g_state.raypath_color, 99, /*alt_down=*/true);

  ASSERT_TRUE(gui::g_state.raypath_color[0].visible);
  ASSERT_TRUE(!gui::g_state.raypath_color[0].solo);
}

// AnySolo + EffectiveVisible
// are render-time derived helpers that MUST mirror the compositor's
// GatherActiveClasses:55 rule (`any_solo ? cls.solo_ : cls.visible_`).
// These pure-helper unit tests pin that mirror to the compositor's actual
// predicate; the Colors-window eye-icon rendering and the merged
// NoVisibleMatchedColorClass predicate both read through these helpers, so
// a drift here would immediately surface as a UI↔composite disagreement.
TEST(ColorWindow, any_solo_false_when_pool_empty_or_all_flags_false) {
  std::vector<gui::ColorClassConfig> classes;
  ASSERT_TRUE(!gui::AnySolo(classes));
  gui::ColorClassConfig c;
  c.solo = false;
  classes.push_back(c);
  classes.push_back(c);
  ASSERT_TRUE(!gui::AnySolo(classes));
}

TEST(ColorWindow, any_solo_true_when_any_flag_set) {
  std::vector<gui::ColorClassConfig> classes;
  gui::ColorClassConfig a;
  a.solo = false;
  gui::ColorClassConfig b;
  b.solo = true;
  classes.push_back(a);
  classes.push_back(b);
  ASSERT_TRUE(gui::AnySolo(classes));
}

TEST(ColorWindow, effective_visible_falls_back_to_visible_when_no_solo) {
  gui::ColorClassConfig c;
  c.visible = true;
  c.solo = false;
  ASSERT_TRUE(gui::EffectiveVisible(c, /*any_solo=*/false));
  c.visible = false;
  ASSERT_TRUE(!gui::EffectiveVisible(c, /*any_solo=*/false));
}

TEST(ColorWindow, effective_visible_reads_solo_when_any_solo) {
  // AC2 core: with any_solo=true, a non-solo peer (visible=true) is
  // effectively hidden — the eye icon must render EYE_SLASH, not EYE.
  gui::ColorClassConfig peer;
  peer.visible = true;
  peer.solo = false;
  ASSERT_TRUE(!gui::EffectiveVisible(peer, /*any_solo=*/true));
  // The solo'd class itself remains visible even if its own visible=false.
  gui::ColorClassConfig soloed;
  soloed.visible = false;
  soloed.solo = true;
  ASSERT_TRUE(gui::EffectiveVisible(soloed, /*any_solo=*/true));
}

// Merged predicate coverage.
// The rename from AllConfiguredColorClassesUnmatched left the "no match"
// scenarios above intact (defaults visible=true, solo=false ⇒ EffectiveVisible=true
// ⇒ old semantics preserved). The tests below cover the NEW cases where
// matched signals exist but are hidden — either by `visible=false` (AC1) or
// by another class being solo'd (AC2 storage-level).
TEST(ColorWindow, no_visible_matched_true_when_matched_class_hidden_via_visible_false) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  // Two configured classes, both with signal=1 (matched), both with
  // visible=false (user hid every matched class). Composite would be empty.
  gui::ColorClassConfig c;
  gui::ColorClassRefConfig r;
  c.match.push_back(r);
  c.visible = false;
  c.solo = false;
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  std::vector<int> flags = { 1, 1 };
  ASSERT_TRUE(gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

TEST(ColorWindow, no_visible_matched_true_when_matched_class_hidden_by_peer_solo) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  // Two configured+matched classes. Class 0 is solo'd → class 1 is
  // effectively hidden. But class 1 has no signal (0), so composite has
  // "class 0 solo'd and matched" — should return false. Then flip: put
  // signal on class 1 only, solo on class 0 — composite is empty because
  // the only matched class (1) is effectively hidden by the peer's solo.
  gui::ColorClassConfig c;
  gui::ColorClassRefConfig r;
  c.match.push_back(r);
  c.visible = true;
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color[0].solo = true;

  // Case A: matched-and-solo'd class 0 satisfies the predicate → false.
  std::vector<int> flags_a = { 1, 1 };
  ASSERT_TRUE(!gui::NoVisibleMatchedColorClass(gui::g_state, flags_a));

  // Case B: only class 1 has signal, but class 0 is solo'd → class 1 is
  // effectively hidden → composite would be empty → true.
  std::vector<int> flags_b = { 0, 1 };
  ASSERT_TRUE(gui::NoVisibleMatchedColorClass(gui::g_state, flags_b));
}

TEST(ColorWindow, no_visible_matched_false_when_soloed_class_is_the_matched_one) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  // Storage semantic: HandleEyeClick sets solo but does NOT touch visible
  // of peers. Confirm the merged predicate still returns false when the
  // solo'd class is itself the (matched) one — the composite is non-empty.
  gui::ColorClassConfig c;
  gui::ColorClassRefConfig r;
  c.match.push_back(r);
  c.visible = true;
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color.push_back(c);
  gui::g_state.raypath_color[1].solo = true;

  std::vector<int> flags = { 1, 1 };
  // AnySolo=true → only class 1 counts as effectively visible. Class 1 has
  // signal → composite is non-empty → predicate returns false.
  ASSERT_TRUE(!gui::NoVisibleMatchedColorClass(gui::g_state, flags));
}

// BuildClassFromFilter — a row whose single Factor still expands to more
// than one alternative ("1-3;5-7", the same ';' OR-separator case as
// validate_single_atom_rejects_semicolon_or) must be skipped like an
// AND-row, not silently imported as a legal single-atom ref that
// FillColorPredicate would then drop at the next commit (same root cause
// as the single-atom ';' rejection above). Uses gui::ParseSummandText for
// `.factors` (rather than the dummy single-Factor placeholder the other
// cases here use) so CountFactorAlternatives sees the real ';' content.
TEST(ColorWindow, build_class_from_filter_skips_multi_alt_row) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  gui::FilterConfig f;
  f.param.clear();
  const std::string text = "1-3;5-7";
  f.param.push_back(gui::SummandText{ text, gui::ParseSummandText(text) });
  f.param.push_back(
      gui::SummandText{ std::string{ "3-5" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });

  int skipped = -1;
  gui::ColorClassConfig cls = gui::BuildClassFromFilter(0, 2, f, skipped);

  ASSERT_EQ(skipped, 1);
  ASSERT_EQ(static_cast<int>(cls.match.size()), 1);
  ASSERT_EQ(cls.match[0].predicate_text, "3-5");
}

// The topbar Colors button must
// render with a distinct tint when at least one color class is configured.
// The tint decision is pinned in the pure predicate ShouldTintColorsButton
// so this test locks the decision boundary directly (raypath_color empty →
// no tint; non-empty → tint), without depending on ImGuiTestEngine's ability
// to read pushed style colors (which its ItemInfo API does not expose).
TEST(ColorWindow, topbar_colors_button_tint_predicate) {
  gui::DoNew();
  gui::ResetColorClassSignalCacheForTest();
  ASSERT_TRUE(!gui::ShouldTintColorsButton(true));  // empty → no tint
  ASSERT_TRUE(gui::ShouldTintColorsButton(false));  // has classes → tint
}
