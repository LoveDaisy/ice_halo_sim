// The Colours window's decisions that need no live ImGui context.
//
// What the window has to get right is mostly bookkeeping the user never sees directly but always
// feels: which class paints on top, which classes are hidden, whether the "no rays matched" warning
// is telling the truth, and whether importing a filter produced a class that means the same thing.
//
// Its siblings in test/gui/functional/test_color_window.cpp need a real frame — most drive the
// window through an ImGuiTestContext (clicking the eye icon, toggling checkboxes, reading disabled
// state and glyphs), and three call gui::RefreshColorClassSignals, whose body reaches
// ImGui::GetTime(). That last group is the interesting one: those bodies contain no ImGui call of
// their own, so a criterion that only greps a test body for `ImGui::` passes them as pure logic —
// and without a context the result is a segfault, not a failed assertion. A production function's
// own context requirement is part of the case's requirement.
//
// Two mechanical notes carried over from the IM_* form these cases were translated from:
//   * IM_CHECK / IM_CHECK_EQ RETURN from the test function when they fail, so ASSERT_*, not
//     EXPECT_*, is the mapping that preserves what they did before the move.
//   * The fixture's DoNew() plus ResetColorClassSignalCacheForTest() replaces gui_test's
//     ResetTestState(). The second half is not decoration: the signal cache is a file-scope static
//     keyed on (server, epoch), and gui_test got it reset between cases for free.

#include <gtest/gtest.h>

#include <initializer_list>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "gui/raypath_segments.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

namespace {

// One class as the cases below need to describe it: whether the user has wired any ref into it, and
// the two independent ways it can be kept out of the picture.
struct ClassSpec {
  bool configured = false;
  bool visible = true;
  bool solo = false;
  int z_order = 0;
};

}  // namespace

class ColorWindow : public ::testing::Test {
 protected:
  void SetUp() override {
    gui::DoNew();
    gui::ResetColorClassSignalCacheForTest();
  }

  static void SeedClasses(std::initializer_list<ClassSpec> specs) {
    for (const ClassSpec& s : specs) {
      gui::ColorClassConfig c;
      if (s.configured) {
        c.match.push_back(gui::ColorClassRefConfig{});
      }
      c.visible = s.visible;
      c.solo = s.solo;
      c.z_order = s.z_order;
      gui::g_state.raypath_color.push_back(c);
    }
  }

  static std::vector<gui::ColorClassConfig>& Classes() { return gui::g_state.raypath_color; }
};

// ---- Layer order is a set of numbers, not the order of the vector ----
//
// The vector index is the C API's lane binding: GetColorClassLaneY(i) means physical slot i, so a
// reorder that moved entries would silently re-point every lane. Reordering is therefore expressed
// by rewriting z_order alone — and a delete leaves a hole in that numbering, which
// LUMICE_SetRaypathColors rejects because it demands a permutation of [0, N).
TEST_F(ColorWindow, ReorderingAndCompactingRewriteTheLayerNumbersAndNeverMoveTheClasses) {
  gui::ColorClassConfig red;
  red.color[0] = 1.0f;
  red.z_order = 0;
  gui::ColorClassConfig green;
  green.color[1] = 1.0f;
  green.z_order = 1;
  Classes().push_back(red);
  Classes().push_back(green);

  gui::SwapZOrder(gui::g_state, 0, 1);

  ASSERT_EQ(Classes()[0].color[0], 1.0f);  // slot 0 is still the red class
  ASSERT_EQ(Classes()[1].color[1], 1.0f);
  ASSERT_EQ(Classes()[0].z_order, 1);
  ASSERT_EQ(Classes()[1].z_order, 0);

  // The caller is UI code iterating on user input, so a bad index must leave the state alone rather
  // than corrupt it.
  gui::SwapZOrder(gui::g_state, 0, 0);
  gui::SwapZOrder(gui::g_state, 0, 42);
  gui::SwapZOrder(gui::g_state, 99, 100);
  ASSERT_EQ(Classes()[0].z_order, 1);
  ASSERT_EQ(Classes()[1].z_order, 0);

  // Compaction packs the same numbering without changing what is on top. Arbitrary numbers with
  // gaps: the visible order is slot 1, slot 0, slot 2.
  gui::DoNew();
  SeedClasses(
      { ClassSpec{ false, true, false, 5 }, ClassSpec{ false, true, false, 2 }, ClassSpec{ false, true, false, 9 } });
  gui::CompactZOrder(gui::g_state);
  ASSERT_EQ(Classes()[1].z_order, 0);
  ASSERT_EQ(Classes()[0].z_order, 1);
  ASSERT_EQ(Classes()[2].z_order, 2);

  // And after a delete, which is where the hole comes from in practice: erasing the middle of
  // [0, 1, 2] leaves [0, 2], which is not a permutation.
  gui::DoNew();
  SeedClasses(
      { ClassSpec{ false, true, false, 0 }, ClassSpec{ false, true, false, 1 }, ClassSpec{ false, true, false, 2 } });
  Classes().erase(Classes().begin() + 1);
  gui::CompactZOrder(gui::g_state);
  ASSERT_EQ(static_cast<int>(Classes().size()), 2);
  ASSERT_EQ(Classes()[0].z_order, 0);
  ASSERT_EQ(Classes()[1].z_order, 1);
}

// ---- A colour ref carries ONE raypath atom ----
//
// A LUMICE_ColorPredicate is a single atom; a class combines several of them with combine:any or
// combine:all. So the two ways of writing a compound expression inside one ref have to be refused at
// the input, or FillColorPredicate silently drops the extra terms at commit time and the class
// quietly matches more than the user asked for.
TEST_F(ColorWindow, ARefRejectsAnyTextThatIsMoreThanASingleAtom) {
  struct AtomCase {
    const char* name;
    const char* text;
    bool valid;
  };
  const AtomCase kCases[] = {
    // Empty means the whole crystal, which is a legal single atom.
    { "empty", "", true },
    { "one raypath", "3-5-1", true },
    // AND inside one atom: there is no intra-atom AND, only combine:all across refs.
    { "an AND of two factors", "3-5 & entry:2", false },
    // The ';' separator expands one factor into two alternatives. Both halves here use legal prism
    // face numbers, so the face-range checks pass and the rejection can only come from the
    // alternative count — otherwise this row would go green for the wrong reason.
    { "two alternatives in one factor", "1-3;5-7", false },
    // The colour window and the filter edit modal share one validator, so retiring ',' as a path
    // connector reaches this input box with no code of its own — this row is what says so.
    { "legacy comma connector", "3,5", false },
  };

  for (const AtomCase& c : kCases) {
    const auto v = gui::ValidateSingleAtomText(c.text);
    EXPECT_EQ(v.state, c.valid ? LUMICE_RAYPATH_VALID : LUMICE_RAYPATH_INVALID) << c.name;
    // A rejection the user cannot read is the same as a silent one.
    EXPECT_EQ(v.message.empty(), c.valid) << c.name;
  }

  // ...and the ',' rejection specifically has to arrive here with the message that names the fix,
  // not a generic one — the shared validator is only worth sharing if what it says survives the
  // trip through this window's own wrapper.
  const auto comma = gui::ValidateSingleAtomText("3,5");
  EXPECT_NE(comma.message.find("'-'"), std::string::npos) << comma.message;
  EXPECT_NE(comma.message.find("';'"), std::string::npos) << comma.message;
}

// ---- Per-ref symmetry ----
//
// Three independent bits, defaulting to none — deliberately unlike FilterConfig, and mirroring core's
// RaypathColorRef. All three must reach operator==, because the frame-tail reconciler diffs whole
// structs to decide whether an edit needs a re-simulation: a bit missing from the comparison is a
// symmetry edit that never re-runs.
TEST_F(ColorWindow, EachSymmetryBitIsIndependentAndVisibleToTheReconcilersDiff) {
  const gui::ColorClassRefConfig factory;
  ASSERT_TRUE(!factory.sym_p && !factory.sym_b && !factory.sym_d);

  struct BitCase {
    const char* name;
    bool gui::ColorClassRefConfig::*bit;
  };
  const BitCase kBits[] = {
    { "p", &gui::ColorClassRefConfig::sym_p },
    { "b", &gui::ColorClassRefConfig::sym_b },
    { "d", &gui::ColorClassRefConfig::sym_d },
  };

  for (const BitCase& b : kBits) {
    gui::ColorClassRefConfig ref;
    ref.*b.bit = true;
    // Setting one bit sets exactly that one.
    for (const BitCase& other : kBits) {
      EXPECT_EQ(ref.*other.bit, &other == &b) << b.name << " vs " << other.name;
    }
    // ... and the diff sees it.
    EXPECT_TRUE(factory != ref) << b.name;
  }

  // Clearing one leaves its siblings alone.
  gui::ColorClassRefConfig both;
  both.sym_p = true;
  both.sym_d = true;
  both.sym_p = false;
  EXPECT_TRUE(!both.sym_p && !both.sym_b && both.sym_d);

  // A whole-crystal ref matches every raypath through the placement, so symmetry is a no-op there
  // and the checkboxes freeze rather than clear — un-checking "whole" must bring the previous
  // selection back.
  gui::ColorClassRefConfig ref;
  ref.match_all = true;
  EXPECT_FALSE(gui::IsRefSymmetryEditable(ref));
  ref.match_all = false;
  EXPECT_TRUE(gui::IsRefSymmetryEditable(ref));
}

// Toggling "whole crystal" in either direction must leave the typed text where it was: the field is
// frozen, not cleared. Clearing it meant un-checking the box left the row blank and the user's
// expression gone.
TEST_F(ColorWindow, TogglingWholeCrystalNeverDiscardsTheTypedExpression) {
  for (bool to_whole : { true, false }) {
    gui::ColorClassRefConfig ref;
    ref.predicate_text = "3-5";
    ref.match_all = !to_whole;
    gui::SetRefMatchAll(ref, to_whole);
    EXPECT_EQ(ref.match_all, to_whole);
    EXPECT_EQ(ref.predicate_text, std::string("3-5"));
  }
}

// ---- Importing a filter as a colour class ----
//
// Each single-atom row of the filter becomes one ref with the same text; a row the ref format cannot
// carry is skipped and counted rather than half-imported. The class then owns its refs by value, so
// editing the filter afterwards does not reach back into it.
//
// One filter carrying every row kind, rather than one filter per kind: the kinds are distinguished
// by their position in the imported class and by the skip count, which a per-kind case with a
// one-row filter cannot tell apart from a wholesale reject. Both skip reasons are present because
// they are found in different places — an AND is two factors, whereas the ';' row is a SINGLE factor
// holding two alternatives, and it is parsed for real rather than given a placeholder factor so the
// alternative count sees the actual content.
TEST_F(ColorWindow, ImportingAFilterKeepsTheRowsARefCanCarryAndCountsTheRest) {
  gui::FilterConfig f;
  f.name = "src_filter";
  f.param.clear();
  f.param.push_back(
      gui::SummandText{ std::string{ "3-5" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });
  f.param.push_back(
      gui::SummandText{ std::string{ "1-2-3" }, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });
  // Two factors AND-ed: no single atom can hold this.
  f.param.push_back(gui::SummandText{
      std::string{ "5-7 & entry:2" },
      std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} }, gui::Factor{ gui::EntryExitParams{} } } });
  // One factor, two alternatives. A row imported here would be dropped again at the next commit,
  // which is a silent loss.
  const std::string multi = "1-3;5-7";
  f.param.push_back(gui::SummandText{ multi, gui::ParseSummandText(multi) });
  // No text at all means the whole crystal, mirroring the ref's own match_all.
  f.param.push_back(gui::SummandText{ std::string{}, std::vector<gui::Factor>{ gui::Factor{ gui::RaypathParams{} } } });

  int skipped = -1;
  gui::ColorClassConfig cls = gui::BuildClassFromFilter(0, 7, f, skipped);

  ASSERT_EQ(skipped, 2);
  // Rows of a filter are OR-ed, so the imported class must be too.
  ASSERT_EQ(cls.combine, LUMICE_COLOR_COMBINE_ANY);
  ASSERT_EQ(static_cast<int>(cls.match.size()), 3);
  ASSERT_EQ(cls.match[0].layer_idx, 0);
  ASSERT_EQ(cls.match[0].crystal_pool_id, 7);
  ASSERT_TRUE(!cls.match[0].match_all);
  ASSERT_EQ(cls.match[0].predicate_text, "3-5");
  ASSERT_EQ(cls.match[1].predicate_text, "1-2-3");
  ASSERT_TRUE(cls.match[2].match_all);
  // Full symmetry on EVERY imported ref, which is what catches the flag being set once outside the
  // loop rather than per row.
  for (const auto& ref : cls.match) {
    EXPECT_TRUE(ref.sym_p && ref.sym_b && ref.sym_d);
  }

  // The class owns its refs: editing the source afterwards must not reach it.
  f.param[0].text = "MUTATED";
  ASSERT_EQ(cls.match[0].predicate_text, "3-5");
}

// ---- Adding or deleting a class must not make the others look unmatched ----
//
// The signal cache is resized to follow the class list, and the value it fills new slots with is the
// whole point: zero would mean "we know this class matched nothing", and for one frame after adding
// a class EVERY class would carry the "no rays matched" warning. One means "not known yet", which is
// the truth until the next poll.
TEST_F(ColorWindow, ResizingTheSignalCacheKeepsWhatIsKnownAndAssumesNothingAboutTheRest) {
  SeedClasses({ ClassSpec{}, ClassSpec{} });
  std::vector<int> flags = { 1, 1 };

  // The user adds a third class; no commit has happened, so a null server takes the early return
  // AFTER the resize and leaves the resize itself under test.
  SeedClasses({ ClassSpec{} });
  gui::PollColorClassSignal(gui::g_state, nullptr, flags);
  ASSERT_EQ(static_cast<int>(flags.size()), 3);
  EXPECT_EQ(flags[0], 1);
  EXPECT_EQ(flags[1], 1);
  EXPECT_EQ(flags[2], 1);

  // And a delete: the cache shrinks, and the prefix that survives keeps its values.
  gui::DoNew();
  SeedClasses({ ClassSpec{} });
  std::vector<int> shrinking = { 1, 1, 0 };
  gui::PollColorClassSignal(gui::g_state, nullptr, shrinking);
  ASSERT_EQ(static_cast<int>(shrinking.size()), 1);
  EXPECT_EQ(shrinking[0], 1);
}

// ---- "No rays matched" must mean the picture is really empty ----
//
// The warning fires when the user has configured at least one class AND nothing they can currently
// see matched anything. Every column below is a way that sentence can be got wrong, and each has its
// own failure the user would report: warning on an untouched document, warning while the cache has
// not caught up with a just-added class, or staying quiet while the composite is genuinely blank
// because everything matched is hidden.
TEST_F(ColorWindow, TheNoMatchWarningFiresExactlyWhenNothingVisibleMatched) {
  const ClassSpec kUnconfigured{ /*configured=*/false };
  const ClassSpec kConfigured{ /*configured=*/true };
  const ClassSpec kConfiguredHidden{ /*configured=*/true, /*visible=*/false };
  const ClassSpec kConfiguredSolo{ /*configured=*/true, /*visible=*/true, /*solo=*/true };

  struct WarnCase {
    const char* name;
    std::vector<ClassSpec> classes;
    std::vector<int> flags;
    bool expect_warning;
  };
  const WarnCase kCases[] = {
    { "no classes at all", {}, {}, false },
    // Added but not yet wired to anything: there is nothing to have matched.
    { "classes with no refs", { kUnconfigured, kUnconfigured }, { 0, 0 }, false },
    { "every configured class silent", { kConfigured, kConfigured }, { 0, 0 }, true },
    { "one configured class has signal", { kConfigured, kConfigured }, { 0, 1 }, false },
    // A class with no refs neither triggers the warning nor suppresses it.
    { "a silent configured class beside an unconfigured one", { kConfigured, kUnconfigured }, { 0, 0 }, true },
    // The cache has not grown to cover the class yet. Treating a missing entry as a confirmed
    // no-signal is a false warning on a document the user just edited.
    { "the only configured class is not in the cache yet", { kConfigured }, {}, false },
    // ... but an unknown peer must not suppress a class that IS known to be silent.
    { "a known-silent class beside an unknown one", { kConfigured, kConfigured }, { 0 }, true },
    // Matched, but hidden: the composite really is empty, so the warning is correct.
    { "every matched class hidden", { kConfiguredHidden, kConfiguredHidden }, { 1, 1 }, true },
    // Solo hides the peers without touching their visible flags, so the aggregate has to read
    // effective visibility rather than the stored one.
    { "the solo'd class is the matched one", { kConfiguredSolo, kConfigured }, { 1, 1 }, false },
    { "only the peer hidden by a solo has signal", { kConfiguredSolo, kConfigured }, { 0, 1 }, true },
  };

  for (const WarnCase& c : kCases) {
    gui::DoNew();
    gui::ResetColorClassSignalCacheForTest();
    for (const ClassSpec& s : c.classes) {
      SeedClasses({ s });
    }
    std::vector<int> flags = c.flags;
    EXPECT_EQ(gui::NoVisibleMatchedColorClass(gui::g_state, flags), c.expect_warning) << c.name;
  }
}

// ---- The eye icon ----
//
// A plain click hides one class. Alt-click means "show only this one", which is a different thing:
// it is exclusive, so it must clear every other solo first rather than toggling its own — otherwise
// two classes can be solo'd at once and the at-most-one invariant the compositor relies on breaks.
TEST_F(ColorWindow, ClickingTheEyeHidesOneClassAndAltClickingSolosExactlyOne) {
  struct EyeCase {
    const char* name;
    int seed_solo;  // -1 = nobody
    int click_index;
    bool alt;
    int expect_solo;  // -1 = nobody
    int expect_hidden;
  };
  const EyeCase kCases[] = {
    { "a plain click hides only what was clicked", -1, 0, false, -1, 0 },
    { "alt-click solos one and nothing else", -1, 1, true, 1, -1 },
    // A second alt-click on the same class clears every solo, so the compositor falls back to plain
    // visibility rather than leaving one class permanently alone.
    { "alt-clicking the soloed class clears it", 1, 1, true, -1, -1 },
    // The target was not previously solo, so a "toggle self" implementation would leave the seeded
    // one set as well.
    { "alt-click moves the solo off whoever had it", 0, 2, true, 2, -1 },
    { "an index nobody has is a no-op", -1, 99, false, -1, -1 },
    { "... whether or not alt is held", -1, 99, true, -1, -1 },
  };

  for (const EyeCase& c : kCases) {
    gui::DoNew();
    gui::ResetColorClassSignalCacheForTest();
    SeedClasses({ ClassSpec{}, ClassSpec{}, ClassSpec{} });
    if (c.seed_solo >= 0) {
      Classes()[c.seed_solo].solo = true;
    }

    gui::HandleEyeClick(Classes(), c.click_index, c.alt);

    for (int i = 0; i < static_cast<int>(Classes().size()); ++i) {
      EXPECT_EQ(Classes()[i].solo, i == c.expect_solo) << c.name << ", class " << i;
      EXPECT_EQ(Classes()[i].visible, i != c.expect_hidden) << c.name << ", class " << i;
    }
  }
}

// What "hidden" means once a solo exists, which is the rule the compositor applies and the eye icon
// has to mirror or the two disagree on screen: with any class solo'd, only solo counts; with none,
// only visible does. A class solo'd with its own visible flag off is still shown — solo overrides
// rather than combines.
TEST_F(ColorWindow, WithAnyClassSoloedOnlySoloDecidesWhatIsShown) {
  std::vector<gui::ColorClassConfig> classes;
  EXPECT_FALSE(gui::AnySolo(classes));
  classes.resize(2);
  EXPECT_FALSE(gui::AnySolo(classes));
  classes[1].solo = true;
  EXPECT_TRUE(gui::AnySolo(classes));

  struct VisibilityCase {
    const char* name;
    bool visible;
    bool solo;
    bool any_solo;
    bool expected;
  };
  const VisibilityCase kCases[] = {
    { "no solo anywhere: visible decides", true, false, false, true },
    { "no solo anywhere: hidden stays hidden", false, false, false, false },
    // A perfectly visible class is hidden by someone else's solo — the eye must show it struck out.
    { "a solo exists elsewhere", true, false, true, false },
    // ... and the soloed class is shown even though its own visible flag is off.
    { "the soloed class itself", false, true, true, true },
  };

  for (const VisibilityCase& c : kCases) {
    gui::ColorClassConfig cls;
    cls.visible = c.visible;
    cls.solo = c.solo;
    EXPECT_EQ(gui::EffectiveVisible(cls, c.any_solo), c.expected) << c.name;
  }
}

// The top-bar Colours button is tinted once the document has classes, which is the only cue that the
// feature is in use while the window is closed. Pinned on the predicate because the test engine's
// item API cannot read a pushed style colour.
TEST_F(ColorWindow, TheTopBarColoursButtonIsTintedOnlyWhenTheDocumentHasClasses) {
  ASSERT_TRUE(!gui::ShouldTintColorsButton(/*raypath_color_empty=*/true));
  ASSERT_TRUE(gui::ShouldTintColorsButton(/*raypath_color_empty=*/false));
}
