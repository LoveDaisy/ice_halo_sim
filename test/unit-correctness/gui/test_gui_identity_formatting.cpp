// The four display formatters every panel now shares, and the ref-resolution decision the Colours
// window's two combos make.
//
// These are grouped in one file because they are one defect family, not four functions. A user
// reported that a colour class named a crystal and a layer that could not be found anywhere in the
// left panel. Three independent things were true at once:
//
//   * The layer number was rendered 1-based in the panel header and file_io's overflow locator and
//     0-based in four other places, so "Layer 1" meant different layers depending on which widget
//     said it.
//   * The crystal was addressed by a name the GUI had no control for, so every crystal created in
//     the GUI was unnamed and the Colours window fell back to `crystal#N` — an N that appears on no
//     card, and that is a POOL index, so it is not even the card's position in its layer.
//   * A ref whose layer or crystal no longer existed was rendered with a clamped / first-in-list
//     substitute, so the window showed a value that resolves while the scene received the stored
//     one that does not.
//
// Each case below asserts against a hand-written expectation, never against a second call of the
// function under test — the point is the exact string a user reads, so the string is written out.

#include <gtest/gtest.h>

#include <string>

#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/gui_state.hpp"
#include "lumice.h"

namespace gui = lumice::gui;

namespace {

// A scene the resolution cases can break in specific ways: two layers, the second holding a crystal
// the first does not.
void SeedTwoLayerScene() {
  gui::DoNew();
  gui::g_state.crystals[0].name = "plate";

  gui::CrystalConfig second;
  second.name = "column";
  second.type = gui::CrystalType::kPyramid;
  gui::g_state.crystals.push_back(second);

  gui::Layer layer1;
  gui::EntryCard entry;
  entry.crystal_id = 1;
  layer1.entries.push_back(entry);
  gui::g_state.layers.push_back(layer1);
}

gui::ColorClassRefConfig Ref(int layer_idx, int crystal_pool_id) {
  gui::ColorClassRefConfig ref;
  ref.layer_idx = layer_idx;
  ref.crystal_pool_id = crystal_pool_id;
  ref.match_all = true;
  return ref;
}

}  // namespace

// ---- Layer / entry numbering ----

// Stored indices are 0-based (they are vector subscripts) and every user-visible rendering of them
// is 1-based. These two assertions look trivial and are the whole point: the conversion existed as a
// re-typed `+ 1` at six sites, four of which had it missing.
TEST(DisplayNumbering, IndexZeroIsTheFirstOneTheUserSees) {
  EXPECT_EQ(gui::DisplayLayerNumber(0), 1);
  EXPECT_EQ(gui::DisplayEntryNumber(0), 1);
  EXPECT_EQ(gui::DisplayLayerNumber(4), 5);
  EXPECT_EQ(gui::DisplayEntryNumber(4), 5);
}

// ---- Crystal identity ----

TEST(CrystalIdentity, LeadsWithThePoolIdThenTheNameThenTheType) {
  gui::DoNew();
  gui::g_state.crystals[0].name = "plate";
  EXPECT_EQ(gui::FormatCrystalIdentity(gui::g_state, 0), "#0 · plate · Prism");
}

// The id is never dropped in favour of the name. Nothing stops two crystals sharing a name, and two
// identical entries in a combo are worse to choose between than two id-led ones.
TEST(CrystalIdentity, KeepsThePoolIdEvenWhenTwoCrystalsShareAName) {
  gui::DoNew();
  gui::g_state.crystals[0].name = "twin";
  gui::CrystalConfig second;
  second.name = "twin";
  gui::g_state.crystals.push_back(second);

  const std::string a = gui::FormatCrystalIdentity(gui::g_state, 0);
  const std::string b = gui::FormatCrystalIdentity(gui::g_state, 1);
  EXPECT_EQ(a, "#0 · twin · Prism");
  EXPECT_EQ(b, "#1 · twin · Prism");
  EXPECT_NE(a, b) << "two same-named crystals must still be distinguishable in a combo";
}

TEST(CrystalIdentity, AnUnnamedCrystalIsIdAndTypeWithNoEmptySegment) {
  gui::DoNew();
  gui::CrystalConfig pyramid;
  pyramid.type = gui::CrystalType::kPyramid;
  gui::g_state.crystals.push_back(pyramid);
  EXPECT_EQ(gui::FormatCrystalIdentity(gui::g_state, 1), "#1 · Pyramid");
}

// A dangling ref's whole diagnostic value is WHICH id dangles, so the id leads even here.
TEST(CrystalIdentity, ADanglingPoolIdStillShowsTheIdItStored) {
  gui::DoNew();
  EXPECT_EQ(gui::FormatCrystalIdentity(gui::g_state, 7), "#7 <missing>");
  EXPECT_EQ(gui::FormatCrystalIdentity(gui::g_state, -1), "#-1 <missing>");
}

// The anti-drift pin. CrystalDisplayName is the Colours window's `const char*` adapter and is the
// site that historically carried its own spelling; if a branch is ever added back to it, the card
// and the Colours window start naming the same crystal differently again and this goes red.
TEST(CrystalIdentity, TheColorsWindowAdapterSpellsItExactlyLikeTheSharedFormatter) {
  SeedTwoLayerScene();
  gui::CrystalConfig unnamed;
  gui::g_state.crystals.push_back(unnamed);  // pool id 2, no name

  for (int pool_id : { -1, 0, 1, 2, 9 }) {
    SCOPED_TRACE(pool_id);
    EXPECT_EQ(std::string(gui::CrystalDisplayName(gui::g_state, pool_id)),
              gui::FormatCrystalIdentity(gui::g_state, pool_id));
  }
}

// ---- Colour-ref resolution ----

TEST(ColorRefResolution, ARefIntoALayerThatHoldsItsCrystalResolves) {
  SeedTwoLayerScene();
  EXPECT_EQ(gui::ResolveColorRef(gui::g_state, Ref(1, 1)), gui::ColorRefResolution::kResolved);
  EXPECT_EQ(gui::FormatColorRefLayerLabel(gui::g_state, Ref(1, 1)), "Layer 2");
  EXPECT_EQ(gui::FormatColorRefCrystalLabel(gui::g_state, Ref(1, 1)), "#1 · column · Pyramid");
}

// The layer half of the display/truth split. The window used to clamp this index for display only:
// a ref storing layer 1 in a one-layer scene rendered as layer 0 while the scene received 1.
TEST(ColorRefResolution, AMissingLayerIsNamedByTheNumberTheRefActuallyStores) {
  gui::DoNew();  // one layer
  const gui::ColorClassRefConfig ref = Ref(1, 0);
  EXPECT_EQ(gui::ResolveColorRef(gui::g_state, ref), gui::ColorRefResolution::kLayerMissing);

  const std::string label = gui::FormatColorRefLayerLabel(gui::g_state, ref);
  EXPECT_EQ(label, "Layer 2 (deleted)");
  EXPECT_NE(label, "Layer 1") << "the label showed the layer the ref would be clamped to, not the one it stores";
}

// The crystal half of the same split: a pool id absent from the chosen layer used to render as that
// layer's FIRST crystal, silently, while the ref kept pointing somewhere else.
TEST(ColorRefResolution, ACrystalAbsentFromTheLayerIsNamedByTheIdTheRefStores) {
  SeedTwoLayerScene();
  const gui::ColorClassRefConfig ref = Ref(0, 1);  // crystal 1 lives in layer 1, not layer 0
  EXPECT_EQ(gui::ResolveColorRef(gui::g_state, ref), gui::ColorRefResolution::kCrystalNotInLayer);

  const std::string label = gui::FormatColorRefCrystalLabel(gui::g_state, ref);
  EXPECT_EQ(label, "#1 · column · Pyramid (not in this layer)");
  EXPECT_EQ(label.find("plate"), std::string::npos) << "the label named layer 0's first crystal instead of the ref's";
}

// A ref can be broken in both halves at once; the layer is the outer question, so it wins the
// report. Otherwise a ref pointing into a deleted layer would be described by which crystals that
// (non-existent) layer does not contain.
TEST(ColorRefResolution, AMissingLayerIsReportedBeforeAMissingCrystal) {
  gui::DoNew();
  EXPECT_EQ(gui::ResolveColorRef(gui::g_state, Ref(3, 9)), gui::ColorRefResolution::kLayerMissing);
}
