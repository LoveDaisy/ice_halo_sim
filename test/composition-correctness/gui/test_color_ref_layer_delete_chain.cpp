// Composition chain: deleting a layer in the main window, seen from the Colors panel.
//
// Units in the chain: edit_modals (NotifyLayerDeleted, the one delete-site hook) × gui_state
// (Layer / ColorClassRefConfig) × color_window (ResolveColorRef / FormatColorRefLayerLabel, the
// panel's reading of a ref).
//
// A colour-class ref names its layer by POSITION, and `state.layers.erase` is the one operation
// that changes what a position means. What the collaboration must produce: after the erase, every
// ref still denotes the same physical layer it did before — or, if that layer is the one that went,
// reads as deleted in the panel. What it produced before this chain was closed: the ref kept its
// old number and denoted whichever layer shifted into that slot.
//
// Why that was quiet, and why the assertions below are shaped the way they are: the panel's two
// checks are bounds and "crystal is in the layer". Both are side effects of the mistake, not
// detections of it. Reusing one crystal pool slot across layers is a legal document, and in that
// document the shifted-in layer contains the same crystal, so both checks pass and ResolveColorRef
// says kResolved for a ref that now points at the wrong layer. A test that only asserted
// `ResolveColorRef == kResolved` would therefore be green with the bug in place. Every case here
// pins the ref's numeric index AND the identity of the layer it lands on (a per-layer probability
// used as a tag), which is the pair the bug cannot satisfy.

#include <gtest/gtest.h>

#include <string>

#include "gui/color_window.hpp"
#include "gui/edit_modals.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

constexpr int kSharedCrystal = 0;
constexpr int kOwnCrystal = 1;

// One entry per layer; `crystal_id` chooses the pool slot, `tag` is written to the layer's
// probability so a layer can be recognised after the vector has been reshuffled.
Layer MakeLayer(int crystal_id, float tag) {
  Layer layer;
  layer.probability = tag;
  EntryCard card;
  card.crystal_id = crystal_id;
  layer.entries.push_back(card);
  return layer;
}

GuiState MakeDoc() {
  GuiState s;
  s.crystals.clear();
  s.layers.clear();
  s.crystals.emplace_back();  // kSharedCrystal
  s.crystals.emplace_back();  // kOwnCrystal
  return s;
}

ColorClassRefConfig& AddRef(GuiState& s, int layer_idx, int crystal_pool_id) {
  ColorClassConfig cls;
  ColorClassRefConfig ref;
  ref.layer_idx = layer_idx;
  ref.crystal_pool_id = crystal_pool_id;
  cls.match.push_back(ref);
  s.raypath_color.push_back(cls);
  return s.raypath_color.back().match.back();
}

void DeleteLayer(GuiState& s, int layer_idx) {
  // The production order at the delete site: erase first, then notify.
  s.layers.erase(s.layers.begin() + layer_idx);
  NotifyLayerDeleted(s, layer_idx);
}

// NotifyLayerDeleted also reads the modal statics shared by every case in this binary; start and
// end each case with no modal open so a sibling case's leftover binding cannot change the branch
// taken here.
class ColorRefLayerDeleteChain : public ::testing::Test {
 protected:
  void SetUp() override { ResetModalState(); }
  void TearDown() override { ResetModalState(); }
};

// Equal branch: the ref's own layer is the one deleted.
TEST_F(ColorRefLayerDeleteChain, DeletedLayerRefGoesDangling) {
  GuiState s = MakeDoc();
  s.layers.push_back(MakeLayer(kOwnCrystal, 0.1f));
  s.layers.push_back(MakeLayer(kSharedCrystal, 0.2f));
  ColorClassRefConfig& ref = AddRef(s, 1, kSharedCrystal);

  DeleteLayer(s, 1);

  EXPECT_EQ(ref.layer_idx, -1);
  EXPECT_EQ(ResolveColorRef(s, ref), ColorRefResolution::kLayerMissing);
  EXPECT_NE(FormatColorRefLayerLabel(s, ref).find("(deleted)"), std::string::npos);
}

// Greater-than branch — the regression this chain exists for. layer1 and layer2 share a crystal
// pool slot; the ref points at layer1; layer0 is deleted. Without compensation the ref stays at 1,
// which is now the old layer2: same crystal, in range, kResolved, wrong layer.
TEST_F(ColorRefLayerDeleteChain, RefAfterDeletedLayerIsReindexedNotSilentlyResolved) {
  GuiState s = MakeDoc();
  s.layers.push_back(MakeLayer(kOwnCrystal, 0.1f));
  s.layers.push_back(MakeLayer(kSharedCrystal, 0.3f));
  s.layers.push_back(MakeLayer(kSharedCrystal, 0.7f));
  ColorClassRefConfig& ref = AddRef(s, 1, kSharedCrystal);
  ASSERT_EQ(ResolveColorRef(s, ref), ColorRefResolution::kResolved);

  DeleteLayer(s, 0);

  EXPECT_EQ(ref.layer_idx, 0);
  ASSERT_EQ(ResolveColorRef(s, ref), ColorRefResolution::kResolved);
  // The layer it lands on is the one it named before the erase, not merely a slot that also
  // happens to hold the same crystal.
  EXPECT_FLOAT_EQ(s.layers[static_cast<size_t>(ref.layer_idx)].probability, 0.3f);
}

// Less-than branch: a ref before the deleted layer is left exactly where it was.
TEST_F(ColorRefLayerDeleteChain, RefBeforeDeletedLayerIsUnaffected) {
  GuiState s = MakeDoc();
  s.layers.push_back(MakeLayer(kOwnCrystal, 0.4f));
  s.layers.push_back(MakeLayer(kSharedCrystal, 0.5f));
  s.layers.push_back(MakeLayer(kSharedCrystal, 0.6f));
  ColorClassRefConfig& ref = AddRef(s, 0, kOwnCrystal);

  DeleteLayer(s, 1);

  EXPECT_EQ(ref.layer_idx, 0);
  ASSERT_EQ(ResolveColorRef(s, ref), ColorRefResolution::kResolved);
  EXPECT_FLOAT_EQ(s.layers[static_cast<size_t>(ref.layer_idx)].probability, 0.4f);
}

// The refs are re-indexed whether or not an edit modal is open: the colour panel's refs outlive
// any modal, so the compensation must not sit behind the modal's own gate.
TEST_F(ColorRefLayerDeleteChain, RefsAreReindexedWithNoModalOpen) {
  GuiState s = MakeDoc();
  s.layers.push_back(MakeLayer(kOwnCrystal, 0.1f));
  s.layers.push_back(MakeLayer(kSharedCrystal, 0.3f));
  ColorClassRefConfig& ref = AddRef(s, 1, kSharedCrystal);
  ASSERT_FALSE(IsEditModalOpen());

  DeleteLayer(s, 0);

  EXPECT_EQ(ref.layer_idx, 0);
  EXPECT_FLOAT_EQ(s.layers[0].probability, 0.3f);
}

}  // namespace
}  // namespace lumice::gui
