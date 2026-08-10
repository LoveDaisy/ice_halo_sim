// The left panel's entry cards, and the structure they present.
//
// What this suite is for. Each card in `##LeftPanel` stands for one entry — a crystal drawn from a
// shared pool, a filter, a share of the population — and the card's thumbnail is the only place the
// user ever sees what that crystal looks like without opening the modal. The thumbnails below are
// therefore asked the question the cache's design turns on: they are keyed by CRYSTAL id, not by
// card position, so two cards pointing at one pool slot are meant to share an image and two cards
// pointing at different slots are meant not to. Both halves need a real frame — the images are
// produced by the update queue running across several of them.
//
// NOT YET here, and belonging here. This file currently holds only the thumbnail propositions. The
// rest of what a card and a layer can do — the three Edit buttons and their initial tab, duplicate
// and delete, the link badge two cards on one pool slot wear, `+ Layer` / `+ Crystal` and the
// probability gates a new layer inherits, and the pick-mode strip a "Link to..." puts above the
// cards — are still asserted from test_gui_interaction.cpp, whose cases are grouped by an old
// `p1_card` / `p1_entry` / `p2_linked` category rather than by the panel they drive. When those
// move, they belong in THIS file: a second file on the same panel is the shape this suite is being
// rewritten to remove.
//
// Deliberately NOT here. That the fixed-seed thumbnail draw is deterministic (same config, same
// seed, identical mesh) is test/gui/functional/test_gui_preview_animation.cpp; the crystal
// renderer's own output is pinned against committed pixels in
// test/gui/visual/test_preview_pixels.cpp. Nothing below re-asserts either.
//
// What a user sees when these break: cards with a blank square where the crystal should be, or —
// worse, because it reads as a working feature — two different crystals showing the same picture.

#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// Frames to let ProcessUpdateQueue render queued thumbnails. Generous rather than tight: what is
// under test is that the work happens at all, and a tight bound would turn a slow frame into a red
// run about the wrong thing.
constexpr int kThumbnailFrames = 10;

}  // namespace

void RegisterEntryManagementTests(ImGuiTestEngine* engine) {
  // The default document's single card gets an image without anyone asking for one.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "the_default_entry_card_gets_a_thumbnail");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(kThumbnailFrames);

      IM_CHECK_NE(gui::g_thumbnail_cache.GetTexture(gui::g_state.layers[0].entries[0].crystal_id), 0u);
    };
  }

  // The cache is keyed by CRYSTAL id, and both halves of that sentence are asserted here: a card
  // pointing at a new pool slot gets a texture of its own, and a card pointing at an existing slot
  // gets the one already there.
  //
  // The distinctness assertion is the load-bearing one. A first draft of this case asserted only
  // that both ids returned a non-zero texture, and a red-state probe walked straight through it —
  // a cache that handed every caller the same texture satisfies "not zero" twice over, and the
  // symptom a user reports is precisely two different crystals drawn with one picture.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "each_crystal_slot_has_its_own_thumbnail_and_shares_it");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(5);

      // A second card on a NEW pool slot, and a third card pointing back at the first slot — the
      // shape a duplicate entry produces.
      gui::CrystalConfig second;
      second.type = gui::CrystalType::kPyramid;
      gui::EntryCard on_new_slot;
      on_new_slot.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(second);
      gui::EntryCard sharing_first_slot;
      sharing_first_slot.crystal_id = gui::g_state.layers[0].entries[0].crystal_id;
      gui::g_state.layers[0].entries.push_back(on_new_slot);
      gui::g_state.layers[0].entries.push_back(sharing_first_slot);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(kThumbnailFrames);

      const int first_id = gui::g_state.layers[0].entries[0].crystal_id;
      const int second_id = gui::g_state.layers[0].entries[1].crystal_id;
      const int shared_id = gui::g_state.layers[0].entries[2].crystal_id;
      IM_CHECK_NE(first_id, second_id);  // the premise: two cards, two pool slots
      IM_CHECK_EQ(first_id, shared_id);  // and a third card back on the first slot

      const uintptr_t first_tex = gui::g_thumbnail_cache.GetTexture(first_id);
      const uintptr_t second_tex = gui::g_thumbnail_cache.GetTexture(second_id);
      IM_CHECK_NE(first_tex, 0u);
      IM_CHECK_NE(second_tex, 0u);
      IM_CHECK_NE(first_tex, second_tex);
      IM_CHECK_EQ(gui::g_thumbnail_cache.GetTexture(shared_id), first_tex);
    };
  }
}
