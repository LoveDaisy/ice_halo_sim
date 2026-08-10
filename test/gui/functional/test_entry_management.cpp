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

  // A second entry bound to its OWN pool slot gets its own image. The cache is keyed by crystal id,
  // so this is the case that separates "the cache regenerates when the layer structure changes"
  // from "the cache happens to hold one texture and every card reads it".
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "a_second_crystal_slot_gets_a_thumbnail_of_its_own");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(5);

      gui::CrystalConfig second;
      second.type = gui::CrystalType::kPyramid;
      gui::EntryCard card;
      card.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(second);
      gui::g_state.layers[0].entries.push_back(card);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(kThumbnailFrames);

      const int first_id = gui::g_state.layers[0].entries[0].crystal_id;
      const int second_id = gui::g_state.layers[0].entries[1].crystal_id;
      IM_CHECK_NE(first_id, second_id);  // the premise: two cards, two pool slots
      IM_CHECK_NE(gui::g_thumbnail_cache.GetTexture(first_id), 0u);
      IM_CHECK_NE(gui::g_thumbnail_cache.GetTexture(second_id), 0u);
    };
  }
}
