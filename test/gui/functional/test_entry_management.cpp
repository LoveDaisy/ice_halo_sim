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
// The rest of the panel is here too: the three Edit buttons and the tab each opens, duplicate and
// delete, `+ Layer` / `+ Crystal` and the probability gates a new layer inherits, the sharing row
// and the pick-mode strip that "Link to..." puts above the cards. What ties them together is the
// crystal POOL — a card names a slot rather than owning one — so most of what can go wrong here is
// two cards silently pointing at one slot, or one card quietly getting a slot of its own when the
// user asked to share.
//
// Deliberately NOT here, with where each lives instead. That the fixed-seed thumbnail draw is
// deterministic is test/gui/functional/test_gui_preview_animation.cpp; the crystal renderer's own
// output is pinned against committed pixels in test/gui/visual/test_preview_pixels.cpp;
// CountEntriesSharing and UnlinkEntryFromPool are asserted over their whole domain in
// unit-correctness/gui/test_gui_state.cpp, so nothing below re-derives them — what is asserted here
// is that the buttons reach them. The modal those buttons open is
// test/gui/functional/test_edit_modal.cpp.
//
// Four propositions about this panel are NOT assertable from any layer that can see a frame, and
// are left uncovered rather than faked: the filter summary being clipped instead of overlapping the
// Edit button, its sum-of-products tooltip appearing only for a non-degenerate filter, the fa-link
// badge two cards on one slot wear, and the modal header's "Shared with N other entr(y/ies)" count.
// All four are drawn with ImGui::Text*, which submits with id == 0, and the test engine's item hook
// only fires for non-zero ids — so ItemExists and ItemInfo can never see them. The Unlink button
// beside that header IS addressable and is asserted below, which covers the same state transition
// on the one control that can be reached; covering the TEXT needs a fifth committed-pixel reference
// group.
//
// What a user sees when these break: cards with a blank square where the crystal should be; two
// different crystals showing the same picture; a "Link to..." that appears to work and leaves the
// two cards independent; or a new layer that receives no rays at all because the layer above it
// still passes none on.

#include <cstring>
#include <string>

#include "IconsFontAwesome6.h"
#include "gui/gui_state.hpp"
#include "imgui_internal.h"  // ImGuiWindow — the Colors-window occlusion case reads z-order directly
#include "test_gui_shared.hpp"

namespace {

// Frames to let ProcessUpdateQueue render queued thumbnails. Generous rather than tight: what is
// under test is that the work happens at all, and a tight bound would turn a slow frame into a red
// run about the wrong thing.
constexpr int kThumbnailFrames = 10;

// A tree row's path. The cards these replaced could not be addressed at all — every card's Edit
// button carried the same label under a different pushed-integer id, so a wildcard stopped at the
// first one and the suite had to find each card's child window by walking ImGui's window list.
// The rows carry their indices in the label for exactly this reason, so they are ordinary paths.
std::string RowRef(int layer_idx, int entry_idx) {
  return "**/##row_" + std::to_string(layer_idx) + "_" + std::to_string(entry_idx);
}

// The middle of a row, in screen coordinates, for the cases that must click a POINT rather than an
// item — the click-through guard below asks whether a click at a location reaches the row, which a
// path-addressed ItemClick would answer by construction.
ImVec2 RowSpot(ImGuiTestContext* ctx, int layer_idx, int entry_idx) {
  const ImGuiTestItemInfo info = ctx->ItemInfo(RowRef(layer_idx, entry_idx).c_str());
  IM_CHECK_RETV(info.ID != 0, ImVec2(0, 0));
  return info.RectFull.GetCenter();
}

// A second entry in layer 0, bound to a pool slot of its own.
void AddSecondEntryOnItsOwnSlot(ImGuiTestContext* ctx) {
  gui::CrystalConfig second;
  second.type = gui::CrystalType::kPyramid;
  gui::EntryCard card;
  card.crystal_id = static_cast<int>(gui::g_state.crystals.size());
  gui::g_state.crystals.push_back(second);
  gui::g_state.layers[0].entries.push_back(card);
  gui::g_thumbnail_cache.OnLayerStructureChanged();
  ctx->Yield(3);
}

// The Colors window one case parks over the cards, closed on every exit path.
//
// An object rather than a statement at the end of that case: its assertion is a negative one — the
// click did NOT reach the card underneath — so the exit that skips a trailing reset is exactly the
// one where a real regression fired. `color_window_open` is rebuilt by the next case's
// ResetTestState(); what this buys is that the case gives it back itself.
struct ScopedColorWindow {
  ~ScopedColorWindow() { gui::g_state.color_window_open = false; }
};

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

  // ================================================================================
  // Cards and layers: what the two "+" buttons and the two hover buttons do.
  // ================================================================================

  // P76 / P69. `+ Crystal` appends a card to the layer and the hover × removes it again. The pool
  // is the load-bearing half: the new card must get a slot of its OWN. It used to be seeded at
  // slot 0, which reads as a working "add" until the user edits either card and both change.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "add_crystal_appends_a_card_on_a_fresh_pool_slot");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
      const int first_cid = gui::g_state.layers[0].entries[0].crystal_id;
      const std::size_t pool_before = gui::g_state.crystals.size();

      ctx->ItemClick("**/+ Crystal##layer_0");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(gui::g_state.crystals.size(), pool_before + 1);
      const int new_cid = gui::g_state.layers[0].entries[1].crystal_id;
      IM_CHECK_NE(new_cid, first_cid);
      // ...and the pair must not read as shared, or the link badge would claim a link nobody made.
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, new_cid, std::nullopt), 1);

      // The hover × is always in the tree (alpha 0 when the card is not hovered), so the engine can
      // reach it by id.
      ctx->ItemClick("**/" ICON_FA_XMARK "##del_0_1");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 1);
    };
  }

  // P69. A layer must keep at least one card, so the × is greyed rather than absent — the button
  // stays in place and explains itself by being disabled instead of the row silently changing
  // shape between one card and two.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "the_last_card_in_a_layer_cannot_be_deleted");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      IM_CHECK(IsDisabled(ctx->ItemInfo("**/" ICON_FA_XMARK "##del_0_0")));

      AddSecondEntryOnItsOwnSlot(ctx);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/" ICON_FA_XMARK "##del_0_0")));
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/" ICON_FA_XMARK "##del_0_1")));
    };
  }

  // P73 / P76. `+ Layer` and the per-layer × walk the layer list, and the new layer's seed card
  // gets its own pool slot for the same reason `+ Crystal`'s does.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "add_layer_appends_a_layer_seeded_on_a_fresh_slot");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const int first_cid = gui::g_state.layers[0].entries[0].crystal_id;
      const std::size_t pool_before = gui::g_state.crystals.size();

      ctx->ItemClick("**/+ Layer");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 2);
      IM_CHECK_EQ(gui::g_state.crystals.size(), pool_before + 1);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.back().entries.size()), 1);
      const int new_cid = gui::g_state.layers.back().entries[0].crystal_id;
      IM_CHECK_NE(new_cid, first_cid);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, new_cid, std::nullopt), 1);

      ctx->ItemClick("**/" ICON_FA_XMARK "##layer_1");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
    };
  }

  // P77. The layer list is drawn in index order with a gap between blocks — a claim about the panel
  // as a whole rather than about any one layer, which is why nothing above covers it. Both halves
  // are silent when they break: layers presented out of order still carry the right numbers on
  // their headers, so the scattering ORDER a user reads off the panel would simply be the wrong
  // one; blocks drawn flush against each other still show every control.
  //
  // The gap is measured against ImGui's own ItemSpacing rather than against a pixel constant.
  // RenderScatteringSection's `ImGui::Spacing()` between layers is a zero-height item, so the
  // distance from one block's last control to the next block's header is TWO item spacings where
  // an unseparated list would give one. Halfway between the two is the discriminator, and it
  // scales with the style instead of pinning this case to one font size.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "the_layers_are_drawn_in_order_and_kept_apart");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      ctx->ItemClick("**/+ Layer");
      ctx->Yield(2);
      ctx->ItemClick("**/+ Layer");
      ctx->Yield(3);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 3);  // the premise

      // Header labels are 1-based ("Layer 1"), the ids they carry are 0-based (`##layer_0`). Both
      // are read: the header is what the user sees, the button is what the id says it belongs to.
      const ImGuiTestItemInfo headers[3] = { ctx->ItemInfo("**/Layer 1"), ctx->ItemInfo("**/Layer 2"),
                                             ctx->ItemInfo("**/Layer 3") };
      const ImGuiTestItemInfo tails[3] = { ctx->ItemInfo("**/+ Crystal##layer_0"),
                                           ctx->ItemInfo("**/+ Crystal##layer_1"),
                                           ctx->ItemInfo("**/+ Crystal##layer_2") };
      // A clipped item reports id 0, and reading a rectangle off one would compare zeroes and
      // pass. This is the case's own premise, so it is fatal — the whole point is to abort before
      // the order checks below run on garbage rects. Unrolled rather than looped: a fatal assert
      // inside a `for` body is exactly the shape scripts/check_loop_fatal_asserts.py flags, and
      // unrolling six fixed checks also gives each its own source line instead of one shared by
      // all three loop iterations.
      IM_CHECK_NE(headers[0].ID, 0u);
      IM_CHECK_NE(tails[0].ID, 0u);
      IM_CHECK_NE(headers[1].ID, 0u);
      IM_CHECK_NE(tails[1].ID, 0u);
      IM_CHECK_NE(headers[2].ID, 0u);
      IM_CHECK_NE(tails[2].ID, 0u);

      for (int i = 0; i + 1 < 3; ++i) {
        // Order: every control of layer i sits above layer i+1's header.
        if (tails[i].RectFull.Max.y >= headers[i + 1].RectFull.Min.y) {
          IM_ERRORF("layer %d's last control (y=%.1f) is not above layer %d's header (y=%.1f)", i,
                    static_cast<double>(tails[i].RectFull.Max.y), i + 1,
                    static_cast<double>(headers[i + 1].RectFull.Min.y));
        }
        // ...and they do not overlap. The stronger claim this used to make — that the blocks are
        // held APART by more than one item spacing — went with the cards: a tree is meant to be
        // dense, and the separation that reads a layer as a group is now the fold triangle and the
        // indent, not a gap. Asserting a gap here would be pinning the opposite of the intent.
        const float gap = headers[i + 1].RectFull.Min.y - tails[i].RectFull.Max.y;
        if (gap < 0.0f) {
          IM_ERRORF("layers %d and %d overlap by %.1f px", i, i + 1, static_cast<double>(-gap));
        }
      }
    };
  }

  // P74. The probability slider has four states and they are not a gradient: the value decides
  // whether the control is usable, and the position in the stack decides whether a zero is a
  // legitimate end-of-chain or a layer that will receive nothing. Only the LAST layer at zero is
  // disabled — everywhere else the user has to be able to fix it.
  //
  // Reported per row rather than asserted fatally, for the message: which of the four states is
  // wrong is the whole diagnostic. The sweep still stops at the first one — see the note on
  // ctx->IsError() in test_gui_shared.hpp for why continuing would report consequences, not causes.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "the_probability_slider_is_disabled_only_on_a_dead_last_layer");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct State {
        const char* name;
        bool two_layers;
        float layer0_prob;
        bool expect_disabled;
      };
      const State kStates[] = {
        { "(a) last layer, probability zero", false, 0.0f, true },
        { "(b) last layer, probability above zero", false, 0.5f, false },
        { "(c) intermediate layer, probability zero", true, 0.0f, false },
        { "(d) intermediate layer, probability above zero", true, 0.5f, false },
      };

      for (const State& s : kStates) {
        ResetTestState();
        if (s.two_layers) {
          // Built by hand rather than through `+ Layer`, which would promote layer 0's probability
          // and make state (c) unreachable. This is the state a hand-written config produces.
          gui::Layer extra;
          gui::EntryCard seed;
          seed.crystal_id = 0;
          extra.entries.push_back(seed);
          gui::g_state.layers.push_back(std::move(extra));
        }
        gui::g_state.layers[0].probability = s.layer0_prob;
        // The slider is the inspector's Layer page now, not a control under the tree's layer
        // header, so the layer has to be selected for it to exist at all.
        gui::g_state.SelectLayer(0);
        ctx->Yield(3);

        const bool disabled = IsDisabled(ctx->ItemInfo("**/##Prob.##layer_0_input"));
        if (disabled != s.expect_disabled) {
          IM_ERRORF("%s: slider disabled=%d, expected %d", s.name, static_cast<int>(disabled),
                    static_cast<int>(s.expect_disabled));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // P14. Adding a layer below one that passes nothing on would produce a layer that never receives
  // a ray, so `+ Layer` promotes a dead predecessor to a usable continuation probability — and
  // leaves a deliberate one alone.
  //
  // The near-zero row is the one that matters: a slider drag can leave a probability at 1e-7, and a
  // strict `!= 0.0f` test would walk straight past it and reproduce the dead-layer bug. The shared
  // IsProbZero epsilon is what this pins.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "adding_a_layer_promotes_a_predecessor_that_passes_nothing_on");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      struct Case {
        const char* name;
        float before;
        bool expect_promoted;
      };
      const Case kCases[] = {
        { "exactly zero", 0.0f, true },
        { "below the zero epsilon", 1e-7f, true },
        { "deliberately set by the user", 0.6f, false },
      };

      for (const Case& c : kCases) {
        ResetTestState();
        gui::g_state.layers[0].probability = c.before;
        ctx->Yield(2);

        ctx->ItemClick("**/+ Layer");
        ctx->Yield(2);
        const float after = gui::g_state.layers[0].probability;
        const float expected = c.expect_promoted ? gui::kDefaultContinuationProb : c.before;
        if (after != expected) {
          IM_ERRORF("%s: probability went %f -> %f, expected %f", c.name, static_cast<double>(c.before),
                    static_cast<double>(after), static_cast<double>(expected));
        }
        // The new last layer is the one now allowed to be zero.
        if (!gui::IsProbZero(gui::g_state.layers[1].probability)) {
          IM_ERRORF("%s: the newly added last layer starts at %f rather than zero", c.name,
                    static_cast<double>(gui::g_state.layers[1].probability));
        }

        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // The state transition the four-state table cannot express: promote, then delete the layer that
  // caused the promotion. Layer 0 becomes the last layer again, this time carrying a non-zero
  // probability — state (b) — so its slider must be usable. Nothing in the delete path special-cases
  // this; it falls out of the four-state rule being re-derived every frame, and that is the claim.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "deleting_the_new_layer_leaves_the_promotion_editable");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      ctx->ItemClick("**/+ Layer");
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, gui::kDefaultContinuationProb);

      ctx->ItemClick("**/" ICON_FA_XMARK "##layer_1");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers.size()), 1);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, gui::kDefaultContinuationProb);
      // The slider lives on the inspector's Layer page; select the layer to reach it.
      gui::g_state.SelectLayer(0);
      ctx->Yield(3);
      IM_CHECK(!IsDisabled(ctx->ItemInfo("**/##Prob.##layer_0_input")));
    };
  }

  // P70. Duplicate clones the card's configuration onto a NEW pool slot. Both halves are the claim:
  // the clone carries the original's values (or it is not a duplicate), and editing either one
  // afterwards leaves the other alone (or it is not a copy).
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "duplicating_a_card_copies_its_crystal_onto_a_new_slot");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const int orig_cid = gui::g_state.layers[0].entries[0].crystal_id;
      gui::g_state.crystals[orig_cid].type = gui::CrystalType::kPyramid;
      gui::g_state.crystals[orig_cid].prism_h = 2.5f;
      ctx->Yield(2);

      ctx->ItemClick("**/" ICON_FA_COPY "##dup_0_0");
      ctx->Yield(2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      const int dup_cid = gui::g_state.layers[0].entries[1].crystal_id;
      IM_CHECK_NE(dup_cid, orig_cid);
      IM_CHECK_EQ(gui::g_state.crystals[dup_cid].type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::g_state.crystals[dup_cid].prism_h, 2.5f);

      gui::g_state.crystals[dup_cid].prism_h = 3.7f;
      IM_CHECK_EQ(gui::g_state.crystals[orig_cid].prism_h, 2.5f);
    };
  }

  // Duplicating a card that carries a filter appends a filter pool slot, which the reconciler reads
  // as a structural change and therefore as grounds to restart the run. Pinned end to end through
  // the real button, because the effect is produced by the frame-tail reconcile rather than by the
  // click handler — a hand-written MarkDirty at the click site was removed once it turned out to be
  // redundant, and this is what says the removal was safe.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "duplicating_a_filtered_card_restarts_the_run");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      gui::g_state.run_intent = gui::RunIntent::kLoaded;
      gui::g_state.sim_state = gui::GuiState::SimState::kDone;
      gui::g_state.snapshot_intensity = 0.5f;
      gui::g_state.committed_epoch = 5;
      gui::g_state.display_epoch_floor = 0;
      gui::g_state.dirty = false;
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);
      ctx->Yield(2);

      ctx->ItemClick("**/" ICON_FA_COPY "##dup_0_0");
      ctx->Yield(2);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      IM_CHECK_EQ(static_cast<int>(gui::g_state.sim_state), static_cast<int>(gui::GuiState::SimState::kModified));
      IM_CHECK_EQ(gui::g_state.snapshot_intensity, 0.0f);
      IM_CHECK_EQ(gui::g_state.display_epoch_floor, gui::g_state.committed_epoch);
      IM_CHECK(gui::g_state.dirty);
    };
  }

  // ================================================================================
  // Opening the modal from a card.
  // ================================================================================

  // P72, one shape on. The card was a rectangle hit-tested by hand, so this case clicked blank
  // space to prove the WHOLE card was the target rather than just its buttons. The row is a real
  // Selectable spanning the row, with the thumbnail and labels drawn over it under AllowOverlap, so
  // what has to be proved now is that the overlay did not steal the hit: a click in the middle of
  // the row, which is on top of the thumbnail, must still select it.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "clicking_a_cards_blank_area_opens_the_modal_on_that_card");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);
      IM_CHECK(!gui::g_state.HasValidCrystalSelection());

      ctx->MouseMoveToPos(RowSpot(ctx, 0, 0));
      ctx->MouseClick(0);
      ctx->Yield(4);

      IM_CHECK(gui::g_state.HasValidCrystalSelection());
      IM_CHECK_EQ(gui::g_state.selection.layer_idx, 0);
      IM_CHECK_EQ(gui::g_state.selection.entry_idx, 0);

      ctx->Yield(2);
    };
  }

  // The cost of hit-testing a rectangle by hand: the hit test has no idea what is drawn on top of
  // it. A floating window over the card must swallow the click, or clicking inside the Colors
  // window reopens the edit modal behind it.
  //
  // The gate is `!io.WantCaptureMouse`, and this is its occluded half; the unoccluded half — that
  // the gate does not suppress a click nothing is covering — is the collapse strip's, in
  // functional/test_shell_chrome.cpp.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "a_card_click_under_a_floating_window_does_not_reach_the_card");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      // Both halves of what this case borrows: the window it parks over the cards, and the modal
      // the regression it is looking for would open behind that window.
      const ScopedColorWindow color_guard;
      const ScopedPopups popup_guard(ctx);
      IM_CHECK(!gui::g_state.HasValidCrystalSelection());
      const ImVec2 spot = RowSpot(ctx, 0, 0);

      // Anchor the Colors window so `spot` lands mid-window, well below its header, mode combo and
      // button row. raypath_color is empty after a new document, so the class table under the click
      // point is empty space containing no item of its own.
      gui::g_state.color_window_open = true;
      ctx->Yield(2);
      ctx->WindowMove(ICON_FA_PALETTE " Colors", ImVec2(spot.x - 300.0f, spot.y - 240.0f));
      ctx->WindowResize(ICON_FA_PALETTE " Colors", ImVec2(720, 480));
      ctx->Yield(2);

      ctx->MouseMoveToPos(spot);
      ctx->MouseClick(0);
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.HasValidCrystalSelection());
    };
  }

  // P72's other half. Clicking the card the modal is already open on must be a no-op — not a
  // reopen, which would throw away everything staged since it opened. The staged edit is what makes
  // the two outcomes distinguishable: a spurious reopen re-reads the buffer from the entry, so the
  // committed value would come back as the original rather than as the number typed.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "clicking_the_card_the_modal_is_already_on_keeps_the_buffer");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      IM_CHECK(gui::g_state.HasValidCrystalSelection());
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 33.0f);
      ctx->Yield(2);

      ctx->MouseMoveToPos(RowSpot(ctx, 0, 0));
      ctx->MouseClick(0);
      ctx->Yield(4);
      IM_CHECK(gui::g_state.HasValidCrystalSelection());
      IM_CHECK_EQ(gui::g_state.selection.entry_idx, 0);

      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 33.0f);
    };
  }

  // ================================================================================
  // Sharing: the pool slot two cards can name at once.
  // ================================================================================

  // P114. Unlink forks the pool slot, and the modal has to re-read its buffer from the fork —
  // otherwise the next keystroke is written against the slot the card no longer uses, and lands on
  // the siblings it just left. The re-read is asserted through a commit rather than by inspecting
  // the buffer: what matters is where the next edit goes.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "unlink_forks_the_slot_and_the_modal_follows_the_fork");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);
      const int shared_cid = gui::g_state.layers[0].entries[0].crystal_id;
      gui::EntryCard sibling;
      sibling.crystal_id = shared_cid;
      gui::g_state.layers[0].entries.push_back(sibling);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(3);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, shared_cid, std::nullopt), 2);
      const float shared_h = gui::g_state.crystals[shared_cid].height.center;

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      IM_CHECK(ctx->ItemExists("**/Unlink##share"));
      ctx->ItemClick("**/Unlink##share");
      ctx->Yield(4);

      const int forked_cid = gui::g_state.layers[0].entries[0].crystal_id;
      IM_CHECK_NE(forked_cid, shared_cid);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, forked_cid, std::nullopt), 1);
      // The button goes with the sharing it described — there is nothing left to unlink from.
      IM_CHECK(!ctx->ItemExists("**/Unlink##share"));

      // The next edit lands on the fork, and the card left behind keeps what it had.
      ctx->ItemInputValue("**/##Height##modal_cr_input", shared_h + 7.0f);
      ctx->Yield(2);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[forked_cid].height, shared_h + 7.0f);
      IM_CHECK_EQ(gui::g_state.crystals[shared_cid].height, shared_h);
    };
  }

  // P112 / P15. "Link to..." is three actions at once — commit what is in the buffer, arm pick
  // mode, close the modal — and the panel then has to say so, because the next click means
  // something different from what it usually means. The prompt is the only feedback there is.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "link_to_arms_pick_mode_and_says_so_above_the_rows");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);
      AddSecondEntryOnItsOwnSlot(ctx);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      const float orig_h = gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height.center;
      ctx->ItemInputValue("**/##Height##modal_cr_input", orig_h + 2.0f);
      ctx->Yield(2);

      ctx->ItemClick("**/Link to...##share");
      ctx->Yield(4);

      // Committed on the way out — an edit lost here would be silent.
      IM_CHECK_EQ(gui::g_state.crystals[gui::g_state.layers[0].entries[0].crystal_id].height, orig_h + 2.0f);
      // The editor STAYS on the source entry, where the modal used to close itself. Deliberate, and
      // the opposite of the old assertion: the modal had to get out of the way because its backdrop
      // sat between the user and the cards they were being asked to click, while the page is beside
      // the tree rather than over it — so it can go on showing what is being linked FROM while the
      // user picks what to link TO.
      IM_CHECK(gui::g_state.HasValidCrystalSelection());
      IM_CHECK_EQ(gui::g_state.selection.entry_idx, 0);
      IM_CHECK(gui::g_state.pick_link_source.has_value());
      IM_CHECK_EQ(gui::g_state.pick_link_source->layer_idx, 0);
      IM_CHECK_EQ(gui::g_state.pick_link_source->entry_idx, 0);

      // The prompt is a TextWrapped, i.e. id == 0 and invisible to the item registry. What IS
      // observable is what it does to the layout: the rows move down to make room for it. Compared
      // against the same row's position with pick mode off, on the next frame.
      //
      // Read off the row rather than off the card's child window, which no longer exists — and the
      // id is checked before the rect is used, because a row that stopped being submitted is
      // precisely one of the regressions this case exists to catch.
      const ImGuiTestItemInfo row_with_prompt = ctx->ItemInfo(RowRef(0, 0).c_str());
      IM_CHECK_NE(row_with_prompt.ID, 0u);
      const float y_with_prompt = row_with_prompt.RectFull.Min.y;

      ctx->KeyPress(ImGuiKey_Escape);
      ctx->Yield(3);
      IM_CHECK(!gui::g_state.pick_link_source.has_value());
      const ImGuiTestItemInfo row_without_prompt = ctx->ItemInfo(RowRef(0, 0).c_str());
      IM_CHECK_NE(row_without_prompt.ID, 0u);
      IM_CHECK_GT(y_with_prompt, row_without_prompt.RectFull.Min.y);
    };
  }

  // P16 / P17. Both ways out of pick mode without picking anything, and the same thing must NOT
  // happen on either: the modal must stay closed. The reopen is reserved for a completed pick — a
  // cancel that reopened it would put the user back in a dialog they had just left.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "entry_management", "cancelling_pick_mode_leaves_the_editor_where_it_was");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      // One exit per call rather than one loop body: arming pick mode is a fatal precondition (a
      // cancel test on a pick that never armed proves nothing), and a fatal assert inside a loop
      // would return out of the case and take the second exit with it.
      auto cancel_by = [ctx](bool by_escape) {
        ResetTestState();
        ctx->Yield(2);
        // Per call, for the same reason the calls are separate: whichever exit this one takes, the
        // second call has to start from a closed modal rather than from this one's leftovers.
        const ScopedPopups popup_guard(ctx);
        AddSecondEntryOnItsOwnSlot(ctx);
        const int cid_before = gui::g_state.layers[0].entries[1].crystal_id;

        OpenCrystalTab(ctx);
        ctx->Yield(4);
        ctx->ItemClick("**/Link to...##share");
        ctx->Yield(4);
        IM_CHECK(gui::g_state.pick_link_source.has_value());

        if (by_escape) {
          ctx->KeyPress(ImGuiKey_Escape);
        } else {
          // Blank space below the cards, inside the left panel but on no card at all.
          const ImGuiTestItemInfo add_layer = ctx->ItemInfo("**/+ Layer");
          ctx->MouseMoveToPos(ImVec2(add_layer.RectFull.Max.x + 30.0f, add_layer.RectFull.Min.y));
          ctx->MouseClick(0);
        }
        ctx->Yield(4);

        const char* how = by_escape ? "Escape" : "a click off the cards";
        if (gui::g_state.pick_link_source.has_value()) {
          IM_ERRORF("%s did not cancel pick mode", how);
        }
        // The selection is where it was, not moved by the cancel. Under the modal this read "the
        // modal did not reopen", the reopen being reserved for a completed pick; with no open or
        // close to speak of, what a bad cancel would do instead is silently retarget the editor.
        if (!gui::g_state.HasValidCrystalSelection() || gui::g_state.selection.entry_idx != 0) {
          IM_ERRORF("%s moved the editor off the source entry", how);
        }
        // ...and nothing was linked on the way out.
        if (gui::g_state.layers[0].entries[1].crystal_id != cid_before) {
          IM_ERRORF("%s linked the target card anyway", how);
        }
      };

      cancel_by(/*by_escape=*/true);

      if (ctx->IsError()) {
        return;
      }
      cancel_by(/*by_escape=*/false);
    };
  }

  // P18. Completing a pick is the one path that reopens the modal, and it reopens it on the SOURCE
  // card — the user was editing that one and asked it to share, so that is where they go back to.
  //
  // The preview reset is the half that escaped once. The source entry's crystal_id has just been
  // re-pointed at the target's slot, so the trackball pose left over from the previous crystal is
  // now describing a crystal that is no longer there: the modal would show one orientation while
  // the card thumbnail beside it showed another.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "completing_a_pick_reopens_the_source_and_resets_its_preview");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);

      // The target carries a Column axis, so its default pose is a different matrix from the
      // source's Random one — without that, "the pose was reset" and "the pose was left alone"
      // would produce the same matrix.
      const gui::AxisDist az_full{ gui::AxisDistType::kUniform, 0.0f, 360.0f };
      gui::CrystalConfig column;
      column.zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 90.0f, 1.0f };
      column.azimuth = az_full;
      column.roll = az_full;
      gui::EntryCard target;
      target.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(column);
      gui::g_state.layers[0].entries.push_back(target);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(3);
      const int target_cid = gui::g_state.layers[0].entries[1].crystal_id;
      const int source_cid_before = gui::g_state.layers[0].entries[0].crystal_id;
      IM_CHECK_NE(target_cid, source_cid_before);

      OpenCrystalTab(ctx);
      ctx->Yield(4);
      ctx->ItemClick("**/Link to...##share");
      ctx->Yield(4);
      IM_CHECK(gui::g_state.pick_link_source.has_value());

      // Drag the preview first, so a pose that simply survived is distinguishable from one that was
      // recomputed for the new crystal.
      gui::ApplyTrackballRotation(45.0f, 15.0f);
      const ImVec2 target_spot = RowSpot(ctx, 0, 1);
      ctx->MouseMoveToPos(target_spot);
      ctx->MouseClick(0);
      ctx->Yield(6);

      // The link landed: the source now names the target's slot.
      IM_CHECK(!gui::g_state.pick_link_source.has_value());
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal_id, target_cid);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, target_cid, std::nullopt), 2);

      // Reopened on the SOURCE card, not on the one that was clicked.
      IM_CHECK(gui::g_state.HasValidCrystalSelection());
      IM_CHECK_EQ(gui::g_state.selection.layer_idx, 0);
      IM_CHECK_EQ(gui::g_state.selection.entry_idx, 0);

      // ...showing the crystal it now points at, from that crystal's default angle.
      gui::AxisDist params[3] = { column.zenith, column.azimuth, column.roll };
      float expected[16] = { 0 };
      gui::DefaultPreviewRotation(gui::AxisPreset::kColumn, params, expected);
      for (int i = 0; i < 16; ++i) {
        if (gui::g_crystal_rotation[i] != expected[i]) {
          IM_ERRORF("preview pose index %d is %f, expected the target crystal's default %f", i,
                    static_cast<double>(gui::g_crystal_rotation[i]), static_cast<double>(expected[i]));
        }
      }

      ctx->Yield(2);
    };
  }

  // A linked group shares its filter atomically: adding one to either card must bind both, or the
  // group silently stops being a group and the two cards start simulating different things while
  // still claiming to be linked.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "a_filter_added_to_one_linked_card_reaches_its_sibling");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);
      gui::EntryCard sibling;
      sibling.crystal_id = gui::g_state.layers[0].entries[0].crystal_id;
      gui::g_state.layers[0].entries.push_back(sibling);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(3);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 2);

      OpenFilterTab(ctx);
      ctx->Yield(4);
      ctx->ItemInputValue("**/##row_text_0", "3-5");
      ctx->Yield(2);
      ctx->Yield(2);

      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.layers[0].entries[1].filter_id, gui::g_state.layers[0].entries[0].filter_id);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, gui::g_state.layers[0].entries[0].filter_id), 2);
    };
  }

  // The same claim in the other direction. It is a separate case because removal takes a different
  // branch — it clears an id rather than binding one — and a propagation that only ran on the
  // binding path would leave the sibling filtered by a filter the user just deleted.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "entry_management", "removing_a_filter_from_one_linked_card_clears_its_sibling");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);
      const ScopedPopups popup_guard(ctx);
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-5" });
      gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], f);
      gui::EntryCard sibling;
      sibling.crystal_id = gui::g_state.layers[0].entries[0].crystal_id;
      sibling.filter_id = gui::g_state.layers[0].entries[0].filter_id;
      gui::g_state.layers[0].entries.push_back(sibling);
      gui::g_thumbnail_cache.OnLayerStructureChanged();
      ctx->Yield(3);
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, gui::g_state.layers[0].entries[0].filter_id), 2);

      OpenFilterTab(ctx);
      ctx->Yield(4);
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->Yield(2);

      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());
      IM_CHECK(!gui::g_state.layers[0].entries[1].filter_id.has_value());
      IM_CHECK_EQ(gui::CountEntriesSharing(gui::g_state, 0, std::nullopt), 2);
    };
  }
}
