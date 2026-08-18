// The document column: a tree of everything the document contains, above an inspector that edits
// whichever item the tree names (doc/gui-layout-architecture.md §2).
//
// What this suite is for. The column's two halves are one proposition split in two windows, and
// almost everything that can go wrong with it is a property of a rendered frame: whether both
// halves exist as separate dock nodes, whether each scrolls without dragging the other, whether the
// separator between them moves, whether folding one hands its height to the other, and whether a
// click in the tree reaches the right page of the inspector. None of that is expressible without a
// live layout, which is why it is here and not in composition-correctness.
//
// Deliberately NOT here, with where each lives instead. The whole-column collapse (the top bar's
// chevron and the strip that brings it back) is chrome shared with the right panel, and stays in
// functional/test_shell_chrome.cpp. What the inspector's crystal page CONTAINS — every shape
// scalar, the axis distributions, the filter's sum-of-products rows — is
// functional/test_edit_modal.cpp and functional/test_filter_editor.cpp, which drive those controls
// through the selection now that the modal is gone. Adding and deleting entries is
// functional/test_entry_management.cpp.
//
// What a user sees when these break: a tree they cannot scroll without the editor below jumping, a
// separator that will not move, or an inspector still showing the crystal they just stopped
// looking at.

#include <string>

#include "IconsFontAwesome6.h"
#include "gui/dock_layout.hpp"
#include "gui/gui_constants.hpp"
#include "gui/panels.hpp"
#include "gui/theme.hpp"
#include "imgui_internal.h"
#include "test_gui_shared.hpp"

namespace {

// A layer/entry shape big enough that the tree must scroll: `layers` layers of `entries` crystals
// each. Every entry gets its own pool slot, matching what "+ Crystal" does, so nothing is
// accidentally shared.
void BuildScene(int layers, int entries) {
  auto& s = gui::g_state;
  s.layers.clear();
  s.crystals.clear();
  s.filters.clear();
  for (int li = 0; li < layers; ++li) {
    gui::Layer layer;
    for (int ei = 0; ei < entries; ++ei) {
      gui::EntryCard entry;
      entry.crystal_id = static_cast<int>(s.crystals.size());
      s.crystals.emplace_back();
      layer.entries.push_back(entry);
    }
    // Every layer but the last passes rays on, so the scene is one the simulator would accept
    // rather than a shape that only exists in this test.
    layer.probability = (li == layers - 1) ? 0.0f : 0.8f;
    s.layers.push_back(std::move(layer));
  }
  s.SelectNone();
  gui::g_thumbnail_cache.OnLayerStructureChanged();
}

}  // namespace

void RegisterDocumentColumnTests(ImGuiTestEngine* engine) {
  // The column is two dock nodes, not one window with a hand-rolled splitter. Everything else in
  // this file rests on that, and it is the one thing no other case would notice losing: fold the
  // pair back into a single window and the tree and inspector still render, still scroll together,
  // and every content assertion still passes.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "document_column", "the_tree_and_the_inspector_are_two_nodes_in_one_column");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      const gui::DockPanelNodeIds& ids = gui::GetPanelNodeIds();
      IM_CHECK_NE(ids.document_tree, 0u);
      IM_CHECK_NE(ids.document_inspector, 0u);
      IM_CHECK_NE(ids.document_tree, ids.document_inspector);
      // Both are children of the same parent, and that parent is the node the whole-column
      // collapse resizes. Checking the parentage rather than just "two ids exist" is what rules
      // out the pair having been split off the dockspace root as two independent columns.
      IM_CHECK_NE(ids.left, 0u);
      const ImGuiDockNode* tree_node = ImGui::DockBuilderGetNode(ids.document_tree);
      const ImGuiDockNode* inspector_node = ImGui::DockBuilderGetNode(ids.document_inspector);
      IM_CHECK(tree_node != nullptr);
      IM_CHECK(inspector_node != nullptr);
      IM_CHECK(tree_node->ParentNode != nullptr);
      IM_CHECK(inspector_node->ParentNode != nullptr);
      IM_CHECK_EQ(tree_node->ParentNode->ID, ids.left);
      IM_CHECK_EQ(inspector_node->ParentNode->ID, ids.left);

      // Stacked, not side by side, and the tree is the one on top.
      IM_CHECK_EQ(tree_node->Pos.x, inspector_node->Pos.x);
      IM_CHECK_LT(tree_node->Pos.y, inspector_node->Pos.y);
      IM_CHECK_EQ(tree_node->Size.x, gui::kLeftPanelWidth);
      IM_CHECK_EQ(inspector_node->Size.x, gui::kLeftPanelWidth);

      ImGuiWindow* tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
    };
  }

  // AC3, part 1: each half scrolls on its own. The failure this rules out is not "scrolling is
  // broken" — it is the two halves sharing one scroll region, which looks fine until the tree is
  // long enough to push the inspector off the bottom of the column.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "each_half_scrolls_without_moving_the_other");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      // Enough rows that the tree overflows its half of the column, and a crystal selected so the
      // inspector below is showing its tallest page rather than the empty-state line.
      BuildScene(4, 6);
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(4);

      ImGuiWindow* tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      const float inspector_y_before = inspector->Pos.y;
      const float inspector_h_before = inspector->Size.y;

      // The scroller is the tree's CHILD, not the tree window: the tree window is NoScrollbar and
      // sizes the child to its own content region, so its own ScrollMax is zero and asking it to
      // scroll does nothing at all. Scrolling the wrong window here is not a smaller version of
      // this test — it is a test that cannot fail, because the assertions below all hold when
      // nothing moved.
      ImGuiWindow* scroller = ctx->WindowInfo(kTreeScrollRef).Window;
      IM_CHECK(scroller != nullptr);
      IM_CHECK_GT(scroller->ScrollMax.y, 0.0f);  // premise: the tree really does overflow its half
      ctx->ScrollToBottom(scroller->ID);
      ctx->Yield(3);

      // The tree scrolled...
      scroller = ctx->WindowInfo(kTreeScrollRef).Window;
      IM_CHECK(scroller != nullptr);
      IM_CHECK_GT(scroller->Scroll.y, 0.0f);
      // ...and the inspector did not move or resize because of it. A shared scroll region would
      // have slid the inspector up as the tree's content advanced.
      inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_EQ(inspector->Pos.y, inspector_y_before);
      IM_CHECK_EQ(inspector->Size.y, inspector_h_before);

      // And the reverse: scrolling the inspector leaves the tree where it is.
      const float tree_scroll = scroller->Scroll.y;
      ctx->ScrollToBottom(gui::kDocumentInspectorWindowName);
      ctx->Yield(3);
      scroller = ctx->WindowInfo(kTreeScrollRef).Window;
      IM_CHECK(scroller != nullptr);
      IM_CHECK_EQ(scroller->Scroll.y, tree_scroll);

      ctx->ScrollToTop(scroller->ID);
      ctx->ScrollToTop(gui::kDocumentInspectorWindowName);
      ctx->Yield(2);
    };
  }

  // AC3, part 2: the separator between the two halves is draggable, and the drag is reversible.
  // Both directions matter — a resize that quantises differently each way leaves the user unable
  // to get their proportions back — and the return trip is also how this case cleans up after
  // itself, since the dock layout outlives ResetTestState and every later case (including the
  // reference-image ones) would otherwise run at whatever split this left behind.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "dragging_the_separator_repartitions_the_column");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);

      ImGuiWindow* tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      const float tree_h_before = tree->Size.y;
      const float inspector_h_before = inspector->Size.y;
      const float inspector_y_before = inspector->Pos.y;

      // The separator sits in the gap immediately below the tree; aim at its middle, which is half
      // of style.DockingSeparatorSize past the tree's bottom edge. Same derivation the left
      // splitter's case in test_shell_chrome.cpp uses, one axis over.
      const float seam_x = tree->Pos.x + tree->Size.x * 0.5f;
      const float seam_y = tree->Pos.y + tree->Size.y + ImGui::GetStyle().DockingSeparatorSize * 0.5f;
      constexpr float kDragBy = 50.0f;

      ctx->MouseMoveToPos(ImVec2(seam_x, seam_y));
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(ImVec2(seam_x, seam_y + kDragBy));
      ctx->MouseUp(0);
      ctx->Yield(3);

      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      // Not an exact height: the separator lands where the pointer left it and ImGui truncates the
      // resulting node sizes. What must be true is that the drag moved it, the way it was dragged,
      // and that the column as a whole did not grow — the two halves traded height.
      IM_CHECK_GT(tree->Size.y, tree_h_before);
      IM_CHECK_LT(inspector->Size.y, inspector_h_before);
      IM_CHECK_GT(inspector->Pos.y, inspector_y_before);

      ctx->MouseMoveToPos(ImVec2(seam_x, seam_y + kDragBy));
      ctx->MouseDown(0);
      ctx->MouseMoveToPos(ImVec2(seam_x, seam_y));
      ctx->MouseUp(0);
      ctx->Yield(3);

      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_EQ(tree->Size.y, tree_h_before);
      IM_CHECK_EQ(inspector->Size.y, inspector_h_before);
    };
  }

  // AC3, part 3: folding a half at its section header gives the height to the OTHER half, and
  // unfolding gives it back. This is the behaviour that distinguishes the pair-in-one-column from
  // two independent panels — the column's total height does not change, the split inside it does.
  //
  // Driven through the header row's own click rather than by setting the flags, because the header
  // is the mechanism under test: it is a Selectable whose visible text changes every time the
  // selection retitles the page, and the `###` id that keeps it one control across those retitles
  // is exactly the kind of thing that fails silently.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "folding_a_half_hands_its_height_to_the_other");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(2, 3);
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(4);

      ImGuiWindow* tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      const float tree_h_before = tree->Size.y;
      const float inspector_h_before = inspector->Size.y;
      const float column_h_before = tree_h_before + inspector_h_before;
      IM_CHECK_GT(inspector_h_before, 0.0f);  // premise: there is height to hand over

      // Fold the tree. Its node becomes the header strip; the inspector takes the rest.
      ctx->ItemClick("**/###tree_fold");
      ctx->Yield(4);
      IM_CHECK(gui::g_state.document_tree_folded);
      IM_CHECK(!gui::g_state.document_inspector_folded);
      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_LT(tree->Size.y, tree_h_before);
      IM_CHECK_GT(inspector->Size.y, inspector_h_before);
      // The column did not grow to make room — the height came from the other half. A few pixels
      // of slack for the separator and ImGui's integer node sizes.
      IM_CHECK_LT(ImFabs((tree->Size.y + inspector->Size.y) - column_h_before), 4.0f);

      // Unfold, and the split comes back to where it was rather than to some default.
      ctx->ItemClick("**/###tree_fold");
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.document_tree_folded);
      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_EQ(tree->Size.y, tree_h_before);
      IM_CHECK_EQ(inspector->Size.y, inspector_h_before);

      // The other direction, and the invariant that only one half is ever folded: folding the
      // inspector while the tree is folded must leave exactly one of them folded, not both.
      gui::g_state.FoldDocumentHalves(/*tree_folded=*/true, /*inspector_folded=*/false);
      ctx->Yield(3);
      ctx->ItemClick("**/###inspector_fold");
      ctx->Yield(4);
      IM_CHECK(gui::g_state.document_inspector_folded);
      IM_CHECK(!gui::g_state.document_tree_folded);
      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK_GT(tree->Size.y, tree_h_before);

      // Put the column back for the cases after this one: the dock layout outlives ResetTestState.
      ctx->ItemClick("**/###inspector_fold");
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.document_inspector_folded);
    };
  }

  // The two collapses compose: the column's own (a width, handled in test_shell_chrome.cpp) and a
  // half's fold (a height). Combination states are where this layout has already produced bugs
  // once — the substrate task's ini-restore defects were both "each piece works, the pair does
  // not" — and the specific risk here is that the fold is written to a node whose id the column
  // collapse leaves untouched, so nothing re-asserts it on the way back.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "a_folded_half_survives_the_whole_column_collapsing");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(2, 3);
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(4);

      gui::g_state.FoldDocumentHalves(/*tree_folded=*/true, /*inspector_folded=*/false);
      ctx->Yield(4);
      ImGuiWindow* tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      IM_CHECK(tree != nullptr);
      const float folded_tree_h = tree->Size.y;

      gui::g_state.left_panel_collapsed = true;
      ctx->Yield(4);
      gui::g_state.left_panel_collapsed = false;
      ctx->Yield(4);

      // Still folded, and still folded to the same strip — not silently re-expanded, and not
      // collapsed a second time onto an already-folded node.
      IM_CHECK(gui::g_state.document_tree_folded);
      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      ImGuiWindow* inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(tree != nullptr);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_EQ(tree->Size.y, folded_tree_h);
      IM_CHECK_GT(inspector->Size.y, folded_tree_h);
      // And the column is back to its full width, i.e. the fold did not interfere with the
      // whole-column collapse it was nested inside.
      IM_CHECK_EQ(tree->Size.x, gui::kLeftPanelWidth);

      gui::g_state.FoldDocumentHalves(false, false);
      ctx->Yield(3);
    };
  }

  // The tree's rows are what pick mode is waiting to be clicked, so arming it has to put them on
  // screen. Folded away, "Link to..." would arm a mode whose only exit is Esc.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "arming_pick_mode_unfolds_the_tree");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 2);
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);

      gui::g_state.FoldDocumentHalves(/*tree_folded=*/true, /*inspector_folded=*/false);
      ctx->Yield(3);
      IM_CHECK(gui::g_state.document_tree_folded);
      IM_CHECK(!ctx->ItemExists("**/##row_0_1"));  // premise: the rows really are away

      gui::g_state.pick_link_source = gui::GuiState::EntryRef{ 0, 0 };
      ctx->Yield(3);
      IM_CHECK(!gui::g_state.document_tree_folded);
      IM_CHECK(ctx->ItemExists("**/##row_0_1"));  // the target the user was told to click

      gui::g_state.pick_link_source.reset();
      ctx->Yield(2);
    };
  }

  // Clicking a row is what selects it, and the inspector follows. Asserted through the real click
  // rather than by writing g_state.selection, because the click path is the half that can break on
  // its own: the row is a Selectable with the thumbnail, the labels and two buttons drawn over it,
  // so "the row is still what gets hit" is a claim about that overlap, not about the selection
  // model.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "clicking_a_row_selects_it_and_the_inspector_follows");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(2, 3);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.selection.kind, gui::GuiState::SelectionKind::kNone);

      // The last row of a scene built to overflow the tree's half — see test_gui_shared.hpp for why
      // ItemClick cannot reach it on its own.
      IM_CHECK(ScrollTreeTo(ctx, "**/##row_1_2"));
      ctx->ItemClick("**/##row_1_2");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.selection.kind, gui::GuiState::SelectionKind::kCrystal);
      IM_CHECK_EQ(gui::g_state.selection.layer_idx, 1);
      IM_CHECK_EQ(gui::g_state.selection.entry_idx, 2);
      // ...and the inspector is showing that entry's editor, not the empty state.
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));
      IM_CHECK(ctx->ItemExists("**/###axis_tab"));
      IM_CHECK(ctx->ItemExists("**/###filter_tab"));

      // The two singleton rows answer the same way, and each replaces the previous page. They sit
      // at the TOP of the tree, which the scroll above left at the bottom — hence the same helper
      // again rather than a bare click.
      IM_CHECK(ScrollTreeTo(ctx, "**/" ICON_FA_SUN " Sun"));
      ctx->ItemClick("**/" ICON_FA_SUN " Sun");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.selection.kind, gui::GuiState::SelectionKind::kSun);
      IM_CHECK(!ctx->ItemExists("**/###crystal_tab"));
      IM_CHECK(ctx->ItemExists("**/##Altitude"));

      IM_CHECK(ScrollTreeTo(ctx, "**/" ICON_FA_CAMERA " Camera"));
      ctx->ItemClick("**/" ICON_FA_CAMERA " Camera");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.selection.kind, gui::GuiState::SelectionKind::kCamera);
      IM_CHECK(ctx->ItemExists("**/##FOV##view"));
    };
  }

  // AC2. The inspector has no confirm step: a typed value is in the document on the next frame.
  // This is the proposition the whole migration turns on — the modal offered this behaviour as one
  // of two modes, and what replaced it must not have quietly reverted to the other.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "an_edit_reaches_the_document_without_any_confirm");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 2);
      gui::g_state.SelectCrystal(0, 1);
      ctx->Yield(3);

      const int crystal_id = gui::g_state.layers[0].entries[1].crystal_id;
      const float before = gui::g_state.crystals[crystal_id].height.center;
      ctx->ItemInputValue("**/##Height##modal_cr", before + 3.0f);
      ctx->Yield(2);

      // No OK, no Close, nothing dismissed — and the pool already has it.
      IM_CHECK_EQ(gui::g_state.crystals[crystal_id].height.center, before + 3.0f);
      IM_CHECK(ctx->ItemExists("**/###crystal_tab"));  // premise: the page never went away
    };
  }

  // task338's lesson, re-anchored on the mechanism that replaced the modal. The page is ONE
  // persistent window reused for every entry, so the thing that keeps an in-flight edit from
  // landing in the next entry is the per-(layer, entry) id scope around the tab bodies. Losing it
  // fails silently — the value simply appears under a crystal the user never edited.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "switching_rows_does_not_leak_the_previous_edit");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 2);
      const int id0 = gui::g_state.layers[0].entries[0].crystal_id;
      const int id1 = gui::g_state.layers[0].entries[1].crystal_id;
      IM_CHECK_NE(id0, id1);  // premise: separate pool slots, so a leak is observable

      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);
      const float base0 = gui::g_state.crystals[id0].height.center;
      const float base1 = gui::g_state.crystals[id1].height.center;
      ctx->ItemInputValue("**/##Height##modal_cr", base0 + 2.0f);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[id0].height.center, base0 + 2.0f);

      // Move to the other row WITHOUT dismissing anything — there is nothing to dismiss, which is
      // exactly the condition the modal never had to survive.
      ctx->ItemClick("**/##row_0_1");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.crystals[id1].height.center, base1);
      // The page is now showing entry 1's value, not entry 0's.
      IM_CHECK_EQ(gui::g_state.crystals[id0].height.center, base0 + 2.0f);

      // And an edit here lands here only.
      ctx->ItemInputValue("**/##Height##modal_cr", base1 + 5.0f);
      ctx->Yield(2);
      IM_CHECK_EQ(gui::g_state.crystals[id1].height.center, base1 + 5.0f);
      IM_CHECK_EQ(gui::g_state.crystals[id0].height.center, base0 + 2.0f);
    };
  }

  // The reload trigger that is NOT a selection change. Completing a pick rebinds the selected
  // entry to a different pool slot while the selection itself stays put, so a page that only
  // reloads when (layer, entry) changes goes on holding the old crystal — and its next per-frame
  // commit writes that old crystal straight over the one the user just linked to. The failure is
  // silent and destructive, which is why it gets its own case rather than a line in another.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "completing_a_pick_reloads_the_page_onto_the_new_slot");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 2);
      const int id0 = gui::g_state.layers[0].entries[0].crystal_id;
      const int id1 = gui::g_state.layers[0].entries[1].crystal_id;
      // Make the two slots tell each other apart by a value the page displays.
      gui::g_state.crystals[id1].height.center = gui::g_state.crystals[id0].height.center + 7.0f;
      const float model_h = gui::g_state.crystals[id1].height.center;

      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);
      ctx->ItemClick("**/Link to...##share");
      ctx->Yield(2);
      IM_CHECK(gui::g_state.pick_link_source.has_value());

      // Entry 0 adopts entry 1's slot.
      ctx->ItemClick("**/##row_0_1");
      ctx->Yield(4);
      IM_CHECK(!gui::g_state.pick_link_source.has_value());
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].crystal_id, id1);
      // The selection came back to the source entry, and — the point of the case — several frames
      // of per-frame commits later the shared slot still holds the model's value rather than
      // having been overwritten from a stale buffer.
      IM_CHECK_EQ(gui::g_state.selection.layer_idx, 0);
      IM_CHECK_EQ(gui::g_state.selection.entry_idx, 0);
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.crystals[id1].height.center, model_h);
    };
  }

  // The other reload trigger: the page reappearing. Anything that edits the document while the
  // crystal page is hidden — a load, the defaults panel — leaves the buffers describing a state
  // that is no longer the document's, and the first commit after the page comes back would put it
  // there. Deselecting and reselecting is the cheapest way to state that, and it is also the shape
  // a user hits by clicking Sun and then clicking back.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "the_page_rereads_the_entry_when_it_comes_back");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 1);
      const int id0 = gui::g_state.layers[0].entries[0].crystal_id;
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);

      // Away from the page, then change the document behind its back the way another panel would.
      gui::g_state.SelectSun();
      ctx->Yield(3);
      const float external = gui::g_state.crystals[id0].height.center + 4.0f;
      gui::g_state.crystals[id0].height.center = external;
      ctx->Yield(2);

      // Back to the page. It must adopt what the document says, not overwrite it.
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(4);
      IM_CHECK_EQ(gui::g_state.crystals[id0].height.center, external);
    };
  }

  // ---- Row meta: the dimmed secondary value at each row's right edge ----
  //
  // What these cases are for. The tree names what the document contains; the meta says where each
  // item is currently set, which is what lets a user check a value without clicking into the
  // inspector. The proposition that can break is "the row reads LIVE state", not "the format string
  // is right": a row that formats once and caches, or reads a stale copy, still looks correct in a
  // screenshot taken before the edit.
  //
  // How it is asserted, and why not directly. The meta is drawn with a bare TextDisabled, which
  // ImGui gives no item id, so ItemInfo cannot read the string back — the same constraint recorded
  // at the top of functional/test_status_bar.cpp. So the two halves are asserted separately: the
  // formatting functions are called directly (below), and the render sites are driven through the
  // real inspector controls with the observable consequence checked in the frame after.
  //
  // All four row kinds are driven through a real control here, not just their formatting function.
  //
  // What that leaves uncovered, stated rather than implied: none of these cases fails if the
  // TextDisabled call itself is deleted from the row, because nothing addressable sits downstream
  // of a string with no id. The one exception is the layer row, whose delete button IS positioned
  // relative to the meta and is asserted below. That the strings reach the screen at all is covered
  // by pixels instead — the tree is inside capture_harness/fullframe and visual/left_panel.
  //
  // The tier the meta is drawn AT, which is the other half of the same feature. TextDisabled reads
  // ImGuiCol_TextDisabled by construction, so what is worth asserting is not which call was made
  // but that the slot it reads is (a) a real second tier rather than the same colour as body text,
  // and (b) the theme's own value rather than a shade invented at some call site and pushed over
  // it. ApplyStyle into a scratch style is what makes (b) checkable without exporting the palette:
  // it is the same seam theme_coverage uses.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "the_dim_tier_is_the_themes_own_second_text_color");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield();

      const ImVec4 live_dim = ImGui::GetStyle().Colors[ImGuiCol_TextDisabled];
      const ImVec4 live_text = ImGui::GetStyle().Colors[ImGuiCol_Text];
      IM_CHECK(live_dim.x != live_text.x || live_dim.y != live_text.y || live_dim.z != live_text.z ||
               live_dim.w != live_text.w);

      ImGuiStyle scratch;
      gui::ApplyStyle(scratch);
      const ImVec4 themed_dim = scratch.Colors[ImGuiCol_TextDisabled];
      IM_CHECK_EQ(live_dim.x, themed_dim.x);
      IM_CHECK_EQ(live_dim.y, themed_dim.y);
      IM_CHECK_EQ(live_dim.z, themed_dim.z);
      IM_CHECK_EQ(live_dim.w, themed_dim.w);
    };
  }

  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "each_row_kind_formats_its_own_secondary_value");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 1);
      ctx->Yield();  // the tree these strings are drawn into must exist for the case to be about it

      // Each carries the value, and a different value produces a different string — the pair rules
      // out both "shows nothing" and "shows a constant".
      IM_CHECK(gui::FormatSunTreeMeta(20.0f).find("20.0") != std::string::npos);
      IM_CHECK_STR_NE(gui::FormatSunTreeMeta(20.0f).c_str(), gui::FormatSunTreeMeta(35.5f).c_str());

      IM_CHECK_STR_EQ(gui::FormatCameraTreeMeta(gui::kLensTypeLinear).c_str(), "Linear");
      IM_CHECK_STR_NE(gui::FormatCameraTreeMeta(gui::kLensTypeLinear).c_str(),
                      gui::FormatCameraTreeMeta(gui::kLensTypeRectangular).c_str());

      IM_CHECK(gui::FormatLayerTreeMeta(1.0f).find("1.00") != std::string::npos);
      IM_CHECK_STR_NE(gui::FormatLayerTreeMeta(1.0f).c_str(), gui::FormatLayerTreeMeta(0.25f).c_str());

      IM_CHECK(gui::FormatCrystalTreeMeta(100.0f).find("100") != std::string::npos);
      IM_CHECK_STR_NE(gui::FormatCrystalTreeMeta(100.0f).c_str(), gui::FormatCrystalTreeMeta(5.0f).c_str());
    };
  }

  // The Sun row, edited through the control a user would use. What is checked in the frame after is
  // that the DOCUMENT holds the new altitude, which is the value the row formats every frame — the
  // row cannot show the old one without also contradicting this.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "editing_sun_altitude_updates_the_tree_meta");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 1);
      IM_CHECK(ScrollTreeTo(ctx, "**/" ICON_FA_SUN " Sun"));
      ctx->ItemClick("**/" ICON_FA_SUN " Sun");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.selection.kind, gui::GuiState::SelectionKind::kSun);

      const float before = gui::g_state.sun.altitude;
      const std::string meta_before = gui::FormatSunTreeMeta(before);
      const float after = before + 12.5f;
      ctx->ItemInputValue("**/##sun_props/##Altitude", after);
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.sun.altitude, after);
      const std::string meta_after = gui::FormatSunTreeMeta(gui::g_state.sun.altitude);
      IM_CHECK_STR_NE(meta_before.c_str(), meta_after.c_str());
      IM_CHECK(meta_after.find("32.5") != std::string::npos);
    };
  }

  // The Layer row carries two things on one line — the probability meta and the delete button — and
  // the meta was inserted into the SameLine chain that positions the button. So this case asserts
  // both halves: the value the row formats follows the edit, AND the button did not move off the
  // right edge or lose its enabled state when the text was slipped in front of it.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "document_column", "editing_layer_probability_leaves_the_delete_button_put");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(2, 1);
      gui::g_state.SelectLayer(0);
      ctx->Yield(3);

      const float before = gui::g_state.layers[0].probability;
      const std::string meta_before = gui::FormatLayerTreeMeta(before);
      ctx->ItemInputValue("**/##Prob.##layer_0", 0.25f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.layers[0].probability, 0.25f);
      IM_CHECK_STR_NE(meta_before.c_str(), gui::FormatLayerTreeMeta(gui::g_state.layers[0].probability).c_str());

      // Still pinned to the right edge of the tree's scrolling child, and still clickable. This is
      // stated absolutely rather than as a before/after comparison on purpose: the meta text is
      // drawn from the same SameLine chain that places this button, so a version of that insertion
      // that let the text push the button would leave the button wherever the text ended — which a
      // self-comparison cannot see, because both samples would be equally wrong. A red-state probe
      // confirmed exactly that (a comparison-only form of this check stayed green against a
      // deliberately broken insertion; this form goes red).
      const ImGuiTestItemInfo del = ctx->ItemInfo("**/" ICON_FA_XMARK "##layer_0");
      IM_CHECK(del.ID != 0);
      IM_CHECK(!IsDisabled(del));
      ImGuiWindow* scroller = ctx->WindowInfo(kTreeScrollRef).Window;
      IM_CHECK(scroller != nullptr);
      IM_CHECK_LT(ImFabs(del.RectFull.Max.x - scroller->ContentRegionRect.Max.x), 2.0f);
    };
  }

  // The Camera row, whose meta is a NAME rather than a number and so is the one that would survive
  // a format string dropping its value. Driven through the lens picker on the inspector's Camera
  // page. The explicit combo path (rather than "**/") is the same one test_view_display_controls.cpp
  // documents: a BeginCombo preview button reports no label, so a wildcard cannot find it.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "switching_the_lens_updates_the_tree_meta");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 1);
      gui::g_state.renderer.lens_type = gui::kLensTypeLinear;
      IM_CHECK(ScrollTreeTo(ctx, "**/" ICON_FA_CAMERA " Camera"));
      ctx->ItemClick("**/" ICON_FA_CAMERA " Camera");
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.selection.kind, gui::GuiState::SelectionKind::kCamera);

      const std::string meta_before = gui::FormatCameraTreeMeta(gui::g_state.renderer.lens_type);
      // Scoped to the inspector window and picked through ComboPick, for the two reasons
      // test_view_display_controls.cpp records: the combo's preview button is not in the item
      // registry under a wildcard, and the popup scrolls, so an entry past the fold needs
      // revealing before it can be clicked.
      const std::string inspector_ref = std::string("//") + gui::kDocumentInspectorWindowName;
      ctx->SetRef(inspector_ref.c_str());
      ComboPick(ctx, "##cam_lens/##Lens Type##view", "Rectangular");
      ctx->SetRef("");
      ctx->Yield(3);

      IM_CHECK_EQ(gui::g_state.renderer.lens_type, gui::kLensTypeRectangular);
      const std::string meta_after = gui::FormatCameraTreeMeta(gui::g_state.renderer.lens_type);
      IM_CHECK_STR_NE(meta_before.c_str(), meta_after.c_str());
      IM_CHECK_STR_EQ(meta_after.c_str(), "Rectangular");
    };
  }

  // The Crystal row's weight, edited on the inspector page that owns it. The row's badges and hover
  // buttons are laid out from the right edge inward BEFORE the weight text, so their positions say
  // nothing about it — there is no addressable widget downstream of this string, which is why this
  // case asserts the document rather than a pixel or a rect.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "document_column", "editing_a_weight_updates_the_tree_meta");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildScene(1, 1);
      gui::g_state.layers[0].entries[0].proportion = 5.0f;
      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);
      IM_CHECK_STR_EQ(gui::FormatCrystalTreeMeta(gui::g_state.layers[0].entries[0].proportion).c_str(), "w 5");

      ctx->ItemInputValue("**/##Weight##prop_0_0", 100.0f);
      ctx->Yield(3);
      IM_CHECK_EQ(gui::g_state.layers[0].entries[0].proportion, 100.0f);
      IM_CHECK_STR_EQ(gui::FormatCrystalTreeMeta(gui::g_state.layers[0].entries[0].proportion).c_str(), "w 100");
    };
  }
}
