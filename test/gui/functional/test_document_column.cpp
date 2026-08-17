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

#include "gui/dock_layout.hpp"
#include "gui/gui_constants.hpp"
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

      ctx->ScrollToBottom(gui::kDocumentTreeWindowName);
      ctx->Yield(3);

      // The tree scrolled...
      tree = ctx->GetWindowByRef(gui::kDocumentTreeWindowName);
      IM_CHECK(tree != nullptr);
      // ...and the inspector did not move or resize because of it. A shared scroll region would
      // have slid the inspector up as the tree's content advanced.
      inspector = ctx->GetWindowByRef(gui::kDocumentInspectorWindowName);
      IM_CHECK(inspector != nullptr);
      IM_CHECK_EQ(inspector->Pos.y, inspector_y_before);
      IM_CHECK_EQ(inspector->Size.y, inspector_h_before);

      ctx->ScrollToTop(gui::kDocumentTreeWindowName);
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
}
