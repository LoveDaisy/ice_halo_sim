#include "gui/dock_layout.hpp"

#include <algorithm>

#include "gui/gui_constants.hpp"
#include "imgui_internal.h"

namespace lumice::gui {

namespace {

// The host window is invisible by construction (no background, no border, no padding); it exists
// only to give the DockSpace a rectangle. The "##" prefix keeps it out of the window list the same
// way the other chrome panels do.
constexpr const char* kDockHostWindowName = "##DockHost";
constexpr const char* kDockSpaceName = "##MainDockSpace";

// Flags shared by the DockSpace call and by the layout the builder writes, so the two cannot drift.
//
// PassthruCentralNode + NoDockingOverCentralNode together are what keeps the OpenGL preview visible.
// PassthruCentralNode makes DockSpace() fill the whole node with ImGuiCol_WindowBg *except* a hole
// over the central node -- and imgui.cpp punches that hole only while the central node is EMPTY
// (`central_node_hole` in DockNodeUpdate). Dock any window into the central node and the hole is
// gone, i.e. the fill covers the region the preview shader draws into. NoDockingOverCentralNode is
// therefore not decoration: it is the invariant the transparency depends on.
constexpr ImGuiDockNodeFlags kDockSpaceFlags =
    ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingOverCentralNode;

DockPanelNodeIds g_panel_node_ids;

// "Rebuild the layout on the next BuildDefaultDockLayout call", set by RequestDockLayoutReset and
// consumed there. Deliberately not a GuiState field: it is an implementation detail of when this
// module rebuilds, and a second place able to set it would be a second owner of that decision.
bool g_reset_layout_requested = false;

// Set by RenderDockSpaceHost. ImGui::GetID() is scoped to the window that is current when it runs,
// so the ID can only be computed inside the host window -- which is why it is cached here rather
// than recomputed by every caller that needs to look the node up.
ImGuiID g_dockspace_id = 0;

// The node a panel window currently lives in, or 0 if it lives in none. Reading it back from the
// window is what makes a layout restored from the .ini usable: that path never runs the builder, so
// the IDs it hands out are the ones the settings file recorded, not the ones any build produced. It
// also means a panel dragged to a different node keeps working, without a cache to invalidate.
// DockId rather than DockNode covers the frames after settings are loaded but before the window's
// first Begin, where the binding exists and the node pointer is not yet resolved.
ImGuiID DockedNodeIdOf(const char* window_name) {
  const ImGuiWindow* window = ImGui::FindWindowByName(window_name);
  if (window == nullptr) {
    return 0;
  }
  return window->DockNode != nullptr ? window->DockNode->ID : window->DockId;
}

void RefreshPanelNodeIds() {
  if (const ImGuiID tree = DockedNodeIdOf(kDocumentTreeWindowName)) {
    g_panel_node_ids.document_tree = tree;
    // The column's parent is read back from the tree's node rather than remembered from the last
    // build, for the same reason the leaf ids are: a layout restored from the .ini never ran the
    // builder. ParentNode is null while the tree is undocked (a user can drag it out), in which
    // case the previous value is kept rather than zeroed — a whole-column collapse then still
    // resizes the column the inspector is left in, instead of silently doing nothing.
    if (const ImGuiDockNode* tree_node = ImGui::DockBuilderGetNode(tree);
        tree_node != nullptr && tree_node->ParentNode != nullptr) {
      g_panel_node_ids.left = tree_node->ParentNode->ID;
    }
  }
  if (const ImGuiID inspector = DockedNodeIdOf(kDocumentInspectorWindowName)) {
    g_panel_node_ids.document_inspector = inspector;
  }
}

}  // namespace

void ApplyDockingConfig(ImGuiIO& io) {
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

ImGuiID RenderDockSpaceHost(float x, float y, float w, float h) {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + x, vp->Pos.y + y));
  ImGui::SetNextWindowSize(ImVec2(w, h));
  ImGui::SetNextWindowViewport(vp->ID);

  // NoDocking: the host itself must never be dragged into another dockspace.
  // NoBackground: the dockspace paints the panel background (see kDockSpaceFlags); if the host
  // painted one too, it would also paint over the central node's hole.
  // NoBringToFrontOnFocus / NoNavFocus keep it in the background cluster described at the top of
  // app_panels.cpp, below every floating window.
  const ImGuiWindowFlags host_flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBackground;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::Begin(kDockHostWindowName, nullptr, host_flags);
  ImGui::PopStyleVar(3);

  const ImGuiID dockspace_id = ImGui::GetID(kDockSpaceName);
  ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), kDockSpaceFlags);
  ImGui::End();

  g_dockspace_id = dockspace_id;
  return dockspace_id;
}

void BuildDefaultDockLayout(ImGuiID dockspace_id, float w, float h) {
  if (dockspace_id == 0 || w <= 0.0f || h <= 0.0f) {
    return;
  }

  // "Already built" is a question about the in-memory dock tree, not about whether an .ini file
  // exists. gui_test runs with io.IniFilename == nullptr forever, so an ini-based predicate would
  // report "not built" on every frame there and rebuild the whole tree 60 times a second.
  const ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockspace_id);
  const bool already_built = node != nullptr && node->IsSplitNode();
  if (already_built && !g_reset_layout_requested) {
    RefreshPanelNodeIds();
    return;
  }
  g_reset_layout_requested = false;

  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, kDockSpaceFlags | ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImVec2(w, h));

  // The panel width constants stay the single source of the layout: the ratios below are derived
  // from them, and the exact node sizes are written back straight after the splits so that
  // truncation inside DockBuilderSplitNode cannot make the panels a pixel narrower than the rest of
  // the GUI (aspect-ratio fitting, the left-panel capture rect) computes with.
  //
  // Ratio math needs `w` at least kLeftPanelWidth + a nonzero center strip, or the split's ratio
  // reaches >=1 and DockBuilderSplitNode's size_ratio_for_node_at_dir asserts on it in a debug ImGui
  // build (and produces an undefined negative-width node otherwise). `kMinWindowWidth` (main.cpp's
  // GLFW size hint) already keeps the real app window above this floor, but this function also runs
  // for whatever `w` a caller passes, so the clamp is local rather than relying on that.
  const float ratio_w = std::max(w, kLeftPanelWidth + 1.0f);
  ImGuiID center_id = dockspace_id;
  const ImGuiID left_id =
      ImGui::DockBuilderSplitNode(center_id, ImGuiDir_Left, kLeftPanelWidth / ratio_w, nullptr, &center_id);
  ImGui::DockBuilderSetNodeSize(left_id, ImVec2(kLeftPanelWidth, h));

  // Second split, inside the left column: the document tree on top, the inspector below. This is
  // the master-detail pair of doc/gui-layout-architecture.md §2, and it is built out of a real
  // dock split rather than a hand-rolled child-window splitter precisely because everything the
  // blueprint asks of it — independent scrolling, a draggable separator, per-half collapse — is
  // what a dock node already does. `left_id` stays valid afterwards: DockBuilderSplitNode turns
  // the node it is given into the parent of the two new children and keeps its ID.
  ImGuiID tree_id = left_id;
  const ImGuiID inspector_id =
      ImGui::DockBuilderSplitNode(tree_id, ImGuiDir_Down, kDocumentInspectorHeightRatio, nullptr, &tree_id);

  // No tab bar: this layout keeps the panels looking exactly like the fixed strips they replace,
  // and a single-window node would otherwise grow a tab header the previous layout never had. The
  // flag goes on the two LEAF nodes; the left column's parent is a split node, which has no tab
  // bar of its own to suppress.
  for (const ImGuiID leaf : { tree_id, inspector_id }) {
    if (ImGuiDockNode* node_ptr = ImGui::DockBuilderGetNode(leaf)) {
      node_ptr->SetLocalFlags(node_ptr->LocalFlags | ImGuiDockNodeFlags_NoTabBar);
    }
  }

  // ##PreviewPanel and ##DisplayStrip are intentionally NOT docked -- see kDockSpaceFlags. They are
  // positioned over the (permanently empty) central node by GetCentralNodeRect(), the preview above
  // and the strip along its bottom edge.
  ImGui::DockBuilderDockWindow(kDocumentTreeWindowName, tree_id);
  ImGui::DockBuilderDockWindow(kDocumentInspectorWindowName, inspector_id);
  ImGui::DockBuilderFinish(dockspace_id);

  g_panel_node_ids.left = left_id;
  g_panel_node_ids.document_tree = tree_id;
  g_panel_node_ids.document_inspector = inspector_id;
}

void RequestDockLayoutReset() {
  g_reset_layout_requested = true;
}

const DockPanelNodeIds& GetPanelNodeIds() {
  return g_panel_node_ids;
}

void ResizePanelNode(ImGuiID node_id, ImVec2 size) {
  if (node_id == 0 || size.x <= 0.0f || size.y <= 0.0f) {
    return;
  }
  ImGui::DockBuilderSetNodeSize(node_id, size);
  ImGui::DockBuilderFinish(node_id);
}

float GetPanelNodeWidth(ImGuiID node_id) {
  const ImGuiDockNode* node = node_id != 0 ? ImGui::DockBuilderGetNode(node_id) : nullptr;
  return node != nullptr ? node->Size.x : 0.0f;
}

float GetPanelNodeHeight(ImGuiID node_id) {
  const ImGuiDockNode* node = node_id != 0 ? ImGui::DockBuilderGetNode(node_id) : nullptr;
  return node != nullptr ? node->Size.y : 0.0f;
}

bool GetCentralNodeRect(ImVec2* out_pos, ImVec2* out_size) {
  if (out_pos == nullptr || out_size == nullptr || g_dockspace_id == 0) {
    return false;
  }
  const ImGuiDockNode* root = ImGui::DockBuilderGetNode(g_dockspace_id);
  if (root == nullptr || !root->IsSplitNode() || root->CentralNode == nullptr) {
    return false;
  }
  const ImGuiDockNode* central = root->CentralNode;
  if (central->Size.x <= 0.0f || central->Size.y <= 0.0f) {
    return false;
  }
  // ImGui reports node positions in absolute screen space under multi-viewport; every geometry
  // computation in app_panels.cpp is main-viewport-local, so convert here rather than at each caller.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  *out_pos = ImVec2(central->Pos.x - vp->Pos.x, central->Pos.y - vp->Pos.y);
  *out_size = central->Size;
  return true;
}

}  // namespace lumice::gui
