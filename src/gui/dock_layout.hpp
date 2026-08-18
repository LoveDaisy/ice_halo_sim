#ifndef LUMICE_GUI_DOCK_LAYOUT_HPP
#define LUMICE_GUI_DOCK_LAYOUT_HPP

#include "imgui.h"

namespace lumice::gui {

// Single owner of the GUI shell's docking substrate: the ImGui docking config flag, the DockSpace
// host window, the default dock layout, and every write to a dock node's geometry.
//
// Both the product (src/gui/main.cpp) and the test harness (test/gui/test_gui_main.cpp) call these
// and nothing else, for the same reason theme.cpp owns the visual language: two independently
// maintained docking setups would make a gui_test screenshot evidence about the harness rather than
// about the real app. The rule is mechanically checkable — `grep -rn "DockBuilder\|DockSpace(" src/`
// must only hit dock_layout.{hpp,cpp}.
//
// Division of labour with the callers:
//   - This module is the authority on dock node IDs (GetPanelNodeIds) and on the central node's
//     rectangle (GetCentralNodeRect), because no ImGui window owns that region — the central node is
//     deliberately kept empty so the OpenGL preview shows through it.
//   - It is NOT a general node-geometry query service. A window that IS docked reads its own size
//     from ImGui::GetWindowSize() like any other window; adding a second way to ask for that size
//     would create a value that can disagree with the window's actual geometry.

// The docked panels' ImGui window names. Defined here because this module has to name them in
// order to dock them, and a second spelling in app_panels.cpp's Begin calls would dock a window that
// does not exist — silently, since docking a never-submitted window is not an error.
//
// The document column is TWO windows, not one: the document tree (master) above and the inspector
// (detail) below, split by a native docking separator. The single "##LeftPanel" they replaced is
// gone rather than kept as the tree's name — a name that says "left panel" would make the pair's
// asymmetry (one of them happens to keep the old name) look like a hierarchy that does not exist.
//
// They are also the only two. The right panel that once balanced the column is gone: its Scene/View
// groups became inspector pages (doc/gui-layout-architecture.md §2) and its display groups became
// the strip under the viewport (§4), which is fixed chrome rather than a dock node — so the
// dockspace now splits once, into the document column and the central node.
constexpr const char* kDocumentTreeWindowName = "##DocumentTree";
constexpr const char* kDocumentInspectorWindowName = "##DocumentInspector";

// IDs of the dock nodes the side panels currently occupy. Valid only after
// BuildDefaultDockLayout() has run at least once; before that every field is 0 (a value no dock
// node ever has), and callers must treat 0 as "layout not built yet" rather than as a node.
//
// `left` is the document column's PARENT node — a split node no window occupies, holding
// document_tree above document_inspector. It is what the whole-column collapse resizes: writing
// the parent's size propagates down to both children on the same frame (ImGui's
// DockNodeTreeUpdatePosSize recurses from the root every frame and distributes each split node's
// size to its children), so collapsing the column does not need to know it has two halves.
struct DockPanelNodeIds {
  ImGuiID left = 0;  // Parent split node, NOT a leaf — no window docks into `left` itself; see above.
  ImGuiID document_tree = 0;
  ImGuiID document_inspector = 0;
};

// Fraction of the document column's height given to the inspector by the default layout. The tree
// gets the rest. Both halves are freely resizable by dragging their separator afterwards; this is
// only where they start.
constexpr float kDocumentInspectorHeightRatio = 0.55f;

// Enables ImGuiConfigFlags_DockingEnable. Call once, early during IO setup (alongside the other
// io.ConfigFlags assignments), before the first ImGui::NewFrame().
void ApplyDockingConfig(ImGuiIO& io);

// Renders the full-bleed, transparent host window that carries the main DockSpace, and returns the
// dockspace's ImGuiID. Coordinates are main-viewport-local (same convention as the fixed chrome
// panels in app_panels.cpp). Must be called every frame, before any window that docks into it.
ImGuiID RenderDockSpaceHost(float x, float y, float w, float h);

// Builds the default Left | Center | Right layout, but only when it does not exist yet or a reset
// was requested. `w`/`h` are the dockspace's size in logical pixels. Call every frame, immediately
// after RenderDockSpaceHost() and before the panels render, so a fresh layout is already in place
// for the same frame's ImGui::Begin calls.
void BuildDefaultDockLayout(ImGuiID dockspace_id, float w, float h);

// Requests that the next BuildDefaultDockLayout() call discards the current layout and rebuilds the
// default one (View -> Reset Layout). The pending flag lives inside this module: a caller that could
// set it directly would be a second owner of "when does the layout get rebuilt".
void RequestDockLayoutReset();

// Current side-node IDs. See DockPanelNodeIds for the not-built-yet contract. Refreshed by
// BuildDefaultDockLayout on every frame, so a layout that came back from the .ini (which never goes
// through the builder) reports the nodes it actually restored, not the ones a previous run built.
const DockPanelNodeIds& GetPanelNodeIds();

// Sets a dock node's size, e.g. to collapse a side panel to a narrow strip. This is the only way for
// code outside this module to move a dock node; the DockBuilder call itself stays here.
void ResizePanelNode(ImGuiID node_id, ImVec2 size);

// Current width of a dock node, or 0 when there is no such node.
//
// This is not a general geometry service, and does not contradict the rule above that a docked
// window reads its size from ImGui::GetWindowSize(): it answers a question about the layout BEFORE
// the window exists — after a restore from the .ini, the first frame has a dock tree and no windows
// at all — which no window can be asked. Per-frame geometry still comes from the window.
float GetPanelNodeWidth(ImGuiID node_id);

// Current height of a dock node, or 0 when there is no such node.
//
// Same exemption as GetPanelNodeWidth above, plus one the width query did not need: the document
// column's parent node is a SPLIT node, which no window occupies at all. ResizePanelNode takes a
// full ImVec2, so collapsing the column has to name its height, and the only honest source for
// that number is the node. The tree window's own GetWindowSize().y is a different quantity (one
// half of the column) and writing it back would shrink the column every time it collapsed.
float GetPanelNodeHeight(ImGuiID node_id);

// Rectangle of the dockspace's central node, in main-viewport-local coordinates. The central node is
// kept empty on purpose (ImGuiDockNodeFlags_PassthruCentralNode + NoDockingOverCentralNode), so this
// rectangle is what the preview viewport occupies and no ImGui window can report it. Returns false
// when the layout has not been built yet, leaving the outputs untouched.
bool GetCentralNodeRect(ImVec2* out_pos, ImVec2* out_size);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DOCK_LAYOUT_HPP
