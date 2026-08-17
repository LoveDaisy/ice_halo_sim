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

// IDs of the two side dock nodes of the default layout. Valid only after BuildDefaultDockLayout()
// has executed at least once; before that both fields are 0 (a value no dock node ever has), and
// callers must treat 0 as "layout not built yet" rather than as a node.
struct DockPanelNodeIds {
  ImGuiID left = 0;
  ImGuiID right = 0;
};

// Enables ImGuiConfigFlags_DockingEnable. Call once, right after ImGui::CreateContext().
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

// Current side-node IDs. See DockPanelNodeIds for the not-built-yet contract.
const DockPanelNodeIds& GetPanelNodeIds();

// Sets a dock node's size, e.g. to collapse a side panel to a narrow strip. This is the only way for
// code outside this module to move a dock node; the DockBuilder call itself stays here.
void ResizePanelNode(ImGuiID node_id, ImVec2 size);

// Rectangle of the dockspace's central node, in main-viewport-local coordinates. The central node is
// kept empty on purpose (ImGuiDockNodeFlags_PassthruCentralNode + NoDockingOverCentralNode), so this
// rectangle is what the preview viewport occupies and no ImGui window can report it. Returns false
// when the layout has not been built yet, leaving the outputs untouched.
bool GetCentralNodeRect(ImVec2* out_pos, ImVec2* out_size);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DOCK_LAYOUT_HPP
