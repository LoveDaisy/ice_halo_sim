#ifndef LUMICE_GUI_DESTRUCTIVE_STYLE_HPP
#define LUMICE_GUI_DESTRUCTIVE_STYLE_HPP

// task-color-window-controls-polish (A2) — Shared destructive (red) button
// palette + Push/Pop helpers, extracted from panels.cpp and edit_modals.cpp's
// previously-duplicated file-local definitions. Third consumer (color_window.cpp
// class/ref delete x + Remove All button) triggered the promotion, matching the
// convention documented at the old edit_modals.cpp definition site.
//
// Usage: wrap a destructive control (delete/remove) with
//   PushDestructiveStyle();
//   if (ImGui::SmallButton(id)) { ... }
//   PopDestructiveStyle();
// Push/Pop must be paired on every code path (no early return between them).

namespace lumice::gui {

void PushDestructiveStyle();
void PopDestructiveStyle();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_DESTRUCTIVE_STYLE_HPP
