#pragma once

#include "imgui.h"

namespace lumice::gui {

// Single owner of the GUI's visual language: ImGui style baseline plus the font
// atlas (body font + merged FontAwesome icon glyphs).
//
// Both the product (src/gui/main.cpp) and the test harness
// (test/gui/test_gui_main.cpp) call this and nothing else, so a screenshot taken
// by gui_test is evidence about the real app's appearance rather than a
// coincidence of two independently maintained initialization paths.
//
// Must be called after ImGui::CreateContext(), before the render loop.
void ApplyVisualLanguage(ImGuiIO& io);

}  // namespace lumice::gui
