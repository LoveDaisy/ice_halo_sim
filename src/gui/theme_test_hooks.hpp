#pragma once

#include "imgui.h"

namespace lumice::gui {

// Test-only entry point into theme.cpp, kept in its own header on purpose.
//
// It could have been declared in theme.hpp behind a comment saying "do not call this from the
// app". A separate header makes that a structural fact instead of a promise: no production
// translation unit has any reason to include a file named theme_test_hooks.hpp, so an accidental
// production call site shows up as an odd include line rather than as one more function in the
// theme's public list. The only includer today is test/gui/test_gui_main.cpp.
//
// What it does: re-applies ONLY the palette (not spacing, not rounding, not the font atlas), using
// a deliberately distant test palette. A frame captured after this call and a frame captured under
// the production ApplyStyle() therefore differ in colour and in nothing else, which is what makes
// the pair a valid input to a pixel diff: a pixel that is the same in both was drawn with a colour
// the theme does not own.
//
// Call after ApplyVisualLanguage(), before the render loop.
void ApplyContrastPaletteForTest(ImGuiStyle& style);

}  // namespace lumice::gui
