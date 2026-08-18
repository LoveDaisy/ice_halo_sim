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

// The half of the visual language that is pure data: the colour palette and the size rhythm,
// written into whichever ImGuiStyle is handed in. ApplyVisualLanguage calls this on the active
// style and then loads the fonts, which is the half that needs an ImGuiIO.
//
// It is separate so a caller can inspect what the palette CLAIMS without a live context to read it
// back out of: the coverage test fills a scratch style with a sentinel, calls this, and requires
// that no sentinel survives. Reading the active style instead would only prove the slots differ
// from ImGui's dark defaults, which is a different (and weaker) statement — two slots are
// deliberately assigned that very value.
void ApplyStyle(ImGuiStyle& style);

// Draws ImGui::Checkbox, then — because the theme's FrameBorderSize is 0 — adds a
// low-contrast inset border around an UNCHECKED box so it stays legible against the panel
// background. A checked box is already filled with the accent colour and needs no such aid.
//
// Scope note: theme.cpp only houses widget-level compensations that are direct consequences
// of FrameBorderSize=0. If a future borderless-widget legibility issue is unrelated to that
// decision, it belongs in a separate widgets module, not here — and so does this one once a
// second such compensation appears (two is the point at which the pair, not the theme, is
// the thing being maintained).
bool Checkbox(const char* label, bool* v);

}  // namespace lumice::gui
