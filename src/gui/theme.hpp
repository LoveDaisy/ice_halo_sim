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

// The colour+geometry half of ApplyVisualLanguage: grid spacing plus the palette, applied to a
// caller-supplied style rather than to ImGui::GetStyle(). Split out so a test can hand it a
// scratch ImGuiStyle and ask what it wrote — the "was this slot claimed" question needs a style
// it owns, and it must reach the very code path the app uses, not a copy of it
// (test/gui/functional/test_theme_coverage.cpp).
//
// It deliberately does NOT call ImGui::StyleColorsDark() first: the coverage test fills its
// scratch style with a sentinel and requires none of it to survive, which is only a question
// about this function if the baseline preset is not run inside it.
void ApplyStyle(ImGuiStyle& style);

// The accent colour as it is CURRENTLY INSTALLED in the live style, optionally at a different
// alpha. This is the one place that knows how to read it back: ApplyPalette writes Palette::accent
// into several slots and ImGuiCol_CheckMark is one of them, so asking the style is both cheaper
// than exporting the Palette and — the part that matters — it follows whatever palette is actually
// in force rather than a compile-time constant.
//
// It exists because "borrow the accent here" was being spelled out at call sites as a literal blue
// that no longer moved when the palette did. A call site that wants the accent says so; the
// coupling to a particular ImGuiCol_ slot lives here and nowhere else.
ImVec4 AccentColor(float alpha = 1.0f);

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

// Checkbox() above, for a value DERIVED from several booleans — an "All" row over a column of
// switches. `all_true` is the AND of them and `mixed` says whether the OR disagrees with it; the
// box then draws ImGui's filled square (ImGuiItemFlags_MixedValue) instead of a tick or nothing.
//
// No click rule of its own, on purpose. ImGui's Checkbox flips the value it was handed, and the
// value handed in is the AND — so a mixed box (AND false) turns everything ON, an all-on box turns
// everything off, and an all-off box turns everything on: the usual select-all convention falls out
// of passing the AND rather than the OR, with no state machine to explain "what does clicking
// 'some' do". The caller fans the returned `*all_true` back out to the source booleans.
//
// A wrapper around Checkbox() rather than a sibling of it: the inset border an unchecked box gets
// under FrameBorderSize=0 is the same aid a mixed box (also unchecked, as ImGui reads it) needs.
bool TriStateCheckbox(const char* label, bool* all_true, bool mixed);

}  // namespace lumice::gui
