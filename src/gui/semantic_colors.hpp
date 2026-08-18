#ifndef LUMICE_GUI_SEMANTIC_COLORS_HPP
#define LUMICE_GUI_SEMANTIC_COLORS_HPP

#include "imgui.h"

// Single owner of the GUI's *content-semantic* colours: the three grades that
// answer "what does this element say about the thing it describes" — good /
// warning / destructive. Before this module they were ImVec4 literals at ~17
// call sites across app_panels.cpp, edit_modals.cpp and color_window.cpp, with
// five near-duplicate ambers and three near-duplicate reds that no one could
// change together.
//
// What belongs here and what does not:
//   - Semantic colour (here): the *content* carries a judgement — this value is
//     fine / needs your attention / is an error or a destructive action.
//   - Accent colour (NOT here): interaction state — hovered, active, focused,
//     "currently highlighted". Deliberately kept at different values, so
//     "this is being operated on" never reads as "this is risky".
//   - Data colour (NOT here): a value the user chose (overlay line colours) or
//     an identity colour derived from an index (sync-group swatches). Those
//     carry no product judgement and must not be folded into a grade.
//
// Two consumption forms per grade, because a colour that reads correctly as
// 15px text is far too loud as a filled input background:
//   - *TextColor()  — bright, for text / icons / borders.
//   - *FillColor(a) — muted, for FrameBg / CellBg tinting; alpha is the
//     caller's, since existing call sites deliberately differ (0.5 for a
//     validation cell, 0.6 for the Resolution input).
//
// Button triples are only provided where a consumer exists: good (the Run
// button) below, warning (the top bar's "Changed - re-run" chip) below that,
// and destructive in gui/destructive_style.hpp.

namespace lumice::gui {

// "Fine / succeeded / valid" — Ready status, a passing validation cell.
ImVec4 GoodTextColor();
ImVec4 GoodFillColor(float alpha);

// Run button's Normal/Hovered/Active triple. Push/Pop must be paired on every
// code path (no early return between them), matching PushDestructiveStyle.
void PushGoodButtonStyle();
void PopGoodButtonStyle();

// "Needs your attention, but not an error" — a Resolution change that will
// re-run the simulation, unsaved edits, an incomplete row, a soft cap exceeded.
ImVec4 WarningTextColor();
ImVec4 WarningFillColor(float alpha);

// Warning button's Normal/Hovered/Active triple, for a control whose whole
// message is "this needs your attention" — currently the top bar's
// "Changed - re-run" chip. Push/Pop must be paired on every code path, matching
// PushGoodButtonStyle / PushDestructiveStyle.
//
// It is derived from the same amber as WarningTextColor rather than being a
// fourth hand-picked shade: a button surface has to be darker than the text
// grade at the same hue to keep its own label legible, so the triple sits at
// the fill end of the grade, not at the text end. Anything that changes the
// amber must change both, or the chip and the "Modified" status text stop
// reading as the same statement.
void PushWarningButtonStyle();
void PopWarningButtonStyle();

// "Error / destructive action" — invalid input, a hard cap exceeded, delete /
// remove / Stop.
ImVec4 DestructiveTextColor();
ImVec4 DestructiveFillColor(float alpha);
// The destructive *button* triple is PushDestructiveStyle/PopDestructiveStyle in
// gui/destructive_style.hpp — the already-validated implementation with 12 paired
// call sites. It is deliberately not re-declared here: one grade, one owner per
// form. See doc/gui-visual-language.md §7.

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SEMANTIC_COLORS_HPP
