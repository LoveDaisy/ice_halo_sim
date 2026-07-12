#ifndef LUMICE_GUI_SYMMETRY_UI_HPP
#define LUMICE_GUI_SYMMETRY_UI_HPP

// task-356.3 — Shared P/B/D symmetry UI helpers, extracted from edit_modals.cpp
// so both the filter modal (edit_modals.cpp) and the Colors window per-ref row
// (color_window.cpp) render identical checkbox behaviour (a12 单一部件).
//
// Note: IsDApplicableGuiAxis() is a PURE function (no ImGui dependency); it lives
// alongside the ImGui-based RenderSymmetryCheckboxes() only because both are
// consumed by the same two call sites. Test code needing D-applicability logic
// alone can call it without initializing an ImGui context.

#include "gui/gui_state.hpp"  // AxisDist

namespace lumice::gui {

// Returns true when the given (azimuth, roll) axis config satisfies D-symmetry
// conditions: azimuth uniform 360° AND roll mean a multiple of 30°.
// Mirrors core detail::IsDApplicable (src/core/crystal.cpp). Keep in sync.
bool IsDApplicableGuiAxis(const AxisDist& az, const AxisDist& roll);

// Renders the "P B D [i]" checkbox row (ImGui). `id_suffix` disambiguates ImGui
// item IDs across call sites (mirrors the existing "##filter_modal" convention;
// pass e.g. "filter_modal" or "color_ref"). When `d_applicable` is false, the
// D checkbox is still writable but is followed by a small info icon whose
// tooltip explains the axis conditions.
void RenderSymmetryCheckboxes(bool& sym_p, bool& sym_b, bool& sym_d, bool d_applicable, const char* id_suffix);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_SYMMETRY_UI_HPP
