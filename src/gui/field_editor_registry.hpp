#ifndef LUMICE_GUI_FIELD_EDITOR_REGISTRY_HPP
#define LUMICE_GUI_FIELD_EDITOR_REGISTRY_HPP

// Which serialized settings can be edited in place, and with WHAT constraint.
//
// One entry per editable leaf of the serialized GuiState document, keyed by the SERIALIZED key
// path ("overlay_grid_color", not the C++ field name `grid_color` — 18 of the 42 leaves differ,
// and the key path is what the diff engine, the override file and the user all speak).
//
// REGISTERED, NOT GENERATED — the opposite of defaults_diff.hpp, and the split is deliberate:
//   * the EXISTENCE of a row is generated (walk the JSON tree), so a new serialized field shows
//     up in the panel with zero edits here;
//   * the EDITOR for a row is registered, because a control and its domain cannot be derived from
//     a JSON leaf. "0.3" does not say whether it is an alpha in [0,1], a probability, or an angle.
// A field with no entry here is READ-ONLY in the panel and says so visually. That degradation is
// the designed one: a generic input box on a constrained field is actively harmful (it accepts
// values the real control would refuse), so "no entry" must mean "not editable", never "fall back
// to a plain box".
//
// TWO FUNCTIONS PER ENTRY, and the split is what makes this reusable:
//   Constraint(state) — pure data, NO ImGui calls. The domain and whether the field currently
//                       applies at all. The panel's "this loaded value is out of range" notice
//                       reads it, and it is the half a future main-UI migration consumes.
//   Render(state, id) — draws the real control, bound to the SAME GuiState field the main UI
//                       edits. Returns true when the call actually wrote a new value.
//
// CROSS-FIELD COMBINATIONS — the position, because a per-field editor raises the question and
// silence would be an answer by default. This table must not become a second way to reach states
// the main UI refuses (a saved default of "full-sky lens WITH a non-zero roll", say, which no
// sequence of main-UI actions can produce and which the loader would then quietly ignore).
//
// It cannot, and by construction rather than by a list of forbidden pairs:
//   1. every entry's `enabled` restates the BeginDisabled(...) expression at the field's main-UI
//      call site, so the set of (field, configuration) pairs this table will write is the same set
//      the main UI will write. A field that does not apply is greyed here exactly when it is greyed
//      there;
//   2. the renderer invariants in app_panels.cpp (RenderPreviewPanel) run every frame, panel open
//      or not — this is a popup, not a replacement for the frame. Setting roll and THEN switching
//      to a full-sky lens does not smuggle the pair through: roll is forced back to 0 on the next
//      frame, before any Save can see it. Pinned directly (not just by this comment) by
//      `p2_render/lens_full_sky_view_controls_disabled` in test_gui_interaction.cpp, which sets a
//      non-zero roll under every entry of kFullSkyLensTypes and asserts the next frame zeroes it.
// Point 1 is the rule; point 2 is why a mistake in point 1 still cannot produce the bad state. What
// this does NOT do is validate combinations that ARE reachable from the main UI — those need no
// defending, since a user could have saved them by hand anyway.
//
// KNOWN DEBT, stated here because it is invisible at the call sites: each entry currently
// RE-STATES the constraint its main-UI call site spells out inline (the same min/max/format/
// disabled-when expression, written a second time). The two can therefore drift, and only the
// parity tests in test_gui_defaults_panel.cpp would notice. Folding the main UI's ~16 slider call
// sites onto this registry — so the constraint has one home instead of two — is a separate,
// already-scoped task; keeping every restatement inside THIS file (rather than letting the panel
// compute a bound of its own) is what keeps that migration a one-file change.

#include <functional>
#include <string>
#include <vector>

#include "gui/gui_state.hpp"

namespace lumice::gui {

// The control shape a field is edited with. Carried explicitly (rather than inferred from the JSON
// leaf type) so a coverage test can assert "this key is edited as a colour, that one as an int
// slider" per key rather than merely counting entries.
enum class FieldEditorKind {
  kFloatSlider,
  kIntSlider,
  kCheckbox,
  kCombo,
  kColor,
};

// What the field's editor allows RIGHT NOW. A function of the whole state, never a constant: fov's
// upper bound follows the lens type, and four view fields stop applying entirely under a full-sky
// lens.
struct FieldEditorConstraint {
  // False when the field exists but does not currently apply (a full-sky lens has no roll). The
  // panel greys the control and shows `disabled_reason` on hover — deliberately NOT the same
  // visual as an unregistered field, so "cannot be edited here at all" and "cannot be edited in
  // this configuration" stay distinguishable.
  bool enabled = true;
  const char* disabled_reason = nullptr;
  // Whether [min_value, max_value] is meaningful. False for booleans, colours and combos: a combo
  // over an enum table has no interval to be outside of, and reporting one would invite a range
  // check that can only produce false alarms.
  bool has_numeric_domain = false;
  double min_value = 0.0;
  double max_value = 0.0;
};

struct FieldEditorEntry {
  FieldEditorKind kind = FieldEditorKind::kFloatSlider;
  std::function<FieldEditorConstraint(const GuiState&)> Constraint;
  // `id_base` is the caller's per-row id fragment (the panel passes "value_<key_path>"). Widgets
  // built from it are addressable by test and unique per row; the entry never invents an id of its
  // own, so no two rows can collide.
  //
  // Returns true only when this call CHANGED the state. A control that merely re-displays a value
  // — including one that had to clamp an out-of-range value for display — returns false: opening a
  // panel must not edit the document the user came to look at.
  std::function<bool(GuiState&, const char* id_base)> Render;
};

// The editor for `key_path`, or nullptr when the field is read-only in the panel. Total function:
// nullptr is a legitimate, designed answer, not a lookup failure.
const FieldEditorEntry* FindFieldEditor(const std::string& key_path);

// Every registered key path. For coverage assertions ("which of the document's leaves have an
// editor") — the panel itself never enumerates, it looks up per row.
std::vector<std::string> RegisteredFieldEditorKeyPaths();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FIELD_EDITOR_REGISTRY_HPP
