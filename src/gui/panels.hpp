#ifndef LUMICE_GUI_PANELS_HPP
#define LUMICE_GUI_PANELS_HPP

// Full include (not forward declaration) because RenderAxisDist takes AxisDist by reference,
// requiring the complete type definition. This propagates gui_state.hpp to all includers.
#include "gui/gui_state.hpp"

namespace lumice::gui {

// Slider scale modes for SliderWithInput
enum class SliderScale { kLinear, kSqrt, kLog, kLogLinear };

// ---- Trailing-label column ----

// The one place this repo decides how wide the gap between a row's last control and its
// row-trailing label is. Every consumer of that gap — the [slider][input] Label rows, the entry
// card's button rows, the combo rows, and the wedge-angle editor's mirror of the same layout —
// takes its value from here, so the gap is a single edit rather than a set of copies that happen
// to agree today.
//
// It forwards ImGui's ItemInnerSpacing.x rather than introducing a constant of its own, and that
// is deliberate: ImGui's own Combo hardcodes ItemInnerSpacing.x between the frame and the label
// inside BeginCombo, where no caller can reach it. A combo row and a row laid out here can only
// put their labels on the same vertical line if this side takes that value; a private constant
// would just be a second source that has to be kept equal to it by hand.
float LabelColumnGapX();

// PushItemWidth for a control that is followed by nothing but the row-trailing label: takes the
// full content width minus the label column and the gap above.
//
// This exists as a function rather than an expression repeated at each call site because it is
// the one shape where the whole expression, not just the gap, is identical everywhere it appears
// (the Spectrum combo, a no-op push around a Checkbox that reads no item width, and the Lens Type
// / Resolution combos). The [slider][input] Label rows compute a width from the same gap but also
// subtract the input column and the slider-to-input spacing, so they take LabelColumnGapX()
// directly and keep their own formula.
void PushLabelColumnItemWidth();

// One control per value: a single DragFloat where a [slider][input] pair would otherwise sit.
// Ctrl+click (or, with io.ConfigDragClickToInputText on, a plain click-release) types an exact
// value, so the input half is not lost, it is folded in. The item id is "##<label>", with no
// _slider / _input suffix, because there is no longer a pair to distinguish. Today's only call
// sites are the Overlays table's alpha cells (RenderOverlaysTab, app_panels.cpp), where a cell
// has room for exactly one control; SliderWithInput remains the default elsewhere.
//
// `scale` keeps meaning what it means for SliderWithInput, but a Drag has no track position
// to map, only a speed, so the three non-linear modes collapse onto ImGui's own logarithmic
// drag: kLog and kLogLinear are what it natively is, and kSqrt takes it as the closer of the
// two available approximations (a linear drag over a 0-360 domain moves ~1.6 deg per pixel,
// which cannot express the sub-degree spreads the sqrt mapping existed to make reachable).
// Dragging to either end still yields exactly min / max — ImGui special-cases the extents.
//
// The value is clamped to [min_val, max_val] UNCONDITIONALLY, not just when the widget moved it:
// ImGuiSliderFlags_AlwaysClamp constrains what the widget produces and leaves an out-of-range value
// it was handed alone, and fields do arrive here from outside any control (a hand-written .lmc, a
// lens switch that narrows a bound under a value that was legal a frame ago). Returns true when the
// value is not what it was — clamp included, since a clamp is a change the caller has to commit
// like any other. Both match what SliderWithInput does.
bool DragFloatField(const char* label, float* value, float min_val, float max_val, const char* fmt = "%.1f",
                    SliderScale scale = SliderScale::kLinear);

// Where SliderWithInput / SliderIntWithInput put the field's name relative to its controls.
//
//   kTrailing  [slider] [input] Label   the panel default (Axis / Sun / Simulation / View / Display)
//   kLeading   Label [slider] [input]   entry-card rows, whose label column is shared with three
//                                       plain-text rows above it
//   kNone      [slider] [input]         TABLE-CELL MODE — this is not a "neither of the above"
//                                       fallback: it is for a caller whose field name already has
//                                       a dedicated table column of its own (RenderShapeDistTableRow
//                                       and the defaults panel's field-editor registry), so drawing
//                                       one here would print the name twice. Dropping the label also
//                                       drops kLabelColWidth from the width, letting the pair fill
//                                       the cell.
//
// kLeading and kTrailing reserve the SAME geometry — one kLabelColWidth and the same two gaps — and
// differ only in draw order, so a row can switch between them without any other row moving.
enum class LabelPlacement { kTrailing, kLeading, kNone };

// Slider + InputFloat + label text, laid out per `label_placement` (default: [slider] [input] Label).
// Uses a fixed label column width so vertically stacked sliders align.
// Returns true if value changed.
//
// `committed` (optional out-param, defaults to nullptr so existing call sites are unaffected):
// set to `ImGui::IsItemDeactivatedAfterEdit()` semantics OR-ed across the slider AND the input
// sub-widget — true exactly on the frame interaction with either one ends, not on every frame a
// drag moves the value. The field-editor registry (`field_editor_registry.cpp`) needs this to
// gate its own commit-and-refresh at "once per edit" rather than "once per frame of a drag".
//
// `active` (optional out-param, same nullptr default and the same OR across both sub-widgets): set
// to `ImGui::IsItemActive()` semantics — whether the user is STILL holding either sub-widget this
// frame. It answers a different question from `committed`: not "did an edit just end" but "may I
// overwrite the value I passed in". A caller that keeps its own copy of the field (the field-editor
// registry does, so that a drag's intermediate frames never reach the document) must refresh that
// copy from its source only on frames where this is false — refreshed on the release frame, the
// copy would be reset to the pre-drag value before `committed` ever gets read, and the whole drag
// would be dropped. The distinction cannot be rebuilt at the call site: only this function sees
// both sub-widgets' `IsItemActive()`, and after it returns ImGui's "last item" is the input box
// alone.
//
// `avail_override` (0 = use the content region, the default): the width the row is allowed to
// occupy, when the caller owns a strip NARROWER than the content region and the widget cannot see
// where it ends. The entry card is the case: its right edge is reserved for an icon rail that is
// drawn by absolute position, so a row sized to the content region would run underneath it. Passing
// the number is what makes the card's four rows share one right edge by construction rather than by
// two formulas that have to agree.
bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt = "%.1f",
                     SliderScale scale = SliderScale::kLinear,
                     LabelPlacement label_placement = LabelPlacement::kTrailing, bool* committed = nullptr,
                     bool* active = nullptr, float avail_override = 0.0f);

// SliderInt + InputInt + label text — SliderWithInput's integer sibling, same layout and the same
// `label_placement` modes and the same optional `committed` / `active` out-params.
// Returns true if the value changed.
//
// Exported (it was file-static in panels.cpp) because the defaults panel's field-editor registry
// renders integer settings with it: an integer setting has to be edited by the SAME control the
// main UI uses, or the two disagree about what a valid value is — which is the whole point of that
// registry.
bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val,
                        LabelPlacement label_placement = LabelPlacement::kTrailing, bool* committed = nullptr,
                        bool* active = nullptr);

// ---- Edit request (shared between panels.cpp and app_panels.cpp) ----
enum class EditTarget { kNone, kCrystal, kAxis, kFilter, kCard };

struct EditRequest {
  EditTarget target = EditTarget::kNone;
  int layer_idx = -1;
  int entry_idx = -1;
};

const EditRequest& GetEditRequest();
void ResetEditRequest();

// ---- Shared ImGui combo-popup fix (panels.cpp and edit_modals.cpp both call this) ----

// Mark the next combo's popup viewport as TopMost so it shares NSWindow level with a detached
// OS-viewport modal (see panels.cpp for the full mechanism writeup). Single definition — both
// modal-internal combos (edit_modals.cpp) and panel-internal combos (RenderShapeDistTypeCombo
// below) call this same function rather than each inlining their own copy.
void SetNextComboPopupTopMost();

// ---- Axis distribution controls (shared between panels and edit modals) ----

// Render axis distribution controls (combo + mean + std sliders).
// Returns true if any value changed. Does NOT call MarkDirty() — caller is responsible.
bool RenderAxisDist(const char* label, AxisDist& axis, float mean_min, float mean_max);

// ---- Shape distribution controls (crystal geometry randomization) ----

// Default spread fraction applied when a shape field's randomization is first enabled: the
// distribution spread starts at 0.2 × center (i.e. roughly ±10% for uniform's center±spread/2).
// An arbitrary but reasonable seed value; the user adjusts from there. Shared between
// RenderShapeDist below and edit_modals.cpp's face_distance unified/per-face "enable" branches so
// all three sites agree on the same default heuristic.
constexpr float kShapeDistDefaultSpreadFraction = 0.2f;

// Column count of the crystal shape-parameter property table (see RenderCrystalModal). Single
// source shared by the structure-definition site (edit_modals.cpp TableSetupColumn) and the
// structure-consumption sites (RenderShapeDistTableRow / the wedge + face rows' TableNextColumn
// sequences). ImGui does NOT hard-assert when a row emits a different number of columns than the
// header declares — it silently misaligns — so keeping every site pinned to this one constant is
// how the "columns everywhere" invariant is enforced. Columns: Param | Value | Sync | Rand | Spread.
// (The GUI edits uniform-only — no distribution-type column; non-uniform types loaded from JSON are
// downgraded to uniform on load, see file_io.cpp ParseShapeDist.)
constexpr int kShapeTableColumnCount = 5;

// Emit the Parameter (first) column of a shape-table row: strip any "##id" suffix from `label` and
// show the human-readable name. Shared so the randomizable rows (RenderShapeDistTableRow) and the
// structurally-non-randomizable wedge rows (edit_modals.cpp) render their name column identically.
// Must be called right after TableNextColumn() for the Parameter column.
void ShapeTableParamLabel(const char* label);

// Render one crystal shape distribution as a property-table row:
//   [Param label] [center slider+input] [sync-group swatch] [Randomize checkbox] [spread input]
// The spread column is wrapped in BeginDisabled(!randomize) so a not-yet-randomized field still
// shows it (greyed = "available to turn on"), matching the owner's "everything on the surface"
// design — no hidden state, the table stays a regular rectangle. Enabling randomization sets the
// distribution to Uniform with spread = 0.2 × center (the GUI is uniform-only); disabling collapses
// to NO_RANDOM and zeroes the (now meaningless) spread. Must be called between BeginTable/EndTable,
// and advances exactly kShapeTableColumnCount columns — every row helper in this table (this one and
// the file-local wedge row) must honor that column count or ImGui silently misaligns the grid.
//
// Takes the whole `cr` plus the LUMICE_SHAPE_SCALAR_* `slot` this row edits, rather than the single
// ShapeDist& it used to: the Sync column's popup enumerates the OTHER scalars of the same crystal
// (group membership lists, leader lookup, next free group number), so the row needs to see its
// siblings. Resolution goes through gui_state.hpp's ShapeScalarAt — the one mapping authority —
// so no second slot→field table exists to drift.
// Callers MUST pass a named LUMICE_SHAPE_SCALAR_* constant, never a bare integer: the slot order is
// NOT CrystalConfig's field order (UPPER_H is slot 1, PRISM_H slot 2 — see the SLOT-ORDER TRAP note
// in gui_state.hpp), so a positional guess lands one pyramid height's grouping on the other.
// The center slider's range, format and scale are NOT passed in: they are a property of the slot,
// read from gui/shape_scalar_domain.hpp. They used to be four arguments spelled out at every call
// site, which made "what does Prism H allow" a fact about the caller rather than about the field.
// Returns true if any value changed. Does NOT call MarkDirty() — caller is responsible.
bool RenderShapeDistTableRow(const char* label, CrystalConfig& cr, int slot);

// ---- Axis preset classification ----

// Classify crystal axis configuration into a named preset (Parry/Column/Lowitz/Plate/Random/Custom).
std::string AxisPresetName(const CrystalConfig& c);

// Render filter summary text consumed by entry-card rows. Exposed so unit /
// GUI tests can assert the rendered string directly. Format spec lives next
// to the implementation in panels.cpp.
std::string FilterSummary(const std::optional<FilterConfig>& f);

// ---- Panel rendering ----

// Render a single entry card within a layer. Returns true if the delete button was clicked.
bool RenderEntryCard(GuiState& state, int layer_idx, int entry_idx);

// Render a full layer (collapsing header + entry cards + controls).
void RenderLayer(GuiState& state, int layer_idx);

// Scattering section (layer management, rendered inside left panel scroll area).
void RenderScatteringSection(GuiState& state);

// Scene controls (Sun + Simulation) rendered in the right panel Scene group.
void RenderSceneControls(GuiState& state);

// Reset all panel editing state: edit request, selection indices.
// Name kept for GUI test teardown compatibility (was pending-delete only, now broader).
void ResetPendingDeleteState();

// ---- ID-pool sharing helpers (task-gui-linked-entries) ----

// Count how many entries (across all layers) reference (crystal_id, filter_id).
// "Sharing" predicate = both ids equal (filter_id compared including nullopt).
int CountEntriesSharing(const GuiState& state, int crystal_id, const std::optional<int>& filter_id);

// Unlink the given entry from any shared crystal/filter pool slot: clone the
// current pool content into a fresh slot so the entry becomes independent.
// Returns true if the entry actually became disjoint from any other entry.
bool UnlinkEntryFromPool(GuiState& state, int layer_idx, int entry_idx);

// Complete a pick-mode share: copy source entry's (crystal_id, filter_id) onto
// the target entry. Returns true if any id actually changed.
bool ApplyPickLink(GuiState& state, GuiState::EntryRef source, GuiState::EntryRef target);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PANELS_HPP
