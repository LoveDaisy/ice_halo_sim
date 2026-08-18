#ifndef LUMICE_GUI_PANELS_HPP
#define LUMICE_GUI_PANELS_HPP

// Full include (not forward declaration) because RenderAxisDist takes AxisDist by reference,
// requiring the complete type definition. This propagates gui_state.hpp to all includers.
#include "gui/gui_state.hpp"

namespace lumice::gui {

// Slider scale modes for SliderWithInput
enum class SliderScale { kLinear, kSqrt, kLog, kLogLinear };

// Slider + InputFloat + label text, laid out as: [slider] [input] Label
// Uses a fixed label column width so vertically stacked sliders align.
// Returns true if value changed.
//
// `trailing_label = false` (table-cell mode): omit the trailing text label and let
// the [slider][input] pair fill the whole content region — used when the field's
// name already lives in a dedicated table column (see RenderShapeDistTableRow). The
// default keeps the ~25 existing panel call sites (Axis / Sun / Simulation / View /
// Display) byte-for-byte unchanged.
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
bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt = "%.1f",
                     SliderScale scale = SliderScale::kLinear, bool trailing_label = true, bool* committed = nullptr,
                     bool* active = nullptr);

// SliderInt + InputInt + label text — SliderWithInput's integer sibling, same layout and the same
// `trailing_label = false` table-cell mode and the same optional `committed` / `active` out-params.
// Returns true if the value changed.
//
// Exported (it was file-static in panels.cpp) because the defaults panel's field-editor registry
// renders integer settings with it: an integer setting has to be edited by the SAME control the
// main UI uses, or the two disagree about what a valid value is — which is the whole point of that
// registry.
//
// `total_width > 0` sizes the [slider][input](label) triple explicitly instead of from the content
// region. Needed only in a horizontal toolbar row, where "the content region" is the distance to
// the window's right edge and would swallow everything after it.
bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val, bool trailing_label = true,
                        bool* committed = nullptr, bool* active = nullptr, float total_width = 0.0f);

// The ray-budget control: [slider][input] Rays(M), where the slider's rightmost band is a detent
// that reads "until stopped" instead of a number and sets sim.infinite.
//
// It is a bespoke control for ONE field, not a general SliderWithInput mode, and it should stay
// that way: the reason it exists is that this field's domain has a non-numeric top end, which no
// other field editable through SliderWithInput has. Two properties are load-bearing —
//   - the detent's boundary is closed on the infinite side, so the largest FINITE value cannot be
//     reached by dragging at all (not "is hard to hit"), and
//   - the input box is never disabled, so that value stays typeable —
// which together are what keeps ∞ a termination mode rather than a synonym for "a big number"
// (doc/gui-visual-language.md §4.5). Disabling the input box while the detent is engaged, or
// snapping a near-max drag up to max, each individually breaks that.
//
// Returns true if sim.infinite or sim.ray_num_millions changed this frame.
bool RaysBudgetControl(GuiState& state, float total_width = 0.0f);

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

// Render one entry as a compact tree row: thumbnail, "Type · AxisPreset", the weight, the sharing
// and filter badges, and hover-revealed duplicate / delete. Clicking the row selects the entry (or,
// while a pick is in flight, completes the link). Returns true if the delete button was clicked;
// the caller performs the erase, because deleting inside the loop that renders the rows would
// invalidate it.
bool RenderEntryRow(GuiState& state, int layer_idx, int entry_idx);

// Render a full layer: a foldable tree node (arrow folds, row selects) plus its entry rows and the
// "+ Crystal" button.
void RenderLayer(GuiState& state, int layer_idx);

// The document tree's contents — Sun, Camera, then the scattering layers. Rendered inside the
// document-tree window's scroll area (app_panels.cpp).
void RenderDocumentTreeRows(GuiState& state);

// The inspector's Layer page: the multi-scatter probability, how many crystals the layer holds, and
// a delete button. Returns true when the user asked to delete the layer; the CALLER erases it, so
// that the selection fix-up has a single owner and this function never leaves its own `layer`
// reference dangling.
bool RenderLayerInspector(GuiState& state, int layer_idx);

// Sun controls, rendered on the inspector's Sun page.
void RenderSunControls(GuiState& state);

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
