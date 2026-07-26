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
bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt = "%.1f",
                     SliderScale scale = SliderScale::kLinear, bool trailing_label = true);

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
// how the "columns everywhere" invariant is enforced. Columns: Param | Value | Rand | Spread | Sync.
// (The GUI edits uniform-only — no distribution-type column; non-uniform types loaded from JSON are
// downgraded to uniform on load, see file_io.cpp ParseShapeDist.)
constexpr int kShapeTableColumnCount = 5;

// Emit the Parameter (first) column of a shape-table row: strip any "##id" suffix from `label` and
// show the human-readable name. Shared so the randomizable rows (RenderShapeDistTableRow) and the
// structurally-non-randomizable wedge rows (edit_modals.cpp) render their name column identically.
// Must be called right after TableNextColumn() for the Parameter column.
void ShapeTableParamLabel(const char* label);

// Render one crystal shape distribution as a property-table row:
//   [Param label] [center slider+input] [Randomize checkbox] [spread input]
// The spread column is wrapped in BeginDisabled(!randomize) so a not-yet-randomized field still
// shows it (greyed = "available to turn on"), matching the owner's "everything on the surface"
// design — no hidden state, the table stays a regular rectangle. Enabling randomization sets the
// distribution to Uniform with spread = 0.2 × center (the GUI is uniform-only); disabling collapses
// to NO_RANDOM and zeroes the (now meaningless) spread. Must be called between BeginTable/EndTable,
// and advances exactly kShapeTableColumnCount columns — every row helper in this table (this one and
// the file-local wedge row) must honor that column count or ImGui silently misaligns the grid.
// Returns true if any value changed. Does NOT call MarkDirty() — caller is responsible.
bool RenderShapeDistTableRow(const char* label, ShapeDist& dist, float center_min, float center_max,
                             const char* center_fmt = "%.3f", SliderScale center_scale = SliderScale::kLinear);

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
