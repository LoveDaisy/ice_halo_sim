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
bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt = "%.1f",
                     SliderScale scale = SliderScale::kLinear);

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

// Render just the distribution-type combo (the 5 randomized ShapeDistType values). Single source
// for the enum<->combo-string mapping and its `kCount` static_assert guard — RenderShapeDist below,
// the face_distance unified view, and the face_distance per-face view all call this instead of each
// hand-rolling the same combo string, so a future ShapeDistType addition only needs updating here.
// Calls SetNextComboPopupTopMost() internally. Returns true and writes the new value to *type if
// the user picked a different entry.
bool RenderShapeDistTypeCombo(const char* id, ShapeDistType* type);

// Render one crystal shape distribution: the center slider (always shown, using the caller's
// range/format/scale) + a "Randomize" checkbox + (when randomized) a distribution-type combo and
// a spread slider. Enabling randomization defaults to Uniform with spread = 0.2 × center (owner
// default); disabling collapses to NO_RANDOM and zeroes the (now meaningless) spread. Returns true
// if any value changed. Does NOT call MarkDirty() — caller is responsible.
//
// Unlike RenderAxisDist, the type combo's top-most-popup fix is handled INTERNALLY (the combo only
// renders when randomized, so the caller cannot reliably pre-queue it) — callers need NOT precede
// this with SetNextComboPopupTopMost().
bool RenderShapeDist(const char* label, ShapeDist& dist, float center_min, float center_max,
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
