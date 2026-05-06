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
enum class EditTarget { kNone, kCrystal, kAxis, kFilter };

struct EditRequest {
  EditTarget target = EditTarget::kNone;
  int layer_idx = -1;
  int entry_idx = -1;
};

const EditRequest& GetEditRequest();
void ResetEditRequest();

// ---- Axis distribution controls (shared between panels and edit modals) ----

// Render axis distribution controls (combo + mean + std sliders).
// Returns true if any value changed. Does NOT call MarkDirty() — caller is responsible.
bool RenderAxisDist(const char* label, AxisDist& axis, float mean_min, float mean_max);

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

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PANELS_HPP
