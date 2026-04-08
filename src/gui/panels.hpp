#ifndef LUMICE_GUI_PANELS_HPP
#define LUMICE_GUI_PANELS_HPP

namespace lumice::gui {

struct GuiState;

// Slider scale modes for SliderWithInput
enum class SliderScale { kLinear, kSqrt, kLog };

// Slider + InputFloat + label text, laid out as: [slider] [input] Label
// Uses a fixed label column width so vertically stacked sliders align.
// Returns true if value changed.
bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt = "%.1f",
                     SliderScale scale = SliderScale::kLinear);

void RenderCrystalTab(GuiState& state);
void RenderSceneTab(GuiState& state);
void RenderFilterTab(GuiState& state);

// Reset pending-delete state (for test isolation)
void ResetPendingDeleteState();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_PANELS_HPP
