#include "gui/panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config/raypath_validation.hpp"
#include "config/render_config.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "imgui.h"
#include "lumice.h"

namespace lumice::gui {

namespace {
// LogLinear hybrid mapping constants.
// Tuned for prism_h [0, 100] range. Re-validate before reusing for other ranges.
// REQUIRES: min_val == 0, max_val > kLogLinearX0 at call site.
constexpr float kLogLinearX0 = 0.01f;       // Value threshold: linear below, log above
constexpr float kLogLinearTSwitch = 0.15f;  // Slider position threshold (fraction of [0,1])

// Log-scale: compute normalized [0,1] position from value in [min_val, max_val].
static float LogValueToNorm(float value, float min_val, float max_val) {
  value = std::max(value, min_val);
  float log_ratio = std::log(max_val / min_val);
  float norm = std::log(value / min_val) / log_ratio;
  return std::clamp(norm, 0.0f, 1.0f);
}

// Log-scale: compute value from normalized [0,1] position.
static float LogNormToValue(float norm, float min_val, float max_val) {
  float log_ratio = std::log(max_val / min_val);
  return min_val * std::exp(norm * log_ratio);
}

// LogLinear hybrid: compute normalized [0,1] position from value in [0, max_val].
// Linear in [0, x0], log in [x0, max_val], C0 continuous at x0.
static float LogLinearValueToNorm(float value, float max_val) {
  value = std::clamp(value, 0.0f, max_val);
  float log_ratio = std::log(max_val / kLogLinearX0);
  float norm;
  if (value <= kLogLinearX0) {
    norm = kLogLinearTSwitch * value / kLogLinearX0;
  } else {
    norm = kLogLinearTSwitch + (1.0f - kLogLinearTSwitch) * std::log(value / kLogLinearX0) / log_ratio;
  }
  return std::clamp(norm, 0.0f, 1.0f);
}

// LogLinear hybrid: compute value from normalized [0,1] position.
static float LogLinearNormToValue(float norm, float max_val) {
  float log_ratio = std::log(max_val / kLogLinearX0);
  if (norm <= kLogLinearTSwitch) {
    return kLogLinearX0 * norm / kLogLinearTSwitch;
  }
  float t_log = (norm - kLogLinearTSwitch) / (1.0f - kLogLinearTSwitch);
  return kLogLinearX0 * std::exp(t_log * log_ratio);
}
}  // namespace

// Compute slider width and prepare IDs for the [slider] [input] Label layout.
// Writes slider_id and input_id buffers, returns the computed slider width.
static float PrepareSliderLayout(const char* label, char* display_label_out, size_t display_buf_size, char* slider_id,
                                 size_t slider_id_size, char* input_id, size_t input_id_size) {
  // Strip ImGui ID suffix (e.g. "Azimuth##view" → display "Azimuth")
  const char* hash_pos = strstr(label, "##");
  if (hash_pos) {
    auto len = static_cast<size_t>(hash_pos - label);
    if (len >= display_buf_size)
      len = display_buf_size - 1;
    memcpy(display_label_out, label, len);
    display_label_out[len] = '\0';
  } else {
    snprintf(display_label_out, display_buf_size, "%s", label);
  }

  snprintf(slider_id, slider_id_size, "##%s_slider", label);
  snprintf(input_id, input_id_size, "##%s_input", label);

  float spacing = ImGui::GetStyle().ItemSpacing.x;
  float avail_w = ImGui::GetContentRegionAvail().x;
  float slider_w = avail_w - kInputWidth - kLabelColWidth - spacing * 2;
  if (slider_w < 40.0f)
    slider_w = 40.0f;
  return slider_w;
}

// Render the label text after slider + input.
static void FinishSliderLayout(const char* display_label) {
  ImGui::SameLine();
  ImGui::TextUnformatted(display_label);
}

bool SliderWithInput(const char* label, float* value, float min_val, float max_val, const char* fmt,
                     SliderScale scale) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id));

  bool changed = false;

  ImGui::PushItemWidth(slider_w);
  if (scale == SliderScale::kSqrt && min_val >= 0.0f) {
    // Sqrt-scale slider: more resolution at small values.
    // Slider operates on sqrt(value); actual value = slider_val^2.
    float sqrt_val = std::sqrt(std::max(*value, 0.0f));
    float sqrt_max = std::sqrt(max_val);
    if (ImGui::SliderFloat(slider_id, &sqrt_val, 0.0f, sqrt_max, "")) {
      *value = sqrt_val * sqrt_val;
      changed = true;
    }
  } else if (scale == SliderScale::kLog && min_val > 0.0f) {
    // Log-scale slider: uniform resolution across orders of magnitude.
    float norm = LogValueToNorm(*value, min_val, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "")) {
      *value = LogNormToValue(norm, min_val, max_val);
      changed = true;
    }
  } else if (scale == SliderScale::kLogLinear && min_val == 0.0f) {
    // LogLinear hybrid: linear near zero, log above x0. Allows reaching zero.
    float norm = LogLinearValueToNorm(*value, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "")) {
      *value = LogLinearNormToValue(norm, max_val);
      changed = true;
    }
  } else {
    changed |= ImGui::SliderFloat(slider_id, value, min_val, max_val, fmt);
  }
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return changed;
}

// SliderInt + InputInt + label text, same layout as SliderWithInput.
// Returns true if value changed.
static bool SliderIntWithInput(const char* label, int* value, int min_val, int max_val) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  float slider_w = PrepareSliderLayout(label, display_buf, sizeof(display_buf), slider_id, sizeof(slider_id), input_id,
                                       sizeof(input_id));

  bool changed = false;

  ImGui::PushItemWidth(slider_w);
  changed |= ImGui::SliderInt(slider_id, value, min_val, max_val);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::PushItemWidth(kInputWidth);
  changed |= ImGui::InputInt(input_id, value, 0, 0);
  ImGui::PopItemWidth();

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return changed;
}

struct ValuePreset {
  const char* label;
  float value;
};

// Slider + InputFloat with dropdown arrow for presets, laid out as: [slider] [input▼] Label
// Returns true if value changed.
static bool SliderWithPreset(const char* label, float* value, float min_val, float max_val, const char* fmt,
                             SliderScale scale, const ValuePreset* presets, int preset_count) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  constexpr float kArrowBtnWidth = 20.0f;
  float input_w = kInputWidth - kArrowBtnWidth;

  // Prepare layout (manually, since input area is split into input + arrow)
  const char* hash_pos = strstr(label, "##");
  if (hash_pos) {
    auto len = static_cast<size_t>(hash_pos - label);
    if (len >= sizeof(display_buf))
      len = sizeof(display_buf) - 1;
    memcpy(display_buf, label, len);
    display_buf[len] = '\0';
  } else {
    snprintf(display_buf, sizeof(display_buf), "%s", label);
  }
  snprintf(slider_id, sizeof(slider_id), "##%s_slider", label);
  snprintf(input_id, sizeof(input_id), "##%s_input", label);

  float spacing = ImGui::GetStyle().ItemSpacing.x;
  float avail_w = ImGui::GetContentRegionAvail().x;
  float slider_w = avail_w - kInputWidth - kArrowBtnWidth - kLabelColWidth - spacing * 3;
  if (slider_w < 40.0f)
    slider_w = 40.0f;

  bool changed = false;

  // Slider (nonlinear scale support)
  ImGui::PushItemWidth(slider_w);
  if (scale == SliderScale::kSqrt && min_val >= 0.0f) {
    float sqrt_val = std::sqrt(std::max(*value, 0.0f));
    float sqrt_max = std::sqrt(max_val);
    if (ImGui::SliderFloat(slider_id, &sqrt_val, 0.0f, sqrt_max, "")) {
      *value = sqrt_val * sqrt_val;
      changed = true;
    }
  } else if (scale == SliderScale::kLog && min_val > 0.0f) {
    float norm = LogValueToNorm(*value, min_val, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "")) {
      *value = LogNormToValue(norm, min_val, max_val);
      changed = true;
    }
  } else if (scale == SliderScale::kLogLinear && min_val == 0.0f) {
    float norm = LogLinearValueToNorm(*value, max_val);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "")) {
      *value = LogLinearNormToValue(norm, max_val);
      changed = true;
    }
  } else {
    changed |= ImGui::SliderFloat(slider_id, value, min_val, max_val, fmt);
  }
  ImGui::PopItemWidth();

  // Input
  ImGui::SameLine();
  ImGui::PushItemWidth(input_w);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  // Arrow button + preset popup
  char popup_id[64];
  snprintf(popup_id, sizeof(popup_id), "##%s_presets", label);
  ImGui::SameLine(0, 0);
  char arrow_id[64];
  snprintf(arrow_id, sizeof(arrow_id), "##%s_arrow", label);
  if (ImGui::ArrowButton(arrow_id, ImGuiDir_Down)) {
    ImGui::OpenPopup(popup_id);
  }
  if (ImGui::BeginPopup(popup_id)) {
    for (int i = 0; i < preset_count; i++) {
      bool selected = std::abs(*value - presets[i].value) < 0.05f;
      if (ImGui::Selectable(presets[i].label, selected)) {
        *value = presets[i].value;
        changed = true;
      }
    }
    ImGui::EndPopup();
  }

  *value = std::clamp(*value, min_val, max_val);

  FinishSliderLayout(display_buf);
  return changed;
}

namespace {

// Pending delete state for reference-warning popups
int g_pending_delete_crystal_idx = -1;
int g_pending_delete_filter_idx = -1;

// Inline editing state for double-click rename
int g_editing_crystal_idx = -1;
int g_editing_filter_idx = -1;
char g_editing_name_buf[64] = {};
bool g_editing_focus_needed = false;

void FormatCrystalLabel(char* buf, size_t size, int id, CrystalType type, const std::string& name) {
  if (name.empty()) {
    const char* t = type == CrystalType::kPrism ? "Prism" : "Pyramid";
    snprintf(buf, size, "[%d] %s", id, t);
  } else {
    snprintf(buf, size, "[%d] %s", id, name.c_str());
  }
}

void FormatFilterLabel(char* buf, size_t size, int id, const std::string& name) {
  if (name.empty()) {
    snprintf(buf, size, "[%d] Raypath", id);
  } else {
    snprintf(buf, size, "[%d] %s", id, name.c_str());
  }
}

// ID reference management functions removed — copy-model (EntryCard/Layer) eliminates the need.

// Helper: wrap ImGui control and mark dirty on change
#define DIRTY_IF(expr) \
  if (expr)            \
  state.MarkDirty()

void RenderAxisDist(const char* label, AxisDist& axis, GuiState& state, float mean_min, float mean_max) {
  ImGui::PushID(label);
  ImGui::Text("%s", label);
  ImGui::SameLine(100);

  int dist_type = static_cast<int>(axis.type);
  auto prev_type = axis.type;

  static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update combo items when adding new AxisDistType");
  if (ImGui::Combo("##dist", &dist_type, "Gauss\0Uniform\0Zigzag\0Laplacian\0Gauss (legacy)\0")) {
    axis.type = static_cast<AxisDistType>(dist_type);
    state.MarkDirty();
  }

  // Clamp std to new range when switching type.
  if (axis.type != prev_type) {
    float max_std = 0.0f;
    switch (axis.type) {
      case AxisDistType::kGauss:
        max_std = 180.0f;
        break;
      case AxisDistType::kUniform:
        max_std = 360.0f;
        break;
      case AxisDistType::kZigzag:
        max_std = 90.0f;
        break;
      case AxisDistType::kLaplacian:
        max_std = 90.0f;
        break;
      case AxisDistType::kGaussLegacy:
        max_std = 180.0f;
        break;
      default:
        max_std = 180.0f;
        break;
    }
    axis.std = std::min(axis.std, max_std);
  }

  DIRTY_IF(SliderWithInput("Mean", &axis.mean, mean_min, mean_max));

  switch (axis.type) {
    case AxisDistType::kGauss:
    case AxisDistType::kGaussLegacy:
      DIRTY_IF(SliderWithInput("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt));
      break;
    case AxisDistType::kUniform:
      DIRTY_IF(SliderWithInput("Range", &axis.std, 0.0f, 360.0f, "%.1f", SliderScale::kSqrt));
      break;
    case AxisDistType::kZigzag:
      DIRTY_IF(SliderWithInput("Amplitude", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt));
      break;
    case AxisDistType::kLaplacian:
      DIRTY_IF(SliderWithInput("Scale", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt));
      break;
    default:
      DIRTY_IF(SliderWithInput("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt));
      break;
  }

  ImGui::PopID();
}

}  // namespace

void ResetPendingDeleteState() {
  g_pending_delete_crystal_idx = -1;
  g_pending_delete_filter_idx = -1;
  g_editing_crystal_idx = -1;
  g_editing_filter_idx = -1;
}


// ========== Crystal Tab ==========

void RenderCrystalTab(GuiState& state) {
  // TODO(card-layout): stub - implement card-based rendering
  (void)state;
  ImGui::TextDisabled("Crystal editing moved to card layout (pending implementation)");
}

// ========== Scene Tab ==========
#if 0   // Old RenderCrystalTab code removed — see git history
[[maybe_unused]] static void DEAD_CODE_PLACEHOLDER_crystaltab() {
  ImGui::BeginDisabled(false);
  if (ImGui::SmallButton("Add##crystal")) {
    CrystalConfig c;
    c.id = state.next_crystal_id++;
    state.crystals.push_back(c);
    state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
    state.MarkDirty();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (state.selected_crystal >= 0 && state.selected_crystal < static_cast<int>(state.crystals.size()) &&
      state.crystals.size() > 1) {
    if (ImGui::SmallButton("Del##crystal")) {
      auto& cr_del = state.crystals[state.selected_crystal];
      if (IsCrystalReferenced(state, cr_del.id)) {
        g_pending_delete_crystal_idx = state.selected_crystal;
        ImGui::OpenPopup("Delete Crystal?");
      } else {
        g_editing_crystal_idx = -1;
        state.crystals.erase(state.crystals.begin() + state.selected_crystal);
        if (state.selected_crystal >= static_cast<int>(state.crystals.size())) {
          state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
        }
        state.MarkDirty();
      }
    }
  } else {
    ImGui::BeginDisabled();
    ImGui::SmallButton("Del##crystal");
    ImGui::EndDisabled();
  }

  int item_count = static_cast<int>(state.crystals.size());
  float line_h = ImGui::GetTextLineHeightWithSpacing();
  float list_h = std::min(line_h * std::max(item_count, 1) + ImGui::GetStyle().FramePadding.y * 2,
                          line_h * 4 + ImGui::GetStyle().FramePadding.y * 2);
  if (ImGui::BeginListBox("##crystal_list", ImVec2(-1, list_h))) {
    for (int i = 0; i < static_cast<int>(state.crystals.size()); i++) {
      auto& cr = state.crystals[i];
      ImGui::PushID(i);
      if (g_editing_crystal_idx == i) {
        if (g_editing_focus_needed) {
          ImGui::SetKeyboardFocusHere();
          g_editing_focus_needed = false;
        }
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##edit_name", g_editing_name_buf, sizeof(g_editing_name_buf),
                             ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll)) {
          cr.name = g_editing_name_buf;
          g_editing_crystal_idx = -1;
        } else if (ImGui::IsItemFocused() && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
          g_editing_crystal_idx = -1;
        } else if (ImGui::IsItemDeactivatedAfterEdit()) {
          cr.name = g_editing_name_buf;
          g_editing_crystal_idx = -1;
        }
      } else {
        char label[64];
        FormatCrystalLabel(label, sizeof(label), cr.id, cr.type, cr.name);
        if (ImGui::Selectable(label, state.selected_crystal == i, ImGuiSelectableFlags_AllowDoubleClick)) {
          state.selected_crystal = i;
          if (ImGui::IsMouseDoubleClicked(0)) {
            g_editing_crystal_idx = i;
            g_editing_filter_idx = -1;
            g_editing_focus_needed = true;
            strncpy(g_editing_name_buf, cr.name.c_str(), sizeof(g_editing_name_buf) - 1);
            g_editing_name_buf[sizeof(g_editing_name_buf) - 1] = '\0';
          }
        }
      }
      ImGui::PopID();
    }
    ImGui::EndListBox();
  }

  // Confirmation popup for deleting a referenced crystal
  if (ImGui::BeginPopupModal("Delete Crystal?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("This crystal is referenced by scattering entries.");
    if (g_pending_delete_crystal_idx >= 0 && g_pending_delete_crystal_idx < static_cast<int>(state.crystals.size())) {
      auto action = ClassifyCrystalRefAction(state, state.crystals[g_pending_delete_crystal_idx].id);
      switch (action) {
        case CrystalRefAction::kRemoveOnly:
          ImGui::Text("Those entries will be removed.");
          break;
        case CrystalRefAction::kReassignOnly:
          ImGui::Text("Those entries will be reassigned to another crystal.");
          break;
        case CrystalRefAction::kMixed:
          ImGui::Text("Entries in multi-entry layers will be removed;");
          ImGui::Text("sole entries will be reassigned to another crystal.");
          break;
      }
    }
    ImGui::Separator();
    if (ImGui::Button("Delete", ImVec2(80, 0))) {
      if (g_pending_delete_crystal_idx >= 0 && g_pending_delete_crystal_idx < static_cast<int>(state.crystals.size())) {
        int del_id = state.crystals[g_pending_delete_crystal_idx].id;
        HandleDeletedCrystalRefs(state, del_id);
        state.crystals.erase(state.crystals.begin() + g_pending_delete_crystal_idx);
        if (state.selected_crystal >= static_cast<int>(state.crystals.size())) {
          state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
        }
        state.MarkDirty();
      }
      g_pending_delete_crystal_idx = -1;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      g_pending_delete_crystal_idx = -1;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  ImGui::Separator();

  if (state.selected_crystal < 0 || state.selected_crystal >= static_cast<int>(state.crystals.size())) {
    ImGui::TextDisabled("No crystal selected");
    return;
  }

  auto& cr = state.crystals[state.selected_crystal];

  int type_idx = static_cast<int>(cr.type);
  ImGui::Text("Type");
  ImGui::SameLine();
  if (ImGui::RadioButton("Prism##type", &type_idx, 0)) {
    cr.type = static_cast<CrystalType>(type_idx);
    state.MarkDirty();
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Pyramid##type", &type_idx, 1)) {
    cr.type = static_cast<CrystalType>(type_idx);
    state.MarkDirty();
  }

  ImGui::SeparatorText("Shape");
  if (cr.type == CrystalType::kPrism) {
    DIRTY_IF(SliderWithInput("Height", &cr.height, 0.01f, 100.0f, "%.3f", SliderScale::kLog));
  } else {
    DIRTY_IF(SliderWithInput("Prism H", &cr.prism_h, 0.0f, 100.0f, "%.3f", SliderScale::kLogLinear));
    DIRTY_IF(SliderWithInput("Upper H", &cr.upper_h, 0.0f, 1.0f, "%.2f"));
    DIRTY_IF(SliderWithInput("Lower H", &cr.lower_h, 0.0f, 1.0f, "%.2f"));
  }

  if (ImGui::TreeNode("Advanced")) {
    if (cr.type == CrystalType::kPyramid) {
      // Wedge angle presets (common Miller indices for ice crystals)
      static constexpr ValuePreset kWedgePresets[] = {
        { "{1,0,-1,1}  28.0\xC2\xB0", 28.0f },  // atan(sqrt3/2 * 1/1 / 1.629)
        { "{2,0,-2,1}  14.7\xC2\xB0", 14.7f },  // atan(sqrt3/2 * 1/2 / 1.629)
        { "{3,0,-3,2}  19.9\xC2\xB0", 19.9f },  // atan(sqrt3/2 * 2/3 / 1.629)
      };
      static constexpr int kWedgePresetCount = sizeof(kWedgePresets) / sizeof(kWedgePresets[0]);
      DIRTY_IF(SliderWithPreset("Upper Wedge", &cr.upper_alpha, 0.1f, 89.9f, "%.1f", SliderScale::kSqrt, kWedgePresets,
                                kWedgePresetCount));
      DIRTY_IF(SliderWithPreset("Lower Wedge", &cr.lower_alpha, 0.1f, 89.9f, "%.1f", SliderScale::kSqrt, kWedgePresets,
                                kWedgePresetCount));
    }

    for (int i = 0; i < 6; i++) {
      char label[16];
      snprintf(label, sizeof(label), "Face %d", i + 3);
      DIRTY_IF(SliderWithInput(label, &cr.face_distance[i], 0.0f, 2.0f, "%.3f"));
    }
    ImGui::Spacing();
    if (ImGui::Button("Reset to Default##face_dist")) {
      bool any_changed = false;
      for (int i = 0; i < 6; i++) {
        if (std::abs(cr.face_distance[i] - 1.0f) > 1e-6f) {
          any_changed = true;
        }
        cr.face_distance[i] = 1.0f;
      }
      if (any_changed) {
        state.MarkDirty();
      }
    }
    ImGui::TreePop();
  }

  ImGui::SeparatorText("Axis Distribution");
  RenderAxisDist("Zenith", cr.zenith, state, 0.0f, 180.0f);
  ImGui::Spacing();
  RenderAxisDist("Roll", cr.roll, state, 0.0f, 360.0f);
  if (ImGui::TreeNode("Advanced##axis")) {
    RenderAxisDist("Azimuth", cr.azimuth, state, 0.0f, 360.0f);
    ImGui::TreePop();
  }
}
#endif  // Old RenderCrystalTab dead code


// ========== Scene Controls (shared between left Scene tab and right panel Scene group) ==========

void RenderSceneControls(GuiState& state) {
  ImGui::SeparatorText("Sun");
  DIRTY_IF(SliderWithInput("Altitude", &state.sun.altitude, -90.0f, 90.0f));
  DIRTY_IF(SliderWithInput("Diameter", &state.sun.diameter, 0.1f, 5.0f));
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  DIRTY_IF(ImGui::Combo("Spectrum", &state.sun.spectrum_index, kSpectrumNames, kSpectrumCount));
  ImGui::PopItemWidth();

  ImGui::SeparatorText("Simulation");
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  DIRTY_IF(ImGui::Checkbox("Infinite rays", &state.sim.infinite));
  ImGui::PopItemWidth();
  if (!state.sim.infinite) {
    DIRTY_IF(SliderWithInput("Rays(M)", &state.sim.ray_num_millions, 0.1f, 100.0f));
  } else {
    ImGui::BeginDisabled();
    SliderWithInput("Rays(M)", &state.sim.ray_num_millions, 0.1f, 100.0f);
    ImGui::EndDisabled();
  }
  DIRTY_IF(SliderIntWithInput("Max hits", &state.sim.max_hits, 1, 20));
}


// ========== Scene Tab ==========

void RenderSceneTab(GuiState& state) {
  (void)state;
  // Sun and Simulation controls have been migrated to the right panel Scene group.
  // TODO(card-layout): stub - implement card-based scattering/layer rendering
  ImGui::SeparatorText("Scattering");
  ImGui::TextDisabled("Scattering editing moved to card layout (pending implementation)");
}


// ========== Filter Tab ==========

void RenderFilterTab(GuiState& state) {
  // TODO(card-layout): stub - implement card-based filter editing
  (void)state;
  ImGui::TextDisabled("Filter editing moved to card layout (pending implementation)");
}

#undef DIRTY_IF

}  // namespace lumice::gui
