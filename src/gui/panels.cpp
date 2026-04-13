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
    // Slider operates on normalized [0,1] position; actual value = min * exp(norm * log(max/min)).
    *value = std::max(*value, min_val);  // Defend against 0/negative from InputFloat
    float log_ratio = std::log(max_val / min_val);
    float norm = std::log(*value / min_val) / log_ratio;
    norm = std::clamp(norm, 0.0f, 1.0f);
    if (ImGui::SliderFloat(slider_id, &norm, 0.0f, 1.0f, "")) {
      *value = min_val * std::exp(norm * log_ratio);
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

  // Slider (sqrt scale)
  ImGui::PushItemWidth(slider_w);
  if (scale == SliderScale::kSqrt && min_val >= 0.0f) {
    float sqrt_val = std::sqrt(std::max(*value, 0.0f));
    float sqrt_max = std::sqrt(max_val);
    if (ImGui::SliderFloat(slider_id, &sqrt_val, 0.0f, sqrt_max, "")) {
      *value = sqrt_val * sqrt_val;
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

// Check if a crystal ID is referenced by any scattering entry
bool IsCrystalReferenced(const GuiState& state, int crystal_id) {
  for (auto& layer : state.scattering) {
    for (auto& entry : layer.entries) {
      if (entry.crystal_id == crystal_id)
        return true;
    }
  }
  return false;
}

// Check if a filter ID is referenced by any scattering entry
bool IsFilterReferenced(const GuiState& state, int filter_id) {
  for (auto& layer : state.scattering) {
    for (auto& entry : layer.entries) {
      if (entry.filter_id == filter_id)
        return true;
    }
  }
  return false;
}

// Clear references to a crystal ID in all scattering entries
// Handle scattering references after deleting a crystal:
// - Multi-entry layer: remove the entry referencing deleted crystal
// - Single-entry layer: reassign to first available crystal
void HandleDeletedCrystalRefs(GuiState& state, int deleted_id) {
  // Find first crystal that is NOT the one being deleted
  int fallback_id = -1;
  for (auto& c : state.crystals) {
    if (c.id != deleted_id) {
      fallback_id = c.id;
      break;
    }
  }
  for (auto& layer : state.scattering) {
    if (layer.entries.size() > 1) {
      layer.entries.erase(std::remove_if(layer.entries.begin(), layer.entries.end(),
                                         [deleted_id](const ScatterEntry& e) { return e.crystal_id == deleted_id; }),
                          layer.entries.end());
    } else {
      for (auto& entry : layer.entries) {
        if (entry.crystal_id == deleted_id) {
          entry.crystal_id = fallback_id;
        }
      }
    }
  }
}

// Describe what will happen when deleting a referenced crystal.
enum class CrystalRefAction { kRemoveOnly, kReassignOnly, kMixed };
CrystalRefAction ClassifyCrystalRefAction(const GuiState& state, int crystal_id) {
  bool has_remove = false;
  bool has_reassign = false;
  for (auto& layer : state.scattering) {
    for (auto& entry : layer.entries) {
      if (entry.crystal_id == crystal_id) {
        if (layer.entries.size() > 1) {
          has_remove = true;
        } else {
          has_reassign = true;
        }
      }
    }
  }
  if (has_remove && has_reassign)
    return CrystalRefAction::kMixed;
  if (has_reassign)
    return CrystalRefAction::kReassignOnly;
  return CrystalRefAction::kRemoveOnly;
}

// Clear references to a filter ID in all scattering entries
void ClearFilterReferences(GuiState& state, int filter_id) {
  for (auto& layer : state.scattering) {
    for (auto& entry : layer.entries) {
      if (entry.filter_id == filter_id)
        entry.filter_id = -1;
    }
  }
}

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
  ImGui::Text("Crystals");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 80);
  ImGui::BeginDisabled(static_cast<int>(state.crystals.size()) >= LUMICE_MAX_CONFIG_CRYSTALS);
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
    DIRTY_IF(SliderWithInput("Prism H", &cr.prism_h, 0.0f, 100.0f, "%.3f", SliderScale::kSqrt));
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


// ========== Scene Tab ==========

void RenderSceneTab(GuiState& state) {
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

  ImGui::SeparatorText("Scattering");
  // Align combo right edge with SliderWithInput's input right edge
  ImGui::PushItemWidth(-(kLabelColWidth + ImGui::GetStyle().ItemSpacing.x));
  for (int li = 0; li < static_cast<int>(state.scattering.size()); li++) {
    auto& layer = state.scattering[li];
    ImGui::PushID(li);

    char layer_label[32];
    snprintf(layer_label, sizeof(layer_label), "Layer %d", li + 1);
    bool layer_open = ImGui::TreeNodeEx(layer_label, ImGuiTreeNodeFlags_DefaultOpen);

    // Don't allow deleting the last layer (Core requires at least one)
    if (state.scattering.size() > 1) {
      ImGui::SameLine(ImGui::GetContentRegionAvail().x - 20);
      if (ImGui::SmallButton("X##layer")) {
        state.scattering.erase(state.scattering.begin() + li);
        // If only one layer remains, force its probability to 0
        if (state.scattering.size() == 1) {
          state.scattering[0].probability = 0.0f;
        }
        state.MarkDirty();
        ImGui::PopID();
        if (layer_open) {
          ImGui::TreePop();
        }
        break;
      }
    }

    if (layer_open) {
      // Probability — layer-level control, visually separated from entries
      bool single_layer = (state.scattering.size() == 1);
      if (single_layer) {
        layer.probability = 0.0f;
        ImGui::BeginDisabled();
        SliderWithInput("Prob.", &layer.probability, 0.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();
      } else {
        DIRTY_IF(SliderWithInput("Prob.", &layer.probability, 0.0f, 1.0f, "%.2f"));
      }
      ImGui::SameLine();
      ImGui::TextDisabled("(?)");
      if (ImGui::IsItemHovered()) {
        if (single_layer) {
          ImGui::SetTooltip("Fraction of rays continuing to the next layer.\nAlways 0 for a single layer.");
        } else {
          ImGui::SetTooltip("Fraction of rays continuing to the next layer");
        }
      }
      ImGui::Separator();

      for (int ei = 0; ei < static_cast<int>(layer.entries.size()); ei++) {
        auto& entry = layer.entries[ei];
        ImGui::PushID(ei);

        // Crystal combo
        if (ImGui::BeginCombo("Crystal", [&]() -> const char* {
              for (auto& c : state.crystals) {
                if (c.id == entry.crystal_id) {
                  static char buf[64];
                  FormatCrystalLabel(buf, sizeof(buf), c.id, c.type, c.name);
                  return buf;
                }
              }
              return "None";
            }())) {
          for (auto& c : state.crystals) {
            char item_label[64];
            FormatCrystalLabel(item_label, sizeof(item_label), c.id, c.type, c.name);
            if (ImGui::Selectable(item_label, entry.crystal_id == c.id)) {
              entry.crystal_id = c.id;
              state.MarkDirty();
            }
          }
          ImGui::EndCombo();
        }

        DIRTY_IF(SliderWithInput("Prop.", &entry.proportion, 0.0f, 100.0f));

        // Filter combo
        if (ImGui::BeginCombo("Filter", [&]() -> const char* {
              if (entry.filter_id < 0) {
                return "None";
              }
              for (auto& f : state.filters) {
                if (f.id == entry.filter_id) {
                  static char buf[64];
                  FormatFilterLabel(buf, sizeof(buf), f.id, f.name);
                  return buf;
                }
              }
              return "None";
            }())) {
          if (ImGui::Selectable("None", entry.filter_id < 0)) {
            entry.filter_id = -1;
            state.MarkFilterDirty();
          }
          for (auto& f : state.filters) {
            char item_label[64];
            FormatFilterLabel(item_label, sizeof(item_label), f.id, f.name);
            if (ImGui::Selectable(item_label, entry.filter_id == f.id)) {
              entry.filter_id = f.id;
              state.MarkFilterDirty();
            }
          }
          ImGui::EndCombo();
        }

        // Don't allow deleting the last entry (Core requires at least one per layer)
        if (layer.entries.size() > 1) {
          ImGui::SameLine();
          if (ImGui::SmallButton("X##entry")) {
            layer.entries.erase(layer.entries.begin() + ei);
            state.MarkDirty();
            ImGui::PopID();
            break;
          }
        }

        ImGui::PopID();
        ImGui::Separator();
      }

      ImGui::BeginDisabled(static_cast<int>(layer.entries.size()) >= LUMICE_MAX_CONFIG_SCATTER_ENTRIES);
      if (ImGui::SmallButton("+ Entry")) {
        ScatterEntry e;
        if (!state.crystals.empty()) {
          e.crystal_id = state.crystals[0].id;
        }
        layer.entries.push_back(e);
        state.MarkDirty();
      }
      ImGui::EndDisabled();

      ImGui::TreePop();
    }

    ImGui::PopID();
  }

  ImGui::BeginDisabled(static_cast<int>(state.scattering.size()) >= LUMICE_MAX_CONFIG_SCATTER_LAYERS);
  if (ImGui::SmallButton("+ Layer")) {
    ScatterLayer new_layer;
    ScatterEntry e;
    if (!state.crystals.empty()) {
      e.crystal_id = state.crystals[0].id;
    }
    new_layer.entries.push_back(e);
    state.scattering.push_back(new_layer);
    state.MarkDirty();
  }
  ImGui::EndDisabled();
  ImGui::PopItemWidth();
}


// ========== Filter Tab ==========

void RenderFilterTab(GuiState& state) {
  ImGui::Text("Filters");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 80);
  ImGui::BeginDisabled(static_cast<int>(state.filters.size()) >= LUMICE_MAX_CONFIG_FILTERS);
  if (ImGui::SmallButton("Add##filter")) {
    FilterConfig f;
    f.id = state.next_filter_id++;
    state.filters.push_back(f);
    state.selected_filter = static_cast<int>(state.filters.size()) - 1;
    state.MarkDirty();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (state.selected_filter >= 0 && state.selected_filter < static_cast<int>(state.filters.size())) {
    if (ImGui::SmallButton("Del##filter")) {
      auto& f_del = state.filters[state.selected_filter];
      if (IsFilterReferenced(state, f_del.id)) {
        g_pending_delete_filter_idx = state.selected_filter;
        ImGui::OpenPopup("Delete Filter?");
      } else {
        g_editing_filter_idx = -1;
        state.filters.erase(state.filters.begin() + state.selected_filter);
        if (state.selected_filter >= static_cast<int>(state.filters.size())) {
          state.selected_filter = static_cast<int>(state.filters.size()) - 1;
        }
        state.MarkFilterDirty();
      }
    }
  } else {
    ImGui::BeginDisabled();
    ImGui::SmallButton("Del##filter");
    ImGui::EndDisabled();
  }

  int filter_count = static_cast<int>(state.filters.size());
  float filter_line_h = ImGui::GetTextLineHeightWithSpacing();
  float filter_list_h = std::min(filter_line_h * std::max(filter_count, 1) + ImGui::GetStyle().FramePadding.y * 2,
                                 filter_line_h * 8 + ImGui::GetStyle().FramePadding.y * 2);
  if (ImGui::BeginListBox("##filter_list", ImVec2(-1, filter_list_h))) {
    for (int i = 0; i < static_cast<int>(state.filters.size()); i++) {
      auto& f = state.filters[i];
      ImGui::PushID(i);
      if (g_editing_filter_idx == i) {
        if (g_editing_focus_needed) {
          ImGui::SetKeyboardFocusHere();
          g_editing_focus_needed = false;
        }
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##edit_name", g_editing_name_buf, sizeof(g_editing_name_buf),
                             ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll)) {
          f.name = g_editing_name_buf;
          g_editing_filter_idx = -1;
        } else if (ImGui::IsItemFocused() && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
          g_editing_filter_idx = -1;
        } else if (ImGui::IsItemDeactivatedAfterEdit()) {
          f.name = g_editing_name_buf;
          g_editing_filter_idx = -1;
        }
      } else {
        char label[64];
        FormatFilterLabel(label, sizeof(label), f.id, f.name);
        if (ImGui::Selectable(label, state.selected_filter == i, ImGuiSelectableFlags_AllowDoubleClick)) {
          state.selected_filter = i;
          if (ImGui::IsMouseDoubleClicked(0)) {
            g_editing_filter_idx = i;
            g_editing_crystal_idx = -1;
            g_editing_focus_needed = true;
            strncpy(g_editing_name_buf, f.name.c_str(), sizeof(g_editing_name_buf) - 1);
            g_editing_name_buf[sizeof(g_editing_name_buf) - 1] = '\0';
          }
        }
      }
      ImGui::PopID();
    }
    ImGui::EndListBox();
  }

  // Confirmation popup for deleting a referenced filter
  if (ImGui::BeginPopupModal("Delete Filter?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("This filter is referenced by scattering entries.");
    ImGui::Text("Delete it and clear those references?");
    ImGui::Separator();
    if (ImGui::Button("Delete", ImVec2(80, 0))) {
      if (g_pending_delete_filter_idx >= 0 && g_pending_delete_filter_idx < static_cast<int>(state.filters.size())) {
        int del_id = state.filters[g_pending_delete_filter_idx].id;
        ClearFilterReferences(state, del_id);
        state.filters.erase(state.filters.begin() + g_pending_delete_filter_idx);
        if (state.selected_filter >= static_cast<int>(state.filters.size())) {
          state.selected_filter = static_cast<int>(state.filters.size()) - 1;
        }
        state.MarkFilterDirty();
      }
      g_pending_delete_filter_idx = -1;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(80, 0))) {
      g_pending_delete_filter_idx = -1;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  ImGui::Separator();

  if (state.selected_filter < 0 || state.selected_filter >= static_cast<int>(state.filters.size())) {
    ImGui::TextDisabled("No filter selected");
    return;
  }

  auto& f = state.filters[state.selected_filter];
  ImGui::PushItemWidth(-80);

  if (ImGui::Combo("Action", &f.action, kFilterActionNames, kFilterActionCount)) {
    state.MarkFilterDirty();
  }

  // Validate current raypath text; background color reflects previous-frame value (normal for ImGui immediate mode).
  auto validation = ValidateRaypathText(f.raypath_text);
  int style_pushes = 0;
  if (validation == RaypathValidation::kIncomplete) {
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.6f, 0.4f, 0.1f, 0.6f));  // Orange
    style_pushes = 1;
  } else if (validation == RaypathValidation::kInvalid) {
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.7f, 0.15f, 0.15f, 0.6f));  // Red
    style_pushes = 1;
  }

  char raypath_buf[256];
  snprintf(raypath_buf, sizeof(raypath_buf), "%s", f.raypath_text.c_str());
  if (ImGui::InputText("Raypath", raypath_buf, sizeof(raypath_buf))) {
    f.raypath_text = raypath_buf;
    // Update validation to current-frame value so hint text is accurate this frame.
    validation = ValidateRaypathText(f.raypath_text);
    if (validation == RaypathValidation::kValid) {
      state.MarkFilterDirty();
    }
  }
  ImGui::PopStyleColor(style_pushes);

  if (validation == RaypathValidation::kIncomplete) {
    ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.2f, 1.0f), "Incomplete input");
  } else if (validation == RaypathValidation::kInvalid) {
    ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "Invalid input");
  } else {
    ImGui::TextDisabled("Face indices separated by '-', e.g. 3-1-5-7-4 (comma also accepted)");
  }

  ImGui::Text("Symmetry:");
  ImGui::SameLine();
  if (ImGui::Checkbox("P", &f.sym_p)) {
    state.MarkFilterDirty();
  }
  ImGui::SameLine();
  if (ImGui::Checkbox("B", &f.sym_b)) {
    state.MarkFilterDirty();
  }
  ImGui::SameLine();
  if (ImGui::Checkbox("D", &f.sym_d)) {
    state.MarkFilterDirty();
  }

  ImGui::PopItemWidth();
}

#undef DIRTY_IF

}  // namespace lumice::gui
