#include "gui/panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "config/render_config.hpp"
#include "gui/app.hpp"
#include "gui/crystal_preview.hpp"
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
// Render a slider with nonlinear scale mapping (sqrt/log/loglinear/linear).
// Must be called between PushItemWidth/PopItemWidth. Does NOT clamp — caller must clamp after.
// Note: `fmt` is only used for kLinear mode; nonlinear modes display a blank slider label.
static bool RenderNonlinearSlider(const char* slider_id, float* value, float min_val, float max_val, const char* fmt,
                                  SliderScale scale) {
  bool changed = false;
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
  return changed;
}

// ---- Selection state ----
int g_selected_layer = 0;
int g_selected_entry = 0;

// ---- Edit request state ----
EditRequest g_edit_request;

}  // namespace

// ---- Epsilon for axis preset name matching (degrees) ----
namespace {
constexpr float kPresetEpsilon = 1.0f;

bool FloatNear(float a, float b) {
  return std::abs(a - b) <= kPresetEpsilon;
}

bool IsGaussType(AxisDistType t) {
  return t == AxisDistType::kGauss || t == AxisDistType::kGaussLegacy;
}
}  // namespace

// Infer axis orientation preset name from crystal config.
// Match order: Parry -> Column -> Lowitz -> Plate -> Random -> Custom
std::string AxisPresetName(const CrystalConfig& c) {
  auto zt = c.zenith.type;
  auto rt = c.roll.type;
  auto at = c.azimuth.type;

  bool z_gauss = IsGaussType(zt);
  bool az_full_uniform = (at == AxisDistType::kUniform) && FloatNear(c.azimuth.std, 360.0f);
  bool roll_locked = (rt == AxisDistType::kGauss || rt == AxisDistType::kGaussLegacy || rt == AxisDistType::kUniform) &&
                     FloatNear(c.roll.mean, 0.0f) && c.roll.std < 5.0f;

  // Parry: zenith≈90, std<30, roll locked, azimuth full uniform
  if (z_gauss && FloatNear(c.zenith.mean, 90.0f) && c.zenith.std < 30.0f && roll_locked && az_full_uniform) {
    return "Parry";
  }

  // Column: zenith≈90, std<30, azimuth full uniform (roll NOT locked)
  if (z_gauss && FloatNear(c.zenith.mean, 90.0f) && c.zenith.std < 30.0f && az_full_uniform) {
    return "Column";
  }

  // Lowitz: zenith≈0, std>30, roll locked, azimuth full uniform
  if (z_gauss && FloatNear(c.zenith.mean, 0.0f) && c.zenith.std > 30.0f && roll_locked && az_full_uniform) {
    return "Lowitz";
  }

  // Plate: zenith≈0, std<30, azimuth full uniform
  if (z_gauss && FloatNear(c.zenith.mean, 0.0f) && c.zenith.std < 30.0f && az_full_uniform) {
    return "Plate";
  }

  // Random: all three axes uniform with full range (std≈360)
  if (zt == AxisDistType::kUniform && FloatNear(c.zenith.std, 360.0f) && rt == AxisDistType::kUniform &&
      FloatNear(c.roll.std, 360.0f) && az_full_uniform) {
    return "Random";
  }

  return "Custom";
}

namespace {
// Generate filter summary text from filter config.
std::string FilterSummary(const std::optional<FilterConfig>& f) {
  if (!f.has_value()) {
    return "None";
  }
  const auto& fc = f.value();
  std::string result;
  result += (fc.action == 0) ? "In " : "Out ";

  if (fc.raypath_text.size() > 12) {
    result += fc.raypath_text.substr(0, 12) + "...";
  } else if (!fc.raypath_text.empty()) {
    result += fc.raypath_text;
  } else {
    result += "*";
  }

  std::string sym;
  if (fc.sym_p)
    sym += "P";
  if (fc.sym_b)
    sym += "B";
  if (fc.sym_d)
    sym += "D";
  if (!sym.empty()) {
    result += " " + sym;
  }
  return result;
}

// Helper: wrap ImGui control and mark dirty on change
#define DIRTY_IF(expr) \
  if (expr)            \
  state.MarkDirty()

// Card layout constants (kThumbnailSize is in gui_constants.hpp)
constexpr float kCardHeight = 84.0f;
constexpr float kCardSpacing = 4.0f;

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
  changed |= RenderNonlinearSlider(slider_id, value, min_val, max_val, fmt, scale);
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
  changed |= RenderNonlinearSlider(slider_id, value, min_val, max_val, fmt, scale);
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


// ---- Axis distribution controls (shared with edit modals) ----

bool RenderAxisDist(const char* label, AxisDist& axis, float mean_min, float mean_max) {
  bool changed = false;
  ImGui::PushID(label);
  ImGui::Text("%s", label);
  ImGui::SameLine(100);

  int dist_type = static_cast<int>(axis.type);
  auto prev_type = axis.type;

  static_assert(static_cast<int>(AxisDistType::kCount) == 5, "Update combo items when adding new AxisDistType");
  if (ImGui::Combo("##dist", &dist_type, "Gauss\0Uniform\0Zigzag\0Laplacian\0Gauss (legacy)\0")) {
    axis.type = static_cast<AxisDistType>(dist_type);
    changed = true;
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

  changed |= SliderWithInput("Mean", &axis.mean, mean_min, mean_max);

  switch (axis.type) {
    case AxisDistType::kGauss:
    case AxisDistType::kGaussLegacy:
      changed |= SliderWithInput("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kUniform:
      changed |= SliderWithInput("Range", &axis.std, 0.0f, 360.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kZigzag:
      changed |= SliderWithInput("Amplitude", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt);
      break;
    case AxisDistType::kLaplacian:
      changed |= SliderWithInput("Scale", &axis.std, 0.0f, 90.0f, "%.1f", SliderScale::kSqrt);
      break;
    default:
      changed |= SliderWithInput("Std", &axis.std, 0.0f, 180.0f, "%.1f", SliderScale::kSqrt);
      break;
  }

  ImGui::PopID();
  return changed;
}


// ---- Selection and edit accessors ----

const EditRequest& GetEditRequest() {
  return g_edit_request;
}

void ResetEditRequest() {
  g_edit_request = EditRequest{};
}

int GetSelectedLayerIdx() {
  return g_selected_layer;
}

int GetSelectedEntryIdx() {
  return g_selected_entry;
}

void SetSelectedLayerIdx(int idx) {
  g_selected_layer = idx;
}

void SetSelectedEntryIdx(int idx) {
  g_selected_entry = idx;
}

void ResetPendingDeleteState() {
  g_edit_request = EditRequest{};
  g_selected_layer = 0;
  g_selected_entry = 0;
}


// ========== Entry Card ==========

bool RenderEntryCard(GuiState& state, int layer_idx, int entry_idx) {
  auto& entry = state.layers[layer_idx].entries[entry_idx];
  bool is_selected = (g_selected_layer == layer_idx && g_selected_entry == entry_idx);

  ImGui::PushID(entry_idx);

  // Highlight selected card
  if (is_selected) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.25f, 0.35f, 0.55f, 0.4f));
  }

  ImGui::BeginChild("##card", ImVec2(0, kCardHeight), ImGuiChildFlags_Borders);

  // Click anywhere in card to select
  if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    if (!is_selected) {
      g_crystal_mesh_hash = -1;  // Force 3D preview refresh on selection change
    }
    g_selected_layer = layer_idx;
    g_selected_entry = entry_idx;
  }

  // Left column: crystal thumbnail (or grey placeholder if not yet rendered)
  ImVec2 thumb_pos = ImGui::GetCursorScreenPos();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  auto thumb_tex = g_thumbnail_cache.GetTexture(layer_idx, entry_idx);
  constexpr float kThumbSize = static_cast<float>(kThumbnailSize);
  if (thumb_tex != 0) {
    // OpenGL texture Y-axis is flipped relative to ImGui: uv0=(0,1) uv1=(1,0)
    draw_list->AddImage(static_cast<ImTextureID>(thumb_tex), thumb_pos,
                        ImVec2(thumb_pos.x + kThumbSize, thumb_pos.y + kThumbSize), ImVec2(0, 1), ImVec2(1, 0));
  } else {
    draw_list->AddRectFilled(thumb_pos, ImVec2(thumb_pos.x + kThumbSize, thumb_pos.y + kThumbSize),
                             IM_COL32(60, 60, 60, 255));
  }
  draw_list->AddRect(thumb_pos, ImVec2(thumb_pos.x + kThumbSize, thumb_pos.y + kThumbSize),
                     IM_COL32(100, 100, 100, 255));

  // Right column
  float right_x = thumb_pos.x + kThumbnailSize + ImGui::GetStyle().ItemSpacing.x;
  float right_w = ImGui::GetContentRegionAvail().x - kThumbnailSize - ImGui::GetStyle().ItemSpacing.x;

  ImGui::SetCursorScreenPos(ImVec2(right_x, thumb_pos.y));

  // Row 1: Crystal type + Edit
  const char* type_name = (entry.crystal.type == CrystalType::kPrism) ? "Prism" : "Pyramid";
  ImGui::Text("%s", type_name);
  ImGui::SameLine(right_w - 30);
  if (ImGui::SmallButton("E##cr")) {
    g_edit_request = { EditTarget::kCrystal, layer_idx, entry_idx };
  }

  // Row 2: Axis preset + Edit
  std::string preset = AxisPresetName(entry.crystal);
  ImGui::SetCursorScreenPos(ImVec2(right_x, thumb_pos.y + ImGui::GetTextLineHeightWithSpacing()));
  ImGui::Text("%s", preset.c_str());
  ImGui::SameLine(right_w - 30);
  if (ImGui::SmallButton("E##ax")) {
    g_edit_request = { EditTarget::kAxis, layer_idx, entry_idx };
  }

  // Row 3: Filter summary + Edit
  std::string filter_text = FilterSummary(entry.filter);
  ImGui::SetCursorScreenPos(ImVec2(right_x, thumb_pos.y + ImGui::GetTextLineHeightWithSpacing() * 2));
  ImGui::Text("%s", filter_text.c_str());
  ImGui::SameLine(right_w - 30);
  if (ImGui::SmallButton("E##fi")) {
    g_edit_request = { EditTarget::kFilter, layer_idx, entry_idx };
  }

  // Row 4: Proportion slider (compact)
  ImGui::SetCursorScreenPos(ImVec2(right_x, thumb_pos.y + ImGui::GetTextLineHeightWithSpacing() * 3));
  ImGui::PushItemWidth(right_w - 50);
  char prop_id[32];
  snprintf(prop_id, sizeof(prop_id), "##prop_%d_%d", layer_idx, entry_idx);
  if (ImGui::SliderFloat(prop_id, &entry.proportion, 0.0f, 100.0f, "%.1f")) {
    state.MarkDirty();
  }
  ImGui::PopItemWidth();

  ImGui::EndChild();  // ##card — must be unconditional

  if (is_selected) {
    ImGui::PopStyleColor();
  }

  // Buttons after the card (on same line or next line)
  // Copy button
  char copy_id[32];
  snprintf(copy_id, sizeof(copy_id), "Copy##%d_%d", layer_idx, entry_idx);
  if (ImGui::SmallButton(copy_id)) {
    // Deep copy this entry and append to the same layer
    auto& entries = state.layers[layer_idx].entries;
    entries.push_back(entry);  // copy
    g_thumbnail_cache.OnLayerStructureChanged();
    state.MarkDirty();
  }

  // Delete button (returns whether clicked)
  ImGui::SameLine();
  char del_id[32];
  snprintf(del_id, sizeof(del_id), "x##%d_%d", layer_idx, entry_idx);
  bool delete_clicked = ImGui::SmallButton(del_id);

  ImGui::PopID();
  return delete_clicked;
}


// ========== Layer ==========

void RenderLayer(GuiState& state, int layer_idx) {
  auto& layer = state.layers[layer_idx];

  ImGui::PushID(layer_idx);

  char header_label[32];
  snprintf(header_label, sizeof(header_label), "Layer %d", layer_idx + 1);

  if (ImGui::CollapsingHeader(header_label, ImGuiTreeNodeFlags_DefaultOpen)) {
    // Multi-scatter probability slider
    char prob_id[32];
    snprintf(prob_id, sizeof(prob_id), "Prob.##layer_%d", layer_idx);
    DIRTY_IF(SliderWithInput(prob_id, &layer.probability, 0.0f, 1.0f, "%.2f"));

    // Render entry cards with deferred deletion
    int pending_delete_entry = -1;
    for (int i = 0; i < static_cast<int>(layer.entries.size()); i++) {
      ImGui::Spacing();
      bool del = RenderEntryCard(state, layer_idx, i);
      if (del) {
        pending_delete_entry = i;
      }
    }

    // Deferred delete
    if (pending_delete_entry >= 0 && layer.entries.size() > 1) {
      layer.entries.erase(layer.entries.begin() + pending_delete_entry);
      g_thumbnail_cache.OnLayerStructureChanged();
      // Clamp selected entry if needed
      if (g_selected_layer == layer_idx && g_selected_entry >= static_cast<int>(layer.entries.size())) {
        g_selected_entry = static_cast<int>(layer.entries.size()) - 1;
      }
      state.MarkDirty();
    }

    // Add entry button
    ImGui::Spacing();
    char add_id[32];
    snprintf(add_id, sizeof(add_id), "+ Entry##layer_%d", layer_idx);
    if (ImGui::SmallButton(add_id)) {
      layer.entries.emplace_back();
      g_thumbnail_cache.OnLayerStructureChanged();
      state.MarkDirty();
    }
  }

  ImGui::PopID();
}


// ========== Scattering Section (layer management) ==========

void RenderScatteringSection(GuiState& state) {
  for (int i = 0; i < static_cast<int>(state.layers.size()); i++) {
    RenderLayer(state, i);
    ImGui::Spacing();
  }
}


// ========== Scene Controls (rendered in the right panel Scene group) ==========

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

#undef DIRTY_IF

}  // namespace lumice::gui
