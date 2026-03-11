#include "gui/panels.hpp"

#include <algorithm>
#include <cstdio>

#include "gui/gui_state.hpp"
#include "imgui.h"

namespace lumice::gui {

// Slider + InputFloat + label text, laid out as: [slider] [input] Label
// Uses a fixed label column width so vertically stacked sliders align.
// Returns true if value changed.
static bool SliderWithInput(const char* label, float* value, float min_val, float max_val,
                            const char* fmt = "%.1f") {
  // Strip ImGui ID suffix (e.g. "Azimuth##view" → display "Azimuth")
  const char* display_label = label;
  const char* hash_pos = strstr(label, "##");
  char display_buf[64];
  if (hash_pos) {
    auto len = static_cast<size_t>(hash_pos - label);
    if (len >= sizeof(display_buf)) len = sizeof(display_buf) - 1;
    memcpy(display_buf, label, len);
    display_buf[len] = '\0';
    display_label = display_buf;
  }

  constexpr float kLabelColWidth = 70.0f;  // Fixed width for label column
  constexpr float kInputWidth = 60.0f;
  float spacing = ImGui::GetStyle().ItemSpacing.x;
  float avail_w = ImGui::GetContentRegionAvail().x;
  float slider_w = avail_w - kInputWidth - kLabelColWidth - spacing * 2;
  if (slider_w < 40.0f) slider_w = 40.0f;

  bool changed = false;

  char slider_id[64];
  snprintf(slider_id, sizeof(slider_id), "##%s_slider", label);
  ImGui::PushItemWidth(slider_w);
  changed |= ImGui::SliderFloat(slider_id, value, min_val, max_val, fmt);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  char input_id[64];
  snprintf(input_id, sizeof(input_id), "##%s_input", label);
  ImGui::PushItemWidth(kInputWidth);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  ImGui::TextUnformatted(display_label);

  return changed;
}

namespace {

// Helper: wrap ImGui control and mark dirty on change
#define DIRTY_IF(expr) \
  if (expr)            \
  state.MarkDirty()

void RenderAxisDist(const char* label, AxisDist& axis, GuiState& state) {
  ImGui::PushID(label);
  ImGui::Text("%s", label);
  ImGui::SameLine(100);

  const char* dist_names[] = { "Gauss", "Uniform" };
  int dist_type = static_cast<int>(axis.type);
  ImGui::PushItemWidth(80);
  if (ImGui::Combo("##type", &dist_type, dist_names, 2)) {
    axis.type = static_cast<AxisDistType>(dist_type);
    state.MarkDirty();
  }
  ImGui::PopItemWidth();

  DIRTY_IF(SliderWithInput("Mean", &axis.mean, -360.0f, 360.0f));

  if (axis.type == AxisDistType::kGauss) {
    DIRTY_IF(SliderWithInput("Std", &axis.std, 0.0f, 180.0f));
  } else {
    DIRTY_IF(SliderWithInput("Range", &axis.std, 0.0f, 360.0f));
  }

  ImGui::PopID();
}

}  // namespace


// ========== Crystal Tab ==========

void RenderCrystalTab(GuiState& state) {
  ImGui::Text("Crystals");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50);
  if (ImGui::SmallButton("Add##crystal")) {
    CrystalConfig c;
    c.id = state.next_crystal_id++;
    state.crystals.push_back(c);
    state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
    state.MarkDirty();
  }

  int item_count = static_cast<int>(state.crystals.size());
  float line_h = ImGui::GetTextLineHeightWithSpacing();
  float list_h = std::min(line_h * std::max(item_count, 1) + ImGui::GetStyle().FramePadding.y * 2,
                          line_h * 4 + ImGui::GetStyle().FramePadding.y * 2);
  if (ImGui::BeginListBox("##crystal_list", ImVec2(-1, list_h))) {
    for (int i = 0; i < static_cast<int>(state.crystals.size()); i++) {
      auto& cr = state.crystals[i];
      char label[64];
      const char* type_str = cr.type == CrystalType::kPrism ? "Prism" : "Pyramid";
      snprintf(label, sizeof(label), "[%d] %s", cr.id, type_str);
      if (ImGui::Selectable(label, state.selected_crystal == i)) {
        state.selected_crystal = i;
      }
    }
    ImGui::EndListBox();
  }

  if (state.selected_crystal >= 0 && state.selected_crystal < static_cast<int>(state.crystals.size())) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Del##crystal")) {
      state.crystals.erase(state.crystals.begin() + state.selected_crystal);
      if (state.selected_crystal >= static_cast<int>(state.crystals.size())) {
        state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
      }
      state.MarkDirty();
    }
  }

  ImGui::Separator();

  if (state.selected_crystal < 0 || state.selected_crystal >= static_cast<int>(state.crystals.size())) {
    ImGui::TextDisabled("No crystal selected");
    return;
  }

  auto& cr = state.crystals[state.selected_crystal];

  const char* type_names[] = { "Prism", "Pyramid" };
  int type_idx = static_cast<int>(cr.type);
  ImGui::PushItemWidth(-50);
  if (ImGui::Combo("Type##crystal", &type_idx, type_names, 2)) {
    cr.type = static_cast<CrystalType>(type_idx);
    state.MarkDirty();
  }
  ImGui::PopItemWidth();

  if (ImGui::CollapsingHeader("Shape", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (cr.type == CrystalType::kPrism) {
      DIRTY_IF(SliderWithInput("Height", &cr.height, 0.1f, 5.0f, "%.2f"));
    } else {
      DIRTY_IF(SliderWithInput("Prism H", &cr.prism_h, 0.0f, 5.0f, "%.2f"));
      DIRTY_IF(SliderWithInput("Upper H", &cr.upper_h, 0.0f, 1.0f, "%.2f"));
      DIRTY_IF(SliderWithInput("Lower H", &cr.lower_h, 0.0f, 1.0f, "%.2f"));
      ImGui::PushItemWidth(-100);
      DIRTY_IF(ImGui::InputInt3("Upper Idx", cr.upper_indices));
      DIRTY_IF(ImGui::InputInt3("Lower Idx", cr.lower_indices));
      ImGui::PopItemWidth();
    }
  }

  if (ImGui::CollapsingHeader("Axis Distribution", ImGuiTreeNodeFlags_DefaultOpen)) {
    RenderAxisDist("Zenith", cr.zenith, state);
    ImGui::Spacing();
    RenderAxisDist("Azimuth", cr.azimuth, state);
    ImGui::Spacing();
    RenderAxisDist("Roll", cr.roll, state);
  }
}


// ========== Scene Tab ==========

void RenderSceneTab(GuiState& state) {
  if (ImGui::CollapsingHeader("Sun", ImGuiTreeNodeFlags_DefaultOpen)) {
    DIRTY_IF(SliderWithInput("Altitude", &state.sun.altitude, -90.0f, 90.0f));
    DIRTY_IF(SliderWithInput("Azimuth##sun", &state.sun.azimuth, -180.0f, 180.0f));
    DIRTY_IF(SliderWithInput("Diameter", &state.sun.diameter, 0.1f, 5.0f));
    ImGui::PushItemWidth(-100);
    DIRTY_IF(ImGui::Combo("Spectrum", &state.sun.spectrum_index, kSpectrumNames, kSpectrumCount));
    ImGui::PopItemWidth();
  }

  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-100);
    DIRTY_IF(ImGui::Checkbox("Infinite rays", &state.sim.infinite));
    if (!state.sim.infinite) {
      DIRTY_IF(ImGui::SliderFloat("Ray num (M)", &state.sim.ray_num_millions, 0.1f, 100.0f, "%.1f"));
    } else {
      ImGui::BeginDisabled();
      ImGui::SliderFloat("Ray num (M)", &state.sim.ray_num_millions, 0.1f, 100.0f, "%.1f");
      ImGui::EndDisabled();
    }
    DIRTY_IF(ImGui::SliderInt("Max hits", &state.sim.max_hits, 1, 20));
    ImGui::PopItemWidth();
  }

  if (ImGui::CollapsingHeader("Scattering", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-100);
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
          state.MarkDirty();
          ImGui::PopID();
          if (layer_open) {
            ImGui::TreePop();
          }
          break;
        }
      }

      if (layer_open) {
        bool single_layer = (state.scattering.size() == 1);
        if (single_layer) {
          layer.probability = 0.0f;
          ImGui::BeginDisabled();
          ImGui::SliderFloat("Probability", &layer.probability, 0.0f, 1.0f, "%.2f");
          ImGui::EndDisabled();
          ImGui::SameLine();
          ImGui::TextDisabled("(?)");
          if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Single layer: probability is always 0 (no multi-scatter)");
          }
        } else {
          DIRTY_IF(ImGui::SliderFloat("Probability", &layer.probability, 0.0f, 1.0f, "%.2f"));
        }

        for (int ei = 0; ei < static_cast<int>(layer.entries.size()); ei++) {
          auto& entry = layer.entries[ei];
          ImGui::PushID(ei);

          // Crystal combo
          if (ImGui::BeginCombo("Crystal", [&]() -> const char* {
                for (auto& c : state.crystals) {
                  if (c.id == entry.crystal_id) {
                    static char buf[32];
                    snprintf(buf, sizeof(buf), "[%d]", c.id);
                    return buf;
                  }
                }
                return "None";
              }())) {
            for (auto& c : state.crystals) {
              char item_label[64];
              const char* t = c.type == CrystalType::kPrism ? "Prism" : "Pyramid";
              snprintf(item_label, sizeof(item_label), "[%d] %s", c.id, t);
              if (ImGui::Selectable(item_label, entry.crystal_id == c.id)) {
                entry.crystal_id = c.id;
                state.MarkDirty();
              }
            }
            ImGui::EndCombo();
          }

          DIRTY_IF(ImGui::SliderFloat("Proportion", &entry.proportion, 0.0f, 100.0f, "%.1f"));

          // Filter combo
          if (ImGui::BeginCombo("Filter", [&]() -> const char* {
                if (entry.filter_id < 0) {
                  return "None";
                }
                for (auto& f : state.filters) {
                  if (f.id == entry.filter_id) {
                    static char buf[32];
                    snprintf(buf, sizeof(buf), "[%d]", f.id);
                    return buf;
                  }
                }
                return "None";
              }())) {
            if (ImGui::Selectable("None", entry.filter_id < 0)) {
              entry.filter_id = -1;
              state.MarkDirty();
            }
            for (auto& f : state.filters) {
              char item_label[32];
              snprintf(item_label, sizeof(item_label), "[%d] Raypath", f.id);
              if (ImGui::Selectable(item_label, entry.filter_id == f.id)) {
                entry.filter_id = f.id;
                state.MarkDirty();
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

        if (ImGui::SmallButton("+ Entry")) {
          ScatterEntry e;
          if (!state.crystals.empty()) {
            e.crystal_id = state.crystals[0].id;
          }
          layer.entries.push_back(e);
          state.MarkDirty();
        }

        ImGui::TreePop();
      }

      ImGui::PopID();
    }

    if (ImGui::SmallButton("+ Layer")) {
      ScatterLayer layer;
      layer.probability = 1.0f;
      state.scattering.push_back(layer);
      state.MarkDirty();
    }
    ImGui::PopItemWidth();
  }
}


// ========== Render Tab ==========

void RenderRenderTab(GuiState& state) {
  // Ensure at least one renderer exists
  if (state.renderers.empty()) {
    RenderConfig r;
    r.id = state.next_renderer_id++;
    state.renderers.push_back(r);
    state.selected_renderer = 0;
  }
  state.selected_renderer = 0;

  auto& r = state.renderers[0];

  if (ImGui::CollapsingHeader("Lens & View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-100);
    ImGui::Combo("Lens Type", &r.lens_type, kLensTypeNames, kLensTypeCount);
    ImGui::PopItemWidth();

    bool full_sky = (r.lens_type >= 4);  // dual fisheye (4-6) and rectangular (7)
    if (full_sky) {
      r.elevation = 0.0f;
      r.azimuth = 0.0f;
      r.roll = 0.0f;
      ImGui::BeginDisabled();
    }
    SliderWithInput("FOV", &r.fov, 1.0f, 360.0f, "%.0f");
    SliderWithInput("Elevation", &r.elevation, -90.0f, 90.0f);
    SliderWithInput("Azimuth##view", &r.azimuth, -180.0f, 180.0f);
    SliderWithInput("Roll##view", &r.roll, -180.0f, 180.0f);
    if (full_sky) {
      ImGui::EndDisabled();
    }

    ImGui::PushItemWidth(-100);
    ImGui::Combo("Visible", &r.visible, kVisibleNames, kVisibleCount);
    ImGui::SliderFloat("Opacity", &r.opacity, 0.0f, 1.0f, "%.2f");
    ImGui::PopItemWidth();
  }

  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(-100);
    const char* res_labels[] = { "512", "1024", "2048", "4096" };
    DIRTY_IF(ImGui::Combo("Sim Resolution", &r.sim_resolution_index, res_labels, kSimResolutionCount));
    DIRTY_IF(ImGui::SliderFloat("Intensity", &r.intensity_factor, 0.1f, 10.0f, "%.1f"));
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Changing these requires re-running the simulation");
    }
  }
}


// ========== Filter Tab ==========

void RenderFilterTab(GuiState& state) {
  ImGui::Text("Filters");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50);
  if (ImGui::SmallButton("Add##filter")) {
    FilterConfig f;
    f.id = state.next_filter_id++;
    state.filters.push_back(f);
    state.selected_filter = static_cast<int>(state.filters.size()) - 1;
    state.MarkDirty();
  }

  if (ImGui::BeginListBox("##filter_list", ImVec2(-1, 60))) {
    for (int i = 0; i < static_cast<int>(state.filters.size()); i++) {
      auto& f = state.filters[i];
      char label[32];
      snprintf(label, sizeof(label), "[%d] Raypath", f.id);
      if (ImGui::Selectable(label, state.selected_filter == i)) {
        state.selected_filter = i;
      }
    }
    ImGui::EndListBox();
  }

  if (state.selected_filter >= 0 && state.selected_filter < static_cast<int>(state.filters.size())) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Del##filter")) {
      state.filters.erase(state.filters.begin() + state.selected_filter);
      if (state.selected_filter >= static_cast<int>(state.filters.size())) {
        state.selected_filter = static_cast<int>(state.filters.size()) - 1;
      }
      state.MarkDirty();
    }
  }

  ImGui::Separator();

  if (state.selected_filter < 0 || state.selected_filter >= static_cast<int>(state.filters.size())) {
    ImGui::TextDisabled("No filter selected");
    return;
  }

  auto& f = state.filters[state.selected_filter];
  ImGui::PushItemWidth(-80);

  DIRTY_IF(ImGui::Combo("Action", &f.action, kFilterActionNames, kFilterActionCount));

  char raypath_buf[256];
  snprintf(raypath_buf, sizeof(raypath_buf), "%s", f.raypath_text.c_str());
  if (ImGui::InputText("Raypath", raypath_buf, sizeof(raypath_buf))) {
    f.raypath_text = raypath_buf;
    state.MarkDirty();
  }
  ImGui::TextDisabled("Comma-separated face indices, e.g. 3,1,5,7,4");

  ImGui::Text("Symmetry:");
  ImGui::SameLine();
  DIRTY_IF(ImGui::Checkbox("P", &f.sym_p));
  ImGui::SameLine();
  DIRTY_IF(ImGui::Checkbox("B", &f.sym_b));
  ImGui::SameLine();
  DIRTY_IF(ImGui::Checkbox("D", &f.sym_d));

  ImGui::PopItemWidth();
}

#undef DIRTY_IF

}  // namespace lumice::gui
