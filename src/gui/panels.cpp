#include "gui/panels.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>

#include "gui/gui_state.hpp"
#include "imgui.h"

namespace lumice::gui {
namespace {

void RenderAxisDist(const char* label, AxisDist& axis) {
  ImGui::PushID(label);
  ImGui::Text("%s", label);
  ImGui::SameLine(100);

  const char* dist_names[] = { "Gauss", "Uniform" };
  int dist_type = static_cast<int>(axis.type);
  ImGui::PushItemWidth(80);
  if (ImGui::Combo("##type", &dist_type, dist_names, 2)) {
    axis.type = static_cast<AxisDistType>(dist_type);
  }
  ImGui::PopItemWidth();

  ImGui::PushItemWidth(-1);
  ImGui::SliderFloat("Mean", &axis.mean, -360.0f, 360.0f, "%.1f");
  if (axis.type == AxisDistType::kGauss) {
    ImGui::SliderFloat("Std", &axis.std, 0.0f, 180.0f, "%.1f");
  } else {
    ImGui::SliderFloat("Range", &axis.std, 0.0f, 360.0f, "%.1f");
  }
  ImGui::PopItemWidth();

  ImGui::PopID();
}

}  // namespace


// ========== Crystal Tab ==========

void RenderCrystalTab(GuiState& state) {
  // Crystal list
  ImGui::Text("Crystals");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50);
  if (ImGui::SmallButton("Add##crystal")) {
    CrystalConfig c;
    c.id = state.next_crystal_id++;
    state.crystals.push_back(c);
    state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
  }

  if (ImGui::BeginListBox("##crystal_list", ImVec2(-1, 80))) {
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

  // Delete button
  if (state.selected_crystal >= 0 && state.selected_crystal < static_cast<int>(state.crystals.size())) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Del##crystal")) {
      state.crystals.erase(state.crystals.begin() + state.selected_crystal);
      if (state.selected_crystal >= static_cast<int>(state.crystals.size())) {
        state.selected_crystal = static_cast<int>(state.crystals.size()) - 1;
      }
    }
  }

  ImGui::Separator();

  // Edit selected crystal
  if (state.selected_crystal < 0 || state.selected_crystal >= static_cast<int>(state.crystals.size())) {
    ImGui::TextDisabled("No crystal selected");
    return;
  }

  auto& cr = state.crystals[state.selected_crystal];

  // Type combo
  const char* type_names[] = { "Prism", "Pyramid" };
  int type_idx = static_cast<int>(cr.type);
  ImGui::PushItemWidth(-1);
  if (ImGui::Combo("Type##crystal", &type_idx, type_names, 2)) {
    cr.type = static_cast<CrystalType>(type_idx);
  }

  // Shape section
  if (ImGui::CollapsingHeader("Shape", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (cr.type == CrystalType::kPrism) {
      ImGui::SliderFloat("Height", &cr.height, 0.1f, 5.0f, "%.2f");
    } else {
      ImGui::SliderFloat("Prism H", &cr.prism_h, 0.0f, 5.0f, "%.2f");
      ImGui::SliderFloat("Upper H", &cr.upper_h, 0.0f, 1.0f, "%.2f");
      ImGui::SliderFloat("Lower H", &cr.lower_h, 0.0f, 1.0f, "%.2f");
      ImGui::InputInt3("Upper Indices", cr.upper_indices);
      ImGui::InputInt3("Lower Indices", cr.lower_indices);
    }
  }

  // Axis section
  if (ImGui::CollapsingHeader("Axis Distribution", ImGuiTreeNodeFlags_DefaultOpen)) {
    RenderAxisDist("Zenith", cr.zenith);
    ImGui::Spacing();
    RenderAxisDist("Azimuth", cr.azimuth);
    ImGui::Spacing();
    RenderAxisDist("Roll", cr.roll);
  }
  ImGui::PopItemWidth();
}


// ========== Scene Tab ==========

void RenderSceneTab(GuiState& state) {
  ImGui::PushItemWidth(-1);

  // Sun section
  if (ImGui::CollapsingHeader("Sun", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SliderFloat("Altitude", &state.sun.altitude, -90.0f, 90.0f, "%.1f");
    ImGui::SliderFloat("Azimuth##sun", &state.sun.azimuth, -180.0f, 180.0f, "%.1f");
    ImGui::SliderFloat("Diameter", &state.sun.diameter, 0.1f, 5.0f, "%.1f");
    ImGui::Combo("Spectrum", &state.sun.spectrum_index, kSpectrumNames, kSpectrumCount);
  }

  // Simulation section
  if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Infinite rays", &state.sim.infinite);
    if (!state.sim.infinite) {
      ImGui::SliderFloat("Ray num (M)", &state.sim.ray_num_millions, 0.1f, 100.0f, "%.1f");
    } else {
      ImGui::BeginDisabled();
      ImGui::SliderFloat("Ray num (M)", &state.sim.ray_num_millions, 0.1f, 100.0f, "%.1f");
      ImGui::EndDisabled();
    }
    ImGui::SliderInt("Max hits", &state.sim.max_hits, 1, 20);
  }

  // Scattering section
  if (ImGui::CollapsingHeader("Scattering", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (int li = 0; li < static_cast<int>(state.scattering.size()); li++) {
      auto& layer = state.scattering[li];
      ImGui::PushID(li);

      char layer_label[32];
      snprintf(layer_label, sizeof(layer_label), "Layer %d", li + 1);
      bool layer_open = ImGui::TreeNodeEx(layer_label, ImGuiTreeNodeFlags_DefaultOpen);

      ImGui::SameLine(ImGui::GetContentRegionAvail().x - 20);
      if (ImGui::SmallButton("X##layer")) {
        state.scattering.erase(state.scattering.begin() + li);
        ImGui::PopID();
        if (layer_open) {
          ImGui::TreePop();
        }
        break;
      }

      if (layer_open) {
        ImGui::SliderFloat("Probability", &layer.probability, 0.0f, 1.0f, "%.2f");

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
              }
            }
            ImGui::EndCombo();
          }

          ImGui::SliderFloat("Proportion", &entry.proportion, 0.0f, 100.0f, "%.1f");

          // Filter combo
          if (ImGui::BeginCombo("Filter", [&]() -> const char* {
                if (entry.filter_id < 0)
                  return "None";
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
            }
            for (auto& f : state.filters) {
              char item_label[32];
              snprintf(item_label, sizeof(item_label), "[%d] Raypath", f.id);
              if (ImGui::Selectable(item_label, entry.filter_id == f.id)) {
                entry.filter_id = f.id;
              }
            }
            ImGui::EndCombo();
          }

          ImGui::SameLine();
          if (ImGui::SmallButton("X##entry")) {
            layer.entries.erase(layer.entries.begin() + ei);
            ImGui::PopID();
            break;
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
        }

        ImGui::TreePop();
      }

      ImGui::PopID();
    }

    if (ImGui::SmallButton("+ Layer")) {
      ScatterLayer layer;
      layer.probability = 1.0f;
      state.scattering.push_back(layer);
    }
  }

  ImGui::PopItemWidth();
}


// ========== Render Tab ==========

void RenderRenderTab(GuiState& state) {
  // Renderer list
  ImGui::Text("Renderers");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50);
  if (ImGui::SmallButton("Add##render")) {
    RenderConfig r;
    r.id = state.next_renderer_id++;
    state.renderers.push_back(r);
    state.selected_renderer = static_cast<int>(state.renderers.size()) - 1;
  }

  if (ImGui::BeginListBox("##render_list", ImVec2(-1, 60))) {
    for (int i = 0; i < static_cast<int>(state.renderers.size()); i++) {
      auto& r = state.renderers[i];
      char label[64];
      snprintf(label, sizeof(label), "[%d] %s, FOV %.0f", r.id, kLensTypeNames[r.lens_type], r.fov);
      if (ImGui::Selectable(label, state.selected_renderer == i)) {
        state.selected_renderer = i;
      }
    }
    ImGui::EndListBox();
  }

  if (state.selected_renderer >= 0 && state.selected_renderer < static_cast<int>(state.renderers.size())) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Del##render")) {
      state.renderers.erase(state.renderers.begin() + state.selected_renderer);
      if (state.selected_renderer >= static_cast<int>(state.renderers.size())) {
        state.selected_renderer = static_cast<int>(state.renderers.size()) - 1;
      }
    }
  }

  ImGui::Separator();

  if (state.selected_renderer < 0 || state.selected_renderer >= static_cast<int>(state.renderers.size())) {
    ImGui::TextDisabled("No renderer selected");
    return;
  }

  auto& r = state.renderers[state.selected_renderer];
  ImGui::PushItemWidth(-1);

  // Simulation resolution
  {
    const char* res_labels[] = { "512", "1024", "2048", "4096" };
    ImGui::Combo("Sim Resolution", &r.sim_resolution_index, res_labels, kSimResolutionCount);
  }

  // Lens & View
  if (ImGui::CollapsingHeader("Lens & View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Combo("Lens Type", &r.lens_type, kLensTypeNames, kLensTypeCount);

    // FOV disabled for dual_* and rectangular
    bool fov_disabled = (r.lens_type >= 4);  // dual_* and rectangular
    if (fov_disabled) {
      ImGui::BeginDisabled();
    }
    ImGui::SliderFloat("FOV", &r.fov, 1.0f, 360.0f, "%.0f");
    if (fov_disabled) {
      ImGui::EndDisabled();
    }

    ImGui::SliderFloat("Elevation", &r.elevation, -90.0f, 90.0f, "%.1f");
    ImGui::SliderFloat("Azimuth##view", &r.azimuth, -180.0f, 180.0f, "%.1f");
    ImGui::SliderFloat("Roll##view", &r.roll, -180.0f, 180.0f, "%.1f");
  }

  // Appearance
  if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Combo("Visible", &r.visible, kVisibleNames, kVisibleCount);
    ImGui::ColorEdit3("Background", r.background);
    ImGui::ColorEdit3("Ray Color", r.ray_color);
    ImGui::SliderFloat("Opacity", &r.opacity, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Intensity", &r.intensity_factor, 0.1f, 10.0f, "%.1f");
  }

  ImGui::PopItemWidth();
}


// ========== Filter Tab ==========

void RenderFilterTab(GuiState& state) {
  // Filter list
  ImGui::Text("Filters");
  ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50);
  if (ImGui::SmallButton("Add##filter")) {
    FilterConfig f;
    f.id = state.next_filter_id++;
    state.filters.push_back(f);
    state.selected_filter = static_cast<int>(state.filters.size()) - 1;
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
    }
  }

  ImGui::Separator();

  if (state.selected_filter < 0 || state.selected_filter >= static_cast<int>(state.filters.size())) {
    ImGui::TextDisabled("No filter selected");
    return;
  }

  auto& f = state.filters[state.selected_filter];
  ImGui::PushItemWidth(-1);

  // Action
  ImGui::Combo("Action", &f.action, kFilterActionNames, kFilterActionCount);

  // Raypath input
  char raypath_buf[256];
  snprintf(raypath_buf, sizeof(raypath_buf), "%s", f.raypath_text.c_str());
  if (ImGui::InputText("Raypath", raypath_buf, sizeof(raypath_buf))) {
    f.raypath_text = raypath_buf;
  }
  ImGui::TextDisabled("Comma-separated face indices, e.g. 3,1,5,7,4");

  // Symmetry checkboxes
  ImGui::Text("Symmetry:");
  ImGui::SameLine();
  ImGui::Checkbox("P", &f.sym_p);
  ImGui::SameLine();
  ImGui::Checkbox("B", &f.sym_b);
  ImGui::SameLine();
  ImGui::Checkbox("D", &f.sym_d);

  ImGui::PopItemWidth();
}

}  // namespace lumice::gui
