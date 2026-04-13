#include "gui/edit_modals.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

#include "config/raypath_validation.hpp"
#include "gui/app.hpp"
#include "gui/crystal_preview.hpp"
#include "gui/crystal_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "imgui.h"
#include "lumice.h"

namespace lumice::gui {

// ============================================================
// Modal state (file scope — not shared via gui_state.hpp)
// ============================================================

enum class ActiveModal { kNone, kCrystal, kAxis, kFilter };

static ActiveModal g_active_modal = ActiveModal::kNone;
static int g_modal_layer_idx = -1;
static int g_modal_entry_idx = -1;

// Edit buffers
static CrystalConfig g_crystal_buf;
static AxisDist g_axis_buf[3];  // zenith, azimuth, roll
static FilterConfig g_filter_buf;
static char g_raypath_buf[256];

// Crystal modal: trackball state saved on open, restored on Cancel
static float g_saved_rotation[16];
static float g_saved_zoom;

// Crystal modal: mesh hash for edit buffer change detection
static int g_modal_mesh_hash = 0;

// Flag: the OpenPopup call must happen exactly once per modal open,
// on the frame following the EditRequest.
static bool g_pending_open = false;

// ============================================================
// Wedge angle presets (same as in panels.cpp SliderWithPreset for alpha)
// ============================================================

namespace {

struct ValuePreset {
  const char* label;
  float value;
};

constexpr ValuePreset kWedgePresets[] = {
  { "{1,0,-1,1} 28.0\xc2\xb0", 28.0f },
  { "{2,0,-2,1} 47.3\xc2\xb0", 47.3f },
  { "{1,0,-1,0} 90.0\xc2\xb0", 90.0f },
  { "{1,0,-1,2} 14.7\xc2\xb0", 14.7f },
};
constexpr int kWedgePresetCount = 4;

// Update g_crystal_renderer mesh from the crystal edit buffer.
// Returns the computed hash.
int UpdateModalMesh(const CrystalConfig& cr) {
  char json_buf[512];
  auto* fd = cr.face_distance;
  if (cr.type == CrystalType::kPrism) {
    snprintf(json_buf, sizeof(json_buf),
             R"({"type":"prism","shape":{"height":%.4f,)"
             R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
             cr.height, fd[0], fd[1], fd[2], fd[3], fd[4], fd[5]);
  } else {
    snprintf(json_buf, sizeof(json_buf),
             R"({"type":"pyramid","shape":{"prism_h":%.4f,"upper_h":%.4f,"lower_h":%.4f,)"
             R"("upper_wedge_angle":%.4f,"lower_wedge_angle":%.4f,)"
             R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
             cr.prism_h, cr.upper_h, cr.lower_h, cr.upper_alpha, cr.lower_alpha, fd[0], fd[1], fd[2], fd[3], fd[4],
             fd[5]);
  }

  LUMICE_CrystalMesh mesh{};
  if (LUMICE_GetCrystalMesh(nullptr, json_buf, &mesh) == LUMICE_OK) {
    // Y-Z swap (same transform as app_panels.cpp)
    for (int vi = 0; vi < mesh.vertex_count; vi++) {
      float y = mesh.vertices[vi * 3 + 1];
      float z = mesh.vertices[vi * 3 + 2];
      mesh.vertices[vi * 3 + 1] = z;
      mesh.vertices[vi * 3 + 2] = -y;
    }
    for (int ei = 0; ei < mesh.edge_count; ei++) {
      for (int side = 0; side < 2; side++) {
        float* n = &mesh.edge_face_normals[ei * 6 + side * 3];
        float ny = n[1];
        float nz = n[2];
        n[1] = nz;
        n[2] = -ny;
      }
    }

    // AABB normalization
    if (mesh.vertex_count > 0) {
      float min_x = mesh.vertices[0], max_x = mesh.vertices[0];
      float min_y = mesh.vertices[1], max_y = mesh.vertices[1];
      float min_z = mesh.vertices[2], max_z = mesh.vertices[2];
      for (int vi = 1; vi < mesh.vertex_count; vi++) {
        float x = mesh.vertices[vi * 3];
        float y = mesh.vertices[vi * 3 + 1];
        float z = mesh.vertices[vi * 3 + 2];
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
      }
      float extent = std::max({ max_x - min_x, max_y - min_y, max_z - min_z });
      if (extent > 1e-6f) {
        float scale = 1.0f / extent;
        for (int vi = 0; vi < mesh.vertex_count; vi++) {
          mesh.vertices[vi * 3] *= scale;
          mesh.vertices[vi * 3 + 1] *= scale;
          mesh.vertices[vi * 3 + 2] *= scale;
        }
      }
    }

    g_crystal_renderer.UpdateMesh(mesh.vertices, mesh.vertex_count, mesh.edges, mesh.edge_count, mesh.triangles,
                                  mesh.triangle_count, mesh.edge_face_normals);
  }

  return CrystalParamHash(cr);
}

// Render a slider + input + preset dropdown for wedge angle.
// Simplified version from panels.cpp SliderWithPreset, operating on edit buffer (no MarkDirty).
bool SliderWithPresetEdit(const char* label, float* value, float min_val, float max_val, const char* fmt,
                          SliderScale scale, const ValuePreset* presets, int preset_count) {
  char display_buf[64];
  char slider_id[64];
  char input_id[64];
  constexpr float kArrowBtnWidth = 20.0f;
  float input_w = kInputWidth - kArrowBtnWidth;

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

  ImGui::SameLine();
  ImGui::PushItemWidth(input_w);
  changed |= ImGui::InputFloat(input_id, value, 0, 0, fmt);
  ImGui::PopItemWidth();

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

  ImGui::SameLine();
  ImGui::TextUnformatted(display_buf);
  return changed;
}

}  // namespace

// ============================================================
// OpenEditModal — called from RenderLeftPanel on EditRequest
// ============================================================

void OpenEditModal(const EditRequest& req, GuiState& state) {
  if (req.target == EditTarget::kNone) {
    return;
  }

  // Validate index
  int ly = req.layer_idx;
  int en = req.entry_idx;
  if (ly < 0 || ly >= static_cast<int>(state.layers.size())) {
    return;
  }
  if (en < 0 || en >= static_cast<int>(state.layers[ly].entries.size())) {
    return;
  }

  auto& entry = state.layers[ly].entries[en];
  g_modal_layer_idx = ly;
  g_modal_entry_idx = en;

  switch (req.target) {
    case EditTarget::kCrystal:
      g_active_modal = ActiveModal::kCrystal;
      g_crystal_buf = entry.crystal;
      // Save trackball state for Cancel restoration
      std::memcpy(g_saved_rotation, g_crystal_rotation, sizeof(g_saved_rotation));
      g_saved_zoom = g_crystal_zoom;
      g_modal_mesh_hash = 0;  // Force mesh update on first frame
      break;
    case EditTarget::kAxis:
      g_active_modal = ActiveModal::kAxis;
      g_axis_buf[0] = entry.crystal.zenith;
      g_axis_buf[1] = entry.crystal.azimuth;
      g_axis_buf[2] = entry.crystal.roll;
      break;
    case EditTarget::kFilter:
      g_active_modal = ActiveModal::kFilter;
      if (entry.filter.has_value()) {
        g_filter_buf = entry.filter.value();
      } else {
        g_filter_buf = FilterConfig{};
      }
      snprintf(g_raypath_buf, sizeof(g_raypath_buf), "%s", g_filter_buf.raypath_text.c_str());
      break;
    default:
      return;
  }

  g_pending_open = true;
}


// ============================================================
// Crystal Modal
// ============================================================

static void RenderCrystalModal(GuiState& state) {
  auto& cr = g_crystal_buf;

  // Update mesh if crystal params changed
  int hash = CrystalParamHash(cr);
  if (hash != g_modal_mesh_hash) {
    g_modal_mesh_hash = UpdateModalMesh(cr);
  }

  // Render to FBO
  auto crystal_style = static_cast<CrystalStyle>(g_crystal_style);
  g_crystal_renderer.Render(g_crystal_rotation, g_crystal_zoom, crystal_style);

  // Layout: controls on left, 3D preview on right
  constexpr float kPreviewSize = 200.0f;

  // -- 3D Preview --
  auto tex_id = static_cast<ImTextureID>(g_crystal_renderer.GetTextureId());
  ImGui::Image(tex_id, ImVec2(kPreviewSize, kPreviewSize), ImVec2(0, 1), ImVec2(1, 0));

  // Trackball interaction on preview
  ImVec2 preview_min = ImGui::GetItemRectMin();
  ImVec2 preview_max = ImGui::GetItemRectMax();
  if (ImGui::IsItemHovered()) {
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
    ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      ApplyTrackballRotation(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (io.MouseWheel != 0.0f) {
      g_crystal_zoom *= (1.0f - io.MouseWheel * 0.1f);
      g_crystal_zoom = std::max(0.5f, std::min(10.0f, g_crystal_zoom));
    }
  }

  // Style combo + Reset View
  ImGui::PushItemWidth(120.0f);
  ImGui::Combo("##ModalCrystalStyle", &g_crystal_style, kCrystalStyleNames, kCrystalStyleCount);
  ImGui::PopItemWidth();
  ImGui::SameLine();
  if (ImGui::SmallButton("Reset View##modal")) {
    ResetCrystalView();
  }

  ImGui::Separator();

  // -- Crystal type --
  int type_int = static_cast<int>(cr.type);
  if (ImGui::RadioButton("Prism##modal", &type_int, 0)) {
    cr.type = CrystalType::kPrism;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Pyramid##modal", &type_int, 1)) {
    cr.type = CrystalType::kPyramid;
  }

  ImGui::Spacing();

  // -- Parameters --
  if (cr.type == CrystalType::kPrism) {
    SliderWithInput("Height##modal_cr", &cr.height, 0.01f, 100.0f, "%.2f", SliderScale::kLogLinear);
  } else {
    SliderWithInput("Prism H##modal_cr", &cr.prism_h, 0.0f, 100.0f, "%.2f", SliderScale::kLogLinear);
    SliderWithInput("Upper H##modal_cr", &cr.upper_h, 0.0f, 100.0f, "%.2f", SliderScale::kLogLinear);
    SliderWithInput("Lower H##modal_cr", &cr.lower_h, 0.0f, 100.0f, "%.2f", SliderScale::kLogLinear);
    SliderWithPresetEdit("Upper A##modal_cr", &cr.upper_alpha, 0.1f, 90.0f, "%.1f", SliderScale::kLinear, kWedgePresets,
                         kWedgePresetCount);
    SliderWithPresetEdit("Lower A##modal_cr", &cr.lower_alpha, 0.1f, 90.0f, "%.1f", SliderScale::kLinear, kWedgePresets,
                         kWedgePresetCount);
  }

  // -- Face distance --
  if (ImGui::TreeNode("Face Distance##modal")) {
    for (int i = 0; i < 6; i++) {
      char label[32];
      snprintf(label, sizeof(label), "Face %d##modal_fd", i + 3);
      SliderWithInput(label, &cr.face_distance[i], 0.01f, 10.0f, "%.3f");
    }
    if (ImGui::SmallButton("Reset All##modal_fd")) {
      for (auto& fd : cr.face_distance) {
        fd = 1.0f;
      }
    }
    ImGui::TreePop();
  }

  ImGui::Separator();

  // -- OK / Cancel --
  if (ImGui::Button("OK##crystal", ImVec2(80, 0))) {
    int ly = g_modal_layer_idx;
    int en = g_modal_entry_idx;
    if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
        en < static_cast<int>(state.layers[ly].entries.size())) {
      state.layers[ly].entries[en].crystal = cr;
      state.MarkDirty();
      g_crystal_mesh_hash = -1;  // Force left panel preview refresh
    }
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel##crystal", ImVec2(80, 0))) {
    // Restore trackball state
    std::memcpy(g_crystal_rotation, g_saved_rotation, sizeof(g_crystal_rotation));
    g_crystal_zoom = g_saved_zoom;
    g_crystal_mesh_hash = -1;  // Force left panel to re-render model crystal
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }
}


// ============================================================
// Axis Modal
// ============================================================

static void RenderAxisModal(GuiState& state) {
  // Preset buttons
  ImGui::Text("Presets:");
  ImGui::SameLine();
  if (ImGui::SmallButton("Column")) {
    g_axis_buf[0] = { AxisDistType::kGauss, 90.0f, 10.0f };
    g_axis_buf[1] = { AxisDistType::kUniform, 0.0f, 360.0f };
    g_axis_buf[2] = { AxisDistType::kUniform, 0.0f, 360.0f };
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Plate")) {
    g_axis_buf[0] = { AxisDistType::kGauss, 0.0f, 10.0f };
    g_axis_buf[1] = { AxisDistType::kUniform, 0.0f, 360.0f };
    g_axis_buf[2] = { AxisDistType::kUniform, 0.0f, 360.0f };
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Random")) {
    g_axis_buf[0] = { AxisDistType::kUniform, 0.0f, 360.0f };
    g_axis_buf[1] = { AxisDistType::kUniform, 0.0f, 360.0f };
    g_axis_buf[2] = { AxisDistType::kUniform, 0.0f, 360.0f };
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Parry")) {
    g_axis_buf[0] = { AxisDistType::kGauss, 90.0f, 10.0f };
    g_axis_buf[1] = { AxisDistType::kUniform, 0.0f, 360.0f };
    g_axis_buf[2] = { AxisDistType::kGauss, 0.0f, 0.0f };
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Lowitz")) {
    g_axis_buf[0] = { AxisDistType::kGauss, 0.0f, 60.0f };
    g_axis_buf[1] = { AxisDistType::kUniform, 0.0f, 360.0f };
    g_axis_buf[2] = { AxisDistType::kGauss, 0.0f, 0.0f };
  }

  ImGui::Separator();

  // Axis distribution controls (zenith: 0-180, azimuth: 0-360, roll: 0-360)
  RenderAxisDist("Zenith", g_axis_buf[0], 0.0f, 180.0f);
  RenderAxisDist("Azimuth", g_axis_buf[1], 0.0f, 360.0f);
  RenderAxisDist("Roll", g_axis_buf[2], 0.0f, 360.0f);

  ImGui::Separator();

  // OK / Cancel
  if (ImGui::Button("OK##axis", ImVec2(80, 0))) {
    int ly = g_modal_layer_idx;
    int en = g_modal_entry_idx;
    if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
        en < static_cast<int>(state.layers[ly].entries.size())) {
      auto& crystal = state.layers[ly].entries[en].crystal;
      crystal.zenith = g_axis_buf[0];
      crystal.azimuth = g_axis_buf[1];
      crystal.roll = g_axis_buf[2];
      state.MarkDirty();
    }
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel##axis", ImVec2(80, 0))) {
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }
}


// ============================================================
// Filter Modal
// ============================================================

static void RenderFilterModal(GuiState& state) {
  // Action combo
  ImGui::Combo("Action##filter_modal", &g_filter_buf.action, kFilterActionNames, kFilterActionCount);

  // Raypath text input with validation color feedback
  g_filter_buf.raypath_text = g_raypath_buf;
  auto validation = ValidateRaypathText(g_filter_buf.raypath_text);

  ImVec4 border_color;
  switch (validation) {
    case RaypathValidation::kValid:
      border_color = ImVec4(0.2f, 0.8f, 0.2f, 1.0f);
      break;
    case RaypathValidation::kIncomplete:
      border_color = ImVec4(0.9f, 0.8f, 0.1f, 1.0f);
      break;
    case RaypathValidation::kInvalid:
      border_color = ImVec4(0.9f, 0.2f, 0.2f, 1.0f);
      break;
  }

  ImGui::PushStyleColor(ImGuiCol_FrameBg,
                        ImVec4(border_color.x * 0.3f, border_color.y * 0.3f, border_color.z * 0.3f, 0.5f));
  ImGui::InputText("Raypath##filter_modal", g_raypath_buf, sizeof(g_raypath_buf));
  ImGui::PopStyleColor();

  // Validation hint
  switch (validation) {
    case RaypathValidation::kValid:
      if (g_filter_buf.raypath_text.empty()) {
        ImGui::TextDisabled("No raypath filter (match all)");
      } else {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "Valid");
      }
      break;
    case RaypathValidation::kIncomplete:
      ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.1f, 1.0f), "Incomplete (still typing?)");
      break;
    case RaypathValidation::kInvalid:
      ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "Invalid raypath");
      break;
  }

  // Symmetry checkboxes
  ImGui::Checkbox("P##filter_modal", &g_filter_buf.sym_p);
  ImGui::SameLine();
  ImGui::Checkbox("B##filter_modal", &g_filter_buf.sym_b);
  ImGui::SameLine();
  ImGui::Checkbox("D##filter_modal", &g_filter_buf.sym_d);

  ImGui::Separator();

  // OK / Cancel / Remove Filter
  bool ok_disabled = (validation != RaypathValidation::kValid);
  if (ok_disabled) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("OK##filter", ImVec2(80, 0))) {
    int ly = g_modal_layer_idx;
    int en = g_modal_entry_idx;
    if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
        en < static_cast<int>(state.layers[ly].entries.size())) {
      g_filter_buf.raypath_text = g_raypath_buf;
      state.layers[ly].entries[en].filter = g_filter_buf;
      state.MarkFilterDirty();
    }
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }
  if (ok_disabled) {
    ImGui::EndDisabled();
  }

  ImGui::SameLine();
  if (ImGui::Button("Cancel##filter", ImVec2(80, 0))) {
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }

  ImGui::SameLine();
  if (ImGui::Button("Remove Filter##filter", ImVec2(120, 0))) {
    int ly = g_modal_layer_idx;
    int en = g_modal_entry_idx;
    if (ly >= 0 && ly < static_cast<int>(state.layers.size()) && en >= 0 &&
        en < static_cast<int>(state.layers[ly].entries.size())) {
      state.layers[ly].entries[en].filter = std::nullopt;
      state.MarkFilterDirty();
    }
    g_active_modal = ActiveModal::kNone;
    ImGui::CloseCurrentPopup();
  }
}


// ============================================================
// Public API
// ============================================================

bool IsCrystalModalOpen() {
  return g_active_modal == ActiveModal::kCrystal;
}

void RenderEditModals(GuiState& state) {
  // Deferred OpenPopup: must happen outside BeginPopupModal
  if (g_pending_open && g_active_modal != ActiveModal::kNone) {
    switch (g_active_modal) {
      case ActiveModal::kCrystal:
        ImGui::OpenPopup("Edit Crystal");
        break;
      case ActiveModal::kAxis:
        ImGui::OpenPopup("Edit Axis");
        break;
      case ActiveModal::kFilter:
        ImGui::OpenPopup("Edit Filter");
        break;
      default:
        break;
    }
    g_pending_open = false;
  }

  // Index validity guard: if the target entry was deleted while modal is open, close it
  if (g_active_modal != ActiveModal::kNone) {
    bool valid = g_modal_layer_idx >= 0 && g_modal_layer_idx < static_cast<int>(state.layers.size()) &&
                 g_modal_entry_idx >= 0 &&
                 g_modal_entry_idx < static_cast<int>(state.layers[g_modal_layer_idx].entries.size());
    if (!valid) {
      g_active_modal = ActiveModal::kNone;
      // If a popup was open, it will simply not render this frame
    }
  }

  // Crystal modal
  if (ImGui::BeginPopupModal("Edit Crystal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    RenderCrystalModal(state);
    ImGui::EndPopup();
  }

  // Axis modal
  if (ImGui::BeginPopupModal("Edit Axis", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    RenderAxisModal(state);
    ImGui::EndPopup();
  }

  // Filter modal
  if (ImGui::BeginPopupModal("Edit Filter", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    RenderFilterModal(state);
    ImGui::EndPopup();
  }
}

}  // namespace lumice::gui
