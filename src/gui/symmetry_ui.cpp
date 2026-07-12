#include "gui/symmetry_ui.hpp"

#include <cmath>

#include "IconsFontAwesome6.h"
#include "imgui.h"

namespace lumice::gui {

namespace {

constexpr const char* kDTooltipText =
    "D applies when azimuth = uniform 360\xc2\xb0 and roll mean is a multiple of 30\xc2\xb0.\n"
    "Current config does not meet this condition, so D has no effect.";

}  // namespace

bool IsDApplicableGuiAxis(const AxisDist& az, const AxisDist& roll) {
  const bool az_sym = az.type == AxisDistType::kUniform && std::fabs(az.std - 360.0f) < 1e-3f;
  const float rem = std::fmod(std::fmod(roll.mean, 30.0f) + 30.0f, 30.0f);
  const bool roll_ok = rem < 1e-3f || std::fabs(rem - 30.0f) < 1e-3f;
  return az_sym && roll_ok;
}

void RenderSymmetryCheckboxes(bool& sym_p, bool& sym_b, bool& sym_d, bool d_applicable, const char* id_suffix) {
  // P/B/D Checkboxes: unconditional under H5 (raypath / EE / AND mix all consume
  // crystal symmetry at the core layer). The pre-H5 `sym_active` gate was already
  // effectively constant.
  char id_buf[64];
  std::snprintf(id_buf, sizeof(id_buf), "P##%s", id_suffix);
  ImGui::Checkbox(id_buf, &sym_p);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Prism-face reflection symmetry");
  }
  ImGui::SameLine();
  std::snprintf(id_buf, sizeof(id_buf), "B##%s", id_suffix);
  ImGui::Checkbox(id_buf, &sym_b);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Basal-face reflection symmetry");
  }
  ImGui::SameLine();
  std::snprintf(id_buf, sizeof(id_buf), "D##%s", id_suffix);
  ImGui::Checkbox(id_buf, &sym_d);
  if (!d_applicable) {
    ImGui::SameLine();
    // SmallButton with transparent styling acts as a hover target for the tooltip
    // (TextDisabled lacks a stable item ID needed by the test engine).
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    std::snprintf(id_buf, sizeof(id_buf), ICON_FA_CIRCLE_INFO "##d_tooltip_icon_%s", id_suffix);
    ImGui::SmallButton(id_buf);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(4);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", kDTooltipText);
    }
  }
}

}  // namespace lumice::gui
