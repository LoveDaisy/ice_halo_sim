#include "gui/destructive_style.hpp"

#include "imgui.h"

namespace lumice::gui {

namespace {

// Destructive action button palette (delete/remove). RGB values previously lived
// as file-local constants in panels.cpp (kBtnDestructive*) and were duplicated
// verbatim in edit_modals.cpp; both copies agreed to the digit. Extracted here
// so all consumers share one source (a12 单一部件).
constexpr ImVec4 kBtnDestructiveNormal{ 0.70f, 0.22f, 0.22f, 1.0f };
constexpr ImVec4 kBtnDestructiveHovered{ 0.85f, 0.30f, 0.30f, 1.0f };
constexpr ImVec4 kBtnDestructiveActive{ 0.60f, 0.15f, 0.15f, 1.0f };

}  // namespace

void PushDestructiveStyle() {
  ImGui::PushStyleColor(ImGuiCol_Button, kBtnDestructiveNormal);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kBtnDestructiveHovered);
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, kBtnDestructiveActive);
}

void PopDestructiveStyle() {
  ImGui::PopStyleColor(3);
}

}  // namespace lumice::gui
