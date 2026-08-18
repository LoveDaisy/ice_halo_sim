#include "gui/semantic_colors.hpp"

namespace lumice::gui {

namespace {

// Every canonical value below is one of the literals that already existed at a
// call site — the most-reused one, or the one the visual-language blueprint
// named. Nothing here is a freshly picked compromise shade: folding five
// near-duplicate ambers onto an existing amber moves a few channel percent,
// while folding them onto a new average would move every one of them.
constexpr ImVec4 kGoodText{ 0.40f, 0.80f, 0.40f, 1.0f };   // was: Ready status text
constexpr ImVec4 kGoodFill{ 0.06f, 0.24f, 0.06f, 1.0f };   // was: VALID validation cell
constexpr ImVec4 kWarnText{ 1.00f, 0.75f, 0.20f, 1.0f };   // was: 3 call sites, verbatim
constexpr ImVec4 kWarnFill{ 0.45f, 0.28f, 0.12f, 1.0f };   // was: Resolution input FrameBg
constexpr ImVec4 kDestrText{ 0.90f, 0.30f, 0.30f, 1.0f };  // was: invalid predicate border
constexpr ImVec4 kDestrFill{ 0.27f, 0.06f, 0.06f, 1.0f };  // was: INVALID validation cell

// Good-button triple. Kept beside the grade it belongs to rather than in
// destructive_style.cpp, which owns only the destructive triple.
constexpr ImVec4 kGoodBtnNormal{ 0.15f, 0.45f, 0.15f, 1.0f };
constexpr ImVec4 kGoodBtnHovered{ 0.20f, 0.55f, 0.20f, 1.0f };
constexpr ImVec4 kGoodBtnActive{ 0.10f, 0.35f, 0.10f, 1.0f };

// Warning-button triple. Built from kWarnFill (the existing muted amber, hue ~32°) the way the
// good triple is built around its own fill: Normal IS kWarnFill, Hovered is one step brighter,
// Active one step darker. Deriving it keeps the chip at the same hue as kWarnText and the
// "Modified" status text, which is the property that makes them read as one signal; picking a
// fresh amber here would be the fifth near-duplicate this file exists to have removed.
constexpr ImVec4 kWarnBtnNormal{ 0.45f, 0.28f, 0.12f, 1.0f };
constexpr ImVec4 kWarnBtnHovered{ 0.58f, 0.37f, 0.16f, 1.0f };
constexpr ImVec4 kWarnBtnActive{ 0.34f, 0.21f, 0.09f, 1.0f };

ImVec4 WithAlpha(const ImVec4& c, float alpha) {
  return ImVec4(c.x, c.y, c.z, alpha);
}

}  // namespace

ImVec4 GoodTextColor() {
  return kGoodText;
}

ImVec4 GoodFillColor(float alpha) {
  return WithAlpha(kGoodFill, alpha);
}

void PushGoodButtonStyle() {
  ImGui::PushStyleColor(ImGuiCol_Button, kGoodBtnNormal);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kGoodBtnHovered);
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, kGoodBtnActive);
}

void PopGoodButtonStyle() {
  ImGui::PopStyleColor(3);
}

ImVec4 WarningTextColor() {
  return kWarnText;
}

ImVec4 WarningFillColor(float alpha) {
  return WithAlpha(kWarnFill, alpha);
}

void PushWarningButtonStyle() {
  ImGui::PushStyleColor(ImGuiCol_Button, kWarnBtnNormal);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kWarnBtnHovered);
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, kWarnBtnActive);
}

void PopWarningButtonStyle() {
  ImGui::PopStyleColor(3);
}

ImVec4 DestructiveTextColor() {
  return kDestrText;
}

ImVec4 DestructiveFillColor(float alpha) {
  return WithAlpha(kDestrFill, alpha);
}

}  // namespace lumice::gui
