#include "gui/theme.hpp"

#include <cstring>

#include "IconsFontAwesome6.h"
#include "gui/fa_solid_900_embed.h"
#include "gui/gui_logger.hpp"
#include "gui/roboto_medium_embed.h"
#include "imgui.h"

namespace lumice::gui {

namespace {

// Body text size in pixels. The icon atlas is merged at the same size so glyphs
// and text share a baseline; keeping them bound to one constant removes a
// "two places that must change together" hazard.
constexpr float kBodyFontSizePx = 15.0f;

// Adds Roboto Medium (embedded at build time) as the body font. Returns nullptr
// on failure, leaving the caller to fall back rather than run with no font.
ImFont* AddBodyFont(ImGuiIO& io, float size_px) {
  ImFontConfig body_cfg;
  // 3x horizontal oversampling sharpens a proportional face at this size; ImGui's
  // stb rasterizer has no hinting, so subpixel positioning is what carries legibility.
  body_cfg.OversampleH = 3;
  body_cfg.OversampleV = 1;
  // ImGui takes ownership of the buffer it rasterizes from and frees it with the
  // atlas, so hand it a copy rather than the const embedded array.
  void* body_buf = IM_ALLOC(kRobotoMediumSize);
  std::memcpy(body_buf, kRobotoMediumData, kRobotoMediumSize);
  return io.Fonts->AddFontFromMemoryTTF(body_buf, static_cast<int>(kRobotoMediumSize), size_px, &body_cfg);
}

// Merges FontAwesome 6 Solid glyphs into the atlas of the font added last.
void MergeIconGlyphs(ImGuiIO& io, float size_px) {
  ImFontConfig icon_cfg;
  icon_cfg.MergeMode = true;
  icon_cfg.PixelSnapH = true;
  icon_cfg.OversampleH = 2;
  icon_cfg.OversampleV = 2;
  icon_cfg.GlyphMinAdvanceX = size_px;
  static const ImWchar kIconRanges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
  void* icon_buf = IM_ALLOC(kFaSolid900Size);
  std::memcpy(icon_buf, kFaSolid900Data, kFaSolid900Size);
  ImFont* icon_font =
      io.Fonts->AddFontFromMemoryTTF(icon_buf, static_cast<int>(kFaSolid900Size), size_px, &icon_cfg, kIconRanges);
  if (!icon_font) {
    GUI_LOG_WARNING("FA icon font failed to load; ICON_FA_* glyphs will be blank.");
  }
}

// ---------------------------------------------------------------------------
// Size rhythm
// ---------------------------------------------------------------------------

// Refinement is not a function of how much room the rows get, it is a function of how
// few distinct measurements the panel uses. Every value below is a multiple of 4 (or a
// half-step of one), the row pitch lands within a pixel of the pre-Roboto layout despite
// a two-point-larger face, and the corner radii come down — a large radius on a short
// row eats the corner that was carrying the alignment.
void ApplyGridSpacing(ImGuiStyle& style) {
  style.WindowPadding = ImVec2(8.0f, 6.0f);
  style.FramePadding = ImVec2(6.0f, 3.0f);
  style.CellPadding = ImVec2(4.0f, 2.0f);
  style.ItemSpacing = ImVec2(8.0f, 3.0f);
  style.ItemInnerSpacing = ImVec2(4.0f, 3.0f);
  style.IndentSpacing = 16.0f;
  style.ScrollbarSize = 10.0f;
  style.GrabMinSize = 8.0f;

  style.WindowBorderSize = 1.0f;
  style.ChildBorderSize = 1.0f;
  style.PopupBorderSize = 1.0f;
  // The single largest source of visual noise in the panels: with a per-frame border,
  // every input, combo and button draws its own outline, so a column of eight settings
  // reads as sixteen boxes. Hierarchy comes from the spacing rhythm above instead.
  // An unchecked checkbox is the one widget this costs legibility — see Checkbox() below.
  style.FrameBorderSize = 0.0f;

  style.WindowRounding = 4.0f;
  style.ChildRounding = 3.0f;
  style.FrameRounding = 2.0f;
  style.PopupRounding = 4.0f;
  style.ScrollbarRounding = 3.0f;
  style.GrabRounding = 2.0f;
  style.TabRounding = 2.0f;

  style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
  style.SeparatorTextBorderSize = 1.0f;
  style.SeparatorTextPadding = ImVec2(10.0f, 2.0f);
  style.SeparatorTextAlign = ImVec2(0.0f, 0.5f);
}

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------

struct Palette {
  ImVec4 text;
  ImVec4 text_dim;
  ImVec4 window_bg;
  ImVec4 child_bg;
  ImVec4 popup_bg;
  ImVec4 frame_bg;
  ImVec4 frame_hover;
  ImVec4 frame_active;
  ImVec4 button;
  ImVec4 button_hover;
  ImVec4 button_active;
  ImVec4 border;
  ImVec4 title_bg;
  ImVec4 title_active;
  ImVec4 accent;       // interactive emphasis only — never a large filled area
  ImVec4 grab;         // slider/scrollbar handle at rest: neutral by design
  ImVec4 header_tint;  // CollapsingHeader fill: a tint, not a saturated bar
};

// Measured, not guessed: ImGui's default dark theme — what this app shipped before —
// puts its frame backgrounds at hue 216 deg / 59% saturation and its group header bars
// at 212 deg / 65%. The app is already a saturated-blue theme, so "add blue" candidates
// in fact removed most of it. The design space is how much of that blue to keep, and the
// two knobs are independent: hue, and how much AREA is filled with it. This keeps the hue
// near the old one on the small elements (frames, buttons) and cuts the area instead —
// the group header stops being a solid saturated bar and becomes a tint, and the slider
// handle stays neutral until it is being dragged.
constexpr Palette kIceTruePalette{
  /*text*/ { 0.800f, 0.820f, 0.850f, 1.00f },
  /*text_dim*/ { 0.400f, 0.415f, 0.450f, 1.00f },
  /*window_bg*/ { 0.063f, 0.067f, 0.078f, 1.00f },
  /*child_bg*/ { 0.082f, 0.088f, 0.102f, 1.00f },
  /*popup_bg*/ { 0.094f, 0.102f, 0.120f, 1.00f },
  /*frame_bg*/ { 0.110f, 0.157f, 0.235f, 1.00f },
  /*frame_hover*/ { 0.150f, 0.210f, 0.310f, 1.00f },
  /*frame_active*/ { 0.180f, 0.250f, 0.365f, 1.00f },
  /*button*/ { 0.130f, 0.180f, 0.265f, 1.00f },
  /*button_hover*/ { 0.175f, 0.240f, 0.350f, 1.00f },
  /*button_active*/ { 0.105f, 0.145f, 0.215f, 1.00f },
  /*border*/ { 0.000f, 0.000f, 0.000f, 0.55f },
  /*title_bg*/ { 0.055f, 0.060f, 0.072f, 1.00f },
  /*title_active*/ { 0.082f, 0.088f, 0.102f, 1.00f },
  /*accent*/ { 0.350f, 0.620f, 0.920f, 1.00f },
  /*grab*/ { 0.500f, 0.530f, 0.580f, 1.00f },
  /*header_tint*/ { 0.350f, 0.620f, 0.950f, 0.160f },
};

ImVec4 WithAlpha(const ImVec4& c, float a) {
  return ImVec4(c.x, c.y, c.z, a);
}

void ApplyPalette(ImGuiStyle& style, const Palette& p) {
  ImVec4* c = style.Colors;
  c[ImGuiCol_Text] = p.text;
  c[ImGuiCol_TextDisabled] = p.text_dim;
  c[ImGuiCol_WindowBg] = p.window_bg;
  c[ImGuiCol_ChildBg] = p.child_bg;
  c[ImGuiCol_PopupBg] = p.popup_bg;
  c[ImGuiCol_Border] = p.border;
  c[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_FrameBg] = p.frame_bg;
  c[ImGuiCol_FrameBgHovered] = p.frame_hover;
  c[ImGuiCol_FrameBgActive] = p.frame_active;
  c[ImGuiCol_TitleBg] = p.title_bg;
  c[ImGuiCol_TitleBgActive] = p.title_active;
  c[ImGuiCol_TitleBgCollapsed] = p.title_bg;
  c[ImGuiCol_MenuBarBg] = p.title_active;
  c[ImGuiCol_ScrollbarBg] = WithAlpha(p.window_bg, 0.0f);
  c[ImGuiCol_ScrollbarGrab] = WithAlpha(p.grab, 0.35f);
  c[ImGuiCol_ScrollbarGrabHovered] = WithAlpha(p.grab, 0.55f);
  c[ImGuiCol_ScrollbarGrabActive] = WithAlpha(p.grab, 0.75f);
  c[ImGuiCol_CheckMark] = p.accent;
  // Grab neutral at rest, accent only while dragging: this is the "spend the colour
  // budget on the data, not the chrome" rule made concrete — sliders are the single
  // most repeated widget in the panels, so their resting hue sets the whole tone.
  c[ImGuiCol_SliderGrab] = p.grab;
  c[ImGuiCol_SliderGrabActive] = p.accent;
  c[ImGuiCol_Button] = p.button;
  c[ImGuiCol_ButtonHovered] = p.button_hover;
  c[ImGuiCol_ButtonActive] = p.button_active;
  // A tint, not a filled bar: the header's job is to group, not to be the loudest thing
  // on the panel.
  c[ImGuiCol_Header] = p.header_tint;
  c[ImGuiCol_HeaderHovered] = WithAlpha(p.header_tint, p.header_tint.w * 2.0f);
  c[ImGuiCol_HeaderActive] = WithAlpha(p.header_tint, p.header_tint.w * 2.8f);
  c[ImGuiCol_Separator] = ImVec4(1.0f, 1.0f, 1.0f, 0.10f);
  c[ImGuiCol_SeparatorHovered] = WithAlpha(p.accent, 0.55f);
  c[ImGuiCol_SeparatorActive] = p.accent;
  c[ImGuiCol_ResizeGrip] = ImVec4(1.0f, 1.0f, 1.0f, 0.08f);
  c[ImGuiCol_ResizeGripHovered] = WithAlpha(p.accent, 0.55f);
  c[ImGuiCol_ResizeGripActive] = p.accent;
  c[ImGuiCol_Tab] = p.button;
  c[ImGuiCol_TabHovered] = p.button_hover;
  c[ImGuiCol_TabSelected] = p.frame_active;
  c[ImGuiCol_TabDimmed] = p.title_bg;
  c[ImGuiCol_TabDimmedSelected] = p.frame_bg;
  c[ImGuiCol_TableHeaderBg] = p.title_active;
  c[ImGuiCol_TableBorderStrong] = p.border;
  c[ImGuiCol_TableBorderLight] = ImVec4(1.0f, 1.0f, 1.0f, 0.06f);
  c[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
  c[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.025f);
  c[ImGuiCol_TextSelectedBg] = WithAlpha(p.accent, 0.35f);
  c[ImGuiCol_NavCursor] = p.accent;
  c[ImGuiCol_DragDropTarget] = p.accent;
  c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.55f);
}

// Inset-border alpha for an unchecked checkbox. The value comes from the layout
// prototype's CSS rule for the same element; it is low enough that a column of
// unchecked boxes stays quiet, high enough that a single isolated one is still a box.
constexpr float kUncheckedBoxBorderAlpha = 0.13f;

}  // namespace

bool Checkbox(const char* label, bool* v) {
  const bool changed = ImGui::Checkbox(label, v);
  if (!*v) {
    // ImGui draws the square starting at the item's top-left corner (the label follows to
    // its right), and its side length is label_size.y + FramePadding.y * 2 — which for a
    // single line of text is exactly GetFrameHeight(). So this box is the widget's box,
    // independent of whether the label is empty, short or long.
    const ImVec2 box_min = ImGui::GetItemRectMin();
    const float box_size = ImGui::GetFrameHeight();
    // GetColorU32 folds in style.Alpha, which BeginDisabled() has already scaled by
    // DisabledAlpha — so the border fades with a disabled checkbox instead of outliving it.
    ImGui::GetWindowDrawList()->AddRect(box_min, ImVec2(box_min.x + box_size, box_min.y + box_size),
                                        ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, kUncheckedBoxBorderAlpha)),
                                        ImGui::GetStyle().FrameRounding);
  }
  return changed;
}

void ApplyVisualLanguage(ImGuiIO& io) {
  ImGui::StyleColorsDark();

  // Lets a click-release (no drag, no ctrl) on a DragFloat enter text-input mode — the only
  // consumer today is the Overlay table's Alpha cells (RenderOverlaysTab, app_panels.cpp), so
  // this flag's blast radius is exactly that table. Re-read this note before adding a second bare
  // DragFloat/DragInt call site anywhere in the GUI.
  io.ConfigDragClickToInputText = true;

  ImGuiStyle& style = ImGui::GetStyle();
  ApplyGridSpacing(style);
  ApplyPalette(style, kIceTruePalette);

  if (!AddBodyFont(io, kBodyFontSizePx)) {
    GUI_LOG_WARNING("Roboto Medium failed to load; falling back to the built-in bitmap font.");
    io.Fonts->AddFontDefault();
  }
  MergeIconGlyphs(io, kBodyFontSizePx);
}

}  // namespace lumice::gui
