#include "gui/theme.hpp"

#include <cstring>

#include "IconsFontAwesome6.h"
#include "gui/fa_solid_900_embed.h"
#include "gui/gui_logger.hpp"
#include "imgui.h"

namespace lumice::gui {

namespace {

// Body text size in pixels. The icon atlas is merged at the same size so glyphs
// and text share a baseline; keeping them bound to one constant removes a
// "two places that must change together" hazard.
constexpr float kBodyFontSizePx = 13.0f;

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

}  // namespace

void ApplyVisualLanguage(ImGuiIO& io) {
  ImGui::StyleColorsDark();

  io.Fonts->AddFontDefault();
  MergeIconGlyphs(io, kBodyFontSizePx);
}

}  // namespace lumice::gui
