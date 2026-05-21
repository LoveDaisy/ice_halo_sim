#pragma once

#include <cstdio>
#include <cstring>

#include "IconsFontAwesome6.h"
#include "gui/fa_solid_900_embed.h"
#include "imgui.h"

namespace lumice::gui {

// Loads the default font and merges FontAwesome 6 Solid icon glyphs into the
// same atlas. Must be called after ImGui::CreateContext(), before the render loop.
// OversampleH/V and RasterizerMultiply may need per-platform tuning (see plan §7 risk 3).
inline void LoadFontAtlas(ImGuiIO& io) {
  io.Fonts->AddFontDefault();
  {
    ImFontConfig icon_cfg;
    icon_cfg.MergeMode = true;
    icon_cfg.PixelSnapH = true;
    icon_cfg.OversampleH = 2;
    icon_cfg.OversampleV = 2;
    icon_cfg.GlyphMinAdvanceX = 13.0f;
    static const ImWchar kIconRanges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
    void* icon_buf = IM_ALLOC(kFaSolid900Size);
    std::memcpy(icon_buf, kFaSolid900Data, kFaSolid900Size);
    ImFont* icon_font = io.Fonts->AddFontFromMemoryTTF(
        icon_buf, static_cast<int>(kFaSolid900Size), 13.0f, &icon_cfg, kIconRanges);
    if (!icon_font) {
      fprintf(stderr, "[gui] WARNING: FA icon font failed to load; ICON_FA_* glyphs will be blank.\n");
    }
  }
}

}  // namespace lumice::gui
