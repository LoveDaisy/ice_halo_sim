// The body font's glyph coverage (src/gui/theme.cpp, AddBodyFont), read off the live atlas.
//
// The proposition is about what the atlas HOLDS, not about what a panel draws: a codepoint left
// out of the font's glyph range rasterises as ImGui's fallback "?" — which looks like a typo, not
// like a missing glyph, and which no screenshot reference can distinguish from a typo either (the
// modal_layout references were shot with that "?" in the crystal name hint for months). Asking
// the atlas directly, with FindGlyphNoFallback, is the one read that cannot be fooled by the
// fallback. The negative row pins the reason the analysis list draws its chain joiner as an icon
// rather than as U+2192: Roboto Medium has no glyph for it, and if a future font swap gives it
// one, this row is what says JoinerForDisplay could go back to a plain character.

#include "test_gui_shared.hpp"

void RegisterBodyFontGlyphCoverageTests(ImGuiTestEngine* engine) {
  ImGuiTest* t = IM_REGISTER_TEST(engine, "font_glyph_coverage", "general_punctuation_present_arrows_absent");
  t->TestFunc = [](ImGuiTestContext* ctx) {
    // One frame first, so the atlas read below is the one the renderer has built and drawn with.
    ctx->Yield(1);
    ImFontAtlas* atlas = ImGui::GetIO().Fonts;
    IM_CHECK(atlas != nullptr);
    IM_CHECK(atlas->Fonts.Size >= 1);
    // The body font is the first one added (AddBodyFont runs before MergeIconGlyphs, and the
    // AddFontDefault fallback replaces it rather than preceding it).
    ImFont* body = atlas->Fonts[0];
    IM_CHECK(body != nullptr);
    ctx->LogInfo("[font_glyph_coverage] atlas %dx%d, %d fonts", atlas->TexWidth, atlas->TexHeight, atlas->Fonts.Size);
    // General Punctuation landed: the dashes and the single angle quote, plus the guillemet from
    // Latin-1 Supplement that ImGui's default range already covered.
    IM_CHECK(body->FindGlyphNoFallback(0x2013) != nullptr);  // en dash
    IM_CHECK(body->FindGlyphNoFallback(0x2014) != nullptr);  // em dash
    IM_CHECK(body->FindGlyphNoFallback(0x203A) != nullptr);  // single right-pointing angle quote
    IM_CHECK(body->FindGlyphNoFallback(0x00BB) != nullptr);  // right-pointing double angle quote
    // The icon the chain joiner is drawn with is merged into the same font.
    IM_CHECK(body->FindGlyphNoFallback(0xF061) != nullptr);  // ICON_FA_ARROW_RIGHT
    // Still absent, by the font's cmap and not by the range: U+2192 is why the joiner is an icon.
    IM_CHECK(body->FindGlyphNoFallback(0x2192) == nullptr);
  };
}
