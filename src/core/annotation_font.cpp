#include "core/annotation_font.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "util/logger.hpp"

// STBTT_STATIC keeps every stb_truetype symbol internal to this translation unit. Dear ImGui
// compiles its own vendored copy of the same library (imstb_truetype.h, also STBTT_STATIC), and
// two non-static copies in one binary would be a duplicate-symbol link error in any build with
// BUILD_GUI on.
#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

namespace lumice::annotation {

// The embedded typeface, generated at build time by scripts/embed_binary.py from the same
// Roboto-Medium.ttf the GUI embeds (see the roboto_font CPM entry in the top-level CMakeLists).
extern const unsigned char kLabelFontData[];
extern const std::size_t kLabelFontSize;

namespace {

// The closed alphabet FormatAngleDeg can emit. Packing only these keeps the atlas a few kB
// instead of a full ASCII sheet, and makes an out-of-alphabet codepoint a visible miss rather
// than something that happens to render.
constexpr int kCodepoints[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.', '-', 0x00B0 };
constexpr int kCodepointCount = static_cast<int>(sizeof(kCodepoints) / sizeof(kCodepoints[0]));

// 13 glyphs at 15 px fit in a fraction of this; the margin is for a future alphabet, and the whole
// sheet is 16 kB either way.
constexpr int kAtlasW = 256;
constexpr int kAtlasH = 128;

// The packed atlas plus its metrics, built once per process. A function-local static: the build
// reads only immutable inputs (the embedded bytes and the constants above), so it needs no
// configuration, and C++11 guarantees the initialization itself is thread-safe.
struct PackedFont {
  bool ok = false;
  std::vector<unsigned char> atlas;
  stbtt_packedchar chars[kCodepointCount]{};
  // Scaled to kLabelPixelHeight. ascent is positive above the baseline, descent negative below,
  // which is what makes (ascent - descent) the line box the GUI's ImGui::CalcTextSize also
  // reports — the two have to agree or a label sits at a different height in the two renders.
  float ascent = 0.0f;
  float descent = 0.0f;
};

PackedFont BuildPackedFont() {
  PackedFont f;
  f.atlas.assign(static_cast<size_t>(kAtlasW) * kAtlasH, 0);

  stbtt_pack_context ctx{};
  if (stbtt_PackBegin(&ctx, f.atlas.data(), kAtlasW, kAtlasH, 0, 1, nullptr) == 0) {
    LOG_WARNING("[AnnotationFont] stbtt_PackBegin failed; labels will not be drawn");
    return f;
  }
  // 1x1 oversampling: the labels are drawn at a fixed integer size onto an integer pixel grid,
  // so the subpixel precision oversampling buys has nothing to act on here.
  stbtt_PackSetOversampling(&ctx, 1, 1);

  stbtt_pack_range range{};
  range.font_size = kLabelPixelHeight;
  // Non-null array_of_unicode_codepoints is what makes this a sparse range rather than a
  // contiguous one starting at first_unicode_codepoint_in_range.
  range.array_of_unicode_codepoints = const_cast<int*>(kCodepoints);
  range.num_chars = kCodepointCount;
  range.chardata_for_range = f.chars;

  const int packed = stbtt_PackFontRanges(&ctx, kLabelFontData, 0, &range, 1);
  stbtt_PackEnd(&ctx);
  if (packed == 0) {
    LOG_WARNING("[AnnotationFont] stbtt_PackFontRanges failed; labels will not be drawn");
    return f;
  }

  stbtt_fontinfo info{};
  if (stbtt_InitFont(&info, kLabelFontData, 0) == 0) {
    LOG_WARNING("[AnnotationFont] stbtt_InitFont failed; labels will not be drawn");
    return f;
  }
  int ascent = 0;
  int descent = 0;
  int line_gap = 0;
  stbtt_GetFontVMetrics(&info, &ascent, &descent, &line_gap);
  const float scale = stbtt_ScaleForPixelHeight(&info, kLabelPixelHeight);
  f.ascent = static_cast<float>(ascent) * scale;
  f.descent = static_cast<float>(descent) * scale;
  f.ok = true;
  return f;
}

const PackedFont& Font() {
  static const PackedFont f = BuildPackedFont();
  return f;
}

// Index into kCodepoints, or -1 for a codepoint outside the alphabet.
int GlyphIndex(int codepoint) {
  for (int i = 0; i < kCodepointCount; ++i) {
    if (kCodepoints[i] == codepoint) {
      return i;
    }
  }
  return -1;
}

// Minimal UTF-8: ASCII and the two-byte forms. That covers the whole alphabet (the degree sign is
// the only non-ASCII member, 0xC2 0xB0) and nothing else can appear, so the three- and four-byte
// lead bytes are consumed and skipped rather than decoded — dropping a codepoint that cannot occur
// is preferable to carrying a general decoder nothing exercises. Returns the codepoint and
// advances `i` past the sequence; returns -1 for a malformed or unsupported sequence.
int NextCodepoint(const std::string& text, size_t& i) {
  const auto b0 = static_cast<unsigned char>(text[i]);
  if (b0 < 0x80) {
    ++i;
    return b0;
  }
  if ((b0 & 0xE0) == 0xC0 && i + 1 < text.size()) {
    const auto b1 = static_cast<unsigned char>(text[i + 1]);
    if ((b1 & 0xC0) == 0x80) {
      i += 2;
      return ((b0 & 0x1F) << 6) | (b1 & 0x3F);
    }
  }
  ++i;
  return -1;
}

}  // namespace

TextBitmap RasterizeLabel(const std::string& text) {
  TextBitmap out;
  const PackedFont& font = Font();
  if (!font.ok || text.empty()) {
    return out;
  }

  // Pass 1: lay the run out on a baseline at y = ascent, and take the ink bounding box. The pen
  // has to run over the whole string before anything is written, because both the bitmap's size
  // and the anchor offset are functions of the finished run.
  struct Quad {
    int index;
    stbtt_aligned_quad q;
  };
  std::vector<Quad> quads;
  quads.reserve(text.size());
  float pen_x = 0.0f;
  float pen_y = font.ascent;
  float ink_x0 = 0.0f;
  float ink_y0 = 0.0f;
  float ink_x1 = 0.0f;
  float ink_y1 = 0.0f;
  bool any_ink = false;
  for (size_t i = 0; i < text.size();) {
    const int cp = NextCodepoint(text, i);
    const int gi = cp < 0 ? -1 : GlyphIndex(cp);
    if (gi < 0) {
      continue;
    }
    Quad quad{ gi, {} };
    // align_to_integer = 1: the run starts at an integer pen position and every glyph lands on
    // the pixel grid, which is what lets the blit below be a plain copy instead of a resample.
    stbtt_GetPackedQuad(font.chars, kAtlasW, kAtlasH, gi, &pen_x, &pen_y, &quad.q, 1);
    if (quad.q.x1 <= quad.q.x0 || quad.q.y1 <= quad.q.y0) {
      continue;  // a space-like glyph: it advanced the pen and has no ink
    }
    if (!any_ink) {
      ink_x0 = quad.q.x0;
      ink_y0 = quad.q.y0;
      ink_x1 = quad.q.x1;
      ink_y1 = quad.q.y1;
      any_ink = true;
    } else {
      ink_x0 = std::min(ink_x0, quad.q.x0);
      ink_y0 = std::min(ink_y0, quad.q.y0);
      ink_x1 = std::max(ink_x1, quad.q.x1);
      ink_y1 = std::max(ink_y1, quad.q.y1);
    }
    quads.push_back(quad);
  }
  if (!any_ink) {
    return out;
  }

  // The GUI draws every label twice, one pixel apart horizontally, to fake a bold weight
  // (AppendOverlayToDrawList). Reproduced here as a one-pixel dilation of the coverage rather
  // than a second blit, which is the same result for an opaque-over-opaque draw and cannot
  // double-blend where the two copies overlap. It widens the ink by exactly one pixel.
  constexpr int kFakeBoldPx = 1;

  const auto x0 = static_cast<int>(std::floor(ink_x0));
  const auto y0 = static_cast<int>(std::floor(ink_y0));
  const auto x1 = static_cast<int>(std::ceil(ink_x1)) + kFakeBoldPx;
  const auto y1 = static_cast<int>(std::ceil(ink_y1));
  out.width = x1 - x0;
  out.height = y1 - y0;
  if (out.width <= 0 || out.height <= 0) {
    out.width = 0;
    out.height = 0;
    return out;
  }
  out.coverage.assign(static_cast<size_t>(out.width) * static_cast<size_t>(out.height), 0);

  // Pass 2: blit each glyph out of the atlas. The quads are already on the pixel grid, so the
  // source rectangle is copied one-to-one and `max` is the accumulate rule — two glyphs never
  // overlap, but the bold dilation below does overlap itself.
  for (const Quad& quad : quads) {
    const auto sx0 = static_cast<int>(std::lround(quad.q.s0 * kAtlasW));
    const auto sy0 = static_cast<int>(std::lround(quad.q.t0 * kAtlasH));
    const auto dx0 = static_cast<int>(std::lround(quad.q.x0)) - x0;
    const auto dy0 = static_cast<int>(std::lround(quad.q.y0)) - y0;
    const auto gw = static_cast<int>(std::lround(quad.q.x1 - quad.q.x0));
    const auto gh = static_cast<int>(std::lround(quad.q.y1 - quad.q.y0));
    for (int row = 0; row < gh; ++row) {
      const int dy = dy0 + row;
      const int sy = sy0 + row;
      if (dy < 0 || dy >= out.height || sy < 0 || sy >= kAtlasH) {
        continue;
      }
      for (int col = 0; col < gw; ++col) {
        const int sx = sx0 + col;
        if (sx < 0 || sx >= kAtlasW) {
          continue;
        }
        const unsigned char v = font.atlas[static_cast<size_t>(sy) * kAtlasW + sx];
        if (v == 0) {
          continue;
        }
        for (int b = 0; b <= kFakeBoldPx; ++b) {
          const int dx = dx0 + col + b;
          if (dx < 0 || dx >= out.width) {
            continue;
          }
          uint8_t& dst = out.coverage[static_cast<size_t>(dy) * out.width + dx];
          dst = std::max(dst, static_cast<uint8_t>(v));
        }
      }
    }
  }

  // The anchor is the centre of the LINE BOX — advance width by (ascent - descent) — not of the
  // ink. Same box ImGui::CalcTextSize reports, so "22" and "0" hang at the same height rather
  // than each centring its own ink.
  const float run_w = pen_x + static_cast<float>(kFakeBoldPx);
  const float run_h = font.ascent - font.descent;
  out.offset_x = x0 - static_cast<int>(std::lround(run_w * 0.5f));
  out.offset_y = y0 - static_cast<int>(std::lround(run_h * 0.5f));
  return out;
}

}  // namespace lumice::annotation
