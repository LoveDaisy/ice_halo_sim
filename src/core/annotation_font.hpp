#ifndef CORE_ANNOTATION_FONT_H_
#define CORE_ANNOTATION_FONT_H_

#include <cstdint>
#include <string>
#include <vector>

namespace lumice::annotation {

// =================================================================================================
// Glyph rasterization for annotation labels, for the drawers that have no font stack of their own.
//
// WHY THIS EXISTS. annotation::ComputeOverlay already answers "where does this label go and what
// does it say" (Label::px / py / text). The GUI turns that into pixels with ImGui's font, which
// needs a GL context and an ImGui frame; the CLI renderer has neither, so before this file the
// text half of the annotation layer simply did not exist off the GUI. This is the missing half:
// text in, an 8-bit coverage bitmap out.
//
// SCOPE. Coverage only — no colour, no alpha, no blending. The caller composites, because the
// colour and the opacity belong to the annotation family the label came from and the blend has to
// happen in whatever domain that caller composites in (linear RGB, for server/render.cpp). A
// rasterizer that also blended would have to be told about both, and would then be a second place
// where the annotation appearance model lives.
//
// ALPHABET. Only what FormatAngleDeg (annotation_overlay.cpp) can produce: the digits, '.', '-'
// and U+00B0 DEGREE SIGN. `text` is UTF-8 and is decoded as such rather than byte by byte — the
// degree sign is two bytes (0xC2 0xB0), and walking it as bytes would draw two wrong glyphs
// instead of one right one. A codepoint outside the alphabet is skipped, not substituted: the
// alphabet is closed by construction, so a miss is a defect upstream and a tofu box would only
// disguise it.
// =================================================================================================

// The em size labels are rasterized at. Matches the GUI's body font size (theme.cpp loads Roboto
// Medium at 15 px), so the same annotation reads at the same weight in the preview and in a CLI
// render. NOT scaled by the export resolution — the GUI's own export does not scale either
// (export_fbo_renderer draws with the loaded atlas), and a CLI that did would put text at a
// different size than the preview it is supposed to reproduce.
inline constexpr float kLabelPixelHeight = 15.0f;

// One rasterized run of text: the ink, plus where to put it relative to the anchor.
//
// The anchor is the CENTRE of the run, which is what the GUI's drawer uses
// (AppendOverlayToDrawList offsets by half the text size before drawing), so the two place a
// label on the same point rather than one drawing from the corner.
struct TextBitmap {
  int width = 0;
  int height = 0;
  // Offset from the anchor pixel to this bitmap's top-left corner. Normally negative on both
  // axes, since the anchor sits inside the run.
  int offset_x = 0;
  int offset_y = 0;
  // Row-major width*height. 0 = the glyph does not cover this pixel, 255 = fully covered.
  std::vector<uint8_t> coverage;

  bool Empty() const { return coverage.empty(); }
};

// Rasterize one label at `kLabelPixelHeight`. Returns an empty bitmap for empty text, for a text
// made only of unsupported codepoints, and for a font that failed to initialize — all three are
// "draw nothing", and none of them is worth a separate failure channel at a call site whose only
// possible response is to draw nothing.
TextBitmap RasterizeLabel(const std::string& text);

}  // namespace lumice::annotation

#endif  // CORE_ANNOTATION_FONT_H_
