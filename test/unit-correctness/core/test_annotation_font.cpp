#include <algorithm>
#include <string>
#include <vector>

#include "core/annotation_font.hpp"
#include "gtest/gtest.h"

// Glyph rasterization for annotation labels. What is under test is NOT "does stb_truetype work" —
// it is the layer this repo owns on top of it: the closed alphabet, the UTF-8 decode of the one
// non-ASCII member, the run layout, and the anchor being the centre of the run rather than a
// corner. Each case below is a proposition about one of those.

namespace {

namespace ann = lumice::annotation;

// Total ink in a bitmap. A single number is enough for the "did anything at all get drawn"
// direction and, compared between two texts, for "is this a different glyph".
long TotalCoverage(const ann::TextBitmap& b) {
  long sum = 0;
  for (const uint8_t v : b.coverage) {
    sum += v;
  }
  return sum;
}

// Column occupancy: for each x, whether any row has ink. Two glyphs that differ only in weight
// have similar profiles; two that differ in shape do not.
std::vector<int> ColumnInk(const ann::TextBitmap& b) {
  std::vector<int> cols(static_cast<size_t>(std::max(b.width, 0)), 0);
  for (int y = 0; y < b.height; ++y) {
    for (int x = 0; x < b.width; ++x) {
      cols[static_cast<size_t>(x)] += b.coverage[static_cast<size_t>(y) * b.width + x];
    }
  }
  return cols;
}

}  // namespace

// The baseline proposition: a digit produces ink, and the bitmap is self-consistent (the buffer
// is exactly width*height). Everything below assumes this much works.
TEST(AnnotationFont, ADigitRasterizesToInk) {
  const ann::TextBitmap b = ann::RasterizeLabel("22");
  ASSERT_FALSE(b.Empty()) << "the embedded font failed to produce any glyph for \"22\"";
  EXPECT_GT(b.width, 0);
  EXPECT_GT(b.height, 0);
  EXPECT_EQ(b.coverage.size(), static_cast<size_t>(b.width) * static_cast<size_t>(b.height));
  EXPECT_GT(TotalCoverage(b), 0);
}

// The run grows with the number of characters. Pins the pen actually advancing: a layout that
// drew every glyph at the same position, or only the first one, would still pass the case above.
TEST(AnnotationFont, WidthGrowsWithCharacterCount) {
  const ann::TextBitmap one = ann::RasterizeLabel("2");
  const ann::TextBitmap two = ann::RasterizeLabel("22");
  const ann::TextBitmap three = ann::RasterizeLabel("222");
  ASSERT_FALSE(one.Empty());
  ASSERT_FALSE(two.Empty());
  ASSERT_FALSE(three.Empty());
  EXPECT_GT(two.width, one.width);
  EXPECT_GT(three.width, two.width);
  // And the ink grows with it, which a run that advanced the pen but drew blanks would not do.
  EXPECT_GT(TotalCoverage(two), TotalCoverage(one));
}

// THE DEGREE SIGN, decoded as one codepoint and not as two bytes. This is the case the module's
// UTF-8 decode exists for: "22\xC2\xB0" walked byte by byte would look up 0xC2 and 0xB0
// separately, find neither in the alphabet, and silently drop BOTH — leaving a bitmap identical to
// plain "22". So the assertion is not "there is ink" (there would be) but "the degree sign added
// its own, to the RIGHT of the digits".
TEST(AnnotationFont, DegreeSignIsOneGlyphNotTwoBytes) {
  const ann::TextBitmap plain = ann::RasterizeLabel("22");
  const ann::TextBitmap degrees = ann::RasterizeLabel("22\xC2\xB0");
  ASSERT_FALSE(plain.Empty());
  ASSERT_FALSE(degrees.Empty());
  EXPECT_GT(degrees.width, plain.width) << "the degree sign added no advance — it was dropped";
  EXPECT_GT(TotalCoverage(degrees), TotalCoverage(plain)) << "the degree sign added no ink";

  // Its ink is in the columns past where the digits end, and it is a ring near the top of the
  // line rather than a full-height glyph — the two properties that distinguish a real degree sign
  // from any digit that might have been substituted for it.
  const std::vector<int> cols = ColumnInk(degrees);
  int tail_ink = 0;
  for (size_t x = static_cast<size_t>(plain.width); x < cols.size(); ++x) {
    tail_ink += cols[x];
  }
  EXPECT_GT(tail_ink, 0) << "the extra ink is not where the trailing glyph should be";

  // Vertical extent: every row of the degree sign's own columns sits in the upper half of the
  // run. A digit in that slot would reach the baseline.
  int lowest_row = -1;
  for (int y = 0; y < degrees.height; ++y) {
    for (int x = plain.width; x < degrees.width; ++x) {
      if (degrees.coverage[static_cast<size_t>(y) * degrees.width + x] != 0) {
        lowest_row = std::max(lowest_row, y);
      }
    }
  }
  ASSERT_GE(lowest_row, 0);
  EXPECT_LT(lowest_row, degrees.height * 3 / 4) << "the trailing glyph reaches the baseline; it is not a degree sign";
}

// A fractional value — the other format FormatAngleDeg emits. '.' and '-' are alphabet members,
// so neither may be dropped.
TEST(AnnotationFont, DecimalPointAndMinusAreInTheAlphabet) {
  const ann::TextBitmap digits = ann::RasterizeLabel("15");
  const ann::TextBitmap decimal = ann::RasterizeLabel("15.5");
  const ann::TextBitmap negative = ann::RasterizeLabel("-15");
  ASSERT_FALSE(decimal.Empty());
  ASSERT_FALSE(negative.Empty());
  EXPECT_GT(decimal.width, digits.width);
  EXPECT_GT(negative.width, digits.width);
}

// The anchor is the CENTRE of the run, so both offsets are negative and the horizontal one scales
// with the run: a two-character label hangs twice as far to the left of its anchor as a one-
// character label does. A drawer that treated the anchor as the top-left corner would place every
// label half a run off, which no per-glyph assertion above would notice.
TEST(AnnotationFont, TheAnchorIsTheCentreOfTheRun) {
  const ann::TextBitmap one = ann::RasterizeLabel("8");
  const ann::TextBitmap four = ann::RasterizeLabel("8888");
  ASSERT_FALSE(one.Empty());
  ASSERT_FALSE(four.Empty());
  EXPECT_LT(one.offset_x, 0);
  EXPECT_LT(one.offset_y, 0);
  EXPECT_LT(four.offset_x, one.offset_x) << "a longer run did not extend further left of its anchor";
  // Vertical placement does NOT depend on the run's length: the anchor centres the line box, not
  // the ink, so two runs of the same glyph sit at the same height.
  EXPECT_EQ(four.offset_y, one.offset_y);
}

// Empty in, empty out — and, the point of the case, a text made only of codepoints outside the
// closed alphabet is empty too rather than a row of substitute boxes. The alphabet is closed by
// construction (FormatAngleDeg), so a glyph appearing here would mean something upstream produced
// text this layer was never meant to draw.
TEST(AnnotationFont, EmptyAndOutOfAlphabetTextDrawNothing) {
  EXPECT_TRUE(ann::RasterizeLabel("").Empty());
  EXPECT_TRUE(ann::RasterizeLabel("abc").Empty());
  EXPECT_TRUE(ann::RasterizeLabel(" ").Empty());
}
