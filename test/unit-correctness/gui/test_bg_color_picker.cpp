// Background-photo eyedropper: the CPU mapping from a cursor position over the preview to one
// pixel of the imported photograph, and the sRGB triple that pixel becomes.
//
// Everything asserted here is reachable without a GL context or an ImGui frame, which is the whole
// reason the mapping was written as free functions in preview_renderer.hpp next to
// ComputeBgUvTransform rather than inline in the panel: the panel's job is reduced to reading the
// mouse, calling one of these, and writing the result into one field.

#include <array>
#include <fstream>
#include <iterator>
#include <optional>
#include <regex>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "gui/preview_renderer.hpp"

namespace {

using lumice::gui::BgPixelIndex;
using lumice::gui::BgSampleGeometry;
using lumice::gui::BgUvToPixelIndex;
using lumice::gui::ComputeBgUvTransform;
using lumice::gui::NdcPoint;
using lumice::gui::SampleBgColorAtScreenPos;
using lumice::gui::ScreenDeltaToNdcDelta;
using lumice::gui::ScreenPosToNdc;

// ==================================================================================================
// ScreenPosToNdc / ScreenDeltaToNdcDelta — the one owner of the screen<->NDC relation
// ==================================================================================================

TEST(BgColorPicker, ScreenPosToNdcMapsViewportCornersToNdcCorners) {
  // 200x100 device pixels at dpi 1: the rect in points is the same size.
  EXPECT_FLOAT_EQ(ScreenPosToNdc(0.0f, 0.0f, 1.0f, 1.0f, 200, 100).x, -1.0f);
  EXPECT_FLOAT_EQ(ScreenPosToNdc(0.0f, 0.0f, 1.0f, 1.0f, 200, 100).y, 1.0f);
  EXPECT_FLOAT_EQ(ScreenPosToNdc(200.0f, 100.0f, 1.0f, 1.0f, 200, 100).x, 1.0f);
  EXPECT_FLOAT_EQ(ScreenPosToNdc(200.0f, 100.0f, 1.0f, 1.0f, 200, 100).y, -1.0f);
  // Center.
  EXPECT_FLOAT_EQ(ScreenPosToNdc(100.0f, 50.0f, 1.0f, 1.0f, 200, 100).x, 0.0f);
  EXPECT_FLOAT_EQ(ScreenPosToNdc(100.0f, 50.0f, 1.0f, 1.0f, 200, 100).y, 0.0f);
}

TEST(BgColorPicker, ScreenPosToNdcFoldsDpiOnce) {
  // Retina: 200x100 POINTS is 400x200 device pixels, and vp_w/vp_h are device pixels. The far
  // corner in points must still land on the far corner in NDC — this is the factor that, dropped,
  // makes the picked colour come from the wrong quarter of the photo on a HiDPI screen.
  const NdcPoint far_corner = ScreenPosToNdc(200.0f, 100.0f, 2.0f, 2.0f, 400, 200);
  EXPECT_FLOAT_EQ(far_corner.x, 1.0f);
  EXPECT_FLOAT_EQ(far_corner.y, -1.0f);
}

TEST(BgColorPicker, ScreenDeltaToNdcDeltaFlipsYAndIgnoresOrigin) {
  const NdcPoint d = ScreenDeltaToNdcDelta(10.0f, 10.0f, 1.0f, 1.0f, 200, 100);
  EXPECT_FLOAT_EQ(d.x, 10.0f * 2.0f / 200.0f);
  // Screen Y grows downward, NDC Y upward.
  EXPECT_FLOAT_EQ(d.y, -10.0f * 2.0f / 100.0f);
  // The delta is the difference of two absolute positions, by construction. NEAR rather than
  // FLOAT_EQ: both positions carry the corner's -1, and subtracting them cancels it away, which
  // costs several significant digits. The identity is exact in arithmetic, not in float.
  const NdcPoint a = ScreenPosToNdc(30.0f, 40.0f, 1.0f, 1.0f, 200, 100);
  const NdcPoint b = ScreenPosToNdc(40.0f, 50.0f, 1.0f, 1.0f, 200, 100);
  EXPECT_NEAR(b.x - a.x, d.x, 1e-6f);
  EXPECT_NEAR(b.y - a.y, d.y, 1e-6f);
}

TEST(BgColorPicker, ScreenToNdcOnDegenerateViewportIsInertRatherThanDividingByZero) {
  const NdcPoint d = ScreenDeltaToNdcDelta(10.0f, 10.0f, 1.0f, 1.0f, 0, 0);
  EXPECT_FLOAT_EQ(d.x, 0.0f);
  EXPECT_FLOAT_EQ(d.y, 0.0f);
}

// ==================================================================================================
// BgUvToPixelIndex — the uv square's boundary, which is also the letterbox boundary
// ==================================================================================================

TEST(BgColorPicker, BgUvToPixelIndexCoversTheUnitSquareBoundary) {
  constexpr int kW = 4;
  constexpr int kH = 8;

  // Inside, including both closed ends. u == 1.0f floors to kW and must clamp back to kW - 1
  // rather than run one column past the buffer.
  struct Row {
    float u;
    float v;
    int col;
    int row;
  };
  const Row kInside[] = {
    { 0.0f, 0.0f, 0, 0 },   { 1.0f, 1.0f, kW - 1, kH - 1 }, { 0.5f, 0.5f, 2, 4 },
    { 0.99f, 0.01f, 3, 0 }, { 0.24f, 0.99f, 0, 7 },
  };
  for (const Row& r : kInside) {
    const std::optional<BgPixelIndex> got = BgUvToPixelIndex(r.u, r.v, kW, kH);
    // Non-fatal + continue rather than ASSERT: one row falling outside the square must not hide
    // the verdict on every row after it.
    if (!got.has_value()) {
      ADD_FAILURE() << "u=" << r.u << " v=" << r.v << " returned nullopt";
      continue;
    }
    EXPECT_EQ(got->col, r.col) << "u=" << r.u << " v=" << r.v;
    EXPECT_EQ(got->row, r.row) << "u=" << r.u << " v=" << r.v;
  }

  // Outside on either axis, either side — the letterbox the shader paints black.
  const float kOutside[][2] = {
    { -0.001f, 0.5f }, { 1.001f, 0.5f }, { 0.5f, -0.001f }, { 0.5f, 1.001f }, { -1.0f, 2.0f },
  };
  for (const auto& uv : kOutside) {
    EXPECT_FALSE(BgUvToPixelIndex(uv[0], uv[1], kW, kH).has_value()) << "u=" << uv[0] << " v=" << uv[1];
  }
}

TEST(BgColorPicker, BgUvToPixelIndexRejectsAnEmptyImage) {
  EXPECT_FALSE(BgUvToPixelIndex(0.5f, 0.5f, 0, 0).has_value());
  EXPECT_FALSE(BgUvToPixelIndex(0.5f, 0.5f, 4, 0).has_value());
}

// ==================================================================================================
// SampleBgColorAtScreenPos — screen point to sRGB, end to end
// ==================================================================================================

// A 2x2 image whose four pixels are mutually distinguishable AND asymmetric under both a
// horizontal and a vertical flip. A checkerboard or any symmetric pattern would let a mirrored
// mapping pass, and a mirrored mapping is precisely the defect this function is exposed to: the
// Y flip is owned by ComputeBgUvTransform's negative scale_y, and a second flip added here would
// be invisible to a symmetric fixture.
//
// Row-major, top-down, RGB — the order stbi decodes into and the order the bytes were uploaded in.
//   top-left  = (10, 20, 30)      top-right    = (40, 50, 60)
//   bot-left  = (70, 80, 90)      bottom-right = (100, 110, 120)
std::vector<unsigned char> MakeAsymmetric2x2() {
  return { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
}

BgSampleGeometry MakeSquareGeometry() {
  BgSampleGeometry geom;
  geom.dpi_scale_x = 1.0f;
  geom.dpi_scale_y = 1.0f;
  geom.vp_w = 200;
  geom.vp_h = 200;
  geom.bg_aspect = 1.0f;  // Matches the viewport: contain fit is the identity, no letterbox.
  geom.pan_x = 0.0f;
  geom.pan_y = 0.0f;
  geom.zoom = 1.0f;
  geom.img_w = 2;
  geom.img_h = 2;
  return geom;
}

void ExpectByteColor(const std::optional<std::array<float, 3>>& got, int r, int g, int b, const char* where) {
  ASSERT_TRUE(got.has_value()) << where;
  // Exact, not near: the photo is composited after the gamma curve, so its bytes and the
  // `background` field are the same encoding and byte/255 is the whole conversion. A tolerance
  // here would hide a colour-space round trip being introduced.
  EXPECT_FLOAT_EQ((*got)[0], static_cast<float>(r) / 255.0f) << where;
  EXPECT_FLOAT_EQ((*got)[1], static_cast<float>(g) / 255.0f) << where;
  EXPECT_FLOAT_EQ((*got)[2], static_cast<float>(b) / 255.0f) << where;
}

TEST(BgColorPicker, SampleReturnsTheExactPixelUnderTheCursorWithNoExtraFlip) {
  const std::vector<unsigned char> pixels = MakeAsymmetric2x2();
  const BgSampleGeometry geom = MakeSquareGeometry();

  // Screen points are relative to the viewport's top-left, y down. The upper-left quarter of the
  // screen must therefore read the image's TOP-LEFT pixel.
  ExpectByteColor(SampleBgColorAtScreenPos(50.0f, 50.0f, geom, pixels), 10, 20, 30, "upper-left");
  ExpectByteColor(SampleBgColorAtScreenPos(150.0f, 50.0f, geom, pixels), 40, 50, 60, "upper-right");
  ExpectByteColor(SampleBgColorAtScreenPos(50.0f, 150.0f, geom, pixels), 70, 80, 90, "lower-left");
  ExpectByteColor(SampleBgColorAtScreenPos(150.0f, 150.0f, geom, pixels), 100, 110, 120, "lower-right");
}

TEST(BgColorPicker, SampleAgreesWithTheShadersOwnUvLineRatherThanARewriteOfIt) {
  // Independent recomputation of the composition through the two functions the shader's CPU half
  // is made of. Its value is not the arithmetic (that would be a tautology) but the linkage: if
  // SampleBgColorAtScreenPos ever stops routing through ComputeBgUvTransform — the AC7 constraint,
  // "one owner of the uv mapping" — the two sides come apart here.
  const std::vector<unsigned char> pixels = MakeAsymmetric2x2();
  BgSampleGeometry geom = MakeSquareGeometry();
  geom.pan_x = 0.13f;
  geom.pan_y = -0.07f;
  geom.zoom = 1.7f;

  const float kScreenX = 62.0f;
  const float kScreenY = 141.0f;
  const NdcPoint ndc = ScreenPosToNdc(kScreenX, kScreenY, geom.dpi_scale_x, geom.dpi_scale_y, geom.vp_w, geom.vp_h);
  const lumice::gui::BgUvTransform t =
      ComputeBgUvTransform(geom.vp_w, geom.vp_h, geom.bg_aspect, geom.pan_x, geom.pan_y, geom.zoom);
  const std::optional<BgPixelIndex> expect_idx =
      BgUvToPixelIndex(ndc.x * t.scale_x + t.offset_x, ndc.y * t.scale_y + t.offset_y, geom.img_w, geom.img_h);
  ASSERT_TRUE(expect_idx.has_value());
  const size_t off = (static_cast<size_t>(expect_idx->row) * 2u + static_cast<size_t>(expect_idx->col)) * 3u;

  ExpectByteColor(SampleBgColorAtScreenPos(kScreenX, kScreenY, geom, pixels), pixels[off], pixels[off + 1],
                  pixels[off + 2], "pan+zoom");
}

TEST(BgColorPicker, SampleOnTheLetterboxIsANonAnswer) {
  const std::vector<unsigned char> pixels = MakeAsymmetric2x2();
  BgSampleGeometry geom = MakeSquareGeometry();
  // A wide viewport over a square photo: the contain fit pillarboxes it, so the outer columns of
  // the screen show black that belongs to no pixel of the image.
  geom.vp_w = 400;
  geom.vp_h = 200;
  geom.bg_aspect = 1.0f;

  EXPECT_FALSE(SampleBgColorAtScreenPos(5.0f, 100.0f, geom, pixels).has_value()) << "left band";
  EXPECT_FALSE(SampleBgColorAtScreenPos(395.0f, 100.0f, geom, pixels).has_value()) << "right band";
  // ... while the middle, where the photo actually is, still answers.
  EXPECT_TRUE(SampleBgColorAtScreenPos(150.0f, 50.0f, geom, pixels).has_value()) << "inside the photo";
}

TEST(BgColorPicker, SampleWithNoCpuCopyOrAShortOneIsANonAnswer) {
  const BgSampleGeometry geom = MakeSquareGeometry();
  // No photo loaded: the mode is not supposed to be reachable, but the read must not be a read
  // past the end if it is.
  EXPECT_FALSE(SampleBgColorAtScreenPos(50.0f, 50.0f, geom, {}).has_value());
  // Dimensions and buffer disagree (a stale copy): the last pixel is not there.
  std::vector<unsigned char> truncated = MakeAsymmetric2x2();
  truncated.resize(6);
  EXPECT_FALSE(SampleBgColorAtScreenPos(150.0f, 150.0f, geom, truncated).has_value());
}

TEST(BgColorPicker, ZoomingInNarrowsWhichPartOfThePhotoIsReachable) {
  // Not a formula check — a statement that the user's pan/zoom is in the loop at all. At zoom 4 the
  // centre quarter of the photo fills the screen, so the whole viewport reads the two pixels
  // straddling the centre rather than all four corners.
  const std::vector<unsigned char> pixels = MakeAsymmetric2x2();
  BgSampleGeometry geom = MakeSquareGeometry();
  geom.zoom = 4.0f;
  ExpectByteColor(SampleBgColorAtScreenPos(5.0f, 5.0f, geom, pixels), 10, 20, 30, "zoomed upper-left");
  ExpectByteColor(SampleBgColorAtScreenPos(195.0f, 195.0f, geom, pixels), 100, 110, 120, "zoomed lower-right");
  // Pan enters the same way: bg_offset_* is ADDED to the uv offset (ComputeBgUvTransform), so a
  // positive pan_x slides the sampled uv toward +u. At the screen's horizontal centre ndc.x is 0
  // and u is the offset alone — 0.5 + 0.30 = 0.8, which is the RIGHT column of a 2-wide image
  // rather than the boundary between the two that pan 0 would land on.
  geom.zoom = 1.0f;
  geom.pan_x = 0.30f;
  ExpectByteColor(SampleBgColorAtScreenPos(100.0f, 50.0f, geom, pixels), 40, 50, 60, "panned centre");
}

// ==================================================================================================
// AC6 — "picking writes nothing to the document but the colour"
// ==================================================================================================
//
// Two independent halves, because neither alone is the claim.
//
// The first is structural and needs no test to state: SampleBgColorAtScreenPos above takes a
// BgSampleGeometry by value and the pixel buffer by const reference. It cannot reach GuiState, so
// no amount of future editing INSIDE the sampling path can write to the document.
//
// The second is the part that is not structural — the panel's pick branch, which does hold
// `g_state` and `rc`. It is inside an ImGui frame and cannot be called from here, so what is
// asserted is its TEXT: the set of assignments the branch contains. This is the same source-scan
// shape test_user_defaults.cpp uses to prove a call site exists, applied to prove that assignments
// do not. It goes red the day someone parks a "remember the last picked point" field in GuiState.

TEST(BgColorPicker, PickBranchAssignsNothingButTheSkyColour) {
  std::ifstream in(LUMICE_GUI_APP_PANELS_CPP_PATH);
  ASSERT_TRUE(in.is_open());
  const std::string src((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(src.empty());

  const std::string open_marker = "if (g_bg_pick.active && is_hovered) {";
  const size_t begin = src.find(open_marker);
  ASSERT_NE(begin, std::string::npos) << "the pick branch was renamed; update this scan with it";
  ASSERT_EQ(src.find(open_marker, begin + open_marker.size()), std::string::npos) << "expected exactly one branch";

  // Walk to the branch's matching close brace so the scan cannot silently spill into the code
  // after it (which is where the four gesture branches, full of legitimate writes, live).
  size_t depth = 0;
  size_t end = std::string::npos;
  for (size_t i = begin + open_marker.size() - 1; i < src.size(); ++i) {
    if (src[i] == '{') {
      ++depth;
    } else if (src[i] == '}') {
      --depth;
      if (depth == 0) {
        end = i;
        break;
      }
    }
  }
  ASSERT_NE(end, std::string::npos) << "unbalanced braces in the pick branch";
  const std::string body = src.substr(begin, end - begin);

  // Every `<something>.<field> = ` and `<something>-><field> = ` in the branch. `==` is excluded by
  // requiring the character after `=` not to be `=`; compound assignments are caught by the
  // optional operator character before it.
  const std::regex assign(R"((\w+)(?:\.|->)([A-Za-z_]\w*)(\[\d+\])?\s*[-+*/]?=[^=])");
  std::vector<std::string> targets;
  for (std::sregex_iterator it(body.begin(), body.end(), assign), last; it != last; ++it) {
    targets.push_back((*it)[1].str() + "." + (*it)[2].str());
  }

  // `geom.*` is the local sample-geometry carrier being filled in — a stack value, not state.
  // `rc.background` is the one document write; `g_bg_pick.*` is the mode's own flag, which lives
  // outside GuiState precisely so that setting it is not a document write.
  for (const std::string& t : targets) {
    const bool allowed = t.rfind("geom.", 0) == 0 || t.rfind("g_bg_pick.", 0) == 0 || t == "rc.background";
    EXPECT_TRUE(allowed) << "the eyedropper branch assigns to `" << t
                         << "`; picking must write nothing to the document but renderer.background";
  }
  // And the one write it IS allowed to make must actually be there — otherwise an empty branch
  // would pass this test while doing nothing.
  EXPECT_NE(std::find(targets.begin(), targets.end(), std::string("rc.background")), targets.end());
}

}  // namespace
