// The preview shader's per-fragment evaluation of the annotation curves, at the places the rule is
// easy to get wrong and no reference image happens to look.
//
// The curves are level sets of world-space angle fields evaluated in the fragment shader
// (preview_renderer.cpp overlayAuxLines), against the same definition the CLI evaluates on the CPU
// (src/core/lens_proj_build.hpp LevelSetMaskFromField). Two of that definition's details are
// invisible on an ordinary frame and wrong in a specific, silent way when missed:
//
//   1. The azimuth field is CIRCULAR. It jumps from +180 to -180 across the anti-meridian, and a
//      naive evaluation fails there twice over: the distance from a fragment at azimuth -179.9 to
//      the level 180 reads as 359.9 (no line on that side), and the hardware derivative reads the
//      seam as a wall of gradient (a half-width clamped to its ceiling, i.e. a band several
//      degrees wide, on the other side). The CPU folds both through WrapAngleDiffDeg; the shader
//      must fold them through wrapAngleDiffDeg, and this file looks at the one column where the
//      difference shows.
//   2. The level lists are searched, not scanned, and the search wants them sorted. The uploader
//      sorts (UploadGridLevels); a caller's ordering must therefore make no difference to the
//      pixels, and a case here says so by rendering the same list twice in opposite orders.
//
// Capture path: RenderExportToRgba's own off-screen FBO, as test_angular_dist_circles.cpp uses,
// so nothing here depends on window size or panel layout. No simulation is run and the source
// texture is cleared, so the frame is black everywhere the annotation is not.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <utility>
#include <vector>

#include "gui/app.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// Large enough that a seam band and a proper line are unmistakably different widths: at this
// size and field of view a degree is ~4.3 px, so a half-width clamped to the 2 deg/px ceiling
// (3 deg either side) would be a band ~25 px wide against a 3 px line.
constexpr int kProbeW = 512;
constexpr int kProbeH = 512;
constexpr float kFov = 120.0f;

// A saturated primary, separable from any blend against black by a single channel test.
constexpr float kGridR = 0.0f;
constexpr float kGridG = 0.0f;
constexpr float kGridB = 1.0f;

struct RenderRequest {
  bool requested = false;
  bool done = false;
  float azimuth = 0.0f;
  std::vector<float> longitude_deg;
  std::vector<float> elevation_deg;
  std::vector<unsigned char> rgba;

  void Reset() { *this = RenderRequest{}; }
};

RenderRequest g_req;

void RunRenderRequest() {
  gui::g_preview.ClearTexture();

  gui::PreviewParams params = gui::g_preview_vp.params;
  params.view_proj.lens_type = gui::kLensTypeFisheyeEqualArea;
  params.view_proj.fov = kFov;
  params.view_proj.visible = gui::kVisibleFull;
  params.view_proj.elevation = 0.0f;
  params.view_proj.azimuth = g_req.azimuth;
  params.view_proj.roll = 0.0f;
  params.view_proj.front = false;
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // 8-bit RGB mode; no simulation has been run

  params.overlay.show_horizon = false;
  params.overlay.show_sun_circles = false;
  params.overlay.show_grid = true;
  params.overlay.grid_color[0] = kGridR;
  params.overlay.grid_color[1] = kGridG;
  params.overlay.grid_color[2] = kGridB;
  params.overlay.grid_alpha = 1.0f;  // opaque: the pixel test reads a colour, not a blend
  params.overlay.longitude_deg = g_req.longitude_deg;
  params.overlay.elevation_deg = g_req.elevation_deg;

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, kProbeW, kProbeH);
  g_req.done = true;
  g_req.requested = false;
}

void SeamGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
}

std::vector<unsigned char> RenderFrame(ImGuiTestContext* ctx, float azimuth, std::vector<float> longitude_deg,
                                       std::vector<float> elevation_deg) {
  g_req.Reset();
  g_req.azimuth = azimuth;
  g_req.longitude_deg = std::move(longitude_deg);
  g_req.elevation_deg = std::move(elevation_deg);
  g_req.requested = true;
  for (int i = 0; i < 120 && !g_req.done; ++i) {
    ctx->Yield();
  }
  return g_req.rgba;
}

bool LooksLikeTheGrid(const unsigned char* rgb) {
  return rgb[2] > 100 && rgb[0] < 80 && rgb[1] < 80;
}

// The runs of grid-coloured pixels along one row (top-down buffer), as [first, last] column pairs.
std::vector<std::pair<int, int>> GridRunsOnRow(const std::vector<unsigned char>& rgba, int row) {
  std::vector<std::pair<int, int>> runs;
  bool in_run = false;
  for (int col = 0; col < kProbeW; ++col) {
    const std::size_t off = (static_cast<std::size_t>(row) * kProbeW + col) * 4;
    const bool lit = LooksLikeTheGrid(&rgba[off]);
    if (lit && !in_run) {
      runs.emplace_back(col, col);
      in_run = true;
    } else if (lit) {
      runs.back().second = col;
    } else {
      in_run = false;
    }
  }
  return runs;
}

}  // namespace

void RegisterAnnotationLineSeamTests(ImGuiTestEngine* engine) {
  // The anti-meridian, looked at head-on: the camera at azimuth 180 puts the level azimuth = 180
  // on the frame's central column, with azimuth -179.x to one side of it and +179.x to the other.
  // Correctly folded, that is one ordinary line: a few pixels wide, straddling the centre. The two
  // ways of missing the fold each leave a different fingerprint on the central row — a run that
  // stops at the centre column instead of straddling it (distance not folded), or a run several
  // times too wide (gradient not folded) — and the assertions below are those two fingerprints.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "annotation_lines", "the_meridian_at_the_azimuth_seam_is_one_thin_line");
    t->GuiFunc = SeamGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<unsigned char> rgba = RenderFrame(ctx, /*azimuth=*/180.0f, { 180.0f }, {});
      IM_CHECK(!rgba.empty());
      IM_CHECK_EQ(rgba.size(), static_cast<std::size_t>(kProbeW) * kProbeH * 4);

      // The row through the frame centre, where the meridian is vertical and the seam's two sides
      // are left and right of one column.
      const std::vector<std::pair<int, int>> runs = GridRunsOnRow(rgba, kProbeH / 2);
      for (const auto& r : runs) {
        fprintf(stderr, "[annotation_lines] seam row run: cols %d..%d (width %d)\n", r.first, r.second,
                r.second - r.first + 1);
      }
      // Exactly one line crosses this row: the level 180 is the only one requested.
      IM_CHECK_EQ(runs.size(), static_cast<std::size_t>(1));
      const int first = runs[0].first;
      const int last = runs[0].second;
      const int centre = kProbeW / 2;
      // Straddles the centre column: pixels on BOTH sides of the seam are on the line. A distance
      // that was not folded lights only the +179.x side.
      IM_CHECK(first < centre);
      IM_CHECK(last >= centre);
      // Thin: a 3 px line plus its antialiasing fringe, never the ~25 px band a gradient that was
      // not folded produces here.
      IM_CHECK_LE(last - first + 1, 5);
    };
  }

  // The same meridian away from the seam, as the control: the line at azimuth 0 seen head-on has
  // no seam anywhere near it, and must be the same one thin, centred run. If this case and the one
  // above disagree, the seam is what differs.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "annotation_lines", "a_meridian_away_from_the_seam_is_the_same_line");
    t->GuiFunc = SeamGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<unsigned char> rgba = RenderFrame(ctx, /*azimuth=*/0.0f, { 0.0f }, {});
      IM_CHECK(!rgba.empty());
      const std::vector<std::pair<int, int>> runs = GridRunsOnRow(rgba, kProbeH / 2);
      IM_CHECK_EQ(runs.size(), static_cast<std::size_t>(1));
      const int centre = kProbeW / 2;
      IM_CHECK(runs[0].first < centre);
      IM_CHECK(runs[0].second >= centre);
      IM_CHECK_LE(runs[0].second - runs[0].first + 1, 5);
    };
  }

  // The order a level list arrives in is not part of the picture. The uploader sorts because the
  // shader's nearest-level search needs it sorted; a caller that hands the list over in descending
  // order — or any order — must get the same pixels, byte for byte.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "annotation_lines", "level_list_order_does_not_change_the_pixels");
    t->GuiFunc = SeamGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      const std::vector<float> lon_asc = {
        -150.0f, -120.0f, -60.0f, -30.0f, 0.0f, 30.0f, 60.0f, 120.0f, 150.0f, 180.0f
      };
      const std::vector<float> ele_asc = { -60.0f, -30.0f, 30.0f, 60.0f };
      std::vector<float> lon_desc(lon_asc.rbegin(), lon_asc.rend());
      std::vector<float> ele_desc(ele_asc.rbegin(), ele_asc.rend());

      const std::vector<unsigned char> ascending = RenderFrame(ctx, /*azimuth=*/40.0f, lon_asc, ele_asc);
      const std::vector<unsigned char> descending = RenderFrame(ctx, /*azimuth=*/40.0f, lon_desc, ele_desc);
      IM_CHECK(!ascending.empty());
      IM_CHECK_EQ(ascending.size(), descending.size());
      // Not a vacuous comparison: the frame must actually carry grid pixels.
      std::size_t lit = 0;
      for (std::size_t i = 0; i + 3 < ascending.size(); i += 4) {
        lit += LooksLikeTheGrid(&ascending[i]) ? 1 : 0;
      }
      IM_CHECK(lit > 100);
      IM_CHECK(ascending == descending);
    };
  }
}
