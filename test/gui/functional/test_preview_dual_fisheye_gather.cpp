// Where the preview shader READS the dual-fisheye source texture.
//
// Core writes that texture with `px = floor(fx)` — texel `px` owns the continuous interval
// `[px, px+1)`, so its centre sits at `fx = px + 0.5` (doc/coordinate-convention.md §11). The
// preview's fragment shader runs the same projection backwards to decide which texel a canvas
// pixel should read, and its `dualFisheyeToUV` has to hand GL a UV in that same convention.
// GL's own texture addressing already places texel centres at `uv * tex_res = px + 0.5`, so the
// conversion is `uv = fx / tex_res` and nothing else: writing `uv = (fx + 0.5) / tex_res` applies
// the half-texel centring a SECOND time, and every fragment then lands on a texel corner and
// returns a 2x2 bilinear average instead of the texel it asked for.
//
// That is not a "reads the wrong cell" bug — it is a permanent half-texel blur over the whole
// preview, plus a visible artefact where the blur's direction is observable: the two discs map
// `y_norm` to `fx` with opposite signs, so one shared pixel offset means the equator overlap band
// blends two DIFFERENT patches of sky, and a smear appears along the horizon. This is the third
// pairing of the same floor convention (core binning <-> GUI source gather); the other two
// (forward/inverse render-domain mask) are pinned by test/golden-analytic/core/test_visible_mask.cpp
// and were the only ones audited when the convention changed in f1bf1e9e.
//
// HOW THIS IS MEASURED, and why it needs no mirrored copy of the shader's arithmetic.
// Set the source format's `r_scale` to 1.0 (each disc images exactly one hemisphere, no overlap
// margin) and render the dual-equal-area export preset at a canvas the same size as the source
// texture. The display-side inverse and the gather-side forward are then the same projection at
// the same scale, and the whole chain collapses to an identity: canvas pixel (col, row) must read
// source texel (col, row), whose centre is the continuous coordinate (col + 0.5, row + 0.5).
// Derivation, upper disc, canvas W x H and texture W x H with R = min(W/2, H)/2:
//
//   pos      = v_ndc * u_resolution * 0.5      -> (col + 0.5 - W/2,  H/2 - (row + 0.5))
//   pos_ovl  = (pos.x, -pos.y)                 -> left_pos = (col + 0.5 - W/2 + R, row + 0.5 - H/2)
//   theta    = 2*asin(use_r / sqrt(2)),  use_r = |left_pos| / R
//   sin(theta) / sqrt(1 + cos(theta)) == sqrt(1 - cos(theta)) == use_r   (r_scale = 1)
//   => xy_norm = (use_r*sin(phi), -use_r*cos(phi)) and dualFisheyeToUV's `pixel` comes back
//      exactly (col + 0.5, row + 0.5).
//
// So the ground truth here is the source texture itself, not a second implementation of the
// formula that would drift away from the shader the moment anyone edited one of them.
//
// The probe is a one-texel checkerboard. Under the correct convention each canvas pixel samples a
// single texel with zero bilinear weight left over, so the frame comes back with exactly two
// values in it — full contrast. Under the doubled half-texel each fragment averages a 2x2 block
// containing two of each colour, so EVERY pixel renders the same mid grey and the contrast
// collapses to zero. Any partial offset lands in between, on the same scale.
//
// What a user sees when this breaks: a horizontal smear along the horizon (the overlap band
// blending two different patches of sky), and a preview, PNG export and .lmc bake that are all
// half a texel softer than the data they were given.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "test_gui_shared.hpp"

namespace {

// Source texture / canvas size. Width = 2 * height is the dual-fisheye layout: two discs of
// radius R = min(W/2, H)/2 = H/2 side by side, each exactly filling its half. These geometry
// constants (and kDiscCx below) restate that layout by hand rather than deriving it from a
// production function: the layout is a stable convention other cases already cover, and it is not
// what this case is verifying. If it ever changes, nothing here goes red — the probes just land
// somewhere else — so they have to be updated with it.
constexpr int kTexW = 256;
constexpr int kTexH = 128;
constexpr int kDiscR = 64;  // min(kTexW/2, kTexH) / 2

// How far from a disc centre the probes are allowed to reach. Well inside the rim: `use_r` -> 1 is
// where asin's derivative diverges and where the disc clip and the equator hemisphere test live,
// none of which this case is about.
constexpr int kProbeRadius = 50;

// The two checkerboard colours, in XYZ. The bright one is D65 white at Y = 1 so the gamut clip
// leaves it neutral; the dark one is zero energy, which with no background renders pure black.
constexpr float kBrightXyz[3] = { 0.9505f, 1.0f, 1.089f };
constexpr float kDarkXyz[3] = { 0.0f, 0.0f, 0.0f };
constexpr float kIntensityScale = 0.5f;  // renders the bright texel around byte 188, off both rails

// Both thresholds are placed against MEASURED values, not against the defect they were written
// for. On this tree's GPU the sharp frame reads exactly 188 / 0 with a spread of 0 in each parity
// class, and the half-texel frame reads a flat 137 everywhere (the 2x2 mean of the same two
// colours) — so the contrast the case is separating is 188 against 0.
//
// A sampling point offset by `a` texels in both axes scales the checkerboard contrast by
// (1 - 2a)^2, so 150 LSB (80% of sharp) is the point below which an offset of roughly 0.05 texel
// or more is reported. That is a long way above anything float noise in the asin/atan chain can
// produce (offsets there are ~1e-6 texels) and a long way below the whole defect this guards.
constexpr double kMinCheckerContrastLsb = 150.0;
// Measured 0 on both classes on both discs. 2 LSB of slack is for another driver's last bit in the
// same trig chain; genuine drift across the disc would spread by tens, not by two.
constexpr int kMaxParitySpreadLsb = 2;

// Both numbers above were calibrated on one machine's GPU and have NOT been checked across GPUs or
// drivers — this file's category does not run in CI (AGENTS.md), so no other renderer has ever
// evaluated them. A first red on unfamiliar hardware is therefore worth measuring before it is
// treated as a regression: read the reported contrast/spread against the values quoted above.

// A render request marshalled to the frame loop: the upload and RenderExportToRgba both need the
// thread that owns the GL context, which is the render thread and not the test coroutine. Same
// crossing test_preview_background.cpp and test_lens_border.cpp make.
struct RenderRequest {
  bool requested = false;
  bool done = false;
  std::vector<unsigned char> rgba;
};

RenderRequest g_req;

void RunRenderRequest() {
  std::vector<float> xyz(static_cast<std::size_t>(kTexW) * kTexH * 3);
  for (int row = 0; row < kTexH; ++row) {
    for (int col = 0; col < kTexW; ++col) {
      const float* src = ((col + row) % 2 == 0) ? kBrightXyz : kDarkXyz;
      const std::size_t off = (static_cast<std::size_t>(row) * kTexW + col) * 3;
      xyz[off + 0] = src[0];
      xyz[off + 1] = src[1];
      xyz[off + 2] = src[2];
    }
  }
  gui::g_preview.UploadXyzTexture(xyz.data(), kTexW, kTexH);

  gui::PreviewParams params{};
  gui::ConfigureDualFisheyeExportParams(params);
  // Set AFTER the preset: the preset owns the view and the decorations, the source format is the
  // caller's (production fills it from kDualFisheyeOverlap). r_scale = 1 and no overlap band is
  // what makes the display inverse and the gather forward the same projection at the same scale,
  // and therefore what makes the identity above exact. The convention under test does not depend
  // on either value — both sit outside dualFisheyeToUV — so pinning them here narrows the
  // measurement without narrowing the claim.
  params.source.r_scale = 1.0f;
  params.source.max_abs_dz = 0.0f;
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = kIntensityScale;

  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, params, kTexW, kTexH);
  g_req.done = true;
  g_req.requested = false;
}

void GatherGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_req.requested && !g_req.done) {
    RunRenderRequest();
  }
}

// Ask the render thread for one frame and wait for it. Bounded so a wiring regression fails at an
// assertion rather than hanging the suite.
void RenderFrame(ImGuiTestContext* ctx) {
  g_req.requested = true;
  g_req.done = false;
  for (int i = 0; i < 60 && !g_req.done; ++i) {
    ctx->Yield();
  }
}

// Per-parity statistics over the probe discs. Green channel only: the checkerboard is neutral, so
// all three channels carry the same signal and one of them is the whole measurement.
struct ParityStats {
  int count = 0;
  int min_v = 255;
  int max_v = 0;
  long long sum = 0;

  void Add(int v) {
    ++count;
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
    sum += v;
  }
  double Mean() const { return count > 0 ? static_cast<double>(sum) / count : 0.0; }
  int Spread() const { return count > 0 ? max_v - min_v : 0; }
};

}  // namespace

void RegisterPreviewDualFisheyeGatherTests(ImGuiTestEngine* engine) {
  // The regression guard for the third pairing of the pixel-centre convention. It fails on any
  // net offset between what core binned and what the shader gathers, in either direction and from
  // either side of the pair: the contrast it measures is a property of the rendered frame against
  // the uploaded texels, so a change to core's binning, to dualFisheyeToUV, or to the display
  // inverse that breaks the agreement shows up here.
  //
  // Scope, stated so the next reader does not over-trust it: this measures a SYSTEMATIC offset
  // shared by every pixel. It is not a check that some particular direction lands on some
  // particular texel — a projection error that moved content around WITHOUT smearing it (a
  // rotation, a mirrored disc) would keep the checkerboard sharp and pass here. Those belong to
  // the lens_proj reference images and to test_visible_mask_gui_parity.cpp.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "preview_dual_fisheye_gather", "source_texel_centres_survive_the_round_trip");
    t->GuiFunc = GatherGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      RenderFrame(ctx);
      IM_CHECK(!g_req.rgba.empty());
      IM_CHECK_EQ(static_cast<int>(g_req.rgba.size()), kTexW * kTexH * 4);

      // Both discs: the upper one is laid out with `-y_norm * R` and the lower with `+y_norm * R`,
      // two separate branches of dualFisheyeToUV, and the horizon artefact is precisely what their
      // disagreement produces. Probing only one would leave the other unguarded.
      const int kDiscCx[2] = { kTexW / 2 - kDiscR, kTexW / 2 + kDiscR };
      const char* kDiscName[2] = { "upper", "lower" };

      for (int disc = 0; disc < 2; ++disc) {
        ParityStats even_st;
        ParityStats odd_st;
        for (int row = 0; row < kTexH; ++row) {
          for (int col = 0; col < kTexW; ++col) {
            const int dx = col - kDiscCx[disc];
            const int dy = row - kTexH / 2;
            if (dx * dx + dy * dy > kProbeRadius * kProbeRadius) {
              continue;
            }
            const int g = g_req.rgba[(static_cast<std::size_t>(row) * kTexW + col) * 4 + 1];
            ((col + row) % 2 == 0 ? even_st : odd_st).Add(g);
          }
        }

        // Non-fatal: a fatal assert here would return out of the whole case and hide the second
        // disc, which is the one the horizon artefact actually lives on.
        if (even_st.count == 0 || odd_st.count == 0) {
          IM_ERRORF(
              "%s disc: probe window collected no pixels (bright %d, dark %d) — the disc centre or radius is wrong",
              kDiscName[disc], even_st.count, odd_st.count);
          continue;
        }

        const double contrast = even_st.Mean() - odd_st.Mean();
        // A one-texel checkerboard sampled exactly comes back as the two colours it was written
        // with. Half a texel off in both axes averages two of each and the contrast is zero; a
        // partial offset scales in between. The threshold sits far below the sharp value and far
        // above zero, so neither a GPU's last bit nor an honest rounding lands near it.
        if (contrast < kMinCheckerContrastLsb) {
          IM_ERRORF(
              "%s disc: checkerboard contrast is %.2f LSB (bright mean %.2f over %d px, dark mean "
              "%.2f over %d px). The source texels are one apart, so a contrast near zero means "
              "every fragment is averaging a 2x2 block instead of reading the texel it asked for — "
              "dualFisheyeToUV is applying the half-texel centre offset that GL's own addressing "
              "already applies. Expected at least %.1f.",
              kDiscName[disc], contrast, even_st.Mean(), even_st.count, odd_st.Mean(), odd_st.count,
              kMinCheckerContrastLsb);
        }

        // Each parity class must also be FLAT. The contrast above survives an offset that is the
        // same everywhere; a gather that drifts across the disc (a scale error, a wrong disc
        // radius) would keep some contrast while spreading each class out, and only this catches
        // that.
        if (even_st.Spread() > kMaxParitySpreadLsb || odd_st.Spread() > kMaxParitySpreadLsb) {
          IM_ERRORF(
              "%s disc: a parity class is not flat (bright spread %d LSB over [%d, %d], dark spread "
              "%d LSB over [%d, %d]). Every texel of one colour renders through an identical chain, "
              "so anything above %d LSB means the sampling point is drifting across the disc rather "
              "than sitting on texel centres.",
              kDiscName[disc], even_st.Spread(), even_st.min_v, even_st.max_v, odd_st.Spread(), odd_st.min_v,
              odd_st.max_v, kMaxParitySpreadLsb);
        }
      }
    };
  }
}
