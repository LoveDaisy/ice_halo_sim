// What the two panorama export presets are allowed to change about a render.
//
// "Dual Fisheye Equal Area..." and "Equirectangular..." exist to force one canonical full-sky view
// regardless of where the user has the preview pointed. They do that by overwriting part of the
// PreviewParams the caller hands them — and the part they overwrite is the whole contract: the
// source geometry the preview computed and the exposure the user dialled in have to survive, or the
// exported panorama stops being a picture of the document in front of them.
//
// This is a struct in and a struct out, with no frame and no GL, which is why it lives here rather
// than beside the pixel-level export cases in test/gui/functional/test_export.cpp. Those cases lean
// on this one: they assert determinism and exposure once, at the preview lens, and do NOT repeat
// per projection — the reason they do not have to is that a preset provably leaves the exposure
// alone, which is asserted below and nowhere else.
//
// What a user sees when this breaks: the two panorama exports come out at a different brightness
// from the Screenshot of the same scene, or from each other; or the halo geometry shifts between
// the preview and the panorama because the source sub-struct was reset to its defaults on the way
// through.

#include <gtest/gtest.h>

#include "gui/export_fbo_renderer.hpp"
#include "gui/preview_renderer.hpp"

namespace lumice::gui {
namespace {

struct Preset {
  const char* name;
  void (*configure)(PreviewParams&);
  int expected_lens;
};

const Preset kPresets[] = {
  { "dual fisheye equal area", &ConfigureDualFisheyeExportParams, kLensTypeDualFisheyeEqualArea },
  { "equirectangular", &ConfigureEquirectExportParams, kLensTypeRectangular },
};

// Values a default-constructed PreviewParams does not hold, so "the preset inherited this" is
// distinguishable from "the field happened to already be right".
constexpr float kMaxAbsDz = 0.1234f;
constexpr float kRScale = 0.5678f;
constexpr float kIntensityFactor = 3.5f;
constexpr float kIntensityScale = 7.25f;
// The sky colour behind the halo. Its neighbour in the struct — the background IMAGE overlay —
// is one of the things a preset is REQUIRED to clear, which is exactly why this one has to be
// asserted: the two read alike and the correct treatment is opposite.
constexpr float kBackgroundR = 0.0331f;
constexpr float kBackgroundG = 0.1015f;
constexpr float kBackgroundB = 0.3185f;

PreviewParams CallerParams() {
  PreviewParams params{};
  params.source.max_abs_dz = kMaxAbsDz;
  params.source.r_scale = kRScale;
  params.exposure.intensity_factor = kIntensityFactor;
  params.exposure.intensity_scale = kIntensityScale;
  // Decoration state a panorama export is expected to clear: a horizon line drawn across an
  // equirectangular sky is a line across a picture nobody asked to annotate.
  params.overlay.show_horizon = true;
  params.overlay.show_grid = true;
  params.overlay.show_sun_circles = true;
  // The lens border belongs to the preview's lens, and a panorama export replaces that lens
  // outright — so drawing the old lens's image circle over the new projection would annotate the
  // picture with a boundary that is not in it.
  params.overlay.show_lens_border = true;
  params.bg.enabled = true;
  params.bg.alpha = 0.7f;
  // The sky COLOUR, which a preset must keep — unlike the background IMAGE above. A panorama
  // export is a picture of the same sky the preview shows; dropping the colour would export a
  // halo on black while the screen shows it on blue.
  params.background_color_linear[0] = kBackgroundR;
  params.background_color_linear[1] = kBackgroundG;
  params.background_color_linear[2] = kBackgroundB;
  return params;
}

// One loop, EXPECT rather than ASSERT: the first preset to drift must not hide the second, and
// "which of the two" is the question this case is asked.
TEST(ExportPresets, EachPresetOverwritesOnlyTheViewAndTheDecorations) {
  for (const Preset& preset : kPresets) {
    PreviewParams params = CallerParams();
    preset.configure(params);

    EXPECT_FLOAT_EQ(params.source.max_abs_dz, kMaxAbsDz) << preset.name << ": source geometry was overwritten";
    EXPECT_FLOAT_EQ(params.source.r_scale, kRScale) << preset.name << ": source geometry was overwritten";

    EXPECT_FLOAT_EQ(params.exposure.intensity_factor, kIntensityFactor)
        << preset.name << ": the exposure the caller set was overwritten";
    EXPECT_FLOAT_EQ(params.exposure.intensity_scale, kIntensityScale)
        << preset.name << ": the exposure the caller set was overwritten";

    EXPECT_EQ(params.view_proj.lens_type, preset.expected_lens) << preset.name;
    EXPECT_FLOAT_EQ(params.view_proj.fov, 180.0f) << preset.name << ": the canonical view is full-sky";
    EXPECT_EQ(params.view_proj.visible, kVisibleFull) << preset.name;

    EXPECT_FALSE(params.overlay.show_horizon) << preset.name << ": a decoration survived into the export";
    EXPECT_FALSE(params.overlay.show_grid) << preset.name << ": a decoration survived into the export";
    EXPECT_FALSE(params.overlay.show_sun_circles) << preset.name << ": a decoration survived into the export";
    EXPECT_FALSE(params.overlay.show_lens_border) << preset.name << ": a decoration survived into the export";
    EXPECT_FALSE(params.bg.enabled) << preset.name << ": the background image survived into the export";

    // Not the same field as the line above, and not the same verdict: the image overlay is
    // cleared, the sky colour is kept.
    EXPECT_FLOAT_EQ(params.background_color_linear[0], kBackgroundR)
        << preset.name << ": the sky colour was dropped on the way into the export";
    EXPECT_FLOAT_EQ(params.background_color_linear[1], kBackgroundG)
        << preset.name << ": the sky colour was dropped on the way into the export";
    EXPECT_FLOAT_EQ(params.background_color_linear[2], kBackgroundB)
        << preset.name << ": the sky colour was dropped on the way into the export";
  }
}

// The presets must also be idempotent in the sense the export path relies on: applying one to
// params that already describe that exact view changes nothing at all. This is the precondition
// under which functional/test_export.cpp's preview-vs-preset pixel parity case is a real test
// rather than a comparison of a render to itself.
TEST(ExportPresets, ApplyingAPresetTwiceIsTheSameAsApplyingItOnce) {
  for (const Preset& preset : kPresets) {
    PreviewParams once = CallerParams();
    preset.configure(once);
    PreviewParams twice = once;
    preset.configure(twice);

    EXPECT_EQ(twice.view_proj.lens_type, once.view_proj.lens_type) << preset.name;
    EXPECT_FLOAT_EQ(twice.view_proj.fov, once.view_proj.fov) << preset.name;
    EXPECT_EQ(twice.view_proj.visible, once.view_proj.visible) << preset.name;
    EXPECT_FLOAT_EQ(twice.exposure.intensity_factor, once.exposure.intensity_factor) << preset.name;
    EXPECT_FLOAT_EQ(twice.source.max_abs_dz, once.source.max_abs_dz) << preset.name;
    EXPECT_EQ(twice.overlay.show_horizon, once.overlay.show_horizon) << preset.name;
    EXPECT_EQ(twice.bg.enabled, once.bg.enabled) << preset.name;
    EXPECT_FLOAT_EQ(twice.background_color_linear[0], once.background_color_linear[0]) << preset.name;
    EXPECT_FLOAT_EQ(twice.background_color_linear[1], once.background_color_linear[1]) << preset.name;
    EXPECT_FLOAT_EQ(twice.background_color_linear[2], once.background_color_linear[2]) << preset.name;
  }
}

}  // namespace
}  // namespace lumice::gui
