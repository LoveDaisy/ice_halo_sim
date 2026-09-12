// The annotation layers are independent of the exposure scalar: a frame rendered at
// `intensity_factor: 0` carries exactly the grid lines, circles, horizon, markers and label text the
// same frame carries at `intensity_factor: 1`, byte for byte, and only the ray-lit pixels differ.
//
// The shape of the defect this guards: ExposureScale() multiplies intensity_factor_ into every
// branch, so `0` makes the scale 0, and PostSnapshot used to take that as "nothing to draw at all"
// and return a bare zero-energy frame — grid, circles, horizon, markers and text included. The
// annotations are pure geometry (a config's angle lists and the view); the exposure has no
// business in them, and "only the grid, no light" is a picture a user does ask for.
//
// Three propositions, each its own case:
//   - AnnotationsSurviveZeroIntensity (screen) and PrintAnnotationsSurviveZeroIntensity (print):
//     where the annotation-free frame at intensity 1 is at that tone's zero-energy colour — i.e.
//     where no ray landed — the intensity 0 and intensity 1 frames agree byte for byte. Ray-lit
//     pixels are excluded from the comparison because those are the ones the factor is FOR.
//   - ZeroIntensityAndZeroCanvasEnergyRenderTheSameBytes (screen, and a print twin): the two code
//     paths that can produce an annotated zero-energy frame (the intensity 0 early exit, and the
//     full pixel loop with a scale above 0 whose every canvas pixel happens to carry no energy)
//     produce the SAME WHOLE IMAGE.
//     This one is about the implementation, not the product: it is the mechanical statement that
//     the two paths share one annotation composite rather than each carrying a copy that has to be
//     kept in step by hand. A canvas with a live scale and no lit pixel is made by landing the one
//     ray under the horizon, where `visible: upper` clips it out of the display.
//
// Sibling of test_render_consumer_grid.cpp and the other per-layer files; it reuses their fixture
// shape (a 120 deg linear frame centred 45 deg up, one ray, black background) and turns every
// annotation family on at once — the proposition is about the composite as a whole, and a family
// left out of the fixture is a family whose layer could quietly be the one still gated.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/annotation_overlay.hpp"
#include "server/render.hpp"
#include "support/render_anchor.hpp"
#include "util/color_space.hpp"

namespace lumice {
namespace {

constexpr int kW = 96;
constexpr int kH = 96;
constexpr int kTotalPix = kW * kH;
constexpr float kSunAltitude = 45.0f;

SunParam MakeSun() {
  return SunParam{ kSunAltitude, 0.0f, 0.5f };
}

GridLineParam Line(float value_deg, float opacity, float r, float g, float b) {
  GridLineParam p;
  p.value_ = value_deg;
  p.opacity_ = opacity;
  p.color_[0] = r;
  p.color_[1] = g;
  p.color_[2] = b;
  return p;
}

// A 120 deg linear frame centred 45 deg up, due 0 azimuth: altitudes [-15, 105]. Holds the sun and
// its 22 deg circle, the 30 / 60 deg parallels, the 0 / 90 deg meridians, the horizon along the
// bottom, the zenith, and the label text of every family.
RenderConfig MakeConfig(float intensity_factor, bool annotate, RenderConfig::Tone tone) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kLinear;
  cfg.lens_.fov_ = 120.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = kSunAltitude;
  cfg.visible_ = RenderConfig::kUpper;
  cfg.intensity_factor_ = intensity_factor;
  cfg.tone_ = tone;
  if (!annotate) {
    return cfg;
  }
  // One family at full opacity and one at half, so the comparison covers both the blend that
  // ignores its base (alpha 1) and the one that reads it (alpha 0.5).
  cfg.elevation_grid_ = { Line(30.0f, 1.0f, 1.0f, 0.0f, 0.0f), Line(60.0f, 1.0f, 1.0f, 0.0f, 0.0f) };
  cfg.longitude_grid_ = { Line(0.0f, 0.5f, 0.0f, 0.0f, 1.0f), Line(90.0f, 0.5f, 0.0f, 0.0f, 1.0f) };
  cfg.angular_dist_grid_ = { Line(22.0f, 1.0f, 0.0f, 1.0f, 0.0f) };
  cfg.horizon_ = true;
  cfg.zenith_nadir_.enabled_ = true;
  cfg.grid_label_ = true;
  cfg.angular_dist_label_ = true;
  cfg.horizon_label_ = true;
  return cfg;
}

// The ray that lands at a given altitude on the 0 deg meridian. SunWorldDir is the mapping every
// annotation uses for (altitude, azimuth) -> direction; borrowed here so the fixture's "under the
// horizon" is the renderer's own.
SimData MakeOneRayBatchAt(float altitude_deg) {
  SimData data;
  data.curr_wl_ = 550.0f;
  float dir[3];
  annotation::SunWorldDir(SunParam{ altitude_deg, 0.0f, 0.5f }, dir);
  data.outgoing_d_ = { dir[0], dir[1], dir[2] };
  data.outgoing_w_ = { 1.0f };
  return data;
}

std::vector<uint8_t> SnapshotOnce(RenderConsumer* rc, float ray_altitude_deg) {
  auto data = MakeOneRayBatchAt(ray_altitude_deg);
  rc->Consume(data);
  lumice::test::TakeSnapshotAtFormerSelfAnchor(rc);
  auto result = rc->GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  if (rr == nullptr || rr->img_buffer_ == nullptr) {
    return {};
  }
  return std::vector<uint8_t>(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
}

bool PixelIs(const std::vector<uint8_t>& img, int i, const uint8_t rgb[3]) {
  return img[static_cast<size_t>(i) * 3] == rgb[0] && img[static_cast<size_t>(i) * 3 + 1] == rgb[1] &&
         img[static_cast<size_t>(i) * 3 + 2] == rgb[2];
}

bool SamePixel(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, int i) {
  return a[static_cast<size_t>(i) * 3] == b[static_cast<size_t>(i) * 3] &&
         a[static_cast<size_t>(i) * 3 + 1] == b[static_cast<size_t>(i) * 3 + 1] &&
         a[static_cast<size_t>(i) * 3 + 2] == b[static_cast<size_t>(i) * 3 + 2];
}

// Every pixel any annotation mask marks, so the comparison can be shown to cover the lines and
// not only the empty sky around them. Labels and marker rings carry no mask and are not counted;
// they are compared all the same, as pixels of the frame.
std::vector<uint8_t> UnionOfMasks(const RenderConsumer& rc) {
  std::vector<uint8_t> out(static_cast<size_t>(kTotalPix), 0);
  const auto fold = [&out](const std::vector<uint8_t>& mask) {
    if (mask.size() != out.size()) {
      return;
    }
    for (size_t i = 0; i < out.size(); ++i) {
      out[i] = static_cast<uint8_t>(out[i] | mask[i]);
    }
  };
  for (const auto& m : rc.ElevationMasksForTest()) {
    fold(m);
  }
  for (const auto& m : rc.LongitudeMasksForTest()) {
    fold(m);
  }
  for (const auto& m : rc.AngularDistMasksForTest()) {
    fold(m);
  }
  fold(rc.HorizonMaskForTest());
  return out;
}

// The proposition shared by the screen and print cases: with the ray at the zenith (lit, and on the
// canvas), the annotated frames at intensity 1 and 0 agree everywhere the intensity 1 frame WITHOUT
// annotations is at `zero_energy` — i.e. everywhere no ray landed.
void ExpectAnnotationsIndependentOfIntensity(RenderConfig::Tone tone, const uint8_t zero_energy[3]) {
  RenderConsumer bare(MakeConfig(1.0f, false, tone), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_bare = SnapshotOnce(&bare, 90.0f);
  ASSERT_EQ(img_bare.size(), static_cast<size_t>(kTotalPix) * 3);

  RenderConsumer lit(MakeConfig(1.0f, true, tone), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_lit = SnapshotOnce(&lit, 90.0f);
  RenderConsumer dark(MakeConfig(0.0f, true, tone), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_dark = SnapshotOnce(&dark, 90.0f);
  ASSERT_EQ(img_lit.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_EQ(img_dark.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_GT(lit.ExposureScale(), 0.0f) << "the intensity 1 arm must go through the pixel loop";
  ASSERT_EQ(dark.ExposureScale(), 0.0f) << "the intensity 0 arm must take the zero-scale exit";

  const std::vector<uint8_t> on_a_line = UnionOfMasks(lit);
  size_t lit_pixels = 0;
  size_t compared = 0;
  size_t compared_on_a_line = 0;
  size_t differing = 0;
  size_t annotated_in_dark = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (!PixelIs(img_bare, i, zero_energy)) {
      ++lit_pixels;
      continue;  // a ray landed here; this is the pixel the factor exists to change
    }
    ++compared;
    if (on_a_line[static_cast<size_t>(i)] != 0) {
      ++compared_on_a_line;
    }
    if (!SamePixel(img_lit, img_dark, i)) {
      ++differing;
    }
    if (!PixelIs(img_dark, i, zero_energy)) {
      ++annotated_in_dark;
    }
  }
  EXPECT_GT(lit_pixels, 0u) << "the fixture's ray must land, or the intensity 1 arm is not lit at all";
  ASSERT_GT(compared_on_a_line, 0u) << "the comparison must reach the lines, not only the sky between them";
  EXPECT_GT(annotated_in_dark, 0u) << "intensity_factor 0 must still draw the annotations";
  EXPECT_EQ(differing, 0u) << "an annotation pixel may not depend on intensity_factor";
}

TEST(RenderConsumerIntensityIndependence, AnnotationsSurviveZeroIntensity) {
  constexpr uint8_t kBlack[3]{ 0, 0, 0 };
  ExpectAnnotationsIndependentOfIntensity(RenderConfig::kScreen, kBlack);
}

TEST(RenderConsumerIntensityIndependence, PrintAnnotationsSurviveZeroIntensity) {
  // print's zero-energy colour is the paper, encoded the way FillZeroEnergyImage encodes it.
  const RenderConfig cfg = MakeConfig(1.0f, false, RenderConfig::kPrint);
  uint8_t paper[3];
  for (int j = 0; j < 3; ++j) {
    paper[j] = static_cast<uint8_t>(LinearToSrgb(std::clamp(cfg.paper_[j], 0.0f, 1.0f)) * 255);
  }
  ExpectAnnotationsIndependentOfIntensity(RenderConfig::kPrint, paper);
}

// The two code paths that can produce an annotated zero-energy frame, held to the same bytes. Both
// arms are annotated. The `clipped` arm lands its ray 10 deg UNDER the horizon: inside the 120 deg
// frame, so it counts as landed and the exposure scale is live, but in the hemisphere
// `visible: upper` clips, so no canvas pixel carries energy and the pixel loop's base is the
// zero-energy colour everywhere. The `dark` arm lands its ray at the zenith and is told intensity
// 0. Same geometry, same annotations, one frame through the pixel loop and one through the early
// exit: the bytes must be the same, all of them.
void ExpectBothPathsRenderTheSameBytes(RenderConfig::Tone tone) {
  RenderConsumer clipped(MakeConfig(1.0f, true, tone), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_clipped = SnapshotOnce(&clipped, -10.0f);
  RenderConsumer dark(MakeConfig(0.0f, true, tone), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_dark = SnapshotOnce(&dark, 90.0f);
  ASSERT_EQ(img_clipped.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_EQ(img_dark.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_GT(clipped.ExposureScale(), 0.0f) << "the clipped arm must have a live scale";
  ASSERT_EQ(dark.ExposureScale(), 0.0f) << "the dark arm must take the zero-scale exit";

  size_t differing = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (!SamePixel(img_clipped, img_dark, i)) {
      ++differing;
    }
  }
  EXPECT_EQ(differing, 0u) << "the two paths must share one annotation composite";
  EXPECT_NE(img_dark, std::vector<uint8_t>(static_cast<size_t>(kTotalPix) * 3, img_dark[0]))
      << "the frame must not be a flat colour, or the comparison says nothing";
}

TEST(RenderConsumerIntensityIndependence, ZeroIntensityAndZeroCanvasEnergyRenderTheSameBytes) {
  ExpectBothPathsRenderTheSameBytes(RenderConfig::kScreen);
}

TEST(RenderConsumerIntensityIndependence, PrintZeroIntensityAndZeroCanvasEnergyRenderTheSameBytes) {
  ExpectBothPathsRenderTheSameBytes(RenderConfig::kPrint);
}

}  // namespace
}  // namespace lumice
