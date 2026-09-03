// Core-side annotation overlay: the generalized level-set extraction that every auxiliary line
// goes through, and the ComputeOverlay entry point that assembles a whole request out of it.
//
// The level-set half is asserted against HAND-COMPUTED expectations on synthetic fields rather
// than against another implementation, because there is no second implementation to compare to —
// the whole point of the generalization is that the horizon, the parallels, the meridians and the
// angular-distance circles stop having separate ones. Synthetic ramps make the shader's width rule
// (half-width = 1.5 x the local gradient) arithmetic instead of pictorial: a field that climbs k
// degrees per row lights exactly the rows within 1.5k degrees of the level.
//
// ComputeOverlay's own assertions are structural (is the mask a subset of the drawable region, do
// n levels produce n bands, does a circular field close its seam) plus one bit-identity check of
// the row-parallel split against a serial reference. Cross-implementation agreement with the GUI
// is a different question and lives in test/unit-correctness/gui/
// test_annotation_overlay_gui_parity.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "core/annotation_overlay.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation

namespace {

using lumice::LensParam;
using lumice::RenderConfig;
namespace ann = lumice::annotation;
namespace md = lumice::mask_detail;

size_t CountOn(const std::vector<uint8_t>& m) {
  return static_cast<size_t>(std::count(m.begin(), m.end(), uint8_t{ 1 }));
}

// Which rows a mask lights, for a mask built from a field that varies only with the row.
std::vector<int> LitRows(const std::vector<uint8_t>& mask, int width, int height) {
  std::vector<int> rows;
  for (int py = 0; py < height; ++py) {
    for (int px = 0; px < width; ++px) {
      if (mask[static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px)] != 0) {
        rows.push_back(py);
        break;
      }
    }
  }
  return rows;
}

// A field climbing `slope` degrees per row, centred so row `height/2` reads 0.
std::vector<float> RowRampField(int width, int height, float slope) {
  std::vector<float> f(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0f);
  for (int py = 0; py < height; ++py) {
    for (int px = 0; px < width; ++px) {
      f[static_cast<size_t>(py) * static_cast<size_t>(width) + static_cast<size_t>(px)] =
          (static_cast<float>(py) - static_cast<float>(height) / 2.0f) * slope;
    }
  }
  return f;
}

ann::ViewSnapshot MakeView(LensParam::LensType type, float fov, int w, int h) {
  ann::ViewSnapshot v;
  v.lens_type = type;
  v.fov_deg = fov;
  v.width = w;
  v.height = h;
  v.visible = RenderConfig::kFull;
  return v;
}

}  // namespace

// =================================================================================================
// Generalized level-set extraction
// =================================================================================================

TEST(AnnotationLevelSet, LevelZeroReproducesTheHorizonWidthRule) {
  // slope = 1 deg/row, so the local gradient fw is 1 deg (vertical neighbour) and the half-width
  // is 1.5 deg: rows whose value is strictly inside +/-1.5 deg light up, i.e. the three rows at
  // offset -1, 0, +1 from the zero crossing.
  constexpr int kW = 8;
  constexpr int kH = 16;
  const std::vector<float> field = RowRampField(kW, kH, 1.0f);
  const std::vector<uint8_t> all(static_cast<size_t>(kW) * kH, 1);

  const std::vector<uint8_t> mask = md::LevelSetMaskFromField(field, all, all, kW, kH, { 0.0f }, /*circular=*/false);
  EXPECT_EQ(LitRows(mask, kW, kH), (std::vector<int>{ 7, 8, 9 }));
}

TEST(AnnotationLevelSet, NonZeroLevelShiftsTheBandWithoutChangingItsWidth) {
  constexpr int kW = 8;
  constexpr int kH = 32;
  const std::vector<float> field = RowRampField(kW, kH, 1.0f);
  const std::vector<uint8_t> all(static_cast<size_t>(kW) * kH, 1);

  // Row 16 reads 0, so level 5 sits at row 21. The width rule reads the same local gradient, so
  // the band is the same three rows wide — this is why shifting the field needs no separate rule.
  const std::vector<uint8_t> mask = md::LevelSetMaskFromField(field, all, all, kW, kH, { 5.0f }, /*circular=*/false);
  EXPECT_EQ(LitRows(mask, kW, kH), (std::vector<int>{ 20, 21, 22 }));
}

TEST(AnnotationLevelSet, EveryRequestedLevelLandsInOneMask) {
  constexpr int kW = 8;
  constexpr int kH = 40;
  const std::vector<float> field = RowRampField(kW, kH, 1.0f);
  const std::vector<uint8_t> all(static_cast<size_t>(kW) * kH, 1);

  const std::vector<uint8_t> mask =
      md::LevelSetMaskFromField(field, all, all, kW, kH, { -10.0f, 0.0f, 10.0f }, /*circular=*/false);
  EXPECT_EQ(LitRows(mask, kW, kH), (std::vector<int>{ 9, 10, 11, 19, 20, 21, 29, 30, 31 }));
}

TEST(AnnotationLevelSet, ACircularFieldMeasuresDistanceAcrossItsSeam) {
  // An azimuth field stepping 1 deg per column across the +/-180 seam: 178, 179, 180, -179, -178,
  // ... The meridian at 180 deg is ONE line, and the three columns straddling it are all within a
  // degree of it. A subtraction that does not wrap reads the column holding -179 as 359 deg away
  // and drops it, so the meridian is drawn with a piece missing exactly at the wrap.
  constexpr int kW = 36;
  constexpr int kH = 4;
  std::vector<float> field(static_cast<size_t>(kW) * kH, 0.0f);
  for (int py = 0; py < kH; ++py) {
    for (int px = 0; px < kW; ++px) {
      float v = 178.0f + static_cast<float>(px);
      while (v > 180.0f) {
        v -= 360.0f;
      }
      field[static_cast<size_t>(py) * kW + static_cast<size_t>(px)] = v;
    }
  }
  const std::vector<uint8_t> all(static_cast<size_t>(kW) * kH, 1);

  // kW is captured explicitly rather than left to the "constexpr needs no capture" rule: MSVC
  // rejects the implicit form (C3493) where clang and gcc accept it.
  auto lit_columns = [kW](const std::vector<uint8_t>& m) {
    std::vector<int> cols;
    for (int px = 0; px < kW; ++px) {
      if (m[static_cast<size_t>(px)] != 0) {
        cols.push_back(px);
      }
    }
    return cols;
  };

  // Circular: gradient is a uniform 1 deg, half-width 1.5 deg, so 179 / 180 / -179 all light.
  const std::vector<uint8_t> circular =
      md::LevelSetMaskFromField(field, all, all, kW, kH, { 180.0f }, /*circular=*/true);
  EXPECT_EQ(lit_columns(circular), (std::vector<int>{ 1, 2, 3 }));

  // The same input read as a plain number line loses the far side of the seam. Asserted rather
  // than described, because "the wrap matters here" is exactly the claim the circular flag makes.
  const std::vector<uint8_t> plain = md::LevelSetMaskFromField(field, all, all, kW, kH, { 180.0f }, /*circular=*/false);
  EXPECT_EQ(lit_columns(plain), (std::vector<int>{ 1, 2 }));

  // A level no column comes near draws nothing — the wrapping must not fold distant values in.
  EXPECT_EQ(CountOn(md::LevelSetMaskFromField(field, all, all, kW, kH, { 0.0f }, /*circular=*/true)), 0u);
}

TEST(AnnotationLevelSet, GradientIsMeasuredAcrossImagedButDrawnOnlyOnDrawable) {
  // The distinction that makes a half-sky horizon survive: `imaged` carries the gradient,
  // `drawable` gates the paint. Blank the lower half of `drawable` and the upper half of the band
  // must still light — measuring the gradient across `drawable` instead collapses it to the 1e-4
  // clamp and the line disappears exactly where it was asked for.
  constexpr int kW = 8;
  constexpr int kH = 16;
  const std::vector<float> field = RowRampField(kW, kH, 1.0f);
  const std::vector<uint8_t> imaged(static_cast<size_t>(kW) * kH, 1);
  std::vector<uint8_t> drawable(static_cast<size_t>(kW) * kH, 1);
  for (int py = kH / 2; py < kH; ++py) {
    for (int px = 0; px < kW; ++px) {
      drawable[static_cast<size_t>(py) * kW + static_cast<size_t>(px)] = 0;
    }
  }
  const std::vector<uint8_t> mask =
      md::LevelSetMaskFromField(field, imaged, drawable, kW, kH, { 0.0f }, /*circular=*/false);
  EXPECT_EQ(LitRows(mask, kW, kH), (std::vector<int>{ 7 }));
}

TEST(AnnotationLevelSet, MalformedInputYieldsAnEmptyMaskRatherThanReadingOutOfBounds) {
  const std::vector<float> field(10, 0.0f);
  const std::vector<uint8_t> all(10, 1);
  EXPECT_EQ(CountOn(md::LevelSetMaskFromField(field, all, all, 4, 4, { 0.0f }, false)), 0u);
  EXPECT_EQ(CountOn(md::LevelSetMaskFromField(field, all, all, 5, 2, {}, false)), 0u);
}

// =================================================================================================
// The row-parallel split
// =================================================================================================

TEST(AnnotationParallelRows, ParallelMaskBuildMatchesASerialReference) {
  // 512x512 is above ParallelRows' threshold, so this exercises the threaded path. The reference
  // is written out here rather than reusing the production loop: a comparison against the same
  // code cannot fail.
  RenderConfig cfg;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 140.0f;
  cfg.resolution_[0] = 512;
  cfg.resolution_[1] = 512;
  cfg.view_.el_ = 35.0f;
  cfg.view_.az_ = 20.0f;
  cfg.visible_ = RenderConfig::kUpper;

  const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
  const float short_pix = 512.0f;
  const lm_proj::ProjParams p = lumice::BuildProjParams(cfg, rot, short_pix);

  std::vector<uint8_t> expected(512u * 512u, 0);
  for (int py = 0; py < 512; ++py) {
    for (int px = 0; px < 512; ++px) {
      const md::MaskDir dir = md::PixelToWorld(cfg, p, rot, px, py);
      expected[static_cast<size_t>(py) * 512u + static_cast<size_t>(px)] =
          (dir.valid && md::VisibleByRange(cfg.visible_, dir.z)) ? 1 : 0;
    }
  }
  const std::vector<uint8_t> actual = lumice::BuildVisibleMask(cfg, rot, short_pix);
  ASSERT_EQ(actual.size(), expected.size());
  EXPECT_EQ(actual, expected);
  EXPECT_GT(CountOn(actual), 0u) << "an all-zero mask makes the comparison vacuous";
}

// =================================================================================================
// ComputeOverlay
// =================================================================================================

TEST(AnnotationOverlay, DegenerateViewReturnsAnEmptyOverlay) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 0, 64);
  req.horizon = true;
  const ann::Overlay out = ann::ComputeOverlay(req);
  EXPECT_EQ(out.width, 0);
  EXPECT_TRUE(out.drawable.empty());
  EXPECT_TRUE(out.horizon.empty());
  EXPECT_TRUE(out.labels.empty());
}

TEST(AnnotationOverlay, OnlyRequestedCategoriesAreBuilt) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.horizon = true;
  const ann::Overlay out = ann::ComputeOverlay(req);
  EXPECT_FALSE(out.horizon.empty());
  EXPECT_TRUE(out.elevation.empty());
  EXPECT_TRUE(out.longitude.empty());
  EXPECT_TRUE(out.angular_dist.empty());
  EXPECT_FALSE(out.zenith.valid) << "zenith_nadir was not requested";
}

TEST(AnnotationOverlay, EveryCategoryMaskIsASubsetOfTheDrawableRegion) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.horizon = true;
  req.elevation_deg = { -30.0f, 30.0f, 60.0f };
  req.longitude_deg = { 0.0f, 90.0f, 180.0f, -90.0f };
  req.angular_dist_deg = { 22.0f, 46.0f };
  req.reference_dir[0] = 0.0f;
  req.reference_dir[1] = -1.0f;
  req.reference_dir[2] = 0.0f;
  const ann::Overlay out = ann::ComputeOverlay(req);

  ASSERT_EQ(out.drawable.size(), 128u * 64u);
  const std::vector<const std::vector<uint8_t>*> masks{ &out.horizon, &out.elevation, &out.longitude,
                                                        &out.angular_dist };
  // Non-fatal per category and per pixel, and only the FIRST stray pixel of each category is
  // reported: a category-wide leak would otherwise print one line per pixel, and a fatal assert
  // would stop at the first category and hide whether the others leak too.
  for (const auto* m : masks) {
    if (m->size() != out.drawable.size()) {
      ADD_FAILURE() << "category mask is " << m->size() << " bytes, drawable is " << out.drawable.size();
      continue;
    }
    EXPECT_GT(CountOn(*m), 0u) << "a category that draws nothing makes the subset check vacuous";
    size_t stray = 0;
    size_t first_stray = 0;
    for (size_t i = 0; i < m->size(); ++i) {
      if ((*m)[i] != 0 && out.drawable[i] == 0 && stray++ == 0) {
        first_stray = i;
      }
    }
    EXPECT_EQ(stray, 0u) << "annotation painted on " << stray << " pixel(s) outside the drawable region; first at "
                         << first_stray;
  }
}

TEST(AnnotationOverlay, MoreParallelsPaintMorePixels) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.labels = false;
  req.elevation_deg = { 30.0f };
  const size_t one = CountOn(ann::ComputeOverlay(req).elevation);
  req.elevation_deg = { 30.0f, -30.0f, 60.0f };
  const size_t three = CountOn(ann::ComputeOverlay(req).elevation);
  EXPECT_GT(one, 0u);
  EXPECT_GT(three, one);
}

TEST(AnnotationOverlay, FrontClipShrinksTheDrawableRegion) {
  ann::ViewSnapshot view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  ann::Request req;
  req.view = view;
  req.horizon = true;
  const size_t full = CountOn(ann::ComputeOverlay(req).drawable);

  req.view.front = true;
  const ann::Overlay clipped = ann::ComputeOverlay(req);
  EXPECT_GT(full, 0u);
  EXPECT_LT(CountOn(clipped.drawable), full);
  EXPECT_GT(CountOn(clipped.drawable), 0u) << "the front clip removed the whole sky";
}

TEST(AnnotationOverlay, ZenithAndNadirProjectOntoAFullSkyCanvas) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.zenith_nadir = true;
  const ann::Overlay out = ann::ComputeOverlay(req);
  EXPECT_TRUE(out.zenith.valid);
  EXPECT_TRUE(out.nadir.valid);
  // Dual fisheye puts the upper hemisphere in the left disc and the lower in the right one, so the
  // two poles land on opposite halves of the canvas — a cheap check that they are not the same
  // point.
  EXPECT_LT(out.zenith.px, 64.0f);
  EXPECT_GT(out.nadir.px, 64.0f);
}

TEST(AnnotationOverlay, ZenithIsReportedInvisibleWhenTheHemisphereExcludesIt) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.view.visible = RenderConfig::kLower;
  req.zenith_nadir = true;
  const ann::Overlay out = ann::ComputeOverlay(req);
  EXPECT_FALSE(out.zenith.valid);
  EXPECT_TRUE(out.nadir.valid);
}

TEST(AnnotationOverlay, LabelsCarryTheirCurveIdentityAndText) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.horizon = true;
  req.elevation_deg = { 30.0f, -30.0f };
  req.longitude_deg = { 0.0f, 90.0f };
  const ann::Overlay out = ann::ComputeOverlay(req);
  ASSERT_FALSE(out.labels.empty());

  bool saw_horizon = false;
  bool saw_elevation_30 = false;
  bool saw_longitude_90 = false;
  for (const ann::Label& l : out.labels) {
    EXPECT_FALSE(l.text.empty());
    EXPECT_GE(l.px, 0.0f);
    EXPECT_LT(l.px, 128.0f);
    EXPECT_GE(l.py, 0.0f);
    EXPECT_LT(l.py, 64.0f);
    if (l.kind == ann::kLabelHorizon) {
      saw_horizon = true;
      EXPECT_EQ(l.index, -1) << "the horizon comes from no list, so it carries no list index";
      EXPECT_EQ(l.text, "0\xC2\xB0");
    }
    if (l.kind == ann::kLabelElevation && l.index == 0) {
      saw_elevation_30 = true;
      EXPECT_EQ(l.text, "30\xC2\xB0");
      EXPECT_FLOAT_EQ(l.value_deg, 30.0f);
    }
    if (l.kind == ann::kLabelLongitude && l.index == 1) {
      saw_longitude_90 = true;
      EXPECT_EQ(l.text, "90\xC2\xB0");
    }
  }
  EXPECT_TRUE(saw_horizon);
  EXPECT_TRUE(saw_elevation_30);
  EXPECT_TRUE(saw_longitude_90);
}

TEST(AnnotationOverlay, FractionalAnglesKeepOneDecimal) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.elevation_deg = { 22.5f };
  const ann::Overlay out = ann::ComputeOverlay(req);
  ASSERT_FALSE(out.labels.empty());
  EXPECT_EQ(out.labels.front().text, "22.5\xC2\xB0");
}

TEST(AnnotationOverlay, LabelsCanBeSkipped) {
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 128, 64);
  req.horizon = true;
  req.elevation_deg = { 30.0f };
  req.labels = false;
  const ann::Overlay out = ann::ComputeOverlay(req);
  EXPECT_TRUE(out.labels.empty());
  EXPECT_FALSE(out.horizon.empty()) << "geometry is still built when only the anchors are skipped";
}

TEST(AnnotationOverlay, AngularDistanceCirclesAreCentredOnTheReferenceDirection) {
  // A ring at 0 deg degenerates to the reference direction itself; a ring at 22 deg must lie
  // around it. Comparing the two mask centroids is enough to show the ring is centred there and
  // not, say, on the optical axis.
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128);
  req.labels = false;
  req.reference_dir[0] = 0.0f;
  req.reference_dir[1] = -0.7071f;
  req.reference_dir[2] = -0.7071f;
  req.angular_dist_deg = { 22.0f };
  const ann::Overlay ring = ann::ComputeOverlay(req);
  ASSERT_GT(CountOn(ring.angular_dist), 0u);

  double sx = 0.0;
  double sy = 0.0;
  double count = 0.0;
  for (size_t i = 0; i < ring.angular_dist.size(); ++i) {
    if (ring.angular_dist[i] != 0) {
      sx += static_cast<double>(i % 256u);
      sy += static_cast<double>(i / 256u);
      count += 1.0;
    }
  }
  const RenderConfig cfg = ann::ToRenderConfig(req.view);
  const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
  lm_proj::ProjParams p = lumice::BuildProjParams(cfg, rot, 128.0f);
  const ann::CanvasPoint centre =
      ann::ProjectWorldDir(p, req.reference_dir[0], req.reference_dir[1], req.reference_dir[2]);
  ASSERT_TRUE(centre.valid);
  EXPECT_NEAR(sx / count, static_cast<double>(centre.px), 4.0);
  EXPECT_NEAR(sy / count, static_cast<double>(centre.py), 4.0);
}

// =============== SunWorldDir: which vector is "the sun" ===============
// The sign of this vector is the one thing in the CLI's angular-distance path that fails
// invisibly-but-totally: a circle of radius r around -v is a circle of radius 180-r around v, so
// an inverted sun direction turns the 22 deg halo into a 158 deg ring that is still a perfectly
// well-formed circle. It is pinned three ways below, against the three places that already answer
// the same question, rather than against a fourth hand-derived formula.

TEST(SunWorldDir, MatchesTheGuiFormulaAtAzimuthZero) {
  // The GUI has no sun azimuth field at all; its preview passes (-cos(alt), 0, -sin(alt)), which
  // is this function's azimuth-0 slice. That formula is what the shipped preview draws its sun
  // circles from, so agreeing with it is agreeing with a value users have already validated.
  for (float alt : { -90.0f, -30.0f, 0.0f, 12.5f, 45.0f, 90.0f }) {
    const lumice::SunParam sun{ alt, 0.0f, 0.5f };
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::SunWorldDir(sun, d);
    const float rad = alt * lumice::math::kDegreeToRad;
    EXPECT_NEAR(d[0], -std::cos(rad), 1e-5f) << "altitude=" << alt;
    EXPECT_NEAR(d[1], 0.0f, 1e-5f) << "altitude=" << alt;
    EXPECT_NEAR(d[2], -std::sin(rad), 1e-5f) << "altitude=" << alt;
  }
}

TEST(SunWorldDir, MatchesTheRayGeneratorsCapCentre) {
  // SampleRayDir (simulator.cpp) emits SampleSphCapPoint(azimuth + 180, -altitude, diameter/2).
  // With a zero-radius cap that sampler returns its centre exactly, so this compares against the
  // direction the simulator actually launches sunlight along — the transform whose "+180 and
  // negate" this function had to reproduce rather than guess at.
  const float azimuths[] = { 0.0f, 37.0f, 90.0f, 180.0f, 271.0f };
  const float altitudes[] = { -20.0f, 0.0f, 23.5f, 60.0f };
  for (float az : azimuths) {
    for (float alt : altitudes) {
      float cap_centre[3] = { 0.0f, 0.0f, 0.0f };
      lumice::SampleSphCapPoint(az + 180.0f, -alt, 0.0f, cap_centre);
      const lumice::SunParam sun{ alt, az, 0.5f };
      float d[3] = { 0.0f, 0.0f, 0.0f };
      ann::SunWorldDir(sun, d);
      for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(d[k], cap_centre[k], 1e-5f) << "az=" << az << " alt=" << alt << " k=" << k;
      }
    }
  }
}

TEST(SunWorldDir, InvertsThePreviewShadersAngleRecovery) {
  // The preview fragment shader reads a world direction back as altitude = asin(-z) and
  // azimuth = atan2(-y, -x). Round-tripping through it is what makes the vector the SAME
  // (altitude, azimuth) the rest of the GUI means, not merely a unit vector of the right length.
  const float azimuths[] = { 0.0f, 37.0f, 90.0f, 180.0f, 271.0f };
  const float altitudes[] = { -20.0f, 0.0f, 23.5f, 60.0f };
  for (float az : azimuths) {
    for (float alt : altitudes) {
      const lumice::SunParam sun{ alt, az, 0.5f };
      float d[3] = { 0.0f, 0.0f, 0.0f };
      ann::SunWorldDir(sun, d);

      EXPECT_NEAR(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), 1.0f, 1e-5f);
      const float alt_back = std::asin(std::clamp(-d[2], -1.0f, 1.0f)) / lumice::math::kDegreeToRad;
      EXPECT_NEAR(alt_back, alt, 1e-3f) << "az=" << az << " alt=" << alt;
      // atan2 returns (-180, 180]; compare on the circle so 271 and -89 agree.
      float az_back = std::atan2(-d[1], -d[0]) / lumice::math::kDegreeToRad;
      float delta = std::fmod(az_back - az + 540.0f, 360.0f) - 180.0f;
      EXPECT_NEAR(delta, 0.0f, 1e-3f) << "az=" << az << " alt=" << alt;
    }
  }
}

TEST(SunWorldDir, PutsTheCircleCentreWhereTheSunIsImaged) {
  // End to end: feed the vector to ComputeOverlay as reference_dir and check the ring it produces
  // is centred on the pixel SUNLIGHT lands at.
  //
  // The expected centre is projected from the ray generator's own cap centre, NOT from `d`. Using
  // `d` on both sides was the first shape of this test and it is a tautology: reference_dir and
  // the expected pixel would move together under any sign convention, so a fully inverted
  // SunWorldDir passed it while the other three cases in this suite went red. The independent
  // source is what makes this case add anything.
  const lumice::SunParam sun{ 25.0f, 40.0f, 0.5f };
  float d[3] = { 0.0f, 0.0f, 0.0f };
  ann::SunWorldDir(sun, d);
  float sun_ray[3] = { 0.0f, 0.0f, 0.0f };
  lumice::SampleSphCapPoint(sun.azimuth_ + 180.0f, -sun.altitude_, 0.0f, sun_ray);

  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128);
  req.view.visible = RenderConfig::kFull;
  req.labels = false;
  req.angular_dist_deg = { 22.0f };
  std::copy(d, d + 3, req.reference_dir);
  const ann::Overlay ring = ann::ComputeOverlay(req);
  ASSERT_GT(CountOn(ring.angular_dist), 0u);

  double sx = 0.0;
  double sy = 0.0;
  double count = 0.0;
  for (size_t i = 0; i < ring.angular_dist.size(); ++i) {
    if (ring.angular_dist[i] != 0) {
      sx += static_cast<double>(i % 256u);
      sy += static_cast<double>(i / 256u);
      count += 1.0;
    }
  }

  const RenderConfig cfg = ann::ToRenderConfig(req.view);
  const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
  lm_proj::ProjParams p = lumice::BuildProjParams(cfg, rot, 128.0f);
  const ann::CanvasPoint centre = ann::ProjectWorldDir(p, sun_ray[0], sun_ray[1], sun_ray[2]);
  ASSERT_TRUE(centre.valid);
  EXPECT_NEAR(sx / count, static_cast<double>(centre.px), 4.0);
  EXPECT_NEAR(sy / count, static_cast<double>(centre.py), 4.0);
}

TEST(AnnotationOverlay, AngularDistLabelsSitOnTheCircleTheyName) {
  // Every angular-distance anchor must lie on its own ring: the direction the anchor's pixel
  // images is `value_deg` away from the reference direction.
  //
  // Asserted against the geometry, not against a second implementation. It replaces a parity case
  // that compared these anchors with the GUI's own ring walk, which no longer exists now that the
  // GUI consumes these anchors instead of deriving them (see the note at that test's former home,
  // test/unit-correctness/gui/test_annotation_overlay_gui_parity.cpp). Agreement with a walk would
  // in any case have been the weaker claim — two implementations can be wrong the same way, and a
  // ring at the wrong radius is a picture that looks entirely plausible.
  ann::Request req;
  req.view = MakeView(LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128);
  req.view.visible = RenderConfig::kFull;
  req.angular_dist_deg = { 22.0f, 46.0f };
  // Sun on the horizon to the north-east, so both rings straddle the disc seam and the boundary
  // branch is exercised alongside the interior one.
  const float sun[3] = { -0.7071f, -0.7071f, 0.0f };
  std::copy(sun, sun + 3, req.reference_dir);

  const ann::Overlay out = ann::ComputeOverlay(req);
  const RenderConfig cfg = ann::ToRenderConfig(req.view);
  const lumice::Rotation rot = lumice::MakeCameraRotation(cfg);
  const float short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  const lm_proj::ProjParams p = lumice::BuildProjParams(cfg, rot, short_pix);

  int checked = 0;
  int saw_22 = 0;
  int saw_46 = 0;
  for (const ann::Label& l : out.labels) {
    if (l.kind != ann::kLabelAngularDist) {
      continue;
    }
    // Non-fatal, and the index checked before it is used: one bad label must not stop the loop
    // reporting the rest, and an out-of-range index would read past the request list below.
    if (l.index < 0 || l.index >= 2) {
      ADD_FAILURE() << "a circle label carries index " << l.index << ", outside the 2-entry request list";
      continue;
    }
    EXPECT_FLOAT_EQ(l.value_deg, req.angular_dist_deg[static_cast<size_t>(l.index)])
        << "the label's value and the entry it indexes disagree";
    l.value_deg > 30.0f ? ++saw_46 : ++saw_22;

    // Invert the anchor pixel back to a world direction and measure its angle to the sun. The
    // inverse is core's own (the same one the mask is built from), so what this checks is that the
    // FORWARD the anchor came out of and that inverse describe the same curve.
    const lumice::mask_detail::MaskDir dir =
        lumice::mask_detail::PixelToWorld(cfg, p, rot, static_cast<int>(l.px), static_cast<int>(l.py));
    if (!dir.valid) {
      continue;  // an anchor half a degree past the rim (kLabelHemisphereToleranceDeg) images nothing
    }
    ++checked;
    const float dot = std::clamp(dir.x * sun[0] + dir.y * sun[1] + dir.z * sun[2], -1.0f, 1.0f);
    const float deg = std::acos(dot) / lumice::math::kDegreeToRad;
    // One curve sample is 1 degree and the anchor is placed AT a sample, so a whole degree of
    // slack is the sampling itself; the pixel quantisation of px/py adds a fraction more.
    EXPECT_NEAR(deg, l.value_deg, 2.0f) << "an anchor labelled " << l.value_deg << " deg sits " << deg
                                        << " deg from the reference direction";
  }
  EXPECT_GT(checked, 0) << "no circle anchor was imageable — the assertion above never ran";
  EXPECT_GT(saw_22, 0) << "the 22 deg ring produced no label";
  EXPECT_GT(saw_46, 0) << "the 46 deg ring produced no label";
}

// =============== The named direction table ===============
// Each id is asserted against the GEOMETRIC RELATION that defines it, not against the assignment
// that implements it. "subsun.z == -sun.z" would be a transcription of ResolveMarkerDir's body and
// would pass just as happily if both were wrong; "the subsun is the sun mirrored in the horizontal
// plane, so it has the sun's azimuth and the negated altitude" is a statement about the sky that
// the code has to satisfy. Where a relation is expressible as a dot product it is written that
// way, because a dot product cannot be satisfied by a sign error in one component.
//
// The sun directions swept below come from SunWorldDir rather than from hand-written vectors, so
// these cases are anchored to the same convention the rest of the module uses (altitude = asin(-z))
// instead of re-deriving it — and if that convention ever changed, these would move with it.

namespace {

// Every sun direction the table is exercised at: a spread of azimuths crossed with a spread of
// altitudes, above and below the horizon. Poles are excluded here on purpose — they are where the
// AZIMUTH degenerates, which is SunHorizonDir's problem and is swept in its own cases below.
std::vector<std::array<float, 3>> SunDirSweep() {
  std::vector<std::array<float, 3>> out;
  for (float az : { 0.0f, 37.0f, 90.0f, 180.0f, 271.0f }) {
    for (float alt : { -62.0f, -15.0f, 0.0f, 23.5f, 71.0f }) {
      const lumice::SunParam sun{ alt, az, 0.5f };
      std::array<float, 3> d{ 0.0f, 0.0f, 0.0f };
      ann::SunWorldDir(sun, d.data());
      out.push_back(d);
    }
  }
  return out;
}

float Dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float Length(const float v[3]) {
  return std::sqrt(Dot(v, v));
}

// Altitude and azimuth in this module's convention, so the relations below can be stated in the
// vocabulary they are actually defined in.
float AltOf(const float v[3]) {
  return std::asin(std::clamp(-v[2], -1.0f, 1.0f)) / lumice::math::kDegreeToRad;
}

float AzOf(const float v[3]) {
  return std::atan2(-v[1], -v[0]) / lumice::math::kDegreeToRad;
}

// Signed difference between two azimuths, on the circle: 271 and -89 are the same bearing.
float AzDelta(float a, float b) {
  return std::fmod(a - b + 540.0f, 360.0f) - 180.0f;
}

}  // namespace

TEST(MarkerDirectionTable, EveryIdResolvesToAUnitVector) {
  // The four sun-relative ids are reflections of a unit vector and the two poles are constants, so
  // unit length is not automatic only in the sense that a typo would break it — which is exactly
  // what this guards. A non-unit direction would still project to a plausible pixel, because the
  // lens forward normalizes internally, so nothing downstream would report it.
  const int ids[] = { ann::kMarkerZenith, ann::kMarkerNadir,     ann::kMarkerSun,
                      ann::kMarkerSubsun, ann::kMarkerAnthelion, ann::kMarkerAntisolar };
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    for (int id : ids) {
      float d[3] = { 0.0f, 0.0f, 0.0f };
      ann::ResolveMarkerDir(static_cast<ann::MarkerId>(id), sun.data(), d);
      EXPECT_NEAR(Length(d), 1.0f, 1e-5f) << "id=" << id;
      for (int k = 0; k < 3; ++k) {
        EXPECT_TRUE(std::isfinite(d[k])) << "id=" << id << " k=" << k;
      }
    }
  }
}

TEST(MarkerDirectionTable, ZenithAndNadirAreTheAntipodalPolesAndIgnoreTheSun) {
  // Altitude +90 and -90 in this module's convention, which puts the zenith at z = -1. The
  // altitude assertion is the one that catches the sign confusion with the crystal-orientation
  // convention (where +z is up); a bare "they are antipodal" would pass with both flipped.
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    float z[3] = { 0.0f, 0.0f, 0.0f };
    float n[3] = { 0.0f, 0.0f, 0.0f };
    ann::ResolveMarkerDir(ann::kMarkerZenith, sun.data(), z);
    ann::ResolveMarkerDir(ann::kMarkerNadir, sun.data(), n);
    EXPECT_NEAR(AltOf(z), 90.0f, 1e-4f);
    EXPECT_NEAR(AltOf(n), -90.0f, 1e-4f);
    EXPECT_NEAR(Dot(z, n), -1.0f, 1e-6f);
    // Independent of the sun: same answer for every direction in the sweep.
    EXPECT_EQ(z[0], 0.0f);
    EXPECT_EQ(z[1], 0.0f);
    EXPECT_LT(z[2], 0.0f) << "the zenith must be z < 0 in this module's convention";
    EXPECT_GT(n[2], 0.0f);
  }
}

TEST(MarkerDirectionTable, SunResolvesToTheReferenceDirectionItself) {
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::ResolveMarkerDir(ann::kMarkerSun, sun.data(), d);
    EXPECT_NEAR(Dot(d, sun.data()), 1.0f, 1e-6f) << "the sun marker is not the sun direction";
  }
}

TEST(MarkerDirectionTable, SubsunIsTheSunMirroredInTheHorizontalPlane) {
  // Definition: the sun's reflection in a horizontal surface — same bearing, altitude negated.
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::ResolveMarkerDir(ann::kMarkerSubsun, sun.data(), d);
    EXPECT_NEAR(AltOf(d), -AltOf(sun.data()), 1e-3f);
    if (std::abs(AltOf(sun.data())) < 89.0f) {  // azimuth is defined away from the poles
      EXPECT_NEAR(AzDelta(AzOf(d), AzOf(sun.data())), 0.0f, 1e-3f) << "the subsun changed bearing";
    }
  }
}

TEST(MarkerDirectionTable, AnthelionIsOppositeInBearingAtTheSameAltitude) {
  // Definition: 180 degrees around from the sun, at the sun's own altitude. This is the id most
  // easily confused with the antisolar point, and the altitude clause is what separates them.
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::ResolveMarkerDir(ann::kMarkerAnthelion, sun.data(), d);
    EXPECT_NEAR(AltOf(d), AltOf(sun.data()), 1e-3f);
    if (std::abs(AltOf(sun.data())) < 89.0f) {
      EXPECT_NEAR(std::abs(AzDelta(AzOf(d), AzOf(sun.data()))), 180.0f, 1e-3f);
    }
  }
}

TEST(MarkerDirectionTable, AntisolarIsExactlyOppositeTheSun) {
  // Definition: the antipode. Stated as a dot product rather than component-wise so that no single
  // sign error can satisfy it.
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::ResolveMarkerDir(ann::kMarkerAntisolar, sun.data(), d);
    EXPECT_NEAR(Dot(d, sun.data()), -1.0f, 1e-6f);
  }
}

TEST(MarkerDirectionTable, TheFourSunRelativeIdsAreDistinctForAGenericSun) {
  // Without this, every relation above could be satisfied by a table that collapsed several ids
  // onto one direction (a sun at altitude 0 really does merge the sun with its subsun, which is
  // why the sun here is deliberately generic rather than swept).
  const lumice::SunParam sun_param{ 34.0f, 61.0f, 0.5f };
  float sun[3] = { 0.0f, 0.0f, 0.0f };
  ann::SunWorldDir(sun_param, sun);

  const int ids[] = { ann::kMarkerSun, ann::kMarkerSubsun, ann::kMarkerAnthelion, ann::kMarkerAntisolar };
  std::vector<std::array<float, 3>> dirs;
  for (int id : ids) {
    std::array<float, 3> d{ 0.0f, 0.0f, 0.0f };
    ann::ResolveMarkerDir(static_cast<ann::MarkerId>(id), sun, d.data());
    dirs.push_back(d);
  }
  for (size_t i = 0; i < dirs.size(); ++i) {
    for (size_t j = i + 1; j < dirs.size(); ++j) {
      EXPECT_LT(Dot(dirs[i].data(), dirs[j].data()), 0.999f) << "ids " << ids[i] << " and " << ids[j] << " coincide";
    }
  }
}

TEST(MarkerDirectionTable, AnOutOfRangeIdYieldsTheZenithRatherThanGarbage) {
  // ResolveMarkerDir's callers project whatever it writes, so the contract is that `out` is always
  // a usable unit vector. The C API rejects a bad id before it gets here; a C++ caller inside core
  // is not required to, which is why the default branch exists at all.
  float sun[3] = { 0.0f, 0.0f, -1.0f };
  for (int bad : { -1, static_cast<int>(ann::kMarkerCount), 999 }) {
    float d[3] = { 7.0f, 7.0f, 7.0f };
    ann::ResolveMarkerDir(static_cast<ann::MarkerId>(bad), sun, d);
    EXPECT_NEAR(Length(d), 1.0f, 1e-5f) << "id=" << bad;
    EXPECT_NEAR(AltOf(d), 90.0f, 1e-4f) << "id=" << bad;
  }
}

// =============== SunHorizonDir: the view-preset direction, and its pole ===============

TEST(SunHorizonDir, KeepsTheSunsBearingAndDropsItsAltitude) {
  // The defining relation: same bearing, altitude 0. Asserted through the azimuth recovery rather
  // than against normalize(x, y, 0), which would just restate the implementation.
  for (const std::array<float, 3>& sun : SunDirSweep()) {
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::SunHorizonDir(sun.data(), d);
    EXPECT_NEAR(Length(d), 1.0f, 1e-5f);
    EXPECT_EQ(d[2], 0.0f) << "the horizon direction must have zero altitude exactly";
    EXPECT_NEAR(AzDelta(AzOf(d), AzOf(sun.data())), 0.0f, 1e-3f);
  }
}

TEST(SunHorizonDir, StaysAUnitVectorThroughTheDegenerateBand) {
  // The sweep crosses the fallback threshold: 89.9 and 89.99 deg are above it (the horizontal
  // component is ~1.7e-3 and ~1.7e-4, both well over kSunHorizonDegenerateEps), while exactly
  // +/-90 is below it — the horizontal component there is the float residue of cos(pi/2), 4.4e-8.
  //
  // The requirement is NOT continuity across that boundary. Azimuth is undefined at a pole, so no
  // choice of fallback can be continuous there; a nearest-neighbour fallback would be worse, since
  // the residue's sign makes the recovered bearing at exactly 90 deg point 180 deg away from the
  // bearing at 89.999 deg. What IS required, and is what this asserts, is that the answer is
  // always a finite unit vector in the horizon plane.
  for (float az : { 0.0f, 37.0f, 214.0f }) {
    for (float alt : { 90.0f, 89.999f, 89.99f, 89.9f, -89.9f, -89.99f, -89.999f, -90.0f }) {
      const lumice::SunParam sun_param{ alt, az, 0.5f };
      float sun[3] = { 0.0f, 0.0f, 0.0f };
      ann::SunWorldDir(sun_param, sun);
      float d[3] = { 0.0f, 0.0f, 0.0f };
      ann::SunHorizonDir(sun, d);
      for (int k = 0; k < 3; ++k) {
        EXPECT_TRUE(std::isfinite(d[k])) << "az=" << az << " alt=" << alt << " k=" << k;
      }
      EXPECT_NEAR(Length(d), 1.0f, 1e-5f) << "az=" << az << " alt=" << alt;
      EXPECT_EQ(d[2], 0.0f) << "az=" << az << " alt=" << alt;
    }
  }
}

TEST(SunHorizonDir, ExactPolesFallBackToTheDocumentedFixedBearing) {
  // A sun handed in as an exact pole vector has a horizontal component of exactly zero, so this is
  // the true division-by-zero input rather than the near-miss the sweep above produces. Both poles
  // must give the SAME fixed answer: the fallback is a constant, not a function of which pole.
  const float poles[2][3] = { { 0.0f, 0.0f, -1.0f }, { 0.0f, 0.0f, 1.0f } };
  for (const auto& p : poles) {
    float d[3] = { 0.0f, 0.0f, 0.0f };
    ann::SunHorizonDir(p, d);
    EXPECT_EQ(d[0], 1.0f);
    EXPECT_EQ(d[1], 0.0f);
    EXPECT_EQ(d[2], 0.0f);
  }
}

TEST(SunHorizonDir, TheThresholdIsWellClearOfTheAltitudesItMustNotSwallow) {
  // Guards the threshold VALUE, not just the branch: a kSunHorizonDegenerateEps raised to, say,
  // 1e-3 would silently start returning the fixed bearing for a sun at 89.9 deg — a real altitude
  // whose azimuth is perfectly well defined — and every other case in this file would still pass.
  const lumice::SunParam sun_param{ 89.9f, 214.0f, 0.5f };
  float sun[3] = { 0.0f, 0.0f, 0.0f };
  ann::SunWorldDir(sun_param, sun);
  const float horizontal = std::sqrt(sun[0] * sun[0] + sun[1] * sun[1]);
  EXPECT_GT(horizontal, ann::kSunHorizonDegenerateEps * 100.0f)
      << "the fallback threshold has grown into altitudes that carry a real bearing";

  float d[3] = { 0.0f, 0.0f, 0.0f };
  ann::SunHorizonDir(sun, d);
  EXPECT_NEAR(AzDelta(AzOf(d), AzOf(sun)), 0.0f, 1e-3f) << "89.9 deg was treated as degenerate";
}
