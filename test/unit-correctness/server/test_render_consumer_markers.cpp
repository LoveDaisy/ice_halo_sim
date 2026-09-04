// RenderConsumer's side of the reference-point markers (grid.markers): that N named directions
// each get a ring in their own colour at one shared radius and opacity, that a non-empty list wins
// over the legacy grid.zenith_nadir block, and that the four sun-relative ids follow the sun when
// ResetWith moves it.
//
// Sibling of test_render_consumer_zenith_nadir.cpp, which stays exactly as it was: this file adds
// propositions about the generalized list, and that file's continuing to pass unedited is what says
// the legacy pair still renders what it always did. Two of the cases here are about the seam
// between them rather than about either alone:
//
//   - LegacyPairAndTheEquivalentListPaintIdenticalBytes is the equivalence claim stated as a byte
//     comparison instead of as an argument about which floating-point operations run in which
//     order. It is the one assertion in the tree that would fail if the list path drew the same
//     rings by a different route (a second clamp, a different blend order, a colour converted at a
//     different time);
//   - SunRelativeMarkersFollowTheSunThroughResetWith is about a lifetime that did not exist before.
//     The zenith/nadir pair was computed once at construction and could be: two poles are pure
//     geometry, and every layout field they depend on is pinned for the consumer's life. Four of
//     the six ids here are reflections of the SUN, which is precisely what ResetWith changes.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/light_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"
#include "support/render_anchor.hpp"

namespace lumice {
namespace {

constexpr int kW = 128;
constexpr int kH = 64;
constexpr int kTotalPix = kW * kH;

// Off the horizon and off the meridian, so the four sun-relative directions are four distinct
// places rather than a degenerate pile. A sun ON the horizon puts the sun and the subsun at the
// same point, and the anthelion and the antisolar point too.
SunParam MakeSun() {
  return SunParam{ 35.0f, 20.0f, 0.5f };
}

// A second sun far from the first, for the ResetWith case.
SunParam MakeOtherSun() {
  return SunParam{ -10.0f, 140.0f, 0.5f };
}

// The half-width the renderer draws the ring at (server/render.cpp kMarkerHalfWidthPx). Restated
// rather than shared, for the same reason the zenith/nadir file restates it: these assertions are
// about the ring a viewer sees, and a change to the renderer's thickness should have to be
// acknowledged here.
constexpr float kRingHalfWidthPx = 1.5f;

// A dual equal-area fisheye over the full sphere. The only lens family that images the WHOLE sky at
// once, which is what a six-marker fixture needs: the single-lens family culls everything past
// theta = 90 deg from the view axis whatever the FOV says, so at least one of the six would always
// be a miss and the case would silently test five. Background stays black so a tinted pixel is
// unambiguous.
RenderConfig MakeFullSkyConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kDualFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.visible_ = RenderConfig::kFull;
  return cfg;
}

MarkerStyleParam Marker(MarkerRefId id, float r, float g, float b) {
  MarkerStyleParam m;
  m.id_ = id;
  m.enabled_ = true;
  m.color_[0] = r;
  m.color_[1] = g;
  m.color_[2] = b;
  return m;
}

// One ray straight up: only rays that LAND reach total_intensity_, and PostSnapshot takes a
// zero-intensity early-out that would make every assertion below vacuous.
SimData MakeOneRayBatch() {
  SimData data;
  data.curr_wl_ = 550.0f;
  data.outgoing_d_ = { 0.0f, 0.0f, -1.0f };
  data.outgoing_w_ = { 1.0f };
  return data;
}

std::vector<uint8_t> SnapshotOnce(RenderConsumer* rc) {
  auto data = MakeOneRayBatch();
  rc->Consume(data);
  lumice::test::TakeSnapshotAtFormerSelfAnchor(rc);
  auto result = rc->GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  if (rr == nullptr || rr->img_buffer_ == nullptr) {
    return {};
  }
  return std::vector<uint8_t>(rr->img_buffer_, rr->img_buffer_ + static_cast<size_t>(kTotalPix) * 3);
}

bool IsBlack(const std::vector<uint8_t>& img, int i) {
  return img[static_cast<size_t>(i) * 3] == 0 && img[static_cast<size_t>(i) * 3 + 1] == 0 &&
         img[static_cast<size_t>(i) * 3 + 2] == 0;
}

// Which of the three channels dominates a painted pixel. "Unmistakably this hue", not an exact
// triple: the blend runs in linear RGB through the sRGB transfer curve, so pinning the byte would
// pin the whole colour pipeline in a file about geometry and arbitration. Returns -1 for a pixel
// that is not clearly one of the three primaries.
int DominantChannel(const std::vector<uint8_t>& img, int i) {
  const int c[3] = { img[static_cast<size_t>(i) * 3], img[static_cast<size_t>(i) * 3 + 1],
                     img[static_cast<size_t>(i) * 3 + 2] };
  for (int j = 0; j < 3; ++j) {
    const int a = c[(j + 1) % 3];
    const int b = c[(j + 2) % 3];
    if (c[j] >= 120 && c[j] > a + 60 && c[j] > b + 60) {
      return j;
    }
  }
  return -1;
}

float DistanceTo(const annotation::CanvasPoint& p, int i) {
  const auto px = static_cast<float>(i % kW);
  const auto py = static_cast<float>(i / kW);
  return std::hypot(px - p.px, py - p.py);
}

// Every pixel the annotation painted over what was black background before it.
std::vector<int> PaintedPixels(const std::vector<uint8_t>& img_off, const std::vector<uint8_t>& img_on) {
  std::vector<int> out;
  for (int i = 0; i < kTotalPix; ++i) {
    if (IsBlack(img_off, i) && !IsBlack(img_on, i)) {
      out.push_back(i);
    }
  }
  return out;
}


TEST(RenderConsumerMarkers, EmptyListByDefaultChangesNoPixel) {
  // Opt-in, and off must be byte-identical to a build that had never heard of the field. The same
  // property test_render_consumer_post_snapshot_fusion.cpp holds the whole loop to.
  RenderConsumer a(MakeFullSkyConfig(), ColorClassTable{}, MakeSun());
  RenderConsumer b(MakeFullSkyConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_a = SnapshotOnce(&a);
  const std::vector<uint8_t> img_b = SnapshotOnce(&b);
  ASSERT_EQ(img_a.size(), static_cast<size_t>(kTotalPix) * 3);
  EXPECT_EQ(img_a, img_b);
  EXPECT_TRUE(MakeFullSkyConfig().markers_.empty()) << "the field must be opt-in";
}

TEST(RenderConsumerMarkers, SixMarkersEachOwnColourAtOneSharedRadiusAndOpacity) {
  RenderConsumer off(MakeFullSkyConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);

  constexpr float kRadius = 9.0f;
  RenderConfig cfg = MakeFullSkyConfig();
  cfg.markers_radius_px_ = kRadius;
  cfg.markers_opacity_ = 1.0f;
  // Three hues over six ids on purpose: the point of this case is that colour is PER ENTRY, and
  // three distinguishable primaries are enough to show two entries can differ while the radius and
  // the opacity cannot. Pairs are chosen so the two members of a hue are never adjacent on the sky.
  cfg.markers_.push_back(Marker(MarkerRefId::kZenith, 1.0f, 0.0f, 0.0f));
  cfg.markers_.push_back(Marker(MarkerRefId::kNadir, 0.0f, 1.0f, 0.0f));
  cfg.markers_.push_back(Marker(MarkerRefId::kSun, 0.0f, 0.0f, 1.0f));
  cfg.markers_.push_back(Marker(MarkerRefId::kSubsun, 1.0f, 0.0f, 0.0f));
  cfg.markers_.push_back(Marker(MarkerRefId::kAnthelion, 0.0f, 1.0f, 0.0f));
  cfg.markers_.push_back(Marker(MarkerRefId::kAntisolar, 0.0f, 0.0f, 1.0f));

  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  const auto& pts = on.MarkerPointsForTest();
  // Every row reported before the fixture check bites: a fatal assert inside the loop would name
  // the first missing marker and hide the rest, which is the difference between "the sun is off
  // canvas" and "this whole fixture images nothing".
  size_t imaged = 0;
  for (size_t k = 0; k < pts.size(); ++k) {
    if (pts[k].valid) {
      imaged++;
    } else {
      ADD_FAILURE() << "marker " << k << " is not imaged by this fixture, so it tests nothing";
    }
  }
  ASSERT_EQ(imaged, pts.size()) << "the fixture must image all six markers";

  const std::vector<int> painted = PaintedPixels(img_off, img_on);
  ASSERT_FALSE(painted.empty()) << "the markers must reach the image";

  // Every painted pixel sits one shared radius from SOME marker point, and carries the colour of a
  // marker it is at that radius from. The two halves together are what "per-entry colour, shared
  // radius" means: the first would pass if every ring were drawn at its own radius as long as some
  // point matched, the second if every ring were the same colour.
  size_t per_marker[annotation::kMarkerCount] = {};
  for (int i : painted) {
    bool matched = false;
    for (size_t k = 0; k < pts.size(); ++k) {
      if (std::fabs(DistanceTo(pts[k], i) - kRadius) >= kRingHalfWidthPx) {
        continue;
      }
      const int want =
          static_cast<int>(cfg.markers_[k].color_[1] > 0.5f ? 1 : (cfg.markers_[k].color_[2] > 0.5f ? 2 : 0));
      if (DominantChannel(img_on, i) == want) {
        matched = true;
        per_marker[k]++;
      }
    }
    EXPECT_TRUE(matched) << "pixel " << i << " is painted but is not its own marker's colour at the shared radius";
  }
  for (size_t k = 0; k < pts.size(); ++k) {
    EXPECT_GT(per_marker[k], 20u) << "marker " << k << " must draw a ring, not a handful of pixels";
  }
}

TEST(RenderConsumerMarkers, LegacyPairAndTheEquivalentListPaintIdenticalBytes) {
  // The equivalence claim, as a byte comparison rather than as an argument. A markers_ list naming
  // zenith and nadir with one colour, the family radius and the family opacity must produce the
  // SAME image as the legacy zenith_nadir block carrying those same three values — not a similar
  // one. This is the assertion that fails if the two paths ever stop being one blend loop over a
  // list: a second clamp, a colour converted at a different point, the two rings composited in the
  // other order.
  // Neither value may coincide with the OTHER family's default (radius 8, opacity 0.6): if it did,
  // a branch that read config_.markers_opacity_ where it meant config_.zenith_nadir_.opacity_ would
  // produce the same bytes anyway and this case would pass while cross-reading. Measured — that
  // exact mistake was injected on purpose here, and with kOpacity at 0.6 it did not show.
  constexpr float kRadius = 11.0f;
  constexpr float kOpacity = 0.45f;
  constexpr float kColor[3] = { 0.8f, 0.2f, 0.2f };

  RenderConfig legacy = MakeFullSkyConfig();
  legacy.zenith_nadir_.enabled_ = true;
  legacy.zenith_nadir_.radius_px_ = kRadius;
  legacy.zenith_nadir_.opacity_ = kOpacity;
  std::copy(std::begin(kColor), std::end(kColor), std::begin(legacy.zenith_nadir_.color_));

  ASSERT_NE(kOpacity, RenderConfig{}.markers_opacity_);
  ASSERT_NE(kRadius, RenderConfig{}.markers_radius_px_);
  ASSERT_NE(kOpacity, ZenithNadirParam{}.opacity_);
  ASSERT_NE(kRadius, ZenithNadirParam{}.radius_px_);

  RenderConfig listed = MakeFullSkyConfig();
  listed.markers_radius_px_ = kRadius;
  listed.markers_opacity_ = kOpacity;
  listed.markers_.push_back(Marker(MarkerRefId::kZenith, kColor[0], kColor[1], kColor[2]));
  listed.markers_.push_back(Marker(MarkerRefId::kNadir, kColor[0], kColor[1], kColor[2]));

  RenderConsumer a(legacy, ColorClassTable{}, MakeSun());
  RenderConsumer b(listed, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_a = SnapshotOnce(&a);
  const std::vector<uint8_t> img_b = SnapshotOnce(&b);
  ASSERT_EQ(img_a.size(), static_cast<size_t>(kTotalPix) * 3);
  // The fixture has to actually draw something, or "identical" is a statement about two blank
  // frames.
  RenderConsumer none(MakeFullSkyConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_none = SnapshotOnce(&none);
  ASSERT_NE(img_a, img_none) << "the legacy fixture must paint rings, or this case compares nothing";

  EXPECT_EQ(img_a, img_b);
}

TEST(RenderConsumerMarkers, NonEmptyListWinsOverTheLegacyPair) {
  // The arbitration rule, and the reason it is a rule rather than a merge: a config carrying both
  // gets the list ALONE. The legacy block here asks for red at a different radius, so "merged"
  // and "list wins" are distinguishable by colour AND by geometry.
  RenderConsumer off(MakeFullSkyConfig(), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);

  constexpr float kListRadius = 9.0f;
  RenderConfig cfg = MakeFullSkyConfig();
  cfg.zenith_nadir_.enabled_ = true;
  cfg.zenith_nadir_.radius_px_ = 20.0f;
  cfg.zenith_nadir_.opacity_ = 1.0f;
  cfg.zenith_nadir_.color_[0] = 1.0f;
  cfg.zenith_nadir_.color_[1] = 0.0f;
  cfg.zenith_nadir_.color_[2] = 0.0f;
  cfg.markers_radius_px_ = kListRadius;
  cfg.markers_opacity_ = 1.0f;
  cfg.markers_.push_back(Marker(MarkerRefId::kZenith, 0.0f, 0.0f, 1.0f));

  RenderConsumer on(cfg, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  const auto& pts = on.MarkerPointsForTest();
  ASSERT_TRUE(pts[annotation::kMarkerZenith].valid);
  ASSERT_TRUE(pts[annotation::kMarkerNadir].valid) << "the nadir must be imaged, or 'no nadir ring' is vacuous";

  const std::vector<int> painted = PaintedPixels(img_off, img_on);
  ASSERT_FALSE(painted.empty());

  size_t on_list_ring = 0;
  for (int i : painted) {
    EXPECT_NE(DominantChannel(img_on, i), 0) << "pixel " << i << " is red: the ignored legacy colour reached the image";
    // Nothing at the legacy radius, and nothing at the nadir at all — the legacy block asked for
    // both and got neither.
    EXPECT_GE(DistanceTo(pts[annotation::kMarkerNadir], i), 20.0f - kRingHalfWidthPx)
        << "pixel " << i << " sits on a nadir ring the list never asked for";
    if (std::fabs(DistanceTo(pts[annotation::kMarkerZenith], i) - kListRadius) < kRingHalfWidthPx) {
      on_list_ring++;
    }
  }
  EXPECT_EQ(on_list_ring, painted.size()) << "every painted pixel must belong to the list's own zenith ring";
  EXPECT_GT(on_list_ring, 20u);
}

TEST(RenderConsumerMarkers, EmptyListFallsBackToTheLegacyPair) {
  // The other side of the same rule, and the case that pins the known boundary: emptiness is the
  // ONLY absence signal, so an explicit empty list is indistinguishable from no list at all and
  // the legacy block still draws. Stated as a test rather than only as a comment because it is a
  // behaviour a reader could reasonably expect to go the other way.
  RenderConfig with_legacy = MakeFullSkyConfig();
  with_legacy.zenith_nadir_.enabled_ = true;
  with_legacy.zenith_nadir_.radius_px_ = 11.0f;
  with_legacy.zenith_nadir_.opacity_ = 1.0f;
  with_legacy.zenith_nadir_.color_[0] = 1.0f;
  with_legacy.zenith_nadir_.color_[1] = 0.0f;
  with_legacy.zenith_nadir_.color_[2] = 0.0f;

  RenderConfig also_empty_list = with_legacy;
  also_empty_list.markers_.clear();

  RenderConsumer a(with_legacy, ColorClassTable{}, MakeSun());
  RenderConsumer b(also_empty_list, ColorClassTable{}, MakeSun());
  EXPECT_EQ(SnapshotOnce(&a), SnapshotOnce(&b));
}

TEST(RenderConsumerMarkers, DisabledEntryDrawsNothingForThatId) {
  // `enabled_` is per entry: a listed-but-off marker must be as invisible as an unlisted one, while
  // the list stays non-empty and therefore still wins over the legacy block.
  RenderConfig one_on = MakeFullSkyConfig();
  one_on.markers_opacity_ = 1.0f;
  one_on.markers_radius_px_ = 10.0f;
  one_on.markers_.push_back(Marker(MarkerRefId::kZenith, 1.0f, 0.0f, 0.0f));

  RenderConfig plus_one_off = one_on;
  MarkerStyleParam disabled = Marker(MarkerRefId::kNadir, 0.0f, 1.0f, 0.0f);
  disabled.enabled_ = false;
  plus_one_off.markers_.push_back(disabled);

  RenderConsumer a(one_on, ColorClassTable{}, MakeSun());
  RenderConsumer b(plus_one_off, ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_a = SnapshotOnce(&a);
  ASSERT_EQ(img_a.size(), static_cast<size_t>(kTotalPix) * 3);
  EXPECT_EQ(img_a, SnapshotOnce(&b));

  // And the nadir really is on this canvas, so "drew nothing" is about the switch and not about a
  // direction that missed the frame.
  EXPECT_TRUE(b.MarkerPointsForTest()[annotation::kMarkerNadir].valid);
}

TEST(RenderConsumerMarkers, SunRelativeMarkersFollowTheSunThroughResetWith) {
  // The lifetime this change widened. The two poles are pure geometry and every layout field they
  // depend on is pinned for the consumer's life by NeedsRebuild — which is why the pair this
  // generalizes was computed once, in the constructor. The other four ids are reflections of the
  // sun, and ResetWith is exactly what moves the sun, so a table built only at construction would
  // leave them pointing at yesterday's sun with nothing to report it.
  RenderConfig cfg = MakeFullSkyConfig();
  cfg.markers_opacity_ = 1.0f;
  cfg.markers_radius_px_ = 8.0f;
  cfg.markers_.push_back(Marker(MarkerRefId::kSun, 0.0f, 0.0f, 1.0f));

  RenderConsumer rc(cfg, ColorClassTable{}, MakeSun());
  std::array<annotation::CanvasPoint, annotation::kMarkerCount> before = rc.MarkerPointsForTest();
  size_t imaged_before = 0;
  for (size_t k = 0; k < before.size(); ++k) {
    if (before[k].valid) {
      imaged_before++;
    } else {
      ADD_FAILURE() << "marker " << k << " is not imaged before the sun moves";
    }
  }
  ASSERT_EQ(imaged_before, before.size()) << "the fixture must image every marker before the sun moves";

  // Appearance-only change plus a new sun: NeedsRebuild says false for this pair, so this is the
  // path a live consumer actually takes.
  ASSERT_FALSE(NeedsRebuild(cfg, cfg));
  rc.ResetWith(cfg, MakeOtherSun());
  const std::array<annotation::CanvasPoint, annotation::kMarkerCount>& after = rc.MarkerPointsForTest();

  // The two poles do not depend on the sun and must not move; the four reflections of it must.
  for (int id : { annotation::kMarkerZenith, annotation::kMarkerNadir }) {
    EXPECT_FLOAT_EQ(after[id].px, before[id].px) << "marker " << id << " is a pole and must not follow the sun";
    EXPECT_FLOAT_EQ(after[id].py, before[id].py);
  }
  for (int id : { annotation::kMarkerSun, annotation::kMarkerSubsun, annotation::kMarkerAnthelion,
                  annotation::kMarkerAntisolar }) {
    EXPECT_GT(std::hypot(after[id].px - before[id].px, after[id].py - before[id].py), 1.0f)
        << "marker " << id << " is defined relative to the sun and did not move with it";
  }
}

}  // namespace
}  // namespace lumice
