// RenderConsumer's side of the TEXT labels: that `grid.horizon_label` / `grid.label` /
// `grid.angular_dist_label` each reach the image, that they reach it independently of one another,
// and — the two cases this file exists for — that the two LAYERS of "independent" are the ones the
// design says they are.
//
// The two layers, because every assertion below is about one of them:
//   GEOMETRY   — is the anchor computed at all. Independent of the family's own line switch: a
//                horizon_label_ with horizon_ false still produces anchors. Read through the
//                *LabelsForTest() handles, because a computed-then-invisible label and a
//                never-computed one are the same image and the difference is the whole point.
//   COMPOSITING — is the label visible once drawn. NOT independent: a label is painted in its
//                family's own colour and opacity, so a line at opacity 0 takes its labels with it.
//                Read off the image.
// The second is deliberate rather than a shortcut: the GUI has always given a label its family's
// colour and alpha (gui/overlay_labels.cpp), and a core that let a label outlive its line would be
// a NEW GUI/CLI divergence — which is the class of defect this annotation layer exists to remove.
//
// Method, shared with test_render_consumer_horizon.cpp: difference two snapshots of the SAME
// config, one with the switch off and one on, so the ray energy cancels and what is left is the
// annotation alone.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "server/render.hpp"
#include "support/render_anchor.hpp"

namespace lumice {
namespace {

constexpr int kW = 128;
constexpr int kH = 96;
constexpr int kTotalPix = kW * kH;

// Which switches a case wants on. A struct rather than three bool arguments so a call site says
// which switch it means at the call site.
struct LabelSwitches {
  bool horizon = false;
  bool grid = false;
  bool angular_dist = false;
};

// A linear 120 deg view centred 45 deg up: wide enough that the horizon, a 30 deg parallel, a
// meridian and a 22 deg circle around the sun all cross the frame, so every family has somewhere
// to put a label. Black background, so any non-black pixel is annotation or ray.
//
// The elevation is 45 rather than anything lower for a reason that has nothing to do with the
// annotations: the one ray below goes to the zenith, and PostSnapshot takes a zero-intensity
// early-out if it lands off-canvas, after which EVERY arm is a black frame and every difference is
// zero. Centring the view on the zenith puts it in the middle. NotAVacuousFixture below is the
// guard that says so out loud.
//
// `line_opacity` is a parameter because the compositing-layer case needs to set it to zero while
// leaving everything else — including the angle list, i.e. the geometry — untouched.
RenderConfig MakeLabelConfig(LabelSwitches on, bool draw_horizon_line = true, float line_opacity = 1.0f) {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kLinear;
  cfg.lens_.fov_ = 120.0f;
  cfg.resolution_[0] = kW;
  cfg.resolution_[1] = kH;
  cfg.view_.el_ = 45.0f;
  cfg.visible_ = RenderConfig::kFull;
  cfg.horizon_ = draw_horizon_line;
  cfg.horizon_label_ = on.horizon;
  cfg.grid_label_ = on.grid;
  cfg.angular_dist_label_ = on.angular_dist;

  GridLineParam parallel;
  parallel.value_ = 30.0f;
  parallel.opacity_ = line_opacity;
  parallel.color_[0] = parallel.color_[1] = parallel.color_[2] = 1.0f;
  cfg.elevation_grid_.push_back(parallel);

  GridLineParam meridian;
  meridian.value_ = 0.0f;
  meridian.opacity_ = line_opacity;
  meridian.color_[0] = meridian.color_[1] = meridian.color_[2] = 1.0f;
  cfg.longitude_grid_.push_back(meridian);

  GridLineParam circle;
  circle.value_ = 22.0f;
  circle.opacity_ = line_opacity;
  circle.color_[0] = 0.2f;
  circle.color_[1] = 1.0f;
  circle.color_[2] = 0.2f;
  cfg.angular_dist_grid_.push_back(circle);
  return cfg;
}

// The sun 20 deg up, which is where the angular-distance circles are centred. Its default (0) is
// on the horizon and would put the 22 deg circle half below it; nothing here depends on the exact
// value beyond the circle being fully inside the frame.
SunParam MakeSun() {
  SunParam sun;
  sun.altitude_ = 20.0f;
  return sun;
}

// One ray straight up, for the reason test_render_consumer_horizon.cpp states: PostSnapshot takes
// a zero-intensity early-out and every assertion would pass vacuously against an all-black frame.
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

// Whether anything at all was rendered. Every case below reads a DIFFERENCE between two frames,
// and two black frames differ by nothing — so a fixture that silently stopped producing an image
// would make the whole file pass while testing nothing.
bool HasAnyNonBlackPixel(const std::vector<uint8_t>& img) {
  for (const uint8_t v : img) {
    if (v != 0) {
      return true;
    }
  }
  return false;
}

// How many pixels differ between two renders of the same scene.
int DifferingPixels(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
  int n = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    const size_t base = static_cast<size_t>(i) * 3;
    if (a[base] != b[base] || a[base + 1] != b[base + 1] || a[base + 2] != b[base + 2]) {
      ++n;
    }
  }
  return n;
}

// (a) The switches are opt-in, and off costs nothing. The second half is asserted white-box rather
// than by comparing images: "no anchor was computed" is what makes the painting step provably a
// no-op, while an image comparison could only ever say the two arms happened to agree.
TEST(RenderConsumerLabel, TheSwitchesAreOptInAndOffComputesNoAnchors) {
  const RenderConfig defaults;
  EXPECT_FALSE(defaults.horizon_label_);
  EXPECT_FALSE(defaults.grid_label_);
  EXPECT_FALSE(defaults.angular_dist_label_);

  RenderConsumer off(MakeLabelConfig(LabelSwitches{}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img = SnapshotOnce(&off);
  ASSERT_EQ(img.size(), static_cast<size_t>(kTotalPix) * 3);
  EXPECT_TRUE(off.HorizonLabelsForTest().empty());
  EXPECT_TRUE(off.ElevationLabelsForTest().empty());
  EXPECT_TRUE(off.LongitudeLabelsForTest().empty());
  EXPECT_TRUE(off.AngularDistLabelsForTest().empty());
}

// (b) Each switch, on its own, puts text into the image. Three cases in one loop because the
// proposition is per family and the shape of the check is identical; the loop body reports which
// family it was on failure, so one broken family does not read as three.
TEST(RenderConsumerLabel, EachSwitchDrawsTextOnItsOwn) {
  RenderConsumer off(MakeLabelConfig(LabelSwitches{}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_TRUE(HasAnyNonBlackPixel(img_off)) << "the reference frame is entirely black — see MakeLabelConfig";

  struct Row {
    const char* name;
    LabelSwitches on;
  };
  for (const Row& row :
       { Row{ "horizon", LabelSwitches{ true, false, false } }, Row{ "grid", LabelSwitches{ false, true, false } },
         Row{ "angular_dist", LabelSwitches{ false, false, true } } }) {
    RenderConsumer on(MakeLabelConfig(row.on), ColorClassTable{}, MakeSun());
    const std::vector<uint8_t> img_on = SnapshotOnce(&on);
    // Non-fatal: one family that draws nothing must not take the other two's reports with it, and
    // WHICH family is the whole diagnostic.
    if (img_on.size() != static_cast<size_t>(kTotalPix) * 3) {
      ADD_FAILURE() << row.name << ": the snapshot came back the wrong size";
      continue;
    }
    const int changed = DifferingPixels(img_off, img_on);
    EXPECT_GT(changed, 0) << row.name << ": switching the labels on changed no pixel";
    // Text, not a fill. A label run at 15 px is tens of pixels per anchor; a quarter of the canvas
    // would mean something other than glyphs got painted.
    EXPECT_LT(changed, kTotalPix / 4) << row.name << ": the 'labels' covered a quarter of the canvas";
  }
}

// (c) The three switches are independent of each other: turning one on computes that family's
// anchors and leaves the other two families' alone.
TEST(RenderConsumerLabel, TheThreeSwitchesDoNotReachEachOther) {
  {
    RenderConsumer horizon(MakeLabelConfig(LabelSwitches{ true, false, false }), ColorClassTable{}, MakeSun());
    SnapshotOnce(&horizon);
    EXPECT_FALSE(horizon.HorizonLabelsForTest().empty());
    EXPECT_TRUE(horizon.ElevationLabelsForTest().empty());
    EXPECT_TRUE(horizon.LongitudeLabelsForTest().empty());
    EXPECT_TRUE(horizon.AngularDistLabelsForTest().empty());
  }
  {
    // One switch covers BOTH grid families — the GUI has a single grid label control — so this
    // arm expects two non-empty lists, not one.
    RenderConsumer grid(MakeLabelConfig(LabelSwitches{ false, true, false }), ColorClassTable{}, MakeSun());
    SnapshotOnce(&grid);
    EXPECT_TRUE(grid.HorizonLabelsForTest().empty());
    EXPECT_FALSE(grid.ElevationLabelsForTest().empty());
    EXPECT_FALSE(grid.LongitudeLabelsForTest().empty());
    EXPECT_TRUE(grid.AngularDistLabelsForTest().empty());
  }
  {
    RenderConsumer circles(MakeLabelConfig(LabelSwitches{ false, false, true }), ColorClassTable{}, MakeSun());
    SnapshotOnce(&circles);
    EXPECT_TRUE(circles.HorizonLabelsForTest().empty());
    EXPECT_TRUE(circles.ElevationLabelsForTest().empty());
    EXPECT_TRUE(circles.LongitudeLabelsForTest().empty());
    EXPECT_FALSE(circles.AngularDistLabelsForTest().empty());
  }
}

// (d) GEOMETRY LAYER. `horizon_label_` drives the anchors on its own: with `horizon_` false — no
// line drawn at all — the numbers still appear. The horizon is the family this can be asked of,
// because it is the only one whose line has a switch separate from its geometry (the other three
// are "is the angle in the list").
TEST(RenderConsumerLabel, TheHorizonLabelIsDrawnWithTheLineSwitchedOff) {
  RenderConsumer no_line_no_label(MakeLabelConfig(LabelSwitches{}, /*draw_horizon_line=*/false), ColorClassTable{},
                                  MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&no_line_no_label);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_TRUE(HasAnyNonBlackPixel(img_off)) << "the reference frame is entirely black — see MakeLabelConfig";

  RenderConsumer label_only(MakeLabelConfig(LabelSwitches{ true, false, false }, /*draw_horizon_line=*/false),
                            ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&label_only);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  EXPECT_FALSE(label_only.HorizonLabelsForTest().empty())
      << "no horizon anchor was computed — the label switch was read as depending on the line switch";
  EXPECT_GT(DifferingPixels(img_off, img_on), 0) << "the horizon's numbers did not reach the image";

  // And the LINE really is still off in both arms: the mask is non-empty, so if `horizon_label_`
  // had been ORed into the painting gate as well, these pixels would differ from the background.
  const std::vector<uint8_t>& mask = label_only.HorizonMaskForTest();
  ASSERT_EQ(mask.size(), static_cast<size_t>(kTotalPix));
  int painted_mask_pixels = 0;
  for (int i = 0; i < kTotalPix; ++i) {
    if (mask[static_cast<size_t>(i)] == 0) {
      continue;
    }
    const size_t base = static_cast<size_t>(i) * 3;
    // The horizon line is red at 60% (render.cpp kOutlineSrgb / kOutlineAlpha); the labels are the
    // same colour, so "red" alone cannot tell them apart. What can is the mask: a LINE paints
    // every masked pixel, a label covers a handful of them at most. Counting is the test.
    if (img_on[base] > img_on[base + 1] + 60 && img_on[base] > img_on[base + 2] + 60) {
      ++painted_mask_pixels;
    }
  }
  const auto marked = static_cast<int>(std::count(mask.begin(), mask.end(), uint8_t{ 1 }));
  ASSERT_GT(marked, 0) << "an empty horizon mask makes the assertion below vacuous";
  EXPECT_LT(painted_mask_pixels, marked / 4)
      << "most of the horizon mask came out red — the line was painted despite horizon_ being false";
}

// (e) COMPOSITING LAYER, and the case that pins the decision this file's header explains. A grid
// line at opacity 0 is invisible, and its labels go with it — even though the switch is on and the
// anchors WERE computed. The two halves are asserted separately on purpose: "the anchors exist"
// is what rules out the alternative explanation that the geometry was skipped, which would make
// the image assertion pass for the wrong reason.
TEST(RenderConsumerLabel, AZeroOpacityLineTakesItsLabelWithIt) {
  RenderConsumer off(MakeLabelConfig(LabelSwitches{}, /*draw_horizon_line=*/true, /*line_opacity=*/0.0f),
                     ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&off);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_TRUE(HasAnyNonBlackPixel(img_off)) << "the reference frame is entirely black — see MakeLabelConfig";

  RenderConsumer on(MakeLabelConfig(LabelSwitches{ false, true, true }, /*draw_horizon_line=*/true,
                                    /*line_opacity=*/0.0f),
                    ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&on);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  EXPECT_FALSE(on.ElevationLabelsForTest().empty()) << "the anchors were not computed — this case would then be "
                                                       "passing for a reason it is not testing";
  EXPECT_FALSE(on.AngularDistLabelsForTest().empty());
  EXPECT_EQ(DifferingPixels(img_off, img_on), 0)
      << "a label was painted for a line the user made fully transparent; label opacity must follow its family's";
}

// The label switches are APPEARANCE fields, so a config that flips one arrives at an existing
// consumer through ResetWith() with no rebuild. Without a change detector that watches the switch,
// the anchors computed for the old value would survive — empty exactly when the user has just
// asked for the text.
TEST(RenderConsumerLabel, FlippingASwitchMidRunReachesTheImage) {
  RenderConsumer rc(MakeLabelConfig(LabelSwitches{}), ColorClassTable{}, MakeSun());
  const std::vector<uint8_t> img_off = SnapshotOnce(&rc);
  ASSERT_EQ(img_off.size(), static_cast<size_t>(kTotalPix) * 3);
  ASSERT_TRUE(HasAnyNonBlackPixel(img_off)) << "the reference frame is entirely black — see MakeLabelConfig";
  ASSERT_TRUE(rc.ElevationLabelsForTest().empty());

  const RenderConfig on = MakeLabelConfig(LabelSwitches{ true, true, true });
  ASSERT_FALSE(NeedsRebuild(MakeLabelConfig(LabelSwitches{}), on))
      << "the label switches were classified as layout fields; this case is then testing a path the "
         "server never takes";
  rc.ResetWith(on, MakeSun());
  const std::vector<uint8_t> img_on = SnapshotOnce(&rc);
  ASSERT_EQ(img_on.size(), static_cast<size_t>(kTotalPix) * 3);

  EXPECT_FALSE(rc.HorizonLabelsForTest().empty());
  EXPECT_FALSE(rc.ElevationLabelsForTest().empty());
  EXPECT_FALSE(rc.AngularDistLabelsForTest().empty());
  EXPECT_GT(DifferingPixels(img_off, img_on), 0) << "the switch flipped but the image did not change";
}

}  // namespace
}  // namespace lumice
