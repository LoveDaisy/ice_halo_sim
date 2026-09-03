// The two routes into a marker's canvas position must be one route.
//
// core/annotation_overlay.hpp offers a caller two ways to ask where the zenith and the nadir land:
// the legacy `Request::zenith_nadir` boolean, published through the C API since v4.19 and read by
// both shipped consumers, and the general `Request::markers` list added alongside it. Whether
// those two agree is not an implementation detail — it is the whole reason the generalization was
// allowed to keep the old field instead of migrating its callers. A second implementation that
// merely happens to agree today would silently stop agreeing the first time either side is
// touched, and the failure would be a marker drawn a few pixels off in the GUI while the CLI drew
// it correctly, or the reverse: visible only to someone comparing two renders side by side.
//
// So this file asserts BIT-IDENTICAL results (`==` on the raw floats, not EXPECT_NEAR), which is
// the strongest statement available and is only reachable because the two routes call literally
// the same sampler inside ComputeOverlay. A tolerance here would be the tell that they had drifted
// into two implementations: if they ever need slack to agree, the design assumption is already
// broken and this test should fail rather than absorb it.
//
// The five view cases below are chosen so that agreement is not vacuous. Two things can make it
// vacuous, and both are guarded:
//   - a marker that misses the canvas everywhere agrees trivially (false == false with no pixels
//     compared), so the cases are checked for a MIX of valid and invalid outcomes across the file;
//   - a marker that lands dead centre under every lens agrees for reasons that have nothing to do
//     with the sampler, so the cases spread across four lens families, both hemispheric clips and
//     the front clip.
//
// What this file does NOT cover: the C API bridge (LUMICE_ComputeAnnotationOverlay's id
// validation and array copy). That is a transport layer over this one and is pinned in
// test/unit-correctness/server/test_c_api.cpp.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "config/render_config.hpp"
#include "core/annotation_overlay.hpp"

namespace {

using lumice::LensParam;
using lumice::RenderConfig;
namespace ann = lumice::annotation;

// One view to compare the two routes under. The fields are spelled out per case rather than built
// by a helper with defaults, because "which knob is set" is exactly what a reader has to be able
// to check here.
struct ViewCase {
  const char* name;
  LensParam::LensType lens_type;
  float fov_deg;
  int width;
  int height;
  float az_deg;
  float el_deg;
  float roll_deg;
  RenderConfig::VisibleRange visible;
  bool front;
};

// Five combinations, each present for a stated reason:
//
//   all_sky_dual        the baseline: an all-sky dual fisheye images BOTH poles, so this is the
//                       case where the comparison actually has two valid points to compare.
//   upper_only_fisheye  visible = kUpper cuts the nadir out. Guards the `valid = false` half of
//                       the agreement, which a test built only on visible markers never reaches.
//   front_clip_linear   front = true is the second, independent clip. A narrow linear lens tilted
//                       up puts the zenith on canvas and leaves the nadir behind the camera.
//   tilted_rect_lower   a rectangular lens with a non-zero roll and visible = kLower — the case
//                       where px/py are not on any symmetry axis of the canvas, so an agreement in
//                       both coordinates is not an artefact of the point being centred.
//   equidistant_offset  a single fisheye pointed away from both poles at a non-square resolution,
//                       so width and height enter the projection differently.
const ViewCase kViewCases[] = {
  { "all_sky_dual", LensParam::kDualFisheyeEqualArea, 180.0f, 256, 128, 0.0f, 0.0f, 0.0f, RenderConfig::kFull, false },
  { "upper_only_fisheye", LensParam::kFisheyeEqualArea, 180.0f, 192, 192, 0.0f, 90.0f, 0.0f, RenderConfig::kUpper,
    false },
  { "front_clip_linear", LensParam::kLinear, 60.0f, 200, 150, 33.0f, 55.0f, 0.0f, RenderConfig::kFull, true },
  { "tilted_rect_lower", LensParam::kRectangular, 120.0f, 240, 160, 137.0f, -25.0f, 17.0f, RenderConfig::kLower,
    false },
  { "equidistant_offset", LensParam::kFisheyeEquidistant, 140.0f, 320, 180, 271.0f, 12.5f, 0.0f, RenderConfig::kFull,
    false },
};

ann::Request MakeRequest(const ViewCase& c) {
  ann::Request req;
  req.view.width = c.width;
  req.view.height = c.height;
  req.view.lens_type = c.lens_type;
  req.view.fov_deg = c.fov_deg;
  req.view.az_deg = c.az_deg;
  req.view.el_deg = c.el_deg;
  req.view.roll_deg = c.roll_deg;
  req.view.visible = c.visible;
  req.view.front = c.front;
  // The sun direction the marker table reflects. Off both poles and off both axes so that the four
  // sun-relative ids resolve to four DIFFERENT directions; the zenith/nadir pair ignores it, but
  // the same request feeds the coverage case at the bottom of this file.
  req.reference_dir[0] = -0.6f;
  req.reference_dir[1] = -0.48f;
  req.reference_dir[2] = -0.64f;
  // Off: the curve walk is a large, slow, and here irrelevant part of the computation, and the
  // marker sampling this file is about happens before it either way.
  req.labels = false;
  return req;
}

std::string Where(const ViewCase& c) {
  return std::string("view=") + c.name;
}

}  // namespace

// The load-bearing assertion of the whole task: same points, bit for bit.
//
// What this can and cannot catch, established by running both probes before it was committed
// rather than by reasoning about it:
//   - Break ONE route (give `zenith_nadir` back its own hardcoded sampling with a flipped sign)
//     and this goes red on every case that images a pole. That is the failure it exists for.
//   - Flip the sign inside ResolveMarkerDir and this stays GREEN, because both routes call it and
//     both move together. That is not a gap in this test, it is the direct consequence of the
//     design it verifies: agreement is guaranteed by shared code, so shared code is invisible to
//     an agreement test.
// So this file says "there is one implementation", never "the direction is right". The direction
// is pinned separately, by the MarkerDirectionTable cases in
// test/unit-correctness/core/test_annotation_overlay.cpp, which is what actually went red under
// the second probe. Anyone reading the two as one guard will trust this file for something it
// cannot do.
TEST(AnnotationMarkerEquivalence, MarkerListReproducesTheLegacyZenithNadirPairExactly) {
  int valid_seen = 0;
  int invalid_seen = 0;

  for (const ViewCase& c : kViewCases) {
    ann::Request legacy = MakeRequest(c);
    legacy.zenith_nadir = true;

    ann::Request via_markers = MakeRequest(c);
    via_markers.markers = { ann::kMarkerZenith, ann::kMarkerNadir };

    const ann::Overlay a = ann::ComputeOverlay(legacy);
    const ann::Overlay b = ann::ComputeOverlay(via_markers);

    if (b.markers.size() != 2u) {
      // Non-fatal: a fatal assert here would return out of the loop and leave every later view
      // case unevaluated, reporting one red where there may be five.
      ADD_FAILURE() << Where(c) << ": expected 2 marker points, got " << b.markers.size();
      continue;
    }
    // The legacy route must not be perturbed by the new one existing, and vice versa: neither
    // request set the other's switch, so each result carries exactly what was asked for.
    EXPECT_TRUE(a.markers.empty()) << Where(c);
    EXPECT_FALSE(b.zenith.valid) << Where(c) << ": zenith_nadir was not requested on this route";
    EXPECT_FALSE(b.nadir.valid) << Where(c) << ": zenith_nadir was not requested on this route";

    EXPECT_EQ(a.zenith.valid, b.markers[0].valid) << Where(c) << " zenith";
    EXPECT_EQ(a.nadir.valid, b.markers[1].valid) << Where(c) << " nadir";
    // px/py are only meaningful where valid; comparing them unconditionally would pin an
    // unspecified value.
    if (a.zenith.valid) {
      EXPECT_EQ(a.zenith.px, b.markers[0].px) << Where(c) << " zenith px";
      EXPECT_EQ(a.zenith.py, b.markers[0].py) << Where(c) << " zenith py";
    }
    if (a.nadir.valid) {
      EXPECT_EQ(a.nadir.px, b.markers[1].px) << Where(c) << " nadir px";
      EXPECT_EQ(a.nadir.py, b.markers[1].py) << Where(c) << " nadir py";
    }

    for (const ann::CanvasPoint& p : b.markers) {
      (p.valid ? valid_seen : invalid_seen)++;
    }
  }

  // Without these two the whole loop above passes on a build where every marker misses the canvas,
  // which is the one way "identical results" can be true and worthless.
  EXPECT_GT(valid_seen, 0) << "no marker landed anywhere: the equality above compared nothing";
  EXPECT_GT(invalid_seen, 0) << "no marker was clipped: the valid=false half of the agreement is untested";
}

// Setting both routes in ONE request must not make them interfere — the legacy pair and the list
// are independent inputs, and a caller migrating from one to the other will pass both for a while.
TEST(AnnotationMarkerEquivalence, RequestingBothRoutesAtOnceYieldsTheSameTwoPoints) {
  for (const ViewCase& c : kViewCases) {
    ann::Request req = MakeRequest(c);
    req.zenith_nadir = true;
    req.markers = { ann::kMarkerNadir, ann::kMarkerZenith };  // reversed on purpose

    const ann::Overlay out = ann::ComputeOverlay(req);
    if (out.markers.size() != 2u) {
      ADD_FAILURE() << Where(c) << ": expected 2 marker points, got " << out.markers.size();
      continue;
    }
    EXPECT_EQ(out.nadir.valid, out.markers[0].valid) << Where(c);
    EXPECT_EQ(out.zenith.valid, out.markers[1].valid) << Where(c);
    if (out.nadir.valid) {
      EXPECT_EQ(out.nadir.px, out.markers[0].px) << Where(c);
      EXPECT_EQ(out.nadir.py, out.markers[0].py) << Where(c);
    }
    if (out.zenith.valid) {
      EXPECT_EQ(out.zenith.px, out.markers[1].px) << Where(c);
      EXPECT_EQ(out.zenith.py, out.markers[1].py) << Where(c);
    }
  }
}

// The list is positional, not a set: the result must line up index for index with what was asked,
// duplicates included. A consumer reads marker_points[i] as "the marker I put at request index i",
// so an implementation that deduplicated or sorted would corrupt every colour assignment downstream
// while still returning plausible pixel coordinates.
TEST(AnnotationMarkerEquivalence, ResultsAreParallelToTheRequestIncludingDuplicates) {
  const ViewCase& c = kViewCases[0];
  ann::Request req = MakeRequest(c);
  req.markers = { ann::kMarkerSun, ann::kMarkerZenith, ann::kMarkerSun, ann::kMarkerAntisolar, ann::kMarkerZenith };

  const ann::Overlay out = ann::ComputeOverlay(req);
  ASSERT_EQ(out.markers.size(), req.markers.size());
  // Same id => same point, wherever it appears in the list.
  EXPECT_EQ(out.markers[0].valid, out.markers[2].valid);
  EXPECT_EQ(out.markers[0].px, out.markers[2].px);
  EXPECT_EQ(out.markers[0].py, out.markers[2].py);
  EXPECT_EQ(out.markers[1].valid, out.markers[4].valid);
  EXPECT_EQ(out.markers[1].px, out.markers[4].px);
  EXPECT_EQ(out.markers[1].py, out.markers[4].py);
  // ...and different ids must not collapse onto one point, or the equality above would be
  // satisfied by an implementation that returned the same point for everything.
  ASSERT_TRUE(out.markers[0].valid && out.markers[3].valid) << "the coverage below needs both on canvas";
  EXPECT_TRUE(out.markers[0].px != out.markers[3].px || out.markers[0].py != out.markers[3].py)
      << "the sun and the antisolar point resolved to the same pixel";
}

// An empty marker list is the default, and it is what every existing caller sends. It must cost
// nothing and return nothing — this is the structural reason the GUI and the CLI needed no change.
TEST(AnnotationMarkerEquivalence, NoMarkersRequestedReturnsNoMarkers) {
  for (const ViewCase& c : kViewCases) {
    ann::Request req = MakeRequest(c);
    req.zenith_nadir = true;
    const ann::Overlay out = ann::ComputeOverlay(req);
    EXPECT_TRUE(out.markers.empty()) << Where(c);
  }
}
