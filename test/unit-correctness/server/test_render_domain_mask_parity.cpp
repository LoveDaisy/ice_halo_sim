// LUMICE_TEST_ComputeRenderDomainMask (test/support/lumice_test_api.h, the test-only export
// surface) must hand out the SAME bytes as core's own render-domain mask,
// lumice::annotation::ComputeOverlay(...).drawable — the mask the CLI renderer composites against
// in-process. The hook exists so pytest can read that mask without the product ABI carrying a
// test-only reason to export anything, and this file is what makes "same computation" a checked
// claim rather than a comment.
//
// It is also the direct refutation path for the one shortcut the hook takes: it feeds the core
// sweep a Request with every line list empty and `labels = false`, on the assumption that the
// drawable sweep does not depend on any of them. If ComputeOverlay ever couples the drawable to
// the label walk or to a requested line family, this test goes red before any downstream consumer
// notices a shifted mask — because the oracle below asks core with the line families ON.
//
// THE ORACLE IS CORE ITSELF, and that is a weaker comparison than this file once made. Until
// v4.28 the product API exported the mask (LUMICE_AnnotationOverlay::drawable) and stood here as
// an independent door onto the same computation; the hook calls ComputeOverlay directly, so
// comparing it against ComputeOverlay is the hook checking its own homework as far as the sweep
// is concerned. What the comparison still holds honestly: the hook's view translation
// (ToAnnotationViewSnapshot, shared by construction), its storage and pointer handling, its
// degenerate-view and error contract, and the independence of the sweep from the line lists —
// the oracle request below carries lines and labels, the hook's carries none. The product API no
// longer has a mask to compare against, by design: see the v4.28 note at LUMICE_API_VERSION.
//
// Flavor note: unit_correctness_test compiles lumice_test_api.cpp into a static, test-flavor
// binary, while pytest loads the same source compiled into the shared-flavor liblumice_testapi.
// The two differ only in export options (-fvisibility=hidden / WINDOWS_EXPORT_ALL_SYMBOLS), which
// govern which symbols a dlopen can see and not what a function body computes; the parity pinned
// here is taken to hold for the shared build on that basis.
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "core/annotation_overlay.hpp"
#include "include/lumice.h"
#include "server/c_api_internal.hpp"  // ToAnnotationViewSnapshot
#include "support/lumice_test_api.h"

namespace {

struct Scene {
  const char* name;
  LUMICE_AnnotationView view;
};

// One all-sky lens and one single lens, each under a `visible` / `front` combination the other
// does not use, so the mask carries every predicate the hook is supposed to reproduce (imaged,
// VisibleByRange, FrontVisible) at least once. Odd canvas sizes so a row-stride mistake cannot hide
// behind a power of two; a lens shift and a roll so the view is not the all-defaults one either.
LUMICE_AnnotationView MakeView(int width, int height, int lens_type, float fov, int shift_x, int shift_y, float overlap,
                               float az, float el, float roll, int visible, int front) {
  LUMICE_AnnotationView v{};
  v.width = width;
  v.height = height;
  v.lens_type = lens_type;
  v.lens_fov = fov;
  v.lens_shift[0] = shift_x;
  v.lens_shift[1] = shift_y;
  v.overlap = overlap;
  v.view_azimuth = az;
  v.view_elevation = el;
  v.view_roll = roll;
  v.visible = visible;
  v.front = front;
  return v;
}

const Scene kScenes[] = {
  { "dual_fisheye_equal_area_full_nofront", MakeView(257, 131, LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA, 180.0f, 0, 0,
                                                     0.0f, 0.0f, 0.0f, 0.0f, LUMICE_VISIBLE_FULL, 0) },
  { "linear_upper_front",
    MakeView(193, 121, LUMICE_LENS_TYPE_LINEAR, 60.0f, 3, -2, 0.0f, 30.0f, 12.0f, 5.0f, LUMICE_VISIBLE_UPPER, 1) },
  { "fisheye_equal_area_lower_front", MakeView(171, 171, LUMICE_LENS_TYPE_FISHEYE_EQUAL_AREA, 150.0f, 0, 0, 0.0f,
                                               200.0f, -35.0f, 0.0f, LUMICE_VISIBLE_LOWER, 1) },
  { "dual_fisheye_equal_area_upper_front_overlap", MakeView(255, 129, LUMICE_LENS_TYPE_DUAL_FISHEYE_EQUAL_AREA, 180.0f,
                                                            0, 0, 0.1f, 90.0f, 20.0f, 0.0f, LUMICE_VISIBLE_UPPER, 1) },
};

// Core's own answer for a view, asked WITH lines and labels so the comparison also says the
// drawable does not depend on them (the hook asks with none). The view goes through the same
// translation the hook uses — there is one, on purpose (a56) — so what this compares is the sweep
// and the hook's handling of its result, not two hand-copied field mappings.
struct CoreOverlay {
  lumice::annotation::Overlay overlay;
  explicit CoreOverlay(const LUMICE_AnnotationView& view) {
    lumice::annotation::Request req;
    req.view = ToAnnotationViewSnapshot(view);
    req.horizon = true;
    req.elevation_deg = { -30.0f, 30.0f };
    req.longitude_deg = { 0.0f, 90.0f };
    req.angular_dist_deg = { 22.0f };
    req.markers = { lumice::annotation::kMarkerZenith };
    req.labels = true;
    overlay = lumice::annotation::ComputeOverlay(req);
  }
};

struct HookMask {
  LUMICE_TEST_RenderDomainMask mask{};
  explicit HookMask(const LUMICE_AnnotationView& view) {
    EXPECT_EQ(LUMICE_TEST_ComputeRenderDomainMask(&view, &mask), LUMICE_OK);
  }
  ~HookMask() { LUMICE_TEST_ReleaseRenderDomainMask(&mask); }
  HookMask(const HookMask&) = delete;
  HookMask& operator=(const HookMask&) = delete;
};

// One scene's comparison, in its own function so a fatal assert ends THIS scene and not the loop
// that drives the others (scripts/check_loop_fatal_asserts.py pins that shape).
void ExpectHookMatchesCore(const Scene& scene) {
  SCOPED_TRACE(scene.name);
  const CoreOverlay core(scene.view);
  const HookMask hook(scene.view);
  const lumice::annotation::Overlay& o = core.overlay;
  const LUMICE_TEST_RenderDomainMask& m = hook.mask;

  EXPECT_EQ(m.width, o.width);
  EXPECT_EQ(m.height, o.height);
  EXPECT_EQ(m.width, scene.view.width);
  EXPECT_EQ(m.height, scene.view.height);
  ASSERT_FALSE(o.drawable.empty());
  ASSERT_NE(m.imaged, nullptr);
  // Distinct allocations: the hook owns its own storage, it does not alias core's.
  EXPECT_NE(m.imaged, o.drawable.data());

  const size_t n = static_cast<size_t>(o.width) * static_cast<size_t>(o.height);
  EXPECT_EQ(std::memcmp(m.imaged, o.drawable.data(), n), 0);

  // A mask that is all-0 or all-1 would satisfy memcmp without exercising the predicate; every
  // scene here is built to have both inside and outside pixels.
  size_t ones = 0;
  for (size_t i = 0; i < n; ++i) {
    ones += m.imaged[i] != 0;
  }
  EXPECT_GT(ones, 0u);
  EXPECT_LT(ones, n);
}

}  // namespace


TEST(RenderDomainMaskParity, HookMatchesCoreDrawableByteForByte) {
  for (const Scene& scene : kScenes) {
    ExpectHookMatchesCore(scene);
  }
}


TEST(RenderDomainMaskParity, DegenerateViewIsNotAnErrorAndStillReleases) {
  LUMICE_AnnotationView view = kScenes[0].view;
  view.width = 0;
  LUMICE_TEST_RenderDomainMask mask{};
  ASSERT_EQ(LUMICE_TEST_ComputeRenderDomainMask(&view, &mask), LUMICE_OK);
  EXPECT_EQ(mask.width, 0);
  EXPECT_EQ(mask.height, 0);
  EXPECT_EQ(mask.imaged, nullptr);
  EXPECT_NE(mask.storage, nullptr);
  LUMICE_TEST_ReleaseRenderDomainMask(&mask);
  EXPECT_EQ(mask.storage, nullptr);
}


TEST(RenderDomainMaskParity, ArgumentValidationMirrorsTheAnchorsApi) {
  LUMICE_TEST_RenderDomainMask mask{};
  EXPECT_EQ(LUMICE_TEST_ComputeRenderDomainMask(nullptr, &mask), LUMICE_ERR_NULL_ARG);
  EXPECT_EQ(LUMICE_TEST_ComputeRenderDomainMask(&kScenes[0].view, nullptr), LUMICE_ERR_NULL_ARG);

  LUMICE_AnnotationView bad_lens = kScenes[0].view;
  bad_lens.lens_type = LUMICE_LENS_TYPE_GLOBE + 1;
  EXPECT_EQ(LUMICE_TEST_ComputeRenderDomainMask(&bad_lens, &mask), LUMICE_ERR_INVALID_VALUE);

  LUMICE_AnnotationView bad_visible = kScenes[0].view;
  bad_visible.visible = 7;
  EXPECT_EQ(LUMICE_TEST_ComputeRenderDomainMask(&bad_visible, &mask), LUMICE_ERR_INVALID_VALUE);

  // A failed call leaves `*out` untouched, so there is nothing to release — and releasing the
  // untouched zero struct is a no-op rather than a fault.
  EXPECT_EQ(mask.storage, nullptr);
  EXPECT_EQ(mask.imaged, nullptr);
  LUMICE_TEST_ReleaseRenderDomainMask(&mask);
  LUMICE_TEST_ReleaseRenderDomainMask(nullptr);
}


TEST(RenderDomainMaskParity, ReleaseIsIdempotentAndNullsTheView) {
  LUMICE_TEST_RenderDomainMask mask{};
  ASSERT_EQ(LUMICE_TEST_ComputeRenderDomainMask(&kScenes[1].view, &mask), LUMICE_OK);
  ASSERT_NE(mask.storage, nullptr);
  LUMICE_TEST_ReleaseRenderDomainMask(&mask);
  EXPECT_EQ(mask.storage, nullptr);
  EXPECT_EQ(mask.imaged, nullptr);
  EXPECT_EQ(mask.width, 0);
  EXPECT_EQ(mask.height, 0);
  LUMICE_TEST_ReleaseRenderDomainMask(&mask);  // second release: no-op, no double free
  EXPECT_EQ(mask.storage, nullptr);
}
