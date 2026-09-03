#ifndef LUMICE_TEST_SUPPORT_RENDER_ANCHOR_HPP
#define LUMICE_TEST_SUPPORT_RENDER_ANCHOR_HPP

// The exposure-anchor handoff a RenderConsumer driven OUTSIDE the server does not get.
//
// `RenderConsumer::anchor_l99_sky_` is a property of the SESSION, measured by AnchorConsumer over
// a full-sky plane the renderer never sees and pushed in by ServerImpl::DoSnapshot between that
// measurement and the bake. A unit test that constructs one RenderConsumer and calls
// PrepareSnapshot/PostSnapshot by hand has no AnchorConsumer and therefore no anchor, so
// ExposureScale's kRelative branch returns 0 and PostSnapshot memsets the frame to black — which
// would make every annotation assertion in this suite pass vacuously, in both arms of a
// difference test.
//
// WHAT VALUE IT PUSHES, AND WHY THAT ONE. The sky radiance for which this frame's own coarse P99
// IS the anchor: `L99_sky = P99(this frame) / Omega_axis(this view)`. That is deliberately the
// retired per-view rule, restated as an input rather than left in the code — it holds every
// annotation fixture's exposure exactly where it was, so a red in this suite after the anchor
// change is about the annotation and never about a brightness that moved underneath it. It is
// NOT what production computes, and nothing here should be read as a statement about the anchor:
// a fixture that fires one ray into an otherwise empty sky has no meaningful sky radiance, which
// is precisely why running a real AnchorConsumer over these batches would expose them at
// something like 1/700 of their former brightness and turn every fixture black. Tests that are
// ABOUT the anchor (test_anchor_consumer.cpp, test_render_consumer_exposure_scale.cpp) set it
// themselves and must not use this.

#include "core/ev_anchor.hpp"
#include "server/render.hpp"
#include "server/server.hpp"

namespace lumice::test {

// PrepareSnapshot -> push the anchor -> PostSnapshot, in the server's order.
inline void TakeSnapshotAtFormerSelfAnchor(RenderConsumer* rc) {
  rc->PrepareSnapshot();
  const RawXyzResult raw = rc->GetRawXyzResult();
  const float p99 = ComputeP99Y(raw.xyz_buffer_, raw.img_width_, raw.img_height_, kMonoAnchorDownsampleFactor);
  const float omega_axis = rc->AxisSolidAngle();
  rc->SetAnchorL99Sky(omega_axis > 0.0f ? p99 / omega_axis : 0.0f);
  rc->PostSnapshot();
}

}  // namespace lumice::test

#endif  // LUMICE_TEST_SUPPORT_RENDER_ANCHOR_HPP
