#ifndef CORE_ANCHOR_BUFFER_H_
#define CORE_ANCHOR_BUFFER_H_

#include <algorithm>

#include "config/render_config.hpp"
#include "core/ev_anchor.hpp"
#include "core/geo3d.hpp"
#include "core/lens_proj_build.hpp"
#include "core/shared/projection_shared.h"

namespace lumice {

// =============================================================================
// The EXPOSURE ANCHOR BUFFER — one fixed view of the whole sky, per session.
// =============================================================================
//
// WHAT IT IS. A second accumulation target, alongside whatever renderers the user
// configured, whose geometry is a constant of this header rather than a function of the
// scene: a dual-fisheye EQUAL-AREA pair covering the full sphere at a fixed resolution,
// with no view rotation, no fov, no `visible` clip and no overlap ring. The statistic
// taken over it — AnchorL99Sky below — is therefore a property of the SCENE and not of
// any lens looking at it.
//
// WHY IT EXISTS. `ev_mode: relative` anchors a frame to its own P99. Anchoring that P99
// to the renderer's OWN output buffer makes the exposure a function of the lens, the
// fov and the output resolution — three things that describe how the sky is being
// LOOKED AT, not how bright it is. Two renderers on one scene then disagree about the
// scene's brightness by up to a couple of stops, and the GUI (which anchors to a
// full-sky texture) and the CLI (which anchors to the output buffer) carry a constant
// gain difference between them for the same reason. One sky, one anchor.
//
// WHY EQUAL-AREA SPECIFICALLY. The statistic has to be a RADIANCE (per steradian), and
// on an equal-area projection every pixel subtends the same solid angle, so the
// conversion is one constant (kAnchorPixelSolidAngle) instead of a per-pixel Jacobian.
// Any other projection would make "the P99 pixel" mean a different amount of sky
// depending on where in the frame it landed.
//
// WHY THIS RESOLUTION, AND WHY IT IS PAIRED WITH THE DOWNSAMPLE FACTOR. The anchor is a
// P99 over BOX-SUMMED bins, so what the number actually depends on is the ANGULAR
// DIAMETER of a bin — the (resolution, downsample_factor) pair, not either alone. At
// 2048x1024 with f=8 a coarse bin spans 1.27 degrees. Measured against a 4096x2048
// reference the shift is under 0.16 stop on every scene tried, including a compact
// parhelion, whereas 1024x512 costs 0.4 stop on that same compact feature; and below
// about 128x64 the sample count collapses far enough that NthElementP99's
// `idx = floor(n*0.99)` degenerates to the maximum (n <= 100 => idx = n-1), which is
// what makes the low-resolution end drift NON-monotonically. Changing either member of
// the pair without the other silently re-scales every `relative` render.
constexpr int kAnchorWidth = 2048;
constexpr int kAnchorHeight = 1024;

// The same box-sum factor the mono self-anchor uses. Named separately from
// kMonoAnchorDownsampleFactor rather than used directly at the call sites, because the
// pair above is what has to be changed together — see the paragraph on angular diameter.
constexpr int kAnchorDownsampleFactor = kMonoAnchorDownsampleFactor;

// Solid angle subtended by ONE anchor pixel, in steradians.
//
// DualFisheyeToPixelXY lays out two discs of radius r = min(W/2, H)/2, and
// FisheyeEqualAreaForward maps a hemisphere onto the unit disc as rho = sqrt(1 - dz).
// Inverting that, the cap inside radius rho subtends 2*pi*rho^2 while covering an area
// pi*rho^2, i.e. exactly 2 sr per unit of disc area everywhere on the disc. In pixels
// that is 2/r^2. At 2048x1024: r = 512, so 7.62939453125e-6 sr.
constexpr float kAnchorDiscRadius =
    static_cast<float>(kAnchorWidth / 2 < kAnchorHeight ? kAnchorWidth / 2 : kAnchorHeight) / 2.0f;
constexpr float kAnchorPixelSolidAngle = 2.0f / (kAnchorDiscRadius * kAnchorDiscRadius);

// The RenderConfig the anchor projection is built from. Not a user config and never
// reachable from one: it exists so the anchor can go through the SAME BuildProjParams /
// ProjectExitToPixel path every renderer uses, rather than growing a second, parallel
// projection implementation that would then have to be kept in agreement with it.
inline RenderConfig MakeAnchorRenderConfig() {
  RenderConfig cfg;
  cfg.lens_.type_ = LensParam::kDualFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = kAnchorWidth;
  cfg.resolution_[1] = kAnchorHeight;
  // overlap_ stays 0: an overlap ring dual-writes a band of rays into BOTH discs, which
  // would double-count exactly the energy near the seam. The renderer wants that ring for
  // appearance; a measurement must not have it.
  cfg.overlap_ = 0.0f;
  return cfg;
}

// The projection POD every anchor accumulator uses — the host consumer's CPU loop and the
// buffers the Metal and CUDA kernels are handed. ONE function so the three backends cannot
// disagree about the anchor's geometry the way they could if each built its own.
//
// The rotation is the identity: the dual-fisheye branch of ProjectExitToPixel does not
// read `rot` at all, so the anchor is invariant to the camera pose as well as to the lens.
inline lm_proj::ProjParams BuildAnchorProjParams() {
  const RenderConfig cfg = MakeAnchorRenderConfig();
  const auto short_pix = static_cast<float>(std::min(cfg.resolution_[0], cfg.resolution_[1]));
  return BuildProjParams(cfg, Rotation{}, short_pix);
}

// The published scalar: the P99 of the anchor plane, expressed as a RADIANCE.
//
// `anchor_y` is a kAnchorWidth * kAnchorHeight plane of accumulated Y (one float per
// pixel, no interleaved X/Z — the anchor's only consumer is this statistic, and the Y
// channel is all it reads).
//
// ComputeP99Y already divides the coarse P99 by f^2, so what comes back is a
// fine-equivalent per-pixel Y; dividing by the pixel's solid angle turns it into Y per
// steradian. That last division is what makes the number independent of kAnchorWidth /
// kAnchorHeight as well: a consumer never has to know the anchor's resolution to use it,
// which is the whole point of publishing L99_sky rather than the raw P99.
inline float AnchorL99Sky(const float* anchor_y) {
  const float p99 = ComputeP99Y(anchor_y, kAnchorWidth, kAnchorHeight, kAnchorDownsampleFactor,
                                /*channel_stride=*/1, /*y_offset=*/0);
  return p99 / kAnchorPixelSolidAngle;
}

}  // namespace lumice

#endif  // CORE_ANCHOR_BUFFER_H_
